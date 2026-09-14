#include "PlaceSorting.h"

#include "Device.h"
#include "RegBunch.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <memory>
#include <print>
#include <ranges>
#include <unordered_map>
#include <unordered_set>

namespace {

using fpga::Coord;

void collectInsts(rtl::Inst& inst, std::vector<rtl::Inst*>& result)
{
    result.push_back(&inst);
    for (rtl::Inst& child : inst.insts) collectInsts(child, result);
}

bool validCoord(Coord coord, int width, int height)
{
    return coord.x >= 0 && coord.x < width
        && coord.y >= 0 && coord.y < height;
}

int tileIndex(Coord coord, int width)
{
    return coord.y*width + coord.x;
}

Coord scaled(Coord coord, int factor)
{
    return {coord.x*factor, coord.y*factor};
}

}

const char* pnr::placeSortingDirectionName(PlaceSortingDirection direction)
{
    switch (direction) {
    case PlaceSortingDirection::north: return "north";
    case PlaceSortingDirection::east: return "east";
    case PlaceSortingDirection::south: return "south";
    case PlaceSortingDirection::west: return "west";
    case PlaceSortingDirection::none: return "none";
    }
    return "none";
}

pnr::PlaceSortingDirection pnr::PlaceSorting::directionFor(
    Coord cell, Coord peer, Coord device_size, PlaceSortingDirection preferred)
{
    auto directions = evacuationDirections(cell, peer, device_size, preferred);
    return directions.empty() ? PlaceSortingDirection::none : directions.front();
}

pnr::PlaceSortingDirection pnr::PlaceSorting::nextDirection(
    PlaceSortingDirection direction)
{
    switch (direction) {
    case PlaceSortingDirection::north: return PlaceSortingDirection::east;
    case PlaceSortingDirection::east: return PlaceSortingDirection::south;
    case PlaceSortingDirection::south: return PlaceSortingDirection::west;
    case PlaceSortingDirection::west:
    case PlaceSortingDirection::none: return PlaceSortingDirection::north;
    }
    return PlaceSortingDirection::north;
}

std::vector<pnr::PlaceSortingDirection> pnr::PlaceSorting::evacuationDirections(
    Coord cell, Coord peer, Coord device_size, PlaceSortingDirection preferred)
{
    if (!validCoord(cell, device_size.x, device_size.y)
        || !validCoord(peer, device_size.x, device_size.y)) return {};
    // Rotate the preference, retaining only rays whose opposite movement
    // brings the selected cell toward its destination. Packing/timing decides
    // whether each such ray is actually achievable.
    std::vector<PlaceSortingDirection> result;
    auto direction = preferred == PlaceSortingDirection::none
        ? PlaceSortingDirection::north : preferred;
    for (int i = 0; i < 4; ++i, direction = nextDirection(direction)) {
        Coord step = directionStep(direction);
        int toward_peer = (peer.x - cell.x)*step.x + (peer.y - cell.y)*step.y;
        if (toward_peer < 0) result.push_back(direction);
    }
    return result;
}

Coord pnr::PlaceSorting::directionStep(PlaceSortingDirection direction)
{
    switch (direction) {
    case PlaceSortingDirection::north: return {0, -1};
    case PlaceSortingDirection::east: return {1, 0};
    case PlaceSortingDirection::south: return {0, 1};
    case PlaceSortingDirection::west: return {-1, 0};
    case PlaceSortingDirection::none: return {0, 0};
    }
    return {0, 0};
}

pnr::PlaceSortingChain pnr::PlaceSorting::chainCenter(
    const PlaceTimingEndpoint& endpoint)
{
    PlaceSortingChain chain;
    std::unordered_set<rtl::Inst*> seen;
    auto append = [&](rtl::Inst* inst) {
        if (!inst || !inst->tile.peer || !seen.insert(inst).second) return;
        chain.cells.push_back(inst);
        chain.x += inst->coord.x;
        chain.y += inst->coord.y;
    };
    for (auto it = endpoint.critical_edges.rbegin();
         it != endpoint.critical_edges.rend(); ++it) {
        append(it->driver);
        append(it->sink);
    }
    if (!chain.cells.empty()) {
        chain.x /= chain.cells.size();
        chain.y /= chain.cells.size();
    }
    return chain;
}

size_t pnr::PlaceSorting::estimateShiftTiles(
    double slack_ns, PlaceSortingDirection direction) const
{
    if (slack_ns >= 0 || direction == PlaceSortingDirection::none) return 0;
    PlaceTiming calibration;
    if (tech) calibration.tech = tech;
    bool vertical = direction == PlaceSortingDirection::north
        || direction == PlaceSortingDirection::south;
    double delay_per_tile = vertical
        ? calibration.calibration.vertical_ns_per_tile
        : calibration.calibration.horizontal_ns_per_tile;
    if (delay_per_tile <= 0) return 0;
    // Preserve the half-deficit step calibration in both modes, so the
    // chain-center experiment changes participants/directions, not step size.
    double tiles = (-slack_ns)*0.5/delay_per_tile;
    return std::max<size_t>(1, static_cast<size_t>(
        std::ceil(tiles - 1e-9)));
}

pnr::PlaceSortingResult pnr::PlaceSorting::run(clk::Timings& timings)
{
    std::vector<rtl::Inst*> cells;
    if (tech) collectInsts(tech->design.top, cells);
    return run(timings, cells);
}

pnr::PlaceSortingResult pnr::PlaceSorting::run(
    clk::Timings& timings, const std::vector<rtl::Inst*>& cells)
{
    const auto started = std::chrono::steady_clock::now();
    PlaceSortingResult result;
    if (!tech || cells.empty()) return result;

    fpga::Device& device = fpga::Device::current();
    const int width = device.size_width;
    const int height = device.size_height;
    if (width <= 0 || height <= 0 || device.tile_grid.empty()) return result;

    PlaceTiming timing;
    timing.tech = tech;
    timings.calculateTimings();
    result.before = timing.analyze(timings);
    PlaceTimingAnalysis current = result.before;
    PlaceTimingIncremental incremental(timing, current);

    auto timedOut = [&] {
        return config.maximum_runtime_seconds > 0
            && std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - started).count()
                >= config.maximum_runtime_seconds;
    };

    std::unordered_map<rtl::Inst*, size_t> stable_order;
    stable_order.reserve(cells.size());
    for (size_t index = 0; index < cells.size(); ++index) {
        if (cells[index]) stable_order.emplace(cells[index], index);
    }

    auto buildOccupancy = [&] {
        std::vector<std::vector<rtl::Inst*>> occupancy(
            static_cast<size_t>(width*height));
        for (rtl::Inst* inst : cells) {
            if (!inst || !inst->tile.peer || !fpga::isPlaceableElement(*inst))
                continue;
            if (validCoord(inst->coord, width, height)) {
                occupancy[static_cast<size_t>(tileIndex(inst->coord, width))]
                    .push_back(inst);
            }
        }
        for (auto& tile_cells : occupancy) {
            std::ranges::stable_sort(tile_cells, [&](rtl::Inst* left,
                                                     rtl::Inst* right) {
                return stable_order[left] < stable_order[right];
            });
        }
        return occupancy;
    };

    auto updateOutline = [&](rtl::Inst& inst) {
        float aspect_x = std::max(tech->place.aspect_x, 0.0001F);
        float aspect_y = std::max(tech->place.aspect_y, 0.0001F);
        inst.outline.x = (inst.coord.x + 0.25F*(inst.pos % 4))/aspect_x;
        inst.outline.y = (inst.coord.y + 0.25F*(inst.pos / 4))/aspect_y;
    };

    struct DeficiteEntry {
        rtl::Conn* data_in = nullptr;
        double slack_ns = 0;
        size_t stable = 0;
    };
    std::vector<DeficiteEntry> deficite;
    std::unordered_set<rtl::Inst*> deficite_cells;
    for (const PlaceTimingEndpoint& endpoint : current.endpoint_details) {
        rtl::Inst* endpoint_cell = endpoint.data_in
            ? endpoint.data_in->inst_ref.peer : nullptr;
        if (!endpoint_cell
            || endpoint.slack_ns >= config.deficite_slack_ns) continue;
        deficite.push_back({endpoint.data_in, endpoint.slack_ns,
            stable_order.contains(endpoint_cell)
                ? stable_order[endpoint_cell]
                : std::numeric_limits<size_t>::max()});
        deficite_cells.insert(endpoint_cell);
    }
    result.deficite_cells = deficite_cells.size();
    std::ranges::sort(deficite, [](const DeficiteEntry& left,
                                   const DeficiteEntry& right) {
        if (left.slack_ns != right.slack_ns)
            return left.slack_ns < right.slack_ns;
        return left.stable < right.stable;
    });

    std::unordered_map<rtl::Conn*, size_t> endpoint_indices;
    for (size_t i = 0; i < current.endpoint_details.size(); ++i)
        endpoint_indices.emplace(current.endpoint_details[i].data_in, i);
    auto findEndpoint = [&](rtl::Conn* data_in) -> PlaceTimingEndpoint* {
        auto found = endpoint_indices.find(data_in);
        return found == endpoint_indices.end() ? nullptr
            : &current.endpoint_details[found->second];
    };

    struct ShiftAttempt {
        bool packed = false;
        Coord target{-1, -1};
        Coord free_tile{-1, -1};
        std::vector<rtl::Inst*> changed;
        size_t shifted_cells = 0;
        double slack_before = 0;
    };

    constexpr double epsilon = 1e-9;
    struct Relocation {
        rtl::Inst* inst = nullptr;
        fpga::Tile* source = nullptr;
        fpga::Tile* destination = nullptr;
        int source_pos = -1;
        int pos = -1;
    };
    auto occupancy = buildOccupancy();
    std::vector<std::array<int, fpga::ELEMENT_TYPE_COUNT>> capacities(device.tile_grid.size());
    for (size_t i = 0; i < device.tile_grid.size(); ++i) {
        auto& tile = device.tile_grid[i];
        tile.hasFreeElement(fpga::ELEMENT_FD); // Initialize abstract position masks.
        for (size_t type = 0; type < fpga::ELEMENT_TYPE_COUNT; ++type)
            capacities[i][type] = std::popcount(tile.elements_pos[type]);
    }

    // Predict timing without changing coordinates. Once packing is proven,
    // commit forward; timing is refreshed only after the real movement.
    auto tryPlan = [&](std::vector<Relocation>& plan, rtl::Conn* data_in,
                       Coord target, Coord free, bool possible) {
        ShiftAttempt attempt;
        ++result.shift_attempts;
        if (!possible) {
            ++result.rejected_timing;
            return attempt;
        }
        PlaceTimingEndpoint* endpoint = findEndpoint(data_in);
        const double slack_before = endpoint->slack_ns;

        ++result.packing_previews;
        // Temporarily expose the final occupancy to the Element model, but
        // leave committed counters, outlines and packing untouched. Keep all
        // reservations alive together so chain checks see earlier members.
        std::vector<fpga::Tile*> touched;
        std::unordered_map<fpga::Tile*, size_t> preview_index;
        for (auto& move : plan) {
            touched.push_back(move.source);
            touched.push_back(move.destination);
            move.inst->tile.clear();
        }
        for (fpga::Tile* tile : touched) tile->invalidatePlacementCaches();
        std::vector<std::unique_ptr<fpga::ElementPackingPreview>> previews;
        for (auto& move : plan) {
            if (!preview_index.contains(move.destination)) {
                preview_index.emplace(move.destination, previews.size());
                previews.push_back(std::make_unique<fpga::ElementPackingPreview>(
                    *move.destination));
            }
        }
        std::vector<size_t> pending;
        std::vector<size_t> commit_order;
        // Clear the segment from its boundary inward. The selected A/B is
        // packed last, after the displaced contents have freed its target.
        for (size_t i = plan.size(); i-- > 0;) pending.push_back(i);
        while (!pending.empty()) {
            size_t before = pending.size();
            for (auto it = pending.begin(); it != pending.end();) {
                if (*it == 0 && pending.size() != 1) { ++it; continue; }
                auto& move = plan[*it];
                auto& preview = *previews[preview_index.at(move.destination)];
                int pos = preview.reserveAt(move.inst, move.source_pos, false);
                // At a partially occupied boundary the old slot may be busy.
                // Try the normal linear selector, never reservePack's search.
                if (pos < 0) pos = preview.reserve(move.inst, false);
                if (pos >= 0) {
                    move.pos = pos;
                    commit_order.push_back(*it);
                    it = pending.erase(it);
                } else ++it;
            }
            if (pending.size() == before) break;
        }
        while (!previews.empty()) previews.pop_back();
        for (auto& move : plan) {
            move.inst->tile.set(static_cast<Referable<fpga::Tile>*>(move.source));
            move.inst->coord = move.source->coord;
            move.inst->pos = move.source_pos;
        }
        for (fpga::Tile* tile : touched) tile->invalidatePlacementCaches();
        if (!pending.empty()) {
            ++result.rejected_packing;
            return attempt;
        }

        // Commit a proven plan once, at the exact previewed slots. There is
        // no vacancy search, timing rollback, or packing search in this operation.
        for (auto& move : plan) {
            move.source->unassign(move.inst);
            std::erase(occupancy[tileIndex(move.source->coord, width)], move.inst);
        }
        PNR_ASSERT(!commit_order.empty() && commit_order.back() == 0,
            "PlaceSorting must free the target before placing A/B");
        for (size_t index : commit_order) {
            auto& move = plan[index];
            int pos = move.destination->tryAddAt(move.inst, move.pos, false);
            PNR_ASSERT(pos == move.pos, "PlaceSorting could not commit planned shift");
            updateOutline(*move.inst);
            occupancy[tileIndex(move.destination->coord, width)].push_back(move.inst);
        }
        std::vector<rtl::Inst*> changed;
        changed.reserve(plan.size());
        for (auto& move : plan) changed.push_back(move.inst);
        ++result.timing_evaluations;
        incremental.updateForward(changed);
        attempt.packed = true;
        attempt.target = target;
        attempt.free_tile = free;
        attempt.changed = std::move(changed);
        attempt.shifted_cells = plan.size() - 1;
        attempt.slack_before = slack_before;
        return attempt;
    };

    auto attemptCascade = [&](rtl::Inst& moving, Coord target,
                              PlaceSortingDirection direction,
                              rtl::Conn* data_in,
                              const std::vector<std::vector<rtl::Inst*>>& occupancy) {
        const Coord step = directionStep(direction);
        std::vector<Relocation> plan;
        std::array<int, fpga::ELEMENT_TYPE_COUNT> boundary_incoming{};
        std::unordered_map<rtl::Inst*, Coord> proposed_coords;
        std::unordered_map<size_t, double> predicted_slacks;
        size_t below_wns = 0;
        const size_t selected = endpoint_indices.at(data_in);
        auto geometry = [&](Coord a, Coord b) {
            int dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
            return dx*timing.calibration.horizontal_ns_per_tile
                + dy*timing.calibration.vertical_ns_per_tile
                + (dx && dy ? timing.calibration.bend_ns : 0.0);
        };
        auto wireGeometry = [&](const PlaceTimingEdge& edge,
                                rtl::Inst* changed, Coord coordinate) {
            auto coord = [&](rtl::Inst* inst) {
                if (inst == changed) return coordinate;
                auto found = proposed_coords.find(inst);
                return found == proposed_coords.end() ? inst->coord : found->second;
            };
            return geometry(coord(edge.driver), coord(edge.sink));
        };
        auto append = [&](rtl::Inst* inst, Coord destination) {
            if (auto type = fpga::elementTypeForInst(*inst)) ++boundary_incoming[*type];
            // Extend the optimistic timing bound only for this new cell.
            // Previously appended Tile contents are never rescanned. Fanout
            // and cell delays are unchanged, so only geometry contributes.
            auto dependencies = incremental.endpoints_by_cell.find(inst);
            if (dependencies != incremental.endpoints_by_cell.end()) {
                for (size_t index : dependencies->second) {
                    const auto& endpoint = current.endpoint_details[index];
                    double delta = 0;
                    for (const auto& edge : endpoint.critical_edges) {
                        if (!edge.driver || !edge.sink || !edge.driver->tile.peer
                            || !edge.sink->tile.peer) continue;
                        if (edge.driver != inst && edge.sink != inst) continue;
                        delta += wireGeometry(edge, inst, destination)
                            - wireGeometry(edge, inst, inst->coord);
                    }
                    if (delta == 0) continue;
                    auto [found, inserted] = predicted_slacks.emplace(index, endpoint.slack_ns);
                    double before = found->second;
                    double after = before - delta;
                    if (before + epsilon < current.worst_slack_ns) --below_wns;
                    if (after + epsilon < current.worst_slack_ns) ++below_wns;
                    found->second = after;
                }
            }
            proposed_coords.emplace(inst, destination);
            plan.push_back({inst, inst->tile.peer,
                &device.tile_grid[tileIndex(destination, width)], inst->pos, inst->pos});
        };
        append(&moving, target);
        auto continuationCanImprove = [&](Coord boundary) {
            // Bound every possible longer cascade: remaining cells can stay
            // or move exactly one Tile. Even allowing those choices separately
            // for each edge must improve the selected path, otherwise no
            // farther vacancy is worth scanning. This is a safe bound, not a
            // radius/candidate limit.
            auto choices = [&](rtl::Inst* inst) {
                auto found = proposed_coords.find(inst);
                if (found != proposed_coords.end())
                    return std::array<Coord, 2>{found->second, found->second};
                Coord from = inst->coord;
                bool on_ray = step.x
                    ? from.y == boundary.y && (from.x - boundary.x)*step.x >= 0
                    : from.x == boundary.x && (from.y - boundary.y)*step.y >= 0;
                Coord to = from + step;
                bool can_move = on_ray && !inst->outline.fixed
                    && validCoord(to, width, height) && stable_order.contains(inst)
                    && fpga::isPlaceableElement(*inst);
                return std::array<Coord, 2>{from, can_move ? to : from};
            };
            double best_delta = 0;
            for (const auto& edge : current.endpoint_details[selected].critical_edges) {
                if (!edge.driver || !edge.sink || !edge.driver->tile.peer
                    || !edge.sink->tile.peer) continue;
                double best = std::numeric_limits<double>::infinity();
                for (Coord a : choices(edge.driver))
                    for (Coord b : choices(edge.sink))
                        best = std::min(best, geometry(a, b));
                best_delta += best - geometry(edge.driver->coord, edge.sink->coord);
            }
            return best_delta < -epsilon;
        };
        // Try the destination itself, then extend a single plan toward the
        // edge. A vacancy between target and origin is just as useful as one
        // beyond the origin. Each source Tile is appended at most once.
        for (Coord free = target; validCoord(free, width, height); free = free + step) {
            if (timedOut()) {
                result.timed_out = true;
                break;
            }
            if (!continuationCanImprove(free)) {
                ++result.rejected_timing;
                break;
            }
            ++result.free_tiles_examined;
            // Forward acceptance uses the current critical-path geometry:
            // improve this endpoint without a predicted global WNS regression.
            // TNS is deliberately not a veto. Critical-input switching can
            // change the exact outcome, but committed shifts are never undone.
            auto selected_slack = predicted_slacks.find(selected);
            bool possible = selected_slack != predicted_slacks.end()
                && selected_slack->second > current.endpoint_details[selected].slack_ns + epsilon
                && below_wns == 0;
            // The boundary keeps its current occupants and receives the last
            // shifted Tile. Reject an impossible slot count before constructing
            // packing previews for the whole segment. Counts are a necessary
            // condition only; exact packing still checks shared LUT resources
            // and chain connectivity. No legal candidate is removed here.
            auto needed = boundary_incoming;
            const size_t boundary_index = tileIndex(free, width);
            for (auto* inst : occupancy[boundary_index]) {
                if (inst == &moving) continue; // Its original slot is vacated.
                if (auto type = fpga::elementTypeForInst(*inst)) ++needed[*type];
            }
            bool capacity_possible = true;
            for (size_t type = 0; type < needed.size(); ++type)
                if (needed[type] > capacities[boundary_index][type]) capacity_possible = false;
            if (possible && !capacity_possible) {
                ++result.shift_attempts;
                ++result.rejected_packing;
            } else {
                auto attempt = tryPlan(plan, data_in, target, free, possible);
                if (attempt.packed) return attempt;
            }
            Coord next = free + step;
            if (!validCoord(next, width, height)) break;
            const auto& occupants = occupancy[tileIndex(free, width)];
            // An empty boundary cannot change the plan by extending it.
            // Likewise, moving's own vacated slot is already accounted for.
            if (std::ranges::none_of(occupants,
                    [&](rtl::Inst* inst) { return inst != &moving; })) break;
            for (rtl::Inst* inst : occupants)
                if (inst != &moving && inst->outline.fixed) return ShiftAttempt{};
            boundary_incoming.fill(0);
            for (rtl::Inst* inst : occupants)
                if (inst != &moving) append(inst, next);
        }
        return ShiftAttempt{};
    };

    // The occupancy index changes only after an accepted cascade. Rebuilding
    // it for every rejected direction made a nominally linear sorting pass
    // rescan the entire design hundreds of times on dense puzzles.
    // One strict N/E/S/W preference sequence for this entire invocation.
    // Advance once per movable cell's operation; fallbacks never reset it.
    PlaceSortingDirection preferred_direction = PlaceSortingDirection::north;
    std::array<size_t, 4> accepted_directions{};
    for (const DeficiteEntry& entry : deficite) {
        if (timedOut()) {
            result.timed_out = true;
            break;
        }
        PlaceTimingEndpoint* endpoint = findEndpoint(entry.data_in);
        if (!endpoint || endpoint->slack_ns >= config.deficite_slack_ns
            || endpoint->critical_edges.empty()) continue;
        ++result.endpoints_examined;

        // Critical edges are stored capture-to-launch. A/B are the two
        // endpoints of the whole setup path, not an internal longest wire.
        rtl::Inst* driver = endpoint->critical_edges.back().driver;
        rtl::Inst* sink = entry.data_in->inst_ref.peer;
        std::array<std::pair<rtl::Inst*, rtl::Inst*>, 2> sides{
            std::pair{driver, sink},
            std::pair{sink, driver},
        };
        PlaceSortingChain chain;
        std::vector<std::pair<rtl::Inst*, rtl::Inst*>> work(sides.begin(), sides.end());
        Coord center{-1, -1};
        size_t participants = 2;
        if (config.chain_center) {
            chain = chainCenter(*endpoint);
            center = {static_cast<int>(std::lround(chain.x)),
                      static_cast<int>(std::lround(chain.y))};
            work.clear();
            participants = 0;
            for (rtl::Inst* inst : chain.cells) {
                work.emplace_back(inst, nullptr);
                if (!inst->outline.fixed && fpga::isPlaceableElement(*inst)) ++participants;
            }
            if (config.trace_chain_moves) {
                std::print("PLACE_SORTING_CHAIN endpoint={} slack={:.6f} center=({:.3f},{:.3f}) target=({},{}) cells={} movable={}\n",
                    sink->makeName(200), endpoint->slack_ns, chain.x, chain.y,
                    center.x, center.y, chain.cells.size(), participants);
                for (rtl::Inst* inst : chain.cells)
                    std::print("PLACE_SORTING_CHAIN_CELL endpoint={} cell={} coord=({},{}) fixed={}\n",
                        sink->makeName(200), inst->makeName(200), inst->coord.x, inst->coord.y, inst->outline.fixed);
            }
        }
        // Freeze this chain and its center for one visit. Coordinate/timing
        // updates remain immediate, but earlier members cannot drag the target
        // away from members still waiting for their turn.
        for (const auto& [moving, peer] : work) {
            if (timedOut()) {
                result.timed_out = true;
                break;
            }
            endpoint = findEndpoint(entry.data_in);
            // Once an endpoint entered the DEFICITE list, give both A and B
            // their half of the correction. Stop early only when the first
            // side has already closed setup timing completely.
            if (!endpoint || endpoint->slack_ns >= 0)
                break;
            ++result.endpoint_sides_examined;
            if (config.chain_center) ++result.chain_cells_examined;
            if (!moving || !moving->tile.peer
                || (!config.chain_center && (!peer || !peer->tile.peer))
                || moving->outline.fixed || !fpga::isPlaceableElement(*moving)) {
                ++result.skipped_fixed;
                continue;
            }

            Coord destination = config.chain_center ? center : peer->coord;
            auto directions = evacuationDirections(
                moving->coord, destination, {width, height}, preferred_direction);
            // Even a blocked movable cell must not pin the rotation.
            preferred_direction = nextDirection(preferred_direction);
            if (directions.empty()) {
                ++result.skipped_no_direction;
                continue;
            }
            bool found_free = false;
            bool accepted = false;
            for (PlaceSortingDirection direction : directions) {
                ++result.direction_attempts;
                Coord step = directionStep(direction);
                size_t requested = estimateShiftTiles(
                    endpoint->slack_ns, direction);
                if (requested == 0) continue;
                Coord moving_from = moving->coord;
                // Never overshoot the peer or turn a toward-peer movement
                // into an away-from-peer correction on a short axis.
                size_t limit = step.x ? std::abs(moving_from.x - destination.x)
                    : std::abs(moving_from.y - destination.y);
                requested = std::min(requested, limit);
                ShiftAttempt attempt;
                const size_t pack_before = result.rejected_packing;
                const size_t timing_before = result.rejected_timing;
                const double slack_before = endpoint->slack_ns;
                for (; requested > 0 && !timedOut(); --requested) {
                    Coord target = moving_from
                        - scaled(step, static_cast<int>(requested));
                    attempt = attemptCascade(*moving, target, direction,
                        entry.data_in, occupancy);
                    if (attempt.packed) break;
                }
                if (config.chain_center && config.trace_chain_moves)
                    std::print("PLACE_SORTING_CHAIN_TRY endpoint={} cell={} from=({},{}) to=({},{}) evacuation={} accepted={} shifted={} pack_rejected={} timing_rejected={} slack={:.6f}->{:.6f}\n",
                        sink->makeName(200), moving->makeName(200), moving_from.x, moving_from.y,
                        moving->coord.x, moving->coord.y, placeSortingDirectionName(direction),
                        attempt.packed, attempt.shifted_cells, result.rejected_packing-pack_before,
                        result.rejected_timing-timing_before, slack_before, endpoint->slack_ns);
                if (!attempt.packed) {
                    continue;
                }
                found_free = true;
                endpoint = findEndpoint(entry.data_in);

                std::unordered_set<RegBunch*> affected_bunches;
                float aspect_x = std::max(tech->place.aspect_x, 0.0001F);
                float aspect_y = std::max(tech->place.aspect_y, 0.0001F);
                for (rtl::Inst* changed : attempt.changed) {
                    if (changed && changed->bunch_ref.peer)
                        affected_bunches.insert(changed->bunch_ref.peer);
                }
                for (RegBunch* bunch : affected_bunches) {
                    if (!bunch || !bunch->reg || !bunch->reg->tile.peer) continue;
                    bunch->x = bunch->reg->coord.x/aspect_x;
                    bunch->y = bunch->reg->coord.y/aspect_y;
                }
                ++result.accepted_moves;
                ++accepted_directions[static_cast<size_t>(direction)-1];
                result.shifted_cells += attempt.shifted_cells;
                result.moves.push_back({moving, peer, direction,
                    moving_from,
                    moving->coord, attempt.free_tile, requested,
                    attempt.shifted_cells, attempt.slack_before,
                    endpoint->slack_ns, entry.data_in, config.chain_center, chain.x, chain.y});
                accepted = true;
                break;
            }
            if (!accepted && !found_free) ++result.skipped_no_free_tile;
        }
    }

    result.after = std::move(current);
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    std::print("PLACE_SORTING_DIRECTIONS north={} east={} south={} west={} preference=rotating\n",
        accepted_directions[0], accepted_directions[1],
        accepted_directions[2], accepted_directions[3]);
    std::print(
        "\nPLACE_SORTING_SUMMARY endpoints={} violations={}->{} "
        "worst_slack_ns={:.3f}->{:.3f} tns_ns={:.3f}->{:.3f} "
        "DEFICITE={} examined={}/{} directions={} free_tiles={} "
        "attempts={} accepted={} shifted_cells={} rejected_pack={} "
        "rejected_timing={} skipped_fixed={} skipped_no_direction={} "
        "skipped_no_free_tile={} timing_evaluations={} packing_previews={} "
        "timed_out={} elapsed_ms={:.3f}\n",
        result.after.endpoints, result.before.violated_endpoints,
        result.after.violated_endpoints, result.before.worst_slack_ns,
        result.after.worst_slack_ns, result.before.total_negative_slack_ns,
        result.after.total_negative_slack_ns, result.deficite_cells,
        result.endpoints_examined, result.endpoint_sides_examined,
        result.direction_attempts, result.free_tiles_examined,
        result.shift_attempts, result.accepted_moves, result.shifted_cells,
        result.rejected_packing, result.rejected_timing,
        result.skipped_fixed, result.skipped_no_direction,
        result.skipped_no_free_tile, result.timing_evaluations,
        result.packing_previews, result.timed_out, result.elapsed_ms);
    return result;
}
