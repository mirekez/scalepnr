#include "PlaceSorting.h"

#include "Device.h"
#include "RegBunch.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <print>
#include <ranges>
#include <unordered_map>
#include <unordered_set>

namespace {

using fpga::Coord;

struct PlacementSnapshot {
    rtl::Inst* inst = nullptr;
    fpga::Tile* tile = nullptr;
    Coord coord{-1, -1};
    int pos = -1;
    float outline_x = 0;
    float outline_y = 0;
};

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

bool sameCoord(Coord left, Coord right)
{
    return left.x == right.x && left.y == right.y;
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
    Coord cell, Coord peer)
{
    int dx = cell.x - peer.x;
    int dy = cell.y - peer.y;
    if (dx == 0 && dy == 0) return PlaceSortingDirection::none;
    // The two diagonals divide the plane into four triangles. Direction names
    // describe where the displacement cascade is evacuated; the timing cell
    // itself moves in the opposite direction, toward its peer.
    if (std::abs(dy) >= std::abs(dx)) {
        return dy < 0 ? PlaceSortingDirection::north
                      : PlaceSortingDirection::south;
    }
    return dx > 0 ? PlaceSortingDirection::east
                  : PlaceSortingDirection::west;
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
    // A and B are considered independently. Each side is responsible for
    // closing half of the current setup deficit.
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

    auto restore = [&](const std::vector<PlacementSnapshot>& snapshots) {
        for (const PlacementSnapshot& snapshot : snapshots) {
            if (snapshot.inst && snapshot.inst->tile.peer) {
                snapshot.inst->tile->unassign(snapshot.inst);
            }
        }
        std::vector<const PlacementSnapshot*> pending;
        pending.reserve(snapshots.size());
        for (const PlacementSnapshot& snapshot : snapshots) {
            pending.push_back(&snapshot);
        }
        while (!pending.empty()) {
            size_t before = pending.size();
            for (auto it = pending.begin(); it != pending.end();) {
                const PlacementSnapshot& snapshot = **it;
                int restored = snapshot.tile
                    ? snapshot.tile->tryAddAt(snapshot.inst, snapshot.pos, false)
                    : -1;
                if (restored == snapshot.pos) {
                    snapshot.inst->outline.x = snapshot.outline_x;
                    snapshot.inst->outline.y = snapshot.outline_y;
                    it = pending.erase(it);
                }
                else {
                    ++it;
                }
            }
            PNR_ASSERT(pending.size() < before,
                "PlaceSorting could not restore {} placements",
                pending.size());
        }
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

    auto findEndpoint = [&](rtl::Conn* data_in) -> PlaceTimingEndpoint* {
        auto found = std::ranges::find(
            current.endpoint_details, data_in, &PlaceTimingEndpoint::data_in);
        return found == current.endpoint_details.end() ? nullptr : &*found;
    };

    struct ShiftAttempt {
        bool packed = false;
        Coord target{-1, -1};
        Coord free_tile{-1, -1};
        std::vector<PlacementSnapshot> snapshots;
        std::vector<rtl::Inst*> changed;
        size_t shifted_cells = 0;
    };

    auto attemptCascade = [&](rtl::Inst& moving, Coord target,
                              PlaceSortingDirection direction,
                              const std::vector<std::vector<rtl::Inst*>>&
                                  occupancy) {
        ShiftAttempt attempt;
        attempt.target = target;
        const Coord step = directionStep(direction);
        if (step.x == 0 && step.y == 0) return attempt;

        // Search from the endpoint toward the selected edge. A candidate is
        // useful only if a compatible vacancy can be propagated back through
        // every crossed Tile to the desired target.
        for (Coord free = moving.coord + step;
             validCoord(free, width, height); free = free + step) {
            if (timedOut()) {
                result.timed_out = true;
                return attempt;
            }
            ++result.free_tiles_examined;
            fpga::Tile& free_tile = device.tile_grid[
                static_cast<size_t>(tileIndex(free, width))];
            bool has_free_element = false;
            for (size_t type = 0; type < fpga::ELEMENT_TYPE_COUNT; ++type) {
                if (free_tile.hasFreeElement(
                        static_cast<fpga::ElementType>(type))) {
                    has_free_element = true;
                    break;
                }
            }
            if (!has_free_element) continue;
            std::vector<PlacementSnapshot> snapshots;
            std::unordered_set<rtl::Inst*> saved;
            auto save = [&](rtl::Inst* inst) {
                if (!inst || !saved.insert(inst).second) return;
                snapshots.push_back({inst, inst->tile.peer, inst->coord,
                    inst->pos, inst->outline.x, inst->outline.y});
            };

            bool failed = false;
            Coord hole = free;
            // Shift complete Tile contents toward the vacancy. Moving only
            // one compatible cell would leave the source Tile occupied and
            // would not create the Tile-sized reserve requested by sorting.
            for (Coord source = free - step;
                 !sameCoord(source, target - step); source = source - step) {
                if (timedOut()) {
                    result.timed_out = true;
                    failed = true;
                    break;
                }
                if (!validCoord(source, width, height)) {
                    failed = true;
                    break;
                }
                const auto& source_cells = occupancy[static_cast<size_t>(
                    tileIndex(source, width))];
                std::vector<rtl::Inst*> occupants;
                occupants.reserve(source_cells.size());
                for (rtl::Inst* candidate : source_cells) {
                    if (!candidate || candidate == &moving
                        || !candidate->tile.peer
                        || !sameCoord(candidate->coord, source)) continue;
                    if (candidate->outline.fixed) {
                        failed = true;
                        break;
                    }
                    save(candidate);
                    occupants.push_back(candidate);
                }
                if (failed) break;

                if (sameCoord(source, moving.coord)) {
                    save(&moving);
                    if (moving.tile.peer) moving.tile->unassign(&moving);
                }
                for (rtl::Inst* candidate : occupants) {
                    candidate->tile->unassign(candidate);
                }

                if (!occupants.empty()) {
                    fpga::Tile& destination = device.tile_grid[
                        static_cast<size_t>(tileIndex(hole, width))];
                    std::vector<fpga::ElementPackingChoice> choices;
                    {
                        fpga::ElementPackingPreview preview(destination);
                        std::vector<fpga::ElementPackingChoice> pending;
                        pending.reserve(occupants.size());
                        for (rtl::Inst* occupant : occupants) {
                            auto snapshot = std::ranges::find(
                                snapshots, occupant,
                                &PlacementSnapshot::inst);
                            PNR_ASSERT(snapshot != snapshots.end(),
                                "PlaceSorting lost source position");
                            pending.push_back({occupant, snapshot->pos});
                        }
                        // Preserve the source Tile's exact element layout.
                        // A few dependency-ordered passes handle chains while
                        // keeping this simple stage polynomial rather than
                        // invoking the general exponential pack search.
                        while (!pending.empty()) {
                            size_t before = pending.size();
                            for (auto it = pending.begin();
                                 it != pending.end();) {
                                if (preview.reserveAt(
                                        it->inst, it->pos, false) >= 0) {
                                    choices.push_back(*it);
                                    it = pending.erase(it);
                                } else {
                                    ++it;
                                }
                            }
                            if (pending.size() == before) {
                                failed = true;
                                break;
                            }
                        }
                    }
                    if (failed) break;
                    PNR_ASSERT(choices.size() == occupants.size(),
                        "PlaceSorting preview omitted {} Tile occupants",
                        occupants.size() - choices.size());
                    for (const fpga::ElementPackingChoice& choice : choices) {
                        int placed = destination.tryAddAt(
                            choice.inst, choice.pos, false);
                        PNR_ASSERT(placed == choice.pos,
                            "PlaceSorting could not commit previewed Tile shift");
                        updateOutline(*choice.inst);
                        ++attempt.shifted_cells;
                    }
                }
                hole = source;
            }

            if (!failed) {
                if (moving.tile.peer) {
                    save(&moving);
                    moving.tile->unassign(&moving);
                }
                fpga::Tile& target_tile = device.tile_grid[
                    static_cast<size_t>(tileIndex(target, width))];
                if (target_tile.tryAdd(&moving, false) >= 0) {
                    updateOutline(moving);
                    attempt.packed = true;
                    attempt.target = target;
                    attempt.free_tile = free;
                    attempt.snapshots = std::move(snapshots);
                    attempt.changed.reserve(attempt.snapshots.size());
                    for (const PlacementSnapshot& snapshot : attempt.snapshots)
                        attempt.changed.push_back(snapshot.inst);
                    return attempt;
                }
            }
            restore(snapshots);
        }
        return attempt;
    };

    constexpr double epsilon = 1e-9;
    const std::array<PlaceSortingDirection, 4> cardinal{
        PlaceSortingDirection::north,
        PlaceSortingDirection::east,
        PlaceSortingDirection::south,
        PlaceSortingDirection::west,
    };
    // The occupancy index changes only after an accepted cascade. Rebuilding
    // it for every rejected direction made a nominally linear sorting pass
    // rescan the entire design hundreds of times on dense puzzles.
    auto occupancy = buildOccupancy();

    for (const DeficiteEntry& entry : deficite) {
        if (timedOut()) {
            result.timed_out = true;
            break;
        }
        PlaceTimingEndpoint* endpoint = findEndpoint(entry.data_in);
        if (!endpoint || endpoint->slack_ns >= config.deficite_slack_ns
            || endpoint->critical_edges.empty()) continue;
        ++result.endpoints_examined;

        PlaceTimingEdge selected = *std::ranges::max_element(
            endpoint->critical_edges, {}, &PlaceTimingEdge::wire_delay_ns);
        std::array<std::pair<rtl::Inst*, rtl::Inst*>, 2> sides{
            std::pair{selected.driver, selected.sink},
            std::pair{selected.sink, selected.driver},
        };
        for (const auto& [moving, peer] : sides) {
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
            if (!moving || !peer || !moving->tile.peer || !peer->tile.peer
                || moving->outline.fixed || !fpga::isPlaceableElement(*moving)) {
                ++result.skipped_fixed;
                continue;
            }

            PlaceSortingDirection primary = directionFor(
                moving->coord, peer->coord);
            if (primary == PlaceSortingDirection::none) {
                ++result.skipped_no_direction;
                continue;
            }
            std::array<PlaceSortingDirection, 4> directions = cardinal;
            auto primary_position = std::ranges::find(directions, primary);
            std::rotate(directions.begin(), primary_position,
                        primary_position + 1);

            bool found_free = false;
            bool accepted = false;
            for (PlaceSortingDirection direction : directions) {
                ++result.direction_attempts;
                Coord step = directionStep(direction);
                size_t requested = estimateShiftTiles(
                    endpoint->slack_ns, direction);
                if (requested == 0) continue;
                Coord moving_from = moving->coord;
                Coord target = moving->coord
                    - scaled(step, static_cast<int>(requested));
                if (!validCoord(target, width, height)) continue;
                // Alternative directions are permitted, but the eventual
                // exact timing check—not geometry alone—decides usefulness.
                ++result.shift_attempts;
                ShiftAttempt attempt = attemptCascade(
                    *moving, target, direction, occupancy);
                if (!attempt.packed) {
                    ++result.rejected_packing;
                    continue;
                }
                found_free = true;

                const double slack_before = endpoint->slack_ns;
                const double wns_before = current.worst_slack_ns;
                const double tns_before = current.total_negative_slack_ns;
                const size_t violations_before = current.violated_endpoints;
                // Reject an obviously wrong fallback from the already-known
                // critical path before rebuilding any affected timing cone.
                // Exact incremental evaluation below is still authoritative
                // and detects critical-input switching.
                PlaceTimingEndpoint projected = *endpoint;
                timing.correctSetupTiming(projected);
                if (projected.slack_ns <= slack_before + epsilon) {
                    restore(attempt.snapshots);
                    ++result.rejected_timing;
                    continue;
                }
                PlaceTimingIncremental::Transaction timing_snapshot =
                    incremental.update(attempt.changed);
                endpoint = findEndpoint(entry.data_in);
                bool timing_better = endpoint
                    && endpoint->slack_ns > slack_before + epsilon
                    && current.worst_slack_ns + epsilon >= wns_before
                    && current.total_negative_slack_ns <= tns_before + epsilon
                    && current.violated_endpoints <= violations_before;
                if (!timing_better) {
                    restore(attempt.snapshots);
                    incremental.restore(std::move(timing_snapshot));
                    ++result.rejected_timing;
                    endpoint = findEndpoint(entry.data_in);
                    continue;
                }

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
                result.shifted_cells += attempt.shifted_cells;
                result.moves.push_back({moving, peer, direction,
                    moving_from,
                    moving->coord, attempt.free_tile, requested,
                    attempt.shifted_cells, slack_before,
                    endpoint->slack_ns});
                occupancy = buildOccupancy();
                accepted = true;
                break;
            }
            if (!accepted && !found_free) ++result.skipped_no_free_tile;
        }
    }

    result.after = std::move(current);
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    std::print(
        "\nPLACE_SORTING_SUMMARY endpoints={} violations={}->{} "
        "worst_slack_ns={:.3f}->{:.3f} tns_ns={:.3f}->{:.3f} "
        "DEFICITE={} examined={}/{} directions={} free_tiles={} "
        "attempts={} accepted={} shifted_cells={} rejected_pack={} "
        "rejected_timing={} skipped_fixed={} skipped_no_direction={} "
        "skipped_no_free_tile={} timed_out={} elapsed_ms={:.3f}\n",
        result.after.endpoints, result.before.violated_endpoints,
        result.after.violated_endpoints, result.before.worst_slack_ns,
        result.after.worst_slack_ns, result.before.total_negative_slack_ns,
        result.after.total_negative_slack_ns, result.deficite_cells,
        result.endpoints_examined, result.endpoint_sides_examined,
        result.direction_attempts, result.free_tiles_examined,
        result.shift_attempts, result.accepted_moves, result.shifted_cells,
        result.rejected_packing, result.rejected_timing,
        result.skipped_fixed, result.skipped_no_direction,
        result.skipped_no_free_tile, result.timed_out, result.elapsed_ms);
    return result;
}
