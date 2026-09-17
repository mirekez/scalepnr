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
#include <cstdlib>
#include <format>
#include <limits>
#include <memory>
#include <print>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#if defined(__unix__) || defined(__APPLE__)
#include <sys/wait.h>
#include <unistd.h>
#endif

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
    const bool benchmark = std::getenv("SCALEPNR_PLACE_SORT_COMPARE_TIMING") != nullptr;
    const bool audit = std::getenv("SCALEPNR_PLACE_SORT_AUDIT") != nullptr;
    bool audit_trial = false, audit_ignore_global_guard = false;
    double local_update_ms = 0;
    double select_ms = 0, cascade_ms = 0, prediction_ms = 0, bound_ms = 0, commit_ms = 0;
    struct Measure {
        bool enabled;
        double& total;
        std::chrono::steady_clock::time_point start;
        Measure(bool enabled, double& total) : enabled(enabled), total(total),
            start(enabled ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{}) {}
        ~Measure() {
            if (enabled) total += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now()-start).count();
        }
    };
    size_t local_wires = 0, local_outputs = 0, local_endpoints = 0;
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
    // Keep the full-cone implementation only as a diagnostic reference.
    // Normal sorting updates the live timing objects incident to moved cells.
    PlaceTimingLocal incremental(timing, current,
        std::getenv("SCALEPNR_PLACE_SORT_REFERENCE_TIMING") == nullptr);

    auto timedOut = [&] {
        return !audit_trial && config.maximum_runtime_seconds > 0
            && std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - started).count()
                >= config.maximum_runtime_seconds;
    };

    std::unordered_map<rtl::Inst*, size_t> stable_order;
    stable_order.reserve(cells.size());
    for (size_t index = 0; index < cells.size(); ++index) {
        if (cells[index]) stable_order.emplace(cells[index], index);
    }
    struct AuditMovement { rtl::Inst* cell; Coord from, to; size_t move; };
    std::vector<AuditMovement> audit_movements;
    std::vector<Coord> audit_initial;
    std::vector<size_t> audit_visits;
    std::vector<std::pair<double, double>> audit_wns;
    struct AuditTry {
        rtl::Conn* endpoint;
        rtl::Inst* cell;
        size_t after_moves;
        PlaceSortingDirection direction;
        Coord from, to, destination;
        bool accepted;
        size_t pack_rejections, timing_rejections;
        double before, after;
    };
    std::vector<AuditTry> audit_tries;
    if (audit) {
        for (auto* cell : cells) audit_initial.push_back(cell ? cell->coord : Coord{-1,-1});
        audit_visits.resize(current.endpoint_details.size(), std::numeric_limits<size_t>::max());
        audit_movements.reserve(cells.size()*10);
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
    auto collectDeficite = [&] {
        deficite.clear();
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
    };
    collectDeficite();

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
    using Counts = std::array<int, fpga::ELEMENT_TYPE_COUNT>;
    auto modelCounts = [&](size_t index) {
        const auto& tile = device.tile_grid[index];
        Counts counts{};
        counts[fpga::ELEMENT_FD] = tile.regs_cnt;
        counts[fpga::ELEMENT_LUT5] = tile.luts5cnt + tile.luts6cnt;
        counts[fpga::ELEMENT_LUT1] = tile.luts1cnt;
        counts[fpga::ELEMENT_CARRY] = tile.carry/4;
        return counts;
    };
    // Read existing Tile counters, not a duplicate occupancy cache. Some callers
    // supply a subset of cells or manually assigned fixtures; keep the reference
    // counting path unless the model counters describe exactly this occupancy.
    // MUX counters combine two element classes, so those use the reference too.
    bool use_model_counts = config.capacity_guided_displacement;
    for (size_t i = 0; use_model_counts && i < occupancy.size(); ++i) {
        Counts actual{};
        for (auto* inst : occupancy[i]) {
            auto type = fpga::elementTypeForInst(*inst);
            if (type) ++actual[*type];
            // Legacy single-input LUT aliases are classified differently by
            // the packing counters and elementTypeForInst. Do not mix them.
            if (type == fpga::ELEMENT_LUT5 && inst->cnt_inputs == 1
                && inst->cell_ref->type != "INV") use_model_counts = false;
        }
        if (device.tile_grid[i].mux != 0 || actual != modelCounts(i)) use_model_counts = false;
    }
    auto occupantCounts = [&](size_t index, rtl::Inst& moving) {
        Counts counts{};
        if (use_model_counts) {
            counts = modelCounts(index);
            if (moving.tile.peer == &device.tile_grid[index])
                if (auto type = fpga::elementTypeForInst(moving)) --counts[*type];
        } else {
            for (auto* inst : occupancy[index])
                if (inst != &moving)
                    if (auto type = fpga::elementTypeForInst(*inst)) ++counts[*type];
        }
        return counts;
    };
    std::vector<std::array<int, fpga::ELEMENT_TYPE_COUNT>> capacities(device.tile_grid.size());
    for (size_t i = 0; i < device.tile_grid.size(); ++i) {
        auto& tile = device.tile_grid[i];
        tile.hasFreeElement(fpga::ELEMENT_FD); // Initialize abstract position masks.
        for (size_t type = 0; type < fpga::ELEMENT_TYPE_COUNT; ++type)
            capacities[i][type] = std::popcount(tile.elements_pos[type]);
    }

    // One reverse scan answers which requested distances can free a slot.
    // Read live occupants; retain nothing across moves. A whole Tile can
    // evacuate iff its contents fit in the next Tile either as it stands or
    // after that next Tile evacuates. Counts are only a necessary condition:
    // the existing exact Element preview remains authoritative.
    auto feasibleDistances = [&](rtl::Inst& moving, PlaceSortingDirection direction,
                                 size_t requested) {
        Measure measure(benchmark, select_ms);
        std::vector<bool> feasible(requested + 1, true);
        if (!config.capacity_guided_displacement) return feasible;
        auto moving_type = fpga::elementTypeForInst(moving);
        if (!moving_type) return feasible;
        const Coord step = directionStep(direction);
        const Coord last = moving.coord - scaled(step, static_cast<int>(requested));
        const size_t target_index = tileIndex(last, width);
        int target_used = occupantCounts(target_index, moving)[*moving_type];
        // The preferred distance already has a slot: let exact timing/packing
        // try it directly, without walking an otherwise irrelevant full row.
        if (target_used < capacities[target_index][*moving_type]) return feasible;
        Coord edge = moving.coord;
        if (step.x) edge.x = step.x > 0 ? width - 1 : 0;
        else edge.y = step.y > 0 ? height - 1 : 0;
        Counts next_contents{}, next_capacity{};
        bool next_exists = false, next_evacuates = false;
        for (Coord at = edge; ; at = at - step) {
            ++result.displacement_tiles_examined;
            const size_t index = tileIndex(at, width);
            Counts contents = occupantCounts(index, moving);
            bool movable = true;
            for (auto* inst : occupancy[index]) {
                if (inst == &moving) continue; // A/B vacates its original slot.
                if (inst->outline.fixed) movable = false;
            }
            bool fits_next = next_exists, fits_evacuated_next = next_exists;
            for (size_t type = 0; type < contents.size(); ++type) {
                fits_next &= contents[type] + next_contents[type] <= next_capacity[type];
                fits_evacuated_next &= contents[type] <= next_capacity[type];
            }
            const bool evacuates = movable && (fits_next
                || (fits_evacuated_next && next_evacuates));
            const int distance = (moving.coord.x - at.x)*step.x
                + (moving.coord.y - at.y)*step.y;
            if (distance > 0) {
                feasible[distance] = contents[*moving_type] + 1 <= capacities[index][*moving_type]
                    || (evacuates && capacities[index][*moving_type] >= 1);
                if (!feasible[distance]) ++result.impossible_displacements;
            }
            if (at.x == last.x && at.y == last.y) break;
            next_contents = contents;
            next_capacity = capacities[index];
            next_evacuates = evacuates;
            next_exists = true;
        }
        return feasible;
    };

    // Predict timing without changing coordinates. Once packing is proven,
    // commit forward; timing is refreshed only after the real movement.
    auto tryPlan = [&](std::vector<Relocation>& plan, rtl::Conn* data_in,
                       Coord target, Coord free, bool possible) {
        Measure measure(benchmark, commit_ms);
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
            size_t remaining = 0;
            for (size_t index : pending) {
                // A/B is last in pending. It can reserve only after every
                // earlier displaced cell succeeds, just as in the erase loop.
                if (index == 0 && remaining != 0) {
                    pending[remaining++] = index;
                    continue;
                }
                auto& move = plan[index];
                auto& preview = *previews[preview_index.at(move.destination)];
                int pos = preview.reserveAt(move.inst, move.source_pos, false);
                // At a partially occupied boundary the old slot may be busy.
                // Try the normal linear selector, never reservePack's search.
                if (pos < 0) pos = preview.reserve(move.inst, false);
                if (pos >= 0) {
                    move.pos = pos;
                    commit_order.push_back(index);
                } else pending[remaining++] = index;
            }
            pending.resize(remaining);
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
        const double audit_before_wns = current.worst_slack_ns;
        if (audit && !audit_trial)
            for (auto& move : plan)
                audit_movements.push_back({move.inst, move.source->coord,
                    move.destination->coord, result.accepted_moves});
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
        const auto update_started = benchmark ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        incremental.updateForward(changed);
        PNR_ASSERT(endpoint->slack_ns > slack_before + epsilon,
            "PlaceSorting predicted improvement disagrees with committed timing");
        PNR_ASSERT(audit_ignore_global_guard
            || current.worst_slack_ns + epsilon >= audit_before_wns,
            "PlaceSorting affected-path check missed a WNS regression");
        if (audit && !audit_trial)
            audit_wns.emplace_back(audit_before_wns, current.worst_slack_ns);
        if (benchmark) {
            local_update_ms += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - update_started).count();
            local_wires += incremental.updated_wires;
            local_outputs += incremental.updated_outputs;
            local_endpoints += incremental.updated_endpoints;
        }
        attempt.packed = true;
        attempt.target = target;
        attempt.free_tile = free;
        attempt.changed = std::move(changed);
        attempt.shifted_cells = plan.size() - 1;
        attempt.slack_before = slack_before;
        return attempt;
    };

    // Only deduplicate affected endpoint IDs. No predicted timing is retained
    // between trials: read the actual forest and proposed coordinates directly.
    std::vector<size_t> affected_generation(current.endpoint_details.size(), 0);
    size_t candidate_generation = 0;
    auto attemptCascade = [&](rtl::Inst& moving, Coord target,
                              PlaceSortingDirection direction,
                              rtl::Conn* data_in,
                              const std::vector<std::vector<rtl::Inst*>>& occupancy) {
        Measure measure(benchmark, cascade_ms);
        const Coord step = directionStep(direction);
        std::vector<Relocation> plan;
        std::array<int, fpga::ELEMENT_TYPE_COUNT> boundary_incoming{};
        std::unordered_map<rtl::Inst*, Coord> proposed_coords;
        std::vector<size_t> affected;
        if (++candidate_generation == 0) {
            std::fill(affected_generation.begin(), affected_generation.end(), 0);
            ++candidate_generation;
        }
        const size_t selected = endpoint_indices.at(data_in);
        auto geometry = [&](Coord a, Coord b) {
            int dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
            return dx*timing.calibration.horizontal_ns_per_tile
                + dy*timing.calibration.vertical_ns_per_tile
                + (dx && dy ? timing.calibration.bend_ns : 0.0);
        };
        auto coord = [&](rtl::Inst* inst) {
            auto found = proposed_coords.find(inst);
            return found == proposed_coords.end() ? inst->coord : found->second;
        };
        auto canShift = [&](rtl::Inst* inst, Coord boundary) {
            if (!inst || !inst->tile.peer || proposed_coords.contains(inst)
                || inst->outline.fixed || !stable_order.contains(inst)
                || !fpga::isPlaceableElement(*inst)) return false;
            Coord from = inst->coord;
            bool on_ray = step.x
                ? from.y == boundary.y && (from.x - boundary.x)*step.x >= 0
                : from.x == boundary.x && (from.y - boundary.y)*step.y >= 0;
            return on_ray && validCoord(from + step, width, height);
        };
        auto coneCanShift = [&](auto&& self, clk::TimingPath& input, Coord boundary) -> bool {
            auto* driver = input.data_in ? input.data_in->follow() : nullptr;
            if (!driver) return false;
            if (canShift(driver->inst_ref.peer, boundary)
                || canShift(input.data_in->inst_ref.peer, boundary)) return true;
            auto& output = input.precalculated ? *input.precalculated : input;
            if (output.data_output) for (auto& branch : output.sub_paths)
                if (self(self, branch, boundary)) return true;
            return false;
        };
        // Walk every input of the affected cone, not just yesterday's critical
        // input. A previously shorter branch can become the longest after a move.
        auto arrival = [&](auto&& self, clk::TimingPath& input) -> double {
            auto* driver = input.data_in ? input.data_in->follow() : nullptr;
            if (!driver) return 0;
            auto& output = input.precalculated ? *input.precalculated : input;
            double longest = -std::numeric_limits<double>::infinity();
            if (output.data_output) for (auto& branch : output.sub_paths) {
                if (!branch.data_in) continue;
                auto* port = output.data_output;
                double intrinsic = port->inst_ref.peer && port->inst_ref->cell_ref.peer
                    && port->port_ref.peer && branch.data_in->port_ref.peer
                    ? tech->comb_delays.getDelay(port->inst_ref->cell_ref->type,
                        branch.data_in->port_ref->index, port->port_ref->index) : 0;
                longest = std::max(longest, self(self, branch) + intrinsic);
            }
            double wire = input.placement.input_ready ? input.placement.wire_ns
                : timing.estimateWireDelay(*input.data_in, *driver);
            auto* a = driver->inst_ref.peer;
            auto* b = input.data_in->inst_ref.peer;
            if (a && b && a->tile.peer && b->tile.peer)
                wire += geometry(coord(a), coord(b)) - geometry(a->coord, b->coord);
            return wire + (std::isfinite(longest) ? longest : 0);
        };
        bool timing_blocked = false;
        auto improvesTiming = [&](Coord boundary) {
            Measure measure(benchmark, prediction_ms);
            const auto& selected_endpoint = current.endpoint_details[selected];
            auto slack = [&](const PlaceTimingEndpoint& endpoint) {
                return endpoint.required_ns - arrival(arrival, *endpoint.timing_path);
            };
            const double selected_after = slack(selected_endpoint);
            if (selected_after <= selected_endpoint.slack_ns + epsilon) {
                timing_blocked = !coneCanShift(coneCanShift,
                    *selected_endpoint.timing_path, boundary);
                return false;
            }
            if (audit_ignore_global_guard) return true;
            for (size_t index : affected) {
                if (index == selected) continue;
                const auto& endpoint = current.endpoint_details[index];
                // A healthier path cannot sacrifice a worse one. A less
                // critical neighbor may lose slack only while remaining
                // strictly better than the selected path was before the move.
                // Equality would merely transfer the violation back and forth.
                double after = slack(endpoint);
                if (after + epsilon < endpoint.slack_ns
                    && after <= selected_endpoint.slack_ns + epsilon) {
                    // If no remaining cell of this cone lies on the ray,
                    // extending it cannot repair the conflict. Try a shorter
                    // displacement, not every unrelated Tile up to the edge.
                    timing_blocked = !coneCanShift(coneCanShift,
                        *endpoint.timing_path, boundary);
                    return false;
                }
            }
            return true;
        };
        auto append = [&](rtl::Inst* inst, Coord destination) {
            if (auto type = fpga::elementTypeForInst(*inst)) ++boundary_incoming[*type];
            auto dependencies = incremental.endpoints_by_cell.find(inst);
            if (dependencies != incremental.endpoints_by_cell.end()) {
                for (size_t index : dependencies->second) {
                    if (affected_generation[index] != candidate_generation) {
                        affected_generation[index] = candidate_generation;
                        affected.push_back(index);
                    }
                }
            }
            proposed_coords.emplace(inst, destination);
            plan.push_back({inst, inst->tile.peer,
                &device.tile_grid[tileIndex(destination, width)], inst->pos, inst->pos});
        };
        append(&moving, target);
        // Reject an irreparable conflict caused by A/B itself before walking
        // the row. Do not repeatedly time the growing plan at full boundaries;
        // those cannot be packed, regardless of their timing.
        if (!improvesTiming(target) && timing_blocked) {
            ++result.shift_attempts;
            ++result.rejected_timing;
            return ShiftAttempt{};
        }
        auto continuationCanImprove = [&](Coord boundary) {
            Measure measure(benchmark, bound_ms);
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
                Coord to = from + step;
                // Capacity-only scanning can precede materializing the plan.
                // Cells already crossed by this scan must shift; cells still
                // ahead may stay or shift. This is the same bound as before.
                int from_target = (from.x-target.x)*step.x + (from.y-target.y)*step.y;
                int to_boundary = (boundary.x-from.x)*step.x + (boundary.y-from.y)*step.y;
                bool on_line = step.x ? from.y == target.y : from.x == target.x;
                if (config.capacity_guided_displacement && on_line
                    && from_target >= 0 && to_boundary > 0 && canShift(inst, target))
                    return std::array<Coord, 2>{to, to};
                return std::array<Coord, 2>{from, canShift(inst, boundary) ? to : from};
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
        Coord pending = target;
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
            // The boundary keeps its current occupants and receives the last
            // shifted Tile. Reject an impossible slot count before constructing
            // packing previews for the whole segment. Counts are a necessary
            // condition only; exact packing still checks shared LUT resources
            // and chain connectivity. No legal candidate is removed here.
            auto needed = boundary_incoming;
            const size_t boundary_index = tileIndex(free, width);
            const auto current_contents = occupantCounts(boundary_index, moving);
            for (size_t type = 0; type < needed.size(); ++type) needed[type] += current_contents[type];
            bool capacity_possible = true;
            for (size_t type = 0; type < needed.size(); ++type)
                if (needed[type] > capacities[boundary_index][type]) capacity_possible = false;
            if (!capacity_possible) {
                ++result.shift_attempts;
                ++result.rejected_packing;
            } else {
                // Do not insert every cell/dependent endpoint in maps for
                // full boundaries. Only now is there a possible destination
                // for the cascade, so materialize the crossed Tiles once.
                if (config.capacity_guided_displacement) {
                    auto incoming = boundary_incoming;
                    for (; pending.x != free.x || pending.y != free.y; pending = pending + step)
                        for (auto* inst : occupancy[tileIndex(pending, width)])
                            if (inst != &moving) append(inst, pending + step);
                    boundary_incoming = incoming;
                }
                const bool possible = improvesTiming(free);
                if (!possible && timing_blocked) {
                    ++result.shift_attempts;
                    ++result.rejected_timing;
                    break;
                }
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
            if (config.capacity_guided_displacement) boundary_incoming = current_contents;
            else {
                boundary_incoming.fill(0);
                for (rtl::Inst* inst : occupants)
                    if (inst != &moving) append(inst, next);
            }
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
    size_t traversals = 0;
    while (!deficite.empty() && !timedOut()
        && (!config.maximum_passes || traversals < config.maximum_passes)) {
        const size_t accepted_before = result.accepted_moves;
        ++traversals;
        for (const DeficiteEntry& entry : deficite) {
            if (timedOut()) {
                result.timed_out = true;
                break;
            }
            PlaceTimingEndpoint* endpoint = findEndpoint(entry.data_in);
            if (!endpoint || endpoint->slack_ns >= config.deficite_slack_ns
                || endpoint->critical_edges.empty()) continue;
            ++result.endpoints_examined;
            if (audit) audit_visits[endpoint_indices.at(entry.data_in)] = result.accepted_moves;

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
                    if (timedOut()) {
                        result.timed_out = true;
                        break;
                    }
                    endpoint = findEndpoint(entry.data_in);
                    if (!endpoint || endpoint->slack_ns >= 0) break;
                    // A successful axis must not suppress the other axis. Use the
                    // new coordinates/slack, but keep this visit's chain center.
                    destination = config.chain_center ? center : peer->coord;
                    Coord step = directionStep(direction);
                    if ((destination.x - moving->coord.x)*step.x
                        + (destination.y - moving->coord.y)*step.y >= 0) continue;
                    ++result.direction_attempts;
                    size_t requested = estimateShiftTiles(
                        endpoint->slack_ns, direction);
                    if (requested == 0) continue;
                    Coord moving_from = moving->coord;
                    // Never overshoot the peer or turn a toward-peer movement
                    // into an away-from-peer correction on a short axis.
                    size_t limit = step.x ? std::abs(moving_from.x - destination.x)
                        : std::abs(moving_from.y - destination.y);
                    requested = std::min(requested, limit);
                    const auto feasible = feasibleDistances(*moving, direction, requested);
                    ShiftAttempt attempt;
                    const size_t pack_before = result.rejected_packing;
                    const size_t timing_before = result.rejected_timing;
                    const double slack_before = endpoint->slack_ns;
                    for (; requested > 0 && !timedOut(); --requested) {
                        if (!feasible[requested]) continue;
                        Coord target = moving_from
                            - scaled(step, static_cast<int>(requested));
                        attempt = attemptCascade(*moving, target, direction,
                            entry.data_in, occupancy);
                        if (attempt.packed) break;
                    }
                    if (audit)
                        audit_tries.push_back({entry.data_in, moving, result.accepted_moves,
                            direction, moving_from, moving->coord, destination, attempt.packed,
                            result.rejected_packing-pack_before, result.rejected_timing-timing_before,
                            slack_before, endpoint->slack_ns});
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
                    if (benchmark && (result.accepted_moves == 1000 || result.accepted_moves == 5000)) {
                        std::print("PLACE_SORTING_CHECKPOINT accepted={} attempts={} elapsed_ms={:.3f}\n",
                            result.accepted_moves, result.shift_attempts,
                            std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now()-started).count());
                    }
                    ++accepted_directions[static_cast<size_t>(direction)-1];
                    result.shifted_cells += attempt.shifted_cells;
                    result.moves.push_back({moving, peer, direction,
                        moving_from,
                        moving->coord, attempt.free_tile, requested,
                        attempt.shifted_cells, attempt.slack_before,
                        endpoint->slack_ns, entry.data_in, config.chain_center, chain.x, chain.y});
                    accepted = true;
                }
                if (!accepted && !found_free) ++result.skipped_no_free_tile;
            }
        }
        std::print("PLACE_SORTING_TRAVERSAL pass={} endpoints={} accepted={} wns={:.6f}\n",
            traversals, deficite.size(), result.accepted_moves - accepted_before, current.worst_slack_ns);
        if (result.accepted_moves == accepted_before || timedOut()) break;
        collectDeficite();
    }
    result.timed_out |= timedOut();

    // One independent full validation at the pass boundary, never per shift.
    auto verified = timing.analyze(timings);
    PNR_ASSERT(verified.endpoint_details.size() == current.endpoint_details.size(),
        "PlaceSorting local timing lost endpoints");
    for (size_t i = 0; i < current.endpoint_details.size(); ++i) {
        const auto& actual = current.endpoint_details[i];
        const auto& exact = verified.endpoint_details[i];
        PNR_ASSERT(actual.data_in == exact.data_in
            && std::abs(actual.slack_ns - exact.slack_ns) < 1e-9,
            "PlaceSorting local timing differs from full analysis");
    }
    PNR_ASSERT(std::abs(current.total_negative_slack_ns
        - verified.total_negative_slack_ns) < 1e-6,
        "PlaceSorting local timing aggregates differ from full analysis");
    if (audit) {
        std::vector<size_t> worst;
        for (size_t i = 0; i < current.endpoint_details.size(); ++i) worst.push_back(i);
        std::ranges::sort(worst, [&](size_t a, size_t b) {
            auto left = current.endpoint_details[a].slack_ns;
            auto right = current.endpoint_details[b].slack_ns;
            return left != right ? left < right : a < b;
        });
        worst.resize(std::min<size_t>(6, worst.size()));
        // Optional comma-separated endpoint names keep earlier observations in
        // scope even if the separately timed audit ends at a different move.
        const char* watch = std::getenv("SCALEPNR_PLACE_SORT_AUDIT_ENDPOINTS");
        const std::string watched = std::string{","} + (watch ? watch : "") + ",";
        for (size_t i = 0; i < current.endpoint_details.size(); ++i) {
            if (!watch) break;
            auto name = current.endpoint_details[i].data_in->inst_ref->makeName(200);
            if (watched.find("," + name + ",") != std::string::npos
                && std::ranges::find(worst, i) == worst.end())
                worst.push_back(i);
        }
        std::print("SORT_AUDIT_SUMMARY WNS={:.6f} TNS={:.6f} accepted={} examined={} histories={}\n",
            current.worst_slack_ns, current.total_negative_slack_ns,
            result.accepted_moves, result.endpoints_examined, audit_movements.size());
        for (size_t index : worst) {
            auto& endpoint = current.endpoint_details[index];
            const auto name = endpoint.data_in->inst_ref->makeName(200);
            std::print("SORT_AUDIT_PATH endpoint={} before={:.6f} final={:.6f} visited={} visit_after_moves={}\n",
                name, result.before.endpoint_details[index].slack_ns, endpoint.slack_ns,
                audit_visits[index] != std::numeric_limits<size_t>::max(), audit_visits[index]);
            for (const auto& trial : audit_tries) if (trial.endpoint == endpoint.data_in)
                std::print("SORT_AUDIT_VISIT_TRY endpoint={} cell={} after_moves={} evacuation={} from=({},{}) to=({},{}) destination=({},{}) accepted={} packing_rejections={} timing_rejections={} slack={:.6f}->{:.6f}\n",
                    name, trial.cell->makeName(200), trial.after_moves,
                    placeSortingDirectionName(trial.direction), trial.from.x, trial.from.y,
                    trial.to.x, trial.to.y, trial.destination.x, trial.destination.y,
                    trial.accepted, trial.pack_rejections, trial.timing_rejections,
                    trial.before, trial.after);
            for (const auto& edge : endpoint.critical_edges)
                std::print("SORT_AUDIT_EDGE endpoint={} driver={} type={} from=({},{}) sink={} type={} to=({},{}) distance={} wire_ns={:.6f}\n",
                    name, edge.driver->makeName(200), edge.driver->cell_ref->type,
                    edge.driver->coord.x, edge.driver->coord.y,
                    edge.sink->makeName(200), edge.sink->cell_ref->type,
                    edge.sink->coord.x, edge.sink->coord.y,
                    std::abs(edge.driver->coord.x-edge.sink->coord.x)+std::abs(edge.driver->coord.y-edge.sink->coord.y),
                    edge.wire_delay_ns);
            // Replay only this endpoint's complete input cone. All coordinates
            // are restored before any packing trial; the normal updater is not
            // used during replay, and the full evaluator is independent.
            auto dependencies = timing.setupDependencies(endpoint);
            std::unordered_map<rtl::Inst*, Coord> saved;
            for (auto* cell : dependencies) {
                saved.emplace(cell, cell->coord);
                if (stable_order.contains(cell)) cell->coord = audit_initial[stable_order.at(cell)];
            }
            auto replay = endpoint;
            timing.evaluateSetupTiming({&replay});
            for (size_t at = 0; at < audit_movements.size();) {
                const size_t move_index = audit_movements[at].move;
                const auto& action = result.moves[move_index];
                const double before = replay.slack_ns;
                auto* launch_before = replay.critical_edges.back().driver;
                bool touched = false;
                do {
                    const auto& movement = audit_movements[at++];
                    if (!saved.contains(movement.cell)) continue;
                    touched = true;
                    PNR_ASSERT(movement.cell->coord.x == movement.from.x
                        && movement.cell->coord.y == movement.from.y, "Sorting audit history has a gap");
                    movement.cell->coord = movement.to;
                    std::print("SORT_AUDIT_MOVEMENT endpoint={} move={} cell={} role={} from=({},{}) to=({},{}) actor={} actor_endpoint={}\n",
                        name, move_index+1, movement.cell->makeName(200),
                        movement.cell == action.cell ? "target" : "collateral",
                        movement.from.x, movement.from.y, movement.to.x, movement.to.y,
                        action.cell->makeName(200), action.setup_endpoint->inst_ref->makeName(200));
                } while (at < audit_movements.size() && audit_movements[at].move == move_index);
                if (!touched) continue;
                timing.evaluateSetupTiming({&replay});
                std::print("SORT_AUDIT_HISTORY endpoint={} move={} own_visit={} slack={:.6f}->{:.6f} launch={}->{} global_wns={:.6f}->{:.6f}\n",
                    name, move_index+1, action.setup_endpoint == endpoint.data_in,
                    before, replay.slack_ns, launch_before->makeName(200),
                    replay.critical_edges.back().driver->makeName(200),
                    audit_wns[move_index].first, audit_wns[move_index].second);
            }
            PNR_ASSERT(std::abs(replay.slack_ns-endpoint.slack_ns)<1e-9,
                "Sorting audit replay does not reproduce final timing");
            for (auto [cell, coordinate] : saved) cell->coord = coordinate;

#if defined(__unix__) || defined(__APPLE__)
            auto chain = chainCenter(endpoint);
            const Coord center{static_cast<int>(std::lround(chain.x)), static_cast<int>(std::lround(chain.y))};
            for (auto* cell : chain.cells) {
                if (cell->outline.fixed) continue;
                const Coord origin = cell->coord;
                for (bool vertical : {false, true}) {
                    Coord best = origin;
                    double best_slack = endpoint.slack_ns;
                    for (int position = 0; position < (vertical ? height : width); ++position) {
                        cell->coord = vertical ? Coord{origin.x, position} : Coord{position, origin.y};
                        auto trial = endpoint;
                        timing.evaluateSetupTiming({&trial});
                        int distance = std::abs(cell->coord.x-origin.x) + std::abs(cell->coord.y-origin.y);
                        int best_distance = std::abs(best.x-origin.x) + std::abs(best.y-origin.y);
                        if (trial.slack_ns > best_slack + epsilon
                            || (std::abs(trial.slack_ns-best_slack) <= epsilon && distance < best_distance)) {
                            best_slack = trial.slack_ns;
                            best = cell->coord;
                        }
                    }
                    cell->coord = origin;
                    std::print("SORT_AUDIT_AXIS endpoint={} cell={} axis={} from=({},{}) center=({},{}) best=({},{}) unpacked_slack={:.6f}\n",
                        name, cell->makeName(200), vertical ? "y" : "x",
                        origin.x, origin.y, center.x, center.y, best.x, best.y, best_slack);
                    Coord centered = vertical ? Coord{origin.x, center.y} : Coord{center.x, origin.y};
                    std::vector<std::pair<const char*, Coord>> targets{{"center", centered}};
                    if (best.x != centered.x || best.y != centered.y) targets.emplace_back("axis_best", best);
                    for (auto [kind, target] : targets) {
                        if (target.x == origin.x && target.y == origin.y) continue;
                        auto direction = vertical
                            ? (target.y > origin.y ? PlaceSortingDirection::north : PlaceSortingDirection::south)
                            : (target.x > origin.x ? PlaceSortingDirection::west : PlaceSortingDirection::east);
                        for (bool relaxed : {false, true}) {
                            std::fflush(nullptr);
                            pid_t child = fork();
                            PNR_ASSERT(child >= 0, "Sorting audit fork failed");
                            if (child == 0) {
                                audit_trial = true;
                                audit_ignore_global_guard = relaxed;
                                const auto pack_before = result.rejected_packing;
                                const auto timing_before = result.rejected_timing;
                                auto attempt = attemptCascade(*cell, target, direction, endpoint.data_in, occupancy);
                                auto exact = endpoint;
                                timing.evaluateSetupTiming({&exact});
                                PNR_ASSERT(std::abs(exact.slack_ns-endpoint.slack_ns)<1e-9,
                                    "Sorting audit trial left stale local timing");
                                std::print("SORT_AUDIT_TRIAL endpoint={} cell={} kind={} target=({},{}) ignore_global_guard={} packed={} shifted={} slack={:.6f}->{:.6f} WNS={:.6f} TNS={:.6f} pack_rejections={} timing_rejections={}\n",
                                    name, cell->makeName(200), kind, target.x, target.y, relaxed,
                                    attempt.packed, attempt.shifted_cells, replay.slack_ns, exact.slack_ns,
                                    current.worst_slack_ns, current.total_negative_slack_ns,
                                    result.rejected_packing-pack_before, result.rejected_timing-timing_before);
                                std::fflush(nullptr);
                                _exit(attempt.packed ? 0 : 2);
                            }
                            int status = 0;
                            PNR_ASSERT(waitpid(child, &status, 0) == child && WIFEXITED(status)
                                && (WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == 2), "Sorting audit trial failed");
                            if (WEXITSTATUS(status) == 0) break;
                        }
                    }
                }
            }
#endif
        }
    }
    result.after = std::move(verified);
    if (benchmark) {
        std::print("PLACE_SORTING_LOCAL_UPDATES elapsed_ms={:.3f} wires={} outputs={} endpoints={}\n",
            local_update_ms, local_wires, local_outputs, local_endpoints);
    }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    std::print("PLACE_SORTING_DIRECTIONS north={} east={} south={} west={} preference=rotating\n",
        accepted_directions[0], accepted_directions[1],
        accepted_directions[2], accepted_directions[3]);
    if (benchmark) std::print("SORT_PHASE_MS select={:.3f} cascade={:.3f} prediction={:.3f} bound={:.3f} packing_commit={:.3f} update={:.3f}\n",
        select_ms, cascade_ms, prediction_ms, bound_ms, commit_ms-local_update_ms, local_update_ms);
    std::print("PLACE_SORTING_DISPLACEMENT tiles_examined={} impossible_distances={} guided={}\n",
        result.displacement_tiles_examined, result.impossible_displacements,
        config.capacity_guided_displacement);
    if (benchmark) std::print("SORT_MODEL_COUNTS direct={}\n", use_model_counts);
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
