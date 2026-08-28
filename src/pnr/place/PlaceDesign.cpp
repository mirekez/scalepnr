#include "PlaceDesign.h"
#include "Device.h"
#include "Tech.h"
#include "on_return.h"

#include <algorithm>
#include <bit>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <limits>
#include <unordered_set>
#include <vector>

using namespace pnr;

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

bool placeChainTraceMatches(const rtl::Inst& inst)
{
    const char* filter = std::getenv("SCALEPNR_PLACE_CHAIN_TRACE");
    if (!filter || !*filter) {
        return false;
    }
    std::string name = const_cast<rtl::Inst&>(inst).makeName(FULL_NAME_LIMIT);
    return name.find(filter) != std::string::npos;
}

bool isCarry(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("CARRY") != std::string::npos;
}

bool isLut(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("LUT") == 0;
}

bool isMuxF7(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("MUXF7") == 0;
}

int placeRegion(int x, int y, int fpga_width, int fpga_height)
{
    int region_x = std::clamp(x * PlaceDesign::mesh_width / std::max(1, fpga_width),
                              0, PlaceDesign::mesh_width - 1);
    int region_y = std::clamp(y * PlaceDesign::mesh_height / std::max(1, fpga_height),
                              0, PlaceDesign::mesh_height - 1);
    return region_y * PlaceDesign::mesh_width + region_x;
}

bool isMuxF8(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("MUXF8") == 0;
}

bool isPackableLogic(rtl::Inst& inst)
{
    if (!inst.cell_ref.peer) {
        return false;
    }

    const std::string& type = inst.cell_ref->type;
    return type.find("FD") != std::string::npos
        || type.find("LUT") != std::string::npos
        || type.find("CARRY") != std::string::npos
        || type.find("MUX") != std::string::npos;
}

void collectInsts(rtl::Inst& inst, std::vector<rtl::Inst*>& insts)
{
    insts.push_back(&inst);
    for (auto& sub_inst : inst.insts) {
        collectInsts(sub_inst, insts);
    }
}

bool strictLocalChain(rtl::Inst& left, rtl::Inst& right)
{
    return (isLut(left) && isMuxF7(right)) || (isMuxF7(left) && isMuxF8(right));
}

bool strictLocalChainInput(rtl::Inst& left, rtl::Inst& right, rtl::Port* sink_port)
{
    // Only mux data inputs are strict tile-local arcs; selector inputs route through fabric.
    if (!sink_port || !strictLocalChain(left, right)) {
        return false;
    }
    return sink_port->name == "I0" || sink_port->name == "I1";
}

bool strictChainPreferredCoord(rtl::Inst& inst, Coord& coord, float aspect_x, float aspect_y,
                               bool& fixed_anchor)
{
    // Connected LUT->MUXF7 and MUXF7->MUXF8 arcs are tile-local, so use an already placed peer as anchor.
    fixed_anchor = false;
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (driver && driver->tile.peer && strictLocalChainInput(*driver, inst, conn.port_ref.peer)) {
            coord = driver->coord;
            fixed_anchor = true;
            return true;
        }
    }

    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (sink && sink->tile.peer && strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                coord = sink->coord;
                fixed_anchor = true;
                return true;
            }
            if (!sink || !strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                continue;
            }
            for (auto& input : sink->conns) {
                if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
                    continue;
                }
                rtl::Conn* driver_conn = input.follow();
                rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
                if (driver && driver != &inst && driver->tile.peer
                    && strictLocalChainInput(*driver, *sink, input.port_ref.peer)) {
                    coord = driver->coord;
                    fixed_anchor = true;
                    return true;
                }
            }
            coord = {static_cast<int>(sink->outline.x*aspect_x), static_cast<int>(sink->outline.y*aspect_y)};
            return true;
        }
    }
    return false;
}

rtl::Inst* carryChainDriver(rtl::Inst& inst)
{
    if (!isCarry(inst)) {
        return nullptr;
    }

    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->name != "CI") {
            continue;
        }

        rtl::Conn* driver_conn = conn.follow();
        if (!driver_conn || !driver_conn->inst_ref.peer) {
            continue;
        }

        rtl::Inst& driver = *driver_conn->inst_ref;
        if (isCarry(driver)) {
            return &driver;
        }
    }

    return nullptr;
}

void visitStrictLocalChainDrivers(rtl::Inst& inst, auto&& visit)
{
    // MUX chain input drivers must be packed before the mux that consumes their tile-local output.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (driver && strictLocalChainInput(*driver, inst, conn.port_ref.peer)) {
            visit(*driver);
        }
    }
}

void visitStrictLocalChainSinks(rtl::Inst& inst, auto&& visit)
{
    // Once a chain producer is placed, immediately pack tile-local mux consumers beside it.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (sink && strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                visit(*sink);
            }
        }
    }
}

void visitStrictLocalChainSiblingDrivers(rtl::Inst& inst, auto&& visit)
{
    // A mux input producer must bring the other strict mux producers before it commits.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (!sink || !strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr) || sink->tile.peer) {
                continue;
            }
            for (auto& input : sink->conns) {
                if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
                    continue;
                }
                rtl::Conn* driver_conn = input.follow();
                rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
                if (driver && driver != &inst && strictLocalChainInput(*driver, *sink, input.port_ref.peer)) {
                    if (isMuxF7(inst) && isMuxF7(*driver)) {
                        continue;
                    }
                    visit(*driver);
                }
            }
        }
    }
}

bool strictLocalChainDriversReady(rtl::Inst& inst)
{
    // Strict mux sinks are packable only after every tile-local producer has a tile.
    bool ready = true;
    visitStrictLocalChainDrivers(inst, [&](rtl::Inst& driver) {
        if (!driver.tile.peer) {
            ready = false;
        }
    });
    return ready;
}

bool hasLockedUnplacedStrictSiblingDriver(rtl::Inst& inst)
{
    // A locked unplaced sibling is the caller waiting for this chain lane to be reserved.
    bool found = false;
    visitStrictLocalChainSiblingDrivers(inst, [&](rtl::Inst& driver) {
        if (driver.locked && !driver.tile.peer) {
            found = true;
        }
    });
    return found;
}

bool hasLockedUnplacedStrictSink(rtl::Inst& inst)
{
    // A locked unplaced sink is the caller waiting for this producer to return.
    bool found = false;
    visitStrictLocalChainSinks(inst, [&](rtl::Inst& sink) {
        if (sink.locked && !sink.tile.peer) {
            found = true;
        }
    });
    return found;
}

bool carryChainPreferredCoord(rtl::Inst& inst, Coord& coord)
{
    rtl::Inst* driver = carryChainDriver(inst);
    if (!driver || !driver->tile.peer) {
        return false;
    }

    coord = driver->coord + Coord{0, -1};
    return true;
}

struct TimingPlacementSnapshot
{
    rtl::Inst* inst = nullptr;
    fpga::Tile* tile = nullptr;
    Coord coord;
    int pos = -1;
};

std::vector<rtl::Inst*> timingPeers(rtl::Inst& inst, technology::Tech* tech)
{
    std::vector<rtl::Inst*> peers;
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer) {
            continue;
        }
        if (conn.port_ref->type == rtl::Port::PORT_IN) {
            if (conn.port_ref->is_global
                || (tech && tech->check_clocked(
                    inst.cell_ref->type, conn.port_ref->name))) {
                continue;
            }
            rtl::Conn* driver = conn.follow();
            if (driver && driver->inst_ref.peer) {
                peers.push_back(driver->inst_ref.peer);
            }
            continue;
        }
        if (conn.port_ref->type == rtl::Port::PORT_OUT) {
            for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
                rtl::Conn* sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
                if (sink && sink->inst_ref.peer && sink->port_ref.peer
                    && !sink->port_ref->is_global
                    && (!tech || !tech->check_clocked(
                        sink->inst_ref->cell_ref->type,
                        sink->port_ref->name))) {
                    peers.push_back(sink->inst_ref.peer);
                }
            }
        }
    }
    return peers;
}

std::vector<rtl::Inst*> timingConstellation(rtl::Inst& anchor,
                                             rtl::Inst* attraction_peer,
                                             technology::Tech* tech)
{
    constexpr int max_depth = 2;
    constexpr int max_radius = 2;
    constexpr size_t max_cells = 8;
    std::vector<rtl::Inst*> result;
    std::deque<std::pair<rtl::Inst*, int>> pending;
    std::unordered_set<rtl::Inst*> visited;
    pending.emplace_back(&anchor, 0);
    visited.insert(&anchor);

    while (!pending.empty() && result.size() < max_cells) {
        auto [inst, depth] = pending.front();
        pending.pop_front();
        if (!inst || !inst->tile.peer || inst->outline.fixed
            || !fpga::isPlaceableElement(*inst)) {
            continue;
        }
        result.push_back(inst);
        if (depth >= max_depth) {
            continue;
        }
        for (rtl::Inst* peer : timingPeers(*inst, tech)) {
            if (!peer || peer == attraction_peer || visited.contains(peer)
                || !peer->tile.peer || peer->outline.fixed) {
                continue;
            }
            int distance = std::abs(peer->coord.x - anchor.coord.x)
                + std::abs(peer->coord.y - anchor.coord.y);
            if (distance > max_radius) {
                continue;
            }
            visited.insert(peer);
            pending.emplace_back(peer, depth + 1);
        }
    }
    return result;
}

bool timingObjectiveImproved(const PlaceTimingAnalysis& before,
                             const PlaceTimingAnalysis& after)
{
    constexpr double epsilon = 1e-9;
    if (after.total_negative_slack_ns
        < before.total_negative_slack_ns - epsilon) {
        return true;
    }
    return std::abs(after.total_negative_slack_ns
                    - before.total_negative_slack_ns) <= epsilon
        && after.worst_slack_ns > before.worst_slack_ns + epsilon;
}

}

void PlaceDesign::preparePlaceCandidates()
{
    // Bucket every compatible resource tile by the same coarse regions used by outline placement.
    for (auto& by_region : place_candidates) {
        for (CandidateList& candidates : by_region) {
            candidates.clear();
        }
    }
    place_candidate_cursor = {};
    for (uint32_t index = 0; index < tile_grid->size(); ++index) {
        fpga::Tile& tile = (*tile_grid)[index];
        if (tile.coord.x < 0 || tile.coord.y < 0 || !tile.tile_type) {
            continue;
        }
        int region = placeRegion(tile.coord.x, tile.coord.y, fpga_width, fpga_height);
        for (int type_index = 0; type_index < fpga::ELEMENT_TYPE_COUNT; ++type_index) {
            fpga::ElementType element_type = static_cast<fpga::ElementType>(type_index);
            bool supports_type = std::ranges::any_of(
                tile.tile_type->elements,
                [element_type](const fpga::Element& element) {
                    return element.type == element_type;
                });
            if (supports_type) {
                place_candidates[type_index][region].push_back(index);
            }
        }
    }
}

int PlaceDesign::tryAddNear(rtl::Inst& inst, fpga::ElementType type, const Coord& origin)
{
    // Search complete outline regions radially while skipping tiles without this element type.
    Coord region_coord{origin.x * mesh_width / std::max(1, fpga_width),
                       origin.y * mesh_height / std::max(1, fpga_height)};
    region_coord.x = std::clamp(region_coord.x, 0, mesh_width - 1);
    region_coord.y = std::clamp(region_coord.y, 0, mesh_height - 1);
    int dir = 0;
    int steps = 1;
    int search_pos = 0;
    for (int region_trial = 0; region_trial < place_region_count; ++region_trial) {
        if (region_coord.x >= 0 && region_coord.x < mesh_width
            && region_coord.y >= 0 && region_coord.y < mesh_height) {
            int region = region_coord.y * mesh_width + region_coord.x;
            CandidateList& candidates = place_candidates[type][region];
            size_t& cursor = place_candidate_cursor[type][region];
            if (!candidates.empty()) {
                cursor %= candidates.size();
                size_t checked = 0;
                while (checked < candidates.size()) {
                    size_t position = cursor % candidates.size();
                    fpga::Tile& tile = (*tile_grid)[candidates[position]];
                    ++place_tile_trials;
                    auto now = std::chrono::steady_clock::now();
                    if (now >= place_next_report) {
                        double elapsed = std::chrono::duration<double>(
                            now - place_started).count();
                        std::print("\nPLACE_PROGRESS elapsed_s={:.1f} calls={} tile_trials={} commits={} current='{}' region={}/{} candidates={}",
                            elapsed, place_calls, place_tile_trials,
                            place_commits, inst.makeName(FULL_NAME_LIMIT),
                            region_trial + 1, place_region_count,
                            candidates.size());
                        std::fflush(stdout);
                        place_next_report = now + std::chrono::minutes(1);
                    }
                    if (!tile.hasFreeElement(type)) {
                        // Initial placement only consumes elements, so an exhausted
                        // tile can be removed from this type/region candidate set.
                        candidates[position] = candidates.back();
                        candidates.pop_back();
                        if (candidates.empty()) {
                            cursor = 0;
                            break;
                        }
                        cursor %= candidates.size();
                        continue;
                    }
                    bool stall_debug = std::getenv("SCALEPNR_PLACE_STALL_DEBUG") != nullptr
                        && placeChainTraceMatches(inst);
                    if (stall_debug) {
                        std::print("\nPLACE_BUCKET_TRY inst='{}' tile=({}, {}) full='{}' "
                                   "region={}/{} checked={}/{}",
                            inst.makeName(FULL_NAME_LIMIT), tile.coord.x, tile.coord.y,
                            tile.full_name, region_trial + 1, place_region_count,
                            checked + 1, candidates.size());
                        std::fflush(stdout);
                    }
                    int placed_pos = tile.tryAdd(&inst, false);
                    if (stall_debug) {
                        std::print("\nPLACE_BUCKET_RESULT inst='{}' tile=({}, {}) result={}",
                            inst.makeName(FULL_NAME_LIMIT), tile.coord.x, tile.coord.y, placed_pos);
                        std::fflush(stdout);
                    }
                    if (placed_pos >= 0) {
                        cursor = (position + 1) % candidates.size();
                        return placed_pos;
                    }
                    cursor = (position + 1) % candidates.size();
                    ++checked;
                }
            }
        }
        radialSearch(region_coord, dir, steps, search_pos);
    }
    return -1;
}

int PlaceDesign::tryAddBySharedInput(rtl::Inst& inst, fpga::ElementType type, const Coord& origin)
{
    // A shared non-clock input can use one physical tile endpoint for all of its packed sinks.
    std::vector<fpga::Tile*> tried;
    constexpr size_t max_shared_tile_trials = 16;
    constexpr int max_shared_tile_distance = 2;
    for (rtl::Conn& input : inst.conns) {
        if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN
            || tech->check_clocked(inst.cell_ref->type, input.port_ref->name)) {
            continue;
        }
        rtl::Conn* driver = input.follow();
        if (!driver || !driver->port_ref.peer || driver->port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        if (driver->port_ref->is_global) {
            continue;
        }
        const auto& sinks = rtl::Conn::getSinks(*driver);
        if (sinks.size() < 2) {
            continue;
        }
        // Keep timing-sized fanouts near their outline target. Large shared
        // control sets instead need to reuse compatible packed tiles; forcing
        // every sink into a two-tile radius defeats that physical constraint.
        bool enforce_shared_tile_distance = sinks.size() <= 8;
        auto report_progress = [&]() {
            auto now = std::chrono::steady_clock::now();
            if (now < place_next_report) {
                return;
            }
            double elapsed = std::chrono::duration<double>(now - place_started).count();
            std::print("\nPLACE_PROGRESS elapsed_s={:.1f} calls={} tile_trials={} commits={} "
                       "current='{}' phase=shared-input port={} driver='{}' fanout={} tried={}",
                elapsed, place_calls, place_tile_trials, place_commits,
                inst.makeName(FULL_NAME_LIMIT), input.port_ref->name,
                driver->makeName(nullptr, FULL_NAME_LIMIT), sinks.size(), tried.size());
            std::fflush(stdout);
            place_next_report = now + std::chrono::minutes(1);
        };
        // Recent sinks are most likely to belong to the currently open control set.
        for (auto sink_it = sinks.rbegin(); sink_it != sinks.rend(); ++sink_it) {
            report_progress();
            auto* sink_ref = *sink_it;
            rtl::Conn* sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sibling = sink ? sink->inst_ref.peer : nullptr;
            fpga::Tile* tile = sibling && sibling != &inst ? sibling->tile.peer : nullptr;
            if (!tile || std::find(tried.begin(), tried.end(), tile) != tried.end()
                || !tile->hasFreeElement(type)
                || (enforce_shared_tile_distance
                    && std::abs(tile->coord.x - origin.x)
                        + std::abs(tile->coord.y - origin.y)
                        > max_shared_tile_distance)) {
                continue;
            }
            tried.push_back(tile);
            ++place_tile_trials;
            bool stall_debug = std::getenv("SCALEPNR_PLACE_STALL_DEBUG") != nullptr
                && placeChainTraceMatches(inst);
            if (stall_debug) {
                std::print("\nPLACE_SHARED_TRY inst='{}' port={} driver='{}' fanout={} "
                           "tile=({}, {}) full='{}' trial={}",
                    inst.makeName(FULL_NAME_LIMIT), input.port_ref->name,
                    driver->makeName(nullptr, FULL_NAME_LIMIT), sinks.size(),
                    tile->coord.x, tile->coord.y, tile->full_name, tried.size());
                std::fflush(stdout);
            }
            int placed_pos = tile->tryAdd(&inst, false);
            if (stall_debug) {
                std::print("\nPLACE_SHARED_RESULT inst='{}' tile=({}, {}) result={}",
                    inst.makeName(FULL_NAME_LIMIT), tile->coord.x, tile->coord.y, placed_pos);
                std::fflush(stdout);
            }
            if (placed_pos >= 0) {
                return placed_pos;
            }
            if (tried.size() >= max_shared_tile_trials) {
                return -1;
            }
        }
    }
    return -1;
}

int PlaceDesign::tryAddSparseTile(rtl::Inst& inst, fpga::ElementType type, const Coord& origin)
{
    // Probe lightly occupied tiles first so a new input-control set avoids saturated endpoints.
    if (origin.x >= 0 && origin.x < fpga_width
        && origin.y >= 0 && origin.y < fpga_height) {
        fpga::Tile& preferred = (*tile_grid)[
            static_cast<size_t>(origin.y*fpga_width + origin.x)];
        if (preferred.tile_type && preferred.hasFreeElement(type)) {
            ++place_tile_trials;
            int placed_pos = preferred.tryAdd(&inst, false);
            if (placed_pos >= 0) {
                return placed_pos;
            }
        }
    }
    Coord region_coord{origin.x * mesh_width / std::max(1, fpga_width),
                       origin.y * mesh_height / std::max(1, fpga_height)};
    region_coord.x = std::clamp(region_coord.x, 0, mesh_width - 1);
    region_coord.y = std::clamp(region_coord.y, 0, mesh_height - 1);
    int dir = 0;
    int steps = 1;
    int search_pos = 0;
    constexpr int max_candidates_per_region = 4;
    for (int region_trial = 0; region_trial < place_region_count; ++region_trial) {
        if (region_coord.x >= 0 && region_coord.x < mesh_width
            && region_coord.y >= 0 && region_coord.y < mesh_height) {
            int region = region_coord.y * mesh_width + region_coord.x;
            CandidateList& candidates = place_candidates[type][region];
            int best_occupied = fpga::ELEMENT_TYPE_COUNT * fpga::ELEMENT_BITMAP_BITS + 1;
            std::array<uint32_t, max_candidates_per_region> best_tiles{};
            int best_count = 0;
            for (uint32_t tile_index : candidates) {
                fpga::Tile& tile = (*tile_grid)[tile_index];
                if (!tile.hasFreeElement(type)) {
                    continue;
                }
                int occupied = 0;
                for (int type_index = 0; type_index < fpga::ELEMENT_TYPE_COUNT; ++type_index) {
                    unsigned occupied_mask = static_cast<unsigned>(
                        tile.elements_pos[type_index]
                        & static_cast<uint16_t>(~tile.elements_free[type_index]));
                    occupied += std::popcount(occupied_mask);
                }
                if (occupied < best_occupied) {
                    best_occupied = occupied;
                    best_count = 0;
                }
                if (occupied == best_occupied && best_count < max_candidates_per_region) {
                    best_tiles[best_count++] = tile_index;
                }
            }
            for (int candidate = 0; candidate < best_count; ++candidate) {
                fpga::Tile& tile = (*tile_grid)[best_tiles[candidate]];
                ++place_tile_trials;
                int placed_pos = tile.tryAdd(&inst, false);
                if (placed_pos >= 0) {
                    return placed_pos;
                }
            }
        }
        radialSearch(region_coord, dir, steps, search_pos);
    }
    return -1;
}

void PlaceDesign::recursivePackBunch(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    ++place_calls;
    if (inst.locked) {
        return;
    }

    inst.locked = true;
    on_return clear_lock([&]() { inst.locked = false; });
    inst.mark = travers_mark;

    int x = inst.outline.x*aspect_x;
    int y = inst.outline.y*aspect_y;

    PNR_LOG2_("PLCE", depth, "packBunch, bunch: '{}' inst: '{}' ({}), x: {}, y: {} => {} {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
        inst.outline.x, inst.outline.y, x, y);
    bool trace_chain = placeChainTraceMatches(inst);
    if (trace_chain) {
        std::print("\nPLACE_CHAIN begin inst='{}' type='{}' placed={} coord=({}, {}) pos={} outline=({}, {})",
            inst.makeName(FULL_NAME_LIMIT), inst.cell_ref.peer ? inst.cell_ref->type : std::string{},
            inst.tile.peer != nullptr,
            inst.tile.peer ? inst.tile->coord.x : -1,
            inst.tile.peer ? inst.tile->coord.y : -1,
            inst.pos, inst.outline.x, inst.outline.y);
    }
    const bool return_to_locked_chain_peer =
        hasLockedUnplacedStrictSiblingDriver(inst) || hasLockedUnplacedStrictSink(inst);

    if (rtl::Inst* driver = carryChainDriver(inst); driver && (driver->mark != travers_mark || !driver->tile.peer)) {
        recursivePackBunch(*driver, nullptr, depth + 1);
    }
    visitStrictLocalChainDrivers(inst, [&](rtl::Inst& driver) {
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN driver-before sink='{}' driver='{}' type='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.cell_ref.peer ? driver.cell_ref->type : std::string{},
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
        if (driver.mark != travers_mark || !driver.tile.peer) {
            recursivePackBunch(driver, nullptr, depth + 1);
        }
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN driver-after sink='{}' driver='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
    });
    visitStrictLocalChainSiblingDrivers(inst, [&](rtl::Inst& driver) {
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN sibling-before inst='{}' sibling='{}' type='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.cell_ref.peer ? driver.cell_ref->type : std::string{},
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
        if (driver.mark != travers_mark || !driver.tile.peer) {
            recursivePackBunch(driver, nullptr, depth + 1);
        }
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN sibling-after inst='{}' sibling='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
    });
    if (!strictLocalChainDriversReady(inst)) {
        PNR_LOG2_("PLCE", depth, "defer strict local chain sink: '{}' ({})", inst.makeName(), inst.cell_ref->type);
        return;
    }

    if (fpga::isPlaceableElement(inst)) {

        if (inst.tile.peer) {
            inst.coord = inst.tile->coord;
            inst.outline.x = inst.coord.x + 0.25f*(inst.pos%4);
            inst.outline.y = inst.coord.y + 0.25f*(inst.pos/4);
            PNR_LOG2_("PLCE", depth, "inst already packed: '{}' ({}) to {} {}, pos: {}", inst.makeName(), inst.cell_ref->type,
                inst.coord.x, inst.coord.y, inst.pos);
        }
        else {
            Coord coord = {x,y};
            bool strict_chain_anchor = false;
            bool carry_chain_anchor = carryChainPreferredCoord(inst, coord);
            if (carry_chain_anchor) {
                PNR_LOG2_("PLCE", depth, "packBunch, carry chain preferred coord for '{}': {} {}", inst.makeName(), coord.x, coord.y);
            }
            else if (strictChainPreferredCoord(inst, coord, aspect_x, aspect_y,
                                               strict_chain_anchor)) {
                PNR_LOG2_("PLCE", depth, "packBunch, strict local chain preferred coord for '{}': {} {}", inst.makeName(), coord.x, coord.y);
                if (trace_chain) {
                    std::print("\nPLACE_CHAIN preferred inst='{}' coord=({}, {})",
                        inst.makeName(FULL_NAME_LIMIT), coord.x, coord.y);
                }
            }
            const Coord search_origin = coord;
            std::optional<fpga::ElementType> element_type = fpga::elementTypeForInst(inst);
            bool placed_by_bucket = false;
            if (!strict_chain_anchor && !carry_chain_anchor && element_type) {
                int placed_pos = -1;
                if (*element_type == fpga::ELEMENT_FD) {
                    placed_pos = tryAddBySharedInput(inst, *element_type, search_origin);
                }
                if (placed_pos < 0) {
                    placed_pos = tryAddSparseTile(inst, *element_type, search_origin);
                }
                if (placed_pos < 0) {
                    placed_pos = tryAddNear(inst, *element_type, search_origin);
                }
                if (placed_pos < 0) {
                    std::print("cant place inst: '{}' ({}) near {}:{}", inst.makeName(),
                               inst.cell_ref->type, search_origin.x, search_origin.y);
                    exit(1);
                }
                inst.coord = inst.tile->coord;
                inst.outline.x = (inst.coord.x + 0.25*(placed_pos%4))/aspect_x;
                inst.outline.y = (inst.coord.y + 0.25*(placed_pos/4))/aspect_y;
                inst.pos = placed_pos;
                ++place_commits;
                placed_by_bucket = true;
            }
            if (!placed_by_bucket) {
            int dir = 0, steps = 1, search_pos = 0, placed_pos = 0;
            size_t i;
            // A strict local-chain consumer can only use its already placed
            // producer's tile; scanning other tiles cannot produce a legal fit.
            const size_t max_place_search_steps = strict_chain_anchor ? 1
                : radialSearchCoverageSteps(search_origin, fpga_width, fpga_height);
            for (i=0; i < max_place_search_steps; ++i) {
                ++place_tile_trials;
                auto now = std::chrono::steady_clock::now();
                if (now >= place_next_report) {
                    double elapsed = std::chrono::duration<double>(now - place_started).count();
                    std::print("\nPLACE_PROGRESS elapsed_s={:.1f} calls={} tile_trials={} commits={} current='{}' trial={}/{} origin=({}, {})",
                        elapsed, place_calls, place_tile_trials, place_commits,
                        inst.makeName(FULL_NAME_LIMIT), i + 1, max_place_search_steps,
                        search_origin.x, search_origin.y);
                    std::fflush(stdout);
                    place_next_report = now + std::chrono::minutes(1);
                }
                if (coord.x < 0 || coord.x >= fpga_width ||
                    coord.y < 0 || coord.y >= fpga_height ||
                    (*tile_grid)[coord.y*fpga_width+coord.x].coord.x == -1 ||
                    (*tile_grid)[coord.y*fpga_width+coord.x].coord.y == -1) {

                    radialSearch(coord, dir, steps, search_pos);
                    continue;
                }
//std::print("\neeeeeeeeeeeeeee {}", inst.makeName());
                if ((placed_pos = (*tile_grid)[coord.y*fpga_width+coord.x].tryAdd(&inst, false)) >= 0) {
                    PNR_LOG2_("PLCE", depth, "put inst: '{}' ({}), x: {}, y: {} to {} {}, pos: {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
                        x, y, coord.x, coord.y, placed_pos);
                    inst.coord = coord;
                    inst.outline.x = (coord.x + 0.25*(placed_pos%4))/aspect_x;  // just for drawing
                    inst.outline.y = (coord.y + 0.25*(placed_pos/4))/aspect_y;
                    inst.pos = placed_pos;
                    ++place_commits;
                    break;
                }
                radialSearch(coord, dir, steps, search_pos);
            }

            if (i == max_place_search_steps) {
                PNR_LOG2_("PLCE", depth, "cant place inst: '{}' ({}), coord: {}:{} => {}:{} => {}:{}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, coord.x, coord.y);
                std::print("cant place inst: '{}' ({}), coord: {}:{} => {}:{} => {}:{}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, coord.x, coord.y);
                exit(1);
            }
            }
        }
    }

    visitStrictLocalChainSiblingDrivers(inst, [&](rtl::Inst& driver) {
        if (driver.mark != travers_mark || !driver.tile.peer) {
            recursivePackBunch(driver, nullptr, depth + 1);
        }
    });
    if (return_to_locked_chain_peer) {
        return;
    }

    visitStrictLocalChainSinks(inst, [&](rtl::Inst& sink) {
        if (sink.mark != travers_mark || !sink.tile.peer) {
            recursivePackBunch(sink, nullptr, depth + 1);
        }
    });

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;

            if (peer->mark != travers_mark || !peer->tile.peer) {
                recursivePackBunch(*peer, nullptr, depth + 1);
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recursivePackBunch(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}

void PlaceDesign::placeDesign(std::list<Referable<RegBunch>>& bunch_list)
{
    PNR_LOG1("PLCE", "placeDesign");
    fpga = &fpga::Device::current();
    tile_grid = &fpga->tile_grid;

    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
//    combs_per_box = /*total_comb*/(float)fpga->cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga->size_width;
    fpga_height = fpga->size_height;

    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;
    place_calls = 0;
    place_tile_trials = 0;
    place_commits = 0;
    place_started = std::chrono::steady_clock::now();
    place_next_report = place_started + std::chrono::minutes(1);
    preparePlaceCandidates();

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recursivePackBunch(*bunch.reg, &bunch);
    }

    std::vector<rtl::Inst*> all_insts;
    collectInsts(tech->design.top, all_insts);

    constexpr int max_cleanup_passes = 64;
    for (int pass = 0; pass < max_cleanup_passes; ++pass) {
        int before = 0;
        for (rtl::Inst* inst : all_insts) {
            if (isPackableLogic(*inst) && !inst->tile.peer) {
                ++before;
            }
        }
        if (before == 0) {
            break;
        }

        travers_mark = rtl::Inst::genMark();
        int placed = 0;
        for (rtl::Inst* inst : all_insts) {
            if (!isPackableLogic(*inst) || inst->tile.peer) {
                continue;
            }

            recursivePackBunch(*inst, nullptr);
            if (inst->tile.peer) {
                ++placed;
            }
        }

        int after = 0;
        for (rtl::Inst* inst : all_insts) {
            if (isPackableLogic(*inst) && !inst->tile.peer) {
                ++after;
            }
        }

        std::print("\nPLACE_SWEEP pass={} before={} placed={} after={}", pass + 1, before, placed, after);
        if (after == 0) {
            break;
        }
        if (placed == 0) {
            std::print("\nPLACE_SWEEP blocked first_unplaced:");
            int printed = 0;
            for (rtl::Inst* inst : all_insts) {
                if (isPackableLogic(*inst) && !inst->tile.peer) {
                    std::print("\n  {} ({})", inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type);
                    if (++printed >= 12) {
                        break;
                    }
                }
            }
            break;
        }
    }

    timing_refinement = {};
    if (tech && !tech->timings.clocked_inputs.empty()) {
        timing_refinement = refineTiming(tech->timings);
    }

    travers_mark = rtl::Inst::genMark();
    image.init(mesh_width*aspect_x*image_zoom, mesh_height*aspect_y*image_zoom);
    image.clear();
    for (auto& bunch : bunch_list) {
        recurseDrawDesign(*bunch.reg, &bunch);
    }
    image.write("place_output.png");
}

PlaceTimingRefinement PlaceDesign::refineTiming(
    clk::Timings& timings, size_t max_passes,
    size_t max_anchor_cells_per_pass)
{
    auto started = std::chrono::steady_clock::now();
    fpga = &fpga::Device::current();
    tile_grid = &fpga->tile_grid;
    fpga_width = fpga->size_width;
    fpga_height = fpga->size_height;
    if (fpga_width <= 0 || fpga_height <= 0 || tile_grid->empty()) {
        return {};
    }
    aspect_x = static_cast<float>(fpga_width)/mesh_width;
    aspect_y = static_cast<float>(fpga_height)/mesh_height;
    place_timing.tech = tech;

    PlaceTimingRefinement refinement;
    refinement.before = place_timing.analyze(timings);
    PlaceTimingAnalysis current = refinement.before;
    size_t anchor_limit = std::max<size_t>(1, max_anchor_cells_per_pass);

    auto restore = [&](std::vector<TimingPlacementSnapshot>& moved) {
        // Remove the complete attempted constellation before restoring any
        // member, so temporary chain separation cannot reject an original slot.
        for (auto it = moved.rbegin(); it != moved.rend(); ++it) {
            TimingPlacementSnapshot& snapshot = *it;
            if (snapshot.inst && snapshot.inst->tile.peer) {
                snapshot.inst->tile->unassign(snapshot.inst);
            }
        }
        for (auto it = moved.rbegin(); it != moved.rend(); ++it) {
            TimingPlacementSnapshot& snapshot = *it;
            if (!snapshot.inst || !snapshot.tile) {
                continue;
            }
            int restored = snapshot.tile->tryAddAt(snapshot.inst, snapshot.pos);
            PNR_ASSERT(restored == snapshot.pos,
                "failed to restore timing-moved inst '{}' to ({},{}) pos {}",
                snapshot.inst->makeName(FULL_NAME_LIMIT), snapshot.coord.x,
                snapshot.coord.y, snapshot.pos);
        }
    };

    auto move_one = [&](rtl::Inst& inst, Coord direction,
                        TimingPlacementSnapshot& snapshot) {
        ++refinement.attempted_cells;
        if (!inst.tile.peer || inst.outline.fixed
            || inst.tile->hasOccupiedElementNeighbors(&inst)
            || (direction.x == 0 && direction.y == 0)) {
            return false;
        }
        Coord target = inst.coord + direction;
        if (target.x < 0 || target.x >= fpga_width
            || target.y < 0 || target.y >= fpga_height) {
            return false;
        }
        fpga::Tile& target_tile = (*tile_grid)[target.y*fpga_width + target.x];
        if (target_tile.coord.x < 0 || target_tile.coord.y < 0
            || !target_tile.tile_type) {
            return false;
        }

        snapshot = TimingPlacementSnapshot{
            .inst = &inst,
            .tile = inst.tile.peer,
            .coord = inst.coord,
            .pos = inst.pos,
        };
        snapshot.tile->unassign(&inst);
        int new_pos = target_tile.tryAdd(&inst, false);
        if (new_pos < 0) {
            return false;
        }
        inst.coord = target_tile.coord;
        inst.pos = new_pos;
        inst.outline.x = (target_tile.coord.x + 0.25F*(new_pos%4))
            / std::max(aspect_x, 0.0001F);
        inst.outline.y = (target_tile.coord.y + 0.25F*(new_pos/4))
            / std::max(aspect_y, 0.0001F);
        return true;
    };

    for (size_t pass = 0; pass < max_passes
         && current.violated_endpoints != 0; ++pass) {
        std::vector<TimingPlacementSnapshot> pass_moves;
        std::unordered_set<rtl::Inst*> moved_this_pass;
        size_t anchors = 0;

        for (const PlaceTimingForce& force : current.forces) {
            if (anchors >= anchor_limit) {
                break;
            }
            rtl::Inst* anchor = force.inst;
            if (!anchor || moved_this_pass.contains(anchor)
                || !anchor->tile.peer || anchor->outline.fixed) {
                continue;
            }
            Coord direction{
                (force.x > 0) - (force.x < 0),
                (force.y > 0) - (force.y < 0),
            };
            if (direction.x == 0 && direction.y == 0) {
                continue;
            }

            std::vector<rtl::Inst*> constellation = timingConstellation(
                *anchor, force.strongest_peer, tech);
            std::ranges::sort(constellation,
                [&](rtl::Inst* left, rtl::Inst* right) {
                    int left_projection = left->coord.x*direction.x
                        + left->coord.y*direction.y;
                    int right_projection = right->coord.x*direction.x
                        + right->coord.y*direction.y;
                    return left_projection > right_projection;
                });

            size_t move_begin = pass_moves.size();
            bool anchor_moved = false;
            bool constellation_rejected = false;
            for (rtl::Inst* member : constellation) {
                if (!member || moved_this_pass.contains(member)) {
                    continue;
                }
                TimingPlacementSnapshot snapshot;
                if (move_one(*member, direction, snapshot)) {
                    anchor_moved |= member == anchor;
                    moved_this_pass.insert(member);
                    pass_moves.push_back(snapshot);
                }
                else if (snapshot.inst) {
                    // A failed target leaves this member unassigned. Roll back
                    // it together with earlier members of this constellation.
                    pass_moves.push_back(snapshot);
                    constellation_rejected = true;
                    break;
                }
            }
            if (!anchor_moved || constellation_rejected) {
                std::vector<TimingPlacementSnapshot> rejected(
                    pass_moves.begin() + static_cast<std::ptrdiff_t>(move_begin),
                    pass_moves.end());
                restore(rejected);
                for (const TimingPlacementSnapshot& snapshot : rejected) {
                    moved_this_pass.erase(snapshot.inst);
                }
                pass_moves.resize(move_begin);
                continue;
            }
            ++anchors;
        }

        if (pass_moves.empty()) {
            break;
        }

        PlaceTimingAnalysis candidate = place_timing.analyze(timings);
        if (!timingObjectiveImproved(current, candidate)) {
            refinement.reverted_cells += pass_moves.size();
            restore(pass_moves);
            if (anchor_limit > 1) {
                anchor_limit = std::max<size_t>(1, anchor_limit/2);
                continue;
            }
            break;
        }

        refinement.moved_cells += pass_moves.size();
        ++refinement.passes;
        std::print(
            "\nPLACE_TIMING_PASS pass={} moved={} violations={}->{} worst_slack_ns={:.3f}->{:.3f} tns_ns={:.3f}->{:.3f}",
            refinement.passes, pass_moves.size(), current.violated_endpoints,
            candidate.violated_endpoints, current.worst_slack_ns,
            candidate.worst_slack_ns, current.total_negative_slack_ns,
            candidate.total_negative_slack_ns);
        current = std::move(candidate);
    }

    refinement.after = std::move(current);
    refinement.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    std::print(
        "\nPLACE_TIMING_SUMMARY endpoints={} nodes={} edges={} violations={}->{} worst_slack_ns={:.3f}->{:.3f} tns_ns={:.3f}->{:.3f} passes={} attempted={} moved={} reverted={} elapsed_ms={:.3f}\n",
        refinement.after.endpoints, refinement.after.evaluated_nodes,
        refinement.after.evaluated_edges, refinement.before.violated_endpoints,
        refinement.after.violated_endpoints, refinement.before.worst_slack_ns,
        refinement.after.worst_slack_ns,
        refinement.before.total_negative_slack_ns,
        refinement.after.total_negative_slack_ns, refinement.passes,
        refinement.attempted_cells, refinement.moved_cells,
        refinement.reverted_cells, refinement.elapsed_ms);
    std::fflush(stdout);
    // The refinement result is a summary. Retaining every endpoint's complete
    // critical path after placement can consume gigabytes on large designs.
    refinement.before.endpoint_details = {};
    refinement.before.forces = {};
    refinement.after.endpoint_details = {};
    refinement.after.forces = {};
    return refinement;
}

void PlaceDesign::recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    if (inst.cell_ref->type.find("BUF") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 255, 255);
    }
    else if (inst.cell_ref->type.find("LUT") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 0, 255);
    }
    else {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 0, 255, 255);
    }

//    std::print("set_property LOC SLICE_X{}Y{} [get_cells {}]\n", (int)(inst.outline.x*aspect_x*aspect_x/10), (int)(inst.outline.y*aspect_y*aspect_y/10), inst.makeName(1000));

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;

            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDrawDesign(*peer, nullptr, depth + 1);
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDrawDesign(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}
