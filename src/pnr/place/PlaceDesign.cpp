#include "PlaceDesign.h"
#include "Device.h"
#include "Tech.h"
#include "on_return.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

#ifndef SCALEPNR_PLACE_REDISTRIBUTION_STEP
#define SCALEPNR_PLACE_REDISTRIBUTION_STEP 0.20
#endif

#ifndef SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR
#define SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR 2.0
#endif

static_assert(SCALEPNR_PLACE_REDISTRIBUTION_STEP > 0.0,
    "placement redistribution scale must be positive");
static_assert(SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR > 0.0,
    "placement redistribution pass factor must be positive");

using namespace pnr;

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

bool isPlacementAttractor(const rtl::Inst& inst)
{
    return inst.cell_ref.peer
        && technology::Tech::clocked_ports.find(inst.cell_ref.peer->type)
            != technology::Tech::clocked_ports.end();
}

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

double PlaceDesign::timingPlacementCost(rtl::Inst& inst,
                                        const Coord& candidate) const
{
    double cost = 0;
    for (rtl::Inst* peer : timingPeers(inst, tech)) {
        if (!peer || peer == &inst) continue;
        Coord peer_coord;
        if (peer->tile.peer) {
            peer_coord = peer->coord;
        }
        else {
            peer_coord = {
                std::clamp(static_cast<int>(peer->outline.x*aspect_x),
                           0, fpga_width - 1),
                std::clamp(static_cast<int>(peer->outline.y*aspect_y),
                           0, fpga_height - 1),
            };
        }
        int dx = std::abs(candidate.x - peer_coord.x);
        int dy = std::abs(candidate.y - peer_coord.y);
        double delay = dx*place_timing.calibration.horizontal_ns_per_tile
            + dy*place_timing.calibration.vertical_ns_per_tile;
        if (dx != 0 && dy != 0) {
            delay += place_timing.calibration.bend_ns;
        }
        cost += place_timing.placementNetWeight(inst, *peer)*delay;
    }
    return cost;
}

PlacePreSmearResult PlaceDesign::preSmearBunches(
    const std::vector<rtl::Inst*>& cells)
{
    PlacePreSmearResult result;
    bunch_reservations.clear();
    bunch_reservation_order.clear();
    if (!tile_grid || fpga_width <= 0 || fpga_height <= 0) {
        return result;
    }

    constexpr int type_count = fpga::ELEMENT_TYPE_COUNT;
    using TypeCounts = std::array<uint32_t, type_count>;
    const size_t tile_count = static_cast<size_t>(
        fpga_width*fpga_height);
    std::vector<TypeCounts> remaining(tile_count);
    TypeCounts maximum_tile_capacity{};
    for (fpga::Tile& tile : *tile_grid) {
        if (!tile.tile_type || tile.coord.x < 0 || tile.coord.y < 0
            || tile.coord.x >= fpga_width
            || tile.coord.y >= fpga_height) {
            continue;
        }
        // Initializing the compact element masks also subtracts any fixed
        // cells which were packed before general placement.
        for (int type = 0; type < type_count; ++type) {
            tile.hasFreeElement(static_cast<fpga::ElementType>(type));
        }
        size_t index = static_cast<size_t>(
            tile.coord.y*fpga_width + tile.coord.x);
        for (int type = 0; type < type_count; ++type) {
            uint32_t available = std::popcount(static_cast<unsigned>(
                tile.elements_free[type]));
            remaining[index][type] = available;
            maximum_tile_capacity[type] = std::max(
                maximum_tile_capacity[type], available);
        }
    }

    struct AtomicBunch
    {
        struct PatternTile
        {
            Coord offset{0, 0};
            TypeCounts demand{};
        };
        RegBunch* bunch = nullptr;
        std::vector<rtl::Inst*> members;
        TypeCounts demand{};
        std::vector<PatternTile> pattern;
        Coord preferred{-1, -1};
        int pattern_min_x = 0;
        int pattern_max_x = 0;
        int pattern_min_y = 0;
        int pattern_max_y = 0;
        bool exact_pattern = true;
        size_t order = 0;
        bool fixed = false;
    };
    std::vector<AtomicBunch> groups;
    std::unordered_map<RegBunch*, size_t> group_by_bunch;
    group_by_bunch.reserve(cells.size()/2 + 1);
    for (rtl::Inst* inst : cells) {
        if (!inst || !inst->bunch_ref.peer || inst->tile.peer) continue;
        std::optional<fpga::ElementType> type =
            fpga::elementTypeForInst(*inst);
        if (!type) continue;
        RegBunch* bunch = inst->bunch_ref.peer;
        auto [where, inserted] = group_by_bunch.emplace(
            bunch, groups.size());
        if (inserted) {
            AtomicBunch group;
            group.bunch = bunch;
            group.order = groups.size();
            group.preferred = {
                std::clamp(static_cast<int>(bunch->x*aspect_x),
                           0, fpga_width - 1),
                std::clamp(static_cast<int>(bunch->y*aspect_y),
                           0, fpga_height - 1),
            };
            group.fixed = bunch->fixed || inst->outline.fixed;
            groups.push_back(std::move(group));
        }
        AtomicBunch& group = groups[where->second];
        group.members.push_back(inst);
        ++group.demand[*type];
        group.fixed = group.fixed || inst->outline.fixed;
    }

    for (AtomicBunch& group : groups) {
        bool first_offset = true;
        for (rtl::Inst* member : group.members) {
            std::optional<fpga::ElementType> type =
                fpga::elementTypeForInst(*member);
            if (!type) continue;
            Coord member_coord{
                std::clamp(static_cast<int>(member->outline.x*aspect_x),
                           0, fpga_width - 1),
                std::clamp(static_cast<int>(member->outline.y*aspect_y),
                           0, fpga_height - 1),
            };
            Coord offset{
                member_coord.x - group.preferred.x,
                member_coord.y - group.preferred.y,
            };
            auto pattern = std::find_if(
                group.pattern.begin(), group.pattern.end(),
                [&](const AtomicBunch::PatternTile& tile) {
                    return tile.offset.x == offset.x
                        && tile.offset.y == offset.y;
                });
            if (pattern == group.pattern.end()) {
                group.pattern.push_back(
                    AtomicBunch::PatternTile{.offset = offset});
                pattern = std::prev(group.pattern.end());
            }
            ++pattern->demand[*type];
            if (first_offset) {
                group.pattern_min_x = group.pattern_max_x = offset.x;
                group.pattern_min_y = group.pattern_max_y = offset.y;
                first_offset = false;
            }
            else {
                group.pattern_min_x = std::min(
                    group.pattern_min_x, offset.x);
                group.pattern_max_x = std::max(
                    group.pattern_max_x, offset.x);
                group.pattern_min_y = std::min(
                    group.pattern_min_y, offset.y);
                group.pattern_max_y = std::max(
                    group.pattern_max_y, offset.y);
            }
        }
        for (const AtomicBunch::PatternTile& tile : group.pattern) {
            for (int type = 0; type < type_count; ++type) {
                if (tile.demand[type] > maximum_tile_capacity[type]) {
                    group.exact_pattern = false;
                }
            }
        }
    }

    std::stable_sort(groups.begin(), groups.end(),
        [](const AtomicBunch& left, const AtomicBunch& right) {
            if (left.preferred.y != right.preferred.y) {
                return left.preferred.y < right.preferred.y;
            }
            if (left.preferred.x != right.preferred.x) {
                return left.preferred.x < right.preferred.x;
            }
            return left.order < right.order;
        });

    // Keep one live preview transaction per Tile for the complete linear
    // traversal. Consequently every later bunch sees the exact element
    // positions reserved by every earlier bunch, including local-chain and
    // neighbor compatibility rather than only an arithmetic capacity count.
    std::vector<std::unique_ptr<fpga::ElementPackingPreview>> previews(
        tile_count);
    auto previewAt = [&](int x, int y) -> fpga::ElementPackingPreview& {
        size_t index = static_cast<size_t>(y*fpga_width + x);
        if (!previews[index]) {
            previews[index] = std::make_unique<fpga::ElementPackingPreview>(
                (*tile_grid)[index]);
        }
        return *previews[index];
    };

    auto reservePrecisely = [&](const AtomicBunch& group, int left,
                                int top, int width, int height,
                                std::vector<PlaceBunchReservation::Placement>&
                                    placements) {
        struct PreviewCheckpoint
        {
            fpga::ElementPackingPreview* preview = nullptr;
            size_t checkpoint = 0;
        };
        std::vector<PreviewCheckpoint> checkpoints;
        auto remember = [&](fpga::ElementPackingPreview& preview) {
            auto found = std::find_if(checkpoints.begin(), checkpoints.end(),
                [&](const PreviewCheckpoint& saved) {
                    return saved.preview == &preview;
                });
            if (found == checkpoints.end()) {
                checkpoints.push_back({&preview, preview.checkpoint()});
            }
        };
        auto rollback = [&]() {
            for (auto saved = checkpoints.rbegin();
                 saved != checkpoints.rend(); ++saved) {
                saved->preview->rollback(saved->checkpoint);
            }
            placements.clear();
        };

        if (group.exact_pattern) {
            struct TileMembers
            {
                Coord coord{-1, -1};
                std::vector<rtl::Inst*> members;
            };
            std::vector<TileMembers> tile_members;
            for (rtl::Inst* member : group.members) {
                Coord member_coord{
                    std::clamp(static_cast<int>(member->outline.x*aspect_x),
                               0, fpga_width - 1),
                    std::clamp(static_cast<int>(member->outline.y*aspect_y),
                               0, fpga_height - 1),
                };
                Coord offset{member_coord.x - group.preferred.x,
                             member_coord.y - group.preferred.y};
                Coord coord{left + offset.x - group.pattern_min_x,
                            top + offset.y - group.pattern_min_y};
                auto bucket = std::find_if(tile_members.begin(),
                    tile_members.end(), [&](const TileMembers& candidate) {
                        return candidate.coord.x == coord.x
                            && candidate.coord.y == coord.y;
                    });
                if (bucket == tile_members.end()) {
                    tile_members.push_back(TileMembers{.coord = coord});
                    bucket = std::prev(tile_members.end());
                }
                bucket->members.push_back(member);
            }
            for (TileMembers& bucket : tile_members) {
                fpga::ElementPackingPreview& preview = previewAt(
                    bucket.coord.x, bucket.coord.y);
                remember(preview);
                std::vector<fpga::ElementPackingChoice> choices;
                if (!preview.reservePack(bucket.members, choices, false)) {
                    rollback();
                    return false;
                }
                for (const fpga::ElementPackingChoice& choice : choices) {
                    placements.push_back({
                        .inst = choice.inst,
                        .coord = bucket.coord,
                        .pos = choice.pos,
                    });
                }
            }
            return placements.size() == group.members.size();
        }

        // Oversized bunches cannot fit one Tile. Keep the bunch atomic while
        // reserving each member in a compact row-major envelope; right-first
        // and down-first envelope shapes are selected 50/50 by the caller.
        for (rtl::Inst* member : group.members) {
            bool placed = false;
            for (int y = top; y < top + height && !placed; ++y) {
                for (int x = left; x < left + width; ++x) {
                    fpga::ElementPackingPreview& preview = previewAt(x, y);
                    remember(preview);
                    int pos = preview.reserve(member, false);
                    if (pos < 0) continue;
                    placements.push_back({
                        .inst = member,
                        .coord = {x, y},
                        .pos = pos,
                    });
                    placed = true;
                    break;
                }
            }
            if (!placed) {
                rollback();
                return false;
            }
        }
        return true;
    };

    auto regionFits = [&](const AtomicBunch& group, int left, int top,
                          int width, int height) {
        if (left < 0 || top < 0 || width <= 0 || height <= 0
            || left + width > fpga_width
            || top + height > fpga_height) {
            return false;
        }
        if (group.exact_pattern) {
            for (const AtomicBunch::PatternTile& pattern : group.pattern) {
                int x = left + pattern.offset.x - group.pattern_min_x;
                int y = top + pattern.offset.y - group.pattern_min_y;
                if (x < 0 || x >= fpga_width
                    || y < 0 || y >= fpga_height) {
                    return false;
                }
                const TypeCounts& tile = remaining[static_cast<size_t>(
                    y*fpga_width + x)];
                for (int type = 0; type < type_count; ++type) {
                    if (tile[type] < pattern.demand[type]) return false;
                }
            }
            return true;
        }
        TypeCounts available{};
        for (int y = top; y < top + height; ++y) {
            for (int x = left; x < left + width; ++x) {
                const TypeCounts& tile = remaining[static_cast<size_t>(
                    y*fpga_width + x)];
                for (int type = 0; type < type_count; ++type) {
                    available[type] += tile[type];
                }
            }
        }
        for (int type = 0; type < type_count; ++type) {
            if (available[type] < group.demand[type]) return false;
        }
        return true;
    };

    auto reserveRegion = [&](const AtomicBunch& group, int left, int top,
                             int width, int height) {
        if (group.exact_pattern) {
            for (const AtomicBunch::PatternTile& pattern : group.pattern) {
                int x = left + pattern.offset.x - group.pattern_min_x;
                int y = top + pattern.offset.y - group.pattern_min_y;
                TypeCounts& tile = remaining[static_cast<size_t>(
                    y*fpga_width + x)];
                for (int type = 0; type < type_count; ++type) {
                    tile[type] -= pattern.demand[type];
                }
            }
            return;
        }
        TypeCounts needed = group.demand;
        for (int y = top; y < top + height; ++y) {
            for (int x = left; x < left + width; ++x) {
                TypeCounts& tile = remaining[static_cast<size_t>(
                    y*fpga_width + x)];
                for (int type = 0; type < type_count; ++type) {
                    uint32_t consumed = std::min(tile[type], needed[type]);
                    tile[type] -= consumed;
                    needed[type] -= consumed;
                }
            }
        }
    };

    for (AtomicBunch& group : groups) {
        ++result.bunches;
        size_t member_count = group.members.size();
        result.reserved_cells += member_count;
        uint32_t minimum_tiles = 1;
        bool type_available = true;
        for (int type = 0; type < type_count; ++type) {
            if (group.demand[type] == 0) continue;
            if (maximum_tile_capacity[type] == 0) {
                type_available = false;
                break;
            }
            minimum_tiles = std::max(minimum_tiles,
                (group.demand[type] + maximum_tile_capacity[type] - 1)
                    / maximum_tile_capacity[type]);
        }
        if (!type_available) {
            ++result.failed_bunches;
            continue;
        }

        bool prefer_horizontal = result.right_first_bunches
            <= result.down_first_bunches;
        Coord selected{-1, -1};
        int selected_width = 0;
        int selected_height = 0;
        auto searchShape = [&](int width, int height) {
            if (selected.x >= 0 || width <= 0 || height <= 0
                || width > fpga_width || height > fpga_height) {
                return;
            }
            int base_x = group.exact_pattern
                ? group.preferred.x + group.pattern_min_x
                : std::clamp(group.preferred.x - (width - 1)/2,
                             0, fpga_width - width);
            int base_y = group.exact_pattern
                ? group.preferred.y + group.pattern_min_y
                : std::clamp(group.preferred.y - (height - 1)/2,
                             0, fpga_height - height);
            if (group.fixed) {
                if (regionFits(group, base_x, base_y, width, height)) {
                    selected = {base_x, base_y};
                    selected_width = width;
                    selected_height = height;
                }
                return;
            }

            // Process bunches in row-major order. When their requested
            // regions collide, move the complete current bunch only toward
            // later Tiles: right and down are alternated to avoid a one-axis
            // density bias.
            for (int distance = 0;
                 distance <= fpga_width + fpga_height
                    && selected.x < 0;
                 ++distance) {
                for (int split = 0; split <= distance; ++split) {
                    int dx = prefer_horizontal
                        ? distance - split : split;
                    int dy = distance - dx;
                    int left = base_x + dx;
                    int top = base_y + dy;
                    if (regionFits(group, left, top, width, height)) {
                        selected = {left, top};
                        selected_width = width;
                        selected_height = height;
                        break;
                    }
                }
            }
        };

        if (group.exact_pattern) {
            int pattern_width =
                group.pattern_max_x - group.pattern_min_x + 1;
            int pattern_height =
                group.pattern_max_y - group.pattern_min_y + 1;
            searchShape(pattern_width, pattern_height);
            if (selected.x < 0 && !group.fixed) {
                int base_x = group.preferred.x + group.pattern_min_x;
                int base_y = group.preferred.y + group.pattern_min_y;
                // A bunch at the right/bottom boundary may have no legal
                // monotonic move. Complete nearest rings only as a boundary
                // fallback so a feasible design is not rejected merely due
                // to traversal direction.
                for (int radius = 1;
                     radius <= fpga_width + fpga_height
                        && selected.x < 0; ++radius) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        int dy = radius - std::abs(dx);
                        for (int sign : {-1, 1}) {
                            if (dy == 0 && sign == 1) continue;
                            int left = base_x + dx;
                            int top = base_y + sign*dy;
                            if (regionFits(group, left, top,
                                           pattern_width,
                                           pattern_height)) {
                                selected = {left, top};
                                selected_width = pattern_width;
                                selected_height = pattern_height;
                                break;
                            }
                        }
                        if (selected.x >= 0) break;
                    }
                }
            }
        }
        else {
            uint32_t maximum_compact_area = std::min<uint32_t>(
                static_cast<uint32_t>(fpga_width*fpga_height),
                std::max<uint32_t>(minimum_tiles + 8, minimum_tiles*2));
            for (uint32_t area = minimum_tiles;
                 area <= maximum_compact_area && selected.x < 0; ++area) {
                int long_side = static_cast<int>(std::ceil(std::sqrt(area)));
                int short_side = static_cast<int>(
                    (area + long_side - 1)/long_side);
                if (prefer_horizontal) {
                    searchShape(long_side, short_side);
                    if (long_side != short_side) {
                        searchShape(short_side, long_side);
                    }
                }
                else {
                    searchShape(short_side, long_side);
                    if (long_side != short_side) {
                        searchShape(long_side, short_side);
                    }
                }
            }
            // Fragmented heterogeneous resources can require a larger
            // envelope than the ideal capacity quotient.
            for (int radius = 1; selected.x < 0
                 && radius <= std::max(fpga_width, fpga_height); ++radius) {
                int width = std::min(fpga_width,
                    static_cast<int>(std::ceil(std::sqrt(minimum_tiles)))
                        + radius);
                int height = std::min(fpga_height,
                    static_cast<int>((minimum_tiles + width - 1)/width)
                        + radius);
                searchShape(width, height);
                if (selected.x < 0) searchShape(height, width);
            }
        }
        if (selected.x < 0
            && regionFits(group, 0, 0, fpga_width, fpga_height)) {
            selected = {0, 0};
            selected_width = fpga_width;
            selected_height = fpga_height;
        }
        if (selected.x < 0) {
            ++result.failed_bunches;
            continue;
        }

        std::vector<PlaceBunchReservation::Placement> precise_placements;
        bool precisely_reserved = reservePrecisely(
            group, selected.x, selected.y,
            selected_width, selected_height, precise_placements);
        if (!precisely_reserved && !group.fixed) {
            struct PreciseCandidate
            {
                Coord coord{-1, -1};
                int monotonic = 0;
                int distance = 0;
            };
            int base_x = group.exact_pattern
                ? group.preferred.x + group.pattern_min_x
                : std::clamp(group.preferred.x - (selected_width - 1)/2,
                             0, fpga_width - selected_width);
            int base_y = group.exact_pattern
                ? group.preferred.y + group.pattern_min_y
                : std::clamp(group.preferred.y - (selected_height - 1)/2,
                             0, fpga_height - selected_height);
            std::vector<PreciseCandidate> candidates;
            for (int y = 0; y + selected_height <= fpga_height; ++y) {
                for (int x = 0; x + selected_width <= fpga_width; ++x) {
                    if ((x == selected.x && y == selected.y)
                        || !regionFits(group, x, y,
                                       selected_width, selected_height)) {
                        continue;
                    }
                    candidates.push_back({
                        .coord = {x, y},
                        .monotonic = x >= base_x && y >= base_y ? 0 : 1,
                        .distance = std::abs(x - base_x)
                            + std::abs(y - base_y),
                    });
                }
            }
            std::stable_sort(candidates.begin(), candidates.end(),
                [&](const PreciseCandidate& left,
                    const PreciseCandidate& right) {
                    if (left.monotonic != right.monotonic) {
                        return left.monotonic < right.monotonic;
                    }
                    if (left.distance != right.distance) {
                        return left.distance < right.distance;
                    }
                    int left_primary = prefer_horizontal
                        ? left.coord.y : left.coord.x;
                    int right_primary = prefer_horizontal
                        ? right.coord.y : right.coord.x;
                    return left_primary < right_primary;
                });
            for (const PreciseCandidate& candidate : candidates) {
                if (reservePrecisely(group, candidate.coord.x,
                                     candidate.coord.y, selected_width,
                                     selected_height,
                                     precise_placements)) {
                    selected = candidate.coord;
                    precisely_reserved = true;
                    break;
                }
            }
        }
        if (!precisely_reserved) {
            ++result.failed_bunches;
            continue;
        }
        reserveRegion(group, selected.x, selected.y,
                      selected_width, selected_height);
        double old_x = group.bunch->x*aspect_x;
        double old_y = group.bunch->y*aspect_y;
        double new_x = group.exact_pattern
            ? old_x + selected.x
                - (group.preferred.x + group.pattern_min_x)
            : selected.x + 0.5*(selected_width - 1);
        double new_y = group.exact_pattern
            ? old_y + selected.y
                - (group.preferred.y + group.pattern_min_y)
            : selected.y + 0.5*(selected_height - 1);
        double delta_x = new_x - old_x;
        double delta_y = new_y - old_y;
        if (!group.fixed && (std::abs(delta_x) > 0.000001
                             || std::abs(delta_y) > 0.000001)) {
            ++result.moved_bunches;
            int shift_x = static_cast<int>(std::lround(
                delta_x));
            int shift_y = static_cast<int>(std::lround(
                delta_y));
            if (shift_x > 0) {
                ++result.moved_right;
            }
            if (shift_y > 0) {
                ++result.moved_down;
            }
            if (prefer_horizontal) {
                ++result.right_first_bunches;
            }
            else {
                ++result.down_first_bunches;
            }
            result.maximum_shift = std::max(result.maximum_shift,
                std::max(std::abs(shift_x), std::abs(shift_y)));
        }
        if (!group.fixed) {
            // Publish one rigid translation after the reservation is frozen.
            // No member can observe or react to another member moving during
            // this phase.
            for (rtl::Inst* member : group.members) {
                member->outline.x += static_cast<float>(
                    delta_x/std::max(aspect_x, 0.0001F));
                member->outline.y += static_cast<float>(
                    delta_y/std::max(aspect_y, 0.0001F));
            }
            group.bunch->x = static_cast<float>(
                new_x/std::max(aspect_x, 0.0001F));
            group.bunch->y = static_cast<float>(
                new_y/std::max(aspect_y, 0.0001F));
        }
        bunch_reservations[group.bunch] = PlaceBunchReservation{
            .minimum = selected,
            .maximum = {
                selected.x + selected_width - 1,
                selected.y + selected_height - 1,
            },
            .demand = group.demand,
            .placements = std::move(precise_placements),
            .cells = member_count,
        };
        bunch_reservation_order.push_back(group.bunch);
    }

    std::print(
        "\nPLACE_PRE_SMEAR bunches={} moved={} right_first={} down_first={} used_right={} used_down={} cells={} failed={} maximum_shift={}",
        result.bunches, result.moved_bunches, result.right_first_bunches,
        result.down_first_bunches, result.moved_right,
        result.moved_down, result.reserved_cells,
        result.failed_bunches, result.maximum_shift);
    return result;
}

size_t PlaceDesign::commitPreSmearReservations()
{
    size_t committed = 0;
    for (RegBunch* bunch : bunch_reservation_order) {
        auto found = bunch_reservations.find(bunch);
        if (found == bunch_reservations.end()) continue;
        for (const PlaceBunchReservation::Placement& placement
             : found->second.placements) {
            if (!placement.inst || placement.inst->tile.peer) continue;
            PNR_ASSERT(placement.coord.x >= 0
                    && placement.coord.x < fpga_width
                    && placement.coord.y >= 0
                    && placement.coord.y < fpga_height,
                "invalid pre-smear reservation for '{}' at ({},{})",
                placement.inst->makeName(FULL_NAME_LIMIT),
                placement.coord.x, placement.coord.y);
            fpga::Tile& tile = (*tile_grid)[static_cast<size_t>(
                placement.coord.y*fpga_width + placement.coord.x)];
            int pos = tile.tryAddAt(placement.inst, placement.pos, false);
            PNR_ASSERT(pos == placement.pos,
                "precise pre-smear reservation became illegal for '{}' "
                "at ({},{}) pos={} result={}",
                placement.inst->makeName(FULL_NAME_LIMIT),
                placement.coord.x, placement.coord.y,
                placement.pos, pos);
            ++committed;
            ++place_commits;
            ++place_tile_trials;
        }
    }
    std::print("\nPLACE_PRE_PACK committed={}", committed);
    return committed;
}

std::vector<PlacePredictedMove> PlaceDesign::calculatePredictedDirections(
    const std::vector<rtl::Inst*>& cells,
    const std::unordered_map<rtl::Inst*, Coord>& predicted,
    size_t& oversubscribed_groups) const
{
    constexpr int type_count = fpga::ELEMENT_TYPE_COUNT;
    using TypeCounts = std::array<int, type_count>;
    const size_t tile_count = static_cast<size_t>(fpga_width*fpga_height);
    std::vector<TypeCounts> capacity(tile_count);
    std::vector<TypeCounts> occupancy(tile_count);
    std::vector<std::vector<rtl::Inst*>> groups(
        tile_count*static_cast<size_t>(type_count));

    for (const fpga::Tile& tile : *tile_grid) {
        if (!tile.tile_type || tile.coord.x < 0 || tile.coord.y < 0
            || tile.coord.x >= fpga_width || tile.coord.y >= fpga_height) {
            continue;
        }
        std::array<uint16_t, type_count> masks{};
        for (const fpga::Element& element : tile.tile_type->elements) {
            if (element.bitmap_pos < fpga::ELEMENT_BITMAP_BITS) {
                masks[element.type] |= static_cast<uint16_t>(
                    1U << element.bitmap_pos);
            }
        }
        size_t tile_index = static_cast<size_t>(
            tile.coord.y*fpga_width + tile.coord.x);
        for (int type = 0; type < type_count; ++type) {
            capacity[tile_index][type] = std::popcount(
                static_cast<unsigned>(masks[type]));
        }
    }

    for (rtl::Inst* inst : cells) {
        if (!inst) continue;
        std::optional<fpga::ElementType> type =
            fpga::elementTypeForInst(*inst);
        auto coordinate = predicted.find(inst);
        if (!type || coordinate == predicted.end()) continue;
        const Coord& coord = coordinate->second;
        if (coord.x < 0 || coord.x >= fpga_width
            || coord.y < 0 || coord.y >= fpga_height) {
            continue;
        }
        size_t tile_index = static_cast<size_t>(
            coord.y*fpga_width + coord.x);
        ++occupancy[tile_index][*type];
        groups[tile_index*type_count + *type].push_back(inst);
    }

    std::vector<size_t> overfull_keys;
    for (size_t tile_index = 0; tile_index < tile_count; ++tile_index) {
        for (int type = 0; type < type_count; ++type) {
            if (occupancy[tile_index][type] > capacity[tile_index][type]) {
                overfull_keys.push_back(tile_index*type_count + type);
            }
        }
    }
    oversubscribed_groups = overfull_keys.size();
    if (overfull_keys.empty()) return {};

    // Keep the completed pass recorded in the cells after legalization. Clear
    // it only when a new prediction pass is actually about to replace it.
    for (rtl::Inst* inst : cells) {
        if (inst) inst->placement_motion = {};
    }

    // Capacity pressure starts the cooling region.  Continue through several
    // register tiers so every affected register computes its own force from
    // the frozen placement.  A downstream register must not merely inherit
    // the upstream register's direction: that was the one-tier behavior that
    // dragged complete constellations to an edge.
    struct ActiveSearch {
        rtl::Inst* inst = nullptr;
        int depth = 0;
        int register_tier = 0;
    };
    constexpr int active_connection_depth = 16;
    constexpr int active_register_tiers = 4;
    std::unordered_set<rtl::Inst*> visited;
    std::unordered_set<rtl::Inst*> active_attractors;
    std::deque<ActiveSearch> pending;
    visited.reserve(cells.size());
    active_attractors.reserve(cells.size()/2);
    for (size_t key : overfull_keys) {
        for (rtl::Inst* inst : groups[key]) {
            if (inst && visited.insert(inst).second) {
                pending.push_back({inst, 0, 0});
            }
        }
    }
    while (!pending.empty()) {
        ActiveSearch current = pending.front();
        pending.pop_front();
        if (!current.inst || current.inst->outline.fixed
            || current.inst->tile.peer) {
            continue;
        }
        if (isPlacementAttractor(*current.inst)) {
            active_attractors.insert(current.inst);
        }
        if (current.depth >= active_connection_depth
            || current.register_tier >= active_register_tiers) {
            continue;
        }
        for (rtl::Inst* peer : timingPeers(*current.inst, tech)) {
            if (!peer || peer->outline.fixed || peer->tile.peer
                || !visited.insert(peer).second) {
                continue;
            }
            pending.push_back({
                peer,
                current.depth + 1,
                current.register_tier
                    + (isPlacementAttractor(*peer) ? 1 : 0),
            });
        }
    }

    auto traceMarkerActivation = [&](const char* label, rtl::Inst* inst) {
        if (!inst) return;
        auto coordinate = predicted.find(inst);
        std::optional<fpga::ElementType> type =
            fpga::elementTypeForInst(*inst);
        int occupied = -1;
        int available = -1;
        bool overfull = false;
        if (coordinate != predicted.end() && type
            && coordinate->second.x >= 0 && coordinate->second.x < fpga_width
            && coordinate->second.y >= 0 && coordinate->second.y < fpga_height) {
            size_t tile_index = static_cast<size_t>(
                coordinate->second.y*fpga_width + coordinate->second.x);
            occupied = occupancy[tile_index][*type];
            available = capacity[tile_index][*type];
            overfull = occupied > available;
        }
        std::print(
            "\nPLACE_MARKER_ACTIVE marker={} inst='{}' cell_type='{}' pos=({}, {}) element_type={} occupancy={}/{} own_group_overfull={} reached={} attractor={} active_attractor={} fixed={} bunch=({:.3f},{:.3f}) bunch_reg='{}'",
            label, inst->makeName(FULL_NAME_LIMIT),
            inst->cell_ref.peer ? inst->cell_ref->type : "<none>",
            coordinate != predicted.end() ? coordinate->second.x : -1,
            coordinate != predicted.end() ? coordinate->second.y : -1,
            type ? static_cast<int>(*type) : -1, occupied, available,
            overfull, visited.contains(inst), isPlacementAttractor(*inst),
            active_attractors.contains(inst), inst->outline.fixed,
            inst->bunch_ref.peer ? inst->bunch_ref->x : -1.0F,
            inst->bunch_ref.peer ? inst->bunch_ref->y : -1.0F,
            inst->bunch_ref.peer && inst->bunch_ref->reg
                ? inst->bunch_ref->reg->makeName(FULL_NAME_LIMIT)
                : "<none>");
        if (coordinate == predicted.end()) return;
        size_t peer_index = 0;
        for (rtl::Inst* peer : timingPeers(*inst, tech)) {
            auto peer_coordinate = predicted.find(peer);
            std::print(
                "\nPLACE_MARKER_PEER marker={} peer={} name='{}' type='{}' pos=({}, {}) delta=({}, {}) weight={:.4f} attractor={} fixed={}",
                label, peer_index++,
                peer ? peer->makeName(FULL_NAME_LIMIT) : "<none>",
                peer && peer->cell_ref.peer ? peer->cell_ref->type : "<none>",
                peer_coordinate != predicted.end() ? peer_coordinate->second.x : -1,
                peer_coordinate != predicted.end() ? peer_coordinate->second.y : -1,
                peer_coordinate != predicted.end()
                    ? peer_coordinate->second.x - coordinate->second.x : 0,
                peer_coordinate != predicted.end()
                    ? peer_coordinate->second.y - coordinate->second.y : 0,
                peer ? place_timing.placementNetWeight(*inst, *peer) : 0.0,
                peer && isPlacementAttractor(*peer),
                peer ? peer->outline.fixed : false);
        }
    };
    traceMarkerActivation("A", movement_marker_a);
    traceMarkerActivation("B", movement_marker_b);

    std::vector<PlacePredictedMove> moves;
    moves.reserve(active_attractors.size());
    for (rtl::Inst* inst : active_attractors) {
        auto origin_it = predicted.find(inst);
        std::optional<fpga::ElementType> type = fpga::elementTypeForInst(*inst);
        if (origin_it == predicted.end() || !type) continue;
        const Coord origin = origin_it->second;
        double force_x = 0;
        double force_y = 0;
        size_t external_peers = 0;
        for (rtl::Inst* peer : timingPeers(*inst, tech)) {
            if (!peer) continue;
            auto peer_coord = predicted.find(peer);
            if (peer_coord == predicted.end()) continue;
            int dx = peer_coord->second.x - origin.x;
            int dy = peer_coord->second.y - origin.y;
            if (dx == 0 && dy == 0) continue;
            double weight = place_timing.placementNetWeight(*inst, *peer);
            force_x += dx*weight;
            force_y += dy*weight;
            ++external_peers;
        }
        if (external_peers != 0) {
            force_x /= external_peers;
            force_y /= external_peers;
        }
        Coord direction{
            (force_x > 0) - (force_x < 0),
            (force_y > 0) - (force_y < 0),
        };
        double acceleration = std::hypot(force_x, force_y);
        inst->placement_motion = rtl::PlacementMotion{
            .force_x = force_x,
            .force_y = force_y,
            .acceleration = acceleration,
            .direction = direction,
            .from = origin,
            .to = origin,
            .active = acceleration > 0,
        };
        if (acceleration <= 0) continue;
        moves.push_back(PlacePredictedMove{
            .inst = inst,
            .from = origin,
            .to = origin,
            .direction = direction,
            .type = *type,
            .external_force_x = force_x,
            .external_force_y = force_y,
            .external_peers = external_peers,
        });
    }
    auto traceMarkerForce = [&](const char* label, rtl::Inst* inst) {
        if (!inst) return;
        auto move = std::ranges::find_if(moves,
            [&](const PlacePredictedMove& candidate) {
                return candidate.inst == inst;
            });
        std::print(
            "\nPLACE_MARKER_FORCE marker={} inst='{}' has_direct_move={} force=({:.4f},{:.4f}) acceleration={:.4f} peers={}",
            label, inst->makeName(FULL_NAME_LIMIT), move != moves.end(),
            move != moves.end() ? move->external_force_x : 0.0,
            move != moves.end() ? move->external_force_y : 0.0,
            move != moves.end()
                ? std::hypot(move->external_force_x, move->external_force_y)
                : 0.0,
            move != moves.end() ? move->external_peers : 0);
    };
    traceMarkerForce("A", movement_marker_a);
    traceMarkerForce("B", movement_marker_b);
    return moves;
}

void PlaceDesign::drawPlacementSnapshot(
    const std::vector<rtl::Inst*>& cells, const std::string& filename,
    const std::unordered_map<rtl::Inst*, std::pair<double, double>>* movement,
    double progress,
    const std::unordered_map<rtl::Inst*, std::pair<double, double>>*
        absolute_positions)
{
    int zoom = std::max(1, static_cast<int>(std::round(image_zoom)));
    image.init(fpga_width*zoom, fpga_height*zoom);
    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            image.set_pixel(x, y, 10, 12, 18, 255);
        }
    }
    for (int x = 0; x <= fpga_width; ++x) {
        image.draw_line(x*zoom, 0, x*zoom, image.height - 1,
                        28, 30, 38, 255);
    }
    for (int y = 0; y <= fpga_height; ++y) {
        image.draw_line(0, y*zoom, image.width - 1, y*zoom,
                        28, 30, 38, 255);
    }

    auto physicalPosition = [&](rtl::Inst* inst) {
        std::pair<double, double> position = inst->tile.peer
            ? std::pair<double, double>{
                static_cast<double>(inst->coord.x),
                static_cast<double>(inst->coord.y)}
            : std::pair<double, double>{
                inst->outline.x*aspect_x, inst->outline.y*aspect_y};
        if (absolute_positions) {
            auto saved = absolute_positions->find(inst);
            if (saved != absolute_positions->end()) {
                return saved->second;
            }
        }
        if (movement) {
            auto found = movement->find(inst);
            if (found != movement->end()) {
                position.first += progress*found->second.first;
                position.second += progress*found->second.second;
            }
        }
        return position;
    };

    if (movement && progress > 0) {
        for (const auto& [inst, delta] : *movement) {
            if (!inst) continue;
            double from_x = inst->outline.x*aspect_x;
            double from_y = inst->outline.y*aspect_y;
            image.draw_line(
                static_cast<int>(from_x*zoom),
                static_cast<int>(from_y*zoom),
                static_cast<int>((from_x + progress*delta.first)*zoom),
                static_cast<int>((from_y + progress*delta.second)*zoom),
                isPlacementAttractor(*inst) ? 255 : 180,
                isPlacementAttractor(*inst) ? 115 : 180,
                35, 255);
        }
    }

    for (rtl::Inst* inst : cells) {
        if (!inst || !inst->cell_ref.peer) continue;
        auto position = physicalPosition(inst);
        uint8_t r = 70;
        uint8_t g = 130;
        uint8_t b = 255;
        if (inst->outline.fixed) {
            r = 0; g = 240; b = 255;
        }
        else if (movement && movement->contains(inst)) {
            if (isPlacementAttractor(*inst)) {
                r = 255; g = 130; b = 35;
            }
            else {
                r = 245; g = 225; b = 40;
            }
        }
        else if (inst->cell_ref.peer->type.find("LUT") != std::string::npos) {
            r = 40; g = 225; b = 95;
        }
        int px = static_cast<int>(position.first*zoom);
        int py = static_cast<int>(position.second*zoom);
        image.set_pixel(px, py, r, g, b, 255);
        image.set_pixel(px + 1, py, r, g, b, 255);
        image.set_pixel(px, py + 1, r, g, b, 255);
        image.set_pixel(px + 1, py + 1, r, g, b, 255);
    }

    // Keep one known-bad direct connection visible above the cell cloud in
    // every diagnostic frame. A is a magenta upward triangle, B a yellow
    // rectangle, and the white segment ties their exact positions.
    auto markerPosition = [&](rtl::Inst* inst) {
        return inst ? physicalPosition(inst) : std::pair<double, double>{-1, -1};
    };
    auto a_position = markerPosition(movement_marker_a);
    auto b_position = markerPosition(movement_marker_b);
    if (movement_marker_a && movement_marker_b) {
        int ax = static_cast<int>(a_position.first*zoom);
        int ay = static_cast<int>(a_position.second*zoom);
        int bx = static_cast<int>(b_position.first*zoom);
        int by = static_cast<int>(b_position.second*zoom);
        for (int offset = -1; offset <= 1; ++offset) {
            image.draw_line(ax + offset, ay, bx + offset, by,
                            255, 255, 255, 255);
        }
    }
    auto drawTriangle = [&](std::pair<double, double> position,
                            bool points_up,
                            uint8_t r, uint8_t g, uint8_t b) {
        if (position.first < 0 || position.second < 0) return;
        int exact_x = static_cast<int>(position.first*zoom);
        int exact_y = static_cast<int>(position.second*zoom);
        int half_width = std::max(9, zoom);
        int half_height = std::max(9, zoom);
        int center_x = std::clamp(
            exact_x, half_width + 3, image.width - half_width - 4);
        int center_y = std::clamp(
            exact_y, half_height + 3, image.height - half_height - 4);
        image.draw_line(exact_x, exact_y, center_x, center_y,
                        r, g, b, 255);
        for (int thickness = 0; thickness < 3; ++thickness) {
            int apex_y = points_up
                ? center_y - half_height + thickness
                : center_y + half_height - thickness;
            int base_y = points_up
                ? center_y + half_height - thickness
                : center_y - half_height + thickness;
            int left_x = center_x - half_width + thickness;
            int right_x = center_x + half_width - thickness;
            image.draw_line(center_x, apex_y, left_x, base_y,
                            r, g, b, 255);
            image.draw_line(center_x, apex_y, right_x, base_y,
                            r, g, b, 255);
            image.draw_line(left_x, base_y, right_x, base_y,
                            r, g, b, 255);
        }
    };
    auto drawRectangle = [&](std::pair<double, double> position,
                             uint8_t r, uint8_t g, uint8_t b) {
        if (position.first < 0 || position.second < 0) return;
        int exact_x = static_cast<int>(position.first*zoom);
        int exact_y = static_cast<int>(position.second*zoom);
        int half_width = std::max(9, zoom);
        int half_height = std::max(9, zoom);
        int center_x = std::clamp(
            exact_x, half_width + 3, image.width - half_width - 4);
        int center_y = std::clamp(
            exact_y, half_height + 3, image.height - half_height - 4);
        image.draw_line(exact_x, exact_y, center_x, center_y,
                        r, g, b, 255);
        for (int thickness = 0; thickness < 3; ++thickness) {
            int left = center_x - half_width + thickness;
            int right = center_x + half_width - thickness;
            int top = center_y - half_height + thickness;
            int bottom = center_y + half_height - thickness;
            image.draw_line(left, top, right, top, r, g, b, 255);
            image.draw_line(right, top, right, bottom, r, g, b, 255);
            image.draw_line(right, bottom, left, bottom, r, g, b, 255);
            image.draw_line(left, bottom, left, top, r, g, b, 255);
        }
    };
    drawTriangle(a_position, true, 255, 20, 220);
    drawRectangle(b_position, 255, 235, 20);
    image.write(filename);
    std::print("\nPLACE_MOVEMENT_PNG file='{}' progress={:.2f}",
               filename, progress);
}

void PlaceDesign::captureMovementSnapshot(
    const std::vector<rtl::Inst*>& cells, const std::string& filename,
    const std::unordered_map<rtl::Inst*, std::pair<double, double>>* movement,
    double progress)
{
    if (movement_png_prefix.empty()) return;
    if (movement_snapshot_cells.empty()) {
        movement_snapshot_cells = cells;
    }
    if (movement_snapshot_cells.size() != cells.size()) return;

    PlaceMovementFrame frame;
    frame.filename = filename;
    frame.progress = progress;
    frame.positions.reserve(cells.size());
    frame.active.reserve(cells.size());
    for (rtl::Inst* inst : cells) {
        std::pair<double, double> position{-1, -1};
        bool active = false;
        if (inst) {
            position = inst->tile.peer
                ? std::pair<double, double>{
                    static_cast<double>(inst->coord.x),
                    static_cast<double>(inst->coord.y)}
                : std::pair<double, double>{
                    inst->outline.x*aspect_x,
                    inst->outline.y*aspect_y};
            if (movement) {
                auto delta = movement->find(inst);
                if (delta != movement->end()) {
                    position.first += progress*delta->second.first;
                    position.second += progress*delta->second.second;
                    active = true;
                }
            }
        }
        frame.positions.push_back(position);
        frame.active.push_back(active);
    }
    movement_snapshots.push_back(std::move(frame));
}

std::string PlaceDesign::movementPngFilename(
    const std::string& stage_and_label) const
{
    size_t separator = movement_png_prefix.find_last_of("/\\");
    std::string directory = separator == std::string::npos
        ? std::string{} : movement_png_prefix.substr(0, separator + 1);
    std::string basename = separator == std::string::npos
        ? movement_png_prefix : movement_png_prefix.substr(separator + 1);
    return directory + stage_and_label + '_' + basename + ".png";
}

void PlaceDesign::redrawMovementSnapshotsWithMarkers()
{
    if (!movement_marker_a || !movement_marker_b
        || movement_snapshot_cells.empty()) {
        return;
    }
    for (const PlaceMovementFrame& frame : movement_snapshots) {
        if (frame.positions.size() != movement_snapshot_cells.size()
            || frame.active.size() != movement_snapshot_cells.size()) {
            continue;
        }
        std::unordered_map<rtl::Inst*, std::pair<double, double>> positions;
        std::unordered_map<rtl::Inst*, std::pair<double, double>> active;
        positions.reserve(movement_snapshot_cells.size());
        std::pair<double, double> marker_a{-1, -1};
        std::pair<double, double> marker_b{-1, -1};
        for (size_t index = 0; index < movement_snapshot_cells.size(); ++index) {
            rtl::Inst* inst = movement_snapshot_cells[index];
            if (!inst) continue;
            positions.emplace(inst, frame.positions[index]);
            if (inst == movement_marker_a) marker_a = frame.positions[index];
            if (inst == movement_marker_b) marker_b = frame.positions[index];
            if (frame.active[index]) {
                active.emplace(inst, std::pair{0.0, 0.0});
            }
        }
        drawPlacementSnapshot(
            movement_snapshot_cells, frame.filename,
            active.empty() ? nullptr : &active,
            frame.progress, &positions);
        if (marker_a.first >= 0 && marker_b.first >= 0) {
            std::print(
                "\nPLACE_MARKER_FRAME file='{}' A=({:.3f},{:.3f}) B=({:.3f},{:.3f}) manhattan={:.3f}",
                frame.filename, marker_a.first, marker_a.second,
                marker_b.first, marker_b.second,
                std::abs(marker_a.first - marker_b.first)
                    + std::abs(marker_a.second - marker_b.second));
        }
    }
}

size_t PlaceDesign::applyPredictedDisplacementSimultaneously(
    const std::vector<PlacePredictedMove>& moves,
    const std::vector<rtl::Inst*>* drawing_cells,
    double displacement_scale)
{
    struct Displacement
    {
        double x = 0;
        double y = 0;
        bool direct = false;
    };

    std::unordered_map<rtl::Inst*, Displacement> displacement;
    displacement.reserve(moves.size()*2);
    std::unordered_map<rtl::Inst*, double> follower_influence;
    follower_influence.reserve(moves.size());
    constexpr double propagation_decay = 0.5;
    constexpr double minimum_propagated_displacement = 0.01;
    constexpr double maximum_cell_displacement = 2.0;
    constexpr int maximum_propagation_depth = 16;
    if (displacement_scale <= 0) {
        displacement_scale = SCALEPNR_PLACE_REDISTRIBUTION_STEP;
    }

    for (const PlacePredictedMove& move : moves) {
        if (!move.inst || move.inst->outline.fixed
            || move.inst->tile.peer || !isPlacementAttractor(*move.inst)) {
            continue;
        }
        double dx = displacement_scale
            * move.external_force_x;
        double dy = displacement_scale
            * move.external_force_y;
        if (dx == 0 && dy == 0) continue;

        Displacement& anchor_displacement = displacement[move.inst];
        anchor_displacement.x += dx;
        anchor_displacement.y += dy;
        anchor_displacement.direct = true;

        struct Propagation
        {
            rtl::Inst* inst = nullptr;
            rtl::Inst* previous = nullptr;
            double dx = 0;
            double dy = 0;
            double influence = 0;
            int depth = 0;
        };
        std::deque<Propagation> pending;
        std::unordered_set<rtl::Inst*> visited;
        visited.insert(move.inst);
        for (rtl::Inst* peer : timingPeers(*move.inst, tech)) {
            if (!peer || peer->outline.fixed || peer->tile.peer) {
                continue;
            }
            pending.push_back(Propagation{
                .inst = peer,
                .previous = move.inst,
                .dx = dx*propagation_decay,
                .dy = dy*propagation_decay,
                .influence = propagation_decay,
                .depth = 1,
            });
        }

        while (!pending.empty()) {
            Propagation current = pending.front();
            pending.pop_front();
            if (!current.inst || visited.contains(current.inst)
                || current.depth > maximum_propagation_depth
                || std::max(std::abs(current.dx), std::abs(current.dy))
                    < minimum_propagated_displacement) {
                continue;
            }
            visited.insert(current.inst);
            // Every affected register is already represented by one direct,
            // personally calculated move.  Do not drag it as a follower of a
            // neighboring register as well.
            if (isPlacementAttractor(*current.inst)) {
                continue;
            }
            double& influence = follower_influence[current.inst];
            influence = std::max(influence, current.influence);
            for (rtl::Inst* peer : timingPeers(*current.inst, tech)) {
                if (!peer || peer == current.previous
                    || visited.contains(peer) || peer->outline.fixed
                    || peer->tile.peer) {
                    continue;
                }
                pending.push_back(Propagation{
                    .inst = peer,
                    .previous = current.inst,
                    .dx = current.dx*propagation_decay,
                    .dy = current.dy*propagation_decay,
                    .influence = current.influence*propagation_decay,
                    .depth = current.depth + 1,
                });
            }
        }
    }

    // Limit every saved velocity before deriving group motion.  The force is
    // still preserved in placement_motion; this is only the physical cooling
    // step applied during this pass.
    for (auto& [inst, delta] : displacement) {
        (void)inst;
        double displacement_length = std::hypot(delta.x, delta.y);
        if (displacement_length > maximum_cell_displacement) {
            delta.x *= maximum_cell_displacement/displacement_length;
            delta.y *= maximum_cell_displacement/displacement_length;
        }
    }

    // Combinational cells have no gravity source of their own. They follow
    // the local shape formed by their connected stars: the frozen neighbor
    // positions provide tension, predicted register movement carries the
    // constellation, and graph depth fades both effects. Copying a register's
    // direction verbatim is wrong when that register is moving toward the LUT
    // itself; it makes both endpoints move in the same direction.
    for (const auto& [inst, influence] : follower_influence) {
        if (!inst || influence <= 0 || inst->outline.fixed
            || inst->tile.peer || isPlacementAttractor(*inst)) {
            continue;
        }
        double origin_x = inst->outline.x*aspect_x;
        double origin_y = inst->outline.y*aspect_y;
        double position_force_x = 0;
        double position_force_y = 0;
        double predicted_neighbor_x = 0;
        double predicted_neighbor_y = 0;
        size_t peer_count = 0;
        for (rtl::Inst* peer : timingPeers(*inst, tech)) {
            if (!peer) continue;
            double peer_x = peer->tile.peer
                ? static_cast<double>(peer->coord.x)
                : peer->outline.x*aspect_x;
            double peer_y = peer->tile.peer
                ? static_cast<double>(peer->coord.y)
                : peer->outline.y*aspect_y;
            double weight = place_timing.placementNetWeight(*inst, *peer);
            position_force_x += weight*(peer_x - origin_x);
            position_force_y += weight*(peer_y - origin_y);
            auto peer_displacement = displacement.find(peer);
            if (peer_displacement != displacement.end()
                && peer_displacement->second.direct) {
                predicted_neighbor_x += peer_displacement->second.x;
                predicted_neighbor_y += peer_displacement->second.y;
            }
            ++peer_count;
        }
        if (peer_count == 0) continue;
        Displacement follower{
            .x = influence*(
                displacement_scale*position_force_x/peer_count
                + predicted_neighbor_x/peer_count),
            .y = influence*(
                displacement_scale*position_force_y/peer_count
                + predicted_neighbor_y/peer_count),
            .direct = false,
        };
        double follower_length = std::hypot(follower.x, follower.y);
        if (follower_length > maximum_cell_displacement) {
            follower.x *= maximum_cell_displacement/follower_length;
            follower.y *= maximum_cell_displacement/follower_length;
        }
        if (inst == movement_marker_a || inst == movement_marker_b) {
            std::print(
                "\nPLACE_MARKER_FOLLOW marker={} inst='{}' influence={:.4f} peers={} position_force=({:.4f},{:.4f}) predicted_neighbor=({:.4f},{:.4f}) displacement=({:.4f},{:.4f})",
                inst == movement_marker_a ? "A" : "B",
                inst->makeName(FULL_NAME_LIMIT), influence, peer_count,
                position_force_x/peer_count, position_force_y/peer_count,
                predicted_neighbor_x/peer_count,
                predicted_neighbor_y/peer_count,
                follower.x, follower.y);
        }
        if (follower.x != 0 || follower.y != 0) {
            displacement[inst] = follower;
        }
    }

    struct BunchTranslation
    {
        double x = 0;
        double y = 0;
        size_t clipped_attractors = 0;
    };
    std::unordered_map<RegBunch*, BunchTranslation> bunch_translations;

    auto bunchBounds = [&](const RegBunch& bunch) {
        std::array<double, 4> bounds{
            std::max(0.0, (bunch.x - 0.5)*aspect_x),
            std::min(static_cast<double>(fpga_width) - 0.001,
                     (bunch.x + 0.5)*aspect_x),
            std::max(0.0, (bunch.y - 0.5)*aspect_y),
            std::min(static_cast<double>(fpga_height) - 0.001,
                     (bunch.y + 0.5)*aspect_y),
        };
        if (bounds[0] > bounds[1]) bounds[0] = bounds[1];
        if (bounds[2] > bounds[3]) bounds[2] = bounds[3];
        return bounds;
    };

    // A bunch window is a locality envelope, not a fixed cage.  If a
    // personally calculated register move hits that envelope, save the
    // rejected residual as a proposed translation of the whole bunch.  All
    // proposals are calculated from the frozen snapshot before any bunch or
    // cell is changed.
    for (const auto& [inst, delta] : displacement) {
        if (!inst || !delta.direct || !inst->bunch_ref.peer
            || inst->bunch_ref->fixed) {
            continue;
        }
        RegBunch& bunch = *inst->bunch_ref;
        auto bounds = bunchBounds(bunch);
        double before_x = inst->outline.x*aspect_x;
        double before_y = inst->outline.y*aspect_y;
        double accepted_x = std::clamp(
            before_x + delta.x, bounds[0], bounds[1]);
        double accepted_y = std::clamp(
            before_y + delta.y, bounds[2], bounds[3]);
        double residual_x = before_x + delta.x - accepted_x;
        double residual_y = before_y + delta.y - accepted_y;
        if (std::abs(residual_x) < 0.000001
            && std::abs(residual_y) < 0.000001) {
            continue;
        }
        BunchTranslation& translation = bunch_translations[&bunch];
        translation.x += residual_x;
        translation.y += residual_y;
        ++translation.clipped_attractors;
    }

    for (auto& [bunch, translation] : bunch_translations) {
        if (!bunch || translation.clipped_attractors == 0) continue;
        translation.x /= translation.clipped_attractors;
        translation.y /= translation.clipped_attractors;
        double before_x = bunch->x*aspect_x;
        double before_y = bunch->y*aspect_y;
        double after_x = std::clamp(
            before_x + translation.x, 0.0,
            static_cast<double>(fpga_width) - 0.001);
        double after_y = std::clamp(
            before_y + translation.y, 0.0,
            static_cast<double>(fpga_height) - 0.001);
        translation.x = after_x - before_x;
        translation.y = after_y - before_y;
        bunch->x = static_cast<float>(
            after_x/std::max(aspect_x, 0.0001F));
        bunch->y = static_cast<float>(
            after_y/std::max(aspect_y, 0.0001F));
    }

    // Members without their own saved motion inherit the simultaneously
    // calculated bunch translation. Members with a personal/propagated move
    // retain it; translating the window lets that absolute move be accepted
    // instead of adding the group shift twice.
    if (drawing_cells) {
        for (rtl::Inst* inst : *drawing_cells) {
            if (!inst || inst->outline.fixed || inst->tile.peer
                || !inst->bunch_ref.peer || displacement.contains(inst)) {
                continue;
            }
            auto translation = bunch_translations.find(inst->bunch_ref.peer);
            if (translation == bunch_translations.end()) continue;
            if (translation->second.x == 0 && translation->second.y == 0) {
                continue;
            }
            displacement.emplace(inst, Displacement{
                .x = translation->second.x,
                .y = translation->second.y,
                .direct = false,
            });
        }
    }

    // Every displacement above was calculated from the same frozen Outline
    // snapshot. Publish only after all register forces and propagated
    // constellation motion have been accumulated.
    size_t moved = 0;
    for (auto& [inst, delta] : displacement) {
        if (!inst || (delta.x == 0 && delta.y == 0)) continue;
        double before_x = inst->outline.x*aspect_x;
        double before_y = inst->outline.y*aspect_y;
        double minimum_x = 0;
        double maximum_x = std::max(
            0.0, static_cast<double>(fpga_width) - 0.001);
        double minimum_y = 0;
        double maximum_y = std::max(
            0.0, static_cast<double>(fpga_height) - 0.001);
        if (inst->bunch_ref.peer && !inst->bunch_ref->fixed) {
            // The simultaneously shifted bunch window keeps members local
            // while allowing timing force to translate the whole group.
            minimum_x = std::max(
                minimum_x, (inst->bunch_ref->x - 0.5)*aspect_x);
            maximum_x = std::min(
                maximum_x, (inst->bunch_ref->x + 0.5)*aspect_x);
            minimum_y = std::max(
                minimum_y, (inst->bunch_ref->y - 0.5)*aspect_y);
            maximum_y = std::min(
                maximum_y, (inst->bunch_ref->y + 0.5)*aspect_y);
            if (minimum_x > maximum_x) minimum_x = maximum_x;
            if (minimum_y > maximum_y) minimum_y = maximum_y;
        }
        double after_x = std::clamp(
            before_x + delta.x, minimum_x, maximum_x);
        double after_y = std::clamp(
            before_y + delta.y, minimum_y, maximum_y);
        double applied_x = after_x - before_x;
        double applied_y = after_y - before_y;
        if (inst == movement_marker_a || inst == movement_marker_b) {
            std::print(
                "\nPLACE_MARKER_APPLY marker={} inst='{}' direct={} scale={:.5f} raw_delta=({:.4f},{:.4f}) before=({:.4f},{:.4f}) bounds=({:.4f}..{:.4f},{:.4f}..{:.4f}) after=({:.4f},{:.4f}) applied=({:.4f},{:.4f})",
                inst == movement_marker_a ? "A" : "B",
                inst->makeName(FULL_NAME_LIMIT), delta.direct,
                displacement_scale, delta.x, delta.y, before_x, before_y,
                minimum_x, maximum_x, minimum_y, maximum_y,
                after_x, after_y, applied_x, applied_y);
        }
        if (applied_x == 0 && applied_y == 0) continue;

        inst->outline.x = static_cast<float>(
            after_x/std::max(aspect_x, 0.0001F));
        inst->outline.y = static_cast<float>(
            after_y/std::max(aspect_y, 0.0001F));
        rtl::PlacementMotion& motion = inst->placement_motion;
        if (!delta.direct) {
            motion.force_x = applied_x/displacement_scale;
            motion.force_y = applied_y/displacement_scale;
            motion.acceleration = std::hypot(
                motion.force_x, motion.force_y);
            motion.direction = {
                (applied_x > 0) - (applied_x < 0),
                (applied_y > 0) - (applied_y < 0),
            };
            motion.from = {
                std::clamp(static_cast<int>(before_x), 0, fpga_width - 1),
                std::clamp(static_cast<int>(before_y), 0, fpga_height - 1),
            };
        }
        motion.displacement_x = applied_x;
        motion.displacement_y = applied_y;
        motion.to = {
            std::clamp(static_cast<int>(after_x), 0, fpga_width - 1),
            std::clamp(static_cast<int>(after_y), 0, fpga_height - 1),
        };
        motion.active = true;
        ++moved;
    }
    auto traceMissingMarker = [&](const char* label, rtl::Inst* inst) {
        if (inst && !displacement.contains(inst)) {
            std::print(
                "\nPLACE_MARKER_APPLY marker={} inst='{}' no_displacement_entry=1",
                label, inst->makeName(FULL_NAME_LIMIT));
        }
    };
    traceMissingMarker("A", movement_marker_a);
    traceMissingMarker("B", movement_marker_b);
    return moved;
}

void PlaceDesign::smearOversubscribedCells(
    const std::vector<rtl::Inst*>& cells)
{
    constexpr double fastest_register_step = 2.0;
    int maximum_passes = std::max(1, static_cast<int>(std::ceil(
        0.5*fpga_width*SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR)));
    size_t initial_groups = 0;
    size_t remaining_groups = 0;
    size_t total_attractors = 0;
    size_t total_moved = 0;
    size_t total_followers = 0;
    double minimum_scale = std::numeric_limits<double>::infinity();
    double maximum_scale = 0;
    double maximum_displacement = 0;
    int completed_passes = 0;

    if (!movement_png_prefix.empty() && movement_snapshot_cells.empty()) {
        movement_snapshot_cells = cells;
    }

    auto writeFrame = [&](int frame) {
        if (movement_png_frames < 2 || movement_png_prefix.empty()) return;
        double progress = static_cast<double>(frame)
            / static_cast<double>(movement_png_frames - 1);
        char stage_and_label[64];
        const char* label = frame == 0 ? "outline"
            : (frame + 1 == movement_png_frames ? "shaped" : "movement");
        std::snprintf(stage_and_label, sizeof(stage_and_label),
                      "%02d_%s_%03d",
                      frame, label,
                      static_cast<int>(std::round(progress*100)));
        std::string filename = movementPngFilename(stage_and_label);
        if (frame == 0) {
            drawPlacementSnapshot(cells, filename);
            captureMovementSnapshot(cells, filename);
            return;
        }
        std::unordered_map<rtl::Inst*, std::pair<double, double>> active;
        for (rtl::Inst* inst : cells) {
            if (inst && inst->placement_motion.active) {
                active.emplace(inst, std::pair{0.0, 0.0});
            }
        }
        drawPlacementSnapshot(cells, filename, &active, progress);
        captureMovementSnapshot(cells, filename, &active, progress);
    };

    int next_frame = 0;
    if (movement_png_frames >= 2 && !movement_png_prefix.empty()) {
        writeFrame(next_frame++);
    }

    for (int pass = 0; pass < maximum_passes; ++pass) {
        std::unordered_map<rtl::Inst*, Coord> predicted;
        predicted.reserve(cells.size());
        for (rtl::Inst* inst : cells) {
            if (!inst || !fpga::elementTypeForInst(*inst)) continue;
            predicted[inst] = inst->tile.peer ? inst->coord : Coord{
                std::clamp(static_cast<int>(inst->outline.x*aspect_x),
                           0, fpga_width - 1),
                std::clamp(static_cast<int>(inst->outline.y*aspect_y),
                           0, fpga_height - 1),
            };
        }

        size_t oversubscribed_groups = 0;
        std::vector<PlacePredictedMove> moves = calculatePredictedDirections(
            cells, predicted, oversubscribed_groups);
        if (pass == 0) initial_groups = oversubscribed_groups;
        remaining_groups = oversubscribed_groups;
        if (oversubscribed_groups == 0 || moves.empty()) {
            break;
        }

        size_t active_attractors = std::ranges::count_if(
            moves, [](const PlacePredictedMove& move) {
                return move.inst && move.inst->placement_motion.active;
            });
        double maximum_acceleration = 0;
        for (const PlacePredictedMove& move : moves) {
            maximum_acceleration = std::max(maximum_acceleration,
                std::hypot(move.external_force_x, move.external_force_y));
        }
        if (maximum_acceleration <= 0) {
            break;
        }
        double scale = std::min(
            static_cast<double>(SCALEPNR_PLACE_REDISTRIBUTION_STEP),
            fastest_register_step/maximum_acceleration);
        size_t moved = applyPredictedDisplacementSimultaneously(
            moves, &cells, scale);
        ++completed_passes;
        total_attractors += active_attractors;
        total_moved += moved;
        minimum_scale = std::min(minimum_scale, scale);
        maximum_scale = std::max(maximum_scale, scale);

        size_t moved_followers = 0;
        double pass_maximum_displacement = 0;
        for (rtl::Inst* inst : cells) {
            if (!inst || !inst->placement_motion.active) continue;
            if (!isPlacementAttractor(*inst)) ++moved_followers;
            pass_maximum_displacement = std::max(
                pass_maximum_displacement,
                std::hypot(inst->placement_motion.displacement_x,
                           inst->placement_motion.displacement_y));
        }
        total_followers += moved_followers;
        maximum_displacement = std::max(
            maximum_displacement, pass_maximum_displacement);
        std::print(
            "\nPLACE_SMEAR_PASS pass={}/{} groups={} attractors={} moved={} followers={} max_acceleration={:.3f} calibrated_scale={:.4f} maximum_displacement={:.3f}",
            completed_passes, maximum_passes, oversubscribed_groups,
            active_attractors, moved, moved_followers,
            maximum_acceleration, scale, pass_maximum_displacement);

        while (next_frame < movement_png_frames
            && static_cast<double>(completed_passes)/maximum_passes
                + 0.000001
                >= static_cast<double>(next_frame)
                    /(movement_png_frames - 1)) {
            writeFrame(next_frame++);
        }
        if (moved == 0) break;
    }

    while (next_frame < movement_png_frames) {
        writeFrame(next_frame++);
    }
    if (!std::isfinite(minimum_scale)) minimum_scale = 0;
    std::print(
        "\nPLACE_SMEAR_SUMMARY initial_groups={} remaining_groups={} attractor_moves={} moved={} follower_moves={} scale_limit={:.3f} calibrated_scale_min={:.4f} calibrated_scale_max={:.4f} fastest_register_step={:.1f} maximum_displacement={:.3f} passes={}/{} pass_factor={:.2f}",
        initial_groups, remaining_groups, total_attractors, total_moved,
        total_followers,
        static_cast<double>(SCALEPNR_PLACE_REDISTRIBUTION_STEP),
        minimum_scale, maximum_scale, fastest_register_step,
        maximum_displacement, completed_passes, maximum_passes,
        static_cast<double>(SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR));
}

int PlaceDesign::tryAddTimingAware(rtl::Inst& inst, fpga::ElementType type,
                                   const Coord& requested_origin)
{
    struct Candidate
    {
        fpga::Tile* tile = nullptr;
        double timing_cost = 0;
        int occupied = 0;
    };

    Coord origin{
        std::clamp(requested_origin.x, 0, fpga_width - 1),
        std::clamp(requested_origin.y, 0, fpga_height - 1),
    };
    auto appendCandidate = [&](std::vector<Candidate>& candidates,
                               Coord coordinate) {
        if (coordinate.x < 0 || coordinate.x >= fpga_width
            || coordinate.y < 0 || coordinate.y >= fpga_height) {
            return;
        }
        size_t index = static_cast<size_t>(
            coordinate.y*fpga_width + coordinate.x);
        if (index >= tile_grid->size()) return;
        fpga::Tile& tile = (*tile_grid)[index];
        if (!tile.tile_type || tile.coord.x != coordinate.x
            || tile.coord.y != coordinate.y || !tile.hasFreeElement(type)) {
            return;
        }
        int occupied = 0;
        for (int element = 0; element < fpga::ELEMENT_TYPE_COUNT; ++element) {
            unsigned occupied_mask = static_cast<unsigned>(
                tile.elements_pos[element]
                & static_cast<uint16_t>(~tile.elements_free[element]));
            occupied += std::popcount(occupied_mask);
        }
        candidates.push_back(Candidate{
            .tile = &tile,
            .timing_cost = timingPlacementCost(inst, coordinate),
            .occupied = occupied,
        });
    };

    for (int radius = 0; radius < fpga_width + fpga_height; ++radius) {
        std::vector<Candidate> candidates;
        candidates.reserve(static_cast<size_t>(std::max(1, 4*radius)));
        for (int dx = -radius; dx <= radius; ++dx) {
            int dy = radius - std::abs(dx);
            appendCandidate(candidates, {origin.x + dx, origin.y + dy});
            if (dy != 0) {
                appendCandidate(candidates, {origin.x + dx, origin.y - dy});
            }
        }
        auto candidateLess = [](const Candidate& left,
                                const Candidate& right) {
            if (left.timing_cost != right.timing_cost) {
                return left.timing_cost < right.timing_cost;
            }
            if (left.occupied != right.occupied) {
                return left.occupied < right.occupied;
            }
            if (left.tile->coord.y != right.tile->coord.y) {
                return left.tile->coord.y < right.tile->coord.y;
            }
            return left.tile->coord.x < right.tile->coord.x;
        };
        std::ranges::sort(candidates, candidateLess);
        std::vector<int> placement_results(candidates.size(), -2);
        for (size_t candidate_index = 0;
             candidate_index < candidates.size(); ++candidate_index) {
            const Candidate& candidate = candidates[candidate_index];
            ++place_tile_trials;
            int placed_pos = candidate.tile->tryAdd(&inst, false);
            placement_results[candidate_index] = placed_pos;
            if (placed_pos >= 0) {
                if (record_timing_history && radius > 0) {
                    InitialPlacementTrace trace{
                        .inst = &inst,
                        .origin = origin,
                        .selected = candidate.tile->coord,
                        .radius = radius,
                    };
                    for (rtl::Inst* peer : timingPeers(inst, tech)) {
                        if (!peer || peer == &inst) continue;
                        bool already_placed = peer->tile.peer != nullptr;
                        Coord target = already_placed ? peer->coord : Coord{
                            std::clamp(static_cast<int>(
                                peer->outline.x*aspect_x), 0, fpga_width - 1),
                            std::clamp(static_cast<int>(
                                peer->outline.y*aspect_y), 0, fpga_height - 1),
                        };
                        trace.peers.push_back(InitialPlacementPeerTrace{
                            .peer = peer,
                            .target = target,
                            .timing_weight = place_timing.placementNetWeight(
                                inst, *peer),
                            .already_placed = already_placed,
                        });
                    }
                    constexpr size_t recorded_candidate_limit = 16;
                    size_t recorded = std::min(
                        recorded_candidate_limit, candidates.size());
                    trace.candidates.reserve(recorded + 1);
                    for (size_t index = 0; index < recorded; ++index) {
                        trace.candidates.push_back(
                            InitialPlacementCandidateTrace{
                                .coord = candidates[index].tile->coord,
                                .radius = radius,
                                .timing_cost = candidates[index].timing_cost,
                                .occupied = candidates[index].occupied,
                                .placement_result = placement_results[index],
                            });
                    }
                    if (candidate_index >= recorded) {
                        trace.candidates.push_back(
                            InitialPlacementCandidateTrace{
                                .coord = candidate.tile->coord,
                                .radius = radius,
                                .timing_cost = candidate.timing_cost,
                                .occupied = candidate.occupied,
                                .placement_result = placed_pos,
                            });
                    }
                    constexpr int future_ring_count = 3;
                    constexpr size_t future_candidates_per_ring = 4;
                    for (int future_radius = radius + 1;
                         future_radius <= radius + future_ring_count
                            && future_radius < fpga_width + fpga_height;
                         ++future_radius) {
                        std::vector<Candidate> future_candidates;
                        for (int dx = -future_radius; dx <= future_radius;
                             ++dx) {
                            int dy = future_radius - std::abs(dx);
                            appendCandidate(future_candidates,
                                {origin.x + dx, origin.y + dy});
                            if (dy != 0) {
                                appendCandidate(future_candidates,
                                    {origin.x + dx, origin.y - dy});
                            }
                        }
                        std::ranges::sort(future_candidates, candidateLess);
                        size_t future_recorded = std::min(
                            future_candidates_per_ring,
                            future_candidates.size());
                        for (size_t index = 0; index < future_recorded;
                             ++index) {
                            trace.candidates.push_back(
                                InitialPlacementCandidateTrace{
                                    .coord = future_candidates[index].tile->coord,
                                    .radius = future_radius,
                                    .timing_cost =
                                        future_candidates[index].timing_cost,
                                    .occupied =
                                        future_candidates[index].occupied,
                                    .placement_result = -3,
                                });
                        }
                    }
                    initial_placement_history.push_back(std::move(trace));
                }
                return placed_pos;
            }
        }
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
                || std::abs(tile->coord.x - origin.x)
                    + std::abs(tile->coord.y - origin.y)
                    > max_shared_tile_distance) {
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
                    placed_pos = tryAddTimingAware(
                        inst, *element_type, search_origin);
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
    placement_before_timing.clear();
    timing_move_history.clear();
    initial_placement_history.clear();
    bunch_reservations.clear();
    bunch_reservation_order.clear();
    timing_refinement_runs = 0;
    place_started = std::chrono::steady_clock::now();
    place_next_report = place_started + std::chrono::minutes(1);
    place_timing.tech = tech;
    if (tech) {
        place_timing.preparePlacementGuide(tech->timings);
    }

    std::vector<rtl::Inst*> all_insts;
    collectInsts(tech->design.top, all_insts);
    if (write_debug_images && !movement_png_prefix.empty()) {
        movement_snapshot_cells = all_insts;
        movement_snapshots.clear();
        std::string filename = movementPngFilename("00_outline");
        drawPlacementSnapshot(all_insts, filename);
        captureMovementSnapshot(all_insts, filename);
    }
    preSmearBunches(all_insts);
    if (write_debug_images && !movement_png_prefix.empty()) {
        std::string filename = movementPngFilename(
            "01_pre_smear_reserved");
        drawPlacementSnapshot(all_insts, filename);
        captureMovementSnapshot(all_insts, filename);
    }
    commitPreSmearReservations();
    if (write_debug_images && !movement_png_prefix.empty()) {
        std::string filename = movementPngFilename("02_exact_reserved");
        drawPlacementSnapshot(all_insts, filename);
        captureMovementSnapshot(all_insts, filename);
    }
    std::vector<rtl::Inst*> unreserved_insts;
    unreserved_insts.reserve(all_insts.size());
    for (rtl::Inst* inst : all_insts) {
        if (inst && !inst->tile.peer) {
            unreserved_insts.push_back(inst);
        }
    }
    smearOversubscribedCells(unreserved_insts);

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recursivePackBunch(*bunch.reg, &bunch);
    }

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

    if (record_timing_history) {
        placement_before_timing.reserve(all_insts.size());
        for (rtl::Inst* inst : all_insts) {
            if (inst && inst->tile.peer) {
                placement_before_timing.push_back(PlaceTimingPlacementSnapshot{
                    .inst = inst,
                    .coord = inst->coord,
                    .pos = inst->pos,
                });
            }
        }
    }

    timing_refinement = {};
    if (tech && !tech->timings.clocked_inputs.empty()) {
        timing_refinement = refineTiming(tech->timings);
    }

    if (write_debug_images && !movement_png_prefix.empty()) {
        std::string filename = movementPngFilename(
            "03_place_design_timed");
        drawPlacementSnapshot(all_insts, filename);
        captureMovementSnapshot(all_insts, filename);
    }

    if (write_debug_images) {
        travers_mark = rtl::Inst::genMark();
        image.init(
            mesh_width*aspect_x*image_zoom,
            mesh_height*aspect_y*image_zoom);
        image.clear();
        for (auto& bunch : bunch_list) {
            recurseDrawDesign(*bunch.reg, &bunch);
        }
        image.write("place_output.png");
    }
}

PlaceTimingRefinement PlaceDesign::refineTiming(
    clk::Timings& timings, size_t max_passes,
    size_t max_anchor_cells_per_pass)
{
    auto started = std::chrono::steady_clock::now();
    size_t refinement_run = ++timing_refinement_runs;
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
        for (auto it = moved.rbegin(); it != moved.rend(); ++it) {
            TimingPlacementSnapshot& snapshot = *it;
            if (!snapshot.inst || !snapshot.tile) {
                continue;
            }
            if (snapshot.inst->tile.peer) {
                snapshot.inst->tile->unassign(snapshot.inst);
            }
            int restored = snapshot.tile->tryAddAt(snapshot.inst, snapshot.pos);
            PNR_ASSERT(restored == snapshot.pos,
                "failed to restore timing-moved inst '{}' to ({},{}) pos {}",
                snapshot.inst->makeName(FULL_NAME_LIMIT), snapshot.coord.x,
                snapshot.coord.y, snapshot.pos);
        }
    };

    auto move_one = [&](rtl::Inst& inst, Coord direction,
                        TimingPlacementSnapshot& snapshot,
                        PlaceTimingMoveBlockReason& block_reason) {
        ++refinement.attempted_cells;
        block_reason = PlaceTimingMoveBlockReason::none;
        if (!inst.tile.peer || inst.outline.fixed) {
            block_reason = PlaceTimingMoveBlockReason::immovable;
            return false;
        }
        if (direction.x == 0 && direction.y == 0) {
            block_reason = PlaceTimingMoveBlockReason::zero_direction;
            return false;
        }
        Coord target = inst.coord + direction;
        if (target.x < 0 || target.x >= fpga_width
            || target.y < 0 || target.y >= fpga_height) {
            block_reason = PlaceTimingMoveBlockReason::boundary;
            return false;
        }
        fpga::Tile& target_tile = (*tile_grid)[target.y*fpga_width + target.x];
        if (target_tile.coord.x < 0 || target_tile.coord.y < 0
            || !target_tile.tile_type) {
            block_reason = PlaceTimingMoveBlockReason::invalid_tile;
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
            block_reason = PlaceTimingMoveBlockReason::no_capacity;
            int restored = snapshot.tile->tryAddAt(&inst, snapshot.pos);
            PNR_ASSERT(restored == snapshot.pos,
                "failed to restore rejected timing move for '{}'",
                inst.makeName(FULL_NAME_LIMIT));
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
        size_t pass_trace_begin = timing_move_history.size();
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
            size_t move_trace_begin = timing_move_history.size();
            bool anchor_moved = false;
            for (rtl::Inst* member : constellation) {
                if (!member || moved_this_pass.contains(member)) {
                    continue;
                }
                TimingPlacementSnapshot snapshot;
                Coord before = member->coord;
                PlaceTimingMoveBlockReason block_reason;
                if (move_one(*member, direction, snapshot, block_reason)) {
                    anchor_moved |= member == anchor;
                    moved_this_pass.insert(member);
                    pass_moves.push_back(snapshot);
                    if (record_timing_history) {
                        timing_move_history.push_back(PlaceTimingMoveTrace{
                            .inst = member,
                            .anchor = anchor,
                            .strongest_peer = force.strongest_peer,
                            .from = before,
                            .to = member->coord,
                            .direction = direction,
                            .run = refinement_run,
                            .pass = pass + 1,
                            .force_x = force.x,
                            .force_y = force.y,
                            .force_weight = force.weight,
                            .is_anchor = member == anchor,
                        });
                    }
                }
                else if (member == anchor && record_timing_history) {
                    timing_move_history.push_back(PlaceTimingMoveTrace{
                        .inst = member,
                        .anchor = anchor,
                        .strongest_peer = force.strongest_peer,
                        .from = before,
                        .to = before,
                        .direction = direction,
                        .run = refinement_run,
                        .pass = pass + 1,
                        .force_x = force.x,
                        .force_y = force.y,
                        .force_weight = force.weight,
                        .is_anchor = true,
                        .outcome = PlaceTimingMoveOutcome::anchor_blocked,
                        .block_reason = block_reason,
                    });
                }
            }
            if (!anchor_moved) {
                std::vector<TimingPlacementSnapshot> rejected(
                    pass_moves.begin() + static_cast<std::ptrdiff_t>(move_begin),
                    pass_moves.end());
                restore(rejected);
                for (const TimingPlacementSnapshot& snapshot : rejected) {
                    moved_this_pass.erase(snapshot.inst);
                }
                for (size_t trace = move_trace_begin;
                     trace < timing_move_history.size(); ++trace) {
                    if (timing_move_history[trace].outcome
                        == PlaceTimingMoveOutcome::pending) {
                        timing_move_history[trace].outcome =
                            PlaceTimingMoveOutcome::anchor_reverted;
                    }
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
            for (size_t trace = pass_trace_begin;
                 trace < timing_move_history.size(); ++trace) {
                if (timing_move_history[trace].outcome
                    == PlaceTimingMoveOutcome::pending) {
                    timing_move_history[trace].outcome =
                        PlaceTimingMoveOutcome::objective_reverted;
                }
            }
            if (anchor_limit > 1) {
                anchor_limit = std::max<size_t>(1, anchor_limit/2);
                continue;
            }
            break;
        }

        refinement.moved_cells += pass_moves.size();
        ++refinement.passes;
        for (size_t trace = pass_trace_begin;
             trace < timing_move_history.size(); ++trace) {
            if (timing_move_history[trace].outcome
                == PlaceTimingMoveOutcome::pending) {
                timing_move_history[trace].outcome =
                    PlaceTimingMoveOutcome::accepted;
            }
        }
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
