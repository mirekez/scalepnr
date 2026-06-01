#include "RouteDesign.h"
#include "Device.h"
#include "Tech.h"
#include "Wire.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <format>
#include <iterator>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace pnr;

void RouteDesign::RouteStats::clear()
{
    *this = RouteStats{};
}

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

size_t statDepthBucket(int depth)
{
    if (depth < 0) {
        return 0;
    }
    size_t bucket = static_cast<size_t>(depth);
    return std::min(bucket, RouteDesign::RouteStats::max_depth - 1);
}

std::string statArray(const std::array<size_t, RouteDesign::RouteStats::max_depth>& values)
{
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        out << values[i];
    }
    out << ']';
    return out.str();
}

std::string maskString(u256 value)
{
    return value.str();
}

size_t countSetBits(u256 value)
{
    size_t count = 0;
    value.for_each_set_bit([&](int) {
        ++count;
        return false;
    });
    return count;
}

size_t countDeadendBits(const std::unordered_map<uint64_t, u256>& deadends)
{
    size_t count = 0;
    for (const auto& entry : deadends) {
        count += countSetBits(entry.second);
    }
    return count;
}

std::string csvField(const std::string& value)
{
    bool quote = value.find_first_of(",\"\n\r") != std::string::npos;
    if (!quote) {
        return value;
    }
    std::string out = "\"";
    for (char ch : value) {
        if (ch == '"') {
            out += "\"\"";
        }
        else {
            out += ch;
        }
    }
    out += '"';
    return out;
}

double elapsedSeconds(std::chrono::steady_clock::time_point start,
                      std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<double>(end - start).count();
}

int routeDebugPass()
{
    const char* value = std::getenv("SCALEPNR_ROUTE_DEBUG_PASS");
    if (!value || !*value) {
        return 0;
    }
    char* end = nullptr;
    long pass = std::strtol(value, &end, 10);
    if (end == value || pass <= 0 || pass > std::numeric_limits<int>::max()) {
        return 0;
    }
    return static_cast<int>(pass);
}

bool stopAfterRouteDebugPass()
{
    const char* value = std::getenv("SCALEPNR_ROUTE_STOP_AFTER_DEBUG");
    return value && *value && std::string(value) != "0";
}

bool sameCoord(const Coord& a, const Coord& b)
{
    return a.x == b.x && a.y == b.y;
}

rtl::Inst* instFromTileRef(RefBase<Referable<Tile>>* ref)
{
    auto* tile_ref = Ref<Tile>::fromBase(ref);
    return reinterpret_cast<rtl::Inst*>(reinterpret_cast<char*>(tile_ref) - offsetof(rtl::Inst, tile));
}

rtl::Inst* tileInstAtPos(Tile& tile, int pos)
{
    auto& referable_tile = static_cast<Referable<Tile>&>(tile);
    for (auto* peer : referable_tile.getPeers()) {
        if (!peer) {
            continue;
        }

        rtl::Inst* candidate = instFromTileRef(peer);
        if (!candidate || candidate->pos != pos || candidate->tile.peer != &referable_tile) {
            continue;
        }

        return candidate;
    }
    return nullptr;
}

bool hasLocalNodeUse(const fpga::CBType& cb_type, int local)
{
    if (local < 0 || local >= CB_MAX_NODES) {
        return false;
    }
    auto not_empty = [](u256 value) {
        return value != u256{};
    };
    return cb_type.nodeName(fpga::CB_NODE_LOCAL, local)
        || not_empty(cb_type.local_src[local].jump)
        || not_empty(cb_type.local_joint[local].joint)
        || not_empty(cb_type.local_local[local].local)
        || not_empty(cb_type.dst_local[local].local);
}

bool isConcreteRouteTile(const Tile& tile)
{
    return tile.cb_type && (!tile.tile_type || tile.tile_type->name == tile.cb_type->name);
}

bool supportsLocalNodes(const Tile& tile, u256 nodes)
{
    if (!isConcreteRouteTile(tile)) {
        return false;
    }
    if (nodes == u256{}) {
        return true;
    }
    return nodes.for_each_set_bit([&](int local) {
        return hasLocalNodeUse(*tile.cb_type, local);
    });
}

bool isRoutableOutputLocal(const Tile& tile, int local)
{
    return tile.cb_type && tile.cb_type->srcNodes(fpga::CB_NODE_LOCAL, local) != nullptr;
}

bool supportsOutputLocalNodes(const Tile& tile, u256 nodes)
{
    if (!isConcreteRouteTile(tile)) {
        return false;
    }
    if (nodes == u256{}) {
        return true;
    }
    return nodes.for_each_set_bit([&](int local) {
        return isRoutableOutputLocal(tile, local);
    });
}

u256 routeTileInputNodes(const Tile& route_tile, rtl::Inst& inst, const std::string& port)
{
    if (!inst.tile.peer || !inst.cell_ref.peer) {
        return {};
    }
    u256 endpoint_nodes = inst.tile->getPinNodes(inst.cell_ref->type, port, inst.pos);
    if (endpoint_nodes != u256{} && supportsLocalNodes(route_tile, endpoint_nodes)) {
        return endpoint_nodes;
    }
    return route_tile.getPinNodes(inst.cell_ref->type, port, inst.pos);
}

bool isInputOnlyLocal(const Tile& tile, int local)
{
    if (!tile.cb_type || local < 0 || local >= CB_MAX_NODES) {
        return false;
    }
    u256 bit = u256{0,1} << local;
    return (tile.cb_type->local_input_nodes & bit) != u256{}
        && (tile.cb_type->local_output_nodes & bit) == u256{};
}

int routeDistance(const Coord& a, const Coord& b)
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

int actualJumpId(int curr, int orig_curr)
{
    int dir = curr / 32;
    int path = curr % 32;
    return fpga::search_dirs[orig_curr / 32][dir % 8] * 32 + path;
}

Coord makeActualJump(const Coord& src, int actual_curr)
{
    int dir = actual_curr / 32;
    int path = actual_curr % 32;
    int step = std::max(1, path / 4 / 2);
    switch (dir) {
        case 0: return src + Coord{0, -step};
        case 1: return src + Coord{step, -step};
        case 2: return src + Coord{step, 0};
        case 3: return src + Coord{step, step};
        case 4: return src + Coord{0, step};
        case 5: return src + Coord{-step, step};
        case 6: return src + Coord{-step, 0};
        case 7: return src + Coord{-step, -step};
    }
    return Coord{-1, -1};
}

bool hasConcreteCrossbarPath(const Tile& tile, fpga::CBNodeNameType from_type, int from_value,
                             fpga::CBNodeNameType to_type, int to_value, int joint)
{
    if (!tile.cb_type) {
        return false;
    }
    if (joint >= 0) {
        return tile.cb_type->connName(from_type, from_value, fpga::CB_NODE_JOINT, joint)
            && tile.cb_type->connName(fpga::CB_NODE_JOINT, joint, to_type, to_value);
    }
    return tile.cb_type->connName(from_type, from_value, to_type, to_value) != nullptr;
}

bool routeIsComplete(const std::vector<Wire>& route)
{
    return !route.empty() && route.back().type == Wire::WIRE_TILE_PIN;
}

size_t routeCrossbarFragments(const std::vector<Wire>* route)
{
    if (!route) {
        return 0;
    }
    return std::count_if(route->begin(), route->end(), [](const Wire& wire) {
        return wire.type == Wire::WIRE_CROSSBAR;
    });
}

std::vector<Wire>* findRoute(rtl::Inst& inst, const std::string& net_name)
{
    for (auto& route : inst.wires) {
        if (!route.empty() && route.front().net_name == net_name) {
            return &route;
        }
    }
    return nullptr;
}

const std::vector<Wire>* findRoute(const rtl::Inst& inst, const std::string& net_name)
{
    for (const auto& route : inst.wires) {
        if (!route.empty() && route.front().net_name == net_name) {
            return &route;
        }
    }
    return nullptr;
}

size_t findRouteIndex(const rtl::Inst& inst, const std::vector<Wire>* route)
{
    for (size_t i = 0; i < inst.wires.size(); ++i) {
        if (&inst.wires[i] == route) {
            return i;
        }
    }
    return std::numeric_limits<size_t>::max();
}

rtl::Net* findNetByDesignator(rtl::Inst& inst, int designator)
{
    if (!inst.cell_ref.peer || !inst.cell_ref->module_ref.peer) {
        return nullptr;
    }
    rtl::Module* parent = inst.cell_ref->module_ref->parent_ref.peer;
    if (!parent) {
        return nullptr;
    }
    for (auto& net : parent->nets) {
        for (int net_designator : net.designators) {
            if (net_designator == designator) {
                return &net;
            }
        }
    }
    return nullptr;
}

struct TransitVictim
{
    rtl::Net* net = nullptr;
    size_t binding_index = 0;
};

TransitVictim findTransitDstVictim(Tile& tile, int dst_node, rtl::Net* current_net, rtl::Inst* current_source, const std::string& current_source_port)
{
    for (auto& ref : tile.routedNets) {
        rtl::Net* net = ref.peer;
        if (!net || net == current_net) {
            continue;
        }
        for (size_t binding_index = 0; binding_index < net->routes.size(); ++binding_index) {
            rtl::NetRouteBinding& binding = net->routes[binding_index];
            if (current_source && binding.from == current_source && binding.from_port == current_source_port) {
                continue;
            }
            if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
                continue;
            }
            const std::vector<Wire>& route = binding.owner->wires[binding.route_index];
            for (const Wire& fragment : route) {
                if (fragment.type != Wire::WIRE_CROSSBAR || fragment.local != dst_node || fragment.pos == 0) {
                    continue;
                }
                if (sameCoord(fragment.from, tile.coord) && !sameCoord(fragment.to, tile.coord)) {
                    return TransitVictim{net, binding_index};
                }
            }
        }
    }
    return TransitVictim{};
}

bool hasRoutedNet(const rtl::Inst& inst, const std::string& net_name)
{
    for (const auto& route : inst.wires) {
        if (!route.empty() && route.front().net_name == net_name && routeIsComplete(route)) {
            return true;
        }
    }
    return false;
}

bool sameRouteTask(const RouteDesign::RouteTask& left, const RouteDesign::RouteTask& right)
{
    return left.net == right.net
        && left.from == right.from
        && left.to == right.to
        && left.from_port == right.from_port
        && left.to_port == right.to_port
        && left.net_name == right.net_name;
}

bool appendUniqueRouteTask(std::vector<RouteDesign::RouteTask>& queue, const RouteDesign::RouteTask& task)
{
    for (RouteDesign::RouteTask& old : queue) {
        if (!sameRouteTask(old, task)) {
            continue;
        }
        if (!task.fanout) {
            old.fanout = false;
        }
        return false;
    }
    queue.push_back(task);
    return true;
}

const fpga::CBConnName* selectConcreteConn(const fpga::CBType* type,
                                           fpga::CBNodeNameType from_type, int from_value,
                                           fpga::CBNodeNameType to_type, int to_value,
                                           const std::string& preferred_from = {},
                                           const std::string& preferred_to = {})
{
    if (!type) {
        return nullptr;
    }
    const std::vector<fpga::CBConnName>* conns = type->connNames(from_type, from_value, to_type, to_value);
    if (!conns || conns->empty()) {
        return nullptr;
    }
    auto matches = [&](const fpga::CBConnName& conn, bool match_from, bool match_to) {
        return (!match_from || conn.from == preferred_from)
            && (!match_to || conn.to == preferred_to);
    };
    if (!preferred_from.empty() && !preferred_to.empty()) {
        for (const fpga::CBConnName& conn : *conns) {
            if (matches(conn, true, true)) {
                return &conn;
            }
        }
        return nullptr;
    }
    if (!preferred_from.empty()) {
        for (const fpga::CBConnName& conn : *conns) {
            if (matches(conn, true, false)) {
                return &conn;
            }
        }
        return nullptr;
    }
    if (!preferred_to.empty()) {
        for (const fpga::CBConnName& conn : *conns) {
            if (matches(conn, false, true)) {
                return &conn;
            }
        }
        return nullptr;
    }
    return &conns->front();
}

constexpr int ROUTE_POS_SOURCE = 0;
constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;

bool leaseConcreteOut(CBState& cb, int local, int src)
{
    u256 src_bit = u256{0,1} << src;
    if ((cb.src_deadend.jump & src_bit) != u256{} || (cb.src.jump & src_bit) != u256{}) {
        return false;
    }
    (void)local;
    cb.src.jump |= src_bit;
    return true;
}

bool leaseConcreteJump(CBState& cb, int dst, int src)
{
    u256 dst_bit = u256{0,1} << dst;
    u256 src_bit = u256{0,1} << src;
    if ((cb.dst_deadend.jump & dst_bit) != u256{} || (cb.src_deadend.jump & src_bit) != u256{}
        || (cb.dst.jump & dst_bit) != u256{} || (cb.src.jump & src_bit) != u256{}) {
        return false;
    }
    cb.dst.jump |= dst_bit;
    cb.src.jump |= src_bit;
    return true;
}

bool leaseConcreteFork(CBState& cb, int dst, int src)
{
    u256 dst_bit = u256{0,1} << dst;
    u256 src_bit = u256{0,1} << src;
    if ((cb.dst_deadend.jump & dst_bit) != u256{} || (cb.src_deadend.jump & src_bit) != u256{}
        || (cb.src.jump & src_bit) != u256{}) {
        return false;
    }
    cb.dst.jump |= dst_bit;
    cb.src.jump |= src_bit;
    return true;
}

enum class ConcreteBusyReason {
    none,
    dst,
    src,
    local,
    dst_deadend,
    src_deadend,
};

ConcreteBusyReason concreteJumpBusyReason(const CBState& cb, int dst, int src)
{
    u256 dst_bit = u256{0,1} << dst;
    u256 src_bit = u256{0,1} << src;
    if ((cb.dst_deadend.jump & dst_bit) != u256{}) {
        return ConcreteBusyReason::dst_deadend;
    }
    if ((cb.src_deadend.jump & src_bit) != u256{}) {
        return ConcreteBusyReason::src_deadend;
    }
    if ((cb.dst.jump & dst_bit) != u256{}) {
        return ConcreteBusyReason::dst;
    }
    if ((cb.src.jump & src_bit) != u256{}) {
        return ConcreteBusyReason::src;
    }
    return ConcreteBusyReason::none;
}

ConcreteBusyReason concreteOutBusyReason(const CBState& cb, int src)
{
    u256 src_bit = u256{0,1} << src;
    if ((cb.src_deadend.jump & src_bit) != u256{}) {
        return ConcreteBusyReason::src_deadend;
    }
    if ((cb.src.jump & src_bit) != u256{}) {
        return ConcreteBusyReason::src;
    }
    return ConcreteBusyReason::none;
}

ConcreteBusyReason concreteInBusyReason(const CBState& cb, int dst, int local)
{
    u256 dst_bit = u256{0,1} << dst;
    u256 local_bit = u256{0,1} << local;
    if ((cb.dst.jump & dst_bit) != u256{}) {
        return ConcreteBusyReason::dst;
    }
    if ((cb.local.local & local_bit) != u256{}) {
        return ConcreteBusyReason::local;
    }
    return ConcreteBusyReason::none;
}

std::string concreteSrcWireName(const Tile& tile, fpga::CBNodeNameType from_type, int from_value,
                                int src_node, int joint, const std::string& incoming_wire);

const char* busyReasonName(ConcreteBusyReason reason)
{
    switch (reason) {
    case ConcreteBusyReason::none: return "none";
    case ConcreteBusyReason::dst: return "dst_busy";
    case ConcreteBusyReason::src: return "src_busy";
    case ConcreteBusyReason::local: return "local_busy";
    case ConcreteBusyReason::dst_deadend: return "dst_deadend";
    case ConcreteBusyReason::src_deadend: return "src_deadend";
    }
    return "unknown";
}

std::string nodeDebugName(const Tile& tile, fpga::CBNodeNameType type, int node)
{
    if (!tile.cb_type || node < 0) {
        return {};
    }
    const std::string* name = tile.cb_type->nodeName(type, node);
    return name ? *name : std::string{};
}

std::string netDebugName(rtl::Net* net)
{
    return net ? net->makeName(FULL_NAME_LIMIT) : std::string{};
}

void brutalLocalExitFailure(Tile& tile, const std::vector<uint8_t>* src_nodes,
                            int local, const Coord& target_coord,
                            rtl::Net* current_net, RouteDesign::RouteStats* stats)
{
    PNR_LOG1("ROUT", "BRUTAL local-exit failure: net='{}', tile='{}', coord=({},{}), local={} '{}', target=({},{}), src_count={}",
        netDebugName(current_net), tile.makeName(), tile.coord.x, tile.coord.y,
        local, nodeDebugName(tile, fpga::CB_NODE_LOCAL, local),
        target_coord.x, target_coord.y, src_nodes ? src_nodes->size() : 0);
    PNR_LOG1("ROUT", "BRUTAL tile masks: src={}, dst={}, local={}, src_deadend={}, dst_deadend={}",
        tile.cb.src.jump.str(), tile.cb.dst.jump.str(), tile.cb.local.local.str(),
        tile.cb.src_deadend.jump.str(), tile.cb.dst_deadend.jump.str());
    if (stats) {
        PNR_LOG1("ROUT", "BRUTAL stats: edge_trials={}, edge_ok={}, reject(name={},busy={},busy_dst={},busy_src={},busy_local={},target={},deadend={},dst_deadend={},src_deadend={}), no_src={}, failed={}",
            stats->edge_trials, stats->edge_accepted,
            stats->edge_rejected_no_name, stats->edge_rejected_busy,
            stats->edge_rejected_busy_dst, stats->edge_rejected_busy_src,
            stats->edge_rejected_busy_local, stats->edge_rejected_no_target,
            stats->edge_rejected_deadend, stats->edge_rejected_dst_deadend,
            stats->edge_rejected_src_deadend, stats->no_src_nodes, stats->failed);
    }
    if (src_nodes) {
        for (uint8_t src_node : *src_nodes) {
            ConcreteBusyReason reason = concreteOutBusyReason(tile.cb, src_node);
            rtl::Net* src_owner = fpga::findNetByNode(tile, fpga::CB_NODE_SRC, src_node, false);
            rtl::Net* transit_owner = fpga::findNetByNode(tile, fpga::CB_NODE_SRC, src_node, true);
            int joint = -1;
            std::string src_wire = concreteSrcWireName(tile, fpga::CB_NODE_LOCAL, local, src_node, joint, {});
            fpga::TileJumpTarget target = src_wire.empty() ? fpga::TileJumpTarget{} : fpga::Device::current().resolveJump(tile, src_node, src_wire);
            PNR_LOG1("ROUT", "BRUTAL src candidate: src={} '{}', joint={}, reason={}, owner='{}', transit_owner='{}', concrete='{}', target=({},{}):{} '{}'",
                static_cast<int>(src_node), nodeDebugName(tile, fpga::CB_NODE_SRC, src_node), joint,
                busyReasonName(reason), netDebugName(src_owner), netDebugName(transit_owner), src_wire,
                target.tile ? target.tile->coord.x : -1, target.tile ? target.tile->coord.y : -1,
                target.dst_node, target.dst_wire);
        }
    }
    PNR_ASSERT(false, "route cannot exit local node {} in tile '{}' at ({},{}), and no preemptable transit route was found",
        local, tile.makeName(), tile.coord.x, tile.coord.y);
}

bool leaseConcreteIn(CBState& cb, int dst, int local)
{
    u256 dst_bit = u256{0,1} << dst;
    u256 local_bit = u256{0,1} << local;
    if ((cb.dst.jump & dst_bit) != u256{} || (cb.local.local & local_bit) != u256{}) {
        return false;
    }
    cb.dst.jump |= dst_bit;
    cb.local.local |= local_bit;
    return true;
}

void fillTilePinEndpoint(Wire& wire, rtl::Inst& inst, const std::string& port, fpga::TilePinNameType dir);
void refreshTilePinEndpoint(Wire& wire, rtl::Inst& inst, const std::string& port, fpga::TilePinNameType dir);
void prependSourceEndpoint(std::vector<Wire>& route, rtl::Inst& from, const std::string& from_port);
bool sourceLocalOwnedByDifferentEndpoint(Tile& route_tile, int local, rtl::Inst& from,
                                         const std::string& from_port, rtl::Net* net);

std::string concreteSrcWireName(const Tile& tile, fpga::CBNodeNameType from_type, int from_value,
                                int src_node, int joint, const std::string& from_wire_name = {})
{
    if (!tile.cb_type) {
        return {};
    }
    if (joint >= 0) {
        std::string joint_name;
        if (const fpga::CBConnName* first = selectConcreteConn(tile.cb_type, from_type, from_value,
                fpga::CB_NODE_JOINT, joint, from_wire_name)) {
            if (!from_wire_name.empty() && first->from != from_wire_name) {
                return {};
            }
            joint_name = first->to;
        }
        else if (!from_wire_name.empty()) {
            return {};
        }
        if (const fpga::CBConnName* second = selectConcreteConn(tile.cb_type, fpga::CB_NODE_JOINT, joint,
                fpga::CB_NODE_SRC, src_node, joint_name)) {
            return second->to;
        }
        if (const fpga::CBConnName* second = selectConcreteConn(tile.cb_type, fpga::CB_NODE_SRC, src_node,
                fpga::CB_NODE_JOINT, joint, {}, joint_name)) {
            return second->from;
        }
    }
    if (const fpga::CBConnName* conn = selectConcreteConn(tile.cb_type, from_type, from_value,
            fpga::CB_NODE_SRC, src_node, from_wire_name)) {
        if (!from_wire_name.empty() && conn->from != from_wire_name) {
            return {};
        }
        return conn->to;
    }
    if (!from_wire_name.empty()) {
        return {};
    }
    if (const std::string* src = tile.cb_type->nodeName(fpga::CB_NODE_SRC, src_node)) {
        return *src;
    }
    return {};
}

bool hasRoutableExitFromDst(Tile& tile, int dst_node)
{
    if (!tile.cb_type || dst_node < 0) {
        return false;
    }
    u256 dst_bit = u256{0,1} << dst_node;
    if ((tile.cb.dst_deadend.jump & dst_bit) != u256{}) {
        return false;
    }
    for (int src_node = 0; src_node < CB_MAX_NODES; ++src_node) {
        if ((tile.cb.src_deadend.jump & (u256{0,1} << src_node)) != u256{}) {
            continue;
        }
        int joint = -1;
        if ((tile.cb_type->dst_src[dst_node].jump & (u256{0,1} << src_node)) == u256{}) {
            u256 dst_to_joints = tile.cb_type->dst_joint[dst_node].joint;
            u256 joints_to_src = tile.cb_type->src_joint[src_node].joint;
            u256 intersect = dst_to_joints & joints_to_src;
            joint = intersect.ffs256();
            if (joint < 0) {
                dst_to_joints.for_each_set_bit([&](int index) {
                    joint = (joints_to_src & tile.cb_type->joint_joint[index].joint).ffs256();
                    return joint >= 0;
                });
            }
        }
        if (joint < 0 && (tile.cb_type->dst_src[dst_node].jump & (u256{0,1} << src_node)) == u256{}) {
            continue;
        }
        std::string src_wire = concreteSrcWireName(tile, fpga::CB_NODE_DST, dst_node, src_node, joint);
        if (src_wire.empty()) {
            continue;
        }
        CBState test_cb = tile.cb;
        if (!leaseConcreteJump(test_cb, dst_node, src_node)) {
            continue;
        }
        fpga::TileJumpTarget target = fpga::Device::current().resolveJump(tile, src_node, src_wire);
        if (target.tile && target.tile->cb_type && target.dst_node >= 0) {
            return true;
        }
        if (fpga::Device::current().tile_conn_rules.empty()) {
            Coord next = makeActualJump(tile.coord, src_node);
            Tile* next_tile = fpga::Device::current().getTile(next.x, next.y);
            if (next_tile && next_tile->cb_type) {
                return true;
            }
        }
    }
    return false;
}

std::vector<Tile*> routeTileCandidates(rtl::Inst& inst, const std::string& port, bool output)
{
    std::vector<Tile*> candidates;
    constexpr size_t max_route_candidates = 8;
    auto add_candidate = [&](Tile* tile) {
        if (!tile) {
            return;
        }
        for (Tile* candidate : candidates) {
            if (candidate == tile) {
                return;
            }
        }
        candidates.push_back(tile);
    };

    if (!inst.tile.peer) {
        return candidates;
    }

    u256 endpoint_nodes = output
        ? inst.tile->getOutputPinNodes(inst.cell_ref->type, port, inst.pos)
        : inst.tile->getPinNodes(inst.cell_ref->type, port, inst.pos);

    struct Candidate
    {
        Tile* tile = nullptr;
        int score = 0;
    };
    std::vector<Candidate> scored;
    constexpr int endpoint_search_radius = 6;
    fpga::Device& device = fpga::Device::current();
    for (int dy = -endpoint_search_radius; dy <= endpoint_search_radius; ++dy) {
        for (int dx = -endpoint_search_radius; dx <= endpoint_search_radius; ++dx) {
            Coord coord = inst.tile->coord + Coord{dx, dy};
            Tile* tile_ptr = device.getTile(coord.x, coord.y);
            if (!tile_ptr) {
                continue;
            }
            Tile& tile = *tile_ptr;
            int distance = std::abs(dx) + std::abs(dy);
            if (distance > endpoint_search_radius) {
                continue;
            }
            u256 candidate_nodes = endpoint_nodes;
            if (candidate_nodes == u256{} || !(output ? supportsOutputLocalNodes(tile, candidate_nodes) : supportsLocalNodes(tile, candidate_nodes))) {
                candidate_nodes = output
                    ? tile.getOutputPinNodes(inst.cell_ref->type, port, inst.pos)
                    : tile.getPinNodes(inst.cell_ref->type, port, inst.pos);
            }
            if (candidate_nodes == u256{}) {
                continue;
            }
            if (!(output ? supportsOutputLocalNodes(tile, candidate_nodes) : supportsLocalNodes(tile, candidate_nodes))) {
                continue;
            }
            int score = distance * 4;
            if (tile.coord == inst.tile->coord) {
                score -= 3;
            }
            if (tile.name == inst.tile->name) {
                score -= 2;
            }
            scored.push_back(Candidate{&tile, score});
        }
    }

    std::sort(scored.begin(), scored.end(), [](const Candidate& a, const Candidate& b) {
        return a.score < b.score;
    });
    for (const Candidate& candidate : scored) {
        add_candidate(candidate.tile);
        if (candidates.size() >= max_route_candidates) {
            break;
        }
    }
    if (candidates.empty() && isConcreteRouteTile(*inst.tile)) {
        add_candidate(&*inst.tile);
    }
    return candidates;
}

bool tryBestFirstRoute(Tile& from, Tile& to, int from_pos, rtl::Inst& dst_inst,
                       const std::string& to_port, std::vector<Wire>& wire, int iteration_limit,
                       bool start_from_dst = false, const std::string& start_dst_wire = std::string{},
                       bool* complete = nullptr, RouteDesign::RouteStats* stats = nullptr, RouteDesign* router = nullptr,
                       rtl::Net* current_net = nullptr, bool branch_from_existing = false,
                       rtl::Inst* current_source = nullptr, const std::string& current_source_port = std::string{},
                       u256 dst_pin_nodes = {})
{
    struct Step
    {
        Coord coord;
        int local = -1;
        int prev = -1;
        int jump = -1;
        int joint = -1;
        int depth = 0;
        std::string src_wire;
        std::string dst_wire;
    };
    struct QueueItem
    {
        int score = 0;
        int step = 0;
        bool operator<(const QueueItem& other) const
        {
            return score > other.score;
        }
    };

    u256 pin_nodes = dst_pin_nodes == u256{} ? to.getPinNodes(dst_inst.cell_ref->type, to_port, dst_inst.pos) : dst_pin_nodes;
    if (pin_nodes == u256{}) {
        return false;
    }
    if (stats) {
        ++stats->route_searches;
    }

    std::vector<Step> steps;
    std::priority_queue<QueueItem> queue;
    steps.push_back(Step{from.coord, from_pos, -1, -1, -1, start_from_dst ? 1 : 0, {}, start_dst_wire});
    queue.push(QueueItem{routeDistance(from.coord, to.coord), 0});

    int final_step = -1;
    int final_pin = -1;
    int final_joint = -1;
    int best_step = -1;
    int fallback_step = -1;
    int best_distance = routeDistance(from.coord, to.coord);
    constexpr int max_depth = 5;
    size_t max_steps = static_cast<size_t>(std::max(32, iteration_limit * 16));

    while (!queue.empty() && steps.size() < max_steps) {
        int idx = queue.top().step;
        queue.pop();
        Step step = steps[idx];
        Tile* tile = fpga::Device::current().getTile(step.coord.x, step.coord.y);
        if (!tile || !isConcreteRouteTile(*tile) || step.depth > max_depth) {
            continue;
        }
        if (stats) {
            ++stats->search_pops;
            ++stats->pops_by_depth[statDepthBucket(step.depth)];
            if (tile->cb.dst_deadend.jump != u256{} || tile->cb.src_deadend.jump != u256{}) {
                ++stats->pops_on_deadend_tile;
            }
            if (tile->cb.dst_deadend.jump != u256{}) {
                ++stats->pops_on_dst_deadend_tile;
            }
            if (tile->cb.src_deadend.jump != u256{}) {
                ++stats->pops_on_src_deadend_tile;
            }
        }
        int distance = routeDistance(step.coord, to.coord);
        if (idx != 0) {
            if (distance < best_distance || (distance == best_distance && (best_step < 0 || step.depth > steps[best_step].depth))) {
                best_distance = distance;
                best_step = idx;
            }
            if (fallback_step < 0 || step.depth > steps[fallback_step].depth
                || (step.depth == steps[fallback_step].depth && distance < routeDistance(steps[fallback_step].coord, to.coord))) {
                fallback_step = idx;
            }
        }

        if (step.coord == to.coord) {
            bool ok = pin_nodes.for_each_set_bit([&](int pin) {
                int joint = -1;
                if (tile->isPinNodeLeased(pin) || !tile->cb_type->canIn(step.local, pin, joint)) {
                    return false;
                }
                CBState test_cb = tile->cb;
                if (!leaseConcreteIn(test_cb, step.local, pin)) {
                    TransitVictim victim = router
                        ? findTransitDstVictim(*tile, step.local, current_net, current_source, current_source_port)
                        : TransitVictim{};
                    if (!current_net || !victim.net || victim.net == current_net
                        || netDebugName(victim.net) == netDebugName(current_net)
                        || victim.binding_index >= victim.net->routes.size()) {
                        return false;
                    }
                    rtl::NetRouteBinding victim_binding = victim.net->routes[victim.binding_index];
                    if (!fpga::unrouteNetRoute(*victim.net, victim.binding_index)) {
                        return false;
                    }
                    if (victim_binding.from && victim_binding.to && !victim_binding.route_name.empty()) {
                        auto same_task = [&](const RouteDesign::RouteTask& task) {
                            return task.net == victim.net
                                && task.from == victim_binding.from
                                && task.to == victim_binding.to
                                && task.from_port == victim_binding.from_port
                                && task.to_port == victim_binding.to_port
                                && task.net_name == victim_binding.route_name;
                        };
                        bool already_queued = std::any_of(router->route_todo.begin(), router->route_todo.end(), same_task)
                            || std::any_of(router->pending_route_todo.begin(), router->pending_route_todo.end(), same_task);
                        if (!already_queued) {
                            bool victim_has_base_route = false;
                            for (size_t route_index = 0; route_index < victim.net->routes.size(); ++route_index) {
                                const rtl::NetRouteBinding& binding = victim.net->routes[route_index];
                                if (route_index == victim.binding_index || !binding.owner || binding.route_index >= binding.owner->wires.size()) {
                                    continue;
                                }
                                if (routeIsComplete(binding.owner->wires[binding.route_index])) {
                                    victim_has_base_route = true;
                                    break;
                                }
                            }
                            router->pending_route_todo.push_back(RouteDesign::RouteTask{
                                victim_binding.from,
                                victim_binding.to,
                                victim.net,
                                victim_binding.from_port,
                                victim_binding.to_port,
                                victim_binding.route_name,
                                0,
                                victim_has_base_route
                            });
                        }
                    }
                    if (stats) {
                        ++stats->preempt_attempts;
                        ++stats->preempt_success;
                    }
                    PNR_LOG3("ROUT", "routeDesign grounding preempt: tile=({},{}), dst={}, pin={}, victim='{}'",
                        tile->coord.x, tile->coord.y, step.local, pin, victim.net->makeName(FULL_NAME_LIMIT));
                    test_cb = tile->cb;
                    if (!leaseConcreteIn(test_cb, step.local, pin)) {
                        return false;
                    }
                }
                final_step = idx;
                final_pin = pin;
                final_joint = joint;
                return true;
            });
            if (ok) {
                break;
            }
        }

        fpga::CBNodeNameType from_type = step.depth == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
        if (from_type == fpga::CB_NODE_DST
            && (tile->cb.dst_deadend.jump & (u256{0,1} << step.local)) != u256{}) {
            if (stats) {
                ++stats->edge_rejected_deadend;
                ++stats->edge_rejected_dst_deadend;
            }
            continue;
        }
        const std::vector<uint8_t>* src_nodes = tile->cb_type->srcNodes(from_type, step.local);
        if (!src_nodes) {
            if (stats) {
                ++stats->no_src_nodes;
                if (step.depth == 0) {
                    ++stats->no_src_nodes_depth0;
                }
                u256 joint_mask = from_type == fpga::CB_NODE_LOCAL
                    ? tile->cb_type->local_joint[step.local].joint
                    : tile->cb_type->dst_joint[step.local].joint;
                if (joint_mask != u256{}) {
                    ++stats->no_src_nodes_with_joint_path;
                }
                stats->has_last_no_src = true;
                stats->last_no_src_coord = tile->coord;
                stats->last_no_src_depth = step.depth;
                stats->last_no_src_local = step.local;
                stats->last_no_src_joint_mask = joint_mask;
            }
            if (step.depth == 0) {
                brutalLocalExitFailure(*tile, nullptr, step.local, to.coord, current_net, stats);
            }
            continue;
        }
        std::vector<uint8_t> active_src_nodes;
        active_src_nodes.reserve(src_nodes->size());
        for (uint8_t src_node : *src_nodes) {
            if ((tile->cb.src_deadend.jump & (u256{0,1} << src_node)) != u256{}) {
                continue;
            }
            active_src_nodes.push_back(src_node);
        }

        bool accepted_from_step = false;
        rtl::Net* preempt_victim = nullptr;
        int preempt_src = -1;
        for (uint8_t src_node : active_src_nodes) {
                if (stats) {
                    ++stats->edge_trials;
                    ++stats->trials_by_depth[statDepthBucket(step.depth)];
                }
                int joint = -1;
                std::string src_wire = concreteSrcWireName(*tile, from_type, step.local, src_node, joint, step.dst_wire);
                if (src_wire.empty()) {
                    if (stats) {
                        ++stats->edge_rejected_no_name;
                    }
                    continue;
                }
                CBState test_cb = tile->cb;
                bool lease_ok = false;
                if (branch_from_existing && idx == 0 && step.depth != 0) {
                    lease_ok = leaseConcreteFork(test_cb, step.local, src_node);
                }
                else {
                    lease_ok = step.depth == 0
                        ? leaseConcreteOut(test_cb, step.local, src_node)
                        : leaseConcreteJump(test_cb, step.local, src_node);
                }
                if (!lease_ok) {
                    ConcreteBusyReason reason = step.depth == 0
                        ? concreteOutBusyReason(tile->cb, src_node)
                        : concreteJumpBusyReason(tile->cb, step.local, src_node);
                    if (step.depth == 0 && reason == ConcreteBusyReason::src && !preempt_victim) {
                        rtl::Net* victim = fpga::findNetByNode(*tile, fpga::CB_NODE_SRC, src_node, true);
                        if (victim && victim != current_net) {
                            preempt_victim = victim;
                            preempt_src = src_node;
                        }
                    }
                }
                if (!lease_ok) {
                    if (stats) {
                        ++stats->edge_rejected_busy;
                        ConcreteBusyReason reason = step.depth == 0
                            ? concreteOutBusyReason(tile->cb, src_node)
                            : concreteJumpBusyReason(tile->cb, step.local, src_node);
                        if (reason == ConcreteBusyReason::dst) {
                            ++stats->edge_rejected_busy_dst;
                        }
                        else if (reason == ConcreteBusyReason::src) {
                            ++stats->edge_rejected_busy_src;
                        }
                        else if (reason == ConcreteBusyReason::local) {
                            ++stats->edge_rejected_busy_local;
                        }
                        else if (reason == ConcreteBusyReason::dst_deadend) {
                            ++stats->edge_rejected_deadend;
                            ++stats->edge_rejected_dst_deadend;
                        }
                        else if (reason == ConcreteBusyReason::src_deadend) {
                            ++stats->edge_rejected_deadend;
                            ++stats->edge_rejected_src_deadend;
                        }
                        stats->has_last_busy = true;
                        stats->last_busy_coord = tile->coord;
                        stats->last_busy_depth = step.depth;
                        stats->last_busy_local = step.local;
                        stats->last_busy_src = src_node;
                        stats->last_busy_src_mask = tile->cb.src.jump;
                        stats->last_busy_dst_mask = tile->cb.dst.jump;
                        stats->last_busy_local_mask = tile->cb.local.local;
                    }
                    continue;
                }
                fpga::TileJumpTarget target = fpga::Device::current().resolveJump(*tile, src_node, src_wire);
                Coord next;
                int next_local = -1;
            if (target.tile && target.tile->cb_type && target.dst_node >= 0) {
                next = target.tile->coord;
                next_local = target.dst_node;
                if ((target.tile->cb.dst_deadend.jump & (u256{0,1} << next_local)) != u256{}) {
                    if (stats) {
                        ++stats->edge_rejected_deadend;
                        ++stats->edge_rejected_dst_deadend;
                    }
                    continue;
                }
            }
            else {
                if (!fpga::Device::current().tile_conn_rules.empty()) {
                    if (stats) {
                        ++stats->edge_rejected_no_target;
                    }
                    continue;
                }
                next = makeActualJump(step.coord, src_node);
                Tile* next_tile = fpga::Device::current().getTile(next.x, next.y);
                if (!next_tile || !next_tile->cb_type) {
                    if (stats) {
                        ++stats->edge_rejected_no_target;
                    }
                    continue;
                }
                next_local = src_node;
            }
            steps.push_back(Step{next, next_local, idx, src_node, joint, step.depth + 1, src_wire, target.dst_wire});
            if (stats) {
                ++stats->edge_accepted;
                ++stats->accepted_by_depth[statDepthBucket(step.depth)];
            }
            accepted_from_step = true;
            int next_idx = static_cast<int>(steps.size() - 1);
            int score = routeDistance(next, to.coord) * 4 + step.depth;
            queue.push(QueueItem{score, next_idx});
        }
        if (!accepted_from_step && step.depth == 0 && preempt_victim && router) {
            if (stats) {
                ++stats->preempt_attempts;
            }
            if (fpga::unrouteNet(*preempt_victim)) {
                router->requeueNet(*preempt_victim);
                if (stats) {
                    ++stats->preempt_success;
                }
                PNR_LOG3("ROUT", "routeDesign preempt: tile=({},{}), local={}, src={}, victim='{}'",
                    tile->coord.x, tile->coord.y, step.local, preempt_src,
                    preempt_victim->makeName(FULL_NAME_LIMIT));
                queue.push(QueueItem{routeDistance(step.coord, to.coord) * 4 - 1, idx});
            }
        }
        else if (!accepted_from_step && step.depth == 0 && !preempt_victim) {
            brutalLocalExitFailure(*tile, src_nodes, step.local, to.coord, current_net, stats);
        }
    }

    bool completed = final_step >= 0;
    if (!completed) {
        if (best_step >= 0) {
            final_step = best_step;
        }
        else if (start_from_dst) {
            return false;
        }
        else {
            final_step = fallback_step;
        }
    }
    if (final_step < 0) {
        return false;
    }
    if (complete) {
        *complete = completed;
    }
    if (stats) {
        size_t depth = statDepthBucket(steps[final_step].depth);
        if (completed) {
            ++stats->completed_by_depth[depth];
        }
        else {
            ++stats->partial_by_depth[depth];
        }
    }

    struct Commit
    {
        Tile* tile = nullptr;
        CBState cb;
        TilePinState pin_state;
    };
    std::vector<Commit> commits;
    auto snapshot_tile = [&](Tile* tile) {
        for (Commit& commit : commits) {
            if (commit.tile == tile) {
                return;
            }
        }
        commits.push_back(Commit{tile, tile->cb, tile->pin_state});
    };
    size_t final_depth = statDepthBucket(steps[final_step].depth);
    auto rollback = [&]() {
        if (stats) {
            ++stats->commit_rollbacks;
            ++stats->rollbacks_by_depth[final_depth];
        }
        for (Commit& commit : commits) {
            commit.tile->cb = commit.cb;
            commit.tile->pin_state = commit.pin_state;
        }
    };

    std::vector<int> path;
    for (int idx = final_step; idx >= 0; idx = steps[idx].prev) {
        path.push_back(idx);
    }
    std::reverse(path.begin(), path.end());

    for (size_t i = 1; i < path.size(); ++i) {
        const Step& prev = steps[path[i - 1]];
        const Step& curr = steps[path[i]];
        Tile* prev_tile = fpga::Device::current().getTile(prev.coord.x, prev.coord.y);
        if (!prev_tile) {
            rollback();
            return false;
        }
        snapshot_tile(prev_tile);
        bool lease_ok = false;
        if (branch_from_existing && i == 1 && prev.depth != 0) {
            lease_ok = leaseConcreteFork(prev_tile->cb, prev.local, curr.jump);
        }
        else {
            lease_ok = prev.depth == 0
                ? leaseConcreteOut(prev_tile->cb, prev.local, curr.jump)
                : leaseConcreteJump(prev_tile->cb, prev.local, curr.jump);
        }
        if (!lease_ok) {
            rollback();
            return false;
        }
    }

    if (completed) {
        Tile* final_tile = fpga::Device::current().getTile(to.coord.x, to.coord.y);
        if (!final_tile) {
            rollback();
            return false;
        }
        snapshot_tile(final_tile);
        if (!leaseConcreteIn(final_tile->cb, steps[final_step].local, final_pin)
            || !final_tile->leasePinNode(final_pin)) {
            rollback();
            return false;
        }
    }

    wire.clear();
    for (size_t i = 1; i < path.size(); ++i) {
        const Step& prev = steps[path[i - 1]];
        const Step& curr = steps[path[i]];
        Wire fragment;
        fragment.from = prev.coord;
        fragment.to = curr.coord;
        fragment.local = prev.local;
        fragment.jump = curr.jump;
        fragment.joint = curr.joint;
        fragment.pos = (branch_from_existing && i == 1 && prev.depth != 0)
            ? ROUTE_POS_FORK
            : (prev.depth == 0 ? ROUTE_POS_SOURCE : ROUTE_POS_TRANSIT);
        const Tile* prev_tile = fpga::Device::current().getTile(prev.coord.x, prev.coord.y);
        const Tile* curr_tile = fpga::Device::current().getTile(curr.coord.x, curr.coord.y);
        if (prev_tile) {
            fpga::CBNodeNameType from_type = prev.depth == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
            fragment.src_wire_name = concreteSrcWireName(*prev_tile, from_type, prev.local, curr.jump, curr.joint, prev.dst_wire);
        }
        if (!curr.dst_wire.empty()) {
            fragment.dst_wire_name = curr.dst_wire;
        }
        else if (curr_tile && curr_tile->cb_type) {
            if (const std::string* dst = curr_tile->cb_type->nodeName(fpga::CB_NODE_DST, curr.local)) {
                fragment.dst_wire_name = *dst;
            }
        }
        wire.push_back(fragment);
    }

    if (completed) {
        Wire enter;
        enter.from = to.coord;
        enter.to = to.coord;
        enter.local = steps[final_step].local;
        enter.joint = final_joint;
        enter.pos = 1;
        enter.dst_wire_name = steps[final_step].dst_wire;
        wire.push_back(enter);

        Wire pin;
        fillTilePinEndpoint(pin, dst_inst, to_port, fpga::TILE_PIN_INPUT);
        pin.from = to.coord;
        pin.to = to.coord;
        pin.local = final_pin;
        refreshTilePinEndpoint(pin, dst_inst, to_port, fpga::TILE_PIN_INPUT);
        wire.push_back(pin);
    }
    return true;
}

bool isIoBuffer(rtl::Inst& inst)
{
    return technology::Tech::current().buffers_ports.find(inst.cell_ref->type) != technology::Tech::current().buffers_ports.end();
}

// Initializes a tile-pin wire with the resource-side identity known from the placed instance.
// The selected local may be refined later, so callers can refresh resource_node after choosing it.
void fillTilePinEndpoint(Wire& wire, rtl::Inst& inst, const std::string& port, fpga::TilePinNameType dir)
{
    wire.type = Wire::WIRE_TILE_PIN;
    wire.resource = inst.tile.peer ? inst.tile->coord : Coord{};
    wire.pos = inst.pos;
    wire.resource_node = (inst.tile.peer && inst.cell_ref.peer)
        ? inst.tile->getResourceNodeNum(inst.cell_ref->type, port, inst.pos, dir, wire.local)
        : -1;
    wire.pin_dir = dir;
    wire.cell_type = inst.cell_ref.peer ? inst.cell_ref->type : std::string{};
    wire.port = port;
}

// Recomputes the resource endpoint after routing has selected the concrete local node.
// This keeps exported annotations tied to the actual local-to-resource connection used.
void refreshTilePinEndpoint(Wire& wire, rtl::Inst& inst, const std::string& port, fpga::TilePinNameType dir)
{
    if (!inst.tile.peer || !inst.cell_ref.peer) {
        return;
    }
    wire.resource_node = inst.tile->getResourceNodeNum(inst.cell_ref->type, port, inst.pos, dir, wire.local);
}

// Adds the source resource-to-crossbar hop before the first crossbar fragment.
// Routing stores crossbar fragments first, so export needs this explicit endpoint fragment.
void prependSourceEndpoint(std::vector<Wire>& route, rtl::Inst& from, const std::string& from_port)
{
    if (route.empty() || route.front().type == Wire::WIRE_TILE_PIN || !from.tile.peer) {
        return;
    }

    Wire source;
    fillTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
    source.from = route.front().from;
    source.to = route.front().from;
    source.local = route.front().local;
    refreshTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
    source.net_name = route.front().net_name;
    route.insert(route.begin(), std::move(source));
}

void unplaceInst(rtl::Inst& inst)
{
    if (!inst.tile.peer) {
        return;
    }
    Tile& tile = *inst.tile;
    const std::string& type = inst.cell_ref->type;
    if (type.find("FD") == 0) {
        tile.regs_cnt = std::max(0, tile.regs_cnt - 1);
    }
    else if (type.find("LUT") == 0) {
        if (inst.cnt_inputs == 1) {
            tile.luts1cnt = std::max(0, tile.luts1cnt - 1);
        }
        else if (inst.cnt_inputs == 6) {
            tile.luts6cnt = std::max(0, tile.luts6cnt - 1);
        }
        else {
            tile.luts5cnt = std::max(0, tile.luts5cnt - 1);
        }
    }
    else if (type.find("CARRY") == 0) {
        tile.carry = 0;
    }
    else if (type.find("MUX") == 0) {
        tile.mux = 0;
        tile.luts6cnt = std::max(0, tile.luts6cnt - 2);
    }
    inst.tile.clear();
}

void restoreInstPlacement(rtl::Inst& inst, Tile& tile, int pos)
{
    const std::string& type = inst.cell_ref->type;
    if (type.find("FD") == 0) {
        ++tile.regs_cnt;
    }
    else if (type.find("LUT") == 0) {
        if (inst.cnt_inputs == 1) {
            ++tile.luts1cnt;
        }
        else if (inst.cnt_inputs == 6) {
            ++tile.luts6cnt;
        }
        else {
            ++tile.luts5cnt;
        }
    }
    else if (type.find("CARRY") == 0) {
        tile.carry = 4;
    }
    else if (type.find("MUX") == 0) {
        tile.mux = 1;
        tile.luts6cnt += 2;
    }
    tile.assign(&inst);
    inst.coord = tile.coord;
    inst.pos = pos;
}

rtl::Module* parentModule(rtl::Inst& inst)
{
    if (!inst.cell_ref.peer || !inst.cell_ref->module_ref.peer) {
        return nullptr;
    }
    return inst.cell_ref->module_ref->parent_ref.peer;
}

bool netHasCompleteRouteExcept(rtl::Net& net, size_t skip_index)
{
    for (size_t index = 0; index < net.routes.size(); ++index) {
        if (index == skip_index) {
            continue;
        }
        rtl::NetRouteBinding& binding = net.routes[index];
        if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
            continue;
        }
        if (routeIsComplete(binding.owner->wires[binding.route_index])) {
            return true;
        }
    }
    return false;
}

uint64_t placementKey(const Coord& coord, int pos)
{
    return (static_cast<uint64_t>(static_cast<uint16_t>(coord.x)) << 48)
        | (static_cast<uint64_t>(static_cast<uint16_t>(coord.y)) << 32)
        | static_cast<uint32_t>(pos);
}

bool placementWasTried(const std::vector<uint64_t>& tried, const Coord& coord, int pos)
{
    uint64_t key = placementKey(coord, pos);
    return std::find(tried.begin(), tried.end(), key) != tried.end();
}

int iterationLimitFromCells(int cells)
{
    (void)cells;
    return 5;
}

int moveAttemptLimitFromCells(int cells)
{
    return std::max(16, cells / 10);
}

int countReachableCells(rtl::Inst& inst, RegBunch* bunch, uint64_t mark)
{
    if (inst.mark == mark) {
        return 0;
    }
    inst.mark = mark;

    int cells = inst.cell_ref.peer && inst.cell_ref->module_ref.peer && inst.cell_ref->module_ref->is_blackbox ? 1 : 0;

    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        if (!driver_conn || !driver_conn->inst_ref.peer) {
            continue;
        }
        cells += countReachableCells(*driver_conn->inst_ref, nullptr, mark);
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            if (subbunch.reg) {
                cells += countReachableCells(*subbunch.reg, &subbunch, mark);
            }
        }
    }

    return cells;
}

int countDesignCells(std::list<Referable<RegBunch>>& bunch_list)
{
    uint64_t mark = rtl::Inst::genMark();
    int cells = 0;
    for (auto& bunch : bunch_list) {
        if (bunch.reg) {
            cells += countReachableCells(*bunch.reg, &bunch, mark);
        }
    }
    return cells;
}

void resetRoutingState()
{
    for (auto& tile : fpga::Device::current().tile_grid) {
        std::memset(&tile.cb, 0, sizeof(tile.cb));
        tile.cb.type = tile.cb_type;
        tile.pin_state = {};
        tile.routedNets.clear();
    }
}

Wire makeEndpointWire(rtl::Inst& from, const std::string& from_port, rtl::Inst& to, const std::string& to_port)
{
    Wire wire;
    wire.port = to_port.empty() ? from_port : to_port;

    if (isIoBuffer(from) && from.tile.peer) {
        fillTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
        wire.from = from.tile->coord;
        wire.to = from.tile->coord;
        wire.local = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos).ffs256();
        if (wire.local < 0) {
            wire.local = from.pos;
        }
        refreshTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
        wire.pos = from.pos;
        wire.port = from_port;
        return wire;
    }

    if (isIoBuffer(to) && to.tile.peer) {
        fillTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
        wire.from = to.tile->coord;
        wire.to = to.tile->coord;
        wire.local = to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos).ffs256();
        if (wire.local < 0) {
            wire.local = to.pos;
        }
        refreshTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
        wire.pos = to.pos;
        wire.port = to_port;
        return wire;
    }

    if (to.tile.peer) {
        fillTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
        wire.from = to.tile->coord;
        wire.to = to.tile->coord;
        wire.local = to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos).ffs256();
        refreshTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
        wire.pos = to.pos;
        return wire;
    }

    if (from.tile.peer) {
        fillTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
        wire.from = from.tile->coord;
        wire.to = from.tile->coord;
        wire.local = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos).ffs256();
        refreshTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
        wire.pos = from.pos;
        return wire;
    }

    wire.from = Coord{-1, -1};
    wire.to = Coord{-1, -1};
    return wire;
}

bool anyRoutableOutputCandidate(const std::vector<Tile*>& route_tiles, u256 output_nodes)
{
    for (Tile* tile : route_tiles) {
        if (!tile) {
            continue;
        }
        bool has_routable = output_nodes.for_each_set_bit([&](int local) {
            return isRoutableOutputLocal(*tile, local);
        });
        if (has_routable) {
            return true;
        }
    }
    return false;
}

bool tryDirectResourceRoute(rtl::Inst& from, const std::string& from_port,
                            rtl::Inst& to, const std::string& to_port,
                            std::vector<Wire>& wire, rtl::Net* net)
{
    if (!from.tile.peer || !to.tile.peer) {
        return false;
    }

    u256 output_nodes = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
    u256 input_nodes = to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos);
    if (output_nodes == u256{}) {
        return false;
    }

    int output_node = output_nodes.ffs256();
    if (sourceLocalOwnedByDifferentEndpoint(*from.tile, output_node, from, from_port, net)) {
        return false;
    }
    int input_node = -1;
    if (input_nodes == u256{} && from.tile.peer == to.tile.peer) {
        // Same-tile packed shapes may expose only the driven output node in the tile map.
        input_nodes = u256{0,1} << output_node;
    }
    if (input_nodes == u256{}) {
        return false;
    }
    bool input_ok = input_nodes.for_each_set_bit([&](int local) {
        if (to.tile->isPinNodeLeased(local)) {
            return false;
        }
        input_node = local;
        return true;
    });
    if (!input_ok || input_node < 0 || !to.tile->leasePinNode(input_node)) {
        return false;
    }

    wire.clear();
    Wire source;
    fillTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
    source.from = from.tile->coord;
    source.to = from.tile->coord;
    source.local = output_node;
    refreshTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
    source.net_name = net ? net->makeName(FULL_NAME_LIMIT) : std::string{};
    wire.push_back(source);

    Wire sink;
    fillTilePinEndpoint(sink, to, to_port, fpga::TILE_PIN_INPUT);
    sink.from = to.tile->coord;
    sink.to = to.tile->coord;
    sink.local = input_node;
    refreshTilePinEndpoint(sink, to, to_port, fpga::TILE_PIN_INPUT);
    sink.net_name = source.net_name;
    wire.push_back(sink);
    return true;
}

bool partialRouteEndpoint(const std::vector<Wire>& route, Tile*& tile, int& local, std::string& dst_wire)
{
    tile = nullptr;
    local = -1;
    dst_wire.clear();
    for (auto it = route.rbegin(); it != route.rend(); ++it) {
        const Wire& fragment = *it;
        if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0) {
            continue;
        }
        const Tile* from_tile = fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
        if (!from_tile) {
            return false;
        }
        fpga::TileJumpTarget target = fpga::Device::current().resolveJump(*from_tile, fragment.jump, fragment.src_wire_name);
        if (!target.tile || target.dst_node < 0) {
            return false;
        }
        tile = target.tile;
        local = target.dst_node;
        dst_wire = target.dst_wire;
        return true;
    }
    return false;
}

bool continuePartialRoute(std::vector<Wire>& route, rtl::Inst& to, const std::string& to_port,
                          int iteration_limit, bool& complete, RouteDesign::RouteStats* stats = nullptr,
                          RouteDesign* router = nullptr, rtl::Net* current_net = nullptr,
                          rtl::Inst* current_source = nullptr, const std::string& current_source_port = std::string{})
{
    complete = false;
    Tile* from_tile = nullptr;
    int from_pos = -1;
    std::string from_dst_wire;
    if (!partialRouteEndpoint(route, from_tile, from_pos, from_dst_wire) || !from_tile) {
        return false;
    }
    std::vector<Tile*> to_route_tiles = routeTileCandidates(to, to_port, false);
    for (Tile* to_route_tile : to_route_tiles) {
        std::vector<Wire> continuation;
        bool attempt_complete = false;
        u256 pin_nodes = to_route_tile ? routeTileInputNodes(*to_route_tile, to, to_port) : u256{};
        if (to_route_tile && tryBestFirstRoute(*from_tile, *to_route_tile, from_pos, to, to_port,
                continuation, iteration_limit, true, from_dst_wire, &attempt_complete, stats, router, current_net, false, current_source, current_source_port, pin_nodes)) {
            route.insert(route.end(), continuation.begin(), continuation.end());
            complete = attempt_complete;
            return true;
        }
    }
    return false;
}

bool ripLastRouteStep(std::vector<Wire>& route, RouteDesign::RouteStats* stats = nullptr)
{
    if (stats) {
        ++stats->backstep_attempts;
    }
    size_t removed = 0;
    while (!route.empty() && route.back().type == Wire::WIRE_TILE_PIN) {
        route.pop_back();
        ++removed;
    }
    if (route.empty()) {
        if (stats) {
            stats->backstep_fragments += removed;
        }
        return false;
    }
    size_t depth = 0;
    for (const Wire& fragment : route) {
        if (fragment.type == Wire::WIRE_CROSSBAR) {
            ++depth;
        }
    }
    route.pop_back();
    ++removed;
    if (stats) {
        ++stats->backstep_success;
        stats->backstep_fragments += removed;
        ++stats->backsteps_by_depth[statDepthBucket(static_cast<int>(depth))];
    }
    return true;
}

bool replayRouteLeases(const std::vector<Wire>& route)
{
    for (const Wire& fragment : route) {
        if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0) {
            continue;
        }
        Tile* tile = fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
        if (!tile || !tile->cb_type) {
            return false;
        }
        bool ok = false;
        if (fragment.pos == ROUTE_POS_SOURCE) {
            ok = leaseConcreteOut(tile->cb, fragment.local, fragment.jump);
        }
        else if (fragment.pos == ROUTE_POS_FORK) {
            ok = leaseConcreteFork(tile->cb, fragment.local, fragment.jump);
        }
        else {
            ok = leaseConcreteJump(tile->cb, fragment.local, fragment.jump);
        }
        if (!ok) {
            return false;
        }
    }

    for (size_t i = 0; i < route.size(); ++i) {
        if (route[i].type != Wire::WIRE_TILE_PIN) {
            continue;
        }
        if (i + 1 != route.size()) {
            continue;
        }
        Tile* tile = fpga::Device::current().getTile(route[i].from.x, route[i].from.y);
        if (!tile || !tile->cb_type || route[i].local < 0) {
            return false;
        }
        int from_local = -1;
        if (i > 0 && route[i - 1].type == Wire::WIRE_CROSSBAR) {
            from_local = route[i - 1].local;
        }
        if (from_local >= 0 && !leaseConcreteIn(tile->cb, from_local, route[i].local)) {
            return false;
        }
        if (!tile->leasePinNode(route[i].local)) {
            return false;
        }
    }
    return true;
}

void collectInstRoutes(rtl::Inst& inst, std::vector<std::vector<Wire>*>& routes)
{
    for (auto& route : inst.wires) {
        if (!route.empty()) {
            routes.push_back(&route);
        }
    }
    for (auto& sub_inst : inst.insts) {
        collectInstRoutes(sub_inst, routes);
    }
}

void collectInstsWithRoutes(rtl::Inst& inst, std::vector<rtl::Inst*>& insts)
{
    insts.push_back(&inst);
    for (auto& sub_inst : inst.insts) {
        collectInstsWithRoutes(sub_inst, insts);
    }
}


bool sameSourceEndpoint(const Wire& existing, rtl::Inst& from, const std::string& from_port, int local)
{
    if (!from.tile.peer || !from.cell_ref.peer) {
        return false;
    }

    int resource_node = from.tile->getResourceNodeNum(from.cell_ref->type, from_port,
        from.pos, fpga::TILE_PIN_OUTPUT, local);
    bool resource_node_matches = existing.resource_node < 0 || resource_node < 0
        || existing.resource_node == resource_node;
    return existing.pin_dir == fpga::TILE_PIN_OUTPUT
        && existing.local == local
        && existing.resource.x == from.tile->coord.x && existing.resource.y == from.tile->coord.y
        && existing.pos == from.pos
        && existing.cell_type == from.cell_ref->type
        && existing.port == from_port
        && resource_node_matches;
}

bool sourceLocalOwnedByDifferentEndpoint(Tile& route_tile, int local, rtl::Inst& from,
                                         const std::string& from_port, rtl::Net* net)
{
    if (local < 0 || !from.tile.peer || !from.cell_ref.peer) {
        return false;
    }

    std::vector<rtl::Inst*> insts;
    collectInstsWithRoutes(technology::Tech::current().design.top, insts);
    for (rtl::Inst* owner : insts) {
        if (!owner) {
            continue;
        }
        for (const std::vector<Wire>& route : owner->wires) {
            if (route.empty()) {
                continue;
            }
            const Wire& existing = route.front();
            if (existing.type != Wire::WIRE_TILE_PIN
                || existing.pin_dir != fpga::TILE_PIN_OUTPUT
                || existing.local != local
                || existing.from.x != route_tile.coord.x || existing.from.y != route_tile.coord.y) {
                continue;
            }
            if (sameSourceEndpoint(existing, from, from_port, local)) {
                continue;
            }

            PNR_LOG1("ROUT", "source local conflict: net='{}', tile='{}' ({},{}), local={}, new={}/{} pos={} resource=({},{}), existing_net='{}', existing={}/{} pos={} resource=({},{}), owner='{}'",
                net ? net->makeName(FULL_NAME_LIMIT) : std::string{},
                route_tile.makeName(), route_tile.coord.x, route_tile.coord.y, local,
                from.cell_ref->type, from_port, from.pos, from.tile->coord.x, from.tile->coord.y,
                existing.net_name, existing.cell_type, existing.port, existing.pos,
                existing.resource.x, existing.resource.y, owner->makeName());
            return true;
        }
    }
    return false;
}

void collectBunchRoutes(RegBunch& bunch, std::vector<std::vector<Wire>*>& routes)
{
    if (bunch.reg) {
        collectInstRoutes(*bunch.reg, routes);
    }
    for (auto& subbunch : bunch.sub_bunches) {
        collectBunchRoutes(subbunch, routes);
    }
}

void registerExistingNetRoutes(rtl::Module& module)
{
    for (auto& net : module.nets) {
        for (rtl::NetRouteBinding& binding : net.routes) {
            std::vector<Wire>* route = nullptr;
            if (binding.owner && binding.route_index < binding.owner->wires.size()) {
                route = &binding.owner->wires[binding.route_index];
            }
            if (route && !route->empty()) {
                fpga::registerNetRouteTiles(net, *route);
            }
        }
    }
}

void registerExistingNetRoutes(rtl::Design& design)
{
    for (auto& module : design.modules) {
        registerExistingNetRoutes(module);
    }
}

std::string routeTreeDebugFilter()
{
    const char* filter = std::getenv("SCALEPNR_ROUTE_TREE_DEBUG_NET");
    return filter ? std::string(filter) : std::string{};
}

bool routeTreeDebugMatches(const rtl::Net& net, const std::string& filter)
{
    if (filter.empty()) {
        return true;
    }
    if (net.name.find(filter) != std::string::npos) {
        return true;
    }
    for (const rtl::NetRouteBinding& binding : net.routes) {
        if (binding.route_name.find(filter) != std::string::npos) {
            return true;
        }
    }
    return false;
}

std::string routeNodeId(const Coord& coord, const std::string& wire_name,
                        const char* fallback_type, int fallback_node)
{
    if (!wire_name.empty()) {
        return std::format("({},{})/{}", coord.x, coord.y, wire_name);
    }
    return std::format("({},{})/{}{}", coord.x, coord.y, fallback_type, fallback_node);
}

std::vector<std::string> routeTreeNodes(const std::vector<Wire>& route)
{
    std::vector<std::string> nodes;
    auto push_unique = [&](std::string node) {
        if (node.empty()) {
            return;
        }
        if (nodes.empty() || nodes.back() != node) {
            nodes.push_back(std::move(node));
        }
    };

    for (const Wire& fragment : route) {
        if (fragment.type == Wire::WIRE_TILE_PIN) {
            push_unique(routeNodeId(fragment.from, fragment.src_wire_name, "L", fragment.local));
            continue;
        }
        if (fragment.type != Wire::WIRE_CROSSBAR) {
            continue;
        }
        push_unique(routeNodeId(fragment.from, fragment.src_wire_name, "S", fragment.jump));
        push_unique(routeNodeId(fragment.to, fragment.dst_wire_name, "D", fragment.local));
    }
    return nodes;
}

std::vector<Wire>* routeBindingRoute(rtl::NetRouteBinding& binding)
{
    if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
        return nullptr;
    }
    return &binding.owner->wires[binding.route_index];
}

std::string routeTreeOwnerList(const std::vector<size_t>& owners)
{
    std::ostringstream out;
    out << '[';
    for (size_t i = 0; i < owners.size(); ++i) {
        if (i) {
            out << ',';
        }
        out << owners[i];
    }
    out << ']';
    return out.str();
}

void logMalformedRouteTrees(rtl::Design& design)
{
    const std::string filter = routeTreeDebugFilter();
    size_t malformed = 0;
    for (auto& module : design.modules) {
        for (auto& net : module.nets) {
            if (net.routes.empty() || !routeTreeDebugMatches(net, filter)) {
                continue;
            }

            std::unordered_map<std::string, std::vector<size_t>> node_routes;
            std::unordered_map<std::string, std::vector<std::string>> children;
            std::vector<std::vector<std::string>> route_nodes;
            std::vector<size_t> binding_indices;
            bool self_repeat = false;
            bool duplicate_non_root = false;
            bool duplicate_source_takeoff = false;

            for (size_t binding_index = 0; binding_index < net.routes.size(); ++binding_index) {
                rtl::NetRouteBinding& binding = net.routes[binding_index];
                std::vector<Wire>* route = routeBindingRoute(binding);
                if (!route || route->empty() || !routeIsComplete(*route)) {
                    continue;
                }
                std::vector<std::string> nodes = routeTreeNodes(*route);
                if (nodes.empty()) {
                    continue;
                }
                std::unordered_set<std::string> seen_in_route;
                for (const std::string& node : nodes) {
                    if (!seen_in_route.insert(node).second) {
                        self_repeat = true;
                    }
                    node_routes[node].push_back(binding_index);
                }
                for (size_t i = 0; i + 1 < nodes.size(); ++i) {
                    auto& outs = children[nodes[i]];
                    if (std::find(outs.begin(), outs.end(), nodes[i + 1]) == outs.end()) {
                        outs.push_back(nodes[i + 1]);
                    }
                }
                binding_indices.push_back(binding_index);
                route_nodes.push_back(std::move(nodes));
            }

            if (route_nodes.size() < 2 && !self_repeat) {
                continue;
            }

            std::string root = route_nodes.empty() || route_nodes[0].empty() ? std::string{} : route_nodes[0][0];
            for (const auto& [node, owners] : node_routes) {
                if (node != root && owners.size() > 1) {
                    duplicate_non_root = true;
                    break;
                }
            }
            if (!root.empty()) {
                auto child_it = children.find(root);
                duplicate_source_takeoff = child_it != children.end() && child_it->second.size() > 1;
            }

            if (!self_repeat && !duplicate_non_root && !duplicate_source_takeoff) {
                continue;
            }

            ++malformed;
            PNR_LOG1("ROUT", "routeTree malformed: net='{}', routes={}, root='{}', self_repeat={}, duplicate_non_root={}, duplicate_source_takeoff={}",
                net.makeName(FULL_NAME_LIMIT), route_nodes.size(), root,
                self_repeat, duplicate_non_root, duplicate_source_takeoff);
            for (size_t i = 0; i < route_nodes.size(); ++i) {
                rtl::NetRouteBinding& binding = net.routes[binding_indices[i]];
                PNR_LOG1("ROUT", "routeTree route: net='{}', binding={}, route_name='{}', from='{}' port='{}', to='{}' port='{}', nodes={}, owner='{}', route_index={}",
                    net.makeName(FULL_NAME_LIMIT), binding_indices[i], binding.route_name,
                    binding.from ? binding.from->makeName(FULL_NAME_LIMIT) : std::string{},
                    binding.from_port,
                    binding.to ? binding.to->makeName(FULL_NAME_LIMIT) : std::string{},
                    binding.to_port,
                    route_nodes[i].size(),
                    binding.owner ? binding.owner->makeName(FULL_NAME_LIMIT) : std::string{},
                    binding.route_index);
                if (route_nodes[i].size() >= 2) {
                    PNR_LOG1("ROUT", "routeTree route ends: binding={}, first='{}', second='{}', last='{}'",
                        binding_indices[i], route_nodes[i][0], route_nodes[i][1], route_nodes[i].back());
                }
            }
            for (const auto& [node, owners] : node_routes) {
                if (node == root || owners.size() <= 1) {
                    continue;
                }
                PNR_LOG1("ROUT", "routeTree duplicate node: net='{}', node='{}', bindings={}",
                    net.makeName(FULL_NAME_LIMIT), node, routeTreeOwnerList(owners));
            }
            if (!root.empty()) {
                auto child_it = children.find(root);
                if (child_it != children.end() && child_it->second.size() > 1) {
                    PNR_LOG1("ROUT", "routeTree source takeoff: net='{}', root='{}', children={}",
                        net.makeName(FULL_NAME_LIMIT), root, child_it->second.size());
                    for (const std::string& child : child_it->second) {
                        PNR_LOG1("ROUT", "routeTree source child: net='{}', root='{}', child='{}'",
                            net.makeName(FULL_NAME_LIMIT), root, child);
                    }
                }
            }
        }
    }
    if (malformed || !filter.empty()) {
        PNR_LOG1("ROUT", "routeTree diagnostics: malformed={}, filter='{}'", malformed, filter);
    }
}

std::string debugTileName(const Tile* tile)
{
    if (!tile) {
        return {};
    }
    if (!tile->full_name.empty()) {
        return tile->full_name;
    }
    if (tile->tile_type) {
        return std::format("{}_X{}Y{}", tile->tile_type->name, tile->name.x, tile->name.y);
    }
    if (tile->cb_type) {
        return std::format("{}_X{}Y{}", tile->cb_type->name, tile->name.x, tile->name.y);
    }
    return std::format("TILE_X{}Y{}", tile->name.x, tile->name.y);
}

std::string wireTypeName(Wire::Type type)
{
    switch (type) {
    case Wire::WIRE_CROSSBAR:
        return "crossbar";
    case Wire::WIRE_TILE_PIN:
        return "tile_pin";
    }
    return "unknown";
}

void exportPartialRouteState(std::list<Referable<RegBunch>>& bunch_list, const std::string& filename)
{
    (void)bunch_list;
    std::ofstream out(filename);
    if (!out) {
        return;
    }

    out << "SECTION: ROUTE_FRAGMENTS\n";
    out << "inst,net,route_index,frag_index,type,tile,coord_x,coord_y,from_x,from_y,to_x,to_y,local,pos,jump,dst_node,joint,src_wire,dst_wire,port\n";
    std::vector<rtl::Inst*> insts;
    collectInstsWithRoutes(technology::Tech::current().design.top, insts);
    for (rtl::Inst* inst : insts) {
        for (size_t route_index = 0; route_index < inst->wires.size(); ++route_index) {
            const auto& route = inst->wires[route_index];
            for (size_t frag_index = 0; frag_index < route.size(); ++frag_index) {
                const Wire& wire = route[frag_index];
                Tile* tile = fpga::Device::current().getTile(wire.from.x, wire.from.y);
                Tile* dst_tile = fpga::Device::current().getTile(wire.to.x, wire.to.y);
                int dst_node = dst_tile && dst_tile->cb_type && !wire.dst_wire_name.empty()
                    ? dst_tile->cb_type->nodeNum(fpga::CB_NODE_DST, wire.dst_wire_name)
                    : -1;
                out << csvField(inst->makeName(FULL_NAME_LIMIT)) << ","
                    << csvField(wire.net_name) << ","
                    << route_index << ","
                    << frag_index << ","
                    << wireTypeName(wire.type) << ","
                    << csvField(debugTileName(tile)) << ","
                    << wire.from.x << ","
                    << wire.from.y << ","
                    << wire.from.x << ","
                    << wire.from.y << ","
                    << wire.to.x << ","
                    << wire.to.y << ","
                    << wire.local << ","
                    << wire.pos << ","
                    << wire.jump << ","
                    << dst_node << ","
                    << wire.joint << ","
                    << csvField(wire.src_wire_name) << ","
                    << csvField(wire.dst_wire_name) << ","
                    << csvField(wire.port) << "\n";
            }
        }
    }

    out << "\nSECTION: TILE_LEASES\n";
    out << "tile,coord_x,coord_y,src_jump,dst_jump,src_deadend,dst_deadend,local,joint,pin\n";
    for (auto& tile_ref : fpga::Device::current().tile_grid) {
        Tile& tile = tile_ref;
        if (tile.cb.src.jump == u256{} && tile.cb.dst.jump == u256{}
            && tile.cb.src_deadend.jump == u256{}
            && tile.cb.dst_deadend.jump == u256{}
            && tile.cb.local.local == u256{} && tile.cb.joint.jump == u256{}
            && tile.pin_state.leased_nodes == u256{}) {
            continue;
        }
        out << csvField(debugTileName(&tile)) << ","
            << tile.coord.x << ","
            << tile.coord.y << ","
            << maskString(tile.cb.src.jump) << ","
            << maskString(tile.cb.dst.jump) << ","
            << maskString(tile.cb.src_deadend.jump) << ","
            << maskString(tile.cb.dst_deadend.jump) << ","
            << maskString(tile.cb.local.local) << ","
            << maskString(tile.cb.joint.jump) << ","
            << maskString(tile.pin_state.leased_nodes) << "\n";
    }
}

void rebuildRoutingState(std::list<Referable<RegBunch>>& bunch_list)
{
    resetRoutingState();
    std::vector<std::vector<Wire>*> routes;
    for (auto& bunch : bunch_list) {
        collectBunchRoutes(bunch, routes);
    }
    for (auto* route : routes) {
        if (!replayRouteLeases(*route)) {
            route->clear();
        }
    }
    registerExistingNetRoutes(technology::Tech::current().design);
}

uint64_t tileDeadendKey(const Coord& coord)
{
    return (static_cast<uint64_t>(static_cast<uint16_t>(coord.x)) << 48)
        | (static_cast<uint64_t>(static_cast<uint16_t>(coord.y)) << 32);
}

void applyRouteDeadends(const std::unordered_map<uint64_t, u256>& dst_deadends,
                        const std::unordered_map<uint64_t, u256>& src_deadends)
{
    for (auto& tile_ref : fpga::Device::current().tile_grid) {
        Tile& tile = tile_ref;
        auto dst_it = dst_deadends.find(tileDeadendKey(tile.coord));
        if (dst_it != dst_deadends.end()) {
            tile.cb.dst_deadend.jump |= dst_it->second;
        }
        auto src_it = src_deadends.find(tileDeadendKey(tile.coord));
        if (src_it != src_deadends.end()) {
            tile.cb.src_deadend.jump |= src_it->second;
        }
    }
}

void writeRouteTaskDebugHeader(std::ofstream& out)
{
    out << "pass,task_index,net,from_inst,from_type,from_port,to_inst,to_type,to_port,attempt,"
        << "from_coord_x,from_coord_y,from_pos,to_coord_x,to_coord_y,to_pos,from_output_nodes,to_input_nodes,"
        << "route_size_before,route_xbars_before,route_complete_before,"
        << "route_size_after,route_xbars_after,route_complete_after,"
        << "task_complete,task_progress,task_changed,recursions_used,"
        << "tasks,new,cont,done,partial_start,partial_adv,rip,backtry,backok,backfrag,rollback,"
        << "failed,no_src,no_src_depth0,no_src_joint_path,searches,pops,edge_trials,edge_ok,"
        << "reject_name,reject_busy,reject_busy_dst,reject_busy_src,reject_busy_local,reject_target,"
        << "reject_deadend,reject_dst_deadend,reject_src_deadend,"
        << "last_busy_coord_x,last_busy_coord_y,last_busy_depth,last_busy_local,last_busy_src,"
        << "last_no_src_coord_x,last_no_src_coord_y,last_no_src_depth,last_no_src_local,last_no_src_joint_mask,"
        << "last_deadend_net,last_deadend_dst_x,last_deadend_dst_y,last_deadend_dst_node,"
        << "last_deadend_src_x,last_deadend_src_y,last_deadend_src_node\n";
}

void writeRouteTaskDebugRow(std::ofstream& out, int pass, size_t task_index,
                            const RouteDesign::RouteTask& task,
                            const RouteDesign::RouteStats& before_stats,
                            const RouteDesign::RouteStats& after_stats,
                            size_t route_size_before, size_t route_xbars_before,
                            bool route_complete_before,
                            size_t route_size_after, size_t route_xbars_after,
                            bool route_complete_after,
                            bool task_complete, bool task_progress, bool task_changed,
                            int recursions_used)
{
    auto delta = [](size_t after, size_t before) {
        return after >= before ? after - before : 0;
    };
    auto coord_or_empty = [](bool valid, int value) -> std::string {
        return valid ? std::to_string(value) : std::string{};
    };

    out << pass << ","
        << task_index << ","
        << csvField(task.net_name) << ","
        << csvField(task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{}) << ","
        << csvField(task.from && task.from->cell_ref.peer ? task.from->cell_ref->type : std::string{}) << ","
        << csvField(task.from_port) << ","
        << csvField(task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{}) << ","
        << csvField(task.to && task.to->cell_ref.peer ? task.to->cell_ref->type : std::string{}) << ","
        << csvField(task.to_port) << ","
        << task.attempt << ","
        << (task.from && task.from->tile.peer ? std::to_string(task.from->tile->coord.x) : std::string{}) << ","
        << (task.from && task.from->tile.peer ? std::to_string(task.from->tile->coord.y) : std::string{}) << ","
        << (task.from ? std::to_string(task.from->pos) : std::string{}) << ","
        << (task.to && task.to->tile.peer ? std::to_string(task.to->tile->coord.x) : std::string{}) << ","
        << (task.to && task.to->tile.peer ? std::to_string(task.to->tile->coord.y) : std::string{}) << ","
        << (task.to ? std::to_string(task.to->pos) : std::string{}) << ","
        << (task.from && task.from->tile.peer && task.from->cell_ref.peer
                ? maskString(task.from->tile->getOutputPinNodes(task.from->cell_ref->type, task.from_port, task.from->pos))
                : std::string{}) << ","
        << (task.to && task.to->tile.peer && task.to->cell_ref.peer
                ? maskString(task.to->tile->getPinNodes(task.to->cell_ref->type, task.to_port, task.to->pos))
                : std::string{}) << ","
        << route_size_before << ","
        << route_xbars_before << ","
        << (route_complete_before ? 1 : 0) << ","
        << route_size_after << ","
        << route_xbars_after << ","
        << (route_complete_after ? 1 : 0) << ","
        << (task_complete ? 1 : 0) << ","
        << (task_progress ? 1 : 0) << ","
        << (task_changed ? 1 : 0) << ","
        << recursions_used << ","
        << delta(after_stats.task_attempts, before_stats.task_attempts) << ","
        << delta(after_stats.new_attempts, before_stats.new_attempts) << ","
        << delta(after_stats.continuation_attempts, before_stats.continuation_attempts) << ","
        << delta(after_stats.completed, before_stats.completed) << ","
        << delta(after_stats.partial_started, before_stats.partial_started) << ","
        << delta(after_stats.partial_advanced, before_stats.partial_advanced) << ","
        << delta(after_stats.rip_backs, before_stats.rip_backs) << ","
        << delta(after_stats.backstep_attempts, before_stats.backstep_attempts) << ","
        << delta(after_stats.backstep_success, before_stats.backstep_success) << ","
        << delta(after_stats.backstep_fragments, before_stats.backstep_fragments) << ","
        << delta(after_stats.commit_rollbacks, before_stats.commit_rollbacks) << ","
        << delta(after_stats.failed, before_stats.failed) << ","
        << delta(after_stats.no_src_nodes, before_stats.no_src_nodes) << ","
        << delta(after_stats.no_src_nodes_depth0, before_stats.no_src_nodes_depth0) << ","
        << delta(after_stats.no_src_nodes_with_joint_path, before_stats.no_src_nodes_with_joint_path) << ","
        << delta(after_stats.route_searches, before_stats.route_searches) << ","
        << delta(after_stats.search_pops, before_stats.search_pops) << ","
        << delta(after_stats.edge_trials, before_stats.edge_trials) << ","
        << delta(after_stats.edge_accepted, before_stats.edge_accepted) << ","
        << delta(after_stats.edge_rejected_no_name, before_stats.edge_rejected_no_name) << ","
        << delta(after_stats.edge_rejected_busy, before_stats.edge_rejected_busy) << ","
        << delta(after_stats.edge_rejected_busy_dst, before_stats.edge_rejected_busy_dst) << ","
        << delta(after_stats.edge_rejected_busy_src, before_stats.edge_rejected_busy_src) << ","
        << delta(after_stats.edge_rejected_busy_local, before_stats.edge_rejected_busy_local) << ","
        << delta(after_stats.edge_rejected_no_target, before_stats.edge_rejected_no_target) << ","
        << delta(after_stats.edge_rejected_deadend, before_stats.edge_rejected_deadend) << ","
        << delta(after_stats.edge_rejected_dst_deadend, before_stats.edge_rejected_dst_deadend) << ","
        << delta(after_stats.edge_rejected_src_deadend, before_stats.edge_rejected_src_deadend) << ","
        << coord_or_empty(after_stats.has_last_busy, after_stats.last_busy_coord.x) << ","
        << coord_or_empty(after_stats.has_last_busy, after_stats.last_busy_coord.y) << ","
        << (after_stats.has_last_busy ? std::to_string(after_stats.last_busy_depth) : std::string{}) << ","
        << (after_stats.has_last_busy ? std::to_string(after_stats.last_busy_local) : std::string{}) << ","
        << (after_stats.has_last_busy ? std::to_string(after_stats.last_busy_src) : std::string{}) << ","
        << coord_or_empty(after_stats.has_last_no_src, after_stats.last_no_src_coord.x) << ","
        << coord_or_empty(after_stats.has_last_no_src, after_stats.last_no_src_coord.y) << ","
        << (after_stats.has_last_no_src ? std::to_string(after_stats.last_no_src_depth) : std::string{}) << ","
        << (after_stats.has_last_no_src ? std::to_string(after_stats.last_no_src_local) : std::string{}) << ","
        << (after_stats.has_last_no_src ? maskString(after_stats.last_no_src_joint_mask) : std::string{}) << ","
        << csvField(after_stats.has_last_deadend_mark ? after_stats.last_deadend_net : std::string{}) << ","
        << coord_or_empty(after_stats.has_last_deadend_mark, after_stats.last_dst_deadend_coord.x) << ","
        << coord_or_empty(after_stats.has_last_deadend_mark, after_stats.last_dst_deadend_coord.y) << ","
        << (after_stats.has_last_deadend_mark ? std::to_string(after_stats.last_dst_deadend_node) : std::string{}) << ","
        << coord_or_empty(after_stats.has_last_deadend_mark, after_stats.last_src_deadend_coord.x) << ","
        << coord_or_empty(after_stats.has_last_deadend_mark, after_stats.last_src_deadend_coord.y) << ","
        << (after_stats.has_last_deadend_mark ? std::to_string(after_stats.last_src_deadend_node) : std::string{})
        << "\n";
}

}

bool RouteDesign::tryNext(Tile& from, Tile& to, int from_pos, int to_pos, const std::string& to_port, std::vector<Wire>& wire, int depth, rtl::Inst* dst_inst_override)
{
    if (depth >= iteration_limit || route_iteration_budget <= 0) {
        return false;
    }
    --route_iteration_budget;

    wire.resize(depth+1);
    wire[depth].from = from.coord;
    wire[depth].to = to.coord;
    wire[depth].local = from_pos;

    if (from.coord == to.coord) {
        PNR_ASSERT(from.cb_type, "cb_type is NULL in tile '{}' at ({},{}) type {}\n", from.makeName(), from.coord.x, from.coord.y, (int)from.type);

        rtl::Inst* dst_inst = dst_inst_override ? dst_inst_override : tileInstAtPos(to, to_pos);
        if (!dst_inst) {
            PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, no destination inst at pos",
                from.coord, to.coord, from_pos, to_pos);
            return false;
        }

        u256 pin_nodes = to.getPinNodes(dst_inst->cell_ref->type, to_port, dst_inst->pos);
        if (pin_nodes == u256{}) {
            PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, port: {}, no tile pin nodes",
                from.coord, to.coord, from_pos, to_pos, to_port);
            return false;
        }

        bool routed = pin_nodes.for_each_set_bit([&](int local) {
            int joint = -1;
            if (to.isPinNodeLeased(local) || !from.cb_type->canIn(from_pos, local, joint)) {
                return false;
            }
            CBState prev_cb = to.cb;
            TilePinState prev_pin_state = to.pin_state;
            if (!to.cb.leaseIn(from_pos, local, joint)) {
                return false;
            }
            if (!to.leasePinNode(local)) {
                to.cb = prev_cb;
                to.pin_state = prev_pin_state;
                return false;
            }
            wire[depth].local = from_pos;
            wire[depth].joint = joint;
            wire.resize(depth+2);
            fillTilePinEndpoint(wire[depth+1], *dst_inst, to_port, fpga::TILE_PIN_INPUT);
            wire[depth+1].from = to.coord;
            wire[depth+1].to = to.coord;
            wire[depth+1].local = local;
            refreshTilePinEndpoint(wire[depth+1], *dst_inst, to_port, fpga::TILE_PIN_INPUT);
            return true;
        });

        PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, routed: {}", from.coord, to.coord, from_pos, to_pos, routed);
        return routed;
    }

    int curr = -1;
    int orig_curr = -1;
    while ((curr = from.cb.iterate(depth != 0, from_pos, from.coord, to.coord, curr)) >= 0)
    {
        if (orig_curr == -1) {
            orig_curr = curr;
        }
        int joint;
        if ((depth != 0 ? from.cb_type->canJump(from_pos, curr, orig_curr, joint) : from.cb_type->canOut(from_pos, curr, orig_curr, joint))) {
            PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, curr: {}, can", from.coord, to.coord, from_pos, to_pos, curr);
            int actual_curr = actualJumpId(curr, orig_curr);
            fpga::CBNodeNameType from_type = depth == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
            if (!hasConcreteCrossbarPath(from, from_type, from_pos, fpga::CB_NODE_SRC, actual_curr, joint)) {
                PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, curr: {}, no concrete connection", from.coord, to.coord, from_pos, to_pos, curr);
                continue;
            }
            std::string incoming_wire = depth > 0 ? wire[depth - 1].dst_wire_name : std::string{};
            std::string src_wire = concreteSrcWireName(from, from_type, from_pos, actual_curr, joint, incoming_wire);
            if (src_wire.empty()) {
                continue;
            }

            CBState prev_cb = from.cb;
            if (depth != 0) {
                if (!from.cb.leaseJump(from_pos, curr, orig_curr)) {
                    PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, curr: {}, jump busy", from.coord, to.coord, from_pos, to_pos, curr);
                    continue;
                }
            }
            else {
                if (!from.cb.leaseOut(from_pos, curr, orig_curr)) {
                    PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, curr: {}, out busy", from.coord, to.coord, from_pos, to_pos, curr);
                    continue;
                }
            }

            fpga::TileJumpTarget target = fpga::Device::current().resolveJump(from, actual_curr, src_wire);
            Coord next;
            int next_pos = actual_curr;
            Tile* from1 = nullptr;
            if (target.tile && target.tile->cb_type && target.dst_node >= 0) {
                next = target.tile->coord;
                next_pos = target.dst_node;
                from1 = target.tile;
            }
            else {
                if (!fpga::Device::current().tile_conn_rules.empty()) {
                    from.cb = prev_cb;
                    continue;
                }
                next = from.cb.makeJump(from.coord, curr, orig_curr);
                from1 = fpga->getTile(next.x, next.y);
                if (!from1) {
                    from.cb = prev_cb;
                    continue;
                }
            }
            wire[depth].to = next;
            wire[depth].jump = actual_curr;
            wire[depth].joint = joint;
            wire[depth].pos = depth == 0 ? 0 : 1;
            wire[depth].src_wire_name = src_wire;
            wire[depth].dst_wire_name = target.dst_wire;

            if (tryNext(*from1, to, next_pos, to_pos, to_port, wire, depth+1, dst_inst_override)) {
                return true;
            }
            from.cb = prev_cb;
        }
    }

    return false;
}

bool RouteDesign::routeNet(rtl::Inst& from, const std::string& from_port, rtl::Inst& to, const std::string& to_port, std::vector<Wire>& wire, bool& complete, size_t attempt, rtl::Net* net)
{
//    PNR_ASSERT(!from.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not assigned", from.makeName())
//    PNR_ASSERT(!to.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not assigned", to.makeName())
    complete = true;
    if (!from.tile.peer || !to.tile.peer) {
        return true;
    }
    std::vector<Tile*> from_route_tiles = routeTileCandidates(from, from_port, true);
    std::vector<Tile*> to_route_tiles = routeTileCandidates(to, to_port, false);
    if (!from_route_tiles.empty()) {
        std::rotate(from_route_tiles.begin(), from_route_tiles.begin() + (attempt % from_route_tiles.size()), from_route_tiles.end());
    }
    if (!to_route_tiles.empty()) {
        std::rotate(to_route_tiles.begin(), to_route_tiles.begin() + ((attempt / std::max<size_t>(1, from_route_tiles.size())) % to_route_tiles.size()), to_route_tiles.end());
    }
    u256 output_nodes = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
    if (from.tile.peer == to.tile.peer) {
        // Packed same-tile resources should be connected locally before trying global routing.
        if (tryDirectResourceRoute(from, from_port, to, to_port, wire, net)) {
            complete = true;
            return true;
        }
    }

    auto check_output_local = [&](Tile& route_tile, int local, bool assert_on_invalid) {
        if (isRoutableOutputLocal(route_tile, local)) {
            return true;
        }
        if (assert_on_invalid) {
            const std::string* name = route_tile.cb_type ? route_tile.cb_type->nodeName(fpga::CB_NODE_LOCAL, local) : nullptr;
            PNR_ASSERT(false,
                "routeNet tried to start net '{}' from non-endpoint or non-routable output local node {}{}{} in tile '{}' at ({},{}); route starts must use the mapped local output node for that source port",
                net ? net->makeName() : std::string{},
                local,
                name ? " '" : "",
                name ? *name + "'" : std::string{},
                route_tile.makeName(), route_tile.coord.x, route_tile.coord.y);
        }
        return false;
    };

    auto try_output_nodes = [&](u256 nodes, bool assert_on_invalid) {
        for (Tile* from_route_tile : from_route_tiles) {
            if (!from_route_tile) {
                continue;
            }
            u256 route_nodes = supportsOutputLocalNodes(*from_route_tile, nodes)
                ? nodes
                : from_route_tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
            bool routed = route_nodes.for_each_set_bit([&](int local) {
                for (Tile* to_route_tile : to_route_tiles) {
                    if (route_iteration_budget <= 0) {
                        return false;
                    }
                    wire.clear();
                    bool attempt_complete = false;
                    if (!to_route_tile || !check_output_local(*from_route_tile, local, assert_on_invalid)) {
                        continue;
                    }
                    if (sourceLocalOwnedByDifferentEndpoint(*from_route_tile, local, from, from_port, net)) {
                        continue;
                    }
                    u256 pin_nodes = routeTileInputNodes(*to_route_tile, to, to_port);
                    --route_iteration_budget;
                    if (tryBestFirstRoute(*from_route_tile, *to_route_tile, local, to, to_port, wire, iteration_limit, false, {}, &attempt_complete, &route_stats, this, net, false, &from, from_port, pin_nodes)) {
                        prependSourceEndpoint(wire, from, from_port);
                        complete = attempt_complete;
                        return true;
                    }
                }
                return false;
            });
            if (routed) {
                return true;
            }
        }
        return false;
    };

    bool routed = try_output_nodes(output_nodes, false);
    if (!routed && !anyRoutableOutputCandidate(from_route_tiles, output_nodes)) {
        routed = tryDirectResourceRoute(from, from_port, to, to_port, wire, net);
        if (routed) {
            complete = true;
        }
    }
    if (!routed) {
        auto try_backtracking = [&](u256 nodes, bool assert_on_invalid) {
            route_iteration_budget = std::min(iteration_limit, 1);
            for (Tile* from_route_tile : from_route_tiles) {
                if (!from_route_tile) {
                    continue;
                }
                u256 route_nodes = supportsOutputLocalNodes(*from_route_tile, nodes)
                    ? nodes
                    : from_route_tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
                bool routed_backtrack = route_nodes.for_each_set_bit([&](int local) {
                    for (Tile* to_route_tile : to_route_tiles) {
                        if (route_iteration_budget <= 0) {
                            return false;
                        }
                        if (!to_route_tile) {
                            continue;
                        }
                        if (!check_output_local(*from_route_tile, local, assert_on_invalid)) {
                            continue;
                        }
                        if (sourceLocalOwnedByDifferentEndpoint(*from_route_tile, local, from, from_port, net)) {
                            continue;
                        }
                        --route_iteration_budget;
                        wire.clear();
                        if (tryNext(*from_route_tile, *to_route_tile, local, to.pos, to_port, wire, 0, &to)) {
                            prependSourceEndpoint(wire, from, from_port);
                            return true;
                        }
                    }
                    return false;
                });
                if (routed_backtrack) {
                    return true;
                }
            }
            return false;
        };
        routed = try_backtracking(output_nodes, false);
    }
    return routed;
}

bool RouteDesign::routeNet(rtl::Inst& from, rtl::Inst& to, const std::string& to_port, std::vector<Wire>& wire)
{
    bool complete = true;
    return routeNet(from, std::string(), to, to_port, wire, complete) && complete;
}

bool RouteDesign::routeNet(rtl::Inst& from, rtl::Inst& to, std::vector<Wire>& wire)
{
    bool complete = true;
    return routeNet(from, std::string(), to, std::string(), wire, complete) && complete;
}


bool RouteDesign::routeFanoutTask(RouteTask& task, int depth)
{
    PNR_ASSERT(task.from && task.to, "routeFanoutTask got null endpoint for net '{}'", task.net_name);
    if (!task.net) {
        return routeNetTask(task, depth);
    }

    ++route_stats.task_attempts;
    PNR_LOG3_("ROUT", depth, "routeFanoutTask, net: '{}', from: '{}' port '{}', to: '{}' port '{}'",
        task.net_name, task.from->makeName(), task.from_port, task.to->makeName(), task.to_port);

    std::vector<Wire>* existing_route = findRoute(*task.to, task.net_name);
    if (existing_route && routeIsComplete(*existing_route)) {
        ++route_stats.already_complete;
        return true;
    }
    if (route_recursion_budget <= 0) {
        return false;
    }

    bool route_complete = false;
    if (existing_route && !existing_route->empty()) {
        ++route_stats.continuation_attempts;
        size_t before_size = existing_route->size();
        route_iteration_budget = iteration_limit;
        constexpr size_t max_fanout_partial_xbars = 96;
        bool partial_within_limit = routeCrossbarFragments(existing_route) <= max_fanout_partial_xbars;
        if (partial_within_limit && continuePartialRoute(*existing_route, *task.to, task.to_port, iteration_limit,
                route_complete, &route_stats, this, task.net, task.from, task.from_port)) {
            for (Wire& fragment : *existing_route) {
                fragment.net_name = task.net_name;
            }
            size_t route_index = findRouteIndex(*task.to, existing_route);
            if (route_index != std::numeric_limits<size_t>::max()) {
                fpga::attachNetRoute(*task.net, *task.to, route_index, task.from, task.to,
                    task.from_port, task.to_port, task.net_name);
                fpga::registerNetRouteTiles(*task.net, *existing_route);
            }
            route_changed = true;
            route_progress = route_progress || route_complete || existing_route->size() > before_size;
            if (route_complete) {
                ++route_stats.completed;
                ++route_stats.cont_completed;
            }
            else if (existing_route->size() > before_size) {
                ++route_stats.partial_advanced;
                ++route_stats.cont_advanced;
            }
            else {
                ++route_stats.cont_no_advance;
            }
            --route_recursion_budget;
            return route_complete;
        }
        for (auto fragment = existing_route->rbegin(); fragment != existing_route->rend(); ++fragment) {
            if (fragment->type != Wire::WIRE_CROSSBAR || fragment->jump < 0) {
                continue;
            }
            Tile* src_tile = fpga::Device::current().getTile(fragment->from.x, fragment->from.y);
            if (src_tile) {
                u256 src_deadend_bit = u256{0,1} << fragment->jump;
                bool was_marked = (src_tile->cb.src_deadend.jump & src_deadend_bit) != u256{};
                src_tile->cb.src_deadend.jump |= src_deadend_bit;
                route_src_deadends[tileDeadendKey(src_tile->coord)] |= src_deadend_bit;
                if (!was_marked) {
                    ++route_stats.src_deadend_marks;
                }
                route_stats.has_last_deadend_mark = true;
                route_stats.last_deadend_net = task.net_name;
                route_stats.last_src_deadend_coord = src_tile->coord;
                route_stats.last_src_deadend_node = fragment->jump;
                route_stats.last_dst_deadend_coord = fragment->to;
                route_stats.last_dst_deadend_node = fragment->local;
                ++route_stats.deadends_by_depth[statDepthBucket(static_cast<int>(existing_route->size()))];
                PNR_LOG3("ROUT", "routeDesign fanout rollback deadend: net='{}', src=({},{}):{}, to=({},{}), local={}",
                    task.net_name, src_tile->coord.x, src_tile->coord.y, fragment->jump,
                    fragment->to.x, fragment->to.y, fragment->local);
            }
            break;
        }
        if (ripLastRouteStep(*existing_route, &route_stats)) {
            route_changed = true;
            ++route_stats.rip_backs;
            ++route_stats.cont_failed_rip;
        }
        else {
            ++route_stats.failed;
            ++route_stats.cont_failed_empty;
        }
        ++task.attempt;
        --route_recursion_budget;
        return false;
    }

    struct BranchPoint
    {
        Tile* tile = nullptr;
        int local = -1;
        std::string dst_wire;
        int score = 0;
        std::vector<Wire> shared_prefix;
    };
    route_iteration_budget = iteration_limit;

    std::vector<BranchPoint> branches;
    for (rtl::NetRouteBinding& binding : task.net->routes) {
        if (binding.route_name == task.net_name || !binding.owner || binding.route_index >= binding.owner->wires.size()) {
            continue;
        }
        std::vector<Wire>& base = binding.owner->wires[binding.route_index];
        if (!routeIsComplete(base)) {
            continue;
        }
        for (size_t fragment_index = 0; fragment_index < base.size(); ++fragment_index) {
            const Wire& fragment = base[fragment_index];
            if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0) {
                continue;
            }
            Tile* from_tile = fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
            if (!from_tile || !from_tile->cb_type) {
                continue;
            }
            fpga::TileJumpTarget target = fpga::Device::current().resolveJump(*from_tile, fragment.jump, fragment.src_wire_name);
            if (!target.tile || !target.tile->cb_type || target.dst_node < 0) {
                continue;
            }
            bool exists = std::any_of(branches.begin(), branches.end(), [&](const BranchPoint& point) {
                return point.tile == target.tile && point.local == target.dst_node && point.dst_wire == target.dst_wire;
            });
            if (exists) {
                continue;
            }
            std::vector<Wire> prefix(base.begin(), base.begin() + static_cast<std::ptrdiff_t>(fragment_index + 1));
            for (Wire& prefix_fragment : prefix) {
                prefix_fragment.shared = true;
            }
            branches.push_back(BranchPoint{
                target.tile,
                target.dst_node,
                target.dst_wire,
                routeDistance(target.tile->coord, task.to->tile->coord),
                std::move(prefix)
            });
        }
    }
    std::sort(branches.begin(), branches.end(), [](const BranchPoint& a, const BranchPoint& b) {
        return a.score < b.score;
    });

    std::vector<Tile*> to_route_tiles = routeTileCandidates(*task.to, task.to_port, false);
    ++route_stats.new_attempts;

    if (branches.empty()) {
        std::vector<Tile*> from_route_tiles = routeTileCandidates(*task.from, task.from_port, true);
        u256 output_nodes = task.from->tile.peer
            ? task.from->tile->getOutputPinNodes(task.from->cell_ref->type, task.from_port, task.from->pos)
            : u256{};
        std::vector<Wire> direct;
        if (!anyRoutableOutputCandidate(from_route_tiles, output_nodes)
            && tryDirectResourceRoute(*task.from, task.from_port, *task.to, task.to_port, direct, task.net)) {
            for (Wire& fragment : direct) {
                fragment.net_name = task.net_name;
            }
            task.to->wires.emplace_back(std::move(direct));
            fpga::attachNetRoute(*task.net, *task.to, task.to->wires.size() - 1, task.from, task.to,
                task.from_port, task.to_port, task.net_name);
            fpga::registerNetRouteTiles(*task.net, task.to->wires.back());
            route_changed = true;
            route_progress = true;
            ++route_stats.completed;
            ++route_stats.new_completed;
            --route_recursion_budget;
            return true;
        }
    }
    for (const BranchPoint& branch : branches) {
        for (Tile* to_route_tile : to_route_tiles) {
            if (route_iteration_budget <= 0) {
                ++task.attempt;
                --route_recursion_budget;
                return false;
            }
            --route_iteration_budget;
            if (!branch.tile || !to_route_tile) {
                continue;
            }
            std::vector<Wire> wire;
            bool attempt_complete = false;
            u256 pin_nodes = to_route_tile ? routeTileInputNodes(*to_route_tile, *task.to, task.to_port) : u256{};
            if (!tryBestFirstRoute(*branch.tile, *to_route_tile, branch.local, *task.to, task.to_port,
                    wire, iteration_limit, true, branch.dst_wire, &attempt_complete, &route_stats, this, task.net, true, task.from, task.from_port, pin_nodes)) {
                continue;
            }
            for (Wire& fragment : wire) {
                fragment.net_name = task.net_name;
            }
            if (wire.empty()) {
                continue;
            }
            if (!branch.shared_prefix.empty()) {
                std::vector<Wire> full_route;
                full_route.reserve(branch.shared_prefix.size() + wire.size());
                full_route.insert(full_route.end(), branch.shared_prefix.begin(), branch.shared_prefix.end());
                full_route.insert(full_route.end(), std::make_move_iterator(wire.begin()), std::make_move_iterator(wire.end()));
                wire = std::move(full_route);
            }
            task.to->wires.emplace_back(std::move(wire));
            fpga::attachNetRoute(*task.net, *task.to, task.to->wires.size() - 1, task.from, task.to,
                task.from_port, task.to_port, task.net_name);
            fpga::registerNetRouteTiles(*task.net, task.to->wires.back());
            route_changed = true;
            route_progress = true;
            route_complete = attempt_complete;
            if (route_complete) {
                ++route_stats.completed;
                ++route_stats.new_completed;
            }
            else {
                ++route_stats.partial_started;
                ++route_stats.new_partial;
            }
            --route_recursion_budget;
            return route_complete;
        }
    }

    if (branches.empty()) {
        std::vector<Wire> wire;
        bool fallback_complete = false;
        route_iteration_budget = iteration_limit;
        if (routeNet(*task.from, task.from_port, *task.to, task.to_port, wire, fallback_complete, task.attempt, task.net)) {
            for (Wire& fragment : wire) {
                fragment.net_name = task.net_name;
            }
            if (!wire.empty()) {
                task.to->wires.emplace_back(std::move(wire));
                fpga::attachNetRoute(*task.net, *task.to, task.to->wires.size() - 1, task.from, task.to,
                    task.from_port, task.to_port, task.net_name);
                fpga::registerNetRouteTiles(*task.net, task.to->wires.back());
                route_changed = true;
                route_progress = true;
                route_complete = fallback_complete;
                if (route_complete) {
                    ++route_stats.completed;
                    ++route_stats.new_completed;
                }
                else {
                    ++route_stats.partial_started;
                    ++route_stats.new_partial;
                }
                --route_recursion_budget;
                return route_complete;
            }
            ++route_stats.new_empty;
        }
    }

    ++route_stats.failed;
    ++route_stats.new_failed;
    ++task.attempt;
    --route_recursion_budget;
    return false;
}

bool RouteDesign::routeNetTask(RouteTask& task, int depth)
{
    PNR_ASSERT(task.from && task.to, "routeNetTask got null endpoint for net '{}'", task.net_name);
    if (task.fanout) {
        return routeFanoutTask(task, depth);
    }
    ++route_stats.task_attempts;
    PNR_LOG3_("ROUT", depth, "routeNetTask, net: '{}', from: '{}' port '{}', to: '{}' port '{}'",
        task.net_name, task.from->makeName(), task.from_port, task.to->makeName(), task.to_port);

    std::vector<Wire>* existing_route = findRoute(*task.to, task.net_name);
    if (existing_route && routeIsComplete(*existing_route)) {
        ++route_stats.already_complete;
        return true;
    }

    if (route_recursion_budget <= 0) {
        return false;
    }

    bool route_complete = false;
    if (existing_route && !existing_route->empty()) {
        ++route_stats.continuation_attempts;
        size_t before_size = existing_route->size();
        if (continuePartialRoute(*existing_route, *task.to, task.to_port, iteration_limit, route_complete, &route_stats, this, task.net, task.from, task.from_port)) {
            for (Wire& fragment : *existing_route) {
                fragment.net_name = task.net_name;
            }
            if (task.net) {
                size_t route_index = findRouteIndex(*task.to, existing_route);
                if (route_index != std::numeric_limits<size_t>::max()) {
                    fpga::attachNetRoute(*task.net, *task.to, route_index, task.from, task.to,
                        task.from_port, task.to_port, task.net_name);
                    fpga::registerNetRouteTiles(*task.net, *existing_route);
                }
            }
            route_changed = true;
            route_progress = route_progress || route_complete || existing_route->size() > before_size;
            if (route_complete) {
                ++route_stats.completed;
                ++route_stats.cont_completed;
            }
            else if (existing_route->size() > before_size) {
                ++route_stats.partial_advanced;
                ++route_stats.cont_advanced;
            }
            else {
                ++route_stats.cont_no_advance;
            }
            --route_recursion_budget;
            if (route_complete) {
                return true;
            }
            return false;
        }
        for (auto fragment = existing_route->rbegin(); fragment != existing_route->rend(); ++fragment) {
            if (fragment->type != Wire::WIRE_CROSSBAR || fragment->jump < 0) {
                continue;
            }
            Tile* src_tile = fpga::Device::current().getTile(fragment->from.x, fragment->from.y);
            if (src_tile) {
                u256 src_deadend_bit = u256{0,1} << fragment->jump;
                bool was_marked = (src_tile->cb.src_deadend.jump & src_deadend_bit) != u256{};
                src_tile->cb.src_deadend.jump |= src_deadend_bit;
                route_src_deadends[tileDeadendKey(src_tile->coord)] |= src_deadend_bit;
                if (!was_marked) {
                    ++route_stats.src_deadend_marks;
                }
                route_stats.has_last_deadend_mark = true;
                route_stats.last_deadend_net = task.net_name;
                route_stats.last_src_deadend_coord = src_tile->coord;
                route_stats.last_src_deadend_node = fragment->jump;
                route_stats.last_dst_deadend_coord = fragment->to;
                route_stats.last_dst_deadend_node = fragment->local;
                ++route_stats.deadends_by_depth[statDepthBucket(static_cast<int>(existing_route->size()))];
                PNR_LOG1("ROUT", "routeDesign rollback deadend: net='{}', src=({},{}):{}, to=({},{}), local={}",
                    task.net_name, src_tile->coord.x, src_tile->coord.y, fragment->jump,
                    fragment->to.x, fragment->to.y, fragment->local);
            }
            break;
        }
        if (ripLastRouteStep(*existing_route, &route_stats)) {
            route_changed = true;
            ++route_stats.rip_backs;
            ++route_stats.cont_failed_rip;
        }
        else {
            ++route_stats.failed;
            ++route_stats.cont_failed_empty;
        }
        ++task.attempt;
        --route_recursion_budget;
        return false;
    }

    ++route_stats.new_attempts;
    std::vector<Wire> wire;
    route_iteration_budget = iteration_limit;
    if (routeNet(*task.from, task.from_port, *task.to, task.to_port, wire, route_complete, task.attempt, task.net)) {
        for (Wire& fragment : wire) {
            fragment.net_name = task.net_name;
        }
        if (!wire.empty()) {
            task.to->wires.emplace_back(std::move(wire));
            if (task.net) {
                fpga::attachNetRoute(*task.net, *task.to, task.to->wires.size() - 1, task.from, task.to,
                    task.from_port, task.to_port, task.net_name);
                fpga::registerNetRouteTiles(*task.net, task.to->wires.back());
            }
            route_changed = true;
            route_progress = true;
            if (route_complete) {
                ++route_stats.completed;
                ++route_stats.new_completed;
            }
            else {
                ++route_stats.partial_started;
                ++route_stats.new_partial;
            }
        }
        else {
            ++route_stats.new_empty;
        }
        --route_recursion_budget;
        if (route_complete) {
            return true;
        }
        return false;
    }

    if ((task.attempt & 63U) == 0U) {
        PNR_LOG3("ROUT", "warning: failed limited route attempt for net '{}' from '{}' port '{}' to '{}' port '{}'",
            task.net_name, task.from->makeName(), task.from_port, task.to->makeName(), task.to_port);
    }
    ++route_stats.failed;
    ++route_stats.new_failed;
    ++task.attempt;
    --route_recursion_budget;
    return false;
}

bool RouteDesign::enqueueRouteTask(const RouteTask& task, std::vector<RouteTask>& queue)
{
    auto same_task = [&](const RouteTask& old) {
        return sameRouteTask(old, task);
    };
    if (std::any_of(route_todo.begin(), route_todo.end(), same_task)
        || std::any_of(pending_route_todo.begin(), pending_route_todo.end(), same_task)
        || std::any_of(fanout_route_todo.begin(), fanout_route_todo.end(), same_task)
        || std::any_of(queue.begin(), queue.end(), same_task)) {
        return false;
    }
    queue.push_back(task);
    return true;
}

void RouteDesign::requeueNet(rtl::Net& net, bool fanout)
{
    for (size_t route_index = 0; route_index < net.routes.size(); ++route_index) {
        const rtl::NetRouteBinding& binding = net.routes[route_index];
        if (!binding.from || !binding.to || binding.route_name.empty()) {
            continue;
        }
        enqueueRouteTask(RouteTask{
            binding.from,
            binding.to,
            &net,
            binding.from_port,
            binding.to_port,
            binding.route_name,
            0,
            fanout || netHasCompleteRouteExcept(net, route_index)
        }, pending_route_todo);
    }
}

bool RouteDesign::moveUnfinishedCell(const RouteTask& task, std::vector<RouteTask>* moved_tasks, const RouteTask* trigger_task)
{
    const RouteTask& route_task = trigger_task ? *trigger_task : task;
    rtl::Inst* inst = task.to;
    if (!inst || !inst->tile.peer || isIoBuffer(*inst) || inst->outline.fixed) {
        return false;
    }
    if (move_finished_insts.contains(reinterpret_cast<uintptr_t>(inst))) {
        return false;
    }

    Tile* old_tile = &*inst->tile;
    int old_pos = inst->pos;
    Coord old_coord = old_tile->coord;
    std::vector<uint64_t>& tried = move_tried_placements[reinterpret_cast<uintptr_t>(inst)];
    uint64_t old_key = placementKey(old_coord, old_pos);
    if (std::find(tried.begin(), tried.end(), old_key) == tried.end()) {
        tried.push_back(old_key);
    }
    if (static_cast<int>(tried.size()) >= move_attempt_limit) {
        PNR_LOG1("ROUT", "routeDesign moving: inst='{}' type='{}' exhausted {} tried placements, dropping focused unfinished route for DB export",
            inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type, tried.size());
        if (moved_tasks) {
            moved_tasks->clear();
            return true;
        }
        return false;
    }

    unplaceInst(*inst);

    auto placement_supports_route_task = [&]() {
        if (!inst->tile.peer || !inst->cell_ref.peer) {
            return false;
        }
        if (route_task.from == inst) {
            u256 output_nodes = inst->tile->getOutputPinNodes(inst->cell_ref->type, route_task.from_port, inst->pos);
            if (output_nodes == u256{}) {
                return false;
            }
            std::vector<Tile*> candidates = routeTileCandidates(*inst, route_task.from_port, true);
            if (!anyRoutableOutputCandidate(candidates, output_nodes)) {
                return false;
            }
        }
        if (route_task.to == inst) {
            u256 input_nodes = inst->tile->getPinNodes(inst->cell_ref->type, route_task.to_port, inst->pos);
            if (input_nodes == u256{}) {
                return false;
            }
            std::vector<Tile*> candidates = routeTileCandidates(*inst, route_task.to_port, false);
            if (candidates.empty()) {
                return false;
            }
        }
        return true;
    };

    constexpr int max_move_search_radius = 64;
    Coord coord = old_coord;
    int dir = 0;
    int steps = 1;
    int search_pos = 0;
    Tile* new_tile = nullptr;
    int new_pos = -1;
    for (int i = 0; i < max_move_search_radius; ++i) {
        radialSearch(coord, dir, steps, search_pos);
        Tile* tile = fpga::Device::current().getTile(coord.x, coord.y);
        if (!tile || tile == old_tile) {
            continue;
        }
        int placed_pos = tile->tryAdd(inst);
        if (placed_pos < 0) {
            continue;
        }
        if (placementWasTried(tried, tile->coord, placed_pos)) {
            unplaceInst(*inst);
            continue;
        }
        if (!placement_supports_route_task()) {
            unplaceInst(*inst);
            continue;
        }
        new_tile = tile;
        new_pos = placed_pos;
        tried.push_back(placementKey(tile->coord, placed_pos));
        break;
    }

    if (!new_tile) {
        restoreInstPlacement(*inst, *old_tile, old_pos);
        return false;
    }

    inst->coord = new_tile->coord;
    inst->pos = new_pos;
    inst->outline.x = (new_tile->coord.x + 0.25f * static_cast<float>(new_pos % 4)) / aspect_x;
    inst->outline.y = (new_tile->coord.y + 0.25f * static_cast<float>(new_pos / 4)) / aspect_y;

    bool only_trigger_task = moved_tasks && moving_focus_inst == inst;
    size_t unrouted = 0;
    rtl::Module* module = parentModule(*inst);
    if (module) {
        for (auto& net_ref : module->nets) {
            rtl::Net& net = net_ref;
            for (size_t route_index = 0; route_index < net.routes.size(); ++route_index) {
                rtl::NetRouteBinding binding = net.routes[route_index];
                if (binding.from != inst && binding.to != inst) {
                    continue;
                }
                RouteTask next_task{
                    binding.from,
                    binding.to,
                    &net,
                    binding.from_port,
                    binding.to_port,
                    binding.route_name,
                    0,
                    false
                };
                if (only_trigger_task && !sameRouteTask(route_task, next_task)) {
                    continue;
                }
                rtl::Inst* other = binding.from == inst ? binding.to : binding.from;
                if (other && move_finished_insts.contains(reinterpret_cast<uintptr_t>(other))) {
                    continue;
                }
                bool fanout = !sameRouteTask(route_task, next_task) && netHasCompleteRouteExcept(net, route_index);
                next_task.fanout = fanout;
                if (fpga::unrouteNetRoute(net, route_index)) {
                    ++unrouted;
                }
                if (binding.from && binding.to && !binding.route_name.empty()) {
                    if (moved_tasks) {
                        appendUniqueRouteTask(*moved_tasks, next_task);
                    }
                    else {
                        enqueueRouteTask(next_task, pending_route_todo);
                    }
                }
            }
        }
    }
    else if (task.net) {
        if (moved_tasks) {
            for (size_t route_index = 0; route_index < task.net->routes.size(); ++route_index) {
                const rtl::NetRouteBinding& binding = task.net->routes[route_index];
                if (!binding.from || !binding.to || binding.route_name.empty()) {
                    continue;
                }
                appendUniqueRouteTask(*moved_tasks, RouteTask{
                    binding.from,
                    binding.to,
                    task.net,
                    binding.from_port,
                    binding.to_port,
                    binding.route_name,
                    0,
                    task.fanout || netHasCompleteRouteExcept(*task.net, route_index)
                });
            }
        }
        else {
            requeueNet(*task.net, task.fanout);
        }
    }
    if (moved_tasks) {
        // Keep the triggering route alive when it has no existing NetRouteBinding yet.
        appendUniqueRouteTask(*moved_tasks, route_task);
    }

    route_dst_deadends.clear();
    route_src_deadends.clear();

    PNR_LOG1("ROUT", "routeDesign moving: inst='{}' type='{}' ({},{})/{} -> ({},{})/{}, tried={}, unrouted_routes={}",
        inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type, old_coord.x, old_coord.y, old_pos,
        new_tile->coord.x, new_tile->coord.y, new_pos, tried.size(), unrouted);
    return true;
}

void RouteDesign::collectRouteTasks(rtl::Inst& inst, RegBunch* bunch)
{
    if (inst.mark == travers_mark) {
        return;
    }

    inst.mark = travers_mark;

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {
            continue;
        }
        curr = curr->follow();
        if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {
            continue;
        }
        rtl::Net* net = findNetByDesignator(inst, conn.port_ref->designator);
        RouteTask task{
            curr->inst_ref.peer,
            &inst,
            net,
            curr->port_ref->makeName(),
            conn.port_ref->makeName(),
            conn.makeNetName(nullptr, FULL_NAME_LIMIT),
            0,
            false
        };
        if (curr->mark == source_route_mark) {
            task.fanout = true;
            fanout_route_todo.push_back(std::move(task));
        }
        else {
            curr->mark = source_route_mark;
            route_todo.push_back(std::move(task));
        }
        collectRouteTasks(*curr->inst_ref.peer, nullptr);
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            collectRouteTasks(*subbunch.reg, &subbunch);
        }
    }
}

bool RouteDesign::routeInstTask(rtl::Inst& inst, int depth)
{
    PNR_LOG2_("ROUT", depth, "routeInst, inst: '{}' ({}), x: {}, y: {}", inst.makeName(), inst.cell_ref->type,
        inst.coord.x, inst.coord.y);

    bool task_complete = true;
    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // clock ports
                //route clocks
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
            std::string net_name = conn.makeNetName(nullptr, FULL_NAME_LIMIT);

            std::vector<Wire>* existing_route = findRoute(inst, net_name);
            if (!hasRoutedNet(inst, net_name)) {
                task_complete = false;
                if (route_recursion_budget <= 0) {
                    return false;
                }

                bool route_complete = routeIsComplete(existing_route ? *existing_route : std::vector<Wire>{});
                while (!route_complete && route_recursion_budget > 0) {
                    if (existing_route && !routeIsComplete(*existing_route)) {
                        size_t before_size = existing_route->size();
                        if (continuePartialRoute(*existing_route, inst, conn.port_ref->makeName(), iteration_limit, route_complete, &route_stats, this, nullptr, peer, curr->port_ref->makeName())) {
                            for (Wire& fragment : *existing_route) {
                                fragment.net_name = net_name;
                            }
                            route_progress = route_progress || route_complete || existing_route->size() > before_size;
                            --route_recursion_budget;
                            continue;
                        }
                        --route_recursion_budget;
                        return false;
                    }

                    std::vector<Wire> wire;
                    route_iteration_budget = iteration_limit;
                    if (routeNet(*peer, curr->port_ref->makeName(), inst, conn.port_ref->makeName(), wire, route_complete)) {
                        if (wire.empty()) {
                            route_complete = false;
                        }
                        for (Wire& fragment : wire) {
                            fragment.net_name = net_name;
                        }
                        if (existing_route && !routeIsComplete(*existing_route) && !wire.empty()) {
                            *existing_route = std::move(wire);
                            route_progress = true;
                        }
                        else if (!wire.empty()) {
                            inst.wires.emplace_back(std::move(wire));
                            existing_route = &inst.wires.back();
                            route_progress = true;
                        }
                        --route_recursion_budget;
                    }
                    else {
                        PNR_LOG1("ROUT", "warning: failed limited route attempt for net '{}' from '{}' port '{}' to '{}' port '{}'",
                            net_name, peer->makeName(), curr->port_ref->makeName(), inst.makeName(), conn.port_ref->makeName());
                        --route_recursion_budget;
                    }
                }
                if (!route_complete) {
                    return false;
                }
                task_complete = true;
            }
        }
    }

    return task_complete;
}

void RouteDesign::routeDesign(std::list<Referable<RegBunch>>& bunch_list)
{
    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
    int design_cells = countDesignCells(bunch_list);
    if (design_cells <= 0) {
        design_cells = std::max(total_bunches, total_regs + total_comb);
    }
    iteration_limit = iterationLimitFromCells(design_cells);
    move_attempt_limit = moveAttemptLimitFromCells(design_cells);
    PNR_LOG1("ROUT", "routeDesign, cells: {}, iteration_limit: {}, move_attempt_limit: {}",
        design_cells, iteration_limit, move_attempt_limit);
//    combs_per_box = /*total_comb*/(float)fpga.cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga->size_width;
    fpga_height = fpga->size_height;

    resetRoutingState();
    route_dst_deadends.clear();
    route_src_deadends.clear();

    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;

    int route_recursion_limit = 5;
    int max_route_passes = std::max(1, design_cells * 20);
    route_todo.clear();
    pending_route_todo.clear();
    fanout_route_todo.clear();
    moving_deferred_todo.clear();
    fanout_stage = false;
    moving_stage = false;
    moving_focus_inst = nullptr;
    move_tried_placements.clear();
    move_finished_insts.clear();
    source_route_mark = rtl::Inst::genMark();
    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        PNR_ASSERT(bunch.reg, "zero reg in bunch with address {}", (uint64_t)&bunch);
        collectRouteTasks(*bunch.reg, &bunch);
    }
    PNR_LOG1("ROUT", "routeDesign tasks: basic={}, deferred_fanout={}", route_todo.size(), fanout_route_todo.size());

    auto route_start_time = std::chrono::steady_clock::now();
    int debug_pass = routeDebugPass();
    int stagnant_passes = 0;
    int moving_passes = 0;
    for (int pass = 0; pass < max_route_passes && !route_todo.empty(); ++pass) {
        auto epoch_start_time = std::chrono::steady_clock::now();
        auto rebuild_start_time = epoch_start_time;
        rebuildRoutingState(bunch_list);
        applyRouteDeadends(route_dst_deadends, route_src_deadends);
        auto rebuild_end_time = std::chrono::steady_clock::now();
        route_stats.clear();
        size_t before = route_todo.size();
        int completed_this_pass = 0;
        int active_this_pass = 0;
        int advanced_this_pass = 0;
        int changed_this_pass = 0;
        bool debug_this_pass = debug_pass > 0 && pass + 1 == debug_pass;
        std::ofstream debug_out;
        if (debug_this_pass) {
            std::string filename = std::format("route_pass_{}_debug.csv", pass + 1);
            debug_out.open(filename);
            if (debug_out) {
                debug_out << std::unitbuf;
                writeRouteTaskDebugHeader(debug_out);
                PNR_LOG1("ROUT", "routeDesign debug: writing per-task pass {} details to {}", pass + 1, filename);
            }
        }

        size_t task_index = 0;
        size_t attempted_this_pass = 0;
        size_t task_limit_this_pass = route_todo.size();
        if (fanout_stage) {
            task_limit_this_pass = std::min(route_todo.size(),
                static_cast<size_t>(std::max(64, design_cells / 4)));
        }
        for (auto it = route_todo.begin(); it != route_todo.end();) {
            if (attempted_this_pass >= task_limit_this_pass) {
                std::rotate(route_todo.begin(), it, route_todo.end());
                break;
            }
            ++attempted_this_pass;
            size_t current_task_index = task_index++;
            if (!it->from || !it->to) {
                it = route_todo.erase(it);
                continue;
            }
            std::vector<Wire>* existing_route_before = findRoute(*it->to, it->net_name);
            size_t route_size_before = existing_route_before ? existing_route_before->size() : 0;
            size_t route_xbars_before = routeCrossbarFragments(existing_route_before);
            bool route_complete_before = existing_route_before && routeIsComplete(*existing_route_before);
            RouteStats stats_before = route_stats;
            route_recursion_budget = route_recursion_limit;
            bool task_complete = false;
            bool task_progress = false;
            bool task_changed = false;
            while (route_recursion_budget > 0) {
                route_changed = false;
                route_progress = false;
                if (routeNetTask(*it)) {
                    task_complete = true;
                    task_progress = true;
                    task_changed = true;
                    break;
                }
                task_changed = task_changed || route_changed;
                if (!route_progress) {
                    break;
                }
                task_progress = true;
            }
            std::vector<Wire>* existing_route_after = findRoute(*it->to, it->net_name);
            size_t route_size_after = existing_route_after ? existing_route_after->size() : 0;
            size_t route_xbars_after = routeCrossbarFragments(existing_route_after);
            bool route_complete_after = existing_route_after && routeIsComplete(*existing_route_after);
            if (debug_out) {
                writeRouteTaskDebugRow(debug_out, pass + 1, current_task_index, *it,
                    stats_before, route_stats,
                    route_size_before, route_xbars_before, route_complete_before,
                    route_size_after, route_xbars_after, route_complete_after,
                    task_complete, task_progress, task_changed,
                    route_recursion_limit - route_recursion_budget);
            }
            if (task_complete) {
                ++completed_this_pass;
                ++active_this_pass;
                it = route_todo.erase(it);
                continue;
            }
            if (task_progress) {
                ++advanced_this_pass;
            }
            if (task_changed) {
                ++changed_this_pass;
            }
            if (task_changed || task_progress) {
                ++active_this_pass;
            }
            ++it;
        }
        if (!pending_route_todo.empty()) {
            route_todo.insert(route_todo.end(),
                std::make_move_iterator(pending_route_todo.begin()),
                std::make_move_iterator(pending_route_todo.end()));
            pending_route_todo.clear();
        }
        bool restored_moving_focus = false;
        if (moving_focus_inst && route_todo.empty()) {
            move_finished_insts.insert(reinterpret_cast<uintptr_t>(moving_focus_inst));
            PNR_LOG1("ROUT", "routeDesign moving: inst='{}' rerouted, restoring {} deferred tasks",
                moving_focus_inst->makeName(FULL_NAME_LIMIT), moving_deferred_todo.size());
            route_todo = std::move(moving_deferred_todo);
            moving_deferred_todo.clear();
            moving_focus_inst = nullptr;
            stagnant_passes = 0;
            moving_passes = 0;
            moving_stage = true;
            restored_moving_focus = true;
        }
        if (route_todo.size() >= before) {
            ++stagnant_passes;
        }
        else {
            stagnant_passes = 0;
        }
        bool route_blocked_this_pass = active_this_pass == 0;
        if (!fanout_stage && !fanout_route_todo.empty()
            && !moving_focus_inst && (route_blocked_this_pass || stagnant_passes >= 3)) {
            fanout_stage = true;
            route_todo.insert(route_todo.end(),
                std::make_move_iterator(fanout_route_todo.begin()),
                std::make_move_iterator(fanout_route_todo.end()));
            fanout_route_todo.clear();
            stagnant_passes = 0;
            route_blocked_this_pass = false;
            PNR_LOG1("ROUT", "routeDesign stage: Fanouts routing, tasks={}", route_todo.size());
        }
        bool should_move_unfocused = fanout_stage && !moving_focus_inst && (route_blocked_this_pass || stagnant_passes >= 3);
        bool should_move_focus = moving_focus_inst && (route_blocked_this_pass || stagnant_passes >= route_recursion_limit);
        if (!restored_moving_focus && !route_todo.empty() && route_todo.size() >= before
            && (should_move_unfocused || should_move_focus)) {
            moving_stage = true;
            ++moving_passes;
            PNR_ASSERT(moving_passes <= move_attempt_limit,
                "routeDesign moving stage did not converge after {} relocation epochs with {} unfinished route tasks",
                move_attempt_limit, route_todo.size());
            bool moved = false;
            auto is_movable_inst = [&](rtl::Inst* inst) {
                return inst && inst->tile.peer && !isIoBuffer(*inst) && !inst->outline.fixed
                    && !move_finished_insts.contains(reinterpret_cast<uintptr_t>(inst));
            };
            auto source_needs_move = [&](const RouteTask& task) {
                if (!task.from || !task.from->tile.peer || !task.from->cell_ref.peer) {
                    return false;
                }
                u256 output_nodes = task.from->tile->getOutputPinNodes(task.from->cell_ref->type, task.from_port, task.from->pos);
                if (output_nodes == u256{}) {
                    return false;
                }
                std::vector<Tile*> route_tiles = routeTileCandidates(*task.from, task.from_port, true);
                return !anyRoutableOutputCandidate(route_tiles, output_nodes);
            };
            auto can_move_task = [&](const RouteTask& task) {
                if (moving_focus_inst && task.to != moving_focus_inst && task.from != moving_focus_inst) {
                    return false;
                }
                return is_movable_inst(source_needs_move(task) ? task.from : task.to);
            };
            for (const RouteTask& task : route_todo) {
                if (!can_move_task(task)) {
                    continue;
                }
                std::vector<RouteTask> moved_tasks;
                RouteTask move_task = task;
                bool switching_focus_endpoint = false;
                if (moving_focus_inst) {
                    std::vector<uint64_t>& focus_tried = move_tried_placements[reinterpret_cast<uintptr_t>(moving_focus_inst)];
                    if (task.to == moving_focus_inst && is_movable_inst(task.from) && focus_tried.size() >= 4) {
                        move_task.to = task.from;
                        switching_focus_endpoint = true;
                        PNR_LOG1("ROUT", "routeDesign moving: switching focus from sink '{}' to source '{}' for net '{}'",
                            moving_focus_inst->makeName(FULL_NAME_LIMIT),
                            task.from->makeName(FULL_NAME_LIMIT), task.net_name);
                    }
                    else {
                        move_task.to = moving_focus_inst;
                    }
                }
                else if (source_needs_move(task) && is_movable_inst(task.from)) {
                    move_task.to = task.from;
                    PNR_LOG1("ROUT", "routeDesign moving: source endpoint has no routable exit, moving source inst='{}' for net '{}'",
                        task.from->makeName(FULL_NAME_LIMIT), task.net_name);
                }
                rtl::Inst* moved_inst = move_task.to;
                if (!moveUnfinishedCell(move_task, &moved_tasks, &task)) {
                    continue;
                }
                if (switching_focus_endpoint) {
                    moving_focus_inst = nullptr;
                }
                if (!moving_focus_inst) {
                    moving_focus_inst = moved_inst;
                    moving_passes = 1;
                    moving_deferred_todo.clear();
                    for (const RouteTask& deferred : route_todo) {
                        if (deferred.from == moving_focus_inst || deferred.to == moving_focus_inst) {
                            continue;
                        }
                        appendUniqueRouteTask(moving_deferred_todo, deferred);
                    }
                    PNR_LOG1("ROUT", "routeDesign moving: focusing inst='{}', deferred_tasks={}, focus_tasks={}",
                        moving_focus_inst->makeName(FULL_NAME_LIMIT), moving_deferred_todo.size(), moved_tasks.size());
                }
                route_todo = std::move(moved_tasks);
                moved = true;
                changed_this_pass++;
                stagnant_passes = 0;
                break;
            }
            if (!pending_route_todo.empty()) {
                std::vector<RouteTask>& target_queue = moving_focus_inst ? route_todo : pending_route_todo;
                if (&target_queue == &route_todo) {
                    for (const RouteTask& task : pending_route_todo) {
                        if (task.from == moving_focus_inst || task.to == moving_focus_inst) {
                            appendUniqueRouteTask(route_todo, task);
                        }
                        else {
                            appendUniqueRouteTask(moving_deferred_todo, task);
                        }
                    }
                }
                else {
                    route_todo.insert(route_todo.end(),
                        std::make_move_iterator(pending_route_todo.begin()),
                        std::make_move_iterator(pending_route_todo.end()));
                }
                pending_route_todo.clear();
            }
            if (!moved) {
                PNR_LOG1("ROUT", "routeDesign moving: no movable unfinished sink found among {} tasks", route_todo.size());
            }
            if (moving_focus_inst && route_todo.empty()) {
                move_finished_insts.insert(reinterpret_cast<uintptr_t>(moving_focus_inst));
                PNR_LOG1("ROUT", "routeDesign moving: inst='{}' had no incident routes left, restoring {} deferred tasks",
                    moving_focus_inst->makeName(FULL_NAME_LIMIT), moving_deferred_todo.size());
                route_todo = std::move(moving_deferred_todo);
                moving_deferred_todo.clear();
                moving_focus_inst = nullptr;
                stagnant_passes = 0;
                moving_passes = 0;
                moving_stage = true;
            }
        }
        if (route_todo.empty() && !fanout_stage && !fanout_route_todo.empty()) {
            fanout_stage = true;
            route_todo = std::move(fanout_route_todo);
            fanout_route_todo.clear();
            PNR_LOG1("ROUT", "routeDesign stage: Fanouts routing, tasks={}", route_todo.size());
        }

        auto route_end_time = std::chrono::steady_clock::now();
        auto export_start_time = route_end_time;
        exportPartialRouteState(bunch_list, "partial_route_state.csv");
        auto export_end_time = std::chrono::steady_clock::now();
        double rebuild_seconds = elapsedSeconds(rebuild_start_time, rebuild_end_time);
        double route_seconds = elapsedSeconds(rebuild_end_time, route_end_time);
        double export_seconds = elapsedSeconds(export_start_time, export_end_time);
        double epoch_seconds = elapsedSeconds(epoch_start_time, export_end_time);
        double total_seconds = elapsedSeconds(route_start_time, export_end_time);

        const char* stage_name = moving_stage ? "Moving" : (fanout_stage ? "Fanouts routing" : "Basic routing");
        PNR_LOG1("ROUT", "routeDesign pass: {}, stage={}, todo: {} -> {}, completed={}, active={}, advanced={}, changed={}",
            pass + 1, stage_name, before, route_todo.size(), completed_this_pass, active_this_pass,
            advanced_this_pass, changed_this_pass);
        PNR_LOG1("ROUT", "routeDesign time: pass={}, epoch={:.3f}s, rebuild={:.3f}s, route={:.3f}s, export={:.3f}s, total={:.3f}s, tasks_per_sec={:.1f}, trials_per_sec={:.1f}",
            pass + 1, epoch_seconds, rebuild_seconds, route_seconds, export_seconds, total_seconds,
            route_seconds > 0.0 ? static_cast<double>(route_stats.task_attempts) / route_seconds : 0.0,
            route_seconds > 0.0 ? static_cast<double>(route_stats.edge_trials) / route_seconds : 0.0);
        PNR_LOG1("ROUT", "routeDesign stats: tasks={}, new={}, cont={}, done={}, partial_start={}, partial_adv={}, rip={}, backtry={}, backok={}, backfrag={}, rollback={}, preempt={}/{}, no_src={}/depth0:{} joint_path:{}, dst_deadend={}, src_deadend={}, dst_deadend_tiles={}, dst_deadend_bits={}, src_deadend_tiles={}, src_deadend_bits={}, failed={}, searches={}, pops={}, deadend_tile_pops={}, dst_deadend_tile_pops={}, src_deadend_tile_pops={}, edge_trials={}, edge_ok={}, reject(name={},busy={},busy_dst={},busy_src={},busy_local={},target={},deadend={},dst_deadend={},src_deadend={})",
            route_stats.task_attempts, route_stats.new_attempts, route_stats.continuation_attempts,
            route_stats.completed, route_stats.partial_started, route_stats.partial_advanced,
            route_stats.rip_backs, route_stats.backstep_attempts, route_stats.backstep_success,
            route_stats.backstep_fragments, route_stats.commit_rollbacks,
            route_stats.preempt_success, route_stats.preempt_attempts,
            route_stats.no_src_nodes, route_stats.no_src_nodes_depth0,
            route_stats.no_src_nodes_with_joint_path,
            route_stats.dst_deadend_marks, route_stats.src_deadend_marks,
            route_dst_deadends.size(), countDeadendBits(route_dst_deadends),
            route_src_deadends.size(), countDeadendBits(route_src_deadends),
            route_stats.failed, route_stats.route_searches,
            route_stats.search_pops, route_stats.pops_on_deadend_tile,
            route_stats.pops_on_dst_deadend_tile, route_stats.pops_on_src_deadend_tile,
            route_stats.edge_trials, route_stats.edge_accepted,
            route_stats.edge_rejected_no_name, route_stats.edge_rejected_busy,
            route_stats.edge_rejected_busy_dst, route_stats.edge_rejected_busy_src,
            route_stats.edge_rejected_busy_local,
            route_stats.edge_rejected_no_target,
            route_stats.edge_rejected_deadend, route_stats.edge_rejected_dst_deadend,
            route_stats.edge_rejected_src_deadend);
        PNR_LOG1("ROUT", "routeDesign outcomes: already_done={}, new(done={},partial={},empty={},failed={}), cont(done={},advanced={},noadv={},rip={},empty={})",
            route_stats.already_complete,
            route_stats.new_completed, route_stats.new_partial, route_stats.new_empty, route_stats.new_failed,
            route_stats.cont_completed, route_stats.cont_advanced, route_stats.cont_no_advance,
            route_stats.cont_failed_rip, route_stats.cont_failed_empty);
        PNR_LOG1("ROUT", "routeDesign depth: pops={}, trials={}, ok={}, back={}, rollback={}, deadend={}, partial={}, done={}",
            statArray(route_stats.pops_by_depth), statArray(route_stats.trials_by_depth),
            statArray(route_stats.accepted_by_depth), statArray(route_stats.backsteps_by_depth),
            statArray(route_stats.rollbacks_by_depth), statArray(route_stats.deadends_by_depth),
            statArray(route_stats.partial_by_depth),
            statArray(route_stats.completed_by_depth));
        if (route_stats.has_last_busy) {
            PNR_LOG1("ROUT", "routeDesign last_busy: coord=({},{}), depth={}, local={}, src={}, src_jump={}, dst_jump={}, local_mask={}",
                route_stats.last_busy_coord.x, route_stats.last_busy_coord.y,
                route_stats.last_busy_depth, route_stats.last_busy_local, route_stats.last_busy_src,
                maskString(route_stats.last_busy_src_mask), maskString(route_stats.last_busy_dst_mask),
                maskString(route_stats.last_busy_local_mask));
        }
        if (route_stats.has_last_no_src) {
            PNR_LOG1("ROUT", "routeDesign last_no_src: coord=({},{}), depth={}, local={}, joint_mask={}",
                route_stats.last_no_src_coord.x, route_stats.last_no_src_coord.y,
                route_stats.last_no_src_depth, route_stats.last_no_src_local,
                maskString(route_stats.last_no_src_joint_mask));
        }
        if (route_stats.has_last_deadend_mark) {
            PNR_LOG1("ROUT", "routeDesign last_deadend: net='{}', dst=({},{}):{}, src=({},{}):{}",
                route_stats.last_deadend_net, route_stats.last_dst_deadend_coord.x,
                route_stats.last_dst_deadend_coord.y, route_stats.last_dst_deadend_node,
                route_stats.last_src_deadend_coord.x, route_stats.last_src_deadend_coord.y,
                route_stats.last_src_deadend_node);
        }
        if (debug_this_pass && stopAfterRouteDebugPass()) {
            PNR_ASSERT(false, "routeDesign stopped after debug pass {} by SCALEPNR_ROUTE_STOP_AFTER_DEBUG", pass + 1);
        }
        PNR_ASSERT(route_todo.empty() || active_this_pass > 0 || route_todo.size() < before || changed_this_pass > 0,
            "routeDesign made no progress in pass {} with {} cells", pass + 1, design_cells);
    }
    if (!route_todo.empty()) {
        PNR_ASSERT(false, "routeDesign did not finish after {} limited passes with {} unfinished route tasks",
            max_route_passes, route_todo.size());
    }

    logMalformedRouteTrees(technology::Tech::current().design);

    travers_mark = rtl::Inst::genMark();
    image.init(mesh_width*aspect_x*image_zoom, mesh_height*aspect_y*image_zoom);
    image.clear();
    for (auto& bunch : bunch_list) {
        recurseDrawDesign(*bunch.reg, &bunch, false);
    }
    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recurseDrawDesign(*bunch.reg, &bunch, true);
    }
    image.write(std::string("route_output.png"));
}

void RouteDesign::recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, bool place, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    if (place) {
        if (inst.cell_ref->type.find("BUF") != std::string::npos) {
            image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 255, 255);
        }
        else if (inst.cell_ref->type.find("LUT") != std::string::npos) {
            image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 0, 255);
        }
        else {
            image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 0, 255, 255);
        }
    }
    else {
        for (auto& wireing : inst.wires) {
            for (auto& wire : wireing) {
//    std::print("\naaaaaaaaaaaaaaaa");
                int r = wire.to.x > wire.from.x ? wire.to.x - wire.from.x : wire.from.x - wire.to.x;
                int g = wire.to.y > wire.from.y ? wire.to.y - wire.from.y : wire.from.y - wire.to.y;
                image.draw_line(wire.from.x*image_zoom+r, wire.from.y*image_zoom+g, wire.to.x*image_zoom+r, wire.from.y*image_zoom+g, r*100, g*100, 0, 255);
                image.draw_line(wire.to.x*image_zoom+r, wire.from.y*image_zoom+g, wire.to.x*image_zoom+r, wire.to.y*image_zoom+g, r*100, g*100, 0, 255);
            }
        }
    }

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

/*            if (peer->coord.fixed || curr->inst_ref->coord.fixed) {
                image.draw_line(inst.coord.x*aspect_x*image_zoom, inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom, peer->coord.y*aspect_y*image_zoom, 200, 200, 200, 100);
            }
            else
            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
if (mode == 1) {
                image.draw_line(inst.coord.x*aspect_x*image_zoom, inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom, peer->coord.y*aspect_y*image_zoom, 255, 0, 0, 100);
}
            }
            else {
if (mode == 1) {
                image.draw_line(inst.coord.x*aspect_x*image_zoom, inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom, peer->coord.y*aspect_y*image_zoom, 0, 200, 200, 100);
}
            }
*/
            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDrawDesign(*peer, nullptr, place, depth + 1);
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDrawDesign(*subbunch.reg, &subbunch, place, depth + 1);
        }
    }
}
