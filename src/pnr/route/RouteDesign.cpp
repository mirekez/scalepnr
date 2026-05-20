#include "RouteDesign.h"
#include "Device.h"
#include "Tech.h"
#include "Wire.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <queue>
#include <unordered_set>

using namespace pnr;

namespace {

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

uint64_t routeVisitKey(const Coord& coord, int local)
{
    return (static_cast<uint64_t>(static_cast<uint16_t>(coord.x)) << 48)
        | (static_cast<uint64_t>(static_cast<uint16_t>(coord.y)) << 32)
        | static_cast<uint32_t>(local);
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

bool hasRoutedNet(const rtl::Inst& inst, const std::string& net_name)
{
    for (const auto& route : inst.wires) {
        if (!route.empty() && route.front().net_name == net_name) {
            return true;
        }
    }
    return false;
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
        return (!match_from || preferred_from.empty() || conn.from == preferred_from)
            && (!match_to || preferred_to.empty() || conn.to == preferred_to);
    };
    for (const fpga::CBConnName& conn : *conns) {
        if (matches(conn, true, true)) {
            return &conn;
        }
    }
    for (const fpga::CBConnName& conn : *conns) {
        if (matches(conn, true, false)) {
            return &conn;
        }
    }
    for (const fpga::CBConnName& conn : *conns) {
        if (matches(conn, false, true)) {
            return &conn;
        }
    }
    return &conns->front();
}

bool leaseConcreteOut(CBState& cb, int local, int src)
{
    u256 src_bit = u256{0,1} << src;
    if ((cb.src.jump & src_bit) != u256{}) {
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
    if ((cb.dst.jump & dst_bit) != u256{} || (cb.src.jump & src_bit) != u256{}) {
        return false;
    }
    cb.dst.jump |= dst_bit;
    cb.src.jump |= src_bit;
    return true;
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
    }
    if (const fpga::CBConnName* conn = selectConcreteConn(tile.cb_type, from_type, from_value,
            fpga::CB_NODE_SRC, src_node, from_wire_name)) {
        if (!from_wire_name.empty() && conn->from != from_wire_name) {
            return {};
        }
        return conn->to;
    }
    if (const std::string* src = tile.cb_type->nodeName(fpga::CB_NODE_SRC, src_node)) {
        return *src;
    }
    return {};
}

std::vector<Tile*> routeTileCandidates(rtl::Inst& inst, const std::string& port, bool output)
{
    std::vector<Tile*> candidates;
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
    for (auto& tile_ref : fpga::Device::current().tile_grid) {
        Tile& tile = static_cast<Tile&>(tile_ref);
        int distance = routeDistance(inst.tile->coord, tile.coord);
        if (distance > endpoint_search_radius) {
            continue;
        }
        u256 candidate_nodes = endpoint_nodes;
        if (candidate_nodes == u256{} || !supportsLocalNodes(tile, candidate_nodes)) {
            candidate_nodes = output
                ? tile.getOutputPinNodes(inst.cell_ref->type, port, inst.pos)
                : tile.getPinNodes(inst.cell_ref->type, port, inst.pos);
        }
        if (!supportsLocalNodes(tile, candidate_nodes)) {
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

    std::sort(scored.begin(), scored.end(), [](const Candidate& a, const Candidate& b) {
        return a.score < b.score;
    });
    for (const Candidate& candidate : scored) {
        add_candidate(candidate.tile);
    }
    if (candidates.empty() && isConcreteRouteTile(*inst.tile)) {
        add_candidate(&*inst.tile);
    }
    return candidates;
}

bool tryBestFirstRoute(Tile& from, Tile& to, int from_pos, rtl::Inst& dst_inst,
                       const std::string& to_port, std::vector<Wire>& wire, int iteration_limit)
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

    u256 pin_nodes = to.getPinNodes(dst_inst.cell_ref->type, to_port, dst_inst.pos);
    if (pin_nodes == u256{}) {
        return false;
    }

    std::vector<Step> steps;
    std::priority_queue<QueueItem> queue;
    std::unordered_set<uint64_t> visited;
    steps.push_back(Step{from.coord, from_pos, -1, -1, -1, 0});
    queue.push(QueueItem{routeDistance(from.coord, to.coord), 0});
    visited.insert(routeVisitKey(from.coord, from_pos));

    int final_step = -1;
    int final_pin = -1;
    int final_joint = -1;
    constexpr int max_depth = 80;
    constexpr size_t max_steps = 8000;

    while (!queue.empty() && steps.size() < max_steps) {
        int idx = queue.top().step;
        queue.pop();
        Step step = steps[idx];
        Tile* tile = fpga::Device::current().getTile(step.coord.x, step.coord.y);
        if (!tile || !isConcreteRouteTile(*tile) || step.depth > max_depth) {
            continue;
        }

        if (step.coord == to.coord) {
            bool ok = pin_nodes.for_each_set_bit([&](int pin) {
                int joint = -1;
                CBState test_cb = tile->cb;
                if (!tile->isPinNodeLeased(pin) && tile->cb_type->canIn(step.local, pin, joint)
                    && leaseConcreteIn(test_cb, step.local, pin)) {
                    final_step = idx;
                    final_pin = pin;
                    final_joint = joint;
                    return true;
                }
                return false;
            });
            if (ok) {
                break;
            }
        }

        fpga::CBNodeNameType from_type = step.depth == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
        const std::vector<uint8_t>* src_nodes = tile->cb_type->srcNodes(from_type, step.local);
        if (!src_nodes) {
            continue;
        }
            for (uint8_t src_node : *src_nodes) {
                int joint = -1;
                std::string src_wire = concreteSrcWireName(*tile, from_type, step.local, src_node, joint, step.dst_wire);
                if (src_wire.empty()) {
                    continue;
                }
                CBState test_cb = tile->cb;
                bool lease_ok = step.depth == 0
                    ? leaseConcreteOut(test_cb, step.local, src_node)
                    : leaseConcreteJump(test_cb, step.local, src_node);
                if (!lease_ok) {
                    continue;
                }
                fpga::TileJumpTarget target = fpga::Device::current().resolveJump(*tile, src_node, src_wire);
                Coord next;
                int next_local = -1;
            if (target.tile && target.tile->cb_type && target.dst_node >= 0) {
                next = target.tile->coord;
                next_local = target.dst_node;
            }
            else {
                if (!fpga::Device::current().tile_conn_rules.empty()) {
                    continue;
                }
                next = makeActualJump(step.coord, src_node);
                Tile* next_tile = fpga::Device::current().getTile(next.x, next.y);
                if (!next_tile || !next_tile->cb_type) {
                    continue;
                }
                next_local = src_node;
            }
            if (!visited.insert(routeVisitKey(next, next_local)).second) {
                continue;
            }

            steps.push_back(Step{next, next_local, idx, src_node, joint, step.depth + 1, src_wire, target.dst_wire});
            int next_idx = static_cast<int>(steps.size() - 1);
            int score = routeDistance(next, to.coord) * 4 + step.depth;
            queue.push(QueueItem{score, next_idx});
        }
    }

    if (final_step < 0) {
        return false;
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
    auto rollback = [&]() {
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
        bool lease_ok = prev.depth == 0
            ? leaseConcreteOut(prev_tile->cb, prev.local, curr.jump)
            : leaseConcreteJump(prev_tile->cb, prev.local, curr.jump);
        if (!lease_ok) {
            rollback();
            return false;
        }
    }

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
        fragment.pos = i == 1 ? 0 : 1;
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

    Wire enter;
    enter.from = to.coord;
    enter.to = to.coord;
    enter.local = steps[final_step].local;
    enter.joint = final_joint;
    enter.pos = 1;
    enter.dst_wire_name = steps[final_step].dst_wire;
    wire.push_back(enter);

    Wire pin;
    pin.type = Wire::WIRE_TILE_PIN;
    pin.from = to.coord;
    pin.to = to.coord;
    pin.local = final_pin;
    pin.pos = dst_inst.pos;
    pin.port = to_port;
    wire.push_back(pin);
    return true;
}

bool isIoBuffer(rtl::Inst& inst)
{
    return technology::Tech::current().buffers_ports.find(inst.cell_ref->type) != technology::Tech::current().buffers_ports.end();
}

int iterationLimitFromCells(int cells)
{
    return std::max(1, cells / 10);
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
    }
}

Wire makeEndpointWire(rtl::Inst& from, const std::string& from_port, rtl::Inst& to, const std::string& to_port)
{
    Wire wire;
    wire.type = Wire::WIRE_TILE_PIN;
    wire.port = to_port.empty() ? from_port : to_port;

    if (isIoBuffer(from) && from.tile.peer) {
        wire.from = from.tile->coord;
        wire.to = from.tile->coord;
        wire.local = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos).ffs256();
        if (wire.local < 0) {
            wire.local = from.pos;
        }
        wire.pos = from.pos;
        wire.port = from_port;
        return wire;
    }

    if (isIoBuffer(to) && to.tile.peer) {
        wire.from = to.tile->coord;
        wire.to = to.tile->coord;
        wire.local = to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos).ffs256();
        if (wire.local < 0) {
            wire.local = to.pos;
        }
        wire.pos = to.pos;
        wire.port = to_port;
        return wire;
    }

    if (to.tile.peer) {
        wire.from = to.tile->coord;
        wire.to = to.tile->coord;
        wire.local = to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos).ffs256();
        wire.pos = to.pos;
        return wire;
    }

    if (from.tile.peer) {
        wire.from = from.tile->coord;
        wire.to = from.tile->coord;
        wire.local = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos).ffs256();
        wire.pos = from.pos;
        return wire;
    }

    wire.from = Coord{-1, -1};
    wire.to = Coord{-1, -1};
    return wire;
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
            wire[depth+1].type = Wire::WIRE_TILE_PIN;
            wire[depth+1].from = to.coord;
            wire[depth+1].to = to.coord;
            wire[depth+1].local = local;
            wire[depth+1].pos = dst_inst->pos;
            wire[depth+1].port = to_port;
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

bool RouteDesign::routeNet(rtl::Inst& from, const std::string& from_port, rtl::Inst& to, const std::string& to_port, std::vector<Wire>& wire)
{
//    PNR_ASSERT(!from.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not assigned", from.makeName())
//    PNR_ASSERT(!to.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not assigned", to.makeName())
    if (!from.tile.peer || !to.tile.peer) {
        return true;
    }
    std::vector<Tile*> from_route_tiles = routeTileCandidates(from, from_port, true);
    std::vector<Tile*> to_route_tiles = routeTileCandidates(to, to_port, false);
    u256 output_nodes = from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
    if (output_nodes == u256{}) {
        output_nodes = u256{0,1} << from.pos;
    }

    auto try_output_nodes = [&](u256 nodes) {
        return nodes.for_each_set_bit([&](int local) {
            for (Tile* from_route_tile : from_route_tiles) {
                for (Tile* to_route_tile : to_route_tiles) {
                    wire.clear();
                    if (from_route_tile && to_route_tile && tryBestFirstRoute(*from_route_tile, *to_route_tile, local, to, to_port, wire, iteration_limit)) {
                        return true;
                    }
                }
            }
            return false;
        });
    };

    bool routed = try_output_nodes(output_nodes);
    if (!routed && from.tile->tile_type) {
        u256 alternate_output_nodes{};
        for (const auto& pair : from.tile->tile_type->pin_map.output_nodes) {
            alternate_output_nodes |= pair.second;
        }
        routed = try_output_nodes(alternate_output_nodes);
    }
    if (!routed) {
        auto try_backtracking = [&](u256 nodes) {
            route_iteration_budget = iteration_limit;
            return nodes.for_each_set_bit([&](int local) {
                for (Tile* from_route_tile : from_route_tiles) {
                    for (Tile* to_route_tile : to_route_tiles) {
                        if (!from_route_tile || !to_route_tile) {
                            continue;
                        }
                        wire.clear();
                        if (tryNext(*from_route_tile, *to_route_tile, local, to.pos, to_port, wire, 0, &to)) {
                            return true;
                        }
                    }
                }
                return false;
            });
        };
        routed = try_backtracking(output_nodes);
        if (!routed && from.tile->tile_type) {
            u256 alternate_output_nodes{};
            for (const auto& pair : from.tile->tile_type->pin_map.output_nodes) {
                alternate_output_nodes |= pair.second;
            }
            routed = try_backtracking(alternate_output_nodes);
        }
    }
    return routed;
}

bool RouteDesign::routeNet(rtl::Inst& from, rtl::Inst& to, const std::string& to_port, std::vector<Wire>& wire)
{
    return routeNet(from, std::string(), to, to_port, wire);
}

bool RouteDesign::routeNet(rtl::Inst& from, rtl::Inst& to, std::vector<Wire>& wire)
{
    return routeNet(from, to, std::string(), wire);
}

void RouteDesign::collectRouteTasks(rtl::Inst& inst, RegBunch* bunch)
{
    if (inst.mark == travers_mark) {
        return;
    }

    inst.mark = travers_mark;
    route_todo.push_back(RouteTask{&inst, bunch});

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

    bool complete = true;
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
            std::string net_name = conn.makeNetName();

            if (!hasRoutedNet(inst, net_name)) {
                if (route_recursion_budget <= 0) {
                    return false;
                }

                std::vector<Wire> wire;
                if (routeNet(*peer, curr->port_ref->makeName(), inst, conn.port_ref->makeName(), wire)) {
                    if (wire.empty()) {
                        wire.emplace_back(makeEndpointWire(*peer, curr->port_ref->makeName(), inst, conn.port_ref->makeName()));
                    }
                    for (Wire& fragment : wire) {
                        fragment.net_name = net_name;
                    }
                    inst.wires.emplace_back(std::move(wire));
                    --route_recursion_budget;
                }
                else {
                    PNR_ASSERT(false, "failed to route net '{}' from '{}' port '{}' to '{}' port '{}'",
                        conn.makeNetName(), peer->makeName(), curr->port_ref->makeName(), inst.makeName(), conn.port_ref->makeName());
                }
            }
        }
    }

    return complete;
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
    PNR_LOG1("ROUT", "routeDesign, cells: {}, iteration_limit: {}", design_cells, iteration_limit);
//    combs_per_box = /*total_comb*/(float)fpga.cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga->size_width;
    fpga_height = fpga->size_height;

    resetRoutingState();

    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;

    constexpr int route_recursion_limit = 5;
    int max_route_passes = std::max(1, design_cells * 4);
    route_todo.clear();
    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        PNR_ASSERT(bunch.reg, "zero reg in bunch with address {}", (uint64_t)&bunch);
        collectRouteTasks(*bunch.reg, &bunch);
    }
    PNR_LOG1("ROUT", "routeDesign tasks: {}", route_todo.size());

    for (int pass = 0; pass < max_route_passes && !route_todo.empty(); ++pass) {
        size_t before = route_todo.size();
        int routed_this_pass = 0;

        for (auto it = route_todo.begin(); it != route_todo.end();) {
            if (!it->inst) {
                it = route_todo.erase(it);
                continue;
            }
            route_recursion_budget = route_recursion_limit;
            if (routeInstTask(*it->inst)) {
                routed_this_pass += route_recursion_limit - route_recursion_budget;
                it = route_todo.erase(it);
                continue;
            }
            routed_this_pass += route_recursion_limit - route_recursion_budget;
            ++it;
        }

        PNR_LOG1("ROUT", "routeDesign pass: {}, routed: {}, todo: {} -> {}",
            pass + 1, routed_this_pass, before, route_todo.size());
        PNR_ASSERT(route_todo.empty() || routed_this_pass > 0 || route_todo.size() < before,
            "routeDesign made no progress in pass {} with {} cells", pass + 1, design_cells);
    }
    PNR_ASSERT(route_todo.empty(), "routeDesign did not finish after {} passes with {} cells", max_route_passes, design_cells);

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
