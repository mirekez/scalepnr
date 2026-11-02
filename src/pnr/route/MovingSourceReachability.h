#pragma once

#include "Device.h"
#include "Wire.h"

#include <array>
#include <ostream>
#include <unordered_map>
#include <vector>

namespace pnr {

// One live source walk while boundary preemptions only RELEASE resources.
// This is an overapproximation: no hop/Tile-repeat or packed-input restrictions.
// A positive answer still needs the ordinary atomic numeric route validator.
// A negative answer avoids restarting that validator in an unchanged component.
class MovingSourceReachability {
    struct Node { fpga::Tile* tile; fpga::CBNodeNameType type; int id; };
    struct State { std::array<NodeMask,5> reached{}, blocked{}, owned{}, targets{}; };
    std::unordered_map<fpga::Tile*,State> states;
    std::vector<Node> frontier;
    size_t head = 0;
    bool connected = false, limited = false;

    bool valid(Node n) const {
        return n.tile && n.tile->cb_type && n.id >= 0 && n.id < CB_MAX_NODES &&
            int(n.type) >= 0 && int(n.type) < 5;
    }
    void add(Node n, bool seed = false) {
        if (!valid(n) || limited) return;
        auto& s = states[n.tile];
        if (s.reached[n.type].testBit(n.id)) return;
        const bool busy = fpga::congestionNodeMask(n.tile->cb,n.type).testBit(n.id) ||
            (n.type == fpga::CB_NODE_LOCAL && n.tile->isPinNodeLeased(n.id));
        if (!seed && busy && !s.owned[n.type].testBit(n.id)) {
            s.blocked[n.type].setBit(n.id);
            return;
        }
        // This release-only rejection walk outlives many exact wire probes.
        // Real free components exceed the exact solver's 512K label cap;
        // saturating here would restart those probes after every later cut.
        // Keep a separate 4M bound. It never authorizes or commits a route.
        if (frontier.size() == 8 * fpga::UNBOUND_ROUTE_STATE_BUDGETS.back()) {
            limited = true; // Unknown is never an impossibility proof.
            return;
        }
        s.reached[n.type].setBit(n.id);
        frontier.push_back(n);
        connected |= s.targets[n.type].testBit(n.id);
    }
public:
    size_t expanded() const { return head; }
    void own(fpga::Tile* tile, fpga::CBNodeNameType type, int id) {
        if (valid({tile,type,id})) states[tile].owned[type].setBit(id);
    }
    void source(fpga::Tile* tile, int local) { add({tile,fpga::CB_NODE_LOCAL,local},true); }
    void target(fpga::Tile* tile, int local) {
        if (!valid({tile,fpga::CB_NODE_LOCAL,local})) return;
        auto& s = states[tile];
        s.targets[fpga::CB_NODE_LOCAL].setBit(local);
        connected |= s.reached[fpga::CB_NODE_LOCAL].testBit(local);
    }
    void released(const std::vector<fpga::RouteCutNode>& nodes) {
        auto& device = fpga::Device::current();
        for (const auto& cut : nodes) {
            auto* tile = device.getTile(cut.tile.x,cut.tile.y);
            auto found = states.find(tile);
            if (found == states.end() || !valid({tile,cut.type,cut.node})) continue;
            if (found->second.blocked[cut.type].testBit(cut.node))
                add({tile,cut.type,cut.node});
        }
    }
    template<class Cancel> bool mayConnect(Cancel cancelled) {
        auto& device = fpga::Device::current();
        while (!connected && !limited && head < frontier.size()) {
            if (cancelled()) return true;
            const auto n = frontier[head++];
            if (auto* trace=fpga::RouteSearchTrace::current)
                trace->stream() << "SOURCE_REACHABILITY_POP tile=(" << n.tile->coord.x
                    << ',' << n.tile->coord.y << ") kind=" << int(n.type) << " node=" << n.id << '\n';
            const auto& cb = *n.tile->cb_type;
            auto mask = [&](const NodeMask& bits, fpga::CBNodeNameType type) {
                bits.for_each_set_bit([&](int id) { add({n.tile,type,id}); return false; });
            };
            switch (n.type) {
            case fpga::CB_NODE_LOCAL:
                mask(cb.local_src[n.id].jump,fpga::CB_NODE_SRC);
                mask(cb.local_joint[n.id].joint,fpga::CB_NODE_JOINT);
                mask(cb.local_local[n.id].local,fpga::CB_NODE_LOCAL);
                for (const auto& t : device.resolveLocalTargets(*n.tile,n.id))
                    add({t.tile,t.node_type,t.node});
                break;
            case fpga::CB_NODE_DST:
                mask(cb.dst_src[n.id].jump,fpga::CB_NODE_SRC);
                mask(cb.dst_joint[n.id].joint,fpga::CB_NODE_JOINT);
                mask(cb.dst_local[n.id].local,fpga::CB_NODE_LOCAL);
                break;
            case fpga::CB_NODE_JOINT:
                mask(cb.joint_src[n.id].jump,fpga::CB_NODE_SRC);
                mask(cb.joint_joint[n.id].joint,fpga::CB_NODE_JOINT);
                mask(cb.joint_local[n.id].local,fpga::CB_NODE_LOCAL);
                break;
            case fpga::CB_NODE_SRC:
                mask(cb.src_joint[n.id].joint,fpga::CB_NODE_JOINT);
                for (const auto& t : device.resolveJumpTargets(*n.tile,n.id))
                    add({t.tile,fpga::CB_NODE_DST,t.dst_node});
                break;
            default: break;
            }
        }
        return connected || limited;
    }
};
} // namespace pnr
