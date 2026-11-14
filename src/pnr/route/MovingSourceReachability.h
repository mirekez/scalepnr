#pragma once
#include "InputRepairBudget.h"

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
    const std::unordered_map<fpga::Tile*,fpga::CBState>* state_view = nullptr;
    size_t head = 0;
    size_t label_limit;
    bool connected = false, limited = false;

    bool valid(Node n) const {
        return n.tile && n.tile->cb_type && n.id >= 0 && n.id < CB_MAX_NODES &&
            int(n.type) >= 0 && int(n.type) < 5;
    }
    void add(Node n, bool seed = false) {
        if (!valid(n) || limited) return;
        auto& s = states[n.tile];
        if (s.reached[n.type].testBit(n.id)) return;
        const auto found=state_view ? state_view->find(n.tile)
            : std::unordered_map<fpga::Tile*,fpga::CBState>::const_iterator{};
        const auto& cb=state_view && found!=state_view->end() ? found->second : n.tile->cb;
        const bool busy = fpga::congestionNodeMask(cb,n.type).testBit(n.id) ||
            (n.type == fpga::CB_NODE_LOCAL && n.tile->isPinNodeLeased(n.id));
        if (!seed && busy && !s.owned[n.type].testBit(n.id)) {
            s.blocked[n.type].setBit(n.id);
            return;
        }
        // Release-only routing retains a large component across cuts. A
        // per-candidate preliminary caller can choose a smaller label budget.
        // Saturation means unknown in both uses, never an impossibility proof.
        if (frontier.size() == label_limit) {
            limited = true;
            if (auto* trace=fpga::RouteSearchTrace::current)
                trace->stream() << "SOURCE_REACHABILITY_LIMIT states=" << frontier.size() << '\n';
            return;
        }
        s.reached[n.type].setBit(n.id);
        frontier.push_back(n);
        connected |= s.targets[n.type].testBit(n.id);
    }
public:
    using LocalRegion = std::unordered_map<fpga::Tile*,NodeMask>;
    explicit MovingSourceReachability(
        const std::unordered_map<fpga::Tile*,fpga::CBState>* view = nullptr,
        size_t max_labels = 8 * fpga::UNBOUND_ROUTE_STATE_BUDGETS.back())
        : state_view(view), label_limit(max_labels) {}
    size_t expanded() const { return head; }
    // A completed negative walk exposes its occupied boundary so callers can
    // invalidate cached impossibility proofs when a later cut opens it.
    template<class Visit> void visitBlocked(Visit visit) const {
        for(const auto& [tile,state]:states)
            for(int kind=0;kind<5;++kind)
                state.blocked[kind].for_each_set_bit([&](int node) {
                    visit(*tile,static_cast<fpga::CBNodeNameType>(kind),node);
                    return false;
                });
    }
    // Only a fully exhausted negative walk proves a closed component. Its
    // reachable LOCALs can reject other targets in the same immutable view;
    // a saturated or interrupted walk must never become such a proof.
    bool closed() const { return !connected && !limited && head == frontier.size(); }
    LocalRegion localRegion() const {
        LocalRegion result;
        for (const auto& [tile,state] : states)
            if (state.reached[fpga::CB_NODE_LOCAL] != NodeMask{})
                result.emplace(tile,state.reached[fpga::CB_NODE_LOCAL]);
        return result;
    }
    void own(fpga::Tile* tile, fpga::CBNodeNameType type, int id) {
        if (valid({tile,type,id})) states[tile].owned[type].setBit(id);
    }
    void source(fpga::Tile* tile, int local) { add({tile,fpga::CB_NODE_LOCAL,local},true); }
    void landing(fpga::Tile* tile, int dst) { add({tile,fpga::CB_NODE_DST,dst},true); }
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
            if (!InputRepairBudget::expand()) return true;
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
