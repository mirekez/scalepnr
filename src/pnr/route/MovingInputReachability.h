#pragma once

#include "Docking.h"

#include <algorithm>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pnr {

// A necessary condition for an input during one release-only Source Moving
// search. Keep the free reverse region and reopen its blocked SRC/DST nodes
// after cuts. Ignore transit JOINT occupancy, hop limits and the exact driver pin:
// reaching any driver/prefix CB means UNKNOWN, never a validated input route.
class MovingInputReachability {
    struct Node { fpga::Tile* tile; int kind, id; };
    struct State { std::array<NodeMask,2> reached{}, blocked{}; };
    enum { DST, SRC };
    fpga::Tile* target;
    size_t max_states;
    std::unordered_map<fpga::Tile*,State> states;
    std::unordered_set<fpga::Tile*> goals;
    std::vector<Node> frontier;
    NodeMask seeded_pins{};
    size_t head = 0;
    bool connected = false, limited = false;

    static const fpga::CBState& state(fpga::Tile& tile,
                                     const BackwardTakeoffStateView* view) {
        if (view) {
            auto found=view->find(&tile);
            if (found!=view->end()) return found->second;
        }
        return tile.cb;
    }
    void add(fpga::Tile* tile, int kind, int id,
             const BackwardTakeoffStateView* view) {
        if (!tile || !tile->cb_type || id<0 || id>=CB_MAX_NODES || limited) return;
        auto& s=states[tile];
        if (s.reached[kind].testBit(id)) return;
        const auto& cb=state(*tile,view);
        if ((kind==SRC ? cb.src.jump : cb.dst.jump).testBit(id)) {
            s.blocked[kind].setBit(id);
            return;
        }
        if (frontier.size()>=max_states) { limited=true; return; }
        s.blocked[kind].clearBit(id);
        s.reached[kind].setBit(id);
        frontier.push_back({tile,kind,id});
        connected |= kind==SRC && goals.contains(tile);
    }
    void seed(const BackwardTakeoffStateView* view) {
        const auto& cb=state(*target,view);
        seeded_pins.for_each_set_bit([&](int pin) {
            if (target->isPinNodeLeased(pin) || cb.local.local.testBit(pin)) return false;
            for (const auto& path:target->cb_type->terminalEntries(pin)) {
                if ((path.joint>=0 && cb.joint.jump.testBit(path.joint)) ||
                    (path.joint2>=0 && cb.joint.jump.testBit(path.joint2))) continue;
                add(target,DST,path.dst,view);
            }
            return false;
        });
    }
public:
    explicit MovingInputReachability(fpga::Tile& target,
            size_t max_states=fpga::UNBOUND_ROUTE_STATE_BUDGETS.back())
        : target(&target), max_states(max_states) {}

    size_t expanded() const { return head; }
    bool uncertain() const { return connected || limited; }

    // No added leases or placement edits are permitted between queries.
    // Wake only blocked nodes which really became free, including exemptions
    // in the caller's independent private view. Unrelated cuts do no work.
    bool released(const std::vector<fpga::Tile*>& changed,
                  const BackwardTakeoffStateView* view=nullptr) {
        const size_t before=frontier.size();
        for (auto* tile:changed) {
            auto found=states.find(tile);
            if (found==states.end()) continue;
            const auto& cb=state(*tile,view);
            for (int kind:{DST,SRC}) {
                const NodeMask free=found->second.blocked[kind] &
                    ~(kind==SRC ? cb.src.jump : cb.dst.jump);
                free.for_each_set_bit([&](int id) {add(tile,kind,id,view);return false;});
            }
        }
        if (std::find(changed.begin(),changed.end(),target)!=changed.end()) seed(view);
        return frontier.size()!=before;
    }

    bool mayReach(const NodeMask& pins, const std::vector<fpga::Tile*>& goal_tiles,
                  BackwardResolveIndex& index,
                  const BackwardTakeoffStateView* view=nullptr,
                  const BackwardTakeoffCancel& cancel={}) {
        if (!target->cb_type || pins==NodeMask{}) return true;
        if (cancel && cancel()) return true; // Cancelled means unknown.
        for (auto* tile:goal_tiles) if (tile) {
            goals.insert(tile);
            const auto found=states.find(tile);
            connected |= tile==target || (found!=states.end() &&
                                         found->second.reached[SRC]!=NodeMask{});
        }
        seeded_pins |= pins;
        seed(view);
        while (!connected && !limited && head<frontier.size()) {
            if (cancel && cancel()) return true;
            const auto n=frontier[head++];
            if (auto* trace=fpga::RouteSearchTrace::current)
                trace->stream() << "INPUT_REACHABILITY_POP target=(" << target->coord.x
                    << ',' << target->coord.y << ") tile=(" << n.tile->coord.x << ','
                    << n.tile->coord.y << ") kind="
                    << int(n.kind==SRC ? fpga::CB_NODE_SRC : fpga::CB_NODE_DST)
                    << " node=" << n.id << '\n';
            if (n.kind==DST) {
                const auto* incoming=resolveBackwardSources(index,
                    {n.tile->coord.x,n.tile->coord.y,n.id},true);
                if (incoming) for (const auto& from:*incoming)
                    add(from.tile,SRC,from.src,view);
            } else {
                n.tile->cb_type->ensureDerivedMasks();
                n.tile->cb_type->dsts_reaching_src.get(n.id).jump.for_each_set_bit(
                    [&](int dst) {add(n.tile,DST,dst,view);return false;});
            }
        }
        return connected || limited;
    }
};
} // namespace pnr
