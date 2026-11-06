#pragma once
#include "InputRepairBudget.h"

#include "Docking.h"

#include <algorithm>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pnr {

// A necessary condition for an input during one release-only Source Moving
// search. Keep the free typed reverse region and reopen released nodes after
// cuts, including resolved LOCAL relays. Ignore hop limits and the exact driver
// pin: reaching any driver/prefix CB means UNKNOWN, never an admitted route.
class MovingInputReachability {
    struct Node { fpga::Tile* tile; int kind, id; };
    struct State { std::array<NodeMask,5> reached{}, blocked{}; };
    fpga::Tile* target;
    size_t max_states;
    std::unordered_map<fpga::Tile*,State> states;
    std::unordered_set<fpga::Tile*> goals;
    std::vector<Node> frontier;
    using Predecessor=std::pair<int,int>;
    std::unordered_map<const fpga::CBType*,
        std::unordered_map<unsigned,std::vector<Predecessor>>> internal_inputs;
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
    static NodeMask occupied(fpga::Tile& tile,int kind,
                              const BackwardTakeoffStateView* view) {
        NodeMask result=fpga::congestionNodeMask(state(tile,view),
            static_cast<fpga::CBNodeNameType>(kind));
        if(kind==fpga::CB_NODE_LOCAL)result|=tile.pin_state.leased_nodes;
        return result;
    }
    void add(fpga::Tile* tile, int kind, int id,
             const BackwardTakeoffStateView* view) {
        if (!tile || !tile->cb_type || id<0 || id>=CB_MAX_NODES || limited ||
            (kind!=0 && kind!=1 && kind!=3 && kind!=4)) return;
        // A prefix CB is only a necessary-condition goal. Its retained input
        // can already be leased; the exact proof decides whether it is usable.
        connected |= goals.contains(tile);
        auto& s=states[tile];
        if (s.reached[kind].testBit(id)) return;
        if (occupied(*tile,kind,view).testBit(id)) {
            s.blocked[kind].setBit(id);
            return;
        }
        if (frontier.size()>=max_states) { limited=true; return; }
        s.blocked[kind].clearBit(id);
        s.reached[kind].setBit(id);
        frontier.push_back({tile,kind,id});
    }
    void seed(const BackwardTakeoffStateView* view) {
        seeded_pins.for_each_set_bit([&](int pin) {
            add(target,fpga::CB_NODE_LOCAL,pin,view);return false;
        });
    }
public:
    explicit MovingInputReachability(fpga::Tile& target,
            size_t max_states=fpga::UNBOUND_ROUTE_STATE_BUDGETS.back())
        : target(&target), max_states(max_states) {}

    size_t expanded() const { return head; }
    bool uncertain() const { return connected || limited; }

    // A complete negative region cannot change unless a preview releases one
    // of its blocked boundary nodes. Inspect the full victim release footprint,
    // not just the contested jump: shared cuts can release additional nodes.
    bool mayChangeAfterRelease(const std::vector<fpga::RouteCutNode>& released) const {
        if (uncertain()) return true;
        for (const auto& cut:released) {
            auto* tile=fpga::Device::current().getTile(cut.tile.x,cut.tile.y);
            auto found=states.find(tile);
            if (found!=states.end() && cut.type>=0 && cut.type<5 &&
                cut.node>=0 && cut.node<CB_MAX_NODES &&
                found->second.blocked[cut.type].testBit(cut.node)) return true;
        }
        return false;
    }

    // No added leases or placement edits are permitted between queries.
    // Wake only blocked nodes which really became free, including exemptions
    // in the caller's independent private view. Unrelated cuts do no work.
    bool released(const std::vector<fpga::Tile*>& changed,
                  const BackwardTakeoffStateView* view=nullptr) {
        if(uncertain())return false;
        const size_t before=frontier.size();
        for (auto* tile:changed) {
            auto found=states.find(tile);
            if (found==states.end()) continue;
            for (int kind:{0,1,3,4}) {
                const NodeMask free=found->second.blocked[kind] &
                    ~occupied(*tile,kind,view);
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
        if(uncertain())return true;
        for (auto* tile:goal_tiles) if (tile) {
            goals.insert(tile);
            const auto found=states.find(tile);
            connected |= tile==target || (found!=states.end() &&
                std::any_of(found->second.reached.begin(),found->second.reached.end(),
                    [](const NodeMask& mask){return mask!=NodeMask{};}));
        }
        seeded_pins |= pins;
        seed(view);
        while (!connected && !limited && head<frontier.size()) {
            if (cancel && cancel()) return true;
            if (!InputRepairBudget::expand()) return true;
            const auto n=frontier[head++];
            if (auto* trace=fpga::RouteSearchTrace::current)
                trace->stream() << "INPUT_REACHABILITY_POP target=(" << target->coord.x
                    << ',' << target->coord.y << ") tile=(" << n.tile->coord.x << ','
                    << n.tile->coord.y << ") kind="
                    << n.kind
                    << " node=" << n.id << '\n';
            auto [row,inserted]=internal_inputs[n.tile->cb_type].try_emplace(
                (unsigned(n.kind)<<16)|unsigned(n.id));
            if(inserted) {
                const auto& cb=*n.tile->cb_type;
                auto scan=[&](const auto& table,auto bits,int kind) {
                    for(const auto& [id,value]:table.values)
                        if(bits(value).testBit(n.id))row->second.emplace_back(kind,id);
                };
                auto locals=[](const auto& value)->const NodeMask& {return value.local;};
                auto joints=[](const auto& value)->const NodeMask& {return value.joint;};
                auto jumps=[](const auto& value)->const NodeMask& {return value.jump;};
                if(n.kind==fpga::CB_NODE_LOCAL) {
                    scan(cb.dst_local,locals,fpga::CB_NODE_DST);
                    scan(cb.joint_local,locals,fpga::CB_NODE_JOINT);
                    scan(cb.local_local,locals,fpga::CB_NODE_LOCAL);
                } else if(n.kind==fpga::CB_NODE_JOINT) {
                    scan(cb.dst_joint,joints,fpga::CB_NODE_DST);
                    scan(cb.local_joint,joints,fpga::CB_NODE_LOCAL);
                    scan(cb.joint_joint,joints,fpga::CB_NODE_JOINT);
                    scan(cb.src_joint,joints,fpga::CB_NODE_SRC);
                } else if(n.kind==fpga::CB_NODE_SRC) {
                    scan(cb.dst_src,jumps,fpga::CB_NODE_DST);
                    scan(cb.local_src,jumps,fpga::CB_NODE_LOCAL);
                    scan(cb.joint_src,jumps,fpga::CB_NODE_JOINT);
                }
            }
            for(auto [kind,id]:row->second)add(n.tile,kind,id,view);
            visitBackwardPhysicalSources(index,n.tile->coord,
                static_cast<fpga::CBNodeNameType>(n.kind),n.id,
                [&](fpga::Coord at,fpga::CBNodeNameType kind,int id) {
                    add(fpga::Device::current().getTile(at.x,at.y),kind,id,view);
                });
        }
        if(uncertain()) {
            // A release-only walk cannot lose a positive/unknown answer.
            // Retain regions only while they can supply a negative proof.
            states.clear();internal_inputs.clear();goals.clear();
            std::vector<Node>().swap(frontier);
        }
        return connected || limited;
    }
};
} // namespace pnr
