#pragma once

#include "Device.h"
#include "Docking.h"
#include "MovingSourceReachability.h"
#include "TraceRouteCommon.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <ostream>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pnr
{

// Trace from a mapped input toward a movable source using reverse topology.
// Called from Moving sources; bounded Docking remains in Docking.cpp.
struct TraceRouteBackward
{
    static constexpr int ROUTE_POS_TRANSIT = 1;
    const Docking *docking = nullptr;
    BackwardTakeoffRoute run(fpga::Tile &target, NodeMask pins,
                             fpga::Coord source_hint,
                             BackwardResolveIndex &index,
                             std::ostream *decisions);
    // Emit trace decisions without mixing formatting into recursive search.
    static void debug_trace(std::ostream *out, fpga::Tile &tile, int dst,
                            fpga::Coord source_hint);
    static void debug_probe(std::ostream *out, fpga::Tile &tile, int src,
                            bool accepted);
    static void debug_ingress(std::ostream *out, fpga::Tile &tile,
                              fpga::CBNodeNameType type, int node);

    // Name one typed node in a candidate input ingress chain.
    struct TerminalNode
    {
        fpga::Tile *tile;
        fpga::CBNodeNameType type;
        int node;
    };

    // Carry terminal nodes and the fixed suffix into the reverse source walk.
    struct TerminalIngress
    {
        fpga::Tile *tile;
        int dst;
        int pin;
        std::vector<TerminalNode> nodes;
        std::vector<fpga::Wire> suffix;
    };

    // Walk upstream from one terminal ingress with speculative leases.
    // The suffix may cross LOCAL links and joints outside the mapped input.
    struct IngressTrace
    {
        const Docking *docking = nullptr;

        // Restore every tile touched by this ingress attempt on return.
        struct Trial
        {
            std::vector<std::pair<fpga::Tile *, fpga::CBState>> saved;
            std::unordered_set<fpga::Tile *> touched;

            void remember(fpga::Tile &tile)
            {
                if (touched.insert(&tile).second)
                    saved.emplace_back(&tile, tile.cb);
            }

            ~Trial()
            {
                for (auto it = saved.rbegin(); it != saved.rend(); ++it)
                    it->first->cb.restoreLeases(it->second);
            }

        };

        // Try source arrivals, ordinary transit, then LOCAL relays in order.
        enum class Phase
        {
            source,
            ordinary,
            direct_local,
            local_chain,
            blocked
        };

        // Remember the next LOCAL in a same-tile relay chain.
        struct LocalStep
        {
            int node;
            NodeMask remaining;
            bool direct_checked = false;
        };

        using Row = decltype(std::declval<const fpga::CBType &>()
                                 .dst_local.values.begin());

        // Read the caller's snapshot when checking a preexisting blocker.
        const fpga::CBState &visible_state(fpga::Tile &tile,
                                           const Trial &trial) const
        {
            if (const auto *state = docking->driverStateOverride(tile))
                return *state;
            for (const auto &[saved_tile, state] : trial.saved)
                if (saved_tile == &tile)
                    return state;
            return tile.cb;
        }

        // Join the traversed reverse edges to the fixed terminal suffix.
        static std::vector<fpga::Wire>
        suffix(const std::vector<fpga::Wire> &reverse_edges,
               const TerminalIngress &ingress)
        {
            std::vector<fpga::Wire> path(reverse_edges.rbegin(),
                                         reverse_edges.rend());
            path.insert(path.end(), ingress.suffix.begin(),
                        ingress.suffix.end());
            return path;
        }

        // Retain the longest failed suffix for diagnostics and cancellation.
        static void retain_failure(BackwardTakeoffRoute &result,
                                   fpga::Tile &tile, int dst,
                                   const std::vector<fpga::Wire> &reverse_edges,
                                   const TerminalIngress &ingress);

        // Keep distinct occupied cuts for the caller's preemption diagnosis.
        static void retain_blocked(BackwardTakeoffRoute &result,
                                   const std::vector<fpga::RouteCutNode> &cuts)
        {
            if (cuts.empty())
                return;
            if (result.blocked_reverse_cut.empty())
            {
                result.blocked_reverse_cut = cuts;
                ++result.blocked_reverse_edges;
            }

            bool seen = false;
            for (const auto &item : result.blocked_reverse_frontier)
            {
                const auto &prior = item.provisional_cuts;
                if (prior.size() != cuts.size())
                    continue;
                bool same = true;
                for (size_t i = 0; i < cuts.size(); ++i)
                    if (prior[i].tile != cuts[i].tile ||
                        prior[i].type != cuts[i].type ||
                        prior[i].node != cuts[i].node)
                    {
                        same = false;
                        break;
                    }

                if (same)
                {
                    seen = true;
                    break;
                }

            }

            if (!seen)
            {
                DockingBridgeBlocker blocker;
                blocker.valid = true;
                blocker.tile = cuts.back().tile;
                blocker.provisional_cuts = cuts;
                result.blocked_reverse_frontier.push_back(std::move(blocker));
            }

        }

        // Append a typed edge in reverse traversal order.
        static void arc(std::vector<fpga::Wire> &reverse_edges,
                        fpga::Coord from, fpga::CBNodeNameType from_type,
                        int from_node, fpga::Coord to,
                        fpga::CBNodeNameType to_type, int to_node)
        {
            fpga::Wire wire;
            wire.type = fpga::Wire::WIRE_ROUTE_EDGE;
            wire.from = from;
            wire.to = to;
            wire.from_node_type = from_type;
            wire.from_node = from_node;
            wire.to_node_type = to_type;
            wire.to_node = to_node;
            reverse_edges.push_back(std::move(wire));
        }

        // Capture the lease and path boundary before a recursive child DST.
        struct Descent
        {
            bool &advanced;
            fpga::Tile *&child_tile;
            int &child_dst;
            fpga::Tile *&changed_tile;
            std::unique_ptr<fpga::CBState> &before;
            size_t &path_size;
            std::vector<fpga::Wire> &reverse_edges;

            void operator()(fpga::Tile &tile, int node)
            {
                changed_tile = &tile;
                before = std::make_unique<fpga::CBState>(tile.cb);
                path_size = reverse_edges.size();
                child_tile = &tile;
                child_dst = node;
                advanced = true;
            }

        };

        // Build one physical jump with the chosen takeoff or transit endpoint.
        struct EdgeBuilder
        {
            fpga::Tile &arrival_tile;
            int arrival_dst;
            const BackwardResolveSource &source;

            fpga::Wire operator()(int local, int joint, int joint2,
                                  bool takeoff_edge = false) const
            {
                auto &tile = *source.tile;
                fpga::Wire wire;
                wire.from = tile.coord;
                wire.to = arrival_tile.coord;
                wire.local = local;
                wire.jump = source.src;
                wire.route_jump = source.route_jump;
                wire.dst = arrival_dst;
                wire.joint = joint;
                wire.joint2 = joint2;
                wire.pos = takeoff_edge ? 0 : ROUTE_POS_TRANSIT;
                const auto type =
                    takeoff_edge ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
                wire.from_wire_name =
                    fromWireName(tile, type, local, source.src, joint, {});
                wire.src_wire_name =
                    srcWireName(tile, type, local, source.src, joint, {});
                if (const auto *name = arrival_tile.cb_type->nodeName(
                        fpga::CB_NODE_DST, arrival_dst))
                    wire.dst_wire_name = *name;
                return wire;
            }

        };

        // Build the reverse rank mask once from the DST's incoming SRC list.
        static void
        order_incoming(fpga::Tile &arrival_tile, fpga::Coord source_hint,
                       const std::vector<BackwardResolveSource> *incoming,
                       NodeMask &remaining_source_ranks,
                       std::array<uint32_t, 225> &first_by_rank,
                       std::vector<uint32_t> &next_source);
        // Pop the next angle-ranked incoming SRC in constant time.
        static const BackwardResolveSource *
        next_incoming(const std::vector<BackwardResolveSource> *incoming,
                      NodeMask &remaining_source_ranks,
                      std::array<uint32_t, 225> &first_by_rank,
                      const std::vector<uint32_t> &next_source);
        // Recurse into one upstream DST, then resume this call's cursors.
        bool next(fpga::Tile &arrival_tile, int arrival_dst,
                  fpga::Coord source_hint, BackwardResolveIndex &index,
                  std::ostream *decisions, BackwardTakeoffRoute &result,
                  bool &stopped, const TerminalIngress &ingress, Trial &trial,
                  std::vector<fpga::Wire> &reverse_edges);
        // Probe upstream sources and backtrack through one ingress chain.
        bool run(const TerminalIngress &ingress, fpga::Tile &target,
                 fpga::Coord source_hint, BackwardResolveIndex &index,
                 std::ostream *decisions, BackwardTakeoffRoute &result,
                 bool &stopped);
    };

    // Follow a longer typed chain until it reaches a source-connected DST.
    struct TypedIngressTrace
    {
        const Docking *docking = nullptr;
        IngressTrace *ingress_trace = nullptr;

        // Sort one call's free predecessors by direction and hop length.
        struct Candidate
        {
            TerminalNode node;
            uint16_t angle;
            int distance;

            bool operator<(const Candidate &other) const
            {
                return angle != other.angle ? angle < other.angle
                                            : distance < other.distance;
            }

            bool operator==(const Candidate &other) const
            {
                return node.tile == other.node.tile &&
                       node.type == other.node.type &&
                       node.node == other.node.node;
            }

        };

        // Represent one step of the typed ingress chain in route order.
        static fpga::Wire typed_edge(const TerminalNode &from,
                                     const TerminalNode &to)
        {
            fpga::Wire edge;
            edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
            edge.from = from.tile->coord;
            edge.to = to.tile->coord;
            edge.from_node_type = from.type;
            edge.from_node = from.node;
            edge.to_node_type = to.type;
            edge.to_node = to.node;
            return edge;
        }

        // Preserve the rejected terminal chain and the first blocked
        // predecessor.
        static void retain_typed_diagnostic(
            const TerminalNode &at, const TerminalNode *blocked,
            const std::vector<fpga::Wire> &reversed, fpga::Tile &target,
            int pin, BackwardTakeoffRoute &result)
        {
            if (reversed.empty() && !blocked)
                return;
            if (reversed.size() + (blocked ? 2 : 1) <
                result.diagnostic_fragments.size())
                return;
            result.diagnostic_fragments.assign(reversed.rbegin(),
                                               reversed.rend());
            if (blocked)
                result.diagnostic_fragments.insert(
                    result.diagnostic_fragments.begin(),
                    typed_edge(*blocked, at));
            fpga::Wire endpoint;
            endpoint.type = fpga::Wire::WIRE_TILE_PIN;
            endpoint.from = endpoint.to = target.coord;
            endpoint.local = pin;
            endpoint.pin_dir = fpga::TILE_PIN_INPUT;
            result.diagnostic_fragments.push_back(endpoint);
            const auto &failure = blocked ? *blocked : at;
            result.failure_tile = failure.tile;
            result.failure_dst =
                failure.type == fpga::CB_NODE_DST ? failure.node : -1;
        }

        // Avoid retrying a complete short ingress handled by terminalEntries.
        static bool tried_compact_path(int dst, const TerminalIngress &ingress,
                                       fpga::Tile &target, int pin)
        {
            if (ingress.nodes.size() > 3)
                return false;
            for (const auto &node : ingress.nodes)
                if (node.tile != &target)
                    return false;
            for (size_t i = 1; i < ingress.nodes.size(); ++i)
                if (ingress.nodes[i].type != fpga::CB_NODE_JOINT)
                    return false;
            const int joint =
                ingress.nodes.size() > 1 ? ingress.nodes[1].node : -1;
            const int joint2 =
                ingress.nodes.size() > 2 ? ingress.nodes[2].node : -1;
            for (const auto &entry : target.cb_type->terminalEntries(pin))
                if (entry.dst == dst && entry.joint == joint &&
                    entry.joint2 == joint2)
                    return true;
            return false;
        }

        // Collect a predecessor for this recursive call's ordered cursor.
        void consider(const TerminalNode &current, fpga::Tile *tile,
                      fpga::CBNodeNameType type, int node,
                      std::vector<Candidate> &choices, TerminalNode &blocked,
                      int &blocked_angle, int &blocked_distance,
                      fpga::Tile &target, fpga::Coord source_hint, int pin,
                      const TerminalIngress &ingress,
                      const std::unordered_set<BackwardLocalKey,
                                               BackwardLocalKeyHash> &seen)
        {
            if (!tile || !tile->cb_type || node < 0)
                return;
            // A compact suffix already had its own attempt. Only that complete
            // DST/JOINT sequence is skipped; a shared JOINT may continue
            // farther.
            if (type == fpga::CB_NODE_DST &&
                tried_compact_path(node, ingress, target, pin))
                return;
            if (seen.contains({tile->coord.x, tile->coord.y, type, node}))
                return;
            const int angle = fpga::CBType::anglePriorityRank(
                source_hint - current.tile->coord,
                tile->coord - current.tile->coord);
            const int distance =
                std::abs(tile->coord.x - current.tile->coord.x) +
                std::abs(tile->coord.y - current.tile->coord.y);
            const auto &cb = docking->driverState(*tile);
            const bool busy =
                type == fpga::CB_NODE_LOCAL
                    ? cb.local.local.testBit(node) ||
                          tile->isPinNodeLeased(node) ||
                          ((tile != &target || node != pin) &&
                           inputLocalReserved(*tile, node))
                : type == fpga::CB_NODE_JOINT
                    ? cb.joint.jump.testBit(node)
                    : type == fpga::CB_NODE_DST && cb.dst.jump.testBit(node);
            if (busy)
            {
                if (angle < blocked_angle ||
                    (angle == blocked_angle && distance < blocked_distance))
                {
                    blocked = {tile, type, node};
                    blocked_angle = angle;
                    blocked_distance = distance;
                }

                return;
            }

            choices.push_back(
                {{tile, type, node}, static_cast<uint16_t>(angle), distance});
        }

        // Feed physical LOCAL predecessors into the same ranked choice.
        struct PhysicalSourceVisitor
        {
            TypedIngressTrace &trace;
            const TerminalNode &current;
            std::vector<Candidate> &choices;
            TerminalNode &blocked;
            int &blocked_angle;
            int &blocked_distance;
            fpga::Tile &target;
            fpga::Coord source_hint;
            int pin;
            const TerminalIngress &ingress;
            const std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash>
                &seen;

            void operator()(fpga::Coord at, fpga::CBNodeNameType type,
                            int node) const
            {
                if (type == fpga::CB_NODE_LOCAL)
                    trace.consider(current,
                                   fpga::Device::current().getTile(at.x, at.y),
                                   type, node, choices, blocked, blocked_angle,
                                   blocked_distance, target, source_hint, pin,
                                   ingress, seen);
            }

        };

        // Each call resumes its ordered predecessors after a failed child.
        bool
        trace(fpga::Tile &target, fpga::Coord source_hint,
              BackwardResolveIndex &index, std::ostream *decisions,
              BackwardTakeoffRoute &result, bool &stopped, int pin,
              TerminalIngress &ingress, std::vector<fpga::Wire> &reversed,
              std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> &seen);
        // Advance one typed predecessor at a time for this input pin.
        bool run(fpga::Tile &target, fpga::Coord source_hint,
                 BackwardResolveIndex &index, std::ostream *decisions,
                 BackwardTakeoffRoute &result, bool &stopped, int pin);
    };

};

// Exact physical predecessors for the numeric reverse trace. LOCAL links
// retain their literal grid coordinates.
void visitBackwardPhysicalSources(
    BackwardResolveIndex &index, fpga::Coord at, fpga::CBNodeNameType type,
    int node, const fpga::RepairIncomingSourceVisitor &visit);

// Direction-guided relocation trace: keep the first free prioritized hop.
// No frontier search, input proofs, expansion budget, or tile-visit limit.
BackwardTakeoffRoute
traceBackwardToTakeoff(fpga::Tile &target, NodeMask pins,
                       fpga::Coord source_hint, BackwardResolveIndex &index,
                       const BackwardTakeoffProbe &probe,
                       const BackwardTakeoffCancel &cancel = {},
                       const BackwardTakeoffStateView *state_view = nullptr,
                       std::ostream *decisions = nullptr);

} // namespace pnr
