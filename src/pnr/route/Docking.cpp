#include "Docking.h"
#include "Device.h"
#include "InputRepairBudget.h"
#include "RepairStatistics.h"
#include "RouteDiagnostics.h"
#include "RoutePreemption.h"
#include "RouteSearch.h"
#include "RouteSource.h"
#include "RouteTaskTiming.h"
#include "Tile.h"
#include "TraceRouteBackward.h"
#include "TraceRouteCommon.h"
#include "TraceRouteForward.h"
#include "Wire.h"

#include <absl/container/flat_hash_map.h>
#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <map>
#include <memory>
#include <ostream>
#include <queue>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace pnr
{
namespace
{

constexpr int ROUTE_POS_SOURCE = 0;
constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;

bool inDockWindow(const fpga::Coord &coord, const fpga::Coord &target,
                  int radius)
{
    return std::abs(coord.x - target.x) <= radius &&
           std::abs(coord.y - target.y) <= radius;
}

bool canLeaseIn(const fpga::CBState &cb, int dst, int local, int joint,
                int joint2)
{
    if (cb.dst.jump.testBit(dst) || cb.local.local.testBit(local) ||
        (joint >= 0 && cb.joint.jump.testBit(joint)) ||
        (joint2 >= 0 && cb.joint.jump.testBit(joint2)))
    {
        return false;
    }

    return true;
}

using Key = BackwardResolveKey;

struct Node
{
    fpga::Tile *tile = nullptr;
    int dst = -1;
    fpga::InternedString dst_wire;
    int parent = -1;
    fpga::Wire edge_from_parent;
    int depth = 0;
    std::vector<fpga::Wire> target_suffix;
    bool blocked_terminal = false;
    int terminal_pin = -1;
    int terminal_joint = -1;
    int terminal_joint2 = -1;
    int seed_dst = -1;
    int tile_visits_root = -1;
    // A numeric intra-CB bridge may contain LOCAL nodes, which cannot be
    // represented by the ordinary DST/joint/joint/SRC hop fields.
    std::vector<fpga::Wire> internal_prefix;
};

struct TakeoffProbeKey
{
    fpga::Tile *tile = nullptr;
    int src = -1;

    bool operator==(const TakeoffProbeKey &) const = default;
};

struct TakeoffProbeKeyHash
{
    size_t operator()(const TakeoffProbeKey &key) const
    {
        size_t tile = std::hash<fpga::Tile *>{}(key.tile);
        return tile ^ (std::hash<int>{}(key.src) + 0x9e3779b9U + (tile << 6) +
                       (tile >> 2));
    }

};

using BackwardSource = BackwardResolveSource;

struct DockingOptions
{
    int max_depth = 5;
    int radius = fpga::GROUNDING_RADIUS;
    bool trace_backward_attempts = false;
    NodeMask reserved_terminal_joints{};
    BackwardResolveIndex *prebuilt_backward_index = nullptr;
    // Local coverage is tried separately before admitting outside predecessors.
    bool bounded_reverse = false;
    bool source_local = false;
    const DockingPreemptionSearch *preemption = nullptr;
    bool forward_preemption = false;
};

struct PendingBridgeBlocker
{
    Key forward_key;
    Key backward_key;
    int backward_index = -1;
    fpga::Wire edge;
    bool dst_busy = false;
    bool src_busy = false;
    bool joint_busy = false;
    bool joint2_busy = false;
    bool from_forward = false;
};

Key nodeKey(const Node &node)
{
    return Key{node.tile->coord.x, node.tile->coord.y, node.dst};
}

std::vector<fpga::Wire> pathToNode(const std::vector<Node> &nodes, int index)
{
    std::vector<fpga::Wire> result;
    for (int curr = index; curr >= 0 && nodes[curr].parent >= 0;
         curr = nodes[curr].parent)
    {
        result.push_back(nodes[curr].edge_from_parent);
    }

    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<fpga::Wire> suffixFromNode(const std::vector<Node> &nodes,
                                       int index)
{
    std::vector<fpga::Wire> result;
    int root = index;
    for (int curr = index; curr >= 0 && nodes[curr].parent >= 0;
         curr = nodes[curr].parent)
    {
        result.insert(result.end(), nodes[curr].internal_prefix.begin(),
                      nodes[curr].internal_prefix.end());
        result.push_back(nodes[curr].edge_from_parent);
        root = nodes[curr].parent;
    }

    if (root >= 0 && static_cast<size_t>(root) < nodes.size())
    {
        result.insert(result.end(), nodes[root].target_suffix.begin(),
                      nodes[root].target_suffix.end());
    }

    markGraphArrivals(result);
    return result;
}

DockingResult dockGroundingImpl(fpga::Tile &forward_tile, int forward_dst,
                                const std::string &forward_dst_wire,
                                fpga::Tile &target_tile, NodeMask pin_nodes,
                                const DockingOptions &options)
{
    DockingResult result;
    if (!forward_tile.cb_type || !target_tile.cb_type || forward_dst < 0 ||
        pin_nodes == NodeMask{})
    {
        return result;
    }

    std::vector<Node> forward_nodes;
    std::vector<Node> backward_nodes;
    CombinatorialTileVisits tile_visits;
    // Count only the path explored by Docking. The angle-led caller may
    // revisit a Tile before reaching this anchor.
    int forward_root = -1;
    if (!tile_visits.append(forward_root, &forward_tile, forward_root))
    {
        ++result.tile_visit_reject_count;
        return result;
    }

    auto path_valid = [&](const std::vector<fpga::Wire> &path)
    {
        const size_t checkpoint = tile_visits.checkpoint();
        int root = forward_root;
        fpga::Coord last = forward_tile.coord;
        bool valid = true;
        for (const auto &wire : path)
        {
            for (auto coord : {wire.from, wire.to})
            {
                if (coord == last)
                    continue;
                if (!tile_visits.append(root, coord, root))
                {
                    valid = false;
                    break;
                }

                last = coord;
            }

            if (!valid)
                break;
        }

        tile_visits.rewind(checkpoint);
        return valid && (!options.preemption ||
                         options.preemption->accept_complete(path));
    };

    // The reverse suffix must fit the path explored by Docking. Its anchor
    // Tile coincides with the forward search's root.
    auto backward_fits_forward = [&](int root, fpga::Coord coord)
    {
        return tile_visits.count(forward_root, coord) +
                   tile_visits.count(root, coord) <=
               COMBINATORIAL_TILE_VISIT_LIMIT +
                   (coord == forward_tile.coord ? 1u : 0u);
    };

    std::deque<int> forward_queue;
    std::deque<int> backward_queue;
    std::unordered_map<Key, int, BackwardResolveKeyHash> forward_seen;
    std::unordered_map<Key, int, BackwardResolveKeyHash> backward_seen;
    std::vector<PendingBridgeBlocker> pending_bridge_blockers;
    std::unordered_set<Key, BackwardResolveKeyHash> forward_blocked_landings;
    BackwardResolveIndex local_backward_index;
    BackwardResolveIndex *backward_index = options.prebuilt_backward_index;
    bool backward_sources_built = false;

    // Preserve each completed backward phase and consume its exact blockers
    // before the phase-local node indexes are rebuilt for another seed class.
    auto finish_frontiers = [&]()
    {
        // Provisional search validates complete paths directly; it does not
        // need the legacy post-search, single-bridge materialization
        // diagnostics.
        if (options.preemption)
            return;
        auto remember_frontier = [](std::vector<DockingFrontierNode> &frontier,
                                    const DockingFrontierNode &node)
        {
            auto recent = recentDockingEntries(frontier);
            auto existing =
                std::find_if(recent.begin(), recent.end(),
                             [&](const DockingFrontierNode &old)
                             {
                                 return old.coord.x == node.coord.x &&
                                        old.coord.y == node.coord.y &&
                                        old.dst == node.dst;
                             });
            if (existing == recent.end())
            {
                frontier.push_back(node);
            }

            else if (node.depth < existing->depth)
            {
                existing->depth = node.depth;
            }

        };

        // forward_seen already contains each numeric state once. The second
        // backward phase does not expand it: replace the snapshot in linear
        // time.
        result.forward_frontier.clear();
        result.forward_frontier.reserve(forward_seen.size());
        result.backward_frontier.reserve(result.backward_frontier.size() +
                                         backward_seen.size());
        for (const auto &[key, index] : forward_seen)
        {
            result.forward_frontier.push_back(DockingFrontierNode{
                {key.x, key.y},
                key.dst,
                index >= 0 ? forward_nodes[index].depth : 0});
        }

        const bool first_backward_phase = result.backward_frontier.empty();
        for (const auto &[key, index] : backward_seen)
        {
            DockingFrontierNode node{{key.x, key.y},
                                     key.dst,
                                     index >= 0 ? backward_nodes[index].depth
                                                : 0};
            // The first snapshot is unique too. Only later seed phases can
            // overlap.
            if (first_backward_phase)
                result.backward_frontier.push_back(node);
            else
                remember_frontier(result.backward_frontier, node);
        }

        for (const PendingBridgeBlocker &pending : pending_bridge_blockers)
        {
            auto front = forward_seen.find(pending.forward_key);
            int backward_index = pending.backward_index;
            if (backward_index < 0)
            {
                auto back = backward_seen.find(pending.backward_key);
                if (back != backward_seen.end())
                {
                    backward_index = back->second;
                }

            }

            if (backward_index < 0 ||
                static_cast<size_t>(backward_index) >= backward_nodes.size() ||
                (front == forward_seen.end() && pending.from_forward))
            {
                continue;
            }

            auto recent_bridges = recentDockingEntries(result.blocked_bridges);
            bool duplicate = std::any_of(
                recent_bridges.begin(), recent_bridges.end(),
                [&](const DockingBridgeBlocker &bridge)
                {
                    return bridge.tile.x == pending.edge.from.x &&
                           bridge.tile.y == pending.edge.from.y &&
                           bridge.dst == pending.edge.local &&
                           bridge.src == pending.edge.jump &&
                           bridge.joint == pending.edge.joint &&
                           bridge.joint2 == pending.edge.joint2 &&
                           bridge.landing_tile.x == pending.edge.to.x &&
                           bridge.landing_tile.y == pending.edge.to.y &&
                           bridge.landing_dst == pending.edge.dst;
                });
            if (duplicate)
            {
                continue;
            }

            DockingBridgeBlocker bridge;
            bridge.valid = true;
            bridge.joins_frontiers = front != forward_seen.end();
            bridge.blocked_terminal =
                backward_nodes[backward_index].blocked_terminal;
            bridge.tile = pending.edge.from;
            bridge.dst = pending.edge.local;
            bridge.src = pending.edge.jump;
            bridge.joint = pending.edge.joint;
            bridge.joint2 = pending.edge.joint2;
            bridge.landing_tile = pending.edge.to;
            bridge.landing_dst = pending.edge.dst;
            bridge.dst_busy = pending.dst_busy;
            bridge.src_busy = pending.src_busy;
            bridge.joint_busy = pending.joint_busy;
            bridge.joint2_busy = pending.joint2_busy;
            if (bridge.joins_frontiers)
            {
                bridge.forward_prefix =
                    pathToNode(forward_nodes, front->second);
            }

            bridge.backward_suffix = {pending.edge};
            std::vector<fpga::Wire> suffix =
                suffixFromNode(backward_nodes, backward_index);
            bridge.backward_suffix.insert(bridge.backward_suffix.end(),
                                          suffix.begin(), suffix.end());
            if (bridge.joins_frontiers)
            {
                auto path = bridge.forward_prefix;
                path.insert(path.end(), bridge.backward_suffix.begin(),
                            bridge.backward_suffix.end());
                if (!path_valid(path))
                {
                    ++result.tile_visit_reject_count;
                    continue; // Never propose cutting a victim for a cyclic
                              // replacement.
                }

            }

            result.blocked_bridges.push_back(std::move(bridge));
        }

        pending_bridge_blockers.clear();
    };

    // Keep complete backward alternatives only for a focused diagnostic run.
    auto record_backward_attempt =
        [&]<typename MakeFragments>(int seed_dst, const char *status,
                                    MakeFragments &&make_fragments)
    {
        if (!options.trace_backward_attempts)
        {
            return;
        }

        result.backward_attempts.push_back(
            DockingBackwardAttempt{seed_dst, status, make_fragments()});
    };

    Node forward_seed{
        &forward_tile, forward_dst, forward_dst_wire, -1, {}, 0, {}};
    forward_seed.tile_visits_root = forward_root;
    forward_nodes.push_back(std::move(forward_seed));
    // A LOCAL is not a DST, even when their numeric identifiers coincide.
    // Do not register a LOCAL under a DST key. Its legal Takeoff transitions
    // still seed the forward half; backward search can also reach it directly.
    if (!options.source_local)
    {
        forward_seen[nodeKey(forward_nodes.front())] = 0;
    }

    forward_queue.push_back(0);

    std::vector<Node> blocked_seeds;
    pin_nodes.for_each_set_bit(
        [&](int pin)
        {
            if (target_tile.isPinNodeLeased(pin))
            {
                if (RouteCongestionTrace::current)
                    RouteCongestionTrace::current->blocked(
                        target_tile, "docking_pin_leased",
                        {{fpga::CB_NODE_LOCAL, pin}});
                return false;
            }

            for (const fpga::CBType::TerminalEntry &path :
                 pnr::Docking::terminalEntries(*(target_tile.cb_type), pin))
            {
                int dst = path.dst;
                ++result.target_entry_count;
                fpga::Wire enter;
                enter.from = target_tile.coord;
                enter.to = target_tile.coord;
                enter.local = dst;
                enter.joint = path.joint;
                enter.joint2 = path.joint2;
                enter.pos = ROUTE_POS_TRANSIT;
                if (const std::string *name =
                        target_tile.cb_type->nodeName(fpga::CB_NODE_DST, dst))
                {
                    enter.dst_wire_name = *name;
                }

                fpga::Wire tile_pin;
                tile_pin.type = fpga::Wire::WIRE_TILE_PIN;
                tile_pin.from = target_tile.coord;
                tile_pin.to = target_tile.coord;
                tile_pin.local = pin;
                tile_pin.pos = ROUTE_POS_TRANSIT;
                std::vector<fpga::Wire> suffix{enter, tile_pin};
                const bool reserved_joint =
                    options.reserved_terminal_joints.testBit(path.joint) ||
                    options.reserved_terminal_joints.testBit(path.joint2);
                bool terminal_busy =
                    reserved_joint || !canLeaseIn(target_tile.cb, dst, pin,
                                                  path.joint, path.joint2);
                if (options.preemption && terminal_busy)
                {
                    // Reserved packing resources and endpoint owners are never
                    // bypassed.
                    if (reserved_joint ||
                        !options.preemption->eligible_edge(enter))
                        continue;
                    terminal_busy = false;
                }

                Node seed{&target_tile,
                          dst,
                          enter.dst_wire_name,
                          -1,
                          {},
                          0,
                          suffix,
                          terminal_busy,
                          pin,
                          path.joint,
                          path.joint2};
                seed.seed_dst = dst;
                if (!tile_visits.append(-1, &target_tile,
                                        seed.tile_visits_root))
                {
                    return false;
                }

                if (!backward_fits_forward(seed.tile_visits_root,
                                           target_tile.coord))
                {
                    ++result.tile_visit_reject_count;
                    continue;
                }

                if (terminal_busy)
                {
                    if (RouteCongestionTrace::current)
                        RouteCongestionTrace::current->blocked(
                            target_tile, "docking_terminal_busy",
                            {{fpga::CB_NODE_DST, dst},
                             {fpga::CB_NODE_LOCAL, pin},
                             {fpga::CB_NODE_JOINT, path.joint},
                             {fpga::CB_NODE_JOINT, path.joint2}});
                    ++result.target_busy_count;
                    record_backward_attempt(dst, "blocked_seed",
                                            [&]() { return suffix; });
                    blocked_seeds.push_back(std::move(seed));
                    continue;
                }

                record_backward_attempt(dst, "seed", [&]() { return suffix; });
                Key key = nodeKey(seed);
                if (backward_seen.find(key) == backward_seen.end())
                {
                    int seed_index = static_cast<int>(backward_nodes.size());
                    backward_nodes.push_back(seed);
                    backward_seen[key] = seed_index;
                    backward_queue.push_back(seed_index);
                    ++result.target_seed_count;
                }

                if (auto it = forward_seen.find(key); it != forward_seen.end())
                {
                    result.fragments = pathToNode(forward_nodes, it->second);
                    result.fragments.insert(result.fragments.end(),
                                            suffix.begin(), suffix.end());
                    if (!path_valid(result.fragments))
                    {
                        ++result.tile_visit_reject_count;
                        result.fragments.clear();
                        continue;
                    }

                    result.success = true;
                    record_backward_attempt(dst, "meet",
                                            [&]() { return suffix; });
                    return result.success;
                }

            }

            return result.success;
        });
    if (result.success)
    {
        finish_frontiers();
        return result;
    }

    // Build the reverse jump index once per docking attempt. Backward nodes
    // then look up incoming transitions without rescanning the whole window.
    auto build_backward_sources = [&]()
    {
        if (backward_sources_built)
        {
            return;
        }

        backward_sources_built = true;
        if (!backward_index)
        {
            fpga::Device &device = fpga::Device::current();
            // Production supplies the once-built device topology. Standalone
            // grounding needs the same complete incoming graph: limiting the
            // source scan to the target window loses long wires entering that
            // window.
            const bool bounded = options.bounded_reverse;
            const fpga::Coord center =
                bounded ? target_tile.coord
                        : fpga::Coord{device.size_width / 2,
                                      device.size_height / 2};
            const int radius =
                bounded ? options.radius
                        : std::max(device.size_width, device.size_height);
            local_backward_index =
                buildBackwardResolveIndex(device, center, radius);
            backward_index = &local_backward_index;
        }

        result.backward_mapping_scan_count = backward_index->mapping_scan_count;
        result.backward_mapping_reaches_count =
            backward_index->mapping_reaches_count;
    };

    auto expand_forward = [&](int node_index) -> bool
    {
        if (!InputRepairBudget::expand())
            return false;
        ++result.forward_pop_count;
        Node node = forward_nodes[node_index];
        const bool takeoff = options.source_local && node_index == 0;
        const auto from_type =
            takeoff ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
        if (node.depth >= options.max_depth ||
            !inDockWindow(node.tile->coord, target_tile.coord, options.radius))
        {
            return false;
        }

        const std::vector<uint16_t> *srcs =
            pnr::Docking::srcNodes(*(node.tile->cb_type), from_type, node.dst);
        if (!srcs)
        {
            return false;
        }

        struct ForwardCandidate
        {
            uint16_t src = 0;
            int joint = -2;
            int joint2 = -1;
            fpga::InternedString src_wire;
            fpga::TileJumpTarget target;
            int tile_visits_root = -1;
        };

        std::vector<ForwardCandidate> candidates;
        candidates.reserve(srcs->size());
        // Bounded docking examines every numeric exit in the crossbar.
        for (uint16_t src : *srcs)
        {
            // Forward Takeoff is a square-bounded completion search, not a
            // Moving trace. Ordinary forward Grounding still filters the
            // persistent SRC mask.
            if (TraceRouteForward::isDeadend(*node.tile, src,
                                             target_tile.coord))
            {
                continue;
            }

            int joint2 = -1;
            int joint =
                selectJointToSrc(*node.tile, from_type, node.dst, src, &joint2);
            if (joint == -2)
            {
                if (RouteCongestionTrace::current)
                    RouteCongestionTrace::current->blocked(
                        *node.tile, "docking_joint_unavailable",
                        {{from_type, node.dst}, {fpga::CB_NODE_SRC, src}});
                continue;
            }

            fpga::InternedString src_wire = srcWireName(
                *node.tile, from_type, node.dst, src, joint, node.dst_wire);
            for (const fpga::TileJumpTarget &target :
                 fpga::Device::current().resolveJumpTargets(*node.tile, src))
            {
                if (!target.tile || !target.tile->cb_type ||
                    target.dst_node < 0 ||
                    (node.tile->cb.src_deadend.jump.testBit(src) &&
                     target.tile->coord != target_tile.coord) ||
                    !inDockWindow(target.tile->coord, target_tile.coord,
                                  options.radius))
                {
                    continue;
                }

                // A same-CB continuation does not leave/re-enter the tile.
                int candidate_visits_root = node.tile_visits_root;
                if (target.tile != node.tile &&
                    !tile_visits.append(node.tile_visits_root, target.tile,
                                        candidate_visits_root))
                {
                    ++result.tile_visit_reject_count;
                    continue;
                }

                const bool allow_existing_dst = node.parent < 0;
                const bool dst_busy = !allow_existing_dst &&
                                      node.tile->cb.dst.jump.testBit(node.dst);
                const bool src_busy = node.tile->cb.src.jump.testBit(src);
                const bool joint_busy =
                    joint >= 0 && node.tile->cb.joint.jump.testBit(joint);
                const bool joint2_busy =
                    joint2 >= 0 && node.tile->cb.joint.jump.testBit(joint2);
                if (dst_busy || src_busy || joint_busy || joint2_busy)
                {
                    if (RouteCongestionTrace::current)
                        RouteCongestionTrace::current->blocked(
                            *node.tile, "docking_forward_busy",
                            {{from_type, node.dst},
                             {fpga::CB_NODE_SRC, src},
                             {fpga::CB_NODE_JOINT, joint},
                             {fpga::CB_NODE_JOINT, joint2}});
                    // Preserve the numeric fast rejection in ordinary searches.
                    // A busy Takeoff has no legacy bridge record and needs
                    // named Wire storage only when provisional forward
                    // expansion will actually inspect it.
                    if (takeoff &&
                        (!options.preemption || !options.forward_preemption))
                    {
                        continue;
                    }

                    fpga::Wire edge;
                    edge.from = node.tile->coord;
                    edge.to = target.tile->coord;
                    edge.local = node.dst;
                    edge.jump = src;
                    edge.route_jump = target.jump_node;
                    edge.dst = target.dst_node;
                    edge.joint = joint;
                    edge.joint2 = joint2;
                    edge.pos = takeoff ? 0 : ROUTE_POS_TRANSIT;
                    edge.from_wire_name = node.dst_wire;
                    edge.src_wire_name = src_wire;
                    edge.dst_wire_name = target.dst_wire;
                    // The provisional pass must grow BOTH frontiers through
                    // eligible transit obstructions. Otherwise a blocked first
                    // exit turns the bidirectional search into a reverse-only
                    // search with half its intended reach. Leases are
                    // unchanged: accept_complete validates the entire
                    // replacement and the caller applies cuts atomically later.
                    if (!options.preemption || !options.forward_preemption ||
                        !options.preemption->eligible_edge(edge))
                    {
                        // Legacy bridge records describe DST->SRC, not output
                        // LOCALs.
                        if (!takeoff)
                        {
                            pending_bridge_blockers.push_back(
                                PendingBridgeBlocker{
                                    nodeKey(node),
                                    Key{target.tile->coord.x,
                                        target.tile->coord.y, target.dst_node},
                                    -1, edge, dst_busy, src_busy, joint_busy,
                                    joint2_busy, true});
                            forward_blocked_landings.insert(
                                Key{target.tile->coord.x, target.tile->coord.y,
                                    target.dst_node});
                        }

                        continue;
                    }

                }

                candidates.push_back(ForwardCandidate{src, joint, joint2,
                                                      src_wire, target,
                                                      candidate_visits_root});
            }

        }

        for (const ForwardCandidate &candidate : candidates)
        {
            fpga::Wire edge;
            edge.from = node.tile->coord;
            edge.to = candidate.target.tile->coord;
            edge.local = node.dst;
            edge.jump = candidate.src;
            edge.route_jump = candidate.target.jump_node;
            edge.dst = candidate.target.dst_node;
            edge.joint = candidate.joint;
            edge.joint2 = candidate.joint2;
            edge.pos = takeoff ? 0 : ROUTE_POS_TRANSIT;
            edge.from_wire_name = node.dst_wire;
            edge.src_wire_name = candidate.src_wire;
            edge.dst_wire_name = candidate.target.dst_wire;
            Node next{candidate.target.tile,
                      candidate.target.dst_node,
                      candidate.target.dst_wire,
                      node_index,
                      edge,
                      node.depth + 1,
                      {}};
            next.tile_visits_root = candidate.tile_visits_root;
            Key key = nodeKey(next);
            if (forward_seen.find(key) != forward_seen.end())
            {
                continue;
            }

            int next_index = static_cast<int>(forward_nodes.size());
            forward_nodes.push_back(next);
            const bool terminal_only =
                node.tile->cb.src_deadend.jump.testBit(candidate.src);
            if (!terminal_only)
            {
                forward_seen[key] = next_index;
                forward_queue.push_back(next_index);
            }

            ++result.forward_push_count;
            if (auto back = backward_seen.find(key);
                back != backward_seen.end() &&
                (!terminal_only || backward_nodes[back->second].depth == 0))
            {
                result.fragments = pathToNode(forward_nodes, next_index);
                std::vector<fpga::Wire> suffix =
                    suffixFromNode(backward_nodes, back->second);
                result.fragments.insert(result.fragments.end(), suffix.begin(),
                                        suffix.end());
                if (!path_valid(result.fragments))
                {
                    ++result.tile_visit_reject_count;
                    result.fragments.clear();
                    continue;
                }

                result.success = true;
                return true;
            }

        }

        return false;
    };

    auto expand_backward = [&](int node_index) -> bool
    {
        // Only the square-local Grounding pass ignores the tracing heuristic.
        // Its wider predecessor pass and all lease conflicts remain restricted.
        auto can_use_jump = [&](const fpga::CBState &cb, int dst, int src,
                                int joint, int joint2,
                                bool allow_existing_dst = false)
        {
            return canUsePhysicalJump(cb, dst, src, joint, joint2,
                                      allow_existing_dst);
        };

        if (!InputRepairBudget::expand())
            return false;
        ++result.backward_pop_count;
        Node node = backward_nodes[node_index];

        Key current_key = nodeKey(node);
        if (node.depth >= options.max_depth)
        {
            record_backward_attempt(
                node.seed_dst, "depth_limit",
                [&]() { return suffixFromNode(backward_nodes, node_index); });
            return false;
        }

        build_backward_sources();
        if (TraceRouteBackward::isDeadend(*node.tile, node.dst,
                                          forward_tile.coord, *backward_index))
            return false;
        const std::vector<BackwardResolveSource> *sources =
            resolveBackwardSources(*backward_index, current_key,
                                   !options.bounded_reverse);
        if (!sources)
        {
            record_backward_attempt(
                node.seed_dst, "no_incoming",
                [&]() { return suffixFromNode(backward_nodes, node_index); });
            return false;
        }

        size_t source_count = sources->size();

        for (size_t source_index = 0; source_index < source_count;
             ++source_index)
        {
            const BackwardSource &source = (*sources)[source_index];
            int src = source.src;
            fpga::Tile *prev_tile = source.tile;
            if (node.tile->cb.dst_deadend.jump.testBit(node.dst) &&
                (!prev_tile || prev_tile->coord != forward_tile.coord))
                continue;
            if (!prev_tile || !prev_tile->cb_type)
            {
                continue;
            }

            // Explicit/non-shared indexes may contain more than the local
            // window.
            if (options.bounded_reverse &&
                !inDockWindow(prev_tile->coord, target_tile.coord,
                              options.radius))
            {
                continue;
            }

            int previous_visits_root = node.tile_visits_root;
            if (prev_tile != node.tile &&
                !tile_visits.append(node.tile_visits_root, prev_tile,
                                    previous_visits_root))
            {
                ++result.tile_visit_reject_count;
                continue;
            }

            if (!backward_fits_forward(previous_visits_root, prev_tile->coord))
            {
                ++result.tile_visit_reject_count;
                continue;
            }

            prev_tile->cb_type->ensureDerivedMasks();
            if (options.source_local && prev_tile == &forward_tile)
            {
                int joint2 = -1;
                int joint = selectJointToSrc(*prev_tile, fpga::CB_NODE_LOCAL,
                                             forward_dst, src, &joint2);
                // The requested output LOCAL may already drive sibling fanouts.
                // Only the new SRC and any intermediate JOINTs must be free.
                if (joint != -2 && (options.preemption ||
                                    can_use_jump(prev_tile->cb, forward_dst,
                                                 src, joint, joint2, true)))
                {
                    fpga::Wire takeoff;
                    takeoff.from = prev_tile->coord;
                    takeoff.to = node.tile->coord;
                    takeoff.local = forward_dst;
                    takeoff.jump = src;
                    takeoff.route_jump = source.route_jump;
                    takeoff.dst = node.dst;
                    takeoff.joint = joint;
                    takeoff.joint2 = joint2;
                    takeoff.pos = 0;
                    takeoff.from_wire_name = forward_dst_wire;
                    takeoff.src_wire_name =
                        srcWireName(*prev_tile, fpga::CB_NODE_LOCAL,
                                    forward_dst, src, joint, {});
                    takeoff.dst_wire_name = node.dst_wire;
                    std::vector<fpga::Wire> joined{takeoff};
                    auto suffix = suffixFromNode(backward_nodes, node_index);
                    joined.insert(joined.end(), suffix.begin(), suffix.end());
                    if ((!options.preemption ||
                         can_use_jump(prev_tile->cb, forward_dst, src, joint,
                                      joint2, true) ||
                         options.preemption->eligible_edge(takeoff)) &&
                        path_valid(joined))
                    {
                        record_backward_attempt(node.seed_dst,
                                                "meet_source_local",
                                                [&]() { return joined; });
                        if (node.blocked_terminal)
                        {
                            result.blocked_terminal_reachable = true;
                            result.blocked_terminal_fragments =
                                std::move(joined);
                            result.blocked_dst = node.seed_dst;
                            result.blocked_pin = node.terminal_pin;
                            result.blocked_joint = node.terminal_joint;
                            result.blocked_joint2 = node.terminal_joint2;
                        }

                        else
                        {
                            result.fragments = std::move(joined);
                            result.success = true;
                        }

                        return true;
                    }

                    ++result.tile_visit_reject_count;
                }

            }

            if (node.tile->cb.dst_deadend.jump.testBit(node.dst))
                continue;
            NodeMask prev_dsts =
                prev_tile->cb_type->dsts_reaching_src.get(src).jump;
            if (prev_dsts == NodeMask{})
            {
                ++result.backward_missing_prev_dst_count;
                record_backward_attempt(
                    node.seed_dst, "no_previous_dst", [&]()
                    { return suffixFromNode(backward_nodes, node_index); });
                continue;
            }

            prev_dsts.for_each_set_bit(
                [&](int prev_dst)
                {
                    const Key key{prev_tile->coord.x, prev_tile->coord.y,
                                  prev_dst};
                    // A complete join must still be tried even if the numeric
                    // state was seen before. Other duplicates/full layers
                    // cannot advance the search: reject them before
                    // constructing paths or consulting victim owners.
                    if (options.preemption && !forward_seen.contains(key))
                    {
                        if (backward_seen.contains(key))
                        {
                            ++result.backward_seen_reject_count;
                            return false;
                        }

                    }

                    int joint2 = -1;
                    int joint = selectJointToSrc(*prev_tile, fpga::CB_NODE_DST,
                                                 prev_dst, src, &joint2);
                    if (joint == -2)
                    {
                        ++result.backward_topology_reject_count;
                        record_backward_attempt(
                            node.seed_dst, "topology_reject",
                            [&]()
                            {
                                return suffixFromNode(backward_nodes,
                                                      node_index);
                            });
                        return false;
                    }

                    // Ordinary free edges that cannot enter the frontier need
                    // no names or Wire/Node objects. Preserve complete joins,
                    // occupied edges used by later preemption, and full
                    // per-candidate diagnostic traces.
                    if (!options.preemption &&
                        !options.trace_backward_attempts &&
                        !forward_seen.contains(key) &&
                        can_use_jump(prev_tile->cb, prev_dst, src, joint,
                                     joint2,
                                     !options.source_local &&
                                         prev_tile == &forward_tile &&
                                         prev_dst == forward_dst))
                    {
                        if (backward_seen.contains(key))
                        {
                            ++result.backward_seen_reject_count;
                            return false;
                        }

                    }

                    fpga::InternedString src_wire =
                        srcWireName(*prev_tile, fpga::CB_NODE_DST, prev_dst,
                                    src, joint, {});
                    fpga::Wire edge;
                    edge.from = prev_tile->coord;
                    edge.to = node.tile->coord;
                    edge.local = prev_dst;
                    edge.jump = src;
                    edge.route_jump = source.route_jump;
                    edge.dst = node.dst;
                    edge.joint = joint;
                    edge.joint2 = joint2;
                    edge.pos = ROUTE_POS_TRANSIT;
                    edge.from_wire_name =
                        fromWireName(*prev_tile, fpga::CB_NODE_DST, prev_dst,
                                     src, joint, src_wire);
                    edge.src_wire_name = src_wire;
                    edge.dst_wire_name = node.dst_wire;
                    auto candidate_fragments = [&]()
                    {
                        std::vector<fpga::Wire> fragments{edge};
                        std::vector<fpga::Wire> suffix =
                            suffixFromNode(backward_nodes, node_index);
                        fragments.insert(fragments.end(), suffix.begin(),
                                         suffix.end());
                        return fragments;
                    };

                    // The backward frontier may terminate at the already-routed
                    // forward anchor; only its outgoing source must still be
                    // free.
                    bool reaches_forward_anchor = !options.source_local &&
                                                  prev_tile == &forward_tile &&
                                                  prev_dst == forward_dst;
                    if (!can_use_jump(prev_tile->cb, prev_dst, src, joint,
                                      joint2, reaches_forward_anchor) &&
                        (!options.preemption ||
                         !options.preemption->eligible_edge(edge)))
                    {
                        if (RouteCongestionTrace::current)
                            RouteCongestionTrace::current->blocked(
                                *prev_tile, "docking_backward_busy",
                                {{fpga::CB_NODE_DST, prev_dst},
                                 {fpga::CB_NODE_SRC, src},
                                 {fpga::CB_NODE_JOINT, joint},
                                 {fpga::CB_NODE_JOINT, joint2}});
                        PendingBridgeBlocker blocker;
                        blocker.forward_key = Key{prev_tile->coord.x,
                                                  prev_tile->coord.y, prev_dst};
                        blocker.backward_key = current_key;
                        blocker.backward_index = node_index;
                        blocker.edge = edge;
                        blocker.dst_busy =
                            !reaches_forward_anchor &&
                            prev_tile->cb.dst.jump.testBit(prev_dst);
                        blocker.src_busy = prev_tile->cb.src.jump.testBit(src);
                        blocker.joint_busy =
                            joint >= 0 &&
                            prev_tile->cb.joint.jump.testBit(joint);
                        blocker.joint2_busy =
                            joint2 >= 0 &&
                            prev_tile->cb.joint.jump.testBit(joint2);
                        pending_bridge_blockers.push_back(std::move(blocker));
                        ++result.backward_busy_reject_count;
                        record_backward_attempt(node.seed_dst, "busy_reject",
                                                candidate_fragments);
                        return false;
                    }

                    Node next{prev_tile,
                              prev_dst,
                              {},
                              node_index,
                              edge,
                              node.depth + 1,
                              {},
                              node.blocked_terminal,
                              node.terminal_pin,
                              node.terminal_joint,
                              node.terminal_joint2};
                    next.seed_dst = node.seed_dst;
                    next.tile_visits_root = previous_visits_root;
                    if (const std::string *name = prev_tile->cb_type->nodeName(
                            fpga::CB_NODE_DST, prev_dst))
                    {
                        next.dst_wire = *name;
                    }

                    // Check every numeric candidate against the opposite
                    // frontier before queuing its continuation.
                    if (auto front = forward_seen.find(key);
                        front != forward_seen.end())
                    {
                        auto joined = pathToNode(forward_nodes, front->second);
                        const auto suffix = candidate_fragments();
                        joined.insert(joined.end(), suffix.begin(),
                                      suffix.end());
                        if (!path_valid(joined))
                        {
                            ++result.tile_visit_reject_count;
                            return false;
                        }

                        if (next.blocked_terminal)
                        {
                            result.blocked_terminal_reachable = true;
                            result.blocked_terminal_fragments =
                                std::move(joined);
                            result.blocked_dst = node.seed_dst;
                            result.blocked_pin = next.terminal_pin;
                            result.blocked_joint = next.terminal_joint;
                            result.blocked_joint2 = next.terminal_joint2;
                            record_backward_attempt(node.seed_dst,
                                                    "blocked_meet",
                                                    candidate_fragments);
                            return true;
                        }

                        result.fragments = std::move(joined);
                        result.success = true;
                        record_backward_attempt(node.seed_dst, "meet",
                                                candidate_fragments);
                        return true;
                    }

                    const bool preserves_blocked_bridge =
                        !options.preemption &&
                        forward_blocked_landings.contains(key);
                    if (backward_seen.find(key) != backward_seen.end())
                    {
                        ++result.backward_seen_reject_count;
                        record_backward_attempt(node.seed_dst, "seen_reject",
                                                candidate_fragments);
                        return false;
                    }

                    int next_index = static_cast<int>(backward_nodes.size());
                    backward_nodes.push_back(next);
                    backward_seen[key] = next_index;
                    if (preserves_blocked_bridge)
                    {
                        record_backward_attempt(node.seed_dst,
                                                "blocked_bridge_landing",
                                                candidate_fragments);
                        return false;
                    }

                    backward_queue.push_back(next_index);
                    ++result.backward_push_count;
                    record_backward_attempt(node.seed_dst, "push",
                                            candidate_fragments);
                    return false;
                });
            if (result.success)
            {
                return true;
            }

        }

        return false;
    };

    auto probe_initial_forward_blockers = [&]()
    {
        const Node &node = forward_nodes.front();
        const std::vector<uint16_t> *srcs = pnr::Docking::srcNodes(
            *(node.tile->cb_type), fpga::CB_NODE_DST, node.dst);
        if (!srcs)
        {
            return;
        }

        for (uint16_t src : *srcs)
        {
            if (TraceRouteForward::isDeadend(*node.tile, src,
                                             target_tile.coord))
                continue;
            int joint2 = -1;
            int joint = selectJointToSrc(*node.tile, fpga::CB_NODE_DST,
                                         node.dst, src, &joint2);
            if (joint == -2)
            {
                continue;
            }

            for (const fpga::TileJumpTarget &target :
                 fpga::Device::current().resolveJumpTargets(*node.tile, src))
            {
                if (!target.tile || !target.tile->cb_type ||
                    target.dst_node < 0 ||
                    !inDockWindow(target.tile->coord, target_tile.coord,
                                  options.radius))
                {
                    continue;
                }

                const bool src_busy = node.tile->cb.src.jump.testBit(src);
                const bool joint_busy =
                    joint >= 0 && node.tile->cb.joint.jump.testBit(joint);
                const bool joint2_busy =
                    joint2 >= 0 && node.tile->cb.joint.jump.testBit(joint2);
                if (!src_busy && !joint_busy && !joint2_busy)
                {
                    continue;
                }

                fpga::InternedString src_wire =
                    srcWireName(*node.tile, fpga::CB_NODE_DST, node.dst, src,
                                joint, node.dst_wire);
                fpga::Wire edge;
                edge.from = node.tile->coord;
                edge.to = target.tile->coord;
                edge.local = node.dst;
                edge.jump = src;
                edge.route_jump = target.jump_node;
                edge.dst = target.dst_node;
                edge.joint = joint;
                edge.joint2 = joint2;
                edge.pos = ROUTE_POS_TRANSIT;
                edge.from_wire_name = node.dst_wire;
                edge.src_wire_name = src_wire;
                edge.dst_wire_name = target.dst_wire;
                Key landing{target.tile->coord.x, target.tile->coord.y,
                            target.dst_node};
                pending_bridge_blockers.push_back(PendingBridgeBlocker{
                    nodeKey(node), landing, -1, edge, false, src_busy,
                    joint_busy, joint2_busy, true});
                forward_blocked_landings.insert(landing);
            }

        }

    };

    // Record occupied root exits before backward search starts.
    // Successful direct backward meets still run before ordinary forward work.
    if (!options.source_local && !options.preemption)
        probe_initial_forward_blockers();

    while ((!forward_queue.empty() || !backward_queue.empty()) &&
           !result.success && !InputRepairBudget::stopped())
    {
        if (!backward_queue.empty())
        {
            int index = backward_queue.front();
            backward_queue.pop_front();
            bool expanded = expand_backward(index);
            if (expanded)
            {
                break;
            }

        }

        if (!forward_queue.empty())
        {
            int index = forward_queue.front();
            forward_queue.pop_front();
            bool expanded = expand_forward(index);
            if (expanded)
            {
                break;
            }

        }

    }

    // Reuse the exhausted forward search to identify a physically reachable
    // busy terminal. The caller may then preempt exactly that numeric entry.
    if (!result.success && !blocked_seeds.empty())
    {
        // Terminal-preemption probing is a separate backward search. Preserve
        // the free-terminal frontier and its exact transit bridge before
        // replacing it.
        finish_frontiers();
        backward_nodes.clear();
        backward_queue.clear();
        backward_seen.clear();
        for (Node &seed : blocked_seeds)
        {
            Key key = nodeKey(seed);
            if (backward_seen.find(key) != backward_seen.end())
            {
                continue;
            }

            int index = static_cast<int>(backward_nodes.size());
            backward_nodes.push_back(std::move(seed));
            backward_seen[key] = index;
            if (auto front = forward_seen.find(key);
                front != forward_seen.end())
            {
                const Node &reached = backward_nodes[index];
                auto joined = pathToNode(forward_nodes, front->second);
                const auto suffix = suffixFromNode(backward_nodes, index);
                joined.insert(joined.end(), suffix.begin(), suffix.end());
                if (!path_valid(joined))
                {
                    ++result.tile_visit_reject_count;
                    continue;
                }

                result.blocked_terminal_reachable = true;
                result.blocked_terminal_fragments = std::move(joined);
                result.blocked_dst = reached.dst;
                result.blocked_pin = reached.terminal_pin;
                result.blocked_joint = reached.terminal_joint;
                result.blocked_joint2 = reached.terminal_joint2;
                break;
            }

            backward_queue.push_back(index);
        }

        while (!result.blocked_terminal_reachable && !backward_queue.empty() &&
               !InputRepairBudget::stopped())
        {
            int index = backward_queue.front();
            backward_queue.pop_front();
            bool expanded = expand_backward(index);
            if (expanded)
            {
                break;
            }

        }

    }

    finish_frontiers();
    return result;
}

} // namespace

// Inspect physical landing Tiles without building a per-hop source list.
bool Docking::canPlaceAtReverseLanding(fpga::Tile &tile, int dst,
                                       const BackwardResolveIndex &index) const
{
    auto incoming = incomingCursor(index, {tile.coord.x, tile.coord.y, dst});
    while (const auto *source = nextIncoming(incoming, {0, 0}))
        if (canPlaceIn(*source->tile))
            return true;
    return false;
}

// Record an occupied source only after Moving admitted its placement Tile.
void Docking::recordBackwardSourceBlock(BackwardTakeoffRoute &result,
                                        fpga::Tile &tile, int src) const
{
    retainBackwardBlocked(result, {{tile.coord, fpga::CB_NODE_SRC, src}});
}

// A blocked reverse DST matters only if its next Tile can host the driver.
void Docking::recordBackwardDstBlock(BackwardTakeoffRoute &result,
                                     fpga::Tile &tile, int dst,
                                     BackwardResolveIndex &index) const
{
    auto incoming = incomingCursor(index, {tile.coord.x, tile.coord.y, dst});
    while (const auto *source = nextIncoming(incoming, {0, 0}))
    {
        if (!canPlaceIn(*source->tile))
            continue;
        std::vector<fpga::RouteCutNode> cuts{
            {tile.coord, fpga::CB_NODE_DST, dst}};
        if (driverState(*source->tile).src.jump.testBit(source->src))
            cuts.insert(cuts.begin(),
                        {source->tile->coord, fpga::CB_NODE_SRC, source->src});
        retainBackwardBlocked(result, cuts);
        return;
    }

}

// Prepare immutable connectivity views before entering recursive tracing.
// FullSearch owns the vector construction; frames only borrow numeric indexes.
void Docking::prepareBackwardTrace(BackwardResolveIndex &index)
{
    auto &trace = index.shared_cache ? index.shared_cache->trace : index.trace;
    if (trace.prepared)
        return;
    const auto &incoming =
        index.shared_cache ? index.shared_cache->incoming : index.sources;
    trace.dst_directions.clear();
    for (const auto &[key, sources] : incoming)
    {
        auto known = trace.hops.find(key);
        if (known == trace.hops.end() ||
            known->second.sources.data() != sources.data() ||
            known->second.sources.size() != sources.size())
            trace.hops[key] =
                FullSearch::backwardHopChoices(sources, {key.x, key.y});
        const auto *target = fpga::Device::current().getTile(key.x, key.y);
        for (const auto &source : sources)
        {
            if (target && source.tile)
                trace
                    .dst_directions[target][fpga::CBType::hopDirectionIndex(
                        source.tile->coord - target->coord)]
                    .jump.setBit(key.dst);
            if (source.tile && source.tile->cb_type &&
                !trace.crossbars.contains(source.tile->cb_type))
                trace.crossbars.emplace(
                    source.tile->cb_type,
                    FullSearch::backwardMasks(*source.tile->cb_type));
        }

    }

    trace.prepared = true;
}

// Select a DST in the reverse target direction using precomputed bitmap rows.
int Docking::nextBackwardDst(const BackwardResolveIndex &index,
                             const fpga::Tile &tile, fpga::Coord angle,
                             DstCursor &cursor, const NodeMask &allowed)
{
    const auto &trace =
        index.shared_cache ? index.shared_cache->trace : index.trace;
    const auto rows = trace.dst_directions.find(&tile);
    if (!cursor.initialized)
    {
        cursor.remaining = allowed;
        cursor.initialized = true;
    }

    cursor.remaining &= allowed;
    cursor.bucket &= cursor.remaining;
    while (cursor.remaining != NodeMask{})
    {
        const int dst = cursor.bucket.firstSetBit();
        if (dst >= 0)
        {
            cursor.bucket.clearBit(dst);
            cursor.remaining.clearBit(dst);
            return dst;
        }

        if (cursor.angle_rank == 225 || rows == trace.dst_directions.end())
        {
            const int remaining = cursor.remaining.firstSetBit();
            cursor.remaining.clearBit(remaining);
            return remaining;
        }

        const auto direction =
            fpga::CBType::priorityDirection(angle, cursor.angle_rank++);
        cursor.bucket = cursor.remaining & rows->second.get(direction).jump;
    }

    return -1;
}

// Borrow a direction-group view; this lookup neither copies nor enumerates it.
Docking::IncomingCursor
Docking::incomingCursor(const BackwardResolveIndex &index,
                        BackwardResolveKey key)
{
    const auto &trace =
        index.shared_cache ? index.shared_cache->trace : index.trace;
    const auto found = trace.hops.find(key);
    return {found == trace.hops.end() ? nullptr : &found->second};
}

// Pop one physical source in angle and short-hop order without a frame vector.
const BackwardResolveSource *Docking::nextIncoming(IncomingCursor &cursor,
                                                   fpga::Coord angle)
{
    if (!cursor.choices)
        return nullptr;
    while (cursor.next == cursor.end)
    {
        if (cursor.angle_rank == 225)
            return nullptr;
        const auto direction =
            fpga::CBType::priorityDirection(angle, cursor.angle_rank++);
        const auto *range = cursor.choices->range(direction);
        if (!range)
            continue;
        cursor.next = range->begin;
        cursor.end = range->end;
    }

    return &cursor.choices
                ->sources[cursor.choices->source_indices[cursor.next++]];
}

// Read direct inverse masks already prepared for this immutable CB type.
const BackwardCrossbarMasks *
Docking::reverseMasks(const BackwardResolveIndex &index,
                      const fpga::CBType &type)
{
    const auto &trace =
        index.shared_cache ? index.shared_cache->trace : index.trace;
    const auto found = trace.crossbars.find(&type);
    return found == trace.crossbars.end() ? nullptr : &found->second;
}

// Moving may cross an ineligible Tile without attempting cell placement.
// Traces also use this gate before admitting a marked endpoint jump.
bool Docking::canPlaceIn(fpga::Tile &tile) const
{
    return !placement_allowed || placement_allowed(tile);
}

// Reject an exitless CB outside a fixed sink's mapped Tiles.
// Forward tracing uses this before enumerating internal LOCAL/JOINT paths.
bool Docking::canContinueIn(const fpga::Tile &tile) const
{
    return terminal_tiles.empty() || !tile.cb_type ||
           !tile.cb_type->dst_by_src.values.empty() ||
           !tile.cb_type->local_by_local.values.empty() ||
           std::find(terminal_tiles.begin(), terminal_tiles.end(), &tile) !=
               terminal_tiles.end();
}

// A reserved input LOCAL may be visited only when this docking attempt can
// accept it as its own terminal.
bool Docking::hasLocalTerminal() const
{
    return static_cast<bool>(local_terminal);
}

bool Docking::canEnterReservedLocal(fpga::Tile &tile, int node) const
{
    return !inputLocalReserved(tile, node) || hasLocalTerminal();
}

bool Docking::blocksReservedTransit(fpga::Tile &tile, int node, bool root) const
{
    return !root && inputLocalReserved(tile, node);
}

// Accept a reached terminal from the correct node perspective. The trace has
// already rejected occupied arrivals before calling this method.
bool Docking::tryTerminal(fpga::Tile &tile, fpga::CBNodeNameType type, int node,
                          bool root_dst,
                          std::vector<fpga::Wire> &terminal) const
{
    if (type == fpga::CB_NODE_DST)
        return sink && canPlaceIn(tile) && sink(tile, node, root_dst, terminal);
    if (type != fpga::CB_NODE_LOCAL || !local_terminal ||
        tile.cb.local.local.testBit(node))
        return false;
    return local_terminal(tile, node, terminal);
}

// Keep terminal enumeration and its result buffer inside Docking.
// A marked arrival may finish here but cannot leave this routing CB again.
bool Docking::appendTerminal(fpga::Tile &tile, fpga::CBNodeNameType type,
                             int node, bool root_dst, bool terminal_only,
                             std::vector<fpga::Wire> &path) const
{
    std::vector<fpga::Wire> terminal;
    if (!tryTerminal(tile, type, node, root_dst, terminal))
        return false;
    if (terminal_only)
        for (const auto &wire : terminal)
        {
            if (wire.from == tile.coord && wire.to == tile.coord)
                continue;
            if (wire.type != fpga::Wire::WIRE_TILE_PIN)
                return false;
            const auto *from =
                fpga::Device::current().getTile(wire.from.x, wire.from.y);
            const auto *to =
                fpga::Device::current().getTile(wire.to.x, wire.to.y);
            if (!from || !to ||
                (from != &tile && from->cb_coord != tile.coord) ||
                (to != &tile && to->cb_coord != tile.coord))
                return false;
        }

    path.insert(path.end(), terminal.begin(), terminal.end());
    return true;
}

// Explore longer typed internal paths only after the angle trace has tried
// its direct and single-bridge exits. The on-path set prevents local cycles.
bool Docking::searchForwardInternal(
    fpga::Tile &start, fpga::CBNodeNameType start_type, int start_node,
    const DockingForwardVisitor &visit, bool &limited) const
{
    limited = false;
    if (!start.cb_type ||
        (start.cb_type->local_local.values.empty() &&
         start.cb_type->joint_joint.values.empty() &&
         start.cb_type->joint_local.values.empty() &&
         start.cb_type->local_by_local.values.empty()))
        return false;

    struct NodeKey
    {
        fpga::Tile *tile;
        fpga::CBNodeNameType type;
        int node;

        bool operator==(const NodeKey &) const = default;
    };

    struct NodeHash
    {
        size_t operator()(const NodeKey &key) const
        {
            size_t hash = std::hash<fpga::Tile *>{}(key.tile);
            hash ^= std::hash<int>{}(key.type) + 0x9e3779b9 + (hash << 6) +
                    (hash >> 2);
            hash ^= std::hash<int>{}(key.node) + 0x9e3779b9 + (hash << 6) +
                    (hash >> 2);
            return hash;
        }
    };

    constexpr size_t max_states = 16384;
    constexpr size_t max_depth = 2048;
    std::vector<DockingForwardNode> path;
    std::unordered_set<NodeKey, NodeHash> on_path;
    size_t states = 0;
    const auto &device = fpga::Device::current();

    auto walk = [&](auto &&self, fpga::Tile &tile,
                    fpga::CBNodeNameType type, int node,
                    bool extended) -> bool
    {
        if (!tile.cb_type || node < 0 || node >= CB_MAX_NODES ||
            !inDockWindow(tile.coord, start.coord, 5))
            return false;
        if (states >= max_states || path.size() >= max_depth)
        {
            limited = true;
            return false;
        }

        const NodeKey key{&tile, type, node};
        if (on_path.contains(key))
            return false;
        if (!path.empty() && path.back().tile != &tile)
        {
            size_t visits = 0;
            for (size_t index = 0; index < path.size(); ++index)
                if (path[index].tile == &tile &&
                    (index == 0 || path[index - 1].tile != &tile))
                    ++visits;
            if (visits >= 2)
                return false;
        }

        const bool root = path.empty();
        if (!root &&
            ((type == fpga::CB_NODE_DST && tile.cb.dst.jump.testBit(node)) ||
             (type == fpga::CB_NODE_LOCAL &&
              tile.cb.local.local.testBit(node)) ||
             (type == fpga::CB_NODE_JOINT &&
              tile.cb.joint.jump.testBit(node)) ||
             (type == fpga::CB_NODE_SRC && tile.cb.src.jump.testBit(node))))
            return false;

        ++states;
        on_path.insert(key);
        path.push_back({&tile, type, node});
        const bool reserved_local =
            type == fpga::CB_NODE_LOCAL && !root &&
            (tile.pin_state.leased_nodes.testBit(node) ||
             inputLocalReserved(tile, node));
        bool found = false;
        if (extended &&
            (type == fpga::CB_NODE_SRC ||
             (type == fpga::CB_NODE_LOCAL && hasLocalTerminal())))
            found = visit(path);

        if (!found && type != fpga::CB_NODE_SRC && !reserved_local)
        {
            tile.cb_type->ensureDerivedMasks();
            auto visit_mask = [&](fpga::CBNodeNameType next_type,
                                  const NodeMask &nodes)
            {
                bool accepted = false;
                nodes.for_each_set_bit(
                    [&](int next_node)
                    {
                        const bool next_extended =
                            extended ||
                            ((type == fpga::CB_NODE_LOCAL ||
                              type == fpga::CB_NODE_JOINT) &&
                             (next_type == fpga::CB_NODE_LOCAL ||
                              next_type == fpga::CB_NODE_JOINT));
                        if (self(self, tile, next_type, next_node,
                                 next_extended))
                        {
                            accepted = true;
                            return true;
                        }
                        return false;
                    });
                return accepted;
            };

            if (type == fpga::CB_NODE_DST)
                found = visit_mask(fpga::CB_NODE_SRC,
                                   tile.cb_type->dst_src.get(node).jump) ||
                        visit_mask(fpga::CB_NODE_LOCAL,
                                   tile.cb_type->dst_local.get(node).local) ||
                        visit_mask(fpga::CB_NODE_JOINT,
                                   tile.cb_type->dst_joint.get(node).joint);
            else if (type == fpga::CB_NODE_LOCAL)
            {
                found = visit_mask(fpga::CB_NODE_SRC,
                                   tile.cb_type->local_src.get(node).jump) ||
                        visit_mask(fpga::CB_NODE_LOCAL,
                                   tile.cb_type->local_local.get(node).local) ||
                        visit_mask(fpga::CB_NODE_JOINT,
                                   tile.cb_type->local_joint.get(node).joint);
                if (!found)
                    for (const auto &target :
                         device.resolveLocalTargets(tile, node))
                    {
                        if (target.tile &&
                            self(self, *target.tile, target.node_type,
                                 target.node, true))
                        {
                            found = true;
                            break;
                        }
                    }
            }
            else if (type == fpga::CB_NODE_JOINT)
                found = visit_mask(fpga::CB_NODE_SRC,
                                   tile.cb_type->joint_src.get(node).jump) ||
                        visit_mask(fpga::CB_NODE_LOCAL,
                                   tile.cb_type->joint_local.get(node).local) ||
                        visit_mask(fpga::CB_NODE_JOINT,
                                   tile.cb_type->joint_joint.get(node).joint);
        }

        path.pop_back();
        on_path.erase(key);
        return found;
    };

    return walk(walk, start, start_type, start_node, false);
}

// Preserve a blocked landing for the caller's later targeted cut attempt.
void Docking::recordBlockedArrival(fpga::Tile &tile, int dst) const
{
    if (blocked_arrival)
        blocked_arrival(tile, dst);
}

// Check a candidate source placement while its speculative path is leased.
bool Docking::hasDriver() const
{
    return driver_probe && static_cast<bool>(*driver_probe);
}

bool Docking::tryDriver(fpga::Tile &tile, int src,
                        BackwardTakeoffChoice &choice) const
{
    return hasDriver() && canPlaceIn(tile) &&
           (*driver_probe)(tile, src, choice);
}

bool Docking::driverStopped() const
{
    return driver_cancel && *driver_cancel && (*driver_cancel)();
}

const fpga::CBState *Docking::driverStateOverride(fpga::Tile &tile) const
{
    if (!driver_state)
        return nullptr;
    const auto found = driver_state->find(&tile);
    return found == driver_state->end() ? nullptr : &found->second;
}

const fpga::CBState &Docking::driverState(fpga::Tile &tile) const
{
    if (const auto *state = driverStateOverride(tile))
        return *state;
    return tile.cb;
}

void dumpBackwardTerminalReport(fpga::Tile &target, NodeMask pins,
                                BackwardResolveIndex &index, int docking_radius,
                                const std::vector<rtl::Net *> &design_nets,
                                std::ostream &out,
                                NodeMask reserved_terminal_joints)
{
    out << "BACKWARD_TERMINAL_SNAPSHOT target=(" << target.coord.x << ','
        << target.coord.y << ") docking_radius=" << docking_radius
        << " index_radius=" << index.radius << '\n'
        << "Final-state topology/lease analysis, NOT recorded search attempts. "
           "No anchor lease exemptions; free first hop does not prove a full "
           "route.\n";
    if (!target.cb_type)
    {
        out << "NO_CROSSBAR\n";
        return;
    }

    std::vector<fpga::Tile *> audited{&target};
    std::unordered_set<fpga::Tile *> seen{&target};
    auto node = [&](fpga::Tile &tile, const char *label,
                    fpga::CBNodeNameType type, int id)
    {
        const std::string *name =
            id >= 0 ? tile.cb_type->nodeName(type, id) : nullptr;
        const bool busy =
            id >= 0 && fpga::congestionNodeMask(tile.cb, type).testBit(id);
        out << ' ' << label << '=' << id << '[' << (name ? *name : "")
            << "] busy=" << busy;
    };

    size_t entries = 0, free_entries = 0, incoming_count = 0, free_hops = 0;
    pins.for_each_set_bit(
        [&](int pin)
        {
            out << "PIN";
            node(target, "local", fpga::CB_NODE_LOCAL, pin);
            out << " pin_leased=" << target.isPinNodeLeased(pin) << '\n';
            for (const auto &entry :
                 pnr::Docking::terminalEntries(*(target.cb_type), pin))
            {
                ++entries;
                const bool reserved =
                    (entry.joint >= 0 &&
                     reserved_terminal_joints.testBit(entry.joint)) ||
                    (entry.joint2 >= 0 &&
                     reserved_terminal_joints.testBit(entry.joint2));
                const bool terminal_free =
                    !reserved && !target.isPinNodeLeased(pin) &&
                    canLeaseIn(target.cb, entry.dst, pin, entry.joint,
                               entry.joint2);
                free_entries += terminal_free;
                out << "TERMINAL";
                node(target, "dst", fpga::CB_NODE_DST, entry.dst);
                node(target, "joint", fpga::CB_NODE_JOINT, entry.joint);
                node(target, "joint2", fpga::CB_NODE_JOINT, entry.joint2);
                out << " reserved_joint=" << reserved
                    << " incoming_mask_allows="
                    << (target.incoming_dst_nodes == NodeMask{} ||
                        target.incoming_dst_nodes.testBit(entry.dst))
                    << " terminal_free=" << terminal_free << '\n';
                const auto *sources = resolveBackwardSources(
                    index, {target.coord.x, target.coord.y, entry.dst});
                if (!sources || sources->empty())
                {
                    out << "  NO_INCOMING_SRC\n";
                    continue;
                }

                for (const auto &source : *sources)
                {
                    if (!source.tile || !source.tile->cb_type)
                        continue;
                    ++incoming_count;
                    auto &tile = *source.tile;
                    if (seen.insert(&tile).second)
                        audited.push_back(&tile);
                    out << "  INCOMING tile=(" << tile.coord.x << ','
                        << tile.coord.y << ')';
                    node(tile, "src", fpga::CB_NODE_SRC, source.src);
                    out << " in_docking_window="
                        << inDockWindow(tile.coord, target.coord,
                                        docking_radius)
                        << '\n';
                    tile.cb_type->ensureDerivedMasks();
                    const auto predecessors =
                        tile.cb_type->dsts_reaching_src.get(source.src).jump;
                    if (predecessors == NodeMask{})
                        out << "    NO_PREVIOUS_DST\n";
                    predecessors.for_each_set_bit(
                        [&](int dst)
                        {
                            int joint2 = -1;
                            const int joint =
                                selectJointToSrc(tile, fpga::CB_NODE_DST, dst,
                                                 source.src, &joint2);
                            const bool jump_free =
                                joint != -2 &&
                                canUsePhysicalJump(tile.cb, dst, source.src,
                                                   joint, joint2);
                            const bool first_hop_free =
                                terminal_free && jump_free;
                            free_hops += first_hop_free;
                            out << "    PREDECESSOR";
                            node(tile, "dst", fpga::CB_NODE_DST, dst);
                            node(tile, "joint", fpga::CB_NODE_JOINT, joint);
                            node(tile, "joint2", fpga::CB_NODE_JOINT, joint2);
                            out << " topology_ok=" << (joint != -2)
                                << " jump_free=" << jump_free
                                << " first_hop_free=" << first_hop_free << '\n';
                            return false;
                        });
                }

            }

            return false;
        });
    out << "SUMMARY terminal_entries=" << entries
        << " free_entries=" << free_entries
        << " incoming_alternatives=" << incoming_count
        << " free_first_hops=" << free_hops
        << " audited_tiles=" << audited.size() << '\n';
    out << "OWNERSHIP_AUDITS (live route claims checked against design "
           "registry)\n";
    // Reconstruct route membership once for this read-only group of Tile
    // proofs.
    fpga::CongestionAuditIndex ownership_index(design_nets);
    for (auto *tile : audited)
        fpga::auditTileCongestion(*tile, out, ownership_index);
}

bool combinatorialRouteVisitsValid(const std::vector<fpga::Wire> &route,
                                   unsigned max_visits)
{
    if (max_visits == 0)
    {
        return route.empty();
    }

    std::unordered_map<uint64_t, unsigned> visits;
    uint64_t previous = 0;
    bool has_previous = false;
    auto append = [&](const fpga::Coord &coord)
    {
        if (coord.x < 0 || coord.y < 0)
        {
            return true;
        }

        uint64_t key =
            (static_cast<uint64_t>(static_cast<uint32_t>(coord.x)) << 32) |
            static_cast<uint32_t>(coord.y);
        if (has_previous && previous == key)
        {
            return true;
        }

        previous = key;
        has_previous = true;
        return ++visits[key] <= max_visits;
    };

    for (const fpga::Wire &fragment : route)
    {
        if (!append(fragment.from) || !append(fragment.to))
        {
            return false;
        }

    }

    return true;
}

DockingGroundingSearchResult
searchGroundingFromFrontier(const DockingGroundingSearch &request)
{
    struct Step
    {
        RoutingNode node;
        int parent = -1;
        int hops = 0;
        int cost = 0;
        int root = -1;
        int prefix_hops = 0;
        int tile_visits_root = -1;
    };

    DockingGroundingSearchResult result;
    std::vector<Step> steps;
    std::map<RoutingNode, int> cheapest;
    std::map<RoutingNode, bool> cut_eligible;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<>>
        pending;
    CombinatorialTileVisits tile_visits;
    for (const auto &seed : request.seeds)
    {
        const fpga::Coord coord{std::get<0>(seed.node), std::get<1>(seed.node)};
        int visits_root = -1;
        if (!tile_visits.append(-1, coord, visits_root))
            continue;
        const int index = static_cast<int>(steps.size());
        cheapest[seed.node] = index;
        steps.push_back(
            {seed.node, -1, 0, 0, seed.root, seed.prefix_hops, visits_root});
        pending.push({0, index});
    }

    while (!pending.empty())
    {
        if (request.stopped && request.stopped())
        {
            result.stopped = true;
            break;
        }

        const int current = pending.top().second;
        pending.pop();
        const Step step = steps[current];
        if (cheapest.at(step.node) != current)
            continue;
        if (!InputRepairBudget::expand())
        {
            result.stopped = true;
            break;
        }

        ++result.expanded;
        if (step.node == request.sink)
        {
            std::vector<RoutingNode> tail;
            for (int at = current; steps[at].parent >= 0; at = steps[at].parent)
                tail.push_back(steps[at].node);
            std::reverse(tail.begin(), tail.end());
            result.found = request.complete(step.root, std::move(tail));
            if (result.found)
                break;
            ++result.rejected_complete;
            continue;
        }

        request.successors(
            step.node,
            [&](const RoutingNode &next)
            {
                const fpga::Coord from{std::get<0>(step.node),
                                       std::get<1>(step.node)};
                const fpga::Coord to{std::get<0>(next), std::get<1>(next)};
                if (std::abs(to.x - request.center.x) > request.radius ||
                    std::abs(to.y - request.center.y) > request.radius)
                    return;
                const bool jump = from != to;
                const int hops = step.hops + jump;
                if (hops > 2 * request.radius ||
                    hops + step.prefix_hops > request.max_hops)
                    return;
                for (int at = current; at >= 0; at = steps[at].parent)
                    if (steps[at].node == next)
                        return;
                int visits_root = step.tile_visits_root;
                if (jump &&
                    !tile_visits.append(step.tile_visits_root, to, visits_root))
                    return;

                const bool needs_cut = !request.free_edge(step.node, next);
                const int cost = step.cost + 1 + 4 * jump + 20 * needs_cut;
                auto old = cheapest.find(next);
                if (old != cheapest.end() && steps[old->second].cost <= cost)
                    return;
                if (needs_cut)
                {
                    const int kind = std::get<2>(next);
                    if (kind != fpga::CB_NODE_DST &&
                        kind != fpga::CB_NODE_SRC &&
                        kind != fpga::CB_NODE_JOINT)
                        return;
                    auto [it, inserted] = cut_eligible.try_emplace(next, false);
                    if (inserted)
                        it->second = request.cut_edge(step.node, next);
                    if (!it->second)
                        return;
                }

                if (steps.size() >= request.max_states)
                {
                    result.budget_limited = true;
                    return;
                }

                const int index = static_cast<int>(steps.size());
                cheapest[next] = index;
                steps.push_back({next, current, hops, cost, step.root,
                                 step.prefix_hops, visits_root});
                pending.push({cost, index});
            });
    }

    result.states = steps.size();
    return result;
}

std::vector<DockingLocalNode> searchDockingLocalPath(
    fpga::Tile &tile, int target_local, const NodeMask &root_nodes,
    const std::function<bool(const DockingLocalNode &)> &available,
    size_t *root_count, size_t *visited_count)
{
    if (!tile.cb_type || target_local < 0 || target_local >= CB_MAX_NODES)
        return {};
    auto key = [](const DockingLocalNode &node)
    {
        return (static_cast<uint32_t>(node.type) << 16) |
               static_cast<uint16_t>(node.value);
    };

    auto from_key = [](uint32_t value)
    {
        return DockingLocalNode{
            static_cast<fpga::CBNodeNameType>((value >> 16) & 0xff),
            static_cast<int>(value & 0xffff)};
    };

    std::deque<DockingLocalNode> queue;
    std::unordered_map<uint32_t, uint32_t> parent;
    std::unordered_set<uint32_t> roots;
    root_nodes.for_each_set_bit(
        [&](int local)
        {
            if (root_count)
                ++*root_count;
            const DockingLocalNode root{fpga::CB_NODE_LOCAL, local};
            const uint32_t id = key(root);
            roots.insert(id);
            parent.emplace(id, id);
            queue.push_back(root);
            return false;
        });

    uint32_t reached = std::numeric_limits<uint32_t>::max();
    while (!queue.empty())
    {
        const DockingLocalNode current = queue.front();
        queue.pop_front();
        if (visited_count)
            ++*visited_count;
        if (current.type == fpga::CB_NODE_LOCAL &&
            current.value == target_local)
        {
            reached = key(current);
            break;
        }

        auto visit = [&](fpga::CBNodeNameType type, const NodeMask &nodes)
        {
            nodes.for_each_set_bit(
                [&](int value)
                {
                    const DockingLocalNode next{type, value};
                    const uint32_t id = key(next);
                    if (!parent.contains(id) && available(next))
                    {
                        parent.emplace(id, key(current));
                        queue.push_back(next);
                    }

                    return false;
                });
        };

        if (current.type == fpga::CB_NODE_LOCAL)
        {
            visit(fpga::CB_NODE_LOCAL,
                  tile.cb_type->local_local.get(current.value).local);
            visit(fpga::CB_NODE_JOINT,
                  tile.cb_type->local_joint.get(current.value).joint);
        }

        else if (current.type == fpga::CB_NODE_JOINT)
        {
            visit(fpga::CB_NODE_LOCAL,
                  tile.cb_type->joint_local.get(current.value).local);
            visit(fpga::CB_NODE_JOINT,
                  tile.cb_type->joint_joint.get(current.value).joint);
        }

    }

    if (reached == std::numeric_limits<uint32_t>::max())
        return {};
    std::vector<DockingLocalNode> path;
    for (uint32_t cursor = reached;; cursor = parent.at(cursor))
    {
        path.push_back(from_key(cursor));
        if (roots.contains(cursor))
            break;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool materializeDockingBridge(const DockingBridgeBlocker &bridge,
                              DockingResult &result)
{
    if (!bridge.valid || !bridge.joins_frontiers ||
        bridge.backward_suffix.empty())
    {
        return false;
    }

    const fpga::Wire &edge = bridge.backward_suffix.front();
    if (edge.type != fpga::Wire::WIRE_CROSSBAR ||
        edge.from.x != bridge.tile.x || edge.from.y != bridge.tile.y ||
        edge.local != bridge.dst || edge.jump != bridge.src ||
        edge.joint != bridge.joint || edge.joint2 != bridge.joint2 ||
        edge.to.x != bridge.landing_tile.x ||
        edge.to.y != bridge.landing_tile.y || edge.dst != bridge.landing_dst)
    {
        return false;
    }

    if (!bridge.forward_prefix.empty())
    {
        const fpga::Wire &prefix_end = bridge.forward_prefix.back();
        if (prefix_end.to.x != bridge.tile.x ||
            prefix_end.to.y != bridge.tile.y || prefix_end.dst != bridge.dst)
        {
            return false;
        }

    }

    result.fragments = bridge.forward_prefix;
    result.fragments.insert(result.fragments.end(),
                            bridge.backward_suffix.begin(),
                            bridge.backward_suffix.end());
    if (!combinatorialRouteVisitsValid(result.fragments))
    {
        result.fragments.clear();
        ++result.tile_visit_reject_count;
        return false;
    }

    result.success = true;
    return true;
}

// Keep reverse arrivals short-first when constructing the numeric index.
// Stable insertion also handles cache extensions; traversal needs no sorting.
static void insertBackwardSource(std::vector<BackwardResolveSource> &sources,
                                 const BackwardResolveSource &source,
                                 fpga::Coord target)
{
    auto length_squared = [&](const BackwardResolveSource &candidate)
    {
        const int64_t dx = int64_t(candidate.tile->coord.x) - target.x;
        const int64_t dy = int64_t(candidate.tile->coord.y) - target.y;
        return dx * dx + dy * dy;
    };

    auto position = std::upper_bound(
        sources.begin(), sources.end(), length_squared(source),
        [&](int64_t length, const BackwardResolveSource &candidate)
        { return length < length_squared(candidate); });
    sources.insert(position, source);
}

BackwardResolveIndex buildBackwardResolveIndex(
    fpga::Device &device, fpga::Coord center, int radius,
    const std::function<bool(const fpga::Coord &)> &include_source,
    BackwardResolveCache *shared_cache)
{
    BackwardResolveIndex result;
    result.center = center;
    result.radius = radius;
    result.shared_cache = shared_cache;
    for (int y = center.y - radius; y <= center.y + radius; ++y)
    {
        for (int x = center.x - radius; x <= center.x + radius; ++x)
        {
            fpga::Coord source_coord{x, y};
            if (!inDockWindow(source_coord, center, radius) ||
                (include_source && !include_source(source_coord)))
            {
                continue;
            }

            fpga::Tile *source_tile = device.getTile(x, y);
            if (!source_tile || !source_tile->cb_type)
            {
                continue;
            }

            // Match Device::resolveLocalTargets exactly. Dedicated links are
            // tied to physical grid coordinates, including resource-side views
            // that the generic jump index redirects below. Build them once with
            // that index.
            if (!shared_cache ||
                shared_cache->processed_local_tiles.insert(source_tile).second)
            {
                auto &incoming = shared_cache ? shared_cache->local_incoming
                                              : result.local_incoming;
                for (const auto &[local, entries] :
                     source_tile->cb_type->local_by_local.values)
                {
                    if (entries.empty())
                        continue;
                    for (const auto &target :
                         device.resolveLocalTargets(*source_tile, local))
                    {
                        if (!target.tile)
                            continue;
                        const auto c = target.tile->coord;
                        incoming[{c.x, c.y, target.node_type, target.node}]
                            .push_back({source_tile, int(local)});
                    }

                }

            }

            // Resource-side tile views can alias the same physical switchbox.
            // Reverse routing indexes only the tile that owns the lease state.
            if (device.routeTile(*source_tile) != source_tile)
            {
                continue;
            }

            if (shared_cache &&
                !shared_cache->processed_tiles.insert(source_tile).second)
            {
                continue;
            }

            if (shared_cache)
                shared_cache->trace.prepared = false;

            for (const auto &[src_key, entries] :
                 source_tile->cb_type->dst_by_src.values)
            {
                ++result.mapping_scan_count;
                if (entries.empty())
                {
                    continue;
                }

                int src = src_key;
                const std::vector<BackwardResolveArc> *arcs = nullptr;
                std::vector<BackwardResolveArc> uncached_arcs;
                BackwardResolveSourceKey source_key{source_tile->cb_type, src};
                if (shared_cache)
                {
                    auto known = shared_cache->arcs.find(source_key);
                    if (known != shared_cache->arcs.end())
                    {
                        arcs = &known->second;
                    }

                }

                if (!arcs)
                {
                    for (const fpga::TileJumpTarget &target :
                         device.resolveJumpTargets(*source_tile, src))
                    {
                        if (!target.tile || !target.tile->cb_type ||
                            target.dst_node < 0)
                        {
                            continue;
                        }

                        uncached_arcs.push_back(BackwardResolveArc{
                            target.tile->coord - source_tile->coord,
                            target.dst_node, target.jump_node});
                    }

                    // A source near the chip edge may have no physical landing.
                    // That empty result is coordinate-specific, not proof that
                    // this CB type has no such wire anywhere on the chip.
                    if (shared_cache && !uncached_arcs.empty())
                    {
                        arcs =
                            &shared_cache->arcs
                                 .emplace(source_key, std::move(uncached_arcs))
                                 .first->second;
                    }

                    else
                    {
                        arcs = &uncached_arcs;
                    }

                }

                if (shared_cache)
                {
                    for (const BackwardResolveArc &arc : *arcs)
                    {
                        fpga::Coord target_coord =
                            source_tile->coord + arc.delta;
                        BackwardResolveKey target{target_coord.x,
                                                  target_coord.y, arc.dst};
                        insertBackwardSource(
                            shared_cache->incoming[target],
                            BackwardResolveSource{source_tile, src,
                                                  arc.route_jump},
                            target_coord);
                    }

                    continue;
                }

                std::unordered_set<BackwardResolveKey, BackwardResolveKeyHash>
                    source_targets;
                for (const BackwardResolveArc &arc : *arcs)
                {
                    fpga::Coord target_coord = source_tile->coord + arc.delta;
                    if (!inDockWindow(target_coord, center, radius))
                    {
                        continue;
                    }

                    BackwardResolveKey key{target_coord.x, target_coord.y,
                                           arc.dst};
                    if (!source_targets.insert(key).second)
                    {
                        continue;
                    }

                    insertBackwardSource(
                        result.sources[key],
                        BackwardResolveSource{source_tile, src, arc.route_jump},
                        target_coord);
                    ++result.mapping_reaches_count;
                }

            }

        }

    }

    return result;
}

const std::vector<BackwardResolveSource> *
resolveBackwardSources(BackwardResolveIndex &index,
                       const BackwardResolveKey &key,
                       bool include_external_sources)
{
    if (index.shared_cache && include_external_sources)
    {
        // Do not populate/reuse the radius-filtered view for this lookup. A
        // prior bounded lookup can legitimately be empty for an existing
        // incoming wire.
        auto incoming = index.shared_cache->incoming.find(key);
        return incoming == index.shared_cache->incoming.end()
                   ? nullptr
                   : &incoming->second;
    }

    if (index.shared_cache && index.resolved_keys.insert(key).second)
    {
        auto incoming = index.shared_cache->incoming.find(key);
        if (incoming != index.shared_cache->incoming.end())
        {
            for (const BackwardResolveSource &source : incoming->second)
            {
                if (!source.tile || !inDockWindow(source.tile->coord,
                                                  index.center, index.radius))
                {
                    continue;
                }

                index.sources[key].push_back(source);
                ++index.mapping_reaches_count;
            }

        }

    }

    auto sources = index.sources.find(key);
    return sources == index.sources.end() ? nullptr : &sources->second;
}

DockingResult
dockGrounding(fpga::Tile &forward_tile, int forward_dst,
              const std::string &forward_dst_wire, fpga::Tile &target_tile,
              NodeMask pin_nodes, int max_depth, int radius,
              bool trace_backward_attempts, NodeMask reserved_terminal_joints,
              BackwardResolveIndex *backward_index, bool source_local,
              const DockingPreemptionSearch *preemption)
{
    if (max_depth < 0)
        max_depth = dockingHopLimit(radius);
    // Standalone calls build topology once for both searches. The production
    // router already supplies its shared complete device index.
    BackwardResolveIndex standalone_index;
    if (!backward_index)
    {
        auto &device = fpga::Device::current();
        standalone_index = buildBackwardResolveIndex(
            device, {device.size_width / 2, device.size_height / 2},
            std::max(device.size_width, device.size_height));
        standalone_index.center = target_tile.coord;
        standalone_index.radius = radius;
        backward_index = &standalone_index;
    }

    auto retain_previous = [](DockingResult &expanded, DockingResult &result)
    {
        // Counters describe all work; terminal counts describe the unchanged
        // terminal alternatives. The topology itself was indexed only once.
        expanded.forward_push_count += result.forward_push_count;
        expanded.backward_push_count += result.backward_push_count;
        expanded.forward_pop_count += result.forward_pop_count;
        expanded.backward_pop_count += result.backward_pop_count;
        expanded.backward_missing_prev_dst_count +=
            result.backward_missing_prev_dst_count;
        expanded.backward_topology_reject_count +=
            result.backward_topology_reject_count;
        expanded.backward_busy_reject_count +=
            result.backward_busy_reject_count;
        expanded.backward_seen_reject_count +=
            result.backward_seen_reject_count;
        expanded.backward_deadend_count += result.backward_deadend_count;
        expanded.backward_deadend_reject_count +=
            result.backward_deadend_reject_count;
        expanded.tile_visit_reject_count += result.tile_visit_reject_count;
        expanded.backward_mapping_scan_count =
            std::max(result.backward_mapping_scan_count,
                     expanded.backward_mapping_scan_count);
        expanded.backward_mapping_reaches_count =
            std::max(result.backward_mapping_reaches_count,
                     expanded.backward_mapping_reaches_count);
        auto prepend = [](auto &to, auto &from)
        {
            to.insert(to.begin(), std::make_move_iterator(from.begin()),
                      std::make_move_iterator(from.end()));
        };

        prepend(expanded.backward_attempts, result.backward_attempts);
        if (!expanded.success)
        {
            // Keep the previously reachable bridge/terminal proposals first
            // rather than replacing them with a different truncated frontier.
            prepend(expanded.blocked_bridges, result.blocked_bridges);
            prepend(expanded.forward_frontier, result.forward_frontier);
            prepend(expanded.backward_frontier, result.backward_frontier);
            if (result.blocked_terminal_reachable)
            {
                expanded.blocked_terminal_reachable = true;
                expanded.blocked_terminal_fragments =
                    std::move(result.blocked_terminal_fragments);
                expanded.blocked_dst = result.blocked_dst;
                expanded.blocked_pin = result.blocked_pin;
                expanded.blocked_joint = result.blocked_joint;
                expanded.blocked_joint2 = result.blocked_joint2;
            }

        }

    };

    DockingResult result;
    bool tried = false, local_success = false;
    // Preserve successful established choices before adding new possibilities.
    // Eager occupied forward growth can win an earlier, different join and
    // displace routes unnecessarily. Only an exhausted provisional search gets
    // the symmetric extension; free searches and successful searches pay none
    // of its extra ownership checks. Every attempt has fresh bounded frontiers.
    for (bool forward_preemption : {false, true})
    {
        if (forward_preemption &&
            (!preemption || !preemption->allow_forward_expansion))
            break;
        for (bool bounded_reverse : {true, false})
        {
            auto candidate = dockGroundingImpl(
                forward_tile, forward_dst, forward_dst_wire, target_tile,
                pin_nodes,
                DockingOptions{max_depth, radius, trace_backward_attempts,
                               reserved_terminal_joints, backward_index,
                               bounded_reverse, source_local, preemption,
                               forward_preemption});
            if (!tried)
                local_success = candidate.success;
            if (tried)
                retain_previous(candidate, result);
            result = std::move(candidate);
            tried = true;
            if (result.success)
                break;
        }

        if (result.success)
            break;
    }

    if (std::getenv("SCALEPNR_AUDIT_DOCK_WINDOW"))
    {
        std::fprintf(stderr,
                     "DOCK_WINDOW_RESULT from=(%d,%d)/%d target=(%d,%d) "
                     "local_success=%d expanded=%d success=%d bpush=%d\n",
                     forward_tile.coord.x, forward_tile.coord.y, forward_dst,
                     target_tile.coord.x, target_tile.coord.y, local_success,
                     !local_success, result.success,
                     result.backward_push_count);
    }

    return result;
}

// Validate an input against its existing driver/tree until success or
// cancellation. Unlike placement enumeration, reaching the driver can finish
// immediately.
BackwardTakeoffRoute routeBackwardToInput(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int radius, const BackwardTakeoffProbe &source_probe,
    BackwardResolveIndex *backward_index, const BackwardTakeoffCancel &cancel,
    const BackwardTakeoffStateView *state_view,
    const std::vector<BackwardRouteAnchor> *anchors, int max_depth)
{
    return routeBackwardToTakeoff(target_tile, pin_nodes, source_hint,
                                  max_depth, radius, -1, source_probe,
                                  backward_index, cancel, 0, 0, 0, state_view,
                                  anchors, source_probe);
}

static BackwardTakeoffRoute
routeBackwardToTakeoffImpl(fpga::Tile &target_tile, NodeMask pin_nodes,
                           fpga::Coord source_hint, int max_depth, int radius,
                           int source_radius, const BackwardTakeoffProbe &probe,
                           BackwardResolveIndex *prebuilt_backward_index,
                           const BackwardTakeoffCancel &cancel,
                           size_t max_expansions, size_t probe_offset,
                           size_t max_probes,
                           const BackwardTakeoffStateView *state_view,
                           const std::vector<BackwardRouteAnchor> *anchors,
                           const BackwardTakeoffProbe &preferred_probe,
                           const BackwardTakeoffPathProbe &path_probe,
                           const BackwardTakeoffBoundary &boundary_probe,
                           const BackwardTakeoffResume &resume_boundary = {})
{
    BackwardTakeoffRoute result;
    std::unordered_set<TakeoffProbeKey, TakeoffProbeKeyHash> reached_sources;
    // This reads the active per-net trace dynamically: the deadline window may
    // enable it in the middle of this search. It never participates in route
    // selection.
    auto trace = []() -> std::ostream *
    {
        return RouteCongestionTrace::current
                   ? &RouteCongestionTrace::current->stream()
                   : nullptr;
    };

    struct SearchReport
    {
        BackwardTakeoffRoute &result;

        ~SearchReport()
        {
            if (RouteCongestionTrace::current)
            {
                auto &out = RouteCongestionTrace::current->stream();
                out << "BACKWARD_END success=" << result.success
                    << " expanded=" << result.expanded
                    << " edges=" << result.incoming_edges
                    << " probes=" << result.probe_calls
                    << " scanned=" << result.probe_candidates_scanned
                    << " remaining_frontier=" << result.remaining_frontier
                    << " failed_states=" << result.failed_states
                    << " failure_cache_hits=" << result.failure_cache_hits
                    << " boundary_released=" << result.boundary_released
                    << '\n';
                out.flush();
            }

        }

    } search_report{result};

    if (auto *out = trace())
        *out << "BACKWARD_BEGIN target=(" << target_tile.coord.x << ','
             << target_tile.coord.y << ") source_hint=(" << source_hint.x << ','
             << source_hint.y << ") radius=" << radius
             << " source_radius=" << source_radius
             << " probe_offset=" << probe_offset << " max_probes=" << max_probes
             << '\n';
    result.failure_tile = &target_tile;
    if (!target_tile.cb_type || pin_nodes == NodeMask{} ||
        (!probe && !path_probe && (!anchors || anchors->empty())) ||
        radius <= 0)
    {
        return result;
    }

    std::vector<Node> nodes;
    std::vector<int> frontier;
    // Completing a fixed prefix must minimize the suffix we permanently lease.
    // Depth-first traversal can accept a chip-spanning detour before trying a
    // nearby alternative, starving later fanouts despite a short free path.
    // Anchor and takeoff searches visit equal-cost hops breadth-first.
    CombinatorialTileVisits tile_visits;
    int diagnostic_node_index = -1;
    int diagnostic_node_depth = -1;
    std::unordered_map<Key, int, BackwardResolveKeyHash> seen;
    size_t frontier_cursor = 0;
    Node last_congestion;

    struct FrontierTakeoff
    {
        BackwardResolveSource source;
        int node_index = -1;
    };

    std::vector<std::vector<FrontierTakeoff>> takeoffs_by_distance;
    std::vector<FrontierTakeoff> reached_takeoffs;

    struct BlockedTransition
    {
        BackwardResolveSource source;
        int parent, dst, visits, joint, joint2;
        bool retired = false;
    };

    std::vector<BlockedTransition> blocked_transitions;
    std::unordered_map<fpga::Tile *, NodeMask> anchor_dsts_by_tile;
    std::unordered_map<Key, const BackwardRouteAnchor *, BackwardResolveKeyHash>
        anchor_by_node;
    if (anchors)
    {
        // Anchors arrive newest first. Keep the first exact landing and expose
        // all candidate destination nodes as a per-tile mask for numeric
        // intersection.
        for (const BackwardRouteAnchor &anchor : *anchors)
        {
            if (!anchor.tile || anchor.dst < 0 || anchor.dst >= CB_MAX_NODES)
            {
                continue;
            }

            Key key{anchor.tile->coord.x, anchor.tile->coord.y, anchor.dst};
            anchor_by_node.try_emplace(key, &anchor);
            anchor_dsts_by_tile[anchor.tile].setBit(anchor.dst);
        }

    }

    auto state = [&](fpga::Tile &tile) -> const fpga::CBState &
    {
        if (state_view)
        {
            auto found = state_view->find(&tile);
            if (found != state_view->end())
            {
                return found->second;
            }

        }

        return tile.cb;
    };

    auto retain_diagnostic_path = [&]()
    {
        if (last_congestion.tile)
        {
            // This blocked hop is diagnostic only: never insert it into the
            // search frontier or live leases. Keep its connected suffix for the
            // failure PNG.
            result.diagnostic_fragments =
                suffixFromNode(nodes, last_congestion.parent);
            result.diagnostic_fragments.insert(
                result.diagnostic_fragments.begin(),
                last_congestion.edge_from_parent);
            result.failure_tile = last_congestion.tile;
            result.failure_dst = last_congestion.dst;
            return;
        }

        if (diagnostic_node_index >= 0 &&
            static_cast<size_t>(diagnostic_node_index) < nodes.size())
        {
            result.diagnostic_fragments =
                suffixFromNode(nodes, diagnostic_node_index);
            result.failure_tile = nodes[diagnostic_node_index].tile;
            result.failure_dst = nodes[diagnostic_node_index].dst;
        }

    };

    // Packing probes are more expensive than numeric expansion. A caller-owned
    // cursor advances this bounded window across retries instead of rescanning
    // the same rejected prefix or probing every reached frontier at once.
    const size_t max_takeoff_probes = max_probes == 0
                                          ? std::numeric_limits<size_t>::max()
                                          : std::max<size_t>(2, max_probes);
    const bool restrict_to_physical_dsts =
        target_tile.incoming_dst_nodes != NodeMask{};
    pin_nodes.for_each_set_bit(
        [&](int pin)
        {
            for (const fpga::CBType::TerminalEntry &path :
                 pnr::Docking::terminalEntries(*(target_tile.cb_type), pin))
            {
                // A shared CB type can contain edge-only arrivals. Seed reverse
                // routing only from destination nodes physically reachable at
                // this coordinate.
                if (restrict_to_physical_dsts &&
                    !target_tile.incoming_dst_nodes.testBit(path.dst))
                {
                    if (auto *out = trace())
                        *out << "BACKWARD_SEED local=" << pin
                             << " dst=" << path.dst
                             << " result=no_physical_incoming\n";
                    continue;
                }

                fpga::Wire enter;
                enter.from = target_tile.coord;
                enter.to = target_tile.coord;
                enter.local = path.dst;
                enter.joint = path.joint;
                enter.joint2 = path.joint2;
                enter.pos = ROUTE_POS_TRANSIT;
                if (const std::string *name = target_tile.cb_type->nodeName(
                        fpga::CB_NODE_DST, path.dst))
                {
                    enter.dst_wire_name = *name;
                }

                fpga::Wire tile_pin;
                tile_pin.type = fpga::Wire::WIRE_TILE_PIN;
                tile_pin.from = target_tile.coord;
                tile_pin.to = target_tile.coord;
                tile_pin.local = pin;
                tile_pin.pos = ROUTE_POS_TRANSIT;
                // Even a fully blocked endpoint is a concrete attempted path.
                // Retain it unless reverse expansion later provides a deeper
                // diagnostic suffix.
                if (result.diagnostic_fragments.empty())
                {
                    result.diagnostic_fragments = {enter, tile_pin};
                }

                if (target_tile.isPinNodeLeased(pin) ||
                    !canLeaseIn(state(target_tile), path.dst, pin, path.joint,
                                path.joint2))
                {
                    if (trace())
                        RouteCongestionTrace::current->blocked(
                            target_tile, "backward_seed_busy",
                            {{fpga::CB_NODE_LOCAL, pin},
                             {fpga::CB_NODE_DST, path.dst},
                             {fpga::CB_NODE_JOINT, path.joint},
                             {fpga::CB_NODE_JOINT, path.joint2}},
                            &state(target_tile));
                    continue;
                }

                Node seed{&target_tile, path.dst, enter.dst_wire_name, -1,
                          {},           0,        {enter, tile_pin}};
                if (!tile_visits.append(-1, &target_tile,
                                        seed.tile_visits_root))
                {
                    return false;
                }

                Key key = nodeKey(seed);
                if (seen.emplace(key, static_cast<int>(nodes.size())).second)
                {
                    if (auto *out = trace())
                        *out << "BACKWARD_SEED local=" << pin
                             << " dst=" << path.dst << " joint=" << path.joint
                             << " joint2=" << path.joint2
                             << " result=enqueue index=" << nodes.size()
                             << '\n';
                    frontier.push_back(static_cast<int>(nodes.size()));
                    nodes.push_back(std::move(seed));
                }

            }

            return false;
        });
    if (frontier.empty())
    {
        return result;
    }

    BackwardResolveIndex local_index;
    BackwardResolveIndex *index = prebuilt_backward_index;
    if (!index)
    {
        local_index = buildBackwardResolveIndex(fpga::Device::current(),
                                                target_tile.coord, radius);
        index = &local_index;
    }

    auto accept_takeoff = [&](const BackwardResolveSource &source,
                              int node_index,
                              const BackwardTakeoffProbe &selected_probe,
                              bool use_path_probe = false)
    {
        const Node &node = nodes[static_cast<size_t>(node_index)];
        std::vector<fpga::Wire> proposed;
        auto materialize = [&](const BackwardTakeoffChoice &choice)
            -> const std::vector<fpga::Wire> &
        {
            if (proposed.empty())
            {
                proposed.emplace_back();
                auto suffix = suffixFromNode(nodes, node_index);
                proposed.insert(proposed.end(),
                                std::make_move_iterator(suffix.begin()),
                                std::make_move_iterator(suffix.end()));
            }

            fpga::Wire &edge = proposed.front();
            edge.from = source.tile->coord;
            edge.to = node.tile->coord;
            edge.local = choice.local;
            edge.jump = source.src;
            edge.route_jump = source.route_jump;
            edge.dst = node.dst;
            edge.joint = choice.joint;
            edge.joint2 = choice.joint2;
            edge.pos = 0;
            markGraphArrivals(proposed);
            return proposed;
        };

        BackwardTakeoffChoice takeoff;
        if (auto *out = trace())
            *out << "PROBE_BEGIN tile=(" << source.tile->coord.x << ','
                 << source.tile->coord.y << ") src=" << source.src
                 << " parent=" << node_index << " placement=" << use_path_probe
                 << '\n';
        bool accepted = use_path_probe && path_probe
                            ? path_probe(*source.tile, source.src, takeoff,
                                         std::ref(materialize))
                            : selected_probe(*source.tile, source.src, takeoff);
        if (auto *out = trace())
            *out << "PROBE_END tile=(" << source.tile->coord.x << ','
                 << source.tile->coord.y << ") src=" << source.src
                 << " accepted=" << accepted << " local=" << takeoff.local
                 << " joint=" << takeoff.joint << " joint2=" << takeoff.joint2
                 << " counted=" << takeoff.counts_toward_probe_limit << '\n';
        if (takeoff.counts_toward_probe_limit)
        {
            ++result.probe_calls;
        }

        if (!accepted || takeoff.local < 0 ||
            !canUsePhysicalJump(state(*source.tile), takeoff.local, source.src,
                                takeoff.joint, takeoff.joint2, true))
        {
            if (auto *out = trace())
                *out << "PROBE_REJECT reason="
                     << (!accepted           ? "placement_or_input_proof"
                         : takeoff.local < 0 ? "no_local"
                                             : "takeoff_busy")
                     << '\n';
            return false;
        }

        materialize(takeoff);
        fpga::Wire &edge = proposed.front();
        edge.from_wire_name =
            fromWireName(*source.tile, fpga::CB_NODE_LOCAL, takeoff.local,
                         source.src, takeoff.joint, {});
        edge.src_wire_name =
            srcWireName(*source.tile, fpga::CB_NODE_LOCAL, takeoff.local,
                        source.src, takeoff.joint, {});
        edge.dst_wire_name = node.dst_wire;
        result.fragments = std::move(proposed);
        if (!combinatorialRouteVisitsValid(result.fragments))
        {
            if (auto *out = trace())
                *out << "PROBE_REJECT reason=tile_visit_limit\n";
            ++result.tile_visit_reject_count;
            result.fragments.clear();
            return false;
        }

        result.source_tile = source.tile;
        result.source_src = source.src;
        result.takeoff = takeoff;
        result.success = true;
        return true;
    };

    // Extend an existing partial route only through its exact numeric landing.
    // The landing remains owned by the prefix; the new suffix leases its SRC.
    auto accept_anchor =
        [&](const BackwardResolveSource &source, int node_index)
    {
        if (!anchors || !source.tile || !source.tile->cb_type)
        {
            return false;
        }

        auto tile_anchors = anchor_dsts_by_tile.find(source.tile);
        if (tile_anchors == anchor_dsts_by_tile.end())
        {
            return false;
        }

        source.tile->cb_type->ensureDerivedMasks();
        NodeMask candidate_dsts =
            tile_anchors->second &
            source.tile->cb_type->dsts_reaching_src.get(source.src).jump;
        return candidate_dsts.for_each_set_bit(
            [&](int anchor_dst)
            {
                Key key{source.tile->coord.x, source.tile->coord.y, anchor_dst};
                auto found = anchor_by_node.find(key);
                if (found == anchor_by_node.end())
                {
                    return false;
                }

                const BackwardRouteAnchor &anchor = *found->second;
                ++result.anchor_candidates;
                int joint2 = -1;
                int joint = selectJointToSrc(*source.tile, fpga::CB_NODE_DST,
                                             anchor.dst, source.src, &joint2);
                if (joint == -2 ||
                    !canUsePhysicalJump(state(*source.tile), anchor.dst,
                                        source.src, joint, joint2, true))
                {
                    return false;
                }

                const Node &node = nodes[static_cast<size_t>(node_index)];
                fpga::Wire edge;
                edge.from = source.tile->coord;
                edge.to = node.tile->coord;
                edge.local = anchor.dst;
                edge.jump = source.src;
                edge.route_jump = source.route_jump;
                edge.dst = node.dst;
                edge.joint = joint;
                edge.joint2 = joint2;
                edge.pos = ROUTE_POS_FORK;
                edge.owns_dst = false;
                edge.from_wire_name = anchor.dst_wire;
                edge.src_wire_name =
                    srcWireName(*source.tile, fpga::CB_NODE_DST, anchor.dst,
                                source.src, joint, anchor.dst_wire);
                edge.dst_wire_name = node.dst_wire;
                result.fragments = {edge};
                std::vector<fpga::Wire> suffix =
                    suffixFromNode(nodes, node_index);
                result.fragments.insert(result.fragments.end(), suffix.begin(),
                                        suffix.end());
                markGraphArrivals(result.fragments);
                if (!combinatorialRouteVisitsValid(result.fragments))
                {
                    ++result.tile_visit_reject_count;
                    result.fragments.clear();
                    return false;
                }

                result.source_tile = source.tile;
                result.source_src = source.src;
                result.anchor_id = anchor.id;
                result.completed_from_anchor = true;
                result.success = true;
                return true;
            });
    };

    size_t candidate_index = 0;
    result.probe_offset_used = probe_offset;
    // Consume each reached takeoff once, nearest to the old source within this
    // layer. Ordinary fixed-source callers retain their complete-walk policy.
    auto probe_takeoffs = [&]()
    {
        for (auto &distance_bucket : takeoffs_by_distance)
        {
            for (const FrontierTakeoff &candidate : distance_bucket)
            {
                if (cancel && cancel())
                {
                    return true;
                }

                if (source_radius >= 0 &&
                    std::max(std::abs(candidate.source.tile->coord.x -
                                      source_hint.x),
                             std::abs(candidate.source.tile->coord.y -
                                      source_hint.y)) > source_radius)
                {
                    continue;
                }

                if (candidate_index++ < probe_offset)
                {
                    continue;
                }

                // A full budget only interrupts when another candidate needs
                // probing; rejecting the final candidate can still exhaust the
                // free component.
                if (result.probe_calls >= max_takeoff_probes)
                {
                    return true;
                }

                ++result.probe_candidates_scanned;
                if (accept_takeoff(candidate.source, candidate.node_index,
                                   probe, true))
                {
                    return true;
                }

                // A cancelled or budget-deferred probe has not proved this SRC
                // failed.
                if (cancel && cancel())
                    return true;
            }

            distance_bucket.clear();
        }

        return false;
    };

    size_t layer_end = frontier.size();

    using InternalNode = std::pair<fpga::CBNodeNameType, int>;
    using InternalInputs =
        std::unordered_map<unsigned, std::vector<InternalNode>>;
    std::unordered_map<const fpga::CBType *, InternalInputs> bridge_inputs;
    auto local_bridges = [&](fpga::Tile &tile, int src)
    {
        std::vector<std::vector<InternalNode>> paths;
        const auto &live = state(tile);
        if (live.src.jump.testBit(src))
            return paths;
        auto [cache, inserted] = bridge_inputs.try_emplace(tile.cb_type);
        auto key = [](fpga::CBNodeNameType kind, int node)
        { return (unsigned(kind) << 16) | unsigned(node); };
        if (inserted)
        {
            const auto &cb = *tile.cb_type;
            auto invert = [&](const auto &table, auto mask,
                              fpga::CBNodeNameType from,
                              fpga::CBNodeNameType to)
            {
                for (const auto &[node, row] : table.values)
                    mask(row).for_each_set_bit(
                        [&](int next)
                        {
                            cache->second[key(to, next)].emplace_back(from,
                                                                      node);
                            return false;
                        });
            };

            auto locals = [](const auto &row) -> const NodeMask &
            { return row.local; };
            auto joints = [](const auto &row) -> const NodeMask &
            { return row.joint; };
            auto jumps = [](const auto &row) -> const NodeMask &
            { return row.jump; };
            invert(cb.local_src, jumps, fpga::CB_NODE_LOCAL, fpga::CB_NODE_SRC);
            invert(cb.joint_src, jumps, fpga::CB_NODE_JOINT, fpga::CB_NODE_SRC);
            invert(cb.local_local, locals, fpga::CB_NODE_LOCAL,
                   fpga::CB_NODE_LOCAL);
            invert(cb.joint_local, locals, fpga::CB_NODE_JOINT,
                   fpga::CB_NODE_LOCAL);
            invert(cb.dst_local, locals, fpga::CB_NODE_DST,
                   fpga::CB_NODE_LOCAL);
            invert(cb.local_joint, joints, fpga::CB_NODE_LOCAL,
                   fpga::CB_NODE_JOINT);
            invert(cb.joint_joint, joints, fpga::CB_NODE_JOINT,
                   fpga::CB_NODE_JOINT);
            invert(cb.dst_joint, joints, fpga::CB_NODE_DST,
                   fpga::CB_NODE_JOINT);
            invert(cb.src_joint, joints, fpga::CB_NODE_SRC,
                   fpga::CB_NODE_JOINT);
        }

        const auto reserved = fpga::packedInputJointReservations(tile);
        struct Step
        {
            InternalNode node;
            int next;
            bool local;
        };

        std::vector<Step> pending{{{fpga::CB_NODE_SRC, src}, -1, false}};
        NodeMask visited[2][5]{};
        visited[0][fpga::CB_NODE_SRC].setBit(src);
        visited[1][fpga::CB_NODE_SRC].setBit(src);
        for (size_t cursor = 0; cursor < pending.size(); ++cursor)
        {
            const auto current = pending[cursor];
            auto predecessors = cache->second.find(
                key(current.node.first, current.node.second));
            if (predecessors == cache->second.end())
                continue;
            for (auto [kind, node] : predecessors->second)
            {
                const bool local = current.local || kind == fpga::CB_NODE_LOCAL;
                if (visited[local][kind].testBit(node))
                    continue;
                bool repeated = false;
                for (int at = int(cursor); at >= 0; at = pending[at].next)
                    if (pending[at].node == InternalNode{kind, node})
                    {
                        repeated = true;
                        break;
                    }

                if (repeated)
                    continue;
                if (kind == fpga::CB_NODE_LOCAL &&
                    (live.local.local.testBit(node) ||
                     tile.isPinNodeLeased(node) ||
                     inputLocalReserved(tile, node)))
                    continue;
                if (kind == fpga::CB_NODE_JOINT &&
                    (live.joint.jump.testBit(node) || reserved.testBit(node)))
                    continue;
                if (kind == fpga::CB_NODE_SRC && live.src.jump.testBit(node))
                    continue;
                if (kind == fpga::CB_NODE_DST)
                {
                    // Ordinary DST/joint/SRC paths retain their existing fast
                    // search and preemption checks. This adds only free paths
                    // crossing LOCAL nodes.
                    if (!local || live.dst.jump.testBit(node))
                        continue;
                    std::vector<InternalNode> path{{kind, node}};
                    for (int at = int(cursor); at >= 0; at = pending[at].next)
                        path.push_back(pending[at].node);
                    paths.push_back(std::move(path));
                }

                else
                    pending.push_back({{kind, node}, int(cursor), local});
                visited[local][kind].setBit(node);
            }

        }

        return paths;
    };

    auto append_previous = [&](const BackwardResolveSource &source, int parent,
                               int previous_dst, int joint, int joint2,
                               int visits, std::vector<int> &pending)
    {
        const Node node = nodes[parent];
        Node previous{source.tile, previous_dst,   {}, parent,
                      {},          node.depth + 1, {}};
        previous.tile_visits_root = visits;
        if (const auto *name =
                source.tile->cb_type->nodeName(fpga::CB_NODE_DST, previous_dst))
            previous.dst_wire = *name;
        auto &edge = previous.edge_from_parent;
        edge.from = source.tile->coord;
        edge.to = node.tile->coord;
        edge.local = previous_dst;
        edge.jump = source.src;
        edge.route_jump = source.route_jump;
        edge.dst = node.dst;
        edge.joint = joint;
        edge.joint2 = joint2;
        edge.pos = ROUTE_POS_TRANSIT;
        edge.from_wire_name = previous.dst_wire;
        edge.src_wire_name = srcWireName(*source.tile, fpga::CB_NODE_DST,
                                         previous_dst, source.src, joint, {});
        edge.dst_wire_name = node.dst_wire;
        const Key key = nodeKey(previous);
        auto [found, inserted] = seen.emplace(key, int(nodes.size()));
        if (inserted)
        {
            pending.push_back(int(nodes.size()));
            nodes.push_back(std::move(previous));
        }

        else if (previous.depth < nodes[found->second].depth)
        {
            nodes[found->second] = std::move(previous);
            pending.push_back(found->second);
        }

        return found->second;
    };

    // This is combinatorial reverse search, not the direction-driven engine.
    // Finish each depth layer so one long lane cannot starve other free
    // entries.
    for (;;)
    {
        while (frontier_cursor < frontier.size() &&
               (max_expansions == 0 || result.expanded < max_expansions))
        {
            if (cancel && cancel())
            {
                result.remaining_frontier = frontier.size() - frontier_cursor;
                if (auto *out = trace())
                    *out << "BACKWARD_STOP reason=cancelled\n";
                retain_diagnostic_path();
                return result;
            }

            if (boundary_probe && frontier_cursor == layer_end)
            {
                // Probe early, but leave live owners untouched while free
                // continuations remain. Keep the bounded blocker sample for
                // actual search exhaustion.
                if (probe_takeoffs())
                {
                    result.remaining_frontier =
                        frontier.size() - frontier_cursor;
                    retain_diagnostic_path();
                    return result;
                }

                layer_end = frontier.size();
            }

            int node_index = frontier[frontier_cursor++];
            const Node node = nodes[static_cast<size_t>(node_index)];
            if (auto *out = trace())
                *out << "BACKWARD_POP index=" << node_index
                     << " parent=" << node.parent << " tile=("
                     << node.tile->coord.x << ',' << node.tile->coord.y
                     << ") dst=" << node.dst << " depth=" << node.depth
                     << " frontier=" << frontier.size() - frontier_cursor
                     << '\n';
            // Keep the deepest concrete failed suffix; later shallow frontier
            // pops must not erase the useful path that explains where reverse
            // search got.
            if (node.depth > diagnostic_node_depth)
            {
                diagnostic_node_index = node_index;
                diagnostic_node_depth = node.depth;
                result.diagnostic_incoming_sources = 0;
                result.diagnostic_cached_sources = 0;
                result.diagnostic_window_sources = 0;
                result.diagnostic_key_was_resolved = false;
                result.diagnostic_depth = node.depth;
                result.diagnostic_depth_limit_reached = false;
                result.diagnostic_previous_dsts = 0;
                result.diagnostic_free_previous_dsts = 0;
                result.diagnostic_children = 0;
            }

            result.failure_tile = node.tile;
            result.failure_dst = node.dst;
            if (!InputRepairBudget::expand())
            {
                result.remaining_frontier =
                    frontier.size() - frontier_cursor + 1;
                retain_diagnostic_path();
                return result;
            }

            ++result.expanded;
            // Driver placement may finish away from the angle hint. Marks
            // suppress transit below, after offering direct driver completion.
            if (TraceRouteBackward::isDeadend(*node.tile, node.dst, source_hint,
                                              *index))
                ++result.failure_cache_hits;

            Key current_key = nodeKey(node);
            if (node_index == diagnostic_node_index && index->shared_cache)
            {
                result.diagnostic_key_was_resolved =
                    index->resolved_keys.contains(current_key);
                auto cached = index->shared_cache->incoming.find(current_key);
                if (cached != index->shared_cache->incoming.end())
                {
                    result.diagnostic_cached_sources = cached->second.size();
                    result.diagnostic_window_sources =
                        static_cast<size_t>(std::count_if(
                            cached->second.begin(), cached->second.end(),
                            [&](const BackwardResolveSource &source)
                            {
                                return source.tile &&
                                       inDockWindow(source.tile->coord,
                                                    index->center,
                                                    index->radius);
                            }));
                }

            }

            const std::vector<BackwardResolveSource> *incoming =
                resolveBackwardSources(*index, current_key);
            if (!incoming)
            {
                if (auto *out = trace())
                    *out << "BACKWARD_REJECT index=" << node_index
                         << " reason=no_incoming\n";
                continue;
            }

            if (node_index == diagnostic_node_index)
            {
                result.diagnostic_incoming_sources = incoming->size();
            }

            if (max_depth > 0 && node.depth >= max_depth)
            {
                if (node_index == diagnostic_node_index)
                {
                    result.diagnostic_depth_limit_reached = true;
                }

                continue;
            }

            // Bounded reverse docking tries every incoming physical source.
            std::vector<int> children;
            for (const BackwardResolveSource &source : *incoming)
            {
                if (!source.tile || !source.tile->cb_type)
                {
                    continue;
                }

                int source_visits_root = node.tile_visits_root;
                if (source.tile != node.tile &&
                    !tile_visits.append(node.tile_visits_root, source.tile,
                                        source_visits_root))
                {
                    if (auto *out = trace())
                        *out << "BACKWARD_REJECT index=" << node_index
                             << " source=(" << source.tile->coord.x << ','
                             << source.tile->coord.y << ") src=" << source.src
                             << " reason=tile_visit_limit\n";
                    ++result.tile_visit_reject_count;
                    continue;
                }

                int src = source.src;
                ++result.incoming_edges;
                if (accept_anchor(source, node_index))
                {
                    return result;
                }

                int source_dx = std::abs(source.tile->coord.x - source_hint.x);
                int source_dy = std::abs(source.tile->coord.y - source_hint.y);
                int source_distance = std::max(source_dx, source_dy);
                TakeoffProbeKey takeoff_key{source.tile, src};
                const bool new_source =
                    reached_sources.insert(takeoff_key).second;
                if (preferred_probe && probe_offset == 0)
                {
                    ++result.takeoff_candidates;
                    ++result.probe_candidates_scanned;
                    if (accept_takeoff(source, node_index, preferred_probe))
                    {
                        return result;
                    }

                }

                if ((probe || path_probe) && new_source)
                {
                    if (takeoffs_by_distance.size() <=
                        static_cast<size_t>(source_distance))
                    {
                        takeoffs_by_distance.resize(
                            static_cast<size_t>(source_distance) + 1);
                    }

                    takeoffs_by_distance[static_cast<size_t>(source_distance)]
                        .push_back({source, node_index});
                    if (resume_boundary)
                        reached_takeoffs.push_back({source, node_index});
                    if (source_radius < 0 || source_distance <= source_radius)
                    {
                        ++result.takeoff_candidates;
                    }

                }

                bool in_source_neighborhood =
                    source_radius < 0 || source_distance <= source_radius;
                if (in_source_neighborhood)
                {
                    ++result.source_neighborhood_edges;
                }

                if (node.tile->cb.dst_deadend.jump.testBit(node.dst))
                    continue;
                source.tile->cb_type->ensureDerivedMasks();
                NodeMask previous_dsts =
                    source.tile->cb_type->dsts_reaching_src.get(src).jump;
                previous_dsts.for_each_set_bit(
                    [&](int previous_dst)
                    {
                        if (node_index == diagnostic_node_index)
                        {
                            ++result.diagnostic_previous_dsts;
                        }

                        int joint2 = -1;
                        int joint =
                            selectJointToSrc(*source.tile, fpga::CB_NODE_DST,
                                             previous_dst, src, &joint2);
                        if (joint == -2)
                        {
                            if (auto *out = trace())
                                *out << "BACKWARD_REJECT index=" << node_index
                                     << " source=(" << source.tile->coord.x
                                     << ',' << source.tile->coord.y
                                     << ") dst=" << previous_dst
                                     << " src=" << src
                                     << " reason=no_usable_joint\n";
                            if (trace())
                                RouteCongestionTrace::current->blocked(
                                    *source.tile, "backward_joint_unavailable",
                                    {{fpga::CB_NODE_DST, previous_dst},
                                     {fpga::CB_NODE_SRC, src}},
                                    &state(*source.tile));
                            return false;
                        }

                        const fpga::CBState &source_state = state(*source.tile);
                        if (!canUsePhysicalJump(source_state, previous_dst, src,
                                                joint, joint2))
                        {
                            if (auto *out = trace())
                            {
                                *out << "BACKWARD_EDGE parent=" << node_index
                                     << " from=(" << source.tile->coord.x << ','
                                     << source.tile->coord.y
                                     << ") dst=" << previous_dst
                                     << " src=" << src << " to=("
                                     << node.tile->coord.x << ','
                                     << node.tile->coord.y
                                     << ") landing=" << node.dst
                                     << " result=busy\n";
                                RouteCongestionTrace::current->blocked(
                                    *source.tile, "backward_takeoff_busy",
                                    {{fpga::CB_NODE_DST, previous_dst},
                                     {fpga::CB_NODE_SRC, src},
                                     {fpga::CB_NODE_JOINT, joint},
                                     {fpga::CB_NODE_JOINT, joint2}},
                                    &source_state);
                            }

                            ++result.blocked_reverse_edges;
                            // Remember the last rejected numeric hop even
                            // after the bounded preemption sample fills;
                            // diagnostics must not select an old hop.
                            last_congestion.tile = source.tile;
                            last_congestion.dst = previous_dst;
                            last_congestion.parent = node_index;
                            auto &edge = last_congestion.edge_from_parent;
                            edge.from = source.tile->coord;
                            edge.to = node.tile->coord;
                            edge.local = previous_dst;
                            edge.jump = src;
                            edge.route_jump = source.route_jump;
                            edge.dst = node.dst;
                            edge.joint = joint;
                            edge.joint2 = joint2;
                            edge.pos = ROUTE_POS_TRANSIT;
                            // The report keeps a bounded sample. A caller
                            // that can release a boundary must retain every
                            // transition, including later cuts.
                            if (boundary_probe ||
                                result.blocked_reverse_frontier.size() < 256)
                            {
                                DockingBridgeBlocker blocker;
                                blocker.valid = true;
                                blocker.tile = source.tile->coord;
                                blocker.dst = previous_dst;
                                blocker.src = src;
                                blocker.joint = joint;
                                blocker.joint2 = joint2;
                                blocker.landing_tile = node.tile->coord;
                                blocker.landing_dst = node.dst;
                                blocker.dst_busy =
                                    source_state.dst.jump.testBit(previous_dst);
                                blocker.src_busy =
                                    source_state.src.jump.testBit(src);
                                blocker.joint_busy =
                                    joint >= 0 &&
                                    source_state.joint.jump.testBit(joint);
                                blocker.joint2_busy =
                                    joint2 >= 0 &&
                                    source_state.joint.jump.testBit(joint2);
                                if (boundary_probe)
                                {
                                    blocked_transitions.push_back(
                                        {source, node_index, previous_dst,
                                         source_visits_root, joint, joint2});
                                }

                                if (result.blocked_reverse_frontier.size() <
                                    256)
                                    result.blocked_reverse_frontier.push_back(
                                        std::move(blocker));
                            }

                            return false;
                        }

                        if (node_index == diagnostic_node_index)
                        {
                            ++result.diagnostic_free_previous_dsts;
                        }

                        Node previous{source.tile, previous_dst, {},
                                      node_index,  {},           node.depth + 1,
                                      {}};
                        previous.tile_visits_root = source_visits_root;
                        if (const std::string *name =
                                source.tile->cb_type->nodeName(
                                    fpga::CB_NODE_DST, previous_dst))
                        {
                            previous.dst_wire = *name;
                        }

                        previous.edge_from_parent.from = source.tile->coord;
                        previous.edge_from_parent.to = node.tile->coord;
                        previous.edge_from_parent.local = previous_dst;
                        previous.edge_from_parent.jump = src;
                        previous.edge_from_parent.route_jump =
                            source.route_jump;
                        previous.edge_from_parent.dst = node.dst;
                        previous.edge_from_parent.joint = joint;
                        previous.edge_from_parent.joint2 = joint2;
                        previous.edge_from_parent.pos = ROUTE_POS_TRANSIT;
                        previous.edge_from_parent.from_wire_name =
                            previous.dst_wire;
                        previous.edge_from_parent.src_wire_name =
                            srcWireName(*source.tile, fpga::CB_NODE_DST,
                                        previous_dst, src, joint, {});
                        previous.edge_from_parent.dst_wire_name = node.dst_wire;
                        Key key = nodeKey(previous);
                        const bool inserted =
                            seen.emplace(key, static_cast<int>(nodes.size()))
                                .second;
                        if (auto *out = trace())
                            *out << "BACKWARD_EDGE parent=" << node_index
                                 << " from=(" << source.tile->coord.x << ','
                                 << source.tile->coord.y
                                 << ") dst=" << previous_dst << " src=" << src
                                 << " joint=" << joint << " joint2=" << joint2
                                 << " to=(" << node.tile->coord.x << ','
                                 << node.tile->coord.y
                                 << ") landing=" << node.dst << " result="
                                 << (inserted ? "enqueue" : "already_searched")
                                 << " index=" << seen.at(key) << '\n';
                        if (inserted)
                        {
                            children.push_back(static_cast<int>(nodes.size()));
                            nodes.push_back(std::move(previous));
                            if (node_index == diagnostic_node_index)
                            {
                                ++result.diagnostic_children;
                            }

                        }

                        return false;
                    });
                // A relay LOCAL is an internal routing node, not a new
                // placement or a physical hop. Preserve its actual typed
                // edges in the same suffix and apply the normal depth,
                // Tile-repeat and failure-proof rules.
                for (const auto &path : local_bridges(*source.tile, src))
                {
                    const int dst = path.front().second;
                    const Key key{source.tile->coord.x, source.tile->coord.y,
                                  dst};
                    auto [found, inserted] =
                        seen.emplace(key, int(nodes.size()));
                    if (inserted)
                    {
                        Node previous{source.tile,    dst, {}, node_index, {},
                                      node.depth + 1, {}};
                        previous.tile_visits_root = source_visits_root;
                        for (size_t i = 1; i < path.size(); ++i)
                        {
                            fpga::Wire edge;
                            edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
                            edge.from = edge.to = source.tile->coord;
                            edge.from_node_type = path[i - 1].first;
                            edge.from_node = path[i - 1].second;
                            edge.to_node_type = path[i].first;
                            edge.to_node = path[i].second;
                            previous.internal_prefix.push_back(std::move(edge));
                        }

                        auto &edge = previous.edge_from_parent;
                        edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
                        edge.from = source.tile->coord;
                        edge.to = node.tile->coord;
                        edge.from_node_type = fpga::CB_NODE_SRC;
                        edge.from_node = src;
                        edge.to_node_type = fpga::CB_NODE_DST;
                        edge.to_node = node.dst;
                        children.push_back(int(nodes.size()));
                        nodes.push_back(std::move(previous));
                    }

                }

            }

            // Keep breadth-first depth while visiting all predecessors.
            frontier.insert(frontier.end(), children.begin(), children.end());
        }

        result.expansion_limit_reached = max_expansions != 0 &&
                                         result.expanded >= max_expansions &&
                                         frontier_cursor < frontier.size();
        result.remaining_frontier = frontier.size() - frontier_cursor;

        // Legacy bounded callers can resume a probe window after a complete
        // walk. Layer-wise callers consume candidates as they arrive instead.
        if (!boundary_probe && probe_offset >= result.takeoff_candidates)
        {
            probe_offset = 0;
        }

        result.probe_offset_used = probe_offset;
        const bool probes_stopped = probe_takeoffs();
        // A budget or cancellation is not exhaustion. Only a completed free
        // search may change ownership, and a successful cut ends this state
        // view immediately.
        if (boundary_probe && !probes_stopped &&
            result.remaining_frontier == 0 && !(cancel && cancel()) &&
            !result.blocked_reverse_frontier.empty())
        {
            // A diagnostic sample is not the search frontier. Keep bounded
            // callback allocation, but visit every
            // batch before declaring that no eligible transit boundary exists.
            std::vector<DockingBridgeBlocker> batch;
            batch.reserve(256);
            auto try_batch = [&]()
            {
                const bool released = boundary_probe(batch);
                batch.clear();
                return released;
            };

            for (const auto &blocked : blocked_transitions)
            {
                if (cancel && cancel())
                    break;
                if (blocked.retired)
                    continue;
                DockingBridgeBlocker b;
                b.valid = true;
                b.tile = blocked.source.tile->coord;
                b.dst = blocked.dst;
                b.src = blocked.source.src;
                b.joint = blocked.joint;
                b.joint2 = blocked.joint2;
                b.landing_tile = nodes[blocked.parent].tile->coord;
                b.landing_dst = nodes[blocked.parent].dst;
                const auto &s = state(*blocked.source.tile);
                b.dst_busy = s.dst.jump.testBit(b.dst);
                b.src_busy = s.src.jump.testBit(b.src);
                b.joint_busy = b.joint >= 0 && s.joint.jump.testBit(b.joint);
                b.joint2_busy = b.joint2 >= 0 && s.joint.jump.testBit(b.joint2);
                batch.push_back(std::move(b));
                if (batch.size() == 256 && try_batch())
                {
                    result.boundary_released = true;
                    break;
                }

            }

            if (!result.boundary_released && !batch.empty() &&
                !(cancel && cancel()))
                result.boundary_released = try_batch();
            if (result.boundary_released)
            {
                ++result.boundary_releases;
                std::vector<fpga::Tile *> changed;
                if (resume_boundary && resume_boundary(changed) &&
                    !(cancel && cancel()))
                {
                    result.boundary_released = false;
                    result.blocked_reverse_frontier.clear();
                    const std::unordered_set<fpga::Tile *> touched(
                        changed.begin(), changed.end());
                    for (auto &blocked : blocked_transitions)
                    {
                        if (blocked.retired)
                            continue;
                        if (touched.contains(blocked.source.tile))
                        {
                            int joint2 = -1;
                            const int joint = selectJointToSrc(
                                *blocked.source.tile, fpga::CB_NODE_DST,
                                blocked.dst, blocked.source.src, &joint2);
                            if (joint != -2 &&
                                canUsePhysicalJump(
                                    state(*blocked.source.tile), blocked.dst,
                                    blocked.source.src, joint, joint2))
                            {
                                append_previous(blocked.source, blocked.parent,
                                                blocked.dst, joint, joint2,
                                                blocked.visits, frontier);
                                blocked.retired = true;
                                continue;
                            }

                            blocked.joint = joint;
                            blocked.joint2 = joint2;
                        }

                        if (result.blocked_reverse_frontier.size() < 256)
                        {
                            DockingBridgeBlocker b;
                            b.valid = true;
                            b.tile = blocked.source.tile->coord;
                            b.dst = blocked.dst;
                            b.src = blocked.source.src;
                            b.joint = blocked.joint;
                            b.joint2 = blocked.joint2;
                            b.landing_tile = nodes[blocked.parent].tile->coord;
                            b.landing_dst = nodes[blocked.parent].dst;
                            const auto &s = state(*blocked.source.tile);
                            b.dst_busy = s.dst.jump.testBit(b.dst);
                            b.src_busy = s.src.jump.testBit(b.src);
                            b.joint_busy =
                                b.joint >= 0 && s.joint.jump.testBit(b.joint);
                            b.joint2_busy =
                                b.joint2 >= 0 && s.joint.jump.testBit(b.joint2);
                            result.blocked_reverse_frontier.push_back(
                                std::move(b));
                        }

                    }

                    // A cut can make an earlier placement's CE/data inputs
                    // routable. Recheck its takeoff without re-expanding its
                    // completed free subtree.
                    candidate_index = 0;
                    result.probe_calls = result.probe_candidates_scanned =
                        result.takeoff_candidates = 0;
                    for (const auto &candidate : reached_takeoffs)
                    {
                        const int distance =
                            std::max(std::abs(candidate.source.tile->coord.x -
                                              source_hint.x),
                                     std::abs(candidate.source.tile->coord.y -
                                              source_hint.y));
                        if (source_radius >= 0 && distance > source_radius)
                            continue;
                        takeoffs_by_distance[distance].push_back(candidate);
                        ++result.takeoff_candidates;
                    }

                    layer_end = frontier.size();
                    if (probe_takeoffs())
                    {
                        retain_diagnostic_path();
                        return result;
                    }

                    continue;
                }

            }

        }

        break;
    }

    // A new shorter suffix may change a Tile-repeat/depth choice for an old
    // label. Keep one fresh, free-only confirmation before declaring failure.
    if (resume_boundary && result.boundary_releases && !result.success &&
        !result.boundary_released && !(cancel && cancel()))
    {
        auto fresh = routeBackwardToTakeoffImpl(
            target_tile, pin_nodes, source_hint, max_depth, radius,
            source_radius, probe, prebuilt_backward_index, cancel,
            max_expansions, probe_offset, max_probes, state_view, anchors,
            preferred_probe, path_probe, {});
        if (fresh.success)
        {
            fresh.boundary_releases = result.boundary_releases;
            return fresh;
        }

    }

    retain_diagnostic_path();
    return result;
}

BackwardTakeoffRoute routeBackwardToTakeoff(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int max_depth, int radius, int source_radius,
    const BackwardTakeoffProbe &probe, BackwardResolveIndex *backward_index,
    const BackwardTakeoffCancel &cancel, size_t max_expansions,
    size_t probe_offset, size_t max_probes,
    const BackwardTakeoffStateView *state_view,
    const std::vector<BackwardRouteAnchor> *anchors,
    const BackwardTakeoffProbe &preferred_probe,
    const BackwardTakeoffPathProbe &path_probe,
    const BackwardTakeoffBoundary &boundary_probe,
    const BackwardTakeoffResume &resume_boundary)
{
    return routeBackwardToTakeoffImpl(
        target_tile, pin_nodes, source_hint, max_depth, radius, source_radius,
        probe, backward_index, cancel, max_expansions, probe_offset, max_probes,
        state_view, anchors, preferred_probe, path_probe, boundary_probe,
        resume_boundary);
}

BackwardTakeoffRoute routeBackwardToAnchor(fpga::Tile &target_tile,
                                           NodeMask pin_nodes,
                                           const BackwardRouteAnchor &anchor,
                                           int max_depth, int radius,
                                           BackwardResolveIndex *backward_index,
                                           const BackwardTakeoffCancel &cancel,
                                           size_t max_expansions)
{
    std::vector<BackwardRouteAnchor> anchors{anchor};
    return routeBackwardToAnchors(target_tile, pin_nodes, anchors, max_depth,
                                  radius, backward_index, cancel,
                                  max_expansions);
}

BackwardTakeoffRoute routeBackwardToAnchors(
    fpga::Tile &target_tile, NodeMask pin_nodes,
    const std::vector<BackwardRouteAnchor> &anchors, int max_depth, int radius,
    BackwardResolveIndex *backward_index, const BackwardTakeoffCancel &cancel,
    size_t max_expansions)
{
    return routeBackwardToTakeoff(
        target_tile, pin_nodes,
        !anchors.empty() && anchors.front().tile ? anchors.front().tile->coord
                                                 : target_tile.coord,
        max_depth, radius, -1, {}, backward_index, cancel, max_expansions, 0, 0,
        nullptr, &anchors);
}

DockingResult dockIOB(fpga::Tile &forward_tile, int forward_dst,
                      const std::string &forward_dst_wire,
                      fpga::Tile &target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts,
                      NodeMask reserved_terminal_joints)
{
    constexpr int radius = 12;
    return dockGroundingImpl(
        forward_tile, forward_dst, forward_dst_wire, target_tile, pin_nodes,
        DockingOptions{dockingHopLimit(radius), radius, trace_backward_attempts,
                       reserved_terminal_joints, nullptr});
}

void Docking::retainBackwardBlocked(BackwardTakeoffRoute &result,
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
                prior[i].type != cuts[i].type || prior[i].node != cuts[i].node)
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

void Docking::TerminalSearch::consider(
    const BackwardTerminalNode &current, fpga::Tile *tile,
    fpga::CBNodeNameType type, int node, std::vector<Candidate> &choices,
    BackwardTerminalNode &blocked, int &blocked_angle, int &blocked_distance,
    fpga::Tile &target, fpga::Coord source_hint, int pin,
    const BackwardTerminalIngress &ingress,
    const std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> &seen)
{
    if (!tile || !tile->cb_type || node < 0)
        return;
    if (!inDockWindow(tile->coord, target.coord, 5))
        return;
    if (tile != current.tile)
    {
        unsigned visits = 0;
        const fpga::Tile *previous = nullptr;
        for (const auto &visited : ingress.nodes)
        {
            if (visited.tile != previous && visited.tile == tile)
                ++visits;
            previous = visited.tile;
        }

        if (visits >= 2)
            return;
    }

    // A compact suffix already had its own attempt. Only that complete
    // DST/JOINT sequence is skipped; a shared JOINT may continue
    // farther.
    if (type == fpga::CB_NODE_DST &&
        tried_compact_path(node, ingress, target, pin))
        return;
    if (seen.contains({tile->coord.x, tile->coord.y, type, node}))
        return;
    const int angle = fpga::CBType::anglePriorityRank(
        source_hint - current.tile->coord, tile->coord - current.tile->coord);
    const int distance = std::abs(tile->coord.x - current.tile->coord.x) +
                         std::abs(tile->coord.y - current.tile->coord.y);
    const auto &cb = docking->driverState(*tile);
    const bool busy =
        type == fpga::CB_NODE_LOCAL
            ? cb.local.local.testBit(node) || tile->isPinNodeLeased(node) ||
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

void Docking::TerminalSearch::PhysicalSourceVisitor::operator()(
    fpga::Coord at, fpga::CBNodeNameType type, int node) const
{
    if (type == fpga::CB_NODE_LOCAL)
        trace.consider(current, fpga::Device::current().getTile(at.x, at.y),
                       type, node, choices, blocked, blocked_angle,
                       blocked_distance, target, source_hint, pin, ingress,
                       seen);
}

void Docking::TerminalSearch::debug_ingress(std::ostream *out, fpga::Tile &tile,
                                            fpga::CBNodeNameType type, int node)
{
    if (out)
        *out << "BACKWARD_INGRESS tile=(" << tile.coord.x << ',' << tile.coord.y
             << ") type=" << type << " node=" << node << '\n';
}

// Extend one pin through typed predecessors with a local recursive cursor.
bool Docking::TerminalSearch::run(fpga::Tile &target, fpga::Coord source_hint,
                                  BackwardResolveIndex &index,
                                  std::ostream *decisions,
                                  BackwardTakeoffRoute &result, bool &stopped,
                                  int pin)
{
    BackwardTerminalIngress ingress{&target, -1, pin};
    ingress.nodes.push_back({&target, fpga::CB_NODE_LOCAL, pin});
    std::vector<fpga::Wire> reversed;
    std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> seen;
    seen.insert({target.coord.x, target.coord.y, fpga::CB_NODE_LOCAL, pin});
    return trace(target, source_hint, index, decisions, result, stopped, pin,
                 ingress, reversed, seen);
}

// Try each free typed predecessor in source-angle order, resuming after
// failure.
bool Docking::TerminalSearch::trace(
    fpga::Tile &target, fpga::Coord source_hint, BackwardResolveIndex &index,
    std::ostream *decisions, BackwardTakeoffRoute &result, bool &stopped,
    int pin, BackwardTerminalIngress &ingress,
    std::vector<fpga::Wire> &reversed,
    std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> &seen)
{
    if (docking->driverStopped())
    {
        stopped = true;
        return true;
    }

    if (remaining_expansions == 0)
    {
        result.expansion_limit_reached = true;
        stopped = true;
        return true;
    }

    --remaining_expansions;
    const BackwardTerminalNode current = ingress.nodes.back();
    if (current.type == fpga::CB_NODE_DST)
    {
        const auto *incoming = resolveBackwardSources(
            index, {current.tile->coord.x, current.tile->coord.y, current.node},
            true);
        if ((incoming && !incoming->empty()) ||
            current.tile->incoming_dst_nodes.testBit(current.node))
        {
            ingress.tile = current.tile;
            ingress.dst = current.node;
            ingress.suffix.assign(reversed.rbegin(), reversed.rend());
            fpga::Wire endpoint;
            endpoint.type = fpga::Wire::WIRE_TILE_PIN;
            endpoint.from = endpoint.to = target.coord;
            endpoint.local = pin;
            endpoint.pin_dir = fpga::TILE_PIN_INPUT;
            ingress.suffix.push_back(endpoint);
            if (docking->visitLeasedIngress(ingress, target, result, visitor))
                return true;
        }

    }

    // A marked DST was offered only for a direct source landing above.
    if (current.type == fpga::CB_NODE_DST &&
        current.tile->cb.dst_deadend.jump.testBit(current.node))
        return false;

    // This call keeps its own ordered alternatives while a child recurses.
    std::vector<Candidate> choices;
    BackwardTerminalNode blocked{nullptr, fpga::CB_NODE_LOCAL, -1};
    int blocked_angle = 225;
    int blocked_distance = 1 << 30;
    const auto &cb = *current.tile->cb_type;
    if (current.type == fpga::CB_NODE_LOCAL)
    {
        for (const auto &[dst, row] : cb.dst_local.values)
            if (row.local.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_DST, dst, choices,
                         blocked, blocked_angle, blocked_distance, target,
                         source_hint, pin, ingress, seen);
        for (const auto &[joint, row] : cb.joint_local.values)
            if (row.local.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_JOINT, joint,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
        for (const auto &[local, row] : cb.local_local.values)
            if (row.local.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_LOCAL, local,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
    }

    else if (current.type == fpga::CB_NODE_JOINT)
    {
        for (const auto &[dst, row] : cb.dst_joint.values)
            if (row.joint.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_DST, dst, choices,
                         blocked, blocked_angle, blocked_distance, target,
                         source_hint, pin, ingress, seen);
        for (const auto &[joint, row] : cb.joint_joint.values)
            if (row.joint.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_JOINT, joint,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
        for (const auto &[local, row] : cb.local_joint.values)
            if (row.joint.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_LOCAL, local,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
    }

    visitBackwardPhysicalSources(
        index, current.tile->coord, current.type, current.node,
        PhysicalSourceVisitor{*this, current, choices, blocked, blocked_angle,
                              blocked_distance, target, source_hint, pin,
                              ingress, seen});
    std::stable_sort(choices.begin(), choices.end());
    choices.erase(std::unique(choices.begin(), choices.end()), choices.end());
    for (const auto &choice : choices)
    {
        const auto &node = choice.node;
        const BackwardLocalKey key{node.tile->coord.x, node.tile->coord.y,
                                   node.type, node.node};
        if (!seen.insert(key).second)
            continue;
        debug_ingress(decisions, *node.tile, node.type, node.node);
        reversed.push_back(typed_edge(node, current));
        ingress.nodes.push_back(node);
        if (trace(target, source_hint, index, decisions, result, stopped, pin,
                  ingress, reversed, seen))
            return true;
        ingress.nodes.pop_back();
        reversed.pop_back();
        seen.erase(key);
    }

    retain_typed_diagnostic(current, blocked.tile ? &blocked : nullptr,
                            reversed, target, pin, result);
    return false;
}

// Present the caller's explicit driver view only at the Docking boundary.
Docking::DriverStateScope::DriverStateScope(const Docking &docking)
{
    if (docking.driver_state)
        for (const auto &[tile, state] : *docking.driver_state)
            if (tile)
            {
                saved.emplace_back(tile, tile->cb);
                tile->cb.restoreLeases(state);
            }

}

Docking::DriverStateScope::~DriverStateScope()
{
    for (auto at = saved.rbegin(); at != saved.rend(); ++at)
        at->first->cb.restoreLeases(at->second);
}

// Access the exact typed occupancy bit used by an ingress path.
NodeMask &Docking::terminalMask(fpga::Tile &tile, fpga::CBNodeNameType type)
{
    switch (type)
    {
    case fpga::CB_NODE_LOCAL:
        return tile.cb.local.local;
    case fpga::CB_NODE_JOINT:
        return tile.cb.joint.jump;
    case fpga::CB_NODE_DST:
        return tile.cb.dst.jump;
    default:
        return tile.cb.src.jump;
    }

}

Docking::TerminalLeaseScope::~TerminalLeaseScope()
{
    if (leased)
        for (const auto &node : nodes)
            terminalMask(*node.tile, node.type).clearBit(node.node);
}

// Lease only the selected ingress nodes; the recursive trace leases each hop.
bool Docking::visitLeasedIngress(
    const BackwardTerminalIngress &ingress, fpga::Tile &target,
    BackwardTakeoffRoute &result,
    const std::function<bool(const BackwardTerminalIngress &)> &visitor) const
{
    DriverStateScope view(*this);
    if (ingress.tile->incoming_dst_nodes != NodeMask{} &&
        !ingress.tile->incoming_dst_nodes.testBit(ingress.dst))
        return false;
    for (const auto &node : ingress.nodes)
    {
        if (terminalMask(*node.tile, node.type).testBit(node.node))
            return false;
        if (node.type == fpga::CB_NODE_LOCAL &&
            (node.tile->isPinNodeLeased(node.node) ||
             ((node.tile != &target || node.node != ingress.pin) &&
              inputLocalReserved(*node.tile, node.node))))
            return false;
    }

    TerminalLeaseScope leases{ingress.nodes};
    for (const auto &node : ingress.nodes)
        terminalMask(*node.tile, node.type).setBit(node.node);
    leases.leased = true;
    result.free_terminal_ingress = true;
    return visitor(ingress);
}

// Enumerate bounded terminal ingress paths before handing one to the hop trace.
void Docking::visitBackwardTerminals(
    fpga::Tile &target, NodeMask pins, fpga::Coord source_hint,
    BackwardResolveIndex &index, std::ostream *decisions,
    BackwardTakeoffRoute &result, bool &stopped, size_t &remaining_expansions,
    const std::function<bool(const BackwardTerminalIngress &)> &visitor) const
{
    TerminalSearch typed_walk{this, visitor, remaining_expansions};
    // Preserve the compact ordinary terminal representation and its existing
    // angle/short-hop behavior before considering longer typed ingress chains.
    NodeMask compact_pins = pins;
    bool compact_finished = false;
    for (int pin = compact_pins.firstSetBit(); pin >= 0 && !compact_finished;
         pin = compact_pins.firstSetBit())
    {
        compact_pins.clearBit(pin);
        for (const auto &entry :
             pnr::Docking::terminalEntries(*(target.cb_type), pin))
        {
            BackwardTerminalIngress ingress{&target, entry.dst, pin};
            ingress.nodes.push_back({&target, fpga::CB_NODE_DST, entry.dst});
            ingress.nodes.push_back({&target, fpga::CB_NODE_LOCAL, pin});
            for (int joint : {entry.joint, entry.joint2})
                if (joint >= 0)
                    ingress.nodes.push_back(
                        {&target, fpga::CB_NODE_JOINT, joint});
            fpga::Wire enter;
            enter.from = enter.to = target.coord;
            enter.local = entry.dst;
            enter.joint = entry.joint;
            enter.joint2 = entry.joint2;
            enter.pos = ROUTE_POS_TRANSIT;
            fpga::Wire endpoint;
            endpoint.type = fpga::Wire::WIRE_TILE_PIN;
            endpoint.from = endpoint.to = target.coord;
            endpoint.local = pin;
            endpoint.pin_dir = fpga::TILE_PIN_INPUT;
            ingress.suffix = {enter, endpoint};
            if (visitLeasedIngress(ingress, target, result, visitor))
            {
                compact_finished = true;
                break;
            }

        }

    }

    if (result.success || stopped)
        return;

    // Compact entries cover only short terminal paths. The bounded full
    // search enumerates longer ingress chains and hands each one to the trace.
    NodeMask remaining_pins = pins;
    for (int pin = remaining_pins.firstSetBit(); pin >= 0;
         pin = remaining_pins.firstSetBit())
    {
        remaining_pins.clearBit(pin);
        if (typed_walk.run(target, source_hint, index, decisions, result,
                           stopped, pin))
            break;
    }

    return;
}

// Visit the exact incoming physical arcs used by reverse tracing.
void visitBackwardPhysicalSources(
    BackwardResolveIndex &index, fpga::Coord at, fpga::CBNodeNameType type,
    int node, const fpga::RepairIncomingSourceVisitor &visit)
{
    if (type == fpga::CB_NODE_DST)
    {
        if (const auto *sources =
                resolveBackwardSources(index, {at.x, at.y, node}, true))
            for (const auto &source : *sources)
                if (source.tile)
                    visit(source.tile->coord, fpga::CB_NODE_SRC, source.src);
    }

    const auto &locals = index.shared_cache ? index.shared_cache->local_incoming
                                            : index.local_incoming;
    const auto sources = locals.find({at.x, at.y, type, node});
    if (sources != locals.end())
        for (const auto &source : sources->second)
            if (source.tile)
                visit(source.tile->coord, fpga::CB_NODE_LOCAL, source.local);
}

} // namespace pnr
