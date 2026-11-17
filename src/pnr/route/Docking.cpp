#include "Docking.h"
#include "Device.h"
#include "InputRepairBudget.h"
#include "RepairStatistics.h"
#include "RouteDiagnostics.h"
#include "RouteSearch.h"
#include "RouteSource.h"
#include "RouteTaskTiming.h"
#include "Tile.h"
#include "TraceRouteCommon.h"
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
#include <unordered_set>
#include <utility>

namespace pnr
{
namespace
{

constexpr int ROUTE_POS_SOURCE = 0;
constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;

// Reverse traversal queues children instead of awaiting them on the stack.
// Publish a failed SRC only after its probe and every child have failed.
class BackwardFailureProof
{
    struct Entry
    {
        fpga::Tile *tile = nullptr;
        int src = -1;
        size_t pending = 0;
        size_t pending_boundary = 0;
        bool probe_done = true, closed = false, failed = false;
        std::vector<size_t> parents;
    };

    std::vector<Entry> entries;
    std::vector<size_t> ready;
    size_t &count;
    bool persist_deadends;

    void settle(size_t id)
    {
        ready.push_back(id);
        while (!ready.empty())
        {
            id = ready.back();
            ready.pop_back();
            auto &entry = entries[id];
            if (entry.failed || !entry.closed || !entry.probe_done ||
                entry.pending || entry.pending_boundary)
                continue;
            entry.failed = true;
            if (persist_deadends && entry.src >= 0 &&
                !entry.tile->cb.src_deadend.jump.testBit(entry.src))
            {
                entry.tile->cb.src_deadend.jump.setBit(entry.src);
                ++count;
            }

            for (size_t parent : entry.parents)
            {
                --entries[parent].pending;
                ready.push_back(parent);
            }

        }

    }

  public:
    BackwardFailureProof(size_t &count, bool persist_deadends)
        : count(count), persist_deadends(persist_deadends)
    {
    }

    size_t add(fpga::Tile *tile = nullptr, int src = -1, bool probe_done = true)
    {
        entries.push_back({tile, src, 0, 0, probe_done, false, false, {}});
        return entries.size() - 1;
    }

    void depend(size_t parent, size_t child)
    {
        if (entries[child].failed)
            return;
        ++entries[parent].pending;
        entries[child].parents.push_back(parent);
    }

    void holdBoundary(size_t id)
    {
        ++entries[id].pending_boundary;
    }

    void resolveBoundary(size_t id)
    {
        if (entries[id].pending_boundary)
            --entries[id].pending_boundary;
        settle(id);
    }

    void close(size_t id)
    {
        entries[id].closed = true;
        settle(id);
    }

    void rejectProbe(size_t id)
    {
        entries[id].probe_done = true;
        settle(id);
    }

    void finishExhausted()
    {
        if (!persist_deadends)
            return;

        for (const auto &entry : entries)
            if (!entry.closed || !entry.probe_done || entry.pending_boundary)
                return;
        for (const auto &entry : entries)
            if (entry.src >= 0 &&
                !entry.tile->cb.src_deadend.jump.testBit(entry.src))
            {
                entry.tile->cb.src_deadend.jump.setBit(entry.src);
                ++count;
            }

    }

};

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
                 target_tile.cb_type->terminalEntries(pin))
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
            node.tile->cb_type->srcNodes(from_type, node.dst);
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
            if ((!options.source_local &&
                 node.tile->cb.src_deadend.jump.testBit(src)))
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
            forward_seen[key] = next_index;
            forward_queue.push_back(next_index);
            ++result.forward_push_count;
            if (auto back = backward_seen.find(key);
                back != backward_seen.end())
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
            return options.bounded_reverse
                       ? canUsePhysicalJump(cb, dst, src, joint, joint2,
                                            allow_existing_dst)
                       : canLeaseJump(cb, dst, src, joint, joint2,
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
            if (!prev_tile || !prev_tile->cb_type)
            {
                continue;
            }

            if (!options.bounded_reverse &&
                prev_tile->cb.src_deadend.jump.testBit(src))
                continue;
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

            NodeMask prev_dsts =
                prev_tile->cb_type->dsts_reaching_src.get(src).jump;
            if (prev_dsts == NodeMask{})
            {
                ++result.backward_missing_prev_dst_count;
                // Reverse docking needs a DST predecessor. A LOCAL or JOINT
                // predecessor can still serve generic forward tracing, so it
                // cannot justify a persistent deadend.
                const auto &type = *prev_tile->cb_type;
                const auto local_or_joint_ingress =
                    [&](const auto &entries)
                {
                    return std::any_of(entries.begin(), entries.end(),
                                       [&](const auto &entry)
                                       {
                                           return entry.second.jump.testBit(src);
                                       });
                };
                if (!options.source_local &&
                    !local_or_joint_ingress(type.local_src.values) &&
                    !local_or_joint_ingress(type.joint_src.values))
                {
                    prev_tile->cb.src_deadend.jump.setBit(src);
                }

                record_backward_attempt(
                    node.seed_dst, "no_previous_dst", [&]()
                    { return suffixFromNode(backward_nodes, node_index); });
                continue;
            }

            const int pushed_before = result.backward_push_count;
            const size_t blockers_before = pending_bridge_blockers.size();
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

            const bool already_queued = prev_dsts.for_each_set_bit(
                [&](int previous_dst)
                {
                    return backward_seen.contains(Key{
                        prev_tile->coord.x, prev_tile->coord.y, previous_dst});
                });
            if (!already_queued &&
                pending_bridge_blockers.size() == blockers_before &&
                result.backward_push_count == pushed_before)
                prev_tile->cb.src_deadend.jump.setBit(src);
        }

        return false;
    };

    auto probe_initial_forward_blockers = [&]()
    {
        const Node &node = forward_nodes.front();
        const std::vector<uint16_t> *srcs =
            node.tile->cb_type->srcNodes(fpga::CB_NODE_DST, node.dst);
        if (!srcs)
        {
            return;
        }

        for (uint16_t src : *srcs)
        {
            if (node.tile->cb.src_deadend.jump.testBit(src))
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
        return sink && sink(tile, node, root_dst, terminal);
    if (type != fpga::CB_NODE_LOCAL || !local_terminal ||
        tile.cb.local.local.testBit(node) || tile.isPinNodeLeased(node))
        return false;
    return local_terminal(tile, node, terminal);
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
    return hasDriver() && (*driver_probe)(tile, src, choice);
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
            for (const auto &entry : target.cb_type->terminalEntries(pin))
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
                                canLeaseJump(tile.cb, dst, source.src, joint,
                                             joint2);
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

static bool combinatorialNodePathCanAppend(
    std::span<const fpga::RouteCutNode> prefix, fpga::RouteCutNode next,
    unsigned max_visits)
{
    unsigned visits = 0;
    fpga::Coord previous{-1, -1};
    for (const auto &node : prefix)
    {
        if (node.tile == next.tile && node.type == next.type &&
            node.node == next.node)
            return false;
        if (node.tile == next.tile && node.tile != previous)
            ++visits;
        previous = node.tile;
    }

    return prefix.empty() || previous == next.tile || visits < max_visits;
}

DockingGroundingCandidateResult tryDockingGroundingCandidate(
    const DockingGroundingCandidate &request)
{
    DockingGroundingCandidateResult result;
    if (request.prefix.empty() || request.tail.empty() || !request.allowed ||
        !request.preview)
        return result;

    result.path.assign(request.prefix.begin(), request.prefix.end());
    result.hops = request.prefix_hops;
    std::vector<fpga::RouteCutNode> resources;
    resources.reserve(request.tail.size() + 1);
    auto resource = [](const DockingNumericNode &node)
    {
        const auto [x, y, kind, id] = node;
        return fpga::RouteCutNode{{x, y},
                                  static_cast<fpga::CBNodeNameType>(kind), id};
    };
    resources.push_back(resource(request.prefix.back()));

    for (const auto &next : request.tail)
    {
        const auto next_resource = resource(next);
        if (std::find(result.path.begin(), result.path.end(), next) !=
            result.path.end())
            return result;
        if (!combinatorialNodePathCanAppend(resources, next_resource, 2))
            return result;
        const auto previous = resource(result.path.back());
        result.hops += previous.tile != next_resource.tile;
        if (result.hops > request.max_hops)
            return result;
        if (!request.allowed(result.path.back(), next, {}))
        {
            if (next_resource.type != fpga::CB_NODE_DST &&
                next_resource.type != fpga::CB_NODE_JOINT &&
                next_resource.type != fpga::CB_NODE_SRC)
                return result;
            result.cuts.push_back(next_resource);
            if (!request.allowed(result.path.back(), next, result.cuts))
                return result;
        }
        result.path.push_back(next);
        resources.push_back(next_resource);
    }

    if (result.cuts.empty() ||
        (request.required_branch &&
         std::find(result.path.begin(), result.path.end(),
                   *request.required_branch) == result.path.end()))
        return result;
    if (!request.preview(result.cuts, result.released))
        return result;
    for (size_t i = 1; i < result.path.size(); ++i)
        if (!request.allowed(result.path[i - 1], result.path[i],
                             result.released))
            return result;
    result.success = true;
    return result;
}

DockingGroundingSearchResult searchGroundingFromFrontier(
    const DockingGroundingSearch &request)
{
    struct Step
    {
        DockingNumericNode node;
        int parent = -1;
        int hops = 0;
        int cost = 0;
        int root = -1;
        int prefix_hops = 0;
        int tile_visits_root = -1;
    };

    DockingGroundingSearchResult result;
    std::vector<Step> steps;
    std::map<DockingNumericNode, int> cheapest;
    std::map<DockingNumericNode, bool> cut_eligible;
    std::priority_queue<std::pair<int, int>,
                        std::vector<std::pair<int, int>>, std::greater<>> pending;
    CombinatorialTileVisits tile_visits;
    for (const auto &seed : request.seeds)
    {
        const fpga::Coord coord{std::get<0>(seed.node), std::get<1>(seed.node)};
        int visits_root = -1;
        if (!tile_visits.append(-1, coord, visits_root))
            continue;
        const int index = static_cast<int>(steps.size());
        cheapest[seed.node] = index;
        steps.push_back({seed.node, -1, 0, 0, seed.root,
                         seed.prefix_hops, visits_root});
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
            std::vector<DockingNumericNode> tail;
            for (int at = current; steps[at].parent >= 0;
                 at = steps[at].parent)
                tail.push_back(steps[at].node);
            std::reverse(tail.begin(), tail.end());
            result.found = request.complete(step.root, std::move(tail));
            if (result.found)
                break;
            ++result.rejected_complete;
            continue;
        }

        request.successors(step.node, [&](const DockingNumericNode &next)
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
            if (jump && !tile_visits.append(step.tile_visits_root, to,
                                            visits_root))
                return;

            const bool needs_cut = !request.free_edge(step.node, next);
            const int cost = step.cost + 1 + 4 * jump + 20 * needs_cut;
            auto old = cheapest.find(next);
            if (old != cheapest.end() && steps[old->second].cost <= cost)
                return;
            if (needs_cut)
            {
                const int kind = std::get<2>(next);
                if (kind != fpga::CB_NODE_DST && kind != fpga::CB_NODE_SRC &&
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
    root_nodes.for_each_set_bit([&](int local)
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
            nodes.for_each_set_bit([&](int value)
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
                               bounded_reverse, source_local,
                               preemption, forward_preemption});
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

static BackwardTakeoffRoute routeBackwardToTakeoffImpl(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int max_depth, int radius, int source_radius,
    const BackwardTakeoffProbe &probe,
    BackwardResolveIndex *prebuilt_backward_index,
    const BackwardTakeoffCancel &cancel, size_t max_expansions,
    size_t probe_offset, size_t max_probes,
    const BackwardTakeoffStateView *state_view,
    const std::vector<BackwardRouteAnchor> *anchors,
    const BackwardTakeoffProbe &preferred_probe,
    const BackwardTakeoffPathProbe &path_probe,
    const BackwardTakeoffBoundary &boundary_probe,
    const BackwardTakeoffResume &resume_boundary = {})
{
    BackwardTakeoffRoute result;
    // A caller-supplied state view contains provisional route leases. A path
    // probe may reject a route because its nested inputs fail under that
    // particular output path. A finite probe window may leave later source
    // candidates untested. None proves a live physical SRC exhausted.
    BackwardFailureProof failure_proof(
        result.failed_states,
        state_view == nullptr && !path_probe &&
            (max_probes == 0 || (!probe && !preferred_probe)));
    std::vector<size_t> node_failure_ids;
    std::unordered_map<TakeoffProbeKey, size_t, TakeoffProbeKeyHash>
        source_failure_ids;
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
        size_t proof_id;
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
                 target_tile.cb_type->terminalEntries(pin))
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
                    node_failure_ids.push_back(failure_proof.add());
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
            !canLeaseJump(state(*source.tile), takeoff.local, source.src,
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
                    !canLeaseJump(state(*source.tile), anchor.dst, source.src,
                                  joint, joint2, true))
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

                TakeoffProbeKey key{candidate.source.tile,
                                    candidate.source.src};
                if (key.tile->cb.src_deadend.jump.testBit(key.src))
                {
                    ++result.failure_cache_hits;
                    continue;
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
                failure_proof.rejectProbe(source_failure_ids.at(key));
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
        if (live.src.jump.testBit(src) || tile.cb.src_deadend.jump.testBit(src))
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
            node_failure_ids.push_back(failure_proof.add());
            nodes.push_back(std::move(previous));
        }

        else if (previous.depth < nodes[found->second].depth)
        {
            nodes[found->second] = std::move(previous);
            pending.push_back(found->second);
        }

        return node_failure_ids[found->second];
    };

    // This is combinatorial reverse search, not the direction-driven engine.
    // Finish each depth layer so one long lane cannot starve other free
    // entries.
    bool proof_search_exhausted = false;
    bool boundary_previews_exhausted = false;
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
                failure_proof.close(node_failure_ids[node_index]);
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
            bool complete_node = true;
            for (const BackwardResolveSource &source : *incoming)
            {
                if (!source.tile || !source.tile->cb_type)
                {
                    continue;
                }

                if (source.tile->cb.src_deadend.jump.testBit(source.src))
                {
                    ++result.failure_cache_hits;
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
                    complete_node = false;
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
                auto [proof_it, new_source] =
                    source_failure_ids.try_emplace(takeoff_key, 0);
                if (new_source)
                    proof_it->second = failure_proof.add(
                        source.tile, src,
                        !(probe || path_probe) ||
                            (source_radius >= 0 &&
                             source_distance > source_radius));
                const size_t source_failure_id = proof_it->second;
                failure_proof.depend(node_failure_ids[node_index],
                                     source_failure_id);
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
                        if (!canLeaseJump(source_state, previous_dst, src,
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
                                    failure_proof.holdBoundary(
                                        source_failure_id);
                                    blocked_transitions.push_back(
                                        {source, node_index, previous_dst,
                                         source_visits_root, joint, joint2,
                                         source_failure_id});
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
                            node_failure_ids.push_back(failure_proof.add());
                            nodes.push_back(std::move(previous));
                            if (node_index == diagnostic_node_index)
                            {
                                ++result.diagnostic_children;
                            }

                        }

                        failure_proof.depend(source_failure_id,
                                             node_failure_ids[seen.at(key)]);
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
                        node_failure_ids.push_back(failure_proof.add());
                        nodes.push_back(std::move(previous));
                    }

                    failure_proof.depend(source_failure_id,
                                         node_failure_ids[found->second]);
                }

                failure_proof.close(source_failure_id);
            }

            // Keep breadth-first depth while visiting all predecessors.
            frontier.insert(frontier.end(), children.begin(), children.end());
            if (complete_node)
                failure_proof.close(node_failure_ids[node_index]);
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
                                canLeaseJump(state(*blocked.source.tile),
                                             blocked.dst, blocked.source.src,
                                             joint, joint2))
                            {
                                const size_t child_id = append_previous(
                                    blocked.source, blocked.parent, blocked.dst,
                                    joint, joint2, blocked.visits, frontier);
                                failure_proof.depend(blocked.proof_id,
                                                     child_id);
                                blocked.retired = true;
                                failure_proof.resolveBoundary(blocked.proof_id);
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

        proof_search_exhausted =
            !result.success && !result.boundary_released && !probes_stopped &&
            result.remaining_frontier == 0 && !result.expansion_limit_reached &&
            !InputRepairBudget::stopped() && !(cancel && cancel());
        boundary_previews_exhausted = boundary_probe && proof_search_exhausted;
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

    // A pending cut is not a failed SRC until every boundary preview declines.
    if (boundary_previews_exhausted)
        for (auto &blocked : blocked_transitions)
            if (!blocked.retired)
            {
                blocked.retired = true;
                failure_proof.resolveBoundary(blocked.proof_id);
            }

    if (proof_search_exhausted)
        failure_proof.finishExhausted();
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

} // namespace pnr

// Route ownership, preemption and repair share Docking's live resource checks.
using namespace fpga;

namespace
{

thread_local uint64_t route_mutation_revision = 0;
thread_local RepairStatistics repair_statistics;

bool sameCoord(const Coord &a, const Coord &b)
{
    return a.x == b.x && a.y == b.y;
}

// Compare only routed resource identity; ownership and annotation fields may
// differ. FORK and TRANSIT both consume a DST at the same physical hop;
// their distinct pos values describe ownership, not different connectivity.
bool samePhysicalFragment(const Wire &left, const Wire &right)
{
    return left.type == right.type && sameCoord(left.from, right.from) &&
           sameCoord(left.to, right.to) && left.local == right.local &&
           (left.pos == right.pos || (left.type == Wire::WIRE_CROSSBAR &&
                                      ((left.pos == 1 && right.pos == 2) ||
                                       (left.pos == 2 && right.pos == 1)))) &&
           left.jump == right.jump && left.route_jump == right.route_jump &&
           left.dst == right.dst && left.joint == right.joint &&
           left.joint2 == right.joint2 &&
           left.from_node_type == right.from_node_type &&
           left.from_node == right.from_node &&
           left.to_node_type == right.to_node_type &&
           left.to_node == right.to_node;
}

std::vector<Wire> *bindingRoute(rtl::NetRouteBinding &binding);

// Return the prefix that is physically shared with another live binding from
// the same source port; stale Wire::shared flags alone do not prove ownership.
size_t liveSharedPrefixLength(
    rtl::Net &net, size_t binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded = nullptr)
{
    if (binding_index >= net.routes.size())
    {
        return 0;
    }

    rtl::NetRouteBinding &binding = net.routes[binding_index];
    std::vector<Wire> *route = bindingRoute(binding);
    if (!route || route->empty() || !binding.from || binding.from_port.empty())
    {
        return 0;
    }

    size_t longest = 0;
    std::unordered_set<rtl::Net *> visited;
    auto inspect_net = [&](rtl::Net &candidate_net)
    {
        if (!visited.insert(&candidate_net).second)
        {
            return;
        }

        for (size_t candidate_index = 0;
             candidate_index < candidate_net.routes.size(); ++candidate_index)
        {
            if (&candidate_net == &net && candidate_index == binding_index)
            {
                continue;
            }

            rtl::NetRouteBinding &candidate_binding =
                candidate_net.routes[candidate_index];
            if (excluded && excluded->contains(&candidate_binding))
            {
                continue;
            }

            if (!rtl::sameRouteSource(candidate_binding.from,
                                      candidate_binding.from_port, binding.from,
                                      binding.from_port))
            {
                continue;
            }

            std::vector<Wire> *candidate = bindingRoute(candidate_binding);
            if (!candidate || candidate->empty())
            {
                continue;
            }

            size_t common = 0;
            while (common < route->size() && common < candidate->size() &&
                   (*route)[common].shared &&
                   samePhysicalFragment((*route)[common], (*candidate)[common]))
            {
                ++common;
            }

            longest = std::max(longest, common);
        }

    };

    inspect_net(net);
    Tile *source_tile =
        Device::current().getTile(route->front().from.x, route->front().from.y);
    if (source_tile)
    {
        for (Ref<rtl::Net> &ref : source_tile->routedNets)
        {
            if (ref.peer)
            {
                inspect_net(*ref.peer);
            }

        }

    }

    return longest;
}

// Transfer a removed Generic prefix to one surviving branch before releasing
// leases.
void promoteSurvivingSourcePrefix(
    rtl::Net &net, size_t removed_binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded = nullptr,
    size_t first_removed_fragment = 0)
{
    RouteHistoryScope history(&net, __func__);
    if (removed_binding_index >= net.routes.size())
    {
        return;
    }

    rtl::NetRouteBinding &removed_binding = net.routes[removed_binding_index];
    std::vector<Wire> *removed = bindingRoute(removed_binding);
    if (!removed || removed->empty() || !removed_binding.from ||
        removed_binding.from_port.empty())
    {
        return;
    }

    auto promote_from_net = [&](rtl::Net &candidate_net)
    {
        size_t transferred_count = 0;
        for (size_t sibling_index = 0;
             sibling_index < candidate_net.routes.size(); ++sibling_index)
        {
            if (&candidate_net == &net &&
                sibling_index == removed_binding_index)
            {
                continue;
            }

            rtl::NetRouteBinding &sibling_binding =
                candidate_net.routes[sibling_index];
            if (excluded && excluded->contains(&sibling_binding))
            {
                continue;
            }

            if (!rtl::sameRouteSource(
                    sibling_binding.from, sibling_binding.from_port,
                    removed_binding.from, removed_binding.from_port))
            {
                continue;
            }

            std::vector<Wire> *sibling = bindingRoute(sibling_binding);
            if (!sibling || sibling->empty())
            {
                continue;
            }

            size_t common = 0;
            // Partial repairs can make the source prefix private while a later
            // island still borrows its branching DST. Physical continuity and
            // the canonical source above establish sharing; the first
            // fragment's flag cannot decide whether that island needs its
            // owner's lease. A partial cut gives each binding its own
            // non-shared GAP record and may promote the first retained fragment
            // after it. Neither flag ends the physically common path. Continue
            // through identical gaps/islands so their landing ownership
            // survives a sibling's source replacement.
            while (common < removed->size() && common < sibling->size() &&
                   samePhysicalFragment((*removed)[common], (*sibling)[common]))
            {
                ++common;
            }

            if (common == 0)
            {
                continue;
            }

            // A sibling can fork immediately after the common prefix. Its
            // private FORK does not own the incoming DST: the departing route's
            // next TRANSIT fragment did. Preserve that boundary as a landing of
            // the surviving prefix before source replacement removes the old
            // fragment. Transferring only explicit owns_landing flags misses
            // this owner.
            Wire &tip = (*sibling)[common - 1];
            if (common >= first_removed_fragment && common < removed->size() &&
                tip.type == Wire::WIRE_CROSSBAR && tip.jump >= 0 &&
                tip.dst >= 0)
            {
                Wire &next = (*removed)[common];
                if (next.type == Wire::WIRE_CROSSBAR && !next.shared &&
                    next.pos != 0 && next.owns_dst && next.local == tip.dst &&
                    sameCoord(next.from, tip.to))
                {
                    tip.owns_landing = true;
                    next.owns_dst = false;
                    ++transferred_count;
                }

            }

            for (size_t index = first_removed_fragment; index < common; ++index)
            {
                if ((*removed)[index].type == Wire::WIRE_GAP)
                    continue;
                if ((*removed)[index].owns_landing)
                {
                    (*sibling)[index].owns_landing = true;
                    (*removed)[index].owns_landing = false;
                    ++transferred_count;
                }

                if ((*removed)[index].shared)
                {
                    continue;
                }

                (*sibling)[index].shared = false;
                (*sibling)[index].owns_dst |= (*removed)[index].owns_dst;
                (*removed)[index].shared = true;
                ++transferred_count;
            }

        }

        return transferred_count;
    };

    promote_from_net(net);

    // One physical source port may be represented by several RTL nets. Every
    // surviving sibling sharing this prefix is registered on its first prefix
    // tile.
    std::unordered_set<rtl::Net *> visited{&net};
    Tile *source_tile = Device::current().getTile(removed->front().from.x,
                                                  removed->front().from.y);
    if (source_tile)
    {
        for (Ref<rtl::Net> &ref : source_tile->routedNets)
        {
            rtl::Net *candidate = ref.peer;
            if (!candidate || !visited.insert(candidate).second)
            {
                continue;
            }

            promote_from_net(*candidate);
        }

    }

}

bool routeUsesNodeOnTile(std::span<const Wire> route, const Tile &tile,
                         CBNodeNameType node_type, int node, bool transit_only,
                         bool owned_only, OwnershipQueryWork *work = nullptr)
{
    for (const Wire &fragment : route)
    {
        if (work)
            ++work->fragment_visits;
        // Most fragments in a Tile-registered route belong to other nodes
        // (often other Tiles). Reject by numeric ID before coordinate/ownership
        // work. This is only a negative filter; all existing lease rules follow
        // below.
        if (fragment.type == Wire::WIRE_ROUTE_EDGE)
        {
            if (!(fragment.to_node_type == node_type &&
                  fragment.to_node == node) &&
                !(!owned_only && fragment.from_node_type == node_type &&
                  fragment.from_node == node))
                continue;
        }

        else if (fragment.type == Wire::WIRE_CROSSBAR)
        {
            switch (node_type)
            {
            case CB_NODE_SRC:
                if (fragment.jump != node)
                    continue;
                break;
            case CB_NODE_DST:
                if (fragment.local != node &&
                    !(fragment.owns_landing && fragment.dst == node))
                    continue;
                break;
            case CB_NODE_JOINT:
                if (fragment.joint != node && fragment.joint2 != node)
                    continue;
                break;
            case CB_NODE_LOCAL:
                if (fragment.local != node)
                    continue;
                break;
            default:
                continue;
            }

        }

        else if (fragment.type == Wire::WIRE_TILE_PIN)
        {
            if (node_type != CB_NODE_LOCAL || fragment.local != node)
                continue;
        }

        else
            continue;
        // A shared replica can own an explicitly reserved landing, but that
        // says nothing about its incoming DST on the other end of the hop.
        if (owned_only && fragment.shared &&
            !(fragment.type == Wire::WIRE_CROSSBAR &&
              node_type == CB_NODE_DST && fragment.owns_landing &&
              sameCoord(fragment.to, tile.coord) && fragment.dst == node))
        {
            continue;
        }

        bool from_tile = sameCoord(fragment.from, tile.coord);
        bool to_tile = sameCoord(fragment.to, tile.coord);
        if (!from_tile && !to_tile)
        {
            continue;
        }

        if (fragment.type == Wire::WIRE_ROUTE_EDGE)
        {
            if (!owned_only && from_tile &&
                fragment.from_node_type == node_type &&
                fragment.from_node == node)
            {
                return true;
            }

            if (to_tile && fragment.to_node_type == node_type &&
                fragment.to_node == node)
            {
                return true;
            }

            continue;
        }

        if (node_type == CB_NODE_SRC)
        {
            if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
                fragment.jump == node)
            {
                if (!transit_only || (fragment.pos != 0 && fragment.jump >= 0 &&
                                      fragment.dst >= 0))
                {
                    return true;
                }

            }

            continue;
        }

        if (node_type == CB_NODE_DST)
        {
            // A fragment leases only its source-tile incoming node. Its landing
            // is normally owned by the following fragment, except when a
            // partial route explicitly reserves that landing for its next
            // continuation.
            if (to_tile && fragment.type == Wire::WIRE_CROSSBAR &&
                fragment.owns_landing && fragment.dst == node &&
                (!transit_only || (fragment.jump >= 0 && fragment.dst >= 0)))
            {
                return true;
            }

            if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
                fragment.pos != 0 && fragment.owns_dst &&
                fragment.local == node &&
                (!transit_only || (fragment.jump >= 0 && fragment.dst >= 0)))
            {
                return true;
            }

            continue;
        }

        if (node_type == CB_NODE_LOCAL)
        {
            if (transit_only)
            {
                continue;
            }

            if (fragment.type == Wire::WIRE_TILE_PIN && from_tile &&
                fragment.local == node)
            {
                return true;
            }

            if (fragment.type == Wire::WIRE_CROSSBAR && from_tile &&
                fragment.pos == 0 && fragment.local == node)
            {
                return true;
            }

            continue;
        }

        if (node_type == CB_NODE_JOINT)
        {
            if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
                (fragment.joint == node || fragment.joint2 == node))
            {
                if (!transit_only || fragment.pos != 0)
                {
                    // A SRC -> DST hop can stay inside one CB. Only a terminal
                    // fragment (no SRC/landing DST), not zero distance, is
                    // Grounding.
                    if (transit_only && (fragment.jump < 0 || fragment.dst < 0))
                    {
                        continue;
                    }

                    return true;
                }

            }

            continue;
        }

    }

    return false;
}

std::vector<Wire> *bindingRoute(rtl::NetRouteBinding &binding)
{
    if (!binding.owner || binding.route_index >= binding.owner->wires.size())
    {
        return nullptr;
    }

    return &binding.owner->wires[binding.route_index];
}

void clearTileNetRef(Tile &tile, rtl::Net &net)
{
    tile.removeRoutedBindings(&net);
}

void appendRouteTiles(const std::vector<Wire> &route,
                      std::unordered_set<Tile *> &tiles)
{
    for (const Wire &fragment : route)
    {
        if (Tile *tile =
                Device::current().getTile(fragment.from.x, fragment.from.y))
        {
            tiles.insert(tile);
        }

        if (Tile *tile =
                Device::current().getTile(fragment.to.x, fragment.to.y))
        {
            tiles.insert(tile);
        }

    }

}

// Rebuild only tile registrations touched by removed or retargeted routes.
// A local route edit must not scan the complete device grid.
void rebuildNetRouteTiles(rtl::Net &net,
                          const std::vector<std::vector<Wire>> &affected)
{
    ++route_mutation_revision;
    std::unordered_set<Tile *> affected_tiles;
    for (const std::vector<Wire> &route : affected)
    {
        appendRouteTiles(route, affected_tiles);
    }

    for (Tile *tile : affected_tiles)
    {
        clearTileNetRef(*tile, net);
    }

    for (size_t binding_index = 0; binding_index < net.routes.size();
         ++binding_index)
    {
        rtl::NetRouteBinding &remaining = net.routes[binding_index];
        std::vector<Wire> *remaining_route = bindingRoute(remaining);
        if (remaining_route && !remaining_route->empty())
        {
            registerNetRouteTiles(net, *remaining_route, binding_index);
        }

    }

}

// Resolve a placed instance port to a stable net endpoint reference.
Referable<rtl::Port> *findInstPort(rtl::Inst *inst,
                                   const std::string &port_name)
{
    if (!inst || port_name.empty())
    {
        return nullptr;
    }

    for (auto &conn : inst->conns)
    {
        if (conn.port_ref.peer && conn.port_ref->name == port_name)
        {
            return conn.port_ref.peer;
        }

    }

    return nullptr;
}

void addTileNetRef(Tile &tile, rtl::Net &net)
{
    tile.addRoutedNet(&net);
}

void releaseFragmentWithOwners(const std::vector<Wire> &route,
                               size_t fragment_index,
                               RouteOwnerQueries &owners);

// Accumulate optional work counters without retaining an index across
// mutations.
void addOwnershipWork(OwnershipQueryWork *work, const OwnershipQueryWork &added)
{
    if (!work)
        return;
    work->queries += added.queries;
    work->binding_resolutions += added.binding_resolutions;
    work->fragment_visits += added.fragment_visits;
    work->tile_builds += added.tile_builds;
}

void clearRouteLeases(const std::vector<const std::vector<Wire> *> &routes,
                      bool clear_shared = false,
                      OwnershipQueryWork *work = nullptr)
{
    if (clear_shared)
    {
        // All victim edits precede this batch; clearing bits does not change
        // owners.
        RouteOwnerQueries owners;
        for (const std::vector<Wire> *route : routes)
        {
            if (!route)
            {
                continue;
            }

            for (size_t fragment_index = 0; fragment_index < route->size();
                 ++fragment_index)
            {
                releaseFragmentWithOwners(*route, fragment_index, owners);
            }

        }

        addOwnershipWork(work, owners.work());
        return;
    }

    for (const std::vector<Wire> *route : routes)
    {
        if (!route)
        {
            continue;
        }

        for (size_t i = 0; i < route->size(); ++i)
        {
            const Wire &fragment = (*route)[i];
            if (fragment.type == Wire::WIRE_CROSSBAR && fragment.owns_landing &&
                fragment.dst >= 0)
            {
                Tile *landing_tile =
                    Device::current().getTile(fragment.to.x, fragment.to.y);
                if (landing_tile)
                {
                    landing_tile->cb.dst.jump &=
                        ~(NodeMask{0, 1} << fragment.dst);
                }

            }

            if (fragment.shared)
            {
                continue;
            }

            if (fragment.type == Wire::WIRE_TILE_PIN)
            {
                Tile *route_tile =
                    Device::current().getTile(fragment.from.x, fragment.from.y);
                if (!route_tile || fragment.local < 0)
                {
                    continue;
                }

                NodeMask keep = ~(NodeMask{0, 1} << fragment.local);
                route_tile->pin_state.leased_nodes &= keep;
                route_tile->cb.local.local &= keep;
                // A resource endpoint may attach to a distinct route tile.
                // Full-tree teardown releases both sides so its dedicated
                // router can rebuild it.
                Tile *resource_tile = Device::current().getTile(
                    fragment.resource.x, fragment.resource.y);
                if (resource_tile)
                {
                    resource_tile->pin_state.leased_nodes &= keep;
                }

                continue;
            }

            if (fragment.type == Wire::WIRE_ROUTE_EDGE)
            {
                Tile *tile =
                    Device::current().getTile(fragment.to.x, fragment.to.y);
                if (!tile || fragment.to_node < 0)
                {
                    continue;
                }

                NodeMask clear = ~(NodeMask{0, 1} << fragment.to_node);
                switch (fragment.to_node_type)
                {
                case CB_NODE_LOCAL:
                    // A removed sink may have retained its pin bit while this
                    // LOCAL was still used as transit. Release both bits with
                    // its last owner.
                    tile->cb.local.local &= clear;
                    tile->pin_state.leased_nodes &= clear;
                    break;
                case CB_NODE_JOINT:
                    tile->cb.joint.jump &= clear;
                    break;
                case CB_NODE_SRC:
                    tile->cb.src.jump &= clear;
                    break;
                case CB_NODE_DST:
                    tile->cb.dst.jump &= clear;
                    break;
                default:
                    break;
                }

                continue;
            }

            if (fragment.type != Wire::WIRE_CROSSBAR)
            {
                continue;
            }

            Tile *tile =
                Device::current().getTile(fragment.from.x, fragment.from.y);
            if (!tile)
            {
                continue;
            }

            if (fragment.jump >= 0)
            {
                tile->cb.src.jump &= ~(NodeMask{0, 1} << fragment.jump);
            }

            if (fragment.joint >= 0)
            {
                tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint);
            }

            if (fragment.joint2 >= 0)
            {
                tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint2);
            }

            if (fragment.pos == 0 && fragment.local >= 0)
            {
                tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
                tile->pin_state.leased_nodes &=
                    ~(NodeMask{0, 1} << fragment.local);
            }

            if (fragment.pos != 0 && fragment.local >= 0 && fragment.owns_dst)
            {
                tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.local);
            }

        }

    }

}

void clearRouteLeases(const std::vector<Wire> &route, bool clear_shared = false)
{
    clearRouteLeases(std::vector<const std::vector<Wire> *>{&route},
                     clear_shared);
}

// Query surviving geometry once per Tile, preserving shared and landing leases.
void releaseFragmentWithOwners(const std::vector<Wire> &route,
                               size_t fragment_index, RouteOwnerQueries &owners)
{
    if (fragment_index >= route.size())
    {
        return;
    }

    auto unowned = [&](Tile &tile, CBNodeNameType type, int node)
    {
        return !owners.allMatch(tile, type, node,
                                [](const NetRouteRef &) { return true; });
    };

    const Wire &fragment = route[fragment_index];
    if (fragment.type == Wire::WIRE_CROSSBAR && fragment.owns_landing &&
        fragment.dst >= 0)
    {
        Tile *landing_tile =
            Device::current().getTile(fragment.to.x, fragment.to.y);
        if (landing_tile && unowned(*landing_tile, CB_NODE_DST, fragment.dst))
        {
            landing_tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.dst);
        }

    }

    if (fragment.type == Wire::WIRE_TILE_PIN)
    {
        // Ordinary takeoff LOCAL leases are released by their CROSSBAR
        // fragment. Dedicated trees have only ROUTE_EDGE fragments, which
        // release their destination nodes, never the root. Release that output
        // pin here too.
        const bool dedicated_root = fragment_index == 0 && route.size() > 1 &&
                                    route[1].type == Wire::WIRE_ROUTE_EDGE;
        if (fragment_index + 1 != route.size() && !dedicated_root)
        {
            return;
        }

        Tile *tile =
            Device::current().getTile(fragment.from.x, fragment.from.y);
        if (!tile || fragment.local < 0)
        {
            return;
        }

        if (unowned(*tile, CB_NODE_LOCAL, fragment.local))
        {
            tile->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
            tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
            if (Tile *resource = Device::current().getTile(fragment.resource.x,
                                                           fragment.resource.y))
            {
                resource->pin_state.leased_nodes &=
                    ~(NodeMask{0, 1} << fragment.local);
            }

        }

        if (fragment_index > 0 &&
            route[fragment_index - 1].type == Wire::WIRE_CROSSBAR &&
            route[fragment_index - 1].local >= 0 &&
            route[fragment_index - 1].owns_dst &&
            unowned(*tile, CB_NODE_DST, route[fragment_index - 1].local))
        {
            tile->cb.dst.jump &=
                ~(NodeMask{0, 1} << route[fragment_index - 1].local);
        }

        return;
    }

    if (fragment.type == Wire::WIRE_ROUTE_EDGE)
    {
        Tile *tile = Device::current().getTile(fragment.to.x, fragment.to.y);
        if (!tile || fragment.to_node < 0 ||
            !unowned(*tile, static_cast<CBNodeNameType>(fragment.to_node_type),
                     fragment.to_node))
        {
            return;
        }

        NodeMask clear = ~(NodeMask{0, 1} << fragment.to_node);
        switch (fragment.to_node_type)
        {
        case CB_NODE_LOCAL:
            // A removed sink may have retained its pin bit while this LOCAL
            // was still used as transit. Release both bits with its last owner.
            tile->cb.local.local &= clear;
            tile->pin_state.leased_nodes &= clear;
            break;
        case CB_NODE_JOINT:
            tile->cb.joint.jump &= clear;
            break;
        case CB_NODE_SRC:
            tile->cb.src.jump &= clear;
            break;
        case CB_NODE_DST:
            tile->cb.dst.jump &= clear;
            break;
        default:
            break;
        }

        return;
    }

    if (fragment.type != Wire::WIRE_CROSSBAR)
    {
        return;
    }

    Tile *tile = Device::current().getTile(fragment.from.x, fragment.from.y);
    if (!tile)
    {
        return;
    }

    if (fragment.jump >= 0)
    {
        if (unowned(*tile, CB_NODE_SRC, fragment.jump))
        {
            tile->cb.src.jump &= ~(NodeMask{0, 1} << fragment.jump);
        }

    }

    if (fragment.joint >= 0)
    {
        if (unowned(*tile, CB_NODE_JOINT, fragment.joint))
        {
            tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint);
        }

    }

    if (fragment.joint2 >= 0)
    {
        if (unowned(*tile, CB_NODE_JOINT, fragment.joint2))
        {
            tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint2);
        }

    }

    if (fragment.pos == 0 && fragment.local >= 0)
    {
        if (unowned(*tile, CB_NODE_LOCAL, fragment.local))
        {
            tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
            tile->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
        }

    }

    if (fragment.pos != 0 && fragment.local >= 0 && fragment.owns_dst &&
        unowned(*tile, CB_NODE_DST, fragment.local))
    {
        tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.local);
    }

}

} // namespace

// Standalone fragment release gets a fresh index of the current live routes.
void fpga::releaseRouteFragmentLease(const std::vector<Wire> &route,
                                     size_t fragment_index,
                                     OwnershipQueryWork *work)
{
    RouteOwnerQueries owners;
    releaseFragmentWithOwners(route, fragment_index, owners);
    addOwnershipWork(work, owners.work());
}

void fpga::releaseRouteLeases(const std::vector<Wire> &route)
{
    clearRouteLeases(route, true);
}

size_t fpga::attachNetRoute(rtl::Net &net, rtl::Inst &owner, size_t route_index,
                            rtl::Inst *from, rtl::Inst *to,
                            const std::string &from_port,
                            const std::string &to_port,
                            const std::string &route_name)
{
    RouteHistoryScope history(&net, __func__);
    if (!net.src_port.peer)
    {
        if (Referable<rtl::Port> *port = findInstPort(from, from_port))
        {
            net.src_port.set(port);
        }

    }

    if (!net.dst_port.peer)
    {
        if (Referable<rtl::Port> *port = findInstPort(to, to_port))
        {
            net.dst_port.set(port);
        }

    }

    // Resolve the complete endpoint identity through the net's stable index.
    rtl::NetRouteLookup lookup =
        net.findRouteBinding(from, to, from_port, to_port, route_name);
    if (lookup.index != std::numeric_limits<size_t>::max())
    {
        rtl::NetRouteBinding &binding = net.routes[lookup.index];
        // A repeated attachment to the same storage requires no update.
        if (binding.owner == &owner && binding.route_index == route_index)
        {
            return lookup.index;
        }

        // Preserve a completed physical owner; otherwise transfer the binding.
        std::vector<Wire> *route = bindingRoute(binding);
        if (route && isRouteComplete(*route))
        {
            return lookup.index;
        }

        std::vector<Wire> removed;
        if (route && !route->empty())
        {
            // Moving can replace a retained input prefix with a different
            // takeoff. Transfer shared ownership before losing the old
            // binding/storage.
            promoteSurvivingSourcePrefix(net, lookup.index);
            if (route_index < owner.wires.size())
            {
                auto &replacement = owner.wires[route_index];
                size_t common = 0;
                for (size_t &i = common;
                     i < route->size() && i < replacement.size() &&
                     replacement[i].shared &&
                     samePhysicalFragment((*route)[i], replacement[i]);
                     ++i)
                {
                    Wire &old = (*route)[i];
                    if (!old.shared)
                    {
                        replacement[i].shared = false;
                        replacement[i].owns_dst = old.owns_dst;
                    }

                    replacement[i].owns_landing |= old.owns_landing;
                }

                // This replacement is not a published sibling yet. Its first
                // fork borrows the DST formerly owned by the removed next
                // TRANSIT hop.
                if (common > 0 && common < route->size())
                {
                    Wire &tip = replacement[common - 1];
                    Wire &next = (*route)[common];
                    if (tip.type == Wire::WIRE_CROSSBAR && tip.jump >= 0 &&
                        tip.dst >= 0 && next.type == Wire::WIRE_CROSSBAR &&
                        !next.shared && next.pos != 0 && next.owns_dst &&
                        next.local == tip.dst && sameCoord(next.from, tip.to))
                    {
                        tip.owns_landing = true;
                        next.owns_dst = false;
                    }

                }

            }

            removed.swap(*route);
        }

        binding.owner = &owner;
        binding.route_index = route_index;
        ++route_mutation_revision;
        if (!removed.empty())
        {
            // Publish the replacement first: it may reuse the old source LOCAL
            // or prefix. Owner-aware release must see it as well as surviving
            // siblings.
            if (auto *replacement = bindingRoute(binding))
            {
                registerNetRouteTiles(net, *replacement, lookup.index);
            }

            clearRouteLeases(removed, true);
            rebuildNetRouteTiles(net, {removed});
        }

        return lookup.index;
    }

    // No exact endpoint identity exists; create one independent physical
    // branch.
    ++route_mutation_revision;
    net.appendRouteBinding(rtl::NetRouteBinding{
        &owner, route_index, from, to, from_port, to_port, route_name});
    return net.routes.size() - 1;
}

size_t fpga::retargetNetRouteBindings(rtl::Net &old_net, rtl::Net &new_net,
                                      rtl::Inst *old_from, rtl::Inst *old_to,
                                      const std::string &old_from_port,
                                      const std::string &old_to_port,
                                      rtl::Inst *new_from, rtl::Inst *new_to,
                                      const std::string &new_from_port,
                                      const std::string &new_to_port,
                                      const std::string &route_name)
{
    size_t changed = 0;
    std::vector<std::vector<Wire>> affected_routes;
    for (size_t index = 0; index < old_net.routes.size();)
    {
        rtl::NetRouteBinding &binding = old_net.routes[index];
        if (binding.from != old_from || binding.to != old_to ||
            binding.from_port != old_from_port ||
            binding.to_port != old_to_port || binding.route_name != route_name)
        {
            ++index;
            continue;
        }

        if (std::vector<Wire> *route = bindingRoute(binding);
            route && !route->empty())
        {
            affected_routes.push_back(*route);
        }

        rtl::NetRouteBinding replacement = binding;
        replacement.from = new_from;
        replacement.to = new_to;
        replacement.from_port = new_from_port;
        replacement.to_port = new_to_port;
        if (&old_net == &new_net)
        {
            binding = std::move(replacement);
            ++index;
        }

        else
        {
            old_net.eraseRouteBinding(index);
            replacement.route_id = 0;
            new_net.appendRouteBinding(std::move(replacement));
            // Existing receiver bindings and their stable IDs are unchanged.
            // Only the transferred branch needs registering; rebuilding the
            // receiver here revisits its entire growing fanout tree for every
            // suffix.
            auto &transferred = new_net.routes.back();
            if (auto *route = bindingRoute(transferred);
                route && !route->empty())
            {
                registerNetRouteTiles(new_net, *route,
                                      new_net.routes.size() - 1);
            }

        }

        ++changed;
    }

    // Retargeting changes the Net found through each routed tile, including an
    // existing partial prefix.
    if (changed != 0)
    {
        old_net.invalidateRouteLookup();
        new_net.invalidateRouteLookup();
        rebuildNetRouteTiles(old_net, affected_routes);
    }

    return changed;
}

// Keep every branch of one physical source tree on the same endpoint identity.
size_t fpga::retargetNetRouteSourceBindings(rtl::Net &net, rtl::Inst *old_from,
                                            const std::string &old_from_port,
                                            rtl::Inst *new_from,
                                            const std::string &new_from_port)
{
    size_t changed = 0;
    for (rtl::NetRouteBinding &binding : net.routes)
    {
        if (binding.from != old_from || binding.from_port != old_from_port)
        {
            continue;
        }

        binding.from = new_from;
        binding.from_port = new_from_port;
        ++changed;
    }

    if (changed != 0)
    {
        ++route_mutation_revision;
        net.invalidateRouteLookup();
    }

    return changed;
}

void fpga::registerNetRouteTiles(rtl::Net &net, const std::vector<Wire> &route,
                                 size_t binding_index)
{
    registerNetRouteTilesFrom(net, route, 0, binding_index);
}

void fpga::registerNetRouteTilesFrom(rtl::Net &net,
                                     const std::vector<Wire> &route,
                                     size_t first_fragment,
                                     size_t binding_index)
{
    RouteHistoryScope history(&net, __func__);
    if (first_fragment < route.size())
        ++route_mutation_revision;
    if (binding_index == std::numeric_limits<size_t>::max())
    {
        for (size_t index = 0; index < net.routes.size(); ++index)
        {
            if (bindingRoute(net.routes[index]) == &route)
            {
                binding_index = index;
                break;
            }

        }

    }

    uint64_t route_id = net.routeId(binding_index);
    for (size_t index = first_fragment; index < route.size(); ++index)
    {
        const Wire &fragment = route[index];
        Tile *from_tile =
            Device::current().getTile(fragment.from.x, fragment.from.y);
        if (from_tile)
        {
            addTileNetRef(*from_tile, net);
            from_tile->addRoutedBinding(&net, route_id);
        }

        Tile *to_tile = Device::current().getTile(fragment.to.x, fragment.to.y);
        if (to_tile)
        {
            addTileNetRef(*to_tile, net);
            to_tile->addRoutedBinding(&net, route_id);
        }

    }

}

rtl::Net *fpga::findNetByNode(Tile &tile, CBNodeNameType node_type, int node,
                              bool transit_only)
{
    std::vector<NetRouteRef> routes =
        findNetRoutesByNode(tile, node_type, node, transit_only);
    return routes.empty() ? nullptr : routes.front().net;
}

NodeMask fpga::congestionNodeMask(const CBState &state, CBNodeNameType type)
{
    switch (type)
    {
    case CB_NODE_SRC:
        return state.src.jump;
    case CB_NODE_DST:
        return state.dst.jump;
    case CB_NODE_JOINT:
        return state.joint.jump;
    case CB_NODE_LOCAL:
        return state.local.local;
    default:
        return {};
    }

}

namespace
{
uint64_t auditCoordKey(Coord coord)
{
    return (uint64_t(uint32_t(coord.x)) << 32) | uint32_t(coord.y);
}

} // namespace

// Bindings are processed contiguously, so the last entry deduplicates repeated
// visits without a second per-tile hash table or copies of route fragments.
fpga::CongestionAuditIndex::CongestionAuditIndex(
    const std::vector<rtl::Net *> &nets)
    : design_nets(nets.begin(), nets.end())
{
    for (auto *net : design_nets)
    {
        if (!net)
            continue;
        for (size_t b = 0; b < net->routes.size(); ++b)
        {
            auto *route = bindingRoute(net->routes[b]);
            if (!route)
                continue;
            for (const auto &wire : *route)
            {
                auto index_coord = [&](Coord coord)
                {
                    auto &refs = by_tile[auditCoordKey(coord)];
                    if (refs.empty() || refs.back().net != net ||
                        refs.back().binding_index != b)
                        refs.push_back({net, b});
                };

                index_coord(wire.from);
                index_coord(wire.to);
                // Dedicated endpoints may also lease their resource-side pin.
                if (wire.type == Wire::WIRE_TILE_PIN && wire.local >= 0 &&
                    wire.resource.x >= 0 && wire.resource.y >= 0)
                    index_coord(wire.resource);
            }

        }

    }

}

// Return an immutable empty range for coordinates untouched by any live route.
const std::vector<fpga::NetRouteRef> &
fpga::CongestionAuditIndex::bindings(Coord tile) const
{
    static const std::vector<NetRouteRef> empty;
    auto found = by_tile.find(auditCoordKey(tile));
    return found == by_tile.end() ? empty : found->second;
}

static fpga::CongestionAudit
auditTileCongestionImpl(fpga::Tile &tile, std::ostream &out,
                        const std::unordered_set<rtl::Net *> &nets,
                        const fpga::CongestionAuditIndex *index)
{
    CongestionAudit audit;
    auto inst_name = [](rtl::Inst *inst)
    {
        return inst && inst->cell_ref.peer ? inst->makeName(1000000)
                                           : std::string("<untyped-or-null>");
    };

    out << "TILE coord=(" << tile.coord.x << ',' << tile.coord.y
        << ") name=" << std::quoted(tile.full_name)
        << " cb=" << std::quoted(tile.cb_type ? tile.cb_type->name : "")
        << " authoritative=" << tile.routed_bindings_authoritative << '\n';
    auto mask = [&](const char *name, NodeMask bits)
    {
        out << "MASK " << name << "=[";
        bits.for_each_set_bit(
            [&](int node)
            {
                out << node << ',';
                return false;
            });
        out << "]\n";
    };

    mask("SRC", tile.cb.src.jump);
    mask("DST", tile.cb.dst.jump);
    mask("JOINT", tile.cb.joint.jump);
    mask("LOCAL", tile.cb.local.local);
    mask("PIN", tile.pin_state.leased_nodes);
    mask("DEADEND", tile.cb.src_deadend.jump);
    mask("INCOMING", tile.incoming_dst_nodes);
    for (const auto &[node, conn] : tile.input_local_reservations)
    {
        out << "PACKED_LOCAL node=" << node << " driver="
            << std::quoted(inst_name(conn ? conn->inst_ref.peer : nullptr))
            << " port="
            << std::quoted(conn && conn->port_ref.peer
                               ? conn->port_ref->makeName()
                               : "")
            << '\n';
    }

    for (const auto &[conn, bits] : tile.input_joint_reservations)
    {
        out << "PACKED_JOINT driver="
            << std::quoted(inst_name(conn ? conn->inst_ref.peer : nullptr))
            << '\n';
        mask("reserved_joints", bits);
    }

    struct Claim
    {
        rtl::Net *net;
        size_t binding;
        size_t fragment;
        bool owner;
        bool requires_lease;
    };

    std::map<std::pair<int, int>, std::vector<Claim>> claims;
    NodeMask required_tip_landings;
    NodeMask required_incoming_dsts;
    // The design scope is the authority for object existence. Do not
    // dereference an indexed pointer that has disappeared from that scope.
    for (const auto &ref : tile.routedNets)
    {
        if (ref.peer && !nets.contains(ref.peer))
        {
            ++audit.stale_registrations;
            out << "STALE_NET_REF net_exists=0\n";
        }

    }

    auto inspect_binding = [&](rtl::Net *net, size_t b)
    {
        auto &binding = net->routes[b];
        auto *route = bindingRoute(binding);
        if (!route)
            return;
        bool touches = false;
        for (size_t f = 0; f < route->size(); ++f)
        {
            const Wire &wire = (*route)[f];
            // Resource pin leases use the endpoint's route-local number. They
            // do not require CB registrations or a second lease in generic
            // routes.
            if (wire.type == Wire::WIRE_TILE_PIN && wire.local >= 0 &&
                wire.local < CB_MAX_NODES &&
                sameCoord(wire.resource, tile.coord) &&
                !sameCoord(wire.from, tile.coord))
                claims[{CB_NODE_LOCAL, wire.local}].push_back(
                    {net, b, f, !wire.shared, false});
            if (!sameCoord(wire.from, tile.coord) &&
                !sameCoord(wire.to, tile.coord))
                continue;
            touches = true;
            std::set<std::pair<int, int>> nodes{
                {CB_NODE_SRC, wire.jump},
                {CB_NODE_DST, wire.local},
                {CB_NODE_DST, wire.dst},
                {CB_NODE_LOCAL, wire.local},
                {CB_NODE_JOINT, wire.joint},
                {CB_NODE_JOINT, wire.joint2},
                {wire.from_node_type, wire.from_node},
                {wire.to_node_type, wire.to_node}};
            for (auto [kind, node] : nodes)
            {
                if (node < 0 || node >= CB_MAX_NODES)
                    continue;
                auto type = static_cast<CBNodeNameType>(kind);
                // A retained unfinished hop physically needs its landing even
                // if buggy cleanup lost owns_landing. Do not let the same
                // corrupted flag hide the missing lease from this independent
                // audit.
                bool tip_landing = f + 1 == route->size() &&
                                   wire.type == Wire::WIRE_CROSSBAR &&
                                   wire.jump >= 0 &&
                                   !sameCoord(wire.from, wire.to) &&
                                   sameCoord(wire.to, tile.coord) &&
                                   type == CB_NODE_DST && wire.dst == node;
                const bool incoming_dst =
                    wire.type == Wire::WIRE_CROSSBAR && wire.pos != 0 &&
                    sameCoord(wire.from, tile.coord) && type == CB_NODE_DST &&
                    wire.local == node;
                if (!tip_landing && !incoming_dst &&
                    !routeUsesNodeOnTile({&wire, 1}, tile, type, node, false,
                                         false))
                    continue;
                if (tip_landing)
                    required_tip_landings.setBit(node);
                // A FORK borrows its incoming DST; owns_dst=false does not make
                // that physical dependency optional. Audit it independently of
                // the exact ownership flag whose corruption can lose the lease.
                if (incoming_dst)
                    required_incoming_dsts.setBit(node);
                // Recheck the owning fragment flags directly, not the owner
                // lookup. A shared hop can own its landing, never its
                // source-side DST.
                bool owns =
                    !wire.shared ||
                    (wire.type == Wire::WIRE_CROSSBAR && type == CB_NODE_DST &&
                     wire.owns_landing && sameCoord(wire.to, tile.coord) &&
                     wire.dst == node);
                if (tip_landing)
                    owns = wire.owns_landing;
                if (incoming_dst && !wire.owns_dst &&
                    !(wire.owns_landing && sameCoord(wire.to, tile.coord) &&
                      wire.dst == node))
                    owns = false;
                // A directed graph edge leases only its destination. Its source
                // is owned by a preceding edge or is an unleased resource root.
                if (wire.type == Wire::WIRE_ROUTE_EDGE)
                    owns = !wire.shared && sameCoord(wire.to, tile.coord) &&
                           wire.to_node_type == kind && wire.to_node == node;
                // Numeric takeoff deliberately does not lease its LOCAL source:
                // several branches may leave the same physical output.
                // Dedicated routers can lease it, but its absence alone is not
                // an error.
                bool source_reference =
                    type == CB_NODE_LOCAL &&
                    ((wire.type == Wire::WIRE_CROSSBAR && wire.pos == 0) ||
                     (wire.type == Wire::WIRE_TILE_PIN &&
                      wire.pin_dir == TILE_PIN_OUTPUT));
                claims[{kind, node}].push_back(
                    {net, b, f, owns, !source_reference});
            }

        }

        if (touches)
        {
            bool registered =
                tile.routed_bindings_authoritative
                    ? std::any_of(tile.routed_bindings.begin(),
                                  tile.routed_bindings.end(),
                                  [&](const auto &ref)
                                  {
                                      return ref.net == net &&
                                             net->findRouteBindingById(
                                                 ref.route_id) == b;
                                  })
                    : std::any_of(tile.routedNets.begin(),
                                  tile.routedNets.end(), [&](const auto &ref)
                                  { return ref.peer == net; });
            if (!registered)
            {
                ++audit.missing_registrations;
                out << "MISSING_REGISTRATION net="
                    << std::quoted(net->makeName(1000000)) << " binding=" << b
                    << " route=" << std::quoted(binding.route_name) << '\n';
            }

        }

    };

    if (index)
    {
        for (const auto &ref : index->bindings(tile.coord))
            inspect_binding(ref.net, ref.binding_index);
    }

    else
    {
        for (rtl::Net *net : nets)
        {
            if (!net)
                continue;
            for (size_t b = 0; b < net->routes.size(); ++b)
                inspect_binding(net, b);
        }

    }

    for (const auto &ref : tile.routed_bindings)
    {
        if (!ref.net || !nets.contains(ref.net))
        {
            ++audit.stale_registrations;
            out << "INDEX route_id=" << ref.route_id
                << " net_exists=0 valid=0\n";
            continue;
        }

        size_t b =
            ref.net ? ref.net->findRouteBindingById(ref.route_id) : SIZE_MAX;
        auto *route = ref.net && b < ref.net->routes.size()
                          ? bindingRoute(ref.net->routes[b])
                          : nullptr;
        bool valid =
            route && std::any_of(route->begin(), route->end(),
                                 [&](const Wire &w)
                                 {
                                     return sameCoord(w.from, tile.coord) ||
                                            sameCoord(w.to, tile.coord);
                                 });
        out << "INDEX net="
            << std::quoted(ref.net ? ref.net->makeName(1000000) : "")
            << " route_id=" << ref.route_id << " binding=" << b
            << " net_exists=1 valid=" << valid << '\n';
        if (!valid)
            ++audit.stale_registrations;
    }

    for (auto type : {CB_NODE_SRC, CB_NODE_DST, CB_NODE_JOINT, CB_NODE_LOCAL})
    {
        NodeMask leased = congestionNodeMask(tile.cb, type);
        if (type == CB_NODE_LOCAL)
            leased |= tile.pin_state.leased_nodes;
        leased.for_each_set_bit(
            [&](int n)
            {
                claims[{type, n}];
                return false;
            });
    }

    for (const auto &[key, users] : claims)
    {
        auto [kind, node] = key;
        auto type = static_cast<CBNodeNameType>(kind);
        bool leased = congestionNodeMask(tile.cb, type).testBit(node) ||
                      (type == CB_NODE_LOCAL &&
                       tile.pin_state.leased_nodes.testBit(node));
        size_t owners = std::count_if(users.begin(), users.end(),
                                      [](const Claim &c) { return c.owner; });
        bool requires_lease =
            (type == CB_NODE_DST && (required_tip_landings.testBit(node) ||
                                     required_incoming_dsts.testBit(node))) ||
            std::any_of(users.begin(), users.end(), [](const Claim &c)
                        { return c.owner && c.requires_lease; });
        const char *label = type == CB_NODE_SRC     ? "SRC"
                            : type == CB_NODE_DST   ? "DST"
                            : type == CB_NODE_JOINT ? "JOINT"
                                                    : "LOCAL";
        const std::string *name =
            tile.cb_type ? tile.cb_type->nodeName(type, node) : nullptr;
        // Multiple same-source replicas/RTL aliases are valid. Different driver
        // ports are not, even when every lease and registration has a live
        // owner.
        const rtl::NetRouteBinding *source = nullptr;
        bool conflicting = false;
        for (const auto &claim : users)
        {
            const auto &binding = claim.net->routes[claim.binding];
            if (!binding.from)
                continue; // no source identity available to compare
            if (!source)
                source = &binding;
            else if (!rtl::sameRouteSource(source->from, source->from_port,
                                           binding.from, binding.from_port))
                conflicting = true;
        }

        audit.conflicting_driver_nodes += conflicting;
        const char *status = leased ? (conflicting ? "CONFLICTING_DRIVERS"
                                       : owners    ? "VERIFIED"
                                                   : "UNOWNED_LEASE")
                                    : (requires_lease ? "MISSING_LEASE"
                                       : owners       ? "SOURCE_REFERENCE"
                                                      : "NONOWNING_REFERENCE");
        audit.leased_nodes += leased;
        audit.orphan_leases += leased && !owners;
        audit.missing_leases += !leased && requires_lease;
        out << "NODE type=" << label << " id=" << node
            << " name=" << std::quoted(name ? *name : "")
            << " leased=" << leased << " claims=" << users.size()
            << " owners=" << owners << " status=" << status << '\n';
        for (const Claim &claim : users)
        {
            const auto &binding = claim.net->routes[claim.binding];
            const Wire &w =
                binding.owner->wires[binding.route_index][claim.fragment];
            out << "PROOF net=" << std::quoted(claim.net->makeName(1000000))
                << " route=" << std::quoted(binding.route_name)
                << " binding=" << claim.binding
                << " storage=" << binding.route_index
                << " fragment=" << claim.fragment << " owns=" << claim.owner
                << " shared=" << w.shared << " owns_dst=" << w.owns_dst
                << " owns_landing=" << w.owns_landing << " pos=" << w.pos
                << " type=" << w.type << " pin_dir=" << w.pin_dir
                << " from_node=" << w.from_node_type << ':' << w.from_node
                << " to_node=" << w.to_node_type << ':' << w.to_node
                << " from=(" << w.from.x << ',' << w.from.y << ") to=("
                << w.to.x << ',' << w.to.y << ")"
                << " local=" << w.local << " src=" << w.jump << " dst=" << w.dst
                << " joint=" << w.joint << " joint2=" << w.joint2
                << " driver=" << std::quoted(inst_name(binding.from))
                << " sink=" << std::quoted(inst_name(binding.to))
                << " complete="
                << isRouteComplete(binding.owner->wires[binding.route_index])
                << '\n';
        }

    }

    out << "AUDIT leased=" << audit.leased_nodes
        << " unowned=" << audit.orphan_leases
        << " missing_lease=" << audit.missing_leases
        << " missing_registration=" << audit.missing_registrations
        << " stale_registration=" << audit.stale_registrations
        << " conflicting_drivers=" << audit.conflicting_driver_nodes << '\n';
    return audit;
}

// The exhaustive entry remains the reference implementation for audit checks.
fpga::CongestionAudit
fpga::auditTileCongestion(Tile &tile, std::ostream &out,
                          const std::vector<rtl::Net *> &design_nets)
{
    std::unordered_set<rtl::Net *> nets(design_nets.begin(), design_nets.end());
    return auditTileCongestionImpl(tile, out, nets, nullptr);
}

// The indexed entry shares all lease and registration verification logic.
fpga::CongestionAudit
fpga::auditTileCongestion(Tile &tile, std::ostream &out,
                          const CongestionAuditIndex &index)
{
    return auditTileCongestionImpl(tile, out, index.nets(), &index);
}

void fpga::dumpNetRouteHistory(rtl::Net &net, std::ostream &out)
{
    auto name = [](rtl::Inst *inst)
    {
        return inst && inst->cell_ref.peer ? inst->makeName(1000000)
                                           : std::string("<untyped-or-null>");
    };

    auto leased = [](Coord coord, CBNodeNameType type, int node)
    {
        Tile *tile = Device::current().getTile(coord.x, coord.y);
        return tile && node >= 0 && node < CB_MAX_NODES &&
               (congestionNodeMask(tile->cb, type).testBit(node) ||
                (type == CB_NODE_LOCAL && tile->isPinNodeLeased(node)));
    };

    out << "TREE net=" << std::quoted(net.makeName(1000000))
        << " bindings=" << net.routes.size() << '\n';
    for (size_t b = 0; b < net.routes.size(); ++b)
    {
        auto &binding = net.routes[b];
        auto *route = bindingRoute(binding);
        out << "BINDING index=" << b << " id=" << net.routeId(b)
            << " route=" << std::quoted(binding.route_name)
            << " storage=" << binding.route_index
            << " driver=" << std::quoted(name(binding.from))
            << " from_port=" << std::quoted(binding.from_port)
            << " sink=" << std::quoted(name(binding.to))
            << " to_port=" << std::quoted(binding.to_port)
            << " fragments=" << (route ? route->size() : 0)
            << " complete=" << (route && isRouteComplete(*route)) << " state="
            << (!route || route->empty()  ? "unrouted"
                : isRouteComplete(*route) ? "routed"
                                          : "partially_routed")
            << '\n';
        if (!route)
            continue;
        for (size_t f = 0; f < route->size(); ++f)
        {
            const Wire &w = (*route)[f];
            out << "FRAGMENT index=" << f << " type=" << w.type << " from=("
                << w.from.x << ',' << w.from.y << ") to=(" << w.to.x << ','
                << w.to.y << ')' << " local=" << w.local << " src=" << w.jump
                << " dst=" << w.dst << " joint=" << w.joint
                << " joint2=" << w.joint2 << " pos=" << w.pos
                << " shared=" << w.shared << " owns_dst=" << w.owns_dst
                << " owns_landing=" << w.owns_landing << " live_local="
                << leased(w.from,
                          w.pos == 0 || w.type == Wire::WIRE_TILE_PIN
                              ? CB_NODE_LOCAL
                              : CB_NODE_DST,
                          w.local)
                << " live_src=" << leased(w.from, CB_NODE_SRC, w.jump)
                << " live_landing=" << leased(w.to, CB_NODE_DST, w.dst)
                << " live_joint=" << leased(w.from, CB_NODE_JOINT, w.joint)
                << " live_joint2=" << leased(w.from, CB_NODE_JOINT, w.joint2)
                << " from_node=" << w.from_node_type << ':' << w.from_node
                << " to_node=" << w.to_node_type << ':' << w.to_node
                << " pin_dir=" << w.pin_dir << '\n';
        }

    }

}

namespace
{
const std::string &routeHistoryNet()
{
    static const std::string selected = []
    {
        const char *value = std::getenv("SCALEPNR_ROUTE_HISTORY_NET");
        return value ? std::string(value) : std::string{};
    }();
    return selected;
}

thread_local std::string_view route_history_actor;
thread_local size_t route_history_sequence = 0;

std::ostream &routeHistoryOutput()
{
    // A dash shares stdout with the per-task trace without opening or
    // truncating it.
    const char *path = std::getenv("SCALEPNR_ROUTE_HISTORY_LOG");
    if (path && std::string_view(path) == "-")
        return std::cout;
    static std::ofstream out(
        []
        {
            const char *value = std::getenv("SCALEPNR_ROUTE_HISTORY_LOG");
            return value && *value ? value : "routing_node_history.log";
        }());
    if (!out)
        throw std::runtime_error("cannot write route ownership history");
    return out;
}

struct RouteHistoryWatch
{
    int x = -1, y = -1, node = -1;
    CBNodeNameType type = CB_NODE_DST;
};

const RouteHistoryWatch &routeHistoryWatch()
{
    static const RouteHistoryWatch watch = []
    {
        RouteHistoryWatch w;
        const char *value = std::getenv("SCALEPNR_ROUTE_HISTORY_NODE");
        char kind[8] = {};
        if (!value || std::sscanf(value, "%d,%d,%7[^,],%d", &w.x, &w.y, kind,
                                  &w.node) != 4)
            return RouteHistoryWatch{};
        std::string_view label(kind);
        if (label == "SRC")
            w.type = CB_NODE_SRC;
        else if (label == "DST")
            w.type = CB_NODE_DST;
        else if (label == "JOINT")
            w.type = CB_NODE_JOINT;
        else if (label == "LOCAL")
            w.type = CB_NODE_LOCAL;
        else
            return RouteHistoryWatch{};
        if (w.x < 0 || w.y < 0 || w.node < 0 || w.node >= CB_MAX_NODES)
            return RouteHistoryWatch{};
        return w;
    }();
    return watch;
}

std::unordered_set<rtl::Net *> route_history_discovered;

bool watchedLease(const Tile &tile, const RouteHistoryWatch &watch)
{
    return congestionNodeMask(tile.cb, watch.type).testBit(watch.node) ||
           (watch.type == CB_NODE_LOCAL && tile.isPinNodeLeased(watch.node));
}

void traceHistoryOwnerLookup(Tile &tile, CBNodeNameType type, int node,
                             const std::vector<NetRouteRef> &result)
{
    const auto &watch = routeHistoryWatch();
    if (node != watch.node || type != watch.type || tile.coord.x != watch.x ||
        tile.coord.y != watch.y)
        return;
    auto &out = routeHistoryOutput();
    out << "OWNER_LOOKUP tile=(" << watch.x << ',' << watch.y
        << ") kind=" << static_cast<int>(type) << " node=" << node
        << " leased=" << congestionNodeMask(tile.cb, type).testBit(node)
        << " returned=" << result.size()
        << " actor=" << std::quoted(std::string(route_history_actor)) << '\n';
    for (const auto &ref : result)
    {
        auto &binding = ref.net->routes[ref.binding_index];
        auto *route = bindingRoute(binding);
        out << "LOOKUP_OWNER net=" << std::quoted(ref.net->makeName(1000000))
            << " binding=" << ref.binding_index
            << " route=" << std::quoted(binding.route_name) << '\n';
        if (!route)
            continue;
        for (size_t f = 0; f < route->size(); ++f)
        {
            const Wire &w = (*route)[f];
            if (!routeUsesNodeOnTile({&w, 1}, tile, type, node, false, false))
                continue;
            bool owns =
                !w.shared || (w.type == Wire::WIRE_CROSSBAR &&
                              type == CB_NODE_DST && w.owns_landing &&
                              sameCoord(w.to, tile.coord) && w.dst == node);
            if (w.type == Wire::WIRE_ROUTE_EDGE)
                owns = !w.shared && sameCoord(w.to, tile.coord) &&
                       w.to_node_type == type && w.to_node == node;
            out << "LOOKUP_PROOF fragment=" << f << " verified_owner=" << owns
                << " from=(" << w.from.x << ',' << w.from.y << ") to=("
                << w.to.x << ',' << w.to.y << ')' << " local=" << w.local
                << " dst=" << w.dst << " shared=" << w.shared
                << " owns_dst=" << w.owns_dst
                << " owns_landing=" << w.owns_landing << '\n';
        }

    }

    out.flush();
}

} // namespace

fpga::RouteHistoryScope::RouteHistoryScope(rtl::Net *net, const char *operation,
                                           std::string_view actor)
    : operation(operation)
{
    const auto &watch = routeHistoryWatch();
    if (routeHistoryNet().empty() && watch.node < 0)
        return;
    active = true;
    previous_actor = route_history_actor;
    if (!actor.empty())
        route_history_actor = actor;
    if (watch.node >= 0)
    {
        if (Tile *tile = Device::current().getTile(watch.x, watch.y))
        {
            watch_valid = true;
            watched_net = net;
            watched_lease = watchedLease(*tile, watch);
        }

    }

    if (!net || (!route_history_discovered.contains(net) &&
                 (routeHistoryNet().empty() ||
                  net->makeName(1000000) != routeHistoryNet())))
        return;
    selected = net;
    id = ++route_history_sequence;
    auto &out = routeHistoryOutput();
    out << "HISTORY_BEGIN id=" << id << " operation=" << operation
        << " actor=" << std::quoted(std::string(route_history_actor)) << '\n';
    dumpNetRouteHistory(*selected, out);
    out.flush();
}

uint64_t fpga::routeMutationRevision()
{
    return route_mutation_revision;
}

const RepairStatistics &fpga::repairStatistics()
{
    return repair_statistics;
}

void fpga::resetRepairStatistics()
{
    repair_statistics = {};
}

void fpga::printRepairStatistics(std::ostream &out, std::string_view context)
{
    if (!std::getenv("SCALEPNR_PROFILE_REPAIR"))
        return;
    out << "REPAIR_TOTAL context=" << std::quoted(std::string(context));
    for (size_t i = 0; i < repair_statistics.counters.size(); ++i)
        out << ' ' << repairCounterNames[i] << '='
            << repair_statistics.counters[i];
    for (size_t i = 0; i < repair_statistics.seconds.size(); ++i)
        out << ' ' << repairPhaseNames[i]
            << "_s=" << repair_statistics.seconds[i];
    out << '\n';
}

fpga::RouteHistoryScope::~RouteHistoryScope()
{
    if (watch_valid)
    {
        const auto &watch = routeHistoryWatch();
        Tile *tile = Device::current().getTile(watch.x, watch.y);
        const bool after = tile && watchedLease(*tile, watch);
        if (after != watched_lease)
        {
            auto &out = routeHistoryOutput();
            out << "WATCH_TRANSITION operation=" << operation
                << " actor=" << std::quoted(std::string(route_history_actor))
                << " net="
                << std::quoted(watched_net ? watched_net->makeName(1000000)
                                           : "<none>")
                << " tile=(" << watch.x << ',' << watch.y
                << ") kind=" << static_cast<int>(watch.type)
                << " node=" << watch.node << " before=" << watched_lease
                << " after=" << after << '\n';
            // Discover the actor even when no owner survives. Thereafter record
            // this net's full before/after trees, including mutations that
            // leave bits set.
            if (watched_net)
            {
                route_history_discovered.insert(watched_net);
                if (!selected)
                {
                    selected = watched_net;
                    id = ++route_history_sequence;
                    out << "HISTORY_DISCOVERED id=" << id
                        << " operation=" << operation
                        << " before_tree_unavailable=1\n";
                }

            }

            out.flush();
        }

    }

    if (selected)
    {
        auto &out = routeHistoryOutput();
        out << "HISTORY_AFTER id=" << id << " operation=" << operation
            << " actor=" << std::quoted(std::string(route_history_actor))
            << '\n';
        dumpNetRouteHistory(*selected, out);
        out << "HISTORY_END id=" << id << '\n';
        out.flush();
    }

    if (active)
        route_history_actor = previous_actor;
}

std::vector<NetRouteRef> fpga::findNetRoutesByNode(Tile &tile,
                                                   CBNodeNameType node_type,
                                                   int node, bool transit_only)
{
    std::vector<NetRouteRef> result;
    if (tile.routed_bindings_authoritative)
    {
        for (const Tile::RoutedBinding &ref : tile.routed_bindings)
        {
            if (!ref.net)
            {
                continue;
            }

            size_t binding_index = ref.net->findRouteBindingById(ref.route_id);
            if (binding_index == std::numeric_limits<size_t>::max() ||
                binding_index >= ref.net->routes.size())
            {
                continue;
            }

            rtl::NetRouteBinding &binding = ref.net->routes[binding_index];
            std::vector<Wire> *route = bindingRoute(binding);
            if (route && !route->empty() &&
                routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                                    false))
            {
                result.push_back(NetRouteRef{ref.net, binding_index});
            }

        }

        return result;
    }

    for (auto &ref : tile.routedNets)
    {
        rtl::Net *net = ref.peer;
        if (!net)
        {
            continue;
        }

        for (size_t binding_index = 0; binding_index < net->routes.size();
             ++binding_index)
        {
            rtl::NetRouteBinding &binding = net->routes[binding_index];
            std::vector<Wire> *route = bindingRoute(binding);
            if (!route || route->empty())
            {
                continue;
            }

            if (routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                                    false))
            {
                result.push_back(NetRouteRef{net, binding_index});
            }

        }

    }

    return result;
}

namespace
{
template <class Accept>
bool visitNodeOwners(Tile &tile, CBNodeNameType node_type, int node,
                     bool transit_only, Accept &&accept,
                     OwnershipQueryWork *work = nullptr)
{
    if (work)
        ++work->queries;
    bool found = false;
    if (tile.routed_bindings_authoritative)
    {
        for (const Tile::RoutedBinding &ref : tile.routed_bindings)
        {
            if (!ref.net)
            {
                continue;
            }

            if (work)
                ++work->binding_resolutions;
            size_t binding_index = ref.net->findRouteBindingById(ref.route_id);
            if (binding_index == std::numeric_limits<size_t>::max() ||
                binding_index >= ref.net->routes.size())
            {
                continue;
            }

            rtl::NetRouteBinding &binding = ref.net->routes[binding_index];
            std::vector<Wire> *route = bindingRoute(binding);
            if (route && !route->empty() &&
                routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                                    true, work))
            {
                found = true;
                if (!accept(NetRouteRef{ref.net, binding_index}))
                    return false;
            }

        }

        return found;
    }

    for (auto &ref : tile.routedNets)
    {
        rtl::Net *net = ref.peer;
        if (!net)
        {
            continue;
        }

        for (size_t binding_index = 0; binding_index < net->routes.size();
             ++binding_index)
        {
            if (work)
                ++work->binding_resolutions;
            rtl::NetRouteBinding &binding = net->routes[binding_index];
            std::vector<Wire> *route = bindingRoute(binding);
            if (!route || route->empty())
            {
                continue;
            }

            if (routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                                    true, work))
            {
                found = true;
                if (!accept(NetRouteRef{net, binding_index}))
                    return false;
            }

        }

    }

    return found;
}

} // namespace

bool fpga::RouteOwnerQueries::allMatch(
    Tile &tile, CBNodeNameType type, int node,
    const std::function<bool(const NetRouteRef &)> &accept, bool transit_only)
{
    ++counts.queries;
    if (node < 0 || node >= CB_MAX_NODES || type > CB_NODE_DST)
        return false;
    // Keep opt-in owner tracing on its exhaustive, uncached diagnostic path.
    const auto &watch = routeHistoryWatch();
    if (node == watch.node && type == watch.type && tile.coord.x == watch.x &&
        tile.coord.y == watch.y)
        return allNetOwnersMatch(tile, type, node, accept, transit_only);
    // Preemption/publication invalidates every stored binding index and owner.
    if (revision != routeMutationRevision())
    {
        tiles.clear();
        revision = routeMutationRevision();
    }

    auto found = tiles.find(&tile);
    if (found == tiles.end())
    {
        // Retain each visited Tile for this search. Clearing the entire index
        // at 64 Tiles made broad searches repeatedly rebuild unchanged route
        // owners. The query object dies with the search; mutations invalidate
        // it above.
        auto &indexed = tiles[&tile];
        ++counts.tile_builds;
        auto add_binding = [&](rtl::Net &net, size_t index)
        {
            if (index >= net.routes.size())
                return;
            const auto *route = bindingRoute(net.routes[index]);
            if (!route)
                return;
            std::array<NodeMask, CB_NODE_DST + 1> owned{}, transit{};
            for (const Wire &w : *route)
            {
                ++counts.fragment_visits;
                const bool from = sameCoord(w.from, tile.coord);
                const bool to = sameCoord(w.to, tile.coord);
                if (!from && !to)
                    continue;
                // Reuse the lease predicate on a single fragment instead of
                // duplicating shared landing, terminal, fork and numeric-edge
                // ownership rules.
                auto remember = [&](CBNodeNameType kind, int id)
                {
                    if (id < 0 || id >= CB_MAX_NODES || kind > CB_NODE_DST)
                        return;
                    const std::span<const Wire> one(&w, 1);
                    if (routeUsesNodeOnTile(one, tile, kind, id, false, true,
                                            &counts))
                    {
                        owned[kind].setBit(id);
                        if (routeUsesNodeOnTile(one, tile, kind, id, true, true,
                                                &counts))
                            transit[kind].setBit(id);
                    }

                };

                if (w.type == Wire::WIRE_ROUTE_EDGE)
                {
                    if (to && w.to_node_type >= 0 &&
                        w.to_node_type <= CB_NODE_DST)
                        remember(static_cast<CBNodeNameType>(w.to_node_type),
                                 w.to_node);
                }

                else if (w.type == Wire::WIRE_CROSSBAR)
                {
                    if (from)
                    {
                        remember(CB_NODE_SRC, w.jump);
                        remember(CB_NODE_DST, w.local);
                        remember(CB_NODE_JOINT, w.joint);
                        remember(CB_NODE_JOINT, w.joint2);
                        remember(CB_NODE_LOCAL, w.local);
                    }

                    if (to && w.owns_landing)
                        remember(CB_NODE_DST, w.dst);
                }

                else if (w.type == Wire::WIRE_TILE_PIN && from)
                {
                    remember(CB_NODE_LOCAL, w.local);
                }

            }

            // Emit each binding only once per typed node, preserving
            // registration order even when its vector repeats fragments or
            // revisits the tile.
            for (unsigned kind = 0; kind <= CB_NODE_DST; ++kind)
                owned[kind].for_each_set_bit(
                    [&](int id)
                    {
                        indexed[kind * CB_MAX_NODES + id].push_back(
                            {{&net, index}, transit[kind].testBit(id)});
                        return false;
                    });
        };

        if (tile.routed_bindings_authoritative)
        {
            for (const auto &ref : tile.routed_bindings)
                if (ref.net)
                {
                    ++counts.binding_resolutions;
                    add_binding(*ref.net,
                                ref.net->findRouteBindingById(ref.route_id));
                }

        }

        else
        {
            for (const auto &ref : tile.routedNets)
                if (ref.peer)
                    for (size_t i = 0; i < ref.peer->routes.size(); ++i)
                    {
                        ++counts.binding_resolutions;
                        add_binding(*ref.peer, i);
                    }

        }

        found = tiles.find(&tile);
    }

    const auto owners =
        found->second.find(unsigned(type) * CB_MAX_NODES + node);
    if (owners == found->second.end())
        return false;
    bool any = false;
    for (const auto &owner : owners->second)
    {
        if (transit_only && !owner.transit)
            continue;
        any = true;
        if (!accept(owner.ref))
            return false;
    }

    return any;
}

std::vector<NetRouteRef> fpga::findNetOwnersByNode(Tile &tile,
                                                   CBNodeNameType node_type,
                                                   int node, bool transit_only,
                                                   OwnershipQueryWork *work)
{
    std::vector<NetRouteRef> result;
    visitNodeOwners(
        tile, node_type, node, transit_only,
        [&](const NetRouteRef &ref)
        {
            result.push_back(ref);
            return true;
        },
        work);
    traceHistoryOwnerLookup(tile, node_type, node, result);
    return result;
}

bool fpga::allNetOwnersMatch(
    Tile &tile, CBNodeNameType node_type, int node,
    const std::function<bool(const NetRouteRef &)> &accept, bool transit_only)
{
    const auto &watch = routeHistoryWatch();
    if (node == watch.node && node_type == watch.type &&
        tile.coord.x == watch.x && tile.coord.y == watch.y)
    {
        // Keep opt-in ownership proofs exhaustive even for a short-circuit
        // query.
        const auto owners =
            findNetOwnersByNode(tile, node_type, node, transit_only);
        return !owners.empty() &&
               std::all_of(owners.begin(), owners.end(), accept);
    }

    return visitNodeOwners(tile, node_type, node, transit_only, accept);
}

bool fpga::unrouteNetRoute(rtl::Net &net, size_t route_binding_index)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    rtl::NetRouteBinding &binding = net.routes[route_binding_index];
    std::vector<Wire> *route = bindingRoute(binding);
    if (!route || route->empty())
    {
        return false;
    }

    // Preserve one owner whenever another branch of this source still uses the
    // prefix.
    promoteSurvivingSourcePrefix(net, route_binding_index);
    std::vector<Wire> removed = *route;
    route->clear();
    // A physical node may still be owned by another net, including protected
    // infrastructure. Release only nodes with no remaining live route owner.
    clearRouteLeases(removed, true);

    rebuildNetRouteTiles(net, {removed});
    return true;
}

bool fpga::unrouteNetRouteFromNode(rtl::Net &net, size_t route_binding_index,
                                   Coord tile_coord, CBNodeNameType node_type,
                                   int node)
{
    return unrouteNetRouteFromNodes(
        net, route_binding_index, {RouteCutNode{tile_coord, node_type, node}});
}

bool fpga::truncateNetRoute(rtl::Net &net, size_t route_binding_index,
                            size_t keep_fragments)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (!route || keep_fragments == 0 || keep_fragments >= route->size())
    {
        return false;
    }

    // A private-looking suffix can have completed children borrowing its hops.
    // Transfer only the removed span before owner-aware release; retain this
    // route's ownership of the prefix that stays in place.
    promoteSurvivingSourcePrefix(net, route_binding_index, nullptr,
                                 keep_fragments);
    std::vector<Wire> removed(route->begin() +
                                  static_cast<std::ptrdiff_t>(keep_fragments),
                              route->end());
    if ((*route)[keep_fragments - 1].type == Wire::WIRE_CROSSBAR &&
        removed.front().type == Wire::WIRE_CROSSBAR &&
        (*route)[keep_fragments - 1].dst == removed.front().local)
    {
        (*route)[keep_fragments - 1].owns_landing = true;
        removed.front().owns_dst = false;
    }

    route->resize(keep_fragments);
    clearRouteLeases(removed, true);
    rebuildNetRouteTiles(net, {removed});
    return true;
}

// Include the incoming hop of a contested DST, not just its outgoing fragment.
static size_t routeNodeCut(std::span<const Wire> route,
                           const std::vector<fpga::RouteCutNode> &nodes,
                           OwnershipQueryWork *work = nullptr)
{
    auto fragment_uses_node = [&](const Wire &fragment, const RouteCutNode &cut)
    {
        if (cut.node < 0)
        {
            return false;
        }

        bool from_tile = sameCoord(fragment.from, cut.tile);
        bool to_tile = sameCoord(fragment.to, cut.tile);
        if (fragment.type == Wire::WIRE_ROUTE_EDGE)
        {
            return (from_tile && fragment.from_node_type == cut.type &&
                    fragment.from_node == cut.node) ||
                   (to_tile && fragment.to_node_type == cut.type &&
                    fragment.to_node == cut.node);
        }

        if (fragment.type == Wire::WIRE_TILE_PIN)
        {
            return cut.type == CB_NODE_LOCAL && from_tile &&
                   fragment.local == cut.node;
        }

        if (fragment.type != Wire::WIRE_CROSSBAR)
        {
            return false;
        }

        if (cut.type == CB_NODE_SRC)
        {
            return from_tile && fragment.jump == cut.node;
        }

        if (cut.type == CB_NODE_DST)
        {
            return (from_tile && fragment.pos != 0 &&
                    fragment.local == cut.node) ||
                   (to_tile && fragment.dst == cut.node);
        }

        if (cut.type == CB_NODE_LOCAL)
        {
            return from_tile && fragment.pos == 0 && fragment.local == cut.node;
        }

        return cut.type == CB_NODE_JOINT && from_tile &&
               (fragment.joint == cut.node || fragment.joint2 == cut.node);
    };

    size_t cut = 0;
    while (cut < route.size() && std::none_of(
                                     nodes.begin(), nodes.end(),
                                     [&](const RouteCutNode &node)
                                     {
                                         if (work)
                                             ++work->fragment_visits;
                                         return fragment_uses_node(route[cut],
                                                                   node);
                                     }))
    {
        ++cut;
    }

    return cut;
}

std::vector<fpga::NetRouteRef>
fpga::findNetRouteCutVictims(const std::vector<RouteCutNode> &cuts,
                             OwnershipQueryWork *work)
{
    if (work)
        ++work->queries;
    std::vector<NetRouteRef> result;
    auto inspect = [&](rtl::Net *net, size_t index)
    {
        if (!net || index >= net->routes.size())
            return;
        if (std::any_of(
                result.begin(), result.end(), [&](const auto &old)
                { return old.net == net && old.binding_index == index; }))
            return;
        const auto *route = bindingRoute(net->routes[index]);
        if (route && routeNodeCut(*route, cuts, work) < route->size())
            result.push_back({net, index});
    };

    for (const auto &node : cuts)
    {
        auto *tile = Device::current().getTile(node.tile.x, node.tile.y);
        if (!tile || node.node < 0)
            continue;
        if (tile->routed_bindings_authoritative)
        {
            for (const auto &ref : tile->routed_bindings)
                if (ref.net)
                {
                    if (work)
                        ++work->binding_resolutions;
                    inspect(ref.net,
                            ref.net->findRouteBindingById(ref.route_id));
                }

        }

        else
        {
            for (const auto &ref : tile->routedNets)
                if (ref.peer)
                    for (size_t i = 0; i < ref.peer->routes.size(); ++i)
                    {
                        if (work)
                            ++work->binding_resolutions;
                        inspect(ref.peer, i);
                    }

        }

    }

    return result;
}

// The smallest contiguous hole containing every conflict. A contested DST
// includes both its arriving hop and its departing hop; otherwise the retained
// suffix would keep that very DST leased. Direct endpoint conflicts remain
// protected. If cutting a transit reaches the terminal tail, keep the LOCAL
// pin, not an unusable old arrival: repair that victim's input path as well.
static size_t transitCutEnd(std::span<const Wire> route, size_t &first,
                            const std::vector<RouteCutNode> &cuts)
{
    auto graph_transit = [&](size_t i)
    {
        // Explicit intra-CB edges can precede a physical jump, including
        // DST -> LOCAL -> SRC. A terminal-only graph path is Grounding.
        for (; i < route.size() && route[i].type == Wire::WIRE_ROUTE_EDGE; ++i)
            if (!sameCoord(route[i].from, route[i].to) &&
                route[i].from_node_type == CB_NODE_SRC &&
                route[i].to_node_type == CB_NODE_DST)
                return true;
        return false;
    };

    size_t end = first;
    for (size_t i = first; i < route.size(); ++i)
    {
        if (routeNodeCut(route.subspan(i, 1), cuts) != 0)
            continue;
        const auto &w = route[i];
        if (w.type == Wire::WIRE_ROUTE_EDGE)
        {
            if (!graph_transit(i))
                return first;
        }

        else if (w.type != Wire::WIRE_CROSSBAR || w.pos == 0 || w.jump < 0 ||
                 w.dst < 0)
            return first;
        end = i + 1;
    }

    // Reusing a SRC also reuses its physical landing. Do not retain a foreign
    // signal on that landing: include the departing transit fragment as well.
    if (end > first && end < route.size())
    {
        const auto &last = route[end - 1];
        if (std::any_of(cuts.begin(), cuts.end(),
                        [&](const auto &cut)
                        {
                            return cut.type == CB_NODE_SRC &&
                                   sameCoord(cut.tile, last.from) &&
                                   (last.type == Wire::WIRE_ROUTE_EDGE
                                        ? last.from_node_type == CB_NODE_SRC &&
                                              cut.node == last.from_node
                                        : cut.node == last.jump);
                        }))
            ++end;
    }

    // Open a hole at DST boundaries, not midway through a LOCAL bridge.
    // Graph edges own only their destination; retain a CROSSBAR at the right
    // boundary so its incoming DST still has a real lease owner.
    while (first > 0 && route[first].type == Wire::WIRE_ROUTE_EDGE &&
           route[first].from_node_type != CB_NODE_DST)
        --first;
    while (end < route.size() && route[end].type == Wire::WIRE_ROUTE_EDGE)
        ++end;
    // Repeated displacement can extend an existing hole in either direction.
    if (first > 0 && route[first - 1].type == Wire::WIRE_GAP)
        --first;
    if (end < route.size() && route[end].type == Wire::WIRE_GAP)
        ++end;
    // Do not freeze a disconnected Grounding at the end of the cut. Its LOCAL
    // endpoint remains leased; the affected branch can choose a different DST.
    if (end + 2 == route.size() && route[end].type == Wire::WIRE_CROSSBAR &&
        route[end].pos != 0 && route[end].jump < 0 &&
        sameCoord(route[end].from, route[end].to) &&
        route.back().type == Wire::WIRE_TILE_PIN && route.back().pin_dir == 0)
        ++end;
    if (first == 0 || end <= first || end >= route.size())
        return first;
    const bool terminal_tail = end + 1 == route.size() &&
                               route[end].type == Wire::WIRE_TILE_PIN &&
                               route[end].pin_dir == 0;
    for (size_t i = first; i < end; ++i)
    {
        const auto &w = route[i];
        if (w.type == Wire::WIRE_GAP)
            continue;
        if (w.type == Wire::WIRE_ROUTE_EDGE)
            continue;
        if (terminal_tail && i + 1 == end && w.type == Wire::WIRE_CROSSBAR &&
            w.pos != 0 && w.jump < 0 && sameCoord(w.from, w.to))
            continue;
        if (w.type != Wire::WIRE_CROSSBAR || w.pos == 0 || w.jump < 0 ||
            w.dst < 0)
            return first;
    }

    const auto &left = route[first], &right = route[end - 1],
               &suffix = route[end];
    if (terminal_tail)
    {
        if (!sameCoord(right.to, suffix.from) ||
            (right.type == Wire::WIRE_GAP &&
             (right.to_node_type != CB_NODE_LOCAL ||
              right.to_node != suffix.local)))
            return first;
        return end;
    }

    const int last_dst =
        right.type == Wire::WIRE_GAP || right.type == Wire::WIRE_ROUTE_EDGE
            ? right.to_node
            : right.dst;
    if ((left.type != Wire::WIRE_CROSSBAR && left.type != Wire::WIRE_GAP &&
         !(left.type == Wire::WIRE_ROUTE_EDGE &&
           left.from_node_type == CB_NODE_DST)) ||
        (right.type != Wire::WIRE_CROSSBAR && right.type != Wire::WIRE_GAP &&
         !(right.type == Wire::WIRE_ROUTE_EDGE &&
           right.to_node_type == CB_NODE_DST)) ||
        suffix.type != Wire::WIRE_CROSSBAR ||
        !sameCoord(right.to, suffix.from) || last_dst != suffix.local)
        return first;
    return end;
}

std::vector<fpga::RouteCutNode>
fpga::previewNetRouteCuts(const std::vector<NetRouteRef> &victims,
                          const std::vector<RouteCutNode> &cuts,
                          bool preserve_downstream, OwnershipQueryWork *work)
{
    struct Suffix
    {
        NetRouteRef ref;
        const std::vector<Wire> *route;
        size_t cut, end;
    };

    std::vector<Suffix> suffixes;
    for (const auto &ref : victims)
    {
        if (!ref.net || ref.binding_index >= ref.net->routes.size())
            return {};
        const auto *route = bindingRoute(ref.net->routes[ref.binding_index]);
        if (!route || route->empty())
            return {};
        size_t cut = routeNodeCut(*route, cuts);
        if (cut == route->size())
            return {};
        const size_t end = preserve_downstream
                               ? transitCutEnd(*route, cut, cuts)
                               : route->size();
        if (end == cut)
            return {};
        suffixes.push_back({ref, route, cut, end});
    }

    std::vector<RouteCutNode> released;
    auto add = [&](Coord coord, CBNodeNameType type, int node)
    {
        Tile *tile = Device::current().getTile(coord.x, coord.y);
        if (!tile || node < 0)
            return;
        const RouteCutNode candidate{coord, type, node};
        if (std::any_of(released.begin(), released.end(),
                        [&](const auto &old)
                        {
                            return sameCoord(old.tile, coord) &&
                                   old.type == type && old.node == node;
                        }))
            return;
        released.push_back(candidate);
    };

    for (const auto &suffix : suffixes)
    {
        const auto *route = suffix.route;
        const size_t cut = suffix.cut;
        const bool preserve_landing =
            cut > 0 && (*route)[cut - 1].type == Wire::WIRE_CROSSBAR &&
            (*route)[cut].type == Wire::WIRE_CROSSBAR &&
            sameCoord((*route)[cut - 1].to, (*route)[cut].from) &&
            (*route)[cut - 1].dst == (*route)[cut].local;
        for (size_t i = cut; i < suffix.end; ++i)
        {
            const Wire &fragment = (*route)[i];
            if (fragment.type == Wire::WIRE_TILE_PIN)
            {
                if (i + 1 == route->size())
                    add(fragment.from, CB_NODE_LOCAL, fragment.local);
            }

            else if (fragment.type == Wire::WIRE_ROUTE_EDGE)
            {
                add(fragment.to,
                    static_cast<CBNodeNameType>(fragment.to_node_type),
                    fragment.to_node);
            }

            else if (fragment.type == Wire::WIRE_CROSSBAR)
            {
                add(fragment.from, CB_NODE_SRC, fragment.jump);
                add(fragment.from, CB_NODE_JOINT, fragment.joint);
                add(fragment.from, CB_NODE_JOINT, fragment.joint2);
                if (fragment.pos == 0)
                    add(fragment.from, CB_NODE_LOCAL, fragment.local);
                else if (fragment.owns_dst && !(preserve_landing && i == cut))
                    add(fragment.from, CB_NODE_DST, fragment.local);
                if (fragment.owns_landing)
                    add(fragment.to, CB_NODE_DST, fragment.dst);
            }

        }

    }

    // Check all proposed releases together. The old per-node query rescanned
    // the same Tile registrations and complete routes for every released SRC,
    // DST and JOINT. Only surviving fragments can prevent release: inspect each
    // live binding once, omitting precisely its selected cut interval.
    // This is a read-only, invocation-local calculation, never a cached policy.
    using Key = std::tuple<int, int, int, int>;
    auto key = [](Coord at, CBNodeNameType type, int node)
    { return Key{at.x, at.y, int(type), node}; };
    std::map<Key, size_t> candidates;
    std::array<NodeMask, 5> requested;
    for (size_t i = 0; i < released.size(); ++i)
    {
        const auto &n = released[i];
        candidates.emplace(key(n.tile, n.type, n.node), i);
        if (unsigned(n.type) < requested.size())
            requested[n.type].setBit(n.node);
    }

    std::vector<bool> retained(released.size());
    auto mark = [&](Coord at, CBNodeNameType type, int node)
    {
        if (node < 0 || unsigned(type) >= requested.size() ||
            !requested[type].testBit(node))
            return;
        const auto found = candidates.find(key(at, type, node));
        if (found != candidates.end())
            retained[found->second] = true;
    };

    std::unordered_set<const rtl::NetRouteBinding *> inspected;
    auto inspect = [&](rtl::Net *net, size_t index)
    {
        if (!net || index >= net->routes.size())
            return;
        auto &binding = net->routes[index];
        if (!inspected.insert(&binding).second)
            return;
        const auto *route = bindingRoute(binding);
        if (!route)
            return;
        const auto selected = std::find_if(
            suffixes.begin(), suffixes.end(), [&](const auto &s)
            { return s.ref.net == net && s.ref.binding_index == index; });
        for (size_t i = 0; i < route->size(); ++i)
        {
            if (selected != suffixes.end() && i >= selected->cut &&
                i < selected->end)
                continue;
            if (work)
                ++work->fragment_visits;
            const auto &w = (*route)[i];
            // Match routeNodeCut's physical dependencies, including shared
            // replicas, non-owning landings and both endpoints of explicit
            // graph edges.
            if (w.type == Wire::WIRE_ROUTE_EDGE)
            {
                mark(w.from, static_cast<CBNodeNameType>(w.from_node_type),
                     w.from_node);
                mark(w.to, static_cast<CBNodeNameType>(w.to_node_type),
                     w.to_node);
            }

            else if (w.type == Wire::WIRE_TILE_PIN)
            {
                mark(w.from, CB_NODE_LOCAL, w.local);
            }

            else if (w.type == Wire::WIRE_CROSSBAR)
            {
                mark(w.from, CB_NODE_SRC, w.jump);
                mark(w.from, w.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST, w.local);
                mark(w.to, CB_NODE_DST, w.dst);
                mark(w.from, CB_NODE_JOINT, w.joint);
                mark(w.from, CB_NODE_JOINT, w.joint2);
            }

        }

    };

    std::unordered_set<Tile *> visited_tiles;
    std::unordered_set<rtl::Net *> visited_fallback_nets;
    for (const auto &node : released)
    {
        auto *tile = Device::current().getTile(node.tile.x, node.tile.y);
        if (!visited_tiles.insert(tile).second)
            continue;
        if (work)
            ++work->queries;
        if (tile->routed_bindings_authoritative)
        {
            for (const auto &ref : tile->routed_bindings)
                if (ref.net)
                {
                    if (work)
                        ++work->binding_resolutions;
                    inspect(ref.net,
                            ref.net->findRouteBindingById(ref.route_id));
                }

        }

        else
        {
            for (const auto &ref : tile->routedNets)
                if (ref.peer && visited_fallback_nets.insert(ref.peer).second)
                    for (size_t i = 0; i < ref.peer->routes.size(); ++i)
                    {
                        if (work)
                            ++work->binding_resolutions;
                        inspect(ref.peer, i);
                    }

        }

    }

    size_t next = 0;
    for (size_t i = 0; i < released.size(); ++i)
        if (!retained[i])
            released[next++] = released[i];
    released.resize(next);
    return released;
}

std::vector<fpga::RouteCutNode>
fpga::previewNetRouteCut(rtl::Net &net, size_t binding_index,
                         const std::vector<RouteCutNode> &cuts)
{
    return previewNetRouteCuts({{&net, binding_index}}, cuts);
}

// Preemption cannot release the source endpoint or its first physical hop.
bool fpga::canPreemptNetRouteFromNodes(rtl::Net &net,
                                       size_t route_binding_index,
                                       const std::vector<RouteCutNode> &nodes)
{
    if (!net.routeCanBePreempted() ||
        route_binding_index >= net.routes.size() || nodes.empty())
    {
        return false;
    }

    const std::vector<Wire> *route =
        bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty())
    {
        return false;
    }

    const size_t cut = routeNodeCut(*route, nodes);
    for (size_t i = 0; i < cut; ++i)
    {
        const Wire &wire = (*route)[i];
        if (wire.type == Wire::WIRE_CROSSBAR && wire.jump >= 0 &&
            !sameCoord(wire.from, wire.to))
        {
            return cut < route->size();
        }

    }

    return false;
}

// Cut before every requested resource while preserving a nonconflicting
// landing.
bool fpga::unrouteNetRouteFromNodes(rtl::Net &net, size_t route_binding_index,
                                    const std::vector<RouteCutNode> &nodes)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size() || nodes.empty())
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty())
    {
        return false;
    }

    const size_t cut = routeNodeCut(*route, nodes);
    if (cut == route->size())
    {
        return false;
    }

    if (cut == 0)
    {
        return unrouteNetRoute(net, route_binding_index);
    }

    std::vector<Wire> removed(route->begin() + static_cast<std::ptrdiff_t>(cut),
                              route->end());
    if (!removed.empty() && (*route)[cut - 1].type == Wire::WIRE_CROSSBAR &&
        removed.front().type == Wire::WIRE_CROSSBAR &&
        sameCoord((*route)[cut - 1].to, removed.front().from) &&
        (*route)[cut - 1].dst == removed.front().local)
    {
        (*route)[cut - 1].owns_landing = true;
        removed.front().owns_dst = false;
    }

    route->resize(cut);
    clearRouteLeases(removed, true);
    rebuildNetRouteTiles(net, {removed});
    return true;
}

bool fpga::unrouteNetRoutesFromNodes(const std::vector<NetRouteRef> &victims,
                                     const std::vector<RouteCutNode> &cuts,
                                     bool preserve_downstream,
                                     OwnershipQueryWork *release_work)
{
    const auto dependent = findNetRouteCutVictims(cuts);
    if (victims.empty() || victims.size() != dependent.size())
        return false;

    struct Removal
    {
        rtl::Net *net;
        std::vector<Wire> *route;
        size_t cut, end;
        std::vector<Wire> removed;
    };

    std::vector<Removal> removals;
    for (const auto &ref : dependent)
    {
        if (std::none_of(victims.begin(), victims.end(),
                         [&](const auto &v)
                         {
                             return v.net == ref.net &&
                                    v.binding_index == ref.binding_index;
                         }) ||
            !canPreemptNetRouteFromNodes(*ref.net, ref.binding_index, cuts))
            return false;
        auto *route = bindingRoute(ref.net->routes[ref.binding_index]);
        size_t cut = routeNodeCut(*route, cuts);
        const size_t end = preserve_downstream
                               ? transitCutEnd(*route, cut, cuts)
                               : route->size();
        if (end == cut)
            return false;
        removals.push_back(
            {ref.net, route, cut, end,
             std::vector<Wire>(
                 route->begin() + static_cast<std::ptrdiff_t>(cut),
                 route->begin() + static_cast<std::ptrdiff_t>(end))});
    }

    // Every check and suffix copy precedes mutation. Remove all claims before
    // releasing any shared resource, so binding enumeration order cannot
    // matter.
    std::vector<std::unique_ptr<RouteHistoryScope>> history;
    if (!routeHistoryNet().empty() || routeHistoryWatch().node >= 0)
    {
        std::unordered_set<rtl::Net *> watched;
        for (const auto &r : removals)
            if (watched.insert(r.net).second)
                history.push_back(
                    std::make_unique<RouteHistoryScope>(r.net, __func__));
    }

    for (auto &r : removals)
    {
        auto &previous = (*r.route)[r.cut - 1];
        auto &first = r.removed.front();
        const int first_dst = first.type == Wire::WIRE_ROUTE_EDGE &&
                                      first.from_node_type == CB_NODE_DST
                                  ? first.from_node
                                  : first.local;
        if (previous.type == Wire::WIRE_CROSSBAR &&
            (first.type == Wire::WIRE_CROSSBAR ||
             first.type == Wire::WIRE_ROUTE_EDGE) &&
            sameCoord(previous.to, first.from) && previous.dst == first_dst)
        {
            previous.owns_landing = true;
            first.owns_dst = false;
        }

        if (preserve_downstream)
        {
            Wire gap;
            gap.type = Wire::WIRE_GAP;
            gap.from = first.from;
            gap.to = r.removed.back().to;
            gap.from_node_type = gap.to_node_type = CB_NODE_DST;
            gap.from_node = first.type == Wire::WIRE_GAP ||
                                    first.type == Wire::WIRE_ROUTE_EDGE
                                ? first.from_node
                                : first.local;
            gap.to_node = r.removed.back().type == Wire::WIRE_GAP ||
                                  r.removed.back().type == Wire::WIRE_ROUTE_EDGE
                              ? r.removed.back().to_node
                              : r.removed.back().dst;
            if ((*r.route)[r.end].type == Wire::WIRE_TILE_PIN)
            {
                gap.to_node_type = CB_NODE_LOCAL;
                gap.to_node = (*r.route)[r.end].local;
            }

            // The retained tail must own its resources independently. Another
            // victim can have a LONGER cut, removing owners inside this tail,
            // not just at its first DST. In particular a retained FORK may
            // borrow its input from that other victim. Promoting only
            // route[end] silently releases the fork's DST during
            // clearRouteLeases(). All conflicting occurrences have already been
            // removed by transitCutEnd; these are genuine surviving same-source
            // dependencies, including later islands.
            for (size_t i = r.end; i < r.route->size(); ++i)
            {
                auto &retained = (*r.route)[i];
                if (retained.type == Wire::WIRE_GAP)
                    continue;
                retained.shared = false;
                if (retained.type == Wire::WIRE_CROSSBAR)
                {
                    retained.owns_dst = retained.pos != 0;
                    retained.owns_landing =
                        retained.jump >= 0 && retained.dst >= 0;
                }

            }

            r.route->erase(r.route->begin() + r.cut, r.route->begin() + r.end);
            r.route->insert(r.route->begin() + r.cut, std::move(gap));
        }

        else
        {
            r.route->resize(r.cut);
        }

    }

    std::vector<const std::vector<Wire> *> removed_refs;
    for (const auto &r : removals)
        removed_refs.push_back(&r.removed);
    clearRouteLeases(removed_refs, true, release_work);
    std::unordered_map<rtl::Net *, std::vector<std::vector<Wire>>> affected;
    for (auto &r : removals)
        affected[r.net].push_back(std::move(r.removed));
    for (const auto &[net, routes] : affected)
        rebuildNetRouteTiles(*net, routes);
    return true;
}

namespace
{
// Ownership alone is insufficient: shared fragments can reference a resource
// without owning its lease. Only the current binding's private terminal nodes
// may change input; the pin stays reserved and no other binding may use them.
// The caller limits this exception to the sink LOCAL and sink-Tile JOINTs.
bool canReplacePrivateTerminalInput(
    rtl::Net &net, size_t index, Tile &tile, fpga::CBNodeNameType kind,
    int node, const std::vector<fpga::NetRouteRef> &owners)
{
    if (owners.empty() ||
        std::any_of(owners.begin(), owners.end(), [&](const auto &o)
                    { return o.net != &net || o.binding_index != index; }))
        return false;
    std::unordered_set<rtl::Net *> nets{&net};
    for (const auto &ref : tile.routedNets)
        if (ref.peer)
            nets.insert(ref.peer);
    for (const auto &ref : tile.routed_bindings)
        if (ref.net)
            nets.insert(ref.net);
    for (auto *n : nets)
        for (size_t i = 0; i < n->routes.size(); ++i)
        {
            if (n == &net && i == index)
                continue;
            const auto *other = bindingRoute(n->routes[i]);
            if (other &&
                routeUsesNodeOnTile(*other, tile, kind, node, false, false))
                return false;
        }

    return true;
}

} // namespace

static bool auditRouteReachability(rtl::Net &net, size_t binding_index,
                                   const rtl::NetRouteBinding &binding,
                                   const std::vector<Wire> *route,
                                   std::ostream &out, bool preserve_inputs,
                                   size_t max_states)
{
    if (!route || route->empty() ||
        route->front().type != Wire::WIRE_TILE_PIN ||
        route->back().type != Wire::WIRE_TILE_PIN)
        return false;
    using Node = std::tuple<int, int, int, int>;
    auto node = [](Coord c, int kind, int id)
    { return Node{c.x, c.y, kind, id}; };
    auto &device = Device::current();
    const Node sink =
        node(route->back().from, CB_NODE_LOCAL, route->back().local);
    auto same_source = [&](const rtl::NetRouteBinding &b)
    {
        return rtl::sameRouteSource(b.from, b.from_port, binding.from,
                                    binding.from_port);
    };

    auto print_node = [&](const Node &n)
    {
        auto [x, y, kind, id] = n;
        out << "tile=" << x << ',' << y << " kind=" << kind << " node=" << id;
    };

    // Recover physical input arcs, including replicas and downstream islands.
    // A same-source busy bit is not permission to short two distinct inputs.
    std::map<Node, std::set<Node>> inputs;
    std::unordered_set<rtl::Net *> nets{&net};
    for (auto &tile : device.tile_grid)
        for (auto &ref : tile.routedNets)
            if (ref.peer)
                nets.insert(ref.peer);
    for (auto *n : nets)
        for (auto &b : n->routes)
        {
            if (!same_source(b))
                continue;
            const auto *wires = bindingRoute(b);
            if (!wires)
                continue;
            for (size_t i = 0; i < wires->size(); ++i)
            {
                const auto &w = (*wires)[i];
                if (w.type == Wire::WIRE_ROUTE_EDGE)
                {
                    inputs[node(w.to, w.to_node_type, w.to_node)].insert(
                        node(w.from, w.from_node_type, w.from_node));
                }

                else if (w.type == Wire::WIRE_CROSSBAR)
                {
                    Node previous =
                        node(w.from, w.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST,
                             w.local);
                    auto arc = [&](Node next)
                    {
                        inputs[next].insert(previous);
                        previous = next;
                    };

                    if (w.joint2 >= 0)
                        arc(node(w.from, CB_NODE_JOINT, w.joint2));
                    if (w.joint >= 0)
                        arc(node(w.from, CB_NODE_JOINT, w.joint));
                    if (w.jump >= 0)
                    {
                        arc(node(w.from, CB_NODE_SRC, w.jump));
                        arc(node(w.to, CB_NODE_DST, w.dst));
                    }

                    else if (i + 1 < wires->size() &&
                             (*wires)[i + 1].type == Wire::WIRE_TILE_PIN)
                    {
                        arc(node((*wires)[i + 1].from, CB_NODE_LOCAL,
                                 (*wires)[i + 1].local));
                    }

                }

            }

        }

    struct Lease
    {
        bool busy = false, foreign = false, private_sink = false;
        std::string owners;
    };

    std::map<Node, Lease> leases;
    auto lease = [&](const Node &n) -> const Lease &
    {
        auto [it, inserted] = leases.try_emplace(n);
        if (!inserted)
            return it->second;
        auto [x, y, kind, id] = n;
        auto *tile = device.getTile(x, y);
        auto &l = it->second;
        if (!tile || !tile->cb_type || id < 0)
        {
            l.foreign = true;
            return l;
        }

        switch (kind)
        {
        case CB_NODE_LOCAL:
            l.busy =
                tile->cb.local.local.testBit(id) || tile->isPinNodeLeased(id);
            break;
        case CB_NODE_DST:
            l.busy = tile->cb.dst.jump.testBit(id);
            break;
        case CB_NODE_SRC:
            l.busy = tile->cb.src.jump.testBit(id);
            break;
        case CB_NODE_JOINT:
            l.busy = tile->cb.joint.jump.testBit(id);
            break;
        default:
            l.foreign = true;
            return l;
        }

        if (!l.busy)
            return l;
        const auto owners =
            findNetOwnersByNode(*tile, static_cast<CBNodeNameType>(kind), id);
        l.foreign = owners.empty();
        for (const auto &ref : owners)
        {
            if (!l.owners.empty())
                l.owners += ';';
            l.owners += ref.net->name + ":" + std::to_string(ref.binding_index);
            l.foreign |= !same_source(ref.net->routes[ref.binding_index]);
        }

        if (binding_index < net.routes.size() && !l.foreign &&
            (n == sink || (kind == CB_NODE_JOINT && x == std::get<0>(sink) &&
                           y == std::get<1>(sink))))
            l.private_sink = canReplacePrivateTerminalInput(
                net, binding_index, *tile, static_cast<CBNodeNameType>(kind),
                id, owners);
        return l;
    };

    const Node source =
        node(route->front().from, CB_NODE_LOCAL, route->front().local);
    std::vector<Node> goals;
    for (const auto &w : *route)
        if (w.type == Wire::WIRE_GAP)
            goals.push_back(node(w.to, w.to_node_type, w.to_node));
    if (std::find(goals.begin(), goals.end(), sink) == goals.end())
        goals.push_back(sink);

    struct Step
    {
        Node node;
        int parent;
    };

    std::vector<Step> steps{{source, -1}};
    std::map<Node, int> seen{{source, 0}};
    std::vector<int> reached(goals.size(), -1);
    size_t foreign = 0, conflicting_input = 0, packed = 0;
    bool limited = false;
    out << "SOURCE_REACH begin net=" << net.name << " binding=" << binding_index
        << " preserve_inputs=" << preserve_inputs
        << " max_states=" << max_states << '\n';
    out.flush();
    for (size_t i = 0; i < steps.size(); ++i)
    {
        const Node current = steps[i].node;
        for (size_t g = 0; g < goals.size(); ++g)
            if (current == goals[g])
                reached[g] = i;
        if (std::all_of(reached.begin(), reached.end(),
                        [](int r) { return r >= 0; }))
            break;
        auto [x, y, kind, id] = current;
        auto *tile = device.getTile(x, y);
        if (!tile || !tile->cb_type)
            continue;
        const auto &cb =
            *tile->cb_type; // const tables: no lazy insertions into topology
        auto add = [&](Node next)
        {
            const auto [nx, ny, nkind, nid] = next;
            if (nkind == CB_NODE_JOINT ||
                (nkind == CB_NODE_LOCAL && next != sink && next != source))
            {
                auto *at = device.getTile(nx, ny);
                if (!at || !at->cb_type)
                    return;
                const bool terminal =
                    nx == std::get<0>(sink) && ny == std::get<1>(sink);
                const auto reserved = packedInputJointReservations(
                    *at, terminal ? binding.to : nullptr,
                    terminal ? binding.to_port : std::string{});
                if ((nkind == CB_NODE_JOINT && reserved.testBit(nid)) ||
                    (nkind == CB_NODE_LOCAL &&
                     at->input_local_reservations.count(nid)))
                {
                    ++packed;
                    return;
                }

            }

            const auto &l = lease(next);
            if (l.foreign)
            {
                ++foreign;
                return;
            }

            if (preserve_inputs && l.busy && !l.private_sink)
            {
                auto old = inputs.find(next);
                if (old != inputs.end() &&
                    (old->second.size() != 1 || !old->second.count(current)))
                {
                    ++conflicting_input;
                    return;
                }

            }

            if (seen.count(next))
                return;
            if (steps.size() >= max_states)
            {
                limited = true;
                return;
            }

            seen[next] = steps.size();
            steps.push_back({next, static_cast<int>(i)});
        };

        auto mask = [&](const NodeMask &bits, int to_kind)
        {
            bits.for_each_set_bit(
                [&](int to)
                {
                    add(Node{x, y, to_kind, to});
                    return false;
                });
        };

        switch (kind)
        {
        case CB_NODE_LOCAL:
            mask(cb.local_src[id].jump, CB_NODE_SRC);
            mask(cb.local_joint[id].joint, CB_NODE_JOINT);
            mask(cb.local_local[id].local, CB_NODE_LOCAL);
            for (const auto &t : device.resolveLocalTargets(*tile, id))
                if (t.tile)
                    add(node(t.tile->coord, t.node_type, t.node));
            break;
        case CB_NODE_DST:
            mask(cb.dst_src[id].jump, CB_NODE_SRC);
            mask(cb.dst_joint[id].joint, CB_NODE_JOINT);
            mask(cb.dst_local[id].local, CB_NODE_LOCAL);
            break;
        case CB_NODE_JOINT:
            mask(cb.joint_src[id].jump, CB_NODE_SRC);
            mask(cb.joint_joint[id].joint, CB_NODE_JOINT);
            mask(cb.joint_local[id].local, CB_NODE_LOCAL);
            break;
        case CB_NODE_SRC:
            mask(cb.src_joint[id].joint, CB_NODE_JOINT);
            for (const auto &t : device.resolveJumpTargets(*tile, id))
                if (t.tile && t.dst_node >= 0)
                    add(node(t.tile->coord, CB_NODE_DST, t.dst_node));
            break;
        }

    }

    out << "SOURCE_REACH end states=" << steps.size() << " limited=" << limited
        << " foreign_rejections=" << foreign
        << " input_rejections=" << conflicting_input
        << " packed_rejections=" << packed << '\n';
    // A bounded router's frontier is not the whole free component. Preserve
    // witnesses from this independent audit before its temporary state goes
    // away, so a failed sink does not hide reachable Grounding approaches.
    int closest = 0, closest_distance = std::numeric_limits<int>::max();
    std::map<std::pair<int, int>, std::pair<size_t, int>> region;
    for (size_t i = 0; i < steps.size(); ++i)
    {
        const auto [x, y, kind, id] = steps[i].node;
        const int distance = std::max(std::abs(x - std::get<0>(sink)),
                                      std::abs(y - std::get<1>(sink)));
        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest = i;
        }

        if (distance <= GROUNDING_RADIUS)
        {
            auto [it, inserted] =
                region.try_emplace({x, y}, std::pair<size_t, int>{0, int(i)});
            ++it->second.first;
            // Prefer a reached DST over an internal bridge as the Tile witness.
            if (kind == CB_NODE_DST &&
                std::get<2>(steps[it->second.second].node) != CB_NODE_DST)
                it->second.second = i;
        }

    }

    out << "SOURCE_CLOSEST distance=" << closest_distance << ' ';
    print_node(steps[closest].node);
    out << " grounding_tiles=" << region.size() << '\n';
    auto witness = [&](int end, const char *tag)
    {
        std::vector<int> path;
        for (int p = end; p >= 0; p = steps[p].parent)
            path.push_back(p);
        std::reverse(path.begin(), path.end());
        for (size_t p = 0; p < path.size(); ++p)
        {
            out << tag << " step=" << p << ' ';
            print_node(steps[path[p]].node);
            out << '\n';
        }

    };

    witness(closest, "SOURCE_CLOSEST_PATH");
    for (const auto &[coord, entry] : region)
    {
        out << "SOURCE_REGION tile=" << coord.first << ',' << coord.second
            << " states=" << entry.first << " witness=" << entry.second << '\n';
        witness(entry.second, "SOURCE_REGION_PATH");
    }

    for (size_t g = 0; g < goals.size(); ++g)
    {
        out << "SOURCE_GOAL goal=" << g << " reached=" << (reached[g] >= 0)
            << ' ';
        print_node(goals[g]);
        out << '\n';
        if (reached[g] < 0)
            continue;
        std::vector<int> path;
        for (int p = reached[g]; p >= 0; p = steps[p].parent)
            path.push_back(p);
        std::reverse(path.begin(), path.end());
        for (size_t p = 0; p < path.size(); ++p)
        {
            const auto &n = steps[path[p]].node;
            const auto &l = lease(n);
            out << "SOURCE_PATH goal=" << g << " step=" << p << ' ';
            print_node(n);
            out << " busy=" << l.busy << " foreign=" << l.foreign
                << " owners=" << l.owners << '\n';
        }

    }

    out.flush();
    for (size_t g = 0; g < goals.size(); ++g)
        if (goals[g] == sink && reached[g] >= 0)
            return true;
    return false;
}

bool fpga::auditNetRouteReachability(rtl::Net &net, size_t binding_index,
                                     std::ostream &out, bool preserve_inputs,
                                     size_t max_states)
{
    if (binding_index >= net.routes.size())
        return false;
    auto &binding = net.routes[binding_index];
    return auditRouteReachability(net, binding_index, binding,
                                  bindingRoute(binding), out, preserve_inputs,
                                  max_states);
}

bool fpga::auditUnroutedReachability(rtl::Net &net,
                                     const rtl::NetRouteBinding &endpoints,
                                     const Wire &source, const Wire &sink,
                                     std::ostream &out, size_t max_states)
{
    // Diagnostic endpoints are not registered or leased. In particular, do not
    // append a fake binding to the live net just to inspect an unrouted task.
    const std::vector<Wire> route{source, sink};
    return auditRouteReachability(net, std::numeric_limits<size_t>::max(),
                                  endpoints, &route, out, true, max_states);
}

namespace
{
using RepairNode = WireNode;

// Read the active trace at each decision so a deadline window can start
// mid-search.
void traceRepairDecision(std::string_view phase, const RepairNode &from,
                         const RepairNode &to, std::string_view result)
{
    auto *trace = fpga::RouteSearchTrace::current;
    if (!trace)
        return;
    auto print = [&](const RepairNode &node)
    {
        const auto [x, y, kind, id] = node;
        trace->stream() << '(' << x << ',' << y << ")/" << kind << ':' << id;
    };

    trace->stream() << "REPAIR_DECISION phase=" << phase << " from=";
    print(from);
    trace->stream() << " to=";
    print(to);
    trace->stream() << " result=" << result << '\n';
    for (const auto &node : {from, to})
    {
        auto *tile = fpga::Device::current().getTile(std::get<0>(node),
                                                     std::get<1>(node));
        if (tile)
            trace->snapshot(*tile);
    }

}

// Search labels are looked up by exact numeric identity, never key order.
using RepairNodeHash = WireNodeHash;
// Numeric label lookup is private to one source trace.
using RepairLabelMap = absl::flat_hash_map<RepairNode, int>;

struct RepairStep
{
    RepairNode node;
    int parent, hops, cost, goal;
};

// Exclusive phase timing and per-call numeric counters. The overlap map is
// temporary diagnostic memory, never an eligibility filter or routing cache.
struct RepairPhaseTimer
{
    static thread_local RepairPhaseTimer *current;
    static thread_local uint64_t sequence;
    bool enabled;
    const rtl::Net &net;
    size_t binding;
    const char *phase;
    bool gap, success = false, cancelled = false;
    RepairPhaseTimer *parent = nullptr;
    uint64_t id = 0, revision = 0, route_id = 0;
    std::string route_name;
    RepairStatistics stats;
    std::unordered_map<RepairNode, uint8_t, RepairNodeHash> expanded_nodes;
    std::chrono::steady_clock::time_point start, began;

    RepairPhaseTimer(const rtl::Net &n, size_t b, const char *p,
                     const rtl::NetRouteBinding *unbound = nullptr,
                     bool observe = true)
        : enabled(observe && std::getenv("SCALEPNR_PROFILE_REPAIR") != nullptr),
          net(n), binding(b), phase(p),
          gap(std::string_view(p) == "exact_gap_search")
    {
        if (!enabled)
            return;
        const auto *bnd = unbound               ? unbound
                          : b < n.routes.size() ? &n.routes[b]
                                                : nullptr;
        // Filter before allocating per-node diagnostics; routing eligibility is
        // unchanged.
        if (const char *filter = std::getenv("SCALEPNR_PROFILE_REPAIR_NET");
            filter && *filter && n.name.find(filter) == std::string::npos &&
            (!bnd || bnd->route_name.find(filter) == std::string::npos))
        {
            enabled = false;
            return;
        }

        began = start = std::chrono::steady_clock::now();
        parent = current;
        if (parent)
            parent->pause(start);
        current = this;
        id = ++sequence;
        revision = routeMutationRevision();
        if (bnd)
        {
            route_id = bnd->route_id;
            route_name = bnd->route_name;
        }

        add(gap ? RepairCounter::gap_calls : RepairCounter::source_calls);
    }

    void add(RepairCounter counter, uint64_t amount = 1)
    {
        if (enabled)
            stats.counters[unsigned(counter)] += amount;
    }

    void expand(const RepairNode &node)
    {
        if (!enabled)
            return;
        add(gap ? RepairCounter::gap_expanded : RepairCounter::source_expanded);
        auto &bits = expanded_nodes[node];
        const uint8_t own = gap ? 1 : 2;
        const bool fresh = !(bits & own);
        add(gap ? (fresh ? RepairCounter::gap_unique
                         : RepairCounter::gap_reexpanded)
                : (fresh ? RepairCounter::source_unique
                         : RepairCounter::source_reexpanded));
        if (gap && fresh && (bits & 2))
            add(RepairCounter::gap_source_overlap);
        bits |= own;
        if (!gap && fresh && parent && parent->gap)
        {
            auto &shared = parent->expanded_nodes[node];
            if (shared & 2)
                parent->add(RepairCounter::source_cross_call_repeat);
            else if (shared & 1)
                parent->add(RepairCounter::gap_source_overlap);
            shared |= 2;
        }

    }

    void pause(std::chrono::steady_clock::time_point now)
    {
        const auto found =
            std::find(repairPhaseNames.begin(), repairPhaseNames.end(), phase);
        if (found != repairPhaseNames.end())
            stats.seconds[found - repairPhaseNames.begin()] +=
                std::chrono::duration<double>(now - start).count();
    }

    void next(const char *p)
    {
        if (enabled)
        {
            const auto now = std::chrono::steady_clock::now();
            pause(now);
            start = now;
        }

        phase = p;
    }

    ~RepairPhaseTimer()
    {
        if (!enabled)
            return;
        const auto end = std::chrono::steady_clock::now();
        pause(end);
        if (cancelled)
            add(RepairCounter::cancelled);
        if (success)
            add(gap ? RepairCounter::gap_success
                    : RepairCounter::source_success);
        for (size_t i = 0; i < stats.counters.size(); ++i)
            repair_statistics.counters[i] += stats.counters[i];
        for (size_t i = 0; i < stats.seconds.size(); ++i)
            repair_statistics.seconds[i] += stats.seconds[i];
        std::ostringstream out;
        out << "REPAIR_CALL id=" << id
            << " parent=" << (parent ? parent->id : 0)
            << " kind=" << (gap ? "gap" : "source")
            << " net=" << std::quoted(net.name)
            << " route=" << std::quoted(route_name) << " binding=" << binding
            << " route_id=" << route_id << " revision=" << revision
            << " end_revision=" << routeMutationRevision() << " result="
            << (success     ? "success"
                : cancelled ? "cancelled"
                            : "failed")
            << " wall_s=" << std::chrono::duration<double>(end - began).count();
        for (size_t i = 0; i < stats.counters.size(); ++i)
            if (stats.counters[i])
                out << ' ' << repairCounterNames[i] << '=' << stats.counters[i];
        for (size_t i = 0; i < stats.seconds.size(); ++i)
            if (stats.seconds[i])
                out << ' ' << repairPhaseNames[i] << "_s=" << stats.seconds[i];
        out << '\n';
        const auto line = out.str();
        std::fwrite(line.data(), 1, line.size(), stderr);
        current = parent;
        if (parent)
            parent->start = std::chrono::steady_clock::now();
    }

};

thread_local RepairPhaseTimer *RepairPhaseTimer::current = nullptr;
thread_local uint64_t RepairPhaseTimer::sequence = 0;

RepairNode repairNode(Coord at, int type, int node)
{
    return {at.x, at.y, type, node};
}

Coord repairCoord(const RepairNode &node)
{
    return {std::get<0>(node), std::get<1>(node)};
}

// Numeric expansion needs one lease bit, not a copy of the 4096-bit mask.
bool repairNodeLeased(const CBState &state, int kind, int id)
{
    switch (kind)
    {
    case CB_NODE_SRC:
        return state.src.jump.testBit(id);
    case CB_NODE_DST:
        return state.dst.jump.testBit(id);
    case CB_NODE_JOINT:
        return state.joint.jump.testBit(id);
    case CB_NODE_LOCAL:
        return state.local.local.testBit(id);
    default:
        return false;
    }

}

// Lease one source-trace node and exclude it from retained-route reuse.
// Source recursion and retained-suffix checks release only their own bit;
// existing leases and reusable route nodes survive the call unchanged.
struct RepairNodeLease
{
    NodeMask &mask;
    int node;
    bool was_leased;
    std::set<RepairNode> *reusable_nodes;
    std::set<RepairNode>::node_type reusable_node;
    std::set<RepairNode> &protected_nodes;
    const RepairNode &at;
    bool protect_cut;

    static NodeMask &nodeMask(CBState &state, int kind)
    {
        switch (kind)
        {
        case CB_NODE_SRC:
            return state.src.jump;
        case CB_NODE_DST:
            return state.dst.jump;
        case CB_NODE_JOINT:
            return state.joint.jump;
        case CB_NODE_LOCAL:
            return state.local.local;
        default:
            throw std::logic_error("invalid source trace node type");
        }

    }

    RepairNodeLease(Tile &tile, const RepairNode &at,
                    std::set<RepairNode> &protected_nodes,
                    std::set<RepairNode> &detached, bool protect_cut = false)
        : mask(nodeMask(tile.cb, std::get<2>(at))), node(std::get<3>(at)),
          was_leased(mask.testBit(node)), reusable_nodes(&protected_nodes),
          reusable_node(protected_nodes.extract(at)),
          protected_nodes(protected_nodes), at(at), protect_cut(protect_cut)
    {
        if (reusable_node.empty())
        {
            reusable_nodes = &detached;
            reusable_node = detached.extract(at);
        }

        mask.setBit(node);
        if (protect_cut)
            protected_nodes.insert(at);
    }

    RepairNodeLease(const RepairNodeLease &) = delete;
    RepairNodeLease &operator=(const RepairNodeLease &) = delete;

    ~RepairNodeLease()
    {
        if (!was_leased)
            mask.clearBit(node);
        if (protect_cut)
            protected_nodes.erase(at);
        if (!reusable_node.empty())
            reusable_nodes->insert(std::move(reusable_node));
    }

};

// Read the current Tile's bindings once for this expanded incoming DST.
// Results live only until the next expansion; no occupancy survives a commit
// or a later arrival at this Tile. Keep the last matching fragment just as the
// per-SRC scan did, including canonical aliases and real SRC ownership.
struct SharedRepairEdge
{
    Coord to;
    int dst = -1;
    int joint = -1;
    int joint2 = -1;
};

using SharedRepairEdges = std::array<SharedRepairEdge, CB_MAX_NODES>;

void collectSharedRepairEdges(Tile &tile, int dst,
                              const rtl::NetRouteBinding &source,
                              NodeMask &found, SharedRepairEdges &edges)
{
    found = {};
    auto *observations = RepairPhaseTimer::current;
    auto inspect = [&](rtl::Net &net, size_t index)
    {
        if (observations)
            observations->add(RepairCounter::shared_binding_visits);
        if (index >= net.routes.size())
            return;
        auto &b = net.routes[index];
        if (observations)
            observations->add(RepairCounter::shared_source_checks);
        if (!rtl::sameRouteSource(b.from, b.from_port, source.from,
                                  source.from_port))
            return;
        const auto *route = bindingRoute(b);
        if (!route)
            return;
        for (size_t i = 0; i < route->size(); ++i)
        {
            const auto &w = (*route)[i];
            if (observations)
                observations->add(RepairCounter::shared_fragment_checks);
            int src = -1;
            SharedRepairEdge edge;
            if (w.type == Wire::WIRE_CROSSBAR)
            {
                if (w.pos == 0 || w.local != dst ||
                    !sameCoord(w.from, tile.coord))
                    continue;
                src = w.jump;
                edge = {w.to, w.dst, w.joint, w.joint2};
            }

            else if (w.type == Wire::WIRE_ROUTE_EDGE &&
                     w.from_node_type == CB_NODE_DST && w.from_node == dst &&
                     sameCoord(w.from, tile.coord))
            {
                // Decode the same local arc and jump from contiguous typed
                // fragments. Never infer a shared exit from SRC ownership alone
                // or cross a gap.
                int kind = CB_NODE_DST, node = dst, joints = 0;
                for (size_t j = i; j < route->size() && j < i + 4; ++j)
                {
                    const auto &part = (*route)[j];
                    if (j != i && observations)
                        observations->add(
                            RepairCounter::shared_fragment_checks);
                    if (part.type != Wire::WIRE_ROUTE_EDGE ||
                        !sameCoord(part.from, tile.coord) ||
                        part.from_node_type != kind || part.from_node != node ||
                        part.to_node < 0 || part.to_node >= CB_MAX_NODES)
                        break;
                    if (kind == CB_NODE_SRC)
                    {
                        if (part.to_node_type == CB_NODE_DST)
                        {
                            src = node;
                            edge.to = part.to;
                            edge.dst = part.to_node;
                        }

                        break;
                    }

                    if (!sameCoord(part.to, tile.coord))
                        break;
                    if (part.to_node_type == CB_NODE_JOINT && joints < 2)
                    {
                        edge.joint2 = edge.joint;
                        edge.joint = part.to_node;
                        ++joints;
                    }

                    else if (part.to_node_type != CB_NODE_SRC)
                        break;
                    kind = part.to_node_type;
                    node = part.to_node;
                }

            }

            if (src < 0 || src >= CB_MAX_NODES || edge.dst < 0 ||
                !routeUsesNodeOnTile(*route, tile, CB_NODE_SRC, src, false,
                                     true))
                continue;
            found.setBit(src);
            edges[src] = edge;
        }

    };

    if (tile.routed_bindings_authoritative)
    {
        for (const auto &ref : tile.routed_bindings)
            if (ref.net)
                inspect(*ref.net, ref.net->findRouteBindingById(ref.route_id));
    }

    else
    {
        for (const auto &ref : tile.routedNets)
            if (ref.peer)
                for (size_t i = 0; i < ref.peer->routes.size(); ++i)
                    inspect(*ref.peer, i);
    }

}

// The configured cap bounds fast free probes, not a later explicit-budget
// retry.
size_t sourceFreeStateLimit(size_t max_states)
{
    if (const char *value = std::getenv("SCALEPNR_FREE_SEARCH_STATES"))
    {
        const std::string_view setting(value);
        size_t requested = 0;
        const auto parsed = std::from_chars(
            setting.data(), setting.data() + setting.size(), requested);
        if (parsed.ec != std::errc{} ||
            parsed.ptr != setting.data() + setting.size() || !requested)
            throw std::runtime_error(
                "SCALEPNR_FREE_SEARCH_STATES must be a positive integer");
        return std::min(max_states, requested);
    }

    return max_states;
}

// A failed exact hole can have a trapped boundary even though another branch
// reaches the sink. Search from the real driver, never from a detached island.
// Used for a bounded free-only probe and after exact-gap search fails.
bool repairFromSource(
    rtl::Net &net, size_t index, const std::function<bool()> &cancel,
    size_t max_states, int max_hops,
    const fpga::RepairGroundingPreemption &grounding_preemption,
    const rtl::NetRouteBinding *unbound = nullptr,
    const std::vector<Wire> *endpoints = nullptr,
    bool existing_takeoff_only = false, bool diagnostic_only = false,
    int cost_scale = 1, std::ostream *diagnostic_log = nullptr,
    bool diagnostic_frontier = false, bool fast_free_probe = true,
    const RepairIncomingSources &incoming_sources = {},
    bool *budget_limited = nullptr,
    const RouteCutNode *required_branch = nullptr,
    bool preview_relocated_source = false)
{
    RepairPhaseTimer timer(net, index, "source_setup", unbound,
                           !diagnostic_only);
    const std::function<bool()> cancelled =
        cancel ? std::function<bool()>(
                     [&]
                     {
                         const bool stop = cancel();
                         timer.cancelled |= stop;
                         return stop;
                     })

               : std::function<bool()>{};
    auto mark_deadend = [&](Tile &tile, int src)
    { tile.cb.src_deadend.jump.setBit(src); };
    auto is_deadend = [&](Tile &tile, int src)
    { return tile.cb.src_deadend.jump.testBit(src); };
    const auto &binding = unbound ? *unbound : net.routes[index];
    const auto *route = endpoints ? endpoints : bindingRoute(net.routes[index]);
    if (!route || route->size() < 2 || !max_states || max_hops < 0 ||
        route->front().type != Wire::WIRE_TILE_PIN ||
        route->back().type != Wire::WIRE_TILE_PIN || (cancelled && cancelled()))
        return false;
    // Limit only free-source labels; terminal preemption keeps its own budget.
    // Failure diagnostics retain the caller's limit to distinguish budget
    // exhaustion.
    const size_t free_states = !diagnostic_only && fast_free_probe
                                   ? sourceFreeStateLimit(max_states)
                                   : max_states;
    auto &device = Device::current();
    const auto source =
        repairNode(route->front().from, CB_NODE_LOCAL, route->front().local);
    const auto sink =
        repairNode(route->back().from, CB_NODE_LOCAL, route->back().local);
    const char *audit_filter =
        std::getenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY");
    const bool trace =
        diagnostic_log || (audit_filter && *audit_filter &&
                           net.name.find(audit_filter) != std::string::npos);
    const bool trace_frontier =
        trace && (!diagnostic_only || diagnostic_frontier);
    const bool debug_repair =
        std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR") != nullptr;
    std::ofstream search_file;
    std::ostream &search_log = diagnostic_log ? *diagnostic_log : search_file;
    if (trace)
    {
        if (!diagnostic_log)
        {
            const char *path = std::getenv("SCALEPNR_SOURCE_REACHABILITY_LOG");
            search_file.open(path && *path ? path : "source_reachability.log",
                             std::ios::app);
        }

        search_log << "SOURCE_SEARCH begin net=" << net.name
                   << " binding=" << index
                   << " diagnostic_only=" << diagnostic_only
                   << " max_states=" << max_states
                   << " free_states=" << free_states
                   << " cost_scale=" << cost_scale << '\n';
    }

    auto same_source = [&](const rtl::NetRouteBinding &other)
    {
        return rtl::sameRouteSource(other.from, other.from_port, binding.from,
                                    binding.from_port);
    };

    // Source-tile registrations contain the source tree, including aliases.
    // No device-wide scan or persistent search cache is needed.
    std::unordered_set<rtl::Net *> nets{&net};
    auto *source_tile =
        device.getTile(std::get<0>(source), std::get<1>(source));
    if (!source_tile)
        return false;
    for (auto &ref : source_tile->routedNets)
        if (ref.peer)
            nets.insert(ref.peer);
    std::map<RepairNode, std::set<RepairNode>> inputs;
    // An interrupted physical input can have several RTL consumers. Replace
    // those bindings together if a new path changes an incoming arc in their
    // disconnected tails. Never repoint a resource used by a live prefix, a
    // completed route, or a different destination's retained tail.
    std::vector<NetRouteRef> terminal_group;
    std::set<RepairNode> detached, protected_nodes;
    for (auto *n : nets)
        for (size_t bi = 0; bi < n->routes.size(); ++bi)
        {
            auto &b = n->routes[bi];
            if (!same_source(b))
                continue;
            const auto *old = bindingRoute(b);
            if (!old)
                continue;
            size_t last_gap = old->size();
            for (size_t i = 0; i < old->size(); ++i)
                if ((*old)[i].type == Wire::WIRE_GAP)
                    last_gap = i;
            const bool replaceable =
                !unbound && last_gap < old->size() && old->size() >= 2 &&
                old->front().type == Wire::WIRE_TILE_PIN &&
                old->back().type == Wire::WIRE_TILE_PIN &&
                repairNode(old->front().from, CB_NODE_LOCAL,
                           old->front().local) == source &&
                repairNode(old->back().from, CB_NODE_LOCAL,
                           old->back().local) == sink;
            if (replaceable)
                terminal_group.push_back({n, bi});
            for (size_t i = 0; i < old->size(); ++i)
            {
                const auto &w = (*old)[i];
                auto mark = [&](const RepairNode &node)
                {
                    (replaceable && i > last_gap ? detached : protected_nodes)
                        .insert(node);
                };

                if (w.type == Wire::WIRE_ROUTE_EDGE)
                {
                    const auto from =
                        repairNode(w.from, w.from_node_type, w.from_node);
                    const auto to = repairNode(w.to, w.to_node_type, w.to_node);
                    inputs[to].insert(from);
                    mark(from);
                    mark(to);
                }

                else if (w.type == Wire::WIRE_CROSSBAR)
                {
                    auto previous = repairNode(
                        w.from, w.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST,
                        w.local);
                    mark(previous);
                    auto add = [&](RepairNode next)
                    {
                        inputs[next].insert(previous);
                        mark(next);
                        previous = next;
                    };

                    if (w.joint2 >= 0)
                        add(repairNode(w.from, CB_NODE_JOINT, w.joint2));
                    if (w.joint >= 0)
                        add(repairNode(w.from, CB_NODE_JOINT, w.joint));
                    if (w.jump >= 0)
                    {
                        add(repairNode(w.from, CB_NODE_SRC, w.jump));
                        add(repairNode(w.to, CB_NODE_DST, w.dst));
                    }

                    else if (i + 1 < old->size() &&
                             (*old)[i + 1].type == Wire::WIRE_TILE_PIN)
                    {
                        add(repairNode((*old)[i + 1].from, CB_NODE_LOCAL,
                                       (*old)[i + 1].local));
                    }

                }

                else if (w.type == Wire::WIRE_TILE_PIN)
                {
                    mark(repairNode(w.from, CB_NODE_LOCAL, w.local));
                }

            }

        }

    for (const auto &node : protected_nodes)
        detached.erase(node);
    size_t terminal_leaf_rejections = 0;
    // A terminal pin may be hidden behind a short JOINT/LOCAL chain. Prove
    // these dead branches locally instead of spending frontier slots on each
    // intermediary. An unresolved/deeper branch stays eligible; this is not
    // a congestion heuristic and never rejects a potentially useful exit.
    auto terminal_branch = [&](const RepairNode &start)
    {
        std::array<RepairNode, 8> visiting;
        auto continues = [&](auto &&self, const RepairNode &node,
                             size_t depth) -> bool
        {
            if (node == source || node == sink)
                return true;
            for (size_t i = 0; i < depth; ++i)
                if (visiting[i] == node)
                    return false;
            if (depth == visiting.size())
                return true;
            const auto [x, y, kind, id] = node;
            auto *at = device.getTile(x, y);
            if (!at || !at->cb_type || id < 0 || id >= CB_MAX_NODES)
                return false;
            if (kind != CB_NODE_LOCAL && kind != CB_NODE_JOINT)
                return true;
            visiting[depth] = node;
            const auto &cb = *at->cb_type;
            auto follow = [&](const NodeMask &bits, int next_kind)
            {
                return bits.for_each_set_bit(
                    [&](int next)
                    {
                        return self(self, RepairNode{x, y, next_kind, next},
                                    depth + 1);
                    });
            };

            if (kind == CB_NODE_JOINT)
            {
                return cb.joint_src[id].jump.firstSetBit() >= 0 ||
                       follow(cb.joint_joint[id].joint, CB_NODE_JOINT) ||
                       follow(cb.joint_local[id].local, CB_NODE_LOCAL);
            }

            if (cb.local_src[id].jump.firstSetBit() >= 0 ||
                follow(cb.local_joint[id].joint, CB_NODE_JOINT) ||
                follow(cb.local_local[id].local, CB_NODE_LOCAL))
                return true;
            for (const auto &target : device.resolveLocalTargets(*at, id))
                if (target.tile &&
                    self(self,
                         repairNode(target.tile->coord, target.node_type,
                                    target.node),
                         depth + 1))
                    return true;
            return false;
        };

        return !continues(continues, start, 0);
    };

    RouteOwnerQueries owner_queries;
    auto allowed = [&](const RepairNode &from, const RepairNode &to,
                       const std::vector<fpga::RouteCutNode> &released =
                           std::vector<fpga::RouteCutNode>{},
                       bool *proven_unusable = nullptr)
    {
        auto decision =
            [&](bool ok, std::string_view reason, bool proof = false)
        {
            if (proven_unusable)
                *proven_unusable = proof;
            if (auto *trace = RouteSearchTrace::current)
                trace->stream() << "ELIGIBILITY accepted=" << ok << '\n';
            traceRepairDecision("source_eligibility", from, to, reason);
            return ok;
        };

        auto [x, y, kind, id] = to;
        auto *tile = device.getTile(x, y);
        if (!tile || !tile->cb_type || id < 0 || id >= CB_MAX_NODES)
            return decision(false, "invalid_node", true);
        if (kind == CB_NODE_SRC && is_deadend(*tile, id))
            return decision(false, "persistent_src_deadend", true);
        bool busy = repairNodeLeased(tile->cb, kind, id);
        if (kind == CB_NODE_LOCAL)
            busy |= tile->isPinNodeLeased(id);
        // Reject occupied nodes unavailable for retained-route reuse, including
        // the active trace. Cut previews still use the owner checks below.
        if (busy && released.empty() && !protected_nodes.contains(to) &&
            !detached.contains(to))
            return decision(false, "unavailable_lease");
        if ((kind == CB_NODE_LOCAL || kind == CB_NODE_JOINT) && to != sink &&
            to != source)
        {
            // An unrelated terminal pin with no outgoing physical arc cannot
            // complete this connection. Do not let these leaves consume the
            // bounded search frontier. Internal LOCAL bridges and dedicated
            // cross-Tile LOCAL links must remain searchable; this is topology,
            // not a congestion/deadend heuristic or an input-pin name check.
            if (terminal_branch(to))
            {
                // CB templates may require a neighbour type absent at this
                // Tile. Only a resolved physical continuation can make this
                // leaf useful.
                ++terminal_leaf_rejections;
                return decision(false, "terminal_leaf", true);
            }

        }

        if (kind == CB_NODE_JOINT ||
            (kind == CB_NODE_LOCAL && to != sink && to != source))
        {
            const bool terminal_tile =
                sameCoord(tile->coord, repairCoord(sink));
            // This also initializes the placement-owned LOCAL reservations.
            // Protect packed inputs on intermediate tiles, not only at the
            // final sink.
            const auto reserved = packedInputJointReservations(
                *tile, terminal_tile ? binding.to : nullptr,
                terminal_tile ? binding.to_port : std::string{});
            if ((kind == CB_NODE_JOINT && reserved.testBit(id)) ||
                (kind == CB_NODE_LOCAL &&
                 tile->input_local_reservations.count(id)))
                return decision(false, "packed_reservation");
        }

        if (busy)
        {
            auto provisionally_released = [&]
            {
                return std::any_of(
                    released.begin(), released.end(),
                    [&](const auto &cut)
                    {
                        return sameCoord(cut.tile, tile->coord) &&
                               cut.type == kind && cut.node == id;
                    });
            };

            // These two sets already describe every node in the same-source
            // fragments above. An absent busy node cannot be a reusable source
            // resource. Do not scan unrelated owners just to rediscover that
            // fact. Explicit provisional cuts still go through full preemption
            // validation.
            if (!protected_nodes.contains(to) && !detached.contains(to))
                return decision(provisionally_released(),
                                "provisional_cut_check");
            const bool terminal_input =
                to == sink || (kind == CB_NODE_JOINT &&
                               sameCoord(tile->coord, repairCoord(sink)));
            std::vector<NetRouteRef> terminal_owners;
            if (debug_repair)
                std::fprintf(stderr,
                             "REPAIR verify_source_owner=(%d,%d)/%d:%d\n", x, y,
                             kind, id);
            timer.add(RepairCounter::owner_checks);
            if (!owner_queries.allMatch(
                    *tile, static_cast<CBNodeNameType>(kind), id,
                    [&](const auto &o)
                    {
                        if (!same_source(o.net->routes[o.binding_index]))
                            return false;
                        if (terminal_input)
                            terminal_owners.push_back(o);
                        return true;
                    }))

                return decision(provisionally_released(),
                                "owner_mismatch_cut_check");
            const auto old = inputs.find(to);
            if (!preview_relocated_source && old != inputs.end() &&
                (old->second.size() != 1 || !old->second.count(from)))
            {
                // The detached suffix may be rejoined at a SRC as well as at
                // its terminal JOINT/LOCAL. This is provisional: all affected
                // consumers are replaced only after a complete source-to-input
                // path is valid.
                if (!detached.contains(to) &&
                    (unbound || !terminal_input ||
                     !canReplacePrivateTerminalInput(
                         net, index, *tile, static_cast<CBNodeNameType>(kind),
                         id, terminal_owners)))
                    return decision(false, "different_incoming_arc");
            }

        }

        return decision(true, "eligible");
    };

    auto *sink_tile = device.getTile(std::get<0>(sink), std::get<1>(sink));
    if (!sink_tile || !sink_tile->cb_type)
        return false;
    // Destination Moving has not invalidated the moved source tree yet. Its
    // owned resources will be released at commit, including the old sink pin.
    // The preview may reuse/repoint those nodes, but still checks every live
    // owner in allowed(); unrelated routes and candidate input leases stay
    // busy.
    if (unbound &&
        !(preview_relocated_source && protected_nodes.contains(sink)) &&
        (sink_tile->isPinNodeLeased(std::get<3>(sink)) ||
         sink_tile->cb.local.local.testBit(std::get<3>(sink))))
        return false;
    WireNodeSearch node_search{device, source, sink,
                               trace ? &search_log : nullptr};

    timer.next("source_free_search");
    // Only the contiguous tail after the LAST gap can complete this binding.
    // Decode it once, locally to this repair; do not search from detached
    // islands.
    size_t tail_begin = route->size();
    for (size_t i = 0; i < route->size(); ++i)
        if ((*route)[i].type == Wire::WIRE_GAP)
            tail_begin = i;
    std::vector<RepairNode> suffix;
    if (tail_begin < route->size())
    {
        const auto &gap = (*route)[tail_begin];
        suffix.push_back(repairNode(gap.to, gap.to_node_type, gap.to_node));
        bool valid = true;
        auto append = [&](RepairNode from, RepairNode to)
        {
            bool connected = false;
            node_search.successors(from, [&](const RepairNode &next)
                                   { connected |= next == to; });
            if (suffix.back() != from || !connected)
                valid = false;
            suffix.push_back(to);
        };

        for (size_t i = tail_begin + 1; i < route->size() && valid; ++i)
        {
            const auto &w = (*route)[i];
            if (w.type == Wire::WIRE_ROUTE_EDGE)
            {
                append(repairNode(w.from, w.from_node_type, w.from_node),
                       repairNode(w.to, w.to_node_type, w.to_node));
            }

            else if (w.type == Wire::WIRE_CROSSBAR)
            {
                auto previous = repairNode(
                    w.from, w.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST, w.local);
                auto add = [&](RepairNode next)
                {
                    append(previous, next);
                    previous = next;
                };

                if (w.joint2 >= 0)
                    add(repairNode(w.from, CB_NODE_JOINT, w.joint2));
                if (w.joint >= 0)
                    add(repairNode(w.from, CB_NODE_JOINT, w.joint));
                if (w.jump >= 0)
                {
                    add(repairNode(w.from, CB_NODE_SRC, w.jump));
                    add(repairNode(w.to, CB_NODE_DST, w.dst));
                }

                else if (i + 2 == route->size())
                    add(sink);
                else
                    valid = false;
            }

            else if (w.type != Wire::WIRE_TILE_PIN || i + 1 != route->size() ||
                     suffix.back() != sink)
            {
                valid = false;
            }

        }

        if (!valid || suffix.back() != sink)
            suffix.clear();
    }

    using Step = RepairStep;
    (void)incoming_sources;
    bool free_budget_limited = false;
    timer.add(RepairCounter::source_labels);
    std::vector<Step> steps{{source, -1, 0, 0, -1}};
    RepairLabelMap best{{source, 0}};
    int reached = -1;
    int completed_hops = -1;
    std::vector<RepairNode> path;
    size_t expanded = 0, limited = 0, first_limit_expanded = 0,
           first_limit_queue = 0;
    size_t eligibility_checks = 0, dominated = 0;
    std::unordered_set<int> traced_expanded;

    bool stopped = false;
    int deadend_dump_index = -1;
    // Every call owns one node lease and one local candidate cursor. A busy
    // candidate is rejected by allowed(); no ancestor traversal is needed.
    auto next_node = [&](auto &&self, int current, int goal, size_t depth,
                         bool &unproven) -> bool
    {
        if (cancelled && cancelled())
        {
            stopped = true;
            return false;
        }

        const Step step = steps[current];
        traceRepairDecision("source_pop", step.node, sink, "expand");
        if (best.at(step.node) != current)
        {
            timer.add(RepairCounter::stale_source_labels);
            unproven = true;
            return false;
        }

        if (!pnr::InputRepairBudget::expand())
        {
            stopped = true;
            return false;
        }

        auto &tile =
            *device.getTile(std::get<0>(step.node), std::get<1>(step.node));
        RepairNodeLease lease(tile, step.node, protected_nodes, detached);
        timer.expand(step.node);
        ++expanded;
        if (trace_frontier)
            traced_expanded.insert(current);
        if (step.node == sink)
        {
            reached = current;
            completed_hops = step.hops;
            return true;
        }

        const auto join = std::find(suffix.begin(), suffix.end(), step.node);
        if (join != suffix.end())
        {
            // Validate the retained continuation with the same live leases.
            // Reconstruct the prefix only after the entire suffix succeeds.
            std::deque<RepairNodeLease> suffix_leases;
            RepairNode previous = step.node;
            int hops = step.hops;
            bool valid = true;
            for (auto next = join + 1; next != suffix.end(); ++next)
            {
                if (cancelled && cancelled())
                {
                    stopped = true;
                    return false;
                }

                bool proven_unusable = false;
                if (!allowed(previous, *next, {}, &proven_unusable))
                {
                    unproven |= !proven_unusable;
                    valid = false;
                    break;
                }

                hops += !sameCoord(repairCoord(previous), repairCoord(*next));
                if (hops > max_hops)
                {
                    unproven = true;
                    valid = false;
                    break;
                }

                auto &next_tile =
                    *device.getTile(std::get<0>(*next), std::get<1>(*next));
                suffix_leases.emplace_back(next_tile, *next, protected_nodes,
                                           detached);
                previous = *next;
            }

            if (valid)
            {
                for (int p = current; p >= 0; p = steps[p].parent)
                    path.push_back(steps[p].node);
                std::reverse(path.begin(), path.end());
                path.insert(path.end(), join + 1, suffix.end());
                reached = current;
                completed_hops = hops;
                if (trace)
                {
                    const auto [x, y, kind, id] = step.node;
                    search_log << "SOURCE_SEARCH_SUFFIX tile=" << x << ',' << y
                               << " kind=" << kind << " node=" << id
                               << " prefix_hops=" << step.hops
                               << " suffix_hops=" << hops - step.hops
                               << " total_hops=" << hops
                               << " states=" << steps.size() << '\n';
                }

                return true;
            }

        }

        if (!trace_frontier && free_budget_limited &&
            steps.size() >= free_states)
        {
            unproven = true;
            return false;
        }

        const auto edges = node_search.orderedSuccessors(step.node, goal);
        for (size_t cursor = 0; cursor < edges.size(); ++cursor)
        {
            if (cancelled && cancelled())
            {
                stopped = true;
                return false;
            }

            const auto [next, next_goal] = edges[cursor];
            timer.add(RepairCounter::source_edges);
            const bool jump =
                !sameCoord(repairCoord(step.node), repairCoord(next));
            const int hops = step.hops + jump;
            if (existing_takeoff_only && step.hops == 0 &&
                std::get<2>(next) == CB_NODE_SRC &&
                !protected_nodes.contains(next))
            {
                unproven = true;
                traceRepairDecision("source_edge", step.node, next,
                                    "new_takeoff_forbidden");
                continue;
            }

            if (hops > max_hops)
            {
                unproven = true;
                traceRepairDecision("source_edge", step.node, next,
                                    "hop_limit");
                continue;
            }

            const int cost = step.cost + 1 + 4 * jump;
            const auto found = best.find(next);
            const bool redundant = next_goal < 0 && found != best.end() &&
                                   steps[found->second].cost <= cost &&
                                   steps[found->second].hops <= hops;
            if (redundant && !trace_frontier)
            {
                unproven = true;
                timer.add(RepairCounter::source_seen);
                ++dominated;
                traceRepairDecision("source_edge", step.node, next,
                                    "dominated");
                continue;
            }

            ++eligibility_checks;
            bool proven_unusable = false;
            if (!allowed(step.node, next, {}, &proven_unusable))
            {
                unproven |= !proven_unusable;
                continue;
            }

            if (trace_frontier)
            {
                const auto [nx, ny, nkind, nid] = next;
                search_log << "SOURCE_SEARCH_EDGE from=" << current
                           << " tile=" << nx << ',' << ny << " kind=" << nkind
                           << " node=" << nid << '\n';
            }

            if (redundant)
            {
                unproven = true;
                timer.add(RepairCounter::source_seen);
                ++dominated;
                traceRepairDecision("source_edge", step.node, next,
                                    "dominated");
                continue;
            }

            if (steps.size() >= free_states)
            {
                timer.add(RepairCounter::source_limit);
                free_budget_limited = true;
                unproven = true;
                if (budget_limited)
                    *budget_limited = true;
                traceRepairDecision("source_edge", step.node, next,
                                    "state_limit");
                if (limited++ == 0)
                {
                    first_limit_expanded = expanded;
                    first_limit_queue = depth;
                }

                if (trace_frontier)
                {
                    const auto [nx, ny, nkind, nid] = next;
                    search_log << "SOURCE_SEARCH_LIMIT from=" << current
                               << " tile=" << nx << ',' << ny
                               << " kind=" << nkind << " node=" << nid
                               << " cost=" << cost << " hops=" << hops << '\n';
                }

                continue;
            }

            const int n = steps.size();
            best[next] = n;
            steps.push_back({next, current, hops, cost, next_goal});
            timer.add(RepairCounter::source_labels);
            traceRepairDecision("source_edge", step.node, next, "enqueue");
            bool child_unproven = false;
            if (self(self, n, next_goal, depth + 1, child_unproven))
                return true;
            if (stopped)
                return false;
            unproven |= child_unproven;
        }

        if (!diagnostic_only && max_hops > 0 && !unproven &&
            std::get<2>(step.node) == CB_NODE_SRC)
        {
            if (trace)
                search_log << "SOURCE_DEADEND_MARK index=" << current
                           << " tile=" << tile.coord.x << ',' << tile.coord.y
                           << " src=" << std::get<3>(step.node)
                           << " hops=" << step.hops << " goal=" << step.goal
                           << '\n';
            const char *tile_filter =
                std::getenv("SCALEPNR_DEBUG_SOURCE_TILE_STATE");
            int filter_x = 0, filter_y = 0;
            if (trace && deadend_dump_index < 0 && tile_filter &&
                (std::sscanf(tile_filter, "%d,%d", &filter_x, &filter_y) != 2 ||
                 (tile.coord.x == filter_x && tile.coord.y == filter_y)))
                deadend_dump_index = current;
            mark_deadend(tile, std::get<3>(step.node));
        }

        return false;
    };

    bool unproven = false;
    next_node(next_node, 0, -1, 1, unproven);
    if (stopped)
        return false;
    // Audit registered ownership after the recursive leases have unwound.
    // A live trace's own occupation must not be reported as orphan routing.
    if (deadend_dump_index >= 0)
    {
        const auto &node = steps[deadend_dump_index].node;
        auto &tile = *device.getTile(std::get<0>(node), std::get<1>(node));
        std::vector<rtl::Net *> owners{&net};
        for (const auto &ref : tile.routedNets)
            if (ref.peer && std::find(owners.begin(), owners.end(), ref.peer) ==
                                owners.end())
                owners.push_back(ref.peer);
        std::ostringstream state;
        const auto audit = auditTileCongestion(tile, state, owners);
        search_log << "SOURCE_DEADEND_TILE_STATE index=" << deadend_dump_index
                   << " phase=after_trace owner_nets=" << owners.size() << '\n'
                   << state.str()
                   << "SOURCE_DEADEND_TILE_AUDIT orphan=" << audit.orphan_leases
                   << " missing=" << audit.missing_leases << '\n';
    }

    if (std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR"))
        std::fprintf(stderr,
                     "REPAIR source states=%zu limit=%zu reached=%d hops=%d "
                     "eligibility_checks=%zu parent_checks=%zu dominated=%zu\n",
                     steps.size(), free_states, reached >= 0, completed_hops,
                     eligibility_checks, size_t{0}, dominated);
    if (trace)
    {
        search_log << "SOURCE_SEARCH end states=" << steps.size()
                   << " unique=" << best.size() << " expanded=" << expanded
                   << " budget_rejections=" << limited
                   << " first_limit_expanded=" << first_limit_expanded
                   << " first_limit_queue=" << first_limit_queue
                   << " reached=" << (reached >= 0)
                   << " terminal_leaf_rejections=" << terminal_leaf_rejections
                   << '\n';
        for (size_t i = 0; i < steps.size() && trace_frontier; ++i)
        {
            const auto &s = steps[i];
            const auto [x, y, kind, id] = s.node;
            search_log << "SOURCE_SEARCH_NODE index=" << i
                       << " parent=" << s.parent << " tile=" << x << ',' << y
                       << " kind=" << kind << " node=" << id
                       << " cost=" << s.cost << " hops=" << s.hops
                       << " goal=" << s.goal
                       << " expanded=" << traced_expanded.count(i)
                       << " best=" << (best.at(s.node) == int(i)) << '\n';
        }

        search_log.flush();
    }

    std::vector<fpga::RouteCutNode> grounding_cuts, released;
    timer.next("source_short_completion");
    if (reached >= 0 && path.empty())
    {
        for (int p = reached; p >= 0; p = steps[p].parent)
            path.push_back(steps[p].node);
        std::reverse(path.begin(), path.end());
    }

    else if (reached < 0 && grounding_preemption)
    {
        // The free search is finished. Reuse its reached states, without
        // another fabric search. Only the final Grounding may preempt transit,
        // including its bounded multi-Tile approach. Never authorize arbitrary
        // remote transit cuts simply because the source search reached that
        // position.
        auto try_terminal = [&](int current, std::vector<RepairNode> tail)
        {
            if (cancelled && cancelled())
                return false;
            std::vector<RepairNode> candidate;
            for (int p = current; p >= 0; p = steps[p].parent)
                candidate.push_back(steps[p].node);
            std::reverse(candidate.begin(), candidate.end());
            pnr::DockingGroundingCandidate request{
                .prefix = candidate,
                .tail = tail,
                .prefix_hops = steps[current].hops,
                .max_hops = max_hops,
                .required_branch =
                    required_branch
                        ? std::optional<RepairNode>{repairNode(
                              required_branch->tile, required_branch->type,
                              required_branch->node)}
                        : std::nullopt,
                .allowed = [&](const RepairNode &from, const RepairNode &to,
                               const std::vector<fpga::RouteCutNode> &cuts)
                { return allowed(from, to, cuts); },
                .preview = [&](const std::vector<fpga::RouteCutNode> &cuts,
                               std::vector<fpga::RouteCutNode> &preview)
                { return grounding_preemption(cuts, false, preview); }};
            auto checked = pnr::tryDockingGroundingCandidate(request);
            if (!checked.success)
                return false;
            candidate = std::move(checked.path);
            auto &cuts = checked.cuts;
            auto &preview = checked.released;
            if (trace)
            {
                search_log << "SOURCE_GROUNDING_WITNESS current=" << current
                           << " hops=" << checked.hops
                           << " cuts=" << cuts.size() << '\n';
                for (const auto &node : candidate)
                {
                    const auto [x, y, kind, id] = node;
                    search_log << "SOURCE_GROUNDING_NODE tile=" << x << ',' << y
                               << " kind=" << kind << " node=" << id << '\n';
                }

                for (const auto &cut : cuts)
                    search_log << "SOURCE_GROUNDING_CUT tile=" << cut.tile.x
                               << ',' << cut.tile.y << " kind=" << int(cut.type)
                               << " node=" << cut.node << '\n';
                std::unordered_set<rtl::Net *> logged;
                for (const auto &owner : findNetRouteCutVictims(cuts))
                    if (logged.insert(owner.net).second)
                        dumpNetRouteHistory(*owner.net, search_log);
                search_log.flush();
            }

            path = std::move(candidate);
            grounding_cuts = std::move(cuts);
            released = std::move(preview);
            return true;
        };

        const auto &cb = *sink_tile->cb_type;
        const int local = std::get<3>(sink);
        auto terminal =
            [&](int current, int dst, bool append_dst,
                std::vector<RepairNode> tail = std::vector<RepairNode>{})
        {
            if (append_dst)
                tail.push_back(repairNode(sink_tile->coord, CB_NODE_DST, dst));
            auto finish = [&](int first = -1, int second = -1)
            {
                auto complete = tail;
                if (first >= 0)
                    complete.push_back(
                        repairNode(sink_tile->coord, CB_NODE_JOINT, first));
                if (second >= 0)
                    complete.push_back(
                        repairNode(sink_tile->coord, CB_NODE_JOINT, second));
                complete.push_back(sink);
                return try_terminal(current, std::move(complete));
            };

            if (cb.dst_local[dst].local.testBit(local) && finish())
                return true;
            return cb.dst_joint[dst].joint.for_each_set_bit(
                [&](int j)
                {
                    if (cb.joint_local[j].local.testBit(local) && finish(j))
                        return true;
                    return cb.joint_joint[j].joint.for_each_set_bit(
                        [&](int k)
                        {
                            return k != j &&
                                   cb.joint_local[k].local.testBit(local) &&
                                   finish(j, k);
                        });
                });
        };

        auto in_grounding_window = [&](Coord at)
        {
            return std::abs(at.x - sink_tile->coord.x) <= GROUNDING_RADIUS &&
                   std::abs(at.y - sink_tile->coord.y) <= GROUNDING_RADIUS;
        };

        std::vector<int> grounding_entries;
        for (size_t i = 0; i < steps.size() && path.empty(); ++i)
        {
            if (cancelled && cancelled())
                return false;
            const auto [x, y, kind, id] = steps[i].node;
            if (best.at(steps[i].node) != int(i))
                continue;
            if (kind == CB_NODE_DST && sameCoord({x, y}, sink_tile->coord))
                terminal(i, id, false);
            if (kind != CB_NODE_SRC)
                continue;
            auto *tile = device.getTile(x, y);
            const bool outside = !in_grounding_window(tile->coord);
            bool enters_window = false;
            for (const auto &target : device.resolveJumpTargets(*tile, id))
            {
                // The free SRC is the boundary, not a resource to preempt. Its
                // landing may enter Grounding from outside the window. Reuse
                // this existing jump scan instead of resolving the frontier
                // again later.
                if (outside && target.tile &&
                    in_grounding_window(target.tile->coord))
                    enters_window = true;
                if (target.tile == sink_tile && target.dst_node >= 0 &&
                    terminal(i, target.dst_node, true))
                    break;
            }

            if (enters_window)
                grounding_entries.push_back(int(i));
        }

        // Preserve the existing free-SRC/terminal choices first. A foreign SRC
        // never entered the free search, even when its jump is the final
        // approach to this sink. Extend reached DSTs across that one physical
        // jump here; try_terminal validates the complete path and previews all
        // owners before any lease changes. Takeoff/Grounding victims remain
        // protected by the same callback as every other terminal preemption.
        for (size_t i = 0; i < steps.size() && path.empty(); ++i)
        {
            if (cancelled && cancelled())
                return false;
            const auto [x, y, kind, id] = steps[i].node;
            if (kind != CB_NODE_DST || best.at(steps[i].node) != int(i))
                continue;
            auto *tile = device.getTile(x, y);
            const auto &type = *tile->cb_type;
            type.dst_src[id].jump.for_each_set_bit(
                [&](int src)
                {
                    // Free SRCs were handled by the free search and the
                    // preceding scan.
                    if (!tile->cb.src.jump.testBit(src))
                        return false;
                    for (const auto &target :
                         device.resolveJumpTargets(*tile, src))
                    {
                        if (target.tile == sink_tile && target.dst_node >= 0 &&
                            terminal(
                                i, target.dst_node, true,
                                {repairNode(tile->coord, CB_NODE_SRC, src)}))
                            return true;
                    }

                    return false;
                });
        }

        // A reached DST can be separated from the sink by TWO occupied transit
        // jumps, both inside the Grounding window. The free search cannot enter
        // the first SRC, so the one-jump scan above never sees the second one.
        // Enumerate these short approaches only after the existing choices
        // fail; validate the WHOLE prefix/tail and preview all cuts before any
        // mutation. This is a local completion, not permission to preempt
        // remote transit.
        if (path.empty())
        {
            timer.next("source_full_completion");
            bool found = false;
            auto nearby = [&](Coord c)
            {
                return std::abs(c.x - sink_tile->coord.x) <= GROUNDING_RADIUS &&
                       std::abs(c.y - sink_tile->coord.y) <= GROUNDING_RADIUS;
            };

            for (size_t i = 0; i < steps.size() && !found; ++i)
            {
                if (cancelled && cancelled())
                    break;
                const auto [x, y, kind, id] = steps[i].node;
                if (kind != CB_NODE_DST || best.at(steps[i].node) != int(i) ||
                    !nearby({x, y}))
                    continue;
                auto *tile = device.getTile(x, y);
                const auto &type = *tile->cb_type;
                found = type.dst_src[id].jump.for_each_set_bit(
                    [&](int src)
                    {
                        for (const auto &middle :
                             device.resolveJumpTargets(*tile, src))
                        {
                            if (!middle.tile || middle.dst_node < 0 ||
                                !middle.tile->cb_type ||
                                !nearby(middle.tile->coord))
                                continue;
                            const auto &mid_type = *middle.tile->cb_type;
                            if (mid_type.dst_src[middle.dst_node]
                                    .jump.for_each_set_bit(
                                        [&](int last)
                                        {
                                            for (const auto &target :
                                                 device.resolveJumpTargets(
                                                     *middle.tile, last))
                                                if (target.tile == sink_tile &&
                                                    target.dst_node >= 0 &&
                                                    terminal(
                                                        i, target.dst_node,
                                                        true,
                                                        {repairNode(tile->coord,
                                                                    CB_NODE_SRC,
                                                                    src),
                                                         repairNode(
                                                             middle.tile->coord,
                                                             CB_NODE_DST,
                                                             middle.dst_node),
                                                         repairNode(
                                                             middle.tile->coord,
                                                             CB_NODE_SRC,
                                                             last)}))
                                                    return true;
                                            return false;
                                        }))

                                return true;
                        }

                        return false;
                    });
            }

            if (trace)
            {
                search_log << "SOURCE_TWO_HOP_RESULT found=" << found << '\n';
                search_log.flush();
            }

        }

        // The short completions above cannot cross an occupied Takeoff exit or
        // an arbitrary chain of JOINTs. Continue numerically from reached
        // states within the Grounding window, at most twice its radius in Tile
        // hops. Only preview transit cuts during search; try_terminal validates
        // the entire connected prefix/completion and the combined cuts before
        // commit.
        if (path.empty())
        {
            timer.next("source_full_completion");
            std::vector<bool> through_branch;
            if (required_branch)
            {
                through_branch.resize(steps.size());
                const auto required =
                    repairNode(required_branch->tile, required_branch->type,
                               required_branch->node);
                for (size_t i = 0; i < steps.size(); ++i)
                    through_branch[i] = steps[i].node == required ||
                                        (steps[i].parent >= 0 &&
                                         through_branch[steps[i].parent]);
            }

            std::vector<pnr::DockingGroundingSeed> seeds;
            auto seed = [&](size_t i)
            {
                if (required_branch && !through_branch[i])
                    return;
                seeds.push_back({steps[i].node, int(i), steps[i].hops});
            };

            for (size_t i = 0; i < steps.size(); ++i)
            {
                const auto [x, y, kind, id] = steps[i].node;
                if (best.at(steps[i].node) != int(i) ||
                    std::abs(x - sink_tile->coord.x) > GROUNDING_RADIUS ||
                    std::abs(y - sink_tile->coord.y) > GROUNDING_RADIUS)
                    continue;
                seed(i);
            }

            for (int i : grounding_entries)
                seed(i);
            pnr::DockingGroundingSearch request{
                .sink = sink,
                .center = sink_tile->coord,
                .radius = GROUNDING_RADIUS,
                .max_hops = max_hops,
                .max_states = max_states,
                .seeds = seeds,
                .successors =
                    [&](const RepairNode &at,
                        const std::function<void(const RepairNode &)> &visit)
                { node_search.successors(at, visit); },
                .free_edge = [&](const RepairNode &from, const RepairNode &to)
                { return allowed(from, to); },
                .cut_edge =
                    [&](const RepairNode &from, const RepairNode &to)
                {
                    const auto [x, y, kind, id] = to;
                    std::vector<fpga::RouteCutNode> cut{
                        {{x, y}, static_cast<CBNodeNameType>(kind), id}};
                    if (!allowed(from, to, cut))
                        return false;
                    std::vector<fpga::RouteCutNode> preview;
                    const bool eligible =
                        grounding_preemption(cut, false, preview);
                    if (trace)
                        search_log << "SOURCE_PREEMPT_NODE tile=" << x << ','
                                   << y << " kind=" << kind << " node=" << id
                                   << " eligible=" << eligible << '\n';
                    return eligible;
                },

                .complete = [&](int root, std::vector<RepairNode> tail)
                { return try_terminal(root, std::move(tail)); },
                .stopped = [&] { return cancelled && cancelled(); }};
            const auto completion = pnr::searchGroundingFromFrontier(request);
            if (completion.stopped)
                return false;
            if (completion.budget_limited && budget_limited)
                *budget_limited = true;
            if (trace)
            {
                search_log << "SOURCE_GROUNDING_SEARCH states="
                           << completion.states
                           << " expanded=" << completion.expanded
                           << " limited=" << completion.budget_limited
                           << " rejected_complete="
                           << completion.rejected_complete
                           << " found=" << completion.found << '\n';
                search_log.flush();
            }

        }

    }

    // A retained source prefix can be trapped OUTSIDE the Grounding window.
    // After free departures and local completion fail, allow a new Takeoff:
    // preview transit cuts only before/at its first landing, then require free
    // transit until bounded Grounding. The common commit below still validates
    // the complete source-to-input path before changing any victim leases.
    if (path.empty() && grounding_preemption && !required_branch &&
        !existing_takeoff_only && cost_scale == 1)
    {
        timer.next("source_takeoff_completion");

        struct ProbeStep
        {
            RepairNode node;
            int parent, hops, cost, grounding_start;
        };

        std::vector<ProbeStep> probe{{source, -1, 0, 0, -1}};
        RepairLabelMap best_probe{{source, 0}};
        bool found = false, limited = false;
        auto nearby = [&](const RepairNode &n)
        {
            return std::abs(std::get<0>(n) - std::get<0>(sink)) <=
                       GROUNDING_RADIUS &&
                   std::abs(std::get<1>(n) - std::get<1>(sink)) <=
                       GROUNDING_RADIUS;
        };

        std::vector<fpga::RouteCutNode> takeoff_cuts;
        auto next_takeoff = [&](auto &&self, int current, int goal,
                                bool cut_node) -> bool
        {
            if (cancelled && cancelled())
            {
                stopped = true;
                return false;
            }

            const auto step = probe[current];
            if (best_probe.at(step.node) != current)
                return false;
            if (!pnr::InputRepairBudget::expand())
            {
                stopped = true;
                return false;
            }

            auto &tile =
                *device.getTile(std::get<0>(step.node), std::get<1>(step.node));
            RepairNodeLease lease(tile, step.node, protected_nodes, detached,
                                  cut_node);
            timer.add(RepairCounter::source_probe_expanded);
            if (step.node == sink)
            {
                std::vector<fpga::RouteCutNode> preview;
                // Every edge passed allowed() before this call leased it.
                // Check the complete cut set once; final path validation runs
                // below after the recursive leases have unwound.
                if (!takeoff_cuts.empty() &&
                    !grounding_preemption(takeoff_cuts, false, preview))
                    return false;
                for (int p = current; p >= 0; p = probe[p].parent)
                    path.push_back(probe[p].node);
                std::reverse(path.begin(), path.end());
                grounding_cuts = std::move(takeoff_cuts);
                released = std::move(preview);
                if (trace)
                {
                    search_log << "SOURCE_TAKEOFF_WITNESS hops=" << step.hops
                               << " cuts=" << grounding_cuts.size() << '\n';
                    for (const auto &n : path)
                    {
                        const auto [x, y, kind, id] = n;
                        search_log << "SOURCE_TAKEOFF_NODE tile=" << x << ','
                                   << y << " kind=" << kind << " node=" << id
                                   << '\n';
                    }

                    for (const auto &cut : grounding_cuts)
                        search_log << "SOURCE_TAKEOFF_CUT tile=" << cut.tile.x
                                   << ',' << cut.tile.y
                                   << " kind=" << int(cut.type)
                                   << " node=" << cut.node << '\n';
                    std::unordered_set<rtl::Net *> logged;
                    for (const auto &owner :
                         findNetRouteCutVictims(grounding_cuts))
                        if (logged.insert(owner.net).second)
                            dumpNetRouteHistory(*owner.net, search_log);
                }

                return true;
            }

            const auto edges = node_search.orderedSuccessors(step.node, goal);
            for (size_t cursor = 0; cursor < edges.size(); ++cursor)
            {
                if (cancelled && cancelled())
                {
                    stopped = true;
                    return false;
                }

                const auto [next, next_goal] = edges[cursor];
                timer.add(RepairCounter::source_probe_edges);
                const bool jump =
                    !sameCoord(repairCoord(step.node), repairCoord(next));
                const int hops = step.hops + jump;
                if (hops > max_hops)
                {
                    limited = true;
                    continue;
                }

                const int base_cost = step.cost + 1 + 4 * jump;
                const auto old = best_probe.find(next);
                if (next_goal < 0 && next != sink && old != best_probe.end() &&
                    probe[old->second].cost <= base_cost)
                    continue;
                int grounding_start = step.grounding_start;
                if (grounding_start >= 0 &&
                    (!nearby(next) ||
                     hops - grounding_start > 2 * GROUNDING_RADIUS))
                    continue;
                const bool cut = !allowed(step.node, next);
                const int cost = base_cost + 20 * cut;
                if (next_goal < 0 && next != sink && old != best_probe.end() &&
                    probe[old->second].cost <= cost)
                    continue;
                const auto [x, y, kind, id] = next;
                if (cut)
                {
                    // A node reserved by this attempt cannot be preempted
                    // again, nor can any protected source-tree resource.
                    if (protected_nodes.contains(next))
                        continue;
                    if (kind != CB_NODE_SRC && kind != CB_NODE_DST &&
                        kind != CB_NODE_JOINT)
                        continue;
                    const bool takeoff =
                        step.hops == 0 &&
                        sameCoord(repairCoord(step.node), repairCoord(source));
                    if (!takeoff && !nearby(next))
                        continue;
                    if (!takeoff && grounding_start < 0)
                        grounding_start = step.hops;
                    std::vector<fpga::RouteCutNode> cuts{
                        {{x, y}, static_cast<CBNodeNameType>(kind), id}},
                        preview;
                    const bool eligible =
                        allowed(step.node, next, cuts) &&
                        grounding_preemption(cuts, false, preview);
                    if (trace && takeoff)
                        search_log << "SOURCE_TAKEOFF_EXIT tile=" << x << ','
                                   << y << " kind=" << kind << " node=" << id
                                   << " eligible=" << eligible << '\n';
                    if (!eligible)
                        continue;
                }

                if (probe.size() >= max_states)
                {
                    limited = true;
                    if (budget_limited)
                        *budget_limited = true;
                    continue;
                }

                if (cut)
                    takeoff_cuts.push_back(
                        {{x, y}, static_cast<CBNodeNameType>(kind), id});
                const int n = probe.size();
                best_probe[next] = n;
                probe.push_back({next, current, hops, cost, grounding_start});
                if (self(self, n, next_goal, cut))
                    return true;
                if (cut)
                    takeoff_cuts.pop_back();
                if (stopped)
                    return false;
            }

            return false;
        };

        found = next_takeoff(next_takeoff, 0, -1, false);
        if (stopped)
            return false;
        if (trace)
        {
            search_log << "SOURCE_TAKEOFF_SEARCH states=" << probe.size()
                       << " limited=" << limited << " found=" << found << '\n';
            search_log.flush();
        }

    }

    if (path.empty() || (cancelled && cancelled()))
        return false;
    // An opt-in failed-task probe uses exactly the production owner policy and
    // full replacement validation, but must stop before cutting or leasing.
    if (diagnostic_only)
    {
        if (trace)
        {
            search_log << "SOURCE_DIAGNOSTIC_COMPLETE nodes=" << path.size()
                       << " cuts=" << grounding_cuts.size()
                       << " cost_scale=" << cost_scale
                       << " max_states=" << max_states << '\n';
            for (const auto &n : path)
            {
                const auto [x, y, kind, id] = n;
                search_log << "SOURCE_DIAGNOSTIC_NODE tile=" << x << ',' << y
                           << " kind=" << kind << " node=" << id << '\n';
            }

        }

        return true;
    }

    timer.next("source_commit");
    // Compress ordinary hops into the existing representation so branch
    // discovery and preemption retain their normal transit/Takeoff distinction.
    // Unusual intra-CB continuations use existing explicit numeric graph edges.
    std::vector<Wire> replacement{route->front()};
    for (size_t i = 0; i + 1 < path.size();)
    {
        size_t end = i + 1;
        std::vector<int> joints;
        while (end < path.size() &&
               sameCoord(repairCoord(path[end]), repairCoord(path[i])) &&
               std::get<2>(path[end]) == CB_NODE_JOINT)
        {
            joints.push_back(std::get<3>(path[end++]));
        }

        const bool input = std::get<2>(path[i]) == CB_NODE_DST ||
                           (i == 0 && path[i] == source);
        const bool hop =
            end + 1 < path.size() && std::get<2>(path[end]) == CB_NODE_SRC &&
            sameCoord(repairCoord(path[end]), repairCoord(path[i])) &&
            std::get<2>(path[end + 1]) == CB_NODE_DST;
        const bool terminal =
            end + 1 == path.size() && path[end] == sink &&
            sameCoord(repairCoord(path[end]), repairCoord(path[i]));
        if (input && joints.size() <= 2 && (hop || terminal))
        {
            Wire w;
            w.from = repairCoord(path[i]);
            w.local = std::get<3>(path[i]);
            w.pos = i == 0 ? 0 : 1;
            if (!joints.empty())
                w.joint = joints.back();
            if (joints.size() == 2)
                w.joint2 = joints.front();
            if (hop)
            {
                w.jump = w.route_jump = std::get<3>(path[end]);
                w.dst = std::get<3>(path[end + 1]);
                w.to = repairCoord(path[end + 1]);
                w.owns_landing = true;
                i = end + 1;
            }

            else
            {
                w.to = w.from;
                i = end;
            }

            w.net_name = binding.route_name;
            replacement.push_back(w);
        }

        else
        {
            Wire w;
            w.type = Wire::WIRE_ROUTE_EDGE;
            w.from = repairCoord(path[i]);
            w.to = repairCoord(path[i + 1]);
            w.from_node_type = std::get<2>(path[i]);
            w.from_node = std::get<3>(path[i]);
            w.to_node_type = std::get<2>(path[i + 1]);
            w.to_node = std::get<3>(path[i + 1]);
            w.net_name = binding.route_name;
            replacement.push_back(w);
            ++i;
        }

    }

    replacement.push_back(route->back());
    auto lease_path = [&]
    {
        for (const auto &n : path)
        {
            auto [x, y, kind, id] = n;
            auto &cb = device.getTile(x, y)->cb;
            switch (kind)
            {
            case CB_NODE_LOCAL:
                if (n != source)
                    cb.local.local.setBit(id);
                break;
            case CB_NODE_JOINT:
                cb.joint.jump.setBit(id);
                break;
            case CB_NODE_SRC:
                cb.src.jump.setBit(id);
                break;
            case CB_NODE_DST:
                cb.dst.jump.setBit(id);
                break;
            }

        }

    };

    if (unbound)
    {
        // A new task uses the same bounded numeric search, without
        // manufacturing an incomplete registered route. The complete projected
        // path must pass before any transit cuts, just as for a retained-route
        // repair. There is no detached tail to repoint, and no mutation on a
        // failed preview.
        size_t storage = binding.to->wires.size();
        const auto existing =
            net.findRouteBinding(binding.from, binding.to, binding.from_port,
                                 binding.to_port, binding.route_name);
        if (existing.index < net.routes.size())
        {
            auto &old = net.routes[existing.index];
            const auto *slot = bindingRoute(old);
            const bool uninstalled =
                !old.owner &&
                old.route_index == std::numeric_limits<size_t>::max();
            if (slot ? !slot->empty() : !uninstalled)
                return false;
            // Unrouting keeps the logical binding. Reuse its empty physical
            // slot when owned by this sink, rather than accumulating abandoned
            // vectors.
            if (old.owner == binding.to)
                storage = old.route_index;
        }

        for (size_t i = 1; i < path.size(); ++i)
            if (!allowed(path[i - 1], path[i], released))
                return false;
        if (cancelled && cancelled())
            return false;
        if (!grounding_cuts.empty() &&
            !grounding_preemption(grounding_cuts, true, released))
            return false;
        RouteHistoryScope history(&net, __func__);
        for (auto &w : replacement)
            w.net_name = binding.route_name;
        replacement.front().shared = true;
        if (storage == binding.to->wires.size())
            binding.to->wires.push_back(std::move(replacement));
        else
            binding.to->wires[storage] = std::move(replacement);
        lease_path();
        sink_tile->pin_state.leased_nodes.setBit(std::get<3>(sink));
        const size_t installed = attachNetRoute(
            net, *binding.to, storage, binding.from, binding.to,
            binding.from_port, binding.to_port, binding.route_name);
        registerNetRouteTiles(net, binding.to->wires[storage], installed);
        timer.success = true;
        return true;
    }

    bool rejoined = false;
    for (size_t i = 1; i < path.size(); ++i)
    {
        const auto old = inputs.find(path[i]);
        rejoined |=
            detached.contains(path[i]) && old != inputs.end() &&
            (old->second.size() != 1 || !old->second.count(path[i - 1]));
    }

    if (!rejoined)
        terminal_group = {{&net, index}};

    struct Edit
    {
        rtl::Net *net;
        size_t index;
        std::vector<Wire> *route;
        std::vector<Wire> next;
    };

    std::vector<Edit> edits;
    std::unordered_set<const rtl::NetRouteBinding *> excluded;
    for (const auto &ref : terminal_group)
    {
        auto &b = ref.net->routes[ref.binding_index];
        auto *old = bindingRoute(b);
        if (!old || old->empty())
            return false;
        excluded.insert(&b);
        // Aliases can register the same stored vector more than once.
        if (std::any_of(edits.begin(), edits.end(),
                        [&](const auto &e) { return e.route == old; }))
            continue;
        auto next = replacement;
        // Keep source ownership only when the replacement uses that same pin.
        // A prefix repair may select another mapped output LOCAL; copying the
        // old endpoint then records a disconnected route and leases the wrong
        // pin.
        if (sameCoord(old->front().from, route->front().from) &&
            old->front().local == route->front().local)
            next.front() = old->front();
        else
            next.front().shared = false;
        // A relocated input can retain only a source prefix, with no old sink
        // endpoint. Keep the newly resolved input in that case, not its last
        // hop.
        if (old->size() > 1 && old->back().type == Wire::WIRE_TILE_PIN)
            next.back() = old->back();
        for (auto &w : next)
        {
            w.net_name = b.route_name;
            if (!edits.empty())
                w.shared = true;
        }

        edits.push_back({ref.net, ref.binding_index, old, std::move(next)});
    }

    if (edits.empty())
        return false;
    if (rejoined)
    {
        // Keep exactly one owner for the common input even when an RTL alias
        // happens to be enumerated first. Preserve borrowed source pins: taking
        // ownership of one would spuriously promote siblings on a later repair.
        edits.front().next.back().shared = false;
        edits.front().next.front().shared = std::all_of(
            edits.begin(), edits.end(),
            [&](const auto &e)
            {
                return e.route->front().shared &&
                       sameCoord(e.route->front().from, route->front().from) &&
                       e.route->front().local == route->front().local;
            });
    }

    // All validation precedes mutation. In particular, no incomplete wandering
    // suffix is installed. A terminal transit cut is applied only after the
    // complete replacement passes projected occupancy and packed-input checks.
    for (size_t i = 1; i < path.size(); ++i)
        if (!allowed(path[i - 1], path[i], released))
            return false;
    if (cancelled && cancelled())
        return false;
    if (!grounding_cuts.empty() &&
        !grounding_preemption(grounding_cuts, true, released))
        return false;
    RouteHistoryScope history(&net, __func__);
    for (const auto &e : edits)
        promoteSurvivingSourcePrefix(*e.net, e.index, &excluded);
    for (auto &e : edits)
        e.route->swap(e.next);
    lease_path();
    if (endpoints)
        sink_tile->pin_state.leased_nodes.setBit(std::get<3>(sink));
    // Register the replacement before releasing old leases, including old
    // islands: live owners in other bindings must keep every shared resource.
    for (const auto &ref : terminal_group)
        registerNetRouteTiles(*ref.net,
                              *bindingRoute(ref.net->routes[ref.binding_index]),
                              ref.binding_index);
    std::vector<const std::vector<Wire> *> removed;
    for (const auto &e : edits)
        removed.push_back(&e.next);
    clearRouteLeases(removed, true);
    for (const auto &e : edits)
    {
        if (sameCoord(e.next.front().from, e.route->front().from) &&
            e.next.front().local == e.route->front().local)
            continue;
        // Ordinary Takeoff cleanup releases CB usage but retains the output PIN
        // reservation. This endpoint changed, so release that old reservation
        // as well, through the live-owner check that protects surviving
        // siblings.
        releaseRouteFragmentLease(std::vector<Wire>{e.next.front()}, 0);
    }

    for (const auto &ref : terminal_group)
        for (const auto &e : edits)
            if (bindingRoute(ref.net->routes[ref.binding_index]) == e.route)
                rebuildNetRouteTiles(*ref.net, {e.next});
    if (!grounding_cuts.empty() && std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR"))
        std::fprintf(
            stderr,
            "REPAIR source grounding net=%s binding=%zu cuts=%zu complete=1\n",
            net.name.c_str(), index, grounding_cuts.size());
    timer.success = true;
    return true;
}

} // namespace

bool fpga::auditRouteCompletion(rtl::Net &net,
                                const rtl::NetRouteBinding &binding,
                                const Wire &source, const Wire &sink,
                                std::ostream &out,
                                const RepairGroundingPreemption &preemption,
                                size_t frontier_states, size_t probe_states,
                                int max_hops, bool capture_probe_frontier)
{
    if (!binding.from || !binding.to || binding.from_port.empty() ||
        binding.to_port.empty() || !frontier_states || !probe_states ||
        frontier_states > 8000000 || probe_states > 8000000)
        return false;
    const auto found =
        net.findRouteBinding(binding.from, binding.to, binding.from_port,
                             binding.to_port, binding.route_name);
    const auto *stored = found.index < net.routes.size()
                             ? bindingRoute(net.routes[found.index])
                             : nullptr;
    // A relocated sink can retain only a source prefix (or an empty slot).
    // Such a binding has no stored destination endpoint to audit. Inspect a
    // complete source-to-current-sink alternative without registering endpoints
    // or mutating that retained prefix, just as for an unbound diagnostic.
    const bool retained = stored && stored->size() >= 2 &&
                          stored->front().type == Wire::WIRE_TILE_PIN &&
                          stored->back().type == Wire::WIRE_TILE_PIN;
    // A retained task must use its real tree, including its already-owned sink.
    if (retained)
    {
        const auto *route = bindingRoute(net.routes[found.index]);
        if (!route || route->size() < 2 ||
            route->front().type != Wire::WIRE_TILE_PIN ||
            route->back().type != Wire::WIRE_TILE_PIN ||
            !sameCoord(route->front().from, source.from) ||
            route->front().local != source.local ||
            !sameCoord(route->back().from, sink.from) ||
            route->back().local != sink.local)
            return false;
        // Keep the retained fragments beside the numeric frontier for offline
        // replay.
        dumpNetRouteHistory(net, out);
    }

    const std::vector<Wire> endpoints{source, sink};
    auto probe = [&](size_t states, bool frontier)
    {
        return repairFromSource(
            net, retained ? found.index : std::numeric_limits<size_t>::max(),
            {}, states, max_hops, preemption, retained ? nullptr : &binding,
            retained ? nullptr : &endpoints, false, true, 1, &out, frontier);
    };

    // These are all diagnostic_only calls: even a fully validated free route
    // or legal combined cut stops before touching leases, bindings or victims.
    if (probe(frontier_states, true))
        return true;
    // Large failure captures are opt-in; keep their eligible arcs as well as
    // the witness so a replay does not turn a cut-only path into a free path.
    return probe_states > frontier_states &&
           probe(probe_states, capture_probe_frontier);
}

bool fpga::previewUnboundNetRoute(rtl::Net &net,
                                  const rtl::NetRouteBinding &binding,
                                  const Wire &source, const Wire &sink,
                                  const std::function<bool()> &cancelled,
                                  size_t max_states, int max_hops,
                                  const RepairGroundingPreemption &preemption,
                                  bool *budget_limited)
{
    if (budget_limited)
        *budget_limited = false;
    const std::vector<Wire> endpoints{source, sink};
    return repairFromSource(net, std::numeric_limits<size_t>::max(), cancelled,
                            max_states, max_hops, preemption, &binding,
                            &endpoints, false, true, 1, nullptr, false, false,
                            {}, budget_limited, nullptr, true);
}

namespace
{
bool routeUnboundWithPolicies(
    rtl::Net &net, const rtl::NetRouteBinding &binding, const Wire &source,
    const Wire &sink, const std::function<bool()> &cancelled, size_t max_states,
    int max_hops, const RepairGroundingPreemption &grounding_preemption,
    const RepairIncomingSources &incoming_sources, bool *budget_limited,
    const RouteCutNode *required_branch = nullptr)
{
    if (budget_limited)
        *budget_limited = false;
    if (!binding.from || !binding.to || binding.from_port.empty() ||
        binding.to_port.empty())
        return false;
    const auto existing =
        net.findRouteBinding(binding.from, binding.to, binding.from_port,
                             binding.to_port, binding.route_name);
    if (existing.index < net.routes.size())
    {
        auto &old = net.routes[existing.index];
        const auto *slot = bindingRoute(old);
        // Shared-prefix repair retires aliased storage without deleting the
        // logical connection. Its explicit uninstalled sentinel needs a new
        // slot. Other dangling indices/owners remain corruption, never free
        // fabric.
        const bool uninstalled =
            !old.owner && old.route_index == std::numeric_limits<size_t>::max();
        if (slot ? !slot->empty() : !uninstalled)
            return false;
    }

    const std::vector<Wire> endpoints{source, sink};
    // Prefer the existing tree, then allow a new departure. These are distinct
    // ownership policies; each call handles free tracing before its Grounding.
    auto attempt = [&](bool existing, bool fast)
    {
        return repairFromSource(
            net, std::numeric_limits<size_t>::max(), cancelled, max_states,
            max_hops, grounding_preemption, &binding, &endpoints, existing,
            false, 1, nullptr, false, fast, incoming_sources, budget_limited,
            required_branch, false);
    };

    if (attempt(true, true) ||
        (!(cancelled && cancelled()) && attempt(false, true)))
        return true;
    if (sourceFreeStateLimit(max_states) >= max_states ||
        (cancelled && cancelled()))
        return false;
    return attempt(true, false) ||
           (!(cancelled && cancelled()) && attempt(false, false));
}

} // namespace

bool fpga::routeUnboundNetFromSource(
    rtl::Net &net, const rtl::NetRouteBinding &binding, const Wire &source,
    const Wire &sink, const std::function<bool()> &cancelled, size_t max_states,
    int max_hops, const RepairGroundingPreemption &preemption,
    const RepairIncomingSources &incoming_sources, bool *budget_limited)
{
    return routeUnboundWithPolicies(net, binding, source, sink, cancelled,
                                    max_states, max_hops, preemption,
                                    incoming_sources, budget_limited);
}

// This wrapper retains one binding identity across candidate placements.
struct fpga::UnboundRouteSearch::Impl
{
    rtl::Net &net;
    rtl::NetRouteBinding endpoints;

    Impl(rtl::Net &n, const rtl::NetRouteBinding &b) : net(n), endpoints(b)
    {
    }

};

fpga::UnboundRouteSearch::UnboundRouteSearch(
    rtl::Net &net, const rtl::NetRouteBinding &binding)
    : impl(std::make_unique<Impl>(net, binding))
{
}

fpga::UnboundRouteSearch::~UnboundRouteSearch() = default;

void fpga::UnboundRouteSearch::clear()
{
}

bool fpga::UnboundRouteSearch::route(
    const Wire &source, const Wire &sink,
    const std::function<bool()> &cancelled, size_t max_states, int max_hops,
    const RepairGroundingPreemption &preemption,
    const RepairIncomingSources &incoming_sources, bool *budget_limited,
    const RouteCutNode *required_branch)
{
    return routeUnboundWithPolicies(impl->net, impl->endpoints, source, sink,
                                    cancelled, max_states, max_hops, preemption,
                                    incoming_sources, budget_limited,
                                    required_branch);
}

bool fpga::completeNetRoutePrefixFromSource(
    rtl::Net &net, size_t binding_index, const Wire &source, const Wire &sink,
    const std::function<bool()> &cancelled, size_t max_states, int max_hops,
    const RepairGroundingPreemption &preemption,
    const RouteCutNode *required_branch)
{
    if (binding_index >= net.routes.size())
        return false;
    auto &binding = net.routes[binding_index];
    const auto *prefix = bindingRoute(binding);
    if (!binding.from || !binding.to || !prefix || prefix->empty() ||
        hasRouteGaps(*prefix) || prefix->front().type != Wire::WIRE_TILE_PIN ||
        (prefix->size() > 1 && prefix->back().type == Wire::WIRE_TILE_PIN) ||
        !sameCoord(prefix->front().from, source.from))
        return false;
    // The caller enumerates the packed output port's physical LOCALs. A
    // retained prefix fixes its old departure, not every legal departure of
    // that port. Let the atomic source-tree replacement prove another LOCAL in
    // this CB; failure still preserves the prefix and every sibling/victim
    // lease.
    const std::vector<Wire> endpoints{source, sink};
    auto search = [&](bool takeoff_only)
    {
        return repairFromSource(net, binding_index, cancelled, max_states,
                                max_hops, preemption, nullptr, &endpoints,
                                takeoff_only, false, 1, nullptr, false, false,
                                {}, nullptr, required_branch);
    };

    return search(true) || (!(cancelled && cancelled()) && search(false));
}

bool fpga::completeNetRouteFromSource(
    rtl::Net &net, size_t binding_index, const std::function<bool()> &cancelled,
    size_t max_states, int max_hops,
    const RepairGroundingPreemption &preemption,
    const RepairIncomingSources &incoming_sources,
    const RouteCutNode *required_branch)
{
    if (binding_index >= net.routes.size())
        return false;
    const auto *route = bindingRoute(net.routes[binding_index]);
    if (!route || !hasRouteGaps(*route))
        return false;
    // One angle-led trace owns the attempt. Its bounded endpoint completion
    // validates cuts only after free continuations; do not restart a FIFO,
    // reverse frontier, or a different cost-weighted source search here.
    return repairFromSource(net, binding_index, cancelled, max_states, max_hops,
                            preemption, nullptr, nullptr, false, false, 1,
                            nullptr, false, false, incoming_sources, nullptr,
                            required_branch);
}

bool fpga::repairNetRouteGaps(
    rtl::Net &net, size_t binding_index, const std::function<bool()> &cancel,
    size_t max_states, int max_hops,
    const std::function<void(Tile &)> &audit_failure,
    const RepairGroundingPreemption &grounding_preemption,
    const RepairIncomingSources &incoming_sources)
{
    RouteHistoryScope history(&net, __func__);
    RepairPhaseTimer timer(net, binding_index, "exact_gap_search");
    const std::function<bool()> cancelled =
        cancel ? std::function<bool()>(
                     [&]
                     {
                         const bool stop = cancel();
                         timer.cancelled |= stop;
                         return stop;
                     })
               : std::function<bool()>{};
    auto is_deadend = [&](Tile &tile, int src)
    { return tile.cb.src_deadend.jump.testBit(src); };
    if (binding_index >= net.routes.size())
        return false;
    const auto &binding = net.routes[binding_index];
    auto *route = bindingRoute(net.routes[binding_index]);
    if (!route)
        return false;
    RouteOwnerQueries owner_queries;
    while (hasRouteGaps(*route))
    {
        if (cancelled && cancelled())
            return false;
        const Wire gap =
            *std::find_if(route->begin(), route->end(), [](const auto &w)
                          { return w.type == Wire::WIRE_GAP; });
        Tile *start = Device::current().getTile(gap.from.x, gap.from.y);
        Tile *finish = Device::current().getTile(gap.to.x, gap.to.y);
        const bool debug_repair =
            std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR") != nullptr;
        if (debug_repair)
            std::fprintf(stderr,
                         "REPAIR net=%s gap=(%d,%d)/DST%d->(%d,%d)/DST%d\n",
                         net.name.c_str(), gap.from.x, gap.from.y,
                         gap.from_node, gap.to.x, gap.to.y, gap.to_node);
        if (!start || !finish || gap.from_node < 0 || gap.to_node < 0)
            return false;
        const bool terminal_tail = gap.to_node_type == CB_NODE_LOCAL;
        const NodeMask reserved_terminal_joints =
            terminal_tail ? packedInputJointReservations(*finish, binding.to,
                                                         binding.to_port)
                          : NodeMask{};
        // The only boundary exemptions are the retained start and destination.
        auto same_source = [&](const rtl::NetRouteBinding &other)
        {
            return rtl::sameRouteSource(binding.from, binding.from_port,
                                        other.from, other.from_port);
        };

        for (const auto &[tile, type, node] :
             {std::tuple{start, CB_NODE_DST, gap.from_node},
              std::tuple{finish, terminal_tail ? CB_NODE_LOCAL : CB_NODE_DST,
                         gap.to_node}})
        {
            auto owners = findNetOwnersByNode(*tile, type, node);
            const bool valid =
                !owners.empty() &&
                std::all_of(owners.begin(), owners.end(),
                            [&](const auto &r)
                            {
                                return r.net &&
                                       r.binding_index < r.net->routes.size() &&
                                       same_source(
                                           r.net->routes[r.binding_index]);
                            });
            // A selected repair records the exact early gate, not just a
            // generic failure.
            if (timer.enabled)
                std::fprintf(stderr,
                             "REPAIR_BOUNDARY call=%llu tile=(%d,%d) type=%d "
                             "node=%d owners=%zu result=%s\n",
                             static_cast<unsigned long long>(timer.id),
                             tile->coord.x, tile->coord.y,
                             static_cast<int>(type), node, owners.size(),
                             valid            ? "accepted"
                             : owners.empty() ? "missing_owner"
                                              : "foreign_or_invalid_owner");
            if (!valid)
                return false;
        }

        // A free-only gap search cannot enter a terminal whose every incoming
        // DST belongs to another signal. Keep free, retained and sibling
        // entries; only a proven foreign cut lets us go straight to the
        // existing fallback.
        bool foreign_terminal_cut = false;
        if (terminal_tail && finish->cb_type)
        {
            finish->cb_type->ensureDerivedMasks();
            const auto entries =
                finish->cb_type->dsts_reaching_local.get(gap.to_node).jump;
            if (entries != NodeMask{} &&
                (entries & ~finish->cb.dst.jump) == NodeMask{})
            {
                foreign_terminal_cut = !entries.for_each_set_bit(
                    [&](int dst)
                    {
                        const auto owners =
                            findNetOwnersByNode(*finish, CB_NODE_DST, dst);
                        return owners.empty() ||
                               std::any_of(
                                   owners.begin(), owners.end(),
                                   [&](const auto &owner)
                                   {
                                       return !owner.net ||
                                              owner.binding_index >=
                                                  owner.net->routes.size() ||
                                              same_source(
                                                  owner.net->routes
                                                      [owner.binding_index]);
                                   });
                    });
            }

        }

        // A retained DST can also have a proven foreign cut on every incoming
        // SRC. Unknown owners, same-signal arcs and free alternatives forbid
        // this shortcut.
        if (!terminal_tail && incoming_sources &&
            !(start == finish && gap.from_node == gap.to_node))
        {
            bool any = false, blocked = true;
            incoming_sources(
                finish->coord, CB_NODE_DST, gap.to_node,
                [&](Coord at, CBNodeNameType kind, int node)
                {
                    any = true;
                    auto *tile = Device::current().getTile(at.x, at.y);
                    if (kind != CB_NODE_SRC || !tile || node < 0 ||
                        !tile->cb.src.jump.testBit(node))
                    {
                        blocked = false;
                        return;
                    }

                    const auto owners = findNetOwnersByNode(*tile, kind, node);
                    if (owners.empty() ||
                        std::any_of(
                            owners.begin(), owners.end(),
                            [&](const auto &owner)
                            {
                                return !owner.net ||
                                       owner.binding_index >=
                                           owner.net->routes.size() ||
                                       same_source(
                                           owner.net
                                               ->routes[owner.binding_index]);
                            }))
                        blocked = false;
                });
            foreign_terminal_cut = any && blocked;
        }

        if (foreign_terminal_cut)
            traceRepairDecision(
                "gap_terminal",
                repairNode(gap.from, CB_NODE_DST, gap.from_node),
                repairNode(gap.to, gap.to_node_type, gap.to_node),
                "all_entries_foreign_owned");

        struct Step
        {
            Tile *tile;
            int dst, parent, depth;
            Wire edge;
        };

        std::vector<Step> steps{{start, gap.from_node, -1, 0, {}}};
        timer.add(RepairCounter::gap_labels);
        // Reserve exact incoming nodes only on the active candidate. A later
        // arrival may have fewer hops or different temporary JOINT/SRC claims.
        std::set<std::tuple<int, int, int>> path_dsts{
            {gap.from.x, gap.from.y, gap.from_node}};
        int reached = -1;
        size_t interrupted_branches = 0;
        Wire grounding;
        // Share only the last decoded owner table. Recursive children may
        // replace it, so each parent checks its key again before reading
        // another SRC.
        SharedRepairEdges shared_edges{};
        NodeMask shared_sources;
        Tile *shared_tile = nullptr;
        int shared_dst = -1;
        // Follow an angle-prioritized exit immediately, retaining the parent's
        // cursor on return. Do not flood a FIFO frontier before trying the aim.
        std::function<bool(size_t)> trace = [&](size_t i)
        {
            if (cancelled && cancelled())
            {
                ++interrupted_branches;
                return false;
            }

            const Step step = steps[i];
            const auto trace_from =
                repairNode(step.tile->coord, CB_NODE_DST, step.dst);
            if (!pnr::InputRepairBudget::expand())
            {
                ++interrupted_branches;
                return false;
            }

            timer.expand(trace_from);
            traceRepairDecision(
                "gap_pop", trace_from,
                repairNode(gap.to, gap.to_node_type, gap.to_node), "expand");
            if (debug_repair)
                std::fprintf(stderr, "REPAIR expand=(%d,%d)/DST%d depth=%d\n",
                             step.tile->coord.x, step.tile->coord.y, step.dst,
                             step.depth);
            if (!terminal_tail && step.tile == finish &&
                step.dst == gap.to_node)
            {
                reached = i;
                return true;
            }

            if (!step.tile->cb_type)
                return false;
            // Check a reached boundary before the label ceiling. Grounding at
            // the current terminal Tile needs no additional inter-tile label.
            if (steps.size() >= max_states &&
                (!terminal_tail || step.tile != finish))
            {
                ++interrupted_branches;
                return false;
            }

            auto &type = *step.tile->cb_type;
            type.ensureDerivedMasks();
            // Reconstruct temporary resource claims once for this recursion
            // frame, not for every candidate SRC. A different node in the same
            // Tile is legal.
            NodeMask occupied = step.tile->cb.joint.jump, path_sources;
            for (int p = static_cast<int>(i); p >= 0; p = steps[p].parent)
            {
                timer.add(RepairCounter::gap_parent_steps);
                const auto &ancestor = steps[p];
                if (ancestor.parent < 0 ||
                    !sameCoord(ancestor.edge.from, step.tile->coord))
                    continue;
                const auto &w = ancestor.edge;
                if (w.jump >= 0)
                    path_sources.setBit(w.jump);
                if (w.joint >= 0)
                    occupied.setBit(w.joint);
                if (w.joint2 >= 0)
                    occupied.setBit(w.joint2);
            }

            if (terminal_tail && step.tile == finish)
            {
                const NodeMask terminal_occupied =
                    occupied | reserved_terminal_joints;
                int joint = -1, joint2 = -1;
                bool connected =
                    type.dst_local[step.dst].local.testBit(gap.to_node);
                if (!connected)
                {
                    const auto first =
                        type.dst_joint[step.dst].joint & ~terminal_occupied;
                    const auto last =
                        type.local_reachable_joints[gap.to_node].joint &
                        ~terminal_occupied;
                    joint = (first & last).firstSetBit();
                    if (joint < 0)
                        first.for_each_set_bit(
                            [&](int node)
                            {
                                joint = (type.joint_joint[node].joint & last)
                                            .firstSetBit();
                                if (joint < 0)
                                    return false;
                                joint2 = node;
                                return true;
                            });
                    connected = joint >= 0;
                }

                if (connected)
                {
                    grounding.from = grounding.to = finish->coord;
                    grounding.pos = 1;
                    grounding.local = step.dst;
                    grounding.joint = joint;
                    grounding.joint2 = joint2;
                    grounding.owns_dst = i != 0;
                    reached = i;
                    return true;
                }

            }

            if (step.depth >= max_hops || steps.size() >= max_states)
            {
                ++interrupted_branches;
                return false;
            }

            CBType::FastSearchCursor source_cursor;
            for (int src; (src = type.fastSearchNextHop(
                               gap.to - step.tile->coord, source_cursor,
                               CB_NODE_DST, step.dst)) >= 0;)
            {
                if (cancelled && cancelled())
                {
                    ++interrupted_branches;
                    return false;
                }

                timer.add(RepairCounter::gap_edges);
                const auto trace_src =
                    repairNode(step.tile->coord, CB_NODE_SRC, src);
                auto reject = [&](std::string_view reason)
                {
                    traceRepairDecision("gap_exit", trace_from, trace_src,
                                        reason);
                };

                if (is_deadend(*step.tile, src))
                {
                    reject("persistent_src_deadend");
                    continue;
                }

                if (debug_repair)
                    std::fprintf(stderr, "REPAIR src=%d busy=%d\n", src,
                                 step.tile->cb.src.jump.testBit(src));
                // A prior repair may already have restored a common portion of
                // this tree. Reuse only that exact same-source DST->SRC->DST
                // fragment, never a foreign lease or another incoming
                // connection to this SRC.
                const SharedRepairEdge *shared_edge = nullptr;
                SharedRepairEdge owned_edge;
                if (step.tile->cb.src.jump.testBit(src))
                {
                    timer.add(RepairCounter::shared_edge_checks);
                    if (shared_tile != step.tile || shared_dst != step.dst)
                    {
                        collectSharedRepairEdges(*step.tile, step.dst, binding,
                                                 shared_sources, shared_edges);
                        shared_tile = step.tile;
                        shared_dst = step.dst;
                    }

                    if (shared_sources.testBit(src))
                    {
                        owned_edge = shared_edges[src];
                        shared_edge = &owned_edge;
                    }

                    if (!shared_edge)
                    {
                        timer.add(RepairCounter::gap_busy_src);
                        reject("busy_src_no_shared_edge");
                        continue;
                    }

                    if (debug_repair)
                        std::fprintf(
                            stderr, "REPAIR verify_shared_src=(%d,%d)/%d\n",
                            step.tile->coord.x, step.tile->coord.y, src);
                    timer.add(RepairCounter::owner_checks);
                    const bool same_tree = owner_queries.allMatch(
                        *step.tile, CB_NODE_SRC, src,
                        [&](const auto &ref)
                        {
                            return same_source(
                                ref.net->routes[ref.binding_index]);
                        });
                    if (!same_tree)
                    {
                        timer.add(RepairCounter::gap_busy_src);
                        reject("foreign_src_owner");
                        continue;
                    }

                }

                // Select an actually free direct/one-joint/two-joint
                // connection.
                if (path_sources.testBit(src))
                {
                    reject("prefix_src");
                    continue;
                }

                int joint = -1, joint2 = -1;
                if (shared_edge)
                {
                    joint = shared_edge->joint;
                    joint2 = shared_edge->joint2;
                }

                else if (!type.dst_src[step.dst].jump.testBit(src))
                {
                    const NodeMask first =
                        type.dst_joint[step.dst].joint & ~occupied;
                    const NodeMask last =
                        type.src_reachable_joints[src].joint & ~occupied;
                    joint = (first & last).firstSetBit();
                    if (joint < 0)
                        first.for_each_set_bit(
                            [&](int node)
                            {
                                joint = (type.joint_joint[node].joint & last)
                                            .firstSetBit();
                                if (joint < 0)
                                    return false;
                                joint2 = node;
                                return true;
                            });
                    if (joint < 0)
                    {
                        reject("joint_blocked");
                        continue;
                    }

                }

                reject("exit_eligible");
                for (const auto &target :
                     Device::current().resolveJumpTargets(*step.tile, src))
                {
                    if (debug_repair && target.tile)
                        std::fprintf(stderr,
                                     "REPAIR target=(%d,%d)/DST%d busy=%d\n",
                                     target.tile->coord.x, target.tile->coord.y,
                                     target.dst_node,
                                     target.dst_node >= 0 &&
                                         target.tile->cb.dst.jump.testBit(
                                             target.dst_node));
                    if (!target.tile || target.dst_node < 0)
                        continue;
                    if (shared_edge &&
                        (!sameCoord(shared_edge->to, target.tile->coord) ||
                         shared_edge->dst != target.dst_node))
                        continue;
                    const bool terminal = !terminal_tail &&
                                          target.tile == finish &&
                                          target.dst_node == gap.to_node;
                    const auto trace_dst = repairNode(
                        target.tile->coord, CB_NODE_DST, target.dst_node);
                    if (!terminal && !shared_edge &&
                        target.tile->cb.dst.jump.testBit(target.dst_node))
                    {
                        timer.add(RepairCounter::gap_busy_dst);
                        traceRepairDecision("gap_jump", trace_src, trace_dst,
                                            "busy_dst");
                        continue;
                    }

                    if (steps.size() >= max_states)
                    {
                        ++interrupted_branches;
                        timer.add(RepairCounter::gap_limit);
                        traceRepairDecision("gap_jump", trace_src, trace_dst,
                                            "state_limit");
                        continue;
                    }

                    const auto key =
                        std::tuple{target.tile->coord.x, target.tile->coord.y,
                                   target.dst_node};
                    if (!path_dsts.insert(key).second)
                    {
                        ++interrupted_branches;
                        timer.add(RepairCounter::gap_seen);
                        traceRepairDecision("gap_jump", trace_src, trace_dst,
                                            "prefix_dst");
                        continue;
                    }

                    Wire w;
                    w.from = step.tile->coord;
                    w.to = target.tile->coord;
                    w.pos = 1;
                    w.local = step.dst;
                    w.jump = src;
                    w.dst = target.dst_node;
                    w.route_jump = target.jump_node;
                    w.joint = joint;
                    w.joint2 = joint2;
                    w.shared = shared_edge != nullptr;
                    w.owns_dst =
                        i != 0; // retained prefix owns the first boundary
                    if (const auto *name = type.nodeName(CB_NODE_DST, step.dst))
                        w.from_wire_name = *name;
                    if (const auto *name = type.nodeName(CB_NODE_SRC, src))
                        w.src_wire_name = *name;
                    w.dst_wire_name = target.dst_wire;
                    steps.push_back({target.tile, target.dst_node,
                                     static_cast<int>(i), step.depth + 1, w});
                    timer.add(RepairCounter::gap_labels);
                    traceRepairDecision("gap_jump", trace_src, trace_dst,
                                        "descend");
                    const bool connected = trace(steps.size() - 1);
                    path_dsts.erase(key);
                    if (connected)
                        return true;
                }

                // This gap search follows DST-to-SRC jumps only. A failed jump
                // can still continue through a LOCAL bridge in the source trace
                // below.
            }

            return false;
        };

        if (!foreign_terminal_cut)
            trace(0);
        if (reached < 0)
        {
            // Inspect the failed frontier before fallback can mutate any
            // leases. This selected, bounded dump is diagnostic only; it never
            // prunes search.
            const char *failure_filter =
                std::getenv("SCALEPNR_DEBUG_GAP_FAILURE_NET");
            if (failure_filter && *failure_filter &&
                (net.name.find(failure_filter) != std::string::npos ||
                 binding.route_name.find(failure_filter) != std::string::npos))
            {
                std::cout << "GAP_FAILURE_FRONTIER net="
                          << std::quoted(net.name)
                          << " route=" << std::quoted(binding.route_name)
                          << " states=" << steps.size()
                          << " shown=" << std::min<size_t>(16, steps.size())
                          << " target=(" << gap.to.x << ',' << gap.to.y << ")/"
                          << int(gap.to_node_type) << ':' << gap.to_node
                          << '\n';
                auto mask = [&](const char *name, const NodeMask &bits)
                {
                    std::cout << ' ' << name << "=[";
                    bits.for_each_set_bit(
                        [&](int bit)
                        {
                            std::cout << bit << ',';
                            return false;
                        });
                    std::cout << ']';
                };

                auto node = [&](Tile &tile, CBNodeNameType kind, int id)
                {
                    const auto *name = tile.cb_type
                                           ? tile.cb_type->nodeName(kind, id)
                                           : nullptr;
                    std::cout
                        << "GAP_FAILURE_NODE tile=(" << tile.coord.x << ','
                        << tile.coord.y << ") type=" << int(kind)
                        << " node=" << id << " name="
                        << std::quoted(name ? std::string(*name)
                                            : std::string{})
                        << " leased=" << repairNodeLeased(tile.cb, kind, id);
                    const auto owners = findNetOwnersByNode(tile, kind, id);
                    std::cout << " owners=" << owners.size() << '\n';
                    for (const auto &owner : owners)
                    {
                        if (!owner.net ||
                            owner.binding_index >= owner.net->routes.size())
                            continue;
                        const auto &other =
                            owner.net->routes[owner.binding_index];
                        std::cout << "GAP_FAILURE_OWNER net="
                                  << std::quoted(owner.net->name)
                                  << " route=" << std::quoted(other.route_name)
                                  << " route_id=" << other.route_id
                                  << " same_source=" << same_source(other)
                                  << '\n';
                    }

                };

                for (size_t i = 0; i < std::min<size_t>(16, steps.size()); ++i)
                {
                    const auto &step = steps[i];
                    auto &tile = *step.tile;
                    std::cout << "GAP_FAILURE_STATE index=" << i
                              << " parent=" << step.parent
                              << " depth=" << step.depth
                              << " via_src=" << step.edge.jump;
                    mask("src_busy", tile.cb.src.jump);
                    mask("dst_busy", tile.cb.dst.jump);
                    mask("joint_busy", tile.cb.joint.jump);
                    std::cout << '\n';
                    node(tile, CB_NODE_DST, step.dst);
                    if (!tile.cb_type)
                        continue;
                    const auto *sources =
                        tile.cb_type->srcNodes(CB_NODE_DST, step.dst);
                    if (!sources)
                        continue;
                    for (int src : *sources)
                    {
                        node(tile, CB_NODE_SRC, src);
                        const auto targets =
                            Device::current().resolveJumpTargets(tile, src);
                        std::cout << "GAP_FAILURE_TARGETS src=" << src
                                  << " count=" << targets.size() << '\n';
                        for (const auto &target : targets)
                            if (target.tile && target.dst_node >= 0)
                                node(*target.tile, CB_NODE_DST,
                                     target.dst_node);
                    }

                }

            }

            timer.next("source_fallback");
            // One selected-call summary distinguishes an empty frontier from a
            // cap.
            if (timer.enabled)
                std::fprintf(
                    stderr,
                    "REPAIR_OUTCOME call=%llu result=gap_unreached labels=%zu "
                    "max_states=%zu max_hops=%d foreign_terminal_cut=%d\n",
                    static_cast<unsigned long long>(timer.id), steps.size(),
                    max_states, max_hops, foreign_terminal_cut);
            if (debug_repair)
                std::fprintf(stderr, "REPAIR failed states=%zu limit=%zu\n",
                             steps.size(), max_states);
            // LOCAL bridges require typed tracing from the connected source
            // tree. Make one angle-led attempt; only endpoint Grounding may
            // search locally.
            if (!(cancelled && cancelled()) &&
                repairFromSource(net, binding_index, cancelled, max_states,
                                 max_hops, grounding_preemption, nullptr,
                                 nullptr, false, false, 1, nullptr, false,
                                 false))
            {
                timer.success = true;
                return true;
            }

            if (const char *filter =
                    std::getenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY");
                filter && *filter && net.name.find(filter) != std::string::npos)
            {
                const char *path =
                    std::getenv("SCALEPNR_SOURCE_REACHABILITY_LOG");
                std::ofstream out(path && *path ? path
                                                : "source_reachability.log",
                                  std::ios::app);
                if (!out)
                    throw std::runtime_error(
                        "cannot open source reachability audit log");
                if (!auditNetRouteReachability(net, binding_index, out))
                    auditNetRouteReachability(net, binding_index, out, false);
            }

            // Opt-in, read-only evidence at the actual repair frontier. The
            // normal task failure location may instead fall back to the
            // retained sink pin.
            if (audit_failure)
            {
                std::unordered_set<Tile *> audited;
                auto audit = [&](Tile *tile)
                {
                    if (tile && audited.insert(tile).second)
                        audit_failure(*tile);
                };

                audit(start);
                audit(finish);
                // Bound diagnostic output when an unrelated failure exhausts a
                // large search. Include every reached Tile for the
                // small-frontier case.
                for (size_t i = 0; i < std::min<size_t>(steps.size(), 16); ++i)
                {
                    const auto &step = steps[i];
                    audit(step.tile);
                    if (!step.tile->cb_type)
                        continue;
                    const auto *sources =
                        step.tile->cb_type->srcNodes(CB_NODE_DST, step.dst);
                    if (!sources)
                        continue;
                    for (int src : *sources)
                    {
                        if (step.tile->cb.src.jump.testBit(src))
                            continue;
                        for (const auto &target :
                             Device::current().resolveJumpTargets(*step.tile,
                                                                  src))
                            audit(target.tile);
                    }

                }

            }

            return false;
        }

        std::vector<Wire> replacement;
        timer.next("exact_gap_commit");
        for (int i = reached; steps[i].parent >= 0; i = steps[i].parent)
            replacement.push_back(steps[i].edge);
        std::reverse(replacement.begin(), replacement.end());
        if (terminal_tail)
            replacement.push_back(grounding);
        if (replacement.empty())
            return false;

        // Discover replicas through the retained boundary, including aliases in
        // other RTL nets. Construct all new vectors before committing any
        // lease.
        struct Edit
        {
            rtl::Net *net;
            std::vector<Wire> *route;
            std::vector<Wire> next;
        };

        std::vector<Edit> edits;
        std::unordered_set<rtl::Net *> nets{&net};
        for (auto &ref : start->routedNets)
            if (ref.peer)
                nets.insert(ref.peer);
        for (auto *other_net : nets)
            for (auto &other : other_net->routes)
            {
                if (!same_source(other))
                    continue;
                auto *wires = bindingRoute(other);
                if (!wires ||
                    std::any_of(edits.begin(), edits.end(), [&](const auto &e)
                                { return e.route == wires; }))
                    continue;
                auto found = std::find_if(
                    wires->begin(), wires->end(),
                    [&](const auto &w)
                    {
                        return w.type == Wire::WIRE_GAP &&
                               sameCoord(w.from, gap.from) &&
                               sameCoord(w.to, gap.to) &&
                               w.from_node == gap.from_node &&
                               w.to_node_type == gap.to_node_type &&
                               w.to_node == gap.to_node;
                    });
                if (found == wires->end())
                    continue;
                std::vector<Wire> next(wires->begin(), found);
                for (auto w : replacement)
                {
                    // A donor or another repaired replica can have a different
                    // prefix (including an equivalent numeric-edge
                    // representation). Common-prefix promotion cannot protect
                    // these later hops when that route is removed. Every
                    // repaired segment therefore keeps independent lease
                    // claims.
                    w.shared = false;
                    next.push_back(std::move(w));
                }

                next.insert(next.end(), found + 1, wires->end());
                edits.push_back({other_net, wires, std::move(next)});
            }

        if (edits.empty() || (cancelled && cancelled()))
            return false;
        for (const auto &w : replacement)
        {
            auto *tile = Device::current().getTile(w.from.x, w.from.y);
            if (w.jump >= 0)
                tile->cb.src.jump.setBit(w.jump);
            if (w.owns_dst)
                tile->cb.dst.jump.setBit(w.local);
            if (w.joint >= 0)
                tile->cb.joint.jump.setBit(w.joint);
            if (w.joint2 >= 0)
                tile->cb.joint.jump.setBit(w.joint2);
        }

        for (auto &edit : edits)
            edit.route->swap(edit.next);
        std::unordered_map<rtl::Net *, std::vector<std::vector<Wire>>> affected;
        for (auto &edit : edits)
            affected[edit.net].push_back(std::move(edit.next));
        for (const auto &[n, old] : affected)
            rebuildNetRouteTiles(*n, old);
        timer.next("exact_gap_search");
    }

    timer.success = true;
    return true;
}

namespace
{

bool trimNetRouteToTakeoff(rtl::Net &net, size_t route_binding_index)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty())
    {
        return false;
    }

    size_t keep = 0;
    while (keep < route->size())
    {
        const Wire &fragment = (*route)[keep++];
        if (fragment.type == Wire::WIRE_CROSSBAR && fragment.jump >= 0 &&
            !sameCoord(fragment.from, fragment.to))
        {
            break;
        }

    }

    if (keep == route->size())
    {
        return false;
    }

    return truncateNetRoute(net, route_binding_index, keep);
}

} // namespace

bool fpga::unrouteNetRouteToTakeoff(rtl::Net &net, size_t route_binding_index)
{
    return trimNetRouteToTakeoff(net, route_binding_index);
}

bool fpga::unrouteNetRouteToTakeoffFromNode(rtl::Net &net,
                                            size_t route_binding_index,
                                            Coord tile_coord,
                                            CBNodeNameType node_type, int node)
{
    if (route_binding_index >= net.routes.size() || node < 0)
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty())
    {
        return false;
    }

    auto fragment_uses_node = [&](const Wire &fragment)
    {
        bool from_tile = sameCoord(fragment.from, tile_coord);
        bool to_tile = sameCoord(fragment.to, tile_coord);
        if (fragment.type == Wire::WIRE_ROUTE_EDGE)
        {
            return (from_tile && fragment.from_node_type == node_type &&
                    fragment.from_node == node) ||
                   (to_tile && fragment.to_node_type == node_type &&
                    fragment.to_node == node);
        }

        if (fragment.type != Wire::WIRE_CROSSBAR)
        {
            return node_type == CB_NODE_LOCAL &&
                   fragment.type == Wire::WIRE_TILE_PIN && from_tile &&
                   fragment.local == node;
        }

        if (node_type == CB_NODE_SRC)
        {
            return from_tile && fragment.jump == node;
        }

        if (node_type == CB_NODE_DST)
        {
            return (from_tile && fragment.pos != 0 && fragment.local == node) ||
                   (to_tile && fragment.owns_landing && fragment.dst == node);
        }

        if (node_type == CB_NODE_LOCAL)
        {
            return from_tile && fragment.pos == 0 && fragment.local == node;
        }

        return node_type == CB_NODE_JOINT && from_tile &&
               (fragment.joint == node || fragment.joint2 == node);
    };

    size_t conflict = 0;
    while (conflict < route->size() && !fragment_uses_node((*route)[conflict]))
    {
        ++conflict;
    }

    if (conflict == route->size())
    {
        return false;
    }

    size_t keep = 0;
    while (keep < route->size())
    {
        const Wire &fragment = (*route)[keep++];
        if (fragment.type == Wire::WIRE_CROSSBAR && fragment.jump >= 0 &&
            !sameCoord(fragment.from, fragment.to))
        {
            break;
        }

    }

    // A conflict on the takeoff itself cannot preserve that takeoff.
    if (keep == route->size() || conflict < keep)
    {
        return false;
    }

    return trimNetRouteToTakeoff(net, route_binding_index);
}

bool fpga::unrouteLastRouteStep(rtl::Net &net, size_t route_binding_index)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty())
    {
        return false;
    }

    size_t step_index = route->size();
    while (step_index > 0 &&
           (*route)[step_index - 1].type == Wire::WIRE_TILE_PIN)
    {
        --step_index;
    }

    if (step_index == 0 || (*route)[step_index - 1].shared)
    {
        return false;
    }

    --step_index;
    // The removed hop owns its incoming DST. Transfer that lease back to the
    // retained prefix before releasing the hop, just as for any other cut.
    // Otherwise a retry prefix ends on an unreserved node another net can take.
    return step_index == 0
               ? unrouteNetRoute(net, route_binding_index)
               : truncateNetRoute(net, route_binding_index, step_index);
}

bool fpga::unrouteNetBranch(rtl::Net &net, size_t route_binding_index)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    rtl::NetRouteBinding &binding = net.routes[route_binding_index];
    std::vector<Wire> *route = bindingRoute(binding);
    if (!route || route->empty())
    {
        return false;
    }

    size_t branch_start = 0;
    while (branch_start < route->size() && (*route)[branch_start].shared)
    {
        ++branch_start;
    }

    if (branch_start == route->size())
    {
        return false;
    }

    std::vector<Wire> branch(route->begin() +
                                 static_cast<std::ptrdiff_t>(branch_start),
                             route->end());
    route->resize(branch_start);
    clearRouteLeases(branch, true);
    rebuildNetRouteTiles(net, {branch});
    return true;
}

bool fpga::unrouteBrunch(rtl::Net &net, size_t route_binding_index)
{
    return unrouteNetBranch(net, route_binding_index);
}

// Match pre-move anchor selection to the fabric retained during sink
// detachment.
size_t fpga::retainedRoutePrefixSize(const std::vector<Wire> &route,
                                     bool source_moves, bool destination_moves)
{
    if (source_moves)
    {
        return 0;
    }

    if (!destination_moves)
    {
        return route.size();
    }

    size_t keep = route.size();
    while (keep > 0 && route[keep - 1].type == Wire::WIRE_TILE_PIN)
    {
        --keep;
    }

    if (keep > 0 && route[keep - 1].type == Wire::WIRE_CROSSBAR)
    {
        const Wire &entry = route[keep - 1];
        if (entry.jump < 0 || sameCoord(entry.from, entry.to))
        {
            --keep;
        }

    }

    return keep;
}

// Release the terminal while preserving proven private and live shared fabric.
static bool detachNetRouteDestinationImpl(
    rtl::Net &net, size_t route_binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded,
    bool retain_private_prefix, size_t proven_prefix = 0)
{
    RouteHistoryScope history(&net, __func__);
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    rtl::NetRouteBinding &binding = net.routes[route_binding_index];
    std::vector<Wire> *route = bindingRoute(binding);
    if (!route || route->empty())
    {
        return false;
    }

    size_t keep =
        std::max(proven_prefix,
                 liveSharedPrefixLength(net, route_binding_index, excluded));

    if (retain_private_prefix)
    {
        // Without an explicit proof, only a completed route supplies a reusable
        // private prefix. Unproven incomplete tails must still be released.
        keep = std::max(keep, retainedRoutePrefixSize(*route, false, true));
    }

    if (keep == route->size() && keep > 1 &&
        (*route)[keep - 1].type == Wire::WIRE_TILE_PIN)
    {
        --keep;
    }

    // The retained prefix owns a landing reused by the first removed terminal
    // fragment.
    if (keep > 0 && keep < route->size() &&
        (*route)[keep - 1].type == Wire::WIRE_CROSSBAR &&
        (*route)[keep].type == Wire::WIRE_CROSSBAR &&
        (*route)[keep - 1].dst == (*route)[keep].local)
    {
        (*route)[keep - 1].owns_landing = true;
        (*route)[keep].owns_dst = false;
    }

    std::vector<Wire> removed(
        route->begin() + static_cast<std::ptrdiff_t>(keep), route->end());
    if (!removed.empty() && keep > 0 &&
        (*route)[keep - 1].type == Wire::WIRE_CROSSBAR &&
        removed.front().type == Wire::WIRE_CROSSBAR &&
        (*route)[keep - 1].dst == removed.front().local)
    {
        removed.front().owns_dst = false;
    }

    route->resize(keep);
    clearRouteLeases(removed, true);
    rebuildNetRouteTiles(net, {removed});
    return true;
}

bool fpga::detachNetRouteDestination(rtl::Net &net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    return detachNetRouteDestinationImpl(net, route_binding_index, nullptr,
                                         route && isRouteComplete(*route));
}

// A moved sink always needs routing again; preserve only the reusable prefix
// when its binding currently owns physical route fragments.
bool fpga::invalidateMovedSinkRoute(rtl::Net &net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size())
    {
        return false;
    }

    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (route && !route->empty())
    {
        // A downstream fanout may branch from this route's private suffix.
        // Transfer that physical prefix before truncating the moved sink.
        promoteSurvivingSourcePrefix(net, route_binding_index);
        bool retain_private = isRouteComplete(*route);
        detachNetRouteDestinationImpl(net, route_binding_index, nullptr,
                                      retain_private);
    }

    return true;
}

bool fpga::invalidateMovedSinkRoutes(
    const std::vector<NetRouteRef> &routes,
    const std::vector<ProvenRoutePrefix> &proven_prefixes)
{
    std::unordered_set<const rtl::NetRouteBinding *> excluded;
    std::unordered_set<const rtl::NetRouteBinding *> complete;
    for (const NetRouteRef &ref : routes)
    {
        if (ref.net && ref.binding_index < ref.net->routes.size())
        {
            rtl::NetRouteBinding *binding = &ref.net->routes[ref.binding_index];
            excluded.insert(binding);
            std::vector<Wire> *route = bindingRoute(*binding);
            if (route && isRouteComplete(*route))
            {
                complete.insert(binding);
            }

        }

    }

    // Validate every proof before changing ownership or releasing any leases.
    std::unordered_map<const rtl::NetRouteBinding *, size_t> proven;
    for (const ProvenRoutePrefix &prefix : proven_prefixes)
    {
        const NetRouteRef &ref = prefix.route;
        if (!ref.net || ref.binding_index >= ref.net->routes.size())
        {
            return false;
        }

        auto *binding = &ref.net->routes[ref.binding_index];
        const auto *route = bindingRoute(*binding);
        if (!excluded.contains(binding) || !route || prefix.fragments == 0 ||
            prefix.fragments > retainedRoutePrefixSize(*route, false, true) ||
            (*route)[prefix.fragments - 1].type != Wire::WIRE_CROSSBAR)
        {
            return false;
        }

        proven[binding] = std::max(proven[binding], prefix.fragments);
    }

    // Transfer ownership only to bindings that survive the complete atomic
    // relocation set.
    for (const NetRouteRef &ref : routes)
    {
        if (!ref.net || ref.binding_index >= ref.net->routes.size())
        {
            continue;
        }

        std::vector<Wire> *route =
            bindingRoute(ref.net->routes[ref.binding_index]);
        if (route && !route->empty())
        {
            promoteSurvivingSourcePrefix(*ref.net, ref.binding_index,
                                         &excluded);
        }

    }

    bool changed = false;
    for (const NetRouteRef &ref : routes)
    {
        if (!ref.net || ref.binding_index >= ref.net->routes.size())
        {
            continue;
        }

        const rtl::NetRouteBinding *binding =
            &ref.net->routes[ref.binding_index];
        // A selected anchor replaces the default complete-route retention
        // boundary.
        auto prefix = proven.find(binding);
        changed = detachNetRouteDestinationImpl(
                      *ref.net, ref.binding_index, &excluded,
                      prefix == proven.end() && complete.contains(binding),
                      prefix == proven.end() ? 0 : prefix->second) ||
                  changed;
    }

    return changed;
}

bool fpga::discardNetBranch(rtl::Net &net, size_t route_binding_index)
{
    RouteHistoryScope history(&net, __func__);
    // A branch's private suffix may now be another branch's shared prefix.
    // Preserve those owners before clearing this binding, release only unused
    // leases, and remove registrations on its entire old path (shared part
    // too).
    return unrouteNetRoute(net, route_binding_index);
}

// Clear an atomic route tree, including shared fanout fragments owned by the
// tree.
bool fpga::unrouteNetRouteTree(rtl::Net &net,
                               const std::vector<size_t> &route_binding_indices)
{
    RouteHistoryScope history(&net, __func__);
    bool changed = false;
    std::vector<std::vector<Wire>> removed_routes;
    for (size_t route_binding_index : route_binding_indices)
    {
        if (route_binding_index >= net.routes.size())
        {
            continue;
        }

        std::vector<Wire> *route =
            bindingRoute(net.routes[route_binding_index]);
        if (!route || route->empty())
        {
            continue;
        }

        // Transfer every retained shared prefix before removing its current
        // owner.
        promoteSurvivingSourcePrefix(net, route_binding_index);
        removed_routes.push_back(*route);
        route->clear();
        changed = true;
    }

    if (!changed)
    {
        return false;
    }

    std::vector<const std::vector<Wire> *> removed_refs;
    removed_refs.reserve(removed_routes.size());
    for (const std::vector<Wire> &removed : removed_routes)
    {
        removed_refs.push_back(&removed);
    }

    clearRouteLeases(removed_refs, true);
    rebuildNetRouteTiles(net, removed_routes);
    return true;
}

bool fpga::unrouteSourceRouteTree(const std::vector<NetRouteRef> &routes)
{
    std::unordered_map<rtl::Net *, std::vector<std::vector<Wire>>>
        removed_routes_by_net;
    std::unordered_set<const rtl::NetRouteBinding *> removed_bindings;
    for (const NetRouteRef &ref : routes)
    {
        if (!ref.net || ref.binding_index >= ref.net->routes.size())
        {
            continue;
        }

        rtl::NetRouteBinding &binding = ref.net->routes[ref.binding_index];
        if (!removed_bindings.insert(&binding).second)
        {
            continue;
        }

        std::vector<Wire> *route = bindingRoute(binding);
        if (!route || route->empty())
        {
            continue;
        }

        std::vector<Wire> &removed =
            removed_routes_by_net[ref.net].emplace_back(*route);
        // The caller supplies the entire source tree, so no shared replica
        // survives.
        for (Wire &fragment : removed)
        {
            fragment.shared = false;
        }

        // Prefix retries can replace this route many times. Return its old
        // backing storage now instead of retaining every historical peak
        // allocation.
        std::vector<Wire>().swap(*route);
    }

    if (removed_routes_by_net.empty())
    {
        return false;
    }

    std::vector<const std::vector<Wire> *> removed_refs;
    removed_refs.reserve(routes.size());
    for (const auto &[net, removed_routes] : removed_routes_by_net)
    {
        (void)net;
        for (const std::vector<Wire> &route : removed_routes)
        {
            removed_refs.push_back(&route);
        }

    }

    // Atomic source ownership makes per-fragment live-owner scans unnecessary.
    clearRouteLeases(removed_refs, false);
    // Rebuild each logical net only from tiles touched by that net's removed
    // bindings; comparing every net with the complete batch is quadratic.
    for (auto &[net, removed_routes] : removed_routes_by_net)
    {
        rebuildNetRouteTiles(*net, removed_routes);
    }

    return true;
}

bool fpga::unrouteNet(rtl::Net &net)
{
    bool changed = false;
    std::vector<std::vector<Wire>> removed_routes;
    for (size_t binding_index = 0; binding_index < net.routes.size();
         ++binding_index)
    {
        rtl::NetRouteBinding &binding = net.routes[binding_index];
        std::vector<Wire> *route = bindingRoute(binding);
        if (!route || route->empty())
        {
            continue;
        }

        // A cross-net fanout can survive whole-net removal and inherit
        // ownership.
        promoteSurvivingSourcePrefix(net, binding_index);
        removed_routes.push_back(*route);
        route->clear();
        changed = true;
    }

    if (changed)
    {
        std::vector<const std::vector<Wire> *> removed_refs;
        removed_refs.reserve(removed_routes.size());
        for (const std::vector<Wire> &removed : removed_routes)
        {
            removed_refs.push_back(&removed);
        }

        clearRouteLeases(removed_refs, true);
        rebuildNetRouteTiles(net, removed_routes);
    }

    return changed;
}

// Release every binding for one exact driver-to-sink connection while
// preserving unrelated connections represented by the same rtl::Net object.
size_t fpga::unrouteNetConnection(rtl::Net &net, rtl::Inst *from, rtl::Inst *to,
                                  const std::string &from_port,
                                  const std::string &to_port)
{
    size_t cleared = 0;
    for (size_t binding_index = 0; binding_index < net.routes.size();
         ++binding_index)
    {
        const rtl::NetRouteBinding &binding = net.routes[binding_index];
        if (binding.from != from || binding.to != to ||
            binding.from_port != from_port || binding.to_port != to_port)
        {
            continue;
        }

        if (unrouteNetRoute(net, binding_index))
        {
            ++cleared;
        }

    }

    return cleared;
}
