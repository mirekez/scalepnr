#pragma once

#include "FullSearch.h"
#include "RoutePreemption.h"

#include "NodeMask.h"
#include "Tile.h"
#include "Wire.h"

#include <functional>
#include <iosfwd>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fpga
{
struct Device;
}

namespace pnr
{

// Bound duplicate suppression, not routing depth. Older duplicates may remain;
// they must never cause a distinct frontier position or bridge to be discarded.
template <typename Entry>
std::span<Entry> recentDockingEntries(std::vector<Entry> &entries)
{
    constexpr size_t lookback = 16;
    return std::span<Entry>(entries).last(
        entries.size() < lookback ? entries.size() : lookback);
}

struct BackwardResolveKey
{
    int x = 0;
    int y = 0;
    int dst = -1;

    bool operator<(const BackwardResolveKey &other) const
    {
        if (x != other.x)
        {
            return x < other.x;
        }

        if (y != other.y)
        {
            return y < other.y;
        }

        return dst < other.dst;
    }

    bool operator==(const BackwardResolveKey &other) const
    {
        return x == other.x && y == other.y && dst == other.dst;
    }

};

struct BackwardResolveKeyHash
{
    size_t operator()(const BackwardResolveKey &key) const
    {
        size_t hash = static_cast<size_t>(static_cast<uint32_t>(key.x));
        hash = hash * 0x9e3779b1U + static_cast<uint32_t>(key.y);
        return hash * 0x9e3779b1U + static_cast<uint32_t>(key.dst);
    }

};

struct BackwardResolveSource
{
    fpga::Tile *tile = nullptr;
    int src = -1;
    int route_jump = -1;
};

// Dedicated LOCAL transitions can land on any node kind. Keep the kind in
// the key: a LOCAL and a SRC with the same numeric id are different resources.
struct BackwardLocalKey
{
    int x = 0, y = 0;
    fpga::CBNodeNameType type = fpga::CB_NODE_LOCAL;
    int node = -1;
    bool operator==(const BackwardLocalKey &) const = default;
};

struct BackwardLocalKeyHash
{
    size_t operator()(const BackwardLocalKey &key) const
    {
        size_t hash = static_cast<uint32_t>(key.x);
        for (int value : {key.y, int(key.type), key.node})
            hash = hash * 0x9e3779b1U + static_cast<uint32_t>(value);
        return hash;
    }

};

struct BackwardLocalSource
{
    fpga::Tile *tile = nullptr;
    int local = -1;
};

using BackwardLocalSources =
    std::unordered_map<BackwardLocalKey, std::vector<BackwardLocalSource>,
                       BackwardLocalKeyHash>;

struct BackwardResolveSourceKey
{
    const fpga::CBType *type = nullptr;
    int src = -1;

    bool operator==(const BackwardResolveSourceKey &other) const
    {
        return type == other.type && src == other.src;
    }

};

struct BackwardResolveSourceKeyHash
{
    size_t operator()(const BackwardResolveSourceKey &key) const
    {
        return (reinterpret_cast<size_t>(key.type) >> 4) ^
               (static_cast<size_t>(key.src) << 1);
    }

};

struct BackwardResolveArc
{
    fpga::Coord delta;
    int dst = -1;
    int route_jump = -1;
};

// Read-only hop groups and direct reverse masks shared by angle traces.
// Constructed by Docking outside the recursive search stack.
struct BackwardTraceIndex
{
    bool prepared = false;
    std::unordered_map<BackwardResolveKey, BackwardHopChoices,
                       BackwardResolveKeyHash>
        hops;
    std::unordered_map<const fpga::CBType *, BackwardCrossbarMasks> crossbars;
    std::unordered_map<const fpga::Tile *,
                       fpga::CBType::StateTable<fpga::CBJumpState>>
        dst_directions;
};

struct BackwardResolveCache
{
    BackwardTraceIndex trace;
    BackwardLocalSources local_incoming;
    std::unordered_set<fpga::Tile *> processed_local_tiles;
    std::unordered_map<BackwardResolveSourceKey,
                       std::vector<BackwardResolveArc>,
                       BackwardResolveSourceKeyHash>
        arcs;
    std::unordered_map<BackwardResolveKey, std::vector<BackwardResolveSource>,
                       BackwardResolveKeyHash>
        incoming;
    std::unordered_set<fpga::Tile *> processed_tiles;
};

struct BackwardResolveIndex
{
    BackwardTraceIndex trace;
    BackwardLocalSources local_incoming;
    std::unordered_map<BackwardResolveKey, std::vector<BackwardResolveSource>,
                       BackwardResolveKeyHash>
        sources;
    int mapping_scan_count = 0;
    int mapping_reaches_count = 0;
    fpga::Coord center;
    int radius = 0;
    BackwardResolveCache *shared_cache = nullptr;
    std::unordered_set<BackwardResolveKey, BackwardResolveKeyHash>
        resolved_keys;
};

// Resolve every numeric SRC mapping in a bounded grid region and index it by
// the destination tile and DST node reached by that source.
BackwardResolveIndex buildBackwardResolveIndex(
    fpga::Device &device, fpga::Coord center, int radius,
    const std::function<bool(const fpga::Coord &)> &include_source = {},
    BackwardResolveCache *shared_cache = nullptr);

// Materialize one numeric destination's incoming sources from a shared cache
// into its bounded reverse-index view and return the resulting source list.
// Grounding may request the complete incoming list instead: the origin of a
// long wire can lie outside the window even when its destination is inside.
// This mode borrows the shared topology directly without changing bounded
// lookups. A non-shared index must already contain the required topology.
const std::vector<BackwardResolveSource> *
resolveBackwardSources(BackwardResolveIndex &index,
                       const BackwardResolveKey &key,
                       bool include_external_sources = false);

// Read-only terminal-state diagnostic, not a replay of search decisions.
// Supply a chip-wide reverse index to expose sources outside the docking
// window.
void dumpBackwardTerminalReport(fpga::Tile &target, NodeMask pins,
                                BackwardResolveIndex &index, int docking_radius,
                                const std::vector<rtl::Net *> &design_nets,
                                std::ostream &out,
                                NodeMask reserved_terminal_joints = {});

struct DockingBackwardAttempt
{
    int target_dst = -1;
    std::string result;
    std::vector<fpga::Wire> fragments;
};

// One numeric destination position reached independently by one side of the
// bidirectional docking search.
struct DockingFrontierNode
{
    fpga::Coord coord;
    int dst = -1;
    int depth = 0;
};

// One concrete edge joining the forward and backward frontiers whose live
// leases prevented docking. The flags identify only resources to be released.
struct DockingBridgeBlocker
{
    // A complete provisional path can require cuts in several crossbars.
    std::vector<fpga::RouteCutNode> provisional_cuts;
    bool valid = false;
    bool joins_frontiers = false;
    // Diagnostic frontier from a blocked terminal is not a one-cut route proof.
    bool blocked_terminal = false;
    fpga::Coord tile;
    int dst = -1;
    int src = -1;
    int joint = -1;
    int joint2 = -1;
    fpga::Coord landing_tile;
    int landing_dst = -1;
    bool dst_busy = false;
    bool src_busy = false;
    bool joint_busy = false;
    bool joint2_busy = false;
    std::vector<fpga::Wire> forward_prefix;
    std::vector<fpga::Wire> backward_suffix;
};

struct DockingResult
{
    bool success = false;
    std::vector<fpga::Wire> fragments;
    bool blocked_terminal_reachable = false;
    // Complete geometric path to the blocked terminal, for full replacement
    // validation before any of its transit owners are cut.
    std::vector<fpga::Wire> blocked_terminal_fragments;
    int blocked_dst = -1;
    int blocked_pin = -1;
    int blocked_joint = -1;
    int blocked_joint2 = -1;
    int target_seed_count = 0;
    int target_entry_count = 0;
    int target_busy_count = 0;
    int forward_push_count = 0;
    int backward_push_count = 0;
    int forward_pop_count = 0;
    int backward_pop_count = 0;
    int backward_mapping_scan_count = 0;
    int backward_mapping_reaches_count = 0;
    int backward_missing_prev_dst_count = 0;
    int backward_topology_reject_count = 0;
    int backward_busy_reject_count = 0;
    int backward_seen_reject_count = 0;
    int backward_deadend_count = 0;
    int backward_deadend_reject_count = 0;
    size_t tile_visit_reject_count = 0;
    std::vector<DockingFrontierNode> forward_frontier;
    std::vector<DockingFrontierNode> backward_frontier;
    std::vector<DockingBridgeBlocker> blocked_bridges;
    std::vector<DockingBackwardAttempt> backward_attempts;
};

// One source-local takeoff selected by a route-guided placement probe.
struct BackwardTakeoffChoice
{
    int local = -1;
    int joint = -1;
    int joint2 = -1;
    // False means the callback had no physical placement candidate at this
    // numeric frontier, so the visit must not consume the expensive-probe
    // budget.
    bool counts_toward_probe_limit = true;
};

// One already-leased partial-route landing that reverse routing may extend.
struct BackwardRouteAnchor
{
    fpga::Tile *tile = nullptr;
    int dst = -1;
    std::string dst_wire;
    size_t id = 0;
};

// A complete free route discovered from a destination pin back to a source
// tile whose local resource placement was accepted by the caller.
struct BackwardTakeoffRoute
{
    bool success = false;
    // A fully free typed chain from the mapped input reaches a physically
    // arriving DST. This includes ordinary DST->LOCAL terminals, even if the
    // later SRC trace fails; it is not an ingress-relocation diagnosis.
    bool free_terminal_ingress = false;
    // A caller released an exact transit boundary; restart with fresh leases.
    bool boundary_released = false;
    size_t boundary_releases = 0;
    bool completed_from_anchor = false;
    size_t anchor_id = 0;
    fpga::Tile *source_tile = nullptr;
    int source_src = -1;
    BackwardTakeoffChoice takeoff;
    std::vector<fpga::Wire> fragments;
    // Unleased attempted path from the last blocked hop (or deepest free
    // frontier when no occupied hop was seen) to the target.
    std::vector<fpga::Wire> diagnostic_fragments;
    size_t expanded = 0;
    size_t incoming_edges = 0;
    size_t failed_states = 0;
    size_t failure_cache_hits = 0;
    size_t source_neighborhood_edges = 0;
    size_t probe_calls = 0;
    size_t probe_candidates_scanned = 0;
    size_t takeoff_candidates = 0;
    size_t anchor_candidates = 0;
    size_t tile_visit_reject_count = 0;
    size_t probe_offset_used = 0;
    size_t blocked_reverse_edges = 0;
    // First blocked numeric alternative on the bounded reverse search. The
    // caller must validate ownership and cut eligibility before retrying.
    std::vector<fpga::RouteCutNode> blocked_reverse_cut;
    bool expansion_limit_reached = false;
    size_t remaining_frontier = 0;
    size_t diagnostic_incoming_sources = 0;
    size_t diagnostic_cached_sources = 0;
    size_t diagnostic_window_sources = 0;
    bool diagnostic_key_was_resolved = false;
    int diagnostic_depth = -1;
    bool diagnostic_depth_limit_reached = false;
    size_t diagnostic_previous_dsts = 0;
    size_t diagnostic_free_previous_dsts = 0;
    size_t diagnostic_children = 0;
    // Last blocked reverse hop, falling back to the deepest explored frontier.
    fpga::Tile *failure_tile = nullptr;
    int failure_dst = -1;
    // Preserve inspected numeric blocked alternatives so Moving
    // Sources can cut one precise transit suffix and retry the same trunk.
    std::vector<DockingBridgeBlocker> blocked_reverse_frontier;
};

// Combinatorial route candidates may pass one crossbar once or twice, but a
// third visit indicates unproductive local circulation and is rejected.
bool combinatorialRouteVisitsValid(const std::vector<fpga::Wire> &route,
                                   unsigned max_visits = 2);

// Reached angle-trace nodes are seeds; this bounded search explores only the
// sink's square and asks the caller to validate completed paths and cuts.
struct DockingGroundingSeed
{
    RoutingNode node;
    int root = -1;
    int prefix_hops = 0;
};

struct DockingGroundingSearch
{
    RoutingNode sink;
    fpga::Coord center;
    int radius = 0;
    int max_hops = 0;
    size_t max_states = 0;
    std::span<const DockingGroundingSeed> seeds;
    std::function<void(const RoutingNode &,
                       const std::function<void(const RoutingNode &)> &)>
        successors;
    std::function<bool(const RoutingNode &, const RoutingNode &)> free_edge;
    std::function<bool(const RoutingNode &, const RoutingNode &)> cut_edge;
    std::function<bool(int, std::vector<RoutingNode>)> complete;
    std::function<bool()> stopped;
};

struct DockingGroundingSearchResult
{
    bool found = false;
    bool budget_limited = false;
    bool stopped = false;
    size_t states = 0;
    size_t expanded = 0;
    size_t rejected_complete = 0;
};

DockingGroundingSearchResult
searchGroundingFromFrontier(const DockingGroundingSearch &request);

struct DockingLocalNode
{
    fpga::CBNodeNameType type = fpga::CB_NODE_LOCAL;
    int value = -1;
};

// One typed node in a bounded forward full search through a crossbar or a
// dedicated LOCAL connection. The first entry is the trace's arrival node.
struct DockingForwardNode
{
    fpga::Tile *tile = nullptr;
    fpga::CBNodeNameType type = fpga::CB_NODE_LOCAL;
    int node = -1;
};

using DockingForwardVisitor =
    std::function<bool(const std::vector<DockingForwardNode> &)>;

// Search one crossbar's numeric LOCAL/JOINT graph from its declared roots.
// The path-local node index rejects repeats; the search never changes Tile.
std::vector<DockingLocalNode> searchDockingLocalPath(
    fpga::Tile &tile, int target_local, const NodeMask &root_nodes,
    const std::function<bool(const DockingLocalNode &)> &available,
    size_t *root_count = nullptr, size_t *visited_count = nullptr);

using BackwardTakeoffProbe =
    std::function<bool(fpga::Tile &, int, BackwardTakeoffChoice &)>;
// Materialize the proposed trunk only after cheap placement/takeoff checks.
// The returned path is local to this probe; changing the choice updates
// takeoff.
using BackwardTakeoffPath = std::function<const std::vector<fpga::Wire> &(
    const BackwardTakeoffChoice &)>;
using BackwardTakeoffPathProbe = std::function<bool(
    fpga::Tile &, int, BackwardTakeoffChoice &, const BackwardTakeoffPath &)>;
using BackwardTakeoffCancel = std::function<bool()>;
using BackwardTakeoffStateView =
    std::unordered_map<fpga::Tile *, fpga::CBState>;

// Own endpoint decisions for forward sink docking and backward driver takeoff.
// The trace retains route traversal, leases, and deadend proof.
// Name one typed node in a candidate input ingress chain.
struct BackwardTerminalNode
{
    fpga::Tile *tile;
    fpga::CBNodeNameType type;
    int node;
};

// Carry terminal nodes and the fixed suffix into the reverse source walk.
struct BackwardTerminalIngress
{
    fpga::Tile *tile;
    int dst;
    int pin;
    std::vector<BackwardTerminalNode> nodes;
    std::vector<fpga::Wire> suffix;
};

struct Docking
{
    // A reverse mark can be bypassed only for a legal next-hop driver
    // placement.
    bool canPlaceAtReverseLanding(fpga::Tile &tile, int dst,
                                  const BackwardResolveIndex &index) const;

    // Retain eligible occupied endpoints for the caller's preemption pass.
    void recordBackwardSourceBlock(BackwardTakeoffRoute &result,
                                   fpga::Tile &tile, int src) const;
    void recordBackwardDstBlock(BackwardTakeoffRoute &result, fpga::Tile &tile,
                                int dst, BackwardResolveIndex &index) const;

    // Follow a longer typed chain until it reaches a source-connected DST.
    struct TerminalSearch
    {
        const Docking *docking = nullptr;
        const std::function<bool(const BackwardTerminalIngress &)> &visitor;
        size_t &remaining_expansions;

        // Sort one call's free predecessors by direction and hop length.
        struct Candidate
        {
            BackwardTerminalNode node;
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
        static fpga::Wire typed_edge(const BackwardTerminalNode &from,
                                     const BackwardTerminalNode &to)
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
            const BackwardTerminalNode &at, const BackwardTerminalNode *blocked,
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
        static bool tried_compact_path(int dst,
                                       const BackwardTerminalIngress &ingress,
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
            for (const auto &entry :
                 pnr::Docking::terminalEntries(*(target.cb_type), pin))
                if (entry.dst == dst && entry.joint == joint &&
                    entry.joint2 == joint2)
                    return true;
            return false;
        }

        // Collect a predecessor for this recursive call's ordered cursor.
        void consider(const BackwardTerminalNode &current, fpga::Tile *tile,
                      fpga::CBNodeNameType type, int node,
                      std::vector<Candidate> &choices,
                      BackwardTerminalNode &blocked, int &blocked_angle,
                      int &blocked_distance, fpga::Tile &target,
                      fpga::Coord source_hint, int pin,
                      const BackwardTerminalIngress &ingress,
                      const std::unordered_set<BackwardLocalKey,
                                               BackwardLocalKeyHash> &seen);

        // Feed physical LOCAL predecessors into the same ranked choice.
        struct PhysicalSourceVisitor
        {
            TerminalSearch &trace;
            const BackwardTerminalNode &current;
            std::vector<Candidate> &choices;
            BackwardTerminalNode &blocked;
            int &blocked_angle;
            int &blocked_distance;
            fpga::Tile &target;
            fpga::Coord source_hint;
            int pin;
            const BackwardTerminalIngress &ingress;
            const std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash>
                &seen;

            void operator()(fpga::Coord at, fpga::CBNodeNameType type,
                            int node) const;
        };

        // Each call resumes its ordered predecessors after a failed child.
        bool
        trace(fpga::Tile &target, fpga::Coord source_hint,
              BackwardResolveIndex &index, std::ostream *decisions,
              BackwardTakeoffRoute &result, bool &stopped, int pin,
              BackwardTerminalIngress &ingress,
              std::vector<fpga::Wire> &reversed,
              std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> &seen);
        // Advance one typed predecessor at a time for this input pin.
        bool run(fpga::Tile &target, fpga::Coord source_hint,
                 BackwardResolveIndex &index, std::ostream *decisions,
                 BackwardTakeoffRoute &result, bool &stopped, int pin);
        static void debug_ingress(std::ostream *out, fpga::Tile &tile,
                                  fpga::CBNodeNameType type, int node);
    };

    static void
    retainBackwardBlocked(BackwardTakeoffRoute &result,
                          const std::vector<fpga::RouteCutNode> &cuts);

    // Own terminal-search vectors and lease only this selected ingress.
    bool visitLeasedIngress(
        const BackwardTerminalIngress &ingress, fpga::Tile &target,
        BackwardTakeoffRoute &result,
        const std::function<bool(const BackwardTerminalIngress &)> &visitor)
        const;
    void visitBackwardTerminals(
        fpga::Tile &target, NodeMask pins, fpga::Coord source_hint,
        BackwardResolveIndex &index, std::ostream *decisions,
        BackwardTakeoffRoute &result, bool &stopped,
        size_t &remaining_expansions,
        const std::function<bool(const BackwardTerminalIngress &)> &visitor)
        const;

    struct DriverStateScope
    {
        std::vector<std::pair<fpga::Tile *, fpga::CBState>> saved;
        explicit DriverStateScope(const Docking &docking);
        ~DriverStateScope();
    };

    struct TerminalLeaseScope
    {
        std::span<const BackwardTerminalNode> nodes;
        bool leased = false;
        ~TerminalLeaseScope();
    };

    static NodeMask &terminalMask(fpga::Tile &tile, fpga::CBNodeNameType type);

    // Only scalar cursors are local to a backward Tile-hop invocation.
    struct IncomingCursor
    {
        const BackwardHopChoices *choices = nullptr;
        uint16_t angle_rank = 0;
        uint32_t next = 0;
        uint32_t end = 0;
    };

    struct DstCursor
    {
        NodeMask remaining;
        NodeMask bucket;
        uint16_t angle_rank = 0;
        bool initialized = false;
    };

    static int nextBackwardDst(const BackwardResolveIndex &index,
                               const fpga::Tile &tile, fpga::Coord angle,
                               DstCursor &cursor, const NodeMask &allowed);

    static void prepareBackwardTrace(BackwardResolveIndex &index);
    static IncomingCursor incomingCursor(const BackwardResolveIndex &index,
                                         BackwardResolveKey key);
    static const BackwardResolveSource *nextIncoming(IncomingCursor &cursor,
                                                     fpga::Coord angle);
    static const BackwardCrossbarMasks *
    reverseMasks(const BackwardResolveIndex &index, const fpga::CBType &type);

    // Docking owns access to vector enumeration; repair uses these entrypoints.
    static const std::vector<uint16_t> *srcNodes(const fpga::CBType &type,
                                                 fpga::CBNodeNameType from_type,
                                                 int from_value)
    {
        return FullSearch::srcNodes(type, from_type, from_value);
    }

    static const std::vector<uint16_t> &orderedSrcNodes(fpga::CBType &type,
                                                        fpga::Coord angle)
    {
        return FullSearch::orderedSrcNodes(type, angle);
    }

    static const std::vector<uint16_t> &
    orderedSrcNodes(fpga::CBType &type, fpga::CBNodeNameType from_type,
                    int from_value, fpga::Coord angle)
    {
        return FullSearch::orderedSrcNodes(type, from_type, from_value, angle);
    }

    static const std::vector<fpga::CBType::TerminalEntry> &
    terminalEntries(fpga::CBType &type, int local)
    {
        return FullSearch::terminalEntries(type, local);
    }

    // Export and diagnostics read loaded aliases through Docking's full view.
    static const fpga::CBConnName *
    connName(const fpga::CBType &type, fpga::CBNodeNameType from_type,
             int from_value, fpga::CBNodeNameType to_type, int to_value)
    {
        return FullSearch::connName(type, from_type, from_value, to_type,
                                    to_value);
    }

    static const std::vector<fpga::CBConnName> *
    connNames(const fpga::CBType &type, fpga::CBNodeNameType from_type,
              int from_value, fpga::CBNodeNameType to_type, int to_value)
    {
        return FullSearch::connNames(type, from_type, from_value, to_type,
                                     to_value);
    }

    // A reached DST can complete a movable sink or a partial route suffix.
    using SinkProbe =
        std::function<bool(fpga::Tile &, int, bool, std::vector<fpga::Wire> &)>;
    // A reached LOCAL can complete an already placed sink pin.
    using LocalTerminalProbe =
        std::function<bool(fpga::Tile &, int, std::vector<fpga::Wire> &)>;
    // A blocked DST may be retained for a later targeted cut; this callback
    // must not change leases during the trace.
    using BlockedArrivalProbe = std::function<void(fpga::Tile &, int)>;

    SinkProbe sink;
    LocalTerminalProbe local_terminal;
    BlockedArrivalProbe blocked_arrival;
    // Backward relocation borrows these for the duration of one trace.
    const BackwardTakeoffProbe *driver_probe = nullptr;
    const BackwardTakeoffCancel *driver_cancel = nullptr;
    const BackwardTakeoffStateView *driver_state = nullptr;
    // Fixed-endpoint callers borrow their mapped sink Tiles during tracing.
    // An empty view leaves placement selection unrestricted.
    std::span<fpga::Tile *const> terminal_tiles;

    // Moving supplies placement eligibility independently of route completion.
    using PlacementAllowed = std::function<bool(fpga::Tile &)>;
    PlacementAllowed placement_allowed;

    bool canPlaceIn(fpga::Tile &tile) const;
    bool canContinueIn(const fpga::Tile &tile) const;
    bool hasLocalTerminal() const;
    bool canEnterReservedLocal(fpga::Tile &tile, int node) const;
    bool blocksReservedTransit(fpga::Tile &tile, int node, bool root) const;
    bool tryTerminal(fpga::Tile &tile, fpga::CBNodeNameType type, int node,
                     bool root_dst, std::vector<fpga::Wire> &terminal) const;
    // Append a completed terminal to the caller-owned path, without a
    // trace-local vector.
    bool appendTerminal(fpga::Tile &tile, fpga::CBNodeNameType type, int node,
                        bool root_dst, bool terminal_only,
                        std::vector<fpga::Wire> &path) const;

    // Exhaust longer LOCAL/JOINT chains and dedicated LOCAL links after the
    // angle trace's direct exits fail. Repeated typed nodes are excluded here.
    bool searchForwardInternal(fpga::Tile &tile, fpga::CBNodeNameType type,
                               int node,
                               const DockingForwardVisitor &visit,
                               bool &limited) const;

    void recordBlockedArrival(fpga::Tile &tile, int dst) const;
    bool hasDriver() const;
    bool tryDriver(fpga::Tile &tile, int src,
                   BackwardTakeoffChoice &choice) const;
    bool driverStopped() const;
    const fpga::CBState *driverStateOverride(fpga::Tile &tile) const;
    const fpga::CBState &driverState(fpga::Tile &tile) const;
};

// Moving Sources probes placement after each depth layer, but requests a live
// transit cut only after exhausting free continuations and placement probes.
// True ends the search immediately; budgets and cancellation never permit a
// cut.
using BackwardTakeoffBoundary =
    std::function<bool(const std::vector<DockingBridgeBlocker> &)>;
// Refresh the caller's lease/input views after a cut. Changed tiles reopen
// blocked transitions in the SAME walk; false returns to the legacy caller.
using BackwardTakeoffResume = std::function<bool(std::vector<fpga::Tile *> &)>;

// Search destination-to-source through the numeric reverse jump index. Zero
// depth or expansion limits leave the stage deadline as the only search bound.
// A rejected driver placement does not exhaust the SRC's forward paths.
// Full reverse Takeoff reads DST deadends and never writes either mask.
BackwardTakeoffRoute routeBackwardToTakeoff(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int max_depth, int radius, int source_radius,
    const BackwardTakeoffProbe &probe,
    BackwardResolveIndex *backward_index = nullptr,
    const BackwardTakeoffCancel &cancel = {}, size_t max_expansions = 32768,
    size_t probe_offset = 0, size_t max_probes = 64,
    const BackwardTakeoffStateView *state_view = nullptr,
    const std::vector<BackwardRouteAnchor> *anchors = nullptr,
    const BackwardTakeoffProbe &preferred_probe = {},
    const BackwardTakeoffPathProbe &path_probe = {},
    const BackwardTakeoffBoundary &boundary_probe = {},
    const BackwardTakeoffResume &resume_boundary = {});

// Prove an input using its fixed driver or retained tree, with an optional
// hop-depth limit and caller cancellation, but no expansion/probe count limit.
BackwardTakeoffRoute routeBackwardToInput(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int radius, const BackwardTakeoffProbe &source_probe,
    BackwardResolveIndex *backward_index, const BackwardTakeoffCancel &cancel,
    const BackwardTakeoffStateView *state_view,
    const std::vector<BackwardRouteAnchor> *anchors, int max_depth = 0);

// Search destination-to-source until a free suffix reaches an existing
// partial-route landing; no placement or source-local probe is performed.
BackwardTakeoffRoute routeBackwardToAnchor(
    fpga::Tile &target_tile, NodeMask pin_nodes,
    const BackwardRouteAnchor &anchor, int max_depth, int radius,
    BackwardResolveIndex *backward_index = nullptr,
    const BackwardTakeoffCancel &cancel = {}, size_t max_expansions = 32768);

// Search in breadth-first hop order for a reachable retained prefix landing.
// For duplicate numeric anchors, keep the first (newest) prefix entry.
BackwardTakeoffRoute routeBackwardToAnchors(
    fpga::Tile &target_tile, NodeMask pin_nodes,
    const std::vector<BackwardRouteAnchor> &anchors, int max_depth, int radius,
    BackwardResolveIndex *backward_index = nullptr,
    const BackwardTakeoffCancel &cancel = {}, size_t max_expansions = 32768);

// Join the already-proven free forward and backward paths through one exact
// bridge after its transit owners have been removed.
bool materializeDockingBridge(const DockingBridgeBlocker &bridge,
                              DockingResult &result);

// Bidirectional grounding fallback: connect a routed forward frontier to one
// destination-entry rail by exploring only loaded crossbar masks and jump
// deltas. The default hop budget is twice the radius for each search direction.
// An explicit max_depth is retained for bounded diagnostic probes.
constexpr int dockingHopLimit(int radius)
{
    return 2 * radius;
}

// Lease-preserving second phase, used only after the free search failed. The
// first callback verifies only the newly encountered edge's occupied resources;
// the second validates the entire replacement (including the caller's forward
// prefix). Neither callback may cut victims or retain temporary leases.
// Docking reads directional deadends without adding marks.
// source_local selects a complete reverse search ending at this exact output
// LOCAL, including Takeoff. Otherwise forward_dst denotes an existing DST.
DockingResult dockGrounding(
    fpga::Tile &forward_tile, int forward_dst,
    const std::string &forward_dst_wire, fpga::Tile &target_tile,
    NodeMask pin_nodes, int max_depth = -1, int radius = fpga::GROUNDING_RADIUS,
    bool trace_backward_attempts = false,
    NodeMask reserved_terminal_joints = {},
    BackwardResolveIndex *backward_index = nullptr, bool source_local = false,
    const DockingPreemptionSearch *preemption = nullptr);

// I/O endpoint docking uses the same bounded full search with a wider window.
DockingResult dockIOB(fpga::Tile &forward_tile, int forward_dst,
                      const std::string &forward_dst_wire,
                      fpga::Tile &target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts = false,
                      NodeMask reserved_terminal_joints = {});

// Exact physical predecessors for the numeric reverse trace. LOCAL links
// retain their literal grid coordinates.
void visitBackwardPhysicalSources(
    BackwardResolveIndex &index, fpga::Coord at, fpga::CBNodeNameType type,
    int node, const fpga::RepairIncomingSourceVisitor &visit);

} // namespace pnr
