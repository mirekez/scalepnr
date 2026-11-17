#pragma once

#include "NodeMask.h"
#include "Tile.h"
#include "Wire.h"

#include <functional>
#include <iosfwd>
#include <span>
#include <optional>
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

struct BackwardResolveCache
{
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

using DockingNumericNode = std::tuple<int, int, int, int>;

// Validate one bounded Grounding completion and preview every required cut.
// Exact nodes may not repeat, while Tile visits count only the Docking tail.
struct DockingGroundingCandidate
{
    std::span<const DockingNumericNode> prefix;
    std::span<const DockingNumericNode> tail;
    int prefix_hops = 0;
    int max_hops = 0;
    std::optional<DockingNumericNode> required_branch;
    std::function<bool(const DockingNumericNode &,
                       const DockingNumericNode &,
                       const std::vector<fpga::RouteCutNode> &)> allowed;
    std::function<bool(const std::vector<fpga::RouteCutNode> &,
                       std::vector<fpga::RouteCutNode> &)> preview;
};

struct DockingGroundingCandidateResult
{
    bool success = false;
    int hops = 0;
    std::vector<DockingNumericNode> path;
    std::vector<fpga::RouteCutNode> cuts;
    std::vector<fpga::RouteCutNode> released;
};

DockingGroundingCandidateResult tryDockingGroundingCandidate(
    const DockingGroundingCandidate &request);

// Reached angle-trace nodes are seeds; this bounded search explores only the
// sink's square and asks the caller to validate completed paths and cuts.
struct DockingGroundingSeed
{
    DockingNumericNode node;
    int root = -1;
    int prefix_hops = 0;
};

struct DockingGroundingSearch
{
    DockingNumericNode sink;
    fpga::Coord center;
    int radius = 0;
    int max_hops = 0;
    size_t max_states = 0;
    std::span<const DockingGroundingSeed> seeds;
    std::function<void(const DockingNumericNode &,
                       const std::function<void(const DockingNumericNode &)> &)>
        successors;
    std::function<bool(const DockingNumericNode &, const DockingNumericNode &)>
        free_edge;
    std::function<bool(const DockingNumericNode &, const DockingNumericNode &)>
        cut_edge;
    std::function<bool(int, std::vector<DockingNumericNode>)> complete;
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

DockingGroundingSearchResult searchGroundingFromFrontier(
    const DockingGroundingSearch &request);

struct DockingLocalNode
{
    fpga::CBNodeNameType type = fpga::CB_NODE_LOCAL;
    int value = -1;
};

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
struct Docking
{
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

    bool hasLocalTerminal() const;
    bool canEnterReservedLocal(fpga::Tile &tile, int node) const;
    bool blocksReservedTransit(fpga::Tile &tile, int node, bool root) const;
    bool tryTerminal(fpga::Tile &tile, fpga::CBNodeNameType type, int node,
                     bool root_dst, std::vector<fpga::Wire> &terminal) const;
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
// Persistent SRC marking waits for the placement probe and all predecessor
// nodes. Provisional leases and capped probe windows cannot publish deadends.
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
// Exhausted SRC choices may still receive persistent deadend marks.
struct DockingPreemptionSearch
{
    std::function<bool(const fpga::Wire &)> eligible_edge;
    std::function<bool(const std::vector<fpga::Wire> &)> accept_complete;
    // Production exhausts its existing full-task alternatives before enabling
    // occupied forward growth; standalone probes can exercise both directions.
    bool allow_forward_expansion = true;
};

// source_local selects a complete reverse search ending at this exact output
// LOCAL, including Takeoff. Otherwise forward_dst denotes an existing DST.
DockingResult dockGrounding(
    fpga::Tile &forward_tile, int forward_dst,
    const std::string &forward_dst_wire, fpga::Tile &target_tile,
    NodeMask pin_nodes, int max_depth = -1, int radius = fpga::GROUNDING_RADIUS,
    bool trace_backward_attempts = false,
    NodeMask reserved_terminal_joints = {},
    BackwardResolveIndex *backward_index = nullptr,
    bool source_local = false,
    const DockingPreemptionSearch *preemption = nullptr);

// I/O endpoint docking uses the same bounded full search with a wider window.
DockingResult dockIOB(fpga::Tile &forward_tile, int forward_dst,
                      const std::string &forward_dst_wire,
                      fpga::Tile &target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts = false,
                      NodeMask reserved_terminal_joints = {});

} // namespace pnr
