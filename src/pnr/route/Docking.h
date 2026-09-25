#pragma once

#include "NodeMask.h"
#include "Tile.h"
#include "Wire.h"

#include <functional>
#include <iosfwd>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fpga {
struct Device;
}

namespace pnr {

struct BackwardResolveKey {
  int x = 0;
  int y = 0;
  int dst = -1;

  bool operator<(const BackwardResolveKey &other) const {
    if (x != other.x) {
      return x < other.x;
    }
    if (y != other.y) {
      return y < other.y;
    }
    return dst < other.dst;
  }

  bool operator==(const BackwardResolveKey &other) const {
    return x == other.x && y == other.y && dst == other.dst;
  }
};

struct BackwardResolveKeyHash {
  size_t operator()(const BackwardResolveKey &key) const {
    size_t hash = static_cast<size_t>(static_cast<uint32_t>(key.x));
    hash = hash * 0x9e3779b1U + static_cast<uint32_t>(key.y);
    return hash * 0x9e3779b1U + static_cast<uint32_t>(key.dst);
  }
};

struct BackwardResolveSource {
  fpga::Tile *tile = nullptr;
  int src = -1;
  int route_jump = -1;
};

struct BackwardResolveSourceKey {
  const fpga::CBType *type = nullptr;
  int src = -1;

  bool operator==(const BackwardResolveSourceKey &other) const {
    return type == other.type && src == other.src;
  }
};

struct BackwardResolveSourceKeyHash {
  size_t operator()(const BackwardResolveSourceKey &key) const {
    return (reinterpret_cast<size_t>(key.type) >> 4) ^
           (static_cast<size_t>(key.src) << 1);
  }
};

struct BackwardResolveArc {
  fpga::Coord delta;
  int dst = -1;
  int route_jump = -1;
};

struct BackwardResolveCache {
  std::unordered_map<BackwardResolveSourceKey,
                     std::vector<BackwardResolveArc>,
                     BackwardResolveSourceKeyHash>
      arcs;
  std::unordered_map<BackwardResolveKey,
                     std::vector<BackwardResolveSource>,
                     BackwardResolveKeyHash>
      incoming;
  std::unordered_set<fpga::Tile *> processed_tiles;
};

struct BackwardResolveIndex {
  std::unordered_map<BackwardResolveKey, std::vector<BackwardResolveSource>,
                     BackwardResolveKeyHash>
      sources;
  int mapping_scan_count = 0;
  int mapping_reaches_count = 0;
  fpga::Coord center;
  int radius = 0;
  BackwardResolveCache *shared_cache = nullptr;
  std::unordered_set<BackwardResolveKey, BackwardResolveKeyHash> resolved_keys;
};

// Resolve every numeric SRC mapping in a bounded grid region and index it by
// the destination tile and DST node reached by that source.
BackwardResolveIndex buildBackwardResolveIndex(
    fpga::Device &device, fpga::Coord center, int radius,
    const std::function<bool(const fpga::Coord &)> &include_source = {},
    BackwardResolveCache *shared_cache = nullptr);

// Materialize one numeric destination's incoming sources from a shared cache
// into its bounded reverse-index view and return the resulting source list.
const std::vector<BackwardResolveSource> *
resolveBackwardSources(BackwardResolveIndex &index,
                       const BackwardResolveKey &key);

// Read-only terminal-state diagnostic, not a replay of search decisions.
// Supply a chip-wide reverse index to expose sources outside the docking window.
void dumpBackwardTerminalReport(
    fpga::Tile &target, NodeMask pins, BackwardResolveIndex &index,
    int docking_radius, const std::vector<rtl::Net *> &design_nets,
    std::ostream &out, NodeMask reserved_terminal_joints = {});

struct DockingBackwardAttempt {
  int target_dst = -1;
  std::string result;
  std::vector<fpga::Wire> fragments;
};

// One numeric destination position reached independently by one side of the
// bidirectional docking search.
struct DockingFrontierNode {
  fpga::Coord coord;
  int dst = -1;
  int depth = 0;
};

// One concrete edge joining the forward and backward frontiers whose live
// leases prevented docking. The flags identify only resources to be released.
struct DockingBridgeBlocker {
  bool valid = false;
  bool joins_frontiers = false;
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

struct DockingResult {
  bool success = false;
  std::vector<fpga::Wire> fragments;
  bool blocked_terminal_reachable = false;
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
struct BackwardTakeoffChoice {
  int local = -1;
  int joint = -1;
  int joint2 = -1;
  // False means the callback had no physical placement candidate at this
  // numeric frontier, so the visit must not consume the expensive-probe budget.
  bool counts_toward_probe_limit = true;
};

// One already-leased partial-route landing that reverse routing may extend.
struct BackwardRouteAnchor {
  fpga::Tile *tile = nullptr;
  int dst = -1;
  std::string dst_wire;
  size_t id = 0;
};

// A complete free route discovered from a destination pin back to a source
// tile whose local resource placement was accepted by the caller.
struct BackwardTakeoffRoute {
  bool success = false;
  // A caller released an exact transit boundary; restart with fresh leases.
  bool boundary_released = false;
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
  size_t source_neighborhood_edges = 0;
  size_t probe_calls = 0;
  size_t probe_candidates_scanned = 0;
  size_t takeoff_candidates = 0;
  size_t anchor_candidates = 0;
  size_t tile_visit_reject_count = 0;
  size_t probe_offset_used = 0;
  size_t blocked_reverse_edges = 0;
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
  // Preserve a bounded numeric boundary of occupied reverse edges so Moving
  // Sources can cut one precise transit suffix and retry the same trunk.
  std::vector<DockingBridgeBlocker> blocked_reverse_frontier;
};

// Combinatorial route candidates may pass one crossbar once or twice, but a
// third visit indicates unproductive local circulation and is rejected.
bool combinatorialRouteVisitsValid(const std::vector<fpga::Wire> &route,
                                   unsigned max_visits = 2);

using BackwardTakeoffProbe =
    std::function<bool(fpga::Tile &, int, BackwardTakeoffChoice &)>;
// Materialize the proposed trunk only after cheap placement/takeoff checks.
// The returned path is local to this probe; changing the choice updates takeoff.
using BackwardTakeoffPath = std::function<const std::vector<fpga::Wire> &(
    const BackwardTakeoffChoice &)>;
using BackwardTakeoffPathProbe = std::function<bool(
    fpga::Tile &, int, BackwardTakeoffChoice &, const BackwardTakeoffPath &)>;
using BackwardTakeoffCancel = std::function<bool()>;
using BackwardTakeoffStateView = std::unordered_map<fpga::Tile *, fpga::CBState>;
// Moving Sources probes placement after each complete depth layer, then may
// release one blocked transit boundary. True stops this speculative search.
using BackwardTakeoffBoundary =
    std::function<bool(const std::vector<DockingBridgeBlocker> &)>;

// Search destination-to-source through the numeric reverse jump index. Zero
// depth or expansion limits leave the stage deadline as the only search bound.
BackwardTakeoffRoute routeBackwardToTakeoff(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int max_depth, int radius, int source_radius,
    const BackwardTakeoffProbe &probe,
    BackwardResolveIndex *backward_index = nullptr,
    const BackwardTakeoffCancel &cancel = {},
    size_t max_expansions = 32768, size_t probe_offset = 0,
    size_t max_probes = 64,
    const BackwardTakeoffStateView *state_view = nullptr,
    const std::vector<BackwardRouteAnchor> *anchors = nullptr,
    const BackwardTakeoffProbe &preferred_probe = {},
    const BackwardTakeoffPathProbe &path_probe = {},
    const BackwardTakeoffBoundary &boundary_probe = {});

// Prove an input using its fixed driver or retained tree, bounded only by the
// caller's cancellation deadline, not an arbitrary expansion/probe count.
BackwardTakeoffRoute routeBackwardToInput(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int radius, const BackwardTakeoffProbe &source_probe,
    BackwardResolveIndex *backward_index,
    const BackwardTakeoffCancel &cancel,
    const BackwardTakeoffStateView *state_view,
    const std::vector<BackwardRouteAnchor> *anchors);

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
// deltas.
DockingResult
dockGrounding(fpga::Tile &forward_tile, int forward_dst,
              const std::string &forward_dst_wire, fpga::Tile &target_tile,
              NodeMask pin_nodes, int max_depth = 5, int radius = 5,
              bool trace_backward_attempts = false,
              NodeMask reserved_terminal_joints = {},
              BackwardResolveIndex *backward_index = nullptr);

// I/O endpoint docking uses the same mask-only transitions with a wider
// edge-interface window, expanded as a direction-led beam toward the endpoint.
DockingResult dockIOB(fpga::Tile &forward_tile, int forward_dst,
                      const std::string &forward_dst_wire,
                      fpga::Tile &target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts = false,
                      NodeMask reserved_terminal_joints = {});

} // namespace pnr
