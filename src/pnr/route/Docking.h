#pragma once

#include "NodeMask.h"
#include "Tile.h"
#include "Wire.h"

#include <functional>
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
  std::vector<DockingFrontierNode> forward_frontier;
  std::vector<DockingFrontierNode> backward_frontier;
  std::vector<DockingBridgeBlocker> blocked_bridges;
  std::vector<DockingBackwardAttempt> backward_attempts;
};

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
