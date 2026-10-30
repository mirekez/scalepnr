#include "Docking.h"

#include "Device.h"
#include "RouteSearch.h"
#include "RouteDiagnostics.h"

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <ostream>
#include <string>
#include <unordered_set>

namespace pnr {
namespace {

constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;

bool inDockWindow(const fpga::Coord &coord, const fpga::Coord &target,
                  int radius) {
  return std::abs(coord.x - target.x) <= radius &&
         std::abs(coord.y - target.y) <= radius;
}

bool canLeaseJump(const fpga::CBState &cb, int dst, int src, int joint,
                  int joint2, bool allow_existing_dst = false) {
  if (cb.src.jump.testBit(src)) {
    return false;
  }
  if (!allow_existing_dst && cb.dst.jump.testBit(dst)) {
    return false;
  }
  if ((joint >= 0 && cb.joint.jump.testBit(joint)) ||
      (joint2 >= 0 && cb.joint.jump.testBit(joint2))) {
    return false;
  }
  return true;
}

bool canLeaseIn(const fpga::CBState &cb, int dst, int local, int joint,
                int joint2) {
  if (cb.dst.jump.testBit(dst) || cb.local.local.testBit(local) ||
      (joint >= 0 && cb.joint.jump.testBit(joint)) ||
      (joint2 >= 0 && cb.joint.jump.testBit(joint2))) {
    return false;
  }
  return true;
}

int selectJointToSrc(const fpga::Tile &tile, fpga::CBNodeNameType from_type,
                     int from_node, int src, int *joint2, bool free_joints_only = false) {
  if (!tile.cb_type) {
    return -2;
  }
  tile.cb_type->ensureDerivedMasks();
  int joint = -1;
  if (joint2) {
    *joint2 = -1;
  }
  // Free placement growth must exhaust alternate joints before asking for a
  // transit cut; the topology-only selector can return an occupied first joint.
  if (free_joints_only && (from_type == fpga::CB_NODE_DST || from_type == fpga::CB_NODE_LOCAL)) {
    const auto &direct = from_type == fpga::CB_NODE_DST
        ? tile.cb_type->dst_src.get(from_node).jump : tile.cb_type->local_src.get(from_node).jump;
    if (direct.testBit(src)) return -1;
    const NodeMask starts = (from_type == fpga::CB_NODE_DST
        ? tile.cb_type->dst_joint.get(from_node).joint : tile.cb_type->local_joint.get(from_node).joint)
        & ~tile.cb.joint.jump;
    const NodeMask ends = tile.cb_type->src_reachable_joints.get(src).joint & ~tile.cb.joint.jump;
    joint = (starts & ends).firstSetBit();
    if (joint >= 0) return joint;
    starts.for_each_set_bit([&](int first) {
      joint = (tile.cb_type->joint_joint.get(first).joint & ends).firstSetBit();
      if (joint < 0) return false;
      if (joint2) *joint2 = first;
      return true;
    });
    return joint >= 0 ? joint : -2;
  }
  if (from_type == fpga::CB_NODE_DST) {
    return tile.cb_type->canJump(from_node, src, src, joint, joint2) ? joint
                                                                     : -2;
  }
  if (from_type == fpga::CB_NODE_LOCAL) {
    return tile.cb_type->canOut(from_node, src, src, joint, joint2) ? joint
                                                                    : -2;
  }
  if (tile.cb_type->joint_src.get(from_node).jump.testBit(src)) {
    return -1;
  }
  NodeMask joints_to_src = tile.cb_type->src_reachable_joints.get(src).joint;
  joint = (tile.cb_type->joint_joint.get(from_node).joint & joints_to_src)
              .firstSetBit();
  return joint >= 0 ? joint : -2;
}

fpga::InternedString srcWireName(const fpga::Tile &tile, fpga::CBNodeNameType from_type,
                        int from_node, int src, int joint,
                        const std::string &incoming_wire) {
  if (!tile.cb_type) {
    return {};
  }
  if (joint >= 0) {
    if (const fpga::CBConnName *conn = tile.cb_type->connName(
            fpga::CB_NODE_JOINT, joint, fpga::CB_NODE_SRC, src)) {
      return conn->to;
    }
    if (const fpga::CBConnName *conn = tile.cb_type->connName(
            fpga::CB_NODE_SRC, src, fpga::CB_NODE_JOINT, joint)) {
      return conn->from;
    }
  }
  if (const fpga::CBConnName *conn = tile.cb_type->connName(
          from_type, from_node, fpga::CB_NODE_SRC, src)) {
    if (!incoming_wire.empty() && conn->from != incoming_wire) {
      return {};
    }
    return conn->to;
  }
  if (!incoming_wire.empty()) {
    return {};
  }
  if (const std::string *name =
          tile.cb_type->nodeName(fpga::CB_NODE_SRC, src)) {
    return *name;
  }
  return {};
}

fpga::InternedString fromWireName(const fpga::Tile &tile, fpga::CBNodeNameType from_type,
                         int from_node, int src, int joint,
                         const std::string &src_wire) {
  if (!tile.cb_type) {
    return {};
  }
  if (joint >= 0) {
    if (const fpga::CBConnName *conn = tile.cb_type->connName(
            from_type, from_node, fpga::CB_NODE_JOINT, joint)) {
      return conn->from;
    }
  }
  if (const fpga::CBConnName *conn = tile.cb_type->connName(
          from_type, from_node, fpga::CB_NODE_SRC, src)) {
    if (!src_wire.empty() && conn->to != src_wire) {
      return {};
    }
    return conn->from;
  }
  if (const std::string *name = tile.cb_type->nodeName(from_type, from_node)) {
    return *name;
  }
  return {};
}

using Key = BackwardResolveKey;

struct Node {
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
};

struct TakeoffProbeKey {
  fpga::Tile *tile = nullptr;
  int src = -1;

  bool operator==(const TakeoffProbeKey &) const = default;
};

struct TakeoffProbeKeyHash {
  size_t operator()(const TakeoffProbeKey &key) const {
    size_t tile = std::hash<fpga::Tile *>{}(key.tile);
    return tile ^ (std::hash<int>{}(key.src) + 0x9e3779b9U + (tile << 6) +
                   (tile >> 2));
  }
};

using BackwardSource = BackwardResolveSource;

struct DockingOptions {
  int max_depth = 5;
  int radius = fpga::GROUNDING_RADIUS;
  bool directional = false;
  int side_limit = 0;
  int candidate_limit = 0;
  bool trace_backward_attempts = false;
  NodeMask reserved_terminal_joints{};
  BackwardResolveIndex *prebuilt_backward_index = nullptr;
  std::span<const fpga::Coord> prefix_tiles;
  // Local coverage is tried separately before admitting outside predecessors.
  bool bounded_reverse = false;
  bool source_local = false;
  const DockingPreemptionSearch *preemption = nullptr;
  bool forward_preemption = false;
};

struct PendingBridgeBlocker {
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

Key nodeKey(const Node &node) {
  return Key{node.tile->coord.x, node.tile->coord.y, node.dst};
}

std::vector<fpga::Wire> pathToNode(const std::vector<Node> &nodes, int index) {
  std::vector<fpga::Wire> result;
  for (int curr = index; curr >= 0 && nodes[curr].parent >= 0;
       curr = nodes[curr].parent) {
    result.push_back(nodes[curr].edge_from_parent);
  }
  std::reverse(result.begin(), result.end());
  return result;
}

std::vector<fpga::Wire> suffixFromNode(const std::vector<Node> &nodes,
                                       int index) {
  std::vector<fpga::Wire> result;
  int root = index;
  for (int curr = index; curr >= 0 && nodes[curr].parent >= 0;
       curr = nodes[curr].parent) {
    result.push_back(nodes[curr].edge_from_parent);
    root = nodes[curr].parent;
  }
  if (root >= 0 && static_cast<size_t>(root) < nodes.size()) {
    result.insert(result.end(), nodes[root].target_suffix.begin(),
                  nodes[root].target_suffix.end());
  }
  return result;
}

int crossToLine(const fpga::Coord &start, const fpga::Coord &target,
                const fpga::Coord &point) {
  fpga::Coord line = target - start;
  fpga::Coord offset = point - start;
  return std::abs(line.x * offset.y - line.y * offset.x);
}

bool inDirectionalCorridor(const fpga::Coord &coord, const fpga::Coord &start,
                           const fpga::Coord &target,
                           const DockingOptions &options) {
  if (!options.directional) {
    return true;
  }
  fpga::Coord line = target - start;
  int norm = std::max(std::abs(line.x), std::abs(line.y));
  if (norm == 0) {
    return true;
  }
  return crossToLine(start, target, coord) <= options.side_limit * norm;
}

DockingResult dockGroundingImpl(fpga::Tile &forward_tile, int forward_dst,
                                const std::string &forward_dst_wire,
                                fpga::Tile &target_tile, NodeMask pin_nodes,
                                const DockingOptions &options) {
  DockingResult result;
  if (!forward_tile.cb_type || !target_tile.cb_type || forward_dst < 0 ||
      pin_nodes == NodeMask{}) {
    return result;
  }

  std::vector<Node> forward_nodes;
  std::vector<Node> backward_nodes;
  CombinatorialTileVisits tile_visits;
  // Include the committed trunk/suffix AND the caller's speculative path to
  // this anchor. Adjacent same-CB steps do not represent Tile reentries.
  int prefix_root = -1;
  fpga::Coord prefix_last{-1, -1};
  for (auto coord : options.prefix_tiles) {
    if (coord == prefix_last) continue;
    if (!tile_visits.append(prefix_root, coord, prefix_root)) {
      ++result.tile_visit_reject_count;
      return result;
    }
    prefix_last = coord;
  }
  if (prefix_last != forward_tile.coord &&
      !tile_visits.append(prefix_root, &forward_tile, prefix_root)) {
    ++result.tile_visit_reject_count;
    return result;
  }
  auto path_valid = [&](const std::vector<fpga::Wire> &path) {
    const size_t checkpoint = tile_visits.checkpoint();
    int root = prefix_root;
    fpga::Coord last = forward_tile.coord;
    bool valid = true;
    for (const auto &wire : path) {
      for (auto coord : {wire.from, wire.to}) {
        if (coord == last) continue;
        if (!tile_visits.append(root, coord, root)) { valid = false; break; }
        last = coord;
      }
      if (!valid) break;
    }
    tile_visits.rewind(checkpoint);
    return valid && (!options.preemption ||
        options.preemption->accept_complete(path));
  };
  // The reverse suffix must fit alongside the retained history. Its one
  // occurrence of the anchor Tile can coincide with the prefix's last visit;
  // a completed join below checks the exact concatenation as well.
  auto backward_fits_prefix = [&](int root, fpga::Coord coord) {
    return tile_visits.count(prefix_root, coord) + tile_visits.count(root, coord) <=
        COMBINATORIAL_TILE_VISIT_LIMIT + (coord == forward_tile.coord ? 1u : 0u);
  };
  std::deque<int> forward_queue;
  std::deque<int> backward_queue;
  std::unordered_map<Key, int, BackwardResolveKeyHash> forward_seen;
  std::unordered_map<Key, int, BackwardResolveKeyHash> backward_seen;
  std::unordered_set<Key, BackwardResolveKeyHash> backward_deadends;
  std::vector<PendingBridgeBlocker> pending_bridge_blockers;
  std::unordered_set<Key, BackwardResolveKeyHash> forward_blocked_landings;
  std::vector<int> forward_width_by_depth(
      static_cast<size_t>(std::max(options.max_depth, 0)) + 1, 0);
  std::vector<int> backward_width_by_depth(
      static_cast<size_t>(std::max(options.max_depth, 0)) + 1, 0);
  std::vector<int> backward_pending_by_depth(
      static_cast<size_t>(std::max(options.max_depth, 0)) + 1, 0);
  BackwardResolveIndex local_backward_index;
  BackwardResolveIndex *backward_index = options.prebuilt_backward_index;
  bool backward_sources_built = false;

  // Reserve one next-layer slot for each unexpanded peer. Dead peers release
  // their reservation, so viable terminal seeds can use the remaining beam.
  auto reset_backward_width = [&]() {
    std::fill(backward_width_by_depth.begin(), backward_width_by_depth.end(), 0);
    std::fill(backward_pending_by_depth.begin(),
              backward_pending_by_depth.end(), 0);
    if (!backward_pending_by_depth.empty()) {
      backward_pending_by_depth[0] = static_cast<int>(backward_queue.size());
    }
  };
  auto backward_has_capacity = [&](const Node &node, int depth) {
    if (options.candidate_limit <= 0) {
      return true;
    }
    if (node.depth < 0 || depth < 0 ||
        static_cast<size_t>(node.depth) >= backward_pending_by_depth.size() ||
        static_cast<size_t>(depth) >= backward_width_by_depth.size()) {
      return false;
    }
    int remaining = options.candidate_limit - backward_width_by_depth[depth];
    return remaining > backward_pending_by_depth[node.depth];
  };
  auto record_backward_push = [&](const Node &, int depth) {
    if (options.candidate_limit > 0) {
      ++backward_width_by_depth[depth];
      ++backward_pending_by_depth[depth];
    }
  };

  // Preserve each completed backward phase and consume its exact blockers
  // before the phase-local node indexes are rebuilt for another seed class.
  auto finish_frontiers = [&]() {
    // Provisional search validates complete paths directly; it does not need
    // the legacy post-search, single-bridge materialization diagnostics.
    if (options.preemption) return;
    auto remember_frontier = [](std::vector<DockingFrontierNode> &frontier,
                                const DockingFrontierNode &node) {
      auto recent = recentDockingEntries(frontier);
      auto existing = std::find_if(
          recent.begin(), recent.end(), [&](const DockingFrontierNode &old) {
            return old.coord.x == node.coord.x && old.coord.y == node.coord.y &&
                   old.dst == node.dst;
          });
      if (existing == recent.end()) {
        frontier.push_back(node);
      } else if (node.depth < existing->depth) {
        existing->depth = node.depth;
      }
    };
    // forward_seen already contains each numeric state once. The second
    // backward phase does not expand it: replace the snapshot in linear time.
    result.forward_frontier.clear();
    result.forward_frontier.reserve(forward_seen.size());
    result.backward_frontier.reserve(result.backward_frontier.size() +
                                     backward_seen.size());
    for (const auto &[key, index] : forward_seen) {
      result.forward_frontier.push_back(
          DockingFrontierNode{{key.x, key.y}, key.dst,
                              index >= 0 ? forward_nodes[index].depth : 0});
    }
    const bool first_backward_phase = result.backward_frontier.empty();
    for (const auto &[key, index] : backward_seen) {
      DockingFrontierNode node{{key.x, key.y}, key.dst,
                              index >= 0 ? backward_nodes[index].depth : 0};
      // The first snapshot is unique too. Only later seed phases can overlap.
      if (first_backward_phase) result.backward_frontier.push_back(node);
      else remember_frontier(result.backward_frontier, node);
    }

    for (const PendingBridgeBlocker &pending : pending_bridge_blockers) {
      auto front = forward_seen.find(pending.forward_key);
      int backward_index = pending.backward_index;
      if (backward_index < 0) {
        auto back = backward_seen.find(pending.backward_key);
        if (back != backward_seen.end()) {
          backward_index = back->second;
        }
      }
      if (backward_index < 0 ||
          static_cast<size_t>(backward_index) >= backward_nodes.size() ||
          (front == forward_seen.end() && pending.from_forward)) {
        continue;
      }
      auto recent_bridges = recentDockingEntries(result.blocked_bridges);
      bool duplicate = std::any_of(
          recent_bridges.begin(), recent_bridges.end(),
          [&](const DockingBridgeBlocker &bridge) {
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
      if (duplicate) {
        continue;
      }
      DockingBridgeBlocker bridge;
      bridge.valid = true;
      bridge.joins_frontiers = front != forward_seen.end();
      bridge.blocked_terminal = backward_nodes[backward_index].blocked_terminal;
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
      if (bridge.joins_frontiers) {
        bridge.forward_prefix = pathToNode(forward_nodes, front->second);
      }
      bridge.backward_suffix = {pending.edge};
      std::vector<fpga::Wire> suffix =
          suffixFromNode(backward_nodes, backward_index);
      bridge.backward_suffix.insert(bridge.backward_suffix.end(),
                                    suffix.begin(), suffix.end());
      if (bridge.joins_frontiers) {
        auto path = bridge.forward_prefix;
        path.insert(path.end(), bridge.backward_suffix.begin(), bridge.backward_suffix.end());
        if (!path_valid(path)) {
          ++result.tile_visit_reject_count;
          continue; // Never propose cutting a victim for a cyclic replacement.
        }
      }
      result.blocked_bridges.push_back(std::move(bridge));
    }
    pending_bridge_blockers.clear();
  };

  // Keep complete backward alternatives only for a focused diagnostic run.
  auto record_backward_attempt =
      [&]<typename MakeFragments>(int seed_dst, const char *status,
                                  MakeFragments &&make_fragments) {
        if (!options.trace_backward_attempts) {
          return;
        }
        result.backward_attempts.push_back(
            DockingBackwardAttempt{seed_dst, status, make_fragments()});
      };

  Node forward_seed{&forward_tile, forward_dst, forward_dst_wire, -1, {}, 0,
                    {}};
  forward_seed.tile_visits_root = prefix_root;
  forward_nodes.push_back(std::move(forward_seed));
  // A LOCAL is not a DST, even when their numeric identifiers coincide.
  // Do not register a LOCAL under a DST key. Its legal Takeoff transitions
  // still seed the forward half; backward search can also reach it directly.
  if (!options.source_local) {
    forward_seen[nodeKey(forward_nodes.front())] = 0;
  }
  forward_queue.push_back(0);

  std::vector<Node> blocked_seeds;
  pin_nodes.for_each_set_bit([&](int pin) {
    if (target_tile.isPinNodeLeased(pin)) {
      if (RouteCongestionTrace::current)
        RouteCongestionTrace::current->blocked(target_tile, "docking_pin_leased",
            {{fpga::CB_NODE_LOCAL, pin}});
      return false;
    }
    for (const fpga::CBType::TerminalEntry &path :
         target_tile.cb_type->terminalEntries(pin)) {
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
              target_tile.cb_type->nodeName(fpga::CB_NODE_DST, dst)) {
        enter.dst_wire_name = *name;
      }
      fpga::Wire tile_pin;
      tile_pin.type = fpga::Wire::WIRE_TILE_PIN;
      tile_pin.from = target_tile.coord;
      tile_pin.to = target_tile.coord;
      tile_pin.local = pin;
      tile_pin.pos = ROUTE_POS_TRANSIT;
      std::vector<fpga::Wire> suffix{enter, tile_pin};
      const bool reserved_joint = options.reserved_terminal_joints.testBit(path.joint)
          || options.reserved_terminal_joints.testBit(path.joint2);
      bool terminal_busy =
          reserved_joint ||
          !canLeaseIn(target_tile.cb, dst, pin, path.joint, path.joint2);
      if (options.preemption && terminal_busy) {
        // Reserved packing resources and endpoint owners are never bypassed.
        if (reserved_joint ||
            !options.preemption->eligible_edge(enter)) continue;
        terminal_busy = false;
      }
      Node seed{
          &target_tile,  dst, enter.dst_wire_name, -1,         {}, 0, suffix,
          terminal_busy, pin, path.joint,          path.joint2};
      seed.seed_dst = dst;
      if (!tile_visits.append(-1, &target_tile, seed.tile_visits_root)) {
        return false;
      }
      if (!backward_fits_prefix(seed.tile_visits_root, target_tile.coord)) {
        ++result.tile_visit_reject_count;
        continue;
      }
      if (terminal_busy) {
        if (RouteCongestionTrace::current)
          RouteCongestionTrace::current->blocked(target_tile, "docking_terminal_busy",
              {{fpga::CB_NODE_DST, dst}, {fpga::CB_NODE_LOCAL, pin},
               {fpga::CB_NODE_JOINT, path.joint}, {fpga::CB_NODE_JOINT, path.joint2}});
        ++result.target_busy_count;
        record_backward_attempt(dst, "blocked_seed", [&]() { return suffix; });
        blocked_seeds.push_back(std::move(seed));
        continue;
      }
      record_backward_attempt(dst, "seed", [&]() { return suffix; });
      Key key = nodeKey(seed);
      if (backward_seen.find(key) == backward_seen.end()) {
        int seed_index = static_cast<int>(backward_nodes.size());
        backward_nodes.push_back(seed);
        backward_seen[key] = seed_index;
        backward_queue.push_back(seed_index);
        ++result.target_seed_count;
      }
      if (auto it = forward_seen.find(key); it != forward_seen.end()) {
        result.fragments = pathToNode(forward_nodes, it->second);
        result.fragments.insert(result.fragments.end(), suffix.begin(),
                                suffix.end());
        if (!path_valid(result.fragments)) {
          ++result.tile_visit_reject_count;
          result.fragments.clear();
          continue;
        }
        result.success = true;
        record_backward_attempt(dst, "meet", [&]() { return suffix; });
        return result.success;
      }
    }
    return result.success;
  });
  if (result.success) {
    finish_frontiers();
    return result;
  }
  reset_backward_width();

  // Build the reverse jump index once per docking attempt. Backward nodes
  // then look up incoming transitions without rescanning the whole window.
  auto build_backward_sources = [&]() {
    if (backward_sources_built) {
      return;
    }
    backward_sources_built = true;
    if (!backward_index) {
      fpga::Device &device = fpga::Device::current();
      // Production supplies the once-built device topology. Standalone
      // grounding needs the same complete incoming graph: limiting the source
      // scan to the target window loses long wires entering that window.
      const bool bounded = options.directional || options.bounded_reverse;
      const fpga::Coord center = bounded ? target_tile.coord
          : fpga::Coord{device.size_width / 2, device.size_height / 2};
      const int radius = bounded ? options.radius
          : std::max(device.size_width, device.size_height);
      local_backward_index = buildBackwardResolveIndex(
          device, center, radius,
          [&](const fpga::Coord &coord) {
            return inDirectionalCorridor(coord, forward_tile.coord,
                                         target_tile.coord, options);
          });
      backward_index = &local_backward_index;
    }
    result.backward_mapping_scan_count = backward_index->mapping_scan_count;
    result.backward_mapping_reaches_count =
        backward_index->mapping_reaches_count;
  };

  auto expand_forward = [&](int node_index) -> bool {
    ++result.forward_pop_count;
    Node node = forward_nodes[node_index];
    const bool takeoff = options.source_local && node_index == 0;
    const auto from_type = takeoff ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
    if (node.depth >= options.max_depth ||
        !inDockWindow(node.tile->coord, target_tile.coord, options.radius)) {
      return false;
    }
    int next_depth = node.depth + 1;
    if (options.candidate_limit > 0 &&
        forward_width_by_depth[next_depth] >= options.candidate_limit) {
      return false;
    }
    const std::vector<uint16_t> *srcs =
        node.tile->cb_type->srcNodes(from_type, node.dst);
    if (!srcs) {
      return false;
    }

    struct ForwardCandidate {
      uint16_t src = 0;
      int joint = -2;
      int joint2 = -1;
      fpga::InternedString src_wire;
      fpga::TileJumpTarget target;
      int tile_visits_root = -1;
    };
    std::vector<ForwardCandidate> candidates;
    candidates.reserve(srcs->size());
    NodeMask source_mask;
    for (uint16_t src : *srcs) {
      source_mask.setBit(src);
    }
    // Docking uses the same loaded angle/length buckets as ordinary routing.
    const std::vector<uint16_t> &ordered_sources =
        node.tile->cb_type->orderedSrcNodes(target_tile.coord -
                                            node.tile->coord);
    for (uint16_t src : ordered_sources) {
      if (!source_mask.testBit(src)) {
        continue;
      }
      int joint2 = -1;
      int joint = selectJointToSrc(*node.tile, from_type, node.dst, src,
                                   &joint2);
      if (joint == -2) {
        if (RouteCongestionTrace::current)
          RouteCongestionTrace::current->blocked(*node.tile, "docking_joint_unavailable",
              {{from_type, node.dst}, {fpga::CB_NODE_SRC, src}});
        continue;
      }
      fpga::InternedString src_wire = srcWireName(*node.tile, from_type,
                                         node.dst, src, joint, node.dst_wire);
      fpga::TileJumpTarget target = fpga::Device::current().resolveJumpToward(
          *node.tile, src, target_tile.coord);
      if (!target.tile || !target.tile->cb_type || target.dst_node < 0 ||
          !inDockWindow(target.tile->coord, target_tile.coord,
                        options.radius) ||
          !inDirectionalCorridor(target.tile->coord, forward_tile.coord,
                                 target_tile.coord, options)) {
        continue;
      }
      // A same-CB continuation does not leave/re-enter the tile.
      int candidate_visits_root = node.tile_visits_root;
      if (target.tile != node.tile && !tile_visits.append(node.tile_visits_root, target.tile,
                              candidate_visits_root)) {
        ++result.tile_visit_reject_count;
        continue;
      }
      const bool allow_existing_dst = node.parent < 0;
      const bool dst_busy =
          !allow_existing_dst && node.tile->cb.dst.jump.testBit(node.dst);
      const bool src_busy = node.tile->cb.src.jump.testBit(src);
      const bool joint_busy =
          joint >= 0 && node.tile->cb.joint.jump.testBit(joint);
      const bool joint2_busy =
          joint2 >= 0 && node.tile->cb.joint.jump.testBit(joint2);
      if (dst_busy || src_busy || joint_busy || joint2_busy) {
        if (RouteCongestionTrace::current)
          RouteCongestionTrace::current->blocked(*node.tile, "docking_forward_busy",
              {{from_type, node.dst}, {fpga::CB_NODE_SRC, src},
               {fpga::CB_NODE_JOINT, joint}, {fpga::CB_NODE_JOINT, joint2}});
        // Preserve the numeric fast rejection in ordinary searches. A busy
        // Takeoff has no legacy bridge record and needs named Wire storage
        // only when provisional forward expansion will actually inspect it.
        if (takeoff && (!options.preemption || !options.forward_preemption))
          continue;
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
        // The provisional pass must grow BOTH frontiers through eligible
        // transit obstructions. Otherwise a blocked first exit turns the
        // bidirectional search into a reverse-only search with half its
        // intended reach. This is read-only: accept_complete validates the
        // entire replacement and the caller applies cuts atomically later.
        if (!options.preemption || !options.forward_preemption ||
            !options.preemption->eligible_edge(edge)) {
          // Legacy bridge records describe DST->SRC, not output LOCALs.
          if (!takeoff) {
            pending_bridge_blockers.push_back(PendingBridgeBlocker{
                nodeKey(node),
                Key{target.tile->coord.x, target.tile->coord.y, target.dst_node},
                -1, edge, dst_busy, src_busy, joint_busy, joint2_busy, true});
            forward_blocked_landings.insert(
                Key{target.tile->coord.x, target.tile->coord.y, target.dst_node});
          }
          continue;
        }
      }
      candidates.push_back(ForwardCandidate{src, joint, joint2, src_wire,
                                             target, candidate_visits_root});
      if (options.candidate_limit > 0 &&
          static_cast<int>(candidates.size()) >= options.candidate_limit) {
        break;
      }
    }
    if (options.candidate_limit > 0 &&
        static_cast<int>(candidates.size()) > options.candidate_limit) {
      candidates.resize(options.candidate_limit);
    }

    for (const ForwardCandidate &candidate : candidates) {
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
      if (forward_seen.find(key) != forward_seen.end()) {
        continue;
      }
      int next_index = static_cast<int>(forward_nodes.size());
      forward_nodes.push_back(next);
      forward_seen[key] = next_index;
      forward_queue.push_back(next_index);
      ++forward_width_by_depth[next_depth];
      ++result.forward_push_count;
      if (auto back = backward_seen.find(key); back != backward_seen.end()) {
        result.fragments = pathToNode(forward_nodes, next_index);
        std::vector<fpga::Wire> suffix =
            suffixFromNode(backward_nodes, back->second);
        result.fragments.insert(result.fragments.end(), suffix.begin(),
                                suffix.end());
        if (!path_valid(result.fragments)) {
          ++result.tile_visit_reject_count;
          result.fragments.clear();
          continue;
        }
        result.success = true;
        return true;
      }
      if (options.candidate_limit > 0 &&
          forward_width_by_depth[next_depth] >= options.candidate_limit) {
        break;
      }
    }
    return false;
  };

  auto expand_backward = [&](int node_index) -> bool {
    ++result.backward_pop_count;
    Node node = backward_nodes[node_index];
    if (node.depth >= 0 &&
        static_cast<size_t>(node.depth) < backward_pending_by_depth.size() &&
        backward_pending_by_depth[node.depth] > 0) {
      --backward_pending_by_depth[node.depth];
    }
    Key current_key = nodeKey(node);
    if (backward_deadends.contains(current_key)) {
      ++result.backward_deadend_reject_count;
      record_backward_attempt(node.seed_dst, "deadend_pop", [&]() {
        return suffixFromNode(backward_nodes, node_index);
      });
      return false;
    }
    if (node.depth >= options.max_depth) {
      backward_deadends.insert(current_key);
      ++result.backward_deadend_count;
      record_backward_attempt(node.seed_dst, "depth_limit", [&]() {
        return suffixFromNode(backward_nodes, node_index);
      });
      return false;
    }
    build_backward_sources();
    const std::vector<BackwardResolveSource> *sources =
        resolveBackwardSources(*backward_index, current_key,
                               !options.directional && !options.bounded_reverse);
    if (!sources) {
      backward_deadends.insert(current_key);
      ++result.backward_deadend_count;
      record_backward_attempt(node.seed_dst, "no_incoming", [&]() {
        return suffixFromNode(backward_nodes, node_index);
      });
      return false;
    }
    size_t source_count = sources->size();

    for (size_t source_index = 0; source_index < source_count; ++source_index) {
      int next_depth = node.depth + 1;
      const BackwardSource &source = (*sources)[source_index];
      int src = source.src;
      fpga::Tile *prev_tile = source.tile;
      if (!prev_tile || !prev_tile->cb_type) {
        continue;
      }
      // Explicit/non-shared indexes may contain more than the local window.
      if (options.bounded_reverse &&
          !inDockWindow(prev_tile->coord, target_tile.coord, options.radius)) {
        continue;
      }
      int previous_visits_root = node.tile_visits_root;
      if (prev_tile != node.tile && !tile_visits.append(node.tile_visits_root, prev_tile,
                              previous_visits_root)) {
        ++result.tile_visit_reject_count;
        continue;
      }
      if (!backward_fits_prefix(previous_visits_root, prev_tile->coord)) {
        ++result.tile_visit_reject_count;
        continue;
      }
      prev_tile->cb_type->ensureDerivedMasks();
      if (options.source_local && prev_tile == &forward_tile) {
        int joint2 = -1;
        int joint = selectJointToSrc(*prev_tile, fpga::CB_NODE_LOCAL,
                                     forward_dst, src, &joint2);
        // The requested output LOCAL may already drive sibling fanouts.
        // Only the new SRC and any intermediate JOINTs must be free.
        if (joint != -2 && (options.preemption ||
            canLeaseJump(prev_tile->cb, forward_dst, src, joint, joint2, true))) {
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
          takeoff.src_wire_name = srcWireName(*prev_tile, fpga::CB_NODE_LOCAL,
                                              forward_dst, src, joint, {});
          takeoff.dst_wire_name = node.dst_wire;
          std::vector<fpga::Wire> joined{takeoff};
          auto suffix = suffixFromNode(backward_nodes, node_index);
          joined.insert(joined.end(), suffix.begin(), suffix.end());
          if ((!options.preemption ||
               canLeaseJump(prev_tile->cb, forward_dst, src, joint, joint2, true) ||
               options.preemption->eligible_edge(takeoff)) &&
              path_valid(joined)) {
            record_backward_attempt(node.seed_dst, "meet_source_local",
                                    [&]() { return joined; });
            if (node.blocked_terminal) {
              result.blocked_terminal_reachable = true;
              result.blocked_terminal_fragments = std::move(joined);
              result.blocked_dst = node.seed_dst;
              result.blocked_pin = node.terminal_pin;
              result.blocked_joint = node.terminal_joint;
              result.blocked_joint2 = node.terminal_joint2;
            } else {
              result.fragments = std::move(joined);
              result.success = true;
            }
            return true;
          }
          ++result.tile_visit_reject_count;
        }
      }
      NodeMask prev_dsts = prev_tile->cb_type->dsts_reaching_src.get(src).jump;
      if (prev_dsts == NodeMask{}) {
        ++result.backward_missing_prev_dst_count;
        record_backward_attempt(node.seed_dst, "no_previous_dst", [&]() {
          return suffixFromNode(backward_nodes, node_index);
        });
        continue;
      }
      prev_dsts.for_each_set_bit([&](int prev_dst) {
        const Key key{prev_tile->coord.x, prev_tile->coord.y, prev_dst};
        // A complete join must still be tried even if the numeric state was
        // seen before. Other duplicates/full layers cannot advance the search:
        // reject them before constructing paths or consulting victim owners.
        if (options.preemption && !forward_seen.contains(key)) {
          if (backward_deadends.contains(key) || backward_seen.contains(key)) {
            ++result.backward_seen_reject_count;
            return false;
          }
          // The quota limits queued exploration, not the remaining numeric
          // predecessors: a later DST may already join the forward frontier.
          if (!backward_has_capacity(node, next_depth)) return false;
        }
        int joint2 = -1;
        int joint = selectJointToSrc(*prev_tile, fpga::CB_NODE_DST, prev_dst,
                                     src, &joint2);
        if (joint == -2) {
          ++result.backward_topology_reject_count;
          record_backward_attempt(node.seed_dst, "topology_reject", [&]() {
            return suffixFromNode(backward_nodes, node_index);
          });
          return false;
        }
        // Ordinary free edges that cannot enter the frontier need no names
        // or Wire/Node objects. Preserve complete joins, occupied edges used
        // by later preemption, and full per-candidate diagnostic traces.
        if (!options.preemption && !options.trace_backward_attempts &&
            !forward_seen.contains(key) &&
            canLeaseJump(prev_tile->cb, prev_dst, src, joint, joint2,
                !options.source_local && prev_tile == &forward_tile &&
                    prev_dst == forward_dst)) {
          const bool bridge = forward_blocked_landings.contains(key);
          if (!bridge && !backward_has_capacity(node, next_depth)) return false;
          if (backward_deadends.contains(key)) {
            ++result.backward_deadend_reject_count;
            return false;
          }
          if (backward_seen.contains(key)) {
            ++result.backward_seen_reject_count;
            return false;
          }
        }
        fpga::InternedString src_wire = srcWireName(*prev_tile, fpga::CB_NODE_DST,
                                           prev_dst, src, joint, {});
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
        edge.from_wire_name = fromWireName(*prev_tile, fpga::CB_NODE_DST,
                                           prev_dst, src, joint, src_wire);
        edge.src_wire_name = src_wire;
        edge.dst_wire_name = node.dst_wire;
        auto candidate_fragments = [&]() {
          std::vector<fpga::Wire> fragments{edge};
          std::vector<fpga::Wire> suffix =
              suffixFromNode(backward_nodes, node_index);
          fragments.insert(fragments.end(), suffix.begin(), suffix.end());
          return fragments;
        };
        // The backward frontier may terminate at the already-routed
        // forward anchor; only its outgoing source must still be free.
        bool reaches_forward_anchor =
            !options.source_local && prev_tile == &forward_tile && prev_dst == forward_dst;
        if (!canLeaseJump(prev_tile->cb, prev_dst, src, joint, joint2,
                          reaches_forward_anchor) &&
            (!options.preemption ||
             !options.preemption->eligible_edge(edge))) {
          if (RouteCongestionTrace::current)
            RouteCongestionTrace::current->blocked(*prev_tile, "docking_backward_busy",
                {{fpga::CB_NODE_DST, prev_dst}, {fpga::CB_NODE_SRC, src},
                 {fpga::CB_NODE_JOINT, joint}, {fpga::CB_NODE_JOINT, joint2}});
          PendingBridgeBlocker blocker;
          blocker.forward_key =
              Key{prev_tile->coord.x, prev_tile->coord.y, prev_dst};
          blocker.backward_key = current_key;
          blocker.backward_index = node_index;
          blocker.edge = edge;
          blocker.dst_busy = !reaches_forward_anchor &&
                             prev_tile->cb.dst.jump.testBit(prev_dst);
          blocker.src_busy = prev_tile->cb.src.jump.testBit(src);
          blocker.joint_busy =
              joint >= 0 && prev_tile->cb.joint.jump.testBit(joint);
          blocker.joint2_busy =
              joint2 >= 0 && prev_tile->cb.joint.jump.testBit(joint2);
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
        if (const std::string *name =
                prev_tile->cb_type->nodeName(fpga::CB_NODE_DST, prev_dst)) {
          next.dst_wire = *name;
        }
        // A bounded beam limits queued alternatives, not proof of a completed
        // connection. Check every numeric candidate against the opposite
        // frontier before applying this terminal seed's width quota.
        if (auto front = forward_seen.find(key); front != forward_seen.end()) {
          auto joined = pathToNode(forward_nodes, front->second);
          const auto suffix = candidate_fragments();
          joined.insert(joined.end(), suffix.begin(), suffix.end());
          if (!path_valid(joined)) {
            ++result.tile_visit_reject_count;
            return false;
          }
          if (next.blocked_terminal) {
            result.blocked_terminal_reachable = true;
            result.blocked_terminal_fragments = std::move(joined);
            result.blocked_dst = node.seed_dst;
            result.blocked_pin = next.terminal_pin;
            result.blocked_joint = next.terminal_joint;
            result.blocked_joint2 = next.terminal_joint2;
            record_backward_attempt(node.seed_dst, "blocked_meet",
                                    candidate_fragments);
            return true;
          }
          result.fragments = std::move(joined);
          result.success = true;
          record_backward_attempt(node.seed_dst, "meet", candidate_fragments);
          return true;
        }
        // The beam bounds queued exploration, but it must not hide a numeric
        // landing separated from the forward frontier by one occupied edge.
        const bool preserves_blocked_bridge =
            !options.preemption && forward_blocked_landings.contains(key);
        if (!preserves_blocked_bridge &&
            !backward_has_capacity(node, next_depth)) {
          return false; // Keep inspecting later complete joins/bridge landings.
        }
        if (backward_deadends.contains(key)) {
          ++result.backward_deadend_reject_count;
          record_backward_attempt(node.seed_dst, "deadend_reject",
                                  candidate_fragments);
          return false;
        }
        if (backward_seen.find(key) != backward_seen.end()) {
          ++result.backward_seen_reject_count;
          record_backward_attempt(node.seed_dst, "seen_reject",
                                  candidate_fragments);
          return false;
        }
        int next_index = static_cast<int>(backward_nodes.size());
        backward_nodes.push_back(next);
        backward_seen[key] = next_index;
        if (preserves_blocked_bridge) {
          record_backward_attempt(node.seed_dst, "blocked_bridge_landing",
                                  candidate_fragments);
          return false;
        }
        backward_queue.push_back(next_index);
        record_backward_push(node, next_depth);
        ++result.backward_push_count;
        record_backward_attempt(node.seed_dst, "push", candidate_fragments);
        return false;
      });
      if (result.success) {
        return true;
      }
    }
    backward_deadends.insert(current_key);
    ++result.backward_deadend_count;
    return false;
  };

  auto probe_initial_forward_blockers = [&]() {
    const Node &node = forward_nodes.front();
    const std::vector<uint16_t> *srcs =
        node.tile->cb_type->srcNodes(fpga::CB_NODE_DST, node.dst);
    if (!srcs) {
      return;
    }
    for (uint16_t src : *srcs) {
      int joint2 = -1;
      int joint = selectJointToSrc(*node.tile, fpga::CB_NODE_DST, node.dst, src,
                                   &joint2);
      if (joint == -2) {
        continue;
      }
      fpga::TileJumpTarget target = fpga::Device::current().resolveJumpToward(
          *node.tile, src, target_tile.coord);
      if (!target.tile || !target.tile->cb_type || target.dst_node < 0 ||
          !inDockWindow(target.tile->coord, target_tile.coord,
                        options.radius) ||
          !inDirectionalCorridor(target.tile->coord, forward_tile.coord,
                                 target_tile.coord, options)) {
        continue;
      }
      const bool src_busy = node.tile->cb.src.jump.testBit(src);
      const bool joint_busy =
          joint >= 0 && node.tile->cb.joint.jump.testBit(joint);
      const bool joint2_busy =
          joint2 >= 0 && node.tile->cb.joint.jump.testBit(joint2);
      if (!src_busy && !joint_busy && !joint2_busy) {
        continue;
      }
      fpga::InternedString src_wire = srcWireName(*node.tile, fpga::CB_NODE_DST,
                                         node.dst, src, joint, node.dst_wire);
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
      Key landing{target.tile->coord.x, target.tile->coord.y, target.dst_node};
      pending_bridge_blockers.push_back(
          PendingBridgeBlocker{nodeKey(node), landing, -1, edge, false,
                               src_busy, joint_busy, joint2_busy, true});
      forward_blocked_landings.insert(landing);
    }
  };

  // Record only occupied root exits before the backward beam is consumed.
  // Successful direct backward meets still run before ordinary forward work.
  if (!options.source_local && !options.preemption) probe_initial_forward_blockers();

  while ((!forward_queue.empty() || !backward_queue.empty()) &&
         !result.success) {
    if (!backward_queue.empty()) {
      int index = backward_queue.front();
      backward_queue.pop_front();
      bool expanded = expand_backward(index);
      if (expanded) {
        break;
      }
    }
    if (!forward_queue.empty()) {
      int index = forward_queue.front();
      forward_queue.pop_front();
      bool expanded = expand_forward(index);
      if (expanded) {
        break;
      }
    }
  }

  // Reuse the exhausted forward search to identify a physically reachable
  // busy terminal. The caller may then preempt exactly that numeric entry.
  if (!result.success && !blocked_seeds.empty()) {
    // Terminal-preemption probing is a separate backward search. Preserve the
    // free-terminal frontier and its exact transit bridge before replacing it.
    finish_frontiers();
    backward_nodes.clear();
    backward_queue.clear();
    backward_seen.clear();
    backward_deadends.clear();
    for (Node &seed : blocked_seeds) {
      Key key = nodeKey(seed);
      if (backward_seen.find(key) != backward_seen.end()) {
        continue;
      }
      int index = static_cast<int>(backward_nodes.size());
      backward_nodes.push_back(std::move(seed));
      backward_seen[key] = index;
      if (auto front = forward_seen.find(key); front != forward_seen.end()) {
        const Node &reached = backward_nodes[index];
        auto joined = pathToNode(forward_nodes, front->second);
        const auto suffix = suffixFromNode(backward_nodes, index);
        joined.insert(joined.end(), suffix.begin(), suffix.end());
        if (!path_valid(joined)) {
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
    reset_backward_width();
    while (!result.blocked_terminal_reachable && !backward_queue.empty()) {
      int index = backward_queue.front();
      backward_queue.pop_front();
      bool expanded = expand_backward(index);
      if (expanded) {
        break;
      }
    }
  }

  finish_frontiers();
  return result;
}

} // namespace

ForwardPlacementResult routeForwardToPlacement(
    const std::vector<ForwardPlacementAnchor> &anchors,
    const ForwardPlacementProbe &probe, const std::function<bool()> &cancelled,
    std::ostream *decisions, const ForwardPlacementObserver &observe) {
  ForwardPlacementResult result;
  auto describe = [&](fpga::Tile &tile, fpga::CBNodeNameType type, int id) {
    if (!decisions)
      return;
    const auto *name = tile.cb_type->nodeName(type, id);
    *decisions << " tile=(" << tile.coord.x << ',' << tile.coord.y
               << ") type=" << static_cast<int>(type) << " node=" << id
               << " name=" << (name ? *name : "?");
  };
  std::vector<size_t> roots;
  std::vector<std::vector<fpga::Tile *>> anchor_visits(anchors.size());
  for (size_t a = 0; a < anchors.size(); ++a) {
    if (cancelled && cancelled()) {
      result.cancelled = true;
      return result;
    }
    const auto &anchor = anchors[a];
    if (decisions && anchor.tile && anchor.tile->cb_type) {
      *decisions << "FORWARD_ANCHOR index=" << a
                 << " prefix=" << anchor.prefix.size();
      describe(*anchor.tile,
               anchor.from_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL,
               anchor.node);
      *decisions << '\n';
    }
    if (!anchor.tile || !anchor.tile->cb_type || anchor.node < 0)
      continue;
    if (anchor.from_dst && !anchor.tile->cb.dst.jump.testBit(anchor.node))
      continue; // A free DST is not a source-connected branch point.
    fpga::Tile *last = nullptr;
    bool valid = true;
    auto visit = [&](fpga::Coord coord) {
      auto *tile = fpga::Device::current().getTile(coord.x, coord.y);
      if (tile != last) {
        auto &history = anchor_visits[a];
        valid = valid && tile &&
                std::count(history.begin(), history.end(), tile) <
                    COMBINATORIAL_TILE_VISIT_LIMIT;
        if (valid)
          history.push_back(tile);
        last = tile;
      }
    };
    for (const auto &wire : anchor.prefix) {
      if (wire.type != fpga::Wire::WIRE_CROSSBAR &&
          wire.type != fpga::Wire::WIRE_ROUTE_EDGE)
        continue;
      visit(wire.from);
      visit(wire.to);
    }
    visit(anchor.tile->coord);
    if (!valid) {
      ++result.tile_visit_rejects;
      continue;
    }
    roots.push_back(a);
  }
  // Reconstruct one depth layer at a time instead of retaining its entire
  // frontier. Probes still run once in BFS order; earlier layers only rebuild
  // prefixes, with path-local leases/visits restored on every recursive return.
  std::vector<fpga::Wire> path;
  std::vector<fpga::Tile *> visited_tiles;
  std::vector<std::pair<fpga::Tile *, fpga::CBState>> saved;
  struct Restore {
    fpga::Tile &tile;
    fpga::CBState before;
    std::vector<std::pair<fpga::Tile *, fpga::CBState>> &saved;
    bool added;
    ~Restore() {
      tile.cb = before;
      if (added)
        saved.pop_back();
    }
  };
  std::vector<size_t> level_bases{0};
  size_t queued = roots.size();
  auto stopped = [&]() {
    if (cancelled && cancelled())
      result.cancelled = true;
    return result.cancelled;
  };
  auto blocked =
      [&](fpga::Tile &tile, const char *reason,
          std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes) {
        if (!RouteCongestionTrace::current)
          return;
        const fpga::CBState search_state = tile.cb;
        for (const auto &original : saved)
          if (original.first == &tile) {
            tile.cb = original.second;
            break;
          }
        RouteCongestionTrace::current->blocked(tile, reason, nodes,
                                               &search_state);
        tile.cb = search_state;
      };
  // Preserve successful jump-only searches. On exhaustion, include internal
  // LOCAL bridges already present in the numeric crossbar tables.
  bool has_local_bridge = false;
  for (bool include_locals : {false, true}) {
    if (include_locals && !has_local_bridge)
      break;
    level_bases = {0};
    queued = roots.size();
    for (size_t layer = 0; !roots.empty() && !stopped(); ++layer) {
      std::vector<size_t> ordinals(layer + 1, 0);
      size_t children = 0;
      auto walk = [&](auto &&self, fpga::Tile &tile, int dst,
                      fpga::CBNodeNameType type, size_t anchor_id, size_t depth,
                      long long parent) -> bool {
        if (stopped())
          return false;
        const bool root = depth == 0;
        const bool frontier = depth == layer;
        const size_t cursor = level_bases[depth] + ordinals[depth]++;
        const auto &anchor = anchors[anchor_id];
        if (!root && (type == fpga::CB_NODE_LOCAL ? tile.cb.local.local
                                                  : tile.cb.dst.jump)
                         .testBit(dst)) {
          if (decisions)
            *decisions << "FORWARD_REJECT state=" << cursor
                       << " reason=arrival_lease\n";
          if (frontier)
            blocked(tile, "forward_arrival_lease", {{type, dst}});
          return false;
        }
        if (frontier) {
          ++result.expanded;
          if (decisions) {
            *decisions << "FORWARD_STATE index=" << cursor
                       << " parent=" << parent << " anchor=" << anchor_id;
            describe(tile, type, dst);
            *decisions << '\n';
          }
          std::vector<fpga::Wire> terminal;
          if (observe)
            observe(saved);
          if (type == fpga::CB_NODE_DST &&
              probe(tile, dst, root && anchor.from_dst, terminal)) {
            result.success = true;
            if (decisions)
              *decisions << "FORWARD_GROUNDED state=" << cursor << '\n';
            result.anchor = anchor_id;
            result.fragments = path;
            result.fragments.insert(result.fragments.end(), terminal.begin(),
                                    terminal.end());
            return true;
          }
        }
        const auto *sources = tile.cb_type->srcNodes(type, dst);
        if (frontier && decisions)
          *decisions << "FORWARD_SOURCES state=" << cursor
                     << " count=" << (sources ? sources->size() : 0) << '\n';
        static const std::vector<uint16_t> no_sources;
        for (int src : sources ? *sources : no_sources) {
          if (stopped())
            return false;
          int joint2 = -1;
          int joint = selectJointToSrc(tile, type, dst, src, &joint2, true);
          const bool free =
              joint != -2 && canLeaseJump(tile.cb, dst, src, joint, joint2,
                                          root || type == fpga::CB_NODE_LOCAL);
          // Report the actual occupied joint when topology exists but no free
          // joint chain does. This diagnostic choice never becomes a free edge.
          if (joint == -2) joint = selectJointToSrc(tile, type, dst, src, &joint2);
          if (frontier && decisions) {
            *decisions << "FORWARD_EDGE state=" << cursor;
            describe(tile, fpga::CB_NODE_SRC, src);
            *decisions << " joint=" << joint << " joint2=" << joint2
                       << " src_busy=" << tile.cb.src.jump.testBit(src)
                       << " free=" << free << '\n';
          }
          if (!free) {
            if (frontier)
              blocked(tile,
                      joint == -2 ? "forward_missing_topology"
                                  : "forward_exit_lease",
                      {{type, dst},
                       {fpga::CB_NODE_SRC, src},
                       {fpga::CB_NODE_JOINT, joint},
                       {fpga::CB_NODE_JOINT, joint2}});
            continue;
          }
          const auto targets =
              fpga::Device::current().resolveJumpTargets(tile, src);
          if (frontier && decisions)
            *decisions << "FORWARD_TARGETS state=" << cursor << " src=" << src
                       << " count=" << targets.size() << '\n';
          for (const auto &target : targets) {
            if (frontier && decisions && target.tile && target.tile->cb_type) {
              *decisions << "FORWARD_TARGET state=" << cursor << " src=" << src;
              describe(*target.tile, fpga::CB_NODE_DST, target.dst_node);
              *decisions << " busy="
                         << (target.dst_node >= 0 &&
                             target.tile->cb.dst.jump.testBit(target.dst_node))
                         << '\n';
            }
            if (!target.tile || !target.tile->cb_type || target.dst_node < 0 ||
                target.tile->cb.dst.jump.testBit(target.dst_node)) {
              if (frontier && target.tile && target.dst_node >= 0)
                blocked(*target.tile, "forward_target_lease",
                        {{fpga::CB_NODE_DST, target.dst_node}});
              continue;
            }
            if (target.tile != &tile &&
                std::count(visited_tiles.begin(), visited_tiles.end(),
                           target.tile) >= COMBINATORIAL_TILE_VISIT_LIMIT) {
              if (frontier)
                ++result.tile_visit_rejects;
              if (frontier && decisions)
                *decisions << "FORWARD_REJECT state=" << cursor
                           << " reason=third_tile_visit\n";
              continue;
            }
            // Same-CB continuity is allowed, but cannot consume its own source
            // DST.
            if (type == fpga::CB_NODE_DST && target.tile == &tile &&
                target.dst_node == dst)
              continue;
            if (frontier) {
              if (decisions)
                *decisions << "FORWARD_ENQUEUE parent=" << cursor
                           << " child=" << queued + children << '\n';
              ++children;
              continue;
            }
            fpga::Wire wire;
            wire.from = tile.coord;
            wire.to = target.tile->coord;
            wire.local = dst;
            wire.jump = src;
            wire.dst = target.dst_node;
            wire.route_jump = target.jump_node;
            wire.joint = joint;
            wire.joint2 = joint2;
            wire.pos = root ? (anchor.from_dst ? ROUTE_POS_FORK : 0)
                            : ROUTE_POS_TRANSIT;
            wire.owns_dst = !(root && anchor.from_dst);
            if (const auto *name = tile.cb_type->nodeName(type, dst))
              wire.from_wire_name = *name;
            wire.src_wire_name = srcWireName(tile, type, dst, src, joint,
                                             std::string(wire.from_wire_name));
            wire.dst_wire_name = target.dst_wire;
            Restore restore{
                tile, tile.cb, saved,
                std::none_of(saved.begin(), saved.end(),
                             [&](const auto &s) { return s.first == &tile; })};
            if (restore.added)
              saved.emplace_back(&tile, tile.cb);
            tile.cb.src.jump.setBit(src);
            if (type == fpga::CB_NODE_DST)
              tile.cb.dst.jump.setBit(dst);
            else if (!root)
              tile.cb.local.local.setBit(dst);
            if (joint >= 0)
              tile.cb.joint.jump.setBit(joint);
            if (joint2 >= 0)
              tile.cb.joint.jump.setBit(joint2);
            const bool visit = target.tile != &tile;
            if (visit)
              visited_tiles.push_back(target.tile);
            const size_t path_begin = path.size();
            if (type == fpga::CB_NODE_LOCAL && !root) {
              // Internal locals are routing resources, not new source takeoffs.
              auto edge = [&](fpga::Coord from, fpga::CBNodeNameType from_type,
                              int from_node, fpga::Coord to,
                              fpga::CBNodeNameType to_type, int to_node) {
                fpga::Wire arc;
                arc.type = fpga::Wire::WIRE_ROUTE_EDGE;
                arc.from = from;
                arc.to = to;
                arc.from_node_type = from_type;
                arc.from_node = from_node;
                arc.to_node_type = to_type;
                arc.to_node = to_node;
                path.push_back(arc);
              };
              auto from_type = fpga::CB_NODE_LOCAL;
              int from_node = dst;
              for (int j : {joint2, joint})
                if (j >= 0) {
                  edge(tile.coord, from_type, from_node, tile.coord,
                       fpga::CB_NODE_JOINT, j);
                  from_type = fpga::CB_NODE_JOINT;
                  from_node = j;
                }
              edge(tile.coord, from_type, from_node, tile.coord,
                   fpga::CB_NODE_SRC, src);
              edge(tile.coord, fpga::CB_NODE_SRC, src, target.tile->coord,
                   fpga::CB_NODE_DST, target.dst_node);
            } else
              path.push_back(wire);
            const bool success =
                self(self, *target.tile, target.dst_node, fpga::CB_NODE_DST,
                     anchor_id, depth + 1, static_cast<long long>(cursor));
            path.resize(path_begin);
            if (visit)
              visited_tiles.pop_back();
            if (success || result.cancelled)
              return success;
          }
        }
        // Never borrow an endpoint reserved by placed logic. Only free internal
        // wires with outgoing connectivity can extend the combinatorial path.
        const auto &locals = type == fpga::CB_NODE_DST
                                 ? tile.cb_type->dst_local.get(dst).local
                                 : tile.cb_type->local_local.get(dst).local;
        bool found = locals.for_each_set_bit([&](int local) {
          if (stopped())
            return true;
          if (frontier && decisions) {
            *decisions << "FORWARD_LOCAL state=" << cursor;
            describe(tile, fpga::CB_NODE_LOCAL, local);
            *decisions << " busy=" << tile.cb.local.local.testBit(local)
                       << " pin_busy=" << tile.isPinNodeLeased(local)
                       << " reserved=" << tile.input_local_reservations.contains(local)
                       << " enabled=" << include_locals << '\n';
          }
          if (tile.cb.local.local.testBit(local) ||
              tile.isPinNodeLeased(local) ||
              tile.input_local_reservations.contains(local) ||
              (type == fpga::CB_NODE_LOCAL && local == dst))
            return false;
          const auto *exits =
              tile.cb_type->srcNodes(fpga::CB_NODE_LOCAL, local);
          if ((!exits || exits->empty()) &&
              tile.cb_type->local_local.get(local).local == NodeMask{})
            return false;
          has_local_bridge = true;
          if (!include_locals)
            return false;
          if (frontier) {
            if (decisions)
              *decisions << "FORWARD_ENQUEUE parent=" << cursor
                         << " child=" << queued + children << '\n';
            ++children;
            return false;
          }
          const fpga::CBState before = tile.cb;
          const bool added =
              std::none_of(saved.begin(), saved.end(),
                           [&](const auto &s) { return s.first == &tile; });
          Restore restore{tile, before, saved, added};
          if (added)
            saved.emplace_back(&tile, before);
          if (type == fpga::CB_NODE_DST)
            tile.cb.dst.jump.setBit(dst);
          else
            tile.cb.local.local.setBit(dst);
          fpga::Wire arc;
          arc.type = fpga::Wire::WIRE_ROUTE_EDGE;
          arc.from = arc.to = tile.coord;
          arc.from_node_type = type;
          arc.from_node = dst;
          arc.to_node_type = fpga::CB_NODE_LOCAL;
          arc.to_node = local;
          const bool crossbar_arrival =
              !path.empty() && path.back().type == fpga::Wire::WIRE_CROSSBAR;
          const bool owned_landing =
              crossbar_arrival && path.back().owns_landing;
          if (crossbar_arrival)
            path.back().owns_landing = true;
          path.push_back(arc);
          const bool success =
              self(self, tile, local, fpga::CB_NODE_LOCAL, anchor_id, depth + 1,
                   static_cast<long long>(cursor));
          path.pop_back();
          if (crossbar_arrival)
            path.back().owns_landing = owned_landing;
          return success || result.cancelled;
        });
        if (found)
          return result.success;
        return false;
      };
      for (size_t anchor_id : roots) {
        visited_tiles = anchor_visits[anchor_id];
        if (walk(walk, *anchors[anchor_id].tile, anchors[anchor_id].node,
                 anchors[anchor_id].from_dst ? fpga::CB_NODE_DST
                                             : fpga::CB_NODE_LOCAL,
                 anchor_id, 0, -1) ||
            result.cancelled)
          return result;
      }
      if (!children)
        break;
      level_bases.push_back(queued);
      queued += children;
    }
  }
  if (decisions)
    *decisions << "FORWARD_END queued=" << queued
               << " reached=" << result.expanded
               << " cancelled=" << result.cancelled << '\n';
  return result;
}

void dumpBackwardTerminalReport(fpga::Tile &target, NodeMask pins,
                                BackwardResolveIndex &index, int docking_radius,
                                const std::vector<rtl::Net *> &design_nets,
                                std::ostream &out,
                                NodeMask reserved_terminal_joints) {
  out << "BACKWARD_TERMINAL_SNAPSHOT target=(" << target.coord.x << ','
      << target.coord.y << ") docking_radius=" << docking_radius
      << " index_radius=" << index.radius << '\n'
      << "Final-state topology/lease analysis, NOT recorded search attempts. "
         "No anchor lease exemptions; free first hop does not prove a full "
         "route.\n";
  if (!target.cb_type) {
    out << "NO_CROSSBAR\n";
    return;
  }
  std::vector<fpga::Tile *> audited{&target};
  std::unordered_set<fpga::Tile *> seen{&target};
  auto node = [&](fpga::Tile &tile, const char *label,
                  fpga::CBNodeNameType type, int id) {
    const std::string *name =
        id >= 0 ? tile.cb_type->nodeName(type, id) : nullptr;
    const bool busy = id >= 0 &&
        fpga::congestionNodeMask(tile.cb, type).testBit(id);
    out << ' ' << label << '=' << id << '[' << (name ? *name : "")
        << "] busy=" << busy;
  };
  size_t entries = 0, free_entries = 0, incoming_count = 0, free_hops = 0;
  pins.for_each_set_bit([&](int pin) {
    out << "PIN";
    node(target, "local", fpga::CB_NODE_LOCAL, pin);
    out << " pin_leased=" << target.isPinNodeLeased(pin) << '\n';
    for (const auto &entry : target.cb_type->terminalEntries(pin)) {
      ++entries;
      const bool reserved =
          (entry.joint >= 0 && reserved_terminal_joints.testBit(entry.joint)) ||
          (entry.joint2 >= 0 && reserved_terminal_joints.testBit(entry.joint2));
      const bool terminal_free = !reserved && !target.isPinNodeLeased(pin) &&
          canLeaseIn(target.cb, entry.dst, pin, entry.joint, entry.joint2);
      free_entries += terminal_free;
      out << "TERMINAL";
      node(target, "dst", fpga::CB_NODE_DST, entry.dst);
      node(target, "joint", fpga::CB_NODE_JOINT, entry.joint);
      node(target, "joint2", fpga::CB_NODE_JOINT, entry.joint2);
      out << " reserved_joint=" << reserved
          << " incoming_mask_allows=" << (target.incoming_dst_nodes == NodeMask{} ||
                                          target.incoming_dst_nodes.testBit(entry.dst))
          << " terminal_free=" << terminal_free << '\n';
      const auto *sources = resolveBackwardSources(
          index, {target.coord.x, target.coord.y, entry.dst});
      if (!sources || sources->empty()) {
        out << "  NO_INCOMING_SRC\n";
        continue;
      }
      for (const auto &source : *sources) {
        if (!source.tile || !source.tile->cb_type) continue;
        ++incoming_count;
        auto &tile = *source.tile;
        if (seen.insert(&tile).second) audited.push_back(&tile);
        out << "  INCOMING tile=(" << tile.coord.x << ',' << tile.coord.y << ')';
        node(tile, "src", fpga::CB_NODE_SRC, source.src);
        out << " in_docking_window="
            << inDockWindow(tile.coord, target.coord, docking_radius) << '\n';
        tile.cb_type->ensureDerivedMasks();
        const auto predecessors = tile.cb_type->dsts_reaching_src.get(source.src).jump;
        if (predecessors == NodeMask{}) out << "    NO_PREVIOUS_DST\n";
        predecessors.for_each_set_bit([&](int dst) {
          int joint2 = -1;
          const int joint = selectJointToSrc(tile, fpga::CB_NODE_DST,
                                            dst, source.src, &joint2);
          const bool jump_free = joint != -2 &&
              canLeaseJump(tile.cb, dst, source.src, joint, joint2);
          const bool first_hop_free = terminal_free && jump_free;
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
  out << "SUMMARY terminal_entries=" << entries << " free_entries=" << free_entries
      << " incoming_alternatives=" << incoming_count << " free_first_hops="
      << free_hops << " audited_tiles=" << audited.size() << '\n';
  out << "OWNERSHIP_AUDITS (live route claims checked against design registry)\n";
  // Reconstruct route membership once for this read-only group of Tile proofs.
  fpga::CongestionAuditIndex ownership_index(design_nets);
  for (auto *tile : audited) fpga::auditTileCongestion(*tile, out, ownership_index);
}

bool combinatorialRouteVisitsValid(const std::vector<fpga::Wire> &route,
                                   unsigned max_visits) {
  if (max_visits == 0) {
    return route.empty();
  }
  std::unordered_map<uint64_t, unsigned> visits;
  uint64_t previous = 0;
  bool has_previous = false;
  auto append = [&](const fpga::Coord &coord) {
    if (coord.x < 0 || coord.y < 0) {
      return true;
    }
    uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(coord.x))
                    << 32) |
                   static_cast<uint32_t>(coord.y);
    if (has_previous && previous == key) {
      return true;
    }
    previous = key;
    has_previous = true;
    return ++visits[key] <= max_visits;
  };
  for (const fpga::Wire &fragment : route) {
    if (!append(fragment.from) || !append(fragment.to)) {
      return false;
    }
  }
  return true;
}

bool materializeDockingBridge(const DockingBridgeBlocker &bridge,
                              DockingResult &result) {
  if (!bridge.valid || !bridge.joins_frontiers ||
      bridge.backward_suffix.empty()) {
    return false;
  }
  const fpga::Wire &edge = bridge.backward_suffix.front();
  if (edge.type != fpga::Wire::WIRE_CROSSBAR ||
      edge.from.x != bridge.tile.x || edge.from.y != bridge.tile.y ||
      edge.local != bridge.dst || edge.jump != bridge.src ||
      edge.joint != bridge.joint || edge.joint2 != bridge.joint2 ||
      edge.to.x != bridge.landing_tile.x ||
      edge.to.y != bridge.landing_tile.y ||
      edge.dst != bridge.landing_dst) {
    return false;
  }
  if (!bridge.forward_prefix.empty()) {
    const fpga::Wire &prefix_end = bridge.forward_prefix.back();
    if (prefix_end.to.x != bridge.tile.x ||
        prefix_end.to.y != bridge.tile.y || prefix_end.dst != bridge.dst) {
      return false;
    }
  }
  result.fragments = bridge.forward_prefix;
  result.fragments.insert(result.fragments.end(),
                          bridge.backward_suffix.begin(),
                          bridge.backward_suffix.end());
  if (!combinatorialRouteVisitsValid(result.fragments)) {
    result.fragments.clear();
    ++result.tile_visit_reject_count;
    return false;
  }
  result.success = true;
  return true;
}

BackwardResolveIndex buildBackwardResolveIndex(
    fpga::Device &device, fpga::Coord center, int radius,
    const std::function<bool(const fpga::Coord &)> &include_source,
    BackwardResolveCache *shared_cache) {
  BackwardResolveIndex result;
  result.center = center;
  result.radius = radius;
  result.shared_cache = shared_cache;
  for (int y = center.y - radius; y <= center.y + radius; ++y) {
    for (int x = center.x - radius; x <= center.x + radius; ++x) {
      fpga::Coord source_coord{x, y};
      if (!inDockWindow(source_coord, center, radius) ||
          (include_source && !include_source(source_coord))) {
        continue;
      }
      fpga::Tile *source_tile = device.getTile(x, y);
      if (!source_tile || !source_tile->cb_type) {
        continue;
      }
      // Resource-side tile views can alias the same physical switchbox.
      // Reverse routing indexes only the tile that owns the lease state.
      if (device.routeTile(*source_tile) != source_tile) {
        continue;
      }
      if (shared_cache &&
          !shared_cache->processed_tiles.insert(source_tile).second) {
        continue;
      }
      for (const auto &[src_key, entries] :
           source_tile->cb_type->dst_by_src.values) {
        ++result.mapping_scan_count;
        if (entries.empty()) {
          continue;
        }
        int src = src_key;
        const std::vector<BackwardResolveArc> *arcs = nullptr;
        std::vector<BackwardResolveArc> uncached_arcs;
        BackwardResolveSourceKey source_key{source_tile->cb_type, src};
        if (shared_cache) {
          auto known = shared_cache->arcs.find(source_key);
          if (known != shared_cache->arcs.end()) {
            arcs = &known->second;
          }
        }
        if (!arcs) {
          for (const fpga::TileJumpTarget &target :
               device.resolveJumpTargets(*source_tile, src)) {
            if (!target.tile || !target.tile->cb_type || target.dst_node < 0) {
              continue;
            }
            uncached_arcs.push_back(
                BackwardResolveArc{target.tile->coord - source_tile->coord,
                                   target.dst_node, target.jump_node});
          }
          // A source near the chip edge may have no physical landing. That
          // empty result is coordinate-specific, not proof that this CB type
          // has no such wire anywhere on the chip.
          if (shared_cache && !uncached_arcs.empty()) {
            arcs = &shared_cache->arcs
                        .emplace(source_key, std::move(uncached_arcs))
                        .first->second;
          } else {
            arcs = &uncached_arcs;
          }
        }
        if (shared_cache) {
          for (const BackwardResolveArc &arc : *arcs) {
            fpga::Coord target_coord = source_tile->coord + arc.delta;
            BackwardResolveKey target{target_coord.x, target_coord.y, arc.dst};
            shared_cache->incoming[target].push_back(
                BackwardResolveSource{source_tile, src, arc.route_jump});
          }
          continue;
        }
        std::unordered_set<BackwardResolveKey, BackwardResolveKeyHash>
            source_targets;
        for (const BackwardResolveArc &arc : *arcs) {
          fpga::Coord target_coord = source_tile->coord + arc.delta;
          if (!inDockWindow(target_coord, center, radius)) {
            continue;
          }
          BackwardResolveKey key{target_coord.x, target_coord.y, arc.dst};
          if (!source_targets.insert(key).second) {
            continue;
          }
          result.sources[key].push_back(
              BackwardResolveSource{source_tile, src, arc.route_jump});
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
                       bool include_external_sources) {
  if (index.shared_cache && include_external_sources) {
    // Do not populate/reuse the radius-filtered view for this lookup. A prior
    // bounded lookup can legitimately be empty for an existing incoming wire.
    auto incoming = index.shared_cache->incoming.find(key);
    return incoming == index.shared_cache->incoming.end() ? nullptr
                                                        : &incoming->second;
  }
  if (index.shared_cache && index.resolved_keys.insert(key).second) {
    auto incoming = index.shared_cache->incoming.find(key);
    if (incoming != index.shared_cache->incoming.end()) {
      for (const BackwardResolveSource &source : incoming->second) {
        if (!source.tile ||
            !inDockWindow(source.tile->coord, index.center, index.radius)) {
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

DockingResult dockGrounding(fpga::Tile &forward_tile, int forward_dst,
                            const std::string &forward_dst_wire,
                            fpga::Tile &target_tile, NodeMask pin_nodes,
                            int max_depth, int radius,
                            bool trace_backward_attempts,
                            NodeMask reserved_terminal_joints,
                            BackwardResolveIndex *backward_index,
                            std::span<const fpga::Coord> prefix_tiles,
                            bool source_local,
                            const DockingPreemptionSearch *preemption) {
  if (max_depth < 0) max_depth = dockingHopLimit(radius);
  // Standalone calls build topology once for both searches. The production
  // router already supplies its shared complete device index.
  BackwardResolveIndex standalone_index;
  if (!backward_index) {
    auto &device = fpga::Device::current();
    standalone_index = buildBackwardResolveIndex(device,
        {device.size_width / 2, device.size_height / 2},
        std::max(device.size_width, device.size_height));
    standalone_index.center = target_tile.coord;
    standalone_index.radius = radius;
    backward_index = &standalone_index;
  }
  auto retain_previous = [](DockingResult &expanded, DockingResult &result) {
    // Counters describe all work; terminal counts describe the unchanged
    // terminal alternatives. The topology itself was indexed only once.
    expanded.forward_push_count += result.forward_push_count;
    expanded.backward_push_count += result.backward_push_count;
    expanded.forward_pop_count += result.forward_pop_count;
    expanded.backward_pop_count += result.backward_pop_count;
    expanded.backward_missing_prev_dst_count += result.backward_missing_prev_dst_count;
    expanded.backward_topology_reject_count += result.backward_topology_reject_count;
    expanded.backward_busy_reject_count += result.backward_busy_reject_count;
    expanded.backward_seen_reject_count += result.backward_seen_reject_count;
    expanded.backward_deadend_count += result.backward_deadend_count;
    expanded.backward_deadend_reject_count += result.backward_deadend_reject_count;
    expanded.tile_visit_reject_count += result.tile_visit_reject_count;
    expanded.backward_mapping_scan_count = std::max(
        result.backward_mapping_scan_count, expanded.backward_mapping_scan_count);
    expanded.backward_mapping_reaches_count = std::max(
        result.backward_mapping_reaches_count, expanded.backward_mapping_reaches_count);
    auto prepend = [](auto &to, auto &from) {
      to.insert(to.begin(), std::make_move_iterator(from.begin()),
                std::make_move_iterator(from.end()));
    };
    prepend(expanded.backward_attempts, result.backward_attempts);
    if (!expanded.success) {
      // Keep the previously reachable bridge/terminal proposals first rather
      // than replacing them with a different truncated frontier.
      prepend(expanded.blocked_bridges, result.blocked_bridges);
      prepend(expanded.forward_frontier, result.forward_frontier);
      prepend(expanded.backward_frontier, result.backward_frontier);
      if (result.blocked_terminal_reachable) {
        expanded.blocked_terminal_reachable = true;
        expanded.blocked_terminal_fragments = std::move(result.blocked_terminal_fragments);
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
  for (bool forward_preemption : {false, true}) {
    if (forward_preemption && (!preemption || !preemption->allow_forward_expansion)) break;
    for (bool bounded_reverse : {true, false}) {
      auto candidate = dockGroundingImpl(
          forward_tile, forward_dst, forward_dst_wire, target_tile, pin_nodes,
          DockingOptions{max_depth, radius, false, 0,
                         source_local && bounded_reverse ? 0 : 32,
                         trace_backward_attempts, reserved_terminal_joints,
                         backward_index, prefix_tiles, bounded_reverse,
                         source_local, preemption, forward_preemption});
      if (!tried) local_success = candidate.success;
      if (tried) retain_previous(candidate, result);
      result = std::move(candidate);
      tried = true;
      if (result.success) break;
    }
    if (result.success) break;
  }
  if (std::getenv("SCALEPNR_AUDIT_DOCK_WINDOW")) {
    std::fprintf(stderr, "DOCK_WINDOW_RESULT from=(%d,%d)/%d target=(%d,%d) local_success=%d expanded=%d success=%d bpush=%d\n",
                 forward_tile.coord.x, forward_tile.coord.y, forward_dst,
                 target_tile.coord.x, target_tile.coord.y, local_success,
                 !local_success, result.success, result.backward_push_count);
  }
  return result;
}

// Validate an input against its existing driver/tree until success or cancellation.
// Unlike placement enumeration, reaching the driver can finish immediately.
BackwardTakeoffRoute routeBackwardToInput(
    fpga::Tile &target_tile, NodeMask pin_nodes, fpga::Coord source_hint,
    int radius, const BackwardTakeoffProbe &source_probe,
    BackwardResolveIndex *backward_index,
    const BackwardTakeoffCancel &cancel,
    const BackwardTakeoffStateView *state_view,
    const std::vector<BackwardRouteAnchor> *anchors, int max_depth) {
  return routeBackwardToTakeoff(target_tile, pin_nodes, source_hint, max_depth, radius,
                               -1, source_probe, backward_index, cancel, 0, 0, 0,
                               state_view, anchors, source_probe);
}

BackwardTakeoffRoute routeBackwardToTakeoff(
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
    const BackwardTakeoffBoundary &boundary_probe) {
  BackwardTakeoffRoute result;
  // This reads the active per-net trace dynamically: stagnation may enable it
  // in the middle of this search. It never participates in route selection.
  auto trace = []() -> std::ostream* {
    return RouteCongestionTrace::current ? &RouteCongestionTrace::current->stream() : nullptr;
  };
  struct SearchReport {
    BackwardTakeoffRoute& result;
    ~SearchReport() {
      if (RouteCongestionTrace::current) {
        auto& out = RouteCongestionTrace::current->stream();
        out << "BACKWARD_END success=" << result.success << " expanded=" << result.expanded
            << " edges=" << result.incoming_edges << " probes=" << result.probe_calls
            << " scanned=" << result.probe_candidates_scanned
            << " remaining_frontier=" << result.remaining_frontier
            << " boundary_released=" << result.boundary_released << '\n';
        out.flush();
      }
    }
  } search_report{result};
  if (auto* out = trace()) *out << "BACKWARD_BEGIN target=(" << target_tile.coord.x << ','
      << target_tile.coord.y << ") source_hint=(" << source_hint.x << ',' << source_hint.y
      << ") radius=" << radius << " source_radius=" << source_radius
      << " probe_offset=" << probe_offset << " max_probes=" << max_probes << '\n';
  result.failure_tile = &target_tile;
  if (!target_tile.cb_type || pin_nodes == NodeMask{} ||
      (!probe && !path_probe && (!anchors || anchors->empty())) ||
      radius <= 0) {
    return result;
  }

  std::vector<Node> nodes;
  std::vector<int> frontier;
  // Completing a fixed prefix must minimize the suffix we permanently lease.
  // Depth-first traversal can accept a chip-spanning detour before trying a
  // nearby alternative, starving later fanouts despite a short free path.
  // Anchor and takeoff proofs visit equal-cost hops breadth-first, preserving
  // angle order within a hop and relocation's distance-ranked probing.
  CombinatorialTileVisits tile_visits;
  int diagnostic_node_index = -1;
  int diagnostic_node_depth = -1;
  std::unordered_map<Key, int, BackwardResolveKeyHash> seen;
  std::unordered_set<TakeoffProbeKey, TakeoffProbeKeyHash> probed_takeoffs;
  size_t frontier_cursor = 0;
  Node last_congestion;
  std::unordered_set<TakeoffProbeKey, TakeoffProbeKeyHash>
      preferred_probed_takeoffs;
  struct FrontierTakeoff {
    BackwardResolveSource source;
    int node_index = -1;
  };
  std::vector<std::vector<FrontierTakeoff>> takeoffs_by_distance;
  std::unordered_set<TakeoffProbeKey, TakeoffProbeKeyHash> recorded_takeoffs;
  std::unordered_map<fpga::Tile *, NodeMask> anchor_dsts_by_tile;
  std::unordered_map<Key, const BackwardRouteAnchor *, BackwardResolveKeyHash>
      anchor_by_node;
  if (anchors) {
    // Anchors arrive newest first. Keep the first exact landing and expose all
    // candidate destination nodes as a per-tile mask for numeric intersection.
    for (const BackwardRouteAnchor &anchor : *anchors) {
      if (!anchor.tile || anchor.dst < 0 || anchor.dst >= CB_MAX_NODES) {
        continue;
      }
      Key key{anchor.tile->coord.x, anchor.tile->coord.y, anchor.dst};
      anchor_by_node.try_emplace(key, &anchor);
      anchor_dsts_by_tile[anchor.tile].setBit(anchor.dst);
    }
  }
  auto state = [&](fpga::Tile &tile) -> const fpga::CBState & {
    if (state_view) {
      auto found = state_view->find(&tile);
      if (found != state_view->end()) {
        return found->second;
      }
    }
    return tile.cb;
  };
  auto retain_diagnostic_path = [&]() {
    if (last_congestion.tile) {
      // This blocked hop is diagnostic only: never insert it into the search
      // frontier or live leases. Keep its connected suffix for the failure PNG.
      result.diagnostic_fragments = suffixFromNode(nodes, last_congestion.parent);
      result.diagnostic_fragments.insert(result.diagnostic_fragments.begin(),
                                         last_congestion.edge_from_parent);
      result.failure_tile = last_congestion.tile;
      result.failure_dst = last_congestion.dst;
      return;
    }
    if (diagnostic_node_index >= 0 &&
        static_cast<size_t>(diagnostic_node_index) < nodes.size()) {
      result.diagnostic_fragments =
          suffixFromNode(nodes, diagnostic_node_index);
      result.failure_tile = nodes[diagnostic_node_index].tile;
      result.failure_dst = nodes[diagnostic_node_index].dst;
    }
  };
  // Packing probes are more expensive than numeric expansion. A caller-owned
  // cursor advances this bounded window across retries instead of rescanning
  // the same rejected prefix or probing every reached frontier at once.
  const size_t max_takeoff_probes =
      max_probes == 0 ? std::numeric_limits<size_t>::max()
                      : std::max<size_t>(2, max_probes);
  const bool restrict_to_physical_dsts =
      target_tile.incoming_dst_nodes != NodeMask{};
  pin_nodes.for_each_set_bit([&](int pin) {
    for (const fpga::CBType::TerminalEntry &path :
         target_tile.cb_type->terminalEntries(pin)) {
      // A shared CB type can contain edge-only arrivals. Seed reverse routing
      // only from destination nodes physically reachable at this coordinate.
      if (restrict_to_physical_dsts &&
          !target_tile.incoming_dst_nodes.testBit(path.dst)) {
        if (auto* out = trace()) *out << "BACKWARD_SEED local=" << pin << " dst=" << path.dst
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
      if (const std::string *name =
              target_tile.cb_type->nodeName(fpga::CB_NODE_DST, path.dst)) {
        enter.dst_wire_name = *name;
      }
      fpga::Wire tile_pin;
      tile_pin.type = fpga::Wire::WIRE_TILE_PIN;
      tile_pin.from = target_tile.coord;
      tile_pin.to = target_tile.coord;
      tile_pin.local = pin;
      tile_pin.pos = ROUTE_POS_TRANSIT;
      // Even a fully blocked endpoint is a concrete attempted path. Retain it
      // unless reverse expansion later provides a deeper diagnostic suffix.
      if (result.diagnostic_fragments.empty()) {
        result.diagnostic_fragments = {enter, tile_pin};
      }
      if (target_tile.isPinNodeLeased(pin) ||
          !canLeaseIn(state(target_tile), path.dst, pin, path.joint,
                      path.joint2)) {
        if (trace()) RouteCongestionTrace::current->blocked(target_tile, "backward_seed_busy",
            {{fpga::CB_NODE_LOCAL, pin}, {fpga::CB_NODE_DST, path.dst},
             {fpga::CB_NODE_JOINT, path.joint}, {fpga::CB_NODE_JOINT, path.joint2}},
            &state(target_tile));
        continue;
      }
      Node seed{&target_tile, path.dst, enter.dst_wire_name, -1, {}, 0,
                {enter, tile_pin}};
      if (!tile_visits.append(-1, &target_tile, seed.tile_visits_root)) {
        return false;
      }
      Key key = nodeKey(seed);
      if (seen.emplace(key, static_cast<int>(nodes.size())).second) {
        if (auto* out = trace()) *out << "BACKWARD_SEED local=" << pin << " dst=" << path.dst
            << " joint=" << path.joint << " joint2=" << path.joint2
            << " result=enqueue index=" << nodes.size() << '\n';
        frontier.push_back(static_cast<int>(nodes.size()));
        nodes.push_back(std::move(seed));
      }
    }
    return false;
  });
  if (frontier.empty()) {
    return result;
  }

  BackwardResolveIndex local_index;
  BackwardResolveIndex *index = prebuilt_backward_index;
  if (!index) {
    local_index = buildBackwardResolveIndex(fpga::Device::current(),
                                            target_tile.coord, radius);
    index = &local_index;
  }
  auto accept_takeoff = [&](const BackwardResolveSource &source,
                            int node_index,
                            const BackwardTakeoffProbe &selected_probe,
                            bool use_path_probe = false) {
    const Node &node = nodes[static_cast<size_t>(node_index)];
    std::vector<fpga::Wire> proposed;
    auto materialize = [&](const BackwardTakeoffChoice &choice)
        -> const std::vector<fpga::Wire> & {
      if (proposed.empty()) {
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
      return proposed;
    };
    BackwardTakeoffChoice takeoff;
    if (auto* out = trace()) *out << "PROBE_BEGIN tile=(" << source.tile->coord.x << ','
        << source.tile->coord.y << ") src=" << source.src << " parent=" << node_index
        << " placement=" << use_path_probe << '\n';
    bool accepted = use_path_probe && path_probe
        ? path_probe(*source.tile, source.src, takeoff, std::ref(materialize))
        : selected_probe(*source.tile, source.src, takeoff);
    if (auto* out = trace()) *out << "PROBE_END tile=(" << source.tile->coord.x << ','
        << source.tile->coord.y << ") src=" << source.src << " accepted=" << accepted
        << " local=" << takeoff.local << " joint=" << takeoff.joint
        << " joint2=" << takeoff.joint2 << " counted=" << takeoff.counts_toward_probe_limit << '\n';
    if (takeoff.counts_toward_probe_limit) {
      ++result.probe_calls;
    }
    if (!accepted || takeoff.local < 0 ||
        !canLeaseJump(state(*source.tile), takeoff.local, source.src,
                      takeoff.joint, takeoff.joint2, true)) {
      if (auto* out = trace()) *out << "PROBE_REJECT reason="
          << (!accepted ? "placement_or_input_proof" : takeoff.local < 0 ? "no_local" : "takeoff_busy") << '\n';
      return false;
    }
    materialize(takeoff);
    fpga::Wire &edge = proposed.front();
    edge.from_wire_name = fromWireName(
        *source.tile, fpga::CB_NODE_LOCAL, takeoff.local, source.src,
        takeoff.joint, {});
    edge.src_wire_name = srcWireName(
        *source.tile, fpga::CB_NODE_LOCAL, takeoff.local, source.src,
        takeoff.joint, {});
    edge.dst_wire_name = node.dst_wire;
    result.fragments = std::move(proposed);
    if (!combinatorialRouteVisitsValid(result.fragments)) {
      if (auto* out = trace()) *out << "PROBE_REJECT reason=tile_visit_limit\n";
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
  auto accept_anchor = [&](const BackwardResolveSource &source,
                           int node_index) {
    if (!anchors || !source.tile || !source.tile->cb_type) {
      return false;
    }
    auto tile_anchors = anchor_dsts_by_tile.find(source.tile);
    if (tile_anchors == anchor_dsts_by_tile.end()) {
      return false;
    }
    source.tile->cb_type->ensureDerivedMasks();
    NodeMask candidate_dsts = tile_anchors->second &
                              source.tile->cb_type->dsts_reaching_src.get(source.src)
                                  .jump;
    return candidate_dsts.for_each_set_bit([&](int anchor_dst) {
      Key key{source.tile->coord.x, source.tile->coord.y, anchor_dst};
      auto found = anchor_by_node.find(key);
      if (found == anchor_by_node.end()) {
        return false;
      }
      const BackwardRouteAnchor &anchor = *found->second;
      ++result.anchor_candidates;
      int joint2 = -1;
      int joint = selectJointToSrc(*source.tile, fpga::CB_NODE_DST, anchor.dst,
                                   source.src, &joint2);
      if (joint == -2 ||
          !canLeaseJump(state(*source.tile), anchor.dst, source.src, joint,
                        joint2, true)) {
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
      edge.src_wire_name = srcWireName(*source.tile, fpga::CB_NODE_DST,
                                       anchor.dst, source.src, joint,
                                       anchor.dst_wire);
      edge.dst_wire_name = node.dst_wire;
      result.fragments = {edge};
      std::vector<fpga::Wire> suffix = suffixFromNode(nodes, node_index);
      result.fragments.insert(result.fragments.end(), suffix.begin(),
                              suffix.end());
      if (!combinatorialRouteVisitsValid(result.fragments)) {
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
  auto probe_takeoffs = [&]() {
    for (auto &distance_bucket : takeoffs_by_distance) {
      for (const FrontierTakeoff &candidate : distance_bucket) {
        if (cancel && cancel()) {
          return true;
        }
        TakeoffProbeKey key{candidate.source.tile, candidate.source.src};
        if (!probed_takeoffs.insert(key).second) {
          continue;
        }
        if (source_radius >= 0 &&
            std::max(std::abs(candidate.source.tile->coord.x - source_hint.x),
                     std::abs(candidate.source.tile->coord.y - source_hint.y)) >
                source_radius) {
          continue;
        }
        if (candidate_index++ < probe_offset) {
          continue;
        }
        // A full budget only interrupts when another candidate needs probing;
        // rejecting the final candidate can still exhaust the free component.
        if (result.probe_calls >= max_takeoff_probes) {
          return true;
        }
        ++result.probe_candidates_scanned;
        if (accept_takeoff(candidate.source, candidate.node_index, probe, true)) {
          return true;
        }
      }
      distance_bucket.clear();
    }
    return false;
  };
  size_t layer_end = frontier.size();

  // This is combinatorial reverse search, not the direction-driven engine.
  // Finish each depth layer so one long lane cannot starve other free entries.
  while (frontier_cursor < frontier.size() &&
         (max_expansions == 0 || result.expanded < max_expansions)) {
    if (cancel && cancel()) {
      result.remaining_frontier = frontier.size() - frontier_cursor;
      if (auto* out = trace()) *out << "BACKWARD_STOP reason=cancelled\n";
      retain_diagnostic_path();
      return result;
    }
    if (boundary_probe && frontier_cursor == layer_end) {
      // Probe early, but leave live owners untouched while free continuations
      // remain. Keep the bounded blocker sample for actual search exhaustion.
      if (probe_takeoffs()) {
        result.remaining_frontier = frontier.size() - frontier_cursor;
        retain_diagnostic_path();
        return result;
      }
      layer_end = frontier.size();
    }
    int node_index = frontier[frontier_cursor++];
    const Node node = nodes[static_cast<size_t>(node_index)];
    if (auto* out = trace()) *out << "BACKWARD_POP index=" << node_index << " parent=" << node.parent
        << " tile=(" << node.tile->coord.x << ',' << node.tile->coord.y << ") dst=" << node.dst
        << " depth=" << node.depth << " frontier=" << frontier.size() - frontier_cursor << '\n';
    // Keep the deepest concrete failed suffix; later shallow frontier pops
    // must not erase the useful path that explains where reverse search got.
    if (node.depth > diagnostic_node_depth) {
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
    ++result.expanded;
    Key current_key = nodeKey(node);
    if (node_index == diagnostic_node_index && index->shared_cache) {
      result.diagnostic_key_was_resolved =
          index->resolved_keys.contains(current_key);
      auto cached = index->shared_cache->incoming.find(current_key);
      if (cached != index->shared_cache->incoming.end()) {
        result.diagnostic_cached_sources = cached->second.size();
        result.diagnostic_window_sources = static_cast<size_t>(std::count_if(
            cached->second.begin(), cached->second.end(),
            [&](const BackwardResolveSource &source) {
              return source.tile &&
                     inDockWindow(source.tile->coord, index->center,
                                  index->radius);
            }));
      }
    }
    const std::vector<BackwardResolveSource> *incoming =
        resolveBackwardSources(*index, current_key);
    if (!incoming) {
      if (auto* out = trace()) *out << "BACKWARD_REJECT index=" << node_index << " reason=no_incoming\n";
      continue;
    }
    if (node_index == diagnostic_node_index) {
      result.diagnostic_incoming_sources = incoming->size();
    }
    if (max_depth > 0 && node.depth >= max_depth) {
      if (node_index == diagnostic_node_index) {
        result.diagnostic_depth_limit_reached = true;
      }
      continue;
    }

    // Reverse traversal is direction-led without runtime sorting. Sources in
    // the preferred octant are visited first, followed by neighboring octants.
    int wanted_x = source_hint.x - node.tile->coord.x;
    int wanted_y = source_hint.y - node.tile->coord.y;
    auto direction = [](int dx, int dy) {
      int sx = (dx > 0) - (dx < 0);
      int sy = (dy > 0) - (dy < 0);
      static constexpr int table[3][3] = {{7, 6, 5}, {0, -1, 4}, {1, 2, 3}};
      return table[sy + 1][sx + 1];
    };
    int wanted = direction(wanted_x, wanted_y);
    static constexpr int offsets[8] = {0, -1, 1, -2, 2, -3, 3, 4};
    std::vector<int> children;
    for (int offset : offsets) {
      int selected_direction = wanted < 0 ? -1 : (wanted + offset + 8) & 7;
      for (const BackwardResolveSource &source : *incoming) {
        if (!source.tile || !source.tile->cb_type) {
          continue;
        }
        fpga::Coord source_delta = source.tile->coord - node.tile->coord;
        const int source_direction = direction(source_delta.x, source_delta.y);
        // Local role continuity has no octant. Visit it once in the first
        // bucket, even when the retained anchor lies in a different tile.
        if (wanted >= 0 && (source_direction < 0 ? offset != 0 :
                            source_direction != selected_direction)) {
          continue;
        }
        int source_visits_root = node.tile_visits_root;
        if (source.tile != node.tile && !tile_visits.append(node.tile_visits_root, source.tile,
                                source_visits_root)) {
          if (auto* out = trace()) *out << "BACKWARD_REJECT index=" << node_index
              << " source=(" << source.tile->coord.x << ',' << source.tile->coord.y
              << ") src=" << source.src << " reason=tile_visit_limit\n";
          ++result.tile_visit_reject_count;
          continue;
        }
        int src = source.src;
        ++result.incoming_edges;
        if (accept_anchor(source, node_index)) {
          return result;
        }
        int source_dx = std::abs(source.tile->coord.x - source_hint.x);
        int source_dy = std::abs(source.tile->coord.y - source_hint.y);
        int source_distance = std::max(source_dx, source_dy);
        TakeoffProbeKey takeoff_key{source.tile, src};
        if (preferred_probe && probe_offset == 0 &&
            preferred_probed_takeoffs.insert(takeoff_key).second) {
          ++result.takeoff_candidates;
          ++result.probe_candidates_scanned;
          if (accept_takeoff(source, node_index, preferred_probe)) {
            return result;
          }
        }
        if ((probe || path_probe) &&
            recorded_takeoffs.insert(takeoff_key).second) {
          if (takeoffs_by_distance.size() <=
              static_cast<size_t>(source_distance)) {
            takeoffs_by_distance.resize(static_cast<size_t>(source_distance) +
                                        1);
          }
          takeoffs_by_distance[static_cast<size_t>(source_distance)].push_back(
              {source, node_index});
          if (source_radius < 0 || source_distance <= source_radius) {
            ++result.takeoff_candidates;
          }
        }
        bool in_source_neighborhood =
            source_radius < 0 || source_distance <= source_radius;
        if (in_source_neighborhood) {
          ++result.source_neighborhood_edges;
        }
        source.tile->cb_type->ensureDerivedMasks();
        NodeMask previous_dsts =
            source.tile->cb_type->dsts_reaching_src.get(src).jump;
        previous_dsts.for_each_set_bit([&](int previous_dst) {
          if (node_index == diagnostic_node_index) {
            ++result.diagnostic_previous_dsts;
          }
          int joint2 = -1;
          int joint = selectJointToSrc(*source.tile, fpga::CB_NODE_DST,
                                       previous_dst, src, &joint2);
          if (joint == -2) {
            if (auto* out = trace()) *out << "BACKWARD_REJECT index=" << node_index
                << " source=(" << source.tile->coord.x << ',' << source.tile->coord.y
                << ") dst=" << previous_dst << " src=" << src << " reason=no_usable_joint\n";
            if (trace()) RouteCongestionTrace::current->blocked(*source.tile, "backward_joint_unavailable",
                {{fpga::CB_NODE_DST, previous_dst}, {fpga::CB_NODE_SRC, src}}, &state(*source.tile));
            return false;
          }
          const fpga::CBState &source_state = state(*source.tile);
          if (!canLeaseJump(source_state, previous_dst, src, joint, joint2)) {
            if (auto* out = trace()) {
              *out << "BACKWARD_EDGE parent=" << node_index << " from=(" << source.tile->coord.x
                   << ',' << source.tile->coord.y << ") dst=" << previous_dst << " src=" << src
                   << " to=(" << node.tile->coord.x << ',' << node.tile->coord.y << ") landing="
                   << node.dst << " result=busy\n";
              RouteCongestionTrace::current->blocked(*source.tile, "backward_takeoff_busy",
                  {{fpga::CB_NODE_DST, previous_dst}, {fpga::CB_NODE_SRC, src},
                   {fpga::CB_NODE_JOINT, joint}, {fpga::CB_NODE_JOINT, joint2}}, &source_state);
            }
            ++result.blocked_reverse_edges;
            // Remember the last rejected numeric hop even after the bounded
            // preemption sample fills; diagnostics must not select an old hop.
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
            // Retain only an angle-ordered sample. Runtime routing remains a
            // numeric mask traversal; this metadata identifies a precise cut.
            if (result.blocked_reverse_frontier.size() < 256) {
              DockingBridgeBlocker blocker;
              blocker.valid = true;
              blocker.tile = source.tile->coord;
              blocker.dst = previous_dst;
              blocker.src = src;
              blocker.joint = joint;
              blocker.joint2 = joint2;
              blocker.landing_tile = node.tile->coord;
              blocker.landing_dst = node.dst;
              blocker.dst_busy = source_state.dst.jump.testBit(previous_dst);
              blocker.src_busy = source_state.src.jump.testBit(src);
              blocker.joint_busy =
                  joint >= 0 && source_state.joint.jump.testBit(joint);
              blocker.joint2_busy =
                  joint2 >= 0 && source_state.joint.jump.testBit(joint2);
              result.blocked_reverse_frontier.push_back(std::move(blocker));
            }
            return false;
          }
          if (node_index == diagnostic_node_index) {
            ++result.diagnostic_free_previous_dsts;
          }
          Node previous{source.tile, previous_dst, {}, node_index, {},
                        node.depth + 1, {}};
          previous.tile_visits_root = source_visits_root;
          if (const std::string *name = source.tile->cb_type->nodeName(
                  fpga::CB_NODE_DST, previous_dst)) {
            previous.dst_wire = *name;
          }
          previous.edge_from_parent.from = source.tile->coord;
          previous.edge_from_parent.to = node.tile->coord;
          previous.edge_from_parent.local = previous_dst;
          previous.edge_from_parent.jump = src;
          previous.edge_from_parent.route_jump = source.route_jump;
          previous.edge_from_parent.dst = node.dst;
          previous.edge_from_parent.joint = joint;
          previous.edge_from_parent.joint2 = joint2;
          previous.edge_from_parent.pos = ROUTE_POS_TRANSIT;
          previous.edge_from_parent.from_wire_name = previous.dst_wire;
          previous.edge_from_parent.src_wire_name = srcWireName(
              *source.tile, fpga::CB_NODE_DST, previous_dst, src, joint, {});
          previous.edge_from_parent.dst_wire_name = node.dst_wire;
          Key key = nodeKey(previous);
          const bool inserted = seen.emplace(key, static_cast<int>(nodes.size())).second;
          if (auto* out = trace()) *out << "BACKWARD_EDGE parent=" << node_index << " from=("
              << source.tile->coord.x << ',' << source.tile->coord.y << ") dst=" << previous_dst
              << " src=" << src << " joint=" << joint << " joint2=" << joint2 << " to=("
              << node.tile->coord.x << ',' << node.tile->coord.y << ") landing=" << node.dst
              << " result=" << (inserted ? "enqueue" : "already_searched")
              << " index=" << seen.at(key) << '\n';
          if (inserted) {
            children.push_back(static_cast<int>(nodes.size()));
            nodes.push_back(std::move(previous));
            if (node_index == diagnostic_node_index) {
              ++result.diagnostic_children;
            }
          }
          return false;
        });
      }
      if (wanted < 0) {
        break;
      }
    }
    // Preserve angle priority within the next layer without runtime sorting.
    frontier.insert(frontier.end(), children.begin(), children.end());
  }

  result.expansion_limit_reached = max_expansions != 0 &&
                                   result.expanded >= max_expansions &&
                                   frontier_cursor < frontier.size();
  result.remaining_frontier = frontier.size() - frontier_cursor;

  // Legacy bounded callers can resume a probe window after a complete walk.
  // Layer-wise callers consume candidates as they arrive instead.
  if (!boundary_probe && probe_offset >= result.takeoff_candidates) {
    probe_offset = 0;
  }
  result.probe_offset_used = probe_offset;
  const bool probes_stopped = probe_takeoffs();
  // A budget or cancellation is not exhaustion. Only a completed free search
  // may change ownership, and a successful cut ends this state view immediately.
  if (boundary_probe && !probes_stopped && result.remaining_frontier == 0 &&
      !(cancel && cancel()) && !result.blocked_reverse_frontier.empty()) {
    result.boundary_released = boundary_probe(result.blocked_reverse_frontier);
  }
  retain_diagnostic_path();
  return result;
}

BackwardTakeoffRoute routeBackwardToAnchor(
    fpga::Tile &target_tile, NodeMask pin_nodes,
    const BackwardRouteAnchor &anchor, int max_depth, int radius,
    BackwardResolveIndex *backward_index,
    const BackwardTakeoffCancel &cancel, size_t max_expansions) {
  std::vector<BackwardRouteAnchor> anchors{anchor};
  return routeBackwardToAnchors(target_tile, pin_nodes, anchors, max_depth,
                                radius, backward_index, cancel,
                                max_expansions);
}

BackwardTakeoffRoute routeBackwardToAnchors(
    fpga::Tile &target_tile, NodeMask pin_nodes,
    const std::vector<BackwardRouteAnchor> &anchors, int max_depth, int radius,
    BackwardResolveIndex *backward_index,
    const BackwardTakeoffCancel &cancel, size_t max_expansions) {
  return routeBackwardToTakeoff(
      target_tile, pin_nodes,
      !anchors.empty() && anchors.front().tile ? anchors.front().tile->coord
                                               : target_tile.coord,
      max_depth, radius,
      -1, {}, backward_index, cancel, max_expansions, 0, 0, nullptr, &anchors);
}

DockingResult dockIOB(fpga::Tile &forward_tile, int forward_dst,
                      const std::string &forward_dst_wire,
                      fpga::Tile &target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts,
                      NodeMask reserved_terminal_joints,
                      std::span<const fpga::Coord> prefix_tiles) {
  constexpr int radius = 12;
  return dockGroundingImpl(
      forward_tile, forward_dst, forward_dst_wire, target_tile, pin_nodes,
      DockingOptions{dockingHopLimit(radius), radius, true, 3, 16, trace_backward_attempts,
                     reserved_terminal_joints, nullptr, prefix_tiles});
}

} // namespace pnr
