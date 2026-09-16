#include "Docking.h"

#include "Device.h"
#include "RouteSearch.h"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <string>
#include <unordered_set>

namespace pnr {
namespace {

constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;

NodeMask bit(int node) {
  NodeMask result;
  result.setBit(node);
  return result;
}

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
                     int from_node, int src, int *joint2) {
  if (!tile.cb_type) {
    return -2;
  }
  tile.cb_type->ensureDerivedMasks();
  int joint = -1;
  if (joint2) {
    *joint2 = -1;
  }
  if (from_type == fpga::CB_NODE_DST) {
    return tile.cb_type->canJump(from_node, src, src, joint, joint2) ? joint
                                                                     : -2;
  }
  if (from_type == fpga::CB_NODE_LOCAL) {
    return tile.cb_type->canOut(from_node, src, src, joint, joint2) ? joint
                                                                    : -2;
  }
  if (tile.cb_type->joint_src[from_node].jump.testBit(src)) {
    return -1;
  }
  NodeMask joints_to_src = tile.cb_type->src_reachable_joints[src].joint;
  joint = (tile.cb_type->joint_joint[from_node].joint & joints_to_src)
              .firstSetBit();
  return joint >= 0 ? joint : -2;
}

std::string srcWireName(const fpga::Tile &tile, fpga::CBNodeNameType from_type,
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

std::string fromWireName(const fpga::Tile &tile, fpga::CBNodeNameType from_type,
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
  std::string dst_wire;
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
  int radius = 5;
  bool directional = false;
  int side_limit = 0;
  int candidate_limit = 0;
  bool trace_backward_attempts = false;
  NodeMask reserved_terminal_joints{};
  BackwardResolveIndex *prebuilt_backward_index = nullptr;
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
    auto remember_frontier = [](std::vector<DockingFrontierNode> &frontier,
                                const DockingFrontierNode &node) {
      auto existing = std::find_if(
          frontier.begin(), frontier.end(), [&](const DockingFrontierNode &old) {
            return old.coord.x == node.coord.x && old.coord.y == node.coord.y &&
                   old.dst == node.dst;
          });
      if (existing == frontier.end()) {
        frontier.push_back(node);
      } else if (node.depth < existing->depth) {
        existing->depth = node.depth;
      }
    };
    result.forward_frontier.reserve(result.forward_frontier.size() +
                                    forward_seen.size());
    result.backward_frontier.reserve(result.backward_frontier.size() +
                                     backward_seen.size());
    for (const auto &[key, index] : forward_seen) {
      remember_frontier(
          result.forward_frontier,
          DockingFrontierNode{{key.x, key.y}, key.dst,
                              index >= 0 ? forward_nodes[index].depth : 0});
    }
    for (const auto &[key, index] : backward_seen) {
      remember_frontier(
          result.backward_frontier,
          DockingFrontierNode{{key.x, key.y}, key.dst,
                              index >= 0 ? backward_nodes[index].depth : 0});
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
      bool duplicate = std::any_of(
          result.blocked_bridges.begin(), result.blocked_bridges.end(),
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
  if (!tile_visits.append(-1, &forward_tile,
                          forward_seed.tile_visits_root)) {
    return result;
  }
  forward_nodes.push_back(std::move(forward_seed));
  forward_seen[nodeKey(forward_nodes.front())] = 0;
  forward_queue.push_back(0);

  std::vector<Node> blocked_seeds;
  pin_nodes.for_each_set_bit([&](int pin) {
    if (target_tile.isPinNodeLeased(pin)) {
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
      NodeMask path_joints;
      if (path.joint >= 0) {
        path_joints |= bit(path.joint);
      }
      if (path.joint2 >= 0) {
        path_joints |= bit(path.joint2);
      }
      bool terminal_busy =
          (path_joints & options.reserved_terminal_joints) != NodeMask{} ||
          !canLeaseIn(target_tile.cb, dst, pin, path.joint, path.joint2);
      Node seed{
          &target_tile,  dst, enter.dst_wire_name, -1,         {}, 0, suffix,
          terminal_busy, pin, path.joint,          path.joint2};
      seed.seed_dst = dst;
      if (!tile_visits.append(-1, &target_tile, seed.tile_visits_root)) {
        return false;
      }
      if (terminal_busy) {
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
        if (!combinatorialRouteVisitsValid(result.fragments)) {
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
      local_backward_index = buildBackwardResolveIndex(
          device, target_tile.coord, options.radius,
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
        node.tile->cb_type->srcNodes(fpga::CB_NODE_DST, node.dst);
    if (!srcs) {
      return false;
    }

    struct ForwardCandidate {
      uint16_t src = 0;
      int joint = -2;
      int joint2 = -1;
      std::string src_wire;
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
      int joint = selectJointToSrc(*node.tile, fpga::CB_NODE_DST, node.dst, src,
                                   &joint2);
      if (joint == -2) {
        continue;
      }
      std::string src_wire = srcWireName(*node.tile, fpga::CB_NODE_DST,
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
      int candidate_visits_root = -1;
      if (!tile_visits.append(node.tile_visits_root, target.tile,
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
        pending_bridge_blockers.push_back(PendingBridgeBlocker{
            nodeKey(node),
            Key{target.tile->coord.x, target.tile->coord.y, target.dst_node},
            -1, edge, dst_busy, src_busy, joint_busy, joint2_busy, true});
        forward_blocked_landings.insert(
            Key{target.tile->coord.x, target.tile->coord.y, target.dst_node});
        continue;
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
      edge.pos = ROUTE_POS_TRANSIT;
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
        if (!combinatorialRouteVisitsValid(result.fragments)) {
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
        resolveBackwardSources(*backward_index, current_key);
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
      int previous_visits_root = -1;
      if (!tile_visits.append(node.tile_visits_root, prev_tile,
                              previous_visits_root)) {
        ++result.tile_visit_reject_count;
        continue;
      }
      prev_tile->cb_type->ensureDerivedMasks();
      NodeMask prev_dsts = prev_tile->cb_type->dsts_reaching_src[src].jump;
      if (prev_dsts == NodeMask{}) {
        ++result.backward_missing_prev_dst_count;
        record_backward_attempt(node.seed_dst, "no_previous_dst", [&]() {
          return suffixFromNode(backward_nodes, node_index);
        });
        continue;
      }
      prev_dsts.for_each_set_bit([&](int prev_dst) {
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
        std::string src_wire = srcWireName(*prev_tile, fpga::CB_NODE_DST,
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
            prev_tile == &forward_tile && prev_dst == forward_dst;
        if (!canLeaseJump(prev_tile->cb, prev_dst, src, joint, joint2,
                          reaches_forward_anchor)) {
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
        Key key = nodeKey(next);
        // A bounded beam limits queued alternatives, not proof of a completed
        // connection. Check every numeric candidate against the opposite
        // frontier before applying this terminal seed's width quota.
        if (auto front = forward_seen.find(key); front != forward_seen.end()) {
          if (next.blocked_terminal) {
            result.blocked_terminal_reachable = true;
            result.blocked_dst = node.seed_dst;
            result.blocked_pin = next.terminal_pin;
            result.blocked_joint = next.terminal_joint;
            result.blocked_joint2 = next.terminal_joint2;
            record_backward_attempt(node.seed_dst, "blocked_meet",
                                    candidate_fragments);
            return true;
          }
          result.fragments = pathToNode(forward_nodes, front->second);
          std::vector<fpga::Wire> suffix = candidate_fragments();
          result.fragments.insert(result.fragments.end(), suffix.begin(),
                                  suffix.end());
          if (!combinatorialRouteVisitsValid(result.fragments)) {
            ++result.tile_visit_reject_count;
            result.fragments.clear();
            return false;
          }
          result.success = true;
          record_backward_attempt(node.seed_dst, "meet", candidate_fragments);
          return true;
        }
        // The beam bounds queued exploration, but it must not hide a numeric
        // landing separated from the forward frontier by one occupied edge.
        const bool preserves_blocked_bridge =
            forward_blocked_landings.contains(key);
        if (!preserves_blocked_bridge &&
            !backward_has_capacity(node, next_depth)) {
          return true;
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
      std::string src_wire = srcWireName(*node.tile, fpga::CB_NODE_DST,
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
  probe_initial_forward_blockers();

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
        result.blocked_terminal_reachable = true;
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
          if (shared_cache) {
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
                       const BackwardResolveKey &key) {
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
                            BackwardResolveIndex *backward_index) {
  return dockGroundingImpl(
      forward_tile, forward_dst, forward_dst_wire, target_tile, pin_nodes,
      DockingOptions{max_depth, radius, false, 0, 32, trace_backward_attempts,
                     reserved_terminal_joints, backward_index});
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
    const BackwardTakeoffProbe &preferred_probe) {
  BackwardTakeoffRoute result;
  result.failure_tile = &target_tile;
  if (!target_tile.cb_type || pin_nodes == NodeMask{} ||
      (!probe && (!anchors || anchors->empty())) ||
      radius <= 0) {
    return result;
  }

  std::vector<Node> nodes;
  std::vector<int> frontier;
  CombinatorialTileVisits tile_visits;
  int diagnostic_node_index = -1;
  int diagnostic_node_depth = -1;
  std::unordered_map<Key, int, BackwardResolveKeyHash> seen;
  std::unordered_set<TakeoffProbeKey, TakeoffProbeKeyHash> probed_takeoffs;
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
        continue;
      }
      Node seed{&target_tile, path.dst, enter.dst_wire_name, -1, {}, 0,
                {enter, tile_pin}};
      if (!tile_visits.append(-1, &target_tile, seed.tile_visits_root)) {
        return false;
      }
      Key key = nodeKey(seed);
      if (seen.emplace(key, static_cast<int>(nodes.size())).second) {
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
                            const BackwardTakeoffProbe &selected_probe) {
    BackwardTakeoffChoice takeoff;
    bool accepted = selected_probe(*source.tile, source.src, takeoff);
    if (takeoff.counts_toward_probe_limit) {
      ++result.probe_calls;
    }
    if (!accepted || takeoff.local < 0 ||
        !canLeaseJump(state(*source.tile), takeoff.local, source.src,
                      takeoff.joint, takeoff.joint2, true)) {
      return false;
    }
    const Node &node = nodes[static_cast<size_t>(node_index)];
    fpga::Wire edge;
    edge.from = source.tile->coord;
    edge.to = node.tile->coord;
    edge.local = takeoff.local;
    edge.jump = source.src;
    edge.route_jump = source.route_jump;
    edge.dst = node.dst;
    edge.joint = takeoff.joint;
    edge.joint2 = takeoff.joint2;
    edge.pos = 0;
    edge.from_wire_name = fromWireName(
        *source.tile, fpga::CB_NODE_LOCAL, takeoff.local, source.src,
        takeoff.joint, {});
    edge.src_wire_name = srcWireName(
        *source.tile, fpga::CB_NODE_LOCAL, takeoff.local, source.src,
        takeoff.joint, {});
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
                              source.tile->cb_type->dsts_reaching_src[source.src]
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

  while (!frontier.empty() &&
         (max_expansions == 0 || result.expanded < max_expansions)) {
    if (cancel && cancel()) {
      retain_diagnostic_path();
      return result;
    }
    int node_index = frontier.back();
    frontier.pop_back();
    const Node node = nodes[static_cast<size_t>(node_index)];
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
        if (wanted >= 0 && direction(source_delta.x, source_delta.y) !=
                               selected_direction) {
          continue;
        }
        int source_visits_root = -1;
        if (!tile_visits.append(node.tile_visits_root, source.tile,
                                source_visits_root)) {
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
        if (probe && recorded_takeoffs.insert(takeoff_key).second) {
          if (takeoffs_by_distance.size() <=
              static_cast<size_t>(source_distance)) {
            takeoffs_by_distance.resize(static_cast<size_t>(source_distance) +
                                        1);
          }
          takeoffs_by_distance[static_cast<size_t>(source_distance)].push_back(
              {source, node_index});
        }
        bool in_source_neighborhood =
            source_radius < 0 || source_distance <= source_radius;
        if (in_source_neighborhood) {
          ++result.source_neighborhood_edges;
        }
        source.tile->cb_type->ensureDerivedMasks();
        NodeMask previous_dsts =
            source.tile->cb_type->dsts_reaching_src[src].jump;
        previous_dsts.for_each_set_bit([&](int previous_dst) {
          if (node_index == diagnostic_node_index) {
            ++result.diagnostic_previous_dsts;
          }
          int joint2 = -1;
          int joint = selectJointToSrc(*source.tile, fpga::CB_NODE_DST,
                                       previous_dst, src, &joint2);
          if (joint == -2) {
            return false;
          }
          const fpga::CBState &source_state = state(*source.tile);
          if (!canLeaseJump(source_state, previous_dst, src, joint, joint2)) {
            ++result.blocked_reverse_edges;
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
          if (seen.emplace(key, static_cast<int>(nodes.size())).second) {
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
    // Children were generated in angle-priority order. Push them in reverse
    // so the depth-first frontier tests the preferred continuation first.
    for (auto child = children.rbegin(); child != children.rend(); ++child) {
      frontier.push_back(*child);
    }
  }

  result.expansion_limit_reached = max_expansions != 0 &&
                                   result.expanded >= max_expansions &&
                                   !frontier.empty();
  result.remaining_frontier = frontier.size();

  // Complete the reverse walk before probing placement. Distance
  // buckets then select the route-proven frontier nearest to the old source;
  // accepting an early destination-side frontier would move the source much
  // farther than the same search actually requires.
  for (size_t distance = 0; distance < takeoffs_by_distance.size(); ++distance) {
    if (source_radius >= 0 && static_cast<int>(distance) > source_radius) {
      continue;
    }
    result.takeoff_candidates += takeoffs_by_distance[distance].size();
  }
  if (probe_offset >= result.takeoff_candidates) {
    probe_offset = 0;
  }
  result.probe_offset_used = probe_offset;
  size_t candidate_index = 0;
  bool probe_budget_exhausted = false;
  for (const std::vector<FrontierTakeoff> &distance_bucket :
       takeoffs_by_distance) {
    for (const FrontierTakeoff &candidate : distance_bucket) {
      if (cancel && cancel()) {
        retain_diagnostic_path();
        return result;
      }
      TakeoffProbeKey key{candidate.source.tile, candidate.source.src};
      if (!probed_takeoffs.insert(key).second) {
        continue;
      }
      int source_dx =
          std::abs(candidate.source.tile->coord.x - source_hint.x);
      int source_dy =
          std::abs(candidate.source.tile->coord.y - source_hint.y);
      int source_distance = std::max(source_dx, source_dy);
      if (source_radius >= 0 && source_distance > source_radius) {
        continue;
      }
      if (candidate_index++ < probe_offset) {
        continue;
      }
      ++result.probe_candidates_scanned;
      if (accept_takeoff(candidate.source, candidate.node_index, probe)) {
        return result;
      }
      if (result.probe_calls >= max_takeoff_probes) {
        probe_budget_exhausted = true;
        break;
      }
    }
    if (probe_budget_exhausted) {
      break;
    }
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
                      NodeMask reserved_terminal_joints) {
  return dockGroundingImpl(
      forward_tile, forward_dst, forward_dst_wire, target_tile, pin_nodes,
      DockingOptions{12, 12, true, 3, 16, trace_backward_attempts,
                     reserved_terminal_joints, nullptr});
}

} // namespace pnr
