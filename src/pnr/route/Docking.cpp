#include "Docking.h"

#include "Device.h"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <string>
#include <unordered_set>

namespace pnr {
namespace {

constexpr int ROUTE_POS_TRANSIT = 1;

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

  forward_nodes.push_back(
      Node{&forward_tile, forward_dst, forward_dst_wire, -1, {}, 0, {}});
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

  auto resolve_backward_key = [&](const Key &key) {
    if (!backward_index || !backward_index->shared_cache ||
        !backward_index->resolved_keys.insert(key).second) {
      return;
    }
    auto incoming = backward_index->shared_cache->incoming.find(key);
    if (incoming == backward_index->shared_cache->incoming.end()) {
      return;
    }
    for (const BackwardResolveSource &source : incoming->second) {
      if (!source.tile ||
          !inDockWindow(source.tile->coord, backward_index->center,
                        backward_index->radius)) {
        continue;
      }
      backward_index->sources[key].push_back(source);
      ++backward_index->mapping_reaches_count;
    }
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
      candidates.push_back(
          ForwardCandidate{src, joint, joint2, src_wire, target});
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
    resolve_backward_key(current_key);
    auto sources_it = backward_index->sources.find(current_key);
    if (sources_it == backward_index->sources.end()) {
      backward_deadends.insert(current_key);
      ++result.backward_deadend_count;
      record_backward_attempt(node.seed_dst, "no_incoming", [&]() {
        return suffixFromNode(backward_nodes, node_index);
      });
      return false;
    }
    const std::vector<BackwardSource> &sources = sources_it->second;
    size_t source_count = sources.size();

    for (size_t source_index = 0; source_index < source_count; ++source_index) {
      int next_depth = node.depth + 1;
      const BackwardSource &source = sources[source_index];
      int src = source.src;
      fpga::Tile *prev_tile = source.tile;
      if (!prev_tile || !prev_tile->cb_type) {
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
