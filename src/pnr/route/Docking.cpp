#include "Docking.h"

#include "Device.h"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <map>
#include <set>
#include <string>

namespace pnr {
namespace {

constexpr int ROUTE_POS_TRANSIT = 1;

NodeMask bit(int node)
{
    return NodeMask{0, 1} << node;
}

bool inDockWindow(const fpga::Coord& coord, const fpga::Coord& target, int radius)
{
    return std::abs(coord.x - target.x) <= radius && std::abs(coord.y - target.y) <= radius;
}

bool leaseJump(fpga::CBState& cb, int dst, int src, bool allow_existing_dst = false)
{
    NodeMask dst_bit = bit(dst);
    NodeMask src_bit = bit(src);
    if ((cb.src.jump & src_bit) != NodeMask{}) {
        return false;
    }
    if (!allow_existing_dst && (cb.dst.jump & dst_bit) != NodeMask{}) {
        return false;
    }
    cb.dst.jump |= dst_bit;
    cb.src.jump |= src_bit;
    return true;
}

bool leaseIn(fpga::CBState& cb, int dst, int local, int joint)
{
    NodeMask dst_bit = bit(dst);
    NodeMask local_bit = bit(local);
    NodeMask joint_bit = joint >= 0 ? bit(joint) : NodeMask{};
    if ((cb.dst.jump & dst_bit) != NodeMask{}
        || (cb.local.local & local_bit) != NodeMask{}
        || (joint >= 0 && (cb.joint.jump & joint_bit) != NodeMask{})) {
        return false;
    }
    cb.dst.jump |= dst_bit;
    cb.local.local |= local_bit;
    if (joint >= 0) {
        cb.joint.jump |= joint_bit;
    }
    return true;
}

std::vector<int> entryJoints(const fpga::CBType& cb_type, int dst, int local)
{
    const_cast<fpga::CBType&>(cb_type).ensureDerivedMasks();
    std::vector<int> joints;
    auto add = [&](int joint) {
        if (std::find(joints.begin(), joints.end(), joint) == joints.end()) {
            joints.push_back(joint);
        }
    };

    if ((cb_type.dst_local[dst].local & bit(local)) != NodeMask{}) {
        add(-1);
    }
    NodeMask joints_to_local = cb_type.local_reachable_joints[local].joint;

    NodeMask one_joint_paths = cb_type.dst_joint[dst].joint & joints_to_local;
    one_joint_paths.for_each_set_bit([&](int joint) {
        add(joint);
        return false;
    });
    NodeMask dst_joints = cb_type.dst_joint[dst].joint;
    dst_joints.for_each_set_bit([&](int first_joint) {
        NodeMask second_joints = cb_type.joint_joint[first_joint].joint & joints_to_local;
        second_joints.for_each_set_bit([&](int second_joint) {
            add(second_joint);
            return false;
        });
        return false;
    });
    return joints;
}

int selectJointToSrc(const fpga::Tile& tile, fpga::CBNodeNameType from_type, int from_node, int src)
{
    if (!tile.cb_type) {
        return -2;
    }
    tile.cb_type->ensureDerivedMasks();
    int joint = -1;
    if (from_type == fpga::CB_NODE_DST) {
        return tile.cb_type->canJump(from_node, src, src, joint) ? joint : -2;
    }
    if (from_type == fpga::CB_NODE_LOCAL) {
        return tile.cb_type->canOut(from_node, src, src, joint) ? joint : -2;
    }
    if ((tile.cb_type->joint_src[from_node].jump & bit(src)) != NodeMask{}) {
        return -1;
    }
    NodeMask joints_to_src = tile.cb_type->src_reachable_joints[src].joint;
    joint = (tile.cb_type->joint_joint[from_node].joint & joints_to_src).firstSetBit();
    return joint >= 0 ? joint : -2;
}

std::string srcWireName(const fpga::Tile& tile, fpga::CBNodeNameType from_type, int from_node,
                        int src, int joint, const std::string& incoming_wire)
{
    if (!tile.cb_type) {
        return {};
    }
    if (joint >= 0) {
        if (const fpga::CBConnName* conn = tile.cb_type->connName(fpga::CB_NODE_JOINT, joint, fpga::CB_NODE_SRC, src)) {
            return conn->to;
        }
        if (const fpga::CBConnName* conn = tile.cb_type->connName(fpga::CB_NODE_SRC, src, fpga::CB_NODE_JOINT, joint)) {
            return conn->from;
        }
    }
    if (const fpga::CBConnName* conn = tile.cb_type->connName(from_type, from_node, fpga::CB_NODE_SRC, src)) {
        if (!incoming_wire.empty() && conn->from != incoming_wire) {
            return {};
        }
        return conn->to;
    }
    if (!incoming_wire.empty()) {
        return {};
    }
    if (const std::string* name = tile.cb_type->nodeName(fpga::CB_NODE_SRC, src)) {
        return *name;
    }
    return {};
}

std::string fromWireName(const fpga::Tile& tile, fpga::CBNodeNameType from_type, int from_node,
                         int src, int joint, const std::string& src_wire)
{
    if (!tile.cb_type) {
        return {};
    }
    if (joint >= 0) {
        if (const fpga::CBConnName* conn = tile.cb_type->connName(from_type, from_node, fpga::CB_NODE_JOINT, joint)) {
            return conn->from;
        }
    }
    if (const fpga::CBConnName* conn = tile.cb_type->connName(from_type, from_node, fpga::CB_NODE_SRC, src)) {
        if (!src_wire.empty() && conn->to != src_wire) {
            return {};
        }
        return conn->from;
    }
    if (const std::string* name = tile.cb_type->nodeName(from_type, from_node)) {
        return *name;
    }
    return {};
}

using Key = BackwardResolveKey;

struct Node
{
    fpga::Tile* tile = nullptr;
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

struct DockingOptions
{
    int max_depth = 5;
    int radius = 5;
    bool directional = false;
    int side_limit = 0;
    int candidate_limit = 0;
    bool trace_backward_attempts = false;
};

Key nodeKey(const Node& node)
{
    return Key{node.tile->coord.x, node.tile->coord.y, node.dst};
}

std::vector<fpga::Wire> pathToNode(const std::vector<Node>& nodes, int index)
{
    std::vector<fpga::Wire> result;
    for (int curr = index; curr >= 0 && nodes[curr].parent >= 0; curr = nodes[curr].parent) {
        result.push_back(nodes[curr].edge_from_parent);
    }
    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<fpga::Wire> suffixFromNode(const std::vector<Node>& nodes, int index)
{
    std::vector<fpga::Wire> result;
    for (int curr = index; curr >= 0 && nodes[curr].parent >= 0; curr = nodes[curr].parent) {
        result.push_back(nodes[curr].edge_from_parent);
    }
    if (index >= 0 && static_cast<size_t>(index) < nodes.size()) {
        result.insert(result.end(), nodes[index].target_suffix.begin(), nodes[index].target_suffix.end());
    }
    return result;
}

int manhattan(const fpga::Coord& a, const fpga::Coord& b)
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

int crossToLine(const fpga::Coord& start, const fpga::Coord& target, const fpga::Coord& point)
{
    fpga::Coord line = target - start;
    fpga::Coord offset = point - start;
    return std::abs(line.x * offset.y - line.y * offset.x);
}

bool inDirectionalCorridor(const fpga::Coord& coord, const fpga::Coord& start,
                           const fpga::Coord& target, const DockingOptions& options)
{
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

int directionalScore(const fpga::Coord& from, const fpga::Coord& to,
                     const fpga::Coord& start, const fpga::Coord& target)
{
    fpga::Coord goal = target - from;
    fpga::Coord step = to - from;
    int dot = goal.x * step.x + goal.y * step.y;
    int cross = std::abs(goal.x * step.y - goal.y * step.x);
    int behind = dot < 0 ? 100000 : 0;
    return behind + cross * 16 + manhattan(to, target);
}

DockingResult dockGroundingImpl(fpga::Tile& forward_tile, int forward_dst,
                                const std::string& forward_dst_wire,
                                fpga::Tile& target_tile, NodeMask pin_nodes,
                                const DockingOptions& options)
{
    DockingResult result;
    if (!forward_tile.cb_type || !target_tile.cb_type || forward_dst < 0 || pin_nodes == NodeMask{}) {
        return result;
    }

    std::vector<Node> forward_nodes;
    std::vector<Node> backward_nodes;
    std::deque<int> forward_queue;
    std::deque<int> backward_queue;
    std::map<Key, int> forward_seen;
    std::map<Key, int> backward_seen;
    std::set<Key> backward_deadends;
    BackwardResolveIndex backward_index;
    bool backward_sources_built = false;

    // Keep complete backward alternatives only for a focused diagnostic run.
    auto record_backward_attempt = [&](int seed_dst, const char* status,
                                       std::vector<fpga::Wire> fragments) {
        if (!options.trace_backward_attempts) {
            return;
        }
        result.backward_attempts.push_back(DockingBackwardAttempt{
            seed_dst, status, std::move(fragments)});
    };

    forward_nodes.push_back(Node{&forward_tile, forward_dst, forward_dst_wire, -1, {}, 0, {}});
    forward_seen[nodeKey(forward_nodes.front())] = 0;
    forward_queue.push_back(0);

    std::vector<Node> blocked_seeds;
    pin_nodes.for_each_set_bit([&](int pin) {
        if (target_tile.isPinNodeLeased(pin)) {
            return false;
        }
        target_tile.cb_type->ensureDerivedMasks();
        for (int dst = 0; dst < CB_MAX_NODES; ++dst) {
            std::vector<int> joints = entryJoints(*target_tile.cb_type, dst, pin);
            if (joints.empty()) {
                continue;
            }
            for (int joint : joints) {
                ++result.target_entry_count;
                fpga::CBState test_cb = target_tile.cb;
                fpga::Wire enter;
                enter.from = target_tile.coord;
                enter.to = target_tile.coord;
                enter.local = dst;
                enter.joint = joint;
                enter.pos = ROUTE_POS_TRANSIT;
                if (const std::string* name = target_tile.cb_type->nodeName(fpga::CB_NODE_DST, dst)) {
                    enter.dst_wire_name = *name;
                }
                fpga::Wire tile_pin;
                tile_pin.type = fpga::Wire::WIRE_TILE_PIN;
                tile_pin.from = target_tile.coord;
                tile_pin.to = target_tile.coord;
                tile_pin.local = pin;
                tile_pin.pos = ROUTE_POS_TRANSIT;
                std::vector<fpga::Wire> suffix{enter, tile_pin};
                bool terminal_busy = !leaseIn(test_cb, dst, pin, joint);
                Node seed{&target_tile, dst, enter.dst_wire_name, -1, {}, 0, suffix,
                          terminal_busy, pin, joint, -1};
                seed.seed_dst = dst;
                if (terminal_busy) {
                    ++result.target_busy_count;
                    record_backward_attempt(dst, "blocked_seed", suffix);
                    blocked_seeds.push_back(std::move(seed));
                    continue;
                }
                record_backward_attempt(dst, "seed", suffix);
                Key key = nodeKey(seed);
                if (backward_seen.find(key) == backward_seen.end()) {
                    int index = static_cast<int>(backward_nodes.size());
                    backward_nodes.push_back(seed);
                    backward_seen[key] = index;
                    backward_queue.push_back(index);
                    ++result.target_seed_count;
                }
                if (auto it = forward_seen.find(key); it != forward_seen.end()) {
                    result.fragments = pathToNode(forward_nodes, it->second);
                    result.fragments.insert(result.fragments.end(), suffix.begin(), suffix.end());
                    result.success = true;
                    record_backward_attempt(dst, "meet", suffix);
                    return result.success;
                }
            }
            if (result.success) {
                return true;
            }
        }
        return result.success;
    });
    if (result.success) {
        return result;
    }

    // Build the reverse jump index once per docking attempt. Backward nodes
    // then look up incoming transitions without rescanning the whole window.
    auto build_backward_sources = [&]() {
        if (backward_sources_built) {
            return;
        }
        backward_sources_built = true;
        fpga::Device& device = fpga::Device::current();
        backward_index = buildBackwardResolveIndex(device, target_tile.coord,
            options.radius, [&](const fpga::Coord& coord) {
                return inDirectionalCorridor(coord, forward_tile.coord,
                    target_tile.coord, options);
            });
        result.backward_mapping_scan_count = backward_index.mapping_scan_count;
        result.backward_mapping_reaches_count = backward_index.mapping_reaches_count;
    };

    auto expand_forward = [&](int node_index) -> bool {
        ++result.forward_pop_count;
        Node node = forward_nodes[node_index];
        if (node.depth >= options.max_depth || !inDockWindow(node.tile->coord, target_tile.coord, options.radius)) {
            return false;
        }
        const std::vector<uint16_t>* srcs = node.tile->cb_type->srcNodes(fpga::CB_NODE_DST, node.dst);
        if (!srcs) {
            return false;
        }

        struct ForwardCandidate
        {
            uint16_t src = 0;
            int joint = -2;
            std::string src_wire;
            fpga::TileJumpTarget target;
            int score = 0;
        };
        std::vector<ForwardCandidate> candidates;
        candidates.reserve(srcs->size());
        for (uint16_t src : *srcs) {
            int joint = selectJointToSrc(*node.tile, fpga::CB_NODE_DST, node.dst, src);
            if (joint == -2) {
                continue;
            }
            std::string src_wire = srcWireName(*node.tile, fpga::CB_NODE_DST, node.dst, src, joint, node.dst_wire);
            fpga::CBState test_cb = node.tile->cb;
            if (!leaseJump(test_cb, node.dst, src, node.parent < 0)) {
                continue;
            }
            fpga::TileJumpTarget target = fpga::Device::current().resolveJumpToward(*node.tile, src, target_tile.coord);
            if (!target.tile || !target.tile->cb_type || target.dst_node < 0
                || !inDockWindow(target.tile->coord, target_tile.coord, options.radius)
                || !inDirectionalCorridor(target.tile->coord, forward_tile.coord, target_tile.coord, options)) {
                continue;
            }
            candidates.push_back(ForwardCandidate{
                src,
                joint,
                src_wire,
                target,
                directionalScore(node.tile->coord, target.tile->coord, forward_tile.coord, target_tile.coord)
            });
        }
        if (options.candidate_limit > 0 && static_cast<int>(candidates.size()) > options.candidate_limit) {
            candidates.resize(options.candidate_limit);
        }

        for (const ForwardCandidate& candidate : candidates) {
            fpga::Wire edge;
            edge.from = node.tile->coord;
            edge.to = candidate.target.tile->coord;
            edge.local = node.dst;
            edge.jump = candidate.src;
            edge.route_jump = candidate.target.jump_node;
            edge.dst = candidate.target.dst_node;
            edge.joint = candidate.joint;
            edge.pos = ROUTE_POS_TRANSIT;
            edge.from_wire_name = node.dst_wire;
            edge.src_wire_name = candidate.src_wire;
            edge.dst_wire_name = candidate.target.dst_wire;
            Node next{candidate.target.tile, candidate.target.dst_node, candidate.target.dst_wire,
                      node_index, edge, node.depth + 1, {}};
            Key key = nodeKey(next);
            if (forward_seen.find(key) != forward_seen.end()) {
                continue;
            }
            int next_index = static_cast<int>(forward_nodes.size());
            forward_nodes.push_back(next);
            forward_seen[key] = next_index;
            forward_queue.push_back(next_index);
            ++result.forward_push_count;
            if (auto back = backward_seen.find(key); back != backward_seen.end()) {
                result.fragments = pathToNode(forward_nodes, next_index);
                std::vector<fpga::Wire> suffix = suffixFromNode(backward_nodes, back->second);
                result.fragments.insert(result.fragments.end(), suffix.begin(), suffix.end());
                result.success = true;
                return true;
            }
        }
        return false;
    };

    auto expand_backward = [&](int node_index) -> bool {
        ++result.backward_pop_count;
        Node node = backward_nodes[node_index];
        Key current_key = nodeKey(node);
        if (backward_deadends.contains(current_key)) {
            ++result.backward_deadend_reject_count;
            record_backward_attempt(node.seed_dst, "deadend_pop",
                suffixFromNode(backward_nodes, node_index));
            return false;
        }
        if (node.depth >= options.max_depth) {
            backward_deadends.insert(current_key);
            ++result.backward_deadend_count;
            record_backward_attempt(node.seed_dst, "depth_limit",
                suffixFromNode(backward_nodes, node_index));
            return false;
        }
        build_backward_sources();
        auto sources_it = backward_index.sources.find(current_key);
        if (sources_it == backward_index.sources.end()) {
            backward_deadends.insert(current_key);
            ++result.backward_deadend_count;
            record_backward_attempt(node.seed_dst, "no_incoming",
                suffixFromNode(backward_nodes, node_index));
            return false;
        }
        const std::vector<BackwardSource>& sources = sources_it->second;
        size_t source_count = sources.size();
        if (options.candidate_limit > 0) {
            source_count = std::min(source_count,
                static_cast<size_t>(options.candidate_limit));
        }

        for (size_t source_index = 0; source_index < source_count; ++source_index) {
            const BackwardSource& source = sources[source_index];
            int src = source.src;
            fpga::Tile* prev_tile = source.tile;
            if (!prev_tile || !prev_tile->cb_type) {
                continue;
            }
            prev_tile->cb_type->ensureDerivedMasks();
            NodeMask prev_dsts = prev_tile->cb_type->dsts_reaching_src[src].jump;
            if (prev_dsts == NodeMask{}) {
                ++result.backward_missing_prev_dst_count;
                record_backward_attempt(node.seed_dst, "no_previous_dst",
                    suffixFromNode(backward_nodes, node_index));
                continue;
            }
            prev_dsts.for_each_set_bit([&](int prev_dst) {
                int joint = selectJointToSrc(*prev_tile, fpga::CB_NODE_DST, prev_dst, src);
                if (joint == -2) {
                    ++result.backward_topology_reject_count;
                    record_backward_attempt(node.seed_dst, "topology_reject",
                        suffixFromNode(backward_nodes, node_index));
                    return false;
                }
                std::string src_wire = srcWireName(*prev_tile, fpga::CB_NODE_DST, prev_dst, src, joint, {});
                fpga::Wire edge;
                edge.from = prev_tile->coord;
                edge.to = node.tile->coord;
                edge.local = prev_dst;
                edge.jump = src;
                edge.route_jump = source.route_jump;
                edge.dst = node.dst;
                edge.joint = joint;
                edge.pos = ROUTE_POS_TRANSIT;
                edge.from_wire_name = fromWireName(*prev_tile, fpga::CB_NODE_DST, prev_dst, src, joint, src_wire);
                edge.src_wire_name = src_wire;
                edge.dst_wire_name = node.dst_wire;
                auto candidate_fragments = [&]() {
                    std::vector<fpga::Wire> fragments{edge};
                    std::vector<fpga::Wire> suffix = suffixFromNode(backward_nodes, node_index);
                    fragments.insert(fragments.end(), suffix.begin(), suffix.end());
                    return fragments;
                };
                fpga::CBState test_cb = prev_tile->cb;
                // The backward frontier may terminate at the already-routed
                // forward anchor; only its outgoing source must still be free.
                bool reaches_forward_anchor = prev_tile == &forward_tile && prev_dst == forward_dst;
                if (!leaseJump(test_cb, prev_dst, src, reaches_forward_anchor)) {
                    ++result.backward_busy_reject_count;
                    record_backward_attempt(node.seed_dst, "busy_reject", candidate_fragments());
                    return false;
                }
                Node next{prev_tile, prev_dst, {}, node_index, edge, node.depth + 1,
                          node.target_suffix, node.blocked_terminal,
                          node.terminal_pin, node.terminal_joint, node.terminal_joint2};
                next.seed_dst = node.seed_dst;
                if (const std::string* name = prev_tile->cb_type->nodeName(fpga::CB_NODE_DST, prev_dst)) {
                    next.dst_wire = *name;
                }
                Key key = nodeKey(next);
                if (backward_deadends.contains(key)) {
                    ++result.backward_deadend_reject_count;
                    record_backward_attempt(node.seed_dst, "deadend_reject", candidate_fragments());
                    return false;
                }
                if (backward_seen.find(key) != backward_seen.end()) {
                    ++result.backward_seen_reject_count;
                    record_backward_attempt(node.seed_dst, "seen_reject", candidate_fragments());
                    return false;
                }
                int next_index = static_cast<int>(backward_nodes.size());
                backward_nodes.push_back(next);
                backward_seen[key] = next_index;
                backward_queue.push_back(next_index);
                ++result.backward_push_count;
                record_backward_attempt(node.seed_dst, "push", candidate_fragments());
                if (auto front = forward_seen.find(key); front != forward_seen.end()) {
                    if (next.blocked_terminal) {
                        result.blocked_terminal_reachable = true;
                        result.blocked_dst = node.target_suffix.empty()
                            ? node.dst : node.target_suffix.front().local;
                        result.blocked_pin = next.terminal_pin;
                        result.blocked_joint = next.terminal_joint;
                        result.blocked_joint2 = next.terminal_joint2;
                        record_backward_attempt(node.seed_dst, "blocked_meet", candidate_fragments());
                        return true;
                    }
                    result.fragments = pathToNode(forward_nodes, front->second);
                    std::vector<fpga::Wire> suffix = suffixFromNode(backward_nodes, next_index);
                    result.fragments.insert(result.fragments.end(), suffix.begin(), suffix.end());
                    result.success = true;
                    record_backward_attempt(node.seed_dst, "meet", candidate_fragments());
                    return true;
                }
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

    while ((!forward_queue.empty() || !backward_queue.empty()) && !result.success) {
        if (!backward_queue.empty()) {
            int index = backward_queue.front();
            backward_queue.pop_front();
            if (expand_backward(index)) {
                break;
            }
        }
        if (!forward_queue.empty()) {
            int index = forward_queue.front();
            forward_queue.pop_front();
            if (expand_forward(index)) {
                break;
            }
        }
    }

    // Reuse the exhausted forward search to identify a physically reachable
    // busy terminal. The caller may then preempt exactly that numeric entry.
    if (!result.success && !blocked_seeds.empty()) {
        backward_nodes.clear();
        backward_queue.clear();
        backward_seen.clear();
        backward_deadends.clear();
        for (Node& seed : blocked_seeds) {
            Key key = nodeKey(seed);
            if (backward_seen.find(key) != backward_seen.end()) {
                continue;
            }
            int index = static_cast<int>(backward_nodes.size());
            backward_nodes.push_back(std::move(seed));
            backward_seen[key] = index;
            if (auto front = forward_seen.find(key); front != forward_seen.end()) {
                const Node& reached = backward_nodes[index];
                result.blocked_terminal_reachable = true;
                result.blocked_dst = reached.dst;
                result.blocked_pin = reached.terminal_pin;
                result.blocked_joint = reached.terminal_joint;
                result.blocked_joint2 = reached.terminal_joint2;
                break;
            }
            backward_queue.push_back(index);
        }
        while (!result.blocked_terminal_reachable && !backward_queue.empty()) {
            int index = backward_queue.front();
            backward_queue.pop_front();
            if (expand_backward(index)) {
                break;
            }
        }
    }

    return result;
}

} // namespace

BackwardResolveIndex buildBackwardResolveIndex(
    fpga::Device& device, fpga::Coord center, int radius,
    const std::function<bool(const fpga::Coord&)>& include_source)
{
    BackwardResolveIndex result;
    for (int y = center.y - radius; y <= center.y + radius; ++y) {
        for (int x = center.x - radius; x <= center.x + radius; ++x) {
            fpga::Coord source_coord{x, y};
            if (!inDockWindow(source_coord, center, radius)
                || (include_source && !include_source(source_coord))) {
                continue;
            }
            fpga::Tile* source_tile = device.getTile(x, y);
            if (!source_tile || !source_tile->cb_type) {
                continue;
            }
            for (const auto& [src_key, entries] : source_tile->cb_type->dst_by_src.values) {
                ++result.mapping_scan_count;
                if (entries.empty()) {
                    continue;
                }
                int src = src_key;
                std::set<BackwardResolveKey> source_targets;
                for (const fpga::TileJumpTarget& target :
                     device.resolveJumpTargets(*source_tile, src)) {
                    if (!target.tile || !target.tile->cb_type || target.dst_node < 0
                        || !inDockWindow(target.tile->coord, center, radius)) {
                        continue;
                    }
                    BackwardResolveKey key{
                        target.tile->coord.x, target.tile->coord.y, target.dst_node};
                    if (!source_targets.insert(key).second) {
                        continue;
                    }
                    result.sources[key].push_back(
                        BackwardResolveSource{source_tile, src, target.jump_node});
                    ++result.mapping_reaches_count;
                }
            }
        }
    }
    return result;
}

DockingResult dockGrounding(fpga::Tile& forward_tile, int forward_dst,
                            const std::string& forward_dst_wire,
                            fpga::Tile& target_tile, NodeMask pin_nodes,
                            int max_depth, int radius, bool trace_backward_attempts)
{
    return dockGroundingImpl(forward_tile, forward_dst, forward_dst_wire, target_tile,
                             pin_nodes, DockingOptions{
                                 max_depth, radius, false, 0, 0, trace_backward_attempts});
}

DockingResult dockIOB(fpga::Tile& forward_tile, int forward_dst,
                      const std::string& forward_dst_wire,
                      fpga::Tile& target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts)
{
    return dockGroundingImpl(forward_tile, forward_dst, forward_dst_wire, target_tile,
                             pin_nodes, DockingOptions{
                                 12, 12, true, 3, 16, trace_backward_attempts});
}

} // namespace pnr
