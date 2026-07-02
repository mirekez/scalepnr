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

struct Key
{
    int x = 0;
    int y = 0;
    int dst = -1;

    bool operator<(const Key& other) const
    {
        if (x != other.x) {
            return x < other.x;
        }
        if (y != other.y) {
            return y < other.y;
        }
        return dst < other.dst;
    }
};

struct Node
{
    fpga::Tile* tile = nullptr;
    int dst = -1;
    std::string dst_wire;
    int parent = -1;
    fpga::Wire edge_from_parent;
    int depth = 0;
    std::vector<fpga::Wire> target_suffix;
};

struct DockingOptions
{
    int max_depth = 5;
    int radius = 5;
    bool directional = false;
    int side_limit = 0;
    int candidate_limit = 0;
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

    forward_nodes.push_back(Node{&forward_tile, forward_dst, forward_dst_wire, -1, {}, 0, {}});
    forward_seen[nodeKey(forward_nodes.front())] = 0;
    forward_queue.push_back(0);

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
                if (!leaseIn(test_cb, dst, pin, joint)) {
                    ++result.target_busy_count;
                    continue;
                }
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
                Node seed{&target_tile, dst, enter.dst_wire_name, -1, {}, 0, suffix};
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
        if (node.depth >= options.max_depth) {
            return false;
        }
        fpga::Device& device = fpga::Device::current();

        struct BackwardSource
        {
            int src = 0;
            fpga::Coord prev_coord;
            int score = 0;
        };
        std::vector<BackwardSource> sources;
        sources.reserve(CB_MAX_NODES);
        for (int y = node.tile->coord.y - options.radius; y <= node.tile->coord.y + options.radius; ++y) {
            for (int x = node.tile->coord.x - options.radius; x <= node.tile->coord.x + options.radius; ++x) {
                fpga::Coord prev_coord{x, y};
                if (!inDockWindow(prev_coord, target_tile.coord, options.radius)
                    || !inDirectionalCorridor(prev_coord, forward_tile.coord, target_tile.coord, options)) {
                    continue;
                }
                fpga::Tile* prev_tile = device.getTile(prev_coord.x, prev_coord.y);
                if (!prev_tile || !prev_tile->cb_type) {
                    continue;
                }
                for (int src = 0; src < CB_MAX_NODES; ++src) {
                    if (prev_tile->cb_type->dst_by_src[src].empty()) {
                        continue;
                    }
                    std::vector<fpga::TileJumpTarget> targets = device.resolveJumpTargets(*prev_tile, src);
                    bool reaches_node = std::any_of(targets.begin(), targets.end(), [&](const fpga::TileJumpTarget& target) {
                        return target.tile == node.tile && target.dst_node == node.dst;
                    });
                    if (!reaches_node) {
                        continue;
                    }
                    sources.push_back(BackwardSource{
                        src,
                        prev_coord,
                        directionalScore(node.tile->coord, prev_coord, target_tile.coord, forward_tile.coord)
                    });
                }
            }
        }
        if (options.candidate_limit > 0 && static_cast<int>(sources.size()) > options.candidate_limit) {
            sources.resize(options.candidate_limit);
        }

        for (const BackwardSource& source : sources) {
            int src = source.src;
            fpga::Coord prev_coord = source.prev_coord;
            fpga::Tile* prev_tile = device.getTile(prev_coord.x, prev_coord.y);
            if (!prev_tile || !prev_tile->cb_type) {
                continue;
            }
            prev_tile->cb_type->ensureDerivedMasks();
            std::vector<fpga::TileJumpTarget> targets = device.resolveJumpTargets(*prev_tile, src);
            auto target_it = std::find_if(targets.begin(), targets.end(), [&](const fpga::TileJumpTarget& target) {
                return target.tile == node.tile && target.dst_node == node.dst;
            });
            if (target_it == targets.end()) {
                continue;
            }
            NodeMask prev_dsts = prev_tile->cb_type->dsts_reaching_src[src].jump;
            prev_dsts.for_each_set_bit([&](int prev_dst) {
                int joint = selectJointToSrc(*prev_tile, fpga::CB_NODE_DST, prev_dst, src);
                if (joint == -2) {
                    return false;
                }
                std::string src_wire = srcWireName(*prev_tile, fpga::CB_NODE_DST, prev_dst, src, joint, {});
                fpga::CBState test_cb = prev_tile->cb;
                if (!leaseJump(test_cb, prev_dst, src)) {
                    return false;
                }
                fpga::Wire edge;
                edge.from = prev_tile->coord;
                edge.to = node.tile->coord;
                edge.local = prev_dst;
                edge.jump = src;
                edge.route_jump = target_it->jump_node;
                edge.dst = node.dst;
                edge.joint = joint;
                edge.pos = ROUTE_POS_TRANSIT;
                edge.src_wire_name = src_wire;
                edge.dst_wire_name = target_it->dst_wire;
                Node next{prev_tile, prev_dst, {}, node_index, edge, node.depth + 1, node.target_suffix};
                if (const std::string* name = prev_tile->cb_type->nodeName(fpga::CB_NODE_DST, prev_dst)) {
                    next.dst_wire = *name;
                }
                Key key = nodeKey(next);
                if (backward_seen.find(key) != backward_seen.end()) {
                    return false;
                }
                int next_index = static_cast<int>(backward_nodes.size());
                backward_nodes.push_back(next);
                backward_seen[key] = next_index;
                backward_queue.push_back(next_index);
                ++result.backward_push_count;
                if (auto front = forward_seen.find(key); front != forward_seen.end()) {
                    result.fragments = pathToNode(forward_nodes, front->second);
                    std::vector<fpga::Wire> suffix = suffixFromNode(backward_nodes, next_index);
                    result.fragments.insert(result.fragments.end(), suffix.begin(), suffix.end());
                    result.success = true;
                    return true;
                }
                return false;
            });
            if (result.success) {
                return true;
            }
        }
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

    return result;
}

} // namespace

DockingResult dockGrounding(fpga::Tile& forward_tile, int forward_dst,
                            const std::string& forward_dst_wire,
                            fpga::Tile& target_tile, NodeMask pin_nodes,
                            int max_depth, int radius)
{
    return dockGroundingImpl(forward_tile, forward_dst, forward_dst_wire, target_tile,
                             pin_nodes, DockingOptions{max_depth, radius, false, 0, 0});
}

DockingResult dockIOB(fpga::Tile& forward_tile, int forward_dst,
                      const std::string& forward_dst_wire,
                      fpga::Tile& target_tile, NodeMask pin_nodes)
{
    return dockGroundingImpl(forward_tile, forward_dst, forward_dst_wire, target_tile,
                             pin_nodes, DockingOptions{12, 12, true, 3, 16});
}

} // namespace pnr
