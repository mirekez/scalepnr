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

int decodeSigned4(int value)
{
    value &= 0xf;
    return (value & 0x8) ? value - 16 : value;
}

fpga::Coord jumpDelta(int jump)
{
    return fpga::Coord{decodeSigned4((jump >> 6) & 0xf), decodeSigned4((jump >> 2) & 0xf)};
}

bool leaseJump(fpga::CBState& cb, int dst, int src, bool allow_existing_dst = false)
{
    NodeMask dst_bit = bit(dst);
    NodeMask src_bit = bit(src);
    if ((cb.src_deadend.jump & src_bit) != NodeMask{}
        || (cb.src.jump & src_bit) != NodeMask{}) {
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

} // namespace

DockingResult dockGrounding(fpga::Tile& forward_tile, int forward_dst,
                            const std::string& forward_dst_wire,
                            fpga::Tile& target_tile, NodeMask pin_nodes,
                            int max_depth, int radius)
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
        NodeMask dst_candidates = target_tile.cb_type->dsts_reaching_local[pin].jump;
        dst_candidates.for_each_set_bit([&](int dst) {
            for (int joint : entryJoints(*target_tile.cb_type, dst, pin)) {
                fpga::CBState test_cb = target_tile.cb;
                if (!leaseIn(test_cb, dst, pin, joint)) {
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
                }
                if (auto it = forward_seen.find(key); it != forward_seen.end()) {
                    result.fragments = pathToNode(forward_nodes, it->second);
                    result.fragments.insert(result.fragments.end(), suffix.begin(), suffix.end());
                    result.success = true;
                    return true;
                }
            }
            return false;
        });
        return result.success;
    });
    if (result.success) {
        return result;
    }

    auto expand_forward = [&](int node_index) -> bool {
        Node node = forward_nodes[node_index];
        if (node.depth >= max_depth || !inDockWindow(node.tile->coord, target_tile.coord, radius)) {
            return false;
        }
        const std::vector<uint16_t>* srcs = node.tile->cb_type->srcNodes(fpga::CB_NODE_DST, node.dst);
        if (!srcs) {
            return false;
        }
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
            fpga::TileJumpTarget target = fpga::Device::current().resolveJump(*node.tile, src, &target_tile.coord);
            if (!target.tile || !target.tile->cb_type || target.dst_node < 0
                || !inDockWindow(target.tile->coord, target_tile.coord, radius)) {
                continue;
            }
            fpga::Wire edge;
            edge.from = node.tile->coord;
            edge.to = target.tile->coord;
            edge.local = node.dst;
            edge.jump = src;
            edge.joint = joint;
            edge.pos = ROUTE_POS_TRANSIT;
            edge.src_wire_name = src_wire;
            edge.dst_wire_name = target.dst_wire;
            Node next{target.tile, target.dst_node, target.dst_wire, node_index, edge, node.depth + 1, {}};
            Key key = nodeKey(next);
            if (forward_seen.find(key) != forward_seen.end()) {
                continue;
            }
            int next_index = static_cast<int>(forward_nodes.size());
            forward_nodes.push_back(next);
            forward_seen[key] = next_index;
            forward_queue.push_back(next_index);
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
        Node node = backward_nodes[node_index];
        if (node.depth >= max_depth) {
            return false;
        }
        fpga::Device& device = fpga::Device::current();
        for (int src = 0; src < CB_MAX_NODES; ++src) {
            fpga::Coord prev_coord = node.tile->coord - jumpDelta(src);
            if (!inDockWindow(prev_coord, target_tile.coord, radius)) {
                continue;
            }
            fpga::Tile* prev_tile = device.getTile(prev_coord.x, prev_coord.y);
            if (!prev_tile || !prev_tile->cb_type) {
                continue;
            }
            fpga::TileJumpTarget target = device.resolveJump(*prev_tile, src);
            if (target.tile != node.tile || target.dst_node != node.dst) {
                continue;
            }
            prev_tile->cb_type->ensureDerivedMasks();
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
                edge.joint = joint;
                edge.pos = ROUTE_POS_TRANSIT;
                edge.src_wire_name = src_wire;
                edge.dst_wire_name = node.dst_wire;
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

} // namespace pnr
