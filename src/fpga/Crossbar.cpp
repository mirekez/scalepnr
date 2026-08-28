#include "Crossbar.h"

#include <array>
#include <cstdlib>
#include <limits>

using namespace fpga;

namespace {

constexpr int encodeSigned4(int value)
{
    return value & 0xf;
}

constexpr int decodeSigned4(int value)
{
    value &= 0xf;
    return (value & 0x8) ? value - 16 : value;
}

Coord directionDelta(int dir, int length)
{
    switch (dir & 7) {
        case 0: return Coord{0, -length};
        case 1: return Coord{length, -length};
        case 2: return Coord{length, 0};
        case 3: return Coord{length, length};
        case 4: return Coord{0, length};
        case 5: return Coord{-length, length};
        case 6: return Coord{-length, 0};
        case 7: return Coord{-length, -length};
    }
    return {};
}

int jumpIndexForDelta(int dx, int dy, int num)
{
    return (encodeSigned4(dx) << 8) | (encodeSigned4(dy) << 4) | (num & 0xf);
}

uint16_t jumpDeltaKey(int dx, int dy)
{
    return static_cast<uint16_t>((encodeSigned4(dx) << 4) | encodeSigned4(dy));
}

std::string jumpLaneKey(std::string name)
{
    if (size_t pos = name.find("SRC"); pos != std::string::npos) {
        name.replace(pos, 3, "JUMP");
    }
    if (size_t pos = name.find("DST"); pos != std::string::npos) {
        name.replace(pos, 3, "JUMP");
    }
    return name;
}

constexpr int jumpDeltaX(int jump)
{
    return decodeSigned4((jump >> 8) & 0xf);
}

constexpr int jumpDeltaY(int jump)
{
    return decodeSigned4((jump >> 4) & 0xf);
}

constexpr int scaledDirectionAxis(int value, int max_abs)
{
    if (value == 0 || max_abs == 0) {
        return 0;
    }
    int scaled = (std::abs(value) * 7 + max_abs / 2) / max_abs;
    if (scaled == 0) {
        scaled = 1;
    }
    return value < 0 ? -scaled : scaled;
}

void jumpTargetBucket(Coord diff, int& target_dx, int& target_dy)
{
    int abs_x = std::abs(diff.x);
    int abs_y = std::abs(diff.y);
    int max_abs = abs_x > abs_y ? abs_x : abs_y;
    target_dx = scaledDirectionAxis(diff.x, max_abs);
    target_dy = scaledDirectionAxis(diff.y, max_abs);
}

struct JumpBucket
{
    int dx = 0;
    int dy = 0;
};

int angleHalf(int dot)
{
    if (dot > 0) {
        return 0;
    }
    if (dot == 0) {
        return 1;
    }
    return 2;
}

bool jumpBucketBefore(const JumpBucket& lhs, const JumpBucket& rhs, int target_dx, int target_dy)
{
    int lhs_dot = lhs.dx * target_dx + lhs.dy * target_dy;
    int rhs_dot = rhs.dx * target_dx + rhs.dy * target_dy;
    int lhs_cross = std::abs(lhs.dx * target_dy - lhs.dy * target_dx);
    int rhs_cross = std::abs(rhs.dx * target_dy - rhs.dy * target_dx);
    int lhs_half = angleHalf(lhs_dot);
    int rhs_half = angleHalf(rhs_dot);
    if (lhs_half != rhs_half) {
        return lhs_half < rhs_half;
    }

    // Compare atan2(|cross|, dot) exactly. For angles beyond 90 degrees,
    // a larger |cross| / -dot ratio is closer to the target direction.
    if (lhs_half == 0) {
        int lhs_ratio = lhs_cross * rhs_dot;
        int rhs_ratio = rhs_cross * lhs_dot;
        if (lhs_ratio != rhs_ratio) {
            return lhs_ratio < rhs_ratio;
        }
    }
    else if (lhs_half == 2) {
        int lhs_ratio = lhs_cross * -rhs_dot;
        int rhs_ratio = rhs_cross * -lhs_dot;
        if (lhs_ratio != rhs_ratio) {
            return lhs_ratio > rhs_ratio;
        }
    }

    int lhs_length = std::abs(lhs.dx) + std::abs(lhs.dy);
    int rhs_length = std::abs(rhs.dx) + std::abs(rhs.dy);
    if (lhs_length != rhs_length) {
        return lhs_length < rhs_length;
    }
    if (lhs.dx != rhs.dx) {
        return lhs.dx < rhs.dx;
    }
    return lhs.dy < rhs.dy;
}

std::array<JumpBucket, 224> makeJumpBucketOrder()
{
    std::array<JumpBucket, 224> order{};
    size_t index = 0;
    for (int length = 1; length <= 7; ++length) {
        for (int dx = -length; dx <= length; ++dx) {
            for (int dy = -length; dy <= length; ++dy) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                if (std::max(std::abs(dx), std::abs(dy)) != length) {
                    continue;
                }
                order[index++] = JumpBucket{dx, dy};
            }
        }
    }
    return order;
}

const std::array<JumpBucket, 224>& jumpBucketOrder()
{
    static const std::array<JumpBucket, 224> order = makeJumpBucketOrder();
    return order;
}

std::array<JumpBucket, 224> makePriorityBucketOrder(int target_dx, int target_dy)
{
    const std::array<JumpBucket, 224>& base = jumpBucketOrder();
    std::array<JumpBucket, 224> ordered{};
    std::array<bool, 224> used{};
    for (size_t out = 0; out < ordered.size(); ++out) {
        size_t best = base.size();
        for (size_t index = 0; index < base.size(); ++index) {
            if (!used[index]
                && (best == base.size()
                    || jumpBucketBefore(base[index], base[best], target_dx, target_dy))) {
                best = index;
            }
        }
        used[best] = true;
        ordered[out] = base[best];
    }
    return ordered;
}

const std::array<JumpBucket, 224>& priorityBucketOrder(int target_dx, int target_dy)
{
    static const std::array<std::array<JumpBucket, 224>, 225> orders = [] {
        std::array<std::array<JumpBucket, 224>, 225> result{};
        for (int dx = -7; dx <= 7; ++dx) {
            for (int dy = -7; dy <= 7; ++dy) {
                result[static_cast<size_t>((dx + 7) * 15 + (dy + 7))] =
                    makePriorityBucketOrder(dx, dy);
            }
        }
        return result;
    }();
    return orders[static_cast<size_t>((target_dx + 7) * 15 + (target_dy + 7))];
}

int mappedJumpLength(int first_id, const std::vector<std::string>& lengths)
{
    if (first_id >= 0 && first_id < static_cast<int>(lengths.size())) {
        return atoi(lengths[first_id].c_str());
    }
    if (first_id == 4 && lengths.size() > 2) {
        return atoi(lengths[2].c_str());
    }
    if (first_id == 6 && lengths.size() > 3) {
        return atoi(lengths[3].c_str());
    }
    return first_id;
}

std::string stripTypePrefix(std::string name, const std::string& type_name)
{
    std::string prefix = type_name + "_";
    if (!type_name.empty() && name.rfind(prefix, 0) == 0) {
        name.erase(0, prefix.size());
    }
    return name;
}

bool debugCBPairMatches(const std::string& a, const std::string& b)
{
    const char* value = std::getenv("SCALEPNR_DEBUG_CB_PAIR");
    if (!value || !*value) {
        return false;
    }
    std::string text = a + "->" + b;
    return text.find(value) != std::string::npos || a.find(value) != std::string::npos || b.find(value) != std::string::npos;
}

int debugEnvInt(const char* name, int fallback = -1)
{
    const char* value = std::getenv(name);
    if (!value || !*value) {
        return fallback;
    }
    return atoi(value);
}

std::string nodeDisplayName(std::string name)
{
    size_t pos = name.find('.');
    if (pos != std::string::npos) {
        name.erase(0, pos + 1);
    }
    return name;
}

int techMapAnnotationScore(const std::string& name, const TechMap& map)
{
    if (map.size() < 3) {
        return 3;
    }
    int best = 3;
    for (const auto& expr : map[2]) {
        if (expr.size() < 2 || expr[0].empty() || expr[0][0].empty()
            || expr[1].empty() || expr[1][0].empty()) {
            continue;
        }
        const std::string& token = expr[0][0][0];
        if (token.empty() || name.find(token) == std::string::npos) {
            continue;
        }
        best = std::min(best, atoi(expr[1][0][0].c_str()));
    }
    return best;
}

void rememberNodeName(CBType& type, CBNodeNameType node_type, int value, const std::string& name, const TechMap& map)
{
    if (value < 0 || value >= CB_MAX_NODES) {
        return;
    }
    std::string display_name = nodeDisplayName(name);
    if (display_name.empty()) {
        return;
    }
    CBNodeNameKey key{static_cast<uint16_t>(node_type), static_cast<uint16_t>(value)};
    auto [it, inserted] = type.node_names.try_emplace(key, display_name);
    if (!inserted && techMapAnnotationScore(display_name, map) < techMapAnnotationScore(it->second, map)) {
        it->second = display_name;
    }

    auto remember_reverse = [&](std::unordered_map<std::string, uint16_t>& reverse_map) {
        reverse_map.try_emplace(name, static_cast<uint16_t>(value));
        reverse_map.try_emplace(display_name, static_cast<uint16_t>(value));
    };
    if (node_type == CB_NODE_LOCAL) {
        remember_reverse(type.local_nodes_by_name);
    }
    if (node_type == CB_NODE_SRC) {
        remember_reverse(type.src_nodes_by_name);
    }
    if (node_type == CB_NODE_DST) {
        remember_reverse(type.dst_nodes_by_name);
    }
    if (node_type == CB_NODE_JOINT) {
        remember_reverse(type.joint_nodes_by_name);
    }
}

void rememberParsedNode(CBType& type, int parsed_type,
                        const CBLocalNode& local_node, const CBJumpNode& src_node,
                        const CBJumpNode& dst_node, const CBJointNode& joint_node,
                        const std::string& name, const TechMap& map)
{
    if (parsed_type == 0) {
        rememberNodeName(type, CB_NODE_LOCAL, local_node.local, name, map);
    }
    if (parsed_type == 1) {
        rememberNodeName(type, CB_NODE_SRC, src_node.jump, name, map);
        rememberNodeName(type, CB_NODE_JUMP, src_node.jump, name, map);
    }
    if (parsed_type == 2) {
        rememberNodeName(type, CB_NODE_DST, dst_node.jump, name, map);
    }
    if (parsed_type == 3) {
        rememberNodeName(type, CB_NODE_JOINT, joint_node.joint, name, map);
    }
}

bool parsedNodeKey(int parsed_type,
                   const CBLocalNode& local_node, const CBJumpNode& src_node,
                   const CBJumpNode& dst_node, const CBJointNode& joint_node,
                   CBNodeNameType& type, int& value)
{
    if (parsed_type == 0) {
        type = CB_NODE_LOCAL;
        value = local_node.local;
        return true;
    }
    if (parsed_type == 1) {
        type = CB_NODE_SRC;
        value = src_node.jump;
        return true;
    }
    if (parsed_type == 2) {
        type = CB_NODE_DST;
        value = dst_node.jump;
        return true;
    }
    if (parsed_type == 3) {
        type = CB_NODE_JOINT;
        value = joint_node.joint;
        return true;
    }
    return false;
}

void rememberConnName(CBType& type, CBNodeNameType from_type, int from_value,
                      CBNodeNameType to_type, int to_value,
                      const std::string& from_name, const std::string& to_name,
                      const TechMap& map)
{
    if (from_value < 0 || from_value >= CB_MAX_NODES || to_value < 0 || to_value >= CB_MAX_NODES) {
        return;
    }
    CBConnName conn{nodeDisplayName(from_name), nodeDisplayName(to_name)};
    if (conn.from.empty() || conn.to.empty()) {
        return;
    }

    CBConnNameKey key{
        static_cast<uint16_t>(from_type),
        static_cast<uint16_t>(from_value),
        static_cast<uint16_t>(to_type),
        static_cast<uint16_t>(to_value)
    };
    auto& conns = type.conn_names[key];
    auto same = [&](const CBConnName& old) {
        return old.from == conn.from && old.to == conn.to;
    };
    if (std::find_if(conns.begin(), conns.end(), same) == conns.end()) {
        int new_score = techMapAnnotationScore(conn.from, map) + techMapAnnotationScore(conn.to, map);
        auto insert_pos = std::find_if(conns.begin(), conns.end(), [&](const CBConnName& old) {
            int old_score = techMapAnnotationScore(old.from, map) + techMapAnnotationScore(old.to, map);
            return new_score < old_score;
        });
        conns.insert(insert_pos, conn);
    }

    if (to_type == CB_NODE_SRC) {
        CBNodeNameKey from_key{static_cast<uint16_t>(from_type), static_cast<uint16_t>(from_value)};
        auto& srcs = type.outgoing_srcs[from_key];
        uint16_t src = static_cast<uint16_t>(to_value);
        if (std::find(srcs.begin(), srcs.end(), src) == srcs.end()) {
            srcs.push_back(src);
        }
    }
}

}

namespace {

void rememberOutgoingSrc(CBType& type, CBNodeNameType from_type, int from_value,
                         CBNodeNameType to_type, int to_value)
{
    if (to_type == CB_NODE_SRC) {
        CBNodeNameKey from_key{static_cast<uint16_t>(from_type), static_cast<uint16_t>(from_value)};
        auto& srcs = type.outgoing_srcs[from_key];
        uint16_t src = static_cast<uint16_t>(to_value);
        if (std::find(srcs.begin(), srcs.end(), src) == srcs.end()) {
            srcs.push_back(src);
        }
    }
}

void addResolvedJump(std::vector<CBType::ResolvedJump>& entries, Coord delta,
                     uint16_t target_cb_type_id, const CBJumpState& dsts,
                     bool target_tile_coord = false)
{
    if (target_cb_type_id == CB_INVALID_TYPE_ID || dsts.jump == NodeMask{}) {
        return;
    }
    for (CBType::ResolvedJump& entry : entries) {
        if (entry.target_cb_type_id == target_cb_type_id
            && entry.delta.x == delta.x && entry.delta.y == delta.y
            && entry.target_tile_coord == target_tile_coord) {
            entry.dsts.jump |= dsts.jump;
            return;
        }
    }
    entries.push_back(CBType::ResolvedJump{delta, target_cb_type_id, dsts, {}, target_tile_coord});
}

}

void CBType::rememberNodeName(CBNodeNameType type, int value, const std::string& name)
{
    if (value < 0 || value >= CB_MAX_NODES) {
        return;
    }
    std::string display_name = nodeDisplayName(name);
    if (display_name.empty()) {
        return;
    }
    CBNodeNameKey key{static_cast<uint16_t>(type), static_cast<uint16_t>(value)};
    auto [it, inserted] = node_names.try_emplace(key, display_name);
    if (!inserted && techMapAnnotationScore(display_name, annotation_map) < techMapAnnotationScore(it->second, annotation_map)) {
        it->second = display_name;
    }

    auto remember_reverse = [&](std::unordered_map<std::string, uint16_t>& map) {
        map.try_emplace(name, static_cast<uint16_t>(value));
        map.try_emplace(display_name, static_cast<uint16_t>(value));
    };
    if (type == CB_NODE_LOCAL) {
        remember_reverse(local_nodes_by_name);
    }
    if (type == CB_NODE_SRC) {
        remember_reverse(src_nodes_by_name);
    }
    if (type == CB_NODE_DST) {
        remember_reverse(dst_nodes_by_name);
    }
    if (type == CB_NODE_JOINT) {
        remember_reverse(joint_nodes_by_name);
    }
}

const std::string* CBType::nodeName(CBNodeNameType type, int value) const
{
    if (value < 0 || value >= CB_MAX_NODES) {
        return nullptr;
    }
    CBNodeNameKey key{static_cast<uint16_t>(type), static_cast<uint16_t>(value)};
    auto it = node_names.find(key);
    if (it != node_names.end()) {
        return &it->second;
    }
    return nullptr;
}

int CBType::nodeNum(CBNodeNameType type, const std::string& name) const
{
    const std::unordered_map<std::string, uint16_t>* map = nullptr;
    if (type == CB_NODE_LOCAL) {
        map = &local_nodes_by_name;
    }
    else if (type == CB_NODE_SRC) {
        map = &src_nodes_by_name;
    }
    else if (type == CB_NODE_DST) {
        map = &dst_nodes_by_name;
    }
    else if (type == CB_NODE_JOINT) {
        map = &joint_nodes_by_name;
    }
    if (!map) {
        return -1;
    }

    auto it = map->find(name);
    if (it != map->end()) {
        return it->second;
    }

    std::string display_name = nodeDisplayName(name);
    it = map->find(display_name);
    return it == map->end() ? -1 : it->second;
}

int CBType::ensureLocalNode(const std::string& wire_name)
{
    int existing = nodeNum(CB_NODE_LOCAL, wire_name);
    if (existing >= 0) {
        return existing;
    }
    for (int value = 0; value < CB_MAX_NODES; ++value) {
        CBNodeNameKey key{static_cast<uint16_t>(CB_NODE_LOCAL), static_cast<uint16_t>(value)};
        if (!node_names.contains(key)) {
            rememberNodeName(CB_NODE_LOCAL, value, wire_name);
            return value;
        }
    }
    PNR_ASSERT(false, "crossbar '{}' has no free local node for tile connection wire '{}'", name, wire_name);
    return -1;
}

int CBType::ensureExactLocalNode(const std::string& wire_name)
{
    // Dedicated wires with a shared numeric stem still need distinct runtime identities.
    std::string display_name = nodeDisplayName(wire_name);
    int existing = nodeNum(CB_NODE_LOCAL, wire_name);
    if (existing >= 0) {
        const std::string* existing_name = nodeName(CB_NODE_LOCAL, existing);
        if (existing_name && *existing_name == display_name) {
            return existing;
        }
    }
    for (int value = 0; value < CB_MAX_NODES; ++value) {
        CBNodeNameKey key{static_cast<uint16_t>(CB_NODE_LOCAL), static_cast<uint16_t>(value)};
        if (node_names.contains(key)) {
            continue;
        }
        node_names.emplace(key, display_name);
        local_nodes_by_name.insert_or_assign(wire_name, static_cast<uint16_t>(value));
        local_nodes_by_name.insert_or_assign(display_name, static_cast<uint16_t>(value));
        return value;
    }
    PNR_ASSERT(false, "crossbar '{}' has no free exact local node for tile wire '{}'", name, wire_name);
    return -1;
}

void CBType::rememberConnName(CBNodeNameType from_type, int from_value,
                              CBNodeNameType to_type, int to_value,
                              const std::string& from_name, const std::string& to_name)
{
    if (from_value < 0 || from_value >= CB_MAX_NODES || to_value < 0 || to_value >= CB_MAX_NODES) {
        return;
    }
    CBConnName conn{nodeDisplayName(from_name), nodeDisplayName(to_name)};
    if (conn.from.empty() || conn.to.empty()) {
        return;
    }

    CBConnNameKey key{
        static_cast<uint16_t>(from_type),
        static_cast<uint16_t>(from_value),
        static_cast<uint16_t>(to_type),
        static_cast<uint16_t>(to_value)
    };
    auto& conns = conn_names[key];
    auto same = [&](const CBConnName& old) {
        return old.from == conn.from && old.to == conn.to;
    };
    if (std::find_if(conns.begin(), conns.end(), same) == conns.end()) {
        int new_score = techMapAnnotationScore(conn.from, annotation_map) + techMapAnnotationScore(conn.to, annotation_map);
        auto insert_pos = std::find_if(conns.begin(), conns.end(), [&](const CBConnName& old) {
            int old_score = techMapAnnotationScore(old.from, annotation_map) + techMapAnnotationScore(old.to, annotation_map);
            return new_score < old_score;
        });
        conns.insert(insert_pos, conn);
    }

    rememberOutgoingSrc(*this, from_type, from_value, to_type, to_value);
}

const CBConnName* CBType::connName(CBNodeNameType from_type, int from_value,
                                   CBNodeNameType to_type, int to_value) const
{
    const std::vector<CBConnName>* conns = connNames(from_type, from_value, to_type, to_value);
    if (!conns || conns->empty()) {
        return nullptr;
    }
    return &conns->front();
}

const std::vector<CBConnName>* CBType::connNames(CBNodeNameType from_type, int from_value,
                                                 CBNodeNameType to_type, int to_value) const
{
    if (from_value < 0 || from_value >= CB_MAX_NODES || to_value < 0 || to_value >= CB_MAX_NODES) {
        return nullptr;
    }
    CBConnNameKey key{
        static_cast<uint16_t>(from_type),
        static_cast<uint16_t>(from_value),
        static_cast<uint16_t>(to_type),
        static_cast<uint16_t>(to_value)
    };
    auto it = conn_names.find(key);
    return it == conn_names.end() ? nullptr : &it->second;
}

const std::vector<uint16_t>* CBType::srcNodes(CBNodeNameType from_type, int from_value) const
{
    if (from_value < 0 || from_value >= CB_MAX_NODES) {
        return nullptr;
    }
    CBNodeNameKey key{static_cast<uint16_t>(from_type), static_cast<uint16_t>(from_value)};
    auto it = outgoing_srcs.find(key);
    return it == outgoing_srcs.end() ? nullptr : &it->second;
}

void CBType::rebuildPrioritySrcsByDelta()
{
    src_priority_deltas.clear();
    priority_srcs_by_delta.clear();

    for (const auto& [src, entries] : dst_by_src.values) {
        NodeMask src_bit = NodeMask{0,1} << src;
        auto& deltas = src_priority_deltas[src];
        for (const ResolvedJump& entry : entries) {
            Coord priority_delta = entry.delta;
            if (priority_delta.x < -8 || priority_delta.x > 7
                || priority_delta.y < -8 || priority_delta.y > 7) {
                priority_delta = Coord{jumpDeltaX(src), jumpDeltaY(src)};
            }
            if (priority_delta.x == 0 && priority_delta.y == 0) {
                continue;
            }
            priority_srcs_by_delta[jumpIndexForDelta(priority_delta.x, priority_delta.y, 0)].jump |= src_bit;
            bool known = false;
            for (const Coord& delta : deltas) {
                if (delta.x == priority_delta.x && delta.y == priority_delta.y) {
                    known = true;
                    break;
                }
            }
            if (!known) {
                deltas.push_back(priority_delta);
            }
        }
    }
}

void CBType::rebuildOutgoingSrcs()
{
    outgoing_srcs.clear();
    ordered_srcs_by_target.clear();
    ordered_node_srcs_by_target.clear();
    terminal_entries_by_local.clear();
    derived_masks_valid = false;
    rebuildPrioritySrcsByDelta();
    joint_reachable_srcs.clear();
    src_reachable_joints.clear();
    local_reachable_joints.clear();
    dsts_reaching_src.clear();
    dsts_reaching_local.clear();
    valid_dst_nodes = {};
    const CBType& self = *this;

    for (int joint = 0; joint < CB_MAX_NODES; ++joint) {
        self.joint_src[joint].jump.for_each_set_bit([&](int src) {
            joint_reachable_srcs[joint].jump |= NodeMask{0,1} << src;
            src_reachable_joints[src].joint |= NodeMask{0,1} << joint;
            return false;
        });
        self.joint_local[joint].local.for_each_set_bit([&](int local) {
            local_reachable_joints[local].joint |= NodeMask{0,1} << joint;
            return false;
        });
    }

    for (int src = 0; src < CB_MAX_NODES; ++src) {
        self.src_joint[src].joint.for_each_set_bit([&](int joint) {
            joint_reachable_srcs[joint].jump |= NodeMask{0,1} << src;
            src_reachable_joints[src].joint |= NodeMask{0,1} << joint;
            return false;
        });
    }

    auto add_dst_to_src = [&](int dst, NodeMask srcs) {
        srcs.for_each_set_bit([&](int src) {
            dsts_reaching_src[src].jump |= NodeMask{0,1} << dst;
            return false;
        });
    };

    auto add_dst_to_local = [&](int dst, NodeMask locals) {
        locals.for_each_set_bit([&](int local) {
            dsts_reaching_local[local].jump |= NodeMask{0,1} << dst;
            return false;
        });
    };

    auto add_dst_through_joints = [&](int dst, NodeMask joints) {
        joints.for_each_set_bit([&](int joint) {
            add_dst_to_src(dst, joint_reachable_srcs[joint].jump);
            add_dst_to_local(dst, self.joint_local[joint].local);
            self.joint_joint[joint].joint.for_each_set_bit([&](int next_joint) {
                add_dst_to_src(dst, joint_reachable_srcs[next_joint].jump);
                add_dst_to_local(dst, self.joint_local[next_joint].local);
                return false;
            });
            return false;
        });
    };

    for (int dst = 0; dst < CB_MAX_NODES; ++dst) {
        if (self.dst_src[dst].jump != NodeMask{} || self.dst_local[dst].local != NodeMask{} || self.dst_joint[dst].joint != NodeMask{}) {
            valid_dst_nodes |= NodeMask{0,1} << dst;
        }
        add_dst_to_src(dst, self.dst_src[dst].jump);
        add_dst_to_local(dst, self.dst_local[dst].local);
        add_dst_through_joints(dst, self.dst_joint[dst].joint);

    }

    auto add_src = [&](CBNodeNameType from_type, int from_value, int src_value) {
        rememberOutgoingSrc(*this, from_type, from_value, CB_NODE_SRC, src_value);
    };

    auto add_joint_srcs = [&](CBNodeNameType from_type, int from_value, NodeMask joints) {
        joints.for_each_set_bit([&](int joint) {
            joint_reachable_srcs[joint].jump.for_each_set_bit([&](int src) {
                add_src(from_type, from_value, src);
                return false;
            });
            self.joint_joint[joint].joint.for_each_set_bit([&](int next_joint) {
                joint_reachable_srcs[next_joint].jump.for_each_set_bit([&](int src) {
                    add_src(from_type, from_value, src);
                    return false;
                });
                return false;
            });
            return false;
        });
    };

    for (int local = 0; local < CB_MAX_NODES; ++local) {
        self.local_src[local].jump.for_each_set_bit([&](int src) {
            add_src(CB_NODE_LOCAL, local, src);
            return false;
        });
        add_joint_srcs(CB_NODE_LOCAL, local, self.local_joint[local].joint);
    }

    for (int dst = 0; dst < CB_MAX_NODES; ++dst) {
        self.dst_src[dst].jump.for_each_set_bit([&](int src) {
            add_src(CB_NODE_DST, dst, src);
            return false;
        });
        add_joint_srcs(CB_NODE_DST, dst, self.dst_joint[dst].joint);
    }

    for (int joint = 0; joint < CB_MAX_NODES; ++joint) {
        self.joint_src[joint].jump.for_each_set_bit([&](int src) {
            add_src(CB_NODE_JOINT, joint, src);
            return false;
        });
        add_joint_srcs(CB_NODE_JOINT, joint, self.joint_joint[joint].joint);
    }

    derived_masks_valid = true;
}

const std::vector<uint16_t>& CBType::orderedSrcNodes(const Coord& target_delta)
{
    int target_dx = 0;
    int target_dy = 0;
    jumpTargetBucket(target_delta, target_dx, target_dy);
    uint16_t target_key = static_cast<uint16_t>((target_dx + 7) * 15 + (target_dy + 7));
    auto known = ordered_srcs_by_target.find(target_key);
    if (known != ordered_srcs_by_target.end()) {
        return known->second;
    }

    ensureDerivedMasks();
    std::vector<uint16_t>& ordered = ordered_srcs_by_target[target_key];
    NodeMask valid_srcs;
    for (const auto& [src, entries] : dst_by_src.values) {
        if (!entries.empty()) {
            valid_srcs |= NodeMask{0,1} << src;
        }
    }
    for (const auto& [key, srcs] : outgoing_srcs) {
        (void)key;
        for (uint16_t src : srcs) {
            valid_srcs |= NodeMask{0,1} << src;
        }
    }

    NodeMask seen;
    const std::array<JumpBucket, 224>& bucket_order =
        priorityBucketOrder(target_dx, target_dy);
    for (const JumpBucket& bucket : bucket_order) {
        NodeMask bucket_srcs = valid_srcs & ~seen
            & priority_srcs_by_delta[jumpIndexForDelta(bucket.dx, bucket.dy, 0)].jump;
        std::array<std::vector<uint16_t>, 16> by_lane;
        bucket_srcs.for_each_set_bit([&](int src) {
            by_lane[src & 0xf].push_back(static_cast<uint16_t>(src));
            return false;
        });
        for (const auto& lane_srcs : by_lane) {
            ordered.insert(ordered.end(), lane_srcs.begin(), lane_srcs.end());
        }
        seen |= bucket_srcs;
    }

    NodeMask fallback = valid_srcs & ~seen;
    for (const JumpBucket& bucket : bucket_order) {
        for (int lane = 0; lane < 16; ++lane) {
            int src = jumpIndexForDelta(bucket.dx, bucket.dy, lane);
            if ((fallback & (NodeMask{0,1} << src)) == NodeMask{}) {
                continue;
            }
            ordered.push_back(static_cast<uint16_t>(src));
            seen |= NodeMask{0,1} << src;
        }
    }
    (valid_srcs & ~seen).for_each_set_bit([&](int src) {
        ordered.push_back(static_cast<uint16_t>(src));
        return false;
    });
    return ordered;
}

const std::vector<uint16_t>& CBType::orderedSrcNodes(
    CBNodeNameType from_type, int from_value, const Coord& target_delta)
{
    static const std::vector<uint16_t> empty;
    if (from_value < 0 || from_value >= CB_MAX_NODES) {
        return empty;
    }
    int target_dx = 0;
    int target_dy = 0;
    jumpTargetBucket(target_delta, target_dx, target_dy);
    uint32_t target_key = static_cast<uint32_t>((target_dx + 7) * 15
                                                + (target_dy + 7));
    uint32_t key = (target_key << 14)
        | (static_cast<uint32_t>(from_type) << 12)
        | static_cast<uint32_t>(from_value);
    auto known = ordered_node_srcs_by_target.find(key);
    if (known != ordered_node_srcs_by_target.end()) {
        return known->second;
    }

    std::vector<uint16_t>& filtered = ordered_node_srcs_by_target[key];
    const std::vector<uint16_t>* reachable = srcNodes(from_type, from_value);
    if (!reachable || reachable->empty()) {
        return filtered;
    }
    NodeMask reachable_mask;
    for (uint16_t src : *reachable) {
        reachable_mask.setBit(src);
    }
    for (uint16_t src : orderedSrcNodes(target_delta)) {
        if (reachable_mask.testBit(src)) {
            filtered.push_back(src);
        }
    }
    return filtered;
}

const std::vector<CBType::TerminalEntry>& CBType::terminalEntries(int local)
{
    static const std::vector<TerminalEntry> empty;
    if (local < 0 || local >= CB_MAX_NODES) {
        return empty;
    }
    ensureDerivedMasks();
    auto known = terminal_entries_by_local.find(static_cast<uint16_t>(local));
    if (known != terminal_entries_by_local.end()) {
        return known->second;
    }

    std::vector<TerminalEntry>& paths =
        terminal_entries_by_local[static_cast<uint16_t>(local)];
    auto add = [&](int dst, int joint, int joint2) {
        auto same = [&](const TerminalEntry& path) {
            return path.dst == dst && path.joint == joint && path.joint2 == joint2;
        };
        if (std::find_if(paths.begin(), paths.end(), same) == paths.end()) {
            paths.push_back(TerminalEntry{
                static_cast<uint16_t>(dst), static_cast<int16_t>(joint),
                static_cast<int16_t>(joint2)});
        }
    };

    NodeMask local_bit = NodeMask{0,1} << local;
    NodeMask joints_to_local = local_reachable_joints[local].joint;
    dsts_reaching_local[local].jump.for_each_set_bit([&](int dst) {
        if ((dst_local[dst].local & local_bit) != NodeMask{}) {
            add(dst, -1, -1);
        }
        (dst_joint[dst].joint & joints_to_local).for_each_set_bit([&](int joint) {
            add(dst, joint, -1);
            return false;
        });
        dst_joint[dst].joint.for_each_set_bit([&](int first_joint) {
            (joint_joint[first_joint].joint & joints_to_local)
                .for_each_set_bit([&](int second_joint) {
                    add(dst, second_joint, first_joint);
                    return false;
                });
            return false;
        });
        return false;
    });
    return paths;
}

void CBType::ensureDerivedMasks()
{
    if (!derived_masks_valid) {
        rebuildOutgoingSrcs();
    }
}

NodeMask CBType::dstMaskForSrc(int src) const
{
    NodeMask mask;
    if (src < 0 || src >= CB_MAX_NODES) {
        return mask;
    }
    for (const ResolvedJump& entry : dst_by_src[src]) {
        mask |= entry.dsts.jump;
    }
    return mask;
}

bool CBType::sameDstBySrc(const CBType& other) const
{
    size_t lhs_nonempty = 0;
    size_t rhs_nonempty = 0;
    for (const auto& [src, lhs] : dst_by_src.values) {
        if (lhs.empty()) {
            continue;
        }
        ++lhs_nonempty;
        auto rhs_it = other.dst_by_src.values.find(src);
        if (rhs_it == other.dst_by_src.values.end()) {
            return false;
        }
        const auto& rhs = rhs_it->second;
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (size_t i = 0; i < lhs.size(); ++i) {
            if (lhs[i].delta.x != rhs[i].delta.x
                || lhs[i].delta.y != rhs[i].delta.y
                || lhs[i].target_cb_type_id != rhs[i].target_cb_type_id
                || lhs[i].dsts.jump != rhs[i].dsts.jump
                || lhs[i].dst_wires != rhs[i].dst_wires
                || lhs[i].target_tile_coord != rhs[i].target_tile_coord) {
                return false;
            }
        }
    }
    for (const auto& [src, rhs] : other.dst_by_src.values) {
        (void)src;
        rhs_nonempty += !rhs.empty();
    }
    return lhs_nonempty == rhs_nonempty;
}

bool CBType::sameRoutingSubtype(const CBType& other) const
{
    if (!sameDstBySrc(other)) {
        return false;
    }
    if (src_priority_deltas.size() != other.src_priority_deltas.size()) {
        return false;
    }
    for (const auto& [src, lhs] : src_priority_deltas) {
        auto rhs_it = other.src_priority_deltas.find(src);
        if (rhs_it == other.src_priority_deltas.end() || lhs.size() != rhs_it->second.size()) {
            return false;
        }
        for (size_t i = 0; i < lhs.size(); ++i) {
            if (lhs[i].x != rhs_it->second[i].x || lhs[i].y != rhs_it->second[i].y) {
                return false;
            }
        }
    }
    auto same_sparse_table = []<typename State, typename GetMask>(
                                 const StateTable<State>& lhs,
                                 const StateTable<State>& rhs,
                                 GetMask get_mask) {
        size_t lhs_nonempty = 0;
        size_t rhs_nonempty = 0;
        for (const auto& [node, state] : lhs.values) {
            auto mask = get_mask(state);
            if (mask == NodeMask{}) {
                continue;
            }
            ++lhs_nonempty;
            auto rhs_it = rhs.values.find(node);
            if (rhs_it == rhs.values.end() || get_mask(rhs_it->second) != mask) {
                return false;
            }
        }
        for (const auto& [node, state] : rhs.values) {
            (void)node;
            rhs_nonempty += get_mask(state) != NodeMask{};
        }
        return lhs_nonempty == rhs_nonempty;
    };
    if (!same_sparse_table(priority_srcs_by_delta,
                           other.priority_srcs_by_delta,
                           [](const CBJumpState& state) { return state.jump; })
        || !same_sparse_table(dst_src, other.dst_src,
                             [](const CBJumpState& state) { return state.jump; })
        || !same_sparse_table(dst_local, other.dst_local,
                             [](const CBLocalState& state) { return state.local; })
        || !same_sparse_table(dst_joint, other.dst_joint,
                             [](const CBJointState& state) { return state.joint; })) {
        return false;
    }

    auto subtype_name = [](const CBNodeNameKey& key) {
        return key.type == CB_NODE_SRC || key.type == CB_NODE_DST
            || key.type == CB_NODE_LOCAL || key.type == CB_NODE_JOINT;
    };
    size_t lhs_names = 0;
    size_t rhs_names = 0;
    for (const auto& [key, name] : node_names) {
        if (!subtype_name(key)) {
            continue;
        }
        ++lhs_names;
        auto rhs_it = other.node_names.find(key);
        if (rhs_it == other.node_names.end() || rhs_it->second != name) {
            return false;
        }
    }
    for (const auto& [key, name] : other.node_names) {
        (void)name;
        rhs_names += subtype_name(key);
    }
    if (lhs_names != rhs_names) {
        return false;
    }
    return true;
}

void CBType::preParseNode(std::string name, TechMap& map, bool finish)
{
    if (finish) {  // enum nodes
        int start = 0;
        for (auto& pair : nodes_enum) {
            pair.second.start_num = start;
            PNR_LOG2("CBAR", "giving '{}' group numbers {}-{}", pair.first, pair.second.start_num, pair.second.start_num + pair.second.cnt-1);
            PNR_ASSERT(pair.second.start_num + pair.second.cnt - 1 < CB_MAX_NODES,
                "nodes enum overflows {} for nodes of type '{}'", CB_MAX_NODES, pair.first);
            start += pair.second.cnt;
        }
        return;
    }

    std::string orig_name = name;
    name = stripTypePrefix(std::move(name), this->name);
    if (map.size()) {  // make replacements according to map
        for (auto& expr : map[0]/*line0*/) {
            if (expr.size()) { // has equals
                if (expr[0].size() /*equal has tokens*/) {
                    PNR_ASSERT(expr[0][0].size(), "token must have at least one part");
                    size_t pos;
                    if ((pos = name.find(expr[0][0][0])) != (size_t)-1) {
                        name.replace(pos, expr[0][0][0].length(), expr[1][0][0]);
                        PNR_LOG2("CBAR", "replacing {} with {}", orig_name, name);
                    }
                }
            }
        }
    }

    std::string base;
    int first_id = -1;
    int second_id = -1;
    int last_id = -1;
    size_t last_digit_pos = std::string::npos;
    int nums = 0;
    const char* ptr = name.c_str();
    for (size_t i=0; i < strlen(ptr); ++i) {
        if (ptr[i] >= '0' && ptr[i] <= '9') {
            ++nums;
            last_digit_pos = i;
            last_id = atoi(ptr + i);
            if (nums == 1) {
                size_t digit_pos = i;
                first_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
                base = std::string(ptr, digit_pos);
            }
            if (nums == 2) {
                second_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
        base = name;
    }

    if (nums >= 2 && name.find("SRC") == std::string::npos && name.find("DST") == std::string::npos) {
        nums = 1;
        first_id = last_id;
        base = name.substr(0, last_digit_pos);
    }

    if (nums == 1) {  // !jump
        auto it = nodes_enum.find(base);
        if (it == nodes_enum.end()) {
            nodes_enum.emplace(base, NodeEnum{first_id, 1, 0});
        }
        else {
            // Keep enum ranges valid even when database names are not sorted by numeric suffix.
            if (first_id < it->second.base_id) {
                it->second.cnt += it->second.base_id - first_id;
                it->second.base_id = first_id;
            }
            else if (first_id - it->second.base_id >= it->second.cnt) {
                it->second.cnt = first_id - it->second.base_id + 1;
            }
        }
    }
}

int /*0-3*/ CBType::parseNode(std::string name, TechMap& map,
                     CBLocalNode& local_node, CBJumpNode& src_node, CBJumpNode& dst_node, CBJointNode& joint_node,
                     CBLocalState& local_state, CBJumpState& src_state, CBJumpState& dst_state, CBJointState& joint_state)
{
    std::string orig_name = name;
    name = stripTypePrefix(std::move(name), this->name);
    if (map.size()) {  // make replacements according to map
        for (auto& expr : map[0]/*line0*/) {
            if (expr.size()) { // has equals
                if (expr[0].size() /*equal has tokens*/) {
                    PNR_ASSERT(expr[0][0].size(), "token must have at least one part");
                    size_t pos;
                    if ((pos = name.find(expr[0][0][0])) != (size_t)-1) {
                        name.replace(pos, expr[0][0][0].length(), expr[1][0][0]);
                        PNR_LOG2("CBAR", "replacing {} with {}", orig_name, name);
                    }
                }
            }
        }
    }

    std::string base;
    int first_id = -1;
    int second_id = -1;
    int last_id = -1;
    size_t last_digit_pos = std::string::npos;
    int nums = 0;
    const char* ptr = name.c_str();
    for (size_t i=0; i < strlen(ptr); ++i) {
        if (ptr[i] >= '0' && ptr[i] <= '9') {
            ++nums;
            last_digit_pos = i;
            last_id = atoi(ptr + i);
            if (nums == 1) {
                size_t digit_pos = i;
                first_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
                base = std::string(ptr, digit_pos);
            }
            if (nums == 2) {
                second_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
        base = name;
    }

    if (nums >= 2 && name.find("SRC") == std::string::npos && name.find("DST") == std::string::npos) {
        nums = 1;
        first_id = last_id;
        base = name.substr(0, last_digit_pos);
    }

    if (nums == 1) {
        auto it = nodes_enum.find(base);
        if (it == nodes_enum.end()) {
            return -1;
        }
        if (name.find("JOINT") != (size_t)-1) {  // joint
            joint_node.joint = it->second.start_num + first_id - it->second.base_id;
            joint_state.joint = NodeMask{0,1} << joint_node.joint;
            PNR_LOG2("CBAR", "for name '{}' found joint num {} with base {}", name, it->second.start_num + first_id - it->second.base_id, it->second.start_num);
            return 3;
        }
        else {  // local
            local_node.local = it->second.start_num + first_id - it->second.base_id;
            local_state.local = NodeMask{0,1} << local_node.local;
            PNR_LOG2("CBAR", "for name '{}' found local num {} with base {}", name, it->second.start_num + first_id - it->second.base_id, it->second.start_num);
            return 0;
        }
    }
    else {  // name has 2 numbers
        if (map.size() > 1) { // 2 lines
            for (auto& expr : map[1]) { // line 1 has exprs
                if (expr.size() > 1) {  // expr has 2 equals
                    if (expr[1].size() > 1 && expr[1][1].size() == 4) { //right equal has 2 tokens, right token has 4 parts
                        if (name.compare(0, expr[0][0][0].length(), expr[0][0][0]) == 0) {
                            CBJumpNode node = {};
                            PNR_ASSERT(expr[0].size() && expr[0][0].size(), "empty left equal in expr");
                            int length = mappedJumpLength(first_id, expr[1][1]);
                            int dir = atoi(expr[1][0][0].c_str());
                            Coord delta = directionDelta(dir, length);
                            if (name.find("_SD") != std::string::npos) {
                                delta = Coord{0, 1};
                            }
                            else if (name.find("_ND") != std::string::npos) {
                                delta = Coord{0, -1};
                            }
                            CBNodeNameType lane_role = name.find("SRC") != std::string::npos ? CB_NODE_SRC : CB_NODE_DST;
                            std::string lane_key = std::to_string(static_cast<int>(lane_role)) + ":" + jumpLaneKey(name);
                            uint16_t exact_key = static_cast<uint16_t>((jumpDeltaKey(delta.x, delta.y) << 4) | (second_id & 0xf));
                            auto node_it = jump_nodes_by_lane_key.find(lane_key);
                            if (node_it != jump_nodes_by_lane_key.end()) {
                                node.jump = node_it->second;
                                node.num = node.jump & 0xf;
                            }
                            else {
                                uint32_t role_delta_key =
                                    (static_cast<uint32_t>(lane_role) << 16) | jumpDeltaKey(delta.x, delta.y);
                                auto& lanes = jump_lane_keys_by_role_delta[role_delta_key];
                                int lane = -1;
                                if (second_id >= 0 && second_id < 16
                                    && (lanes[second_id].empty() || lanes[second_id] == lane_key)) {
                                    lane = second_id;
                                }
                                if (lane < 0) {
                                    for (int candidate = 0; candidate < 16; ++candidate) {
                                        if (lanes[candidate].empty() || lanes[candidate] == lane_key) {
                                            lane = candidate;
                                            break;
                                        }
                                    }
                                }
                                if (lane < 0) {
                                    std::string used;
                                    for (int candidate = 0; candidate < 16; ++candidate) {
                                        if (!used.empty()) {
                                            used += "; ";
                                        }
                                        used += std::to_string(candidate) + "='" + lanes[candidate] + "'";
                                    }
                                    PNR_ASSERT(false,
                                        "jump lane overuse in cb='{}' for '{}' delta=({}, {}) preferred={} exact_key={} used=[{}]",
                                        this->name, name, delta.x, delta.y, second_id, exact_key, used);
                                }
                                lanes[lane] = lane_key;
                                node.num = lane;
                                node.jump = jumpIndexForDelta(delta.x, delta.y, node.num);
                                jump_nodes_by_lane_key.emplace(lane_key, static_cast<uint16_t>(node.jump));
                            }
                            node.jump = jumpIndexForDelta(delta.x, delta.y, node.num);
                            node.delta_x = encodeSigned4(delta.x);
                            node.delta_y = encodeSigned4(delta.y);
                            CBJumpState state = {};
                            state.jump = NodeMask{0,1} << node.jump;
                            if (name.find("SRC") != (size_t)-1) {
                                src_node = node;
                                src_state = state;
                                PNR_LOG2("CBAR", "for name '{}' found rule '{}', it's src jump num={} dx={} dy={} index={}",
                                    name, expr[0][0][0], static_cast<int>(node.num), delta.x, delta.y, static_cast<int>(node.jump));
                                return 1;
                            }
                            if (name.find("DST") != (size_t)-1) {
                                dst_node = node;
                                dst_state = state;
                                PNR_LOG2("CBAR", "for name '{}' found rule '{}', it's dst jump num={} dx={} dy={} index={}",
                                    name, expr[0][0][0], static_cast<int>(node.num), delta.x, delta.y, static_cast<int>(node.jump));
                                return 2;
                            }
                        }
                    }
                }
            }
        }
    }
    PNR_ASSERT(0, "unknown node type: {}", name);
    return -1;
}

void CBType::loadFromSpec(const CBTypeSpec& spec, TechMap& map)
{
    PNR_LOG1("CBAR", "loadFromSpec, size: {}", spec.nodes.size());
    annotation_map = map;
    nodes_enum.clear();
    node_names.clear();
    local_nodes_by_name.clear();
    src_nodes_by_name.clear();
    dst_nodes_by_name.clear();
    joint_nodes_by_name.clear();
    jump_lane_keys_by_role_delta.clear();
    jump_nodes_by_lane_key.clear();
    conn_names.clear();
    outgoing_srcs.clear();
    derived_masks_valid = false;
    local_src.clear();
    local_joint.clear();
    local_local.clear();
    local_by_local.clear();
    src_joint.clear();
    dst_by_src.clear();
    src_priority_deltas.clear();
    priority_srcs_by_delta.clear();
    joint_src.clear();
    joint_local.clear();
    joint_joint.clear();
    dst_src.clear();
    dst_local.clear();
    dst_joint.clear();
    joint_reachable_srcs.clear();
    src_reachable_joints.clear();
    local_reachable_joints.clear();
    dsts_reaching_src.clear();
    dsts_reaching_local.clear();
    valid_dst_nodes = {};
    local_input_nodes = {};
    local_output_nodes = {};
    constant_one_nodes = {};
    constant_zero_nodes = {};
    for (const auto& pair : spec.nodes) {
        PNR_LOG2("CBAR", "loadFromSpec, pair: {} {}", pair.first, pair.second);
        preParseNode(pair.first, map, false);
        preParseNode(pair.second, map, false);
    }
    preParseNode("", map, true);
    for (const auto& pair : spec.nodes) {
        CBJumpNode a_src_node = {}, b_src_node = {};
        CBJumpNode a_dst_node = {}, b_dst_node = {};
        CBLocalNode a_local_node = {}, b_local_node = {};
        CBJointNode a_joint_node = {}, b_joint_node = {};
        CBJumpState a_src_state = {}, b_src_state = {};
        CBJumpState a_dst_state = {}, b_dst_state = {};
        CBLocalState a_local_state = {}, b_local_state = {};
        CBJointState a_joint_state = {}, b_joint_state = {};

        int type_a = parseNode(pair.first, map, a_local_node, a_src_node, a_dst_node, a_joint_node, a_local_state, a_src_state, a_dst_state, a_joint_state);
        int type_b = parseNode(pair.second, map, b_local_node, b_src_node, b_dst_node, b_joint_node, b_local_state, b_src_state, b_dst_state, b_joint_state);

        PNR_ASSERT(type_a != -1 && type_b != -1, "cant parse node type: {} {}: {}, {}\n", pair.first, pair.second, type_a, type_b);
        bool debug_pair = debugCBPairMatches(pair.first, pair.second);
        if (debug_pair) {
            PNR_LOG1("CBAR", "debug pair cb='{}' '{} -> {}' type_a={} type_b={} a_local={} a_src={} a_dst={} a_joint={} b_local={} b_src={} b_dst={} b_joint={} a_src_state={} a_dst_state={} b_local_state={} b_src_state={} b_dst_state={}",
                name, pair.first, pair.second, type_a, type_b,
                static_cast<int>(a_local_node.local), static_cast<int>(a_src_node.jump),
                static_cast<int>(a_dst_node.jump), static_cast<int>(a_joint_node.joint),
                static_cast<int>(b_local_node.local), static_cast<int>(b_src_node.jump),
                static_cast<int>(b_dst_node.jump), static_cast<int>(b_joint_node.joint),
                a_src_state.jump.str(), a_dst_state.jump.str(), b_local_state.local.str(), b_src_state.jump.str(), b_dst_state.jump.str());
        }
        rememberParsedNode(*this, type_a, a_local_node, a_src_node, a_dst_node, a_joint_node, pair.first, map);
        rememberParsedNode(*this, type_b, b_local_node, b_src_node, b_dst_node, b_joint_node, pair.second, map);
        CBNodeNameType a_name_type = CB_NODE_LOCAL;
        CBNodeNameType b_name_type = CB_NODE_LOCAL;
        int a_value = -1;
        int b_value = -1;
        if (parsedNodeKey(type_a, a_local_node, a_src_node, a_dst_node, a_joint_node, a_name_type, a_value)
            && parsedNodeKey(type_b, b_local_node, b_src_node, b_dst_node, b_joint_node, b_name_type, b_value)) {
            ::rememberConnName(*this, a_name_type, a_value, b_name_type, b_value, pair.first, pair.second, map);
        }

        if (type_a == 0) {  // local
            if (type_b == 0) {  // local
                local_local[a_local_node.local].local |= b_local_state.local;
                local_output_nodes |= a_local_state.local;
                local_input_nodes |= b_local_state.local;
            }
            if (type_b == 1) {  // src
                local_src[a_local_node.local].jump |= b_src_state.jump;
                local_output_nodes |= a_local_state.local;
            }
            if (type_b == 2) {  // dst
                local_output_nodes |= a_local_state.local;
                PNR_ASSERT(0, "wire from local to dst {}\n", pair.first, pair.second);
            }
            if (type_b == 3) {  // joint
                local_joint[a_local_node.local].joint |= b_joint_state.joint;
                local_output_nodes |= a_local_state.local;
            }
        }
        if (type_a == 1) {  // src
            if (type_b == 0) {  // local
                local_input_nodes |= b_local_state.local;
                PNR_ASSERT(0, "wire from src to local: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 1) {  // src
                PNR_ASSERT(0, "wire from src to src: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 2) {  // dst
                addResolvedJump(dst_by_src[a_src_node.jump],
                                Coord{jumpDeltaX(a_src_node.jump), jumpDeltaY(a_src_node.jump)},
                                type_id, b_dst_state);
            }
            if (type_b == 3) {  // joint
                src_joint[a_src_node.jump].joint |= b_joint_state.joint;
            }
        }
        if (type_a == 2) {  // dst
            if (type_b == 0) {  // local
                dst_local[a_dst_node.jump].local |= b_local_state.local;
                local_input_nodes |= b_local_state.local;
                if (debug_pair) {
                    PNR_LOG1("CBAR", "debug pair loaded dst_local cb='{}' cb_ptr={} dst={} local_mask={} dst_local={}",
                        name, static_cast<const void*>(this), static_cast<int>(a_dst_node.jump), b_local_state.local.str(), dst_local[a_dst_node.jump].local.str());
                }
            }
            if (type_b == 1) {  // src
                dst_src[a_dst_node.jump].jump |= b_src_state.jump;
            }
            if (type_b == 2) {  // dst
                PNR_ASSERT(0, "wire from dst to dst: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 3) {  // joint
                dst_joint[a_dst_node.jump].joint |= b_joint_state.joint;
            }
        }
        if (type_a == 3) {  // joint
            if (type_b == 0) {  // local
                joint_local[a_joint_node.joint].local |= b_local_state.local;
                local_input_nodes |= b_local_state.local;
            }
            if (type_b == 1) {  // src
                joint_src[a_joint_node.joint].jump |= b_src_state.jump;
            }
            if (type_b == 2) {  // dst
                PNR_ASSERT(0, "wire from joint to dst: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 3) {  // joint
                joint_joint[a_joint_node.joint].joint |= b_joint_state.joint;
            }
        }
    }
    for (const auto& [name, src] : src_nodes_by_name) {
        if (src < CB_MAX_NODES) {
            CBJumpState self_dst{};
            self_dst.jump = NodeMask{0,1} << src;
            addResolvedJump(dst_by_src[src], Coord{jumpDeltaX(src), jumpDeltaY(src)}, type_id, self_dst);
        }
    }
    rebuildOutgoingSrcs();
    int debug_dst = debugEnvInt("SCALEPNR_DEBUG_CB_DST");
    int debug_local = debugEnvInt("SCALEPNR_DEBUG_CB_LOCAL");
    if (debug_dst >= 0 && debug_dst < CB_MAX_NODES) {
        bool local_bit = debug_local >= 0 && debug_local < CB_MAX_NODES
            && (dst_local[debug_dst].local & (NodeMask{0,1} << debug_local)) != NodeMask{};
        PNR_LOG1("CBAR", "debug final cb='{}' cb_ptr={} dst={} local={} local_bit={} dst_local={}",
            name, static_cast<const void*>(this), debug_dst, debug_local, local_bit, dst_local[debug_dst].local.str());
    }
    for (int pos=0; pos < CB_MAX_NODES; ++pos) {
        PNR_LOG3("CBAR", "loadFromSpec, local_src[{}]: {}", pos, local_src[pos].jump.str());
        PNR_LOG3("CBAR", "loadFromSpec, local_joint[{}]: {}", pos, local_joint[pos].joint.str());
        PNR_LOG3("CBAR", "loadFromSpec, local_local[{}]: {}", pos, local_local[pos].local.str());
        PNR_LOG3("CBAR", "loadFromSpec, src_joint[{}]: {}", pos, src_joint[pos].joint.str());
        PNR_LOG3("CBAR", "loadFromSpec, joint_src[{}]: {}", pos, joint_src[pos].jump.str());
        PNR_LOG3("CBAR", "loadFromSpec, joint_local[{}]: {}", pos, joint_local[pos].local.str());
        PNR_LOG3("CBAR", "loadFromSpec, joint_joint[{}]: {}", pos, joint_joint[pos].joint.str());
        PNR_LOG3("CBAR", "loadFromSpec, dst_src[{}]: {}", pos, dst_src[pos].jump.str());
        PNR_LOG3("CBAR", "loadFromSpec, dst_local[{}]: {}", pos, dst_local[pos].local.str());
        PNR_LOG3("CBAR", "loadFromSpec, dst_joint[{}]: {}", pos, dst_joint[pos].joint.str());
    }
}

int CBType::localNodeNum(const std::string& name) const
{
    std::string normalized = stripTypePrefix(name, this->name);
    std::string base;
    int first_id = -1;
    int last_id = -1;
    size_t last_digit_pos = std::string::npos;
    int nums = 0;
    const char* ptr = normalized.c_str();
    for (size_t i=0; i < strlen(ptr); ++i) {
        if (ptr[i] >= '0' && ptr[i] <= '9') {
            ++nums;
            last_digit_pos = i;
            last_id = atoi(ptr + i);
            if (nums == 1) {
                size_t digit_pos = i;
                first_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
                base = std::string(ptr, digit_pos);
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
        base = normalized;
    }
    if (nums >= 2 && normalized.find("SRC") == std::string::npos && normalized.find("DST") == std::string::npos) {
        nums = 1;
        first_id = last_id;
        base = normalized.substr(0, last_digit_pos);
    }
    if (nums != 1 || normalized.find("JOINT") != (size_t)-1) {
        return -1;
    }

    auto it = nodes_enum.find(base);
    if (it == nodes_enum.end() || first_id < it->second.base_id || first_id - it->second.base_id >= it->second.cnt) {
        return -1;
    }
    return it->second.start_num + first_id - it->second.base_id;
}

bool CBType::canOut(int local, int src, int orig_curr, int& joint, int* first_joint)
{
    ensureDerivedMasks();
    PNR_LOG3("CBAR", "canOut, local: {}, src: {}, local_src[local]: {}, local_joint[local]: {}, src_joint[src]: {},  intersect: {}",
        local, src, local_src[local].jump.str(), local_joint[local].joint.str(), src_joint[src].joint.str(), (local_joint[local].joint&src_joint[src].joint).str());
    joint = -1;
    if (first_joint) {
        *first_joint = -1;
    }
    if (local_src[local].jump.testBit(src)) {  // direct path
        return true;
    }
    // trying joint
    NodeMask local_to_joints = local_joint[local].joint;
    NodeMask joints_to_src = src_reachable_joints[src].joint;
    NodeMask intersect = local_to_joints&joints_to_src;
    if ((joint = intersect.firstSetBit()) != -1) {
        return true;
    }
    // joint to joint
    return local_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (joints_to_src&joint_joint[index].joint).firstSetBit()) != -1) {
                if (first_joint) {
                    *first_joint = index;
                }
                PNR_LOG3("CBAR", "canOut, found double joint {} for local_to_joints {} and joint_joint[index] {} and joints_to_src {}", 
                    joint, local_to_joints.str(), joint_joint[index].joint.str(), joints_to_src.str());
                return true;
            }
            return false;
        }
    );
}

bool CBType::canJump(int dst, int src, int orig_curr, int& joint, int* first_joint)
{
    ensureDerivedMasks();
    PNR_LOG3("CBAR", "canJump, dst: {}, src: {}, dst_src[dst]: {}, dst_joint[dst]: {}, src_joint[src]: {},  intersect: {}",
        dst, src, dst_src[dst].jump.str(), dst_joint[dst].joint.str(), src_joint[src].joint.str(), (dst_joint[dst].joint&src_joint[src].joint).str());
    joint = -1;
    if (first_joint) {
        *first_joint = -1;
    }
    if (dst_src[dst].jump.testBit(src)) {  // direct path
        return true;
    }
    // trying joint
    NodeMask dst_to_joints = dst_joint[dst].joint;
    NodeMask joints_to_src = src_reachable_joints[src].joint;
    NodeMask intersect = dst_to_joints&joints_to_src;
    if ((joint = intersect.firstSetBit()) != -1) {
        return true;
    }
    return dst_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (joints_to_src&joint_joint[index].joint).firstSetBit()) != -1) {
                if (first_joint) {
                    *first_joint = index;
                }
                PNR_LOG3("CBAR", "canOut, found double joint {} for dst_to_joints {} and joint_joint[index] {} and joints_to_src {}", 
                    joint, dst_to_joints.str(), joint_joint[index].joint.str(), joints_to_src.str());
                return true;
            }
            return false;
        }
    );
}

bool CBType::canIn(int dst, int local, int& joint, int* first_joint)
{
    ensureDerivedMasks();
    NodeMask joints_to_local = local_reachable_joints[local].joint;
    PNR_LOG3("CBAR", "canIn, dst: {}, local: {}, dst_local[dst]: {}, dst_joint[dst]: {}, joint_local->local: {},  intersect: {}",
        dst, local, dst_local[dst].local.str(), dst_joint[dst].joint.str(), joints_to_local.str(), (dst_joint[dst].joint&joints_to_local).str());
    joint = -1;
    if (first_joint) {
        *first_joint = -1;
    }
    if (dst_local[dst].local.testBit(local)) {  // direct path
        return true;
    }
    // Destination entry through a joint uses dst->joint and joint->local relations.
    NodeMask dst_to_joints = dst_joint[dst].joint;
    NodeMask intersect = dst_to_joints&joints_to_local;
    if ((joint = intersect.firstSetBit()) != -1) {
        return true;
    }
    return dst_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (joints_to_local&joint_joint[index].joint).firstSetBit()) != -1) {
                if (first_joint) {
                    *first_joint = index;
                }
                PNR_LOG3("CBAR", "canIn, found double joint {} for dst_to_joints {} and joint_joint[index] {} and joints_to_local {}",
                    joint, dst_to_joints.str(), joint_joint[index].joint.str(), joints_to_local.str());
                return true;
            }
            return false;
        }
    );
}

bool CBType::canInAvoidingJoint(int local, int blocked_joint)
{
    ensureDerivedMasks();
    if (local < 0 || blocked_joint < 0) {
        return false;
    }
    NodeMask blocked = NodeMask{0,1} << blocked_joint;
    NodeMask direct_joints = local_reachable_joints[local].joint;
    return dsts_reaching_local[local].jump.for_each_set_bit([&](int dst) {
        if ((dst_local[dst].local & (NodeMask{0,1} << local)) != NodeMask{}) {
            return true;
        }
        NodeMask first_joints = dst_joint[dst].joint;
        if ((first_joints & direct_joints & ~blocked) != NodeMask{}) {
            return true;
        }
        return first_joints.for_each_set_bit([&](int first_joint) {
            if (first_joint == blocked_joint) {
                return false;
            }
            return (joint_joint[first_joint].joint & direct_joints & ~blocked) != NodeMask{};
        });
    });
}

int CBState::iterateSrcMask(NodeMask candidates, const Coord& from, const Coord& to,
                            int curr, bool ignore_deadend)
{
    if (!type) {
        return -1;
    }
    candidates &= ~src.jump;
    if (!ignore_deadend) {
        candidates &= ~src_deadend.jump;
    }

    bool past_curr = curr < 0;
    for (uint16_t candidate : type->orderedSrcNodes(to - from)) {
        if (!past_curr) {
            if (candidate == curr) {
                past_curr = true;
            }
            continue;
        }
        if ((candidates & (NodeMask{0,1} << candidate)) != NodeMask{}) {
            return candidate;
        }
    }
    return -1;
}

int CBState::iterate(bool jump, int pos, const Coord& from, const Coord& to, int curr, bool ignore_deadend)
{
    if (!type || pos < 0 || pos >= CB_MAX_NODES) {
        return -1;
    }
    NodeMask candidates = jump ? type->dst_src[pos].jump : type->local_src[pos].jump;
    return iterateSrcMask(candidates, from, to, curr, ignore_deadend);
}

bool CBState::hasFreeOut(int pos)
{
    if (!type || pos < 0 || pos >= CB_MAX_NODES) {
        return false;
    }
    type->ensureDerivedMasks();
    auto has_free_resolved_src = [&](NodeMask candidates) {
        candidates &= ~src.jump;
        return candidates.for_each_set_bit([&](int candidate) {
            return !type->dst_by_src[candidate].empty();
        });
    };
    if (has_free_resolved_src(type->local_src[pos].jump)) {
        return true;
    }
    NodeMask first_joints = type->local_joint[pos].joint & ~joint.jump;
    return first_joints.for_each_set_bit([&](int first_joint) {
        if (has_free_resolved_src(
                type->joint_reachable_srcs[first_joint].jump)) {
            return true;
        }
        NodeMask second_joints =
            type->joint_joint[first_joint].joint & ~joint.jump;
        return second_joints.for_each_set_bit([&](int second_joint) {
            return has_free_resolved_src(
                type->joint_reachable_srcs[second_joint].jump);
        });
    });
}

bool CBState::leaseOut(int pos, int curr, int orig_curr, int joint)
{
    NodeMask prev_local = local.local;
    NodeMask prev_src = src.jump;

    if ((src_deadend.jump & (NodeMask{0,1}<<curr)) != NodeMask{}) {
        return false;
    }
    local.local |= NodeMask{0,1}<<pos;
    src.jump |= NodeMask{0,1}<<curr;

    if (local.local == prev_local || src.jump == prev_src) {  // already busy
        local.local = prev_local;
        src.jump = prev_src;
        return false;
    }
    return true;
}

bool CBState::leaseJump(int pos, int curr, int orig_curr, int joint)
{
    NodeMask prev_dst = dst.jump;
    NodeMask prev_src = src.jump;

    if ((src_deadend.jump & (NodeMask{0,1}<<curr)) != NodeMask{}) {
        return false;
    }
    dst.jump |= NodeMask{0,1}<<pos;
    src.jump |= NodeMask{0,1}<<curr;

    if (dst.jump == prev_dst || src.jump == prev_src) {  // already busy
        dst.jump = prev_dst;
        src.jump = prev_src;
        return false;
    }
    return true;
}

bool CBState::leaseIn(int pos, int curr, int joint)
{
    NodeMask prev_dst = dst.jump;
    NodeMask prev_local = local.local;
    NodeMask prev_joint = this->joint.jump;

    dst.jump |= NodeMask{0,1}<<pos;
    local.local |= NodeMask{0,1}<<curr;
    if (joint >= 0) {
        this->joint.jump |= NodeMask{0,1} << joint;
    }

    if (dst.jump == prev_dst || local.local == prev_local
        || (joint >= 0 && this->joint.jump == prev_joint)) {  // already busy
        dst.jump = prev_dst;
        local.local = prev_local;
        this->joint.jump = prev_joint;
        return false;
    }
    return true;
}

Coord CBState::makeJump(const Coord& src, int curr, int orig_curr)
{
    return src + Coord{jumpDeltaX(curr), jumpDeltaY(curr)};
}
