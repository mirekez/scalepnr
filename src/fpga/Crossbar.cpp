#include "Crossbar.h"

#include <algorithm>
#include <cstdlib>
#include <limits>

using namespace fpga;

namespace {

int encodeSigned4(int value)
{
    return value & 0xf;
}

int decodeSigned4(int value)
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
    return (encodeSigned4(dx) << 6) | (encodeSigned4(dy) << 2) | (num & 0x3);
}

int jumpDeltaX(int jump)
{
    return decodeSigned4((jump >> 6) & 0xf);
}

int jumpDeltaY(int jump)
{
    return decodeSigned4((jump >> 2) & 0xf);
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
        rememberNodeName(type, CB_NODE_JUMP, dst_node.jump, name, map);
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
    if (type == CB_NODE_JUMP) {
        key.type = CB_NODE_SRC;
        it = node_names.find(key);
        if (it != node_names.end()) {
            return &it->second;
        }
        key.type = CB_NODE_DST;
        it = node_names.find(key);
        if (it != node_names.end()) {
            return &it->second;
        }
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

void CBType::rebuildOutgoingSrcs()
{
    outgoing_srcs.clear();
    derived_masks_valid = false;
    std::fill(std::begin(joint_reachable_srcs), std::end(joint_reachable_srcs), CBJumpState{});
    std::fill(std::begin(src_reachable_joints), std::end(src_reachable_joints), CBJointState{});
    std::fill(std::begin(local_reachable_joints), std::end(local_reachable_joints), CBJointState{});
    std::fill(std::begin(dsts_reaching_src), std::end(dsts_reaching_src), CBJumpState{});
    std::fill(std::begin(dsts_reaching_local), std::end(dsts_reaching_local), CBJumpState{});
    valid_dst_nodes = {};

    for (int joint = 0; joint < CB_MAX_NODES; ++joint) {
        joint_src[joint].jump.for_each_set_bit([&](int src) {
            joint_reachable_srcs[joint].jump |= u256{0,1} << src;
            src_reachable_joints[src].joint |= u256{0,1} << joint;
            return false;
        });
        joint_local[joint].local.for_each_set_bit([&](int local) {
            local_reachable_joints[local].joint |= u256{0,1} << joint;
            return false;
        });
    }

    for (int src = 0; src < CB_MAX_NODES; ++src) {
        src_joint[src].joint.for_each_set_bit([&](int joint) {
            joint_reachable_srcs[joint].jump |= u256{0,1} << src;
            src_reachable_joints[src].joint |= u256{0,1} << joint;
            return false;
        });
    }

    auto add_dst_to_src = [&](int dst, u256 srcs) {
        srcs.for_each_set_bit([&](int src) {
            dsts_reaching_src[src].jump |= u256{0,1} << dst;
            return false;
        });
    };

    auto add_dst_to_local = [&](int dst, u256 locals) {
        locals.for_each_set_bit([&](int local) {
            dsts_reaching_local[local].jump |= u256{0,1} << dst;
            return false;
        });
    };

    auto add_dst_through_joints = [&](int dst, u256 joints) {
        joints.for_each_set_bit([&](int joint) {
            add_dst_to_src(dst, joint_reachable_srcs[joint].jump);
            add_dst_to_local(dst, joint_local[joint].local);
            joint_joint[joint].joint.for_each_set_bit([&](int next_joint) {
                add_dst_to_src(dst, joint_reachable_srcs[next_joint].jump);
                add_dst_to_local(dst, joint_local[next_joint].local);
                return false;
            });
            return false;
        });
    };

    for (int dst = 0; dst < CB_MAX_NODES; ++dst) {
        if (dst_src[dst].jump != u256{} || dst_local[dst].local != u256{} || dst_joint[dst].joint != u256{}) {
            valid_dst_nodes |= u256{0,1} << dst;
        }
        add_dst_to_src(dst, dst_src[dst].jump);
        add_dst_to_local(dst, dst_local[dst].local);
        add_dst_through_joints(dst, dst_joint[dst].joint);
    }

    auto add_src = [&](CBNodeNameType from_type, int from_value, int src_value) {
        rememberOutgoingSrc(*this, from_type, from_value, CB_NODE_SRC, src_value);
    };

    auto add_joint_srcs = [&](CBNodeNameType from_type, int from_value, u256 joints) {
        joints.for_each_set_bit([&](int joint) {
            joint_reachable_srcs[joint].jump.for_each_set_bit([&](int src) {
                add_src(from_type, from_value, src);
                return false;
            });
            joint_joint[joint].joint.for_each_set_bit([&](int next_joint) {
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
        local_src[local].jump.for_each_set_bit([&](int src) {
            add_src(CB_NODE_LOCAL, local, src);
            return false;
        });
        add_joint_srcs(CB_NODE_LOCAL, local, local_joint[local].joint);
    }

    for (int dst = 0; dst < CB_MAX_NODES; ++dst) {
        dst_src[dst].jump.for_each_set_bit([&](int src) {
            add_src(CB_NODE_DST, dst, src);
            return false;
        });
        add_joint_srcs(CB_NODE_DST, dst, dst_joint[dst].joint);
    }

    for (int joint = 0; joint < CB_MAX_NODES; ++joint) {
        joint_src[joint].jump.for_each_set_bit([&](int src) {
            add_src(CB_NODE_JOINT, joint, src);
            return false;
        });
        add_joint_srcs(CB_NODE_JOINT, joint, joint_joint[joint].joint);
    }
    derived_masks_valid = true;
}

void CBType::ensureDerivedMasks()
{
    if (!derived_masks_valid) {
        rebuildOutgoingSrcs();
    }
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
            joint_state.joint = u256{0,1} << joint_node.joint;
            PNR_LOG2("CBAR", "for name '{}' found joint num {} with base {}", name, it->second.start_num + first_id - it->second.base_id, it->second.start_num);
            return 3;
        }
        else {  // local
            local_node.local = it->second.start_num + first_id - it->second.base_id;
            local_state.local = u256{0,1} << local_node.local;
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
                            node.num = second_id;
                            PNR_ASSERT(expr[0].size() && expr[0][0].size(), "empty left equal in expr");
                            int length = mappedJumpLength(first_id, expr[1][1]);
                            int dir = atoi(expr[1][0][0].c_str());
                            Coord delta = directionDelta(dir, length);
                            node.jump = jumpIndexForDelta(delta.x, delta.y, node.num);
                            node.delta_x = encodeSigned4(delta.x);
                            node.delta_y = encodeSigned4(delta.y);
                            CBJumpState state = {};
                            state.jump = u256{0,1} << node.jump;
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
    conn_names.clear();
    outgoing_srcs.clear();
    derived_masks_valid = false;
    std::fill(std::begin(local_src), std::end(local_src), CBJumpState{});
    std::fill(std::begin(local_joint), std::end(local_joint), CBJointState{});
    std::fill(std::begin(local_local), std::end(local_local), CBLocalState{});
    std::fill(std::begin(src_joint), std::end(src_joint), CBJointState{});
    std::fill(std::begin(src_dst), std::end(src_dst), CBJumpState{});
    for (auto& by_jump : src_dst_by_jump) {
        by_jump.clear();
    }
    std::fill(std::begin(joint_src), std::end(joint_src), CBJumpState{});
    std::fill(std::begin(joint_local), std::end(joint_local), CBLocalState{});
    std::fill(std::begin(joint_joint), std::end(joint_joint), CBJointState{});
    std::fill(std::begin(dst_src), std::end(dst_src), CBJumpState{});
    std::fill(std::begin(dst_local), std::end(dst_local), CBLocalState{});
    std::fill(std::begin(dst_joint), std::end(dst_joint), CBJointState{});
    std::fill(std::begin(joint_reachable_srcs), std::end(joint_reachable_srcs), CBJumpState{});
    std::fill(std::begin(src_reachable_joints), std::end(src_reachable_joints), CBJointState{});
    std::fill(std::begin(local_reachable_joints), std::end(local_reachable_joints), CBJointState{});
    std::fill(std::begin(dsts_reaching_src), std::end(dsts_reaching_src), CBJumpState{});
    std::fill(std::begin(dsts_reaching_local), std::end(dsts_reaching_local), CBJumpState{});
    valid_dst_nodes = {};
    local_input_nodes = {};
    local_output_nodes = {};
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
                src_dst[a_src_node.jump].jump |= b_dst_state.jump;
                src_dst_by_jump[a_src_node.jump][a_src_node.jump].jump |= b_dst_state.jump;
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
            src_dst[src].jump |= u256{0,1} << src;
            src_dst_by_jump[src][src].jump |= u256{0,1} << src;
        }
    }
    rebuildOutgoingSrcs();
    int debug_dst = debugEnvInt("SCALEPNR_DEBUG_CB_DST");
    int debug_local = debugEnvInt("SCALEPNR_DEBUG_CB_LOCAL");
    if (debug_dst >= 0 && debug_dst < CB_MAX_NODES) {
        bool local_bit = debug_local >= 0 && debug_local < CB_MAX_NODES
            && (dst_local[debug_dst].local & (u256{0,1} << debug_local)) != u256{};
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
    std::string base;
    int first_id = -1;
    int nums = 0;
    const char* ptr = name.c_str();
    for (size_t i=0; i < strlen(ptr); ++i) {
        if (ptr[i] >= '0' && ptr[i] <= '9') {
            ++nums;
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
        base = name;
    }
    if (nums != 1 || name.find("JOINT") != (size_t)-1) {
        return -1;
    }

    auto it = nodes_enum.find(base);
    if (it == nodes_enum.end() || first_id < it->second.base_id || first_id - it->second.base_id >= it->second.cnt) {
        return -1;
    }
    return it->second.start_num + first_id - it->second.base_id;
}

bool CBType::canOut(int local, int src, int orig_curr, int& joint)
{
    ensureDerivedMasks();
    PNR_LOG3("CBAR", "canOut, local: {}, src: {}, local_src[local]: {}, local_joint[local]: {}, src_joint[src]: {},  intersect: {}",
        local, src, local_src[local].jump.str(), local_joint[local].joint.str(), src_joint[src].joint.str(), (local_joint[local].joint&src_joint[src].joint).str());
    joint = -1;
    if ((local_src[local].jump&(u256{0,1}<<src)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 local_to_joints = local_joint[local].joint;
    u256 joints_to_src = src_reachable_joints[src].joint;
    u256 intersect = local_to_joints&joints_to_src;
    if ((joint = intersect.ffs256()) != -1) {
        return true;
    }
    // joint to joint
    return local_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (joints_to_src&joint_joint[index].joint).ffs256()) != -1) {
                PNR_LOG3("CBAR", "canOut, found double joint {} for local_to_joints {} and joint_joint[index] {} and joints_to_src {}", 
                    joint, local_to_joints.str(), joint_joint[index].joint.str(), joints_to_src.str());
                return true;
            }
            return false;
        }
    );
}

bool CBType::canJump(int dst, int src, int orig_curr, int& joint)
{
    ensureDerivedMasks();
    PNR_LOG3("CBAR", "canJump, dst: {}, src: {}, dst_src[dst]: {}, dst_joint[dst]: {}, src_joint[src]: {},  intersect: {}",
        dst, src, dst_src[dst].jump.str(), dst_joint[dst].joint.str(), src_joint[src].joint.str(), (dst_joint[dst].joint&src_joint[src].joint).str());
    joint = -1;
    if ((dst_src[dst].jump&(u256{0,1}<<src)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 dst_to_joints = dst_joint[dst].joint;
    u256 joints_to_src = src_reachable_joints[src].joint;
    u256 intersect = dst_to_joints&joints_to_src;
    if ((joint = intersect.ffs256()) != -1) {
        return true;
    }
    return dst_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (joints_to_src&joint_joint[index].joint).ffs256()) != -1) {
                PNR_LOG3("CBAR", "canOut, found double joint {} for dst_to_joints {} and joint_joint[index] {} and joints_to_src {}", 
                    joint, dst_to_joints.str(), joint_joint[index].joint.str(), joints_to_src.str());
                return true;
            }
            return false;
        }
    );
}

bool CBType::canIn(int dst, int local, int& joint)
{
    ensureDerivedMasks();
    u256 joints_to_local = local_reachable_joints[local].joint;
    PNR_LOG3("CBAR", "canIn, dst: {}, local: {}, dst_local[dst]: {}, dst_joint[dst]: {}, joint_local->local: {},  intersect: {}",
        dst, local, dst_local[dst].local.str(), dst_joint[dst].joint.str(), joints_to_local.str(), (dst_joint[dst].joint&joints_to_local).str());
    joint = -1;
    if ((dst_local[dst].local&(u256{0,1}<<local)) != u256{}) {  // direct path
        return true;
    }
    // Destination entry through a proxy joint uses dst->joint and joint->local relations.
    u256 dst_to_joints = dst_joint[dst].joint;
    u256 intersect = dst_to_joints&joints_to_local;
    if ((joint = intersect.ffs256()) != -1) {
        return true;
    }
    return dst_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (joints_to_local&joint_joint[index].joint).ffs256()) != -1) {
                PNR_LOG3("CBAR", "canIn, found double joint {} for dst_to_joints {} and joint_joint[index] {} and joints_to_local {}",
                    joint, dst_to_joints.str(), joint_joint[index].joint.str(), joints_to_local.str());
                return true;
            }
            return false;
        }
    );
}

int CBState::iterate(bool jump, int pos, const Coord& from, const Coord& to, int curr)
{
    if (!type || pos < 0 || pos >= CB_MAX_NODES) {
        return -1;
    }
    u1024 candidates = jump ? type->dst_src[pos].jump : type->local_src[pos].jump;
    candidates &= ~src.jump;
    candidates &= ~src_deadend.jump;

    int best = -1;
    int best_score = std::numeric_limits<int>::max();
    Coord diff = to - from;
    candidates.for_each_set_bit([&](int candidate) {
        if (candidate <= curr) {
            return false;
        }
        Coord delta{jumpDeltaX(candidate), jumpDeltaY(candidate)};
        int score = std::abs((diff.x - delta.x)) + std::abs((diff.y - delta.y));
        if (score < best_score) {
            best = candidate;
            best_score = score;
        }
        return false;
    });
    return best;
}

bool CBState::leaseOut(int pos, int curr, int orig_curr, int joint)
{
    u256 prev_local = local.local;
    u256 prev_src = src.jump;

    if ((src_deadend.jump & (u256{0,1}<<curr)) != u256{}) {
        return false;
    }
    local.local |= u256{0,1}<<pos;
    src.jump |= u256{0,1}<<curr;

    if (local.local == prev_local || src.jump == prev_src) {  // already busy
        local.local = prev_local;
        src.jump = prev_src;
        return false;
    }
    return true;
}

bool CBState::leaseJump(int pos, int curr, int orig_curr, int joint)
{
    u256 prev_dst = dst.jump;
    u256 prev_src = src.jump;

    if ((src_deadend.jump & (u256{0,1}<<curr)) != u256{}) {
        return false;
    }
    dst.jump |= u256{0,1}<<pos;
    src.jump |= u256{0,1}<<curr;

    if (dst.jump == prev_dst || src.jump == prev_src) {  // already busy
        dst.jump = prev_dst;
        src.jump = prev_src;
        return false;
    }
    return true;
}

bool CBState::leaseIn(int pos, int curr, int joint)
{
    u256 prev_dst = dst.jump;
    u256 prev_local = local.local;
    u256 prev_joint = this->joint.jump;

    dst.jump |= u256{0,1}<<pos;
    local.local |= u256{0,1}<<curr;
    if (joint >= 0) {
        this->joint.jump |= u256{0,1} << joint;
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
