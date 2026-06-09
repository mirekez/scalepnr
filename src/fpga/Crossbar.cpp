#include "Crossbar.h"

#include <algorithm>
#include <cstdlib>

using namespace fpga;

namespace {

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
    CBNodeNameKey key{static_cast<uint8_t>(node_type), static_cast<uint8_t>(value)};
    auto [it, inserted] = type.node_names.try_emplace(key, display_name);
    if (!inserted && techMapAnnotationScore(display_name, map) < techMapAnnotationScore(it->second, map)) {
        it->second = display_name;
    }

    auto remember_reverse = [&](std::unordered_map<std::string, uint8_t>& reverse_map) {
        reverse_map.try_emplace(name, static_cast<uint8_t>(value));
        reverse_map.try_emplace(display_name, static_cast<uint8_t>(value));
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
        static_cast<uint8_t>(from_type),
        static_cast<uint8_t>(from_value),
        static_cast<uint8_t>(to_type),
        static_cast<uint8_t>(to_value)
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
        CBNodeNameKey from_key{static_cast<uint8_t>(from_type), static_cast<uint8_t>(from_value)};
        auto& srcs = type.outgoing_srcs[from_key];
        uint8_t src = static_cast<uint8_t>(to_value);
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
        CBNodeNameKey from_key{static_cast<uint8_t>(from_type), static_cast<uint8_t>(from_value)};
        auto& srcs = type.outgoing_srcs[from_key];
        uint8_t src = static_cast<uint8_t>(to_value);
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
    CBNodeNameKey key{static_cast<uint8_t>(type), static_cast<uint8_t>(value)};
    auto [it, inserted] = node_names.try_emplace(key, display_name);
    if (!inserted && techMapAnnotationScore(display_name, annotation_map) < techMapAnnotationScore(it->second, annotation_map)) {
        it->second = display_name;
    }

    auto remember_reverse = [&](std::unordered_map<std::string, uint8_t>& map) {
        map.try_emplace(name, static_cast<uint8_t>(value));
        map.try_emplace(display_name, static_cast<uint8_t>(value));
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
    CBNodeNameKey key{static_cast<uint8_t>(type), static_cast<uint8_t>(value)};
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
    const std::unordered_map<std::string, uint8_t>* map = nullptr;
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
        static_cast<uint8_t>(from_type),
        static_cast<uint8_t>(from_value),
        static_cast<uint8_t>(to_type),
        static_cast<uint8_t>(to_value)
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
        static_cast<uint8_t>(from_type),
        static_cast<uint8_t>(from_value),
        static_cast<uint8_t>(to_type),
        static_cast<uint8_t>(to_value)
    };
    auto it = conn_names.find(key);
    return it == conn_names.end() ? nullptr : &it->second;
}

const std::vector<uint8_t>* CBType::srcNodes(CBNodeNameType from_type, int from_value) const
{
    if (from_value < 0 || from_value >= CB_MAX_NODES) {
        return nullptr;
    }
    CBNodeNameKey key{static_cast<uint8_t>(from_type), static_cast<uint8_t>(from_value)};
    auto it = outgoing_srcs.find(key);
    return it == outgoing_srcs.end() ? nullptr : &it->second;
}

void CBType::rebuildOutgoingSrcs()
{
    auto add_src = [&](CBNodeNameType from_type, int from_value, int src_value) {
        rememberOutgoingSrc(*this, from_type, from_value, CB_NODE_SRC, src_value);
    };

    auto joint_can_reach_src = [&](int joint, int src) {
        if ((joint_src[joint].jump & (u256{0,1} << src)) != u256{}) {
            return true;
        }
        if ((src_joint[src].joint & (u256{0,1} << joint)) != u256{}) {
            return true;
        }
        return false;
    };

    auto add_joint_srcs = [&](CBNodeNameType from_type, int from_value, u256 joints) {
        joints.for_each_set_bit([&](int joint) {
            for (int src = 0; src < CB_MAX_NODES; ++src) {
                if (joint_can_reach_src(joint, src)) {
                    add_src(from_type, from_value, src);
                }
            }
            joint_joint[joint].joint.for_each_set_bit([&](int next_joint) {
                for (int src = 0; src < CB_MAX_NODES; ++src) {
                    if (joint_can_reach_src(next_joint, src)) {
                        add_src(from_type, from_value, src);
                    }
                }
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
}

void CBType::preParseNode(std::string name, TechMap& map, bool finish)
{
    if (finish) {  // enum nodes
        int start = 0;
        for (auto& pair : nodes_enum) {
            pair.second.start_num = start;
            PNR_LOG2("CBAR", "giving '{}' group numbers {}-{}", pair.first, pair.second.start_num, pair.second.start_num + pair.second.cnt-1);
            PNR_ASSERT(pair.second.start_num + pair.second.cnt-1 < 256, "nodes enum overflows 256 for nodes of type '{}'", pair.first);
            start += pair.second.cnt;
        }
        return;
    }

    std::string orig_name = name;
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
//    int second_id = -1;
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
            if (nums == 2) {
//                second_id = atoi(ptr + i);
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
        base = name;
    }

    if (nums == 1) {  // !jump
        auto it = nodes_enum.find(base);
        if (it == nodes_enum.end()) {
            nodes_enum.emplace(base, NodeEnum{first_id, 1, 0});
        }
        else {
            if (first_id - it->second.base_id >= it->second.cnt) {
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
            if (nums == 2) {
                second_id = atoi(ptr + i);
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
        base = name;
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
                            node.length = first_id<4?atoi(expr[1][1][first_id].c_str()):-1;
                            PNR_ASSERT(expr[0].size() && expr[0][0].size(), "empty left equal in expr");
                            node.dir = atoi(expr[1][0][0].c_str());
                            CBJumpState state = {};
                            state.dirs[node.dir] = 1 << (node.length*4 + node.num);
                            if (name.find("SRC") != (size_t)-1) {
                                src_node = node;
                                src_state = state;
                                PNR_LOG2("CBAR", "for name '{}' found rule '{}', it's src jump {} {} {}", name, expr[0][0][0], (uint8_t)node.num, (uint8_t)node.length, (uint8_t)node.dir);
                                return 1;
                            }
                            if (name.find("DST") != (size_t)-1) {
                                dst_node = node;
                                dst_state = state;
                                PNR_LOG2("CBAR", "for name '{}' found rule '{}', it's dst jump {} {} {}", name, expr[0][0][0], (uint8_t)node.num, (uint8_t)node.length, (uint8_t)node.dir);
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
    memset(local_src, 0, sizeof(local_src));
    memset(local_joint, 0, sizeof(local_joint));
    memset(local_local, 0, sizeof(local_local));
    memset(src_joint, 0, sizeof(src_joint));
    memset(joint_src, 0, sizeof(joint_src));
    memset(joint_local, 0, sizeof(joint_local));
    memset(joint_joint, 0, sizeof(joint_joint));
    memset(dst_src, 0, sizeof(dst_src));
    memset(dst_local, 0, sizeof(dst_local));
    memset(dst_joint, 0, sizeof(dst_joint));
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
                a_local_node.local, a_src_node.jump, a_dst_node.jump, a_joint_node.joint,
                b_local_node.local, b_src_node.jump, b_dst_node.jump, b_joint_node.joint,
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
                        name, static_cast<const void*>(this), a_dst_node.jump, b_local_state.local.str(), dst_local[a_dst_node.jump].local.str());
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
    int dir = src / 32;
    int path = src % 32;
    int new_dir = search_dirs[orig_curr/32][dir%8];
    src = new_dir*32 + path;

    PNR_LOG3("CBAR", "canOut, local: {}, src: {}, local_src[local]: {}, local_joint[local]: {}, src_joint[src]: {},  intersect: {}",
        local, src, local_src[local].jump.str(), local_joint[local].joint.str(), src_joint[src].joint.str(), (local_joint[local].joint&src_joint[src].joint).str());
    joint = -1;
    if ((local_src[local].jump&(u256{0,1}<<src)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 local_to_joints = local_joint[local].joint;
    u256 joints_to_src = src_joint[src].joint;
    for (int index = 0; index < CB_MAX_NODES; ++index) {
        if ((joint_src[index].jump & (u256{0,1} << src)) != u256{}) {
            joints_to_src |= u256{0,1} << index;
        }
    }
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
    int dir = src / 32;
    int path = src % 32;
    int new_dir = search_dirs[orig_curr/32][dir%8];
    src = new_dir*32 + path;

    PNR_LOG3("CBAR", "canJump, dst: {}, src: {}, dst_src[dst]: {}, dst_joint[dst]: {}, src_joint[src]: {},  intersect: {}",
        dst, src, dst_src[dst].jump.str(), dst_joint[dst].joint.str(), src_joint[src].joint.str(), (dst_joint[dst].joint&src_joint[src].joint).str());
    joint = -1;
    if ((dst_src[dst].jump&(u256{0,1}<<src)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 dst_to_joints = dst_joint[dst].joint;
    u256 joints_to_src = src_joint[src].joint;
    for (int index = 0; index < CB_MAX_NODES; ++index) {
        if ((joint_src[index].jump & (u256{0,1} << src)) != u256{}) {
            joints_to_src |= u256{0,1} << index;
        }
    }
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
    u256 joints_to_local{};
    for (int index = 0; index < CB_MAX_NODES; ++index) {
        if ((joint_local[index].local & (u256{0,1} << local)) != u256{}) {
            joints_to_local |= u256{0,1} << index;
        }
    }
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
    int startDir = -1;
    Coord diff = to - from;
    if (diff.x >= 0 && diff.y <= 0) {
        if (diff.x > -diff.y*3) {
            startDir = 2;
        } else
        if (-diff.y > diff.x*3) {
            startDir = 0;
        } else {
            startDir = 1;
        }
    } else
    if (diff.x >= 0 && diff.y > 0) {
        if (diff.x > diff.y*3) {
            startDir = 2;
        } else
        if (diff.y > diff.x*3) {
            startDir = 4;
        } else {
            startDir = 3;
        }
    } else
    if (diff.x < 0 && diff.y > 0) {
        if (-diff.x > diff.y*3) {
            startDir = 6;
        } else
        if (diff.y > -diff.x*3) {
            startDir = 4;
        } else {
            startDir = 5;
        }
    } else
    if (diff.x < 0 && diff.y <= 0) {
        if (-diff.x > -diff.y*3) {
            startDir = 6;
        } else
        if (-diff.y > -diff.x*3) {
            startDir = 0;
        } else {
            startDir = 7;
        }
    }

    int dir = -1;
    int path = -1;
    do {                             // TODO: not use shortest wires first
        if (curr == -1) {
            curr = startDir*32;
            dir = curr / 32;
            path = 0;
        }
        else {
            ++curr;
            dir = curr / 32;
            path = curr % 32;
        }

        if (dir - startDir == 8) {
            return -1;
        }

    } while (((jump?type->dst_src[pos]:type->local_src[pos]).dirs[dir%8] & (1<<path)) != 0
             && ((src.dirs[dir%8] & (1<<path)) != 0 || (src_deadend.dirs[dir%8] & (1<<path)) != 0));

    // try joints?

    return dir*32 + path;
}

bool CBState::leaseOut(int pos, int curr, int orig_curr, int joint)
{
    int dir = curr / 32;
    int path = curr % 32;

    u256 prev_local = local.local;
    u256 prev_src = src.jump;

    if ((src_deadend.jump & (u256{0,1}<<curr)) != u256{}) {
        return false;
    }
    local.local |= u256{0,1}<<pos;
    src.dirs[search_dirs[orig_curr/32][dir%8]] |= 1<<path;

    if (local.local == prev_local || src.jump == prev_src) {  // already busy
        local.local = prev_local;
        src.jump = prev_src;
        return false;
    }
    return true;
}

bool CBState::leaseJump(int pos, int curr, int orig_curr, int joint)
{
    int dir = curr / 32;
    int path = curr % 32;

    u256 prev_dst = dst.jump;
    u256 prev_src = src.jump;

    if ((src_deadend.jump & (u256{0,1}<<curr)) != u256{}) {
        return false;
    }
    dst.jump |= u256{0,1}<<pos;
    src.dirs[search_dirs[orig_curr/32][dir%8]] |= 1<<path;

    if (dst.jump == prev_dst || src.jump == prev_src) {  // already busy
        dst.jump = prev_dst;
        src.jump = prev_src;
        return false;
    }
    return true;
}

bool CBState::leaseIn(int pos, int curr, int joint)
{
    int dir = pos / 32;
    int path = pos % 32;

    u256 prev_dst = dst.jump;
    u256 prev_local = local.local;
    u256 prev_joint = this->joint.jump;

    dst.dirs[dir] |= 1<<path;
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
    int dir = curr / 32;
    int path = curr % 32;
    int step = std::max(1, path / 4 / 2);
    switch (search_dirs[orig_curr/32][dir%8])
    {
        case 0: return src + Coord{0, -step};
        case 1: return src + Coord{step, -step};
        case 2: return src + Coord{step, 0};
        case 3: return src + Coord{step, step};
        case 4: return src + Coord{0, step};
        case 5: return src + Coord{-step, step};
        case 6: return src + Coord{-step, 0};
        case 7: return src + Coord{-step, -step};
    }
    return Coord{-1,-1};
}
