#include "Crossbar.h"

void CBType::preParseNode(std::string name, bool finish)
{
    if (finish) {  // enum nodes
        int start = 0;
        for (auto& pair : modes_enum) {
            pair.second.start_num = start;
            PNR_LOG2("CRBR", "giving '{}' group numbers {}-{}", pair.first, pair.second.start_num, pair.second.start_num + pair.second.cnt-1);
            PNR_ASSERT(pair.second.start_num + pair.second.cnt-1 < 256, "nodes enum overflows 256 for nodes of type '{}'", pair.first);
            start += pair.second.cnt;
        }
        return;
    }

    std::string base;
    int first_id = -1;
//        int second_id = -1;
    int nums = 0;
    const char* ptr = name.c_str();
    for (size_t i=0; i < strlen(ptr); ++i) {
        if (ptr[i] >= '0' && ptr[i] < 9) {
            ++nums;
            if (nums == 1) {
                first_id = atoi(ptr + i);
                base = std::string(ptr, i);
            }
//                if (nums == 2) {
//                    second_id = atoi(ptr + i);
//                }
            while (ptr[i] >= '0' && ptr[i] < 9 && ptr[i] != 0) {
                ++ptr;
            }
        }
    }

    auto it = modes_enum.find(base);
    if (it == modes_enum.end()) {
        modes_enum.emplace(base, NodeEnum{first_id, 1, 0});
    }
    else {
        if (first_id - it->second.base_id > it->second.cnt) {
            it->second.cnt = first_id - it->second.base_id + 1;
        }
    }

}

int /*0-3*/ CBType::parseNode(std::string name, TechMap& map,
                     CBLocalNode& local_node, CBJumpNode& src_node, CBJumpNode& dst_node, CBJointNode& joint_node,
                     CBLocalState& local_state, CBJumpState& src_state, CBJumpState& dst_state, CBJointState& joint_state)
{
    std::string base;
    int first_id = -1;
    int second_id = -1;
    int nums = 0;
    const char* ptr = name.c_str();
    for (size_t i=0; i < strlen(ptr); ++i) {
        if (ptr[i] >= '0' && ptr[i] < '9') {
            ++nums;
            if (nums == 1) {
                first_id = atoi(ptr + i);
                base = std::string(ptr, i);
            }
            if (nums == 2) {
                second_id = atoi(ptr + i);
            }
            while (ptr[i] >= '0' && ptr[i] < '9' && ptr[i] != 0) {
                ++ptr;
            }
        }
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
                        PNR_LOG2("CRBR", "replacing {} with {}", orig_name, name);
                    }
                }
            }
        }
    }

    if (second_id == -1) {  // one number in name
        auto it = modes_enum.find(base);
        if (it == modes_enum.end()) {
            return -1;
        }

        if (name.find("JOINT") != (size_t)-1) {  // joint
            joint_node.joint = it->second.start_num + first_id;
            joint_state.joint = u256(1) << (joint_node.joint);
            PNR_LOG2("CRBR", "for name '{}' found joint num {} with base {}", name, it->second.start_num + first_id, it->second.start_num);
            return 3;
        }
        else {  // local
            local_node.local = it->second.start_num + first_id;
            local_state.local = u256(1) << (local_node.local);
            PNR_LOG2("CRBR", "for name '{}' found local num {} with base {}", name, it->second.start_num + first_id, it->second.start_num);
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
                            node.dir = atoi(expr[0][0][0].c_str());
                            CBJumpState state = {};
                            state.dirs[8] = 1 << (node.length*4 + node.num);
                            if (name.find("SRC") != (size_t)-1) {
                                src_node = node;
                                src_state = state;
                                PNR_LOG2("CRBR", "for name '{}' found rule '{}', it's src jump {} {} {}", name, expr[0][0][0], (uint8_t)node.num, (uint8_t)node.length, (uint8_t)node.dir);
                                return 1;
                            }
                            if (name.find("DST") != (size_t)-1) {
                                dst_node = node;
                                dst_state = state;
                                PNR_LOG2("CRBR", "for name '{}' found rule '{}', it's dst jump {} {} {}", name, expr[0][0][0], (uint8_t)node.num, (uint8_t)node.length, (uint8_t)node.dir);
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
    memset(local_src, 0, sizeof(local_src));
    memset(local_joint, 0, sizeof(local_joint));
    memset(src_joint, 0, sizeof(src_joint));
    memset(joint_src, 0, sizeof(joint_src));
    memset(joint_local, 0, sizeof(joint_local));
    memset(dst_src, 0, sizeof(dst_src));
    memset(dst_local, 0, sizeof(dst_local));
    memset(dst_joint, 0, sizeof(dst_joint));
    for (const auto& pair : spec.nodes) {
        preParseNode(pair.first, false);
        preParseNode(pair.second, false);
    }
    preParseNode("", true);
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

        if (type_a == 0) {  // local
            if (type_b == 0) {  // local
                PNR_ASSERT(0, "wire from local to local: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 1) {  // src
                local_src[a_local_node.local].jump |= b_src_state.jump;
            }
            if (type_b == 2) {  // dst
                PNR_ASSERT(0, "wire from local to dst {}\n", pair.first, pair.second);
            }
            if (type_b == 3) {  // joint
                local_joint[a_local_node.local].joint |= b_joint_state.joint;
            }
        }
        if (type_a == 1) {  // src
            if (type_b == 0) {  // local
                PNR_ASSERT(0, "wire from src to local: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 1) {  // src
                PNR_ASSERT(0, "wire from src to src: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 2) {  // dst
                src_dst[a_src_node.jump].jump |= b_dst_state.jump;
            }
            if (type_b == 3) {  // joint
                PNR_ASSERT(0, "wire from src to joint: {} - {}\n", pair.first, pair.second);
            }
        }
        if (type_a == 2) {  // dst
            if (type_b == 0) {  // local
                dst_local[a_dst_node.jump].local |= b_local_state.local;
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
            }
            if (type_b == 1) {  // src
                joint_src[a_joint_node.joint].jump |= b_src_state.jump;
            }
            if (type_b == 2) {  // dst
                PNR_ASSERT(0, "wire from joint to dst: {} - {}\n", pair.first, pair.second);
            }
            if (type_b == 3) {  // joint
                PNR_ASSERT(0, "wire from joint to joint: {} - {}\n", pair.first, pair.second);
            }
        }
    }
}

bool CBType::canOut(int local, int src, int& joint)
{
    joint = -1;
    if ((local_src[local].jump&(u256{0,1}<<src)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 local_to_joints = local_joint[local].joint;
    u256 joints_to_src = src_joint[src].joint;
    u256 intersect = local_to_joints&joints_to_src;
    if ((joint = intersect.ffs256()) != -1) {
        return true;
    }
    return false;
}

bool CBType::canJump(int dst, int src, int& joint)
{
    joint = -1;
    if ((dst_src[dst].jump&(u256{0,1}<<src)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 dst_to_joints = dst_joint[dst].joint;
    u256 joints_to_src = src_joint[src].joint;
    u256 intersect = dst_to_joints&joints_to_src;
    if ((joint = intersect.ffs256()) != -1) {
        return true;
    }
    return false;
}

bool CBType::canIn(int dst, int local, int& joint)
{
    joint = -1;
    if ((dst_local[dst].local&(u256{0,1}<<local)) != u256{}) {  // direct path
        return true;
    }
    // trying joint
    u256 dst_to_joints = dst_joint[dst].joint;
    u256 local_to_joints = local_joint[local].joint;
    u256 intersect = dst_to_joints&local_to_joints;
    if ((joint = intersect.ffs256()) != -1) {
        return true;
    }
    return false;
}

int CBState::iterateOut(int pos, const Coord& from, const Coord& to, int curr)
{
    int startDir = -1;
    Coord diff = to - from;
    if (diff.x >= 0 && diff.y >= 0) {
        if (diff.x > diff.y*3) {
            startDir = 2;
        } else
        if (diff.y > diff.x*3) {
            startDir = 0;
        } else {
            startDir = 1;
        }
    } else
    if (diff.x >= 0 && diff.y < 0) {
        if (diff.x > -diff.y*3) {
            startDir = 2;
        } else
        if (-diff.y > diff.x*3) {
            startDir = 4;
        } else {
            startDir = 3;
        }
    } else
    if (diff.x < 0 && diff.y < 0) {
        if (-diff.x > -diff.y*3) {
            startDir = 6;
        } else
        if (-diff.y > -diff.x*3) {
            startDir = 4;
        } else {
            startDir = 5;
        }
    } else
    if (diff.x < 0 && diff.y >= 0) {
        if (-diff.x > diff.y*3) {
            startDir = 6;
        } else
        if (diff.y > -diff.x*3) {
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

    } while ((type->local_src[pos].dirs[dir%8] & (1<<path)) != 0 && (src.dirs[dir%8] & (1<<path)) != 0);

    // try joints?

    return dir*32 + path;
}

void CBState::leaseOut(int pos, int curr)
{
    int dir = curr / 32;
    int path = curr % 32;
    PNR_ASSERT((local.local & (u256{0,1}<<pos)) != u256{} && (src.dirs[dir] & (1<<path)) != 0,
        "local pos {} in {} or src pos {} in {} is already busy\n", std::to_string(pos), local.local.str(), std::to_string(curr), src.jump.str());

    local.local &= ~(u256{0,1}<<pos);
    src.dirs[curr/32] &= ~(1<<path);
}

bool CBState::tryIn(int dst, int local)
{
    return (type->dst_local[dst].local & (u256{0,1}<<local)) != u256{};
}

Coord CBState::makeJump(const Coord& src, int curr)
{
    int dir = curr / 32;
    int path = curr % 32;
    int step = path / 4;
    switch (dirs[dir])
    {
        case 0: return src + Coord{0, step};
        case 1: return src + Coord{step, step};
        case 2: return src + Coord{step, 0};
        case 3: return src + Coord{step, -step};
        case 4: return src + Coord{0, -step};
        case 5: return src + Coord{-step, -step};
        case 6: return src + Coord{-step, 0};
        case 7: return src + Coord{-step, step};
    }
    return Coord{-1,-1};
}

