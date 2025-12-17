#include "Crossbar.h"

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
                first_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
                base = std::string(ptr, i);
            }
            if (nums == 2) {
//                second_id = atoi(ptr + i);
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
    }

    if (nums == 1) {  // !jump
        auto it = nodes_enum.find(base);
        if (it == nodes_enum.end()) {
            nodes_enum.emplace(base, NodeEnum{first_id, 1, 0});
        }
        else {
            if (first_id - it->second.base_id > it->second.cnt) {
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
                first_id = atoi(ptr + i);
                while (ptr[i] >= '0' && ptr[i] <= '9' && ptr[i] != 0) {
                    ++i;
                }
                base = std::string(ptr, i);
            }
            if (nums == 2) {
                second_id = atoi(ptr + i);
            }
        }
    }

    if (nums == 0) {
        nums = 1;
        first_id = 0;
    }

    if (nums == 1) {
        auto it = nodes_enum.find(base);
        if (it == nodes_enum.end()) {
            return -1;
        }
        if (name.find("JOINT") != (size_t)-1) {  // joint
            joint_node.joint = it->second.start_num + first_id;
            joint_state.joint = u256(1) << (joint_node.joint);
            PNR_LOG2("CBAR", "for name '{}' found joint num {} with base {}", name, it->second.start_num + first_id, it->second.start_num);
            return 3;
        }
        else {  // local
            local_node.local = it->second.start_num + first_id;
            local_state.local = u256(1) << (local_node.local);
            PNR_LOG2("CBAR", "for name '{}' found local num {} with base {}", name, it->second.start_num + first_id, it->second.start_num);
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

        if (type_a == 0) {  // local
            if (type_b == 0) {  // local
                local_local[a_local_node.local].local |= b_local_state.local;
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
                joint_joint[a_joint_node.joint].joint |= b_joint_state.joint;
            }
        }
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
    PNR_LOG3("CBAR", "canIn, dst: {}, local: {}, dst_local[dst]: {}, dst_joint[dst]: {}, local_joint[local]: {},  intersect: {}",
        dst, local, dst_local[dst].local.str(), dst_joint[dst].joint.str(), local_joint[local].joint.str(), (dst_joint[dst].joint&local_joint[local].joint).str());
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
    return dst_to_joints.for_each_set_bit( [&](int index) {
            if ((joint = (local_to_joints&joint_joint[index].joint).ffs256()) != -1) {
                PNR_LOG3("CBAR", "canIn, found double joint {} for dst_to_joints {} and joint_joint[index] {} and local_to_joints {}", 
                    joint, dst_to_joints.str(), joint_joint[index].joint.str(), local_to_joints.str());
                return true;
            }
            return true;
        }
    );
}

int CBState::iterate(bool jump, int pos, const Coord& from, const Coord& to, int curr)
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

    } while (((jump?type->dst_src[pos]:type->local_src[pos]).dirs[dir%8] & (1<<path)) != 0
             && (src.dirs[dir%8] & (1<<path)) != 0);

    // try joints?

    return dir*32 + path;
}

bool CBState::leaseOut(int pos, int curr, int orig_curr, int joint)
{
    int dir = curr / 32;
    int path = curr % 32;

    u256 prev_local = local.local;
    u256 prev_src = src.jump;

    local.local |= u256{0,1}<<pos;
    src.dirs[search_dirs[orig_curr/32][dir%8]] |= 1<<path;

    if (local.local == prev_local || src.jump == prev_src) {  // already busy
        local.local = prev_local;
        src.jump = prev_src;
        return true;
    }
    return true;
}

bool CBState::leaseJump(int pos, int curr, int orig_curr, int joint)
{
    int dir = curr / 32;
    int path = curr % 32;

    u256 prev_dst = dst.jump;
    u256 prev_src = src.jump;

    dst.jump |= u256{0,1}<<pos;
    src.dirs[search_dirs[orig_curr/32][dir%8]] |= 1<<path;

    if (dst.jump == prev_dst || src.jump == prev_src) {  // already busy
        dst.jump = prev_dst;
        src.jump = prev_src;
        return true;
    }
    return true;
}

bool CBState::leaseIn(int pos, int curr, int joint)
{
    int dir = pos / 32;
    int path = pos % 32;

    u256 prev_dst = dst.jump;
    u256 prev_local = local.local;

    dst.dirs[dir] |= 1<<path;
    local.local |= u256{0,1}<<curr;

    if (dst.jump == prev_dst || local.local == prev_local) {  // already busy
        dst.jump = prev_dst;
        local.local = prev_local;
        return true;
    }
    return false;
}

Coord CBState::makeJump(const Coord& src, int curr, int orig_curr)
{
    int dir = curr / 32;
    int path = curr % 32;
    int step = path / 4/2;
    switch (search_dirs[orig_curr/32][dir%8])
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

