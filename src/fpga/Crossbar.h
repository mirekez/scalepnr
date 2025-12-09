#pragma once
// This is how I see Crossbar, nodes on one side, nodes on other side and intermediate flexible joint nodes.
// Possible connection pairs are read from database for each type of tile
//
//                                                 CHAIN NODES  NODES FROM/TO
//                                                    │││││     UPPER TILE
//                                                    │││││      ││
//       NODES FROM/TO LEFT TILE              ┌───────┼┼┼┼┼──────┼┼────────────────┐      NODES FROM/TO
//    ──────────────────────────────┐         │    ┌──┼┼┼┼┼──────┼┼────────────────┼──────RIGHT TILE
//    ──────────────────────────────┼─────────┼────┼──┼┼┼┼┼──┐   ││    ┌───────────┼──────
//                                  │    ┌────┼────┼──┼┼┼┼┼──┼───┘│    │    ┌─────┐│
//                                  │    │    │    │  │││││  │    │    │    │     ││
//                  ┌───────────────┼────┼────┼────┼──▲▲▲▲▲──▲────▲────▲────▲──┐  ││
//                  │             ┌─▼─┐┌─▼─┐┌─▼─┐┌─▼─┐│││││┌─┴─┐┌─┴─┐┌─┴─┐┌─┴─┐│  ││
//                  │             │MUX││MUX││MUX││MUX│││││││MUX││MUX││MUX││MUX││  ││ NODES
//                  │             └┬┬┬┘└┬┬┬┘└┬┬┬┘└┬┬─┘│││││└▲▲▲┘└─▲▲┘└▲▲▲┘└▲▲▲┘│  ││ FROM/TO
//                  │              │││  │││  │││  │└──┤│││└─┼┼┼─┐ ││  │││┌─┘││ │  ││ BOTTOM
//                  │              │││  │││  ││└──┼───┼┘│└──┼┼┼┐│ ││  ││││┌─┘│ │  ││ TILE
//                  │ Tile         │││  │││  │└───┼───┼─┼───┘││││ ││  ││└┼┼─┐│ │  ││
//                  │              │││  │└┼──┼┬───┼───┼─┼────┘│││ ││  ││ ││ ││ │  ││
//                  │              │└┼──┼─┼─┐││ ┌─┼───┼─┴─────┼┼┼─┼┴──┼┼─┼┼┐││ │  ││
//                  │ Crossbar     │ │  │ │ │││ │ │ ┌─┼──┬──┐ │││ │ ┌─┘│ │││││ │  ││
//                  │              │ └──┼┐│┌▼▼▼─▼─▼─▼┐│  │ ┌┴─┴┴┴─┴─┴┐ │ │││││ │
//                  │              │┌───┘│││MUX JOINT││  │ │MUX JOINT│ │ │││││ │
//                  │              ││    ││└┬┬┬┬┬┬┬─┬┘│  │ └▲▲▲──▲▲─▲┘ │ │││││ │
//                  │              ││┌───┼┼─┘││││││┌┼─┴─┐│  │││  ││┌┼──┼─┘││││ │
//                  │              │││  ┌┼┼──┘││││││└───┼┼──┘││  │││└─┐│  ││││ │
//                  │              │││  │││   ││││└┼────┼┼┬──┘│  │││  ││  ││││ │
//                  │              │││  ││├──┐│││└─┼───┐│││   │  │││  ││┌─┘│││ │
//                  │              │││  │││  │││└──┼──┐││││   │  │││  │││  │││ │
//                  │              │││  │││  ││└──┐│┌─┼┼┼┼┼──┐│  │││  │││  │││ │
//                  │              │││  │││  ││┌──┼┼┼─┼┼┼┼┼─┐││  │││  │││  │││ │
//                  │             ┌▼▼▼┐┌▼▼▼┐┌▼▼▼┐┌▼▼▼┐│││││┌┴┴┴┐┌┴┴┴┐┌┴┴┴┐┌┴┴┴┐│
//                  │             │MUX││MUX││MUX││MUX│││││││MUX││MUX││MUX││MUX││
//                  │             └─┬─┘└─┬─┘└─┬─┘└─┬─┘│││││└─▲─┘└─▲─┘└─▲─┘└─▲─┘│
//                  └───────────────▼────▼────▼────▼-─▼▼▼▼▼──┼────┼────┼────┼──┘
//
//                                   NODES TO LOGIC  CTRL NODES  NODES FROM LOGIC
//
//                                                  ELEMENTS

#include <string>
#include <stdint.h>
#include <map>

#include "DeviceFormat.h"
#include "debug.h"
#include "u256.h"

#define CB_MAX_NODES 256

namespace fpga {

struct CBJumpNode  // this is a generic jump in mesh
{
    union {
        struct {
            uint8_t num:2;
            uint8_t length:3;
            uint8_t dir:3;  // angle from y axis
        };
        uint8_t jump;
    }__attribute__((packed));
}__attribute__((packed));

struct CBLocalNode  // this is a generic local node to a Tile
{
    uint8_t local;
}__attribute__((packed));

struct CBJointNode  // this is a joint node inside CB
{
    uint8_t joint;
}__attribute__((packed));

struct CBJumpState
{  // [dir][length*4 + num]
    union {
        uint32_t dirs[8];
        u256 jump;
    }__attribute__((packed));
}__attribute__((packed));

struct CBLocalState
{
    u256 local;
};

struct CBJointState
{
    u256 joint;
};

struct CBType
{
    std::string name;
    CBJumpState local_src[CB_MAX_NODES];
    CBJointState local_joint[CB_MAX_NODES];
    CBJointState src_joint[CB_MAX_NODES];
    CBJumpState src_dst[CB_MAX_NODES];
    CBJumpState joint_src[CB_MAX_NODES];
    CBLocalState joint_local[CB_MAX_NODES];
    CBJumpState dst_src[CB_MAX_NODES];
    CBLocalState dst_local[CB_MAX_NODES];
    CBJointState dst_joint[CB_MAX_NODES];

    int /*0-2*/ parseNode(std::string name, CBLocalNode& local_node, CBJumpNode& src_node, CBJumpNode& dst_node, CBJointNode& joint_node,
                                           CBLocalState& local_state, CBJumpState& src_state, CBJumpState& dst_state, CBJointState& joint_state)
    {
        
    }

    void loadFromSpec(const CBTypeSpec& spec)
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
            CBJumpNode a_src_node = {}, b_src_node = {};
            CBJumpNode a_dst_node = {}, b_dst_node = {};
            CBLocalNode a_local_node = {}, b_local_node = {};
            CBJointNode a_joint_node = {}, b_joint_node = {};
            CBJumpState a_src_state = {}, b_src_state = {};
            CBJumpState a_dst_state = {}, b_dst_state = {};
            CBLocalState a_local_state = {}, b_local_state = {};
            CBJointState a_joint_state = {}, b_joint_state = {};

            int type_a = parseNode(pair.first, a_local_node, a_src_node, a_dst_node, a_joint_node, a_local_state, a_src_state, a_dst_state, a_joint_state);
            int type_b = parseNode(pair.second, b_local_node, b_src_node, b_dst_node, b_joint_node, b_local_state, b_src_state, b_dst_state, b_joint_state);

            if (type_a == 0) {  // local
                if (type_b == 0) {  // local
                    PNR_ASSERT(0, "wire from local to local\n");
                }
                if (type_b == 1) {  // src
                    local_src[a_local_node.local].jump |= b_src_state.jump;
                }
                if (type_b == 2) {  // dst
                    PNR_ASSERT(0, "wire from local to dst\n");
                }
                if (type_b == 3) {  // joint
                    local_joint[a_local_node.local].joint |= b_joint_state.joint;
                }
            }
            if (type_a == 1) {  // src
                if (type_b == 0) {  // local
                    PNR_ASSERT(0, "wire from src to local\n");
                }
                if (type_b == 1) {  // src
                    PNR_ASSERT(0, "wire from src to src\n");
                }
                if (type_b == 2) {  // dst
                    src_dst[a_src_node.jump].jump |= b_dst_state.jump;
                }
                if (type_b == 3) {  // joint
                    PNR_ASSERT(0, "wire from src to joint\n");
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
                    PNR_ASSERT(0, "wire from dst to dst\n");
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
                    PNR_ASSERT(0, "wire from joint to dst\n");
                }
                if (type_b == 3) {  // joint
                    PNR_ASSERT(0, "wire from joint to joint\n");
                }
            }
        }

    }

    bool canOut(int local, int src, int& joint)
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

    bool canJump(int dst, int src, int& joint)
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

    bool canIn(int dst, int local, int& joint)
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
};

struct CBState
{
    CBJumpState src;
    CBLocalState local;
    CBJumpState joint;
    CBJumpState dst;
    CBType* type;

    static constexpr const int dirs[8] = {0, 1, 7, 2, 6, 3, 5, 4};  // these are reordered directions for better search like 0 -1 +1 -2 +2 ...

    int iterateOut(int pos, const Coord& from, const Coord& to, int curr = 0)
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

    void leaseOut(int pos, int curr = 0)
    {
        int dir = curr / 32;
        int path = curr % 32;
        PNR_ASSERT((local.local & (u256{0,1}<<pos)) != u256{} && (src.dirs[dir] & (1<<path)) != 0,
            "local pos {} in {} or src pos {} in {} is already busy\n", std::to_string(pos), local.local.str(), std::to_string(curr), src.jump.str());

        local.local &= ~(u256{0,1}<<pos);
        src.dirs[curr/32] &= ~(1<<path);
    }

    bool tryIn(int dst, int local)
    {
        return (type->dst_local[dst].local & (u256{0,1}<<local)) != u256{};
    }

    Coord makeJump(const Coord& src, int curr)
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

};

}
