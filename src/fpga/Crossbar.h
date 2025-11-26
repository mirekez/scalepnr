#pragma once
// This is how I see Crossbar, pins on one side, pins on other side and intermediate flexible joint pins.
// Possible connection pairs are read from database for each type of tile
//
//                                                  CHAIN PINS  PINS FROM/TO
//                                                    │││││     UPPER TILE
//                                                    │││││      ││
//       PINS FROM/TO LEFT TILE               ┌───────┼┼┼┼┼──────┼┼────────────────┐      PINS FROM/TO
//    ──────────────────────────────┐         │    ┌──┼┼┼┼┼──────┼┼────────────────┼──────RIGHT TILE
//    ──────────────────────────────┼─────────┼────┼──┼┼┼┼┼──┐   ││    ┌───────────┼──────
//                                  │    ┌────┼────┼──┼┼┼┼┼──┼───┘│    │    ┌─────┐│
//                                  │    │    │    │  │││││  │    │    │    │     ││
//                  ┌───────────────┼────┼────┼────┼──▲▲▲▲▲──▲────▲────▲────▲──┐  ││
//                  │             ┌─▼─┐┌─▼─┐┌─▼─┐┌─▼─┐│││││┌─┴─┐┌─┴─┐┌─┴─┐┌─┴─┐│  ││
//                  │             │MUX││MUX││MUX││MUX│││││││MUX││MUX││MUX││MUX││  ││ PINS
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
//                                   PINS TO LOGIC  CTRL PINS  PINS FROM LOGIC
//
//                                                  ELEMENTS

#include <string>
#include <stdint.h>
#include <map>

#include "DeviceFormat.h"
#include "debug.h"
#include "u256.h"

#define CB_MAX_PINS 256

namespace fpga {

struct CBJumpPin  // this is a generic jump in mesh
{
    uint8_t num:2;
    uint8_t length:3;
    uint8_t dir:3;  // angle from y axis
}__attribute__((packed));

struct CBLocalPin  // this is a generic local pin to a Tile
{
    uint8_t local;
}__attribute__((packed));

struct CBJointPin  // this is a joint pin inside CB
{
    uint8_t joint;
}__attribute__((packed));

struct CBJumpState
{  // [dir][length*4 + num]
    uint32_t dirs[8];
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
    CBJumpState local_src[CB_MAX_PINS];
    CBJointState local_joint[CB_MAX_PINS];
    CBJointState src_joint[CB_MAX_PINS];
    CBJumpState joint_src[CB_MAX_PINS];
    CBJumpState joint_local[CB_MAX_PINS];
    CBJumpState dst_src[CB_MAX_PINS];
    CBLocalState dst_local[CB_MAX_PINS];
    CBJointState dst_joint[CB_MAX_PINS];

    int /*0-2*/ parsePin(std::string name, CBLocalPin& local_pin, CBJumpPin& src_pin, CBJumpPin& dst_pin, CBJointPin& joint_pin,
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
        for (const auto& pair : spec.Pins) {
            CBJumpPin a_src_pin = {}, b_src_pin = {};
            CBJumpPin a_dst_pin = {}, b_dst_pin = {};
            CBLocalPin a_local_pin = {}, b_local_pin = {};
            CBJointPin a_joint_pin = {}, b_joint_pin = {};
            CBJumpState a_src_state = {}, b_src_state = {};
            CBJumpState a_dst_state = {}, b_dst_state = {};
            CBLocalPin a_local_state = {}, b_local_state = {};
            CBJointPin a_joint_state = {}, b_joint_state = {};

            int type_a = parsePin(pair.first, a_local_pin, a_src_pin, a_dst_pin, a_joint_pin, a_local_state, a_src_state, a_dst_state, a_joint_state);
            int type_b = parsePin(pair.second, b_local_pin, b_src_pin, b_dst_pin, b_joint_pin, b_local_state, b_src_state, b_dst_state, b_joint_state);

            if (type_a == 0) {  // local
                if (type_b == 0) {  // local
                    PNR_ASSERT(0, "wire has same src and dst\n");
                }
                if (type_b == 1) {  // src
                    local_src[a_local_pin] |= b_src_state;
                }
                if (type_b == 2) {  // dst
                    local_dst[a_local_pin] |= b_dst_state;
                }
                if (type_b == 3) {  // joint
                    local_joint[a_local_pin] |= b_joint_state;
                }
            }
            if (type_a == 1) {  // src
                if (type_b == 0) {  // local
                    src_local[a_src_pin] |= b_local_state;
                }
                if (type_b == 1) {  // src
                    PNR_ASSERT(0, "wire has same src and dst\n");
                }
                if (type_b == 2) {  // dst
                    src_dst[a_src_pin] |= b_dst_state;
                }
                if (type_b == 3) {  // joint
                    src_joint[a_src_pin] |= b_joint_state;
                }
            }
            if (type_a == 2) {  // dst
                if (type_b == 0) {  // local
                    dst_local[a_dst_pin] |= b_local_state;
                }
                if (type_b == 1) {  // src
                    dst_src[a_dst_pin] |= b_src_state;
                }
                if (type_b == 2) {  // dst
                    PNR_ASSERT(0, "wire has same src and dst\n");
                }
                if (type_b == 3) {  // joint
                    dst_joint[a_dst_pin] |= b_joint_state;
                }
            }
            if (type_a == 3) {  // joint
                if (type_b == 0) {  // local
                    joint_local[a_joint_pin] |= b_local_state;
                }
                if (type_b == 1) {  // src
                    joint_src[a_joint_pin] |= b_src_state;
                }
                if (type_b == 2) {  // dst
                    joint_dst[a_joint_pin] |= b_dst_state;
                }
                if (type_b == 3) {  // joint
                    PNR_ASSERT(0, "wire has same src and dst\n");
                }
            }
        }

    }

    bool canOut(int local, int src, int& joint)
    {
        joint = -1;
        if ((*(u256*)&local_src[local]&((u256)1<<src))) {  // direct path
            return true;
        }
        // trying joint
        u256 joints_from_local = local_joint[local].joint;
        u256 joints_from_src = src_joint[src].joint;
        u256 intersect = joints_from_local&joints_from_src;
        if ((joint = intersect.first_set()) != -1) {
            return true;
        }
        return false;
    }

    bool canJump(int dst, int src, int& joint)
    {
        joint = -1;
        if ((*(u256*)&dst_src[dst]&((u256)1<<src))) {  // direct path
            return true;
        }
        // trying joint
        u256 joints_from_dst = dst_joint[dst];
        u256 joints_from_src = src_joint[src];
        u256 intersect = joints_from_dst&joints_from_src;
        if ((joint = intersect.first_set()) != -1) {
            return true;
        }
        return false;
    }

    bool canIn(int dst, int local, int& joint)
    {
        joint = -1;
        if ((dst_local[dst].local&((u256)1<<local))) {  // direct path
            return true;
        }
        // trying joint
        u256 joints_from_dst = *(u256*)&local_enter[local];
        u256 joints_from_local = *(u256*)&local_enter[dst];
        u256 intersect = joints_from_dst&joints_from_local;
        if ((joint = intersect.first_set()) != -1) {
            return true;
        }
        return false;
    }

};

struct CBState
{
    CBJumpState local_src;
    CBJointState local_joint;
    CBJointState src_joint;
    CBJumpState joint_src;
    CBJumpState joint_local;
    CBJumpState dst_src;
    CBLocalState dst_local;
    CBJointState dst_joint;
    CBType* type;

    static constexpr const int dirs[8] = {0, 1, 7, 2, 6, 3, 5, 4};

    int iterateOut(int pos, const Coord& src, const Coord& to, int curr = 0)
    {
        int startDir = -1;
        Coord diff = to - src;
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
        int step = -1;
        do {                             // TODO: not use shortest lines first
            if (curr == -1) {
                curr = startDir*16;
                dir = curr / 16;
                step = 0;
            }
            else {
                ++curr;
                dir = curr / 16;
                step = curr % 16;
            }

            if (dir - startDir == 8) {
                return -1;
            }

        } while ((local_src.src[dirs[dir%8]] & (1<<step)));

        return dir*16 + step;
    }

    Coord makeJump(const Coord& src, int curr)
    {
        int dir = curr / 16;
        int step = curr % 16;
        const int steps[4] = {1, 2, 4, 6};
        switch (dirs[dir])
        {
            case 0: return src + Coord{0, steps[step/4]};
            case 1: return src + Coord{steps[step/4], steps[step/4]};
            case 2: return src + Coord{steps[step/4], 0};
            case 3: return src + Coord{steps[step/4], -steps[step/4]};
            case 4: return src + Coord{0, -steps[step/4]};
            case 5: return src + Coord{-steps[step/4], -steps[step/4]};
            case 6: return src + Coord{-steps[step/4], 0};
            case 7: return src + Coord{-steps[step/4], steps[step/4]};
        }
        return Coord{-1,-1};
    }

};

}
