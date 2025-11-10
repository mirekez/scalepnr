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

struct CBMuxPin  // this is a mux pin inside CB
{
    uint8_t mux;
}__attribute__((packed));

struct CBJumpState
{  // [dir][length*4 + num]
    uint32_t dirs[8];
}__attribute__((packed));

struct CBLocalState
{
    u256 local;
};

struct CBMuxState
{
    u256 mux;
};

struct CBType
{
    std::string name;
    CBJumpState local[CB_MAX_PINS];
    CBMuxState local_mux[CB_MAX_PINS];
    CBJumpState mux_src[CB_MAX_PINS];
    CBJumpState mux_local[CB_MAX_PINS];
    CBJumpState dst_src[CB_MAX_PINS];
    CBLocalState dst_local[CB_MAX_PINS];
    CBMuxState dst_mux[CB_MAX_PINS];

    int /*0-2*/ parsePin(std::string name, CBJumpPin& int_as_src, CBLocalPin& local_as_src, CBMuxPin& mux_as_src, CBJumpState int_as_dst, CBLocalState local_as_dst, CBMuxState mux_as_dst)
    {
    }
/*
    const int fanmux_from = 0;
    const int fanmux_cnt = 8;
    const int fanmux_src = bypl_src + bypl_cnt;

*/
    void loadFromSpec(const CBTypeSpec& spec)
    {
        memset(cb_exit, 0, sizeof(cb_exit));
        memset(cb_long, 0, sizeof(cb_long));
        memset(local_exit, 0, sizeof(local_exit));
        memset(cb_enter, 0, sizeof(cb_enter));
        memset(local_enter, 0, sizeof(local_enter));
        for (const auto& pair : spec.Pins) {
            CBJumpPin a_src_int = {}, b_src_int = {};
            CBLocalPin a_src_local = {}, b_src_local = {};
            CBJumpState a_exit = {}, b_exit = {};
            CBJumpState a_enter = {}, b_enter = {};
            CBJointState a_bounce = {}, b_bounce = {};

            bool a_external = parsePin(pair.first, a_src_int, a_src_local, a_exit, a_long, a_enter, a_bounce);
            bool b_external = parsePin(pair.second, b_src_int, b_src_local, b_exit, b_long, b_enter, b_bounce);

            if (a_external && b_external) {
                *(__uint128_t*)&cb_exit[*(uint16_t*)&a_src_int] |= *(__uint128_t*)&b_exit;
                *(__uint128_t*)&cb_long[*(uint16_t*)&a_src_int] |= *(__uint128_t*)&b_long;
            }
            if (!a_external && b_external) {
                *(__uint128_t*)&local_exit[*(uint16_t*)&a_src_local] |= *(__uint128_t*)&b_exit;
            }
            if (a_external && !b_external) {
                *(__uint128_t*)&cb_enter[*(uint16_t*)&a_src_int] |= *(__uint128_t*)&b_enter;
            }
            if (!a_external && !b_external) {
                *(__uint128_t*)&local_enter[*(uint16_t*)&a_src_local] |= *(__uint128_t*)&b_bounce;
            }
        }

    }

    int find_first_set_bit_128(__uint128_t num)
    {
        unsigned long long lower_half = (unsigned long long)num;
        unsigned long long upper_half = (unsigned long long)(num >> 64);

        if (lower_half != 0) {
            return __builtin_ctzll(lower_half);
        } else if (upper_half != 0) {
            return 64 + __builtin_ctzll(upper_half);
        } else {
            return -1;
        }
    }

    bool canOut(int in, int out, int& byp)
    {
        byp = -1;
        if ((*(__uint128_t*)&local_exit[in]&((__uint128_t)1<<out))) {
            return true;
        }
        __uint128_t byps_from_out = *(__uint128_t*)&local_enter[out];  // input and output numbers should be in same line (not byp)
        __uint128_t byps_from_in = *(__uint128_t*)&local_enter[in];
        __uint128_t intersect = byps_from_out&byps_from_in;
        if ((byp = find_first_set_bit_128(intersect)) != -1) {
            return true;
        }
        return false;
    }

    bool canJump(int in, int out, int& byp)
    {
        byp = -1;
        if ((*(__uint128_t*)&cb_exit[in]&((__uint128_t)1<<out))) {
            return true;
        }
        __uint128_t byps_from_out = *(__uint128_t*)&local_enter[out];  // input and output numbers should be in same line (not byp)
        __uint128_t byps_from_in = *(__uint128_t*)&local_enter[in];
        __uint128_t intersect = byps_from_out&byps_from_in;
        if ((byp = find_first_set_bit_128(intersect)) != -1) {
            return true;
        }
        return false;
    }

    bool canIn(int in, int out, int& byp)
    {
        byp = -1;
        if ((*(__uint128_t*)&local_enter[in]&((__uint128_t)1<<out))) {
            return true;
        }
        __uint128_t byps_from_out = *(__uint128_t*)&local_enter[out];  // input and output numbers should be in same line (not byp)
        __uint128_t byps_from_in = *(__uint128_t*)&local_enter[in];
        __uint128_t intersect = byps_from_out&byps_from_in;
        if ((byp = find_first_set_bit_128(intersect)) != -1) {
            return true;
        }
        return false;
    }

};

struct CBState
{
    CBJumpState out_state;
    CBJumpState local_state;
    CBJointState cb_state;
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

        } while ((out_state.src[dirs[dir%8]] & (1<<step)));

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
