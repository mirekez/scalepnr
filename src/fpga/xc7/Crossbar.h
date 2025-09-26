#pragma once

#include <string>
#include <stdint.h>
#include <map>

#include "DeviceFormat.h"
#include "debug.h"

#define CB_CLB_NODES 1024  //  2^10
#define CB_MAX_NODES 2048  //  2^11

namespace fpga {

struct CBFromCB
{
    uint8_t num:5;
    uint8_t type:3;  // 1, 2, 4, 6, 0, 9, 12, 18
    uint8_t dir:3;
    uint8_t rsv:5;
}__attribute__((packed));  // 11 bit

struct CBFromCLB
{
    uint16_t imux_outs_byp_fan_alt_bounce_gndvcc_ctrl:11;
}__attribute__((packed));  // 11 bit

struct CBExitState
{  // (1 2 4 6)*4 + num
    uint16_t beg_n;
    uint16_t beg_ne;
    uint16_t beg_e;
    uint16_t beg_se;
    uint16_t beg_s;
    uint16_t beg_sw;
    uint16_t beg_w;
    uint16_t beg_nw;
}__attribute__((packed));

struct CBLongState
{
    uint8_t lv_b;  // 0 9 12 18
    uint8_t lv_bl;  // 0 9 12 18
    uint8_t lv_l;  // 0 9 12 18
    uint8_t lv;  // 0 9 12 18
    uint8_t lh_b;  // 0 9 12 18
    uint8_t lh_bl;  // 0 9 12 18
    uint8_t lh_l;  // 0 9 12 18
    uint8_t lh;  // 0 9 12 18
    uint64_t rsv;
}__attribute__((packed));

struct CBEnterState
{
    __uint128_t imux_outs_byp_fan_alt_bounce_gndvcc_ctrl;
};

struct AltBounceState
{
    __uint128_t alt_bounce;
}; // + clock?

struct CBType
{
    std::string name;
    CBExitState cb_exit[CB_MAX_NODES];
    CBLongState cb_long[CB_MAX_NODES];
    CBExitState clb_exit[CB_MAX_NODES];
    CBEnterState cb_enter[CB_MAX_NODES];
    AltBounceState clb_enter[CB_MAX_NODES];

    const int imuxl_from = 40;
    const int imuxl_cnt = 10;
    const int imuxl_beg = 0;
    const int imux_from = 40;
    const int imux_cnt = 10;
    const int imux_beg = imuxl_beg + imuxl_cnt;
    const int outsl_from = 40;
    const int outsl_cnt = 24;
    const int outsl_beg = imux_beg + imux_cnt;
    const int outs_from = 40;
    const int outs_cnt = 24;
    const int outs_beg = outsl_beg + outsl_cnt;
    const int bypl_from = 0;
    const int bypl_cnt = 8;
    const int bypl_beg = outs_beg + outs_cnt;
    const int bypbouncen_from = 0;
    const int bypbouncen_cnt = 8;
    const int bypbouncen_beg = bypl_beg + bypl_cnt;
    const int bypbounce_from = 0;
    const int bypbounce_cnt = 8;
    const int bypbounce_beg = bypbouncen_beg + bypbounce_cnt;
    const int fanalt_from = 0;
    const int fanalt_cnt = 8;
    const int fanalt_beg = bypbounce_beg + bypbounce_cnt;
    const int fanbounces_from = 0;
    const int fanbounces_cnt = 8;
    const int fanbounces_beg = fanalt_beg + fanalt_cnt;
    const int fanbounce_from = 0;
    const int fanbounce_cnt = 8;
    const int fanbounce_beg = fanbounces_beg + fanbounces_cnt;
    const int gclkb_from = 0;
    const int gclkb_cnt = 12*3;
    const int gclkb_beg = fanbounce_beg + fanbounce_cnt;
    const int gfan_from = 0;
    const int gfan_cnt = 2;
    const int gfan_beg = gclkb_beg + gclkb_cnt;
    const int wire_beg = gfan_beg + gfan_cnt;

    int /*external*/ parseNode(std::string name, CBFromCB& fromcb, CBFromCLB& fromclb, CBExitState& ext, CBLongState& lng, CBEnterState& enter, AltBounceState& bounce)
    {
        if (name.find("IMUX_L") != (size_t)-1) {
            int num = name[6] == '1' && name[7] != 0 ? 10+name[7] :  name[7];
            num -= imuxl_from;
            PNR_ASSERT(num >= 0 && num < imuxl_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = imuxl_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (imuxl_beg+num);
            PNR_ASSERT(imuxl_beg+num < 128, "{}", imuxl_beg+num);
            return false;
        }
        else
        if (name.find("IMUX") != (size_t)-1) {
            int num = name[4] == '1' && name[5] != 0 ? 10+name[5] :  name[5];
            num -= imux_from;
            PNR_ASSERT(num >= 0 && num < imux_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = imux_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (imux_beg+num);
            PNR_ASSERT(imux_beg+num < 128, "{}", imux_beg+num);
            return false;
        }
        else
        if (name.find("LOGIC_OUTS_L") != (size_t)-1) {
            int num = name[12] == '1' && name[13] != 0 ? 10+name[13] :  name[13];
            num -= outsl_from;
            PNR_ASSERT(num >= 0 && num < imuxl_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = outsl_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (outsl_beg+num);
            PNR_ASSERT(outsl_beg+num < 128, "{}", outsl_beg+num);
            return false;
        }
        else
        if (name.find("LOGIC_OUTS") != (size_t)-1) {
            int num = name[10] == '1' && name[11] != 0 ? 10+name[11] :  name[11];
            num -= outs_from;
            PNR_ASSERT(num >= 0 && num < imuxl_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = outs_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (outs_beg+num);
            PNR_ASSERT(outs_beg+num < 128, "{}", outs_beg+num);
            return false;
        }
        else
        if (name.find("BYP_L") != (size_t)-1) {
            int num = name[5];
            num -= bypl_from;
            PNR_ASSERT(num >= 0 && num < bypl_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = bypl_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (bypl_beg+num);
            PNR_ASSERT(bypl_beg+num < 128, "{}", bypl_beg+num);
            return false;
        }
        else
        if (name.find("BYP_BOUNCE_N3_") != (size_t)-1) {
            int num = name[5];
            num -= bypbouncen_from;
            PNR_ASSERT(num >= 0 && num < bypbouncen_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = bypbouncen_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (bypbouncen_beg+num);
            PNR_ASSERT(bypbouncen_beg+num < 128, "{}", bypbouncen_beg+num);

            bounce.alt_bounce = 0 + num;
            return false;
        }
        else
        if (name.find("BYP_BOUNCE") != (size_t)-1) {
            int num = name[5];
            num -= bypbounce_from;
            PNR_ASSERT(num >= 0 && num < bypbounce_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = bypbounce_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (bypbounce_beg+num);
            PNR_ASSERT(bypbounce_beg+num < 128, "{}", bypbounce_beg+num);

            bounce.alt_bounce = 10 + num;
            return false;
        }
        else
        if (name.find("FAN_ALT") != (size_t)-1) {
            int num = name[7];
            num -= fanalt_from;
            PNR_ASSERT(num >= 0 && num < fanalt_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = fanalt_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (fanalt_beg+num);
            PNR_ASSERT(fanalt_beg+num < 128, "{}", fanalt_beg+num);

            bounce.alt_bounce = 20 + num;
            return false;
        }
        else
        if (name.find("FAN_BOUNCE_S") != (size_t)-1) {
            int num = name[12];
            num -= fanbounces_from;
            PNR_ASSERT(num >= 0 && num < fanbounces_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = fanbounces_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (fanbounces_beg+num);
            PNR_ASSERT(fanbounces_beg+num < 128, "{}", fanbounces_beg+num);

            bounce.alt_bounce = 30 + num;
            return false;
        }
        else
        if (name.find("FAN_BOUNCE")) {
            int num = name[10];
            num -= fanbounce_from;
            PNR_ASSERT(num >= 0 && num < fanbounce_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = fanbounce_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (fanbounce_beg+num);
            PNR_ASSERT(fanbounce_beg+num < 128, "{}", fanbounce_beg+num);

            bounce.alt_bounce = 40 + num;
            return false;
        }
        else
        if (name.find("GCLK_B")) {  // 12 + EAST + WEST
            int num = name[6] == '1' && name[7] != 0 ? 10+name[7] :  name[7];
            if (name.find("WEST")) {
                num += gclkb_cnt;
            }
            if (name.find("EAST")) {
                num += gclkb_cnt*2;
            }
            num -= gclkb_from;
            PNR_ASSERT(num >= 0 && num < gclkb_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = gclkb_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (gclkb_beg+num);
            PNR_ASSERT(gclkb_beg+num < 128, "{}", gclkb_beg+num);
            return false;
        }
        else
        if (name.find("GFAN")) {
            int num = name[4];
            num -= gfan_from;
            PNR_ASSERT(num >= 0 && num < gfan_cnt, "{}", num);
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = gfan_beg+num;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (gfan_beg+num);
            PNR_ASSERT(gfan_beg+num < 128, "{}", gfan_beg+num);
            return false;
        }
        else
        if (name == "GND_WIRE") {
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = wire_beg+0;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (wire_beg+0);
            PNR_ASSERT(wire_beg+0 < 128, "{}", wire_beg+0);
            return false;
        }
        else
        if (name == "VCC_WIRE") {
            fromclb.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = wire_beg+1;
            enter.imux_outs_byp_fan_alt_bounce_gndvcc_ctrl = 1 << (wire_beg+1);
            PNR_ASSERT(wire_beg+1 < 128, "{}", wire_beg+1);
            return false;
        }
        else
        if (name[0] == 'L' && name[1] == 'V') {  // LVB_L12
            int num = name[2] == '1' && name[3] != 0 ? 10+name[3] :  name[3];
            fromcb.dir = 0;
            if (name[2] == 'B') {
                num = name[3] == '1' && name[4] != 0 ? 10+name[4] :  name[4];
                fromcb.dir = 1;
            }
            if (name[4] == 'L') {
                num = name[5] == '1' && name[6] != 0 ? 10+name[6] :  name[6];
                fromcb.dir = 2;
            }
            fromcb.num = 0;
            fromcb.type = num == 0 ? 4 : (num == 9 ? 5 : (num == 12 ? 6 : 7 ));  // 1, 2, 4, 6, 0, 9, 12, 18

            if (name[2] == 'B') {
                if (name[4] == 'L') {
                    lng.lv_bl = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
                }
                else {
                    lng.lv_b = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
                }
            }
            else
            if (name[2] == 'L') {
                lng.lv_l = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
            }
            else {
                lng.lv = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
            }
            return true;
        }
        else
        if (name[0] == 'L' && name[1] == 'H') {  // LVB_L12
            int num = name[2] == '1' && name[3] != 0 ? 10+name[3] :  name[3];
            fromcb.dir = 0;
            if (name[2] == 'B') {
                num = name[3] == '1' && name[4] != 0 ? 10+name[4] :  name[4];
                fromcb.dir = 1;
            }
            if (name[4] == 'L') {
                num = name[5] == '1' && name[6] != 0 ? 10+name[6] :  name[6];
                fromcb.dir = 2;
            }
            fromcb.num = 0;
            fromcb.type = num == 0 ? 4 : (num == 9 ? 5 : (num == 12 ? 6 : 7 ));  // 1, 2, 4, 6, 0, 9, 12, 18

            if (name[2] == 'B') {
                if (name[4] == 'L') {
                    lng.lv_bl = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
                }
                else {
                    lng.lv_b = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
                }
            }
            else
            if (name[2] == 'L') {
                lng.lv_l = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
            }
            else {
                lng.lv = 1 << (num == 0 ? 0 : (num == 9 ? 1 : (num == 12 ? 2 : 3 )));
            }
            return true;
        }
        else {
            // WW2BEG1
//            bool a_beg = name[3] == 'B' && name[4] == 'E' && name[5] == 'G';
//            bool a_end = name[3] == 'E' && name[4] == 'N' && name[5] == 'D';
            int len = name[2];
            int num = name[6];
            fromcb.num = num;
            fromcb.type = len == 1 ? 0 : (len == 2 ? 1 : (len == 4 ? 2 : 3));  // 1, 2, 4, 6, 0, 9, 12, 18
            if (name[0] == 'W') {
                ext.beg_w = len*4 + num;
                fromcb.dir = 6;
            }
            if (name[0] == 'E') {
                ext.beg_e = len*4 + num;
                fromcb.dir = 2;
            }
            if (name[0] == 'N') {
                if (name[1] == 'W') {
                    ext.beg_nw = len*4 + num;
                    fromcb.dir = 7;
                }
                else
                if (name[1] == 'E') {
                    ext.beg_ne = len*4 + num;
                    fromcb.dir = 1;
                }
                else {
                    ext.beg_n = len*4 + num;
                    fromcb.dir = 0;
                }
            }
            if (name[0] == 'S') {
                if (name[1] == 'W') {
                    ext.beg_sw = len*4 + num;
                    fromcb.dir = 5;
                }
                else
                if (name[1] == 'E') {
                    ext.beg_se = len*4 + num;
                    fromcb.dir = 3;
                }
                else {
                    ext.beg_s = len*4 + num;
                    fromcb.dir = 4;
                }
            }

            return true;
        }
        PNR_ASSERT(0, "unknown node type: {}", name);
        return true;
    }

    void loadFromSpec(const CBTypeSpec& spec)
    {
        memset(cb_exit, 0, sizeof(cb_exit));
        memset(cb_long, 0, sizeof(cb_long));
        memset(clb_exit, 0, sizeof(clb_exit));
        memset(cb_enter, 0, sizeof(cb_enter));
        memset(clb_enter, 0, sizeof(clb_enter));
        for (const auto& pair : spec.nodes) {
            CBFromCB a_fromcb = {}, b_fromcb = {};
            CBFromCLB a_fromclb = {}, b_fromclb = {};
            CBExitState a_exit = {}, b_exit = {};
            CBLongState a_long = {}, b_long = {};
            CBEnterState a_enter = {}, b_enter = {};
            AltBounceState a_bounce = {}, b_bounce = {};

            bool a_external = parseNode(pair.first, a_fromcb, a_fromclb, a_exit, a_long, a_enter, a_bounce);
            bool b_external = parseNode(pair.second, b_fromcb, b_fromclb, b_exit, b_long, b_enter, b_bounce);

            if (a_external && b_external) {
                *(__uint128_t*)&cb_exit[*(uint16_t*)&a_fromcb] |= *(__uint128_t*)&b_exit;
                *(__uint128_t*)&cb_long[*(uint16_t*)&a_fromcb] |= *(__uint128_t*)&b_long;
            }
            if (!a_external && b_external) {
                *(__uint128_t*)&clb_exit[*(uint16_t*)&a_fromclb] |= *(__uint128_t*)&b_exit;
            }
            if (a_external && !b_external) {
                *(__uint128_t*)&cb_enter[*(uint16_t*)&a_fromcb] |= *(__uint128_t*)&b_enter;
            }
            if (!a_external && !b_external) {
                *(__uint128_t*)&clb_enter[*(uint16_t*)&a_fromclb] |= *(__uint128_t*)&b_bounce;
            }
        }
    }
};

struct CBState
{
    CBExitState out_state;
    CBEnterState clb_state;
    AltBounceState cb_state;
    CBType* type;
};

}
