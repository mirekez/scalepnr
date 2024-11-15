
#include "XC7Tech.h"
#include "Device.h"

XC7Tech& XC7Tech::current()
{
    static XC7Tech tech;
    return tech;
}

void XC7Tech::init()
{
    gear::TileType tile0{"CLBLL", 123};
    tile0.bells.push_back(BelType{"SLICE0L_D5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_D6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE0L_C5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_C6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE0L_B5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_B6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE0L_A5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_A6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});

    tile0.bells.push_back(BelType{"SLICE0L_D5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_DCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_F7BMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_C5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_CCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_F8MUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_B5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_BCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_F7AMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_A5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_ACY0", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE0L_PRECYINIT", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE0L_CEUSEDMUX", BelType::MUX, 1});
    tile0.bells.push_back(BelType{"SLICE0L_SRUSEDMUX", BelType::MUX, 1});

    tile0.bells.push_back(BelType{"SLICE0L_DOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_DFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_COUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_CFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_BOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_BFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_AOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_AFFMUX", BelType::MUX, 5});

    tile0.bells.push_back(BelType{"SLICE0L_D5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_DFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_C5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_CFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_B5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_BFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_A5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_AFF", BelType::FF, 4});

    tile0.bells.push_back(BelType{"SLICE0L_CLKINV", BelType::CLKINV, 2});

    tile0.bells.push_back(BelType{"SLICE1L_D5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_D6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE1L_C5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_C6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE1L_B5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_B6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE1L_A5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_A6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});

    tile0.bells.push_back(BelType{"SLICE1L_D5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_DCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_F7BMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_C5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_CCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_F8MUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_B5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_BCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_F7AMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_A5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_ACY0", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE1L_PRECYINIT", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE1L_CEUSEDMUX", BelType::MUX, 1});
    tile0.bells.push_back(BelType{"SLICE1L_SRUSEDMUX", BelType::MUX, 1});

    tile0.bells.push_back(BelType{"SLICE1L_DOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_DFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_COUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_CFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_BOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_BFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_AOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_AFFMUX", BelType::MUX, 5});

    tile0.bells.push_back(BelType{"SLICE1L_D5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_DFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_C5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_CFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_B5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_BFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_A5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_AFF", BelType::FF, 4});

    tile0.bells.push_back(BelType{"SLICE1L_CLKINV", BelType::CLKINV, 2});
    gear::Device::current().tile_types.push_back(tile0);

    /////////////////////////////////////

    gear::TileType tile1{"CLBLM", 123};
    tile1.bells.push_back(BelType{"SLICE0L_D5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_D6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile1.bells.push_back(BelType{"SLICE0L_C5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_C6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile1.bells.push_back(BelType{"SLICE0L_B5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_B6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile1.bells.push_back(BelType{"SLICE0L_A5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_A6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});

    tile1.bells.push_back(BelType{"SLICE0L_D5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_DCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_F7BMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_C5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_CCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_F8MUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_B5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_BCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_F7AMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_A5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_ACY0", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE0L_PRECYINIT", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE0L_CEUSEDMUX", BelType::MUX, 1});
    tile1.bells.push_back(BelType{"SLICE0L_SRUSEDMUX", BelType::MUX, 1});

    tile1.bells.push_back(BelType{"SLICE0L_DOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_DFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_COUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_CFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_BOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_BFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_AOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_AFFMUX", BelType::MUX, 5});

    tile1.bells.push_back(BelType{"SLICE0L_D5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_DFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_C5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_CFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_B5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_BFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_A5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_AFF", BelType::FF, 4});

    tile1.bells.push_back(BelType{"SLICE0L_CLKINV", BelType::CLKINV, 2});

    tile1.bells.push_back(BelType{"SLICE1M_D5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_D6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});
    tile1.bells.push_back(BelType{"SLICE1M_C5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_C6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});
    tile1.bells.push_back(BelType{"SLICE1M_B5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_B6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});
    tile1.bells.push_back(BelType{"SLICE1M_A5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_A6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});

    tile1.bells.push_back(BelType{"SLICE1M_CDI1MUX", BelType::MUX, 3});
    tile1.bells.push_back(BelType{"SLICE1M_BDI1MUX", BelType::MUX, 3});
    tile1.bells.push_back(BelType{"SLICE1M_ADI1MUX", BelType::MUX, 3});

    tile1.bells.push_back(BelType{"SLICE1M_D5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_DCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_F7BMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_C5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_CCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_F8MUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_B5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_BCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_F7AMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_A5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_ACY0", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE1M_PRECYINIT", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE1M_CEUSEDMUX", BelType::MUX, 1});
    tile1.bells.push_back(BelType{"SLICE1M_SRUSEDMUX", BelType::MUX, 1});

    tile1.bells.push_back(BelType{"SLICE1M_DOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_DFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_COUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_CFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_BOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_BFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_AOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_AFFMUX", BelType::MUX, 5});

    tile1.bells.push_back(BelType{"SLICE1M_D5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_DFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_C5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_CFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_B5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_BFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_A5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_AFF", BelType::FF, 4});

    tile1.bells.push_back(BelType{"SLICE1M_CLKINV", BelType::CLKINV, 2});
    gear::Device::current().tile_types.push_back(tile1);

    gear::TileType tile2{"LIOI3_TBYTESRC", 123};
    tile2.bells.push_back(BelType{"DINV", BelType::MUX, 2});
    tile2.bells.push_back(BelType{"SLICE0_ZHOLD_DELAY", BelType::ZHOLD_DELAY, 1, BelType::SIMPLE, 2});
    tile2.bells.push_back(BelType{"ZHOLD_FABRIC_INV", BelType::MUX, 2});
    tile2.bells.push_back(BelType{"IDEL", BelType::MUX, 3});
    tile2.bells.push_back(BelType{"D2OBYP_SRC", BelType::DMUX, 3});
    tile2.bells.push_back(BelType{"D2OBYP_SEL", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IFFDELMUXE3", BelType::DMUX, 3});
    tile2.bells.push_back(BelType{"D2OFFBYP_SRC", BelType::DMUX, 3});
    tile2.bells.push_back(BelType{"D2OFFBYP_SEL", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IMUX", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IFFMUX", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IFF", BelType::IFF, 6, BelType::SIMPLE, 2});
    tile2.bells.push_back(BelType{"CLKBINV", BelType::CLKINV});
    tile2.bells.push_back(BelType{"CLKINV", BelType::CLKINV});

    tile2.bells.push_back(BelType{"IDELAYE2", BelType::IDELAY, 14, BelType::SIMPLE, 6});
    tile2.bells.push_back(BelType{"IDATAININV", BelType::MUX, 2, BelType::SIMPLE, 1});
    tile2.bells.push_back(BelType{"DATAININV", BelType::MUX, 2, BelType::SIMPLE, 1});
    tile2.bells.push_back(BelType{"CINV", BelType::MUX, 2, BelType::SIMPLE, 1});

    gear::TileType tile3{"LIOI3_TBYTESRC", 123};
    tile3.bells.push_back(BelType{"TFF", BelType::TFF, 6});
    tile3.bells.push_back(BelType{"OUTFF", BelType::TFF, 6});
    tile3.bells.push_back(BelType{"TMUX", BelType::TFF, 2});
    tile3.bells.push_back(BelType{"MISR", BelType::MISR, 1});
    tile3.bells.push_back(BelType{"T2INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"T1INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"D2INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"D1INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"CLKINV", BelType::CLKINV, 1});
    tile3.bells.push_back(BelType{"O_ININV", BelType::ININV, 1});

//    gear::TileType tile4{"PAD", 123, {{{2,0},{10,10}}}};
//    tile4.bells.push_back(BelType{"PAD", BelType::PAD, 0});
//    tile4.bells.push_back(BelType{"OUTBUF", BelType::OUTBUF, 2});
//    tile4.bells.push_back(BelType{"INBUF_EN", BelType::INBUF, 0});
//    tile4.bells.push_back(BelType{"OUTMUX", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"TINMUX", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"OINMUX", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"INTERMDISABLE_SEL", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"IBUFDISABLE_SEL", BelType::MUX, 2});

}
