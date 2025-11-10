#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "TileType.h"
#include "Crossbar.h"

#include <vector>

namespace fpga {

struct Tile
{
    // must have
    Coord coord;
    Coord name;
    enum {
      TILE_NULL,
      TILE_IO,
      TILE_LUTS,
      TILE_LUTS_SRL,
      TILE_BRAM,
      TILE_LRAM,
      TILE_DSP,
    } type = TILE_NULL;

    int luts6cnt = 0;
    int luts5cnt = 0;
    int luts1cnt = 0;
    int regs_cnt = 0;
    int carry = 0;
    int mux7 = 0;
//    int memcnt = 4;
//    int memtype = 6;
    // optional
    int clk_a = -1;
    int clk_b = -1;
    int memctl_a = -1;
    int memctl_b = -1;

    CBState cb;
    CBType* cb_type;

    const std::string makeName() const
    {
        return std::format("TILE_X{}Y{}", name.x, name.y);
    }

    void assign(rtl::Inst* inst);
    int tryAdd(rtl::Inst* inst);  // it's not SRL
};


}
