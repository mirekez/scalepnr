#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "TileType.h"

#include <vector>

namespace gear {

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

    int luts6cnt = 4;
    int luts5cnt = 4;
    int luts1cnt = 4;
    int regs_cnt = 4;
    int carry = 4;
    int mux7 = 1;
//    int memcnt = 4;
//    int memtype = 6;
    // optional
    int clk_a = -1;
    int clk_b = -1;
    int memctl_a = -1;
    int memctl_b = -1;

    const std::string makeName() const
    {
        return std::format("CLBLL_X{}Y{}", name.x, name.y);
    }

    void assign(rtl::Inst* inst);
    bool tryAdd(rtl::Inst* inst);  // it's not SRL
};


}
