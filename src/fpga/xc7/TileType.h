#pragma once

#include <vector>

#include "Types.h"
#include "BelType.h"
#include "Pin.h"

namespace fpga {

struct TileType
{
    // must have
    std::string name;
    size_t num;

    enum {
        TILE_NULL,
        TILE_HCLK_V,
        TILE_INT_FE,
        TILE_INT_IN,
        TILE_INT_L,
        TILE_INT_R,
        TILE_IO_INT,
        TILE_LIOB33,
        TILE_LIOI3,
        TILE_L_TERM,
        TILE_MONITO,
        TILE_PCIE_B,
        TILE_PCIE_I,
        TILE_PCIE_N,
        TILE_PCIE_T,
        TILE_RIOB33,
        TILE_RIOI3,
        TILE_R_TERM,
        TILE_TERM_C,
        TILE_T_TERM,
        TILE_VBRK,
        TILE_VFRAME
    } type;

    // optional
    std::vector<BelType> bells;
    std::vector<Pin> bels_inputs;
    std::vector<Pin> bels_outputs;
    std::vector<Pin> site_inputs;
    std::vector<Pin> site_outputs;
};

}
