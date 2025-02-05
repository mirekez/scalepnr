#pragma once

#include "Tile.h"
#include "referable.h"
#include "Clock.h"

#include <vector>

namespace pnr
{

struct TileSet
{
    // must have
    std::vector<Referable<gear::Tile>> tiles;
//    Clock* clk_in;
    // optional
};


}
