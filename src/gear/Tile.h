#pragma once

#include <vector>
#include <format>

#include "TileType.h"

namespace gear {

struct Tile
{
    std::reference_wrapper<const TileType> type;
    Coord coord;
    int name_x = -1;

    const std::string getName() const
    {
        return std::format("CLBLL_X{}Y{}", coord.x, coord.y);
    }
};


}
