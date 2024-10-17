#pragma once

#include <vector>
#include <format>

#include "TileType.h"

namespace gear {

struct Tile
{
    TileType& type;
    Coord coord;

    const std::string getName() const
    {
        return std::format("CLBLL_X{}Y{}", coord.x, coord.y);
    }

    const Tile& operator =(const Tile& t)
    {
        type = t.type;
        coord = t.coord;
        return *this;
    }
};


}
