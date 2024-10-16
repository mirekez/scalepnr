#pragma once

#include <vector>

#include "TileType.h"

namespace gear {

    struct Tile
    {
        TileType& type;
        Coord coord;

        const Tile& operator =(const Tile& t)
        {
            type = t.type;
            coord = t.coord;
            return *this;
        }
    };

}
