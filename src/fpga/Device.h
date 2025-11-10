#pragma once

#include "TileType.h"
#include "Tile.h"
#include "WireType.h"
#include "Wire.h"
#include "Pin.h"
#include "DeviceFormat.h"
#include "debug.h"

#include "referable.h"

namespace fpga {

struct Device
{
    TileGridSpec grid_spec;
    TileTypesSpec types_spec;
    std::vector<TileType> tile_types;
    std::vector<CBType> cb_types;
    std::vector<Referable<Tile>> tile_grid;
    std::map<Coord,Wire> wires;

    std::map<int,int> x_to_grid;
    std::map<int,int> y_to_grid;
    std::vector<Pin> pins;
    int size_width = 0;
    int size_height = 0;
    int cnt_regs = 0;
    int cnt_luts = 0;
    std::vector<WireType> wire_types;
    std::vector<std::vector<Referable<Wire>>> wire_grid;

    void loadFromSpec(const std::string& device_name)
    {
    }

    Tile* getTile(int x, int y)
    {
        return &tile_grid[x*grid_spec.size.y + y];
    }

    static Device& current();
};


}
