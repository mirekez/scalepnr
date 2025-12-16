#pragma once

#include "TileType.h"
#include "Tile.h"
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
    std::vector<std::vector<Referable<Wire>>> wire_grid;
//tilegrid.json
    void loadFromSpec(const std::string& spec_name, const std::string& pins_spec_name);
    void loadTypeFromSpec(const std::string& spec_name, TechMap& map);
    void loadCBFromSpec(const std::string& spec_name, TechMap& map);
    Tile* getTile(int x, int y);

    static Device& current();
};

}
