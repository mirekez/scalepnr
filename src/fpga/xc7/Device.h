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
        // crossbars & tiles specs
        PNR_LOG("FPGA", "loadFromSpec cbs, device_name: '{}'", device_name);
        TileTypesSpec spec;
        std::map<std::string,CBTypeSpec> cbs;
        readCBTypes(std::string("../db/") + device_name + "/tile_type_INT_L.json", &cbs, &spec);
        readCBTypes(std::string("../db/") + device_name + "/tile_type_INT_R.json", &cbs, &spec);

        for (auto& cb : cbs) {  // INT_L and INT_R always go first !!!
            cb_types.push_back(CBType{cb.first});
            cb_types.back().loadFromSpec(cb.second);
        }

        PNR_LOG("FPGA", "loadFromSpec tiles, device_name: '{}'", device_name);
        std::map<std::string,TileSpec> tiles_spec;
        readTileGrid(std::string("../db/") + device_name + "/tilegrid.json", &tiles_spec, &grid_spec);
        tile_grid.resize(grid_spec.size.y*grid_spec.size.x);

        for (const auto& spec : tiles_spec) {
            for (const auto& type : tile_types) {
                if (spec.second.name.find(type.name) == 0) {
                    PNR_LOG1("FPGA",  "found spec '{}' for type '{}'", spec.second.name, type.name);
                    for (const auto& rect : spec.second.rects) {
                        Coord name = rect.name;
                        PNR_LOG2("FPGA",  "populating rect {}/X{}Y{}...", (Rect)rect, name.x, name.y);
                        for (int x = rect.x.a; x != rect.x.b+1; ++x) {
                            name.y = rect.name.y;
                            for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                                tile_grid[x*grid_spec.size.y + y].coord = {x,y};
                                tile_grid[x*grid_spec.size.y + y].name = name;
                                tile_grid[x*grid_spec.size.y + y].cb_type = x%2 == 0 ? &cb_types[0] : &cb_types[1];
                                x_to_grid[name.x] = x;
                                y_to_grid[name.y] = y;
                                ++name.y;
                            }
                            ++name.x;
                        }
                        for (const auto& range : rect.more_x) {
                            name.x = range.name_x;
                            PNR_LOG4("FPGA", " {}/X{}Y{}'", (Range)range, name.x, rect.name.y);
                            for (int x = range.a; x != range.b+1; ++x) {
                                name.y = rect.name.y;
                                for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                    tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                                    tile_grid[x*grid_spec.size.y + y].coord = {x,y};
                                    tile_grid[x*grid_spec.size.y + y].name = name;
                                    tile_grid[x*grid_spec.size.y + y].cb_type = x%2 == 0 ? &cb_types[0] : &cb_types[1];
                                    x_to_grid[name.x] = x;
                                    y_to_grid[name.y] = y;
                                    ++name.y;
                                }
                                ++name.x;
                            }
                        }
                    }
                }
            }
        }

        size_width = grid_spec.size.x;
        size_height = grid_spec.size.y;
        cnt_regs = 2*grid_spec.size.y*grid_spec.size.x*4;
        cnt_luts = 2*grid_spec.size.y*grid_spec.size.x*4;
        PNR_LOG("FPGA", "loadFromSpec, fpga width: {}, height: {}, cnt_regs: {}, cnt_luts: {}, device_name: '{}'", size_width, size_height, cnt_regs, cnt_luts, device_name);

        PNR_LOG("FPGA", "loadFromSpec pins, device_name: '{}'", device_name);
        std::vector<PinSpec> specs;
        readPackagePins(std::string("../db/") + device_name + "/package_pins.csv", specs);

        for (auto spec : specs) {
            pins.push_back(Pin{spec.name, spec.bank, spec.site, spec.tile, spec.function, spec.pos});
        }

    }


    static Device& current();
};


/*
    std::vector<PinMap> bels_outputs2site_outputs;
    std::vector<PinMap> site_inputs2bels_inputs;
    std::vector<PinMap> site_inputs2site_outputs;
*/

}
