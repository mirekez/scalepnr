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
    void loadFromSpec(const std::string& spec_name, const std::string& pins_spec_name)
    {
        // tiles specs
        PNR_LOG("FPGA", "loadFromSpec, spec_name: '{}'", spec_name);
        std::map<std::string,TileSpec> tiles_spec;
        readTileGrid(spec_name, &tiles_spec, &grid_spec);
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
        PNR_LOG("FPGA", "loadFromSpec, fpga width: {}, height: {}, cnt_regs: {}, cnt_luts: {}, pins_spec_name: '{}'", size_width, size_height, cnt_regs, cnt_luts, pins_spec_name);

        PNR_LOG("FPGA", "loadFromSpec pins, pins_spec_name: '{}'", pins_spec_name);
        std::vector<PinSpec> specs;
        readPackagePins(pins_spec_name, specs);

        for (auto spec : specs) {
            pins.push_back(Pin{spec.name, spec.bank, spec.tile, spec.function, spec.pos});
        }
    }

    void loadTypeFromSpec(const std::string& spec_name, TechMap& map)
    {
        // any types except cb
        PNR_LOG("FPGA", "loadCBFromSpec, spec_name: '{}'", spec_name);
        TileTypesSpec spec;
        std::map<std::string,TypeSpec> types;
        readTypes(spec_name, &types, &spec);
//        for (auto& type : types) {
//            types.emplace(CBType{cb.first});
//            types.back().loadFromSpec(map, cb.second);
//        }
    }

    void loadCBFromSpec(const std::string& spec_name, TechMap& map)
    {
        // crossbars
        PNR_LOG("FPGA", "loadCBFromSpec, spec_name: '{}'", spec_name);
        TileTypesSpec spec;
        std::map<std::string,CBTypeSpec> cbs;
        readCBTypes(spec_name, &cbs, &spec);
        for (auto& cb : cbs) {
            cb_types.push_back(CBType{cb.first});
            cb_types.back().loadFromSpec(cb.second, map);
        }
    }

    Tile* getTile(int x, int y)
    {
        return &tile_grid[x*grid_spec.size.y + y];
    }

    static Device& current();
};

}
