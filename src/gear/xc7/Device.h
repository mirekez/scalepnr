#pragma once

#include "TileType.h"
#include "Tile.h"
#include "DeviceFormat.h"
#include "debug.h"

#include "referable.h"

namespace gear {

struct Device
{
    TileGridSpec grid_spec;
    std::vector<TileType> tile_types;
    std::vector<Referable<Tile>> tile_grid;

    void loadFromSpec(const std::string& device_name)
    {
        PNR_LOG("GEAR", "loadFromSpec, device_name: '{}'", device_name);
        std::map<std::string,TileSpec> tiles_spec;
        readTileGrid(std::string("../db/") + device_name + "/tilegrid.json", JSON_OBJECTS_IDENT, &tiles_spec, &grid_spec);
        tile_types.push_back({"unknown"});
        tile_grid.resize(grid_spec.size.y*grid_spec.size.x);//, Referable<Tile>{/*tile_types.back()*/});

        for (const auto& spec : tiles_spec) {
            for (const auto& type : tile_types) {
                if (spec.second.name.find(type.name) == 0) {
                    PNR_LOG1("GEAR",  "found spec '{}' for type '{}'", spec.second.name, type.name);
                    for (const auto& rect : spec.second.rects) {
                        Coord name = rect.name;
                        PNR_LOG2("GEAR",  "populating rect {}/X{}Y{}...", (Rect)rect, name.x, name.y);
                        for (int x = rect.x.a; x != rect.x.b+1; ++x) {
                            name.y = rect.name.y;
                            for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                                tile_grid[x*grid_spec.size.y + y].coord = {x,y};
                                tile_grid[x*grid_spec.size.y + y].name = name;
                                ++name.y;
                            }
                            ++name.x;
                        }
                        for (const auto& range : rect.more_x) {
                            name.x = range.name_x;
                            PNR_LOG3("GEAR", " {}/X{}Y{}'", (Range)range, name.x, rect.name.y);
                            for (int x = range.a; x != range.b+1; ++x) {
                                name.y = rect.name.y;
                                for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                    tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                                    tile_grid[x*grid_spec.size.y + y].coord = {x,y};
                                    tile_grid[x*grid_spec.size.y + y].name = name;
                                    ++name.y;
                                }
                                ++name.x;
                            }
                        }
                    }
                }
            }
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
