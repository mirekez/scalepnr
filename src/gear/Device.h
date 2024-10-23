#pragma once

#include "TileType.h"
#include "Tile.h"
#include "TileGridSpec.h"
#include "debug.h"

namespace gear {

struct Device
{
    Coord size;
    std::vector<TileType> tileTypes;
    std::vector<Tile> tileGrid;

    void loadFromSpec(const std::string& device_name)
    {
        PNR_LOG("GEAR", "loadFromSpec, device_name: '{}'", device_name);
        std::map<std::string,TileGridSpec> tileGridSpec;
        size = readTileGrid(std::string("../db/") + device_name + "/tilegrid.json", JSON_OBJECTS_IDENT, tileGridSpec);
        tileTypes.push_back({"unknown"});
        tileGrid.resize(size.y*size.x, {tileTypes.back()});

        for (const auto& spec : tileGridSpec) {
            for (const auto& type : tileTypes) {
                if (spec.second.name.find(type.name) == 0) {
                    PNR_LOG1("GEAR",  "found spec '{}' for type '{}'", spec.second.name, type.name);
                    for (const auto& rect : spec.second.rects) {
                        PNR_LOG2("GEAR",  "populating rect {}... ", rect);
                        int name_x = spec.second.name_x;
                        PNR_LOG3("X{}.{}, ", name_x, rect.x);
                        for (int x = rect.x.a; x != rect.x.b+1; ++x) {
                            for (int y = rect.y.a; y != rect.y.b+1; ++y) {
                                tileGrid[x*size.y + y].type = std::reference_wrapper(type);
                                tileGrid[x*size.y + y].coord = {x,y};
                                tileGrid[x*size.y + y].name_x = name_x;
                            }
                        }
//                        Range prev;
                        for (const auto& range : rect.more_x) {
                            PNR_LOG3("{}'X{}' ", range, name_x+1);
                            for (int x = range.a; x != range.b+1; ++x) {
                                ++name_x;
                                for (int y = rect.y.a; y != rect.y.b+1; ++y) {
                                    tileGrid[x*size.y + y].type = std::reference_wrapper(type);
                                    tileGrid[x*size.y + y].coord = {x,y};
                                    tileGrid[x*size.y + y].name_x = name_x;
                                }
//                                prev = range;
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
