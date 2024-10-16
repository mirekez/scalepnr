#pragma once

#include "TileType.h"
#include "Tile.h"
#include "TileGridSpec.h"
#include "debug.h"

namespace gear {


// used as a context
struct Gear
{
    Coord size;
    std::vector<TileType> tileTypes;
    std::vector<Tile> tileGrid;

    void prepareDevice(const std::string& device_name)
    {
        PNR_LOG("GEAR","prepareDevice, device_name: {}", device_name);
        std::map<std::string,TileGridSpec> tileGridSpec;
        size = readTileGrid(std::string("../db/") + device_name + "/tilegrid.json", JSON_OBJECTS_IDENT, tileGridSpec);
        tileTypes.push_back({"unknown"});
        tileGrid.resize(size.y*size.x, {tileTypes.back()});

        for (const auto& spec : tileGridSpec) {
            for (const auto& type : tileTypes) {
                if (spec.second.name.find(type.name) == 0) {
                    PNR_LOG1("GEAR", "found spec '{}' for type '{}'", spec.second.name, type.name);
                    for (const auto& rect : spec.second.rects) {
                        PNR_LOG2("GEAR", "populating rect {}...", rect);
                        for (int x = rect.a.x; x != rect.b.x+1; ++x) {
                            for (int y = rect.a.y; y != rect.b.y+1; ++y) {
                                tileGrid[x*size.y + y].type = type;
                                tileGrid[x*size.y + y].coord = {x,y};
                            }
                        }
                        for (const auto& range : rect.more_x) {
                            for (int x = range.a; x != range.b+1; ++x) {
                                for (int y = rect.a.y; y != rect.b.y+1; ++y) {
                                    tileGrid[x*size.y + y].type = type;
                                    tileGrid[x*size.y + y].coord = {x,y};
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};




/*
    std::vector<PinMap> bels_outputs2site_outputs;
    std::vector<PinMap> site_inputs2bels_inputs;
    std::vector<PinMap> site_inputs2site_outputs;
*/

}
