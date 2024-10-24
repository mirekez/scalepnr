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
                        Coord name = rect.name;
                        PNR_LOG2("GEAR",  "populating rect {}/X{}Y{}... ", (Rect)rect, name.x, name.y);
                        for (int x = rect.x.a; x != rect.x.b+1; ++x) {
                            name.y = rect.name.y;
                            for (int y = rect.y.b; y != rect.y.a-1; --y) {
                                tileGrid[x*size.y + y].type = std::reference_wrapper(type);
                                tileGrid[x*size.y + y].coord = {x,y};
                                tileGrid[x*size.y + y].name = name;
                                ++name.y;
                            }
                            ++name.x;
                        }
                        for (const auto& range : rect.more_x) {
                            name.x = range.name_x;
                            PNR_LOG3("GEAR", "{}/X{}Y{}' ", (Range)range, name.x, rect.name.y);
                            for (int x = range.a; x != range.b+1; ++x) {
                                name.y = rect.name.y;
                                for (int y = rect.y.b; y != rect.y.a-1; --y) {
                                    tileGrid[x*size.y + y].type = std::reference_wrapper(type);
                                    tileGrid[x*size.y + y].coord = {x,y};
                                    tileGrid[x*size.y + y].name = name;
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
