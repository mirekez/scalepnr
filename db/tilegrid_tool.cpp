#include <print>
#include <fstream>
#include <iostream>
#include <io.h>
#include <fcntl.h>
#include <vector>
#include "json/json.h"
#include <map>

#include "debug.h"
#include "sscan.h"
#include "Types.h"
#include "TileGridSpec.h"

void importTileGrid(const std::string& filename, bool xray = false)
{
    std::map<std::string,TileGridSpec> tiles;
    if (xray) {
        readXrayTileGrid(filename, 4, tiles);
    }
    else {
        readTileGrid(filename, 4, tiles);
    }

    std::print("{{\n");
    std::string div = "";
    for (const auto& tile : tiles) {

        std::string populate = "";
        std::string separator = "";
        for (const auto& rect : tile.second.rects) {
            populate += std::format("{}{}", separator, rect);
            separator = ", ";
        }

        Json::Value root;
        Json::Reader reader;
        reader.parse(tile.second.json, root);
        std::string key = root.getMemberNames()[0];
        root[key]["populate"] = populate;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "    ";
        std::string json = Json::writeString(builder, root);
        json.resize(json.size() > 1 ? json.size()-2 : 0);  // } and \n
        std::print("{}{}", div, json.size() > 1 ? json.c_str()+1 : json.c_str());
        div = ",\n";
    }
    std::print("\n}}\n");
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::print("Usage: [--xray] TileGrid <in_file>\n");
        return 1;
    }

    _setmode(_fileno(stdout), _O_BINARY);

    try {
        if (argc == 2) {
            importTileGrid(argv[1]);
        }
        if (argc == 3 && std::string(argv[1]) == "--xray") {
            importTileGrid(argv[2], true);
        }
    } catch (const std::runtime_error& err) {
        std::print(stderr, "ERROR: {}\n", err.what());
    }

    return 0;
}
