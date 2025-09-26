#include <print>
#include <fstream>
#include <iostream>
//#include <io.h>
//#include <fcntl.h>
#include <vector>
#include "json/json.h"
#include <map>

#include "debug.h"
#include "Types.h"
#include "DeviceFormat.h"

void importTileGrid(const std::string& in_name, const std::string& out_name, bool xray = false)
{
    TileGridSpec spec;
    std::map<std::string,TileSpec> tiles;
    if (xray) {
        readXrayTileGrid(in_name, &tiles, &spec);
    }
    else {
        readTileGrid(in_name, &tiles, &spec);
    }

    std::ofstream outfile;
    outfile.open(out_name, std::ios_base::binary);
    if (!outfile) {
        throw std::runtime_error(std::string("cant open file: ") + out_name);
    }
//    _setmode(_fileno(stdout), _O_BINARY);

    std::print(outfile, "{{\n");
    std::string div = "";
    for (const auto& tile : tiles) {

        std::map<int, RectEx> sortRects;
        for (const auto& rect : tile.second.rects) {
            sortRects[rect.x.a] = rect;
        }

        std::string populate = "";
        std::string separator = "";
        for (const auto& rect : sortRects) {
            populate += std::format("{}{}", separator, rect.second);
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
        std::print(outfile, "{}{}", div, json.size() > 1 ? json.c_str()+1 : json.c_str());
        div = ",\n";
    }
    std::print(outfile, "\n}}\n");
    outfile.close();
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::print("Usage: [--xray] TileGrid <in_file> <out_file>\n");
        return 1;
    }

    try {
        if (argc == 3) {
            importTileGrid(argv[1], argv[2]);
        }
        if (argc == 4 && std::string(argv[1]) == "--xray") {
            importTileGrid(argv[2], argv[3], true);
        }
    } catch (const std::runtime_error& err) {
        std::print(stderr, "ERROR: {}\n", err.what());
    }

    return 0;
}
