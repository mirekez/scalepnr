#pragma once

#include <print>
#include <fstream>
#include <iostream>
#include <vector>
#include "json/json.h"
#include <map>

#include "Types.h"
#include "debug.h"
#include "Io.h"
#include "Formatting.h"
#include "sscan.h"

using namespace gear;

struct RectAssembler
{
    std::vector<RectEx> rects;

    void apply()
    {
        RectEx& line = rects.back();
        PNR_LOG2("IOTG", "applying line {}\n", line);
        for (size_t i=0; i < rects.size() - 1; ++i) {
            bool found_alignment = false;
            if (rects[i].y.a == line.y.a && rects[i].y.b <= line.y.b)  // aligned by bottoms
            {
                PNR_LOG1("IOTG", "adding line {} to rect {}\n", line, rects[i]);
                // xor lines
                line.y.a = rects[i].y.b + 1;
                if (line.y.a > line.y.b) {
                    rects.pop_back();
                }
                found_alignment = true;
            }
            if (rects[i].y.b == line.y.b && rects[i].y.a >= line.y.a)  // aligned by tops
            {
                PNR_LOG1("IOTG", "adding line {} to rect {}\n", line, rects[i]);
                // xor lines
                line.y.b = rects[i].y.a - 1;
                if (line.y.b < line.y.a) {
                    rects.pop_back();
                }
                found_alignment = true;
            }
            if (found_alignment) {  // add x to rect
                if (rects[i].x.b == line.x.a - 1) {  // continuous x
                    rects[i].x.b = line.x.a;
                }
                else {  // discontinuous x
                    if (rects[i].more_x.size() && rects[i].more_x.back().b == line.x.a - 1) {
                        ++rects[i].more_x.back().b;
                    }
                    else {
                        rects[i].more_x.push_back({line.x.a, line.x.a, line.name.x});
                    }
                }
            }
        }
    }

    void put(Coord grid, Coord name)
    {
        if (rects.empty()) {
            rects.push_back({Range{grid.x, grid.x}, Range{grid.y, grid.y}, name});
            return;
        }
        if (grid.x == rects.back().x.b) {
            if (grid.y == rects.back().y.b) {  // same
                PNR_WARNING("overlapping Tile found: {} == {}\n", grid, rects.back());
            }
            else
            if (grid.y == rects.back().y.b + 1) {  // next
                rects.back().y.b = grid.y;
            }
            else
            if (grid.y == rects.back().y.a - 1) {  // next
                rects.back().y.a = grid.y;
            }
            else {  // other row
                apply();
                rects.push_back({Range{grid.x, grid.x}, Range{grid.y, grid.y}, name});
            }
        }  // other col
        else {
            apply();
            rects.push_back({Range{grid.x, grid.x}, Range{grid.y, grid.y}, name});
        }
    }
};

struct TileGridSpec
{
    std::string name;
    RectAssembler ra;
    std::vector<RectEx>& rects = ra.rects;
    int y_dir = 0;
    std::string json;
    int name_x = -1;
};

inline Coord readXrayTileGrid(const std::string& filename, size_t start_indent, std::map<std::string,TileGridSpec>& tiles)
{
    Coord size;
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    std::string tile_json = "{";

    Coord prev, prev_grid;  // just to check names continuity
    std::string prev_name;

    while (std::getline(infile, line)) {
        size_t indent = 0;
        for (char ch : line) {
            if (ch == ' ') {
                ++indent;
            }
            else break;
        }
        if (indent >= start_indent) {
            tile_json += line.c_str() + indent;
            if (line[start_indent] == '}') {  // we collected all object
                tile_json.pop_back();
                tile_json += '}';
                Json::Value root;
                Json::Reader reader;
                reader.parse(tile_json, root);
                std::string key = root.getMemberNames()[0];
                std::string name;
                int x, y;
                if (sscan(key, "{}_X{}Y{}", name, x, y) == 3) {
                    PNR_LOG3("IOTG", "{0}_{1}_{2}: {3} {4}, ", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt());
                    Coord grid = {root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt()};
                    if (grid.x > size.x) {
                        size.x = grid.x;
                    }
                    if (grid.y > size.y) {
                        size.y = grid.y;
                    }
                    TileGridSpec& tile = tiles[name];
                    if (!tile.name.size()) {
                        tile.name = name;
                        tile.json = tile_json;
                        tile.name_x = x;
                    }
                    tile.ra.put(grid, {x,y});
                    // just some checks for names continuity
                    if (name != prev_name) {
                        prev = {-1,-1};
                        prev_grid = {-1,-1};
                        prev_name = name;
                    }
                    if (grid.x == prev_grid.x && x != prev.x) {
                        PNR_WARNING("column jump: {}_{}_{} was {}_{}_{}", name, x, y, name, prev.x, prev.y);
                    }
                    else {
                        if (grid.x == prev_grid.x && grid.y == prev_grid.y + 1 && y != prev.y + 1) {
                            if (tile.y_dir == 0 && y == prev.y - 1) {
                                tile.y_dir = -1;
                            }
                            else
                            if (tile.y_dir != -1 || y != prev.y - 1) {
                                PNR_WARNING("row jump\n");
                            }
                        }
                        if (grid.x == prev_grid.x && grid.y == prev_grid.y - 1 && y != prev.y - 1) {
                            if (tile.y_dir == 0 && y == prev.y + 1) {
                                tile.y_dir = 1;
                            }
                            else
                            if (tile.y_dir != 1 || y != prev.y + 1) {
                                PNR_WARNING("row jump\n");
                            }
                        }
                    }
                    prev = {x, y};
                    prev_grid = grid;
                }
                else {
                    PNR_WARNING("cant scan name, skipping\n");
                }
                tile_json = "{";
            }
        }
    }
    return {size.x+1, size.y+1};
}

inline Coord readTileGrid(const std::string& filename, size_t start_indent, std::map<std::string,TileGridSpec>& tiles)
{
    Coord size;
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    std::string tile_json = "{";

    while (std::getline(infile, line)) {
        size_t indent = 0;
        for (char ch : line) {
            if (ch == ' ') {
                ++indent;
            }
            else break;
        }
        if (indent >= start_indent) {
            tile_json += line.c_str() + indent;
            if (line[start_indent] == '}') {  // we collected all object
                tile_json.pop_back();  // comma
                tile_json += '}';
                Json::Value root;
                Json::Reader reader;
                reader.parse(tile_json, root);
                std::string key = root.getMemberNames()[0];
                std::string name;
                int x, y;
                if (sscan(key, "{}_X{}Y{}", name, x, y) == 3) {
                    TileGridSpec& tile = tiles[name];
                    if (!tile.name.size()) {
                        tile.name = key;
                        tile.json = tile_json;
                        tile.name_x = x;
                    }
                    std::string populate = root[key]["populate"].asString();
                    PNR_LOG2("IOTG", "{0}_{1}_{2}, grid: {3}:{4}, populate: {5}... ", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt(), populate);

                    std::string_view sv(populate);
                    std::ispanstream ss(sv);
//                    std::string line;
//                    while (std::getline(is, line, ',')) {
                    do {
                        while (ss.peek() == (int)' ' && ss.get());
//                        std::string_view sv(line);
//                        std::ispanstream ss(sv);
                        RectEx rect;
                        if (!(ss >> rect)) {
                            break;
                        }
                        PNR_LOG3("IOTG", "{}, ", rect);
                        tile.rects.push_back(rect);
                        if (rect.x.b > size.x) {
                            size.x = rect.x.b;
                        }
                        if (rect.y.b > size.y) {
                            size.y = rect.y.b;
                        }
                        for (const auto& more_x : rect.more_x) {
                            if (more_x.b > size.x) {
                                size.x = more_x.b;
                            }
                        }
                        while (ss.peek() == (int)' ' && ss.ignore(1));
                    } while (ss.get() == (int)',');
                }
                else {
                    PNR_WARNING("cant scan name, skipping\n");
                }
                tile_json = "{";
            }
        }
    }
    return {size.x+1, size.y+1};
}
