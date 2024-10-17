#pragma once

#include <print>
#include <fstream>
#include <iostream>
#include <vector>
#include "json/json.h"
#include <map>

#include "Types.h"
#include "debug.h"
#include "Formatting.h"
#include "sscan.h"

using namespace gear;

struct RectAssembler
{
    std::vector<RectEx> rects;

    void apply()
    {
        RectEx& line = rects.back();
        PNR_DEBUG("applying line ({}){}-{}\n", line.a.x, line.a.y, line.b.y);
        for (size_t i=0; i < rects.size() - 1; ++i) {
            bool found_alignment = false;
            if (rects[i].a.y == line.a.y && rects[i].b.y <= line.b.y)  // aligned by bottoms
            {
                PNR_DEBUG("adding line ({}){}-{} to rect ({},{})-({},{})\n", line.a.x, line.a.y, line.b.y, rects[i].a.x, rects[i].a.y, rects[i].b.x, rects[i].b.y);
                // xor lines
                line.a.y = rects[i].b.y + 1;
                if (line.a.y > line.b.y) {
                    rects.pop_back();
                }
                found_alignment = true;
            }
            if (rects[i].b.y == line.b.y && rects[i].a.y >= line.a.y)  // aligned by tops
            {
                PNR_DEBUG("adding line ({}){}-{} to rect ({},{})-({},{})\n", line.a.x, line.a.y, line.b.y, rects[i].a.x, rects[i].a.y, rects[i].b.x, rects[i].b.y);
                // xor lines
                line.b.y = rects[i].a.y - 1;
                if (line.b.y < line.a.y) {
                    rects.pop_back();
                }
                found_alignment = true;
            }
            if (found_alignment) {  // add x to rect
                if (rects[i].b.x == line.a.x - 1) {  // continuous x
                    rects[i].b.x = line.a.x;
                }
                else {  // discontinuous x
                    if (rects[i].more_x.size() && rects[i].more_x.back().b == line.a.x - 1) {
                        ++rects[i].more_x.back().b;
                    }
                    else {
                        rects[i].more_x.push_back({line.a.x, line.a.x});
                    }
                }
            }
        }
    }

    void put(Coord grid, Coord name)
    {
        if (rects.empty()) {
            rects.push_back({grid, grid, name});
            return;
        }
        if (grid.x == rects.back().b.x) {
            if (grid.y == rects.back().b.y) {  // same
                PNR_WARNING("same Tile found: {} == ({},{})\n", grid, rects.back().b.x, rects.back().b.y);
            }
            else
            if (grid.y == rects.back().b.y + 1) {  // next
                rects.back().b.y = grid.y;
            }
            else
            if (grid.y == rects.back().a.y - 1) {  // next
                rects.back().a.y = grid.y;
            }
            else {  // other row
                apply();
                rects.push_back({grid, grid, name});
            }
        }  // other col
        else {
            apply();
            rects.push_back({grid, grid, name});
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
                    PNR_DEBUG("{0}_{1}_{2}: {3} {4}\n", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt());
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
                    }
                    tile.ra.put(grid, {x,y});
                    // just some checks for names continuity
                    if (name != prev_name) {
                        prev = {-1,-1};
                        prev_grid = {-1,-1};
                        prev_name = name;
                    }
                    if (grid.x == prev_grid.x && x != prev.x) {
                        PNR_WARNING("column jump: {}_{}_{} was {}_{}_{}\n", name, x, y, name, prev.x, prev.y);
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
                    PNR_DEBUG("{0}_{1}_{2}: {3} {4}\n", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt());
                    TileGridSpec& tile = tiles[name];
                    if (!tile.name.size()) {
                        tile.name = key;
                        tile.json = tile_json;
                    }
                    std::stringstream is1(std::move(root[key]["populate"].asString()));
                    std::string line1;
                    while (std::getline(is1, line1, ',')) {
                        RectEx rect;
                        scanRect(std::move(line1), rect);
                        tile.rects.push_back(rect);
                        if (rect.b.x > size.x) {
                            size.x = rect.b.x;
                        }
                        if (rect.b.y > size.y) {
                            size.y = rect.b.y;
                        }
                        for (const auto& more_x : rect.more_x) {
                            if (more_x.b > size.x) {
                                size.x = more_x.b;
                            }
                        }
//                        std::print("\n");
                    }
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
