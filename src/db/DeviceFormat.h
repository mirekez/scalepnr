#pragma once

#include <print>
#include <fstream>
#include <iostream>
#include <vector>
#include "json/json.h"
#include <map>

#include "Types.h"
#include "debug.h"
#include "formatting.h"
#include "sscan.h"

using namespace fpga;

struct RectAssembler
{
    std::vector<RectEx> rects;

    void apply()
    {
        RectEx& line = rects.back();
        PNR_LOG2("FRMT", "applying line {}\n", line);
        for (size_t i=0; i < rects.size() - 1; ++i) {
            bool found_alignment = false;
            if (rects[i].y.a == line.y.a && rects[i].y.b <= line.y.b)  // aligned by bottoms
            {
                PNR_LOG1("FRMT", "adding line {} to rect {}\n", line, rects[i]);
                // xor lines
                line.y.a = rects[i].y.b + 1;
                if (line.y.a > line.y.b) {
                    rects.pop_back();
                }
                found_alignment = true;
            }
            if (rects[i].y.b == line.y.b && rects[i].y.a >= line.y.a)  // aligned by tops
            {
                PNR_LOG1("FRMT", "adding line {} to rect {}\n", line, rects[i]);
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

struct TileSpec
{
    std::string name;
    RectAssembler ra;
    std::vector<RectEx>& rects = ra.rects;
    int y_dir = 0;  // just how they follow in JSON file
    std::string json;
    int nameX = -1;
};

struct TileGridSpec
{
    Coord size;
    int naming_dir;  // Y naming direction
};

inline bool readTileGrid(const std::string& filename, std::map<std::string,TileSpec>* tiles, TileGridSpec* spec)
{
    PNR_LOG1("FRMT", "readTileGrid from {}", filename);
    const size_t start_indent = 4;
    spec->size = {-1,-1};
    spec->naming_dir = -1;
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    std::string tile_json = "{";

    Coord prev, prev_grid;  // just to check names continuity
    std::string prev_name;
    int line_number = -1;
    while (std::getline(infile, line)) {
        ++line_number;
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
                if (tile_json.back() == ',') {
                    tile_json.pop_back();
                }
                tile_json += '}';
                std::string key;
                Json::Value root;
                Json::Reader reader;
                try {
                    reader.parse(tile_json, root);
                    if (!root.getMemberNames().empty()) {
                        key = root.getMemberNames()[0];
                    }
                }
                catch (Json::Exception& ex) {
                    PNR_ERROR("readTileGrid('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                    return false;
                }

                std::string name;
                int x, y;
                if (sscan(key, "{}_X{}Y{}", &name, &x, &y) == 3) {
                    Coord grid;
                    try {
                        PNR_LOG3("FRMT", " {}_{}_{}:{}/{}", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt());
                        grid = {root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt()};
                    }
                    catch (Json::Exception& ex) {
                        PNR_ERROR("readTileGrid('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                        return false;
                    }
                    if (grid.x > spec->size.x) {
                        spec->size.x = grid.x;
                    }
                    if (grid.y > spec->size.y) {
                        spec->size.y = grid.y;
                    }
                    TileSpec& tile = (*tiles)[name];
                    if (!tile.name.size()) {
                        tile.name = name;
                        tile.json = tile_json;
                        tile.nameX = x;
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
    spec->size += {1,1};
    return true;
}

inline bool readTileGrid1(const std::string& filename, std::map<std::string,TileSpec>* tiles, TileGridSpec* spec)
{
    PNR_LOG1("FRMT", "readTileGrid1 from {}", filename);
    const size_t start_indent = 4;
    spec->size = {-1,-1};
    spec->naming_dir = -1;
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    std::string tile_json = "{";
    int line_number = -1;
    while (std::getline(infile, line)) {
        ++line_number;
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
                if (tile_json.back() == ',') {
                    tile_json.pop_back();
                }
                tile_json += '}';
                std::string key;
                std::string name;
                Json::Value root;
                Json::Reader reader;
                try {
                    reader.parse(tile_json, root);
                    if (!root.getMemberNames().empty()) {
                        key = root.getMemberNames()[0];
                    }
                }
                catch (Json::Exception& ex) {
                    PNR_ERROR("readTileGrid('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                    return false;
                }

                int x, y;  // like AAAAA_B_X2Y0
                if (sscan(key, "{}_X{}Y{}", &name, &x, &y) == 3) {
                    TileSpec& tile = (*tiles)[name];
                    if (!tile.name.size()) {
                        tile.name = key;
                        tile.json = tile_json;
                        tile.nameX = x;
                    }
                    std::string populate;
                    try {
                        populate = root[key]["populate"].asString();
                        PNR_LOG2("FRMT", "{}_{}_{}, grid: {}:{}, populate: {}...", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt(), populate);
                    }
                    catch (Json::Exception& ex) {
                        PNR_ERROR("readTileGrid('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                        return false;
                    }

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
                        PNR_LOG3("FRMT", " {}", rect);
                        tile.rects.push_back(rect);
                        if (rect.x.b > spec->size.x) {
                            spec->size.x = rect.x.b;
                        }
                        if (rect.y.b > spec->size.y) {
                            spec->size.y = rect.y.b;
                        }
                        for (const auto& more_x : rect.more_x) {
                            if (more_x.b > spec->size.x) {
                                spec->size.x = more_x.b;
                            }
                        }
                        while (ss.peek() == (int)' ' && ss.ignore(1));
                    } while (ss.get() == (int)',');
                }
                else {
                    PNR_WARNING("cant scan name '{}', skipping\n", key);
                }
                tile_json = "{";
            }
        }
    }
    spec->size += {1,1};
    return true;
}

inline bool get_line(std::string_view& sv, std::string_view& line, char delimiter = '\n')
{
    if (sv.empty()) {
        return false;
    }

    size_t pos = sv.find(delimiter);
    if (pos == std::string_view::npos) {
        line = sv;
        sv = {};
    } else {
        line = sv.substr(0, pos);
        sv.remove_prefix(pos + 1);
    }
    return true;
}

struct PinSpec
{
    std::string name;
    std::string bank;
    std::string site;
    std::string tile;
    std::string function;
    Coord pos;
};

inline bool readPackagePins(const std::string& filename, std::vector<PinSpec>& spec)
{
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    int line_number = -1;
    while (std::getline(infile, line)) {
        ++line_number;
        if (line_number == 1) {
            continue;
        }
        PinSpec pin;

        std::string_view view = line;
        std::string_view token;
        if (!get_line(view, token, ',')) {
            return false;
        }
        pin.name = token;
        if (!get_line(view, token, ',')) {
            return false;
        }
        pin.bank = token;
        if (!get_line(view, token, ',')) {
            return false;
        }
        pin.site = token;
        if (!get_line(view, token, ',')) {
            return false;
        }
        pin.tile = token;
        if (!get_line(view, token, ',')) {
            return false;
        }
        pin.function = token;

        std::string name;
        int x = -1, y = -1;
        if (sscan(pin.tile, "{}_X{}Y{}", &name, &x, &y) == 3) {
            pin.pos.x = x;
            pin.pos.y = y;
        }

        PNR_LOG2("FRMT", "readPackagePins, '{}' '{}' '{}' '{}' '{}': X{}Y{}", pin.name, pin.bank, pin.site, pin.tile, pin.function, x, y);
        spec.emplace_back(std::move(pin));
    }
    return true;
}

struct TileTypesSpec
{
    std::map<Coord,std::string> types;
};

struct TypeSpec
{
    std::multimap<std::string,std::string> nodes;
};

inline bool readTypes(const std::string& filename, std::map<std::string,TypeSpec>* types, TileTypesSpec* spec)
{
    PNR_LOG1("FRMT", "readTypes from '{}'", filename);
    std::multimap<std::string,std::string> tmp;

    const size_t start_indent = 8;
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    std::string wire_json = "{";
    int line_number = -1;
    while (std::getline(infile, line)) {
        ++line_number;
        size_t indent = 0;
        for (char ch : line) {
            if (ch == ' ') {
                ++indent;
            }
            else break;
        }
        if (line.find("\"tile_type\":") != (size_t)-1) {
            std::string a, b, c;
            if (sscan(line, "{}\"{}\": \"{}\",", &c, &a, &b) == 3) {  // "tile_type": "INT_L",
                PNR_LOG2("FRMT", "{} node connections in '{}'", tmp.size(), b);
                types->emplace(b, TypeSpec{std::move(tmp)});
            }
        }
        if (indent >= start_indent) {
            wire_json += line.c_str() + indent;
            if (line[start_indent] == '}') {  // we collected all object
                if (wire_json.back() == ',') {
                    wire_json.pop_back();
                }
                wire_json += '}';
                std::string key;
                Json::Value root;
                Json::Reader reader;
                try {
                    reader.parse(wire_json, root);
                    if (!root.getMemberNames().empty()) {
                        key = root.getMemberNames()[0];
                    }
                }
                catch (Json::Exception& ex) {
                    PNR_ERROR("readwireGrid('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                    return false;
                }

                std::string a, b, c;
                if (sscan(key, "{}.{}->>{}", &c, &a, &b) == 3) {
                    PNR_LOG3("FRMT", "'{}'->'{}'", a, b);
                    tmp.emplace(a,std::move(b));
                }
//                else {
//                    PNR_WARNING("cant scan node {}, skipping\n", key);
//                }
                wire_json = "{";
            }
        }
    }
    return true;
}

struct CBTypeSpec
{
    std::multimap<std::string,std::string> nodes;
};

inline bool readCBTypes(const std::string& filename, std::map<std::string,CBTypeSpec>* cbs, TileTypesSpec* spec)
{
    PNR_LOG1("FRMT", "readCBTypes from '{}'", filename);
    std::multimap<std::string,std::string> tmp;

    const size_t start_indent = 8;
    std::ifstream infile(filename);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + filename);
    }
    std::string line;
    std::string wire_json = "{";
    int line_number = -1;
    while (std::getline(infile, line)) {
        ++line_number;
        size_t indent = 0;
        for (char ch : line) {
            if (ch == ' ') {
                ++indent;
            }
            else break;
        }
        if (line.find("\"tile_type\":") != (size_t)-1) {
            std::string a, b, c;
            if (sscan(line, "{}\"{}\": \"{}\",", &c, &a, &b) == 3) {  // "tile_type": "INT_L",
                PNR_LOG2("FRMT", "{} node connections in '{}'", tmp.size(), b);
                cbs->emplace(b, CBTypeSpec{std::move(tmp)});
            }
        }
        if (indent >= start_indent) {
            wire_json += line.c_str() + indent;
            if (line[start_indent] == '}') {  // we collected all object
                if (wire_json.back() == ',') {
                    wire_json.pop_back();
                }
                wire_json += '}';
                std::string key;
                Json::Value root;
                Json::Reader reader;
                try {
                    reader.parse(wire_json, root);
                    if (!root.getMemberNames().empty()) {
                        key = root.getMemberNames()[0];
                    }
                }
                catch (Json::Exception& ex) {
                    PNR_ERROR("readwireGrid('{}') cant parse JSON at line {}, exception: '{}'", filename, line_number, ex.what());
                    return false;
                }

                std::string a, b, c;
                if (sscan(key, "{}.{}->>{}", &c, &a, &b) == 3) {
                    PNR_LOG3("FRMT", "'{}'->'{}'", a, b);
                    tmp.emplace(a, b);
                }
//                else {
//                    PNR_WARNING("cant scan node {}, skipping\n", key);
//                }
                wire_json = "{";
            }
        }
    }
    return true;
}
