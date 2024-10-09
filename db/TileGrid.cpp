#include <print>
#include <fstream>
#include <iostream>
#include <vector>
#include "json/json.h"
#include <string_view>
#include <spanstream>
#include <unordered_map>
#include <format>

#define GEAR_DEBUG(a...) {} // { std::print(a) }
#define GEAR_WARNING(a...) { std::print("WARNING: " a); }

int sscan(std::string_view data, const std::string_view format)
{
    return 0;
}

template <typename T, typename... Args>
int sscan(std::string_view data, const std::string_view format, T& var, Args&... args)
{
    // find needle before {} and search for it in data
    size_t format_first;
    if ((format_first = format.find("{}")) == (size_t)-1) {
        return 0;
    }
    size_t data_first;
    if ((data_first = data.find(&format.front(), 0, format_first)) == (size_t)-1) {
        return 0;
    }
    // try to find next {}, take a next needle between {} and {} and search for it in data
    size_t search_next = (size_t)-1;
    size_t found_next = (size_t)-1;
    size_t found_size;
    if (format.size() != format_first + 2 && (search_next = format.find("{}", format_first + 2)) != (size_t)-1) {
        found_size = search_next - format_first - 2;
        found_next = data.find(&format.front() + format_first + 2, data_first + 1, found_size);
    }
    // if we found both needles - take data between them, if just a one - take whole data after first
    std::string_view part(&data.front() + data_first, found_next == (size_t)-1 ? (data.size() - data_first) : (found_next - data_first));
    std::ispanstream ss(part);
    if (!(ss >> var)) {
        return 0;
    }
    if (found_next != (size_t)-1) {
        return sscan(std::string_view(&data.front() + found_next + found_size, data.size() - found_next - found_size),
                            std::string_view(&format.front() + search_next, format.size() - search_next),
                            args...) + 1;
    }
    else {
        return 1;  // all data was eaten by current {}
    }
}


struct Coord
{
    int x = -1, y = -1;
};

struct Range
{
    int a = -1, b = -1;
};

struct Rect
{
    Coord a, b;
    Coord name;  // name coords
    std::vector<Range> more_x;

    int width()
    {
        return b.x - a.x;
    }

    int height()
    {
        return b.y - a.y;
    }
};

struct RectAssembler
{
    std::vector<Rect> rects;

    void apply()
    {
        Rect& line = rects.back();
        GEAR_DEBUG("applying line ({}){}-{}\n", line.a.x, line.a.y, line.b.y);
        for (size_t i=0; i < rects.size() - 1; ++i) {
            bool found_alignment = false;
            if (rects[i].a.y == line.a.y && rects[i].b.y <= line.b.y)  // aligned by bottoms
            {
                GEAR_DEBUG("adding line ({}){}-{} to rect ({},{})-({},{})\n", line.a.x, line.a.y, line.b.y, rects[i].a.x, rects[i].a.y, rects[i].b.x, rects[i].b.y);
                // xor lines
                line.a.y = rects[i].b.y + 1;
                if (line.a.y > line.b.y) {
                    rects.resize(rects.size()-1);
                }
                found_alignment = true;
            }
            if (rects[i].b.y == line.b.y && rects[i].a.y >= line.a.y)  // aligned by tops
            {
                GEAR_DEBUG("adding line ({}){}-{} to rect ({},{})-({},{})\n", line.a.x, line.a.y, line.b.y, rects[i].a.x, rects[i].a.y, rects[i].b.x, rects[i].b.y);
                // xor lines
                line.b.y = rects[i].a.y - 1;
                if (line.b.y < line.a.y) {
                    rects.resize(rects.size()-1);
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
                    else
                    if (rects[i].more_x.size() && rects[i].more_x.back().a == line.a.x - 1) {
                        rects[i].more_x.back().b = std::max(line.a.x, rects[i].more_x.back().b);  // also considering Range.b == -1
                    }
                    else {
                        rects[i].more_x.push_back({line.a.x, -1});
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
        if (rects.back().b.x == grid.x) {
            if (grid.y == rects.back().b.y) {  // same
                GEAR_WARNING("same object found\n");
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

struct TileDescription
{
    std::string name;
    RectAssembler ra;
    std::vector<Rect>& rects = ra.rects;
    int y_dir = 0;
};

void ReadTileGrid(std::string filename, size_t start_indent, std::unordered_map<std::string,TileDescription>& tiles)
{
    std::ifstream infile(filename);
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
                    GEAR_DEBUG("{0}_{1}_{2}: {3} {4}\n", name, x, y, root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt());
                    Coord grid = {root[key]["grid_x"].asInt(), root[key]["grid_y"].asInt()};
                    TileDescription& tile = tiles[name];
                    if (!tile.name.size()) {
                        tile.name = name;
                    }
                    tile.ra.put(grid, {x,y});
                    // just some checks for names continuity
                    if (name != prev_name) {
                        prev = {-1,-1};
                        prev_grid = {-1,-1};
                        prev_name = name;
                    }
                    if (grid.x == prev_grid.x && x != prev.x) {
                        GEAR_WARNING("column jump: {}_{}_{} was {}_{}_{}\n", name, x, y, name, prev.x, prev.y);
                    }
                    else {
                        if (grid.x == prev_grid.x && grid.y == prev_grid.y + 1 && y != prev.y + 1) {
                            if (tile.y_dir == 0 && y == prev.y - 1) {
                                tile.y_dir = -1;
                            }
                            else
                            if (tile.y_dir != -1 || y != prev.y - 1) {
                                GEAR_WARNING("row jump\n");
                            }
                        }
                        if (grid.x == prev_grid.x && grid.y == prev_grid.y - 1 && y != prev.y - 1) {
                            if (tile.y_dir == 0 && y == prev.y + 1) {
                                tile.y_dir = 1;
                            }
                            else
                            if (tile.y_dir != 1 || y != prev.y + 1) {
                                GEAR_WARNING("row jump\n");
                            }
                        }
                    }
                    prev = {x, y};
                    prev_grid = grid;
                }
                else {
                    GEAR_WARNING("cant scan name, skipping\n");
                }
                tile_json = "{";
            }
        }
    }
}


template<>
struct std::formatter<Rect, char>
{
    bool quoted = false;

    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(Rect r, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        if (r.a.x == r.b.x && r.a.y == r.b.y) {
            std::format_to(ctx.out(), "{}:{}", r.a.x, r.a.y);
        }
        else
        if (r.a.x == r.b.x) {
            std::format_to(ctx.out(), "{}:({}-{})", r.a.x, r.a.y, r.b.y);
        }
        else
        if (r.a.y == r.b.y) {
            std::format_to(ctx.out(), "({}-{}):{}", r.a.x, r.b.x, r.a.y);
        }
        else {
            std::format_to(ctx.out(), "({}:{})-({}:{})", r.a.x, r.a.y, r.b.x, r.b.y);
        }
        if (r.more_x.size()) {
            for (auto range : r.more_x) {
                if (range.b == -1) {
                    std::format_to(ctx.out(), "|{}", range.a);
                }
                else {
                    std::format_to(ctx.out(), "|{}-{}", range.a, range.b);
                }
            }
        }
        return ret;
    }
};


int main()
{
    std::unordered_map<std::string,TileDescription> tiles;
    ReadTileGrid("tilegrid.json", 4, tiles);
    for (auto tile : tiles) {
        std::print("{}: ", tile.second.name);
        std::string separator = " ";
        for (auto rect : tile.second.rects) {
            std::print("{}{}", separator, rect);
            separator = ", ";
        }
        std::print("\n");
    }
    return 0;
}

