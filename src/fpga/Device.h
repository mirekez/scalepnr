#pragma once

#include "TileType.h"
#include "Tile.h"
#include "Wire.h"
#include "Pin.h"
#include "DeviceFormat.h"
#include "debug.h"

#include "referable.h"

#include <set>
#include <unordered_map>

namespace fpga {

struct TileJumpTarget
{
    Tile* tile = nullptr;
    int dst_node = -1;
    int jump_node = -1;
    std::string dst_wire;
};

struct LocalRouteWireMapping
{
    std::string route_type;
    std::string route_wire;
    Coord delta;
};

struct RouteWireGraphEdge
{
    std::string tile_type;
    std::string wire;
    Coord delta;
    bool tileconn = false;
};

struct ParsedTileConnPair
{
    std::string from_wire;
    std::string to_wire;
};

struct ParsedTileConnRule
{
    std::string from_tile_type;
    std::string to_tile_type;
    Coord delta;
    std::vector<ParsedTileConnPair> wire_pairs;
};

struct Device
{
    Device()
    {
        cb_types.reserve(256);
        tile_types.reserve(256);
    }

    TileGridSpec grid_spec;
    TileTypesSpec types_spec;
    std::vector<TileType> tile_types;
    std::vector<CBType> cb_types;
    std::vector<ParsedTileConnRule> tileconn_rules;
    std::vector<Referable<Tile>> tile_grid;
    std::map<Coord,Wire> wires;

    std::map<int,int> x_to_grid;
    std::map<int,int> y_to_grid;
    std::vector<Pin> pins;
    std::unordered_map<std::string, std::vector<LocalRouteWireMapping>> local_route_wire_mappings;
    std::unordered_map<std::string, std::vector<RouteWireGraphEdge>> route_wire_graph;
    int size_width = 0;
    int size_height = 0;
    int cnt_regs = 0;
    int cnt_luts = 0;
    std::vector<std::vector<Referable<Wire>>> wire_grid;
//tilegrid.json
    void loadFromSpec(const std::string& spec_name, const std::string& pins_spec_name);
    void loadTypeFromSpec(const std::string& spec_name, TechMap& map);
    void loadCBFromSpec(const std::string& spec_name, TechMap& map);
    void loadTileConnFromSpec(const std::string& spec_name);
    void applyTileConnSubtypes();
    Tile* getTile(int x, int y);
    TileJumpTarget resolveJump(const Tile& from, int src_node) const;
    std::vector<TileJumpTarget> resolveJumpTargets(const Tile& from, int src_node) const;
    TileJumpTarget resolveJumpToward(const Tile& from, int src_node, const Coord& target) const;

    static Device& current();
};

int testRouteSrcNodeByPhysicalWireName(CBType& cb_type, const std::string& wire);
int testRouteDstNodeByPhysicalWireName(CBType& cb_type, const std::string& wire);

}
