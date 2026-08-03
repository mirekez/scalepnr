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

struct TileLocalTarget
{
    Tile* tile = nullptr;
    CBNodeNameType node_type = CB_NODE_LOCAL;
    int node = -1;
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
    bool routable = true;
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

// Records subtype size and phase costs so database regressions are testable.
struct SubtypeBuildStats
{
    size_t initial_types = 0;
    size_t final_types = 0;
    size_t created_subtypes = 0;
    size_t specialized_tiles = 0;
    size_t signature_hits = 0;
    size_t signature_misses = 0;
    size_t target_calls = 0;
    size_t target_success = 0;
    size_t target_failed = 0;
    size_t target_search_pops = 0;
    size_t edge_calls = 0;
    size_t edge_results = 0;
    double signature_seconds = 0.0;
    double candidate_copy_seconds = 0.0;
    double mapping_seconds = 0.0;
    double target_search_seconds = 0.0;
    double dedup_seconds = 0.0;
    double elapsed_seconds = 0.0;
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
    SubtypeBuildStats last_subtype_build;
    std::vector<Referable<Tile>> tile_grid;
    std::map<Coord,Wire> wires;

    std::map<int,int> x_to_grid;
    std::map<int,int> y_to_grid;
    std::vector<Pin> pins;
    std::unordered_map<std::string, std::vector<LocalRouteWireMapping>> local_route_wire_mappings;
    std::unordered_map<std::string, std::vector<RouteWireGraphEdge>> route_wire_graph;
    std::set<std::string> deferred_cb_types;
    int size_width = 0;
    int size_height = 0;
    int cnt_regs = 0;
    int cnt_luts = 0;
    std::vector<std::vector<Referable<Wire>>> wire_grid;
//tilegrid.json
    void loadFromSpec(const std::string& spec_name, const std::string& pins_spec_name);
    void loadTypeFromSpec(const std::string& spec_name, TechMap& map);
    void loadCBFromSpec(const std::string& spec_name, TechMap& map,
                        bool local_fabric = false,
                        const std::vector<std::string>& constant_one_nodes = {});
    void loadTileConnFromSpec(const std::string& spec_name);
    void rebuildLocalTransitions();
    void activateDeferredCBTypes();
    void applyTileConnSubtypes();
    void rebuildIncomingDstMasks();
    Tile* getTile(int x, int y);
    // Return the unique grid tile that owns this tile's crossbar state.
    Tile* routeTile(const Tile& tile);
    TileJumpTarget resolveJump(const Tile& from, int src_node) const;
    std::vector<TileJumpTarget> resolveJumpTargets(const Tile& from, int src_node) const;
    TileJumpTarget resolveJumpToward(const Tile& from, int src_node, const Coord& target) const;
    std::vector<TileLocalTarget> resolveLocalTargets(const Tile& from, int local_node) const;

    static Device& current();
};

int testRouteSrcNodeByPhysicalWireName(CBType& cb_type, const std::string& wire);
int testRouteDstNodeByPhysicalWireName(CBType& cb_type, const std::string& wire);
std::string testRouteWireFamilyKey(const std::string& wire);

}
