#include "Device.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <filesystem>
#include <fstream>
#include <unordered_set>
#include <unordered_map>

using namespace fpga;

namespace technology {
#if defined(__GNUC__)
std::vector<std::pair<std::string, std::string>> mappedRouteEndpointAliases(
    const std::string& tile_type, const std::string& pin, int site_pos, const std::string& wire) __attribute__((weak));
#else
std::vector<std::pair<std::string, std::string>> mappedRouteEndpointAliases(
    const std::string& tile_type, const std::string& pin, int site_pos, const std::string& wire);
#endif
}

namespace {

TileType* tileTypeFor(std::vector<TileType>& tile_types, const std::string& name)
{
    for (TileType& type : tile_types) {
        if (type.name == name) {
            return &type;
        }
    }
    return nullptr;
}

void addResolvedJumpBit(std::vector<CBType::ResolvedJump>& entries, Coord delta,
                        uint16_t target_cb_type_id, int dst_node,
                        const std::string& source_cb_type, int src_node,
                        const std::string& target_cb_type)
{
    if (target_cb_type_id == CB_INVALID_TYPE_ID || dst_node < 0 || dst_node >= CB_MAX_NODES) {
        return;
    }
    NodeMask bit = NodeMask{0,1} << dst_node;
    for (CBType::ResolvedJump& entry : entries) {
        if (entry.target_cb_type_id != target_cb_type_id) {
            continue;
        }
        if ((entry.dsts.jump & bit) != NodeMask{} && (entry.delta.x != delta.x || entry.delta.y != delta.y)) {
            static int logged_multi_delta = 0;
            if (logged_multi_delta < 16) {
                PNR_LOG1("FPGA",
                    "tileconn resolved same jump relation with another delta: source_cb='{}' src={} target_cb='{}' dst={} old=({}, {}) new=({}, {})",
                    source_cb_type, src_node, target_cb_type, dst_node, entry.delta.x, entry.delta.y, delta.x, delta.y);
                ++logged_multi_delta;
            }
        }
        if (entry.delta.x == delta.x && entry.delta.y == delta.y) {
            entry.dsts.jump |= bit;
            return;
        }
    }
    CBJumpState dsts{};
    dsts.jump = bit;
    entries.push_back(CBType::ResolvedJump{delta, target_cb_type_id, dsts});
}

int countBits(const NodeMask& mask)
{
    int count = 0;
    mask.for_each_set_bit([&](int) {
        ++count;
        return false;
    });
    return count;
}

int decodeJumpDelta4(int value)
{
    value &= 0xf;
    return value & 0x8 ? value - 16 : value;
}

Coord jumpDeltaFromNode(int jump)
{
    return Coord{decodeJumpDelta4((jump >> 6) & 0xf), decodeJumpDelta4((jump >> 2) & 0xf)};
}

bool debugResolveJumpCoord(const Coord& coord)
{
    const char* text = std::getenv("SCALEPNR_DEBUG_RESOLVE_JUMP");
    if (!text || !*text) {
        return false;
    }
    int x = 0;
    int y = 0;
    if (std::sscanf(text, "%d,%d", &x, &y) != 2) {
        return false;
    }
    return coord.x == x && coord.y == y;
}

bool siteTypeHasPackableElement(const std::string& type)
{
    return type.find("LUT") != std::string::npos
        || type.find("FD") != std::string::npos
        || type.find("CARRY") != std::string::npos
        || type.find("MUX") != std::string::npos
        || type.find("SLICE") != std::string::npos;
}

bool typeSpecHasPackableSite(const TypeSpec& type)
{
    return std::any_of(type.sites.begin(), type.sites.end(), [](const TypeSpec::SiteSpec& site) {
        return siteTypeHasPackableElement(site.type);
    });
}

std::string tileTypePrefix(const std::string& tile_name)
{
    size_t pos = tile_name.rfind("_X");
    return pos == std::string::npos ? tile_name : tile_name.substr(0, pos);
}

bool sameVendorTileCoord(const std::string& a, const std::string& b)
{
    size_t ax = a.rfind("_X");
    size_t bx = b.rfind("_X");
    if (ax == std::string::npos || bx == std::string::npos) {
        return false;
    }
    return a.substr(ax) == b.substr(bx);
}

struct AttachedCbKey
{
    const CBType* cb_type = nullptr;
    int x = 0;
    int y = 0;

    bool operator==(const AttachedCbKey& other) const
    {
        return cb_type == other.cb_type && x == other.x && y == other.y;
    }
};

struct AttachedCbKeyHash
{
    std::size_t operator()(const AttachedCbKey& key) const
    {
        std::size_t ptr = reinterpret_cast<std::size_t>(key.cb_type);
        return (ptr >> 4) ^ (static_cast<std::size_t>(key.x) << 20)
            ^ static_cast<std::size_t>(key.y);
    }
};

struct ResolveJumpCacheKey
{
    const Tile* from = nullptr;
    int src_node = -1;
    int preferred_x = 0;
    int preferred_y = 0;
    bool has_preferred = false;

    bool operator==(const ResolveJumpCacheKey& other) const
    {
        return from == other.from && src_node == other.src_node
            && preferred_x == other.preferred_x && preferred_y == other.preferred_y
            && has_preferred == other.has_preferred;
    }
};

struct ResolveJumpCacheKeyHash
{
    std::size_t operator()(const ResolveJumpCacheKey& key) const
    {
        std::size_t ptr = reinterpret_cast<std::size_t>(key.from);
        std::size_t hash = (ptr >> 4) ^ (static_cast<std::size_t>(key.src_node) << 1);
        hash ^= static_cast<std::size_t>(static_cast<uint16_t>(key.preferred_x)) << 20;
        hash ^= static_cast<std::size_t>(static_cast<uint16_t>(key.preferred_y)) << 36;
        hash ^= key.has_preferred ? 0x9e3779b97f4a7c15ULL : 0;
        return hash;
    }
};

CBType* exactCBTypeFor(std::vector<CBType>& cb_types, const std::string& tile_type_name)
{
    for (CBType& cb_type : cb_types) {
        if (cb_type.name == tile_type_name) {
            return &cb_type;
        }
    }
    return nullptr;
}

const CBType* exactCBTypeFor(const std::vector<CBType>& cb_types, const std::string& tile_type_name)
{
    for (const CBType& cb_type : cb_types) {
        if (cb_type.name == tile_type_name) {
            return &cb_type;
        }
    }
    return nullptr;
}

uint16_t cbTypeIdFor(const std::vector<CBType>& cb_types, const CBType* cb_type)
{
    if (!cb_type) {
        return CB_INVALID_TYPE_ID;
    }
    PNR_ASSERT(cb_type >= cb_types.data() && cb_type < cb_types.data() + cb_types.size(),
        "CBType pointer '{}' is not owned by Device::cb_types\n", cb_type->name);
    size_t index = static_cast<size_t>(cb_type - cb_types.data());
    PNR_ASSERT(index < CB_INVALID_TYPE_ID, "too many CB types: {}\n", index);
    return static_cast<uint16_t>(index);
}

const CBType* cbTypeById(const std::vector<CBType>& cb_types, uint16_t cb_type_id)
{
    if (cb_type_id == CB_INVALID_TYPE_ID || cb_type_id >= cb_types.size()) {
        return nullptr;
    }
    return &cb_types[cb_type_id];
}

int nodeNumByPhysicalWireName(const CBType& cb_type, CBNodeNameType node_type, const std::string& wire)
{
    int node = cb_type.nodeNum(node_type, wire);
    if (node >= 0) {
        return node;
    }

    size_t dot = wire.rfind('.');
    if (dot != std::string::npos) {
        node = cb_type.nodeNum(node_type, wire.substr(dot + 1));
        if (node >= 0) {
            return node;
        }
    }

    std::string type_prefix = cb_type.name + "_";
    if (wire.rfind(type_prefix, 0) == 0) {
        node = cb_type.nodeNum(node_type, wire.substr(type_prefix.size()));
        if (node >= 0) {
            return node;
        }
    }

    for (size_t pos = wire.find('_'); pos != std::string::npos; pos = wire.find('_', pos + 1)) {
        node = cb_type.nodeNum(node_type, wire.substr(pos + 1));
        if (node >= 0) {
            return node;
        }
    }
    return -1;
}

int cbTransitScore(CBType& cb_type)
{
    cb_type.ensureDerivedMasks();
    return countBits(cb_type.valid_dst_nodes);
}

bool routeCapableCBType(const CBType* cb_type)
{
    if (!cb_type) {
        return false;
    }
    return cbTransitScore(*const_cast<CBType*>(cb_type)) > 0;
}

CBType* cbTypeFor(std::vector<CBType>& cb_types,
                  const std::unordered_map<std::string, std::vector<LocalRouteWireMapping>>& local_route_wire_mappings,
                  const std::string& tile_type_name, int grid_x)
{
    static const CBType* cached_data = nullptr;
    static size_t cached_size = 0;
    static std::vector<CBType*> cached_best_types;
    static std::unordered_map<std::string, CBType*> cached_assignments;
    if (cached_data != cb_types.data() || cached_size != cb_types.size()) {
        cached_data = cb_types.data();
        cached_size = cb_types.size();
        cached_best_types.clear();
        cached_assignments.clear();
        int best_score = 0;
        for (CBType& cb_type : cb_types) {
            best_score = std::max(best_score, cbTransitScore(cb_type));
        }
        if (best_score > 0) {
            for (CBType& cb_type : cb_types) {
                if (cbTransitScore(cb_type) == best_score) {
                    cached_best_types.push_back(&cb_type);
                }
            }
        }
    }

    int fallback_slots = cached_best_types.empty() ? static_cast<int>(cb_types.size()) : static_cast<int>(cached_best_types.size());
    int fallback_slot = fallback_slots > 0 ? grid_x % fallback_slots : 0;
    std::string cache_key = tile_type_name + "\n" + std::to_string(fallback_slot);
    if (auto it = cached_assignments.find(cache_key); it != cached_assignments.end()) {
        return it->second;
    }

    auto remember_assignment = [&](CBType* cb_type) {
        cached_assignments[cache_key] = cb_type;
        return cb_type;
    };

    if (CBType* cb_type = exactCBTypeFor(cb_types, tile_type_name); cb_type && cbTransitScore(*cb_type) > 0) {
        return remember_assignment(cb_type);
    }

    const std::string key_prefix = tile_type_name + "\n";
    for (const auto& [key, mappings] : local_route_wire_mappings) {
        if (key.compare(0, key_prefix.size(), key_prefix) != 0) {
            continue;
        }
        for (const LocalRouteWireMapping& mapping : mappings) {
            if (CBType* cb_type = exactCBTypeFor(cb_types, mapping.route_type)) {
                if (cbTransitScore(*cb_type) > 0) {
                    return remember_assignment(cb_type);
                }
            }
        }
    }

    if (cached_best_types.empty()) {
        return remember_assignment(cb_types.empty() ? nullptr : &cb_types[fallback_slot]);
    }
    return remember_assignment(cached_best_types[fallback_slot]);
}

std::string genericLocalNodeName(std::string wire)
{
    const std::string clbll = "CLBLL_";
    const std::string clblm = "CLBLM_";
    if (wire.compare(0, clbll.size(), clbll) == 0) {
        wire.erase(0, clbll.size());
    }
    if (wire.compare(0, clblm.size(), clblm) == 0) {
        wire.erase(0, clblm.size());
    }
    return wire;
}

std::vector<int> resolveLocalNodesInCB(const CBType& cb_type, const std::string& wire)
{
    std::string node_name = genericLocalNodeName(wire);
    std::vector<std::string> variants{node_name};
    for (const std::string prefix : {"IMUX", "BYP", "CLK", "CTRL", "FAN"}) {
        if (node_name.compare(0, prefix.length(), prefix) == 0) {
            variants.push_back(prefix + "_L" + node_name.substr(prefix.length()));
            variants.push_back(prefix + "_R" + node_name.substr(prefix.length()));
        }
    }
    std::vector<int> nodes;
    for (const std::string& variant : variants) {
        int node = cb_type.localNodeNum(variant);
        if (node >= 0 && std::find(nodes.begin(), nodes.end(), node) == nodes.end()) {
            nodes.push_back(node);
        }
    }
    return nodes;
}

struct LocalNodeMapping
{
    std::string route_type;
    int local_node = -1;
};

std::string tileConnKey(const std::string& tile_type, const std::string& wire);
void addRouteWireGraphEdge(std::unordered_map<std::string, std::vector<RouteWireGraphEdge>>& graph,
                           const std::string& from_tile_type, const std::string& from_wire,
                           const std::string& to_tile_type, const std::string& to_wire,
                           Coord delta = {});

std::vector<LocalNodeMapping> resolveLocalNodeMappings(const std::vector<CBType>& cb_types,
                                                       const std::unordered_map<std::string, std::vector<LocalRouteWireMapping>>& local_route_wire_mappings,
                                                       const std::unordered_map<std::string, std::vector<RouteWireGraphEdge>>& route_wire_graph,
                                                       const std::string& tile_type_name,
                                                       const std::string& wire)
{
    // Resolve a tile resource wire to concrete locals grouped by adjacent route tile type.
    std::vector<LocalNodeMapping> mappings;
    auto append_nodes = [&](const std::string& route_type, const CBType& cb_type, const std::string& cb_wire) {
        for (int node : resolveLocalNodesInCB(cb_type, cb_wire)) {
            auto same = [&](const LocalNodeMapping& mapping) {
                return mapping.route_type == route_type
                    && mapping.local_node == node;
            };
            if (std::find_if(mappings.begin(), mappings.end(), same) == mappings.end()) {
                mappings.push_back(LocalNodeMapping{route_type, node});
            }
        }
    };
    auto mappings_it = local_route_wire_mappings.find(tileConnKey(tile_type_name, wire));
    if (mappings_it != local_route_wire_mappings.end()) {
        for (const LocalRouteWireMapping& route_mapping : mappings_it->second) {
            const CBType* cb_type = exactCBTypeFor(cb_types, route_mapping.route_type);
            if (!cb_type) {
                continue;
            }
            for (int node : resolveLocalNodesInCB(*cb_type, route_mapping.route_wire)) {
                auto same = [&](const LocalNodeMapping& mapping) {
                    return mapping.route_type == route_mapping.route_type
                        && mapping.local_node == node;
                };
                if (std::find_if(mappings.begin(), mappings.end(), same) == mappings.end()) {
                    mappings.push_back(LocalNodeMapping{route_mapping.route_type, node});
                }
            }
        }
    }
    if (!mappings.empty()) {
        return mappings;
    }

    struct SearchState
    {
        std::string tile_type;
        std::string wire;
        int depth = 0;
    };
    std::vector<SearchState> queue;
    std::unordered_set<std::string> visited;
    queue.push_back(SearchState{tile_type_name, wire, 0});
    size_t queue_pos = 0;
    int found_depth = -1;
    while (queue_pos < queue.size()) {
        SearchState state = queue[queue_pos++];
        if (state.depth > 64 || (found_depth >= 0 && state.depth > found_depth)) {
            continue;
        }
        std::string visit_key = tileConnKey(state.tile_type, state.wire);
        if (!visited.insert(visit_key).second) {
            continue;
        }
        if (state.depth > 0) {
            if (const CBType* cb_type = exactCBTypeFor(cb_types, state.tile_type); routeCapableCBType(cb_type)) {
                for (int node : resolveLocalNodesInCB(*cb_type, state.wire)) {
                    auto same = [&](const LocalNodeMapping& mapping) {
                        return mapping.route_type == state.tile_type
                            && mapping.local_node == node;
                    };
                    if (std::find_if(mappings.begin(), mappings.end(), same) == mappings.end()) {
                        mappings.push_back(LocalNodeMapping{state.tile_type, node});
                    }
                }
                if (!mappings.empty()) {
                    found_depth = state.depth;
                }
                continue;
            }
        }
        auto edge_it = route_wire_graph.find(tileConnKey(state.tile_type, state.wire));
        if (edge_it == route_wire_graph.end()) {
            continue;
        }
        for (const RouteWireGraphEdge& edge : edge_it->second) {
            queue.push_back(SearchState{
                edge.tile_type,
                edge.wire,
                state.depth + 1
            });
        }
    }
    if (!mappings.empty()) {
        return mappings;
    }

    for (const CBType& cb_type : cb_types) {
        append_nodes(cb_type.name, cb_type, wire);
    }
    return mappings;
}

int resourceNodeFromMap(const TechMap& map, const std::string& port, int pos)
{
    if (map.empty()) {
        return -1;
    }
    for (const auto& expr : map[0]) {
        if (expr.empty() || expr[0].empty()) {
            continue;
        }
        for (const std::string& item : expr[0][0]) {
            size_t first_alpha = item.find_first_not_of("0123456789");
            if (first_alpha == std::string::npos) {
                continue;
            }
            int node = atoi(item.c_str());
            std::string mapped_port = item.substr(first_alpha);
            if (mapped_port == port) {
                return node + pos*256;
            }
        }
    }
    return -1;
}

std::string tileConnKey(const std::string& tile_type, const std::string& wire)
{
    return tile_type + "\n" + wire;
}

void addRouteWireGraphEdge(std::unordered_map<std::string, std::vector<RouteWireGraphEdge>>& graph,
                           const std::string& from_tile_type, const std::string& from_wire,
                           const std::string& to_tile_type, const std::string& to_wire,
                           Coord delta)
{
    if (from_tile_type.empty() || from_wire.empty() || to_tile_type.empty() || to_wire.empty()) {
        return;
    }
    auto& edges = graph[tileConnKey(from_tile_type, from_wire)];
    auto same = [&](const RouteWireGraphEdge& edge) {
        return edge.tile_type == to_tile_type
            && edge.wire == to_wire
            && edge.delta.x == delta.x
            && edge.delta.y == delta.y;
    };
    if (std::find_if(edges.begin(), edges.end(), same) == edges.end()) {
        edges.push_back(RouteWireGraphEdge{to_tile_type, to_wire, delta});
    }
}

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

int decodeJumpSigned4(int value)
{
    value &= 0xf;
    return (value & 0x8) ? value - 16 : value;
}

bool canEncodeJumpDelta(Coord delta)
{
    return delta.x >= -8 && delta.x <= 7 && delta.y >= -8 && delta.y <= 7;
}

Coord jumpDelta(int jump)
{
    return Coord{decodeJumpSigned4((jump >> 6) & 0xf), decodeJumpSigned4((jump >> 2) & 0xf)};
}

void loadTileSiteNames(const std::string& spec_name, std::vector<Referable<Tile>>& tile_grid, const TileGridSpec& grid_spec)
{
    std::ifstream in(spec_name);
    if (!in) {
        return;
    }

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(in, root)) {
        return;
    }

    for (const std::string& tile_name : root.getMemberNames()) {
        const Json::Value& tile_spec = root[tile_name];
        int grid_x = tile_spec.get("grid_x", -1).asInt();
        int grid_y = tile_spec.get("grid_y", -1).asInt();
        if (grid_x < 0 || grid_y < 0 || grid_x >= grid_spec.size.x || grid_y >= grid_spec.size.y) {
            continue;
        }

        Tile& tile = tile_grid[grid_y*grid_spec.size.x + grid_x];
        tile.full_name = tile_name;
        tile.sites.clear();
        tile.site_types.clear();

        const Json::Value& sites = tile_spec["sites"];
        std::vector<std::string> site_keys = sites.getMemberNames();
        std::sort(site_keys.begin(), site_keys.end());
        for (const std::string& site : site_keys) {
            tile.sites.push_back(site);
            tile.site_types.push_back(sites[site].asString());
        }
    }
}

void assignAttachedCbTiles(std::vector<Referable<Tile>>& tile_grid)
{
    std::unordered_map<std::string, Tile*> physical_cb_by_type_and_vendor_coord;
    for (auto& tile_ref : tile_grid) {
        Tile& tile = tile_ref;
        if (tile.full_name.empty() || !tile.tile_type) {
            continue;
        }
        size_t suffix_pos = tile.full_name.rfind("_X");
        if (suffix_pos == std::string::npos) {
            continue;
        }
        physical_cb_by_type_and_vendor_coord[tile.tile_type->name + tile.full_name.substr(suffix_pos)] = &tile;
    }

    for (auto& tile_ref : tile_grid) {
        Tile& tile = tile_ref;
        tile.cb_coord = Coord{-1, -1};
        tile.cb_full_name.clear();
        if (!tile.cb_type) {
            continue;
        }
        if (!tile.full_name.empty() && tileTypePrefix(tile.full_name) == tile.cb_type->name) {
            tile.cb_coord = tile.coord;
            tile.cb_full_name = tile.full_name;
            continue;
        }
        if (!tile.full_name.empty()) {
            size_t suffix_pos = tile.full_name.rfind("_X");
            if (suffix_pos != std::string::npos) {
                auto it = physical_cb_by_type_and_vendor_coord.find(tile.cb_type->name + tile.full_name.substr(suffix_pos));
                if (it != physical_cb_by_type_and_vendor_coord.end()) {
                    tile.cb_coord = it->second->coord;
                    tile.cb_full_name = it->second->full_name;
                    continue;
                }
            }
        }

        Tile* best = nullptr;
        int best_distance = std::numeric_limits<int>::max();
        for (auto& candidate_ref : tile_grid) {
            Tile& candidate = candidate_ref;
            if (!candidate.tile_type || candidate.tile_type->name != tile.cb_type->name) {
                continue;
            }
            int distance = std::abs(candidate.coord.x - tile.coord.x) + std::abs(candidate.coord.y - tile.coord.y);
            if (!best || distance < best_distance
                || (distance == best_distance && sameVendorTileCoord(tile.full_name, candidate.full_name))) {
                best = &candidate;
                best_distance = distance;
            }
        }
        if (best) {
            tile.cb_coord = best->coord;
            tile.cb_full_name = best->full_name;
        }
    }
}

Tile* attachedRouteTileForCbCoord(std::vector<Referable<Tile>>& tile_grid, const Coord& cb_coord, const CBType* cb_type)
{
    static const Referable<Tile>* cached_data = nullptr;
    static size_t cached_size = 0;
    static std::unordered_map<AttachedCbKey, Tile*, AttachedCbKeyHash> cache;
    if (cached_data != tile_grid.data() || cached_size != tile_grid.size()) {
        cached_data = tile_grid.data();
        cached_size = tile_grid.size();
        cache.clear();
        for (auto& tile_ref : tile_grid) {
            Tile& tile = tile_ref;
            if (!tile.cb_type || tile.cb_coord.x < 0 || tile.cb_coord.y < 0) {
                continue;
            }
            AttachedCbKey key{tile.cb_type, tile.cb_coord.x, tile.cb_coord.y};
            auto it = cache.find(key);
            bool tile_is_physical_cb = tile.tile_type && tile.tile_type->name == tile.cb_type->name;
            bool old_is_physical_cb = it != cache.end() && it->second->tile_type
                && it->second->tile_type->name == it->second->cb_type->name;
            if (it == cache.end() || (!old_is_physical_cb && tile_is_physical_cb)) {
                cache[key] = &tile;
            }
        }
    }
    auto it = cache.find(AttachedCbKey{cb_type, cb_coord.x, cb_coord.y});
    if (it != cache.end()) {
        return it->second;
    }
    Tile* found = nullptr;
    for (auto& tile_ref : tile_grid) {
        Tile& tile = tile_ref;
        if (tile.cb_type != cb_type || tile.cb_coord.x != cb_coord.x || tile.cb_coord.y != cb_coord.y) {
            continue;
        }
        found = &tile;
        break;
    }
    cache[AttachedCbKey{cb_type, cb_coord.x, cb_coord.y}] = found;
    return found;
}

Coord routeMetricCoord(const std::vector<Referable<Tile>>& tile_grid, int size_width, int size_height, Coord coord)
{
    if (coord.x < 0 || coord.y < 0 || coord.x >= size_width || coord.y >= size_height) {
        return coord;
    }
    const Tile& tile = tile_grid[coord.y * size_width + coord.x];
    if (tile.cb_coord.x >= 0 && tile.cb_coord.y >= 0) {
        return tile.cb_coord;
    }
    return coord;
}

}

Device& Device::current()
{
    static Device current;
    return current;
}

void Device::loadFromSpec(const std::string& spec_name, const std::string& pins_spec_name)
{
    // tiles specs
    PNR_LOG("FPGA", "loadFromSpec, spec_name: '{}'", spec_name);
    std::filesystem::path tileconn = std::filesystem::path(spec_name).parent_path() / "tileconn.json";
    if (std::filesystem::exists(tileconn)) {
        loadTileConnFromSpec(tileconn.string());
    }

    std::map<std::string,TileSpec> tiles_spec;
    readTileGrid(spec_name, &tiles_spec, &grid_spec);
    tile_grid.resize(grid_spec.size.y*grid_spec.size.x);

    for (const auto& spec : tiles_spec) {
//            for (const auto& type : tile_types) {
//                if (spec.second.name.find(type.name) == 0) {
                PNR_LOG1("FPGA",  "found spec '{}'", spec.second.name);
                for (const auto& rect : spec.second.rects) {
                    Coord name = rect.name;
                    PNR_LOG2("FPGA",  "populating rect {}/X{}Y{}...", (Rect)rect, name.x, name.y);
                    for (int x = rect.x.a; x != rect.x.b+1; ++x) {
                        name.y = rect.name.y;
                        for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                            tile_grid[y*grid_spec.size.x + x].coord = {x,y};
                            tile_grid[y*grid_spec.size.x + x].name = name;
                            tile_grid[y*grid_spec.size.x + x].cb_type = cbTypeFor(cb_types, local_route_wire_mappings, spec.second.name, x);
                            tile_grid[y*grid_spec.size.x + x].tile_type = tileTypeFor(tile_types, spec.second.name);
                            tile_grid[y*grid_spec.size.x + x].cb = {};
                            tile_grid[y*grid_spec.size.x + x].cb.type = tile_grid[y*grid_spec.size.x + x].cb_type;
                            tile_grid[y*grid_spec.size.x + x].pin_state = {};
                            x_to_grid[name.x] = x;
                            y_to_grid[name.y] = y;
                            ++name.y;
                        }
                        ++name.x;
                    }
                    for (const auto& range : rect.more_x) {
                        name.x = range.name_x;
                        PNR_LOG4("FPGA", " {}/X{}Y{}'", (Range)range, name.x, rect.name.y);
                        for (int x = range.a; x != range.b+1; ++x) {
                            name.y = rect.name.y;
                            for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                    tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                                tile_grid[y*grid_spec.size.x + x].coord = {x,y};
                                tile_grid[y*grid_spec.size.x + x].name = name;
                                tile_grid[y*grid_spec.size.x + x].cb_type = cbTypeFor(cb_types, local_route_wire_mappings, spec.second.name, x);
                                tile_grid[y*grid_spec.size.x + x].tile_type = tileTypeFor(tile_types, spec.second.name);
                                tile_grid[y*grid_spec.size.x + x].cb = {};
                                tile_grid[y*grid_spec.size.x + x].cb.type = tile_grid[y*grid_spec.size.x + x].cb_type;
                                tile_grid[y*grid_spec.size.x + x].pin_state = {};
                                x_to_grid[name.x] = x;
                                y_to_grid[name.y] = y;
                                ++name.y;
                            }
                            ++name.x;
                        }
                    }
                }
//                }
//            }
    }

    size_width = grid_spec.size.x;
    size_height = grid_spec.size.y;
    loadTileSiteNames(spec_name, tile_grid, grid_spec);
    assignAttachedCbTiles(tile_grid);
    cnt_regs = 2*grid_spec.size.y*grid_spec.size.x*4;
    cnt_luts = 2*grid_spec.size.y*grid_spec.size.x*4;
    PNR_LOG("FPGA", "loadFromSpec, fpga width: {}, height: {}, cnt_regs: {}, cnt_luts: {}, pins_spec_name: '{}'", size_width, size_height, cnt_regs, cnt_luts, pins_spec_name);

    PNR_LOG("FPGA", "loadFromSpec pins, pins_spec_name: '{}'", pins_spec_name);
    std::vector<PinSpec> specs;
    readPackagePins(pins_spec_name, specs);

    for (auto spec : specs) {
        pins.push_back(Pin{spec.name, spec.bank, spec.site, spec.tile, spec.function, spec.pos});
    }

}

void Device::loadTypeFromSpec(const std::string& spec_name, TechMap& map)
{
    // any types except cb
    PNR_LOG("FPGA", "loadTypeFromSpec, spec_name: '{}'", spec_name);
    TileTypesSpec spec;
    std::map<std::string,TypeSpec> types;
    readTypes(spec_name, &types, &spec);
    for (const auto& type_spec : types) {
        auto existing = std::find_if(tile_types.begin(), tile_types.end(), [&](const TileType& type) {
            return type.name == type_spec.first;
        });
        TileType* type = nullptr;
        if (existing == tile_types.end()) {
            tile_types.push_back(TileType{type_spec.first, tile_types.size(), 0});
            type = &tile_types.back();
        }
        else {
            type = &*existing;
        }

        type->sites.clear();
        type->pin_map = {};
        for (const TypeSpec::SiteSpec& site_spec : type_spec.second.sites) {
            SiteModel site;
            site.name = site_spec.name;
            site.type = site_spec.type;
            site.pos = site_spec.pos;
            site.pins = site_spec.pins;
            type->sites.push_back(std::move(site));
        }

        for (const TypeSpec::WireEdgeSpec& edge : type_spec.second.wire_edges) {
            // Tile-internal wires extend endpoint discovery without changing route traversal.
            addRouteWireGraphEdge(route_wire_graph, type_spec.first, edge.src, type_spec.first, edge.dst);
            addRouteWireGraphEdge(route_wire_graph, type_spec.first, edge.dst, type_spec.first, edge.src);
        }

        type->rebuildElementsFromSites();
        bool resolve_pin_map = type_spec.second.direct_site_wire_endpoints
            || (!type->elements.empty() && typeSpecHasPackableSite(type_spec.second));
        if (!resolve_pin_map) {
            PNR_LOG1("FPGA", "loadTypeFromSpec, loaded '{}' with {} input and {} output resource nodes, {} local wire names and {} resource names",
                type->name, type->pin_map.input_nodes.size(), type->pin_map.output_nodes.size(),
                type->pin_map.local_wire_names.size(), type->pin_map.local_resource_names.size());
            continue;
        }

        for (const TypeSpec::PinNodeSpec& pin : type_spec.second.input_pins) {
            int resource_node = resourceNodeFromMap(map, pin.port, pin.pos);
            if (resource_node < 0) {
                continue;
            }
            type->pin_map.rememberResourcePinName(TILE_PIN_INPUT, resource_node, pin.port);
            for (const std::string& wire : pin.nodes) {
                for (const LocalNodeMapping& mapping : resolveLocalNodeMappings(cb_types, local_route_wire_mappings,
                         route_wire_graph, type_spec.first, wire)) {
                    int local_node = mapping.local_node;
                    // Keep input resource pins mapped only to locals that can be entered from routing.
                    type->pin_map.input_nodes[resource_node] |= NodeMask{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointRouteRef(TILE_PIN_INPUT, resource_node, local_node, mapping.route_type);
                }
                if (technology::mappedRouteEndpointAliases) {
                    for (const auto& alias : technology::mappedRouteEndpointAliases(type_spec.first, pin.port, pin.pos, wire)) {
                        for (const LocalNodeMapping& mapping : resolveLocalNodeMappings(cb_types, local_route_wire_mappings,
                                 route_wire_graph, alias.first, alias.second)) {
                            int local_node = mapping.local_node;
                            type->pin_map.input_nodes[resource_node] |= NodeMask{0,1} << local_node;
                            type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointRouteRef(TILE_PIN_INPUT, resource_node, local_node, mapping.route_type);
                        }
                    }
                }
            }
        }

        for (const TypeSpec::PinNodeSpec& pin : type_spec.second.output_pins) {
            int resource_node = resourceNodeFromMap(map, pin.port, pin.pos);
            if (resource_node < 0) {
                continue;
            }
            type->pin_map.rememberResourcePinName(TILE_PIN_OUTPUT, resource_node, pin.port);
            for (const std::string& wire : pin.nodes) {
                for (const LocalNodeMapping& mapping : resolveLocalNodeMappings(cb_types, local_route_wire_mappings,
                         route_wire_graph, type_spec.first, wire)) {
                    int local_node = mapping.local_node;
                    // Keep output resource pins mapped only to locals that can launch into routing.
                    type->pin_map.output_nodes[resource_node] |= NodeMask{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointRouteRef(TILE_PIN_OUTPUT, resource_node, local_node, mapping.route_type);
                }
                if (technology::mappedRouteEndpointAliases) {
                    for (const auto& alias : technology::mappedRouteEndpointAliases(type_spec.first, pin.port, pin.pos, wire)) {
                        for (const LocalNodeMapping& mapping : resolveLocalNodeMappings(cb_types, local_route_wire_mappings,
                                 route_wire_graph, alias.first, alias.second)) {
                            int local_node = mapping.local_node;
                            type->pin_map.output_nodes[resource_node] |= NodeMask{0,1} << local_node;
                            type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointRouteRef(TILE_PIN_OUTPUT, resource_node, local_node, mapping.route_type);
                        }
                    }
                }
            }
        }
        PNR_LOG1("FPGA", "loadTypeFromSpec, loaded '{}' with {} input and {} output resource nodes, {} local wire names and {} resource names",
            type->name, type->pin_map.input_nodes.size(), type->pin_map.output_nodes.size(),
            type->pin_map.local_wire_names.size(), type->pin_map.local_resource_names.size());
    }
}

void Device::loadCBFromSpec(const std::string& spec_name, TechMap& map)
{
    // crossbars
    PNR_LOG("FPGA", "loadCBFromSpec, spec_name: '{}'", spec_name);
    TileTypesSpec spec;
    std::map<std::string,CBTypeSpec> cbs;
    readCBTypes(spec_name, &cbs, &spec);
    for (auto& cb : cbs) {
        if (CBType* existing = exactCBTypeFor(cb_types, cb.first)) {
            existing->type_id = cbTypeIdFor(cb_types, existing);
            PNR_LOG1("FPGA", "loadCBFromSpec, updating cb_type '{}' ptr={}", cb.first, static_cast<const void*>(existing));
            existing->loadFromSpec(cb.second, map);
            continue;
        }
        cb_types.push_back(CBType{cb.first});
        PNR_ASSERT(cb_types.size() - 1 < CB_INVALID_TYPE_ID, "too many CB types: {}\n", cb_types.size());
        cb_types.back().type_id = static_cast<uint16_t>(cb_types.size() - 1);
        PNR_LOG1("FPGA", "loadCBFromSpec, inserting cb_type '{}' ptr={}", cb.first, static_cast<const void*>(&cb_types.back()));
        cb_types.back().loadFromSpec(cb.second, map);
    }
    const char* debug_dst_text = std::getenv("SCALEPNR_DEBUG_CB_DST");
    if (debug_dst_text && *debug_dst_text) {
        int debug_dst = atoi(debug_dst_text);
        int debug_local = -1;
        const char* debug_local_text = std::getenv("SCALEPNR_DEBUG_CB_LOCAL");
        if (debug_local_text && *debug_local_text) {
            debug_local = atoi(debug_local_text);
        }
        if (debug_dst >= 0 && debug_dst < CB_MAX_NODES) {
            for (const CBType& cb_type : cb_types) {
                bool local_bit = debug_local >= 0 && debug_local < CB_MAX_NODES
                    && (cb_type.dst_local[debug_dst].local & (NodeMask{0,1} << debug_local)) != NodeMask{};
                NodeMask dst_local = cb_type.dst_local[debug_dst].local;
                PNR_LOG1("FPGA", "loadCBFromSpec debug cb_type '{}' ptr={} dst={} local={} local_bit={} dst_local={}",
                    cb_type.name, static_cast<const void*>(&cb_type), debug_dst, debug_local, local_bit,
                    dst_local.str());
            }
        }
    }
}

void Device::loadTileConnFromSpec(const std::string& spec_name)
{
    PNR_LOG("FPGA", "loadTileConnFromSpec, spec_name: '{}'", spec_name);
    std::ifstream infile(spec_name);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + spec_name);
    }

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(infile, root)) {
        throw std::runtime_error(std::string("cant parse tileconn JSON: ") + spec_name);
    }

    local_route_wire_mappings.clear();
    route_wire_graph.clear();
    std::unordered_map<std::string, std::vector<RouteWireGraphEdge>> physical_wire_graph;
    std::vector<ParsedTileConnRule> rules;
    for (const Json::Value& item : root) {
        if (!item.isMember("tile_types") || !item.isMember("wire_pairs")) {
            continue;
        }
        const Json::Value& types = item["tile_types"];
        if (!types.isArray() || types.size() != 2) {
            continue;
        }

        ParsedTileConnRule rule;
        rule.from_tile_type = types[0].asString();
        rule.to_tile_type = types[1].asString();
        if (item.isMember("grid_deltas") && item["grid_deltas"].isArray() && item["grid_deltas"].size() == 2) {
            rule.delta = Coord{item["grid_deltas"][0].asInt(), item["grid_deltas"][1].asInt()};
        }
        for (const Json::Value& pair : item["wire_pairs"]) {
            if (!pair.isArray() || pair.size() != 2) {
                continue;
            }
            ParsedTileConnPair parsed_pair{pair[0].asString(), pair[1].asString()};
            addRouteWireGraphEdge(route_wire_graph, rule.from_tile_type, parsed_pair.from_wire,
                                  rule.to_tile_type, parsed_pair.to_wire, rule.delta);
            addRouteWireGraphEdge(route_wire_graph, rule.to_tile_type, parsed_pair.to_wire,
                                  rule.from_tile_type, parsed_pair.from_wire, Coord{-rule.delta.x, -rule.delta.y});
            if (exactCBTypeFor(cb_types, rule.from_tile_type) && exactCBTypeFor(cb_types, rule.to_tile_type)) {
                addRouteWireGraphEdge(physical_wire_graph, rule.from_tile_type, parsed_pair.from_wire,
                                      rule.to_tile_type, parsed_pair.to_wire, rule.delta);
                addRouteWireGraphEdge(physical_wire_graph, rule.to_tile_type, parsed_pair.to_wire,
                                      rule.from_tile_type, parsed_pair.from_wire, Coord{-rule.delta.x, -rule.delta.y});
            }
            rule.wire_pairs.push_back(std::move(parsed_pair));
        }
        rules.push_back(std::move(rule));
    }

    size_t local_mappings = 0;
    for (const ParsedTileConnRule& rule : rules) {
        const CBType* from_cb = exactCBTypeFor(cb_types, rule.from_tile_type);
        const CBType* to_cb = exactCBTypeFor(cb_types, rule.to_tile_type);
        for (const ParsedTileConnPair& pair : rule.wire_pairs) {
            if (from_cb && !to_cb) {
                local_route_wire_mappings[tileConnKey(rule.to_tile_type, pair.to_wire)].push_back(
                    LocalRouteWireMapping{rule.from_tile_type, pair.from_wire});
                ++local_mappings;
            }
            if (to_cb && !from_cb) {
                local_route_wire_mappings[tileConnKey(rule.from_tile_type, pair.from_wire)].push_back(
                    LocalRouteWireMapping{rule.to_tile_type, pair.to_wire});
                ++local_mappings;
            }
        }
    }

    size_t source_nodes = 0;
    size_t mapped_source_nodes = 0;
    size_t mapped_edges = 0;
    for (CBType& source_cb : cb_types) {
        for (const auto& [src_name, src_node_u16] : source_cb.src_nodes_by_name) {
            int src_node = src_node_u16;
            ++source_nodes;
            NodeMask before_dst = source_cb.src_dst[src_node].jump;
            struct SearchState
            {
                std::string tile_type;
                std::string wire;
                Coord delta;
                int depth = 0;
            };
            std::vector<SearchState> queue;
            std::unordered_set<std::string> visited;
            queue.push_back(SearchState{source_cb.name, src_name, {}, 0});
            size_t queue_pos = 0;
            while (queue_pos < queue.size()) {
                SearchState state = queue[queue_pos++];
                if (state.depth > 64) {
                    continue;
                }
                std::string visit_key = tileConnKey(state.tile_type, state.wire);
                if (!visited.insert(visit_key).second) {
                    continue;
                }

                if (state.depth > 0) {
                    if (const CBType* target_cb = exactCBTypeFor(cb_types, state.tile_type)) {
                        int dst_node = nodeNumByPhysicalWireName(*target_cb, CB_NODE_DST, state.wire);
                        if (dst_node >= 0) {
                            source_cb.src_dst[src_node].jump |= NodeMask{0,1} << dst_node;
                            addResolvedJumpBit(source_cb.src_dst_by_jump[src_node],
                                               state.delta, cbTypeIdFor(cb_types, target_cb), dst_node,
                                               source_cb.name, src_node, target_cb->name);
                            ++mapped_edges;
                            continue;
                        }
                    }
                }

                auto edge_it = route_wire_graph.find(tileConnKey(state.tile_type, state.wire));
                if (edge_it == route_wire_graph.end()) {
                    continue;
                }
                for (const RouteWireGraphEdge& edge : edge_it->second) {
                    Coord next_delta{state.delta.x + edge.delta.x, state.delta.y + edge.delta.y};
                    if (!canEncodeJumpDelta(next_delta)) {
                        continue;
                    }
                    queue.push_back(SearchState{
                        edge.tile_type,
                        edge.wire,
                        next_delta,
                        state.depth + 1
                    });
                }
            }
            if (source_cb.src_dst[src_node].jump != before_dst) {
                ++mapped_source_nodes;
            }
        }
    }

    for (CBType& cb_type : cb_types) {
        cb_type.ensureDerivedMasks();
    }

    PNR_LOG("FPGA", "loadTileConnFromSpec loaded {} tile connection rules, {} local pin mappings, mapped {}/{} source nodes through {} dst bits",
        rules.size(), local_mappings, mapped_source_nodes, source_nodes, mapped_edges);
}

Tile* Device::getTile(int x, int y)
{
    if (x < 0 || y < 0 || x >= size_width || y >= size_height) {
        return nullptr;
    }
    return &tile_grid[y*grid_spec.size.x + x];
}

TileJumpTarget Device::resolveJump(const Tile& from, int src_node, const Coord* preferred) const
{
    if (!from.cb_type || src_node < 0 || src_node >= CB_MAX_NODES) {
        return {};
    }
    static const Referable<Tile>* cached_grid_data = nullptr;
    static size_t cached_grid_size = 0;
    static std::unordered_map<ResolveJumpCacheKey, TileJumpTarget, ResolveJumpCacheKeyHash> cache;
    if (cached_grid_data != tile_grid.data() || cached_grid_size != tile_grid.size()) {
        cached_grid_data = tile_grid.data();
        cached_grid_size = tile_grid.size();
        cache.clear();
    }
    ResolveJumpCacheKey cache_key{
        &from,
        src_node,
        preferred ? preferred->x : 0,
        preferred ? preferred->y : 0,
        preferred != nullptr
    };
    if (auto cache_it = cache.find(cache_key); cache_it != cache.end()) {
        return cache_it->second;
    }
    bool debug = debugResolveJumpCoord(from.coord);

    auto make_target = [&](int source_jump, Coord delta, NodeMask dst_candidates, const CBType* target_cb_type) -> TileJumpTarget {
        Coord next = from.coord + delta;
        const CBType* expected_cb_type = target_cb_type ? target_cb_type : from.cb_type;
        Tile* next_tile = const_cast<Device*>(this)->getTile(next.x, next.y);
        if (next_tile && next_tile->cb_type != expected_cb_type) {
            next_tile = nullptr;
        }
        if (!next_tile || !next_tile->cb_type) {
            if (debug) {
                PNR_LOG("FPGA", "resolveJump reject no target src={} jump={} d=({}, {}) target_cb=({}, {}) cb_type='{}'",
                    src_node, source_jump, delta.x, delta.y, next.x, next.y,
                    expected_cb_type ? expected_cb_type->name : std::string{});
            }
            return {};
        }

        next_tile->cb_type->ensureDerivedMasks();
        NodeMask dst_mask = dst_candidates & next_tile->cb_type->valid_dst_nodes;
        if (debug) {
            const std::string* src_name = from.cb_type->nodeName(CB_NODE_SRC, src_node);
            PNR_LOG("FPGA", "resolveJump try from=({}, {}) tile='{}' cb='{}' src={} '{}' jump={} d=({}, {}) target=({}, {}) tile='{}' cb='{}' candidates={} valid={} intersect={}",
                from.coord.x, from.coord.y, from.name, from.cb_type->name, src_node,
                src_name ? *src_name : std::string{},
                source_jump, delta.x, delta.y,
                next.x, next.y, next_tile->name, next_tile->cb_type->name,
                countBits(dst_candidates), countBits(next_tile->cb_type->valid_dst_nodes), countBits(dst_mask));
        }
        int dst_node = -1;
        if ((dst_mask & (NodeMask{0,1} << src_node)) != NodeMask{}) {
            dst_node = src_node;
        }
        else {
            dst_mask.for_each_set_bit([&](int bit) {
                dst_node = bit;
                return true;
            });
        }
        if (dst_node < 0) {
            if (debug) {
                PNR_LOG("FPGA", "resolveJump reject no dst src={} jump={} target_cb=({}, {})",
                    src_node, source_jump, next.x, next.y);
            }
            return {};
        }
        std::string dst_wire;
        if (const std::string* name = next_tile->cb_type->nodeName(CB_NODE_DST, dst_node)) {
            dst_wire = *name;
        }
        if (debug) {
            PNR_LOG("FPGA", "resolveJump accept src={} jump={} dst={} '{}'",
                src_node, source_jump, dst_node, dst_wire);
        }
        return TileJumpTarget{next_tile, dst_node, source_jump, dst_wire};
    };

    const auto& exact = from.cb_type->src_dst_by_jump[src_node];
    if (debug) {
        const std::string* src_name = from.cb_type->nodeName(CB_NODE_SRC, src_node);
        PNR_LOG("FPGA", "resolveJump start from=({}, {}) tile='{}' cb='{}' src={} '{}' exact_entries={} fallback_candidates={}",
            from.coord.x, from.coord.y, from.name, from.cb_type->name, src_node,
            src_name ? *src_name : std::string{},
            exact.size(), countBits(from.cb_type->src_dst[src_node].jump));
    }
    if (!exact.empty()) {
        TileJumpTarget best_target;
        int best_score = std::numeric_limits<int>::max();
        for (const CBType::ResolvedJump& entry : exact) {
            const CBType* target_cb_type = cbTypeById(cb_types, entry.target_cb_type_id);
            TileJumpTarget target = make_target(src_node, entry.delta, entry.dsts.jump, target_cb_type);
            if (!target.tile) {
                continue;
            }
            int score = 0;
            if (preferred) {
                score = std::abs(preferred->x - target.tile->coord.x) + std::abs(preferred->y - target.tile->coord.y);
            }
            if (score < best_score) {
                best_target = target;
                best_score = score;
                if (best_score == 0) {
                    return best_target;
                }
            }
        }
        if (best_target.tile) {
            cache[cache_key] = best_target;
            return best_target;
        }
    }

    TileJumpTarget fallback = make_target(src_node, jumpDelta(src_node), from.cb_type->src_dst[src_node].jump, from.cb_type);
    cache[cache_key] = fallback;
    return fallback;
}
