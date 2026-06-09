#include "Device.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unordered_set>

using namespace fpga;

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

CBType* cbTypeFor(std::vector<CBType>& cb_types,
                  const std::unordered_map<std::string, std::vector<LocalRouteWireMapping>>& local_route_wire_mappings,
                  const std::string& tile_type_name, int grid_x)
{
    if (CBType* cb_type = exactCBTypeFor(cb_types, tile_type_name)) {
        return cb_type;
    }

    const std::string key_prefix = tile_type_name + "\n";
    for (const auto& [key, mappings] : local_route_wire_mappings) {
        if (key.compare(0, key_prefix.size(), key_prefix) != 0) {
            continue;
        }
        for (const LocalRouteWireMapping& mapping : mappings) {
            if (CBType* cb_type = exactCBTypeFor(cb_types, mapping.route_type)) {
                return cb_type;
            }
        }
    }

    return cb_types.empty() ? nullptr : &cb_types[grid_x % cb_types.size()];
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

std::vector<LocalNodeMapping> resolveLocalNodeMappings(const std::vector<CBType>& cb_types,
                                                       const std::unordered_map<std::string, std::vector<LocalRouteWireMapping>>& local_route_wire_mappings,
                                                       const std::string& tile_type_name,
                                                       const std::string& wire)
{
    // Resolve a tile resource wire to concrete locals grouped by adjacent route tile type.
    std::vector<LocalNodeMapping> mappings;
    auto append_nodes = [&](const std::string& route_type, const CBType& cb_type, const std::string& cb_wire) {
        for (int node : resolveLocalNodesInCB(cb_type, cb_wire)) {
            auto same = [&](const LocalNodeMapping& mapping) {
                return mapping.route_type == route_type && mapping.local_node == node;
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
            append_nodes(route_mapping.route_type, *cb_type, route_mapping.route_wire);
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

struct TileConnGraphEdge
{
    std::string tile_type;
    std::string wire;
};

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
                            memset(&tile_grid[y*grid_spec.size.x + x].cb, 0, sizeof(tile_grid[y*grid_spec.size.x + x].cb));
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
                                memset(&tile_grid[y*grid_spec.size.x + x].cb, 0, sizeof(tile_grid[y*grid_spec.size.x + x].cb));
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
        for (const TypeSpec::SiteSpec& site_spec : type_spec.second.sites) {
            SiteModel site;
            site.name = site_spec.name;
            site.type = site_spec.type;
            site.pos = site_spec.pos;
            site.pins = site_spec.pins;
            type->sites.push_back(std::move(site));
        }

        for (const TypeSpec::PinNodeSpec& pin : type_spec.second.input_pins) {
            int resource_node = resourceNodeFromMap(map, pin.port, pin.pos);
            if (resource_node < 0) {
                continue;
            }
            type->pin_map.rememberResourcePinName(TILE_PIN_INPUT, resource_node, pin.port);
            for (const std::string& wire : pin.nodes) {
                for (const LocalNodeMapping& mapping : resolveLocalNodeMappings(cb_types, local_route_wire_mappings, type_spec.first, wire)) {
                    int local_node = mapping.local_node;
                    // Keep input resource pins mapped only to locals that can be entered from routing.
                    type->pin_map.input_nodes[resource_node] |= u256{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointRouteType(TILE_PIN_INPUT, resource_node, local_node, mapping.route_type);
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
                for (const LocalNodeMapping& mapping : resolveLocalNodeMappings(cb_types, local_route_wire_mappings, type_spec.first, wire)) {
                    int local_node = mapping.local_node;
                    // Keep output resource pins mapped only to locals that can launch into routing.
                    type->pin_map.output_nodes[resource_node] |= u256{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointRouteType(TILE_PIN_OUTPUT, resource_node, local_node, mapping.route_type);
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
            PNR_LOG1("FPGA", "loadCBFromSpec, updating cb_type '{}' ptr={}", cb.first, static_cast<const void*>(existing));
            existing->loadFromSpec(cb.second, map);
            continue;
        }
        cb_types.push_back(CBType{cb.first});
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
                    && (cb_type.dst_local[debug_dst].local & (u256{0,1} << debug_local)) != u256{};
                u256 dst_local = cb_type.dst_local[debug_dst].local;
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
    std::vector<ParsedTileConnRule> rules;
    std::unordered_map<std::string, std::vector<TileConnGraphEdge>> graph;
    for (const Json::Value& item : root) {
        if (!item.isMember("grid_deltas") || !item.isMember("tile_types") || !item.isMember("wire_pairs")) {
            continue;
        }
        const Json::Value& deltas = item["grid_deltas"];
        const Json::Value& types = item["tile_types"];
        if (!deltas.isArray() || deltas.size() != 2 || !types.isArray() || types.size() != 2) {
            continue;
        }

        ParsedTileConnRule rule;
        rule.delta = {deltas[0].asInt(), deltas[1].asInt()};
        rule.from_tile_type = types[0].asString();
        rule.to_tile_type = types[1].asString();
        for (const Json::Value& pair : item["wire_pairs"]) {
            if (!pair.isArray() || pair.size() != 2) {
                continue;
            }
            ParsedTileConnPair parsed_pair{pair[0].asString(), pair[1].asString()};
            graph[tileConnKey(rule.from_tile_type, parsed_pair.from_wire)].push_back(TileConnGraphEdge{rule.to_tile_type, parsed_pair.to_wire});
            graph[tileConnKey(rule.to_tile_type, parsed_pair.to_wire)].push_back(TileConnGraphEdge{rule.from_tile_type, parsed_pair.from_wire});
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
        for (const auto& [src_name, src_node_u8] : source_cb.src_nodes_by_name) {
            int src_node = src_node_u8;
            ++source_nodes;
            u256 before = source_cb.src_dst[src_node].jump;
            struct SearchState
            {
                std::string tile_type;
                std::string wire;
                int depth = 0;
            };
            std::vector<SearchState> queue;
            std::unordered_set<std::string> visited;
            queue.push_back(SearchState{source_cb.name, src_name, 0});
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
                        int dst_node = target_cb->nodeNum(CB_NODE_DST, state.wire);
                        if (dst_node >= 0) {
                            source_cb.src_dst[src_node].jump |= u256{0,1} << dst_node;
                            ++mapped_edges;
                            continue;
                        }
                    }
                }

                auto edge_it = graph.find(visit_key);
                if (edge_it == graph.end()) {
                    continue;
                }
                for (const TileConnGraphEdge& edge : edge_it->second) {
                    queue.push_back(SearchState{edge.tile_type, edge.wire, state.depth + 1});
                }
            }
            if (source_cb.src_dst[src_node].jump != before) {
                ++mapped_source_nodes;
            }
        }
    }

    PNR_LOG("FPGA", "loadTileConnFromSpec loaded {} temporary tile connection rules, {} local mappings, mapped {}/{} source nodes through {} dst bits",
        rules.size(), local_mappings, mapped_source_nodes, source_nodes, mapped_edges);
}

Tile* Device::getTile(int x, int y)
{
    if (x < 0 || y < 0 || x >= size_width || y >= size_height) {
        return nullptr;
    }
    return &tile_grid[y*grid_spec.size.x + x];
}

TileJumpTarget Device::resolveJump(const Tile& from, int src_node) const
{
    if (!from.cb_type || src_node < 0 || src_node >= CB_MAX_NODES) {
        return {};
    }
    int dir = src_node / 32;
    int path = src_node % 32;
    int step = std::max(1, path / 4 / 2);
    Coord next = from.coord;
    switch (dir) {
        case 0: next = from.coord + Coord{0, -step}; break;
        case 1: next = from.coord + Coord{step, -step}; break;
        case 2: next = from.coord + Coord{step, 0}; break;
        case 3: next = from.coord + Coord{step, step}; break;
        case 4: next = from.coord + Coord{0, step}; break;
        case 5: next = from.coord + Coord{-step, step}; break;
        case 6: next = from.coord + Coord{-step, 0}; break;
        case 7: next = from.coord + Coord{-step, -step}; break;
        default: return {};
    }
    Tile* next_tile = const_cast<Device*>(this)->getTile(next.x, next.y);
    if (!next_tile || !next_tile->cb_type) {
        return {};
    }

    u256 dst_mask = from.cb_type->src_dst[src_node].jump;
    int dst_node = -1;
    if ((dst_mask & (u256{0,1} << src_node)) != u256{} && next_tile->cb_type->nodeName(CB_NODE_DST, src_node)) {
        dst_node = src_node;
    }
    else {
        dst_mask.for_each_set_bit([&](int bit) {
            if (bit >= 0 && bit < CB_MAX_NODES && next_tile->cb_type->nodeName(CB_NODE_DST, bit)) {
                dst_node = bit;
                return true;
            }
            return false;
        });
    }
    if (dst_node < 0) {
        return {};
    }
    std::string dst_wire;
    if (const std::string* name = next_tile->cb_type->nodeName(CB_NODE_DST, dst_node)) {
        dst_wire = *name;
    }
    return TileJumpTarget{next_tile, dst_node, dst_wire};
}
