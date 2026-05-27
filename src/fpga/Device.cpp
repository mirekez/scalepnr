#include "Device.h"

#include <algorithm>
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

CBType* cbTypeFor(std::vector<CBType>& cb_types, const std::vector<TileConnRule>& tile_conn_rules,
                  const std::string& tile_type_name, int grid_x)
{
    if (CBType* cb_type = exactCBTypeFor(cb_types, tile_type_name)) {
        return cb_type;
    }

    for (const TileConnRule& rule : tile_conn_rules) {
        if (rule.from_tile_type == tile_type_name) {
            if (CBType* cb_type = exactCBTypeFor(cb_types, rule.to_tile_type)) {
                return cb_type;
            }
        }
        if (rule.to_tile_type == tile_type_name) {
            if (CBType* cb_type = exactCBTypeFor(cb_types, rule.from_tile_type)) {
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

std::vector<int> resolveLocalNodes(const std::vector<CBType>& cb_types, const std::vector<TileConnRule>& tile_conn_rules,
                                   const std::string& tile_type_name, const std::string& wire)
{
    std::vector<int> nodes;
    auto append_nodes = [&](const CBType& cb_type, const std::string& cb_wire) {
        for (int node : resolveLocalNodesInCB(cb_type, cb_wire)) {
            if (std::find(nodes.begin(), nodes.end(), node) == nodes.end()) {
                nodes.push_back(node);
            }
        }
    };

    for (const TileConnRule& rule : tile_conn_rules) {
        if (rule.from_tile_type == tile_type_name) {
            const CBType* cb_type = exactCBTypeFor(cb_types, rule.to_tile_type);
            if (!cb_type) {
                continue;
            }
            for (const TileConnWirePair& pair : rule.wire_pairs) {
                if (pair.from_wire == wire) {
                    append_nodes(*cb_type, pair.to_wire);
                }
            }
        }
        if (rule.to_tile_type == tile_type_name) {
            const CBType* cb_type = exactCBTypeFor(cb_types, rule.from_tile_type);
            if (!cb_type) {
                continue;
            }
            for (const TileConnWirePair& pair : rule.wire_pairs) {
                if (pair.to_wire == wire) {
                    append_nodes(*cb_type, pair.from_wire);
                }
            }
        }
    }
    if (!nodes.empty()) {
        return nodes;
    }

    for (const CBType& cb_type : cb_types) {
        append_nodes(cb_type, wire);
    }
    return nodes;
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

std::string tileTypeName(const Tile& tile)
{
    if (tile.tile_type) {
        return tile.tile_type->name;
    }
    if (tile.cb_type) {
        return tile.cb_type->name;
    }
    return {};
}

std::string tileConnKey(const std::string& tile_type, const std::string& wire)
{
    return tile_type + "\n" + wire;
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
                            tile_grid[y*grid_spec.size.x + x].cb_type = cbTypeFor(cb_types, tile_conn_rules, spec.second.name, x);
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
                                tile_grid[y*grid_spec.size.x + x].cb_type = cbTypeFor(cb_types, tile_conn_rules, spec.second.name, x);
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
                for (int local_node : resolveLocalNodes(cb_types, tile_conn_rules, type_spec.first, wire)) {
                    type->pin_map.input_nodes[resource_node] |= u256{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, wire, pin.wire, pin.port);
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
                for (int local_node : resolveLocalNodes(cb_types, tile_conn_rules, type_spec.first, wire)) {
                    type->pin_map.output_nodes[resource_node] |= u256{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, wire, pin.wire, pin.port);
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
        PNR_LOG1("FPGA", "loadCBFromSpec, inserting cb_type '{}'", cb.first);
        cb_types.push_back(CBType{cb.first});
        cb_types.back().loadFromSpec(cb.second, map);
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

    tile_conn_rules.clear();
    tile_conn_by_from.clear();
    tile_conn_by_to.clear();
    for (const Json::Value& item : root) {
        if (!item.isMember("grid_deltas") || !item.isMember("tile_types") || !item.isMember("wire_pairs")) {
            continue;
        }
        const Json::Value& deltas = item["grid_deltas"];
        const Json::Value& types = item["tile_types"];
        if (!deltas.isArray() || deltas.size() != 2 || !types.isArray() || types.size() != 2) {
            continue;
        }

        TileConnRule rule;
        rule.delta = {deltas[0].asInt(), deltas[1].asInt()};
        rule.from_tile_type = types[0].asString();
        rule.to_tile_type = types[1].asString();
        for (const Json::Value& pair : item["wire_pairs"]) {
            if (!pair.isArray() || pair.size() != 2) {
                continue;
            }
            rule.wire_pairs.push_back(TileConnWirePair{pair[0].asString(), pair[1].asString()});
        }
        size_t index = tile_conn_rules.size();
        tile_conn_rules.push_back(std::move(rule));
        for (const TileConnWirePair& pair : tile_conn_rules.back().wire_pairs) {
            tile_conn_by_from[tileConnKey(tile_conn_rules.back().from_tile_type, pair.from_wire)].push_back(index);
            tile_conn_by_to[tileConnKey(tile_conn_rules.back().to_tile_type, pair.to_wire)].push_back(index);
        }
    }
    PNR_LOG("FPGA", "loadTileConnFromSpec loaded {} geometric tile connection rules", tile_conn_rules.size());
}

Tile* Device::getTile(int x, int y)
{
    if (x < 0 || y < 0 || x >= size_width || y >= size_height) {
        return nullptr;
    }
    return &tile_grid[y*grid_spec.size.x + x];
}

TileJumpTarget Device::resolveJump(const Tile& from, int src_node, const std::string& src_wire_name) const
{
    if (!from.cb_type) {
        return {};
    }
    std::string src_wire = src_wire_name;
    if (src_wire.empty()) {
        const std::string* src_name = from.cb_type->nodeName(CB_NODE_SRC, src_node);
        if (src_name) {
            src_wire = *src_name;
        }
    }
    if (src_wire.empty()) {
        return {};
    }

    struct SearchState
    {
        const Tile* tile = nullptr;
        std::string wire;
        int depth = 0;
    };
    auto tile_at = [&](Coord coord) -> const Tile* {
        if (coord.x < 0 || coord.y < 0 || coord.x >= size_width || coord.y >= size_height) {
            return nullptr;
        }
        return &tile_grid[coord.y*grid_spec.size.x + coord.x];
    };

    std::vector<SearchState> queue;
    queue.push_back(SearchState{&from, src_wire, 0});
    size_t queue_pos = 0;
    std::unordered_set<std::string> visited;
    while (queue_pos < queue.size()) {
        SearchState state = queue[queue_pos++];
        if (!state.tile || state.depth > 64) {
            continue;
        }

        std::string curr_type = tileTypeName(*state.tile);
        if (curr_type.empty()) {
            continue;
        }
        std::string visit_key = curr_type + "@" + std::to_string(state.tile->coord.x) + "," + std::to_string(state.tile->coord.y) + ":" + state.wire;
        if (!visited.insert(visit_key).second) {
            continue;
        }

        if (state.depth > 0 && state.tile->cb_type) {
            int dst_node = state.tile->cb_type->nodeNum(CB_NODE_DST, state.wire);
            if (dst_node >= 0) {
                return TileJumpTarget{const_cast<Tile*>(state.tile), dst_node, state.wire};
            }
        }

        auto rules_it = tile_conn_by_from.find(tileConnKey(curr_type, state.wire));
        if (rules_it != tile_conn_by_from.end()) {
            for (size_t rule_index : rules_it->second) {
                const TileConnRule& rule = tile_conn_rules[rule_index];
                if (rule.from_tile_type != curr_type) {
                    continue;
                }
                for (const TileConnWirePair& pair : rule.wire_pairs) {
                    if (pair.from_wire != state.wire) {
                        continue;
                    }
                    Coord next_coord = state.tile->coord + rule.delta;
                    const Tile* next_tile = tile_at(next_coord);
                    if (next_tile && tileTypeName(*next_tile) == rule.to_tile_type) {
                        queue.push_back(SearchState{next_tile, pair.to_wire, state.depth + 1});
                    }
                }
            }
        }

        rules_it = tile_conn_by_to.find(tileConnKey(curr_type, state.wire));
        if (rules_it != tile_conn_by_to.end()) {
            for (size_t rule_index : rules_it->second) {
                const TileConnRule& rule = tile_conn_rules[rule_index];
                if (rule.to_tile_type != curr_type) {
                    continue;
                }
                for (const TileConnWirePair& pair : rule.wire_pairs) {
                    if (pair.to_wire != state.wire) {
                        continue;
                    }
                    Coord next_coord = state.tile->coord - rule.delta;
                    const Tile* next_tile = tile_at(next_coord);
                    if (next_tile && tileTypeName(*next_tile) == rule.from_tile_type) {
                        queue.push_back(SearchState{next_tile, pair.from_wire, state.depth + 1});
                    }
                }
            }
        }
    }

    return {};
}
