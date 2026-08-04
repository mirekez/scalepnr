#include "Device.h"

#include <json/json.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

struct Failure : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw Failure(message);
    }
}

std::string graphKey(const std::string& tile_type, const std::string& wire)
{
    return tile_type + "\n" + wire;
}

std::string stateKey(fpga::Coord coord, const std::string& tile_type, const std::string& wire)
{
    return std::to_string(coord.x) + "," + std::to_string(coord.y)
        + "\n" + tile_type + "\n" + wire;
}

uint16_t baseTypeId(const fpga::CBType& type)
{
    return type.base_type_id != CB_INVALID_TYPE_ID ? type.base_type_id : type.type_id;
}

void readTechMap(const std::string& text, fpga::TechMap& map)
{
    map.clear();
    std::stringstream lines(text);
    std::string line;
    while (std::getline(lines, line, '\n')) {
        map.emplace_back();
        std::stringstream expressions(line);
        std::string expression;
        while (std::getline(expressions, expression, ';')) {
            map.back().emplace_back();
            std::stringstream equals(expression);
            std::string equal;
            while (std::getline(equals, equal, '=')) {
                map.back().back().emplace_back();
                std::stringstream tokens(equal);
                std::string token;
                while (std::getline(tokens, token, ':')) {
                    map.back().back().back().emplace_back();
                    std::stringstream parts(token);
                    std::string part;
                    while (std::getline(parts, part, ',')) {
                        map.back().back().back().back().push_back(part);
                    }
                }
            }
        }
    }
}

fpga::TechMap makeCbMap()
{
    fpga::TechMap map;
    readTechMap(
        "BEG=SRC;END=DST;_S0=_SA;_S3=_SD;_N3=_ND;BOUNCE=JOINTA;ALT=JOINTB\n"
        "WL=6:1,1,1,1;WR=6:1,1,1,1;EL=2:1,1,1,1;ER=2:1,1,1,1;"
        "NL=0:1,1,1,1;NR=0:1,1,1,1;SL=4:1,1,1,1;SR=4:1,1,1,1;"
        "W=6:1,2,4,6;E=2:1,2,4,6;NW=7:1,1,2,3;NE=1:1,1,2,3;"
        "N=0:1,2,4,6;SW=5:1,2,2,3;SE=3:1,2,2,3;S=4:1,2,4,6\n"
        "LOGIC_OUTS=0;IMUX=1;BYP=2;GFAN=2",
        map);
    return map;
}

fpga::TechMap makeTileMap()
{
    fpga::TechMap map;
    readTechMap(
        "39WE,17AI,16A,17A1,18A2,19A3,20A4,21A5,22A6,17AMUX,1AQ,31AX,"
        "80B,81B1,82B2,83B3,84B4,85B5,86B6,81BMUX,65BQ,95BX,"
        "144C,145C1,146C2,147C3,148C4,149C5,150C6,1CE,9CIN,0CLK,145CMUX,63COUT,129CQ,130CX,"
        "212D,213D1,214D2,215D3,216D4,217D5,218D6,213DMUX,197DQ,198DX,199SR,"
        "240I,241O,242T",
        map);
    return map;
}

const std::vector<std::string>& routeCbTypes()
{
    static const std::vector<std::string> names = {
        "BRAM_INT_INTERFACE_L", "BRAM_INT_INTERFACE_R", "CFG_CENTER_BOT",
        "CFG_CENTER_MID", "CFG_CENTER_TOP", "HCLK_BRAM", "HCLK_CLB", "HCLK_CMT",
        "HCLK_CMT_L", "HCLK_DSP_L", "HCLK_DSP_R", "HCLK_FEEDTHRU_1",
        "HCLK_FEEDTHRU_2", "HCLK_FIFO_L", "HCLK_GTX", "HCLK_INT_INTERFACE",
        "HCLK_IOB", "HCLK_IOI3", "HCLK_L", "HCLK_L_BOT_UTURN", "HCLK_R",
        "HCLK_R_BOT_UTURN", "HCLK_TERM", "HCLK_TERM_GTX", "HCLK_VBRK",
        "HCLK_VFRAME", "INT_FEEDTHRU_1", "INT_FEEDTHRU_2", "INT_INTERFACE_L",
        "INT_INTERFACE_R", "INT_L", "INT_R", "IO_INT_INTERFACE_L",
        "IO_INT_INTERFACE_R", "LIOI3", "LIOI3_SING", "LIOI3_TBYTESRC",
        "LIOI3_TBYTETERM", "L_TERM_INT", "MONITOR_BOT", "MONITOR_MID",
        "MONITOR_TOP", "PCIE_INT_INTERFACE_L", "PCIE_INT_INTERFACE_R", "RIOI3",
        "RIOI3_SING", "RIOI3_TBYTESRC", "RIOI3_TBYTETERM", "R_TERM_INT",
        "R_TERM_INT_GTX", "T_TERM_INT", "VBRK", "VBRK_EXT", "VFRAME",
    };
    return names;
}

struct RawEdge
{
    std::string to_type;
    std::string to_wire;
    fpga::Coord delta;
    size_t line = 0;
};

struct RawTileConn
{
    std::unordered_map<std::string, std::vector<RawEdge>> edges;
    std::unordered_map<std::string, std::vector<std::string>> wires_by_type;
};

size_t findJsonStringLine(const std::vector<std::string>& lines, const std::string& value,
                          size_t& cursor)
{
    const std::string needle = "\"" + value + "\"";
    for (; cursor < lines.size(); ++cursor) {
        if (lines[cursor].find(needle) != std::string::npos) {
            return ++cursor;
        }
    }
    throw Failure("could not locate JSON value '" + value + "' in tileconn source text");
}

RawTileConn loadRawTileConn(const std::filesystem::path& filename)
{
    std::ifstream text_input(filename);
    require(static_cast<bool>(text_input), "cannot open " + filename.string());
    std::vector<std::string> lines;
    for (std::string line; std::getline(text_input, line);) {
        lines.push_back(std::move(line));
    }

    std::ifstream json_input(filename);
    Json::Value root;
    Json::Reader reader;
    require(reader.parse(json_input, root), "cannot parse " + filename.string());

    RawTileConn result;
    size_t cursor = 0;
    for (const Json::Value& item : root) {
        const Json::Value& types = item["tile_types"];
        const Json::Value& delta = item["grid_deltas"];
        const std::string from_type = types[0].asString();
        const std::string to_type = types[1].asString();
        (void)findJsonStringLine(lines, from_type, cursor);
        (void)findJsonStringLine(lines, to_type, cursor);
        fpga::Coord forward{delta[0].asInt(), delta[1].asInt()};
        for (const Json::Value& pair : item["wire_pairs"]) {
            const std::string from_wire = pair[0].asString();
            const std::string to_wire = pair[1].asString();
            size_t line = findJsonStringLine(lines, from_wire, cursor);
            (void)findJsonStringLine(lines, to_wire, cursor);
            result.edges[graphKey(from_type, from_wire)].push_back(
                RawEdge{to_type, to_wire, forward, line});
            result.edges[graphKey(to_type, to_wire)].push_back(
                RawEdge{from_type, from_wire, fpga::Coord{-forward.x, -forward.y}, line});
            result.wires_by_type[from_type].push_back(from_wire);
            result.wires_by_type[to_type].push_back(to_wire);
        }
    }
    for (auto& [type, wires] : result.wires_by_type) {
        (void)type;
        std::sort(wires.begin(), wires.end());
        wires.erase(std::unique(wires.begin(), wires.end()), wires.end());
    }
    return result;
}

bool sameEntry(const fpga::CBType::ResolvedJump& left,
               const fpga::CBType::ResolvedJump& right)
{
    return left.delta.x == right.delta.x && left.delta.y == right.delta.y
        && left.target_cb_type_id == right.target_cb_type_id
        && left.dsts.jump == right.dsts.jump
        && left.dst_wires == right.dst_wires
        && left.target_tile_coord == right.target_tile_coord;
}

bool sameEntries(const std::vector<fpga::CBType::ResolvedJump>& left,
                 const std::vector<fpga::CBType::ResolvedJump>& right)
{
    return left.size() == right.size()
        && std::equal(left.begin(), left.end(), right.begin(), sameEntry);
}

struct ProvenancePath
{
    bool found = false;
    std::vector<size_t> lines;
};

ProvenancePath findRawPath(const fpga::Device& device, const RawTileConn& raw,
                           const fpga::Tile& source, const fpga::CBType& source_base,
                           int src_node, const fpga::Tile& target,
                           const fpga::CBType& target_base, int dst_node)
{
    struct Item
    {
        fpga::Coord coord;
        std::string type;
        std::string wire;
        int depth = 0;
        std::vector<size_t> lines;
    };

    auto source_wires_it = raw.wires_by_type.find(source_base.name);
    if (source_wires_it == raw.wires_by_type.end()) {
        return {};
    }
    std::deque<Item> queue;
    std::unordered_set<std::string> seen;
    for (const std::string& wire : source_wires_it->second) {
        int node = fpga::testRouteSrcNodeByPhysicalWireName(
            const_cast<fpga::CBType&>(source_base), wire);
        if (node != src_node) {
            continue;
        }
        std::string key = stateKey(source.coord, source_base.name, wire);
        if (seen.insert(key).second) {
            queue.push_back(Item{source.coord, source_base.name, wire, 0, {}});
        }
    }

    constexpr int max_depth = 64;
    while (!queue.empty()) {
        Item item = std::move(queue.front());
        queue.pop_front();
        if (item.coord.x == target.coord.x && item.coord.y == target.coord.y
            && item.type == target_base.name
            && fpga::testRouteDstNodeByPhysicalWireName(
                const_cast<fpga::CBType&>(target_base), item.wire) == dst_node) {
            return ProvenancePath{true, std::move(item.lines)};
        }
        if (item.depth >= max_depth) {
            continue;
        }
        auto edges_it = raw.edges.find(graphKey(item.type, item.wire));
        if (edges_it == raw.edges.end()) {
            continue;
        }
        for (const RawEdge& edge : edges_it->second) {
            fpga::Coord next{item.coord.x + edge.delta.x, item.coord.y + edge.delta.y};
            if (next.x < 0 || next.y < 0 || next.x >= device.size_width || next.y >= device.size_height) {
                continue;
            }
            const fpga::Tile& grid_tile = device.tile_grid[next.y * device.size_width + next.x];
            if (!grid_tile.tile_type || grid_tile.tile_type->name != edge.to_type) {
                continue;
            }
            std::string key = stateKey(next, edge.to_type, edge.to_wire);
            if (!seen.insert(key).second) {
                continue;
            }
            std::vector<size_t> path_lines = item.lines;
            path_lines.push_back(edge.line);
            queue.push_back(Item{next, edge.to_type, edge.to_wire,
                                 item.depth + 1, std::move(path_lines)});
        }
    }
    return {};
}

fpga::Tile* representativeTile(fpga::Device& device, uint16_t type_id)
{
    for (auto& tile_ref : device.tile_grid) {
        fpga::Tile& tile = tile_ref;
        if (tile.cb_type && tile.cb_type->type_id == type_id
            && tile.coord == tile.cb_coord) {
            return &tile;
        }
    }
    return nullptr;
}

fpga::Tile* targetTile(fpga::Device& device, const fpga::Tile& source,
                       const fpga::CBType::ResolvedJump& entry)
{
    fpga::Coord origin = entry.target_tile_coord ? source.coord : source.cb_coord;
    fpga::Coord expected{origin.x + entry.delta.x, origin.y + entry.delta.y};
    fpga::Tile* attached = nullptr;
    for (auto& tile_ref : device.tile_grid) {
        fpga::Tile& tile = tile_ref;
        if (!tile.cb_type || baseTypeId(*tile.cb_type) != entry.target_cb_type_id) {
            continue;
        }
        if (entry.target_tile_coord) {
            if (tile.coord == expected) {
                return &tile;
            }
            continue;
        }
        if (tile.cb_coord != expected) {
            continue;
        }
        if (tile.coord == tile.cb_coord) {
            return &tile;
        }
        attached = &tile;
    }
    return attached;
}

void loadA7Device(fpga::Device& device, const std::filesystem::path& db)
{
    fpga::TechMap cb_map = makeCbMap();
    for (const std::string& type : routeCbTypes()) {
        std::filesystem::path filename = db / ("tile_type_" + type + ".json");
        if (std::filesystem::exists(filename)) {
            device.loadCBFromSpec(filename.string(), cb_map);
        }
    }

    std::vector<std::filesystem::path> specs;
    for (const auto& entry : std::filesystem::directory_iterator(db)) {
        if (entry.is_regular_file()
            && entry.path().filename().string().rfind("tile_type_", 0) == 0
            && entry.path().extension() == ".json") {
            specs.push_back(entry.path());
        }
    }
    std::sort(specs.begin(), specs.end());
    fpga::TechMap tile_map = makeTileMap();
    for (int pass = 0; pass != 2; ++pass) {
        for (const std::filesystem::path& spec : specs) {
            device.loadTypeFromSpec(spec.string(), tile_map);
        }
    }
    device.loadFromSpec((db / "tilegrid.json").string(),
                        (db / "package_pins.csv").string());
}

void checkFocusedBackwardIndex(fpga::Device& device, const RawTileConn& raw)
{
    fpga::Tile* target = device.getTile(115, 120);
    fpga::Tile* source = device.getTile(121, 120);
    require(target && target->cb_type, "focused target tile has no crossbar");
    require(source && source->cb_type, "focused source tile has no crossbar");

    const uint16_t target_base_id = baseTypeId(*target->cb_type);
    const uint16_t source_base_id = baseTypeId(*source->cb_type);
    require(target_base_id < device.cb_types.size(), "focused target has invalid base type");
    require(source_base_id < device.cb_types.size(), "focused source has invalid base type");
    fpga::CBType& target_base = device.cb_types[target_base_id];
    fpga::CBType& source_base = device.cb_types[source_base_id];
    int target_dst = fpga::testRouteDstNodeByPhysicalWireName(target_base, "WW2END0");
    int source_src = fpga::testRouteSrcNodeByPhysicalWireName(source_base, "WW2BEG0");
    require(target_dst >= 0, "focused WW2END0 is not a switchable destination");
    require(source_src >= 0, "focused WW2BEG0 is not a structural source");

    ProvenancePath raw_path = findRawPath(device, raw, *source, source_base,
        source_src, *target, target_base, target_dst);
    require(raw_path.found,
        "raw tileconn graph does not contain focused WW2BEG0 -> WW2END0 chain");

    bool found_source = false;
    size_t incoming = 0;
    std::cout << "focused backward index target=(115,120) dst=" << target_dst
              << " WW2END0 raw_lines=" << raw_path.lines.size() << '\n';
    constexpr int reverse_radius = 12;
    for (int y = target->coord.y - reverse_radius;
         y <= target->coord.y + reverse_radius; ++y) {
        for (int x = target->coord.x - reverse_radius;
             x <= target->coord.x + reverse_radius; ++x) {
            fpga::Tile* candidate_ptr = device.getTile(x, y);
            if (!candidate_ptr || !candidate_ptr->cb_type) {
                continue;
            }
            fpga::Tile& candidate = *candidate_ptr;
            for (const auto& [src_node, entries] : candidate.cb_type->dst_by_src.values) {
                if (entries.empty()) {
                    continue;
                }
                for (const fpga::TileJumpTarget& resolved :
                     device.resolveJumpTargets(candidate, src_node)) {
                    if (resolved.tile != target || resolved.dst_node != target_dst) {
                        continue;
                    }
                    const std::string* source_name =
                        candidate.cb_type->nodeName(fpga::CB_NODE_SRC, src_node);
                    std::cout << "  source=(" << candidate.coord.x << ',' << candidate.coord.y
                              << ") src=" << src_node << ' '
                              << (source_name ? *source_name : std::string{"<unnamed>"}) << '\n';
                    ++incoming;
                    found_source |= candidate.coord == source->coord && src_node == source_src;
                }
            }
        }
    }
    require(incoming != 0, "focused WW2END0 has an empty constructed backward index");
    require(found_source,
        "constructed backward index omits (121,120)/WW2BEG0 for (115,120)/WW2END0");
}

void checkFocusedLongPassThrough(fpga::Device& device, const RawTileConn& raw)
{
    fpga::Tile* source = device.getTile(25, 157);
    fpga::Tile* expected = device.getTile(36, 157);
    require(source && source->cb_type, "focused long-pass source has no crossbar");
    require(expected && expected->cb_type, "focused long-pass target has no crossbar");
    const uint16_t source_base_id = baseTypeId(*source->cb_type);
    const uint16_t target_base_id = baseTypeId(*expected->cb_type);
    fpga::CBType& source_base = device.cb_types[source_base_id];
    fpga::CBType& target_base = device.cb_types[target_base_id];
    int source_node = fpga::testRouteSrcNodeByPhysicalWireName(
        source_base, "EE4BEG0");
    int target_node = fpga::testRouteDstNodeByPhysicalWireName(
        target_base, "EE4END0");
    require(source_node >= 0 && target_node >= 0,
        "focused long-pass endpoint nodes were not loaded");
    ProvenancePath path = findRawPath(device, raw, *source, source_base,
        source_node, *expected, target_base, target_node);
    require(path.found,
        "raw tileconn graph lacks the focused long pass-through path");
    fpga::TileJumpTarget resolved = device.resolveJump(*source, source_node);
    require(resolved.tile == expected && resolved.dst_node == target_node,
        "subtype cache aliased the focused long pass-through endpoint");
}

void checkFocusedLocalTransition(fpga::Device& device, const std::string& phase = {})
{
    // A known numeric local-to-local tile connection must survive subtype construction.
    // This guards dedicated networks embedded in otherwise ordinary routing tiles.
    fpga::Tile* source = device.getTile(132, 112);
    fpga::Tile* target = device.getTile(131, 112);
    require(source && source->cb_type, "focused local source has no crossbar");
    require(target && target->cb_type, "focused local target has no crossbar");
    int source_local = source->cb_type->nodeNum(fpga::CB_NODE_LOCAL, "GCLK_B0_WEST");
    int target_local = target->cb_type->nodeNum(fpga::CB_NODE_LOCAL, "GCLK_L_B0");
    require(source_local >= 0, "focused source local was not loaded");
    require(target_local >= 0, "focused target local was not loaded");
    std::vector<fpga::TileLocalTarget> resolved = device.resolveLocalTargets(*source, source_local);
    require(std::any_of(resolved.begin(), resolved.end(), [&](const fpga::TileLocalTarget& item) {
        return item.tile == target && item.node_type == fpga::CB_NODE_LOCAL
            && item.node == target_local;
    }), "focused numeric local tile connection was lost" +
        (phase.empty() ? std::string{} : " after " + phase));
    int vertical_local = source->cb_type->nodeNum(fpga::CB_NODE_LOCAL, "GCLK_B0");
    require(vertical_local >= 0, "focused vertical local was not loaded");
    resolved = device.resolveLocalTargets(*source, vertical_local);
    require(std::any_of(resolved.begin(), resolved.end(), [&](const fpga::TileLocalTarget& item) {
        return item.tile && item.tile->coord.x == source->coord.x
            && std::abs(item.tile->coord.y - source->coord.y) == 1
            && item.node_type == fpga::CB_NODE_LOCAL && item.node == vertical_local;
    }), "focused numeric vertical local connection was lost" +
        (phase.empty() ? std::string{} : " after " + phase));
}

void checkDeferredLoadsPreserveLocalTransitions(fpga::Device& device,
                                                const std::filesystem::path& db)
{
    // Post-grid dedicated crossbar loads rebuild numeric local links repeatedly.
    // Every rebuild must preserve previously constructed ordinary-tile links.
    fpga::TechMap map = makeCbMap();
    const std::vector<std::string> dedicated = {
        "BRKH_CLK", "CLK_BUFG_BOT_R", "CLK_BUFG_REBUF", "CLK_BUFG_TOP_R",
        "CLK_FEED", "CLK_HROW_BOT_R", "CLK_HROW_TOP_R"};
    for (const std::string& type : dedicated) {
        device.loadCBFromSpec((db / ("tile_type_" + type + ".json")).string(), map);
        checkFocusedLocalTransition(device, type);
    }
    device.loadCBFromSpec((db / "tile_type_BRKH_INT.json").string(), map, true);
    checkFocusedLocalTransition(device, "local fabric");
    const std::vector<std::string> pass_through = {
        "CLK_MTBF2", "CLK_PMV", "CLK_PMV2", "CLK_PMV2_SVT", "CLK_PMVIOB", "CLK_TERM"};
    for (const std::string& type : pass_through) {
        device.loadCBFromSpec((db / ("tile_type_" + type + ".json")).string(), map, true);
        checkFocusedLocalTransition(device, type);
    }
}

void runA7SubtypeReverseTest()
{
    const std::filesystem::path db =
        std::filesystem::path(SCALEPNR_SOURCE_DIR) / "tests/prjxray/db";
    require(std::filesystem::exists(db / "tileconn.json"),
        "A7 database is not prepared at " + db.string());

    RawTileConn raw = loadRawTileConn(db / "tileconn.json");
    fpga::Device& device = fpga::Device::current();
    loadA7Device(device, db);
    const fpga::SubtypeBuildStats& stats = device.last_subtype_build;

    checkFocusedBackwardIndex(device, raw);
    checkFocusedLongPassThrough(device, raw);
    checkFocusedLocalTransition(device);

    require(stats.created_subtypes != 0, "A7 load created no routing subtypes");
    require(stats.created_subtypes <= 1500,
        "A7 subtype count inflated to " + std::to_string(stats.created_subtypes));
    require(stats.signature_misses <= 7000,
        "A7 subtype signature misses inflated to " + std::to_string(stats.signature_misses));
    require(stats.elapsed_seconds <= 90.0,
        "A7 subtype construction took " + std::to_string(stats.elapsed_seconds) + " seconds");
    require(stats.final_types == stats.initial_types + stats.created_subtypes,
        "reported subtype count does not match CBType storage growth");

    std::unordered_set<uint16_t> used_types;
    for (const auto& tile_ref : device.tile_grid) {
        if (tile_ref.cb_type) {
            used_types.insert(tile_ref.cb_type->type_id);
        }
    }

    size_t checked_subtypes = 0;
    size_t checked_sources = 0;
    size_t changed_sources = 0;
    size_t checked_connections = 0;
    size_t provenance_lines = 0;
    for (const fpga::CBType& subtype : device.cb_types) {
        if (subtype.type_id < stats.initial_types) {
            continue;
        }
        require(used_types.contains(subtype.type_id),
            "generated subtype " + std::to_string(subtype.type_id) + " is unused");
        require(subtype.base_type_id < stats.initial_types,
            "generated subtype has invalid base type " + std::to_string(subtype.base_type_id));
        fpga::Tile* source = representativeTile(device, subtype.type_id);
        require(source != nullptr,
            "generated subtype " + std::to_string(subtype.type_id) + " has no physical representative tile");
        const fpga::CBType& base = device.cb_types[subtype.base_type_id];
        ++checked_subtypes;

        for (const auto& [src_node, entries] : subtype.dst_by_src.values) {
            if (entries.empty()) {
                continue;
            }
            ++checked_sources;
            if (!sameEntries(entries, base.dst_by_src[src_node])) {
                ++changed_sources;
            }
            for (const fpga::CBType::ResolvedJump& entry : entries) {
                require(entry.target_cb_type_id < stats.initial_types,
                    "subtype target references another generated subtype");
                fpga::Tile* target = targetTile(device, *source, entry);
                require(target != nullptr,
                    "subtype " + std::to_string(subtype.type_id) + " src "
                        + std::to_string(src_node) + " has no target tile for delta ("
                        + std::to_string(entry.delta.x) + "," + std::to_string(entry.delta.y) + ")");
                const fpga::CBType& target_base = device.cb_types[entry.target_cb_type_id];
                entry.dsts.jump.for_each_set_bit([&](int dst_node) {
                    ProvenancePath path = findRawPath(device, raw, *source, base,
                        src_node, *target, target_base, dst_node);
                    require(path.found,
                        "no tileconn database path for subtype " + std::to_string(subtype.type_id)
                            + " base '" + base.name + "' at (" + std::to_string(source->coord.x)
                            + "," + std::to_string(source->coord.y) + ") src="
                            + std::to_string(src_node) + " to base '" + target_base.name
                            + "' at (" + std::to_string(target->coord.x) + ","
                            + std::to_string(target->coord.y) + ") dst="
                            + std::to_string(dst_node));
                    require(!path.lines.empty(),
                        "resolved subtype connection has no responsible tileconn source line");
                    provenance_lines += path.lines.size();
                    ++checked_connections;
                    return false;
                });
            }
        }
    }

    require(checked_subtypes == stats.created_subtypes,
        "not every generated subtype was reverse-tested");
    require(checked_connections != 0, "no subtype connections were reverse-tested");
    checkDeferredLoadsPreserveLocalTransitions(device, db);
    std::cout << "A7 subtype reverse test: base_types=" << stats.initial_types
              << " subtypes=" << checked_subtypes
              << " specialized_tiles=" << stats.specialized_tiles
              << " sources=" << checked_sources
              << " changed_sources=" << changed_sources
              << " connections=" << checked_connections
              << " provenance_lines=" << provenance_lines
              << " signature_hits=" << stats.signature_hits
              << " signature_misses=" << stats.signature_misses
              << " phase_seconds=" << stats.signature_seconds << ','
              << stats.candidate_copy_seconds << ',' << stats.mapping_seconds << ','
              << stats.target_search_seconds << ','
              << stats.dedup_seconds
              << " build_seconds=" << stats.elapsed_seconds << '\n';
}

}

int main()
{
    try {
        runA7SubtypeReverseTest();
    }
    catch (const std::exception& error) {
        std::cerr << "subtype_test failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
