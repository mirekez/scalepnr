#include "Device.h"
#include "Docking.h"

#include <cstdint>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

using IndexKey = std::tuple<int, int, int>;
using SourceKey = std::tuple<int, int, int, int>;
using NormalizedIndex = std::map<IndexKey, std::set<SourceKey>>;

struct ExpectedEdge
{
    fpga::Coord source;
    int src = -1;
    fpga::Coord target;
    int dst = -1;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

NodeMask bit(int node)
{
    return NodeMask{0, 1} << node;
}

bool inWindow(fpga::Coord coord, fpga::Coord center, int radius)
{
    return coord.x >= center.x - radius && coord.x <= center.x + radius
        && coord.y >= center.y - radius && coord.y <= center.y + radius;
}

int tileIndex(fpga::Coord coord, int width)
{
    return coord.y * width + coord.x;
}

void resetDevice(int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.tile_types.clear();
    device.wires.clear();
    device.wire_grid.clear();
    device.local_route_wire_mappings.clear();
    device.route_wire_graph.clear();
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;

    const int tile_count = width * height;
    device.cb_types.resize(static_cast<size_t>(tile_count));
    device.tile_grid.resize(static_cast<size_t>(tile_count));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = tileIndex({x, y}, width);
            fpga::CBType& cb_type = device.cb_types[static_cast<size_t>(index)];
            cb_type.name = "GENERIC_TYPE_" + std::to_string(index);
            cb_type.type_id = static_cast<uint16_t>(index);

            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(index)];
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = tile.coord;
            tile.cb = {};
            tile.cb_type = &cb_type;
            tile.cb.type = &cb_type;
            tile.pin_state = {};
            tile.routedNets.clear();
        }
    }
}

void addTargetPin(fpga::CBType& cb_type, int dst)
{
    constexpr int local = 0;
    cb_type.dst_local[dst].local |= bit(local);
    cb_type.rememberNodeName(fpga::CB_NODE_DST, dst,
        "DESTINATION_NODE_" + std::to_string(dst));
    cb_type.rememberNodeName(fpga::CB_NODE_LOCAL, local, "LOCAL_ENDPOINT");
    cb_type.derived_masks_valid = false;
}

void addMapping(fpga::Device& device, int width, fpga::Coord source, int src,
                fpga::Coord target, const std::vector<int>& dsts,
                uint16_t target_type, bool duplicate_entry = false)
{
    fpga::CBType& source_type =
        device.cb_types[static_cast<size_t>(tileIndex(source, width))];
    source_type.rememberNodeName(fpga::CB_NODE_SRC, src,
        "SOURCE_NODE_" + std::to_string(src));

    fpga::CBJumpState destination_state{};
    for (int dst : dsts) {
        destination_state.jump |= bit(dst);
    }
    fpga::CBType::ResolvedJump mapping{};
    mapping.delta = target - source;
    mapping.target_cb_type_id = target_type;
    mapping.dsts = destination_state;
    mapping.target_tile_coord = true;
    source_type.dst_by_src[src].push_back(mapping);
    if (duplicate_entry) {
        source_type.dst_by_src[src].push_back(mapping);
    }
}

NormalizedIndex normalize(const pnr::BackwardResolveIndex& index)
{
    NormalizedIndex result;
    for (const auto& [target, sources] : index.sources) {
        IndexKey target_key{target.x, target.y, target.dst};
        for (const pnr::BackwardResolveSource& source : sources) {
            require(source.tile != nullptr, "reverse index contains a null source tile");
            result[target_key].insert(SourceKey{
                source.tile->coord.x, source.tile->coord.y,
                source.src, source.route_jump});
        }
    }
    return result;
}

NormalizedIndex expectedIndex(const std::vector<ExpectedEdge>& edges,
                              fpga::Coord center, int radius, bool filtered)
{
    NormalizedIndex result;
    for (const ExpectedEdge& edge : edges) {
        if (!inWindow(edge.source, center, radius)
            || !inWindow(edge.target, center, radius)
            || (filtered && ((edge.source.x + edge.source.y) & 1) != 0)) {
            continue;
        }
        result[IndexKey{edge.target.x, edge.target.y, edge.dst}].insert(
            SourceKey{edge.source.x, edge.source.y, edge.src, edge.src});
    }
    return result;
}

int expectedScanCount(fpga::Device& device, fpga::Coord center,
                      int radius, bool filtered)
{
    int scans = 0;
    for (int y = center.y - radius; y <= center.y + radius; ++y) {
        for (int x = center.x - radius; x <= center.x + radius; ++x) {
            if (filtered && ((x + y) & 1) != 0) {
                continue;
            }
            fpga::Tile* tile = device.getTile(x, y);
            if (tile && tile->cb_type) {
                scans += static_cast<int>(tile->cb_type->dst_by_src.values.size());
            }
        }
    }
    return scans;
}

void verifyIndex(const pnr::BackwardResolveIndex& actual,
                 const NormalizedIndex& expected, int expected_scans,
                 const std::string& context)
{
    NormalizedIndex normalized = normalize(actual);
    require(normalized == expected, context + ": reverse index differs from generated mappings");
    require(actual.mapping_scan_count == expected_scans,
        context + ": source mapping scan count is incorrect");

    int expected_reaches = 0;
    for (const auto& [key, sources] : expected) {
        (void)key;
        expected_reaches += static_cast<int>(sources.size());
    }
    require(actual.mapping_reaches_count == expected_reaches,
        context + ": resolved reverse-edge count is incorrect");
}

void runRandomArena(uint32_t seed)
{
    constexpr int width = 14;
    constexpr int height = 14;
    constexpr int radius = 6;
    constexpr fpga::Coord center{7, 7};
    resetDevice(width, height);
    fpga::Device& device = fpga::Device::current();

    std::mt19937 random(seed);
    std::uniform_int_distribution<int> coordinate(0, width - 1);
    std::uniform_int_distribution<int> offset(-4, 4);
    std::vector<int> next_src(static_cast<size_t>(width * height), 0);
    std::vector<int> next_dst(static_cast<size_t>(width * height), 0);
    std::vector<ExpectedEdge> expected_edges;

    for (int connection = 0; connection < 260; ++connection) {
        fpga::Coord source{coordinate(random), coordinate(random)};
        fpga::Coord target;
        do {
            target = {source.x + offset(random), source.y + offset(random)};
        } while ((target.x < 0 || target.x >= width || target.y < 0 || target.y >= height)
                 || (target.x == source.x && target.y == source.y));

        int source_index = tileIndex(source, width);
        int target_index = tileIndex(target, width);
        int src = next_src[static_cast<size_t>(source_index)]++;
        int dst = next_dst[static_cast<size_t>(target_index)]++;
        addTargetPin(device.cb_types[static_cast<size_t>(target_index)], dst);
        addMapping(device, width, source, src, target, {dst},
            static_cast<uint16_t>(target_index), connection % 17 == 0);
        expected_edges.push_back(ExpectedEdge{source, src, target, dst});

        // One source may resolve to several destination bits in one numeric mapping.
        if (connection % 13 == 0) {
            int second_dst = next_dst[static_cast<size_t>(target_index)]++;
            addTargetPin(device.cb_types[static_cast<size_t>(target_index)], second_dst);
            fpga::CBType& source_type = device.cb_types[static_cast<size_t>(source_index)];
            source_type.dst_by_src[src].back().dsts.jump |= bit(second_dst);
            if (connection % 17 == 0) {
                source_type.dst_by_src[src].front().dsts.jump |= bit(second_dst);
            }
            expected_edges.push_back(ExpectedEdge{source, src, target, second_dst});
        }
    }

    // Invalid destinations and target types must never appear in the reverse index.
    for (int invalid = 0; invalid < 30; ++invalid) {
        fpga::Coord source{coordinate(random), coordinate(random)};
        fpga::Coord target{coordinate(random), coordinate(random)};
        int source_index = tileIndex(source, width);
        int target_index = tileIndex(target, width);
        int src = next_src[static_cast<size_t>(source_index)]++;
        int unregistered_dst = next_dst[static_cast<size_t>(target_index)]++;
        addMapping(device, width, source, src, target, {unregistered_dst},
            static_cast<uint16_t>(target_index));

        int wrong_src = next_src[static_cast<size_t>(source_index)]++;
        int valid_dst = next_dst[static_cast<size_t>(target_index)]++;
        addTargetPin(device.cb_types[static_cast<size_t>(target_index)], valid_dst);
        uint16_t wrong_type = static_cast<uint16_t>((target_index + 1) % (width * height));
        addMapping(device, width, source, wrong_src, target, {valid_dst}, wrong_type);
    }

    pnr::BackwardResolveIndex complete =
        pnr::buildBackwardResolveIndex(device, center, radius);
    verifyIndex(complete, expectedIndex(expected_edges, center, radius, false),
        expectedScanCount(device, center, radius, false),
        "seed " + std::to_string(seed) + " unfiltered");

    auto checkerboard_filter = [](const fpga::Coord& source) {
        return ((source.x + source.y) & 1) == 0;
    };
    pnr::BackwardResolveIndex filtered =
        pnr::buildBackwardResolveIndex(device, center, radius, checkerboard_filter);
    verifyIndex(filtered, expectedIndex(expected_edges, center, radius, true),
        expectedScanCount(device, center, radius, true),
        "seed " + std::to_string(seed) + " filtered");
}

} // namespace

int main()
{
    try {
        for (uint32_t iteration = 0; iteration < 20; ++iteration) {
            runRandomArena(0x4b1d0000u + iteration * 7919u);
        }
        std::cout << "backward resolve index: 20 randomized arenas passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "backward resolve index failed: " << error.what() << '\n';
        return 1;
    }
}
