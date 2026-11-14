#include "Device.h"
#include "Docking.h"

#include <cstdint>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <sstream>
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

// Exercise production reverse traversal, not a separate priority comparator.
// Grid scan order deliberately puts long incoming hops first in some octants.
static void shortHopPriority()
{
    const fpga::Coord directions[] = {
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1},
        {1, 0}, {1, 1}, {0, 1}, {-1, 1}};
    size_t cases = 0;
    for (int direction = 0; direction < 8; ++direction) {
        for (int cache_mode = 0; cache_mode < 3; ++cache_mode) {
            for (int blocked = 0; blocked < 3; ++blocked) {
                resetDevice(13, 13);
                auto& device = fpga::Device::current();
                const fpga::Coord center{6, 6};
                auto* target = device.getTile(center.x, center.y);
                constexpr int dst = 19, src = 71, local = 0;
                addTargetPin(*target->cb_type, dst);
                std::vector<fpga::Tile*> candidates;
                auto add = [&](fpga::Coord delta) {
                    auto coord = center + delta;
                    auto* tile = device.getTile(coord.x, coord.y);
                    tile->cb_type->local_src[local].jump |= bit(src);
                    tile->cb_type->rememberNodeName(fpga::CB_NODE_LOCAL, local, "ORIGIN_PIN");
                    tile->cb_type->rebuildOutgoingSrcs();
                    addMapping(device, 13, coord, src, center, {dst}, target->cb_type->type_id);
                    return tile;
                };
                const auto aim = directions[direction];
                for (int length : {1, 2, 4})
                    candidates.push_back(add({aim.x * length, aim.y * length}));
                // A shorter off-axis option must not override direction priority.
                auto* off_axis = add(directions[(direction + 2) % 8]);
                for (int i = 0; i < blocked; ++i)
                    candidates[i]->cb.src.jump |= bit(src);
                pnr::BackwardResolveCache cache;
                pnr::BackwardResolveIndex index;
                if (cache_mode == 2) {
                    // Populate long sources first, then extend the same shared cache.
                    auto initial = pnr::buildBackwardResolveIndex(device, center, 6,
                        [&](fpga::Coord c) { return c == candidates[2]->coord; }, &cache);
                    (void)initial;
                }
                index = pnr::buildBackwardResolveIndex(device, center, 6, {},
                                                       cache_mode ? &cache : nullptr);
                if (cache_mode) {
                    // A warmed filtered view must retain the same ordering.
                    pnr::resolveBackwardSources(index, {center.x, center.y, dst});
                }
                std::vector<fpga::Tile*> attempts;
                auto probe = [&](fpga::Tile& tile, int node, pnr::BackwardTakeoffChoice& choice) {
                    require(node == src, "unexpected reverse source node");
                    attempts.push_back(&tile);
                    choice.local = local;
                    return true;
                };
                auto result = pnr::routeBackwardToTakeoff(*target, bit(local),
                    center + fpga::Coord{aim.x * 5, aim.y * 5}, 5, 6, -1, probe,
                    &index, {}, 0, 0, 0, nullptr, nullptr, probe);
                // Only busy shorter hops may precede the first free preferred hop.
                require(result.success && result.source_tile == candidates[blocked],
                    "backward traversal did not prefer shortest free hop: direction=" +
                    std::to_string(direction) + " cache=" + std::to_string(cache_mode) +
                    " blocked=" + std::to_string(blocked) + " selected=(" +
                    std::to_string(result.source_tile ? result.source_tile->coord.x : -1) + "," +
                    std::to_string(result.source_tile ? result.source_tile->coord.y : -1) +
                    ") expected=(" + std::to_string(candidates[blocked]->coord.x) + "," +
                    std::to_string(candidates[blocked]->coord.y) + ")");
                require(attempts.size() == static_cast<size_t>(blocked + 1),
                    "reverse traversal tried longer/off-axis hops before shorter ones");
                for (size_t i = 0; i < attempts.size(); ++i)
                    require(attempts[i] == candidates[i], "short-hop attempt order changed");
                // One target expansion suffices; prioritization adds no search states.
                require(result.expanded == 1 && result.incoming_edges == static_cast<size_t>(blocked + 1),
                    "short-hop priority added unnecessary expansions");
                require(result.fragments.front().from == candidates[blocked]->coord &&
                            result.fragments.front().to == center,
                    "accepted backward route differs from the selected hop");
                // Probing must leave every lease exactly as it was before the search.
                for (const auto& tile : device.tile_grid) {
                    bool occupied = false;
                    for (int i = 0; i < blocked; ++i) occupied |= &tile == candidates[i];
                    require(tile.cb.src.jump == (occupied ? bit(src) : NodeMask{}) &&
                                tile.cb.dst.jump == NodeMask{} && tile.cb.joint.jump == NodeMask{} &&
                                tile.cb.local.local == NodeMask{}, "reverse priority probe changed leases");
                }
                require(result.source_tile != off_axis, "hop length overrode direction priority");
                ++cases;
            }
        }
    }
    std::cout << "backward short-hop priority: " << cases << " cases passed\n";
}

// Destination relocation must choose by physical hop length, not numeric SRC
// order, for both retained-DST branches and local starts, including joint exits.
static void destinationShortHopPriority()
{
    const fpga::Coord directions[] = {
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1},
        {1, 0}, {1, 1}, {0, 1}, {-1, 1}};
    size_t cases = 0;
    for (auto direction : directions) {
        for (bool from_dst : {false, true}) {
            for (bool through_joint : {false, true}) {
                for (int blocked = 0; blocked < 3; ++blocked) {
                    resetDevice(13, 13);
                    auto& device = fpga::Device::current();
                    const fpga::Coord center{6, 6};
                    auto* start = device.getTile(center.x, center.y);
                    constexpr int root = 17, dst = 19, local = 0, joint = 23;
                    // Smaller source IDs deliberately represent longer physical wires.
                    const int sources[] = {91, 51, 11};
                    const int lengths[] = {1, 2, 4};
                    std::vector<fpga::Tile*> targets;
                    auto& cb = *start->cb_type;
                    cb.rememberNodeName(from_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL,
                                        root, "BRANCH_ORIGIN");
                    cb.rememberNodeName(fpga::CB_NODE_JOINT, joint, "BRANCH_RELAY");
                    for (int i = 0; i < 3; ++i) {
                        auto coord = center + fpga::Coord{direction.x * lengths[i], direction.y * lengths[i]};
                        auto* target = device.getTile(coord.x, coord.y);
                        targets.push_back(target);
                        addTargetPin(*target->cb_type, dst);
                        addMapping(device, 13, center, sources[i], coord, {dst}, target->cb_type->type_id);
                        // Put the shortest path through a joint to defeat direct-first ordering too.
                        if (i == 0 && through_joint) {
                            if (from_dst) cb.dst_joint[root].joint |= bit(joint);
                            else cb.local_joint[root].joint |= bit(joint);
                            cb.joint_src[joint].jump |= bit(sources[i]);
                        } else if (from_dst) cb.dst_src[root].jump |= bit(sources[i]);
                        else cb.local_src[root].jump |= bit(sources[i]);
                        if (i < blocked) target->cb.dst.jump |= bit(dst);
                    }
                    cb.rebuildOutgoingSrcs();
                    // Keep an existing branch anchor leased throughout the speculative search.
                    if (from_dst) start->cb.dst.jump |= bit(root);
                    else start->cb.local.local |= bit(root);
                    std::vector<fpga::Tile*> probes;
                    auto result = pnr::routeForwardToPlacement({{start, root, from_dst, {}}},
                        [&](fpga::Tile& tile, int node, bool at_root, std::vector<fpga::Wire>& terminal) {
                            if (at_root) return false;
                            probes.push_back(&tile);
                            require(node == dst, "destination moving probed the wrong arrival");
                            // Emulate a compatible cell with a free terminal LOCAL at this reached tile.
                            fpga::Wire last;
                            last.type = fpga::Wire::WIRE_ROUTE_EDGE;
                            last.from = last.to = tile.coord;
                            last.from_node_type = fpga::CB_NODE_DST;
                            last.from_node = dst;
                            last.to_node_type = fpga::CB_NODE_LOCAL;
                            last.to_node = local;
                            terminal.push_back(last);
                            return true;
                        });
                    // Accept the shortest free landing, never the smaller-ID long hop.
                    require(result.success && probes.size() == 1 && probes.front() == targets[blocked],
                        "destination moving did not prefer shortest free hop: direction=(" +
                        std::to_string(direction.x) + "," + std::to_string(direction.y) +
                        ") blocked=" + std::to_string(blocked) +
                        " selected_src=" + std::to_string(result.fragments.empty() ? -1 : result.fragments.front().jump) +
                        " expected_src=" + std::to_string(sources[blocked]));
                    require(result.fragments.front().jump == sources[blocked] &&
                                result.fragments.front().to == targets[blocked]->coord,
                        "forward route does not contain the selected short hop");
                    // No longer/free branch needs probing before the shortest free one.
                    require(result.expanded == 2 && result.edge_checks == static_cast<size_t>(blocked + 1),
                        "destination short-hop ordering added extra searches");
                    // Return from success must undo temporary SRC/joint leases, retaining only fixtures.
                    for (const auto& tile : device.tile_grid) {
                        NodeMask expected_dst, expected_local;
                        if (&tile == start) {
                            if (from_dst) expected_dst |= bit(root);
                            else expected_local |= bit(root);
                        }
                        for (int i = 0; i < blocked; ++i)
                            if (&tile == targets[i]) expected_dst |= bit(dst);
                        require(tile.cb.dst.jump == expected_dst && tile.cb.local.local == expected_local &&
                                    tile.cb.src.jump == NodeMask{} && tile.cb.joint.jump == NodeMask{},
                            "destination moving priority probe leaked or removed leases");
                    }
                    ++cases;
                }
            }
        }
    }
    std::cout << "destination short-hop priority: " << cases << " cases passed\n";
}

static void destinationForwardChildRetry()
{
    resetDevice(6, 5);
    auto& device = fpga::Device::current();
    auto* root = device.getTile(2, 2);
    auto* dead = device.getTile(2, 1);
    auto* good = device.getTile(3, 2);
    auto* target = device.getTile(2, 0);
    constexpr int root_dst = 17, dead_dst = 18, good_dst = 19, target_dst = 20;
    constexpr int dead_src = 11, good_src = 12, finish_src = 13;
    root->cb_type->dst_src[root_dst].jump |= bit(dead_src) | bit(good_src);
    good->cb_type->dst_src[good_dst].jump |= bit(finish_src);
    addTargetPin(*dead->cb_type, dead_dst);
    addMapping(device, 6, root->coord, dead_src, dead->coord, {dead_dst}, dead->cb_type->type_id);
    addMapping(device, 6, root->coord, good_src, good->coord, {good_dst}, good->cb_type->type_id);
    addMapping(device, 6, good->coord, finish_src, target->coord, {target_dst}, target->cb_type->type_id);
    addTargetPin(*target->cb_type, target_dst);
    for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
    root->cb.dst.jump |= bit(root_dst);

    std::ostringstream decisions;
    auto result = pnr::routeForwardToPlacement({{root, root_dst, true, {}}},
        [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>& terminal) {
            if (&tile != target || dst != target_dst) return false;
            fpga::Wire edge;
            edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
            edge.from = edge.to = tile.coord;
            edge.from_node_type = fpga::CB_NODE_DST;
            edge.from_node = dst;
            edge.to_node_type = fpga::CB_NODE_LOCAL;
            edge.to_node = 0;
            terminal.push_back(edge);
            return true;
        }, {}, &decisions, {}, target->coord);
    const auto first = decisions.str().find("FORWARD_TARGETS state=0 src=11");
    const auto second = decisions.str().find("FORWARD_TARGETS state=0 src=12");
    require(first != std::string::npos && second != std::string::npos && first < second,
            "forward trace lost angle-first source ordering");
    require(result.success && result.expanded == 4 && result.failed_states == 1 &&
                result.edge_checks == 3 && result.fragments.size() == 3,
            "forward trace did not resume the parent after one dead child");
    require(result.fragments[0].jump == good_src && result.fragments[0].to == good->coord &&
                result.fragments[1].jump == finish_src && result.fragments[1].to == target->coord,
            "forward trace retained a dead suffix or missed the live branch");
    for (const auto& tile : device.tile_grid) {
        const NodeMask expected_dst = &tile == root ? bit(root_dst) : NodeMask{};
        require(tile.cb.dst.jump == expected_dst && tile.cb.src.jump == NodeMask{} &&
                    tile.cb.joint.jump == NodeMask{} && tile.cb.local.local == NodeMask{} &&
                    tile.pin_state.leased_nodes == NodeMask{},
                "forward child retry changed live or temporary leases");
    }
    std::cout << "destination forward child retry passed\n";
}

static void destinationForwardLocalTerminal()
{
    resetDevice(3, 1);
    auto& device = fpga::Device::current();
    auto* source = device.getTile(0, 0);
    auto* target = device.getTile(2, 0);
    constexpr int output = 5, src = 11, dst = 21, relay = 31, input = 0;
    source->cb_type->local_src[output].jump |= bit(src);
    target->cb_type->dst_local[dst].local |= bit(relay);
    target->cb_type->local_local[relay].local |= bit(input);
    addMapping(device, 3, source->coord, src, target->coord, {dst}, target->cb_type->type_id);
    for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
    auto trace = [&](bool accepts_terminal) {
        return pnr::routeForwardToPlacement({{source, output, false, {}}},
            [](fpga::Tile&, int, bool, std::vector<fpga::Wire>&) { return false; },
            {}, nullptr, {}, target->coord,
            [&](fpga::Tile& tile, int local, std::vector<fpga::Wire>& terminal) {
                if (!accepts_terminal || &tile != target || local != input) return false;
                fpga::Wire pin;
                pin.type = fpga::Wire::WIRE_TILE_PIN;
                pin.from = pin.to = tile.coord;
                pin.local = input;
                pin.pin_dir = fpga::TILE_PIN_INPUT;
                terminal.push_back(pin);
                return true;
            });
    };
    auto result = trace(true);
    require(result.success && result.expanded == 4 && result.fragments.size() == 4,
            "forward trace pruned a typed LOCAL terminal leaf");
    require(result.fragments[0].type == fpga::Wire::WIRE_CROSSBAR &&
                result.fragments[0].to == target->coord &&
                result.fragments[1].type == fpga::Wire::WIRE_ROUTE_EDGE &&
                result.fragments[1].from_node_type == fpga::CB_NODE_DST &&
                result.fragments[1].from_node == dst &&
                result.fragments[1].to_node_type == fpga::CB_NODE_LOCAL &&
                result.fragments[1].to_node == relay &&
                result.fragments[2].type == fpga::Wire::WIRE_ROUTE_EDGE &&
                result.fragments[2].from_node_type == fpga::CB_NODE_LOCAL &&
                result.fragments[2].from_node == relay &&
                result.fragments[2].to_node_type == fpga::CB_NODE_LOCAL &&
                result.fragments[2].to_node == input &&
                result.fragments[3].type == fpga::Wire::WIRE_TILE_PIN,
            "forward typed LOCAL terminal changed numeric arc roles");
    target->input_local_reservations[input] = nullptr;
    auto own_terminal = trace(true);
    require(own_terminal.success && own_terminal.fragments.size() == 4,
            "forward trace rejected a reservation accepted by the exact terminal probe");
    auto foreign_terminal = trace(false);
    require(!foreign_terminal.success && foreign_terminal.fragments.empty(),
            "forward trace used a declined terminal reservation as transit");
    target->input_local_reservations.erase(input);
    target->input_local_reservations[relay] = nullptr;
    auto unrelated = trace(true);
    require(!unrelated.success && unrelated.fragments.empty(),
            "forward trace crossed an unrelated reserved LOCAL");
    target->input_local_reservations.erase(relay);
    auto direct_accepts = [&] {
        int callbacks = 0;
        auto direct = pnr::routeForwardToPlacement({{target, input, false, {}}},
            [](fpga::Tile&, int, bool, std::vector<fpga::Wire>&) { return false; },
            {}, nullptr, {}, target->coord,
            [&](fpga::Tile&, int, std::vector<fpga::Wire>&) {
                ++callbacks;
                return true;
            });
        return direct.success || callbacks != 0;
    };
    target->cb.local.local |= bit(input);
    require(!trace(true).success && !direct_accepts(),
            "forward terminal probe accepted a leased LOCAL");
    target->cb.local.local.clearBit(input);
    target->pin_state.leased_nodes |= bit(input);
    require(!trace(true).success && !direct_accepts(),
            "forward terminal probe accepted a leased pin");
    target->pin_state.leased_nodes.clearBit(input);
    for (const auto& tile : device.tile_grid)
        require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                    tile.cb.local.local == NodeMask{} && tile.cb.joint.jump == NodeMask{} &&
                    tile.pin_state.leased_nodes == NodeMask{},
                "forward typed LOCAL terminal leaked a speculative lease");
    std::cout << "destination forward LOCAL terminal passed\n";
}

static void destinationForwardJointLocalTerminal()
{
    for (bool chain : {false, true}) {
        for (int blocked = -1; blocked < (chain ? 2 : 1); ++blocked) {
            resetDevice(3, 1);
            auto& device = fpga::Device::current();
            auto& source = *device.getTile(0, 0);
            auto& target = *device.getTile(2, 0);
            constexpr int output = 5, src = 11, dst = 21;
            constexpr int relay = 78, first_joint = 29, second_joint = 30, input = 7;
            source.cb_type->local_src[output].jump = bit(src);
            target.cb_type->dst_local[dst].local = bit(relay);
            target.cb_type->local_joint[relay].joint = bit(first_joint);
            if (chain) {
                target.cb_type->joint_joint[first_joint].joint = bit(second_joint);
                target.cb_type->joint_local[second_joint].local = bit(input);
            } else {
                target.cb_type->joint_local[first_joint].local = bit(input);
            }
            addMapping(device, 3, source.coord, src, target.coord, {dst},
                       target.cb_type->type_id);
            for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
            if (blocked == 0) target.cb.joint.jump.setBit(first_joint);
            if (blocked == 1) target.cb.joint.jump.setBit(second_joint);
            const NodeMask before_joint = target.cb.joint.jump;
            int accepted = 0;
            auto result = pnr::routeForwardToPlacement(
                {{&source, output, false, {}}},
                [](fpga::Tile&, int, bool, std::vector<fpga::Wire>&) { return false; },
                {}, nullptr, {}, target.coord,
                [&](fpga::Tile& tile, int local, std::vector<fpga::Wire>& terminal) {
                    if (&tile != &target || local != input) return false;
                    ++accepted;
                    fpga::Wire pin;
                    pin.type = fpga::Wire::WIRE_TILE_PIN;
                    pin.from = pin.to = tile.coord;
                    pin.local = input;
                    pin.pin_dir = fpga::TILE_PIN_INPUT;
                    terminal.push_back(pin);
                    return true;
                });
            require(result.success == (blocked < 0),
                    "forward trace crossed a busy JOINT or missed a typed terminal");
            if (blocked < 0) {
                auto arc = [&](size_t index, fpga::CBNodeNameType from_type,
                               int from, fpga::CBNodeNameType to_type, int to) {
                    const auto& wire = result.fragments.at(index);
                    return wire.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                           wire.from == target.coord && wire.to == target.coord &&
                           wire.from_node_type == from_type && wire.from_node == from &&
                           wire.to_node_type == to_type && wire.to_node == to;
                };
                require(accepted == 1 && result.fragments.size() == (chain ? 6u : 5u) &&
                            result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR &&
                            result.fragments.front().to == target.coord &&
                            arc(1, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, relay) &&
                            arc(2, fpga::CB_NODE_LOCAL, relay,
                                fpga::CB_NODE_JOINT, first_joint) &&
                            (chain
                                ? arc(3, fpga::CB_NODE_JOINT, first_joint,
                                      fpga::CB_NODE_JOINT, second_joint) &&
                                  arc(4, fpga::CB_NODE_JOINT, second_joint,
                                      fpga::CB_NODE_LOCAL, input)
                                : arc(3, fpga::CB_NODE_JOINT, first_joint,
                                      fpga::CB_NODE_LOCAL, input)) &&
                            result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                        "forward joint terminal lost numeric node roles");
            } else {
                require(accepted == 0 && result.fragments.empty(),
                        "busy JOINT reached the terminal probe");
            }
            for (const auto& tile : device.tile_grid)
                require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                            tile.cb.local.local == NodeMask{} &&
                            tile.cb.joint.jump == (&tile == &target ? before_joint : NodeMask{}) &&
                            tile.pin_state.leased_nodes == NodeMask{},
                        "forward joint terminal changed live or temporary leases");
        }
    }
}

static void destinationForwardResolvedLocalSource()
{
    for (int control = 0; control < 3; ++control) {
        resetDevice(4, 1);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& relay = *device.getTile(1, 0);
        auto& bridge = *device.getTile(2, 0);
        auto& target = *device.getTile(3, 0);
        constexpr int output = 5, first_src = 11, first_dst = 21;
        constexpr int relay_local = 31, bridge_src = 12, final_dst = 22, input = 0;
        source.cb_type->local_src[output].jump = bit(first_src);
        relay.cb_type->dst_local[first_dst].local = bit(relay_local);
        fpga::CBType::ResolvedLocal resolved{};
        resolved.delta = bridge.coord - relay.coord;
        resolved.target_cb_type_id = bridge.cb_type->type_id;
        resolved.target_node_type = fpga::CB_NODE_SRC;
        resolved.target_nodes = bit(bridge_src);
        relay.cb_type->local_by_local[relay_local].push_back(resolved);
        target.cb_type->dst_local[final_dst].local = bit(input);
        addMapping(device, 4, source.coord, first_src, relay.coord, {first_dst},
                   relay.cb_type->type_id);
        addMapping(device, 4, bridge.coord, bridge_src, target.coord, {final_dst},
                   target.cb_type->type_id);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (control == 1) bridge.cb.src.jump.setBit(bridge_src);
        const NodeMask before_src = bridge.cb.src.jump;
        int polls = 0, accepted = 0;
        auto result = pnr::routeForwardToPlacement(
            {{&source, output, false, {}}},
            [](fpga::Tile&, int, bool, std::vector<fpga::Wire>&) { return false; },
            [&] { return control == 2 && ++polls >= 4; }, nullptr, {}, target.coord,
            [&](fpga::Tile& tile, int local, std::vector<fpga::Wire>& terminal) {
                if (&tile != &target || local != input) return false;
                ++accepted;
                fpga::Wire pin;
                pin.type = fpga::Wire::WIRE_TILE_PIN;
                pin.from = pin.to = tile.coord;
                pin.local = input;
                pin.pin_dir = fpga::TILE_PIN_INPUT;
                terminal.push_back(pin);
                return true;
            });
        require(result.success == (control == 0) &&
                    result.cancelled == (control == 2),
                "forward trace lost a resolved LOCAL-to-SRC link or its controls");
        if (control == 0) {
            auto arc = [&](size_t index, fpga::Coord from,
                           fpga::CBNodeNameType from_type, int from_node,
                           fpga::Coord to, fpga::CBNodeNameType to_type, int to_node) {
                const auto& wire = result.fragments.at(index);
                return wire.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                       wire.from == from && wire.to == to &&
                       wire.from_node_type == from_type && wire.from_node == from_node &&
                       wire.to_node_type == to_type && wire.to_node == to_node;
            };
            require(accepted == 1 && result.fragments.size() == 6 &&
                        result.expanded == 6 &&
                        result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR &&
                        result.fragments.front().to == relay.coord &&
                        arc(1, relay.coord, fpga::CB_NODE_DST, first_dst,
                            relay.coord, fpga::CB_NODE_LOCAL, relay_local) &&
                        arc(2, relay.coord, fpga::CB_NODE_LOCAL, relay_local,
                            bridge.coord, fpga::CB_NODE_SRC, bridge_src) &&
                        arc(3, bridge.coord, fpga::CB_NODE_SRC, bridge_src,
                            target.coord, fpga::CB_NODE_DST, final_dst) &&
                        arc(4, target.coord, fpga::CB_NODE_DST, final_dst,
                            target.coord, fpga::CB_NODE_LOCAL, input) &&
                        result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                    "resolved LOCAL-to-SRC route changed numeric node roles");
        } else require(accepted == 0 && result.fragments.empty(),
                       "blocked or cancelled LOCAL-to-SRC route reached its input");
        for (const auto& tile : device.tile_grid)
            require(tile.cb.src.jump == (&tile == &bridge ? before_src : NodeMask{}) &&
                        tile.cb.dst.jump == NodeMask{} &&
                        tile.cb.joint.jump == NodeMask{} &&
                        tile.cb.local.local == NodeMask{} &&
                        tile.pin_state.leased_nodes == NodeMask{},
                    "resolved LOCAL-to-SRC trace changed live or temporary leases");
    }
}

// Side branches have packable endpoints but must not become a breadth-first
// frontier. The relocation trace follows the first free angle-priority chain.
static void sourceMovingTrace()
{
    for (int path_kind : {0, 1, 2, 3, 4}) for (bool cancel : {false, true}) {
        resetDevice(12, 3);
        auto& device = fpga::Device::current();
        constexpr int dst = 19, src = 71, local = 0, joint = 9, first_joint = 10, relay = 48;
        auto* target = device.getTile(10, 1);
        addTargetPin(*target->cb_type, dst);
        for (int x = 1; x < 10; ++x) {
            auto* tile = device.getTile(x, 1);
            addMapping(device, 12, tile->coord, src, {x + 1, 1}, {dst},
                       device.getTile(x + 1, 1)->cb_type->type_id);
            if (path_kind == 1) {
                tile->cb_type->dst_joint[dst].joint = bit(joint);
                tile->cb_type->joint_src[joint].jump = bit(src);
            } else if (path_kind >= 2) {
                // A routing LOCAL is not necessarily a terminal input. This
                // free relay must survive the direction-driven conversion.
                tile->cb_type->dst_local[dst].local = bit(relay);
                if (path_kind == 2) tile->cb_type->local_src[relay].jump = bit(src);
                else {
                    tile->cb_type->local_joint[relay].joint = bit(path_kind == 3 ? joint : first_joint);
                    if (path_kind == 4) tile->cb_type->joint_joint[first_joint].joint = bit(joint);
                    tile->cb_type->joint_src[joint].jump = bit(src);
                }
                tile->cb_type->rememberNodeName(fpga::CB_NODE_LOCAL, relay, "relay_48");
            } else tile->cb_type->dst_src[dst].jump = bit(src);
            tile->cb_type->local_src[local].jump = bit(src);
            tile->cb_type->rebuildOutgoingSrcs();
            auto* side = device.getTile(x, 0);
            side->cb_type->local_src[local].jump = bit(src);
            addMapping(device, 12, side->coord, src, {x + 1, 1}, {dst},
                       device.getTile(x + 1, 1)->cb_type->type_id);
        }
        auto index = pnr::buildBackwardResolveIndex(device, target->coord, 12);
        std::vector<fpga::Coord> probes;
        auto result = pnr::traceBackwardToTakeoff(*target, bit(local), {1, 1}, index,
            [&](fpga::Tile& tile, int node, pnr::BackwardTakeoffChoice& choice) {
                require(node == src && tile.coord.y == 1,
                        "relocation expanded an off-angle side branch: kind=" + std::to_string(path_kind));
                probes.push_back(tile.coord);
                choice.local = local;
                return tile.coord.x == 1;
            }, [&] { return cancel && probes.size() == 4; });
        // No alternate frontier states: exactly one expansion per accepted hop.
        require(result.success == !cancel && result.expanded == (cancel ? 4 : 9) &&
                    result.incoming_edges == result.expanded && probes.size() == result.expanded,
                "relocation lost or expanded beyond its angle-guided path: kind=" +
                    std::to_string(path_kind));
        for (size_t i = 0; i < probes.size(); ++i)
            require(probes[i] == fpga::Coord{9 - int(i), 1}, "reverse trace restarted or changed direction");
        if (!cancel) {
            require(result.fragments.size() == (path_kind >= 2 ? 27 + 8 * (path_kind - 2) : 11) &&
                        result.source_tile->coord == fpga::Coord{1, 1},
                    "trace did not preserve the complete accepted suffix");
            int hop = 1;
            for (const auto &fragment : result.fragments) {
                if (fragment.from == fragment.to) continue;
                require(fragment.from == fpga::Coord{hop, 1} &&
                            fragment.to == fpga::Coord{hop + 1, 1},
                        "backward trace emitted reversed or discontinuous fragments");
                ++hop;
            }
            require(hop == 10, "trace omitted an inter-tile hop");
            if (path_kind >= 2) {
                // Each relay has distinct typed identities; its numeric LOCAL
                // must never be flattened into an inter-tile DST or a cell pin.
                for (int x = 2; x <= 9; ++x) {
                    int typed_arcs = 0;
                    for (const auto &fragment : result.fragments)
                        if (fragment.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                            fragment.from == fpga::Coord{x, 1} && fragment.from == fragment.to) {
                            ++typed_arcs;
                            const auto &cb = *device.getTile(x, 1)->cb_type;
                            const int from = fragment.from_node, to = fragment.to_node;
                            bool connected = false;
                            if (fragment.from_node_type == fpga::CB_NODE_DST)
                                connected = fragment.to_node_type == fpga::CB_NODE_LOCAL &&
                                    cb.dst_local.get(from).local.testBit(to);
                            else if (fragment.from_node_type == fpga::CB_NODE_LOCAL)
                                connected = fragment.to_node_type == fpga::CB_NODE_SRC
                                    ? cb.local_src.get(from).jump.testBit(to)
                                    : fragment.to_node_type == fpga::CB_NODE_JOINT &&
                                        cb.local_joint.get(from).joint.testBit(to);
                            else if (fragment.from_node_type == fpga::CB_NODE_JOINT)
                                connected = fragment.to_node_type == fpga::CB_NODE_SRC
                                    ? cb.joint_src.get(from).jump.testBit(to)
                                    : fragment.to_node_type == fpga::CB_NODE_JOINT &&
                                        cb.joint_joint.get(from).joint.testBit(to);
                            require(connected, "backward relay emitted a nonexistent or reversed internal arc");
                        }
                    require(typed_arcs == path_kind, "relay/joint chain lost typed internal arcs");
                }
            }
        }
        // Both completion and cancellation leave the live fabric unchanged.
        for (const auto& tile : device.tile_grid)
            require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                        tile.cb.joint.jump == NodeMask{} && tile.cb.local.local == NodeMask{},
                    "relocation trace leaked speculative leases");
    }
    // A relay is transit only while its local, pin and joints are available.
    // These controls must fail without removing the blocking state.
    for (int blocked = 0; blocked < 5; ++blocked) {
        resetDevice(3, 1);
        auto &device = fpga::Device::current();
        auto &source = *device.getTile(0, 0);
        auto &relay = *device.getTile(1, 0);
        auto &target = *device.getTile(2, 0);
        constexpr int dst = 19, src = 71, local = 48, joint = 9;
        source.cb_type->local_src[0].jump = bit(src);
        relay.cb_type->dst_local[dst].local = bit(local);
        relay.cb_type->local_joint[local].joint = bit(joint);
        relay.cb_type->joint_src[joint].jump = bit(src);
        addTargetPin(*target.cb_type, dst);
        addMapping(device, 3, source.coord, src, relay.coord, {dst}, relay.cb_type->type_id);
        addMapping(device, 3, relay.coord, src, target.coord, {dst}, target.cb_type->type_id);
        for (auto &cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (blocked == 1) relay.cb.local.local.setBit(local);
        if (blocked == 2) relay.pin_state.leased_nodes.setBit(local);
        if (blocked == 3) relay.input_local_reservations[local] = nullptr;
        if (blocked == 4) relay.cb.joint.jump.setBit(joint);
        auto index = pnr::buildBackwardResolveIndex(device, target.coord, 3);
        const auto result = pnr::traceBackwardToTakeoff(target, bit(0), source.coord, index,
            [&](fpga::Tile &tile, int node, pnr::BackwardTakeoffChoice &choice) {
                require(node == src, "relay trace selected an unknown departure");
                choice.local = 0;
                return &tile == &source;
            });
        require(result.success == (blocked == 0), "relay trace bypassed a real occupation");
        require(result.free_terminal_ingress,
                "free ordinary DST-to-LOCAL ingress was reported blocked by later relay work");
        require(result.expanded == (blocked == 0 ? 2 : 1),
                "blocked relay started another search frontier");
        for (const auto &tile : device.tile_grid) {
            const bool middle = &tile == &relay;
            require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                    tile.cb.local.local == (middle && blocked == 1 ? bit(local) : NodeMask{}) &&
                    tile.cb.joint.jump == (middle && blocked == 4 ? bit(joint) : NodeMask{}) &&
                    tile.pin_state.leased_nodes == (middle && blocked == 2 ? bit(local) : NodeMask{}),
                    "relay trial changed real leases or leaked temporary state");
        }
        require(relay.input_local_reservations.contains(local) == (blocked == 3),
                "relay trace changed packing reservations");
    }
}

// A mapped input can be reached through typed fabric without a direct
// DST-to-LOCAL terminal entry. Keep these paths numeric and lease-free.
static void typedSourceIngress()
{
    using LiveState = std::tuple<NodeMask, NodeMask, NodeMask, NodeMask, NodeMask>;
    auto liveStates = [] {
        std::vector<LiveState> states;
        for (const auto& tile : fpga::Device::current().tile_grid)
            states.emplace_back(tile.cb.src.jump, tile.cb.dst.jump,
                tile.cb.joint.jump, tile.cb.local.local, tile.pin_state.leased_nodes);
        return states;
    };
    auto addJump = [](fpga::Tile& source, int src, fpga::Tile& target, int dst) {
        fpga::CBType::ResolvedJump mapping{};
        mapping.delta = target.coord - source.coord;
        mapping.target_tile_coord = true;
        mapping.target_cb_type_id = target.cb_type->type_id;
        mapping.dsts.jump = bit(dst);
        source.cb_type->dst_by_src[src].push_back(mapping);
    };
    auto typedArc = [](const fpga::Wire& wire, fpga::Coord from,
                       fpga::CBNodeNameType from_type, int from_node,
                       fpga::Coord to, fpga::CBNodeNameType to_type, int to_node) {
        return wire.type == fpga::Wire::WIRE_ROUTE_EDGE && wire.from == from &&
            wire.to == to && wire.from_node_type == from_type &&
            wire.from_node == from_node && wire.to_node_type == to_type &&
            wire.to_node == to_node;
    };

    for (int compact = 0; compact < 3; ++compact)
    for (int control = 0; control < 4; ++control) {
        resetDevice(3, 1);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& target = *device.getTile(2, 0);
        source.cb_type->local_src[5].jump = bit(71);
        addJump(source, 71, target, 19);
        target.cb_type->dst_joint[19].joint = bit(10);
        target.cb_type->joint_joint[10].joint = bit(11);
        target.cb_type->joint_joint[11].joint = bit(12);
        target.cb_type->joint_local[12].local = bit(0);
        if (compact) {
            // A compact DST->JOINT12->LOCAL0 path shares the long path's
            // terminal JOINT. It is either disconnected or physically busy.
            target.cb_type->dst_joint[20].joint = bit(12);
            if (compact == 2) {
                addJump(*device.getTile(1, 0), 72, target, 20);
                target.cb.dst.jump.setBit(20);
            }
        }
        target.cb_type->local_input_nodes.setBit(0);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (compact) {
            bool found = false;
            for (const auto& entry : target.cb_type->terminalEntries(0))
                found |= entry.dst == 20 && entry.joint == 12 && entry.joint2 == -1;
            require(found, "shared-tail compact path was not represented as a terminal entry");
        }
        if (control == 1) target.cb.joint.jump.setBit(11);
        if (control == 2) target.pin_state.leased_nodes.setBit(0);
        const auto before = liveStates();
        auto index = pnr::buildBackwardResolveIndex(device, target.coord, 3);
        int probes = 0;
        const auto result = pnr::traceBackwardToTakeoff(target, bit(0), source.coord,
            index, [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                ++probes;
                require(&tile == &source && src == 71,
                    "three-joint ingress probed an unrelated source");
                choice.local = 5;
                return true;
            }, [&] { return control == 3; });
        require(result.success == (control == 0),
            "three-joint terminal ingress was missed or bypassed a blocker");
        if (control == 0) {
            require(result.free_terminal_ingress && probes == 1 &&
                    result.source_tile == &source &&
                    result.fragments.size() == 6 &&
                    result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR &&
                    result.fragments.front().from == source.coord &&
                    result.fragments.front().to == target.coord,
                "three-joint ingress lost its ordinary crossbar jump");
            const fpga::CBNodeNameType kinds[] = {fpga::CB_NODE_DST,
                fpga::CB_NODE_JOINT, fpga::CB_NODE_JOINT,
                fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL};
            const int nodes[] = {19, 10, 11, 12, 0};
            for (int n = 0; n < 4; ++n)
                require(typedArc(result.fragments[static_cast<size_t>(n + 1)],
                            target.coord, kinds[n], nodes[n], target.coord,
                            kinds[n + 1], nodes[n + 1]),
                    "three-joint ingress lost a typed terminal arc");
            require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
                        result.fragments.back().local == 0,
                "three-joint ingress lost the mapped input pin");
        } else require(!result.free_terminal_ingress,
            "blocked three-joint ingress claimed a free physical arrival");
        if (control == 1) {
            require(result.failure_tile == &target && result.failure_dst == -1 &&
                    result.diagnostic_fragments.size() == 3 &&
                    typedArc(result.diagnostic_fragments.front(), target.coord,
                        fpga::CB_NODE_JOINT, 11, target.coord,
                        fpga::CB_NODE_JOINT, 12) &&
                    result.diagnostic_fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                "blocked shared-tail JOINT lost its typed diagnostic suffix");
        }
        require(liveStates() == before,
            "three-joint trace changed live or temporary leases");
    }

    for (int control = 0; control < 5; ++control) {
        resetDevice(27, 1);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& entry = *device.getTile(1, 0);
        auto& target = *device.getTile(25, 0);
        source.cb_type->local_src[5].jump = bit(71);
        addJump(source, 71, entry, 19);
        entry.cb_type->dst_local[19].local = bit(48);
        for (int x = 1; x < 25; ++x) {
            auto& from = *device.getTile(x, 0);
            auto& to = *device.getTile(x + 1, 0);
            fpga::CBType::ResolvedLocal link{};
            link.delta = to.coord - from.coord;
            link.target_cb_type_id = to.cb_type->type_id;
            link.target_node_type = fpga::CB_NODE_LOCAL;
            link.target_nodes = bit(x == 24 ? 78 : 48);
            from.cb_type->local_by_local[48].push_back(link);
        }
        // The same input also has a much longer leftward predecessor and a
        // rightward dead end. The accepted trace must take x=24 first.
        for (int x : {2, 26}) {
            auto& from = *device.getTile(x, 0);
            fpga::CBType::ResolvedLocal link{};
            link.delta = target.coord - from.coord;
            link.target_cb_type_id = target.cb_type->type_id;
            link.target_node_type = fpga::CB_NODE_LOCAL;
            link.target_nodes = bit(78);
            from.cb_type->local_by_local[48].push_back(link);
        }
        target.cb_type->local_joint[78].joint = bit(56);
        target.cb_type->joint_local[56].local = bit(0);
        target.cb_type->local_input_nodes.setBit(0);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (control == 1) target.cb.joint.jump.setBit(56);
        if (control == 2) device.getTile(12, 0)->cb.local.local.setBit(48);
        if (control == 3) target.pin_state.leased_nodes.setBit(0);
        const auto before = liveStates();
        auto index = pnr::buildBackwardResolveIndex(device, target.coord, 27);
        int incoming = 0;
        pnr::visitBackwardPhysicalSources(index, target.coord,
            fpga::CB_NODE_LOCAL, 78,
            [&](fpga::Coord at, fpga::CBNodeNameType type, int node) {
                incoming += at == fpga::Coord{24, 0} &&
                    type == fpga::CB_NODE_LOCAL && node == 48;
            });
        require(incoming == 1, "resolved LOCAL predecessor is absent from the numeric index");
        int polls = 0;
        std::ostringstream decisions;
        const auto result = pnr::traceBackwardToTakeoff(target, bit(0), source.coord,
            index, [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                require(&tile == &source && src == 71,
                    "resolved LOCAL ingress probed an unrelated source");
                choice.local = 5;
                return true;
            }, [&] { return control == 4 && ++polls >= 8; }, nullptr, &decisions);
        require(result.success == (control == 0),
            "resolved LOCAL terminal ingress was missed or bypassed a blocker");
        if (control == 0) {
            require(result.source_tile == &source &&
                    result.free_terminal_ingress && result.expanded == 1 &&
                    result.probe_calls == 1 &&
                    result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR &&
                    result.fragments.front().from == source.coord &&
                    result.fragments.front().to == entry.coord,
                "resolved LOCAL ingress lost its ordinary crossbar jump");
            const std::string trace = decisions.str();
            size_t accepted_nodes = 0;
            for (size_t at = 0; (at = trace.find("BACKWARD_INGRESS tile=", at)) !=
                    std::string::npos; at += sizeof("BACKWARD_INGRESS tile=") - 1)
                ++accepted_nodes;
            require(accepted_nodes == 27 &&
                    trace.find("BACKWARD_INGRESS tile=(24,0) type=") != std::string::npos &&
                    trace.find("BACKWARD_INGRESS tile=(26,0) type=") == std::string::npos &&
                    trace.find("BACKWARD_INGRESS tile=(2,0) type=") != std::string::npos &&
                    trace.find("BACKWARD_INGRESS tile=(2,0) type=") >
                        trace.find("BACKWARD_INGRESS tile=(24,0) type="),
                "resolved LOCAL trace materialized alternatives or changed angle priority");
            int links = 0;
            for (const auto& fragment : result.fragments)
                if (fragment.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                    fragment.from != fragment.to) {
                    require(fragment.from_node_type == fpga::CB_NODE_LOCAL &&
                                fragment.to_node_type == fpga::CB_NODE_LOCAL &&
                                fragment.to.x == fragment.from.x + 1,
                        "resolved LOCAL ingress changed a physical link's node type");
                    ++links;
                }
            require(links == 24 &&
                    typedArc(result.fragments[result.fragments.size() - 3],
                        target.coord, fpga::CB_NODE_LOCAL, 78, target.coord,
                        fpga::CB_NODE_JOINT, 56) &&
                    typedArc(result.fragments[result.fragments.size() - 2],
                        target.coord, fpga::CB_NODE_JOINT, 56, target.coord,
                        fpga::CB_NODE_LOCAL, 0) &&
                    result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                "resolved LOCAL chain lost its terminal joint or input pin");
        } else require(!result.free_terminal_ingress,
            "blocked resolved LOCAL ingress claimed a free physical arrival");
        if (control == 2) {
            const auto& blocked = *device.getTile(12, 0);
            require(result.failure_tile == &blocked && result.failure_dst == -1 &&
                    !result.diagnostic_fragments.empty() &&
                    typedArc(result.diagnostic_fragments.front(), blocked.coord,
                        fpga::CB_NODE_LOCAL, 48, {13, 0},
                        fpga::CB_NODE_LOCAL, 48) &&
                    result.diagnostic_fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                "blocked resolved LOCAL lost its typed diagnostic suffix");
        }
        if (control == 4)
            require(polls >= 8, "resolved LOCAL cancellation did not reach the live chain");
        require(liveStates() == before,
            "resolved LOCAL trace changed live or temporary leases");
    }
}

static void forwardResolvedLocalLinks()
{
    for (int control = 0; control < 5; ++control) {
        resetDevice(5, 2);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& relay = *device.getTile(1, 0);
        auto& bridge = *device.getTile(2, 0);
        auto& near = *device.getTile(3, 0);
        auto& target = *device.getTile(4, 0);
        auto& side = *device.getTile(2, 1);
        auto& wrong_type = *device.getTile(3, 1);
        auto link = [&](fpga::Tile& from, int local, fpga::Tile& to,
                        fpga::CBNodeNameType type, int node) {
            fpga::CBType::ResolvedLocal entry{};
            entry.delta = to.coord - from.coord;
            entry.target_cb_type_id = to.cb_type->type_id;
            entry.target_node_type = type;
            entry.target_nodes = bit(node);
            from.cb_type->local_by_local[local].push_back(entry);
        };
        source.cb_type->local_src[5].jump = bit(11);
        relay.cb_type->dst_local[21].local = bit(31);
        relay.cb_type->local_local[31].local = bit(32);
        relay.cb_type->local_src[32].jump = bit(12);
        bridge.cb_type->dst_local[22].local = bit(40);
        link(bridge, 40, side, fpga::CB_NODE_LOCAL, 41);
        link(bridge, 40, wrong_type, fpga::CB_NODE_JOINT, 41);
        link(bridge, 40, near, fpga::CB_NODE_LOCAL, 41);
        link(near, 41, target, fpga::CB_NODE_LOCAL, 0);
        addMapping(device, 5, source.coord, 11, relay.coord, {21}, relay.cb_type->type_id);
        addMapping(device, 5, relay.coord, 12, bridge.coord, {22}, bridge.cb_type->type_id);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (control == 1) near.cb.local.local.setBit(41);
        if (control == 2) near.pin_state.leased_nodes.setBit(41);
        if (control == 3) near.input_local_reservations[41] = nullptr;
        std::vector<std::array<NodeMask, 5>> before;
        for (const auto& tile : device.tile_grid)
            before.push_back({tile.cb.src.jump, tile.cb.dst.jump,
                              tile.cb.joint.jump, tile.cb.local.local,
                              tile.pin_state.leased_nodes});
        int polls = 0, accepted = 0;
        auto result = pnr::routeForwardToPlacement({{&source, 5, false, {}}},
            [](fpga::Tile&, int, bool, std::vector<fpga::Wire>&) { return false; },
            [&] { return control == 4 && ++polls >= 9; }, nullptr, {}, target.coord,
            [&](fpga::Tile& tile, int local, std::vector<fpga::Wire>& terminal) {
                if (&tile != &target || local != 0) return false;
                ++accepted;
                fpga::Wire pin;
                pin.type = fpga::Wire::WIRE_TILE_PIN;
                pin.from = pin.to = tile.coord;
                pin.local = local;
                pin.pin_dir = fpga::TILE_PIN_INPUT;
                terminal.push_back(pin);
                return true;
            });
        require(result.success == (control == 0) &&
                    result.cancelled == (control == 4),
                "forward trace mishandled a resolved LOCAL link or control");
        if (control == 0) {
            require(accepted == 1 && result.expanded == 8 &&
                        result.fragments.size() == 9 &&
                        result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR &&
                        result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                    "forward LOCAL trace expanded an off-angle branch or lost its endpoint");
            int resolved = 0, jumps = 0;
            for (const auto& edge : result.fragments)
                if (edge.type == fpga::Wire::WIRE_ROUTE_EDGE && edge.from != edge.to) {
                    if (edge.from_node_type == fpga::CB_NODE_LOCAL &&
                        edge.to_node_type == fpga::CB_NODE_LOCAL) ++resolved;
                    else {
                        require(edge.from == relay.coord && edge.to == bridge.coord &&
                                    edge.from_node_type == fpga::CB_NODE_SRC &&
                                    edge.to_node_type == fpga::CB_NODE_DST,
                                "forward trace reinterpreted a numeric jump");
                        ++jumps;
                    }
                }
            require(resolved == 2 && jumps == 1,
                    "forward trace missed a resolved LOCAL link or numeric jump");
        }
        for (size_t i = 0; i < before.size(); ++i) {
            const auto& tile = device.tile_grid[i];
            require((std::array<NodeMask, 5>{tile.cb.src.jump, tile.cb.dst.jump,
                        tile.cb.joint.jump, tile.cb.local.local,
                        tile.pin_state.leased_nodes}) == before[i],
                    "forward LOCAL trace changed live or temporary leases");
        }
        if (control == 3)
            require(near.input_local_reservations.contains(41),
                    "forward LOCAL trace changed an input reservation");
    }
    std::cout << "forward resolved LOCAL links: free, blocked, cancel controls passed\n";
}

static void backwardMidtraceLocalChain()
{
    for (int control = 0; control < 5; ++control) {
        resetDevice(3, 1);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& relay = *device.getTile(1, 0);
        auto& target = *device.getTile(2, 0);
        source.cb_type->local_src[5].jump = bit(11);
        relay.cb_type->dst_local[21].local = bit(31);
        relay.cb_type->local_local[31].local = bit(32);
        relay.cb_type->local_src[32].jump = bit(12);
        target.cb_type->dst_local[22].local = bit(0);
        addMapping(device, 3, source.coord, 11, relay.coord, {21}, relay.cb_type->type_id);
        addMapping(device, 3, relay.coord, 12, target.coord, {22}, target.cb_type->type_id);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (control == 1) relay.cb.local.local.setBit(32);
        if (control == 2) relay.pin_state.leased_nodes.setBit(32);
        if (control == 3) relay.input_local_reservations[32] = nullptr;
        std::vector<std::array<NodeMask, 5>> before;
        for (const auto& tile : device.tile_grid)
            before.push_back({tile.cb.src.jump, tile.cb.dst.jump,
                              tile.cb.joint.jump, tile.cb.local.local,
                              tile.pin_state.leased_nodes});
        auto index = pnr::buildBackwardResolveIndex(device, target.coord, 3);
        int polls = 0;
        auto result = pnr::traceBackwardToTakeoff(target, bit(0), source.coord, index,
            [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                if (&tile != &source || src != 11) return false;
                choice.local = 5;
                return true;
            }, [&] { return control == 4 && ++polls >= 2; });
        require(result.success == (control == 0),
                "backward trace mishandled a free or blocked intermediate LOCAL chain");
        if (control == 0) {
            require(result.source_tile == &source && result.fragments.size() == 7 &&
                        result.fragments.front().from == source.coord &&
                        result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                    "backward mid-trace lost its source or input endpoint");
            int local_edges = 0;
            for (const auto& edge : result.fragments)
                if (edge.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                    edge.from_node_type == fpga::CB_NODE_LOCAL &&
                    edge.to_node_type == fpga::CB_NODE_LOCAL) ++local_edges;
            require(local_edges == 1,
                    "backward mid-trace omitted the numeric LOCAL-to-LOCAL relay");
        }
        if (control == 4)
            require(polls >= 2, "backward mid-trace did not reach cancellation");
        for (size_t i = 0; i < before.size(); ++i) {
            const auto& tile = device.tile_grid[i];
            require((std::array<NodeMask, 5>{tile.cb.src.jump, tile.cb.dst.jump,
                        tile.cb.joint.jump, tile.cb.local.local,
                        tile.pin_state.leased_nodes}) == before[i],
                    "backward mid-trace changed live or temporary leases");
        }
        if (control == 3)
            require(relay.input_local_reservations.contains(32),
                    "backward mid-trace changed an input reservation");
    }
}

static void backwardChildReturn()
{
    for (int control : {0, 1, 2}) {
        const bool cancel_after_first_child = control == 1;
        resetDevice(4, 2);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& dead = *device.getTile(2, 0);
        auto& good = *device.getTile(2, 1);
        auto& target = *device.getTile(3, 1);
        constexpr int dead_src = 12, good_src = 13, source_src = 14;
        constexpr int dead_dst = 21, good_dst = 22, target_dst = 30;
        source.cb_type->local_src[5].jump = bit(source_src);
        dead.cb_type->dst_src[dead_dst].jump = bit(dead_src);
        good.cb_type->dst_src[good_dst].jump = bit(good_src);
        addTargetPin(*target.cb_type, target_dst);
        addMapping(device, 4, dead.coord, dead_src, target.coord,
                   {target_dst}, target.cb_type->type_id);
        addMapping(device, 4, good.coord, good_src, target.coord,
                   {target_dst}, target.cb_type->type_id);
        addMapping(device, 4, source.coord, source_src, good.coord,
                   {good_dst}, good.cb_type->type_id);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();

        // Unrelated live leases must survive both a failed child and cancellation.
        dead.cb.src.jump.setBit(99);
        good.cb.local.local.setBit(88);
        source.pin_state.leased_nodes.setBit(77);
        std::vector<std::array<NodeMask, 5>> before;
        for (const auto& tile : device.tile_grid)
            before.push_back({tile.cb.src.jump, tile.cb.dst.jump,
                              tile.cb.joint.jump, tile.cb.local.local,
                              tile.pin_state.leased_nodes});

        auto index = pnr::buildBackwardResolveIndex(device, target.coord, 4);
        std::vector<int> probed;
        int polls = 0;
        const auto result = pnr::traceBackwardToTakeoff(target, bit(0),
            source.coord, index,
            [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                probed.push_back(src);
                if (control == 2 || &tile != &source || src != source_src) return false;
                choice.local = 5;
                return true;
            }, [&] { return cancel_after_first_child && ++polls >= 2; });
        if (cancel_after_first_child) {
            require(!result.success && polls >= 2 && result.probe_calls == 1 &&
                        probed == std::vector<int>{dead_src},
                    "cancelled backward child trace visited another branch");
            require(result.failure_tile == &dead && result.failure_dst == dead_dst &&
                        !result.diagnostic_fragments.empty() &&
                        result.diagnostic_fragments.front().from == dead.coord,
                    "cancelled backward child lost its matching diagnostic suffix");
        } else if (control == 2) {
            require(!result.success && result.probe_calls == 3 &&
                        probed == std::vector<int>{dead_src, good_src, source_src},
                    "failed backward child trace replayed an earlier placement probe");
            require(result.failure_tile == &good && result.failure_dst == good_dst &&
                        result.diagnostic_fragments.size() == 3 &&
                        result.diagnostic_fragments.front().from == good.coord &&
                        result.diagnostic_fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                    "failed backward child lost its deepest matching diagnostic suffix");
        } else {
            require(result.success && result.source_tile == &source &&
                        result.source_src == source_src && result.probe_calls == 3 &&
                        probed == std::vector<int>{dead_src, good_src, source_src},
                    "backward trace did not resume the angle-ordered parent after a dead child");
            require(result.fragments.size() == 4 &&
                        result.fragments.front().from == source.coord &&
                        result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
                    "backward child return lost the physical source-to-pin path");
        }
        for (size_t i = 0; i < before.size(); ++i) {
            const auto& tile = device.tile_grid[i];
            require((std::array<NodeMask, 5>{tile.cb.src.jump, tile.cb.dst.jump,
                        tile.cb.joint.jump, tile.cb.local.local,
                        tile.pin_state.leased_nodes}) == before[i],
                    "backward child return changed live or temporary leases");
        }
    }
}

static void backwardExhaustedSourceReuse()
{
    resetDevice(4, 3);
    auto& device = fpga::Device::current();
    auto& upper = *device.getTile(2, 0);
    auto& lower = *device.getTile(2, 2);
    auto& shared = *device.getTile(1, 1);
    auto& target = *device.getTile(3, 1);
    constexpr int target_dst = 30, upper_dst = 21, lower_dst = 22;
    constexpr int upper_src = 11, lower_src = 12, common_src = 13;
    upper.cb_type->dst_src[upper_dst].jump = bit(upper_src);
    lower.cb_type->dst_src[lower_dst].jump = bit(lower_src);
    shared.cb_type->dst_src[23].jump = bit(common_src);
    shared.cb_type->local_src[5].jump = bit(common_src);
    addTargetPin(*target.cb_type, target_dst);
    addMapping(device, 4, upper.coord, upper_src, target.coord,
               {target_dst}, target.cb_type->type_id);
    addMapping(device, 4, lower.coord, lower_src, target.coord,
               {target_dst}, target.cb_type->type_id);
    addMapping(device, 4, shared.coord, common_src, upper.coord,
               {upper_dst}, upper.cb_type->type_id);
    addMapping(device, 4, shared.coord, common_src, lower.coord,
               {lower_dst}, lower.cb_type->type_id);
    for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
    shared.cb.local.local.setBit(88);
    std::vector<std::array<NodeMask, 5>> before;
    for (const auto& tile : device.tile_grid)
        before.push_back({tile.cb.src.jump, tile.cb.dst.jump,
                          tile.cb.joint.jump, tile.cb.local.local,
                          tile.pin_state.leased_nodes});
    auto index = pnr::buildBackwardResolveIndex(device, target.coord, 4);
    auto check_leases = [&] {
        for (size_t i = 0; i < before.size(); ++i) {
            const auto& tile = device.tile_grid[i];
            require((std::array<NodeMask, 5>{tile.cb.src.jump, tile.cb.dst.jump,
                        tile.cb.joint.jump, tile.cb.local.local,
                        tile.pin_state.leased_nodes}) == before[i],
                    "exhausted backward SRC changed live or temporary leases");
        }
    };
    std::vector<int> probes;
    const auto exhausted = pnr::traceBackwardToTakeoff(target, bit(0),
        {0, 0}, index,
        [&](fpga::Tile&, int src, pnr::BackwardTakeoffChoice&) {
            probes.push_back(src);
            return false;
        });
    require(!exhausted.success && exhausted.probe_calls == 3 &&
                exhausted.expanded == 4 && exhausted.failed_states == 3 &&
                exhausted.failure_cache_hits == 1 &&
                probes == std::vector<int>{upper_src, common_src, lower_src},
            "converging backward branches reprobed a fully exhausted SRC");
    check_leases();

    // A new trace must not inherit the previous call's temporary blacklist.
    const auto fresh = pnr::traceBackwardToTakeoff(target, bit(0), {0, 0}, index,
        [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
            if (&tile != &shared || src != common_src) return false;
            choice.local = 5;
            return true;
        });
    require(fresh.success && fresh.source_tile == &shared &&
                fresh.source_src == common_src,
            "new backward call inherited an exhausted SRC blacklist");
    check_leases();
}

static void backwardLocalSuccessorReturn()
{
    for (int control : {0, 1, 2}) {
        resetDevice(4, 1);
        auto& device = fpga::Device::current();
        auto& source = *device.getTile(0, 0);
        auto& relay = *device.getTile(1, 0);
        auto& target = *device.getTile(3, 0);
        source.cb_type->local_src[5].jump = bit(11);
        relay.cb_type->dst_local[21].local = bit(31);
        relay.cb_type->local_local[31].local = bit(32) | bit(33);
        relay.cb_type->local_src[33].jump = bit(12);
        addTargetPin(*target.cb_type, 30);
        addMapping(device, 4, source.coord, 11, relay.coord,
                   {21}, relay.cb_type->type_id);
        addMapping(device, 4, relay.coord, 12, target.coord,
                   {30}, target.cb_type->type_id);
        for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
        if (control == 1) relay.cb.local.local.setBit(33);
        std::vector<std::array<NodeMask, 5>> before;
        for (const auto& tile : device.tile_grid)
            before.push_back({tile.cb.src.jump, tile.cb.dst.jump,
                              tile.cb.joint.jump, tile.cb.local.local,
                              tile.pin_state.leased_nodes});
        auto index = pnr::buildBackwardResolveIndex(device, target.coord, 4);
        int polls = 0;
        std::ostringstream decisions;
        const auto result = pnr::traceBackwardToTakeoff(target, bit(0),
            source.coord, index,
            [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                if (&tile != &source || src != 11) return false;
                choice.local = 5;
                return true;
            }, [&] { return control == 2 && ++polls >= 2; }, nullptr, &decisions);
        require(result.success == (control == 0),
                "backward LOCAL chain missed its second free successor: control=" +
                    std::to_string(control) + " expanded=" +
                    std::to_string(result.expanded) + " probes=" +
                    std::to_string(result.probe_calls) + " trace=" + decisions.str());
        if (control == 0) {
            bool selected_second = false, selected_dead = false;
            for (const auto& edge : result.fragments)
                if (edge.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                    edge.from_node_type == fpga::CB_NODE_LOCAL &&
                    edge.to_node_type == fpga::CB_NODE_LOCAL &&
                    edge.from_node == 31) {
                    selected_second |= edge.to_node == 33;
                    selected_dead |= edge.to_node == 32;
                }
            require(result.source_tile == &source && result.probe_calls == 2 &&
                        result.expanded == 2 && result.fragments.size() == 7 &&
                        selected_second && !selected_dead,
                    "backward LOCAL trace selected the dead first successor");
        }
        if (control == 2)
            require(polls >= 2 && !result.success,
                    "backward LOCAL chain did not honor cancellation after descent");
        for (size_t i = 0; i < before.size(); ++i) {
            const auto& tile = device.tile_grid[i];
            require((std::array<NodeMask, 5>{tile.cb.src.jump, tile.cb.dst.jump,
                        tile.cb.joint.jump, tile.cb.local.local,
                        tile.pin_state.leased_nodes}) == before[i],
                    "backward LOCAL successor trace changed live or temporary leases");
        }
    }
}

int main(int argc, char** argv)
{
    try {
        if (argc == 2 && std::string(argv[1]) == "source_trace") {
            sourceMovingTrace();
            typedSourceIngress();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "typed_ingress") {
            typedSourceIngress();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "short_hops") {
            shortHopPriority();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "destination_short_hops") {
            destinationShortHopPriority();
            forwardResolvedLocalLinks();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "forward_child_retry") {
            destinationForwardChildRetry();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "forward_local_terminal") {
            destinationForwardLocalTerminal();
            destinationForwardJointLocalTerminal();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "forward_resolved_local_src") {
            destinationForwardResolvedLocalSource();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "backward_midtrace_local") {
            backwardMidtraceLocalChain();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "backward_child_return") {
            backwardChildReturn();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "backward_exhausted_src_reuse") {
            backwardExhaustedSourceReuse();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "backward_local_successor_return") {
            backwardLocalSuccessorReturn();
            return 0;
        }
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
