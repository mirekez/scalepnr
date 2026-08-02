#include "Device.h"

#include <algorithm>
#include <stdexcept>
#include <string>
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

fpga::CBType& addType(fpga::Device& device, const std::string& name)
{
    uint16_t id = static_cast<uint16_t>(device.cb_types.size());
    device.cb_types.emplace_back();
    fpga::CBType& type = device.cb_types.back();
    type.name = name;
    type.type_id = id;
    type.base_type_id = id;
    return type;
}

bool hasBit(NodeMask mask, int node)
{
    return (mask & (NodeMask{0, 1} << node)) != NodeMask{};
}

fpga::ParsedTileConnRule rule(const std::string& from, const std::string& from_wire,
                              const std::string& to, const std::string& to_wire,
                              fpga::Coord delta = {1, 0})
{
    fpga::ParsedTileConnRule result;
    result.from_tile_type = from;
    result.to_tile_type = to;
    result.delta = delta;
    result.wire_pairs.push_back(fpga::ParsedTileConnPair{from_wire, to_wire});
    return result;
}

void runExactLocalIdentityRegression()
{
    fpga::Device device;
    fpga::CBType& fork = addType(device, "RST_FORK_BOX");
    fpga::CBType& upper = addType(device, "UVW_UPPER_BOX");
    fpga::CBType& lower = addType(device, "XYZ_LOWER_BOX");

    // Reproduce the old parser alias: two exact wires shared one numeric stem.
    fork.rememberNodeName(fpga::CB_NODE_LOCAL, 7, "RST_LINE4_LOW");
    fork.rememberNodeName(fpga::CB_NODE_LOCAL, 7, "RST_LINE4_HIGH");
    upper.rememberNodeName(fpga::CB_NODE_LOCAL, 9, "UVW_LANDING");
    lower.rememberNodeName(fpga::CB_NODE_LOCAL, 11, "XYZ_LANDING");

    device.deferred_cb_types.insert(fork.name);
    device.deferred_cb_types.insert(upper.name);
    device.deferred_cb_types.insert(lower.name);
    device.tileconn_rules.push_back(rule(
        fork.name, "RST_LINE4_LOW", lower.name, "XYZ_LANDING", {0, 1}));
    device.tileconn_rules.push_back(rule(
        fork.name, "RST_LINE4_HIGH", upper.name, "UVW_LANDING", {0, -2}));
    device.route_wire_graph[fork.name + "\nRST_LINE4_LOW"].push_back(
        fpga::RouteWireGraphEdge{fork.name, "RST_LINE4_HIGH", {0, 0}, false, true});
    device.route_wire_graph[fork.name + "\nRST_LINE4_HIGH"].push_back(
        fpga::RouteWireGraphEdge{fork.name, "RST_LINE4_LOW", {0, 0}, false, false});

    device.rebuildLocalTransitions();

    int low = fork.nodeNum(fpga::CB_NODE_LOCAL, "RST_LINE4_LOW");
    int high = fork.nodeNum(fpga::CB_NODE_LOCAL, "RST_LINE4_HIGH");
    require(low >= 0 && high >= 0 && low != high,
        "exact dedicated wires with a shared numeric stem remained aliased");

    const auto& low_targets = fork.local_by_local[low];
    require(std::any_of(low_targets.begin(), low_targets.end(), [](const auto& target) {
        return target.delta.x == 0 && target.delta.y == 1;
    }), "lower exact wire lost its direct physical delta");
    require(std::none_of(low_targets.begin(), low_targets.end(), [](const auto& target) {
        return target.delta.x == 0 && target.delta.y == -2;
    }), "lower exact wire inherited the upper wire's physical delta");

    const auto& high_targets = fork.local_by_local[high];
    require(std::any_of(high_targets.begin(), high_targets.end(), [](const auto& target) {
        return target.delta.x == 0 && target.delta.y == -2;
    }), "upper exact wire lost its direct physical delta");
    require(std::any_of(low_targets.begin(), low_targets.end(), [&](const auto& target) {
        return target.delta.x == 0 && target.delta.y == 0
            && target.target_node_type == fpga::CB_NODE_LOCAL
            && hasBit(target.target_nodes, high);
    }), "declared immediate intra-tile transition was not materialized: low="
        + std::to_string(low) + " high=" + std::to_string(high)
        + " targets=" + std::to_string(low_targets.size()));
    require(std::none_of(high_targets.begin(), high_targets.end(), [&](const auto& target) {
        return target.delta.x == 0 && target.delta.y == 0 && hasBit(target.target_nodes, low);
    }), "undeclared reverse intra-tile transition was invented");
}

void runExactLandingTileRegression()
{
    fpga::Device device;
    fpga::CBType& source = addType(device, "AAA_ORIGIN_BOX");
    addType(device, "BBB_BARRIER_BOX");
    fpga::CBType& target = addType(device, "CCC_TARGET_BOX");
    source.rememberNodeName(fpga::CB_NODE_LOCAL, 3, "AAA_LINE");
    target.rememberNodeName(fpga::CB_NODE_LOCAL, 5, "CCC_LINE");
    source.local_by_local[3].push_back(fpga::CBType::ResolvedLocal{
        {1, 0}, target.type_id, fpga::CB_NODE_LOCAL, NodeMask{0, 1} << 5});

    device.size_width = 3;
    device.size_height = 1;
    device.tile_grid.resize(3);
    for (int x = 0; x < 3; ++x) {
        fpga::Tile& tile = device.tile_grid[static_cast<size_t>(x)];
        tile.coord = {x, 0};
        tile.cb_coord = tile.coord;
        tile.cb_type = &device.cb_types[static_cast<size_t>(x)];
        tile.cb.type = tile.cb_type;
    }
    // Reproduce the old bug: the physical landing redirects to a later matching route tile.
    device.tile_grid[1].cb_coord = device.tile_grid[2].coord;

    require(device.resolveLocalTargets(device.tile_grid[0], 3).empty(),
        "local transition skipped its exact physical landing through route-tile redirection");
}

void runDedicatedLocalTransitionRegression()
{
    fpga::Device device;
    fpga::CBType& source = addType(device, "ABC_SOURCE_BOX");
    fpga::CBType& pass = addType(device, "DEF_PASS_BOX");
    fpga::CBType& clock = addType(device, "GHI_SPECIAL_BOX");
    fpga::CBType& landing = addType(device, "JKL_LANDING_BOX");
    fpga::CBType& unrelated = addType(device, "MNO_UNRELATED_BOX");
    fpga::CBType& unrelated_pass = addType(device, "PQR_UNRELATED_PASS");

    source.rememberNodeName(fpga::CB_NODE_LOCAL, 11, "ABC_OUTPUT");
    clock.rememberNodeName(fpga::CB_NODE_LOCAL, 23, "GHI_INPUT");
    clock.rememberNodeName(fpga::CB_NODE_LOCAL, 29, "GHI_OUTPUT");
    landing.rememberNodeName(fpga::CB_NODE_DST, 37, "JKL_DESTINATION");
    unrelated.rememberNodeName(fpga::CB_NODE_LOCAL, 41, "MNO_OUTPUT");

    device.deferred_cb_types.insert(clock.name);
    device.tileconn_rules.push_back(rule(source.name, "ABC_OUTPUT", pass.name, "DEF_LINK"));
    device.tileconn_rules.push_back(rule(pass.name, "DEF_LINK", clock.name, "GHI_INPUT"));
    device.tileconn_rules.push_back(rule(clock.name, "GHI_OUTPUT", landing.name, "JKL_DESTINATION"));
    device.tileconn_rules.push_back(rule(
        unrelated.name, "MNO_OUTPUT", unrelated_pass.name, "PQR_LINK"));

    device.rebuildLocalTransitions();

    int pass_local = pass.nodeNum(fpga::CB_NODE_LOCAL, "DEF_LINK");
    require(pass_local >= 0,
        "dedicated numeric component did not allocate its pass-through local");
    require(unrelated_pass.nodeNum(fpga::CB_NODE_LOCAL, "PQR_LINK") < 0,
        "unrelated tile-connection component was imported into clock routing");

    const auto& source_targets = source.local_by_local[11];
    require(source_targets.size() == 1
            && source_targets.front().target_node_type == fpga::CB_NODE_LOCAL
            && hasBit(source_targets.front().target_nodes, pass_local),
        "source local did not resolve numerically to the pass-through local");

    const auto& landing_targets = clock.local_by_local[29];
    require(landing_targets.size() == 1
            && landing_targets.front().target_node_type == fpga::CB_NODE_DST
            && hasBit(landing_targets.front().target_nodes, 37),
        "dedicated local did not retain the landing node's destination role");
    require(landing.nodeNum(fpga::CB_NODE_LOCAL, "JKL_DESTINATION") < 0,
        "destination-role landing was duplicated as a local node");

    device.grid_spec.size = {4, 1};
    device.size_width = 4;
    device.size_height = 1;
    device.tile_grid.resize(4);
    for (int x = 0; x < 4; ++x) {
        fpga::Tile& tile = device.tile_grid[static_cast<size_t>(x)];
        tile.coord = {x, 0};
        tile.cb_coord = tile.coord;
        tile.cb_type = &device.cb_types[static_cast<size_t>(x)];
        tile.cb.type = tile.cb_type;
    }

    std::vector<fpga::TileLocalTarget> first = device.resolveLocalTargets(
        device.tile_grid[0], 11);
    require(first.size() == 1 && first.front().tile == &device.tile_grid[1]
            && first.front().node_type == fpga::CB_NODE_LOCAL
            && first.front().node == pass_local,
        "runtime numeric lookup did not cross the first tile connection");

    std::vector<fpga::TileLocalTarget> second = device.resolveLocalTargets(
        device.tile_grid[1], pass_local);
    require(second.size() == 2,
        "pass-through local did not retain both physical neighboring targets");
    require(std::any_of(second.begin(), second.end(), [&](const fpga::TileLocalTarget& target) {
        return target.tile == &device.tile_grid[2]
            && target.node_type == fpga::CB_NODE_LOCAL && target.node == 23;
    }), "runtime numeric lookup did not reach the dedicated endpoint");
}

}

int main()
{
    runDedicatedLocalTransitionRegression();
    runExactLocalIdentityRegression();
    runExactLandingTileRegression();
    return 0;
}
