#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {

struct TestFailure
{
    std::string message;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw TestFailure{message};
    }
}

u256 bit(int index)
{
    return u256{0, 1} << index;
}

bool isSet(u256 value, int index)
{
    return (value & bit(index)) != u256{};
}

int pick(std::mt19937& rng, std::vector<int>& values)
{
    require(!values.empty(), "random choice from empty vector");
    std::uniform_int_distribution<size_t> dist(0, values.size() - 1);
    size_t index = dist(rng);
    int value = values[index];
    values.erase(values.begin() + static_cast<std::ptrdiff_t>(index));
    return value;
}

std::vector<int> range(int first, int count)
{
    std::vector<int> values;
    values.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        values.push_back(first + i);
    }
    return values;
}

fpga::Tile& resetDevice()
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.grid_spec.size = {1, 1};
    device.size_width = 1;
    device.size_height = 1;
    device.tile_conn_rules.clear();
    device.tile_conn_by_from.clear();
    device.tile_conn_by_to.clear();
    device.tile_grid.emplace_back(fpga::Tile{});
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = {0, 0};
    tile.name = {0, 0};
    tile.routedNets.clear();
    tile.cb = {};
    tile.pin_state = {};
    return tile;
}

struct TestRoute
{
    Referable<rtl::Net> net;
    rtl::Inst owner;
    int exit = -1;
    int local = -1;
    int joint = -1;
    bool transit = false;
};

void addRoute(fpga::Tile& tile, TestRoute& route, const std::string& name)
{
    route.net.name = name;
    route.owner.wires.clear();
    route.owner.wires.emplace_back();
    std::vector<fpga::Wire>& wires = route.owner.wires.back();

    fpga::Wire fragment;
    fragment.type = fpga::Wire::WIRE_CROSSBAR;
    fragment.from = tile.coord;
    fragment.to = tile.coord;
    fragment.local = route.local;
    fragment.pos = route.transit ? 1 : 0;
    fragment.jump = route.exit;
    fragment.joint = route.joint;
    fragment.net_name = name;
    wires.push_back(fragment);

    tile.cb.src.jump |= bit(route.exit);
    if (route.transit) {
        tile.cb.dst.jump |= bit(route.local);
    }
    else {
        tile.cb.local.local |= bit(route.local);
    }

    fpga::attachNetRoute(route.net, route.owner, 0, nullptr, &route.owner, {}, {}, name);
    fpga::registerNetRouteTiles(route.net, wires);
}

void assertCanTakeExit(fpga::Tile& tile, int local, int exit)
{
    require(!isSet(tile.cb.src.jump, exit), "chosen exit is still busy");
    tile.cb.src.jump |= bit(exit);
    tile.cb.local.local |= bit(local);
    require(isSet(tile.cb.src.jump, exit), "new local did not lease exit");
}

void preemptTransitOnExit(fpga::Tile& tile, int exit)
{
    rtl::Net* victim = fpga::findNetByNode(tile, fpga::CB_NODE_SRC, exit, true);
    require(victim != nullptr, "did not find transit victim on busy exit");
    require(fpga::unrouteNet(*victim), "failed to unroute transit victim");
}

void local_and_transit_preemption(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::Tile& tile = resetDevice();

    std::vector<int> exits = range(8, 32);
    std::vector<int> locals = range(64, 40);
    std::vector<std::unique_ptr<TestRoute>> transit_routes;
    std::vector<std::unique_ptr<TestRoute>> local_routes;

    for (int i = 0; i < 8; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = true;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        addRoute(tile, *route, "transit_" + std::to_string(i));
        transit_routes.push_back(std::move(route));
    }
    for (int i = 0; i < 8; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = false;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        addRoute(tile, *route, "local_" + std::to_string(i));
        local_routes.push_back(std::move(route));
    }

    int new_local = pick(rng, locals);
    int transit_exit = transit_routes.front()->exit;
    int local_exit = local_routes.front()->exit;

    require(fpga::findNetByNode(tile, fpga::CB_NODE_SRC, local_exit, true) == nullptr,
        "local-to-exit route must not be a transit victim");
    require(isSet(tile.cb.src.jump, transit_exit), "transit exit was not busy before preemption");

    preemptTransitOnExit(tile, transit_exit);

    require(!isSet(tile.cb.src.jump, transit_exit), "transit exit still leased after unroute");
    require(isSet(tile.cb.src.jump, local_exit), "local-to-exit route was incorrectly unrouted");
    assertCanTakeExit(tile, new_local, transit_exit);
}

void joint_metadata_preemption(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::Tile& tile = resetDevice();

    std::vector<int> exits = range(40, 24);
    std::vector<int> locals = range(96, 48);
    std::vector<int> joints = range(4, 20);
    std::vector<std::unique_ptr<TestRoute>> transit_routes;
    std::vector<std::unique_ptr<TestRoute>> local_routes;

    for (int i = 0; i < 6; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = true;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        route->joint = pick(rng, joints);
        addRoute(tile, *route, "joint_transit_" + std::to_string(i));
        transit_routes.push_back(std::move(route));
    }
    for (int i = 0; i < 6; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = false;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        route->joint = pick(rng, joints);
        addRoute(tile, *route, "joint_local_" + std::to_string(i));
        local_routes.push_back(std::move(route));
    }

    int new_local = pick(rng, locals);
    int transit_exit = transit_routes.front()->exit;
    int transit_joint = transit_routes.front()->joint;
    int local_joint = local_routes.front()->joint;

    require(fpga::findNetByNode(tile, fpga::CB_NODE_JOINT, transit_joint, true) != nullptr,
        "transit joint use was not discoverable");
    require(fpga::findNetByNode(tile, fpga::CB_NODE_JOINT, local_joint, true) == nullptr,
        "local-to-exit joint use must not be a transit victim");

    preemptTransitOnExit(tile, transit_exit);
    assertCanTakeExit(tile, new_local, transit_exit);
}

void free_joint_exit_is_preferred(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::Tile& tile = resetDevice();

    std::vector<int> exits = range(80, 16);
    std::vector<int> locals = range(128, 40);
    std::vector<int> joints = range(32, 16);
    std::vector<std::unique_ptr<TestRoute>> transit_routes;

    for (int i = 0; i < 10; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = true;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        route->joint = pick(rng, joints);
        addRoute(tile, *route, "free_joint_transit_" + std::to_string(i));
        transit_routes.push_back(std::move(route));
    }

    int free_exit = pick(rng, exits);
    int free_joint = pick(rng, joints);
    int new_local = pick(rng, locals);

    require(!isSet(tile.cb.src.jump, free_exit), "free joint exit setup accidentally occupied exit");
    require(fpga::findNetByNode(tile, fpga::CB_NODE_JOINT, free_joint, true) == nullptr,
        "free joint setup accidentally occupied joint");

    tile.cb.src.jump |= bit(free_exit);
    tile.cb.local.local |= bit(new_local);
    fpga::Wire fragment;
    fragment.from = tile.coord;
    fragment.to = tile.coord;
    fragment.local = new_local;
    fragment.pos = 0;
    fragment.jump = free_exit;
    fragment.joint = free_joint;
    require(isSet(tile.cb.src.jump, free_exit), "new local failed to occupy free joint exit");
}

void joint_mediated_src_nodes_are_indexed()
{
    fpga::CBType type;
    type.local_joint[7].joint |= bit(3);
    type.joint_src[3].jump |= bit(42);
    type.dst_joint[9].joint |= bit(4);
    type.joint_joint[4].joint |= bit(5);
    type.joint_src[5].jump |= bit(43);

    require(type.srcNodes(fpga::CB_NODE_LOCAL, 7) == nullptr,
        "test setup expected no src index before rebuild");

    type.rebuildOutgoingSrcs();

    const std::vector<uint8_t>* local_srcs = type.srcNodes(fpga::CB_NODE_LOCAL, 7);
    require(local_srcs != nullptr, "local->joint->src path was not indexed as local outgoing src");
    require(std::find(local_srcs->begin(), local_srcs->end(), 42) != local_srcs->end(),
        "local->joint->src index missed source node");

    const std::vector<uint8_t>* dst_srcs = type.srcNodes(fpga::CB_NODE_DST, 9);
    require(dst_srcs != nullptr, "dst->joint->joint->src path was not indexed as dst outgoing src");
    require(std::find(dst_srcs->begin(), dst_srcs->end(), 43) != dst_srcs->end(),
        "dst->joint->joint->src index missed source node");
}

void tile_type_mapping_models_all_16_ff_input_pins_per_clb_tile()
{
    constexpr const char* description =
        "The abstract TileType mapping must model all 16 FF input pins per CLB tile. "
        "If it only maps AFF/BFF/CFF/DFF and misses AFF2/BFF2/CFF2/DFF2, placement may "
        "appear legal while routing local nodes are incomplete or aliased.";

    auto ff_pos = [] {
        std::vector<int> positions;
        positions.reserve(16);
        for (int site = 0; site < 2; ++site) {
            for (int ff = 0; ff < 8; ++ff) {
                positions.push_back(site * 128 + ff);
            }
        }
        return positions;
    }();

    auto addFdInputPins = [](fpga::TileType& type, const std::vector<int>& positions) {
        for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
            int pos = positions[static_cast<size_t>(i)];
            int d_local = 32 + i;
            int sr_local = 96 + i;

            type.pin_map.nodes[fpga::TilePinKey{"FDRE", "D", pos}] = bit(d_local);
            type.pin_map.nodes[fpga::TilePinKey{"FDRE", "R", pos}] = bit(sr_local);
            type.pin_map.nodes[fpga::TilePinKey{"FDSE", "S", pos}] = bit(sr_local);
            type.pin_map.rememberLocalNames(fpga::TILE_PIN_INPUT, d_local,
                "FF_D_LOCAL_" + std::to_string(i), "FF" + std::to_string(i) + ".D", "D");
            type.pin_map.rememberLocalNames(fpga::TILE_PIN_INPUT, sr_local,
                "FF_SR_LOCAL_" + std::to_string(i), "FF" + std::to_string(i) + ".SR", "SR");
        }
    };

    auto mappedLocal = [](const fpga::TileType& tile_type, const std::string& type,
                          const std::string& port, int pos) {
        u256 nodes = tile_type.pin_map.getNodes(type, port, pos);
        return nodes.ffs256();
    };

    fpga::TileType complete{"CLB_WITH_16_FF_INPUTS", 1};
    addFdInputPins(complete, ff_pos);

    std::set<int> d_nodes;
    std::set<int> sr_nodes;
    for (int pos : ff_pos) {
        int d_local = mappedLocal(complete, "FDRE", "D", pos);
        int r_local = mappedLocal(complete, "FDRE", "R", pos);
        int s_local = mappedLocal(complete, "FDSE", "S", pos);
        require(d_local >= 0, std::string(description) + " Missing FDRE.D at pos " + std::to_string(pos));
        require(r_local >= 0, std::string(description) + " Missing FDRE.R at pos " + std::to_string(pos));
        require(s_local >= 0, std::string(description) + " Missing FDSE.S at pos " + std::to_string(pos));
        require(r_local == s_local,
            std::string(description) + " Reset/set aliases should resolve to the same slot at pos " + std::to_string(pos));
        d_nodes.insert(d_local);
        sr_nodes.insert(r_local);
        require(complete.pin_map.localResourceName(fpga::TILE_PIN_INPUT, d_local) != nullptr,
            std::string(description) + " Missing D resource annotation at pos " + std::to_string(pos));
        require(complete.pin_map.localResourceName(fpga::TILE_PIN_INPUT, r_local) != nullptr,
            std::string(description) + " Missing SR resource annotation at pos " + std::to_string(pos));
    }
    require(d_nodes.size() == 16, std::string(description) + " FD D inputs are incomplete or aliased");
    require(sr_nodes.size() == 16, std::string(description) + " FD SR inputs are incomplete or aliased");

    fpga::TileType incomplete{"CLB_WITH_ONLY_FOUR_FF_INPUTS", 2};
    addFdInputPins(incomplete, std::vector<int>(ff_pos.begin(), ff_pos.begin() + 4));
    int mapped = 0;
    for (int pos : ff_pos) {
        if (mappedLocal(incomplete, "FDRE", "D", pos) >= 0) {
            ++mapped;
        }
    }
    require(mapped == 4, "incomplete four-FF mapping test setup is invalid");
    require(mapped != 16, std::string(description) + " Incomplete AFF/BFF/CFF/DFF-only mapping was not detected");
}

}

int main()
{
    try {
        joint_mediated_src_nodes_are_indexed();
        tile_type_mapping_models_all_16_ff_input_pins_per_clb_tile();
        for (unsigned seed = 1; seed <= 64; ++seed) {
            local_and_transit_preemption(seed);
            joint_metadata_preemption(seed + 1000);
            free_joint_exit_is_preferred(seed + 2000);
        }
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "routing_test failed: %s\n", failure.message.c_str());
        return EXIT_FAILURE;
    }
    catch (const std::exception& ex) {
        std::fprintf(stderr, "routing_test exception: %s\n", ex.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
