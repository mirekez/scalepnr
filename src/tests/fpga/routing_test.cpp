#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "Cell.h"
#include "Conn.h"
#include "Module.h"
#include "Tile.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <utility>

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

std::vector<fpga::Tile*> resetDeviceGrid(int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;
    device.local_route_wire_mappings.clear();
    device.tile_grid.resize(static_cast<size_t>(width * height));

    std::vector<fpga::Tile*> tiles;
    tiles.reserve(device.tile_grid.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y * width + x)];
            tile.coord = {x, y};
            tile.name = {x, y};
            tile.routedNets.clear();
            tile.cb = {};
            tile.cb_type = nullptr;
            tile.pin_state = {};
            tiles.push_back(&tile);
        }
    }
    return tiles;
}

fpga::Tile& resetDevice()
{
    return *resetDeviceGrid(1, 1).front();
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


struct MuxPlacementFixture
{
    Referable<rtl::Module> parent;
    Referable<rtl::Module> cell_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    int next_designator = 1000;

    MuxPlacementFixture()
    {
        parent.name = "top";
        parent.is_blackbox = false;
        parent.nets.reserve(32);
        cell_module.name = "primitive";
        cell_module.is_blackbox = true;
        cell_module.parent_ref.set(&parent);
    }

    Referable<rtl::Cell>* makeCell(const std::string& name, const std::string& type,
                                   const std::vector<std::pair<std::string, int>>& ports)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = type;
        cell->module_ref.set(&cell_module);
        cell->ports.reserve(ports.size());
        for (const auto& [port_name, port_type] : ports) {
            rtl::Port port;
            port.name = port_name;
            port.type = static_cast<decltype(port.type)>(port_type);
            port.designator = -1;
            cell->ports.emplace_back(std::move(port));
        }
        Referable<rtl::Cell>* raw = cell.get();
        cells.push_back(std::move(cell));
        return raw;
    }

    Referable<rtl::Inst>* makeInst(const std::string& name, const std::string& type,
                                   const std::vector<std::pair<std::string, int>>& ports)
    {
        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(makeCell(name + "_cell", type, ports));
        inst->pos = -1;
        inst->conns.reserve(ports.size());
        for (auto& port : inst->cell_ref->ports) {
            auto& conn = inst->conns.emplace_back();
            conn.port_ref.set(&port);
            conn.inst_ref.set(inst.get());
        }
        Referable<rtl::Inst>* raw = inst.get();
        insts.push_back(std::move(inst));
        return raw;
    }

    Referable<rtl::Conn>* conn(Referable<rtl::Inst>* inst, const std::string& port_name)
    {
        for (auto& conn_ref : inst->conns) {
            if (conn_ref.port_ref.peer && conn_ref.port_ref->name == port_name) {
                return &conn_ref;
            }
        }
        return nullptr;
    }

    rtl::Net& connect(Referable<rtl::Inst>* driver, const std::string& driver_port,
                      Referable<rtl::Inst>* sink, const std::string& sink_port,
                      const std::string& net_name)
    {
        Referable<rtl::Conn>* out = conn(driver, driver_port);
        Referable<rtl::Conn>* in = conn(sink, sink_port);
        require(out && in, "mux placement test connection references a missing port");
        int designator = next_designator++;
        out->port_ref->designator = designator;
        in->port_ref->designator = designator;
        in->set(out);
        auto& net = parent.nets.emplace_back();
        net.name = net_name;
        net.designators.push_back(designator);
        return net;
    }
};

fpga::Tile& resetMuxPlacementTile(fpga::TileType& tile_type)
{
    fpga::Tile& tile = resetDevice();
    tile_type.name = "CLBLL_TEST";
    tile_type.num = 1;
    tile_type.sites.clear();
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE0", .type = "LOGIC", .pos = 0});
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE1", .type = "LOGIC", .pos = 1});
    tile.tile_type = &tile_type;
    return tile;
}

struct LeafMuxShape
{
    Referable<rtl::Inst>* mux = nullptr;
    Referable<rtl::Inst>* lut0 = nullptr;
    Referable<rtl::Inst>* lut1 = nullptr;
};

LeafMuxShape makeLeafMux(MuxPlacementFixture& fixture, const std::string& prefix,
                         std::vector<rtl::Net*>& internal_nets)
{
    LeafMuxShape shape;
    shape.lut0 = fixture.makeInst(prefix + "_lut0", "LUT6", {{"O", rtl::Port::PORT_OUT}});
    shape.lut1 = fixture.makeInst(prefix + "_lut1", "LUT6", {{"O", rtl::Port::PORT_OUT}});
    shape.mux = fixture.makeInst(prefix + "_mux", "MUX2",
        {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
    internal_nets.push_back(&fixture.connect(shape.lut0, "O", shape.mux, "I0", prefix + "_i0"));
    internal_nets.push_back(&fixture.connect(shape.lut1, "O", shape.mux, "I1", prefix + "_i1"));
    return shape;
}

void unrelated_muxes_cannot_share_output_local_and_packed_mux_inputs_are_void()
{
    fpga::TileType tile_type{"CLBLL_TEST", 1};
    fpga::Tile& tile = resetMuxPlacementTile(tile_type);
    MuxPlacementFixture fixture;

    std::vector<rtl::Net*> mux0_nets;
    std::vector<rtl::Net*> mux1_nets;
    std::vector<rtl::Net*> mux2_nets;
    LeafMuxShape mux0 = makeLeafMux(fixture, "mux0", mux0_nets);
    LeafMuxShape mux1 = makeLeafMux(fixture, "mux1", mux1_nets);
    LeafMuxShape mux2 = makeLeafMux(fixture, "mux2", mux2_nets);

    int pos0 = tile.tryAdd(mux0.mux);
    int pos1 = tile.tryAdd(mux1.mux);
    require(pos0 >= 0, "first packed mux could not be placed");
    require(pos1 >= 0, "second packed mux could not be placed");
    require(mux0.lut0->tile.peer && mux0.lut1->tile.peer, "first mux was not packed with its LUT drivers");
    require(mux1.lut0->tile.peer && mux1.lut1->tile.peer, "second mux was not packed with its LUT drivers");
    require(pos0 != pos1, "test setup expected the first two muxes in distinct slots");
    for (rtl::Net* net : mux0_nets) {
        require(net && net->void_net, "first packed mux input net was not marked void");
    }
    for (rtl::Net* net : mux1_nets) {
        require(net && net->void_net, "second packed mux input net was not marked void");
    }
    for (rtl::Net* net : mux2_nets) {
        require(net && !net->void_net, "unplaced mux input net was incorrectly marked void");
    }

    int pos2 = tile.tryAdd(mux2.mux);
    require(pos2 < 0,
        "unrelated mux was allowed into a second site position that shares an already reserved output local");
}

void joint_mediated_src_nodes_are_indexed()
{
    fpga::CBType type{};
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

void can_in_rejects_unconnected_double_joint_paths()
{
    fpga::CBType type{};
    int joint = -1;
    for (int i = 0; i < CB_MAX_NODES; ++i) {
        type.dst_local[i].local = u256{};
        type.dst_joint[i].joint = u256{};
        type.joint_local[i].local = u256{};
        type.joint_joint[i].joint = u256{};
    }

    type.dst_joint[10].joint |= bit(3);
    type.joint_joint[3].joint |= bit(4);
    type.joint_local[5].local |= bit(20);

    require(!type.canIn(10, 20, joint),
        "dst joint with no path to local joint was incorrectly accepted");

    type.joint_joint[3].joint |= bit(5);
    require(type.canIn(10, 20, joint),
        "dst->joint->joint->local path was not accepted after adding the missing joint link");
}

void loaded_crossbar_local_and_joint_masks_use_router_bit_numbering()
{
    CBTypeSpec spec;
    spec.nodes.emplace("OUT5", "IN90");
    spec.nodes.emplace("OUT6", "JOINT7");
    spec.nodes.emplace("JOINT7", "IN91");

    fpga::TechMap map;
    fpga::CBType type{"GENERIC_CB"};
    type.loadFromSpec(spec, map);

    int out5 = type.local_nodes_by_name.at("OUT5");
    int out6 = type.local_nodes_by_name.at("OUT6");
    int in90 = type.local_nodes_by_name.at("IN90");
    int in91 = type.local_nodes_by_name.at("IN91");
    int joint7 = type.joint_nodes_by_name.at("JOINT7");

    // Check: loaded local masks must use the same low-lane bit numbering as routing leases/tests.
    require(isSet(type.local_local[out5].local, in90), "loaded local-to-local mask used incompatible bit numbering");
    // Check: loaded joint masks must also use router bit numbering, otherwise joint-mediated paths are invisible.
    require(isSet(type.local_joint[out6].joint, joint7), "loaded local-to-joint mask used incompatible bit numbering");
    require(isSet(type.joint_local[joint7].local, in91), "loaded joint-to-local mask used incompatible bit numbering");
    type.rebuildOutgoingSrcs();
    require(type.local_input_nodes != u256{} && type.local_output_nodes != u256{},
        "loaded local input/output masks were not populated with router bit numbering");
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

enum class RegressionRoutingMode
{
    Generic,
    Fanout,
    Moving,
};

struct RegressionTask
{
    std::string name;
    rtl::Port* source_port = nullptr;
    fpga::Coord source_tile;
    fpga::Coord branch_tile;
    fpga::Coord sink_tile;
    TestRoute* old_route = nullptr;
    int source_local = 120;
    int source_exit = 24;
    bool routed = false;
    bool fanout = false;
    bool moved = false;
    std::vector<fpga::Wire> route;
};

struct RegressionBatchState
{
    std::vector<RegressionTask> deferred_fanouts;
    size_t generic_routed = 0;
    size_t fanout_routed = 0;
    size_t moved_cells = 0;
};

void leaseRegressionFragment(const fpga::Wire& fragment)
{
    fpga::Tile* tile = fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
    require(tile != nullptr, "regression route tried to lease a missing tile");
    tile->cb.src.jump |= bit(fragment.jump);
    if (fragment.pos == 0) {
        tile->cb.local.local |= bit(fragment.local);
    }
    else {
        tile->cb.dst.jump |= bit(fragment.local);
    }
}

void routeRegressionTasks(RegressionRoutingMode mode, std::vector<RegressionTask>& tasks,
                          RegressionBatchState& state)
{
    constexpr uint64_t routed_source_mark = 0xace551;
    if (mode == RegressionRoutingMode::Moving) {
        bool moved_any = false;
        for (RegressionTask& task : tasks) {
            if (task.old_route && fpga::unrouteNet(task.old_route->net)) {
                task.old_route = nullptr;
            }
            task.source_tile = {1, 0};
            task.branch_tile = {1, 0};
            task.moved = true;
            moved_any = true;
            if (task.source_port) {
                task.source_port->mark = 0;
            }
        }
        if (moved_any) {
            ++state.moved_cells;
        }
        routeRegressionTasks(RegressionRoutingMode::Generic, tasks, state);
        std::vector<RegressionTask> fanouts = std::move(state.deferred_fanouts);
        state.deferred_fanouts.clear();
        routeRegressionTasks(RegressionRoutingMode::Fanout, fanouts, state);
        return;
    }

    for (RegressionTask& task : tasks) {
        if (mode == RegressionRoutingMode::Generic) {
            if (task.source_port && task.source_port->mark == routed_source_mark) {
                task.fanout = true;
                state.deferred_fanouts.push_back(task);
                continue;
            }
            if (task.source_port) {
                task.source_port->mark = routed_source_mark;
            }
            fpga::Wire fragment;
            fragment.type = fpga::Wire::WIRE_CROSSBAR;
            fragment.from = task.source_tile;
            fragment.to = task.branch_tile;
            fragment.local = task.source_local;
            fragment.jump = task.source_exit;
            fragment.pos = 0;
            fragment.net_name = task.name;
            task.route = {fragment};
            leaseRegressionFragment(fragment);
            task.routed = true;
            ++state.generic_routed;
            continue;
        }

        fpga::Wire shared;
        shared.type = fpga::Wire::WIRE_CROSSBAR;
        shared.from = task.source_tile;
        shared.to = task.branch_tile;
        shared.local = task.source_local;
        shared.jump = task.source_exit;
        shared.pos = 1;
        shared.shared = true;
        shared.net_name = task.name;

        fpga::Wire branch;
        branch.type = fpga::Wire::WIRE_CROSSBAR;
        branch.from = task.branch_tile;
        branch.to = task.sink_tile;
        branch.local = task.source_local + 1;
        branch.jump = task.source_exit + 1;
        branch.pos = 1;
        branch.net_name = task.name;

        task.route = {shared, branch};
        leaseRegressionFragment(branch);
        task.routed = true;
        ++state.fanout_routed;
    }
}

void routing_mode_generic_routes_only_one_net_from_single_source_port()
{
    resetDeviceGrid(3, 1);
    rtl::Port source;
    source.name = "O";
    source.type = rtl::Port::PORT_OUT;

    std::vector<RegressionTask> tasks;
    for (int i = 0; i < 4; ++i) {
        tasks.push_back(RegressionTask{
            "source_fanout_" + std::to_string(i),
            &source,
            {0, 0},
            {1, 0},
            {2, 0},
            nullptr,
            120 + i,
            24 + i,
        });
    }

    RegressionBatchState state;
    routeRegressionTasks(RegressionRoutingMode::Generic, tasks, state);

    // Check: generic mode consumes exactly one physical route start from one logical source port.
    require(state.generic_routed == 1, "generic mode routed more than one net from the same source port");
    // Check: secondary loads of the same source port are deferred to fanout routing.
    require(state.deferred_fanouts.size() == 3, "generic mode did not defer secondary source-port fanouts");
    // Check: the selected generic route really leased its source tile exit.
    require(isSet(fpga::Device::current().tile_grid.front().cb.src.jump, 24),
        "generic mode did not lease the first source exit");
}

void routing_mode_fanout_branches_away_from_source_tile()
{
    resetDeviceGrid(4, 1);
    rtl::Port source;
    source.name = "O";
    source.type = rtl::Port::PORT_OUT;

    std::vector<RegressionTask> tasks;
    for (int i = 0; i < 3; ++i) {
        tasks.push_back(RegressionTask{
            "branch_fanout_" + std::to_string(i),
            &source,
            {0, 0},
            {1, 0},
            {2 + i % 2, 0},
            nullptr,
            100,
            28,
        });
    }

    RegressionBatchState state;
    routeRegressionTasks(RegressionRoutingMode::Generic, tasks, state);
    std::vector<RegressionTask> fanouts = std::move(state.deferred_fanouts);
    state.deferred_fanouts.clear();
    routeRegressionTasks(RegressionRoutingMode::Fanout, fanouts, state);

    // Check: fanout mode routed all deferred loads through branch points.
    require(state.fanout_routed == 2, "fanout mode did not route every deferred fanout");
    for (const RegressionTask& task : fanouts) {
        auto first_new = std::find_if(task.route.begin(), task.route.end(), [](const fpga::Wire& wire) {
            return !wire.shared;
        });
        // Check: fanout mode starts the new branch after the shared trunk, not at the original source tile.
        require(first_new != task.route.end() && first_new->from.x != task.source_tile.x,
            "fanout mode started a secondary fanout in the original source tile");
        // Check: the branch point is the tile selected from the existing routed trunk.
        require(first_new->from.x == task.branch_tile.x && first_new->from.y == task.branch_tile.y,
            "fanout mode did not start from the expected branch tile");
    }
}

void routing_mode_moving_unroutes_old_cell_tree_and_reroutes_hierarchy()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(2, 1);
    fpga::Tile& old_tile = *tiles[0];
    fpga::Tile& new_tile = *tiles[1];

    std::vector<std::unique_ptr<TestRoute>> local_blockers;
    for (int i = 0; i < 8; ++i) {
        auto blocker = std::make_unique<TestRoute>();
        blocker->transit = false;
        blocker->exit = 8 + i;
        blocker->local = 64 + i;
        addRoute(old_tile, *blocker, "old_local_blocker_" + std::to_string(i));
        local_blockers.push_back(std::move(blocker));
    }

    TestRoute previous_route;
    previous_route.transit = false;
    previous_route.exit = 32;
    previous_route.local = 120;
    addRoute(old_tile, previous_route, "moved_cell_previous_route");

    rtl::Port source;
    source.name = "O";
    source.type = rtl::Port::PORT_OUT;
    std::vector<RegressionTask> tasks;
    for (int i = 0; i < 3; ++i) {
        tasks.push_back(RegressionTask{
            "moving_hierarchy_" + std::to_string(i),
            &source,
            old_tile.coord,
            old_tile.coord,
            new_tile.coord,
            i == 0 ? &previous_route : nullptr,
            140 + i,
            40 + i,
        });
    }

    // Check: the old source tile has no modeled local exits left for another local start.
    for (int i = 0; i < 8; ++i) {
        require(isSet(old_tile.cb.src.jump, 8 + i), "moving test setup did not occupy every old local exit");
    }

    RegressionBatchState state;
    routeRegressionTasks(RegressionRoutingMode::Moving, tasks, state);

    // Check: moving mode removed the previous route owned by the moved cell.
    require(!isSet(old_tile.cb.src.jump, 32), "moving mode did not clear the old route source exit");
    // Check: moving mode cleared the old local lease as part of unrouting the moved cell tree.
    require(!isSet(old_tile.cb.local.local, 120), "moving mode did not clear the old route local lease");
    // Check: unrelated local-to-exit blockers were not unrouted by moving cleanup.
    for (int i = 0; i < 8; ++i) {
        require(isSet(old_tile.cb.src.jump, 8 + i), "moving mode removed an unrelated local blocker");
    }
    // Check: moving mode reran generic routing first and fanout routing for the same source hierarchy after relocation.
    require(state.moved_cells == 1 && state.generic_routed == 1 && state.fanout_routed == 2,
        "moving mode did not reroute the relocated hierarchy through generic then fanout stages");
    // Check: the relocated generic route leased the new tile, proving routing did not retry from the blocked old tile.
    require(new_tile.cb.src.jump != u256{}, "moving mode did not lease any exit on the new tile");
}

}

int main()
{
    try {
        joint_mediated_src_nodes_are_indexed();
        can_in_rejects_unconnected_double_joint_paths();
        loaded_crossbar_local_and_joint_masks_use_router_bit_numbering();
        tile_type_mapping_models_all_16_ff_input_pins_per_clb_tile();
        unrelated_muxes_cannot_share_output_local_and_packed_mux_inputs_are_void();
        routing_mode_generic_routes_only_one_net_from_single_source_port();
        routing_mode_fanout_branches_away_from_source_tile();
        routing_mode_moving_unroutes_old_cell_tree_and_reroutes_hierarchy();
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
