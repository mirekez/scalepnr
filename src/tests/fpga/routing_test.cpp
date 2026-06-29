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

NodeMask bit(int index)
{
    return NodeMask{0, 1} << index;
}

bool isSet(NodeMask value, int index)
{
    return (value & bit(index)) != NodeMask{};
}

uint16_t elementBit(int index)
{
    return static_cast<uint16_t>(1u << index);
}

fpga::Element makeElement(const std::string& name, fpga::ElementType type, int bit)
{
    fpga::Element element;
    element.name = name;
    element.type = type;
    element.bitmap_pos = static_cast<uint16_t>(bit);
    element.elements_to_left = static_cast<int>(type);
    return element;
}

void connectElements(fpga::TileType& tile_type, fpga::ElementType left_type, int left_bit,
                     fpga::ElementType right_type, int right_bit)
{
    for (fpga::Element& element : tile_type.elements) {
        if (element.type == left_type && element.bitmap_pos == left_bit) {
            element.right_blockers[right_bit] |= elementBit(left_bit);
        }
        if (element.type == right_type && element.bitmap_pos == right_bit) {
            element.left_blockers[left_bit] |= elementBit(right_bit);
        }
    }
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
    fragment.to = route.transit ? fpga::Coord{tile.coord.x + 1, tile.coord.y} : tile.coord;
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

void releasing_route_fragment_keeps_deadend_for_same_src()
{
    fpga::Tile& tile = resetDevice();
    fpga::Wire fragment;
    fragment.type = fpga::Wire::WIRE_CROSSBAR;
    fragment.from = tile.coord;
    fragment.to = {tile.coord.x + 1, tile.coord.y};
    fragment.local = 77;
    fragment.jump = 33;
    fragment.pos = 1;
    std::vector<fpga::Wire> route{fragment};

    tile.cb.src.jump |= bit(fragment.jump);
    tile.cb.src_deadend.jump |= bit(fragment.jump);
    fpga::releaseRouteFragmentLease(route, 0);

    // Check: rollback of the fragment frees the lease but keeps the sticky deadend mark.
    require(!isSet(tile.cb.src.jump, fragment.jump), "rollback release did not clear the source lease");
    require(isSet(tile.cb.src_deadend.jump, fragment.jump), "rollback release cleared the sticky source deadend mark");
}

void releasing_route_fragment_clears_source_and_transit_node_classes()
{
    fpga::Tile& tile = resetDevice();
    fpga::Wire source;
    source.type = fpga::Wire::WIRE_CROSSBAR;
    source.from = tile.coord;
    source.to = {tile.coord.x + 1, tile.coord.y};
    source.local = 82;
    source.jump = 34;
    source.pos = 0;

    tile.cb.local.local |= bit(source.local);
    tile.cb.src.jump |= bit(source.jump);
    fpga::releaseRouteFragmentLease(std::vector<fpga::Wire>{source}, 0);

    // Check: releasing a source fragment frees both the local takeoff and the outgoing source.
    require(!isSet(tile.cb.local.local, source.local), "source fragment release did not clear local lease");
    require(!isSet(tile.cb.src.jump, source.jump), "source fragment release did not clear source lease");

    fpga::Wire transit;
    transit.type = fpga::Wire::WIRE_CROSSBAR;
    transit.from = tile.coord;
    transit.to = {tile.coord.x + 1, tile.coord.y};
    transit.local = 91;
    transit.jump = 48;
    transit.joint = 17;
    transit.pos = 2;

    tile.cb.dst.jump |= bit(transit.local);
    tile.cb.src.jump |= bit(transit.jump);
    tile.cb.joint.jump |= bit(transit.joint);
    tile.cb.src_deadend.jump |= bit(transit.jump);
    fpga::releaseRouteFragmentLease(std::vector<fpga::Wire>{transit}, 0);

    // Check: releasing a transit/fork fragment frees incoming dst, outgoing src, and joint leases.
    require(!isSet(tile.cb.dst.jump, transit.local), "transit fragment release left stale dst lease");
    require(!isSet(tile.cb.src.jump, transit.jump), "transit fragment release did not clear source lease");
    require(!isSet(tile.cb.joint.jump, transit.joint), "transit fragment release did not clear joint lease");
    // Check: release still preserves sticky routing deadend marks.
    require(isSet(tile.cb.src_deadend.jump, transit.jump), "transit release cleared sticky source deadend mark");
}

void preempted_transit_unroute_keeps_deadend_for_same_src()
{
    fpga::Tile& tile = resetDevice();
    TestRoute transit;
    transit.transit = true;
    transit.exit = 45;
    transit.local = 91;
    addRoute(tile, transit, "deadend_preempted_transit");
    tile.cb.src_deadend.jump |= bit(transit.exit);

    rtl::Net* victim = fpga::findNetByNode(tile, fpga::CB_NODE_SRC, transit.exit, true);
    require(victim != nullptr, "deadend preemption test did not find transit victim");
    require(fpga::unrouteNet(*victim), "deadend preemption test failed to unroute victim");

    // Check: preempting a transit route frees the lease but keeps the sticky deadend on its exit.
    require(!isSet(tile.cb.src.jump, transit.exit), "preempted transit source lease was not cleared");
    require(isSet(tile.cb.src_deadend.jump, transit.exit), "preempted transit source deadend was incorrectly cleared");
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
    tile_type.name = "GENERIC_ROUTE_TEST";
    tile_type.num = 1;
    tile_type.sites.clear();
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE0", .type = "LOGIC", .pos = 0});
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE1", .type = "LOGIC", .pos = 1});
    tile_type.elements.clear();
    tile_type.elements.push_back(makeElement("LUT5_0", fpga::ELEMENT_LUT5, 0));
    tile_type.elements.push_back(makeElement("LUT5_1", fpga::ELEMENT_LUT5, 1));
    tile_type.elements.push_back(makeElement("MUXF7_0", fpga::ELEMENT_MUXF7, 0));
    connectElements(tile_type, fpga::ELEMENT_LUT5, 0, fpga::ELEMENT_MUXF7, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT5, 1, fpga::ELEMENT_MUXF7, 0);
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

void connected_mux_inputs_are_void_when_packed_by_element_rules()
{
    fpga::TileType tile_type{"GENERIC_ROUTE_TEST", 1};
    fpga::Tile& tile = resetMuxPlacementTile(tile_type);
    MuxPlacementFixture fixture;

    std::vector<rtl::Net*> mux0_nets;
    LeafMuxShape mux0 = makeLeafMux(fixture, "mux0", mux0_nets);

    int lut0_pos = tile.tryAdd(mux0.lut0);
    int lut1_pos = tile.tryAdd(mux0.lut1);
    rtl::Inst route_owner;
    route_owner.wires.emplace_back();
    fpga::Wire preexisting_route;
    preexisting_route.type = fpga::Wire::WIRE_CROSSBAR;
    preexisting_route.from = tile.coord;
    preexisting_route.to = tile.coord;
    preexisting_route.local = 11;
    preexisting_route.pos = 0;
    preexisting_route.jump = 27;
    route_owner.wires.back().push_back(preexisting_route);
    tile.cb.local.local |= bit(11);
    tile.cb.src.jump |= bit(27);
    fpga::attachNetRoute(*mux0_nets.front(), route_owner, 0, mux0.lut0, mux0.mux, "O", "I0", "preexisting_internal_route");
    fpga::registerNetRouteTiles(*mux0_nets.front(), route_owner.wires.back());
    require(!route_owner.wires.back().empty() && tile.routedNets.size() == 1,
        "test setup failed to register a preexisting routed internal net");

    int mux_pos = tile.tryAdd(mux0.mux);
    require(lut0_pos >= 0 && lut1_pos >= 0, "connected mux LUT drivers could not be placed");
    require(mux_pos >= 0, "connected mux could not be placed next to its LUT drivers");
    require(mux0.lut0->tile.peer && mux0.lut1->tile.peer && mux0.mux->tile.peer,
        "connected mux shape was not fully assigned to the tile");
    for (rtl::Net* net : mux0_nets) {
        require(net && net->void_net, "first packed mux input net was not marked void");
    }
    require(route_owner.wires.back().empty(), "voiding a packed mux input did not clear its old route");
    require(!isSet(tile.cb.local.local, 11) && !isSet(tile.cb.src.jump, 27),
        "voiding a packed mux input did not release its old route leases");
    require(std::none_of(tile.routedNets.begin(), tile.routedNets.end(), [&](const Ref<rtl::Net>& ref) {
        return ref.peer == mux0_nets.front();
    }), "voiding a packed mux input left a stale routed net reference in the tile");
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

    const std::vector<uint16_t>* local_srcs = type.srcNodes(fpga::CB_NODE_LOCAL, 7);
    require(local_srcs != nullptr, "local->joint->src path was not indexed as local outgoing src");
    require(std::find(local_srcs->begin(), local_srcs->end(), 42) != local_srcs->end(),
        "local->joint->src index missed source node");

    const std::vector<uint16_t>* dst_srcs = type.srcNodes(fpga::CB_NODE_DST, 9);
    require(dst_srcs != nullptr, "dst->joint->joint->src path was not indexed as dst outgoing src");
    require(std::find(dst_srcs->begin(), dst_srcs->end(), 43) != dst_srcs->end(),
        "dst->joint->joint->src index missed source node");
}

void can_in_rejects_unconnected_double_joint_paths()
{
    fpga::CBType type{};
    int joint = -1;
    for (int i = 0; i < CB_MAX_NODES; ++i) {
        type.dst_local[i].local = NodeMask{};
        type.dst_joint[i].joint = NodeMask{};
        type.joint_local[i].local = NodeMask{};
        type.joint_joint[i].joint = NodeMask{};
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
    require(type.local_input_nodes != NodeMask{} && type.local_output_nodes != NodeMask{},
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
        NodeMask nodes = tile_type.pin_map.getNodes(type, port, pos);
        return nodes.firstSetBit();
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
    require(new_tile.cb.src.jump != NodeMask{}, "moving mode did not lease any exit on the new tile");
}

bool tileIsBlocked(const fpga::Tile& tile)
{
    return isSet(tile.cb.src.jump, 1);
}

std::vector<fpga::Coord> reconstructSyntheticPath(const std::vector<int>& parent,
                                                  const std::vector<fpga::Coord>& nodes,
                                                  int end)
{
    std::vector<fpga::Coord> path;
    for (int index = end; index >= 0; index = parent[static_cast<size_t>(index)]) {
        path.push_back(nodes[static_cast<size_t>(index)]);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<fpga::Coord> findLimitedSyntheticRouteChunk(fpga::Coord start, fpga::Coord dst,
                                                        int width, int height, int depth_limit,
                                                        const std::set<std::pair<int, int>>& route_seen)
{
    auto distance = [](fpga::Coord a, fpga::Coord b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };
    auto inside = [&](fpga::Coord coord) {
        return coord.x >= 0 && coord.y >= 0 && coord.x < width && coord.y < height;
    };

    std::vector<fpga::Coord> nodes{start};
    std::vector<int> parent{-1};
    std::vector<int> depth{0};
    std::vector<size_t> queue{0};
    std::set<std::pair<int, int>> seen = route_seen;
    seen.erase({start.x, start.y});
    seen.insert({start.x, start.y});
    int best = 0;
    int fallback = 0;

    for (size_t head = 0; head < queue.size(); ++head) {
        int current = static_cast<int>(queue[head]);
        fpga::Coord coord = nodes[static_cast<size_t>(current)];
        if (coord.x == dst.x && coord.y == dst.y) {
            best = current;
            break;
        }
        if (distance(coord, dst) < distance(nodes[static_cast<size_t>(best)], dst)) {
            best = current;
        }
        if (depth[static_cast<size_t>(current)] > depth[static_cast<size_t>(fallback)]) {
            fallback = current;
        }
        if (depth[static_cast<size_t>(current)] >= depth_limit) {
            continue;
        }

        std::vector<fpga::Coord> next = {
            {coord.x + 1, coord.y},
            {coord.x, coord.y - 1},
            {coord.x, coord.y + 1},
            {coord.x - 1, coord.y},
        };
        std::stable_sort(next.begin(), next.end(), [&](fpga::Coord a, fpga::Coord b) {
            return distance(a, dst) < distance(b, dst);
        });
        for (fpga::Coord candidate : next) {
            if (!inside(candidate) || seen.contains({candidate.x, candidate.y})) {
                continue;
            }
            fpga::Tile* tile = fpga::Device::current().getTile(candidate.x, candidate.y);
            if (!tile || tileIsBlocked(*tile)) {
                continue;
            }
            seen.insert({candidate.x, candidate.y});
            nodes.push_back(candidate);
            parent.push_back(current);
            depth.push_back(depth[static_cast<size_t>(current)] + 1);
            queue.push_back(nodes.size() - 1);
        }
    }

    if (best == 0) {
        best = fallback;
    }
    require(best != 0, "limited synthetic router found no usable path fragment");
    return reconstructSyntheticPath(parent, nodes, best);
}

void limited_iterations_find_one_tile_escape_path_behind_source()
{
    constexpr int width = 13;
    constexpr int height = 11;
    fpga::Coord src{5, 5};
    fpga::Coord dst{11, 5};
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(width, height);

    std::set<std::pair<int, int>> free_trail;
    for (int x = 0; x <= src.x; ++x) {
        free_trail.insert({x, src.y});
    }
    for (int y = 0; y <= src.y; ++y) {
        free_trail.insert({0, y});
    }
    for (int x = 0; x <= dst.x; ++x) {
        free_trail.insert({x, 0});
    }
    for (int y = 0; y <= dst.y; ++y) {
        free_trail.insert({dst.x, y});
    }
    free_trail.insert({dst.x, dst.y});

    for (fpga::Tile* tile : tiles) {
        int dx = std::abs(tile->coord.x - src.x);
        int dy = std::abs(tile->coord.y - src.y);
        bool in_filled_radius = dx <= 5 && dy <= 5;
        bool is_free = free_trail.contains({tile->coord.x, tile->coord.y});
        if (in_filled_radius && !is_free) {
            tile->cb.src.jump |= bit(1);
            tile->cb.dst.jump |= bit(1);
            tile->cb.local.local |= bit(1);
        }
    }

    // Check: the direct direction to the destination is blocked, forcing the first useful step backwards.
    require(tileIsBlocked(*fpga::Device::current().getTile(src.x + 1, src.y)),
        "escape test setup did not block the direct source-to-destination direction");
    // Check: the only immediate escape from the source points opposite the destination direction.
    require(!tileIsBlocked(*fpga::Device::current().getTile(src.x - 1, src.y)),
        "escape test setup accidentally blocked the one-tile-wide backward trail");

    fpga::Coord current = src;
    std::vector<fpga::Coord> route{src};
    std::set<std::pair<int, int>> route_seen{{src.x, src.y}};
    for (int pass = 0; pass < 8 && !(current.x == dst.x && current.y == dst.y); ++pass) {
        std::vector<fpga::Coord> chunk = findLimitedSyntheticRouteChunk(current, dst, width, height, 5, route_seen);
        require(chunk.size() > 1, "limited synthetic router did not advance");
        route.insert(route.end(), std::next(chunk.begin()), chunk.end());
        for (fpga::Coord coord : chunk) {
            route_seen.insert({coord.x, coord.y});
        }
        current = route.back();
    }

    // Check: limited-depth routing eventually escapes the blocked source region and reaches the sink.
    require(current.x == dst.x && current.y == dst.y,
        "limited synthetic router did not find the one-tile-wide escape path in several iterations");
    // Check: the route starts by moving opposite the destination direction before going around the blocked region.
    require(route.size() > 1 && route[1].x == src.x - 1 && route[1].y == src.y,
        "limited synthetic router did not take the required backward first step");
    // Check: the route uses only the deliberately preserved one-tile-wide trail through the filled radius.
    for (fpga::Coord coord : route) {
        require(free_trail.contains({coord.x, coord.y}),
            "limited synthetic router left the one-tile-wide free trail");
    }
}

void limited_continuation_is_strictly_incremental_without_rollbacks()
{
    constexpr int width = 15;
    constexpr int height = 9;
    constexpr int depth_limit = 3;
    fpga::Coord src{7, 4};
    fpga::Coord dst{14, 4};
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(width, height);

    std::set<std::pair<int, int>> free_trail;
    for (int x = src.x; x >= 1; --x) {
        free_trail.insert({x, src.y});
    }
    for (int y = src.y; y >= 1; --y) {
        free_trail.insert({1, y});
    }
    for (int x = 1; x <= dst.x; ++x) {
        free_trail.insert({x, 1});
    }
    for (int y = 1; y <= dst.y; ++y) {
        free_trail.insert({dst.x, y});
    }
    free_trail.insert({dst.x, dst.y});

    for (fpga::Tile* tile : tiles) {
        bool is_free = free_trail.contains({tile->coord.x, tile->coord.y});
        if (!is_free) {
            tile->cb.src.jump |= bit(1);
            tile->cb.dst.jump |= bit(1);
            tile->cb.local.local |= bit(1);
        }
    }

    // Check: the obstacle model blocks the direct route and leaves only a narrow detour.
    require(tileIsBlocked(*fpga::Device::current().getTile(src.x + 1, src.y)),
        "incremental continuation setup did not block the direct route");
    require(!tileIsBlocked(*fpga::Device::current().getTile(src.x - 1, src.y)),
        "incremental continuation setup blocked the required first detour tile");

    fpga::Coord current = src;
    std::vector<fpga::Coord> route{src};
    std::set<std::pair<int, int>> route_seen{{src.x, src.y}};
    for (int pass = 0; pass < 16 && !(current.x == dst.x && current.y == dst.y); ++pass) {
        std::vector<fpga::Coord> before = route;
        std::vector<fpga::Coord> chunk = findLimitedSyntheticRouteChunk(current, dst, width, height,
            depth_limit, route_seen);

        // Check: each limited pass must find at least one committed forward fragment.
        require(chunk.size() > 1, "incremental continuation pass did not produce a commit fragment");

        route.insert(route.end(), std::next(chunk.begin()), chunk.end());
        for (fpga::Coord coord : chunk) {
            route_seen.insert({coord.x, coord.y});
        }
        current = route.back();

        // Check: previous prefix is preserved exactly; no committed coordinate may be erased or changed.
        require(route.size() > before.size(), "incremental continuation did not grow the route");
        require(std::equal(before.begin(), before.end(), route.begin()),
            "incremental continuation changed an already committed prefix");
        // Check: incremental growth must not end by returning to an already committed endpoint.
        std::set<std::pair<int, int>> old_points;
        for (fpga::Coord coord : before) {
            old_points.insert({coord.x, coord.y});
        }
        require(!old_points.contains({route.back().x, route.back().y}),
            "incremental continuation ended at an already committed endpoint");
        // Check: the newly committed suffix stays on the deliberately free trail.
        for (auto it = route.begin() + static_cast<std::ptrdiff_t>(before.size()); it != route.end(); ++it) {
            require(free_trail.contains({it->x, it->y}),
                "incremental continuation committed a tile outside the free trail");
        }
        // Check: bounded passes are really partial before the final pass, so this covers continuation behavior.
        if (current.x != dst.x || current.y != dst.y) {
            require(chunk.size() <= static_cast<size_t>(depth_limit + 1),
                "incremental continuation unexpectedly completed in one unbounded pass");
        }
    }

    // Check: repeated incremental commits eventually follow the detour to the destination.
    require(current.x == dst.x && current.y == dst.y,
        "incremental continuation did not reach the target through repeated committed partial routes");
    // Check: the first committed move goes away from the destination; this prevents distance-only rollback policy.
    require(route.size() > 1 && route[1].x == src.x - 1 && route[1].y == src.y,
        "incremental continuation did not preserve the required backward first step");
}

void unroute_net_clears_multifragment_route_state()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(3, 1);
    fpga::Tile& source = *tiles[0];
    fpga::Tile& transit = *tiles[1];
    fpga::Tile& sink = *tiles[2];

    Referable<rtl::Net> net;
    net.name = "multifragment_unroute";
    rtl::Inst owner;
    owner.wires.emplace_back();
    std::vector<fpga::Wire>& route = owner.wires.back();

    fpga::Wire source_fragment;
    source_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    source_fragment.from = source.coord;
    source_fragment.to = transit.coord;
    source_fragment.local = 64;
    source_fragment.jump = 10;
    source_fragment.pos = 0;
    source_fragment.net_name = net.name;
    route.push_back(source_fragment);

    fpga::Wire transit_fragment;
    transit_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    transit_fragment.from = transit.coord;
    transit_fragment.to = sink.coord;
    transit_fragment.local = 20;
    transit_fragment.jump = 11;
    transit_fragment.joint = 5;
    transit_fragment.pos = 1;
    transit_fragment.net_name = net.name;
    route.push_back(transit_fragment);

    fpga::Wire sink_fragment;
    sink_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    sink_fragment.from = sink.coord;
    sink_fragment.to = sink.coord;
    sink_fragment.local = 21;
    sink_fragment.jump = -1;
    sink_fragment.joint = 6;
    sink_fragment.pos = 1;
    sink_fragment.net_name = net.name;
    route.push_back(sink_fragment);

    fpga::Wire pin_fragment;
    pin_fragment.type = fpga::Wire::WIRE_TILE_PIN;
    pin_fragment.from = sink.coord;
    pin_fragment.to = sink.coord;
    pin_fragment.local = 70;
    pin_fragment.pos = 2;
    pin_fragment.net_name = net.name;
    route.push_back(pin_fragment);

    source.cb.local.local |= bit(source_fragment.local);
    source.cb.src.jump |= bit(source_fragment.jump);
    transit.cb.dst.jump |= bit(transit_fragment.local);
    transit.cb.src.jump |= bit(transit_fragment.jump);
    transit.cb.joint.jump |= bit(transit_fragment.joint);
    sink.cb.dst.jump |= bit(sink_fragment.local);
    sink.cb.joint.jump |= bit(sink_fragment.joint);
    sink.cb.local.local |= bit(pin_fragment.local);
    sink.pin_state.leased_nodes |= bit(pin_fragment.local);

    fpga::attachNetRoute(net, owner, 0, nullptr, &owner, {}, {}, net.name);
    fpga::registerNetRouteTiles(net, route);
    require(!source.routedNets.empty() && !transit.routedNets.empty() && !sink.routedNets.empty(),
        "multifragment unroute setup did not register route tiles");

    require(fpga::unrouteNet(net), "multifragment unroute returned false");

    // Check: all source-side leases from the first route fragment are released.
    require(!isSet(source.cb.local.local, source_fragment.local), "source local lease survived unroute");
    require(!isSet(source.cb.src.jump, source_fragment.jump), "source jump lease survived unroute");
    // Check: all transit dst/src/joint leases are released.
    require(!isSet(transit.cb.dst.jump, transit_fragment.local), "transit dst lease survived unroute");
    require(!isSet(transit.cb.src.jump, transit_fragment.jump), "transit src lease survived unroute");
    require(!isSet(transit.cb.joint.jump, transit_fragment.joint), "transit joint lease survived unroute");
    // Check: final entry and resource pin leases are released together.
    require(!isSet(sink.cb.dst.jump, sink_fragment.local), "sink dst lease survived unroute");
    require(!isSet(sink.cb.joint.jump, sink_fragment.joint), "sink joint lease survived unroute");
    require(!isSet(sink.cb.local.local, pin_fragment.local), "sink local pin lease survived unroute");
    require(!isSet(sink.pin_state.leased_nodes, pin_fragment.local), "sink pin_state lease survived unroute");
    // Check: route storage and per-tile net references are empty after atomic net unroute.
    require(route.empty(), "owner route vector was not cleared by unroute");
    require(std::all_of(source.routedNets.begin(), source.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "source routedNets kept unrouted net");
    require(std::all_of(transit.routedNets.begin(), transit.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "transit routedNets kept unrouted net");
    require(std::all_of(sink.routedNets.begin(), sink.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "sink routedNets kept unrouted net");
}

void grounding_preemption_route_tree_unroute_frees_terminal_masks()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(3, 2);
    fpga::Tile& source = *tiles[0];
    fpga::Tile& transit0 = *tiles[1];
    fpga::Tile& target0 = *tiles[2];
    fpga::Tile& transit1 = *tiles[4];
    fpga::Tile& target1 = *tiles[5];

    Referable<rtl::Net> net;
    net.name = "grounding_preemption_tree";
    rtl::Inst driver;
    rtl::Inst sink0;
    rtl::Inst sink1;
    driver.wires.resize(2);

    auto add_source_fragment = [&](std::vector<fpga::Wire>& route, int local, int src_bit,
                                   fpga::Coord to, const std::string& name) {
        fpga::Wire fragment;
        fragment.type = fpga::Wire::WIRE_CROSSBAR;
        fragment.from = source.coord;
        fragment.to = to;
        fragment.local = local;
        fragment.jump = src_bit;
        fragment.pos = 0;
        fragment.net_name = name;
        route.push_back(fragment);
        source.cb.local.local |= bit(local);
        source.cb.src.jump |= bit(src_bit);
    };
    auto add_transit_fragment = [](std::vector<fpga::Wire>& route, fpga::Tile& tile, fpga::Coord to,
                                   int dst_bit, int src_bit, int joint_bit, const std::string& name) {
        fpga::Wire fragment;
        fragment.type = fpga::Wire::WIRE_CROSSBAR;
        fragment.from = tile.coord;
        fragment.to = to;
        fragment.local = dst_bit;
        fragment.jump = src_bit;
        fragment.joint = joint_bit;
        fragment.pos = 1;
        fragment.net_name = name;
        route.push_back(fragment);
        tile.cb.dst.jump |= bit(dst_bit);
        tile.cb.src.jump |= bit(src_bit);
        tile.cb.joint.jump |= bit(joint_bit);
    };
    auto add_terminal_fragment = [](std::vector<fpga::Wire>& route, fpga::Tile& tile,
                                    int dst_bit, int joint_bit, int pin_bit, const std::string& name) {
        fpga::Wire entry;
        entry.type = fpga::Wire::WIRE_CROSSBAR;
        entry.from = tile.coord;
        entry.to = tile.coord;
        entry.local = dst_bit;
        entry.jump = -1;
        entry.joint = joint_bit;
        entry.pos = 1;
        entry.net_name = name;
        route.push_back(entry);

        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = tile.coord;
        pin.to = tile.coord;
        pin.local = pin_bit;
        pin.pos = 2;
        pin.net_name = name;
        route.push_back(pin);

        tile.cb.dst.jump |= bit(dst_bit);
        tile.cb.joint.jump |= bit(joint_bit);
        tile.cb.local.local |= bit(pin_bit);
        tile.pin_state.leased_nodes |= bit(pin_bit);
    };

    std::vector<fpga::Wire>& route0 = driver.wires[0];
    std::vector<fpga::Wire>& route1 = driver.wires[1];
    add_source_fragment(route0, 80, 12, transit0.coord, net.name);
    add_transit_fragment(route0, transit0, target0.coord, 30, 13, 5, net.name);
    add_terminal_fragment(route0, target0, 31, 6, 90, net.name);

    add_source_fragment(route1, 81, 14, transit1.coord, net.name);
    add_transit_fragment(route1, transit1, target1.coord, 32, 15, 7, net.name);
    add_terminal_fragment(route1, target1, 33, 8, 91, net.name);

    source.cb.src_deadend.jump |= bit(12);
    source.cb.src_deadend.jump |= bit(14);

    fpga::attachNetRoute(net, driver, 0, &driver, &sink0, "O", "I", "grounding_preemption_tree_0");
    fpga::attachNetRoute(net, driver, 1, &driver, &sink1, "O", "I", "grounding_preemption_tree_1");
    fpga::registerNetRouteTiles(net, route0);
    fpga::registerNetRouteTiles(net, route1);
    require(net.routes.size() == 2, "grounding preemption tree setup did not register two bindings");
    require(!source.routedNets.empty() && !target0.routedNets.empty() && !target1.routedNets.empty(),
        "grounding preemption tree setup did not register route tiles");

    require(fpga::unrouteNetRouteTree(net, {0, 1}), "grounding preemption route-tree unroute returned false");

    // Check: source-tree unroute frees both source locals and both outgoing takeoff nodes.
    require(!isSet(source.cb.local.local, 80), "grounding preemption left first source local leased");
    require(!isSet(source.cb.local.local, 81), "grounding preemption left second source local leased");
    require(!isSet(source.cb.src.jump, 12), "grounding preemption left first source exit leased");
    require(!isSet(source.cb.src.jump, 14), "grounding preemption left second source exit leased");
    // Check: sticky deadend learning remains after preemption cleanup.
    require(isSet(source.cb.src_deadend.jump, 12), "grounding preemption cleared first sticky deadend");
    require(isSet(source.cb.src_deadend.jump, 14), "grounding preemption cleared second sticky deadend");
    // Check: transit nodes from every preempted branch are fully freed.
    require(!isSet(transit0.cb.dst.jump, 30), "grounding preemption left first transit dst leased");
    require(!isSet(transit0.cb.src.jump, 13), "grounding preemption left first transit src leased");
    require(!isSet(transit0.cb.joint.jump, 5), "grounding preemption left first transit joint leased");
    require(!isSet(transit1.cb.dst.jump, 32), "grounding preemption left second transit dst leased");
    require(!isSet(transit1.cb.src.jump, 15), "grounding preemption left second transit src leased");
    require(!isSet(transit1.cb.joint.jump, 7), "grounding preemption left second transit joint leased");
    // Check: final destination entries and resource pin leases are released for grounding preemption victims.
    require(!isSet(target0.cb.dst.jump, 31), "grounding preemption left first target dst leased");
    require(!isSet(target0.cb.joint.jump, 6), "grounding preemption left first target joint leased");
    require(!isSet(target0.cb.local.local, 90), "grounding preemption left first target pin local leased");
    require(!isSet(target0.pin_state.leased_nodes, 90), "grounding preemption left first target pin_state leased");
    require(!isSet(target1.cb.dst.jump, 33), "grounding preemption left second target dst leased");
    require(!isSet(target1.cb.joint.jump, 8), "grounding preemption left second target joint leased");
    require(!isSet(target1.cb.local.local, 91), "grounding preemption left second target pin local leased");
    require(!isSet(target1.pin_state.leased_nodes, 91), "grounding preemption left second target pin_state leased");
    // Check: selected source-tree route vectors and tile route references are removed atomically.
    require(route0.empty() && route1.empty(), "grounding preemption route tree did not clear route vectors");
    require(std::all_of(source.routedNets.begin(), source.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left source routedNets reference");
    require(std::all_of(transit0.routedNets.begin(), transit0.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left first transit routedNets reference");
    require(std::all_of(transit1.routedNets.begin(), transit1.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left second transit routedNets reference");
    require(std::all_of(target0.routedNets.begin(), target0.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left first target routedNets reference");
    require(std::all_of(target1.routedNets.begin(), target1.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left second target routedNets reference");
}

}

int main()
{
    try {
        joint_mediated_src_nodes_are_indexed();
        can_in_rejects_unconnected_double_joint_paths();
        loaded_crossbar_local_and_joint_masks_use_router_bit_numbering();
        tile_type_mapping_models_all_16_ff_input_pins_per_clb_tile();
        connected_mux_inputs_are_void_when_packed_by_element_rules();
        routing_mode_generic_routes_only_one_net_from_single_source_port();
        routing_mode_fanout_branches_away_from_source_tile();
        routing_mode_moving_unroutes_old_cell_tree_and_reroutes_hierarchy();
        limited_iterations_find_one_tile_escape_path_behind_source();
        limited_continuation_is_strictly_incremental_without_rollbacks();
        unroute_net_clears_multifragment_route_state();
        grounding_preemption_route_tree_unroute_frees_terminal_masks();
        releasing_route_fragment_keeps_deadend_for_same_src();
        releasing_route_fragment_clears_source_and_transit_node_classes();
        preempted_transit_unroute_keeps_deadend_for_same_src();
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
