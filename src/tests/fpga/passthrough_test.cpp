#include "Device.h"
#include "Tile.h"
#include "Wire.h"
#include "RegBunch.h"
#include "Timings.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "passthrough_test failed: " << message << "\n";
        std::exit(1);
    }
}

constexpr uint16_t bit16(int bit)
{
    return static_cast<uint16_t>(1u << bit);
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
            element.right_blockers[right_bit] |= bit16(left_bit);
        }
        if (element.type == right_type && element.bitmap_pos == right_bit) {
            element.left_blockers[left_bit] |= bit16(right_bit);
        }
    }
}

fpga::TileType makePassthroughTileType()
{
    fpga::TileType tile_type{"PASSTHROUGH_TEST", 1, 0};
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE0", .type = "LOGIC", .pos = 0});
    tile_type.elements.push_back(makeElement("LUT5", fpga::ELEMENT_LUT5, 0));
    tile_type.elements.push_back(makeElement("LUT5_1", fpga::ELEMENT_LUT5, 1));
    tile_type.elements.push_back(makeElement("LUT1", fpga::ELEMENT_LUT1, 0));
    tile_type.elements.push_back(makeElement("LUT1_1", fpga::ELEMENT_LUT1, 1));
    tile_type.elements.push_back(makeElement("MUXF7", fpga::ELEMENT_MUXF7, 0));
    tile_type.elements.push_back(makeElement("MUXF8", fpga::ELEMENT_MUXF8, 0));
    tile_type.elements.push_back(makeElement("FD", fpga::ELEMENT_FD, 0));
    connectElements(tile_type, fpga::ELEMENT_LUT5, 0, fpga::ELEMENT_LUT1, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT1, 0, fpga::ELEMENT_MUXF7, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT1, 1, fpga::ELEMENT_MUXF7, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT5, 0, fpga::ELEMENT_MUXF7, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT5, 1, fpga::ELEMENT_MUXF7, 0);
    connectElements(tile_type, fpga::ELEMENT_MUXF7, 0, fpga::ELEMENT_MUXF8, 0);
    connectElements(tile_type, fpga::ELEMENT_MUXF8, 0, fpga::ELEMENT_FD, 0);
    return tile_type;
}

int posFor(fpga::ElementType type)
{
    switch (type) {
    case fpga::ELEMENT_FD: return 0;
    case fpga::ELEMENT_LUT5:
    case fpga::ELEMENT_LUT1: return 3;
    case fpga::ELEMENT_MUXF7:
    case fpga::ELEMENT_MUXF8: return 1;
    default: return 0;
    }
}

struct Fixture
{
    Referable<rtl::Module> parent_module;
    Referable<rtl::Module> primitive_module;
    Referable<rtl::Cell> top_cell;
    Referable<rtl::Inst> top;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    int next_designator = 1;

    Fixture()
    {
        parent_module.name = "top_module";
        parent_module.is_blackbox = false;
        primitive_module.name = "primitive";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&parent_module);
        top_cell.name = "top";
        top_cell.type = "top";
        top_cell.module_ref.set(&parent_module);
        top.cell_ref.set(&top_cell);
    }

    Referable<rtl::Cell>* makeCell(const std::string& name, const std::string& type,
                                   const std::vector<std::pair<std::string, int>>& ports)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = type;
        cell->module_ref.set(&primitive_module);
        for (const auto& [port_name, port_type] : ports) {
            rtl::Port port;
            port.name = port_name;
            port.type = static_cast<decltype(port.type)>(port_type);
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
        inst->parent_ref.set(&top);
        inst->pos = -1;
        for (auto& port : inst->cell_ref->ports) {
            auto& conn = inst->conns.emplace_back();
            conn.port_ref.set(&port);
            conn.inst_ref.set(inst.get());
        }
        Referable<rtl::Inst>* raw = inst.get();
        insts.push_back(std::move(inst));
        return raw;
    }

    rtl::Conn* conn(Referable<rtl::Inst>* inst, const std::string& port_name)
    {
        for (auto& conn_ref : inst->conns) {
            if (conn_ref.port_ref.peer && conn_ref.port_ref->name == port_name) {
                return &conn_ref;
            }
        }
        return nullptr;
    }

    rtl::Net* connect(Referable<rtl::Inst>* driver, const std::string& driver_port,
                      Referable<rtl::Inst>* sink, const std::string& sink_port)
    {
        rtl::Conn* out = conn(driver, driver_port);
        rtl::Conn* in = conn(sink, sink_port);
        require(out && in, "connection references missing ports");
        int designator = next_designator++;
        out->port_ref->designator = designator;
        in->port_ref->designator = designator;
        in->set(&rtl::Conn::fromBase(*out));
        auto& net = parent_module.nets.emplace_back();
        net.name = "n" + std::to_string(designator);
        net.designators.push_back(designator);
        return &net;
    }
};

std::vector<std::pair<std::string, int>> portsFor(fpga::ElementType type)
{
    if (type == fpga::ELEMENT_FD) {
        return {{"D", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}};
    }
    return {{"I0", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}};
}

std::string typeName(fpga::ElementType type)
{
    switch (type) {
    case fpga::ELEMENT_LUT5: return "LUT5";
    case fpga::ELEMENT_LUT1: return "LUT1";
    case fpga::ELEMENT_MUXF7: return "MUXF7";
    case fpga::ELEMENT_MUXF8: return "MUXF8";
    case fpga::ELEMENT_FD: return "FDRE";
    default: return "CELL";
    }
}

void resetOneTileDevice(fpga::TileType& tile_type)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.tile_grid.resize(1);
    device.size_width = 1;
    device.size_height = 1;
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = {0, 0};
    tile.name = {0, 0};
    tile.tile_type = &tile_type;
    tile.elements_initialized = false;
}

void source_passthrough_cases()
{
    for (fpga::ElementType type : {fpga::ELEMENT_LUT5, fpga::ELEMENT_LUT1, fpga::ELEMENT_MUXF7,
             fpga::ELEMENT_MUXF8, fpga::ELEMENT_FD}) {
        fpga::TileType tile_type = makePassthroughTileType();
        resetOneTileDevice(tile_type);
        fpga::Tile& tile = fpga::Device::current().tile_grid.front();
        Fixture fixture;
        fixture.parent_module.nets.reserve(32);

        auto* source = fixture.makeInst("source", typeName(type), portsFor(type));
        auto* sink = fixture.makeInst("sink", "FDRE", portsFor(fpga::ELEMENT_FD));
        source->pos = posFor(type);
        source->coord = tile.coord;
        tile.assign(source);
        rtl::Net* net = fixture.connect(source, type == fpga::ELEMENT_FD ? "Q" : "O", sink, "D");

        rtl::Inst* from = source;
        std::string from_port = type == fpga::ELEMENT_FD ? "Q" : "O";
        rtl::Inst* to = sink;
        std::string to_port = "D";
        bool changed = fpga::preparePassthroughRouteEndpoints(from, from_port, to, to_port, net);
        require(!sink->cell_ref->attributes.contains("scalepnr_passthrough"),
            "passthrough lookup tagged an ordinary sink as generated");
        bool expected = type != fpga::ELEMENT_MUXF8 && type != fpga::ELEMENT_FD;
        require(changed == expected, "unexpected source passthrough decision for " + typeName(type));
        if (expected) {
            require(from != source, "source passthrough did not replace route source for " + typeName(type));
            require(from->tile.peer == &tile, "source passthrough was not placed in source tile");
            require(net && !net->void_net, "source passthrough lost the routable net");
            bool has_void = std::any_of(fixture.parent_module.nets.begin(), fixture.parent_module.nets.end(),
                [](const rtl::Net& candidate) { return candidate.void_net; });
            require(has_void, "source passthrough did not create a void internal net");
        }
    }
}

void source_passthrough_invalidates_sink_joint_reservations()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    fpga::CBType cb_type{"CACHE_ROUTE"};
    tile.cb_type = &cb_type;
    tile.cb.type = &cb_type;
    Fixture fixture;
    fixture.parent_module.nets.reserve(16);

    auto* source = fixture.makeInst(
        "cache_source", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    auto* sink = fixture.makeInst(
        "cache_sink", "FDRE", portsFor(fpga::ELEMENT_FD));
    source->pos = posFor(fpga::ELEMENT_LUT5);
    source->coord = tile.coord;
    tile.assign(source);
    sink->pos = posFor(fpga::ELEMENT_FD);
    sink->coord = tile.coord;
    tile.assign(sink);
    rtl::Net* net = fixture.connect(source, "O", sink, "D");
    rtl::Conn* original_driver = fixture.conn(source, "O");

    // Prime the driver-keyed cache before endpoint preparation rewires D to a
    // generated source passthrough, reproducing the stale self-reservation.
    tile.input_joint_reservations = {{original_driver, NodeMask{0, 1} << 26}};
    tile.input_joint_reservations_initialized = true;
    rtl::Inst* from = source;
    rtl::Inst* to = sink;
    std::string from_port = "O";
    std::string to_port = "D";
    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, net);

    // Every rewired sink must invalidate both the cached owner pointers and
    // their mandatory masks before terminal routing excludes reserved joints.
    require(changed && from != source,
        "source passthrough did not rewire the cached sink");
    require(!tile.input_joint_reservations_initialized
            && tile.input_joint_reservations.empty(),
        "source passthrough retained stale input-joint reservations");
    rtl::Conn* current_driver = fixture.conn(sink, "D")->follow();
    require(current_driver && current_driver->inst_ref.peer == from
            && current_driver->port_ref.peer
            && current_driver->port_ref->makeName() == from_port,
        "source passthrough did not become the sink's physical driver");
    tile.cb.type = nullptr;
    tile.cb_type = nullptr;
}

void forced_fabric_output_keeps_the_original_route_endpoint()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(8);

    auto* source = fixture.makeInst("fabric_source", "LUT2", portsFor(fpga::ELEMENT_LUT5));
    auto* sink = fixture.makeInst("fabric_sink", "FDRE", portsFor(fpga::ELEMENT_FD));
    source->cell_ref->attributes["scalepnr_force_fabric_output"] = "1";
    source->pos = posFor(fpga::ELEMENT_LUT5);
    source->coord = tile.coord;
    tile.assign(source);
    rtl::Net* net = fixture.connect(source, "O", sink, "D");

    rtl::Inst* from = source;
    std::string from_port = "O";
    rtl::Inst* to = sink;
    std::string to_port = "D";
    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, net);

    // A source with a direct fabric mapping must retain its physical identity.
    require(!changed && from == source && from_port == "O",
        "forced fabric output was replaced by a source passthrough");
    // Endpoint preparation must not manufacture an internal net for this route.
    require(fixture.parent_module.nets.size() == 1 && !net->void_net,
        "forced fabric output created a passthrough void net");
}

void empty_passthrough_attribute_is_not_generated_endpoint()
{
    Fixture fixture;
    auto* ordinary = fixture.makeInst("ordinary", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    ordinary->cell_ref->attributes["scalepnr_passthrough"] = "";
    std::string reason;

    // Check: a stale empty attribute cannot make an ordinary cell participate
    // in generated-endpoint Moving rehome behavior.
    require(!fpga::rehomeGeneratedPassthrough(*ordinary, &reason)
            && reason == "instance is not a generated endpoint",
        "empty passthrough attribute classified an ordinary cell as generated");
}

void equal_neighbor_bits_do_not_alias_element_types()
{
    fpga::TileType tile_type{"TYPED_LINK_TEST", 1, 0};
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE0", .type = "LOGIC", .pos = 0});
    tile_type.elements.push_back(makeElement("LEFT_1", fpga::ELEMENT_LUT5, 1));
    tile_type.elements.push_back(makeElement("UNLINKED_0", fpga::ELEMENT_LUT1, 0));
    tile_type.elements.push_back(makeElement("LINKED_0", fpga::ELEMENT_MUXF7, 0));
    connectElements(tile_type, fpga::ELEMENT_LUT5, 1, fpga::ELEMENT_MUXF7, 0);
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(8);

    auto* source = fixture.makeInst("source", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    auto* sink = fixture.makeInst("sink", "FDRE", portsFor(fpga::ELEMENT_FD));
    source->pos = 7;
    source->coord = tile.coord;
    tile.assign(source);
    rtl::Net* net = fixture.connect(source, "O", sink, "D");
    rtl::Inst* from = source;
    rtl::Inst* to = sink;
    std::string from_port = "O";
    std::string to_port = "D";

    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, net);

    // Check: bit zero in an unrelated column cannot masquerade as the real
    // LUT5-to-MUXF7 link merely because both destination bits are numbered zero.
    bool uses_i0 = from && std::any_of(from->conns.begin(), from->conns.end(), [](const rtl::Conn& conn) {
        return conn.port_ref.peer && conn.port_ref.peer->name == "I0";
    });
    require(changed && from != source && from->cell_ref.peer
            && from->cell_ref->type == "MUXF7" && uses_i0,
        "equal neighbor bits aliased an unconnected element type");
}

void target_passthrough_cases()
{
    for (fpga::ElementType type : {fpga::ELEMENT_LUT5, fpga::ELEMENT_LUT1, fpga::ELEMENT_MUXF7,
             fpga::ELEMENT_MUXF8, fpga::ELEMENT_FD}) {
        fpga::TileType tile_type = makePassthroughTileType();
        resetOneTileDevice(tile_type);
        fpga::Tile& tile = fpga::Device::current().tile_grid.front();
        Fixture fixture;
        fixture.parent_module.nets.reserve(32);

        auto* driver = fixture.makeInst("driver", "LUT5", portsFor(fpga::ELEMENT_LUT5));
        auto* target = fixture.makeInst("target", typeName(type), portsFor(type));
        target->pos = posFor(type);
        target->coord = tile.coord;
        tile.assign(target);
        rtl::Net* net = fixture.connect(driver, "O", target, type == fpga::ELEMENT_FD ? "D" : "I0");

        rtl::Inst* from = driver;
        std::string from_port = "O";
        rtl::Inst* to = target;
        std::string to_port = type == fpga::ELEMENT_FD ? "D" : "I0";
        bool changed = fpga::preparePassthroughRouteEndpoints(from, from_port, to, to_port, net);
        bool expected = type != fpga::ELEMENT_LUT5 && type != fpga::ELEMENT_FD;
        require(changed == expected, "unexpected target passthrough decision for " + typeName(type));
        if (expected) {
            require(to != target, "target passthrough did not replace route target for " + typeName(type));
            require(to->tile.peer == &tile, "target passthrough was not placed in target tile");
            require(net && !net->void_net, "target passthrough lost the routable net");
            bool has_void = std::any_of(fixture.parent_module.nets.begin(), fixture.parent_module.nets.end(),
                [](const rtl::Net& candidate) { return candidate.void_net; });
            require(has_void, "target passthrough did not create a void internal net");
        }
    }
}

void protected_external_net_uses_target_passthrough()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(32);

    auto* driver = fixture.makeInst(
        "static_driver", "STATIC_ONE", {{"O", rtl::Port::PORT_OUT}});
    auto* target = fixture.makeInst(
        "packed_mux", "MUXF7", portsFor(fpga::ELEMENT_MUXF7));
    target->pos = posFor(fpga::ELEMENT_MUXF7);
    target->coord = tile.coord;
    tile.assign(target);

    rtl::Conn* output = fixture.conn(driver, "O");
    rtl::Conn* input = fixture.conn(target, "I0");
    require(output && input, "protected passthrough fixture has missing ports");
    output->port_ref->designator = -1;
    input->port_ref->designator = -1;
    input->set(&rtl::Conn::fromBase(*output));

    // This infrastructure route deliberately has no ordinary module-net entry.
    // Endpoint preparation must use the explicit protected route object.
    rtl::Net protected_net;
    protected_net.name = "STATIC_NET";
    protected_net.designators.push_back(7001);
    protected_net.route_protected = true;
    rtl::Net* net = &protected_net;
    rtl::Inst* from = driver;
    rtl::Inst* to = target;
    std::string from_port = "O";
    std::string to_port = "I0";

    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, net, false);

    require(changed && to != target,
        "protected external net did not receive a target passthrough");
    require(net == &protected_net,
        "target passthrough replaced the protected route owner");
    require(to->tile.peer == &tile,
        "protected target passthrough was not packed beside its sink");
    require(input->follow() != output,
        "packed internal connection was not separated from the fabric route");
}

void passthrough_rejects_unrelated_lut_overlay()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(32);

    auto* unrelated = fixture.makeInst("unrelated_lut", "LUT3", portsFor(fpga::ELEMENT_LUT5));
    auto* unrelated_driver = fixture.makeInst("unrelated_driver", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    // MUXF7 I0 uses the upper adjacent LUT lane; occupy that exact lane with
    // an unrelated input so a generated passthrough cannot overlay it.
    unrelated->pos = 4;
    unrelated->coord = tile.coord;
    tile.assign(unrelated);
    fixture.connect(unrelated_driver, "O", unrelated, "I0");

    auto* driver = fixture.makeInst("driver", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    auto* target = fixture.makeInst("target_mux", "MUXF7", portsFor(fpga::ELEMENT_MUXF7));
    target->pos = posFor(fpga::ELEMENT_MUXF7);
    target->coord = tile.coord;
    tile.assign(target);
    rtl::Net* net = fixture.connect(driver, "O", target, "I0");

    rtl::Inst* from = driver;
    std::string from_port = "O";
    rtl::Inst* to = target;
    std::string to_port = "I0";
    bool changed = fpga::preparePassthroughRouteEndpoints(from, from_port, to, to_port, net);

    require(!changed, "unconnected LUT1 passthrough overlaid an unrelated LUT input");
    require(to == target, "target was replaced after rejected passthrough overlay");
}

void distributed_target_defers_and_rehomes_blocked_passthrough()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(32);

    auto* blocker = fixture.makeInst("blocker", "LUT3", portsFor(fpga::ELEMENT_LUT5));
    auto* blocker_driver = fixture.makeInst("blocker_driver", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    blocker->pos = 3;
    blocker->coord = tile.coord;
    tile.assign(blocker);
    fixture.connect(blocker_driver, "O", blocker, "I0");

    auto* driver = fixture.makeInst("distributed_driver", "LUT2", portsFor(fpga::ELEMENT_LUT5));
    auto* data = fixture.makeInst("packed_data", "LUT6", portsFor(fpga::ELEMENT_LUT5));
    auto* target = fixture.makeInst("packed_mux", "MUXF7",
        {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
    data->pos = 7;
    data->coord = tile.coord;
    tile.assign(data);
    target->pos = posFor(fpga::ELEMENT_MUXF7);
    target->coord = tile.coord;
    tile.assign(target);
    fixture.connect(data, "O", target, "I0");
    rtl::Net* net = fixture.connect(driver, "O", target, "I1");
    net->distributed_source = true;
    net->route_protected = true;

    rtl::Inst* from = driver;
    rtl::Inst* to = target;
    std::string from_port = "O";
    std::string to_port = "I1";
    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, net, false);

    // A distributed route keeps its required physical predecessor identity
    // even when the endpoint's current packed lane cannot host it.
    require(changed && to != target && to->tile.peer == nullptr,
        "blocked distributed target did not retain an unplaced passthrough");
    require(to->cell_ref.peer
            && to->cell_ref->attributes["scalepnr_passthrough"] == "target",
        "deferred distributed endpoint lost its target-passthrough identity");
    require(net->distributed_source && net->route_protected,
        "deferred endpoint changed distributed route ownership");

    // Moving must be able to repack the real sink while the generated target
    // remains unplaced; that pending input is temporarily fabric-facing.
    require(tile.unassign(blocker), "failed to release deferred predecessor lane");
    int target_pos = target->pos;
    require(tile.unassign(target), "failed to detach distributed target anchor");
    require(tile.unassign(data), "failed to detach the real MUX predecessor");
    require(tile.tryAddAt(data, 7) >= 0,
        "real MUX predecessor could not repack into its dedicated input lane");
    require(tile.tryAddAt(target, target_pos) >= 0,
        "real target chain did not reserve the deferred endpoint lane");

    // Once Moving provides a compatible lane, the same generated endpoint is
    // rehomed beside its real sink rather than being regenerated or bypassed.
    std::string reason;
    require(fpga::rehomeGeneratedPassthrough(*to, &reason),
        "deferred distributed target did not rehome: " + reason);
    require(to->tile.peer == &tile,
        "rehomed distributed target was not assigned beside its sink");
}

void distributed_target_defers_when_all_predecessors_are_busy()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(32);

    auto* driver = fixture.makeInst(
        "distributed_driver", "LUT2", portsFor(fpga::ELEMENT_LUT5));
    auto* target = fixture.makeInst("packed_mux", "MUXF7",
        {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN},
         {"O", rtl::Port::PORT_OUT}});
    target->pos = posFor(fpga::ELEMENT_MUXF7);
    target->coord = tile.coord;
    tile.assign(target);
    rtl::Net* net = fixture.connect(driver, "O", target, "I1");
    net->distributed_source = true;
    net->route_protected = true;

    // Initialize copied element masks, then emulate a packed tile in which
    // every structurally compatible predecessor endpoint is currently leased.
    (void)tile.candidatePositions(target);
    tile.pin_state.leased_nodes = ~NodeMask{};

    rtl::Inst* from = driver;
    rtl::Inst* to = target;
    std::string from_port = "O";
    std::string to_port = "I1";
    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, net, false);

    // The protected route must target a predecessor instead of the non-fabric
    // MUX input; Moving can relocate/rehome that endpoint when routing is busy.
    require(changed && to != target,
        "all-busy distributed input lost its predecessor endpoint");
    require(to->cell_ref.peer
            && to->cell_ref->attributes["scalepnr_passthrough"] == "target",
        "all-busy distributed input did not retain passthrough identity");
}

void distributed_mux_input_uses_the_free_predecessor_lane()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(32);

    auto* constant = fixture.makeInst("constant", "LUT2", portsFor(fpga::ELEMENT_LUT5));
    auto* data = fixture.makeInst("data", "LUT6", portsFor(fpga::ELEMENT_LUT5));
    auto* target = fixture.makeInst("target_mux", "MUXF7",
        {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
    auto* wide_mux = fixture.makeInst("wide_mux", "MUXF8",
        {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
    auto* output_sink = fixture.makeInst("output_sink", "FDRE", portsFor(fpga::ELEMENT_FD));
    fixture.connect(data, "O", target, "I0");
    rtl::Net* constant_net = fixture.connect(constant, "O", target, "I1");
    fixture.connect(target, "O", wide_mux, "I1");
    fixture.connect(wide_mux, "O", output_sink, "D");
    constant_net->distributed_source = true;
    constant_net->route_protected = true;

    // I0 consumes predecessor lane 1; I1 must retain the distinct free lane 0.
    data->pos = 7;
    data->coord = tile.coord;
    tile.assign(data);
    target->pos = 1;
    target->coord = tile.coord;
    tile.assign(target);
    wide_mux->pos = 1;
    wide_mux->coord = tile.coord;
    tile.assign(wide_mux);

    rtl::Inst* from = constant;
    rtl::Inst* to = target;
    std::string from_port = "O";
    std::string to_port = "I1";
    bool changed = fpga::preparePassthroughRouteEndpoints(
        from, from_port, to, to_port, constant_net, false);

    // Endpoint preparation must consume the free I1 predecessor immediately;
    // deferral would make Moving search for a lane that is already available.
    require(changed && to != target, "distributed MUX input did not create a predecessor endpoint");
    require(to->tile.peer == &tile, "distributed MUX input ignored its free predecessor lane");
    require(to->pos == 3, "distributed MUX I1 endpoint used the wrong predecessor lane");
}

void mux_inputs_use_distinct_lanes()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();

    NodeMask f7_i0 = tile.getPinNodes("MUXF7", "I0", posFor(fpga::ELEMENT_MUXF7));
    NodeMask f7_i1 = tile.getPinNodes("MUXF7", "I1", posFor(fpga::ELEMENT_MUXF7));
    require(f7_i0 != NodeMask{} && f7_i1 != NodeMask{}, "MUXF7 input locals were not modeled");
    require((f7_i0 & f7_i1) == NodeMask{}, "MUXF7 I0 and I1 alias the same input local");

    NodeMask f8_i0 = tile.getPinNodes("MUXF8", "I0", posFor(fpga::ELEMENT_MUXF8));
    NodeMask f8_i1 = tile.getPinNodes("MUXF8", "I1", posFor(fpga::ELEMENT_MUXF8));
    require(f8_i0 != NodeMask{} && f8_i1 != NodeMask{}, "MUXF8 input locals were not modeled");
    require((f8_i0 & f8_i1) == NodeMask{}, "MUXF8 I0 and I1 alias the same input local");
}

}

int main()
{
    distributed_target_defers_when_all_predecessors_are_busy();
    source_passthrough_cases();
    source_passthrough_invalidates_sink_joint_reservations();
    forced_fabric_output_keeps_the_original_route_endpoint();
    target_passthrough_cases();
    protected_external_net_uses_target_passthrough();
    passthrough_rejects_unrelated_lut_overlay();
    distributed_target_defers_and_rehomes_blocked_passthrough();
    distributed_mux_input_uses_the_free_predecessor_lane();
    mux_inputs_use_distinct_lanes();
    empty_passthrough_attribute_is_not_generated_endpoint();
    equal_neighbor_bits_do_not_alias_element_types();
    return 0;
}
