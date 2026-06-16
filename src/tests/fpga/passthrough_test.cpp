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
    tile_type.elements.push_back(makeElement("LUT1", fpga::ELEMENT_LUT1, 0));
    tile_type.elements.push_back(makeElement("MUXF7", fpga::ELEMENT_MUXF7, 0));
    tile_type.elements.push_back(makeElement("MUXF8", fpga::ELEMENT_MUXF8, 0));
    tile_type.elements.push_back(makeElement("FD", fpga::ELEMENT_FD, 0));
    connectElements(tile_type, fpga::ELEMENT_LUT5, 0, fpga::ELEMENT_LUT1, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT1, 0, fpga::ELEMENT_MUXF7, 0);
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

void passthrough_rejects_unrelated_lut_overlay()
{
    fpga::TileType tile_type = makePassthroughTileType();
    resetOneTileDevice(tile_type);
    fpga::Tile& tile = fpga::Device::current().tile_grid.front();
    Fixture fixture;
    fixture.parent_module.nets.reserve(32);

    auto* unrelated = fixture.makeInst("unrelated_lut", "LUT3", portsFor(fpga::ELEMENT_LUT5));
    auto* unrelated_driver = fixture.makeInst("unrelated_driver", "LUT5", portsFor(fpga::ELEMENT_LUT5));
    unrelated->pos = posFor(fpga::ELEMENT_LUT5);
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
    source_passthrough_cases();
    target_passthrough_cases();
    passthrough_rejects_unrelated_lut_overlay();
    mux_inputs_use_distinct_lanes();
    return 0;
}
