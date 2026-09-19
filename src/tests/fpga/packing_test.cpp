#include "Device.h"
#include "RegBunch.h"
#include "TimingPath.h"
#include "Tile.h"
#include "Cell.h"
#include "Conn.h"
#include "Module.h"
#include "RoutePassState.h"

#include <cstdio>
#include <chrono>
#include <memory>
#include <random>
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

uint16_t bit16(int bit)
{
    return static_cast<uint16_t>(1u << bit);
}

int posFor(fpga::ElementType type, int bit)
{
    int site = bit / 4;
    int bel = bit % 4;
    switch (type) {
    case fpga::ELEMENT_FD: {
        int fd_site = bit / 8;
        int lane = bit % 8;
        int fd_bel = lane % 4;
        int fd_column = lane >= 4 ? 64 : 0;
        return fd_site*128 + fd_column + fd_bel*4;
    }
    case fpga::ELEMENT_LUT5:
    case fpga::ELEMENT_LUT1:
        return site*128 + bel*4 + 3;
    case fpga::ELEMENT_MUXF7:
        return site*128 + (bel < 2 ? 0 : 2)*4 + 1;
    case fpga::ELEMENT_MUXF8:
        return site*128 + 1;
    case fpga::ELEMENT_CARRY:
        return site*128 + 2;
    default:
        return -1;
    }
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

fpga::TileType makePackingTileType()
{
    fpga::TileType tile_type{"GENERIC_PACKING_TEST", 1, 0};
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE0", .type = "LOGIC", .pos = 0});
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE1", .type = "LOGIC", .pos = 1});
    for (int bit = 0; bit < 8; ++bit) {
        tile_type.elements.push_back(makeElement("LUT5" + std::to_string(bit), fpga::ELEMENT_LUT5, bit));
        tile_type.elements.push_back(makeElement("LUT1" + std::to_string(bit), fpga::ELEMENT_LUT1, bit));
    }
    for (int bit = 0; bit < 16; ++bit) {
        tile_type.elements.push_back(makeElement("FD" + std::to_string(bit), fpga::ELEMENT_FD, bit));
    }
    for (int bit : {0, 2, 4, 6}) {
        tile_type.elements.push_back(makeElement("MUXF7" + std::to_string(bit), fpga::ELEMENT_MUXF7, bit));
    }
    for (int bit : {0, 4}) {
        tile_type.elements.push_back(makeElement("MUXF8" + std::to_string(bit), fpga::ELEMENT_MUXF8, bit));
        tile_type.elements.push_back(makeElement("CARRY" + std::to_string(bit), fpga::ELEMENT_CARRY, bit));
    }
    for (int site : {0, 4}) {
        for (int pair : {0, 2}) {
            int f7 = site + pair;
            connectElements(tile_type, fpga::ELEMENT_LUT5, f7, fpga::ELEMENT_MUXF7, f7);
            connectElements(tile_type, fpga::ELEMENT_LUT5, f7 + 1, fpga::ELEMENT_MUXF7, f7);
            connectElements(tile_type, fpga::ELEMENT_LUT1, f7, fpga::ELEMENT_MUXF7, f7);
            connectElements(tile_type, fpga::ELEMENT_LUT1, f7 + 1, fpga::ELEMENT_MUXF7, f7);
        }
        connectElements(tile_type, fpga::ELEMENT_MUXF7, site, fpga::ELEMENT_MUXF8, site);
        connectElements(tile_type, fpga::ELEMENT_MUXF7, site + 2, fpga::ELEMENT_MUXF8, site);
        connectElements(tile_type, fpga::ELEMENT_MUXF8, site, fpga::ELEMENT_FD, (site / 4)*8);
        connectElements(tile_type, fpga::ELEMENT_MUXF8, site, fpga::ELEMENT_FD, (site / 4)*8 + 4);
    }
    return tile_type;
}

fpga::Tile& resetTile(fpga::TileType& tile_type)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.tile_grid.resize(1);
    device.size_width = 1;
    device.size_height = 1;
    device.grid_spec.size = {1, 1};
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = {0, 0};
    tile.cb_coord = tile.coord;
    tile.name = {0, 0};
    tile.tile_type = &tile_type;
    tile.cb_type = nullptr;
    tile.cb.type = nullptr;
    tile.elements_initialized = false;
    tile.elements_pos = {};
    tile.elements_free = {};
    tile.elements_left = {};
    tile.elements_right = {};
    return tile;
}

std::pair<fpga::Tile&, fpga::Tile&> resetTwoTiles(fpga::TileType& tile_type)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.tile_grid.resize(2);
    device.size_width = 2;
    device.size_height = 1;
    device.grid_spec.size = {2, 1};
    fpga::Tile& left = device.tile_grid[0];
    left.coord = {0, 0};
    left.cb_coord = left.coord;
    left.name = {0, 0};
    left.tile_type = &tile_type;
    left.cb_type = nullptr;
    left.cb.type = nullptr;
    left.elements_initialized = false;
    left.elements_pos = {};
    left.elements_free = {};
    left.elements_left = {};
    left.elements_right = {};

    fpga::Tile& right = device.tile_grid[1];
    right.coord = {1, 0};
    right.cb_coord = right.coord;
    right.name = {1, 0};
    right.tile_type = &tile_type;
    right.cb_type = nullptr;
    right.cb.type = nullptr;
    right.elements_initialized = false;
    right.elements_pos = {};
    right.elements_free = {};
    right.elements_left = {};
    right.elements_right = {};
    return {left, right};
}

struct Fixture
{
    Referable<rtl::Module> parent;
    Referable<rtl::Module> primitive_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    int next_designator = 1;

    Fixture()
    {
        parent.name = "top";
        primitive_module.name = "primitive";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&parent);
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
        inst->cnt_inputs = 0;
        inst->cnt_outputs = 0;
        inst->pos = -1;
        for (auto& port : inst->cell_ref->ports) {
            if (port.type == rtl::Port::PORT_IN) ++inst->cnt_inputs;
            if (port.type == rtl::Port::PORT_OUT) ++inst->cnt_outputs;
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
        for (auto& conn : inst->conns) {
            if (conn.port_ref.peer && conn.port_ref->name == port_name) {
                return &conn;
            }
        }
        return nullptr;
    }

    void connect(Referable<rtl::Inst>* driver, const std::string& driver_port,
                 Referable<rtl::Inst>* sink, const std::string& sink_port)
    {
        Referable<rtl::Conn>* out = conn(driver, driver_port);
        Referable<rtl::Conn>* in = conn(sink, sink_port);
        require(out && in, "test connection references a missing port");
        int designator = next_designator++;
        out->port_ref->designator = designator;
        in->port_ref->designator = designator;
        in->set(out);
        auto& net = parent.nets.emplace_back();
        net.name = "n" + std::to_string(designator);
        net.designators.push_back(designator);
    }
};

Referable<rtl::Inst>* makeLut(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "LUT5", {{"O", rtl::Port::PORT_OUT}});
}

Referable<rtl::Inst>* makeLut6(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "LUT6", {{"O", rtl::Port::PORT_OUT}});
}

Referable<rtl::Inst>* makeInputLut(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "LUT2", {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
}

Referable<rtl::Inst>* makeLut1(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "LUT1", {{"I0", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
}

Referable<rtl::Inst>* makeF7(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "MUXF7", {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
}

Referable<rtl::Inst>* makeF8(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "MUXF8", {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
}

Referable<rtl::Inst>* makeFd(Fixture& fixture, const std::string& name)
{
    return fixture.makeInst(name, "FDRE", {{"D", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
}

void placeManual(fpga::Tile& tile, Referable<rtl::Inst>* inst, fpga::ElementType type, int bit)
{
    inst->pos = posFor(type, bit);
    inst->coord = tile.coord;
    tile.assign(inst);
}

void occupyOtherBits(fpga::Tile& tile, Fixture& fixture, fpga::ElementType type, const std::vector<int>& bits, int keep)
{
    for (int bit : bits) {
        if (bit == keep) {
            continue;
        }
        Referable<rtl::Inst>* inst = nullptr;
        if (type == fpga::ELEMENT_MUXF7) inst = makeF7(fixture, "busy_f7_" + std::to_string(bit));
        else if (type == fpga::ELEMENT_MUXF8) inst = makeF8(fixture, "busy_f8_" + std::to_string(bit));
        else if (type == fpga::ELEMENT_FD) inst = makeFd(fixture, "busy_fd_" + std::to_string(bit));
        else if (type == fpga::ELEMENT_LUT1) inst = makeLut1(fixture, "busy_lut1_" + std::to_string(bit));
        else inst = makeLut(fixture, "busy_lut_" + std::to_string(bit));
        placeManual(tile, inst, type, bit);
    }
}

fpga::TileType makeComplexConflictTileType(bool lut1_in_lut_fd_chain)
{
    fpga::TileType tile_type{"COMPLEX_CONFLICT_TEST", 1, 0};
    for (int site = 0; site < 4; ++site) {
        tile_type.sites.push_back(fpga::SiteModel{.name = "SITE" + std::to_string(site), .type = "LOGIC", .pos = site});
    }
    for (int bit = 0; bit < 8; ++bit) {
        tile_type.elements.push_back(makeElement("LUT5" + std::to_string(bit), fpga::ELEMENT_LUT5, bit));
        tile_type.elements.push_back(makeElement("LUT1" + std::to_string(bit), fpga::ELEMENT_LUT1, bit));
        tile_type.elements.push_back(makeElement("CARRY" + std::to_string(bit), fpga::ELEMENT_CARRY, bit));
    }
    for (int bit = 0; bit < 16; ++bit) {
        tile_type.elements.push_back(makeElement("FD" + std::to_string(bit), fpga::ELEMENT_FD, bit));
    }
    for (int bit = 0; bit < 8; ++bit) {
        if (lut1_in_lut_fd_chain) {
            connectElements(tile_type, fpga::ELEMENT_LUT5, bit, fpga::ELEMENT_LUT1, bit);
            connectElements(tile_type, fpga::ELEMENT_LUT1, bit, fpga::ELEMENT_CARRY, bit);
        }
        else {
            connectElements(tile_type, fpga::ELEMENT_LUT5, bit, fpga::ELEMENT_CARRY, bit);
        }
        connectElements(tile_type, fpga::ELEMENT_CARRY, bit, fpga::ELEMENT_FD, bit);
    }
    return tile_type;
}

void lut_to_f7_requires_connectivity()
{
    for (int lut_bit = 0; lut_bit < 8; ++lut_bit) {
        int f7_bit = (lut_bit / 4)*4 + ((lut_bit % 4) < 2 ? 0 : 2);
        {
            fpga::TileType tile_type = makePackingTileType();
            fpga::Tile& tile = resetTile(tile_type);
            Fixture fixture;
            auto* lut = makeLut(fixture, "lut");
            auto* f7 = makeF7(fixture, "f7");
            fixture.connect(lut, "O", f7, (lut_bit % 2) ? "I0" : "I1");
            placeManual(tile, lut, fpga::ELEMENT_LUT5, lut_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {0, 2, 4, 6}, f7_bit);
            int pos = tile.tryAdd(f7);
            require(pos == posFor(fpga::ELEMENT_MUXF7, f7_bit), "connected LUT->MUXF7 was not packed");
            require(fixture.parent.nets.front().void_net,
                    "filtered chain refresh lost a packed LUT-to-MUX internal net");
        }
        {
            fpga::TileType tile_type = makePackingTileType();
            fpga::Tile& tile = resetTile(tile_type);
            Fixture fixture;
            auto* lut = makeLut(fixture, "lut");
            auto* f7 = makeF7(fixture, "f7");
            placeManual(tile, lut, fpga::ELEMENT_LUT5, lut_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {0, 2, 4, 6}, f7_bit);
            require(tile.tryAdd(f7) < 0, "unconnected LUT blocked no MUXF7 placement");
        }
    }
}

void f7_to_f8_requires_connectivity()
{
    for (int f7_bit : {0, 2, 4, 6}) {
        int f8_bit = (f7_bit / 4)*4;
        {
            fpga::TileType tile_type = makePackingTileType();
            fpga::Tile& tile = resetTile(tile_type);
            Fixture fixture;
            auto* f7 = makeF7(fixture, "f7");
            auto* f8 = makeF8(fixture, "f8");
            fixture.connect(f7, "O", f8, (f7_bit % 4) ? "I0" : "I1");
            placeManual(tile, f7, fpga::ELEMENT_MUXF7, f7_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF8, {0, 4}, f8_bit);
            require(tile.tryAdd(f8) == posFor(fpga::ELEMENT_MUXF8, f8_bit), "connected MUXF7->MUXF8 was not packed");
        }
        {
            fpga::TileType tile_type = makePackingTileType();
            fpga::Tile& tile = resetTile(tile_type);
            Fixture fixture;
            auto* f7 = makeF7(fixture, "f7");
            auto* f8 = makeF8(fixture, "f8");
            placeManual(tile, f7, fpga::ELEMENT_MUXF7, f7_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF8, {0, 4}, f8_bit);
            require(tile.tryAdd(f8) < 0, "unconnected MUXF7 blocked no MUXF8 placement");
        }
    }
}

void connected_f7_f8_chain_rejects_other_tile()
{
    fpga::TileType tile_type = makePackingTileType();
    auto [chain_tile, other_tile] = resetTwoTiles(tile_type);
    Fixture fixture;
    auto* f7 = makeF7(fixture, "f7");
    auto* f8 = makeF8(fixture, "f8");
    fixture.connect(f7, "O", f8, "I1");

    placeManual(chain_tile, f7, fpga::ELEMENT_MUXF7, 0);

    require(other_tile.tryAdd(f8) < 0,
        "connected MUXF7->MUXF8 chain was allowed to split across tiles");
    require(chain_tile.tryAdd(f8) == posFor(fpga::ELEMENT_MUXF8, 0),
        "connected MUXF7->MUXF8 chain was not accepted in the source tile");
}

void connected_lut_f7_chain_rejects_other_tile()
{
    fpga::TileType tile_type = makePackingTileType();
    auto [chain_tile, other_tile] = resetTwoTiles(tile_type);
    Fixture fixture;
    auto* lut = makeLut(fixture, "lut");
    auto* f7 = makeF7(fixture, "f7");
    fixture.connect(lut, "O", f7, "I1");

    placeManual(chain_tile, lut, fpga::ELEMENT_LUT5, 0);

    require(other_tile.tryAdd(f7) < 0,
        "connected LUT->MUXF7 chain was allowed to split across tiles");
    require(chain_tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 0),
        "connected LUT->MUXF7 chain was not accepted in the source tile");
}

void unplaced_strict_chain_sink_reserves_future_lane()
{
    {
        fpga::TileType tile_type = makePackingTileType();
        fpga::Tile& tile = resetTile(tile_type);
        Fixture fixture;
        auto* lut = makeLut(fixture, "lut");
        auto* f7 = makeF7(fixture, "f7");
        fixture.connect(lut, "O", f7, "I0");
        occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {0, 2, 4, 6}, 2);

        require(tile.tryAdd(lut) == posFor(fpga::ELEMENT_LUT5, 3),
            "LUT did not choose a lane with a future free MUXF7 neighbor");
        require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 2),
            "MUXF7 was not packed into the reserved future lane");
    }
    {
        fpga::TileType tile_type = makePackingTileType();
        fpga::Tile& tile = resetTile(tile_type);
        Fixture fixture;
        auto* lut = makeLut(fixture, "lut");
        auto* f7 = makeF7(fixture, "f7");
        fixture.connect(lut, "O", f7, "I0");
        occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {0, 2, 4, 6}, -1);

        require(tile.tryAdd(lut) < 0,
            "LUT packed even though no future MUXF7 lane was available");
    }
}

void unplaced_strict_sink_avoids_occupied_future_blockers()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* lut = makeLut(fixture, "future_driver");
    auto* f7 = makeF7(fixture, "future_sink");
    fixture.connect(lut, "O", f7, "I0");

    // Block the unused predecessor of the first three F7 lanes. The producer
    // must reserve the fourth lane, whose complete future chain remains legal.
    for (int bit : {0, 2, 4}) {
        placeManual(tile, makeLut1(fixture, "unrelated_" + std::to_string(bit)),
                    fpga::ELEMENT_LUT1, bit);
    }

    require(tile.tryAdd(lut) == posFor(fpga::ELEMENT_LUT5, 7),
        "producer selected a future mux lane blocked by an unrelated predecessor");
    require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 6),
        "future mux could not consume the lane reserved by its producer");
}

void unplaced_mux_sink_keeps_all_drivers_in_one_tile()
{
    fpga::TileType tile_type = makePackingTileType();
    auto [chain_tile, other_tile] = resetTwoTiles(tile_type);
    Fixture fixture;
    auto* lut0 = makeLut(fixture, "lut0");
    auto* lut1 = makeLut(fixture, "lut1");
    auto* f7 = makeF7(fixture, "f7");
    fixture.connect(lut0, "O", f7, "I0");
    fixture.connect(lut1, "O", f7, "I1");

    require(chain_tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 1),
        "first LUT driver did not pack into expected strict-chain lane");
    require(other_tile.tryAdd(lut1) < 0,
        "second LUT driver of an unplaced MUXF7 was allowed to split to another tile");
    require(chain_tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 0),
        "second LUT driver was not accepted beside its sibling strict-chain driver");
    require(chain_tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 0),
        "MUXF7 was not accepted with both LUT drivers in one tile");
}

void unplaced_mux_sink_requires_shared_driver_lane()
{
    {
        fpga::TileType tile_type = makePackingTileType();
        fpga::Tile& tile = resetTile(tile_type);
        Fixture fixture;
        auto* lut0 = makeLut(fixture, "lut0");
        auto* lut1 = makeLut(fixture, "lut1");
        auto* f7 = makeF7(fixture, "f7");
        fixture.connect(lut0, "O", f7, "I0");
        fixture.connect(lut1, "O", f7, "I1");

        placeManual(tile, lut0, fpga::ELEMENT_LUT5, 3);

        require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 2),
            "second LUT driver was not forced into the shared MUXF7 lane");
        require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 2),
            "MUXF7 was not packed into the shared driver lane");
    }
    {
        fpga::TileType tile_type = makePackingTileType();
        fpga::Tile& tile = resetTile(tile_type);
        Fixture fixture;
        auto* lut0 = makeLut(fixture, "lut0");
        auto* lut1 = makeLut(fixture, "lut1");
        auto* f7 = makeF7(fixture, "f7");
        fixture.connect(lut0, "O", f7, "I0");
        fixture.connect(lut1, "O", f7, "I1");

        placeManual(tile, lut0, fpga::ELEMENT_LUT5, 3);
        occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {2}, -1);

        require(tile.tryAdd(lut1) < 0,
            "second LUT driver used a different MUXF7 lane after the shared lane was occupied");
    }
}

void unplaced_mux_sink_requires_free_future_driver_lane()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* busy = makeLut(fixture, "busy_lut0");
    auto* lut1 = makeLut(fixture, "lut1");
    auto* lut0 = makeLut(fixture, "lut0");
    auto* f7 = makeF7(fixture, "f7");
    fixture.connect(lut0, "O", f7, "I0");
    fixture.connect(lut1, "O", f7, "I1");

    placeManual(tile, busy, fpga::ELEMENT_LUT5, 0);
    occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {0, 2, 4, 6}, 0);

    require(tile.tryAdd(lut1) < 0,
        "first LUT driver reserved a MUXF7 lane whose paired future LUT lane was occupied");
}

void mux_sink_waits_for_all_strict_drivers()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* lut0 = makeLut(fixture, "lut0");
    auto* lut1 = makeLut(fixture, "lut1");
    auto* f7 = makeF7(fixture, "f7");
    fixture.connect(lut0, "O", f7, "I0");
    fixture.connect(lut1, "O", f7, "I1");

    placeManual(tile, lut0, fpga::ELEMENT_LUT5, 1);

    require(tile.tryAdd(f7) < 0,
        "MUXF7 packed before all strict LUT drivers were placed");
    require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 0),
        "second LUT driver did not pack into the first driver's shared lane");
    require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 0),
        "MUXF7 did not pack after all strict LUT drivers were placed");
}

void unplaced_f7_f8_sink_reserves_future_lane()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* f7 = makeF7(fixture, "f7");
    auto* f8 = makeF8(fixture, "f8");
    fixture.connect(f7, "O", f8, "I0");
    occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF8, {0, 4}, 4);

    require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 6),
        "MUXF7 did not choose a lane with a future free MUXF8 neighbor");
    require(tile.tryAdd(f8) == posFor(fpga::ELEMENT_MUXF8, 4),
        "MUXF8 was not packed into the reserved future lane");
}

void lut6_pair_into_f7_reserves_future_f8_lane()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* lut0 = makeLut6(fixture, "lut0");
    auto* lut1 = makeLut6(fixture, "lut1");
    auto* f7 = makeF7(fixture, "f7");
    auto* sibling_f7 = makeF7(fixture, "sibling_f7");
    auto* f8 = makeF8(fixture, "f8");
    fixture.connect(lut0, "O", f7, "I0");
    fixture.connect(lut1, "O", f7, "I1");
    fixture.connect(f7, "O", f8, "I1");
    fixture.connect(sibling_f7, "O", f8, "I0");

    require(tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 1),
        "first LUT6 driver did not reserve a legal MUXF7 lane");
    require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 0),
        "second LUT6 driver did not pack into the shared MUXF7 lane");
    require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 0),
        "MUXF7 with LUT6 drivers was rejected while reserving a future MUXF8 lane");
}

void future_f8_lane_requires_packable_sibling_f7()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* busy0 = makeLut6(fixture, "busy0");
    auto* busy1 = makeLut6(fixture, "busy1");
    auto* lut0 = makeLut6(fixture, "lut0");
    auto* lut1 = makeLut6(fixture, "lut1");
    auto* sibling_lut0 = makeLut6(fixture, "sibling_lut0");
    auto* sibling_lut1 = makeLut6(fixture, "sibling_lut1");
    auto* f7 = makeF7(fixture, "f7");
    auto* sibling_f7 = makeF7(fixture, "sibling_f7");
    auto* f8 = makeF8(fixture, "f8");
    fixture.connect(lut0, "O", f7, "I0");
    fixture.connect(lut1, "O", f7, "I1");
    fixture.connect(sibling_lut0, "O", sibling_f7, "I0");
    fixture.connect(sibling_lut1, "O", sibling_f7, "I1");
    fixture.connect(f7, "O", f8, "I1");
    fixture.connect(sibling_f7, "O", f8, "I0");

    placeManual(tile, busy0, fpga::ELEMENT_LUT5, 0);
    placeManual(tile, busy1, fpga::ELEMENT_LUT5, 1);
    placeManual(tile, lut0, fpga::ELEMENT_LUT5, 2);
    placeManual(tile, lut1, fpga::ELEMENT_LUT5, 3);

    require(tile.tryAdd(f7) < 0,
        "MUXF7 reserved a future MUXF8 lane whose sibling MUXF7 input LUTs cannot fit");
}

void future_f8_lane_rejects_unconnected_occupied_sibling_f7_blockers()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* busy0 = makeLut6(fixture, "busy0");
    auto* busy1 = makeLut6(fixture, "busy1");
    auto* lut0 = makeLut6(fixture, "lut0");
    auto* lut1 = makeLut6(fixture, "lut1");
    auto* f7 = makeF7(fixture, "f7");
    auto* sibling_f7 = makeF7(fixture, "sibling_f7");
    auto* f8 = makeF8(fixture, "f8");
    fixture.connect(lut0, "O", f7, "I0");
    fixture.connect(lut1, "O", f7, "I1");
    fixture.connect(f7, "O", f8, "I1");
    fixture.connect(sibling_f7, "O", f8, "I0");

    placeManual(tile, busy0, fpga::ELEMENT_LUT5, 0);
    placeManual(tile, busy1, fpga::ELEMENT_LUT5, 1);
    placeManual(tile, lut0, fpga::ELEMENT_LUT5, 2);
    placeManual(tile, lut1, fpga::ELEMENT_LUT5, 3);

    require(tile.tryAdd(f7) < 0,
        "MUXF7 reserved a sibling MUXF7 lane through unrelated occupied LUT blockers");
}

void f8_to_fd_requires_connectivity()
{
    for (int f8_bit : {0, 4}) {
        int fd_bit = (f8_bit / 4)*8;
        {
            fpga::TileType tile_type = makePackingTileType();
            fpga::Tile& tile = resetTile(tile_type);
            Fixture fixture;
            auto* f8 = makeF8(fixture, "f8");
            auto* fd = makeFd(fixture, "fd");
            fixture.connect(f8, "O", fd, "D");
            placeManual(tile, f8, fpga::ELEMENT_MUXF8, f8_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_FD, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, fd_bit);
            require(tile.tryAdd(fd) == posFor(fpga::ELEMENT_FD, fd_bit), "connected MUXF8->FD was not packed");
        }
        {
            fpga::TileType tile_type = makePackingTileType();
            fpga::Tile& tile = resetTile(tile_type);
            Fixture fixture;
            auto* f8 = makeF8(fixture, "f8");
            auto* fd = makeFd(fixture, "fd");
            placeManual(tile, f8, fpga::ELEMENT_MUXF8, f8_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_FD, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, fd_bit);
            require(tile.tryAdd(fd) < 0, "unconnected MUXF8 blocked no FD placement");
        }
    }
}

void tile_type_has_sixteen_fd_positions()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;

    for (int bit = 0; bit < 16; ++bit) {
        auto* fd = makeFd(fixture, "fd_" + std::to_string(bit));
        require(tile.tryAdd(fd) == posFor(fpga::ELEMENT_FD, bit),
            "FD packing did not expose all 16 FF positions");
    }
    require(tile.elements_free[fpga::ELEMENT_FD] == 0,
        "FD free mask still has bits after packing 16 FF positions");

    auto* moved = fixture.insts[7].get();
    const int pos = moved->pos;
    const auto start = std::chrono::steady_clock::now();
    for (int trial = 0; trial < 10000; ++trial) {
        require(tile.unassign(moved) && tile.tryAddAt(moved, pos) == pos,
                "repeated packing trial did not restore its exact slot");
        require(tile.elements_free[fpga::ELEMENT_FD] == 0 && tile.regs_cnt == 16,
                "repeated packing trial corrupted resource accounting");
    }
    std::printf("PACKING_RESTORE_BENCH cells=16 trials=10000 elapsed_ms=%.3f\n",
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count());
}

void carry_chain_refresh_still_marks_internal_nets()
{
    auto tile_type = makePackingTileType();
    auto& tile = resetTile(tile_type);
    Fixture fixture;
    auto* first = fixture.makeInst("carry_first", "CARRY4",
        {{"CI", rtl::Port::PORT_IN}, {"CO", rtl::Port::PORT_OUT}});
    auto* second = fixture.makeInst("carry_second", "CARRY4",
        {{"CI", rtl::Port::PORT_IN}, {"CO", rtl::Port::PORT_OUT}});
    fixture.connect(first, "CO", second, "CI");
    placeManual(tile, first, fpga::ELEMENT_CARRY, 0);
    placeManual(tile, second, fpga::ELEMENT_CARRY, 4);
    auto* reg = makeFd(fixture, "refresh_trigger");
    require(tile.tryAdd(reg) >= 0, "could not trigger packed-chain refresh");
    require(fixture.parent.nets.front().void_net,
            "filtered chain refresh lost the internal carry connection");
}

void distant_fd_conflicts_are_found_through_free_carry()
{
    fpga::TileType tile_type = makeComplexConflictTileType(false);
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {0, 1, 2, 3, 4, 5, 6, 7}, -1);

    int packed = 0;
    for (int index = 0; index < 16; ++index) {
        auto* fd = makeFd(fixture, "fd_" + std::to_string(index));
        if (tile.tryAdd(fd) >= 0) {
            ++packed;
            require(fd->pos == posFor(fpga::ELEMENT_FD, 8 + packed - 1),
                "unconnected distant LUT blocker did not force FD into independent lane");
        }
    }
    require(packed == 8, "busy unconnected LUTs allowed more than 8 independent FDs");
}

void distant_connected_lut_fd_pairs_and_independent_fds_pack()
{
    fpga::TileType tile_type = makeComplexConflictTileType(false);
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;

    for (int bit = 0; bit < 8; ++bit) {
        auto* lut = makeLut(fixture, "lut_" + std::to_string(bit));
        auto* fd = makeFd(fixture, "fd_pair_" + std::to_string(bit));
        fixture.connect(lut, "O", fd, "D");
        placeManual(tile, lut, fpga::ELEMENT_LUT5, bit);
        require(tile.tryAdd(fd) == posFor(fpga::ELEMENT_FD, bit),
            "connected distant LUT->FD pair was not packed through CARRY");
    }

    for (int bit = 8; bit < 16; ++bit) {
        auto* fd = makeFd(fixture, "fd_independent_" + std::to_string(bit));
        require(tile.tryAdd(fd) == posFor(fpga::ELEMENT_FD, bit),
            "independent FD lane was not packed after connected pairs");
    }
}

void independent_lut1_does_not_block_distant_fd()
{
    constexpr int keep = 5;
    fpga::TileType tile_type = makeComplexConflictTileType(false);
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;

    occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {0, 1, 2, 3, 4, 5, 6, 7}, keep);
    occupyOtherBits(tile, fixture, fpga::ELEMENT_FD, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, keep);
    auto* lut = makeLut(fixture, "connected_lut");
    auto* lut1 = makeLut1(fixture, "independent_lut1");
    auto* fd = makeFd(fixture, "connected_fd");
    fixture.connect(lut, "O", fd, "D");
    placeManual(tile, lut, fpga::ELEMENT_LUT5, keep);
    placeManual(tile, lut1, fpga::ELEMENT_LUT1, keep);

    require(tile.tryAdd(fd) == posFor(fpga::ELEMENT_FD, keep),
        "independent LUT1 blocked a distant LUT5->FD path");
}

void chained_lut1_must_be_connected_for_distant_fd()
{
    constexpr int keep = 6;
    {
        fpga::TileType tile_type = makeComplexConflictTileType(true);
        fpga::Tile& tile = resetTile(tile_type);
        Fixture fixture;
        occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {0, 1, 2, 3, 4, 5, 6, 7}, keep);
        occupyOtherBits(tile, fixture, fpga::ELEMENT_FD, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, keep);
        auto* lut = makeLut(fixture, "lut");
        auto* lut1 = makeLut1(fixture, "unrelated_lut1");
        auto* fd = makeFd(fixture, "fd");
        fixture.connect(lut, "O", fd, "D");
        placeManual(tile, lut, fpga::ELEMENT_LUT5, keep);
        placeManual(tile, lut1, fpga::ELEMENT_LUT1, keep);

        require(tile.tryAdd(fd) < 0, "unconnected chained LUT1 did not block distant FD packing");
    }
    {
        fpga::TileType tile_type = makeComplexConflictTileType(true);
        fpga::Tile& tile = resetTile(tile_type);
        Fixture fixture;
        occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {0, 1, 2, 3, 4, 5, 6, 7}, keep);
        occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT1, {0, 1, 2, 3, 4, 5, 6, 7}, keep);
        occupyOtherBits(tile, fixture, fpga::ELEMENT_FD, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, keep);
        auto* lut = makeLut(fixture, "lut");
        auto* lut1 = makeLut1(fixture, "connected_lut1");
        auto* fd = makeFd(fixture, "fd");
        fixture.connect(lut, "O", lut1, "I0");
        fixture.connect(lut1, "O", fd, "D");
        placeManual(tile, lut, fpga::ELEMENT_LUT5, keep);

        require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT1, keep),
            "connected LUT5->LUT1 chain element was not packed");
        require(fixture.conn(fd, "D")->follow()->inst_ref.peer == lut1,
            "test fixture did not connect LUT1 output to FD input");
        int fd_pos = tile.tryAdd(fd);
        require(fd_pos == posFor(fpga::ELEMENT_FD, keep),
            "connected LUT1->FD chain endpoint was not packed");
    }
}

void independent_inputs_must_not_alias_one_local_node()
{
    fpga::TileType tile_type = makePackingTileType();
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "A1");
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "B1");
    tile_type.pin_map.input_nodes[1] = NodeMask{0,1} << 97;
    tile_type.pin_map.input_nodes[2] = NodeMask{0,1} << 97;
    tile_type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 1, 97, "ROUTE_A");
    tile_type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 2, 97, "ROUTE_A");
    auto cb_type = std::make_unique<fpga::CBType>("ROUTE_A");

    fpga::Tile& tile = resetTile(tile_type);
    tile.cb_type = cb_type.get();
    tile.cb.type = cb_type.get();
    Fixture fixture;
    occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {2, 3, 4, 5, 6, 7}, -1);
    auto* driver0 = fixture.makeInst("driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver1 = fixture.makeInst("driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* lut0 = makeInputLut(fixture, "lut0");
    auto* lut1 = makeInputLut(fixture, "lut1");
    fixture.connect(driver0, "O", lut0, "I0");
    fixture.connect(driver1, "O", lut1, "I0");

    require(tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first LUT input placement did not use expected slot");
    require(tile.tryAdd(lut1) < 0,
        "independent LUT inputs were packed onto the same routed local node");

    fpga::Tile& relaxed_tile = resetTile(tile_type);
    relaxed_tile.cb_type = cb_type.get();
    relaxed_tile.cb.type = cb_type.get();
    Fixture relaxed_fixture;
    occupyOtherBits(relaxed_tile, relaxed_fixture, fpga::ELEMENT_LUT5,
                    {2, 3, 4, 5, 6, 7}, -1);
    auto* relaxed_driver0 = relaxed_fixture.makeInst(
        "relaxed_driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* relaxed_driver1 = relaxed_fixture.makeInst(
        "relaxed_driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* relaxed_lut0 = makeInputLut(relaxed_fixture, "relaxed_lut0");
    auto* relaxed_lut1 = makeInputLut(relaxed_fixture, "relaxed_lut1");
    relaxed_fixture.connect(relaxed_driver0, "O", relaxed_lut0, "I0");
    relaxed_fixture.connect(relaxed_driver1, "O", relaxed_lut1, "I0");

    // A concrete endpoint is a structural site limit, so relaxed initial
    // placement must reject unrelated drivers instead of deferring the alias.
    require(relaxed_tile.tryAdd(relaxed_lut0, false) == posFor(fpga::ELEMENT_LUT5, 0)
            && relaxed_tile.tryAdd(relaxed_lut1, false) < 0,
        "relaxed initial placement admitted unrelated endpoint drivers");

    fpga::Tile& shared_tile = resetTile(tile_type);
    shared_tile.cb_type = cb_type.get();
    shared_tile.cb.type = cb_type.get();
    Fixture shared_fixture;
    occupyOtherBits(shared_tile, shared_fixture, fpga::ELEMENT_LUT5,
                    {2, 3, 4, 5, 6, 7}, -1);
    auto* shared_driver = shared_fixture.makeInst(
        "shared_driver", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* shared_lut0 = makeInputLut(shared_fixture, "shared_lut0");
    auto* shared_lut1 = makeInputLut(shared_fixture, "shared_lut1");
    shared_fixture.connect(shared_driver, "O", shared_lut0, "I0");
    shared_fixture.connect(shared_driver, "O", shared_lut1, "I0");

    // Multiple cells may consume one shared physical control endpoint when
    // the endpoint carries the same logical signal for every consumer.
    require(shared_tile.tryAdd(shared_lut0, false) == posFor(fpga::ELEMENT_LUT5, 0)
            && shared_tile.tryAdd(shared_lut1, false) == posFor(fpga::ELEMENT_LUT5, 1),
        "shared endpoint rejected consumers driven by the same signal");
}

void independent_inputs_must_not_alias_one_mandatory_joint()
{
    fpga::TileType tile_type = makePackingTileType();
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "A1");
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "B1");
    tile_type.pin_map.input_nodes[1] = NodeMask{0,1} << 90;
    tile_type.pin_map.input_nodes[2] = NodeMask{0,1} << 91;

    auto cb_type = std::make_unique<fpga::CBType>("GENERIC_ROUTE");
    cb_type->dst_joint[10].joint |= NodeMask{0,1} << 15;
    cb_type->dst_joint[11].joint |= NodeMask{0,1} << 15;
    cb_type->joint_local[15].local |= (NodeMask{0,1} << 90) | (NodeMask{0,1} << 91);
    cb_type->rebuildOutgoingSrcs();

    fpga::Tile& tile = resetTile(tile_type);
    tile.cb_type = cb_type.get();
    tile.cb.type = cb_type.get();
    Fixture fixture;
    occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {2, 3, 4, 5, 6, 7}, -1);
    auto* driver0 = fixture.makeInst("driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver1 = fixture.makeInst("driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* lut0 = makeInputLut(fixture, "lut0");
    auto* lut1 = makeInputLut(fixture, "lut1");
    fixture.connect(driver0, "O", lut0, "I0");
    fixture.connect(driver1, "O", lut1, "I0");

    require(tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first mandatory-joint input did not use the expected slot");
    NodeMask first_reservations = fpga::packedInputJointReservations(tile, lut1, "I0");
    NodeMask cached_reservations = fpga::packedInputJointReservations(tile, lut1, "I0");
    // Routing must see the same mandatory joint from the placement-owned cache on repeated attempts.
    require(first_reservations == (NodeMask{0,1} << 15)
            && cached_reservations == first_reservations,
        "cached mandatory-joint reservations differ from packed endpoint ownership");
    require(tile.tryAdd(lut1) < 0,
        "independent input locals sharing one mandatory joint were packed together");

    fpga::Tile& relaxed_tile = resetTile(tile_type);
    relaxed_tile.cb_type = cb_type.get();
    relaxed_tile.cb.type = cb_type.get();
    Fixture relaxed_fixture;
    occupyOtherBits(relaxed_tile, relaxed_fixture, fpga::ELEMENT_LUT5,
                    {2, 3, 4, 5, 6, 7}, -1);
    auto* relaxed_driver0 = relaxed_fixture.makeInst(
        "relaxed_driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* relaxed_driver1 = relaxed_fixture.makeInst(
        "relaxed_driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* relaxed_lut0 = makeInputLut(relaxed_fixture, "relaxed_lut0");
    auto* relaxed_lut1 = makeInputLut(relaxed_fixture, "relaxed_lut1");
    relaxed_fixture.connect(relaxed_driver0, "O", relaxed_lut0, "I0");
    relaxed_fixture.connect(relaxed_driver1, "O", relaxed_lut1, "I0");

    // Distinct locals may defer a switch-matrix joint conflict until routing;
    // strict Moving placement above still rejects the same collision.
    require(relaxed_tile.tryAdd(relaxed_lut0, false) == posFor(fpga::ELEMENT_LUT5, 0)
            && relaxed_tile.tryAdd(relaxed_lut1, false) == posFor(fpga::ELEMENT_LUT5, 1),
        "initial placement did not defer mandatory-joint congestion to routing");
}

void complete_pin_queries_match_uncached_models()
{
    fpga::TileType tile_type = makePackingTileType();
    for (auto direction : {fpga::TILE_PIN_INPUT, fpga::TILE_PIN_OUTPUT}) {
        for (int site = 0; site < 2; ++site) {
            int resource = site * 256;
            tile_type.pin_map.rememberResourcePinName(direction, resource, "A1");
            tile_type.pin_map.rememberResourcePinName(direction, resource + 1, "CE");
            tile_type.pin_map.rememberResourcePinName(direction, resource + 2, "AQ");
            auto& nodes = direction == fpga::TILE_PIN_INPUT
                ? tile_type.pin_map.input_nodes : tile_type.pin_map.output_nodes;
            for (int pin = 0; pin < 3; ++pin) {
                int local = 100 + 10 * site + 3 * direction + pin;
                nodes[resource + pin] = NodeMask{0, 1} << local;
                tile_type.pin_map.rememberEndpointRouteRef(direction, resource + pin,
                    local, "FABRIC", {site, 0});
            }
        }
    }
    auto first = std::make_unique<fpga::CBType>("FABRIC");
    auto second = std::make_unique<fpga::CBType>("FABRIC");
    first->local_input_nodes = NodeMask{0, 1} << 21;
    second->local_input_nodes = NodeMask{0, 1} << 22;
    fpga::Tile& tile = resetTile(tile_type);
    std::vector<NodeMask> expected;
    auto check_queries = [&](bool reference) {
        size_t index = 0;
        auto check = [&](NodeMask nodes) {
            if (reference) expected.push_back(nodes);
            else require(nodes == expected.at(index), "complete pin cache changed model lookup");
            ++index;
        };
        for (auto* cb : {first.get(), second.get()}) {
            tile.cb_type = cb;
            for (const std::string type : {"LUT5", "FDRE", "MUXF7"}) {
                for (const std::string pin : {"I0", "I4", "CE", "Q", "O"}) {
                    for (int pos : {0, 7, 64, 128}) {
                        check(tile.getPinNodes(type, pin, pos));
                        check(tile.getOutputPinNodes(type, pin, pos));
                        for (auto dir : {fpga::TILE_PIN_INPUT, fpga::TILE_PIN_OUTPUT}) {
                            for (const std::string route : {"", "FABRIC", "ABSENT"}) {
                                check(tile.getPinNodesForRouteType(type, pin, pos, dir, route, {0, 0}));
                                check(tile.getPinNodesForRouteType(type, pin, pos, dir, route, {1, 0}));
                            }
                        }
                    }
                }
            }
        }
        // Equal crossbar names must not alias different subtype fallback masks.
        tile.cb_type = first.get();
        require(tile.getPinNodes("LUT5", "I4", 0) == (NodeMask{0, 1} << 21),
            "first subtype fallback was lost");
        tile.cb_type = second.get();
        require(tile.getPinNodes("LUT5", "I4", 0) == NodeMask{},
            "pin cache confused crossbars with equal names");
    };
    check_queries(true);
    fpga::PinLookupCache cache;
    check_queries(false);
    check_queries(false);
    require(cache.hits > 0, "complete query regression did not use the cache");
}

void constant_controls_reserve_shared_input_endpoints()
{
    for (bool cached : {false, true}) {
        for (int first_signal : {-2, -1, 1}) {
            for (int second_signal : {-2, -1, 1}) {
                fpga::TileType tile_type = makePackingTileType();
                for (int site = 0; site < 2; ++site) {
                    int resource = 1 + site * 256;
                    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, resource, "CE");
                    tile_type.pin_map.input_nodes[resource].setBit(43 + site);
                    tile_type.pin_map.rememberEndpointRouteRef(
                        fpga::TILE_PIN_INPUT, resource, 43 + site, "CONTROL_FABRIC");
                }
                auto cb = std::make_unique<fpga::CBType>("CONTROL_FABRIC");
                auto& tile = resetTile(tile_type);
                tile.cb_type = cb.get();
                tile.cb.type = cb.get();
                Fixture fixture;
                auto* zero = fixture.makeInst("constant_zero", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
                auto* one = fixture.makeInst("constant_one", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
                auto* signal = fixture.makeInst("control_signal", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
                fixture.conn(zero, "O")->port_ref->designator = -1;
                fixture.conn(one, "O")->port_ref->designator = -2;
                fixture.conn(zero, "O")->port_ref->is_global = true;
                fixture.conn(one, "O")->port_ref->is_global = true;
                auto make_reg = [&](const std::string& name, int value) {
                    auto* reg = fixture.makeInst(name, "FDRE",
                        {{"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
                    if (value > 0) {
                        fixture.connect(signal, "O", reg, "CE");
                    } else {
                        // Constants have global drivers but no ordinary module-net entry.
                        auto* input = fixture.conn(reg, "CE");
                        input->port_ref->designator = value;
                        input->set(fixture.conn(value == -1 ? zero : one, "O"));
                    }
                    return reg;
                };
                auto* first = make_reg("first", first_signal);
                auto* second = make_reg("second", second_signal);
                std::optional<fpga::PinLookupCache> cache;
                if (cached) cache.emplace();
                const int first_pos = posFor(fpga::ELEMENT_FD, 8);
                const int next_pos = posFor(fpga::ELEMENT_FD, 9);
                {
                    fpga::ElementPackingPreview preview(tile);
                    require(preview.reserveAt(first, first_pos, false) >= 0,
                        "could not reserve first constant-control register");
                    auto free_before = tile.elements_free;
                    bool accepted = preview.reserveAt(second, next_pos, false) >= 0;
                    // The same physical CE cannot serve unequal drivers, in either insertion order.
                    require(accepted == (first_signal == second_signal),
                        "constant/signal control conflict was missed: "
                        + std::to_string(first_signal) + "," + std::to_string(second_signal));
                    if (!accepted) {
                        // Rejection must leave element slots and hypothetical ownership unchanged.
                        require(tile.elements_free == free_before && !second->tile.peer,
                            "rejected constant control changed placement state");
                    }
                    preview.rollback(0);
                    // Removing the first owner must release its shared input for a different driver.
                    require(preview.reserveAt(second, next_pos, false) >= 0,
                        "constant control reservation survived rollback");
                }
                // Real placement must enforce the same rule even with relaxed route-capacity checks.
                require(tile.tryAdd(first, false) == posFor(fpga::ELEMENT_FD, 0),
                    "could not place first control register");
                require(tile.tryAdd(second, false) == posFor(fpga::ELEMENT_FD,
                            first_signal == second_signal ? 1 : 8),
                    "real placement ignored a constant control owner");
                for (int floating : {-1, -2, -3, -4}) {
                    auto* unconnected = fixture.makeInst("unconnected", "FDRE",
                        {{"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
                    fixture.conn(unconnected, "CE")->port_ref->designator = floating;
                    fpga::ElementPackingPreview preview(tile);
                    // A negative number alone is not a connected constant and must not claim CE.
                    require(preview.reserveAt(unconnected, posFor(fpga::ELEMENT_FD, 2), false) >= 0,
                        "unconnected input was treated as a routed constant");
                }
            }
        }
    }
}

void shared_fd_control_endpoint_requires_one_driver_per_site(bool cached, bool clock_first = false)
{
    fpga::TileType tile_type = makePackingTileType();
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "CE");
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 257, "CE");
    tile_type.pin_map.input_nodes[1] = NodeMask{0,1} << 43;
    tile_type.pin_map.input_nodes[257] = NodeMask{0,1} << 42;
    tile_type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 1, 43, "ROUTE_FABRIC");
    tile_type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 257, 42, "ROUTE_FABRIC");
    if (clock_first) {
        // Use a model-defined clock name, not a primitive-specific port convention.
        for (auto& element : tile_type.elements) {
            if (element.type != fpga::ELEMENT_FD) continue;
            element.clock_group = element.bitmap_pos / 8;
            element.clock_port = "TICK";
        }
        for (int site = 0; site < 2; ++site) {
            int resource = 2 + site*256;
            tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, resource, "TICK");
            tile_type.pin_map.input_nodes[resource].setBit(40 + site);
            tile_type.pin_map.rememberEndpointRouteRef(
                fpga::TILE_PIN_INPUT, resource, 40 + site, "ROUTE_FABRIC");
        }
    }
    auto cb_type = std::make_unique<fpga::CBType>("ROUTE_FABRIC");

    fpga::Tile& tile = resetTile(tile_type);
    tile.cb_type = cb_type.get();
    tile.cb.type = cb_type.get();
    Fixture fixture;
    auto* driver_a = fixture.makeInst("driver_a", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver_b = fixture.makeInst("driver_b", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver_c = fixture.makeInst("driver_c", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto make_controlled_fd = [&](const std::string& name, Referable<rtl::Inst>* driver) {
        auto* fd = clock_first
            ? fixture.makeInst(name, "FDRE", {{"TICK", rtl::Port::PORT_IN},
                {"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}})
            : fixture.makeInst(name, "FDRE",
                {{"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
        fixture.connect(driver, "O", fd, "CE");
        if (clock_first) fixture.connect(driver_a, "O", fd, "TICK");
        return fd;
    };
    auto* fd_a0 = make_controlled_fd("fd_a0", driver_a);
    auto* fd_a1 = make_controlled_fd("fd_a1", driver_a);
    auto* fd_b = make_controlled_fd("fd_b", driver_b);
    auto* fd_c = make_controlled_fd("fd_c", driver_c);

    std::optional<fpga::PinLookupCache> cache;
    if (cached) cache.emplace();
    {
        fpga::ElementPackingPreview preview(tile);
        // Cached pin masks must not hide a new hypothetical endpoint owner.
        require(preview.reserveAt(fd_a0, posFor(fpga::ELEMENT_FD, 0), false) >= 0,
            "preview could not reserve first control endpoint");
        auto free_before = tile.elements_free;
        auto& tile_ref = static_cast<Referable<fpga::Tile>&>(tile);
        auto peers_before = tile_ref.peers.size();
        // Free register slots do not imply a free shared control input.
        require(preview.peek(fd_b, false) == posFor(fpga::ELEMENT_FD, 8),
            "packing admitted an independently owned control endpoint");
        if (clock_first) {
            // The conflicting-control probe deliberately never looked up this clock.
            require(tile.getPinNodesForRouteType("FDRE", "TICK",
                        posFor(fpga::ELEMENT_FD, 1), fpga::TILE_PIN_INPUT,
                        "ROUTE_FABRIC").testBit(40),
                "model-defined clock endpoint was not loaded");
        }
        size_t pin_queries = cached ? cache->hits + cache->misses : 0;
        int shared_pos = preview.peek(fd_a1, false);
        // With ownership already built, initial packing resolves this one
        // input once, not a union lookup followed by the same endpoint lookup.
        if (cached) require(cache->hits + cache->misses == pin_queries + (clock_first ? 2 : 1),
            "initial packing resolved the same input twice");
        // The same signal can use the next free slot; peeking leases nothing.
        require(shared_pos == posFor(fpga::ELEMENT_FD, 1)
                && tile.elements_free == free_before && tile_ref.peers.size() == peers_before
                && !fd_b->tile.peer && !fd_a1->tile.peer,
            "packing rejected shared controls or changed occupancy");
        pin_queries = cached ? cache->hits + cache->misses : 0;
        require(preview.reserveAt(fd_b, posFor(fpga::ELEMENT_FD, 1), false) < 0,
            "pin cache hid a conflicting preview endpoint owner");
        // A control conflict rejects the slot before looking up its compatible clock.
        if (cached) require(cache->hits + cache->misses == pin_queries + 1,
            "packing resolved the clock before rejecting a conflicting control");
        if (clock_first) {
            auto* wrong_clock = make_controlled_fd("wrong_clock", driver_a);
            fixture.connect(driver_b, "O", wrong_clock, "TICK");
            pin_queries = cached ? cache->hits + cache->misses : 0;
            // Compatible non-clock pins do not bypass the second, clock-input pass.
            require(preview.reserveAt(wrong_clock, posFor(fpga::ELEMENT_FD, 1), false) < 0,
                "packing skipped the deferred clock input");
            if (cached) require(cache->hits + cache->misses == pin_queries + 2,
                "packing did not validate both control and clock endpoints");
        }
        preview.rollback(0);
        // No stale occupancy result may survive undoing a speculative owner.
        require(preview.peek(fd_b, false) == posFor(fpga::ELEMENT_FD, 0),
            "packing retained the rolled-back endpoint owner");
        // Rollback frees ownership even though the immutable pin mask stays cached.
        require(preview.reserveAt(fd_b, posFor(fpga::ELEMENT_FD, 1), false) >= 0,
            "pin cache retained ownership after preview rollback");
    }

    // Same-signal controls may share the first site's physical endpoint.
    require(tile.tryAdd(fd_a0, false) == posFor(fpga::ELEMENT_FD, 0)
            && tile.tryAdd(fd_a1, false) == posFor(fpga::ELEMENT_FD, 1),
        "same control signal did not share one site endpoint");
    // A distinct control must skip all remaining positions in the first site.
    require(tile.tryAdd(fd_b, false) == posFor(fpga::ELEMENT_FD, 8),
        "independent control signal was not redirected to the second site");
    // Both site endpoints are now owned, although many FD element positions remain free.
    require(tile.tryAdd(fd_c, false) < 0,
        "third control signal illegally shared an occupied site endpoint");
    if (cached) require(cache->hits > 0, "packing regression did not exercise cached lookups");
}


void constant_controls_reserve_shared_inputs_before_routing(
    bool one, bool constant_first, bool enforce_capacity)
{
    auto tile_type = makePackingTileType();
    for (int site = 0; site < 2; ++site) {
        const int resource = 1 + site * 256;
        tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, resource, "CE");
        tile_type.pin_map.input_nodes[resource].setBit(42 + site);
        tile_type.pin_map.rememberEndpointRouteRef(
            fpga::TILE_PIN_INPUT, resource, 42 + site, "CONTROL_MATRIX");
    }
    auto cb_type = std::make_unique<fpga::CBType>("CONTROL_MATRIX");
    auto& tile = resetTile(tile_type);
    tile.cb_type = cb_type.get();
    tile.cb.type = cb_type.get();
    Fixture fixture;
    auto* constant = fixture.makeInst("constant", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
    auto* global = fixture.conn(constant, "O");
    global->port_ref->is_global = true;
    global->port_ref->designator = one ? -2 : -1;
    auto make_constant_fd = [&](const std::string& name) {
        auto* fd = fixture.makeInst(name, "FDRE",
            {{"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
        auto* input = fixture.conn(fd, "CE");
        input->port_ref->designator = global->port_ref->designator;
        input->set(global);
        return fd;
    };
    auto* tied = make_constant_fd("tied");
    auto* tied_sibling = make_constant_fd("tied_sibling");
    auto* opposite = fixture.makeInst("opposite", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
    auto* other_global = fixture.conn(opposite, "O");
    other_global->port_ref->is_global = true;
    other_global->port_ref->designator = one ? -1 : -2;
    auto* opposite_fd = make_constant_fd("opposite_fd");
    fixture.conn(opposite_fd, "CE")->set(other_global);
    fixture.conn(opposite_fd, "CE")->port_ref->designator = other_global->port_ref->designator;
    auto* driven = fixture.makeInst("driven", "FDRE",
        {{"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
    auto* driver = fixture.makeInst("driver", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
    fixture.connect(driver, "O", driven, "CE");
    auto* first = constant_first ? tied : driven;
    auto* second = constant_first ? driven : tied;
    {
        fpga::ElementPackingPreview preview(tile);
        require(preview.reserveAt(first, posFor(fpga::ELEMENT_FD, 0), enforce_capacity) >= 0,
            "could not reserve first control owner");
        require(preview.reserveAt(second, posFor(fpga::ELEMENT_FD, 1), enforce_capacity) < 0,
            "constant and dynamic controls illegally shared one physical input");
        require(preview.reserveAt(second, posFor(fpga::ELEMENT_FD, 8), enforce_capacity) >= 0,
            "independent constant control could not use the other site");
        const int sibling_bit = constant_first ? 1 : 9;
        require(preview.reserveAt(opposite_fd, posFor(fpga::ELEMENT_FD, sibling_bit), enforce_capacity) < 0,
            "opposite constants illegally shared one physical input");
        require(preview.reserveAt(tied_sibling, posFor(fpga::ELEMENT_FD, sibling_bit), enforce_capacity) >= 0,
            "identical global constants could not share their reserved input");
    }
    require(!first->tile.peer && !second->tile.peer && !tied_sibling->tile.peer,
        "constant-control preview leaked placement");
    require(tile.tryAdd(first, enforce_capacity) == posFor(fpga::ELEMENT_FD, 0) &&
                tile.tryAdd(second, enforce_capacity) == posFor(fpga::ELEMENT_FD, 8),
        "committed packing ignored the constant control reservation");
}

void unreachable_entries_do_not_hide_mandatory_joint_in_either_order()
{
    auto run_order = [](bool reverse) {
        fpga::TileType tile_type = makePackingTileType();
        tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "A1");
        tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "B1");
        tile_type.pin_map.input_nodes[1] = NodeMask{0,1} << 90;
        tile_type.pin_map.input_nodes[2] = NodeMask{0,1} << 91;

        auto cb_type = std::make_unique<fpga::CBType>("MATRIX_PHYSICAL_INPUTS");
        // Physical entries 10/11 require the shared joint. Unreachable entries
        // 12/13 are direct and must not make that joint appear optional.
        cb_type->dst_joint[10].joint |= NodeMask{0,1} << 15;
        cb_type->dst_joint[11].joint |= NodeMask{0,1} << 15;
        cb_type->joint_local[15].local |= (NodeMask{0,1} << 90) | (NodeMask{0,1} << 91);
        cb_type->dst_local[12].local |= NodeMask{0,1} << 90;
        cb_type->dst_local[13].local |= NodeMask{0,1} << 91;
        cb_type->rebuildOutgoingSrcs();

        fpga::Tile& tile = resetTile(tile_type);
        tile.cb_type = cb_type.get();
        tile.cb.type = cb_type.get();
        tile.incoming_dst_nodes = (NodeMask{0,1} << 10) | (NodeMask{0,1} << 11);
        Fixture fixture;
        occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {2, 3, 4, 5, 6, 7}, -1);
        auto* driver0 = fixture.makeInst("driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
        auto* driver1 = fixture.makeInst("driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
        auto* left = makeInputLut(fixture, "left");
        auto* right = makeInputLut(fixture, "right");
        fixture.connect(driver0, "O", left, "I0");
        fixture.connect(driver1, "O", right, "I0");

        Referable<rtl::Inst>* first = reverse ? right : left;
        Referable<rtl::Inst>* second = reverse ? left : right;
        require(tile.tryAdd(first) >= 0,
            "first physical-input endpoint did not pack");
        // Check: insertion order cannot bypass an unavoidable shared endpoint joint.
        require(tile.tryAdd(second) < 0,
            "unreachable direct entry hid a mandatory joint conflict");
    };

    run_order(false);
    run_order(true);
}

void attached_resource_tiles_share_mandatory_joint_ownership()
{
    fpga::TileType tile_type = makePackingTileType();
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "A1");
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "B1");
    tile_type.pin_map.input_nodes[1] = NodeMask{0,1} << 90;
    tile_type.pin_map.input_nodes[2] = NodeMask{0,1} << 91;

    auto cb_type = std::make_unique<fpga::CBType>("GENERIC_ROUTE");
    cb_type->dst_joint[10].joint |= NodeMask{0,1} << 15;
    cb_type->dst_joint[11].joint |= NodeMask{0,1} << 15;
    cb_type->joint_local[15].local |= (NodeMask{0,1} << 90) | (NodeMask{0,1} << 91);
    cb_type->rebuildOutgoingSrcs();

    auto [left, right] = resetTwoTiles(tile_type);
    left.cb_coord = {0, 0};
    right.cb_coord = {0, 0};
    left.cb_type = right.cb_type = cb_type.get();
    left.cb.type = right.cb.type = cb_type.get();
    Fixture fixture;
    occupyOtherBits(left, fixture, fpga::ELEMENT_LUT5, {1, 2, 3, 4, 5, 6, 7}, -1);
    occupyOtherBits(right, fixture, fpga::ELEMENT_LUT5, {0, 2, 3, 4, 5, 6, 7}, -1);
    auto* driver0 = fixture.makeInst("driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver1 = fixture.makeInst("driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* lut0 = makeInputLut(fixture, "lut0");
    auto* lut1 = makeInputLut(fixture, "lut1");
    fixture.connect(driver0, "O", lut0, "I0");
    fixture.connect(driver1, "O", lut1, "I0");

    require(left.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first attached-resource input did not use the expected slot");
    require(right.tryAdd(lut1) < 0,
        "resource tiles sharing one crossbar packed unrelated mandatory-joint inputs");
}

void exact_route_endpoint_cannot_be_hidden_by_other_route_locals()
{
    fpga::TileType tile_type = makePackingTileType();
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "A1");
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "B1");
    tile_type.pin_map.input_nodes[1] = (NodeMask{0,1} << 90) | (NodeMask{0,1} << 100);
    tile_type.pin_map.input_nodes[2] = (NodeMask{0,1} << 91) | (NodeMask{0,1} << 101);
    tile_type.pin_map.rememberEndpointRouteRef(
        fpga::TILE_PIN_INPUT, 1, 90, "ROUTE_MATRIX", {1, 0});
    tile_type.pin_map.rememberEndpointRouteRef(
        fpga::TILE_PIN_INPUT, 2, 91, "ROUTE_MATRIX", {1, 0});
    tile_type.pin_map.rememberEndpointRouteRef(
        fpga::TILE_PIN_INPUT, 1, 100, "OTHER_MATRIX", {0, 1});
    tile_type.pin_map.rememberEndpointRouteRef(
        fpga::TILE_PIN_INPUT, 2, 101, "OTHER_MATRIX", {0, 1});

    auto cb_type = std::make_unique<fpga::CBType>("ROUTE_MATRIX");
    cb_type->dst_joint[10].joint |= NodeMask{0,1} << 15;
    cb_type->dst_joint[11].joint |= NodeMask{0,1} << 15;
    cb_type->joint_local[15].local |= (NodeMask{0,1} << 90) | (NodeMask{0,1} << 91);
    cb_type->rebuildOutgoingSrcs();

    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.tile_grid.resize(3); // Change the grid generation used by the route-tile lookup cache.
    device.size_width = 3;
    device.size_height = 1;
    device.grid_spec.size = {3, 1};
    fpga::Tile& resource = device.tile_grid[0];
    fpga::Tile& route = device.tile_grid[1];
    resource.coord = resource.name = {0, 0};
    route.coord = route.name = {1, 0};
    resource.tile_type = route.tile_type = &tile_type;
    resource.cb_coord = route.coord;
    route.cb_coord = route.coord;
    resource.cb_type = route.cb_type = cb_type.get();
    resource.cb.type = route.cb.type = cb_type.get();
    route.incoming_dst_nodes = (NodeMask{0,1} << 10) | (NodeMask{0,1} << 11);

    Fixture fixture;
    occupyOtherBits(resource, fixture, fpga::ELEMENT_LUT5, {2, 3, 4, 5, 6, 7}, -1);
    auto* driver0 = fixture.makeInst("driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver1 = fixture.makeInst("driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* lut0 = makeInputLut(fixture, "lut0");
    auto* lut1 = makeInputLut(fixture, "lut1");
    fixture.connect(driver0, "O", lut0, "I0");
    fixture.connect(driver1, "O", lut1, "I0");

    require(resource.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first exact route endpoint did not pack");
    // Check: locals annotated for another route crossbar cannot make the shared
    // mandatory joint on the attached route crossbar appear optional.
    require(resource.tryAdd(lut1) < 0,
        "an unrelated route-type local hid an exact mandatory-joint conflict");
}

void equal_local_on_different_route_types_is_not_an_alias()
{
    fpga::TileType tile_type = makePackingTileType();
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "A1");
    tile_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "B1");
    tile_type.pin_map.input_nodes[1] = NodeMask{0,1} << 97;
    tile_type.pin_map.input_nodes[2] = NodeMask{0,1} << 97;
    tile_type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 1, 97, "ROUTE_A");
    tile_type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 2, 97, "ROUTE_B");

    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    occupyOtherBits(tile, fixture, fpga::ELEMENT_LUT5, {2, 3, 4, 5, 6, 7}, -1);
    auto* driver0 = fixture.makeInst("driver0", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* driver1 = fixture.makeInst("driver1", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* lut0 = makeInputLut(fixture, "lut0");
    auto* lut1 = makeInputLut(fixture, "lut1");
    fixture.connect(driver0, "O", lut0, "I0");
    fixture.connect(driver1, "O", lut1, "I0");

    require(tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first route-endpoint LUT input placement did not use expected slot");
    require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 1),
        "same local number on different route types was treated as an alias");
}

void colliding_resource_ids_keep_distinct_pin_identity()
{
    fpga::TilePinMap pin_map;
    int first = pin_map.distinctResourceNode(fpga::TILE_PIN_OUTPUT, 17, "PIN_ALPHA");
    pin_map.rememberResourcePinName(fpga::TILE_PIN_OUTPUT, first, "PIN_ALPHA");
    int second = pin_map.distinctResourceNode(fpga::TILE_PIN_OUTPUT, 17, "PIN_BETA");
    pin_map.rememberResourcePinName(fpga::TILE_PIN_OUTPUT, second, "PIN_BETA");

    require(first == 17, "first endpoint did not retain the proposed resource ID");
    require(second != first, "different endpoint names were merged at one resource ID");
    require(pin_map.distinctResourceNode(fpga::TILE_PIN_OUTPUT, 17, "PIN_BETA") == second,
        "repeated endpoint name did not retain its disambiguated resource ID");
    require(pin_map.resourceNodesForPin(fpga::TILE_PIN_OUTPUT, "PIN_ALPHA") == std::vector<int>{first},
        "first endpoint lookup included a colliding pin");
    require(pin_map.resourceNodesForPin(fpga::TILE_PIN_OUTPUT, "PIN_BETA") == std::vector<int>{second},
        "second endpoint lookup included a colliding pin");
}

void optional_two_joint_entry_preserves_other_packed_input_reservation()
{
    // The first input can use joints 4+18 or an alternate path; the second input always needs 18.
    NodeMask reserved = NodeMask{0,1} << 18;
    require(!pnr::terminalEntryAvoidsReservedJoints(4, 18, reserved),
        "two-joint terminal path consumed another packed input's mandatory joint");
    require(pnr::terminalEntryAvoidsReservedJoints(5, -1, reserved),
        "independent terminal alternative was rejected by an unrelated reservation");
    require(pnr::terminalEntryAvoidsReservedJoints(-1, -1, reserved),
        "direct terminal path was rejected by a joint reservation");
}

void forced_fabric_input_does_not_require_a_local_element_chain()
{
    fpga::TileType tile_type = makePackingTileType();
    auto [source_tile, sink_tile] = resetTwoTiles(tile_type);
    Fixture fixture;
    auto* source = makeLut(fixture, "fabric_source");
    auto* sink = fixture.makeInst("fabric_sink", "FDRE",
        {{"C", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
    sink->cell_ref->attributes["scalepnr_force_fabric_input"] = "1";
    fixture.connect(source, "O", sink, "C");

    placeManual(sink_tile, sink, fpga::ELEMENT_FD, 0);
    // Check: an explicitly fabric-routed endpoint on another tile must not be
    // interpreted as a mandatory local LUT-to-FD packing chain.
    require(source_tile.tryAdd(source) >= 0,
        "fabric-routed sink incorrectly forced its driver into the same tile");
}

void generic_inverter_uses_the_primary_lut_element_model()
{
    fpga::TileType tile_type = makePackingTileType();
    auto [tile, unused] = resetTwoTiles(tile_type);
    Fixture fixture;
    auto* inverter = fixture.makeInst("generic_inverter", "INV",
        {{"I", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});

    // A primitive alias supported by the abstract primary-LUT model must
    // reach Tile::tryAdd instead of being silently omitted by PlaceDesign.
    require(fpga::isPlaceableElement(*inverter) && tile.tryAdd(inverter) >= 0,
        "generic inverter was not placed as a one-input element");
    // Its generic I/O names resolve to the same numeric endpoints as LUT6 I0/O.
    require(tile.getNodeNum("INV", "I", inverter->pos)
                == tile.getNodeNum("LUT6", "I0", inverter->pos)
            && tile.getNodeNum("INV", "O", inverter->pos)
                == tile.getNodeNum("LUT6", "O", inverter->pos),
        "generic inverter pins did not use the primary LUT endpoints");
}

void wide_mux_output_uses_its_distinct_middle_lane()
{
    fpga::TileType tile_type = makePackingTileType();
    auto [tile, unused] = resetTwoTiles(tile_type);
    int f7_pos = posFor(fpga::ELEMENT_MUXF7, 0);
    int f8_pos = posFor(fpga::ELEMENT_MUXF8, 0);

    // The wide mux output is a separate endpoint between its two narrow muxes.
    require(tile.getNodeNum("MUXF8", "O", f8_pos)
                != tile.getNodeNum("MUXF7", "O", f7_pos),
        "wide mux output aliases the first narrow-mux output lane");
    // Endpoint lookup and pin-mask lookup must agree on that output identity.
    require(tile.getPinNodes("MUXF8", "O", f8_pos)
                != tile.getPinNodes("MUXF7", "O", f7_pos),
        "wide mux output pin mask aliases the first narrow-mux output lane");
}

void radial_placement_search_covers_the_complete_grid()
{
    for (int width : {1, 7, 31}) {
        for (int height : {1, 9, 43}) {
            for (Coord origin : {Coord{0, 0}, Coord{width - 1, height - 1},
                                 Coord{width / 2, height / 2}}) {
                Coord cursor = origin;
                int dir = 0;
                int steps = 1;
                int pos = 0;
                std::vector<bool> seen(static_cast<size_t>(width*height));
                for (size_t i = 0; i < radialSearchCoverageSteps(origin, width, height); ++i) {
                    if (cursor.x >= 0 && cursor.x < width
                        && cursor.y >= 0 && cursor.y < height) {
                        seen[static_cast<size_t>(cursor.y*width + cursor.x)] = true;
                    }
                    radialSearch(cursor, dir, steps, pos);
                }
                require(std::ranges::all_of(seen, [](bool value) { return value; }),
                    "radial placement search did not cover the complete device grid");
            }
        }
    }
}

void element_packing_preview_is_exact_and_non_destructive()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* first = makeFd(fixture, "preview_first");
    auto* second = makeFd(fixture, "preview_second");
    tile.hasFreeElement(fpga::ELEMENT_FD);
    auto free_before = tile.elements_free;

    int peeked = tile.peekAdd(first, false);
    require(peeked >= 0 && !first->tile.peer && first->pos == -1
                && tile.elements_free == free_before,
        "Tile::peekAdd changed placement state");

    std::vector<fpga::ElementPackingChoice> choices;
    std::vector<rtl::Inst*> members{first, second};
    require(fpga::ElementPackingPreview::orderPack(members), "could not order independent cells");
    {
        fpga::ElementPackingPreview preview(tile);
        require(preview.reservePack(members, choices, false)
                    && choices.size() == 2
                    && choices[0].pos != choices[1].pos,
            "Element preview did not reserve two exact distinct positions");
        require(first->tile.peer == &tile && second->tile.peer == &tile,
            "Element preview did not expose hypothetical occupants to packing");
    }
    require(!first->tile.peer && !second->tile.peer
                && first->pos == -1 && second->pos == -1
                && tile.elements_free == free_before,
        "Element preview did not restore Tile and instance state");
    require(tile.tryAddAt(first, choices[0].pos) == choices[0].pos
                && tile.tryAddAt(second, choices[1].pos) == choices[1].pos,
        "real packing rejected positions accepted by the exact preview");
}

void moved_helper_output_checks_candidate_lane_not_stale_position()
{
    for (bool conflict : {false, true}) {
        for (int old_helper_pos : {-1, 1, 129}) {
            auto type = makePackingTileType();
            auto output = [&](int resource, const char* pin, int local) {
                type.pin_map.rememberResourcePinName(fpga::TILE_PIN_OUTPUT, resource, pin);
                type.pin_map.output_nodes[resource].setBit(local);
                type.pin_map.rememberEndpointRouteRef(
                    fpga::TILE_PIN_OUTPUT, resource, local, "FABRIC");
            };
            // Swap the alias between old and candidate sites to catch both false
            // rejection and false acceptance. All connections are synthetic.
            output(0, "C", 200);
            output(1, "AMUX", conflict ? 201 : 200);
            output(256, "AMUX", conflict ? 200 : 201);
            output(257, "A", 202);
            auto cb = std::make_unique<fpga::CBType>("FABRIC");
            auto& tile = resetTile(type);
            tile.cb_type = cb.get();
            tile.cb.type = cb.get();
            Fixture fixture;
            auto* owner = makeLut(fixture, "existing_output");
            auto* external = makeFd(fixture, "external_load");
            auto* lut = makeLut(fixture, "moving_logic");
            auto* helper = makeF7(fixture, "moving_helper");
            auto* sink = makeFd(fixture, "helper_load");
            helper->cell_ref->attributes["scalepnr_passthrough"] = "source";
            fixture.connect(owner, "O", external, "D");
            fixture.connect(lut, "O", helper, "I1");
            fixture.connect(helper, "O", sink, "D");
            const int owner_pos = posFor(fpga::ELEMENT_LUT5, 2);
            const int lut_pos = posFor(fpga::ELEMENT_LUT5, 4);
            const int helper_pos = posFor(fpga::ELEMENT_MUXF7, 4);
            require(tile.tryAddAt(owner, owner_pos, false) == owner_pos,
                "could not place independent output owner");
            const auto free_before = tile.elements_free;
            {
                fpga::ElementPackingPreview preview(tile);
                // The LUT check must consider its unplaced helper's future output.
                require((preview.reserveAt(lut, lut_pos, false) >= 0) == !conflict,
                    "preview did not use the future helper's candidate output");
                if (!conflict) {
                    require(preview.reserveAt(helper, helper_pos, false) == helper_pos,
                        "preview could not complete the connected helper chain");
                }
            }
            // A preview must leave no speculative placement or occupancy behind.
            require(!lut->tile.peer && !helper->tile.peer
                        && lut->pos == -1 && helper->pos == -1
                        && tile.elements_free == free_before,
                "helper preview leaked placement or element occupancy");

            // Moving clears tile references but can retain the previous positions.
            // These values must not override the candidate lane during commit.
            lut->pos = posFor(fpga::ELEMENT_LUT5, 0);
            helper->pos = old_helper_pos;
            require((tile.tryAddAt(lut, lut_pos, false) >= 0) == !conflict,
                "commit used the helper's stale output position instead of its candidate lane");
            if (conflict) {
                // Genuine output aliasing must still fail without modifying occupancy.
                require(!lut->tile.peer && !helper->tile.peer
                            && tile.elements_free == free_before,
                    "rejected helper output conflict changed placement state");
            } else {
                // Direct helper placement must use the same candidate identity too.
                require(tile.tryAddAt(helper, helper_pos, false) == helper_pos,
                    "helper commit disagreed with the accepted preview");
                require(lut->tile.peer == &tile && helper->tile.peer == &tile
                            && lut->pos == lut_pos && helper->pos == helper_pos
                            && !(tile.elements_free[fpga::ELEMENT_LUT5] & bit16(4))
                            && !(tile.elements_free[fpga::ELEMENT_MUXF7] & bit16(4)),
                    "committed helper chain did not occupy its candidate lanes");
            }
            // Neither accepting nor rejecting the move may disturb the existing owner.
            require(owner->tile.peer == &tile && owner->pos == owner_pos
                        && !(tile.elements_free[fpga::ELEMENT_LUT5] & bit16(2)),
                "helper placement disturbed the independent output owner");
        }
    }
}

void first_fit_orders_dependencies_without_position_backtracking()
{
    fpga::TileType type = makePackingTileType();
    fpga::Tile& tile = resetTile(type);
    Fixture fixture;
    auto* a = makeLut6(fixture, "restore_chain_a");
    auto* b = makeLut6(fixture, "restore_chain_b");
    auto* c = makeLut6(fixture, "restore_chain_c");
    auto* d = makeLut6(fixture, "restore_chain_d");
    auto* ab = makeF7(fixture, "restore_chain_ab");
    auto* cd = makeF7(fixture, "restore_chain_cd");
    auto* out = makeF8(fixture, "restore_chain_out");
    fixture.connect(a, "O", ab, "I0");
    fixture.connect(b, "O", ab, "I1");
    fixture.connect(c, "O", cd, "I0");
    fixture.connect(d, "O", cd, "I1");
    fixture.connect(ab, "O", out, "I1");
    fixture.connect(cd, "O", out, "I0");
    // Consumers arrive first; preparing dependencies must not itself place anything.
    std::vector<rtl::Inst*> members{out, cd, ab, a, b, c, d};
    require(fpga::ElementPackingPreview::orderPack(members)
                && members == std::vector<rtl::Inst*>{a, b, ab, c, d, cd, out},
        "packing order did not keep the first ready member ahead of later ones");
    for (auto* member : members) require(!member->tile.peer, "ordering changed placement");
    tile.hasFreeElement(fpga::ELEMENT_LUT5);
    const auto free_before = tile.elements_free;
    std::vector<fpga::ElementPackingChoice> choices;
    {
        fpga::ElementPackingPreview preview(tile);
        require(preview.reservePack(members, choices, false) && choices.size() == members.size(),
            "first-fit failed to pack the ordered local chain");
        // The choices are the first legal lanes, not a rearranged packing.
        require(a->pos == posFor(fpga::ELEMENT_LUT5, 1)
                    && b->pos == posFor(fpga::ELEMENT_LUT5, 0)
                    && c->pos == posFor(fpga::ELEMENT_LUT5, 3)
                    && d->pos == posFor(fpga::ELEMENT_LUT5, 2)
                    && ab->pos == posFor(fpga::ELEMENT_MUXF7, 0)
                    && cd->pos == posFor(fpga::ELEMENT_MUXF7, 2)
                    && out->pos == posFor(fpga::ELEMENT_MUXF8, 0),
            "first-fit changed an earlier choice or ignored dedicated lane order");
    }
    // Successful previews are reversible too, before the real commit.
    require(tile.elements_free == free_before, "chain preview leaked occupancy");
    for (const auto& choice : choices)
        require(tile.tryAddAt(choice.inst, choice.pos, false) == choice.pos,
            "real placement rejected a first-fit chain reservation");

    auto* reg = makeFd(fixture, "chain_capture");
    fixture.connect(out, "O", reg, "D");
    const int reg_pos = posFor(fpga::ELEMENT_FD, 0);
    require(tile.tryAddAt(reg, reg_pos, false) == reg_pos,
        "could not extend the local chain to a register");
    choices.push_back({reg, reg_pos});
    const auto committed_free = tile.elements_free;
    // Try every subset, including holes in the middle of a live chain.
    // Restore in dependency order while unrelated members retain their slots.
    for (unsigned mask = 1; mask < (1u << choices.size()); ++mask) {
        for (size_t i = choices.size(); i-- > 0;)
            if (mask & (1u << i)) tile.unassign(choices[i].inst);
        unsigned pending = mask;
        while (pending) {
            const unsigned before = pending;
            for (size_t i = 0; i < choices.size(); ++i) {
                if (!(pending & (1u << i))) continue;
                if (tile.tryAddAt(choices[i].inst, choices[i].pos, false) == choices[i].pos)
                    pending &= ~(1u << i);
            }
            require(pending != before,
                "legal local-chain subset could not be restored, mask=" + std::to_string(mask));
        }
        // Exact masks and positions, not merely an equal number of cells, must survive.
        require(tile.elements_free == committed_free, "local-chain restore changed occupied lanes");
        for (const auto& choice : choices)
            require(choice.inst->tile.peer == &tile && choice.inst->pos == choice.pos,
                "local-chain restore moved an original occupant");
    }
    // An empty intermediate lane is not permission to bypass an unrelated
    // register: the restored chain must still drive the occupied endpoint.
    tile.unassign(out);
    tile.unassign(ab);
    tile.unassign(a);
    auto* unrelated = makeLut6(fixture, "unrelated_capture_driver");
    fixture.connect(unrelated, "O", reg, "D");
    tile.hasFreeElement(fpga::ELEMENT_LUT5);
    const auto hole_free = tile.elements_free;
    require(tile.tryAddAt(a, choices[0].pos, false) < 0,
        "empty mux lane hid an unrelated occupied register");
    require(tile.elements_free == hole_free && !a->tile.peer,
        "rejected chain restore changed occupancy");
}

void nearer_column_does_not_hide_another_lane_conflict()
{
    std::array<int, 3> order{0, 1, 2};
    do {
        fpga::TileType type = makePackingTileType();
        fpga::Tile& tile = resetTile(type);
        Fixture fixture;
        auto* unrelated = makeLut(fixture, "unrelated_lane_zero");
        auto* driver = makeLut6(fixture, "full_lut_lane_one");
        auto* reg = makeFd(fixture, "shared_chain_register");
        fixture.connect(driver, "O", reg, "D");
        std::array<rtl::Inst*, 3> cells{unrelated, driver, reg};
        std::array<int, 3> positions{posFor(fpga::ELEMENT_LUT5, 0),
            posFor(fpga::ELEMENT_LUT5, 1), posFor(fpga::ELEMENT_FD, 0)};
        int accepted = 0;
        for (int index : order)
            accepted += tile.tryAddAt(cells[index], positions[index], false) >= 0;
        // The full LUT's auxiliary lane is compatible, but must not hide the
        // unrelated primary LUT on the other path to this register.
        require(accepted != 3, "nearer compatible lane concealed a farther incompatible lane");
    } while (std::next_permutation(order.begin(), order.end()));
}

void first_fit_failure_restores_only_current_bunch()
{
    fpga::TileType type = makePackingTileType();
    type.elements.clear();
    for (int bit : {0, 8, 9}) type.elements.push_back(makeElement("REG", fpga::ELEMENT_FD, bit));
    for (int site = 0; site < 2; ++site) {
        type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1 + site*256, "CE");
        type.pin_map.input_nodes[1 + site*256] = NodeMask{0,1} << (42 + site);
        type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 1 + site*256,
                                              42 + site, "FABRIC");
    }
    auto cb = std::make_unique<fpga::CBType>("FABRIC");
    auto [tile, next_tile] = resetTwoTiles(type);
    tile.cb_type = next_tile.cb_type = cb.get();
    tile.cb.type = next_tile.cb.type = cb.get();
    Fixture fixture;
    auto* b = fixture.makeInst("signal_b", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto* c = fixture.makeInst("signal_c", "DRIVER", {{"O", rtl::Port::PORT_OUT}});
    auto make_reg = [&](const std::string& name, Referable<rtl::Inst>* driver) {
        auto* reg = fixture.makeInst(name, "FDRE", {{"CE", rtl::Port::PORT_IN}});
        fixture.connect(driver, "O", reg, "CE");
        return reg;
    };
    auto* seed = make_reg("earlier_bunch", b);
    auto* first = make_reg("flexible", b);
    auto* last = make_reg("constrained", c);
    fpga::ElementPackingPreview preview(tile);
    require(preview.reserveAt(seed, posFor(fpga::ELEMENT_FD, 8), false) >= 0,
        "could not reserve earlier bunch");
    const auto free_before = tile.elements_free;
    std::vector<rtl::Inst*> members{first, last};
    require(fpga::ElementPackingPreview::orderPack(members)
                && members == std::vector<rtl::Inst*>{first, last},
        "independent cells were reordered");
    std::vector<fpga::ElementPackingChoice> choices{{seed, seed->pos}};
    // First takes site 0; the last cell then conflicts with both control owners.
    // Rearranging first to site 1 would work, but first-fit must not search that.
    require(!preview.reservePack(members, choices, false), "packing unexpectedly backtracked");
    require(preview.checkpoint() == 1 && choices.size() == 1
                && seed->tile.peer == &tile && seed->pos == posFor(fpga::ELEMENT_FD, 8)
                && !first->tile.peer && !last->tile.peer
                && first->pos == -1 && last->pos == -1 && tile.elements_free == free_before,
        "failed bunch damaged earlier reservations or leaked element occupancy");
    // Rebuilding ownership after rollback must also release the failed input owner.
    require(preview.peek(last, false) == posFor(fpga::ELEMENT_FD, 0),
        "failed bunch retained a control-input owner");
    require(preview.reserveAt(first, posFor(fpga::ELEMENT_FD, 9), false) >= 0
                && preview.reserveAt(last, posFor(fpga::ELEMENT_FD, 0), false) >= 0,
        "fixture did not contain the alternative packing that first-fit must skip");
    preview.rollback(1);
    // The caller may now try another tile without changing the fixed cell order.
    fpga::ElementPackingPreview next(next_tile);
    require(next.reservePack(members, choices, false)
                && first->tile.peer == &next_tile && last->tile.peer == &next_tile,
        "rolled-back bunch could not pack at the next tile");
}

void output_typed_input_connection_is_not_traversed_as_driver()
{
    fpga::TileType tile_type = makePackingTileType();
    fpga::Tile& tile = resetTile(tile_type);
    Fixture fixture;
    auto* source = makeLut(fixture, "source");
    auto* output_sink = makeLut(fixture, "output_sink");

    // Top-level output mappings can retain an output-typed port while the
    // connection itself points upstream. It is a load, not a sink-list owner.
    fixture.connect(source, "O", output_sink, "O");
    require(fixture.conn(output_sink, "O")->peer != nullptr,
        "regression fixture did not create an output-typed input connection");
    require(tile.peekAdd(output_sink, false) >= 0,
        "packing rejected an output-typed input connection");
}

}

int main()
{
    try {
        lut_to_f7_requires_connectivity();
        f7_to_f8_requires_connectivity();
        connected_f7_f8_chain_rejects_other_tile();
        connected_lut_f7_chain_rejects_other_tile();
        unplaced_strict_chain_sink_reserves_future_lane();
        unplaced_strict_sink_avoids_occupied_future_blockers();
        unplaced_mux_sink_keeps_all_drivers_in_one_tile();
        unplaced_mux_sink_requires_shared_driver_lane();
        unplaced_mux_sink_requires_free_future_driver_lane();
        mux_sink_waits_for_all_strict_drivers();
        unplaced_f7_f8_sink_reserves_future_lane();
        lut6_pair_into_f7_reserves_future_f8_lane();
        future_f8_lane_requires_packable_sibling_f7();
        future_f8_lane_rejects_unconnected_occupied_sibling_f7_blockers();
        f8_to_fd_requires_connectivity();
        tile_type_has_sixteen_fd_positions();
        carry_chain_refresh_still_marks_internal_nets();
        distant_fd_conflicts_are_found_through_free_carry();
        distant_connected_lut_fd_pairs_and_independent_fds_pack();
        independent_lut1_does_not_block_distant_fd();
        chained_lut1_must_be_connected_for_distant_fd();
        independent_inputs_must_not_alias_one_local_node();
        independent_inputs_must_not_alias_one_mandatory_joint();
        complete_pin_queries_match_uncached_models();
        constant_controls_reserve_shared_input_endpoints();
        shared_fd_control_endpoint_requires_one_driver_per_site(false);
        shared_fd_control_endpoint_requires_one_driver_per_site(true);
        shared_fd_control_endpoint_requires_one_driver_per_site(false, true);
        shared_fd_control_endpoint_requires_one_driver_per_site(true, true);
        for (bool one : {false, true}) {
            for (bool constant_first : {false, true}) {
                for (bool enforce_capacity : {false, true}) {
                    constant_controls_reserve_shared_inputs_before_routing(
                        one, constant_first, enforce_capacity);
                }
            }
        }
        unreachable_entries_do_not_hide_mandatory_joint_in_either_order();
        attached_resource_tiles_share_mandatory_joint_ownership();
        exact_route_endpoint_cannot_be_hidden_by_other_route_locals();
        equal_local_on_different_route_types_is_not_an_alias();
        colliding_resource_ids_keep_distinct_pin_identity();
        optional_two_joint_entry_preserves_other_packed_input_reservation();
        forced_fabric_input_does_not_require_a_local_element_chain();
        generic_inverter_uses_the_primary_lut_element_model();
        wide_mux_output_uses_its_distinct_middle_lane();
        radial_placement_search_covers_the_complete_grid();
        element_packing_preview_is_exact_and_non_destructive();
        moved_helper_output_checks_candidate_lane_not_stale_position();
        first_fit_orders_dependencies_without_position_backtracking();
        nearer_column_does_not_hide_another_lane_conflict();
        first_fit_failure_restores_only_current_bunch();
        output_typed_input_connection_is_not_traversed_as_driver();
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "packing_test failed: %s\n", failure.message.c_str());
        return 1;
    }
    catch (const std::exception& ex) {
        std::fprintf(stderr, "packing_test exception: %s\n", ex.what());
        return 1;
    }
    return 0;
}
