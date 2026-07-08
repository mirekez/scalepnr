#include "Device.h"
#include "RegBunch.h"
#include "TimingPath.h"
#include "Tile.h"
#include "Cell.h"
#include "Conn.h"
#include "Module.h"

#include <cstdio>
#include <memory>
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
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = {0, 0};
    tile.name = {0, 0};
    tile.tile_type = &tile_type;
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
    fpga::Tile& left = device.tile_grid[0];
    left.coord = {0, 0};
    left.name = {0, 0};
    left.tile_type = &tile_type;
    left.elements_initialized = false;
    left.elements_pos = {};
    left.elements_free = {};
    left.elements_left = {};
    left.elements_right = {};

    fpga::Tile& right = device.tile_grid[1];
    right.coord = {1, 0};
    right.name = {1, 0};
    right.tile_type = &tile_type;
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
            fixture.connect(lut, "O", f7, "I0");
            placeManual(tile, lut, fpga::ELEMENT_LUT5, lut_bit);
            occupyOtherBits(tile, fixture, fpga::ELEMENT_MUXF7, {0, 2, 4, 6}, f7_bit);
            int pos = tile.tryAdd(f7);
            require(pos == posFor(fpga::ELEMENT_MUXF7, f7_bit), "connected LUT->MUXF7 was not packed");
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
            fixture.connect(f7, "O", f8, "I0");
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
    fixture.connect(f7, "O", f8, "I0");

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
    fixture.connect(lut, "O", f7, "I0");

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

        require(tile.tryAdd(lut) == posFor(fpga::ELEMENT_LUT5, 2),
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

    require(chain_tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first LUT driver did not pack into expected strict-chain lane");
    require(other_tile.tryAdd(lut1) < 0,
        "second LUT driver of an unplaced MUXF7 was allowed to split to another tile");
    require(chain_tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 1),
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

    placeManual(tile, lut0, fpga::ELEMENT_LUT5, 0);

    require(tile.tryAdd(f7) < 0,
        "MUXF7 packed before all strict LUT drivers were placed");
    require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 1),
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

    require(tile.tryAdd(f7) == posFor(fpga::ELEMENT_MUXF7, 4),
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
    fixture.connect(f7, "O", f8, "I0");
    fixture.connect(sibling_f7, "O", f8, "I1");

    require(tile.tryAdd(lut0) == posFor(fpga::ELEMENT_LUT5, 0),
        "first LUT6 driver did not reserve a legal MUXF7 lane");
    require(tile.tryAdd(lut1) == posFor(fpga::ELEMENT_LUT5, 1),
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
    fixture.connect(f7, "O", f8, "I0");
    fixture.connect(sibling_f7, "O", f8, "I1");

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
    fixture.connect(f7, "O", f8, "I0");
    fixture.connect(sibling_f7, "O", f8, "I1");

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
        "first LUT input placement did not use expected slot");
    require(tile.tryAdd(lut1) < 0,
        "independent LUT inputs were packed onto the same routed local node");
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

}

int main()
{
    try {
        lut_to_f7_requires_connectivity();
        f7_to_f8_requires_connectivity();
        connected_f7_f8_chain_rejects_other_tile();
        connected_lut_f7_chain_rejects_other_tile();
        unplaced_strict_chain_sink_reserves_future_lane();
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
        distant_fd_conflicts_are_found_through_free_carry();
        distant_connected_lut_fd_pairs_and_independent_fds_pack();
        independent_lut1_does_not_block_distant_fd();
        chained_lut1_must_be_connected_for_distant_fd();
        independent_inputs_must_not_alias_one_local_node();
        equal_local_on_different_route_types_is_not_an_alias();
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
