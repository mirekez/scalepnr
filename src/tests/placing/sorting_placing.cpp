#include "Device.h"
#include "PlaceSorting.h"
#include "PlaceTiming.h"
#include "Tech.h"
#include "Tile.h"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

struct TestFailure {
    std::string message;
};

void require(bool condition, const std::string& message)
{
    if (!condition) throw TestFailure{message};
}

bool sameCoord(fpga::Coord left, fpga::Coord right)
{
    return left.x == right.x && left.y == right.y;
}

fpga::Coord scaled(fpga::Coord coord, int value)
{
    return {coord.x*value, coord.y*value};
}

fpga::TileType makeTileType()
{
    fpga::TileType type{"SORTING_PLACING_TEST", 1, 0};
    fpga::Element fd;
    fd.name = "REG";
    fd.type = fpga::ELEMENT_FD;
    fd.bitmap_pos = 0;
    fd.elements_to_left = fpga::ELEMENT_FD;
    type.elements.push_back(std::move(fd));
    fpga::Element lut;
    lut.name = "LUT";
    lut.type = fpga::ELEMENT_LUT1;
    lut.bitmap_pos = 0;
    lut.elements_to_left = fpga::ELEMENT_LUT1;
    type.elements.push_back(std::move(lut));
    return type;
}

void resetDevice(fpga::TileType& type, int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.tile_grid.resize(static_cast<size_t>(width*height));
    device.size_width = width;
    device.size_height = height;
    device.grid_spec.size = {width, height};
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[y*width + x];
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = tile.coord;
            tile.tile_type = &type;
            tile.cb_type = nullptr;
            tile.cb.type = nullptr;
            tile.elements_initialized = false;
            tile.elements_pos = {};
            tile.elements_free = {};
            tile.elements_left = {};
            tile.elements_right = {};
        }
    }
}

struct Fixture {
    Referable<rtl::Module> parent;
    Referable<rtl::Module> primitive;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    int designator = 1;

    Fixture()
    {
        parent.name = "top";
        primitive.name = "primitive";
        primitive.is_blackbox = true;
        primitive.parent_ref.set(&parent);
    }

    Referable<rtl::Inst>* makeRegister(const std::string& name)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name + "_cell";
        cell->type = "FD";
        cell->module_ref.set(&primitive);
        rtl::Port input;
        input.name = "D";
        input.type = rtl::Port::PORT_IN;
        input.index = 0;
        cell->ports.push_back(std::move(input));
        rtl::Port output;
        output.name = "Q";
        output.type = rtl::Port::PORT_OUT;
        output.index = 0;
        cell->ports.push_back(std::move(output));

        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->cnt_inputs = 1;
        inst->cnt_outputs = 1;
        inst->pos = -1;
        for (auto& port : cell->ports) {
            auto& conn = inst->conns.emplace_back();
            conn.port_ref.set(&port);
            conn.inst_ref.set(inst.get());
        }
        Referable<rtl::Inst>* result = inst.get();
        cells.push_back(std::move(cell));
        insts.push_back(std::move(inst));
        return result;
    }

    Referable<rtl::Inst>* makeCombinational(const std::string& name)
    {
        Referable<rtl::Inst>* result = makeRegister(name);
        result->cell_ref->type = "LUT1";
        return result;
    }

    Referable<rtl::Conn>* conn(
        Referable<rtl::Inst>* inst, const std::string& name)
    {
        for (auto& candidate : inst->conns) {
            if (candidate.port_ref.peer && candidate.port_ref->name == name)
                return &candidate;
        }
        return nullptr;
    }

    void connect(Referable<rtl::Inst>* driver, Referable<rtl::Inst>* sink)
    {
        Referable<rtl::Conn>* output = conn(driver, "Q");
        Referable<rtl::Conn>* input = conn(sink, "D");
        require(output && input, "sorting fixture lost a timing port");
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);
        rtl::Net& net = parent.nets.emplace_back();
        net.name = "sorting_net_" + std::to_string(designator);
        net.designators.push_back(designator++);
    }
};

void placeAt(Referable<rtl::Inst>* inst, fpga::Coord coordinate)
{
    fpga::Device& device = fpga::Device::current();
    fpga::Tile& tile = device.tile_grid[
        coordinate.y*device.size_width + coordinate.x];
    int position = inst->cell_ref.peer && inst->cell_ref->type == "LUT1"
        ? 3 : 0;
    int placed = tile.tryAddAt(inst, position, false);
    require(placed == position, "failed to build sorting placement");
    inst->outline.x = static_cast<float>(coordinate.x);
    inst->outline.y = static_cast<float>(coordinate.y);
}

void addEndpoint(clk::Timings& timings, rtl::Clock& clock,
                 Referable<rtl::Conn>* data_in)
{
    auto& info = timings.clocked_inputs[&clock].emplace_back();
    info.data_in = data_in;
    info.path.data_in = data_in;
    info.path.data_output = data_in ? data_in->follow() : nullptr;
}

void direction_and_shift_helpers()
{
    using D = pnr::PlaceSortingDirection;
    require(pnr::PlaceSorting::directionFor({5, 2}, {5, 8}, {12, 12}) == D::north,
            "north triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({8, 5}, {2, 5}, {12, 12}) == D::east,
            "east triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({5, 8}, {5, 2}, {12, 12}) == D::south,
            "south triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({2, 5}, {8, 5}, {12, 12}) == D::west,
            "west triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({2, 2}, {5, 5}, {11, 11}) == D::north,
            "north-west diagonal did not use the north triangle");
    require(pnr::PlaceSorting::directionFor({8, 8}, {5, 5}, {11, 11}) == D::east,
            "equal-distance chip boundaries did not use the stable tie order");
    require(pnr::PlaceSorting::directionFor({2, 5}, {3, 10}, {12, 12}) == D::north,
            "closer west boundary overrode the rotating north preference");
    require(pnr::PlaceSorting::directionFor({2, 5}, {3, 10}, {12, 12}, D::west) == D::west,
            "explicit rotated preference was ignored");
    require(pnr::PlaceSorting::directionFor({5, 2}, {10, 3}, {12, 12}) == D::north,
            "north boundary was ignored in favor of the longest connection axis");
    require(pnr::PlaceSorting::directionFor({2, 5}, {0, 5}, {12, 12}) == D::east,
            "evacuation toward the nearest incompatible edge moved A/B away from its peer");
    require(pnr::PlaceSorting::evacuationDirections({5, 5}, {5, 5}, {12, 12}).empty(),
            "coincident cells should have no toward-peer direction");
    // All eight relative regions, in cyclic order from north.
    const std::array<fpga::Coord, 8> peers{{
        {3, 2}, {6, 2}, {6, 5}, {6, 8}, {3, 8}, {1, 8}, {1, 5}, {1, 2}}};
    const std::array<std::vector<D>, 8> expected{{
        {D::south}, {D::south, D::west}, {D::west}, {D::north, D::west},
        {D::north}, {D::north, D::east}, {D::east}, {D::east, D::south}}};
    for (size_t i = 0; i < peers.size(); ++i)
        require(pnr::PlaceSorting::evacuationDirections({3, 5}, peers[i], {12, 12}) == expected[i],
                "eight-region cyclic ordering failed for region " + std::to_string(i));
    for (D preferred : {D::north, D::east, D::south, D::west}) {
        for (const auto& peer : peers) {
            std::vector<D> eligible;
            D direction=preferred;
            for (int i=0;i<4;++i,direction=pnr::PlaceSorting::nextDirection(direction)) {
                auto step=pnr::PlaceSorting::directionStep(direction);
                if ((peer.x-3)*step.x+(peer.y-5)*step.y<0) eligible.push_back(direction);
            }
            require(pnr::PlaceSorting::evacuationDirections({3,5},peer,{12,12},preferred)==eligible,
                    "rotation skipped or duplicated a feasible direction");
        }
    }
    require(pnr::PlaceSorting::nextDirection(D::north)==D::east
                && pnr::PlaceSorting::nextDirection(D::east)==D::south
                && pnr::PlaceSorting::nextDirection(D::south)==D::west
                && pnr::PlaceSorting::nextDirection(D::west)==D::north,
            "N/E/S/W rotation did not wrap");
    require(pnr::PlaceSorting::directionFor({3, 8}, {1, 2}, {8, 20}) == D::east,
            "rectangular chip boundary distance was calculated incorrectly");
    require(pnr::PlaceSorting::directionStep(D::north).y == -1
                && pnr::PlaceSorting::directionStep(D::east).x == 1
                && pnr::PlaceSorting::directionStep(D::south).y == 1
                && pnr::PlaceSorting::directionStep(D::west).x == -1,
            "direction unit vectors are incorrect");

    pnr::PlaceSorting sorting;
    require(sorting.estimateShiftTiles(-0.14, D::east) == 2
                && sorting.estimateShiftTiles(-0.16, D::north) == 2
                && sorting.estimateShiftTiles(0.10, D::west) == 0,
            "setup-deficit-to-distance calibration is incorrect");
}

void cascadeRepairsDirection(pnr::PlaceSortingDirection direction)
{
    using D = pnr::PlaceSortingDirection;
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 12, 12);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister(
        std::string{"A_"} + pnr::placeSortingDirectionName(direction));
    Referable<rtl::Inst>* b = fixture.makeRegister(
        std::string{"B_"} + pnr::placeSortingDirectionName(direction));
    Referable<rtl::Inst>* outer = fixture.makeRegister("outer_blocker");
    Referable<rtl::Inst>* middle = fixture.makeRegister("middle_blocker");
    Referable<rtl::Inst>* target_blocker =
        fixture.makeRegister("target_blocker");
    Referable<rtl::Inst>* outer_lut =
        fixture.makeCombinational("outer_lut");
    Referable<rtl::Inst>* origin_lut =
        fixture.makeCombinational("origin_lut");
    Referable<rtl::Inst>* middle_lut =
        fixture.makeCombinational("middle_lut");
    Referable<rtl::Inst>* target_lut =
        fixture.makeCombinational("target_lut");
    fixture.connect(a, b);

    fpga::Coord a_coord;
    fpga::Coord b_coord;
    switch (direction) {
    case D::north: a_coord = {5, 4}; b_coord = {5, 9}; break;
    case D::east: a_coord = {7, 5}; b_coord = {2, 5}; break;
    case D::south: a_coord = {5, 6}; b_coord = {5, 1}; break;
    case D::west: a_coord = {4, 5}; b_coord = {9, 5}; break;
    case D::none: throw TestFailure{"invalid direction regression"};
    }
    fpga::Coord step = pnr::PlaceSorting::directionStep(direction);
    fpga::Coord target = a_coord - scaled(step, 2);
    fpga::Coord between = a_coord - step;
    fpga::Coord outer_coord = a_coord + step;
    fpga::Coord free_coord = a_coord + scaled(step, 2);
    placeAt(a, a_coord);
    placeAt(b, b_coord);
    placeAt(outer, outer_coord);
    placeAt(middle, between);
    placeAt(target_blocker, target);
    placeAt(outer_lut, outer_coord);
    placeAt(origin_lut, a_coord);
    placeAt(middle_lut, between);
    placeAt(target_lut, target);
    b->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "sorting_clock",
        .conn_ptr = nullptr,
        .conn_name = "sorting_clock",
        .period_ns = 0.075,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    require(sorting.estimateShiftTiles(-0.11, direction) == 2,
            "setup-deficit shift was not divided between A and B");
    std::vector<rtl::Inst*> placed{a, b, outer, middle, target_blocker,
        outer_lut, origin_lut, middle_lut, target_lut};
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.deficite_cells == 1 && result.endpoints_examined == 1,
            "PlaceSorting did not build the one-cell DEFICITE list");
    require(result.accepted_moves == 1 && result.moves.size() == 1,
            "PlaceSorting did not repair the directional fixture");
    const pnr::PlaceSortingMove& move = result.moves.front();
    require(move.cell == a && move.peer == b && move.direction == direction,
            "PlaceSorting repaired the wrong A/B side or direction");
    require(sameCoord(move.from, a_coord) && sameCoord(move.to, target)
                && sameCoord(move.free_tile, free_coord),
            "PlaceSorting selected the wrong target or free Tile");
    require(move.requested_shift == 2 && move.shifted_cells == 7,
            "PlaceSorting did not cascade every occupied row/column step");
    require(sameCoord(outer->coord, free_coord)
                && sameCoord(middle->coord, a_coord)
                && sameCoord(target_blocker->coord, between),
            "row/column occupants were not shifted toward the free Tile");
    require(sameCoord(outer_lut->coord, free_coord)
                && sameCoord(origin_lut->coord, outer_coord)
                && sameCoord(middle_lut->coord, a_coord)
                && sameCoord(target_lut->coord, between),
            "PlaceSorting did not shift complete Tile contents");
    require(result.after.worst_slack_ns > result.before.worst_slack_ns
                && result.after.total_negative_slack_ns
                    < result.before.total_negative_slack_ns,
            "directional cascade did not improve exact setup timing");
}

void all_quadrants_and_axes()
{
    cascadeRepairsDirection(pnr::PlaceSortingDirection::north);
    cascadeRepairsDirection(pnr::PlaceSortingDirection::east);
    cascadeRepairsDirection(pnr::PlaceSortingDirection::south);
    cascadeRepairsDirection(pnr::PlaceSortingDirection::west);
}

void both_endpoint_sides_are_sorted()
{
    using D = pnr::PlaceSortingDirection;
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 12, 12);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("both_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("both_B");
    fixture.connect(a, b);
    std::vector<rtl::Inst*> placed{a, b};
    auto addBlocker = [&](const std::string& name, fpga::Coord coord) {
        Referable<rtl::Inst>* blocker = fixture.makeRegister(name);
        placeAt(blocker, coord);
        placed.push_back(blocker);
    };
    placeAt(a, {5, 2});
    placeAt(b, {5, 9});
    addBlocker("north_outer", {5, 1});
    addBlocker("north_inner_1", {5, 3});
    addBlocker("north_inner_2", {5, 4});
    addBlocker("north_target", {5, 5});
    addBlocker("south_target", {5, 7});
    addBlocker("south_inner", {5, 8});
    addBlocker("south_outer", {5, 10});

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "both_sides_clock",
        .conn_ptr = nullptr,
        .conn_name = "both_sides_clock",
        .period_ns = 0.05,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.accepted_moves == 2 && result.moves.size() == 2,
            "PlaceSorting did not process A and B separately");
    require(result.moves[0].cell == a
                && result.moves[0].direction == D::north
                && result.moves[1].cell == b
                && result.moves[1].direction == D::south,
            "PlaceSorting used the wrong side ordering or triangles");
    require(sameCoord(a->coord, {5, 5}) && sameCoord(b->coord, {5, 7}),
            "A/B half-deficit corrections did not meet in the middle");
    require(result.after.worst_slack_ns >= sorting.config.deficite_slack_ns,
            "two-sided PlaceSorting did not remove the endpoint from DEFICITE");
}

void blocked_primary_uses_fallback_and_rolls_back()
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 5, 5);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("fallback_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("fallback_B");
    Referable<rtl::Inst>* between = fixture.makeRegister("fallback_between");
    Referable<rtl::Inst>* target = fixture.makeRegister("fallback_target");
    Referable<rtl::Inst>* fixed = fixture.makeRegister("fixed_primary_target");
    fixture.connect(a, b);
    placeAt(a, {2, 0});
    placeAt(b, {2, 4});
    placeAt(between, {1, 0});
    placeAt(target, {0, 0});
    placeAt(fixed, {2, 2});
    fixed->outline.fixed = true;
    b->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "fallback_clock",
        .conn_ptr = nullptr,
        .conn_name = "fallback_clock",
        .period_ns = 0.05,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    std::vector<rtl::Inst*> placed{a, b, between, target, fixed};
    auto* near_fixed = fixture.makeRegister("fixed_nearer_target");
    placeAt(near_fixed, {2, 1});
    near_fixed->outline.fixed = true;
    placed.push_back(near_fixed);
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.direction_attempts == 1 && result.rejected_packing >= 1,
            "PlaceSorting tried an axis that cannot move A/B toward its peer");
    require(result.accepted_moves == 0 && sameCoord(a->coord, {2, 0})
                && sameCoord(between->coord, {1, 0})
                && sameCoord(target->coord, {0, 0}),
            "rejected fallback did not restore exact row placement");
    require(std::abs(result.after.worst_slack_ns
                     - result.before.worst_slack_ns) < 1e-9,
            "rejected fallback changed exact setup timing");
}

void blocked_primary_accepts_useful_fallback()
{
    using D = pnr::PlaceSortingDirection;
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 7, 7);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("diagonal_fallback_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("diagonal_fallback_B");
    Referable<rtl::Inst>* fixed = fixture.makeRegister("diagonal_fixed_target");
    fixture.connect(a, b);
    placeAt(a, {2, 0});
    placeAt(b, {5, 5});
    placeAt(fixed, {2, 2});
    fixed->outline.fixed = true;
    b->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "diagonal_fallback_clock",
        .conn_ptr = nullptr,
        .conn_name = "diagonal_fallback_clock",
        .period_ns = 0.20,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    std::vector<rtl::Inst*> placed{a, b, fixed};
    auto* near_fixed = fixture.makeRegister("diagonal_fixed_nearer_target");
    placeAt(near_fixed, {2, 1});
    near_fixed->outline.fixed = true;
    placed.push_back(near_fixed);
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.accepted_moves == 1 && result.direction_attempts == 2,
            "PlaceSorting did not continue through alternate directions");
    require(result.moves.front().cell == a
                && result.moves.front().direction == D::west
                && sameCoord(a->coord, {4, 0}),
            "PlaceSorting did not accept the useful diagonal fallback");
    require(result.after.worst_slack_ns > result.before.worst_slack_ns,
            "accepted fallback did not improve exact setup timing");
}

void no_free_tile_is_detected()
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 5, 5);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("full_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("full_B");
    fixture.connect(a, b);
    placeAt(a, {2, 0});
    placeAt(b, {2, 4});
    b->outline.fixed = true;
    std::vector<rtl::Inst*> placed{a, b};
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            if (!((x == 2 && y == 0) || (x == 2 && y == 4))) {
                Referable<rtl::Inst>* blocker = fixture.makeRegister(
                    "full_reg_" + std::to_string(x) + "_"
                        + std::to_string(y));
                placeAt(blocker, {x, y});
                placed.push_back(blocker);
            }
            Referable<rtl::Inst>* lut = fixture.makeCombinational(
                "full_lut_" + std::to_string(x) + "_"
                    + std::to_string(y));
            placeAt(lut, {x, y});
            placed.push_back(lut);
        }
    }

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "full_clock",
        .conn_ptr = nullptr,
        .conn_name = "full_clock",
        .period_ns = 0.05,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.accepted_moves == 0
                && result.skipped_no_free_tile == 1
                && result.free_tiles_examined > 0,
            "PlaceSorting did not detect a fully occupied row/column search");
    require(sameCoord(a->coord, {2, 0}) && sameCoord(b->coord, {2, 4}),
            "no-free search changed endpoint placement");
}

// Exercise the cheap path, an occupied old slot with another legal lane,
// and the nearest vacancy between the destination and the original cell.
void nearby_space_and_connected_blockers(int mode)
{
    fpga::TileType tile_type = makeTileType();
    if (mode == 1) {
        auto extra = tile_type.elements.front();
        extra.name = "REG_EXTRA";
        extra.bitmap_pos = 1;
        tile_type.elements.push_back(std::move(extra));
    }
    resetDevice(tile_type, 12, 12);
    Fixture fixture;
    auto* a = fixture.makeRegister("nearby_A");
    auto* b = fixture.makeRegister("nearby_B");
    fixture.connect(a, b);
    placeAt(a, {5, 4});
    placeAt(b, {5, 9});
    b->outline.fixed = true;
    std::vector<rtl::Inst*> placed{a, b};
    Referable<rtl::Inst>* blocker = nullptr;
    Referable<rtl::Inst>* neighbor = nullptr;
    std::vector<Referable<rtl::Inst>*> extra_neighbors;
    if (mode != 0) {
        blocker = fixture.makeRegister("connected_blocker");
        placeAt(blocker, {5, 6});
        placed.push_back(blocker);
    }
    if (mode >= 3) {
        neighbor = fixture.makeRegister("protected_neighbor");
        placeAt(neighbor, {6, 6});
        neighbor->outline.fixed = true;
        fixture.connect(blocker, neighbor);
        placed.push_back(neighbor);
    }
    if (mode == 5) {
        // Four affected endpoints make shifting the blocker worsen TNS,
        // even though WNS improves. Forward Sorting must commit this shift.
        for (int x = 7; x <= 9; ++x) {
            auto* extra = fixture.makeRegister("protected_neighbor_" + std::to_string(x));
            placeAt(extra, {x, 6});
            extra->outline.fixed = true;
            fixture.connect(blocker, extra);
            placed.push_back(extra);
            extra_neighbors.push_back(extra);
        }
    }
    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "nearby_clock", .conn_ptr = nullptr,
        .conn_name = "nearby_clock", .period_ns = 0.075, .duty = 50,
    });
    Referable<rtl::Clock> neighbor_clock(rtl::Clock{
        .name = "neighbor_clock", .conn_ptr = nullptr,
        .conn_name = "neighbor_clock",
        .period_ns = mode >= 4 ? 0.075 : 0.3, .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    if (neighbor)
        addEndpoint(timings, neighbor_clock, fixture.conn(neighbor, "D"));
    for (auto* extra : extra_neighbors)
        addEndpoint(timings, neighbor_clock, fixture.conn(extra, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = tech.place.aspect_y = 1;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    auto result = sorting.run(timings, placed);
    if (mode == 4) {
        require(result.accepted_moves == 1 && result.shifted_cells == 1
                    && sameCoord(a->coord, {5, 6})
                    && sameCoord(blocker->coord, {5, 5})
                    && result.moves.front().direction == pnr::PlaceSortingDirection::north,
                "sorting failed to move displaced cells opposite to A/B");
        require(result.after.violated_endpoints > result.before.violated_endpoints
                    && result.after.worst_slack_ns > result.before.worst_slack_ns
                    && result.after.total_negative_slack_ns < result.before.total_negative_slack_ns,
                "sorting rejected a useful WNS/TNS trade-off due to violation count");
    } else if (mode == 5) {
        require(result.accepted_moves == 1 && result.shifted_cells == 1
                    && sameCoord(a->coord, {5, 6})
                    && sameCoord(blocker->coord, {5, 5})
                    && result.after.worst_slack_ns > result.before.worst_slack_ns
                    && result.after.total_negative_slack_ns > result.before.total_negative_slack_ns,
                "forward sorting vetoed or rolled back a WNS-improving shift because TNS increased");
    } else {
        require(result.accepted_moves == 1 && sameCoord(a->coord, {5, 6})
                    && result.after.worst_slack_ns > result.before.worst_slack_ns,
                "sorting missed nearby compatible capacity");
        require(result.shifted_cells == (mode >= 2 ? 1U : 0U),
                "sorting unnecessarily displaced cells or counted rejected plans");
        if (mode == 1)
            require(a->pos == 4 && sameCoord(blocker->coord, {5, 6}),
                    "sorting rejected a spare lane because the old lane was busy");
        if (mode >= 2)
            require(sameCoord(blocker->coord, {5, 5})
                        && sameCoord(result.moves.front().free_tile, {5, 5}),
                    "sorting skipped the vacancy between target and origin");
    }
    require(!result.timed_out, "small sorting task timed out");
    require(result.timing_evaluations == result.accepted_moves,
            "sorting evaluated timing on an uncommitted trial");
    pnr::PlaceTiming timing;
    timing.tech = &tech;
    auto exact = timing.analyze(timings);
    require(std::abs(exact.worst_slack_ns - result.after.worst_slack_ns) < 1e-9
                && std::abs(exact.total_negative_slack_ns
                    - result.after.total_negative_slack_ns) < 1e-9,
            "coordinate preview left incremental timing inconsistent");
    auto& device = fpga::Device::current();
    for (auto& tile : device.tile_grid) {
        int count = 0;
        for (auto* inst : placed) {
            if (inst->tile.peer != &tile) continue;
            ++count;
            require(sameCoord(inst->coord, tile.coord) && inst->pos >= 0,
                    "sorting left inconsistent Tile ownership");
        }
        require(tile.regs_cnt == count, "packing preview leaked register counters");
    }
}

void whole_setup_endpoints_are_processed(bool fixed_driver, bool fixed_sink)
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 50, 50);
    Fixture fixture;
    auto* driver = fixture.makeRegister("setup_launch_A");
    auto* logic = fixture.makeCombinational("setup_middle_LUT");
    auto* sink = fixture.makeRegister("setup_capture_B");
    fixture.connect(driver, logic);
    fixture.connect(logic, sink);
    placeAt(driver, {0, 3});
    placeAt(logic, {12, 13});
    placeAt(sink, {32, 13});
    driver->outline.fixed = fixed_driver;
    sink->outline.fixed = fixed_sink;
    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "setup_clock", .conn_ptr = nullptr,
        .conn_name = "setup_clock", .period_ns = 1.0, .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));
    auto& path = timings.clocked_inputs[&clock].front().path;
    auto& input = path.sub_paths.emplace_back();
    input.data_in = fixture.conn(logic, "D");
    input.data_output = fixture.conn(driver, "Q");
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = tech.place.aspect_y = 1;
    timings.tech = &tech;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    std::vector<rtl::Inst*> placed{driver, logic, sink};
    auto result = sorting.run(timings, placed);
    const auto& edges = result.before.endpoint_details.front().critical_edges;
    require(result.timing_evaluations == result.accepted_moves,
            "setup sorting performed speculative timing evaluations");
    require(edges.size() == 2 && edges.back().wire_delay_ns > edges.front().wire_delay_ns,
            "setup regression did not reproduce the longer launch-to-LUT wire");
    require(result.endpoint_sides_examined == 2
                && result.accepted_moves == static_cast<size_t>(!fixed_driver + !fixed_sink),
            "sorting omitted a launch/capture side of the complete timing path");
    require(sameCoord(logic->coord, {12, 13}),
            "sorting substituted the intermediate LUT for a timing endpoint");
    for (const auto& move : result.moves) {
        require((move.cell == driver && move.peer == sink)
                    || (move.cell == sink && move.peer == driver),
                "sorting reported an internal wire instead of the complete setup endpoints");
        const auto step=pnr::PlaceSorting::directionStep(move.direction);
        const auto peer_coord=move.peer->coord;
        require((move.to.x-move.from.x)*step.x+(move.to.y-move.from.y)*step.y<0
                    && std::abs(move.to.x-peer_coord.x)+std::abs(move.to.y-peer_coord.y)
                        < std::abs(move.from.x-peer_coord.x)+std::abs(move.from.y-peer_coord.y),
                "setup endpoint moved away from its peer or along the evacuation ray");
    }
    require(!fixed_driver || sameCoord(driver->coord, {0, 3}), "fixed launch endpoint moved");
    require(!fixed_sink || sameCoord(sink->coord, {32, 13}), "fixed capture endpoint moved");
    if (!result.moves.empty())
        require(result.after.worst_slack_ns > result.before.worst_slack_ns,
                "whole-path endpoint moves did not improve timing");
    pnr::PlaceTiming estimator;
    estimator.tech = &tech;
    auto full = estimator.analyze(timings);
    require(std::abs(full.worst_slack_ns - result.after.worst_slack_ns) < 1e-9
                && std::abs(full.total_negative_slack_ns - result.after.total_negative_slack_ns) < 1e-9,
            "whole-path endpoint movement left stale timing");
}

double folded_chain_center(bool enabled, int rotation, bool fixed_ends)
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 100, 100);
    Fixture fixture;
    auto* a = fixture.makeRegister("folded_launch");
    auto* logic = fixture.makeCombinational("folded_LUT");
    auto* b = fixture.makeRegister("folded_capture");
    fixture.connect(a, logic);
    fixture.connect(logic, b);
    auto rotate = [&](fpga::Coord c) {
        for (int i=0;i<rotation;++i) c={99-c.y,c.x};
        return c;
    };
    // The measured 100x100 detour: both registers lie below the LUT.
    const auto a_start=rotate({65,84}), lut_start=rotate({38,38}), b_start=rotate({38,79});
    placeAt(a,a_start);placeAt(logic,lut_start);placeAt(b,b_start);
    a->outline.fixed=b->outline.fixed=fixed_ends;
    Referable<rtl::Clock> clock(rtl::Clock{
        .name="folded_clock", .conn_ptr=nullptr, .conn_name="folded_clock",
        .period_ns=1.0, .duty=50,
    });
    clk::Timings timings;
    addEndpoint(timings,clock,fixture.conn(b,"D"));
    auto& path=timings.clocked_inputs[&clock].front().path;
    auto& input=path.sub_paths.emplace_back();
    input.data_in=fixture.conn(logic,"D");input.data_output=fixture.conn(a,"Q");
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD","C");
    technology::Tech tech;
    tech.place.aspect_x=tech.place.aspect_y=1;
    timings.tech=&tech;
    pnr::PlaceSorting sorting;
    sorting.tech=&tech;
    sorting.config.chain_center=enabled;
    sorting.config.trace_chain_moves=enabled;
    std::vector<rtl::Inst*> placed{a,logic,b};
    auto result=sorting.run(timings,placed);
    if(enabled) {
        require(result.chain_cells_examined==3,"chain sorting omitted an internal cell or fixed anchor");
        require(!sameCoord(logic->coord,lut_start),"folded LUT was not brought toward chain center");
        const double cx=(a_start.x+lut_start.x+b_start.x)/3.0;
        const double cy=(a_start.y+lut_start.y+b_start.y)/3.0;
        for(const auto& move:result.moves) {
            require(move.toward_chain_center && move.peer==nullptr && move.setup_endpoint==fixture.conn(b,"D"),
                    "center movement reported a stale endpoint peer");
            require(std::abs(move.center_x-cx)<1e-9 && std::abs(move.center_y-cy)<1e-9,
                    "center double-counted LUT or drifted during the chain visit");
            double before=std::abs(move.from.x-cx)+std::abs(move.from.y-cy);
            double after=std::abs(move.to.x-cx)+std::abs(move.to.y-cy);
            require(after<before,"chain member moved away from center");
        }
        if(fixed_ends) require(sameCoord(a->coord,a_start)&&sameCoord(b->coord,b_start),"chain sorting moved fixed anchors");
    }
    pnr::PlaceTiming timing;timing.tech=&tech;
    auto exact=timing.analyze(timings);
    require(std::abs(exact.worst_slack_ns-result.after.worst_slack_ns)<1e-9,
            "chain-center slack disagrees with independent timing calculation");
    require(result.timing_evaluations==result.accepted_moves,"chain experiment added speculative timing evaluations");
    for(auto* inst:placed) require(inst->tile.peer&&sameCoord(inst->coord,inst->tile.peer->coord),"chain center lost packing ownership");
    std::cout << "SORT_CENTER_REGRESSION mode=" << enabled << " rotation=" << rotation
              << " fixed_ends=" << fixed_ends << " slack=" << result.before.worst_slack_ns
              << "->" << exact.worst_slack_ns << '\n';
    return exact.worst_slack_ns;
}

void whole_chain_center_regression()
{
    for(int rotation=0;rotation<4;++rotation) {
        double baseline=folded_chain_center(false,rotation,false);
        double centered=folded_chain_center(true,rotation,false);
        require(centered>baseline+0.2,"whole-chain center did not improve the reproduced detour over endpoint-only Sorting");
        double fixed_baseline=folded_chain_center(false,rotation,true);
        double fixed_centered=folded_chain_center(true,rotation,true);
        require(fixed_centered>fixed_baseline+0.2,"fixed anchors did not attract the internal LUT");
    }
    require(pnr::PlaceSorting::chainCenter({}).cells.empty(),"empty chain has fabricated members");
}

void rotating_preference_replaces_closest_boundary()
{
    using D = pnr::PlaceSortingDirection;
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 12, 12);
    Fixture fixture;
    auto* a = fixture.makeRegister("nearest_A");
    auto* b = fixture.makeRegister("nearest_B");
    fixture.connect(a, b);
    placeAt(a, {2, 5});
    placeAt(b, {3, 10});
    b->outline.fixed = true;
    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "nearest_clock", .conn_ptr = nullptr,
        .conn_name = "nearest_clock", .period_ns = 0.075, .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = tech.place.aspect_y = 1;
    pnr::PlaceSorting sorting;
    sorting.tech = &tech;
    std::vector<rtl::Inst*> placed{a, b};
    auto result = sorting.run(timings, placed);
    require(result.accepted_moves == 1 && result.moves.front().direction == D::north
                && sameCoord(a->coord, {2, 8}) && result.moves.front().requested_shift == 3,
            "sorting retained the nearest-edge preference instead of rotation");
}

void rotation_persists_across_chains(bool chain_center, int fallback = 0)
{
    using D = pnr::PlaceSortingDirection;
    fpga::TileType tile_type=makeTileType();
    resetDevice(tile_type,100,60);
    Fixture fixture;
    Referable<rtl::Clock> clock(rtl::Clock{
        .name="rotation_clock", .conn_ptr=nullptr, .conn_name="rotation_clock",
        .period_ns=0.05, .duty=50,
    });
    clk::Timings timings;
    std::vector<rtl::Inst*> placed;
    std::vector<rtl::Inst*> drivers;
    const std::array<fpga::Coord,4> starts{{{10,10},{30,10},{50,14},{70,14}}};
    const std::array<fpga::Coord,4> ends{{{14,14},{26,14},{46,10},{74,10}}};
    for(int repeat=0;repeat<2;++repeat) for(size_t i=0;i<4;++i) {
        auto* a=fixture.makeRegister("rotate_A_"+std::to_string(repeat*4+i));
        auto* b=fixture.makeRegister("rotate_B_"+std::to_string(repeat*4+i));
        fixture.connect(a,b);
        placeAt(a,starts[i]+fpga::Coord{0,repeat*20});
        auto end=ends[i]+fpga::Coord{0,repeat*20};
        // A north preference cannot serve this first pair geometrically.
        if(fallback==2 && repeat==0 && i==0) end={14,6};
        placeAt(b,end);
        b->outline.fixed=true;
        addEndpoint(timings,clock,fixture.conn(b,"D"));
        placed.push_back(a);placed.push_back(b);drivers.push_back(a);
    }
    if(fallback==1) {
        // Force the first N preference to fall back to W: every possible
        // southward target of A is fixed. Its eastward targets remain free.
        for(int y=11;y<=14;++y) {
            auto* guard=fixture.makeRegister("rotation_guard_"+std::to_string(y));
            placeAt(guard,{10,y});
            guard->outline.fixed=true;
            placed.push_back(guard);
        }
    }
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD","C");
    technology::Tech tech;
    tech.place.aspect_x=tech.place.aspect_y=1;
    timings.tech=&tech;
    pnr::PlaceSorting sorting;
    sorting.tech=&tech;sorting.config.chain_center=chain_center;
    auto result=sorting.run(timings,placed);
    require(result.moves.size()==8,"rotation fixture did not move all eight independent drivers");
    const std::array<D,4> expected{D::north,D::east,D::south,D::west};
    for(size_t i=0;i<result.moves.size();++i) {
        D expected_direction=expected[i%4];
        if(i==0 && fallback) expected_direction=fallback==1 ? D::west : D::south;
        require(result.moves[i].cell==drivers[i] && result.moves[i].direction==expected_direction,
                "global rotation was reset by a fallback or chain boundary at cell " + std::to_string(i));
    }
    pnr::PlaceTiming timing;timing.tech=&tech;
    auto exact=timing.analyze(timings);
    require(std::abs(exact.worst_slack_ns-result.after.worst_slack_ns)<1e-9,
            "rotating Sorting left stale setup timing");
}

void same_direction_evacuation_is_forbidden()
{
    using D = pnr::PlaceSortingDirection;
    for (D direction : {D::north, D::east, D::south, D::west}) {
        fpga::TileType tile_type = makeTileType();
        resetDevice(tile_type, 16, 16);
        Fixture fixture;
        auto* a = fixture.makeRegister("reverse_A");
        auto* b = fixture.makeRegister("reverse_B");
        auto* blocker = fixture.makeRegister("reverse_blocker");
        auto* guard = fixture.makeRegister("fixed_guard");
        fixture.connect(a, b);
        fpga::Coord origin{7, 7};
        auto step = pnr::PlaceSorting::directionStep(direction);
        auto target = origin - scaled(step, 2);
        placeAt(a, origin);
        placeAt(b, origin - scaled(step, 5));
        placeAt(blocker, target);
        placeAt(guard, origin - step);
        b->outline.fixed = guard->outline.fixed = true;
        Referable<rtl::Clock> clock(rtl::Clock{
            .name = "reverse_clock", .conn_ptr = nullptr,
            .conn_name = "reverse_clock", .period_ns = 0.075, .duty = 50,
        });
        clk::Timings timings;
        addEndpoint(timings, clock, fixture.conn(b, "D"));
        technology::Tech::clocked_ports.clear();
        technology::Tech::clocked_ports.emplace("FD", "C");
        technology::Tech tech;
        tech.place.aspect_x = tech.place.aspect_y = 1;
        pnr::PlaceSorting sorting;
        sorting.tech = &tech;
        std::vector<rtl::Inst*> placed{a, b, blocker, guard};
        auto result = sorting.run(timings, placed);
        require(result.timing_evaluations == 0,
                "blocked cascade evaluated timing before any move was committed");
        require(result.accepted_moves == 0 && result.shifted_cells == 0
                    && sameCoord(a->coord, origin)
                    && sameCoord(blocker->coord, target),
                "sorting evacuated contents in the same direction as A/B on "
                    + std::string(pnr::placeSortingDirectionName(direction)));
        require(sameCoord(guard->coord, origin - step)
                    && sameCoord(b->coord, origin - scaled(step, 5)),
                "reverse evacuation moved a fixed cell");
        pnr::PlaceTiming timing;
        timing.tech = &tech;
        auto exact = timing.analyze(timings);
        require(std::abs(exact.worst_slack_ns - result.after.worst_slack_ns) < 1e-9
                    && std::abs(exact.total_negative_slack_ns - result.after.total_negative_slack_ns) < 1e-9,
                "reverse evacuation incremental timing disagrees with full analysis");
        for (auto* inst : placed)
            require(inst->tile.peer && sameCoord(inst->coord, inst->tile.peer->coord)
                        && inst->tile.peer->regs_cnt == 1,
                    "reverse evacuation left incorrect packing ownership");
    }
}

}

int main()
{
    try {
        direction_and_shift_helpers();
        all_quadrants_and_axes();
        both_endpoint_sides_are_sorted();
        blocked_primary_uses_fallback_and_rolls_back();
        blocked_primary_accepts_useful_fallback();
        no_free_tile_is_detected();
        whole_setup_endpoints_are_processed(false, false);
        whole_setup_endpoints_are_processed(true, false);
        whole_setup_endpoints_are_processed(false, true);
        whole_setup_endpoints_are_processed(true, true);
        rotating_preference_replaces_closest_boundary();
        rotation_persists_across_chains(false);
        rotation_persists_across_chains(true);
        for(int fallback=1;fallback<=2;++fallback) {
            rotation_persists_across_chains(false,fallback);
            rotation_persists_across_chains(true,fallback);
        }
        same_direction_evacuation_is_forbidden();
        whole_chain_center_regression();
        for (int mode = 0; mode <= 5; ++mode)
            nearby_space_and_connected_blockers(mode);
        std::cout << "sorting_placing_test passed\n";
        return 0;
    }
    catch (const TestFailure& failure) {
        std::cerr << "sorting_placing_test failed: " << failure.message
                  << '\n';
        return 1;
    }
    catch (const std::exception& exception) {
        std::cerr << "sorting_placing_test failed with exception: "
                  << exception.what() << '\n';
        return 1;
    }
}
