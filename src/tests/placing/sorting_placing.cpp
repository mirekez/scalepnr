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
    require(pnr::PlaceSorting::directionFor({5, 2}, {5, 8}) == D::north,
            "north triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({8, 5}, {2, 5}) == D::east,
            "east triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({5, 8}, {5, 2}) == D::south,
            "south triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({2, 5}, {8, 5}) == D::west,
            "west triangle was misclassified");
    require(pnr::PlaceSorting::directionFor({2, 2}, {5, 5}) == D::north,
            "north-west diagonal did not use the north triangle");
    require(pnr::PlaceSorting::directionFor({8, 8}, {5, 5}) == D::south,
            "south-east diagonal did not use the south triangle");
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
    fixture.connect(a, b);
    placeAt(a, {2, 0});
    placeAt(b, {2, 4});
    placeAt(between, {1, 0});
    placeAt(target, {0, 0});
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
    std::vector<rtl::Inst*> placed{a, b, between, target};
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.direction_attempts == 4 && result.rejected_timing >= 1,
            "PlaceSorting did not try fallback axes after blocked north");
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
    fixture.connect(a, b);
    placeAt(a, {2, 0});
    placeAt(b, {5, 5});
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
    std::vector<rtl::Inst*> placed{a, b};
    pnr::PlaceSortingResult result = sorting.run(timings, placed);

    require(result.accepted_moves == 1 && result.rejected_timing >= 1,
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
