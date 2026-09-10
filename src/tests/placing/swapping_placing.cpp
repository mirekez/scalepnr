#include "Device.h"
#include "PlaceSwapping.h"
#include "PlaceTiming.h"
#include "RegBunch.h"
#include "Tech.h"
#include "Tile.h"

#include <cmath>
#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct TestFailure
{
    std::string message;
};

void require(bool condition, const std::string& message)
{
    if (!condition) throw TestFailure{message};
}

int fdPos()
{
    return 0;
}

int lutPos()
{
    return 3;
}

fpga::TileType makeTileType()
{
    fpga::TileType type{"SWAPPING_PLACING_TEST", 1, 0};
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

struct Fixture
{
    Referable<rtl::Module> parent;
    Referable<rtl::Module> primitive;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    int designator = 1;

    Fixture()
    {
        parent.name = "top";
        parent.is_blackbox = false;
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
        cell->ports.emplace_back(std::move(input));
        rtl::Port output;
        output.name = "Q";
        output.type = rtl::Port::PORT_OUT;
        output.index = 0;
        cell->ports.emplace_back(std::move(output));

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

    Referable<rtl::Conn>* conn(Referable<rtl::Inst>* inst,
                              const std::string& name)
    {
        for (auto& candidate : inst->conns) {
            if (candidate.port_ref.peer && candidate.port_ref->name == name) {
                return &candidate;
            }
        }
        return nullptr;
    }

    void connect(Referable<rtl::Inst>* driver, Referable<rtl::Inst>* sink)
    {
        Referable<rtl::Conn>* output = conn(driver, "Q");
        Referable<rtl::Conn>* input = conn(sink, "D");
        require(output && input, "reference pair has a missing timing port");
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);
        auto& net = parent.nets.emplace_back();
        net.name = "reference_net_" + std::to_string(designator);
        net.designators.push_back(designator++);
    }
};

void placeAt(Referable<rtl::Inst>* inst, fpga::Coord coordinate)
{
    fpga::Device& device = fpga::Device::current();
    fpga::Tile& tile = device.tile_grid[
        coordinate.y*device.size_width + coordinate.x];
    int position = inst->cell_ref.peer
                           && inst->cell_ref->type.find("FD") == 0
                       ? fdPos()
                       : lutPos();
    int placed = tile.tryAddAt(inst, position);
    require(placed == position, "failed to create reference placement");
    inst->outline.x = static_cast<float>(coordinate.x);
    inst->outline.y = static_cast<float>(coordinate.y);
}

void exchange(Referable<rtl::Inst>* left, Referable<rtl::Inst>* right)
{
    fpga::Coord left_coord = left->coord;
    fpga::Coord right_coord = right->coord;
    left->tile->unassign(left);
    right->tile->unassign(right);
    placeAt(left, right_coord);
    placeAt(right, left_coord);
}

bool sameCoord(fpga::Coord left, fpga::Coord right)
{
    return left.x == right.x && left.y == right.y;
}

void addEndpoint(clk::Timings& timings, rtl::Clock& clock,
                 Referable<rtl::Conn>* data_in)
{
    auto& info = timings.clocked_inputs[&clock].emplace_back();
    info.data_in = data_in;
    info.path.data_in = data_in;
    info.path.data_output = data_in ? data_in->follow() : nullptr;
}

pnr::PlaceTimingAnalysis analyze(technology::Tech& tech,
                                 clk::Timings& timings)
{
    pnr::PlaceTiming timing;
    timing.tech = &tech;
    return timing.analyze(timings);
}

void reference_vertical_misplacement_is_recovered()
{
    constexpr fpga::Coord wrong_a{23, 6};
    constexpr fpga::Coord correct_a{25, 20};
    constexpr fpga::Coord fixed_b{26, 26};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 50, 50);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("reference_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("reference_B");
    Referable<rtl::Inst>* challenger =
        fixture.makeRegister("reference_challenger");
    Referable<rtl::Inst>* challenger_peer =
        fixture.makeRegister("reference_challenger_peer");
    fixture.connect(a, b);
    fixture.connect(challenger, challenger_peer);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    challenger_bunch.reg = challenger;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    // Start from the known reference solution, then deliberately exchange A
    // with an unrelated bunch. This recreates the observed dominant-vertical
    // WNS geometry: A=(23,6), B=(26,26), wire delay 0.920 ns.
    placeAt(a, correct_a);
    placeAt(b, fixed_b);
    placeAt(challenger, wrong_a);
    placeAt(challenger_peer, fpga::Coord{23, 14});
    b->outline.fixed = true;
    challenger_peer->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "reference_clock",
        .conn_ptr = nullptr,
        .conn_name = "reference_clock",
        .period_ns = 0.75,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "proficite_clock",
        .period_ns = 2.0,
        .duty = 50,
    });
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceTimingAnalysis reference = analyze(tech, timings);
    require(reference.violated_endpoints == 0,
            "known reference placement does not meet timing");

    exchange(a, challenger);
    require(sameCoord(a->coord, wrong_a)
                && sameCoord(challenger->coord, correct_a),
            "failed to inject the reference wrong placement");
    pnr::PlaceTimingAnalysis wrong = analyze(tech, timings);
    require(wrong.violated_endpoints == 1 && wrong.worst_slack_ns < 0,
            "injected reference placement is not timing-violating");
    pnr::PlaceTiming local_timing;
    local_timing.tech = &tech;
    rtl::Conn* challenger_peer_input = fixture.conn(challenger_peer, "D");
    rtl::Conn* challenger_output = challenger_peer_input->follow();
    require(challenger_output,
            "challenger timing connection has no driver");
    double challenger_delay_before = local_timing.estimateWireDelay(
        *challenger_peer_input, *challenger_output);

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    std::vector<rtl::Inst*> cells{a, b, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(std::abs(result.initial_temperature
                         - std::abs(wrong.worst_slack_ns)) < 1e-9,
            "PlaceSwapping did not initialize TEMPERATURE from WNS");
    require(result.accepted_swaps == 1,
            "PlaceSwapping did not accept the reference repair");
    require(result.deficite_cells == 0 && result.proficite_cells == 1
                && result.proficite_regions == 2500,
            "PlaceSwapping did not build the expected timing cell maps");
    require(sameCoord(a->coord, correct_a)
                && sameCoord(b->coord, fixed_b),
            "PlaceSwapping did not recover the known reference placement");
    double challenger_delay_after = local_timing.estimateWireDelay(
        *challenger_peer_input, *challenger_output);
    require(!sameCoord(challenger->coord, wrong_a)
                && challenger_delay_after
                    <= challenger_delay_before*1.05 + 1e-9,
            "second search did not protect challenger timing");
    require(result.after.violated_endpoints == 0
                && result.after.worst_slack_ns > wrong.worst_slack_ns
                && std::abs(result.after.worst_slack_ns
                            - reference.worst_slack_ns) < 1e-9,
            "PlaceSwapping recovered coordinates without recovering timing");

    std::cout
        << "SWAPPING_PLACING_REFERENCE A='" << a->makeName()
        << "' B='" << b->makeName()
        << "' wrong_A=(" << wrong_a.x << ',' << wrong_a.y
        << ") repaired_A=(" << a->coord.x << ',' << a->coord.y
        << ") fixed_B=(" << b->coord.x << ',' << b->coord.y
        << ") relocated_C=(" << challenger->coord.x << ','
        << challenger->coord.y << ") challenger_delay_ns="
        << challenger_delay_before << "->" << challenger_delay_after
        << " slack_ns=" << wrong.worst_slack_ns << "->"
        << result.after.worst_slack_ns << '\n';
}

void vacated_origin_is_a_challenger_fallback()
{
    constexpr fpga::Coord wrong_a{2, 2};
    constexpr fpga::Coord fixed_b{20, 2};
    constexpr fpga::Coord repaired_a{18, 2};
    constexpr fpga::Coord blocked_timing_target{10, 10};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 30, 20);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("fallback_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("fallback_B");
    Referable<rtl::Inst>* challenger =
        fixture.makeRegister("fallback_challenger");
    Referable<rtl::Inst>* challenger_peer =
        fixture.makeRegister("fallback_challenger_peer");
    fixture.connect(a, b);
    fixture.connect(challenger, challenger_peer);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    challenger_bunch.reg = challenger;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    placeAt(a, wrong_a);
    placeAt(b, fixed_b);
    placeAt(challenger, repaired_a);
    placeAt(challenger_peer, blocked_timing_target);
    b->outline.fixed = true;
    challenger_peer->outline.fixed = true;

    // replacementOrigin() points C at its timing peer. Occupy that Tile and
    // every Tile in the configured radius-one search, leaving A's origin as
    // the only immediately useful replacement. The old implementation
    // returned pack_failed without ever testing this freshly vacated Tile.
    std::vector<Referable<rtl::Inst>*> blockers;
    for (fpga::Coord coordinate :
         {fpga::Coord{9, 10}, fpga::Coord{11, 10}, fpga::Coord{10, 9},
          fpga::Coord{10, 11}}) {
        Referable<rtl::Inst>* blocker = fixture.makeRegister(
            "fallback_blocker_" + std::to_string(blockers.size()));
        placeAt(blocker, coordinate);
        blocker->outline.fixed = true;
        blockers.push_back(blocker);
    }

    Referable<rtl::Clock> critical_clock(rtl::Clock{
        .name = "fallback_critical_clock",
        .conn_ptr = nullptr,
        .conn_name = "fallback_critical_clock",
        .period_ns = 0.40,
        .duty = 50,
    });
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "fallback_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "fallback_proficite_clock",
        .period_ns = 3.0,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, critical_clock, fixture.conn(b, "D"));
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceTimingAnalysis before = analyze(tech, timings);
    require(before.worst_slack_ns < -0.1,
            "vacated-origin regression did not create a timing deficit");

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 1;
    swapping.config.replacement_search_radius = 1;
    std::vector<rtl::Inst*> cells{a, b, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.accepted_swaps == 1 && result.rejected_pack == 0,
            "challenger did not use the newly vacated swap origin");
    require(sameCoord(a->coord, repaired_a)
                && sameCoord(challenger->coord, wrong_a),
            "vacated-origin fallback did not perform the literal swap");
    require(result.after.worst_slack_ns > before.worst_slack_ns,
            "vacated-origin fallback packed but did not improve timing");

    std::cout
        << "SWAPPING_PLACING_VACATED_ORIGIN A=(" << wrong_a.x << ','
        << wrong_a.y << ")->(" << a->coord.x << ',' << a->coord.y
        << ") C=(" << repaired_a.x << ',' << repaired_a.y << ")->("
        << challenger->coord.x << ',' << challenger->coord.y
        << ") rejected_pack=" << result.rejected_pack << '\n';
}

void rectangle_proficite_region_is_used()
{
    constexpr fpga::Coord wrong_a{10, 4};
    constexpr fpga::Coord repaired_a{10, 20};
    constexpr fpga::Coord fixed_b{10, 22};
    constexpr fpga::Coord challenger_peer_coord{30, 15};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 50, 30);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("scope_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("scope_B");
    Referable<rtl::Inst>* challenger =
        fixture.makeRegister("scope_challenger");
    Referable<rtl::Inst>* challenger_peer =
        fixture.makeRegister("scope_challenger_peer");
    fixture.connect(a, b);
    fixture.connect(challenger, challenger_peer);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    challenger_bunch.reg = challenger;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    placeAt(a, wrong_a);
    placeAt(b, fixed_b);
    placeAt(challenger, repaired_a);
    placeAt(challenger_peer, challenger_peer_coord);
    b->outline.fixed = true;
    challenger_peer->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "scope_clock",
        .conn_ptr = nullptr,
        .conn_name = "scope_clock",
        .period_ns = 0.55,
        .duty = 50,
    });
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "scope_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "scope_proficite_clock",
        // The challenger is below the preferred +1.0 ns reserve but above
        // the +0.5 ns floor, so this sparse region must take the fallback.
        .period_ns = 1.55,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 2;
    std::vector<rtl::Inst*> cells{a, b, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.accepted_swaps == 1 && sameCoord(a->coord, repaired_a),
            "rectangle PROFICITE container did not repair DEFICITE cell");
    require(result.deficite_cells == 0 && result.proficite_cells == 1
                && result.proficite_regions == 2500
                && result.proficite_regions_relaxed == 2500,
            "sparse PROFICITE containers were not rebuilt at +0.5 ns");
}

void padded_horizontal_rectangle_proficite_region_is_used()
{
    // A and B have the same Y coordinate, so their unpadded rectangle has zero
    // physical height. C is eight Tiles below that line and must be discovered
    // through the default ten-Tile rectangle margin.
    constexpr fpga::Coord fixed_a_coord{20, 50};
    constexpr fpga::Coord wrong_b_coord{80, 50};
    constexpr fpga::Coord repaired_b_coord{50, 58};
    constexpr fpga::Coord challenger_peer_coord{50, 59};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 100, 100);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("padded_rectangle_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("padded_rectangle_B");
    Referable<rtl::Inst>* challenger =
        fixture.makeRegister("padded_rectangle_challenger");
    Referable<rtl::Inst>* challenger_peer =
        fixture.makeRegister("padded_rectangle_challenger_peer");
    fixture.connect(a, b);
    fixture.connect(challenger, challenger_peer);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    challenger_bunch.reg = challenger;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    placeAt(a, fixed_a_coord);
    placeAt(b, wrong_b_coord);
    placeAt(challenger, repaired_b_coord);
    placeAt(challenger_peer, challenger_peer_coord);
    a->outline.fixed = true;
    challenger_peer->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "padded_rectangle_clock",
        .conn_ptr = nullptr,
        .conn_name = "padded_rectangle_clock",
        .period_ns = 1.0,
        .duty = 50,
    });
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "padded_rectangle_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "padded_rectangle_proficite_clock",
        .period_ns = 3.0,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    std::vector<rtl::Inst*> cells{a, b, challenger};

    pnr::PlaceSwapping unpadded;
    unpadded.tech = &tech;
    unpadded.config.maximum_passes = 1;
    unpadded.config.proficite_rectangle_margin_tiles = 0;
    pnr::PlaceSwappingResult unpadded_result = unpadded.run(timings, cells);
    require(unpadded_result.accepted_swaps == 0
                && sameCoord(b->coord, wrong_b_coord),
            "horizontal fixture unexpectedly succeeded without padding");

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 1;
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.accepted_swaps == 1,
            "padded horizontal rectangle missed its off-axis PROFICITE region");
    require(sameCoord(b->coord, repaired_b_coord),
            "rectangle challenger did not repair the timing endpoint");
    require(result.after.worst_slack_ns > result.before.worst_slack_ns,
            "rectangle swap did not improve worst setup slack");

    std::cout << "SWAPPING_PLACING_PADDED_RECTANGLE B='" << b->makeName()
              << "' coord=(" << wrong_b_coord.x << ',' << wrong_b_coord.y
              << ")->(" << b->coord.x << ',' << b->coord.y
              << ") slack_ns=" << result.before.worst_slack_ns << "->"
              << result.after.worst_slack_ns << '\n';
}

void longest_edge_uses_its_own_proficite_rectangle()
{
    constexpr fpga::Coord fixed_a_coord{5, 10};
    constexpr fpga::Coord wrong_b_coord{25, 25};
    constexpr fpga::Coord fixed_d_coord{27, 10};
    constexpr fpga::Coord repaired_b_coord{15, 17};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 40, 35);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("edge_line_A");
    Referable<rtl::Inst>* b =
        fixture.makeCombinational("edge_line_B_comb");
    Referable<rtl::Inst>* d = fixture.makeRegister("edge_line_D");
    Referable<rtl::Inst>* challenger =
        fixture.makeRegister("edge_line_challenger");
    Referable<rtl::Inst>* challenger_peer =
        fixture.makeRegister("edge_line_challenger_peer");
    fixture.connect(a, b);
    fixture.connect(b, d);
    fixture.connect(challenger, challenger_peer);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> d_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    d_bunch.reg = d;
    challenger_bunch.reg = challenger;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    d->bunch_ref.set(&d_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    placeAt(a, fixed_a_coord);
    placeAt(b, wrong_b_coord);
    placeAt(d, fixed_d_coord);
    placeAt(challenger, repaired_b_coord);
    placeAt(challenger_peer, {15, 18});
    a->outline.fixed = true;
    d->outline.fixed = true;
    challenger_peer->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "edge_line_clock",
        .conn_ptr = nullptr,
        .conn_name = "edge_line_clock",
        .period_ns = 0.8,
        .duty = 50,
    });
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "edge_line_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "edge_line_proficite_clock",
        .period_ns = 2.0,
        .duty = 50,
    });
    clk::Timings timings;
    auto& endpoint = timings.clocked_inputs[&clock].emplace_back();
    endpoint.data_in = fixture.conn(d, "D");
    endpoint.path.data_in = endpoint.data_in;
    endpoint.path.data_output = fixture.conn(b, "Q");
    auto& critical_input = endpoint.path.sub_paths.emplace_back();
    critical_input.data_in = fixture.conn(b, "D");
    critical_input.data_output = fixture.conn(a, "Q");
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceTimingAnalysis before = analyze(tech, timings);
    require(before.worst_slack_ns < -0.1,
            "edge-rectangle fixture did not create a timing deficit");
    const pnr::PlaceTimingEndpoint* main_endpoint = nullptr;
    for (const pnr::PlaceTimingEndpoint& candidate : before.endpoint_details) {
        if (candidate.data_in == fixture.conn(d, "D")) {
            main_endpoint = &candidate;
            break;
        }
    }
    require(main_endpoint && main_endpoint->critical_edges.size() == 2
                && main_endpoint->critical_edges.front().driver == b
                && main_endpoint->critical_edges.back().driver == a
                && main_endpoint->critical_edges.back().wire_delay_ns
                    > main_endpoint->critical_edges.front().wire_delay_ns,
            "edge-rectangle fixture did not put its longest edge before the final "
            "endpoint edge");

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 1;
    std::vector<rtl::Inst*> cells{a, b, d, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.accepted_swaps == 1,
            "longest edge did not see its on-edge PROFICITE challenger");
    require(sameCoord(b->coord, repaired_b_coord),
            "longest edge was not repaired from its own search rectangle");
    require(result.after.worst_slack_ns > before.worst_slack_ns,
            "edge-local swap did not improve timing");

    std::cout << "SWAPPING_PLACING_EDGE_LINE B='" << b->makeName()
              << "' coord=(" << wrong_b_coord.x << ',' << wrong_b_coord.y
              << ")->(" << b->coord.x << ',' << b->coord.y << ") slack_ns="
              << before.worst_slack_ns << "->" << result.after.worst_slack_ns
              << '\n';
}

void accepted_swap_cap_finishes_pass()
{
    constexpr fpga::Coord correct_a1{10, 19};
    constexpr fpga::Coord wrong_a1{10, 4};
    constexpr fpga::Coord fixed_b1{10, 22};
    constexpr fpga::Coord correct_a2{35, 19};
    constexpr fpga::Coord wrong_a2{35, 4};
    constexpr fpga::Coord fixed_b2{35, 22};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 50, 30);
    Fixture fixture;
    Referable<rtl::Inst>* a1 = fixture.makeRegister("multipass_A1");
    Referable<rtl::Inst>* b1 = fixture.makeRegister("multipass_B1");
    Referable<rtl::Inst>* challenger1 =
        fixture.makeRegister("multipass_challenger1");
    Referable<rtl::Inst>* a2 = fixture.makeRegister("multipass_A2");
    Referable<rtl::Inst>* b2 = fixture.makeRegister("multipass_B2");
    Referable<rtl::Inst>* challenger2 =
        fixture.makeRegister("multipass_challenger2");
    Referable<rtl::Inst>* challenger_peer1 =
        fixture.makeRegister("multipass_challenger_peer1");
    Referable<rtl::Inst>* challenger_peer2 =
        fixture.makeRegister("multipass_challenger_peer2");
    fixture.connect(a1, b1);
    fixture.connect(a2, b2);
    fixture.connect(challenger1, challenger_peer1);
    fixture.connect(challenger2, challenger_peer2);

    Referable<pnr::RegBunch> a1_bunch;
    Referable<pnr::RegBunch> b1_bunch;
    Referable<pnr::RegBunch> challenger1_bunch;
    Referable<pnr::RegBunch> a2_bunch;
    Referable<pnr::RegBunch> b2_bunch;
    Referable<pnr::RegBunch> challenger2_bunch;
    a1_bunch.reg = a1;
    b1_bunch.reg = b1;
    challenger1_bunch.reg = challenger1;
    a2_bunch.reg = a2;
    b2_bunch.reg = b2;
    challenger2_bunch.reg = challenger2;
    a1->bunch_ref.set(&a1_bunch);
    b1->bunch_ref.set(&b1_bunch);
    challenger1->bunch_ref.set(&challenger1_bunch);
    a2->bunch_ref.set(&a2_bunch);
    b2->bunch_ref.set(&b2_bunch);
    challenger2->bunch_ref.set(&challenger2_bunch);

    placeAt(a1, correct_a1);
    placeAt(b1, fixed_b1);
    placeAt(challenger1, wrong_a1);
    placeAt(a2, correct_a2);
    placeAt(b2, fixed_b2);
    placeAt(challenger2, wrong_a2);
    placeAt(challenger_peer1, {12, 19});
    placeAt(challenger_peer2, {37, 19});
    challenger_peer1->outline.fixed = true;
    challenger_peer2->outline.fixed = true;
    exchange(a1, challenger1);
    exchange(a2, challenger2);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "multipass_clock",
        .conn_ptr = nullptr,
        .conn_name = "multipass_clock",
        .period_ns = 0.55,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b1, "D"));
    addEndpoint(timings, clock, fixture.conn(b2, "D"));
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "multipass_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "multipass_proficite_clock",
        .period_ns = 2.0,
        .duty = 50,
    });
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer1, "D"));
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer2, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceTimingAnalysis wrong = analyze(tech, timings);
    require(wrong.violated_endpoints == 2,
            "multipass fixture did not create two timing violations");

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 2;
    swapping.config.maximum_accepted_swaps_per_pass = 1;
    std::vector<rtl::Inst*> cells{
        a1, b1, challenger1, a2, b2, challenger2};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.passes == 2 && result.improving_passes == 2,
            "accepted-swap cap did not split repairs across two passes");
    require(result.attempts >= 2 && result.accepted_swaps == 2,
            "PlaceSwapping did not repair both independent violations");
    require(result.pass_timing_analyses == 2
                && result.acceptance_capped_passes == 2,
            "accepted-swap cap did not validate each bounded batch");
    require(result.locally_corrected_endpoints >= 4,
            "A/B/C setup paths were not corrected after both swaps");
    require(result.after.violated_endpoints == 0,
            "PlaceSwapping left the second independent violation unfixed");
    require(sameCoord(a1->coord, correct_a1)
                && sameCoord(a2->coord, correct_a2),
            "PlaceSwapping did not recover both independent placements");

    std::cout
        << "SWAPPING_PLACING_ACCEPTANCE_CAP passes=" << result.passes
        << " improving_passes=" << result.improving_passes
        << " attempts=" << result.attempts
        << " accepted=" << result.accepted_swaps
        << " violations=" << wrong.violated_endpoints << "->"
        << result.after.violated_endpoints << '\n';
}

void a_bunch_can_move_three_times()
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 32, 2);
    Fixture fixture;
    std::array<Referable<rtl::Inst>*, 6> insts{
        fixture.makeRegister("repeat_source"),
        fixture.makeRegister("repeat_sink"),
        fixture.makeRegister("repeat_moving"),
        fixture.makeRegister("repeat_C1"),
        fixture.makeRegister("repeat_C2"),
        fixture.makeRegister("repeat_C3")};
    std::array<Referable<pnr::RegBunch>, 6> bunches;
    const std::array<fpga::Coord, 6> coords{
        fpga::Coord{0, 0}, {20, 0}, {30, 1}, {1, 1}, {14, 1},
        {10, 1}};
    std::vector<rtl::Inst*> cells;
    for (size_t i = 0; i < insts.size(); ++i) {
        bunches[i].reg = insts[i];
        insts[i]->bunch_ref.set(&bunches[i]);
        placeAt(insts[i], coords[i]);
        insts[i]->outline.fixed = i < 2;
        cells.push_back(insts[i]);
    }
    fixture.connect(insts[0], insts[2]);
    fixture.connect(insts[2], insts[1]);
    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "repeat_clock", .period_ns = 0.4, .duty = 50});
    Referable<rtl::Clock> reserve_clock(rtl::Clock{
        .name = "repeat_reserve_clock", .period_ns = 10.0, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(insts[2], "D"));
    addEndpoint(timings, clock, fixture.conn(insts[1], "D"));
    // Unconnected spare registers have timing reserve independent of position;
    // this fixture isolates reuse eligibility from collateral path damage.
    for (size_t i = 3; i < 6; ++i)
        addEndpoint(timings, reserve_clock, fixture.conn(insts[i], "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = tech.place.aspect_y = 1;
    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 3;
    swapping.config.maximum_accepted_swaps_per_pass = 1;
    swapping.config.replacement_search_radius = 0;
    auto result = swapping.run(timings, cells);
    // The same moving bunch repairs its input, then its output, then reaches
    // the compromise: x=30 -> 1 -> 14 -> 10. Fixed endpoints cannot move.
    require(result.accepted_swaps == 3 && result.passes == 3,
            "same active bunch could not move three times in one stage");
    require(sameCoord(insts[2]->coord, {10, 1}) &&
                sameCoord(insts[3]->coord, {30, 1}) &&
                sameCoord(insts[4]->coord, {1, 1}) &&
                sameCoord(insts[5]->coord, {14, 1}),
            "third repair did not retain the expected bunch placements");
    require(result.after.worst_slack_ns > result.before.worst_slack_ns,
            "repeated active movement did not improve timing");
    std::cout << "SWAPPING_PLACING_REPEATED_ACTIVE accepted="
              << result.accepted_swaps << " passes=" << result.passes << '\n';
}

void a_challenger_can_be_reused_across_passes()
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 32, 2);
    Fixture fixture;
    std::array<Referable<rtl::Inst>*, 7> insts{
        fixture.makeRegister("reuse_source1"),
        fixture.makeRegister("reuse_source2"),
        fixture.makeRegister("reuse_source3"),
        fixture.makeRegister("reuse_sink1"),
        fixture.makeRegister("reuse_sink2"),
        fixture.makeRegister("reuse_sink3"),
        fixture.makeRegister("reuse_C")};
    std::array<Referable<pnr::RegBunch>, 7> bunches;
    const std::array<fpga::Coord, 7> coords{
        fpga::Coord{0, 0}, {11, 0}, {21, 0}, {10, 1}, {20, 1},
        {30, 1}, {1, 1}};
    std::vector<rtl::Inst*> cells;
    for (size_t i = 0; i < insts.size(); ++i) {
        bunches[i].reg = insts[i];
        insts[i]->bunch_ref.set(&bunches[i]);
        placeAt(insts[i], coords[i]);
        insts[i]->outline.fixed = i < 3;
        cells.push_back(insts[i]);
    }
    for (size_t i = 0; i < 3; ++i)
        fixture.connect(insts[i], insts[i + 3]);
    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "reuse_clock", .period_ns = 0.25, .duty = 50});
    Referable<rtl::Clock> reserve_clock(rtl::Clock{
        .name = "reuse_reserve_clock", .period_ns = 10.0, .duty = 50});
    clk::Timings timings;
    for (size_t i = 3; i < 6; ++i)
        addEndpoint(timings, clock, fixture.conn(insts[i], "D"));
    addEndpoint(timings, reserve_clock, fixture.conn(insts[6], "D"));
    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = tech.place.aspect_y = 1;
    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 3;
    swapping.config.maximum_accepted_swaps_per_pass = 1;
    swapping.config.replacement_search_radius = 0;
    auto result = swapping.run(timings, cells);
    // Only C has PROFICITE timing. It is reused successively at x=1,10,20,
    // then ends at x=30 after helping each independent deficient endpoint.
    require(result.accepted_swaps == 3 && result.passes == 3,
            "previously displaced C was not reusable in later passes");
    require(sameCoord(insts[3]->coord, {1, 1}) &&
                sameCoord(insts[4]->coord, {10, 1}) &&
                sameCoord(insts[5]->coord, {20, 1}) &&
                sameCoord(insts[6]->coord, {30, 1}),
            "reusing C did not repair all three reference placements");
    require(result.after.violated_endpoints == 0,
            "reusing C left a reference timing violation");
    std::cout << "SWAPPING_PLACING_REPEATED_CHALLENGER accepted="
              << result.accepted_swaps << " passes=" << result.passes << '\n';
}

void strongest_candidate_wins_single_traversal()
{
    constexpr fpga::Coord wrong_a{2, 2};
    constexpr fpga::Coord first_step{6, 2};
    constexpr fpga::Coord second_step{10, 2};
    constexpr fpga::Coord fixed_b{22, 2};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 30, 12);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("requeue_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("requeue_B");
    Referable<rtl::Inst>* challenger1 =
        fixture.makeRegister("requeue_challenger1");
    Referable<rtl::Inst>* challenger2 =
        fixture.makeRegister("requeue_challenger2");
    Referable<rtl::Inst>* peer1 =
        fixture.makeRegister("requeue_peer1");
    Referable<rtl::Inst>* peer2 =
        fixture.makeRegister("requeue_peer2");
    fixture.connect(a, b);
    fixture.connect(challenger1, peer1);
    fixture.connect(challenger2, peer2);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> challenger1_bunch;
    Referable<pnr::RegBunch> challenger2_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    challenger1_bunch.reg = challenger1;
    challenger2_bunch.reg = challenger2;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    challenger1->bunch_ref.set(&challenger1_bunch);
    challenger2->bunch_ref.set(&challenger2_bunch);

    placeAt(a, wrong_a);
    placeAt(b, fixed_b);
    placeAt(challenger1, first_step);
    placeAt(challenger2, second_step);
    placeAt(peer1, {6, 6});
    placeAt(peer2, {10, 6});
    b->outline.fixed = true;
    peer1->outline.fixed = true;
    peer2->outline.fixed = true;

    Referable<rtl::Clock> critical_clock(rtl::Clock{
        .name = "requeue_critical_clock",
        .conn_ptr = nullptr,
        .conn_name = "requeue_critical_clock",
        .period_ns = 0.40,
        .duty = 50,
    });
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "requeue_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "requeue_proficite_clock",
        .period_ns = 3.0,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, critical_clock, fixture.conn(b, "D"));
    addEndpoint(timings, proficite_clock, fixture.conn(peer1, "D"));
    addEndpoint(timings, proficite_clock, fixture.conn(peer2, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceTimingAnalysis before = analyze(tech, timings);
    require(before.worst_slack_ns < -0.1,
            "requeue regression did not create a timing deficit");

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 2;
    std::vector<rtl::Inst*> cells{a, b, challenger1, challenger2};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.passes == 2 && result.accepted_swaps == 1,
            "strongest candidate selection did not finish in one swap");
    require(result.pass_timing_analyses == 1,
            "strongest candidate received an unexpected extra validation");
    require(sameCoord(a->coord, second_step),
            "PlaceSwapping committed an earlier, weaker candidate");
    require(result.after.worst_slack_ns > before.worst_slack_ns,
            "two-step endpoint repair did not improve timing");

    std::cout
        << "SWAPPING_PLACING_STRONGEST_CANDIDATE A=(" << wrong_a.x << ','
        << wrong_a.y
        << ")->(" << a->coord.x << ',' << a->coord.y
        << ") accepted=" << result.accepted_swaps
        << " passes=" << result.passes << '\n';
}

void subthreshold_negative_slack_is_accepted()
{
    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 30, 30);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("tolerance_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("tolerance_B");
    fixture.connect(a, b);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    placeAt(a, {10, 4});
    placeAt(b, {10, 22});

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "tolerance_clock",
        .conn_ptr = nullptr,
        .conn_name = "tolerance_clock",
        .period_ns = 0.65,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(b, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.slack_tolerance_ns = 0.10;
    std::vector<rtl::Inst*> cells{a, b};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.after.violated_endpoints == 1
                && result.after.worst_slack_ns < 0,
            "tolerance regression lost the exact negative slack");
    require(result.after.worst_slack_ns >= -0.10
                && result.actionable_violations_before == 0
                && result.actionable_violations_after == 0,
            "subthreshold negative slack was not timing-accepted");
    require(result.attempts == 0,
            "PlaceSwapping tried to repair an accepted subthreshold slack");

    std::cout
        << "SWAPPING_PLACING_TOLERANCE slack_ns="
        << result.after.worst_slack_ns
        << " tolerance_ns=" << swapping.config.slack_tolerance_ns
        << " actionable=" << result.actionable_violations_after << '\n';
}

void provisional_swaps_are_timed_once_at_pass_boundary()
{
    constexpr fpga::Coord a_coord{18, 24};
    constexpr fpga::Coord good_b_coord{17, 22};
    constexpr fpga::Coord d_coord{14, 7};
    constexpr fpga::Coord challenger_coord{14, 8};

    fpga::TileType tile_type = makeTileType();
    resetDevice(tile_type, 50, 50);
    Fixture fixture;
    Referable<rtl::Inst>* a = fixture.makeRegister("protected_input_A");
    Referable<rtl::Inst>* b = fixture.makeRegister("protected_input_B");
    Referable<rtl::Inst>* d = fixture.makeRegister("protected_output_D");
    Referable<rtl::Inst>* challenger =
        fixture.makeRegister("protected_challenger");
    Referable<rtl::Inst>* challenger_peer =
        fixture.makeRegister("protected_challenger_peer");
    fixture.connect(a, b);
    fixture.connect(b, d);
    fixture.connect(challenger, challenger_peer);

    Referable<pnr::RegBunch> a_bunch;
    Referable<pnr::RegBunch> b_bunch;
    Referable<pnr::RegBunch> d_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    a_bunch.reg = a;
    b_bunch.reg = b;
    d_bunch.reg = d;
    challenger_bunch.reg = challenger;
    a->bunch_ref.set(&a_bunch);
    b->bunch_ref.set(&b_bunch);
    d->bunch_ref.set(&d_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    placeAt(a, a_coord);
    placeAt(b, good_b_coord);
    placeAt(d, d_coord);
    placeAt(challenger, challenger_coord);
    placeAt(challenger_peer, {14, 9});
    a->outline.fixed = true;
    d->outline.fixed = true;
    challenger_peer->outline.fixed = true;

    Referable<rtl::Clock> input_clock(rtl::Clock{
        .name = "protected_input_clock",
        .conn_ptr = nullptr,
        .conn_name = "protected_input_clock",
        .period_ns = 0.468,
        .duty = 50,
    });
    Referable<rtl::Clock> output_clock(rtl::Clock{
        .name = "protected_output_clock",
        .conn_ptr = nullptr,
        .conn_name = "protected_output_clock",
        .period_ns = 0.388,
        .duty = 50,
    });
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "protected_proficite_clock",
        .conn_ptr = nullptr,
        .conn_name = "protected_proficite_clock",
        .period_ns = 2.0,
        .duty = 50,
    });
    clk::Timings timings;
    addEndpoint(timings, input_clock, fixture.conn(b, "D"));
    addEndpoint(timings, output_clock, fixture.conn(d, "D"));
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech::clocked_ports.clear();
    technology::Tech::clocked_ports.emplace("FD", "C");
    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;

    pnr::PlaceTimingAnalysis before = analyze(tech, timings);
    require(before.worst_slack_ns < -0.3,
            "moving-input regression did not create its output violation");

    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 1;
    std::vector<rtl::Inst*> cells{a, b, d, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.attempts == 1 && result.pass_timing_analyses == 1
                && result.locally_corrected_endpoints >= 3,
            "PlaceSwapping did not defer timing to one pass-boundary analysis");
    require(result.accepted_swaps == 1
                && !sameCoord(b->coord, good_b_coord),
            "provisional swap was not retained after improving global timing");
    require(result.after.worst_slack_ns > before.worst_slack_ns,
            "pass-boundary analysis accepted a globally worse placement");

    std::cout
        << "SWAPPING_PLACING_PASS_BOUNDARY_TIMING B='"
        << b->makeName() << "' coord=(" << b->coord.x << ',' << b->coord.y
        << ") attempts=" << result.attempts
        << " pass_timing_analyses=" << result.pass_timing_analyses
        << " slack_ns=" << result.after.worst_slack_ns << '\n';
}

void strong_improvement_allows_bounded_global_regression()
{
    pnr::PlaceSwapping swapping;
    swapping.config.strong_improvement = 0.80;
    swapping.config.maximum_global_regression = 0.05;
    swapping.config.slack_tolerance_ns = 0.10;
    pnr::PlaceTimingAnalysis current;
    current.worst_slack_ns = -0.80;
    current.total_negative_slack_ns = 100.0;

    pnr::PlaceTimingAnalysis acceptable = current;
    acceptable.worst_slack_ns = -0.82;
    acceptable.total_negative_slack_ns = 104.0;
    bool relaxed = false;
    require(swapping.acceptsTimingTradeoff(
                0.80, current, acceptable, &relaxed) && relaxed,
            "strong improvement did not allow bounded global regression");

    pnr::PlaceTimingAnalysis excessive_tns = acceptable;
    excessive_tns.total_negative_slack_ns = 106.0;
    require(!swapping.acceptsTimingTradeoff(
                0.80, current, excessive_tns),
            "strong improvement allowed more than 5% TNS regression");

    pnr::PlaceTimingAnalysis excessive_wns = acceptable;
    excessive_wns.worst_slack_ns = -0.85;
    require(!swapping.acceptsTimingTradeoff(
                0.80, current, excessive_wns),
            "strong improvement allowed more than 5% WNS-deficit regression");
    require(!swapping.acceptsTimingTradeoff(
                0.79, current, acceptable),
            "weak endpoint improvement enabled relaxed acceptance");

    // A strong local repair may give back an intermediate global gain as long
    // as it remains inside the non-compounding run-entry envelope. These are
    // the values observed in the 50x50 placement puzzle.
    pnr::PlaceTimingAnalysis run_entry;
    run_entry.worst_slack_ns = -0.802;
    run_entry.total_negative_slack_ns = 58.239;
    pnr::PlaceTimingAnalysis improved_current;
    improved_current.worst_slack_ns = -0.539;
    improved_current.total_negative_slack_ns = 56.811;
    pnr::PlaceTimingAnalysis temporary_tradeoff;
    temporary_tradeoff.worst_slack_ns = -0.649;
    temporary_tradeoff.total_negative_slack_ns = 57.311;
    relaxed = false;
    require(swapping.acceptsTimingTradeoff(
                0.80, improved_current, temporary_tradeoff, &relaxed,
                &run_entry) && relaxed,
            "strong repair could not trade an intermediate gain inside the "
            "run-entry envelope");

    pnr::PlaceTimingAnalysis compounded_regression = run_entry;
    compounded_regression.worst_slack_ns = -0.85;
    compounded_regression.total_negative_slack_ns = 62.0;
    require(!swapping.acceptsTimingTradeoff(
                0.80, improved_current, compounded_regression, nullptr,
                &run_entry),
            "relaxed swaps compounded beyond the run-entry 5% envelope");

    // Once swapping has established a much better global state, that state
    // becomes the regression reference. This is the late 50x50-puzzle case
    // which previously accepted -0.189 -> -0.454 ns merely because -0.454
    // was still better than the original -0.802 ns run entry.
    pnr::PlaceTimingAnalysis best_state;
    best_state.worst_slack_ns = -0.189;
    best_state.total_negative_slack_ns = 32.598;
    pnr::PlaceTimingAnalysis destructive_late_trade = best_state;
    destructive_late_trade.worst_slack_ns = -0.454;
    destructive_late_trade.total_negative_slack_ns = 32.863;
    require(!swapping.acceptsTimingTradeoff(
                1.0, best_state, destructive_late_trade, nullptr,
                &best_state),
            "late strong repair escaped the best-state 5% envelope");
}

void temperature_cooling_is_configurable()
{
    pnr::PlaceSwapping swapping;
    constexpr std::array<double, 12> expected{
        1.0, 0.9, 0.8, 0.7, 0.6, 0.5,
        0.4, 0.3, 0.2, 0.1, 0.0, 0.0};
    for (size_t pass = 0; pass < expected.size(); ++pass) {
        require(std::abs(swapping.temperatureForPass(1.0, pass) - expected[pass])
                    < 1e-9,
                "PlaceSwapping TEMPERATURE cooling schedule is incorrect");
    }
    require(std::abs(swapping.criticalSlackLimitForPass(-2.0, 1.0, 0) + 2.0)
                < 1e-9,
            "TEMPERATURE incorrectly imposed an absolute A/B slack floor");
    require(std::abs(swapping.criticalSlackLimitForPass(0.4, 1.0, 0) + 1.0)
                < 1e-9,
            "TEMPERATURE did not set the pass-one A/B target");
    require(std::abs(swapping.challengerSlackLimitForPass(0.4, 1.0, 0) + 0.6)
                < 1e-9,
            "TEMPERATURE did not allow one ns of relative C degradation");
    require(std::abs(swapping.challengerSlackLimitForPass(-2.0, 1.0, 0) + 3.0)
                < 1e-9,
            "TEMPERATURE incorrectly imposed an absolute C slack floor");
    swapping.config.temperature_cooling_per_pass_ns = 0.05;
    require(std::abs(swapping.temperatureForPass(1.0, 3) - 0.85) < 1e-9,
            "PlaceSwapping ignored configured TEMPERATURE cooling");
    std::cout
        << "SWAPPING_PLACING_TEMPERATURE schedule="
           "1.0,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0.0\n";
}

}

int main()
{
    try {
        reference_vertical_misplacement_is_recovered();
        vacated_origin_is_a_challenger_fallback();
        rectangle_proficite_region_is_used();
        padded_horizontal_rectangle_proficite_region_is_used();
        longest_edge_uses_its_own_proficite_rectangle();
        accepted_swap_cap_finishes_pass();
        a_bunch_can_move_three_times();
        a_challenger_can_be_reused_across_passes();
        strongest_candidate_wins_single_traversal();
        subthreshold_negative_slack_is_accepted();
        provisional_swaps_are_timed_once_at_pass_boundary();
        strong_improvement_allows_bounded_global_regression();
        temperature_cooling_is_configurable();
    }
    catch (const TestFailure& failure) {
        std::cerr << "swapping_placing_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "swapping_placing_test passed\n";
    return 0;
}
