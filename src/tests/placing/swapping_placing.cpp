#include "Device.h"
#include "PlaceSwapping.h"
#include "PlaceTiming.h"
#include "RegBunch.h"
#include "Tech.h"
#include "Tile.h"

#include <cmath>
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

fpga::TileType makeTileType()
{
    fpga::TileType type{"SWAPPING_PLACING_TEST", 1, 0};
    fpga::Element fd;
    fd.name = "REG";
    fd.type = fpga::ELEMENT_FD;
    fd.bitmap_pos = 0;
    fd.elements_to_left = fpga::ELEMENT_FD;
    type.elements.push_back(std::move(fd));
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
    int placed = tile.tryAddAt(inst, fdPos());
    require(placed == fdPos(), "failed to create reference placement");
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
    constexpr fpga::Coord correct_a{23, 12};
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

    require(result.accepted_swaps == 1,
            "PlaceSwapping did not accept the reference repair");
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

void expanded_scope_is_a_fallback_after_core_exhaustion()
{
    constexpr fpga::Coord wrong_a{10, 4};
    constexpr fpga::Coord repaired_a{10, 10};
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

    // Occupy every replacement Tile in the five-Tile core. The challenger
    // can be relocated only after the ten-Tile outer scope is enabled.
    for (int dy = -5; dy <= 5; ++dy) {
        for (int dx = -5; dx <= 5; ++dx) {
            if (std::abs(dx) + std::abs(dy) > 5
                || (dx == 0 && dy == 0)) {
                continue;
            }
            Referable<rtl::Inst>* blocker = fixture.makeRegister(
                "scope_blocker_" + std::to_string(dx + 5) + "_"
                + std::to_string(dy + 5));
            placeAt(blocker, challenger_peer_coord + fpga::Coord{dx, dy});
            blocker->outline.fixed = true;
        }
    }

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "scope_clock",
        .conn_ptr = nullptr,
        .conn_name = "scope_clock",
        // Keep the endpoint violated after the known baseline repair. That
        // forces the search to exhaust the complete original geometry and
        // then exercise the configured larger replacement scope.
        .period_ns = 0.25,
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
    swapping.config.replacement_search_radius = 20;
    swapping.config.maximum_passes = 5;
    std::vector<rtl::Inst*> cells{a, b, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    int challenger_peer_distance =
        std::abs(challenger->coord.x - challenger_peer_coord.x)
        + std::abs(challenger->coord.y - challenger_peer_coord.y);
    require(result.scope_expansions == 2
                && result.expanded_beyond_baseline_scope,
            "larger scope was not enabled after core and baseline exhausted");
    require(result.accepted_swaps == 1 && sameCoord(a->coord, repaired_a),
            "expanded scope did not recover the blocked core relocation");
    require(challenger_peer_distance > 5 && challenger_peer_distance <= 10,
            "challenger was not placed in the expanded replacement ring");
    require(result.after.worst_slack_ns
                    >= result.baseline_scope_best.worst_slack_ns - 1e-9
                && (result.after.worst_slack_ns
                        > result.baseline_scope_best.worst_slack_ns + 1e-9
                    || result.after.total_negative_slack_ns
                        <= result.baseline_scope_best.total_negative_slack_ns
                            + 1e-9),
            "larger geometry lost the best result found by baseline geometry");
}

void multiple_passes_receive_fresh_attempt_budgets()
{
    constexpr fpga::Coord correct_a1{10, 10};
    constexpr fpga::Coord wrong_a1{10, 4};
    constexpr fpga::Coord fixed_b1{10, 22};
    constexpr fpga::Coord correct_a2{35, 10};
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
    fixture.connect(a1, b1);
    fixture.connect(a2, b2);

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
    swapping.config.maximum_attempts = 1;
    swapping.config.additional_attempts_per_band = 0;
    std::vector<rtl::Inst*> cells{
        a1, b1, challenger1, a2, b2, challenger2};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.passes == 2 && result.improving_passes == 2,
            "PlaceSwapping did not execute two improving traversals");
    require(result.attempts == 2 && result.accepted_swaps == 2,
            "PlaceSwapping did not refresh its one-attempt pass budget");
    require(result.after.violated_endpoints == 0,
            "PlaceSwapping left the second independent violation unfixed");
    require(sameCoord(a1->coord, correct_a1)
                && sameCoord(a2->coord, correct_a2),
            "PlaceSwapping did not recover both independent placements");

    std::cout
        << "SWAPPING_PLACING_MULTIPASS passes=" << result.passes
        << " improving_passes=" << result.improving_passes
        << " attempts=" << result.attempts
        << " accepted=" << result.accepted_swaps
        << " violations=" << wrong.violated_endpoints << "->"
        << result.after.violated_endpoints << '\n';
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
    require(result.passes == 0 && result.attempts == 0,
            "PlaceSwapping tried to repair an accepted subthreshold slack");

    std::cout
        << "SWAPPING_PLACING_TOLERANCE slack_ns="
        << result.after.worst_slack_ns
        << " tolerance_ns=" << swapping.config.slack_tolerance_ns
        << " actionable=" << result.actionable_violations_after << '\n';
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

}

int main()
{
    try {
        reference_vertical_misplacement_is_recovered();
        expanded_scope_is_a_fallback_after_core_exhaustion();
        multiple_passes_receive_fresh_attempt_budgets();
        subthreshold_negative_slack_is_accepted();
        strong_improvement_allows_bounded_global_regression();
    }
    catch (const TestFailure& failure) {
        std::cerr << "swapping_placing_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "swapping_placing_test passed\n";
    return 0;
}
