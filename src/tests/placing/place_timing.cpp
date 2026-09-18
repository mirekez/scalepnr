#include "Device.h"
#include "PlaceDesign.h"
#include "PlaceSwapping.h"
#include "PlaceTiming.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

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

bool near(double left, double right, double epsilon = 1e-9)
{
    return std::abs(left - right) <= epsilon;
}

int fdPos(int bit = 0)
{
    int site = bit/8;
    int lane = bit%8;
    int bel = lane%4;
    int column = lane >= 4 ? 64 : 0;
    return site*128 + column + bel*4;
}

fpga::TileType makeTileType()
{
    fpga::TileType type{"PLACE_TIMING_TEST", 1, 0};
    fpga::Element fd;
    fd.name = "REG0";
    fd.type = fpga::ELEMENT_FD;
    fd.bitmap_pos = 0;
    fd.elements_to_left = fpga::ELEMENT_FD;
    type.elements.push_back(std::move(fd));
    fpga::Element lut;
    lut.name = "LOGIC0";
    lut.type = fpga::ELEMENT_LUT5;
    lut.bitmap_pos = 0;
    lut.elements_to_left = fpga::ELEMENT_LUT5;
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
        rtl::Port data;
        data.name = "D";
        data.type = rtl::Port::PORT_IN;
        data.index = 0;
        cell->ports.emplace_back(std::move(data));
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

    Referable<rtl::Inst>* makeLogic(const std::string& name, int inputs = 2)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name + "_cell";
        cell->type = "LUT" + std::to_string(inputs);
        cell->module_ref.set(&primitive);
        for (int index = 0; index < inputs; ++index) {
            rtl::Port input;
            input.name = "I" + std::to_string(index);
            input.type = rtl::Port::PORT_IN;
            input.index = index;
            cell->ports.emplace_back(std::move(input));
        }
        rtl::Port output;
        output.name = "O";
        output.type = rtl::Port::PORT_OUT;
        output.index = 0;
        cell->ports.emplace_back(std::move(output));

        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->cnt_inputs = inputs;
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
        require(output && input, "missing synthetic timing port");
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);
        auto& net = parent.nets.emplace_back();
        net.name = "timing_" + std::to_string(designator);
        net.designators.push_back(designator++);
    }

    void connect(Referable<rtl::Inst>* driver, const std::string& output_name,
                 Referable<rtl::Inst>* sink, const std::string& input_name)
    {
        Referable<rtl::Conn>* output = conn(driver, output_name);
        Referable<rtl::Conn>* input = conn(sink, input_name);
        require(output && input, "missing synthetic combinational port");
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);
        auto& net = parent.nets.emplace_back();
        net.name = "timing_" + std::to_string(designator);
        net.designators.push_back(designator++);
    }
};

void addEndpoint(clk::Timings& timings, rtl::Clock& clock,
                 Referable<rtl::Conn>* data_in)
{
    auto& info = timings.clocked_inputs[&clock].emplace_back();
    info.data_in = data_in;
    info.path.data_in = data_in;
    info.path.data_output = data_in ? data_in->follow() : nullptr;
}

void placeAt(Referable<rtl::Inst>* inst, int x, int y)
{
    fpga::Device& device = fpga::Device::current();
    fpga::Tile& tile = device.tile_grid[y*device.size_width + x];
    int placed = tile.tryAddAt(inst, fdPos());
    require(placed == fdPos(), "failed to create synthetic placement");
}

void placeLogicAt(Referable<rtl::Inst>* inst, int x, int y)
{
    fpga::Device& device = fpga::Device::current();
    fpga::Tile& tile = device.tile_grid[y*device.size_width + x];
    constexpr int lut_pos = 3;
    int placed = tile.tryAddAt(inst, lut_pos);
    require(placed == lut_pos, "failed to create synthetic logic placement");
}

void calibrated_manhattan_delay()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 8, 4);
    Fixture fixture;
    auto* source = fixture.makeRegister("source");
    auto* sink = fixture.makeRegister("sink");
    fixture.connect(source, sink);
    placeAt(source, 1, 1);
    placeAt(sink, 6, 3);

    pnr::PlaceTiming estimator;
    double delay = estimator.estimateWireDelay(
        *fixture.conn(sink, "D"), *fixture.conn(source, "Q"));
    double expected = 0.010 + 5*0.035 + 2*0.040 + 0.005;
    require(near(delay, expected),
            "Manhattan wire delay did not use the built-in calibration");
}

void local_setup_correction_matches_full_analysis()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 12, 1);
    Fixture fixture;
    auto* source = fixture.makeRegister("correction_source");
    auto* sink = fixture.makeRegister("correction_sink");
    fixture.connect(source, sink);
    placeAt(source, 0, 0);
    placeAt(sink, 10, 0);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "correction_clk", .conn_ptr = nullptr,
        .conn_name = "correction_clk", .period_ns = 1.0, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));

    pnr::PlaceTiming estimator;
    pnr::PlaceTimingAnalysis local = estimator.analyze(timings);
    require(local.endpoint_details.size() == 1,
            "local correction fixture lost its setup endpoint");

    source->tile->unassign(source);
    placeAt(source, 8, 0);
    estimator.correctSetupTiming(local.endpoint_details.front());
    pnr::PlaceTimingAnalysis exact = estimator.analyze(timings);

    const pnr::PlaceTimingEndpoint& corrected = local.endpoint_details.front();
    const pnr::PlaceTimingEndpoint& recalculated = exact.endpoint_details.front();
    require(near(corrected.arrival_ns, recalculated.arrival_ns)
                && near(corrected.slack_ns, recalculated.slack_ns)
                && corrected.critical_edges.size() == 1
                && near(corrected.critical_edges.front().wire_delay_ns,
                        recalculated.critical_edges.front().wire_delay_ns),
            "local setup correction disagrees with full timing analysis");
}

void violation_and_force_extraction()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 10, 1);
    Fixture fixture;
    auto* source = fixture.makeRegister("source");
    auto* sink = fixture.makeRegister("sink");
    fixture.connect(source, sink);
    placeAt(source, 0, 0);
    placeAt(sink, 8, 0);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.20, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));

    pnr::PlaceTiming estimator;
    pnr::PlaceTimingAnalysis analysis = estimator.analyze(timings);
    require(analysis.endpoints == 1 && analysis.violated_endpoints == 1,
            "violating endpoint was not detected");
    require(analysis.endpoint_details.front().critical_edges.size() == 1,
            "critical placed net was not retained");
    require(analysis.forces.size() == 2,
            "both ends of a critical net did not receive attraction forces");
    const auto sink_force = std::ranges::find_if(
        analysis.forces, [&](const pnr::PlaceTimingForce& force) {
            return force.inst == sink;
        });
    require(sink_force != analysis.forces.end() && sink_force->x < 0,
            "sink attraction did not point toward its driver");
}

void combinational_critical_path_uses_cell_and_wire_delays()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 12, 1);
    Fixture fixture;
    auto* far_source = fixture.makeRegister("far_source");
    auto* near_source = fixture.makeRegister("near_source");
    auto* logic = fixture.makeLogic("logic");
    auto* sink = fixture.makeRegister("sink");
    auto* second_sink = fixture.makeRegister("second_sink");
    fixture.connect(far_source, "Q", logic, "I0");
    fixture.connect(near_source, "Q", logic, "I1");
    fixture.connect(logic, "O", sink, "D");
    fixture.connect(logic, "O", second_sink, "D");
    placeAt(far_source, 0, 0);
    placeAt(near_source, 2, 0);
    placeLogicAt(logic, 3, 0);
    placeAt(sink, 6, 0);
    placeAt(second_sink, 4, 0);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.25, .duty = 50});
    clk::Timings timings;
    auto& infos = timings.clocked_inputs[&clock];
    infos.reserve(2);
    auto& info = infos.emplace_back();
    info.data_in = fixture.conn(sink, "D");
    info.path.data_in = info.data_in;
    info.path.data_output = fixture.conn(logic, "O");
    auto& far_path = info.path.sub_paths.emplace_back();
    far_path.data_in = fixture.conn(logic, "I0");
    far_path.data_output = fixture.conn(far_source, "Q");
    auto& near_path = info.path.sub_paths.emplace_back();
    near_path.data_in = fixture.conn(logic, "I1");
    near_path.data_output = fixture.conn(near_source, "Q");
    auto& reused = infos.emplace_back();
    reused.data_in = fixture.conn(second_sink, "D");
    reused.path.data_in = reused.data_in;
    reused.path.precalculated = &info.path;

    pnr::PlaceTiming estimator;
    estimator.tech = &technology::Tech::current();
    pnr::PlaceTimingAnalysis analysis = estimator.analyze(timings);
    double output_wire = 0.010 + 3*0.035 + 0.002;
    double critical_input_wire = 0.010 + 3*0.035;
    double expected = output_wire + critical_input_wire + 0.080;
    require(near(analysis.endpoint_details.front().arrival_ns, expected),
            "placed timing did not combine critical input wire and cell-arc delay");
    require(analysis.endpoint_details.front().critical_edges.size() == 2,
            "critical combinational path did not retain both placed nets");
    require(analysis.endpoint_details.front().critical_edges.back().driver
                == far_source,
            "critical combinational input was not selected by arrival time");
    require(analysis.endpoints == 2 && analysis.evaluated_nodes == 3,
            "precalculated combinational output was not shared across endpoints");
    auto reused_result = std::ranges::find_if(
        analysis.endpoint_details, [&](const pnr::PlaceTimingEndpoint& endpoint) {
            return endpoint.data_in == reused.data_in;
        });
    double reused_output_wire = 0.010 + 0.035 + 0.002;
    require(reused_result != analysis.endpoint_details.end()
                && near(reused_result->arrival_ns,
                        reused_output_wire + critical_input_wire + 0.080),
            "precalculated path reused another endpoint's outgoing wire delay");

    // Moving an input that was NOT on the cached critical path must still
    // invalidate both fanout endpoints, select the competing branch exactly,
    // and restore the previous branch and aggregates on transaction rollback.
    auto* unrelated_source = fixture.makeRegister("unrelated_source");
    auto* unrelated_sink = fixture.makeRegister("unrelated_sink");
    fixture.connect(unrelated_source, unrelated_sink);
    placeAt(unrelated_source, 8, 0);
    placeAt(unrelated_sink, 9, 0);
    Referable<rtl::Clock> unrelated_clock(rtl::Clock{
        .name = "unrelated", .conn_ptr = nullptr, .conn_name = "unrelated",
        .period_ns = 2.0, .duty = 50});
    addEndpoint(timings, unrelated_clock, fixture.conn(unrelated_sink, "D"));
    analysis = estimator.analyze(timings);
    pnr::PlaceTimingIncremental incremental(estimator, analysis);
    auto original = analysis;
    for (int trial = 0; trial < 8; ++trial) {
        near_source->coord.x = 10;
        near_source->tile.set(&fpga::Device::current().tile_grid[10]);
        auto transaction = incremental.update({near_source});
        auto exact = estimator.analyze(timings);
        require(transaction.size() == 2,
                "noncritical shared input did not invalidate both endpoints");
        require(near(analysis.worst_slack_ns, exact.worst_slack_ns)
                    && near(analysis.total_negative_slack_ns,
                            exact.total_negative_slack_ns)
                    && analysis.violated_endpoints == exact.violated_endpoints,
                "incremental WNS/TNS differs from full analysis");
        for (size_t i = 0; i < analysis.endpoint_details.size(); ++i) {
            require(near(analysis.endpoint_details[i].slack_ns,
                         exact.endpoint_details[i].slack_ns),
                    "incremental endpoint differs from full timing");
            bool unrelated = analysis.endpoint_details[i].data_in ==
                fixture.conn(unrelated_sink, "D");
            require(analysis.endpoint_details[i].critical_edges.back().driver
                            == (unrelated ? unrelated_source : near_source),
                    "incremental setup failed to switch the critical input");
        }
        incremental.restore(std::move(transaction));
        near_source->coord.x = 2;
        near_source->tile.set(&fpga::Device::current().tile_grid[2]);
        require(near(analysis.worst_slack_ns, original.worst_slack_ns)
                    && near(analysis.total_negative_slack_ns,
                            original.total_negative_slack_ns),
                "incremental rollback did not restore branch and timing");
        for (size_t i = 0; i < analysis.endpoint_details.size(); ++i)
            require(analysis.endpoint_details[i].critical_edges.back().driver ==
                        original.endpoint_details[i].critical_edges.back().driver,
                    "incremental rollback did not restore the critical branch");
    }
    // Forward refresh must reselect the same shared critical branches without
    // constructing or restoring a Transaction. Repeated cells are deduplicated.
    for (int destination : {10, 2, 10}) {
        near_source->coord.x = destination;
        near_source->tile.set(&fpga::Device::current().tile_grid[destination]);
        incremental.updateForward({near_source, near_source});
        auto exact = estimator.analyze(timings);
        require(near(analysis.worst_slack_ns, exact.worst_slack_ns)
                    && near(analysis.total_negative_slack_ns, exact.total_negative_slack_ns)
                    && analysis.violated_endpoints == exact.violated_endpoints,
                "forward timing refresh disagrees with full analysis");
        for (size_t i = 0; i < analysis.endpoint_details.size(); ++i) {
            require(near(analysis.endpoint_details[i].slack_ns, exact.endpoint_details[i].slack_ns)
                        && analysis.endpoint_details[i].critical_edges.back().driver
                            == exact.endpoint_details[i].critical_edges.back().driver,
                    "forward timing refresh retained an obsolete critical branch");
        }
        require(near_source->coord.x == destination,
                "forward timing refresh undid a committed position");
    }

    // Direct object updates must handle shared branches, stationary endpoints,
    // and all wires of a simultaneous shift without revisiting unrelated cones.
    for (int lifetime = 0; lifetime < 2; ++lifetime) {
        near_source->coord = {2, 0};
        auto direct_state = estimator.analyze(timings);
        pnr::PlaceTimingLocal direct(estimator, direct_state);
        near_source->coord.x = 3;
        direct.updateForward({near_source, near_source});
        require(direct.updated_wires == 1 && direct.updated_outputs == 1
                    && direct.updated_endpoints == 0,
                "noncritical input update did not stop at unchanged output");
        auto check_direct = [&] {
            auto exact = estimator.analyze(timings);
            require(near(direct_state.worst_slack_ns, exact.worst_slack_ns)
                        && near(direct_state.total_negative_slack_ns, exact.total_negative_slack_ns)
                        && direct_state.violated_endpoints == exact.violated_endpoints,
                    "direct timing aggregates disagree with full timing");
            for (size_t i = 0; i < exact.endpoint_details.size(); ++i) {
                const auto& actual = direct_state.endpoint_details[i];
                const auto& expected = exact.endpoint_details[i];
                require(near(actual.arrival_ns, expected.arrival_ns)
                            && near(actual.slack_ns, expected.slack_ns)
                            && actual.critical_edges.size() == expected.critical_edges.size(),
                        "direct endpoint update disagrees with full timing");
                for (size_t edge = 0; edge < expected.critical_edges.size(); ++edge)
                    require(actual.critical_edges[edge].sink_input == expected.critical_edges[edge].sink_input
                                && actual.critical_edges[edge].driver_output == expected.critical_edges[edge].driver_output
                                && near(actual.critical_edges[edge].wire_delay_ns, expected.critical_edges[edge].wire_delay_ns),
                            "direct timing kept a stale critical branch or wire");
            }
        };
        check_direct();
        near_source->coord.x = 10;
        direct.updateForward({near_source});
        require(direct.updated_endpoints == 2,
                "direct update failed to reach both unmoved shared sinks");
        check_direct();
        // Both ends move together: no wire delay changes, no propagation.
        unrelated_source->coord.x += 1;
        unrelated_sink->coord.x += 1;
        direct.updateForward({unrelated_source, unrelated_sink});
        require(direct.updated_wires == 1 && direct.updated_outputs == 0
                    && direct.updated_endpoints == 0,
                "equal translation caused unnecessary propagation");
        check_direct();
        // No-op updates and noncritical branch changes must not perturb ties.
        // A LUT can change both input and output wires in the same batch.
        for (int trial = 0; trial < 256; ++trial) {
            near_source->coord = {(trial * 7) % 12, trial % 3};
            far_source->coord = {(trial * 11) % 12, (trial / 3) % 3};
            logic->coord = {(trial * 5) % 12, (trial / 7) % 3};
            sink->coord = {(trial * 3) % 12, (trial / 11) % 3};
            direct.updateForward({near_source, far_source, logic, sink, logic});
            check_direct();
            direct.updateForward({logic, sink});
            require(direct.updated_outputs == 0 && direct.updated_endpoints == 0,
                    "unchanged coordinates caused timing propagation");
        }
        far_source->coord = {0, 0};
        logic->coord = {3, 0};
        sink->coord = {6, 0};
    }
    require(near_source->placement_timing_edges.empty()
                && !info.path.placement.updater,
            "direct timing left dangling object links after destruction");

    pnr::PlaceTimingPrepared prepared(estimator);
    auto trial_state = analysis;
    std::vector<pnr::PlaceTimingEndpoint*> targets;
    for (auto& endpoint : trial_state.endpoint_details) targets.push_back(&endpoint);
    // Shared outputs, fanout, input changes, and alternating trial/restore
    // geometries must produce exactly the same paths as the independent walk.
    for (int trial = 0; trial < 128; ++trial) {
        near_source->coord.x = (trial * 7) % 12;
        logic->coord.y = trial % 3;
        prepared.evaluate(targets);
        auto exact = estimator.analyze(timings);
        for (size_t i = 0; i < targets.size(); ++i) {
            const auto& expected = exact.endpoint_details[i];
            require(near(targets[i]->arrival_ns, expected.arrival_ns) &&
                        near(targets[i]->slack_ns, expected.slack_ns) &&
                        targets[i]->critical_edges.size() == expected.critical_edges.size(),
                    "prepared timing differs from the independent timing walk");
            for (size_t e = 0; e < expected.critical_edges.size(); ++e) {
                const auto& edge = targets[i]->critical_edges[e];
                require(edge.driver_output == expected.critical_edges[e].driver_output &&
                            edge.sink_input == expected.critical_edges[e].sink_input &&
                            near(edge.wire_delay_ns, expected.critical_edges[e].wire_delay_ns),
                        "prepared timing retained a stale or incorrect critical input");
            }
        }
    }
    auto benchmark = [&](bool reuse) {
        const auto start = std::chrono::steady_clock::now();
        for (int trial = 0; trial < 10000; ++trial) {
            near_source->coord.x = (trial * 7) % 12;
            if (reuse) prepared.evaluate(targets);
            else estimator.evaluateSetupTiming(targets);
        }
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    };
    const double reference_ms = benchmark(false);
    const double prepared_ms = benchmark(true);
    std::cout << "PLACE_TIMING_PREPARED trials=10000 reference_ms=" << reference_ms
              << " prepared_ms=" << prepared_ms
              << " speedup=" << reference_ms / prepared_ms << '\n';
}

void refinement_improves_timing_and_keeps_placement_legal()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 10, 1);
    Fixture fixture;
    auto* source = fixture.makeRegister("fixed_source");
    auto* sink = fixture.makeRegister("movable_sink");
    auto* follower = fixture.makeRegister("constellation_follower");
    fixture.connect(source, sink);
    fixture.connect(sink, follower);
    placeAt(source, 0, 0);
    placeAt(sink, 8, 0);
    placeAt(follower, 9, 0);
    source->outline.fixed = true;

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.18, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));

    pnr::PlaceDesign placer;
    pnr::PlaceTimingRefinement result = placer.refineTiming(timings, 6, 8);
    require(result.passes > 0 && result.moved_cells >= 2*result.passes,
            "timing refinement made no accepted movement");
    require(result.after.total_negative_slack_ns
                < result.before.total_negative_slack_ns,
            "timing refinement did not improve total negative slack");
    require(result.after.worst_slack_ns > result.before.worst_slack_ns,
            "timing refinement did not improve worst slack");
    require(sink->coord.x < 8 && sink->tile.peer
                && sink->tile->coord.x == sink->coord.x,
            "refined cell placement is missing or inconsistent");
    require(follower->coord.x < 9 && follower->tile.peer
                && follower->tile->coord.x == follower->coord.x,
            "nearby constellation matter did not move with its timing anchor");
    require(source->coord.x == 0,
            "fixed timing anchor moved during refinement");
}

void initial_smearing_follows_external_timing_nets()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 7, 2);
    Fixture fixture;
    auto* left_anchor = fixture.makeRegister("left_anchor");
    auto* left_movable = fixture.makeRegister("left_movable");
    auto* left_blocker = fixture.makeRegister("left_blocker");
    auto* right_movable = fixture.makeRegister("right_movable");
    auto* right_anchor = fixture.makeRegister("right_anchor");
    auto* right_blocker = fixture.makeRegister("right_blocker");
    fixture.connect(left_anchor, left_movable);
    fixture.connect(right_movable, right_anchor);
    placeAt(left_anchor, 0, 0);
    placeAt(left_blocker, 3, 0);
    placeAt(right_blocker, 3, 1);
    placeAt(right_anchor, 6, 1);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.25, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(left_movable, "D"));
    addEndpoint(timings, clock, fixture.conn(right_anchor, "D"));

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;
    placer.place_timing.tech = &tech;
    placer.place_timing.preparePlacementGuide(timings);

    int left_pos = placer.tryAddTimingAware(
        *left_movable, fpga::ELEMENT_FD, {3, 0});
    int right_pos = placer.tryAddTimingAware(
        *right_movable, fpga::ELEMENT_FD, {3, 1});
    require(left_pos >= 0 && left_movable->tile.peer
                && left_movable->tile->coord.x == 2
                && left_movable->tile->coord.y == 0,
            "left external timing net did not steer radial smearing left");
    require(right_pos >= 0 && right_movable->tile.peer
                && right_movable->tile->coord.x == 4
                && right_movable->tile->coord.y == 1,
            "right external timing net did not steer radial smearing right");
    require(placer.place_timing.placementNetWeight(
                *left_anchor, *left_movable) > 1.0
                && placer.place_timing.placementNetWeight(
                    *right_movable, *right_anchor) > 1.0,
            "clocked timing cones did not add placement net pressure");
}

void simultaneous_cooling_movement_shapes_register_constellations()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 5, 1);
    Fixture fixture;
    auto* left_anchor = fixture.makeRegister("batch_left_anchor");
    auto* left_cell = fixture.makeRegister("batch_left_cell");
    auto* center_cell = fixture.makeRegister("batch_center_cell");
    auto* right_cell = fixture.makeRegister("batch_right_cell");
    auto* right_anchor = fixture.makeRegister("batch_right_anchor");
    auto* follower = fixture.makeLogic("batch_left_follower");
    fixture.connect(left_anchor, left_cell);
    fixture.connect(right_cell, right_anchor);
    fixture.connect(left_cell, "Q", follower, "I0");
    placeAt(left_anchor, 0, 0);
    placeAt(right_anchor, 4, 0);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.25, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(left_cell, "D"));
    addEndpoint(timings, clock, fixture.conn(right_anchor, "D"));

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;
    placer.place_timing.tech = &tech;
    placer.place_timing.preparePlacementGuide(timings);

    std::vector<rtl::Inst*> cells{
        left_anchor, left_cell, center_cell, right_cell, right_anchor, follower};
    std::unordered_map<rtl::Inst*, fpga::Coord> predicted{
        {left_anchor, {0, 0}},
        {left_cell, {2, 0}},
        {center_cell, {2, 0}},
        {right_cell, {2, 0}},
        {right_anchor, {4, 0}},
        {follower, {2, 0}},
    };
    left_cell->outline = {.x = 2, .y = 0};
    center_cell->outline = {.x = 2, .y = 0};
    right_cell->outline = {.x = 2, .y = 0};
    follower->outline = {.x = 2, .y = 0};

    size_t oversubscribed = 0;
    std::vector<pnr::PlacePredictedMove> moves =
        placer.calculatePredictedDirections(cells, predicted, oversubscribed);
    require(oversubscribed == 1 && moves.size() == 2,
            "predictor did not identify the crowded register group");
    auto left_prediction = std::ranges::find_if(
        moves, [left_cell](const pnr::PlacePredictedMove& move) {
            return move.inst == left_cell;
        });
    auto right_prediction = std::ranges::find_if(
        moves, [right_cell](const pnr::PlacePredictedMove& move) {
            return move.inst == right_cell;
        });
    require(left_prediction != moves.end()
                && left_prediction->direction.x == -1
                && right_prediction != moves.end()
                && right_prediction->direction.x == 1,
            "register forces did not point toward their timing peers");
    require(near(left_cell->outline.x, 2)
                && near(center_cell->outline.x, 2)
                && near(right_cell->outline.x, 2)
                && near(follower->outline.x, 2),
            "force calculation changed the frozen position snapshot");

    size_t moved =
        placer.applyPredictedDisplacementSimultaneously(moves);
    require(moved == 3,
            "one movement did not move two registers and their LUT follower");
    require(left_cell->outline.x < 2
                && near(center_cell->outline.x, 2)
                && right_cell->outline.x > 2,
            "registers did not separate according to their saved forces");
    require(follower->outline.x < 2
                && std::abs(2 - follower->outline.x)
                    < std::abs(2 - left_cell->outline.x),
            "register displacement did not decay into its LUT constellation");
    require(left_cell->placement_motion.displacement_x < 0
                && right_cell->placement_motion.displacement_x > 0
                && follower->placement_motion.displacement_x < 0,
            "continuous displacement history was not saved in the cells");

    double left_displacement = left_cell->placement_motion.displacement_x;
    double right_displacement = right_cell->placement_motion.displacement_x;
    double follower_displacement = follower->placement_motion.displacement_x;
    left_cell->outline = {.x = 2, .y = 0};
    center_cell->outline = {.x = 2, .y = 0};
    right_cell->outline = {.x = 2, .y = 0};
    follower->outline = {.x = 2, .y = 0};
    std::ranges::reverse(cells);
    size_t reordered_oversubscribed = 0;
    std::vector<pnr::PlacePredictedMove> reordered_moves =
        placer.calculatePredictedDirections(
            cells, predicted, reordered_oversubscribed);
    placer.applyPredictedDisplacementSimultaneously(reordered_moves);
    require(reordered_oversubscribed == 1
                && near(left_cell->placement_motion.displacement_x,
                        left_displacement)
                && near(right_cell->placement_motion.displacement_x,
                        right_displacement)
                && near(follower->placement_motion.displacement_x,
                        follower_displacement),
            "simultaneous continuous movement depends on input cell order");

    left_cell->outline = {.x = 2, .y = 0};
    pnr::PlacePredictedMove fastest = *left_prediction;
    fastest.external_force_x = 100;
    fastest.external_force_y = 0;
    placer.applyPredictedDisplacementSimultaneously(
        std::vector<pnr::PlacePredictedMove>{fastest}, nullptr, 0.20);
    require(near(left_cell->placement_motion.displacement_x, 2.0),
            "fastest placement cell was not capped at two Tiles");
}

void clipped_register_motion_translates_its_bunch()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 8, 2);
    Fixture fixture;
    auto* mover = fixture.makeRegister("boundary_mover");
    auto* companion = fixture.makeLogic("boundary_companion");
    mover->outline = {.x = 2.5F, .y = 0.5F};
    companion->outline = {.x = 2.0F, .y = 0.5F};

    Referable<pnr::RegBunch> bunch;
    bunch.reg = mover;
    bunch.x = 2.0F;
    bunch.y = 0.5F;
    mover->bunch_ref.set(&bunch);
    companion->bunch_ref.set(&bunch);

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;

    std::vector<pnr::PlacePredictedMove> moves{
        pnr::PlacePredictedMove{
            .inst = mover,
            .from = {2, 0},
            .to = {2, 0},
            .direction = {1, 0},
            .type = fpga::ELEMENT_FD,
            .external_force_x = 1.0,
            .external_force_y = 0.0,
            .external_peers = 1,
        },
    };
    std::vector<rtl::Inst*> cells{mover, companion};
    size_t moved = placer.applyPredictedDisplacementSimultaneously(
        moves, &cells, 1.0);

    require(moved == 2,
            "translated bunch did not carry all of its members");
    require(near(bunch.x, 3.0)
                && near(mover->outline.x, 3.5)
                && near(companion->outline.x, 3.0),
            "a correct register force was erased by its old bunch boundary");
}

void pre_smearing_reserves_capacity_for_atomic_bunches()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 6, 6);
    Fixture fixture;
    std::array<Referable<pnr::RegBunch>, 3> bunches;
    std::vector<rtl::Inst*> cells;
    std::array<double, 3> original_member_dx{};
    std::array<double, 3> original_member_dy{};
    for (size_t index = 0; index < bunches.size(); ++index) {
        auto* reg = fixture.makeRegister(
            "pre_smear_reg_" + std::to_string(index));
        auto* logic = fixture.makeLogic(
            "pre_smear_logic_" + std::to_string(index));
        reg->outline = {.x = 2.0F, .y = 2.0F};
        logic->outline = {
            .x = 2.2F + static_cast<float>(index)*0.03F,
            .y = 1.9F - static_cast<float>(index)*0.02F,
        };
        bunches[index].reg = reg;
        bunches[index].x = 2.0F;
        bunches[index].y = 2.0F;
        reg->bunch_ref.set(&bunches[index]);
        logic->bunch_ref.set(&bunches[index]);
        original_member_dx[index] = logic->outline.x - reg->outline.x;
        original_member_dy[index] = logic->outline.y - reg->outline.y;
        cells.push_back(reg);
        cells.push_back(logic);
    }

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;

    pnr::PlacePreSmearResult result = placer.preSmearBunches(cells);
    require(result.bunches == 3 && result.moved_bunches == 2
                && result.moved_right == 1 && result.moved_down == 1
                && result.right_first_bunches == 1
                && result.down_first_bunches == 1
                && result.failed_bunches == 0,
            "pre-smearing did not alternate whole crowded bunches right/down");
    require(near(bunches[0].x, 2) && near(bunches[0].y, 2)
                && near(bunches[1].x, 3) && near(bunches[1].y, 2)
                && near(bunches[2].x, 2) && near(bunches[2].y, 3),
            "pre-smearing selected unexpected row-major reservations");
    for (size_t index = 0; index < bunches.size(); ++index) {
        rtl::Inst* reg = cells[index*2];
        rtl::Inst* logic = cells[index*2 + 1];
        require(near(logic->outline.x - reg->outline.x,
                     original_member_dx[index], 1e-6)
                    && near(logic->outline.y - reg->outline.y,
                            original_member_dy[index], 1e-6),
                "pre-smearing split a bunch instead of translating it atomically");
        auto reservation = placer.bunch_reservations.find(&bunches[index]);
        require(reservation != placer.bunch_reservations.end()
                    && reservation->second.demand[fpga::ELEMENT_FD] == 1
                    && reservation->second.demand[fpga::ELEMENT_LUT5] == 1,
                "pre-smearing did not reserve both element classes");
        require(reservation->second.placements.size() == 2
                    && std::ranges::all_of(
                        reservation->second.placements,
                        [](const pnr::PlaceBunchReservation::Placement& placed) {
                            return placed.inst && placed.pos >= 0
                                && placed.coord.x >= 0
                                && placed.coord.y >= 0;
                        }),
                "pre-smearing stored a count instead of exact pack positions");
    }
    require(placer.commitPreSmearReservations() == cells.size()
                && std::ranges::all_of(cells, [](rtl::Inst* inst) {
                    return inst && inst->tile.peer && inst->pos >= 0;
                }),
            "precise pre-smear reservations were not committed verbatim");

    // One Tile in this synthetic model has one register slot. A three-register
    // bunch must remain one object now while reserving a multi-Tile envelope
    // for the ordinary member-smearing phase which follows.
    resetDevice(type, 6, 6);
    Fixture oversized_fixture;
    Referable<pnr::RegBunch> oversized_bunch;
    std::vector<rtl::Inst*> oversized_cells;
    for (int index = 0; index < 3; ++index) {
        auto* reg = oversized_fixture.makeRegister(
            "oversized_pre_smear_reg_" + std::to_string(index));
        reg->outline = {.x = 2.0F, .y = 2.0F};
        reg->bunch_ref.set(&oversized_bunch);
        oversized_cells.push_back(reg);
        if (index == 0) oversized_bunch.reg = reg;
    }
    oversized_bunch.x = 2.0F;
    oversized_bunch.y = 2.0F;
    pnr::PlaceDesign oversized_placer;
    oversized_placer.tech = &tech;
    oversized_placer.fpga = &fpga::Device::current();
    oversized_placer.tile_grid = &oversized_placer.fpga->tile_grid;
    oversized_placer.fpga_width = oversized_placer.fpga->size_width;
    oversized_placer.fpga_height = oversized_placer.fpga->size_height;
    oversized_placer.aspect_x = 1;
    oversized_placer.aspect_y = 1;
    pnr::PlacePreSmearResult oversized_result =
        oversized_placer.preSmearBunches(oversized_cells);
    auto oversized_reservation =
        oversized_placer.bunch_reservations.find(&oversized_bunch);
    require(oversized_result.bunches == 1
                && oversized_result.failed_bunches == 0
                && oversized_reservation
                    != oversized_placer.bunch_reservations.end(),
            "pre-smearing failed to reserve an oversized bunch");
    const pnr::PlaceBunchReservation& envelope =
        oversized_reservation->second;
    int envelope_tiles = (envelope.maximum.x - envelope.minimum.x + 1)
        * (envelope.maximum.y - envelope.minimum.y + 1);
    require(envelope_tiles >= 3
                && envelope.demand[fpga::ELEMENT_FD] == 3
                && envelope.placements.size() == 3
                && std::ranges::all_of(oversized_cells,
                    [&](rtl::Inst* reg) {
                        return near(reg->outline.x,
                                    oversized_cells.front()->outline.x)
                            && near(reg->outline.y,
                                    oversized_cells.front()->outline.y);
                    }),
            "oversized bunch was split before its reserved envelope was smeared");
    require(oversized_placer.commitPreSmearReservations() == 3
                && std::ranges::all_of(oversized_cells,
                    [](rtl::Inst* reg) { return reg->tile.peer != nullptr; }),
            "oversized bunch reservations were not exact packable positions");
}

void pre_smearing_fixed_io_follower_stays_near_its_outline()
{
    fpga::TileType type=makeTileType();
    resetDevice(type,8,8);
    Fixture fixture;
    technology::Tech tech;
    auto* io=fixture.makeRegister("fixed_output");
    io->cell_ref->type="OBUF";
    io->outline={.x=7,.y=4,.fixed=true};
    io->coord={7,4};
    io->tile.set(&fpga::Device::current().tile_grid[4*8+7]);
    io->pos=0; // Fixed I/O site, outside the LUT/REG Element model.
    auto* blocker=fixture.makeLogic("occupied_outline");
    placeLogicAt(blocker,5,4);
    auto* logic=fixture.makeLogic("movable_output_logic");
    logic->outline={.x=5,.y=4};
    fixture.connect(logic,"O",io,"D");
    Referable<pnr::RegBunch> bunch;
    bunch.fixed=true;bunch.x=7;bunch.y=4;bunch.reg=io;
    io->bunch_ref.set(&bunch);logic->bunch_ref.set(&bunch);
    pnr::PlaceDesign placer;
    placer.tech=&tech;placer.fpga=&fpga::Device::current();
    placer.tile_grid=&placer.fpga->tile_grid;
    placer.fpga_width=placer.fpga_height=8;placer.aspect_x=placer.aspect_y=1;
    auto result=placer.preSmearBunches({logic});
    require(result.failed_bunches==0 && placer.commitPreSmearReservations()==1,
            "movable COMB in fixed-I/O bunch could not find adjacent capacity");
    require(logic->coord.x==6 && logic->coord.y==4,
            "fixed-I/O follower was sent to the origin or away from its I/O peer");
    require(io->coord.x==7 && io->coord.y==4 && near(io->outline.x,7)
                && near(bunch.x,7) && near(bunch.y,4),
            "moving a COMB follower moved the fixed I/O or its bunch anchor");
    require(near(logic->outline.x,6) && near(logic->outline.y,4),
            "reservation did not publish the follower's selected position");

    // An actually fixed member must fail at its occupied site, never escape
    // through a nearby search or through the old whole-chip origin fallback.
    auto* locked=fixture.makeLogic("locked_logic");
    locked->outline={.x=5,.y=4,.fixed=true};
    Referable<pnr::RegBunch> locked_bunch;
    locked_bunch.x=5;locked_bunch.y=4;locked_bunch.reg=locked;
    locked->bunch_ref.set(&locked_bunch);
    auto failed=placer.preSmearBunches({locked});
    require(failed.failed_bunches==1 && placer.commitPreSmearReservations()==0
                && !locked->tile.peer && near(locked->outline.x,5),
            "an actually fixed cell moved after its requested site was blocked");
}

void failed_pre_smear_candidates_do_not_retain_tile_snapshots()
{
    for (int width : {16, 32}) {
        fpga::TileType type = makeTileType();
        fpga::Element second_fd = type.elements.front();
        second_fd.name = "REG1";
        second_fd.bitmap_pos = 1;
        second_fd.left_blockers[0] = 2;
        type.elements.front().left_blockers[0] = 1;
        type.elements[1].right_blockers[0] = 1;
        type.elements[1].right_blockers[1] = 1;
        type.elements.push_back(second_fd);
        resetDevice(type, width, 4);
        Fixture fixture;
        technology::Tech tech;
        std::array<Referable<pnr::RegBunch>, 3> bunches;
        auto* seed = fixture.makeRegister("accepted_register");
        auto* incompatible_reg = fixture.makeRegister("independent_register");
        auto* incompatible_lut = fixture.makeLogic("independent_logic");
        auto* later = fixture.makeLogic("accepted_logic");
        std::vector<rtl::Inst*> cells{seed, incompatible_reg, incompatible_lut, later};
        for (auto* inst : cells) inst->outline = {.x = 0, .y = 0};
        seed->bunch_ref.set(&bunches[0]);
        incompatible_reg->bunch_ref.set(&bunches[1]);
        incompatible_lut->bunch_ref.set(&bunches[1]);
        later->bunch_ref.set(&bunches[2]);
        bunches[0].reg = seed;
        bunches[1].reg = incompatible_reg;
        bunches[2].reg = later;
        for (auto& bunch : bunches) bunch.x = bunch.y = 0;

        pnr::PlaceDesign placer;
        placer.tech = &tech;
        placer.fpga = &fpga::Device::current();
        placer.tile_grid = &placer.fpga->tile_grid;
        placer.fpga_width = width;
        placer.fpga_height = 4;
        placer.aspect_x = placer.aspect_y = 1;
        auto result = placer.preSmearBunches(cells);
        // Unconnected neighboring elements must fail everywhere, despite capacity.
        require(result.failed_bunches == 1
                    && !placer.bunch_reservations.contains(&bunches[1]),
                "memory regression did not exercise failed exact packing");
        // Trying more Tiles must not retain empty snapshots of the entire grid.
        require(result.empty_previews_released >= static_cast<size_t>(width*4 - 1)
                    && result.preview_tiles_peak <= 2
                    && result.preview_tiles_retained == 2,
                "failed packing trials retained empty per-Tile snapshots");
        // Rolling back a trial on the seed's Tile must preserve its reservation.
        require(placer.bunch_reservations.at(&bunches[0]).placements.front().inst == seed
                    && placer.bunch_reservations.at(&bunches[0]).placements.front().pos == fdPos(0),
                "releasing failed previews lost a previously accepted reservation");
        // Preview destruction must restore live assignments and free masks.
        for (auto* inst : cells) require(!inst->tile.peer, "preview leaked a cell assignment");
        for (const auto& tile : *placer.tile_grid) {
            require(tile.elements_free == tile.elements_pos && tile.peers.empty(),
                    "failed previews leaked element occupancy or Tile references");
        }
        require(placer.commitPreSmearReservations() == 2 && seed->tile.peer && later->tile.peer
                    && !incompatible_reg->tile.peer && !incompatible_lut->tile.peer,
                "accepted reservations changed during preview cleanup");
        // The committed Tile state replaces the duplicate temporary placement plan.
        require(placer.bunch_reservations.empty() && placer.bunch_reservation_order.empty()
                    && placer.bunch_reservation_order.capacity() == 0
                    && placer.commitPreSmearReservations() == 0,
                "committing kept duplicate reservations alive");
        std::cout << "PLACE_PRE_SMEAR_MEMORY tiles=" << width*4
                  << " preview_peak=" << result.preview_tiles_peak
                  << " empty_released=" << result.empty_previews_released << '\n';
    }
}

void pre_smearing_uses_nearest_ring_and_external_timing()
{
    for(const fpga::Coord direction : {fpga::Coord{-1,0},fpga::Coord{0,-1},
                                      fpga::Coord{1,0},fpga::Coord{0,1}}) {
        fpga::TileType type=makeTileType();resetDevice(type,8,8);
        Fixture fixture;technology::Tech tech;
        auto* occupied=fixture.makeRegister("occupied_home");placeAt(occupied,3,3);
        auto* peer=fixture.makeRegister("external_peer");
        placeAt(peer,3+3*direction.x,3+3*direction.y);
        auto* reg=fixture.makeRegister("atomic_reg");
        auto* logic=fixture.makeLogic("atomic_logic");
        reg->outline={.x=3,.y=3};logic->outline={.x=3,.y=4};
        fixture.connect(reg,peer);
        fixture.connect(logic,"O",reg,"D");
        Referable<pnr::RegBunch> bunch;
        bunch.x=3;bunch.y=3;bunch.reg=reg;
        reg->bunch_ref.set(&bunch);logic->bunch_ref.set(&bunch);
        pnr::PlaceDesign placer;
        placer.tech=&tech;placer.fpga=&fpga::Device::current();
        placer.tile_grid=&placer.fpga->tile_grid;
        placer.fpga_width=placer.fpga_height=8;placer.aspect_x=placer.aspect_y=1;
        auto result=placer.preSmearBunches({reg,logic});
        require(result.failed_bunches==0 && placer.commitPreSmearReservations()==2,
                "radial atomic reservation failed");
        require(reg->coord.x==3+direction.x && reg->coord.y==3+direction.y,
                "reservation ignored the best external timing direction on the nearest ring");
        require(logic->coord.x==reg->coord.x && logic->coord.y==reg->coord.y+1,
                "timing-driven reservation split the bunch's Outline shape");
    }

    // A nearer legal Tile wins even when a farther rightward site has a
    // shorter external connection. Capacity smearing stays local first.
    fpga::TileType type=makeTileType();resetDevice(type,8,8);
    Fixture fixture;technology::Tech tech;
    for(const fpga::Coord c : {fpga::Coord{3,3},fpga::Coord{4,3},
                              fpga::Coord{3,2},fpga::Coord{3,4}})
        placeAt(fixture.makeRegister("block_"+std::to_string(c.x)+"_"+std::to_string(c.y)),c.x,c.y);
    auto* peer=fixture.makeRegister("distant_right_peer");placeAt(peer,7,3);
    auto* reg=fixture.makeRegister("nearest_left");reg->outline={.x=3,.y=3};
    fixture.connect(reg,peer);
    Referable<pnr::RegBunch> bunch;bunch.x=3;bunch.y=3;bunch.reg=reg;
    reg->bunch_ref.set(&bunch);
    pnr::PlaceDesign placer;
    placer.tech=&tech;placer.fpga=&fpga::Device::current();placer.tile_grid=&placer.fpga->tile_grid;
    placer.fpga_width=placer.fpga_height=8;placer.aspect_x=placer.aspect_y=1;
    require(placer.preSmearBunches({reg}).failed_bunches==0
                && placer.commitPreSmearReservations()==1 && reg->coord.x==2 && reg->coord.y==3,
            "right/down preference skipped a closer legal left Tile");
}

void combinational_follower_moves_toward_neighbor_shape()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 8, 8);
    Fixture fixture;
    auto* first_reg = fixture.makeRegister("shape_first_reg");
    auto* second_reg = fixture.makeRegister("shape_second_reg");
    auto* follower = fixture.makeLogic("shape_follower");
    fixture.connect(first_reg, "Q", follower, "I0");
    fixture.connect(second_reg, "Q", follower, "I1");
    first_reg->outline = {.x = 2.0F, .y = 2.0F};
    second_reg->outline = {.x = 3.0F, .y = 2.0F};
    follower->outline = {.x = 7.0F, .y = 7.0F};

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;

    std::vector<pnr::PlacePredictedMove> moves;
    for (rtl::Inst* reg : {first_reg, second_reg}) {
        moves.push_back(pnr::PlacePredictedMove{
            .inst = reg,
            .from = {static_cast<int>(reg->outline.x),
                     static_cast<int>(reg->outline.y)},
            .to = {static_cast<int>(reg->outline.x),
                   static_cast<int>(reg->outline.y)},
            .direction = {1, 1},
            .type = fpga::ELEMENT_FD,
            .external_force_x = 1.0,
            .external_force_y = 1.0,
            .external_peers = 1,
        });
    }
    std::vector<rtl::Inst*> cells{first_reg, second_reg, follower};
    placer.applyPredictedDisplacementSimultaneously(moves, &cells, 0.2);

    require(follower->outline.x < 7.0F && follower->outline.y < 7.0F,
            "combinational follower copied outward register velocity instead "
            "of following its connected-neighbor shape");
}

void pre_smearing_counts_coupled_element_occupancy()
{
    fpga::TileType type = makeTileType();
    fpga::Element paired;
    paired.name = "PAIRED_LOGIC";
    paired.type = fpga::ELEMENT_LUT1;
    paired.bitmap_pos = 0;
    paired.elements_to_left = fpga::ELEMENT_LUT1;
    type.elements.push_back(paired);
    resetDevice(type, 2, 1);
    Fixture fixture;
    auto* full = fixture.makeLogic("full_logic", 6);
    auto* small = fixture.makeLogic("small_logic", 1);
    std::array<Referable<pnr::RegBunch>, 2> bunches;
    std::vector<rtl::Inst*> cells{full, small};
    for (size_t i = 0; i < cells.size(); ++i) {
        cells[i]->outline = {.x = 0.0F, .y = 0.0F};
        cells[i]->bunch_ref.set(&bunches[i]);
        bunches[i].reg = cells[i];
        bunches[i].x = bunches[i].y = 0;
    }
    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = 2;
    placer.fpga_height = 1;
    placer.aspect_x = placer.aspect_y = 1;
    auto result = placer.preSmearBunches(cells);

    // A LUT6 consumes both columns. The LUT1 must skip the first Tile
    // in the capacity filter, not rediscover that conflict in exact packing.
    require(result.failed_bunches == 0 && result.precise_tile_trials == 2,
        "capacity filtering retried an already consumed paired element");
    require(placer.bunch_reservations.at(&bunches[0]).placements[0].coord == fpga::Coord{0, 0}
                && placer.bunch_reservations.at(&bunches[1]).placements[0].coord == fpga::Coord{1, 0},
        "coupled resources were reserved in the same Tile");
    // Trial destruction restores both columns, not only the primary one.
    for (auto& tile : *placer.tile_grid)
        require(tile.elements_free[fpga::ELEMENT_LUT5] == 1
                    && tile.elements_free[fpga::ELEMENT_LUT1] == 1,
            "coupled-element preview leaked occupancy");
    // Real placement must reproduce exactly the two legal reservations.
    require(placer.commitPreSmearReservations() == 2
                && (*placer.tile_grid)[0].elements_free[fpga::ELEMENT_LUT1] == 0
                && (*placer.tile_grid)[1].elements_free[fpga::ELEMENT_LUT5] == 1,
        "committing coupled-element reservations changed their resource use");
}

void oversized_pre_smearing_scales_with_bunch_size()
{
    constexpr int cell_count = 6000;
    constexpr int side = 80;
    fpga::TileType type = makeTileType();
    resetDevice(type, side, side);
    Fixture fixture;
    Referable<pnr::RegBunch> bunch;
    std::vector<rtl::Inst*> cells;
    cells.reserve(cell_count);
    for (int index = 0; index < cell_count; ++index) {
        rtl::Inst* reg = fixture.makeRegister(
            "large_pre_smear_reg_" + std::to_string(index));
        reg->outline = {.x = 40.0F, .y = 40.0F};
        reg->bunch_ref.set(&bunch);
        if (index == 0) bunch.reg = reg;
        cells.push_back(reg);
    }
    bunch.x = 40.0F;
    bunch.y = 40.0F;

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;

    auto started = std::chrono::steady_clock::now();
    pnr::PlacePreSmearResult result = placer.preSmearBunches(cells);
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    require(result.failed_bunches == 0
                && result.reserved_cells == cell_count,
            "large pre-smear bunch was not reserved");
    require(result.precise_tile_trials <= 2*cell_count,
            "large pre-smear bunch restarted its Tile scan per member");
    require(result.precise_fallback_candidates == 0
                && result.precise_fallback_exhausted == 0,
            "large pre-smear bunch unexpectedly entered fallback search");
    require(elapsed_ms < 5000.0,
            "large pre-smear bunch coordinate indexing regressed");
    std::cout << "PLACE_PRE_SMEAR_TEST cells=" << cell_count
              << " tile_trials=" << result.precise_tile_trials
              << " elapsed_ms=" << elapsed_ms << '\n';
}

void timing_deficit_increases_placement_acceleration()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 6, 1);
    Fixture fixture;
    auto* safe_source = fixture.makeRegister("safe_source");
    auto* safe_sink = fixture.makeRegister("safe_sink");
    auto* deficit_source = fixture.makeRegister("deficit_source");
    auto* deficit_sink = fixture.makeRegister("deficit_sink");
    fixture.connect(safe_source, safe_sink);
    fixture.connect(deficit_source, deficit_sink);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 1.0, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(safe_sink, "D"));
    addEndpoint(timings, clock, fixture.conn(deficit_sink, "D"));
    auto& infos = timings.clocked_inputs[&clock];
    infos[0].path.max_setup_time = 0.5;
    infos[1].path.max_setup_time = 1.75;

    pnr::PlaceTiming estimator;
    estimator.preparePlacementGuide(timings);
    double safe_weight = estimator.placementNetWeight(
        *safe_source, *safe_sink);
    double deficit_weight = estimator.placementNetWeight(
        *deficit_source, *deficit_sink);
    require(deficit_weight > safe_weight,
            "timing deficit did not increase placement force weight");

    placeAt(safe_sink, 4, 0);
    placeAt(deficit_sink, 5, 0);
    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;
    placer.place_timing.tech = &tech;
    placer.place_timing.preparePlacementGuide(timings);
    std::vector<rtl::Inst*> cells{
        safe_source, deficit_source, safe_sink, deficit_sink};
    std::unordered_map<rtl::Inst*, fpga::Coord> predicted{
        {safe_source, {2, 0}},
        {deficit_source, {2, 0}},
        {safe_sink, {4, 0}},
        {deficit_sink, {5, 0}},
    };
    size_t oversubscribed = 0;
    auto moves = placer.calculatePredictedDirections(
        cells, predicted, oversubscribed);
    require(oversubscribed == 1 && moves.size() == 2,
            "acceleration regression did not create one crowded batch");
    require(deficit_source->placement_motion.acceleration
                > safe_source->placement_motion.acceleration,
            "timing deficit did not increase stored cell acceleration");
}

void swapping_accepts_axis_repair_above_threshold()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 12, 3);
    Fixture fixture;
    auto* source = fixture.makeRegister("swap_source");
    auto* sink = fixture.makeRegister("swap_sink");
    auto* challenger = fixture.makeRegister("swap_challenger");
    auto* challenger_peer = fixture.makeRegister("swap_challenger_peer");
    fixture.connect(source, sink);
    fixture.connect(challenger, challenger_peer);
    placeAt(source, 1, 1);
    placeAt(challenger, 5, 1);
    placeAt(sink, 9, 1);
    placeAt(challenger_peer, 5, 0);
    challenger_peer->outline.fixed = true;

    Referable<pnr::RegBunch> source_bunch;
    Referable<pnr::RegBunch> sink_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    source_bunch.reg = source;
    sink_bunch.reg = sink;
    challenger_bunch.reg = challenger;
    source->bunch_ref.set(&source_bunch);
    sink->bunch_ref.set(&sink_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.15, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "proficite_clk", .conn_ptr = nullptr,
        .conn_name = "proficite_clk", .period_ns = 2.0, .duty = 50});
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 1;
    swapping.config.placement_radius = 0;
    swapping.config.proficite_regions_per_axis = 1;
    swapping.config.slack_tolerance_ns = 0;
    std::vector<rtl::Inst*> cells{source, sink, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.accepted_swaps == 1
                && result.after.total_negative_slack_ns
                    < result.before.total_negative_slack_ns
                && source->coord.x == 5,
            "PlaceSwapping did not accept a timing-improving horizontal swap");
}

void swapping_rolls_back_improvement_below_threshold()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 12, 3);
    Fixture fixture;
    auto* source = fixture.makeRegister("rollback_source");
    auto* sink = fixture.makeRegister("rollback_sink");
    auto* challenger = fixture.makeRegister("rollback_challenger");
    auto* challenger_peer =
        fixture.makeRegister("rollback_challenger_peer");
    fixture.connect(source, sink);
    fixture.connect(challenger, challenger_peer);
    placeAt(source, 0, 1);
    placeAt(challenger, 1, 1);
    placeAt(sink, 11, 1);
    sink->outline.fixed = true;
    placeAt(challenger_peer, 1, 0);
    challenger_peer->outline.fixed = true;

    Referable<pnr::RegBunch> source_bunch;
    Referable<pnr::RegBunch> sink_bunch;
    Referable<pnr::RegBunch> challenger_bunch;
    source_bunch.reg = source;
    sink_bunch.reg = sink;
    challenger_bunch.reg = challenger;
    source->bunch_ref.set(&source_bunch);
    sink->bunch_ref.set(&sink_bunch);
    challenger->bunch_ref.set(&challenger_bunch);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.01, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));
    Referable<rtl::Clock> proficite_clock(rtl::Clock{
        .name = "rollback_proficite_clk", .conn_ptr = nullptr,
        .conn_name = "rollback_proficite_clk", .period_ns = 2.0,
        .duty = 50});
    addEndpoint(timings, proficite_clock,
                fixture.conn(challenger_peer, "D"));

    technology::Tech tech;
    tech.place.aspect_x = 1;
    tech.place.aspect_y = 1;
    pnr::PlaceSwapping swapping;
    swapping.tech = &tech;
    swapping.config.maximum_passes = 1;
    swapping.config.placement_radius = 0;
    swapping.config.proficite_regions_per_axis = 1;
    // The built-in distance calibration makes the nearest candidate improve
    // this tiny fixture by about 91%. Keep the acceptance threshold above
    // that value so this case exercises exact rollback, not acceptance.
    swapping.config.minimum_improvement = 0.95;
    swapping.config.slack_tolerance_ns = 0;
    std::vector<rtl::Inst*> cells{source, sink, challenger};
    pnr::PlaceSwappingResult result = swapping.run(timings, cells);

    require(result.accepted_swaps == 0
                && result.rejected_improvement >= 1
                && source->coord.x == 0 && challenger->coord.x == 1
                && sink->coord.x == 11,
            "PlaceSwapping did not restore an under-threshold swap exactly");
}

void rejected_constellation_restores_all_original_slots(bool deferred_route_capacity = false)
{
    fpga::TileType type = makeTileType();
    auto cb = std::make_unique<fpga::CBType>("RESTORE_FABRIC");
    if (deferred_route_capacity) {
        fpga::Element second = type.elements.front();
        second.bitmap_pos = 1;
        type.elements.push_back(second);
        type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "AX");
        type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 2, "BX");
        type.pin_map.input_nodes[1].setBit(90);
        type.pin_map.input_nodes[2].setBit(91);
        cb->dst_joint[10].joint.setBit(15);
        cb->dst_joint[11].joint.setBit(15);
        cb->joint_local[15].local.setBit(90);
        cb->joint_local[15].local.setBit(91);
        cb->rebuildOutgoingSrcs();
    }
    fpga::TileType incompatible{"NO_REGISTER_SLOT", 1, 0};
    resetDevice(type, 8, 1);
    fpga::Device::current().tile_grid[5].tile_type = &incompatible;

    Fixture fixture;
    auto* source = fixture.makeRegister("rollback_source");
    auto* sink = fixture.makeRegister("rollback_sink");
    auto* follower = fixture.makeRegister("rollback_follower");
    fixture.connect(source, sink);
    fixture.connect(sink, follower);
    placeAt(source, 0, 0);
    placeAt(sink, 4, 0);
    placeAt(follower, 6, 0);
    source->outline.fixed = true;
    Referable<rtl::Inst>* stationary = nullptr;
    if (deferred_route_capacity) {
        auto* control = fixture.makeRegister("independent_control");
        stationary = fixture.makeRegister("stationary_joint_user");
        fixture.connect(control, stationary);
        auto& tile = *follower->tile.peer;
        tile.cb_type = tile.cb.type = cb.get();
        tile.input_joint_reservations_initialized = false;
        // Both distinct input locals are admitted before routing; only their
        // possible joint paths overlap. A failed move must restore that state.
        require(tile.tryAddAt(stationary, fdPos(1), false) == fdPos(1),
                "could not set up deferred joint capacity at the saved tile");
        tile.unassign(follower);
        // Prove this fixture detects changing admission policy during rollback.
        require(tile.tryAddAt(follower, fdPos(), true) < 0,
                "strict admission unexpectedly accepts the overlapping joint");
        require(tile.tryAddAt(follower, fdPos(), false) == fdPos(),
                "could not restore the original relaxed placement");
        stationary->outline.fixed = true;
    }

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 0.10, .duty = 50});
    clk::Timings timings;
    addEndpoint(timings, clock, fixture.conn(sink, "D"));

    pnr::PlaceDesign placer;
    pnr::PlaceTimingRefinement result = placer.refineTiming(timings, 1, 1);
    require(result.passes == 0,
            "partially rejected constellation was accepted");
    require(sink->coord == fpga::Coord{4, 0}
                && follower->coord == fpga::Coord{6, 0},
            "rejected constellation did not restore original coordinates");
    require(sink->tile.peer == &fpga::Device::current().tile_grid[4]
                && follower->tile.peer == &fpga::Device::current().tile_grid[6],
            "rejected constellation did not restore original tile ownership");
    // The stationary owner must survive rollback with its distinct saved slot.
    require(!stationary || (stationary->tile.peer == follower->tile.peer
                && stationary->pos == fdPos(1) && follower->pos == fdPos()),
            "rollback displaced the stationary joint user");
}

void traversal_work_is_near_linear()
{
    constexpr int endpoint_count = 2000;
    fpga::TileType type = makeTileType();
    resetDevice(type, 1, 1);
    Fixture fixture;
    fixture.insts.reserve(endpoint_count + 1);
    fixture.cells.reserve(endpoint_count + 1);
    auto* source = fixture.makeRegister("source");
    placeAt(source, 0, 0);

    Referable<rtl::Clock> clock(rtl::Clock{
        .name = "clk", .conn_ptr = nullptr, .conn_name = "clk",
        .period_ns = 1.0, .duty = 50});
    clk::Timings timings;
    auto& infos = timings.clocked_inputs[&clock];
    infos.reserve(endpoint_count);
    for (int i = 0; i < endpoint_count; ++i) {
        auto* sink = fixture.makeRegister("sink_" + std::to_string(i));
        fixture.connect(source, sink);
        // The performance case tests traversal, not packing capacity. A live Tile
        // reference and synthetic coordinates are sufficient for delay estimation.
        fpga::Device::current().tile_grid.front().assign(sink);
        sink->coord = {i%100, i/100};
        addEndpoint(timings, clock, fixture.conn(sink, "D"));
    }

    pnr::PlaceTiming estimator;
    auto started = std::chrono::steady_clock::now();
    pnr::PlaceTimingAnalysis analysis = estimator.analyze(timings);
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    require(analysis.endpoints == endpoint_count,
            "performance traversal lost timing endpoints");
    require(analysis.evaluated_edges == endpoint_count,
            "shared-source traversal did more than one wire evaluation per endpoint");
    require(analysis.evaluated_nodes <= static_cast<size_t>(endpoint_count),
            "timing-node evaluation grew faster than the timing forest");
    std::cout << "PLACE_TIMING_TEST_PERF endpoints=" << analysis.endpoints
              << " nodes=" << analysis.evaluated_nodes
              << " edges=" << analysis.evaluated_edges
              << " elapsed_ms=" << elapsed_ms << '\n';
}

void timing_search_defers_to_sparse_region_fallback()
{
    fpga::TileType type = makeTileType();
    resetDevice(type, 20, 1);
    Fixture fixture;
    for (int x = 0; x <= 4; ++x) {
        placeAt(fixture.makeRegister("local_blocker_" + std::to_string(x)),
                x, 0);
    }
    auto* target = fixture.makeRegister("regional_fallback_target");

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;
    placer.place_started = std::chrono::steady_clock::now();
    placer.place_next_report = placer.place_started + std::chrono::hours(1);
    placer.preparePlaceCandidates();

    require(placer.tryAddTimingAware(
                *target, fpga::ELEMENT_FD, {0, 0}) < 0,
            "timing-aware placement exceeded its local search budget");
    require(placer.tryAddSparseTile(
                *target, fpga::ELEMENT_FD, {0, 0}) >= 0
                && target->tile.peer,
            "sparse regional fallback did not finish bounded placement");
}

void shared_input_tile_reuse_is_independent_of_fanout_size()
{
    constexpr int fanout = 2000;
    fpga::TileType type = makeTileType();
    fpga::Element second_fd;
    second_fd.name = "REG1";
    second_fd.type = fpga::ELEMENT_FD;
    second_fd.bitmap_pos = 1;
    second_fd.elements_to_left = fpga::ELEMENT_FD;
    type.elements.push_back(std::move(second_fd));
    resetDevice(type, 2, 1);
    Fixture fixture;
    fixture.insts.reserve(fanout + 1);
    fixture.cells.reserve(fanout + 1);
    auto* source = fixture.makeRegister("shared_source");
    std::vector<rtl::Inst*> sinks;
    sinks.reserve(fanout);
    for (int index = 0; index < fanout; ++index) {
        auto* sink = fixture.makeRegister(
            "shared_sink_" + std::to_string(index));
        fixture.connect(source, sink);
        sinks.push_back(sink);
    }
    fpga::Tile& shared_tile = fpga::Device::current().tile_grid.front();
    require(shared_tile.tryAddAt(sinks.front(), fdPos(0)) == fdPos(0),
            "failed to seed shared-input tile");

    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = placer.fpga->size_width;
    placer.fpga_height = placer.fpga->size_height;
    placer.aspect_x = 1;
    placer.aspect_y = 1;
    placer.place_started = std::chrono::steady_clock::now();
    placer.place_next_report = placer.place_started + std::chrono::hours(1);
    placer.rebuildSharedInputTileIndex(sinks);
    placer.place_tile_trials = 0;

    require(placer.tryAddBySharedInput(
                *sinks.back(), fpga::ELEMENT_FD, {0, 0}) == fdPos(1),
            "high-fanout load did not reuse its indexed shared-input tile");
    require(placer.place_tile_trials == 1,
            "shared-input placement work grew with logical fanout");
}

void pre_smearing_reuses_shared_control_without_scanning_incompatible_tiles()
{
    fpga::TileType type = makeTileType();
    fpga::Element second = type.elements.front();
    second.bitmap_pos = 1;
    type.elements.push_back(second);
    type.pin_map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 1, "CE");
    type.pin_map.input_nodes[1].setBit(43);
    type.pin_map.rememberEndpointRouteRef(fpga::TILE_PIN_INPUT, 1, 43, "FABRIC");
    resetDevice(type, 12, 1);
    auto cb = std::make_unique<fpga::CBType>("FABRIC");
    for (auto& tile : fpga::Device::current().tile_grid) {
        tile.cb_type = tile.cb.type = cb.get();
    }
    Fixture fixture;
    auto* control = fixture.makeRegister("shared_control");
    auto* other_control = fixture.makeRegister("other_control");
    auto makeSink = [&](const std::string& name, Referable<rtl::Inst>* source) {
        auto* sink = fixture.makeRegister(name);
        fixture.conn(sink, "D")->port_ref->name = "CE";
        fixture.connect(source, "Q", sink, "CE");
        return sink;
    };
    std::vector<rtl::Inst*> cells;
    for (int x = 0; x < 12; ++x) {
        auto* owner = makeSink("owner_" + std::to_string(x),
                              x == 11 ? control : other_control);
        placeAt(owner, x, 0);
        cells.push_back(owner);
    }
    // A large shared control is allowed to reuse its endpoint outside the
    // two-Tile data-fanout preference. Only one new sink is being placed here.
    for (int i = 0; i < 9; ++i) makeSink("other_sink_" + std::to_string(i), control);
    auto* target = makeSink("target", control);
    Referable<pnr::RegBunch> bunch;
    bunch.reg = target;
    bunch.x = bunch.y = 0;
    target->outline = {.x = 0, .y = 0};
    target->bunch_ref.set(&bunch);
    cells.push_back(target);
    technology::Tech tech;
    pnr::PlaceDesign placer;
    placer.tech = &tech;
    placer.fpga = &fpga::Device::current();
    placer.tile_grid = &placer.fpga->tile_grid;
    placer.fpga_width = 12;
    placer.fpga_height = 1;
    placer.aspect_x = placer.aspect_y = 1;
    auto result = placer.preSmearBunches(cells);
    // All eleven nearer Tiles have a free register but an incompatible CE.
    // The exact preferred-Tile rejection and indexed compatible trial suffice.
    require(result.failed_bunches == 0 && result.precise_tile_trials == 2,
        "pre-smearing scanned incompatible control owners instead of reusing its index");
    require(placer.bunch_reservations.at(&bunch).placements.front().coord == fpga::Coord{11, 0},
        "shared-input preference bypassed endpoint ownership");
    // Preview teardown must retain every fixed owner and free the new slot.
    for (int x = 0; x < 12; ++x)
        require(cells[x]->coord == fpga::Coord{x, 0}
                    && (*placer.tile_grid)[x].elements_free[fpga::ELEMENT_FD] == 2,
            "shared-input trial changed an existing owner or leaked a reservation");
    require(!target->tile.peer && placer.commitPreSmearReservations() == 1
                && target->coord == fpga::Coord{11, 0} && target->pos == fdPos(1),
        "shared-input preview did not reproduce the same legal committed placement");
}

}

int main()
{
    try {
        technology::Tech::clocked_ports.clear();
        technology::Tech::clocked_ports.emplace("FD", "C");
        calibrated_manhattan_delay();
        local_setup_correction_matches_full_analysis();
        violation_and_force_extraction();
        combinational_critical_path_uses_cell_and_wire_delays();
        refinement_improves_timing_and_keeps_placement_legal();
        rejected_constellation_restores_all_original_slots();
        rejected_constellation_restores_all_original_slots(true);
        initial_smearing_follows_external_timing_nets();
        simultaneous_cooling_movement_shapes_register_constellations();
        clipped_register_motion_translates_its_bunch();
        pre_smearing_reserves_capacity_for_atomic_bunches();
        failed_pre_smear_candidates_do_not_retain_tile_snapshots();
        pre_smearing_counts_coupled_element_occupancy();
        oversized_pre_smearing_scales_with_bunch_size();
        pre_smearing_fixed_io_follower_stays_near_its_outline();
        pre_smearing_uses_nearest_ring_and_external_timing();
        combinational_follower_moves_toward_neighbor_shape();
        timing_deficit_increases_placement_acceleration();
        swapping_accepts_axis_repair_above_threshold();
        swapping_rolls_back_improvement_below_threshold();
        traversal_work_is_near_linear();
        timing_search_defers_to_sparse_region_fallback();
        shared_input_tile_reuse_is_independent_of_fanout_size();
        pre_smearing_reuses_shared_control_without_scanning_incompatible_tiles();
    }
    catch (const TestFailure& failure) {
        std::cerr << "place_timing_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "place_timing_test passed\n";
    return 0;
}
