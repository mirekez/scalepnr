#include "Device.h"
#include "PlaceDesign.h"
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

    Referable<rtl::Inst>* makeLogic(const std::string& name)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name + "_cell";
        cell->type = "LUT2";
        cell->module_ref.set(&primitive);
        for (int index = 0; index < 2; ++index) {
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
        inst->cnt_inputs = 2;
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
    resetDevice(type, 7, 1);
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

void rejected_constellation_restores_all_original_slots()
{
    fpga::TileType type = makeTileType();
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

}

int main()
{
    try {
        calibrated_manhattan_delay();
        violation_and_force_extraction();
        combinational_critical_path_uses_cell_and_wire_delays();
        refinement_improves_timing_and_keeps_placement_legal();
        rejected_constellation_restores_all_original_slots();
        traversal_work_is_near_linear();
    }
    catch (const TestFailure& failure) {
        std::cerr << "place_timing_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "place_timing_test passed\n";
    return 0;
}
