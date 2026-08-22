#include "Device.h"
#include "EstimateDesign.h"
#include "OutlineDesign.h"
#include "Tech.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
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

enum class Direction
{
    left_to_right,
    right_to_left,
    top_to_bottom,
    bottom_to_top,
};

float progress(const rtl::Inst& inst, Direction direction)
{
    switch (direction) {
    case Direction::left_to_right: return inst.outline.x;
    case Direction::right_to_left: return -inst.outline.x;
    case Direction::top_to_bottom: return inst.outline.y;
    case Direction::bottom_to_top: return -inst.outline.y;
    }
    return 0;
}

struct PaintedTree
{
    int id = 0;
    Direction direction = Direction::left_to_right;
    std::vector<Referable<rtl::Inst>*> ordered_cells;
    std::vector<fpga::Coord> painted_coords;
    std::vector<Referable<rtl::Inst>*> inputs;
    Referable<rtl::Inst>* output = nullptr;
};

struct Fixture
{
    static constexpr int tree_count = 128;
    static constexpr int input_count = 10;
    static constexpr int register_count = 10;
    static constexpr int combinational_count = 100;
    static constexpr int chip_width = 100;
    static constexpr int chip_height = 100;
    static constexpr double clock_period_ns = 3.2;

    rtl::Design design;
    Referable<rtl::Module> top_module;
    Referable<rtl::Module> primitive_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    std::vector<PaintedTree> trees;
    clk::Clocks clocks;
    technology::Tech tech;
    fpga::TileType tile_type;
    std::map<std::string, std::string> assignments;
    std::mt19937 random{0x0a711e5u};
    int next_designator = 1;

    Fixture()
    {
        resetDevice();
        initializeRtl();
        configureTechnology();
        buildClock();
        trees.reserve(tree_count);
        for (int id = 0; id < tree_count; ++id) {
            buildTree(id, static_cast<Direction>(id%4));
        }
    }

    void resetDevice()
    {
        tile_type = fpga::TileType{"SYNTHETIC_OUTLINE_TILE", 1, 0};
        tile_type.sites.push_back(fpga::SiteModel{
            .name = "SITE_X0Y0", .type = "IO", .pos = 0});

        fpga::Device& device = fpga::Device::current();
        device.tile_grid.clear();
        device.tile_grid.resize(chip_width*chip_height);
        device.size_width = chip_width;
        device.size_height = chip_height;
        device.grid_spec.size = {chip_width, chip_height};
        device.cnt_luts = chip_width*chip_height*8;
        device.cnt_regs = chip_width*chip_height*8;
        device.pins.clear();
        device.x_to_grid.clear();
        device.y_to_grid.clear();
        for (int coordinate = -2; coordinate <= chip_width + 2; ++coordinate) {
            device.x_to_grid[coordinate] = std::clamp(coordinate, 0, chip_width - 1);
            device.y_to_grid[coordinate] = std::clamp(coordinate, 0, chip_height - 1);
        }
        for (int y = 0; y < chip_height; ++y) {
            for (int x = 0; x < chip_width; ++x) {
                auto& tile = device.tile_grid[static_cast<size_t>(y*chip_width + x)];
                tile.coord = {x, y};
                tile.name = tile.coord;
                tile.cb_coord = tile.coord;
                tile.type = fpga::Tile::TILE_IO;
                tile.tile_type = &tile_type;
                tile.cb_type = nullptr;
                tile.cb.type = nullptr;
                tile.sites = {"SITE_X0Y0"};
                tile.site_types = {"IO"};
                tile.elements_initialized = false;
            }
        }
    }

    void initializeRtl()
    {
        top_module.name = "outline_puzzle_top";
        top_module.is_blackbox = false;
        primitive_module.name = "outline_puzzle_primitives";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&top_module);

        design.top_cell.name = "top";
        design.top_cell.type = "top";
        design.top_cell.module_ref.set(&top_module);
        design.top.cell_ref.set(&design.top_cell);
        design.top.depth = 0;
        design.top.pos = -1;
        constexpr size_t top_port_count = tree_count*(input_count + 1);
        design.top_cell.ports.reserve(top_port_count);
        design.top.conns.reserve(top_port_count);
        cells.reserve(tree_count*(input_count + register_count
                                  + combinational_count + 1) + 1);
        insts.reserve(cells.capacity());
        top_module.nets.reserve(tree_count*(input_count + register_count
                                           + combinational_count + 4));
    }

    void configureTechnology()
    {
        technology::Tech::clocked_ports.clear();
        technology::Tech::buffers_ports.clear();
        technology::Tech::comb_delays.map.clear();
        technology::Tech::clocked_ports.emplace("REG", "C");
        technology::Tech::buffers_ports.emplace("IBUF", "O");
        technology::Tech::buffers_ports.emplace("OBUF", "O");
        // Ten logic arcs between registers consume most of the 3.2 ns period.
        technology::Tech::comb_delays.map["LOGIC2"] = {
            2, {0.30, 0.30}
        };
    }

    Referable<rtl::Inst>* makeInst(
        const std::string& name, const std::string& type,
        const std::vector<std::pair<std::string, int>>& ports)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = type;
        cell->module_ref.set(&primitive_module);
        cell->ports.reserve(ports.size());
        int input_index = 0;
        int output_index = 0;
        for (const auto& [port_name, port_type] : ports) {
            rtl::Port port;
            port.name = port_name;
            port.type = static_cast<decltype(port.type)>(port_type);
            if (port.type == rtl::Port::PORT_IN) {
                port.index = input_index++;
            }
            else if (port.type == rtl::Port::PORT_OUT) {
                port.index = output_index++;
            }
            cell->ports.emplace_back(std::move(port));
        }

        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->parent_ref.set(&design.top);
        inst->cnt_inputs = input_index;
        inst->cnt_outputs = output_index;
        inst->pos = -1;
        inst->conns.reserve(cell->ports.size());
        for (auto& port : cell->ports) {
            auto& connection = inst->conns.emplace_back();
            connection.port_ref.set(&port);
            connection.inst_ref.set(inst.get());
        }

        Referable<rtl::Inst>* result = inst.get();
        cells.push_back(std::move(cell));
        insts.push_back(std::move(inst));
        return result;
    }

    Referable<rtl::Conn>* conn(Referable<rtl::Inst>* inst,
                              const std::string& port_name)
    {
        for (auto& candidate : inst->conns) {
            if (candidate.port_ref.peer && candidate.port_ref->name == port_name) {
                return &candidate;
            }
        }
        return nullptr;
    }

    void recordNet(Referable<rtl::Conn>* output, Referable<rtl::Conn>* input)
    {
        require(output && input, "synthetic outline net has a missing endpoint");
        int designator = next_designator++;
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);
        auto& net = top_module.nets.emplace_back();
        net.name = "outline_net_" + std::to_string(designator);
        net.designators.push_back(designator);
    }

    void connect(Referable<rtl::Inst>* driver, const std::string& output_name,
                 Referable<rtl::Inst>* sink, const std::string& input_name)
    {
        recordNet(conn(driver, output_name), conn(sink, input_name));
    }

    Referable<rtl::Conn>* addTopPort(const std::string& name, int type)
    {
        rtl::Port port;
        port.name = name;
        port.type = static_cast<decltype(port.type)>(type);
        port.index = static_cast<int>(design.top_cell.ports.size());
        port.designator = next_designator++;
        design.top_cell.ports.emplace_back(std::move(port));
        auto& connection = design.top.conns.emplace_back();
        connection.port_ref.set(&design.top_cell.ports.back());
        connection.inst_ref.set(&design.top);
        return &connection;
    }

    void connectTopInput(const std::string& port_name,
                         Referable<rtl::Inst>* input_buffer)
    {
        Referable<rtl::Conn>* top_connection = addTopPort(
            port_name, rtl::Port::PORT_IN);
        recordNet(top_connection, conn(input_buffer, "I"));
    }

    void connectTopOutput(const std::string& port_name,
                          Referable<rtl::Inst>* output_buffer)
    {
        Referable<rtl::Conn>* top_connection = addTopPort(
            port_name, rtl::Port::PORT_OUT);
        Referable<rtl::Conn>* output = conn(output_buffer, "O");
        require(output, "synthetic output buffer has no output");
        output->port_ref->designator = top_connection->port_ref->designator;
        top_connection->set(output);
    }

    void buildClock()
    {
        Referable<rtl::Inst>* source = makeInst(
            "clock_source", "CLOCK_SOURCE", {{"O", rtl::Port::PORT_OUT}});
        Referable<rtl::Conn>* output = conn(source, "O");
        clocks.clocks_list.reserve(1);
        clocks.clocks_list.emplace_back(rtl::Clock{
            .name = "outline_clock",
            .conn_ptr = output,
            .conn_name = "clock_source.O",
            .period_ns = clock_period_ns,
            .duty = 50,
        });
    }

    Referable<rtl::Inst>* clockSource()
    {
        return insts.front().get();
    }

    fpga::Coord edgeCoordinate(Direction direction, int transverse,
                               bool destination) const
    {
        transverse = std::clamp(transverse, 1, chip_width - 2);
        switch (direction) {
        case Direction::left_to_right:
            return {destination ? chip_width - 1 : 0, transverse};
        case Direction::right_to_left:
            return {destination ? 0 : chip_width - 1, transverse};
        case Direction::top_to_bottom:
            return {transverse, destination ? chip_height - 1 : 0};
        case Direction::bottom_to_top:
            return {transverse, destination ? 0 : chip_height - 1};
        }
        return {};
    }

    fpga::Coord paintedCoordinate(Direction direction, int ordinal,
                                  int total, int start_transverse,
                                  int end_transverse, int random_walk)
    {
        double fraction = static_cast<double>(ordinal)/(total - 1);
        int major = static_cast<int>(std::lround(fraction*(chip_width - 1)));
        int transverse = static_cast<int>(std::lround(
            start_transverse + fraction*(end_transverse - start_transverse)))
            + random_walk;
        transverse = std::clamp(transverse, 1, chip_width - 2);
        switch (direction) {
        case Direction::left_to_right: return {major, transverse};
        case Direction::right_to_left: return {chip_width - 1 - major, transverse};
        case Direction::top_to_bottom: return {transverse, major};
        case Direction::bottom_to_top: return {transverse, chip_height - 1 - major};
        }
        return {};
    }

    void addPin(const std::string& port_name, const fpga::Coord& coordinate)
    {
        fpga::Device& device = fpga::Device::current();
        std::string pin_name = "PIN_" + port_name;
        std::string tile_name = tile_type.name + "_X"
            + std::to_string(coordinate.x) + "Y" + std::to_string(coordinate.y);
        device.pins.push_back(fpga::Pin{
            pin_name, "SYNTHETIC", "SITE_X0Y0", tile_name, "", coordinate});
        assignments[port_name] = pin_name;
    }

    void paint(Referable<rtl::Inst>* inst, const fpga::Coord& coordinate)
    {
        fpga::Device& device = fpga::Device::current();
        auto& tile = device.tile_grid[static_cast<size_t>(
            coordinate.y*chip_width + coordinate.x)];
        tile.assign(inst);
        inst->coord = coordinate;
        inst->pos = 0;
        inst->outline.x = static_cast<float>(coordinate.x)*9.95F/(chip_width - 1);
        inst->outline.y = static_cast<float>(coordinate.y)*9.95F/(chip_height - 1);
    }

    Referable<rtl::Inst>* makeRegister(const std::string& name)
    {
        Referable<rtl::Inst>* reg = makeInst(name, "REG", {
            {"D", rtl::Port::PORT_IN},
            {"C", rtl::Port::PORT_IN},
            {"Q", rtl::Port::PORT_OUT},
        });
        connect(clockSource(), "O", reg, "C");
        return reg;
    }

    void buildTree(int id, Direction direction)
    {
        std::uniform_int_distribution<int> transverse_distribution(10, 89);
        std::uniform_int_distribution<int> end_offset_distribution(-14, 14);
        std::uniform_int_distribution<int> input_offset_distribution(-5, 5);
        std::uniform_int_distribution<int> walk_distribution(-1, 1);
        int start_transverse = transverse_distribution(random);
        int end_transverse = std::clamp(
            start_transverse + end_offset_distribution(random), 5, 94);

        PaintedTree tree;
        tree.id = id;
        tree.direction = direction;
        tree.inputs.reserve(input_count);
        tree.ordered_cells.reserve(combinational_count + register_count + 2);
        tree.painted_coords.reserve(tree.ordered_cells.capacity());

        for (int input = 0; input < input_count; ++input) {
            std::string port_name = "input_" + std::to_string(id)
                + "_" + std::to_string(input);
            Referable<rtl::Inst>* buffer = makeInst(
                "tree_" + std::to_string(id) + "." + port_name, "IBUF",
                {{"I", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
            fpga::Coord coordinate = edgeCoordinate(
                direction, start_transverse + input_offset_distribution(random), false);
            paint(buffer, coordinate);
            addPin(port_name, coordinate);
            connectTopInput(port_name, buffer);
            tree.inputs.push_back(buffer);
        }

        Referable<rtl::Inst>* driver = tree.inputs.front();
        std::string driver_port = "O";
        tree.ordered_cells.push_back(driver);
        tree.painted_coords.push_back(driver->coord);
        int random_walk = 0;
        constexpr int ordered_count = combinational_count + register_count + 2;
        int ordinal = 1;
        int next_side_input = 1;
        for (int logic_index = 0; logic_index < combinational_count; ++logic_index) {
            Referable<rtl::Inst>* logic = makeInst(
                "tree_" + std::to_string(id) + ".logic_"
                    + std::to_string(logic_index),
                "LOGIC2", {{"I0", rtl::Port::PORT_IN},
                            {"I1", rtl::Port::PORT_IN},
                            {"O", rtl::Port::PORT_OUT}});
            connect(driver, driver_port, logic, "I0");
            if (next_side_input < input_count
                && logic_index == next_side_input - 1) {
                connect(tree.inputs[static_cast<size_t>(next_side_input)],
                        "O", logic, "I1");
                ++next_side_input;
            }
            random_walk = std::clamp(
                random_walk + walk_distribution(random), -6, 6);
            fpga::Coord coordinate = paintedCoordinate(
                direction, ordinal++, ordered_count, start_transverse,
                end_transverse, random_walk);
            paint(logic, coordinate);
            tree.ordered_cells.push_back(logic);
            tree.painted_coords.push_back(coordinate);
            driver = logic;
            driver_port = "O";

            if ((logic_index + 1)%(combinational_count/register_count) == 0) {
                Referable<rtl::Inst>* reg = makeRegister(
                    "tree_" + std::to_string(id) + ".reg_"
                        + std::to_string((logic_index + 1)
                                         /(combinational_count/register_count) - 1));
                connect(driver, driver_port, reg, "D");
                coordinate = paintedCoordinate(
                    direction, ordinal++, ordered_count, start_transverse,
                    end_transverse, random_walk);
                paint(reg, coordinate);
                tree.ordered_cells.push_back(reg);
                tree.painted_coords.push_back(coordinate);
                driver = reg;
                driver_port = "Q";
            }
        }

        std::string output_port = "output_" + std::to_string(id);
        tree.output = makeInst(
            "tree_" + std::to_string(id) + "." + output_port, "OBUF",
            {{"I", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
        connect(driver, driver_port, tree.output, "I");
        fpga::Coord output_coordinate = edgeCoordinate(
            direction, end_transverse, true);
        paint(tree.output, output_coordinate);
        addPin(output_port, output_coordinate);
        connectTopOutput(output_port, tree.output);
        tree.ordered_cells.push_back(tree.output);
        tree.painted_coords.push_back(output_coordinate);
        trees.push_back(std::move(tree));
    }

    void clearPlacement()
    {
        for (auto& inst : insts) {
            inst->tile.clear();
            inst->coord = {-1, -1};
            inst->pos = -1;
            inst->outline = rtl::OutlineInfo{.x = -1, .y = -1, .fixed = false};
        }
    }
};

struct OrderingStats
{
    double mean_adjacent_order = 0;
    double minimum_adjacent_order = 1;
    double mean_correlation = 0;
    double minimum_correlation = 1;
    double minimum_span = 100;
    int failing_trees = 0;
};

double rankCorrelation(const PaintedTree& tree)
{
    size_t count = tree.ordered_cells.size();
    double mean_index = (count - 1)*0.5;
    double mean_position = 0;
    for (const auto* cell : tree.ordered_cells) {
        mean_position += progress(*cell, tree.direction);
    }
    mean_position /= count;
    double covariance = 0;
    double index_variance = 0;
    double position_variance = 0;
    for (size_t index = 0; index < count; ++index) {
        double index_delta = static_cast<double>(index) - mean_index;
        double position_delta = progress(*tree.ordered_cells[index], tree.direction)
            - mean_position;
        covariance += index_delta*position_delta;
        index_variance += index_delta*index_delta;
        position_variance += position_delta*position_delta;
    }
    if (position_variance == 0) {
        return 0;
    }
    return covariance/std::sqrt(index_variance*position_variance);
}

OrderingStats measureOrdering(const std::vector<PaintedTree>& trees)
{
    OrderingStats result;
    for (const PaintedTree& tree : trees) {
        int ordered_edges = 0;
        for (size_t index = 1; index < tree.ordered_cells.size(); ++index) {
            if (progress(*tree.ordered_cells[index], tree.direction) + 0.01F
                >= progress(*tree.ordered_cells[index - 1], tree.direction)) {
                ++ordered_edges;
            }
        }
        double adjacent_order = static_cast<double>(ordered_edges)
            /(tree.ordered_cells.size() - 1);
        double correlation = rankCorrelation(tree);
        double span = progress(*tree.ordered_cells.back(), tree.direction)
            - progress(*tree.ordered_cells.front(), tree.direction);
        result.mean_adjacent_order += adjacent_order;
        result.mean_correlation += correlation;
        result.minimum_adjacent_order = std::min(
            result.minimum_adjacent_order, adjacent_order);
        result.minimum_correlation = std::min(
            result.minimum_correlation, correlation);
        result.minimum_span = std::min(result.minimum_span, span);
        if (adjacent_order < 0.90 || correlation < 0.90 || span < 8.0) {
            ++result.failing_trees;
        }
    }
    result.mean_adjacent_order /= trees.size();
    result.mean_correlation /= trees.size();
    return result;
}

void painted_timing_trees_become_taut_between_io_anchors()
{
    Fixture fixture;
    require(fixture.trees.size() == Fixture::tree_count,
            "generator produced the wrong tree count");
    for (const PaintedTree& tree : fixture.trees) {
        require(tree.inputs.size() == Fixture::input_count,
                "generator produced the wrong input I/O count");
        require(tree.ordered_cells.size()
                    == Fixture::combinational_count + Fixture::register_count + 2,
                "generator produced the wrong ordered cell count");
    }
    OrderingStats painted_stats = measureOrdering(fixture.trees);
    require(painted_stats.failing_trees == 0,
            "painting generator did not create ordered edge-to-edge trees");

    fixture.clearPlacement();
    pnr::EstimateDesign estimate;
    estimate.tech = &fixture.tech;
    estimate.clocks = &fixture.clocks;
    estimate.estimateDesign(fixture.design);
    require(estimate.data_outs.size() == Fixture::tree_count,
            "EstimateDesign lost painted output trees");
    int clocked_registers = 0;
    for (const auto& inst : fixture.insts) {
        if (inst->cell_ref->type == "REG" && inst->cnt_clocks == 1) {
            ++clocked_registers;
        }
    }
    require(clocked_registers == Fixture::tree_count*Fixture::register_count,
            "synthetic register trees were not attached to the 3.2 ns clock");

    pnr::OutlineDesign outline;
    outline.tech = &fixture.tech;
    outline.placeIOBs(estimate.data_outs, fixture.assignments);
    outline.placeInstIOBs(fixture.design.top, fixture.assignments);
    outline.optimizeOutline(estimate.data_outs);

    int fixed_inputs = 0;
    int fixed_outputs = 0;
    for (const PaintedTree& tree : fixture.trees) {
        for (const auto* input : tree.inputs) {
            fixed_inputs += input->outline.fixed && input->tile.peer ? 1 : 0;
        }
        fixed_outputs += tree.output->outline.fixed && tree.output->tile.peer ? 1 : 0;
    }
    OrderingStats stats = measureOrdering(fixture.trees);
    std::cout << "OUTLINE_DESIGN_TEST trees=" << fixture.trees.size()
              << " cells_per_tree=" << fixture.trees.front().ordered_cells.size()
              << " fixed_inputs=" << fixed_inputs
              << " fixed_outputs=" << fixed_outputs
              << " painted_correlation_min=" << painted_stats.minimum_correlation
              << " adjacent_mean=" << stats.mean_adjacent_order
              << " adjacent_min=" << stats.minimum_adjacent_order
              << " correlation_mean=" << stats.mean_correlation
              << " correlation_min=" << stats.minimum_correlation
              << " span_min=" << stats.minimum_span
              << " failing_trees=" << stats.failing_trees << '\n';

    require(fixed_inputs == Fixture::tree_count*Fixture::input_count,
            "not all input I/O anchors remained fixed");
    require(fixed_outputs == Fixture::tree_count,
            "not all output I/O anchors remained fixed");
    require(stats.failing_trees == 0,
            "OutlineDesign did not make every timing tree taut and ordered");
}

}

int main()
{
    try {
        painted_timing_trees_become_taut_between_io_anchors();
    }
    catch (const TestFailure& failure) {
        std::cerr << "outline_design_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "outline_design_test passed\n";
    return 0;
}
