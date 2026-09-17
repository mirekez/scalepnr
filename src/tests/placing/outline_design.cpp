#include "Device.h"
#include "EstimateDesign.h"
#include "OutlineDesign.h"
#include "Tech.h"

#include <algorithm>
#include <array>
#include <chrono>
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
    northwest_to_southeast,
    southeast_to_northwest,
    southwest_to_northeast,
    northeast_to_southwest,
};

fpga::Coord directionVector(Direction direction)
{
    switch (direction) {
    case Direction::left_to_right: return {1, 0};
    case Direction::right_to_left: return {-1, 0};
    case Direction::top_to_bottom: return {0, 1};
    case Direction::bottom_to_top: return {0, -1};
    case Direction::northwest_to_southeast: return {1, 1};
    case Direction::southeast_to_northwest: return {-1, -1};
    case Direction::southwest_to_northeast: return {1, -1};
    case Direction::northeast_to_southwest: return {-1, 1};
    }
    return {};
}

float progress(const rtl::Inst& inst, Direction direction)
{
    fpga::Coord vector = directionVector(direction);
    return vector.x*inst.outline.x + vector.y*inst.outline.y;
}

struct PaintedTree
{
    int id = 0;
    Direction direction = Direction::left_to_right;
    std::vector<Referable<rtl::Inst>*> ordered_cells;
    std::vector<std::vector<Referable<rtl::Inst>*>> ordered_paths;
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

    explicit Fixture(bool build_linear_design = true)
    {
        resetDevice();
        initializeRtl();
        configureTechnology();
        buildClock();
        trees.reserve(tree_count);
        if (build_linear_design) {
            for (int id = 0; id < tree_count; ++id) {
                buildTree(id, static_cast<Direction>(id%8));
            }
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
        constexpr size_t top_port_count = tree_count*(244 + 1);
        design.top_cell.ports.reserve(top_port_count);
        design.top.conns.reserve(top_port_count);
        cells.reserve(tree_count*(244 + 32 + 128) + 1);
        insts.reserve(cells.capacity());
        top_module.nets.reserve(tree_count*(244 + 32 + 128 + 8));
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
        technology::Tech::comb_delays.map["LOGIC3"] = {
            3, {0.30, 0.30, 0.30}
        };
        technology::Tech::comb_delays.map["LOGIC1"] = {
            1, {0.30}
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
        port.is_global = true;
        design.top_cell.ports.emplace_back(std::move(port));
        auto& connection = design.top.conns.emplace_back();
        connection.port_ref.set(&design.top_cell.ports.back());
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
        case Direction::northwest_to_southeast:
            if (destination) return {chip_width - 1, chip_height - 1};
            return transverse%2 == 0 ? fpga::Coord{0, transverse}
                                     : fpga::Coord{transverse, 0};
        case Direction::southeast_to_northwest:
            if (destination) return {0, 0};
            return transverse%2 == 0 ? fpga::Coord{chip_width - 1, transverse}
                                     : fpga::Coord{transverse, chip_height - 1};
        case Direction::southwest_to_northeast:
            if (destination) return {chip_width - 1, 0};
            return transverse%2 == 0 ? fpga::Coord{0, transverse}
                                     : fpga::Coord{transverse, chip_height - 1};
        case Direction::northeast_to_southwest:
            if (destination) return {0, chip_height - 1};
            return transverse%2 == 0 ? fpga::Coord{chip_width - 1, transverse}
                                     : fpga::Coord{transverse, 0};
        }
        return {};
    }

    fpga::Coord paintedCoordinate(const fpga::Coord& source,
                                  const fpga::Coord& destination,
                                  int ordinal, int total, int random_walk)
    {
        double fraction = static_cast<double>(ordinal)/(total - 1);
        double dx = destination.x - source.x;
        double dy = destination.y - source.y;
        double length = std::max(1.0, std::sqrt(dx*dx + dy*dy));
        int x = static_cast<int>(std::lround(
            source.x + fraction*dx - random_walk*dy/length));
        int y = static_cast<int>(std::lround(
            source.y + fraction*dy + random_walk*dx/length));
        return {std::clamp(x, 0, chip_width - 1),
                std::clamp(y, 0, chip_height - 1)};
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
        fpga::Coord source_coordinate = driver->coord;
        fpga::Coord destination_coordinate = edgeCoordinate(
            direction, end_transverse, true);
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
                source_coordinate, destination_coordinate, ordinal++,
                ordered_count, random_walk);
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
                    source_coordinate, destination_coordinate, ordinal++,
                    ordered_count, random_walk);
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
        fpga::Coord output_coordinate = destination_coordinate;
        paint(tree.output, output_coordinate);
        addPin(output_port, output_coordinate);
        connectTopOutput(output_port, tree.output);
        tree.ordered_cells.push_back(tree.output);
        tree.painted_coords.push_back(output_coordinate);
        tree.ordered_paths.push_back(tree.ordered_cells);
        trees.push_back(std::move(tree));
    }

    fpga::Coord branchingDestination(Direction direction) const
    {
        fpga::Coord vector = directionVector(direction);
        return {
            vector.x > 0 ? chip_width - 1 : (vector.x < 0 ? 0 : chip_width/2),
            vector.y > 0 ? chip_height - 1 : (vector.y < 0 ? 0 : chip_height/2),
        };
    }

    fpga::Coord randomOppositeSource(Direction direction)
    {
        fpga::Coord vector = directionVector(direction);
        std::uniform_int_distribution<int> group_distribution(0, 2);
        std::uniform_int_distribution<int> full_distribution(1, chip_width - 2);
        std::uniform_int_distribution<int> low_distribution(1, chip_width/2 - 1);
        std::uniform_int_distribution<int> high_distribution(
            chip_width/2, chip_width - 2);
        int group = group_distribution(random);
        if (vector.x > 0) {
            if (group == 0) return {0, full_distribution(random)};
            if (group == 1) return {low_distribution(random), 0};
            return {low_distribution(random), chip_height - 1};
        }
        if (vector.x < 0) {
            if (group == 0) return {chip_width - 1, full_distribution(random)};
            if (group == 1) return {high_distribution(random), 0};
            return {high_distribution(random), chip_height - 1};
        }
        if (vector.y > 0) {
            if (group == 0) return {full_distribution(random), 0};
            if (group == 1) return {0, low_distribution(random)};
            return {chip_width - 1, low_distribution(random)};
        }
        if (group == 0) return {full_distribution(random), chip_height - 1};
        if (group == 1) return {0, high_distribution(random)};
        return {chip_width - 1, high_distribution(random)};
    }

    fpga::Coord coordinateFromProjection(Direction direction, double projection,
                                         double perpendicular) const
    {
        fpga::Coord vector = directionVector(direction);
        double denominator = vector.x*vector.x + vector.y*vector.y;
        fpga::Coord best{};
        double best_error = 1e100;
        for (int candidate_perpendicular = -2*chip_width;
             candidate_perpendicular <= 2*chip_width;
             ++candidate_perpendicular) {
            double x = (vector.x*projection
                        - vector.y*candidate_perpendicular)/denominator;
            double y = (vector.y*projection
                        + vector.x*candidate_perpendicular)/denominator;
            if (x < 0 || x > chip_width - 1 || y < 0 || y > chip_height - 1) {
                continue;
            }
            double error = std::abs(candidate_perpendicular - perpendicular);
            if (error < best_error) {
                best_error = error;
                best = {static_cast<int>(std::lround(x)),
                        static_cast<int>(std::lround(y))};
            }
        }
        return best;
    }

    double coordinateProgress(const fpga::Coord& coordinate,
                              Direction direction) const
    {
        fpga::Coord vector = directionVector(direction);
        return vector.x*coordinate.x + vector.y*coordinate.y;
    }

    double coordinatePerpendicular(const fpga::Coord& coordinate,
                                   Direction direction) const
    {
        fpga::Coord vector = directionVector(direction);
        return -vector.y*coordinate.x + vector.x*coordinate.y;
    }

    struct BranchSubtree
    {
        Referable<rtl::Inst>* output = nullptr;
        std::string output_port;
        fpga::Coord coordinate;
        double max_source_progress = 0;
        double perpendicular_sum = 0;
        int leaves = 0;
        std::vector<std::vector<Referable<rtl::Inst>*>> paths;
    };

    struct DecoratedEdge
    {
        Referable<rtl::Inst>* output = nullptr;
        std::string output_port;
        std::vector<Referable<rtl::Inst>*> cells;
    };

    DecoratedEdge decorateBranchEdge(
        int tree_id, int edge_id, Referable<rtl::Inst>* driver,
        std::string driver_port, const fpga::Coord& from,
        const fpga::Coord& to, const std::vector<char>& tokens)
    {
        DecoratedEdge result{driver, std::move(driver_port), {}};
        result.cells.reserve(tokens.size());
        for (size_t index = 0; index < tokens.size(); ++index) {
            Referable<rtl::Inst>* cell = nullptr;
            std::string input_port;
            std::string output_port;
            std::string name = "branch_tree_" + std::to_string(tree_id)
                + ".edge_" + std::to_string(edge_id) + "_"
                + std::to_string(index);
            if (tokens[index] == 'R') {
                cell = makeRegister(name + "_reg");
                input_port = "D";
                output_port = "Q";
            }
            else {
                cell = makeInst(name + "_logic", "LOGIC1",
                    {{"I", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
                input_port = "I";
                output_port = "O";
            }
            connect(result.output, result.output_port, cell, input_port);
            double fraction = static_cast<double>(index + 1)/(tokens.size() + 1);
            fpga::Coord coordinate{
                static_cast<int>(std::lround(from.x + fraction*(to.x - from.x))),
                static_cast<int>(std::lround(from.y + fraction*(to.y - from.y))),
            };
            paint(cell, coordinate);
            result.output = cell;
            result.output_port = std::move(output_port);
            result.cells.push_back(cell);
        }
        return result;
    }

    BranchSubtree buildBranchSubtree(
        PaintedTree& tree, int tree_id, Direction direction, int level,
        int depth, const fpga::Coord& destination,
        const std::vector<std::vector<char>>& edge_tokens,
        int& edge_cursor, int& node_serial, int& input_serial)
    {
        if (level == depth) {
            std::string port_name = "branch_input_" + std::to_string(tree_id)
                + "_" + std::to_string(input_serial++);
            Referable<rtl::Inst>* input = makeInst(
                "branch_tree_" + std::to_string(tree_id) + "." + port_name,
                "IBUF", {{"I", rtl::Port::PORT_IN},
                           {"O", rtl::Port::PORT_OUT}});
            fpga::Coord coordinate = randomOppositeSource(direction);
            paint(input, coordinate);
            addPin(port_name, coordinate);
            connectTopInput(port_name, input);
            tree.inputs.push_back(input);
            return BranchSubtree{
                .output = input,
                .output_port = "O",
                .coordinate = coordinate,
                .max_source_progress = coordinateProgress(coordinate, direction),
                .perpendicular_sum = coordinatePerpendicular(coordinate, direction),
                .leaves = 1,
                .paths = {{input}},
            };
        }

        std::array<BranchSubtree, 3> children;
        double max_source_progress = -1000000;
        double perpendicular_sum = 0;
        int leaves = 0;
        for (BranchSubtree& child : children) {
            child = buildBranchSubtree(
                tree, tree_id, direction, level + 1, depth, destination,
                edge_tokens, edge_cursor, node_serial, input_serial);
            max_source_progress = std::max(
                max_source_progress, child.max_source_progress);
            perpendicular_sum += child.perpendicular_sum;
            leaves += child.leaves;
        }

        double destination_progress = coordinateProgress(destination, direction);
        double fraction = static_cast<double>(depth - level)/(depth + 1);
        double node_progress = max_source_progress
            + fraction*(destination_progress - max_source_progress);
        fpga::Coord coordinate = coordinateFromProjection(
            direction, node_progress, perpendicular_sum/leaves);
        Referable<rtl::Inst>* node = makeInst(
            "branch_tree_" + std::to_string(tree_id) + ".fork_"
                + std::to_string(node_serial++),
            "LOGIC3", {{"I0", rtl::Port::PORT_IN},
                        {"I1", rtl::Port::PORT_IN},
                        {"I2", rtl::Port::PORT_IN},
                        {"O", rtl::Port::PORT_OUT}});
        paint(node, coordinate);

        std::vector<std::vector<Referable<rtl::Inst>*>> paths;
        for (size_t child_index = 0; child_index < children.size(); ++child_index) {
            int edge_id = edge_cursor++;
            BranchSubtree& child = children[child_index];
            DecoratedEdge edge = decorateBranchEdge(
                tree_id, edge_id, child.output, child.output_port,
                child.coordinate, coordinate,
                edge_tokens[static_cast<size_t>(edge_id)]);
            connect(edge.output, edge.output_port, node,
                    "I" + std::to_string(child_index));
            for (auto& path : child.paths) {
                path.insert(path.end(), edge.cells.begin(), edge.cells.end());
                path.push_back(node);
                paths.push_back(std::move(path));
            }
        }
        return BranchSubtree{
            .output = node,
            .output_port = "O",
            .coordinate = coordinate,
            .max_source_progress = max_source_progress,
            .perpendicular_sum = perpendicular_sum,
            .leaves = leaves,
            .paths = std::move(paths),
        };
    }

    void buildBranchingTree(int id, Direction direction, int depth,
                            int total_registers, int total_combinational)
    {
        int power = 1;
        for (int level = 0; level < depth; ++level) power *= 3;
        int internal_nodes = (power - 1)/2;
        require(total_combinational >= internal_nodes,
                "branching tree needs more combinational fork cells");
        int edge_count = internal_nodes*3 + 1;
        std::vector<std::vector<char>> edge_tokens(
            static_cast<size_t>(edge_count));
        std::vector<int> edge_order(static_cast<size_t>(edge_count));
        std::iota(edge_order.begin(), edge_order.end(), 0);
        std::shuffle(edge_order.begin(), edge_order.end(), random);
        std::vector<char> tokens(static_cast<size_t>(total_registers), 'R');
        tokens.insert(tokens.end(),
                      static_cast<size_t>(total_combinational - internal_nodes), 'C');
        std::shuffle(tokens.begin(), tokens.end(), random);
        for (size_t index = 0; index < tokens.size(); ++index) {
            edge_tokens[static_cast<size_t>(edge_order[index])].push_back(tokens[index]);
        }

        PaintedTree tree;
        tree.id = id;
        tree.direction = direction;
        tree.inputs.reserve(static_cast<size_t>(power));
        fpga::Coord destination = branchingDestination(direction);
        int edge_cursor = 0;
        int node_serial = 0;
        int input_serial = 0;
        BranchSubtree root = buildBranchSubtree(
            tree, id, direction, 0, depth, destination, edge_tokens,
            edge_cursor, node_serial, input_serial);

        std::string output_port_name = "branch_output_" + std::to_string(id);
        tree.output = makeInst(
            "branch_tree_" + std::to_string(id) + "." + output_port_name,
            "OBUF", {{"I", rtl::Port::PORT_IN},
                       {"O", rtl::Port::PORT_OUT}});
        paint(tree.output, destination);
        int root_edge_id = edge_cursor++;
        DecoratedEdge root_edge = decorateBranchEdge(
            id, root_edge_id, root.output, root.output_port,
            root.coordinate, destination,
            edge_tokens[static_cast<size_t>(root_edge_id)]);
        connect(root_edge.output, root_edge.output_port, tree.output, "I");
        addPin(output_port_name, destination);
        connectTopOutput(output_port_name, tree.output);
        for (auto& path : root.paths) {
            path.insert(path.end(), root_edge.cells.begin(), root_edge.cells.end());
            path.push_back(tree.output);
            tree.ordered_paths.push_back(std::move(path));
        }
        require(edge_cursor == edge_count,
                "branching generator did not consume every tree edge");
        tree.ordered_cells = tree.ordered_paths.front();
        trees.push_back(std::move(tree));
    }

    void buildBranchingDesign(int depth, int total_registers,
                              int total_combinational)
    {
        for (int id = 0; id < tree_count; ++id) {
            buildBranchingTree(id, static_cast<Direction>(id%8), depth,
                               total_registers, total_combinational);
        }
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
    double minimum_span_ratio = 100;
    int failing_paths = 0;
};

double rankCorrelation(const std::vector<Referable<rtl::Inst>*>& path,
                       Direction direction)
{
    size_t count = path.size();
    double mean_index = (count - 1)*0.5;
    double mean_position = 0;
    for (const auto* cell : path) {
        mean_position += progress(*cell, direction);
    }
    mean_position /= count;
    double covariance = 0;
    double index_variance = 0;
    double position_variance = 0;
    for (size_t index = 0; index < count; ++index) {
        double index_delta = static_cast<double>(index) - mean_index;
        double position_delta = progress(*path[index], direction)
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
      for (const auto& path : tree.ordered_paths) {
        int ordered_edges = 0;
        for (size_t index = 1; index < path.size(); ++index) {
            if (progress(*path[index], tree.direction) + 0.01F
                >= progress(*path[index - 1], tree.direction)) {
                ++ordered_edges;
            }
        }
        double adjacent_order = static_cast<double>(ordered_edges)
            /(path.size() - 1);
        double correlation = rankCorrelation(path, tree.direction);
        double span = progress(*path.back(), tree.direction)
            - progress(*path.front(), tree.direction);
        double anchor_span = progress(*tree.output, tree.direction)
            - progress(*path.front(), tree.direction);
        double span_ratio = anchor_span > 0 ? span/anchor_span : 0;
        result.mean_adjacent_order += adjacent_order;
        result.mean_correlation += correlation;
        result.minimum_adjacent_order = std::min(
            result.minimum_adjacent_order, adjacent_order);
        result.minimum_correlation = std::min(
            result.minimum_correlation, correlation);
        result.minimum_span_ratio = std::min(result.minimum_span_ratio, span_ratio);
        if (adjacent_order < 0.90 || span_ratio < 0.80) {
            ++result.failing_paths;
        }
      }
    }
    size_t path_count = 0;
    for (const PaintedTree& tree : trees) path_count += tree.ordered_paths.size();
    result.mean_adjacent_order /= path_count;
    result.mean_correlation /= path_count;
    return result;
}

void runPaintedPuzzle(Fixture& fixture, const std::string& label,
                      int expected_inputs_per_tree, int expected_registers_per_tree,
                      int expected_combinational_per_tree)
{
    require(fixture.trees.size() == Fixture::tree_count,
            "generator produced the wrong tree count");
    for (const PaintedTree& tree : fixture.trees) {
        require(tree.inputs.size() == static_cast<size_t>(expected_inputs_per_tree),
                "generator produced the wrong input I/O count");
        require(!tree.ordered_paths.empty(),
                "generator produced no source-to-destination paths");
    }
    OrderingStats painted_stats = measureOrdering(fixture.trees);
    std::cout << "OUTLINE_GENERATION case=" << label
              << " paths=";
    size_t generated_path_count = 0;
    for (const PaintedTree& tree : fixture.trees) {
        generated_path_count += tree.ordered_paths.size();
    }
    std::cout << generated_path_count
              << " adjacent_min=" << painted_stats.minimum_adjacent_order
              << " correlation_min=" << painted_stats.minimum_correlation
              << " span_ratio_min=" << painted_stats.minimum_span_ratio
              << " failing_paths=" << painted_stats.failing_paths << '\n';
    require(painted_stats.failing_paths == 0,
            "painting generator did not create ordered edge-to-edge trees");

    fixture.clearPlacement();
    auto estimate_started = std::chrono::steady_clock::now();
    pnr::EstimateDesign estimate;
    estimate.tech = &fixture.tech;
    estimate.clocks = &fixture.clocks;
    estimate.estimateDesign(fixture.design);
    double estimate_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - estimate_started).count();
    std::cout << "OUTLINE_PUZZLE_PHASE case=" << label
              << " phase=estimate elapsed_s=" << estimate_seconds << std::endl;
    require(estimate.data_outs.size() == Fixture::tree_count,
            "EstimateDesign lost painted output trees");
    int clocked_registers = 0;
    int combinational_cells = 0;
    for (const auto& inst : fixture.insts) {
        if (inst->cell_ref->type == "REG" && inst->cnt_clocks == 1) {
            ++clocked_registers;
        }
        if (inst->cell_ref->type == "LOGIC1"
            || inst->cell_ref->type == "LOGIC2"
            || inst->cell_ref->type == "LOGIC3") {
            ++combinational_cells;
        }
    }
    require(clocked_registers == Fixture::tree_count*expected_registers_per_tree,
            "synthetic register trees were not attached to the 3.2 ns clock");
    require(combinational_cells
                == Fixture::tree_count*expected_combinational_per_tree,
            "generator produced the wrong number of combinational cells");

    pnr::OutlineDesign outline;
    outline.tech = &fixture.tech;
    auto anchors_started = std::chrono::steady_clock::now();
    outline.placeIOBs(estimate.data_outs, fixture.assignments);
    outline.placeInstIOBs(fixture.design.top, fixture.assignments);
    double anchors_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - anchors_started).count();
    std::cout << "OUTLINE_PUZZLE_PHASE case=" << label
              << " phase=anchors elapsed_s=" << anchors_seconds << std::endl;
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
    size_t path_count = 0;
    for (const PaintedTree& tree : fixture.trees) {
        path_count += tree.ordered_paths.size();
    }
    std::cout << "OUTLINE_DESIGN_TEST case=" << label
              << " trees=" << fixture.trees.size()
              << " total_cells=" << fixture.insts.size()
              << " paths=" << path_count
              << " fixed_inputs=" << fixed_inputs
              << " fixed_outputs=" << fixed_outputs
              << " painted_correlation_min=" << painted_stats.minimum_correlation
              << " adjacent_mean=" << stats.mean_adjacent_order
              << " adjacent_min=" << stats.minimum_adjacent_order
              << " correlation_mean=" << stats.mean_correlation
              << " correlation_min=" << stats.minimum_correlation
              << " span_ratio_min=" << stats.minimum_span_ratio
              << " failing_paths=" << stats.failing_paths << '\n';

    require(fixed_inputs == Fixture::tree_count*expected_inputs_per_tree,
            "not all input I/O anchors remained fixed");
    require(fixed_outputs == Fixture::tree_count,
            "not all output I/O anchors remained fixed");
    // Capacity-aware outlining may locally cross neighboring cells, but the
    // complete paths must still follow their anchor direction and span.
    require(stats.mean_adjacent_order >= 0.65
                && stats.minimum_adjacent_order >= 0.50,
            "OutlineDesign lost aggregate adjacent path direction");
    require(stats.mean_correlation >= 0.90
                && stats.minimum_correlation >= 0.60,
            "OutlineDesign lost source-to-destination path correlation");
    require(stats.minimum_span_ratio >= 0.80,
            "OutlineDesign did not span every source-to-destination anchor");
}

void painted_timing_chains_become_taut_between_io_anchors()
{
    Fixture fixture;
    runPaintedPuzzle(fixture, "directional", Fixture::input_count,
                     Fixture::register_count, Fixture::combinational_count);
}

void branching_100kcells()
{
    constexpr int branch_depth = 5;
    constexpr int branch_inputs = 243;
    constexpr int registers = 32;
    constexpr int combinational = 128;
    Fixture fixture(false);
    fixture.buildBranchingDesign(branch_depth, registers, combinational);
    runPaintedPuzzle(fixture, "100Kcells", branch_inputs,
                     registers, combinational);
}

void repeated_outline_reuses_occupancy_storage()
{
    Fixture fixture(false);
    pnr::OutlineDesign outline;
    outline.tech = &fixture.tech;
    std::list<Referable<pnr::RegBunch>> bunches;
    outline.optimizeOutline(bunches);
    const auto* storage = outline.boxes1.data();
    const auto capacity = outline.boxes1.capacity();
    for (int run = 0; run < 16; ++run) {
        std::fill(outline.boxes1.begin(), outline.boxes1.end(), 7);
        outline.optimizeOutline(bunches);
        // A second run used to overwrite the raw pointer, leaking the old grid.
        require(outline.boxes1.data() == storage
                    && outline.boxes1.capacity() == capacity,
                "repeated Outline replaced rather than reused its occupancy grid");
        // Reuse must clear previous occupancy instead of preserving stale cells.
        require(outline.boxes1.size() == 4*Fixture::chip_width*Fixture::chip_height
                    && std::ranges::all_of(outline.boxes1,
                        [](int count) { return count == 0; }),
                "repeated Outline retained occupancy or resized the grid incorrectly");
    }
}

void anchored_capacity_overflow_stays_near_preferred_tile()
{
    Fixture fixture(false);
    fixture.tile_type.elements.clear();
    fixture.tile_type.elements.push_back(fpga::Element{
        .name = "REGISTER_SLOT",
        .type = fpga::ELEMENT_FD,
        .bitmap_pos = 0,
    });

    std::array<Referable<rtl::Inst>*, 3> registers{};
    for (size_t index = 0; index < registers.size(); ++index) {
        registers[index] = fixture.makeInst(
            "capacity_register_" + std::to_string(index), "FDRE",
            {{"D", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
        registers[index]->outline = rtl::OutlineInfo{
            .x = 5.05F, .y = 5.05F, .fixed = false};
    }

    pnr::OutlineDesign outline;
    outline.uniform_unanchored_allocation = false;
    for (Referable<rtl::Inst>* inst : registers) {
        outline.optimization_peers[inst] = {};
    }
    outline.legalizeOutlineCapacity();

    for (Referable<rtl::Inst>* inst : registers) {
        fpga::Coord target{
            static_cast<int>(inst->outline.x*Fixture::chip_width
                             / pnr::OutlineDesign::mesh_width),
            static_cast<int>(inst->outline.y*Fixture::chip_height
                             / pnr::OutlineDesign::mesh_height),
        };
        int distance = std::abs(target.x - 50) + std::abs(target.y - 50);
        require(distance <= 1,
                "anchored capacity overflow selected a distant row-major tile");
    }
}

}

int main(int argc, char** argv)
{
    try {
        std::string selected = argc > 1 ? argv[1] : "all";
        if (selected == "directional" || selected == "all") {
            painted_timing_chains_become_taut_between_io_anchors();
        }
        if (selected == "100Kcells" || selected == "all") {
            branching_100kcells();
        }
        if (selected == "capacity" || selected == "all") {
            anchored_capacity_overflow_stays_near_preferred_tile();
        }
        if (selected == "memory" || selected == "all") {
            repeated_outline_reuses_occupancy_storage();
        }
        require(selected == "directional" || selected == "100Kcells"
                    || selected == "capacity" || selected == "memory" || selected == "all",
                "unknown outline puzzle case '" + selected + "'");
    }
    catch (const TestFailure& failure) {
        std::cerr << "outline_design_test: " << failure.message << '\n';
        return 1;
    }
    std::cout << "outline_design_test passed\n";
    return 0;
}
