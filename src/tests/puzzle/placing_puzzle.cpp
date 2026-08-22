#include "Device.h"
#include "Element.h"
#include "EstimateDesign.h"
#include "OutlineDesign.h"
#include "PlaceDesign.h"
#include "PlaceTiming.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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

struct PuzzleParameters
{
    int size = 0;
    int fullness_percent = 0;
};

constexpr int kLogicRegionsPerTile = 2;
constexpr int kLutsPerRegion = 8;
constexpr int kRegistersPerRegion = 8;
constexpr int kLutsPerTile = kLogicRegionsPerTile*kLutsPerRegion;
constexpr int kRegistersPerTile = kLogicRegionsPerTile*kRegistersPerRegion;
constexpr int kCellsPerTile = kLutsPerTile + kRegistersPerTile;
constexpr double kClockPeriodNs = 1.0;
constexpr uint64_t kPuzzleSeed = 0x51ace91aULL;

int parsePositive(const char* text, const std::string& name)
{
    int value = 0;
    const char* end = text + std::char_traits<char>::length(text);
    auto result = std::from_chars(text, end, value);
    require(result.ec == std::errc{} && result.ptr == end && value > 0,
            name + " must be a positive integer");
    return value;
}

fpga::Element makeElement(const std::string& name,
                          fpga::ElementType type, int bit)
{
    fpga::Element element;
    element.name = name;
    element.type = type;
    element.bitmap_pos = static_cast<uint16_t>(bit);
    element.elements_to_left = static_cast<int>(type);
    return element;
}

fpga::TileType makePuzzleTileType()
{
    fpga::TileType type{"PLACEMENT_PUZZLE_LOGIC", 0,
                        fpga::Tile::TILE_LUTS};
    for (int region = 0; region < kLogicRegionsPerTile; ++region) {
        type.sites.push_back(fpga::SiteModel{
            .name = std::format("LOGIC_REGION_{}", region),
            .type = "LOGIC_REGION",
            .pos = region,
        });
    }
    for (int bit = 0; bit < kLutsPerTile; ++bit) {
        type.elements.push_back(makeElement(
            std::format("LUT6_{}", bit), fpga::ELEMENT_LUT5, bit));
    }
    for (int bit = 0; bit < kRegistersPerTile; ++bit) {
        type.elements.push_back(makeElement(
            std::format("REG_{}", bit), fpga::ELEMENT_FD, bit));
    }
    return type;
}

void resetDevice(fpga::TileType& tile_type, int size)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.tile_types.clear();
    device.tileconn_rules.clear();
    device.local_route_wire_mappings.clear();
    device.route_wire_graph.clear();
    device.pins.clear();
    device.grid_spec.size = {size, size};
    device.size_width = size;
    device.size_height = size;
    device.cnt_luts = size*size*kLutsPerTile;
    device.cnt_regs = size*size*kRegistersPerTile;
    device.tile_grid.resize(static_cast<size_t>(size*size));
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y*size + x)];
            tile = {};
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = tile.coord;
            tile.type = fpga::Tile::TILE_LUTS;
            tile.tile_type = &tile_type;
            tile.cb_type = nullptr;
            tile.cb.type = nullptr;
            tile.full_name = std::format("PLACEMENT_TILE_X{}Y{}", x, y);
        }
    }
    // Every synthetic Tile owns its abstract packing resources directly.
    for (fpga::Tile& tile : device.tile_grid) {
        tile.attached_resource_tiles = {&tile};
    }
}

enum class CellKind
{
    clock,
    lut,
    reg,
    output_buffer,
};

struct PuzzleCell
{
    rtl::Inst* inst = nullptr;
    CellKind kind = CellKind::lut;
    fpga::Coord generated_coord{-1, -1};

    bool isLut() const { return kind == CellKind::lut; }
    bool isRegister() const { return kind == CellKind::reg; }
};

struct MeshStats
{
    size_t core_cells = 0;
    size_t luts = 0;
    size_t registers = 0;
    size_t data_connections = 0;
    size_t horizontal_connections = 0;
    size_t vertical_connections = 0;
    size_t dual_direction_cells = 0;
    size_t inserted_registers = 0;
};

struct CellPlacementHistory
{
    fpga::Coord generated{-1, -1};
    float outline_x = -1;
    float outline_y = -1;
    fpga::Coord outline_target{-1, -1};
    fpga::Coord after_place{-1, -1};
    int after_place_pos = -1;
};

struct PlacementPuzzle
{
    PuzzleParameters parameters;
    fpga::TileType tile_type;
    technology::Tech tech;
    Referable<rtl::Module> top_module;
    Referable<rtl::Module> primitive_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<PuzzleCell> core_cells;
    std::vector<PuzzleCell> output_buffers;
    std::vector<std::vector<rtl::Inst*>> pending;
    std::mt19937_64 random{kPuzzleSeed};
    rtl::Inst* clock_source = nullptr;
    Referable<rtl::Conn>* clock_output = nullptr;
    std::unordered_map<int, size_t> net_by_designator;
    std::unordered_map<rtl::Inst*, size_t> mesh_degree;
    std::unordered_map<rtl::Inst*, CellPlacementHistory> placement_history;
    int next_designator = 1;
    MeshStats stats;

    explicit PlacementPuzzle(PuzzleParameters input)
        : parameters(input)
        , tile_type(makePuzzleTileType())
        , pending(static_cast<size_t>(input.size*input.size))
    {
        require(parameters.size >= 2,
                "placing-puzzle Tile space must be at least 2x2");
        require(parameters.fullness_percent >= 1
                    && parameters.fullness_percent <= 100,
                "placing-puzzle fullness must be between 1 and 100 percent");
        resetDevice(tile_type, parameters.size);
        initializeTechnology();
        initializeDesign();
        buildClock();
        generateMesh();
        addTerminalOutputs();
    }

    void initializeTechnology()
    {
        technology::Tech::clocked_ports.clear();
        technology::Tech::buffers_ports.clear();
        technology::Tech::comb_delays.map.clear();
        technology::Tech::clocked_ports.emplace("FD", "C");
        technology::Tech::buffers_ports.emplace("OBUF", "O");
        technology::Tech::comb_delays.map["LUT6"] = {
            6, std::vector<double>(6, 0.10)};

        tech.clocks.tech = &tech;
        tech.timings.tech = &tech;
        tech.estimate.tech = &tech;
        tech.estimate.clocks = &tech.clocks;
        tech.outline.tech = &tech;
        tech.place.tech = &tech;
        tech.place.place_timing.tech = &tech;
    }

    void initializeDesign()
    {
        size_t tile_count = static_cast<size_t>(parameters.size*parameters.size);
        size_t requested_cells = std::max<size_t>(
            tile_count,
            (tile_count*kCellsPerTile
                * static_cast<size_t>(parameters.fullness_percent) + 50)/100);
        cells.reserve(requested_cells + kCellsPerTile + 1);
        core_cells.reserve(requested_cells + 64);
        output_buffers.reserve(kCellsPerTile);
        top_module.name = "placement_puzzle_top";
        top_module.is_blackbox = false;
        top_module.nets.reserve(requested_cells*2 + 1);
        primitive_module.name = "placement_puzzle_primitives";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&top_module);

        tech.design.top_cell.name = "top";
        tech.design.top_cell.type = "top";
        tech.design.top_cell.module_ref.set(&top_module);
        tech.design.top.cell_ref.set(&tech.design.top_cell);
        tech.design.top.depth = 0;
        tech.design.top.pos = -1;
        tech.design.top_cell.ports.reserve(kCellsPerTile);
        tech.design.top.conns.reserve(kCellsPerTile);
    }

    static void addPort(rtl::Cell& cell, const std::string& name,
                        int type, int index)
    {
        rtl::Port port;
        port.name = name;
        port.type = static_cast<decltype(port.type)>(type);
        port.index = index;
        cell.ports.emplace_back(std::move(port));
    }

    PuzzleCell makeCell(const std::string& name, CellKind kind)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->module_ref.set(&primitive_module);
        switch (kind) {
        case CellKind::clock:
            cell->type = "CLOCK_SOURCE";
            cell->ports.reserve(1);
            addPort(*cell, "O", rtl::Port::PORT_OUT, 0);
            break;
        case CellKind::lut:
            cell->type = "LUT6";
            cell->ports.reserve(7);
            for (int input = 0; input < 6; ++input) {
                addPort(*cell, "I" + std::to_string(input),
                        rtl::Port::PORT_IN, input);
            }
            addPort(*cell, "O", rtl::Port::PORT_OUT, 0);
            break;
        case CellKind::reg:
            cell->type = "FD";
            cell->ports.reserve(3);
            addPort(*cell, "D", rtl::Port::PORT_IN, 0);
            addPort(*cell, "C", rtl::Port::PORT_IN, 1);
            addPort(*cell, "Q", rtl::Port::PORT_OUT, 0);
            break;
        case CellKind::output_buffer:
            cell->type = "OBUF";
            cell->ports.reserve(2);
            addPort(*cell, "I", rtl::Port::PORT_IN, 0);
            addPort(*cell, "O", rtl::Port::PORT_OUT, 0);
            break;
        }

        auto& inst = tech.design.top.insts.emplace_back();
        inst.cell_ref.set(cell.get());
        inst.depth = 1;
        inst.height = 0;
        inst.cnt_inputs = kind == CellKind::lut ? 6
            : (kind == CellKind::clock ? 0 : 1);
        inst.cnt_outputs = 1;
        inst.coord = {-1, -1};
        inst.pos = -1;
        inst.conns.reserve(cell->ports.size());
        for (auto& port : cell->ports) {
            auto& connection = inst.conns.emplace_back();
            connection.port_ref.set(&port);
            connection.inst_ref.set(&inst);
        }
        cells.push_back(std::move(cell));
        return PuzzleCell{&inst, kind, {-1, -1}};
    }

    Referable<rtl::Conn>* connection(rtl::Inst& inst,
                                     const std::string& port_name)
    {
        for (auto& candidate : inst.conns) {
            if (candidate.port_ref.peer
                && candidate.port_ref->name == port_name) {
                return &candidate;
            }
        }
        return nullptr;
    }

    Referable<rtl::Conn>* outputConnection(rtl::Inst& inst)
    {
        for (auto& candidate : inst.conns) {
            if (candidate.port_ref.peer
                && candidate.port_ref->type == rtl::Port::PORT_OUT) {
                return &candidate;
            }
        }
        return nullptr;
    }

    Referable<rtl::Conn>* freeDataInput(rtl::Inst& inst)
    {
        for (auto& candidate : inst.conns) {
            if (!candidate.port_ref.peer
                || candidate.port_ref->type != rtl::Port::PORT_IN
                || candidate.peer != nullptr) {
                continue;
            }
            if (tech.check_clocked(inst.cell_ref->type,
                                   candidate.port_ref->name)) {
                continue;
            }
            return &candidate;
        }
        return nullptr;
    }

    rtl::Net& netForOutput(Referable<rtl::Conn>& output,
                           const std::string& prefix)
    {
        if (output.port_ref->designator < 0) {
            int designator = next_designator++;
            output.port_ref->designator = designator;
            size_t net_index = top_module.nets.size();
            auto& net = top_module.nets.emplace_back();
            net.name = prefix + std::to_string(designator);
            net.designators.push_back(designator);
            net_by_designator.emplace(designator, net_index);
            return net;
        }
        int designator = output.port_ref->designator;
        auto found = net_by_designator.find(designator);
        require(found != net_by_designator.end()
                    && found->second < top_module.nets.size(),
                "placing-puzzle output designator has no RTL net");
        return top_module.nets[found->second];
    }

    void connectToInput(rtl::Inst& driver, Referable<rtl::Conn>& input,
                        const std::string& prefix = "mesh_net_")
    {
        Referable<rtl::Conn>* output = outputConnection(driver);
        require(output && input.peer == nullptr,
                "placing-puzzle connection has an occupied or missing endpoint");
        rtl::Net& net = netForOutput(*output, prefix);
        input.port_ref->designator = output->port_ref->designator;
        input.set(output);
        (void)net;
    }

    void buildClock()
    {
        PuzzleCell clock = makeCell("placement_clock_source", CellKind::clock);
        clock_source = clock.inst;
        clock_output = outputConnection(*clock_source);
        require(clock_output, "placing-puzzle clock has no output");
        tech.clocks.clocks_list.reserve(1);
        tech.clocks.clocks_list.emplace_back(rtl::Clock{
            .name = "placement_clock",
            .conn_ptr = clock_output,
            .conn_name = "placement_clock_source.O",
            .period_ns = kClockPeriodNs,
            .duty = 50,
        });
    }

    void connectClock(rtl::Inst& reg)
    {
        Referable<rtl::Conn>* clock_input = connection(reg, "C");
        require(clock_input, "placing-puzzle register has no clock input");
        connectToInput(*clock_source, *clock_input, "clock_net_");
    }

    size_t requestedCoreCells() const
    {
        size_t tile_count = static_cast<size_t>(parameters.size*parameters.size);
        return std::max<size_t>(
            tile_count,
            (tile_count*kCellsPerTile
                * static_cast<size_t>(parameters.fullness_percent) + 50)/100);
    }

    std::vector<CellKind> tileKinds(int count, int tile_index,
                                    int& remaining_luts,
                                    int remaining_tiles)
    {
        int minimum_luts = std::max(0, count - kRegistersPerTile);
        int maximum_luts = std::min(count, kLutsPerTile);
        int luts = std::clamp(
            static_cast<int>(std::lround(
                static_cast<double>(remaining_luts)/remaining_tiles)),
            minimum_luts, maximum_luts);
        remaining_luts -= luts;
        std::vector<CellKind> kinds(static_cast<size_t>(count), CellKind::reg);
        std::fill_n(kinds.begin(), luts, CellKind::lut);
        std::shuffle(kinds.begin(), kinds.end(), random);
        (void)tile_index;
        return kinds;
    }

    void placeGeneratedCell(PuzzleCell& cell, fpga::Tile& tile,
                            std::vector<int>& lut_bits,
                            std::vector<int>& register_bits)
    {
        std::vector<int>& bits = cell.isLut() ? lut_bits : register_bits;
        require(!bits.empty(),
                "placing-puzzle generated more cells than a Tile can hold");
        int bit = bits.back();
        bits.pop_back();
        int pos = -1;
        if (cell.isLut()) {
            int site = bit/4;
            int bel = bit%4;
            pos = site*128 + bel*4 + 3;
        }
        else {
            int site = bit/8;
            int lane = bit%8;
            int bel = lane%4;
            int column = lane >= 4 ? 64 : 0;
            pos = site*128 + column + bel*4;
        }
        int placed = tile.tryAddAt(cell.inst, pos);
        require(placed == pos,
                "placing-puzzle failed to install its known legal placement");
        cell.generated_coord = tile.coord;
        float divisor = static_cast<float>(std::max(1, parameters.size - 1));
        cell.inst->outline.x = tile.coord.x*9.95F/divisor;
        cell.inst->outline.y = tile.coord.y*9.95F/divisor;
        if (cell.isRegister()) {
            connectClock(*cell.inst);
        }
    }

    rtl::Inst* chooseSink(const PuzzleCell& driver,
                          std::vector<PuzzleCell*>& tile_cells,
                          bool cover_every_sink)
    {
        std::vector<PuzzleCell*> unconnected_preferred;
        std::vector<PuzzleCell*> unconnected_fallback;
        std::vector<PuzzleCell*> preferred;
        std::vector<PuzzleCell*> fallback;
        unconnected_preferred.reserve(tile_cells.size());
        unconnected_fallback.reserve(tile_cells.size());
        preferred.reserve(tile_cells.size());
        fallback.reserve(tile_cells.size());
        for (PuzzleCell* sink : tile_cells) {
            if (!freeDataInput(*sink->inst)) {
                continue;
            }
            bool unconnected = mesh_degree[sink->inst] == 0;
            fallback.push_back(sink);
            if (unconnected) {
                unconnected_fallback.push_back(sink);
            }
            if ((driver.isLut() && sink->isRegister())
                || (!driver.isLut() && sink->isLut())) {
                preferred.push_back(sink);
                if (unconnected) {
                    unconnected_preferred.push_back(sink);
                }
            }
        }
        std::vector<PuzzleCell*>* choices = &fallback;
        if (cover_every_sink && !unconnected_preferred.empty()) {
            choices = &unconnected_preferred;
        }
        else if (cover_every_sink && !unconnected_fallback.empty()) {
            choices = &unconnected_fallback;
        }
        else if (!preferred.empty()) {
            choices = &preferred;
        }
        if (choices->empty()) {
            return nullptr;
        }
        return (*choices)[static_cast<size_t>(random()%choices->size())]->inst;
    }

    void consumePending(int tile_index, std::vector<PuzzleCell*>& tile_cells)
    {
        std::vector<rtl::Inst*>& drivers = pending[static_cast<size_t>(tile_index)];
        bool terminal_tile = tile_index
            == parameters.size*parameters.size - 1;
        std::shuffle(drivers.begin(), drivers.end(), random);
        for (rtl::Inst* driver_inst : drivers) {
            PuzzleCell driver{driver_inst,
                driver_inst->cell_ref->type == "LUT6"
                    ? CellKind::lut : CellKind::reg,
                driver_inst->coord};
            rtl::Inst* sink = chooseSink(driver, tile_cells, terminal_tile);
            if (!sink) {
                require(mesh_degree[driver_inst] > 0,
                        "placing-puzzle could not connect an isolated source cell");
                continue;
            }
            Referable<rtl::Conn>* input = freeDataInput(*sink);
            require(input, "placing-puzzle selected a full sink cell");
            connectToInput(*driver_inst, *input);
            ++mesh_degree[driver_inst];
            ++mesh_degree[sink];
            ++stats.data_connections;
            if (driver_inst->coord.x != sink->coord.x) {
                ++stats.horizontal_connections;
            }
            if (driver_inst->coord.y != sink->coord.y) {
                ++stats.vertical_connections;
            }
        }
    }

    void emitPending(int x, int y, const std::vector<PuzzleCell*>& tile_cells)
    {
        bool can_right = x + 1 < parameters.size;
        bool can_bottom = y + 1 < parameters.size;
        for (size_t index = 0; index < tile_cells.size(); ++index) {
            rtl::Inst* driver = tile_cells[index]->inst;
            bool go_right = can_right;
            bool go_bottom = can_bottom;
            if (can_right && can_bottom) {
                uint64_t choice = random()%100;
                go_right = choice < 80;
                go_bottom = choice >= 20;
                if (tile_cells.size() == 1) {
                    go_right = true;
                    go_bottom = true;
                }
                if (index == 0) go_right = true;
                if (index == 1) go_bottom = true;
            }
            if (go_right) {
                pending[static_cast<size_t>(y*parameters.size + x + 1)]
                    .push_back(driver);
            }
            if (go_bottom) {
                pending[static_cast<size_t>((y + 1)*parameters.size + x)]
                    .push_back(driver);
            }
            if (go_right && go_bottom) {
                ++stats.dual_direction_cells;
            }
        }
    }

    void generateMesh()
    {
        size_t total_cells = requestedCoreCells();
        size_t tile_count = static_cast<size_t>(parameters.size*parameters.size);
        int base_cells = static_cast<int>(total_cells/tile_count);
        int extra_cells = static_cast<int>(total_cells%tile_count);
        int minimum_luts = std::max<int>(
            0, static_cast<int>(total_cells)
                - static_cast<int>(tile_count)*kRegistersPerTile);
        int maximum_luts = std::min<int>(
            static_cast<int>(total_cells),
            static_cast<int>(tile_count)*kLutsPerTile);
        int remaining_luts = std::clamp(
            static_cast<int>(std::lround(total_cells/3.0)),
            minimum_luts, maximum_luts);
        int global_cell_index = 0;

        for (int y = 0; y < parameters.size; ++y) {
            for (int x = 0; x < parameters.size; ++x) {
                int tile_index = y*parameters.size + x;
                int cell_count = base_cells + (tile_index < extra_cells ? 1 : 0);
                require(cell_count >= 1 && cell_count <= kCellsPerTile,
                        "placing-puzzle fullness produced an invalid Tile population");
                int remaining_tiles = parameters.size*parameters.size - tile_index;
                std::vector<CellKind> kinds = tileKinds(
                    cell_count, tile_index, remaining_luts, remaining_tiles);
                std::vector<int> lut_bits(kLutsPerTile);
                std::vector<int> register_bits(kRegistersPerTile);
                for (int bit = 0; bit < kLutsPerTile; ++bit) lut_bits[bit] = bit;
                for (int bit = 0; bit < kRegistersPerTile; ++bit) register_bits[bit] = bit;
                std::shuffle(lut_bits.begin(), lut_bits.end(), random);
                std::shuffle(register_bits.begin(), register_bits.end(), random);

                fpga::Tile& tile = fpga::Device::current().tile_grid[
                    static_cast<size_t>(tile_index)];
                std::vector<PuzzleCell*> tile_cells;
                tile_cells.reserve(static_cast<size_t>(cell_count));
                for (CellKind kind : kinds) {
                    PuzzleCell cell = makeCell(
                        std::format("mesh_cell_{}", global_cell_index++), kind);
                    placeGeneratedCell(cell, tile, lut_bits, register_bits);
                    placement_history[cell.inst].generated = cell.generated_coord;
                    stats.luts += cell.isLut();
                    stats.registers += cell.isRegister();
                    core_cells.push_back(cell);
                    tile_cells.push_back(&core_cells.back());
                }
                require(tile_cells.size() == static_cast<size_t>(cell_count),
                        "placing-puzzle lost generated Tile cells");
                consumePending(tile_index, tile_cells);
                emitPending(x, y, tile_cells);
            }
        }
        stats.core_cells = core_cells.size();
        require(stats.core_cells == total_cells,
                "placing-puzzle generated the wrong overall fullness");
        require(remaining_luts == 0,
                "placing-puzzle failed to distribute its LUT population");
        for (const fpga::Tile& tile : fpga::Device::current().tile_grid) {
            require(tile.regs_cnt + tile.luts6cnt >= 1,
                    "placing-puzzle left a generated Tile empty");
        }
        validateGeneratedMesh();
    }

    void validateGeneratedMesh()
    {
        require(stats.horizontal_connections > 0
                    && stats.vertical_connections > 0,
                "placing-puzzle mesh did not use both forward directions");
        for (const PuzzleCell& cell : core_cells) {
            bool has_neighbor = false;
            for (auto& conn : cell.inst->conns) {
                if (!conn.port_ref.peer) continue;
                if (conn.port_ref->type == rtl::Port::PORT_IN) {
                    if (tech.check_clocked(
                            cell.inst->cell_ref->type, conn.port_ref->name)) {
                        continue;
                    }
                    rtl::Conn* driver = conn.follow();
                    rtl::Inst* peer = driver ? driver->inst_ref.peer : nullptr;
                    if (!peer || (peer->cell_ref->type != "FD"
                                  && peer->cell_ref->type != "LUT6")) {
                        continue;
                    }
                    int distance = std::abs(peer->coord.x - cell.inst->coord.x)
                        + std::abs(peer->coord.y - cell.inst->coord.y);
                    require(distance == 1,
                            "placing-puzzle generated a non-neighbor data edge");
                    has_neighbor = true;
                }
                else if (conn.port_ref->type == rtl::Port::PORT_OUT) {
                    for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
                        rtl::Conn* sink = sink_ref
                            ? rtl::Conn::fromBase(sink_ref) : nullptr;
                        rtl::Inst* peer = sink ? sink->inst_ref.peer : nullptr;
                        if (!peer || (peer->cell_ref->type != "FD"
                                      && peer->cell_ref->type != "LUT6")) {
                            continue;
                        }
                        int distance = std::abs(
                            peer->coord.x - cell.inst->coord.x)
                            + std::abs(peer->coord.y - cell.inst->coord.y);
                        require(distance == 1,
                                "placing-puzzle generated a non-neighbor fanout");
                        has_neighbor = true;
                    }
                }
            }
            require(has_neighbor,
                    "placing-puzzle generated a core cell without a mesh neighbor");
        }
    }

    void addTopOutput(const std::string& name, rtl::Inst& output_buffer)
    {
        rtl::Port port;
        port.name = name;
        port.type = rtl::Port::PORT_OUT;
        port.index = static_cast<int>(tech.design.top_cell.ports.size());
        port.is_global = true;
        port.designator = next_designator++;
        tech.design.top_cell.ports.emplace_back(std::move(port));
        auto& top_connection = tech.design.top.conns.emplace_back();
        top_connection.port_ref.set(&tech.design.top_cell.ports.back());
        top_connection.inst_ref.set(&tech.design.top);
        Referable<rtl::Conn>* output = connection(output_buffer, "O");
        require(output, "placing-puzzle output buffer has no output");
        output->port_ref->designator = top_connection.port_ref->designator;
        top_connection.set(output);
        auto& net = top_module.nets.emplace_back();
        net.name = "top_output_" + name;
        net.designators.push_back(top_connection.port_ref->designator);
    }

    void addTerminalOutputs()
    {
        // The bottom-right cells are the only cells without a future neighbor.
        fpga::Coord terminal_coord{parameters.size - 1, parameters.size - 1};
        for (PuzzleCell& cell : core_cells) {
            if (cell.generated_coord.x != terminal_coord.x
                || cell.generated_coord.y != terminal_coord.y) {
                continue;
            }
            PuzzleCell buffer = makeCell(
                std::format("mesh_output_{}", output_buffers.size()),
                CellKind::output_buffer);
            Referable<rtl::Conn>* input = connection(*buffer.inst, "I");
            require(input, "placing-puzzle output buffer has no input");
            connectToInput(*cell.inst, *input, "output_net_");
            ++stats.data_connections;
            addTopOutput(std::format("output_{}", output_buffers.size()),
                         *buffer.inst);
            output_buffers.push_back(buffer);
        }
        require(!output_buffers.empty(),
                "placing-puzzle generated no terminal output roots");
    }

    void rebuildTimings()
    {
        tech.timings.clocked_inputs.clear();
        tech.timings.makeTimingsList(tech.design, tech.clocks);
        tech.timings.calculateTimings();
    }

    double worstIntrinsicSetup() const
    {
        double worst = 0;
        for (const auto& [clock, infos] : tech.timings.clocked_inputs) {
            (void)clock;
            for (const auto& info : infos) {
                worst = std::max(worst, info.path.max_setup_time);
            }
        }
        return worst;
    }

    fpga::Tile* placeInsertedRegister(rtl::Inst& reg,
                                      fpga::Coord preferred)
    {
        fpga::Device& device = fpga::Device::current();
        int max_radius = parameters.size*2;
        for (int radius = 0; radius <= max_radius; ++radius) {
            for (int dy = -radius; dy <= radius; ++dy) {
                int dx = radius - std::abs(dy);
                for (int sign : {-1, 1}) {
                    if (dx == 0 && sign == 1) continue;
                    fpga::Tile* tile = device.getTile(
                        preferred.x + sign*dx, preferred.y + dy);
                    if (!tile) continue;
                    if (tile->tryAdd(&reg, false) >= 0) {
                        float divisor = static_cast<float>(
                            std::max(1, parameters.size - 1));
                        reg.outline.x = tile->coord.x*9.95F/divisor;
                        reg.outline.y = tile->coord.y*9.95F/divisor;
                        return tile;
                    }
                }
            }
        }
        return nullptr;
    }

    bool insertTimingRegister(const pnr::PlaceTimingEdge& edge)
    {
        if (!edge.sink_input || !edge.driver_output || !edge.sink
            || !edge.driver || edge.sink_input->follow() != edge.driver_output) {
            return false;
        }
        PuzzleCell inserted = makeCell(
            std::format("timing_repair_reg_{}", stats.inserted_registers),
            CellKind::reg);
        connectClock(*inserted.inst);
        fpga::Tile* tile = placeInsertedRegister(
            *inserted.inst, edge.sink->coord);
        require(tile,
                "placing-puzzle timing repair exhausted register capacity");
        inserted.generated_coord = tile->coord;

        Referable<rtl::Conn>* old_sink = rtl::Conn::fromBase(edge.sink_input);
        old_sink->clear();
        Referable<rtl::Conn>* data = connection(*inserted.inst, "D");
        require(data, "placing-puzzle repair register has no data input");
        connectToInput(*edge.driver, *data, "timing_repair_in_");
        connectToInput(*inserted.inst, *old_sink, "timing_repair_out_");
        core_cells.push_back(inserted);
        ++stats.registers;
        ++stats.core_cells;
        ++stats.inserted_registers;
        stats.data_connections += 2;
        return true;
    }

    pnr::PlaceTimingAnalysis closeGeneratedTiming()
    {
        constexpr int max_repair_passes = 8;
        pnr::PlaceTiming estimator;
        estimator.tech = &tech;
        for (int pass = 0; pass <= max_repair_passes; ++pass) {
            rebuildTimings();
            pnr::PlaceTimingAnalysis analysis = estimator.analyze(tech.timings);
            std::cout << "PLACING_PUZZLE_TIMING phase=generated pass=" << pass
                      << " endpoints=" << analysis.endpoints
                      << " violations=" << analysis.violated_endpoints
                      << " worst_slack_ns=" << analysis.worst_slack_ns
                      << " intrinsic_setup_ns=" << worstIntrinsicSetup()
                      << " inserted_registers=" << stats.inserted_registers
                      << '\n';
            if (analysis.violated_endpoints == 0) {
                return analysis;
            }
            require(pass < max_repair_passes,
                    "placing-puzzle could not close its generated baseline timing");
            std::unordered_set<rtl::Conn*> repaired_inputs;
            size_t inserted = 0;
            for (const pnr::PlaceTimingEndpoint& endpoint
                    : analysis.endpoint_details) {
                if (endpoint.slack_ns >= 0) continue;
                for (const pnr::PlaceTimingEdge& edge
                        : endpoint.critical_edges) {
                    if (!edge.sink_input
                        || !repaired_inputs.insert(edge.sink_input).second) {
                        continue;
                    }
                    if (insertTimingRegister(edge)) {
                        ++inserted;
                        break;
                    }
                }
            }
            require(inserted > 0,
                    "placing-puzzle found timing violations without a repairable edge");
        }
        return {};
    }

    void clearPlacement()
    {
        for (PuzzleCell& cell : core_cells) {
            if (cell.inst->tile.peer) {
                bool removed = cell.inst->tile->unassign(cell.inst);
                require(removed,
                        "placing-puzzle could not clear generated placement");
            }
            cell.inst->coord = {-1, -1};
            cell.inst->pos = -1;
            cell.inst->outline = rtl::OutlineInfo{
                .x = -1, .y = -1, .fixed = false};
        }
        for (const PuzzleCell& cell : core_cells) {
            require(!cell.inst->tile.peer,
                    "placing-puzzle left a core cell placed before the test");
        }
    }

    void captureOutlineTargets()
    {
        float aspect_x = static_cast<float>(parameters.size)
            / pnr::PlaceDesign::mesh_width;
        float aspect_y = static_cast<float>(parameters.size)
            / pnr::PlaceDesign::mesh_height;
        for (const PuzzleCell& cell : core_cells) {
            CellPlacementHistory& history = placement_history[cell.inst];
            history.outline_x = cell.inst->outline.x;
            history.outline_y = cell.inst->outline.y;
            history.outline_target = {
                std::clamp(static_cast<int>(history.outline_x*aspect_x),
                           0, parameters.size - 1),
                std::clamp(static_cast<int>(history.outline_y*aspect_y),
                           0, parameters.size - 1),
            };
        }
    }

    void printEstimateStats() const
    {
        struct Population { size_t registers = 0; size_t combs = 0; };
        std::unordered_map<pnr::RegBunch*, Population> populations;
        size_t cells_without_bunch = 0;
        for (const PuzzleCell& cell : core_cells) {
            pnr::RegBunch* bunch = cell.inst->bunch_ref.peer;
            if (!bunch) {
                ++cells_without_bunch;
                continue;
            }
            Population& population = populations[bunch];
            population.registers += cell.isRegister();
            population.combs += cell.isLut();
        }
        size_t register_only = 0;
        size_t comb_chains = 0;
        size_t maximum_cells = 0;
        for (const auto& [bunch, population] : populations) {
            (void)bunch;
            register_only += population.combs == 0;
            comb_chains += population.combs != 0;
            maximum_cells = std::max(
                maximum_cells, population.registers + population.combs);
        }
        std::cout << "PLACING_PUZZLE_ESTIMATE roots="
                  << tech.estimate.data_outs.size()
                  << " bunches=" << populations.size()
                  << " comb_bunches=" << comb_chains
                  << " register_only_bunches=" << register_only
                  << " maximum_bunch_cells=" << maximum_cells
                  << " cells_without_bunch=" << cells_without_bunch << '\n';
    }

    void capturePlacedCells()
    {
        for (const PuzzleCell& cell : core_cells) {
            CellPlacementHistory& history = placement_history[cell.inst];
            history.after_place = cell.inst->coord;
            history.after_place_pos = cell.inst->pos;
        }
    }

    static int distance(fpga::Coord left, fpga::Coord right)
    {
        if (left.x < 0 || left.y < 0 || right.x < 0 || right.y < 0) {
            return -1;
        }
        return std::abs(left.x - right.x) + std::abs(left.y - right.y);
    }

    const CellPlacementHistory* history(const rtl::Inst* inst) const
    {
        auto found = placement_history.find(const_cast<rtl::Inst*>(inst));
        return found == placement_history.end() ? nullptr : &found->second;
    }

    void printViolationHistory(const pnr::PlaceTimingAnalysis& analysis,
                               size_t sample_count) const
    {
        std::vector<const pnr::PlaceTimingEndpoint*> violations;
        for (const pnr::PlaceTimingEndpoint& endpoint
                : analysis.endpoint_details) {
            if (endpoint.slack_ns < 0) {
                violations.push_back(&endpoint);
            }
        }
        std::ranges::sort(violations, {},
            &pnr::PlaceTimingEndpoint::slack_ns);

        size_t measured_edges = 0;
        uint64_t generated_distance = 0;
        uint64_t outline_distance = 0;
        uint64_t placed_distance = 0;
        uint64_t final_distance = 0;
        uint64_t outline_to_placed_distance = 0;
        size_t outline_dilated_edges = 0;
        size_t placement_displaced_cells = 0;
        std::unordered_map<int, size_t> outline_target_population;
        for (const auto& [inst, history] : placement_history) {
            (void)inst;
            if (history.outline_target.x < 0 || history.outline_target.y < 0) {
                continue;
            }
            int target = history.outline_target.y*parameters.size
                + history.outline_target.x;
            ++outline_target_population[target];
        }
        size_t overloaded_targets = 0;
        size_t overflow_cells = 0;
        size_t maximum_target_population = 0;
        for (const auto& [target, population] : outline_target_population) {
            (void)target;
            maximum_target_population = std::max(
                maximum_target_population, population);
            if (population > kCellsPerTile) {
                ++overloaded_targets;
                overflow_cells += population - kCellsPerTile;
            }
        }
        for (const pnr::PlaceTimingEndpoint* endpoint : violations) {
            for (const pnr::PlaceTimingEdge& edge : endpoint->critical_edges) {
                const CellPlacementHistory* driver = history(edge.driver);
                const CellPlacementHistory* sink = history(edge.sink);
                if (!driver || !sink) continue;
                int generated = distance(driver->generated, sink->generated);
                int outlined = distance(
                    driver->outline_target, sink->outline_target);
                int placed = distance(driver->after_place, sink->after_place);
                int final = distance(edge.driver->coord, edge.sink->coord);
                if (generated < 0 || outlined < 0 || placed < 0 || final < 0) {
                    continue;
                }
                ++measured_edges;
                generated_distance += static_cast<uint64_t>(generated);
                outline_distance += static_cast<uint64_t>(outlined);
                placed_distance += static_cast<uint64_t>(placed);
                final_distance += static_cast<uint64_t>(final);
                outline_dilated_edges += outlined > generated;
                int driver_displacement = distance(
                    driver->outline_target, driver->after_place);
                int sink_displacement = distance(
                    sink->outline_target, sink->after_place);
                outline_to_placed_distance += static_cast<uint64_t>(
                    std::max(0, driver_displacement)
                    + std::max(0, sink_displacement));
                placement_displaced_cells += driver_displacement > 0;
                placement_displaced_cells += sink_displacement > 0;
            }
        }
        if (measured_edges != 0) {
            double divisor = static_cast<double>(measured_edges);
            std::cout
                << "PLACING_PUZZLE_VIOLATION_SUMMARY endpoints="
                << violations.size()
                << " critical_edges=" << measured_edges
                << " average_generated_distance="
                << generated_distance/divisor
                << " average_outline_distance=" << outline_distance/divisor
                << " average_after_place_distance=" << placed_distance/divisor
                << " average_final_distance=" << final_distance/divisor
                << " average_outline_to_place_displacement_per_edge="
                << outline_to_placed_distance/(2.0*divisor)
                << " outline_target_tiles="
                << outline_target_population.size()
                << " overloaded_outline_targets=" << overloaded_targets
                << " outline_overflow_cells=" << overflow_cells
                << " maximum_outline_target_population="
                << maximum_target_population
                << " outline_dilated_edges=" << outline_dilated_edges
                << " placed_endpoints_off_outline="
                << placement_displaced_cells << '\n';
            if (overflow_cells != 0
                && outline_to_placed_distance > 4*measured_edges) {
                std::cout
                    << "PLACING_PUZZLE_DIAGNOSIS cause="
                       "outline_density_collapse_then_capacity_spill "
                       "detail='connected cells received compact Outline targets; "
                       "overloaded target regions forced sparse placement to send "
                       "otherwise adjacent cells to unrelated distant Tiles'\n";
            }
        }

        sample_count = std::min(sample_count, violations.size());
        for (size_t sample = 0; sample < sample_count; ++sample) {
            const pnr::PlaceTimingEndpoint& endpoint = *violations[sample];
            std::cout << "PLACING_PUZZLE_VIOLATION sample=" << sample
                      << " sink='"
                      << (endpoint.data_in && endpoint.data_in->inst_ref.peer
                            ? endpoint.data_in->inst_ref->makeName() : "-")
                      << "' slack_ns=" << endpoint.slack_ns
                      << " arrival_ns=" << endpoint.arrival_ns
                      << " required_ns=" << endpoint.required_ns
                      << " critical_edges=" << endpoint.critical_edges.size()
                      << '\n';
            for (size_t index = 0; index < endpoint.critical_edges.size();
                 ++index) {
                const pnr::PlaceTimingEdge& edge = endpoint.critical_edges[index];
                const CellPlacementHistory* driver = history(edge.driver);
                const CellPlacementHistory* sink = history(edge.sink);
                if (!driver || !sink) continue;
                std::cout << "PLACING_PUZZLE_CELL_HISTORY sample=" << sample
                          << " edge=" << index
                          << " driver='" << edge.driver->makeName()
                          << "' sink='" << edge.sink->makeName() << "'"
                          << " driver_generated=(" << driver->generated.x << ','
                          << driver->generated.y << ')'
                          << " sink_generated=(" << sink->generated.x << ','
                          << sink->generated.y << ')'
                          << " driver_outline=(" << driver->outline_x << ','
                          << driver->outline_y << ")->("
                          << driver->outline_target.x << ','
                          << driver->outline_target.y << ')'
                          << " sink_outline=(" << sink->outline_x << ','
                          << sink->outline_y << ")->("
                          << sink->outline_target.x << ','
                          << sink->outline_target.y << ')'
                          << " driver_after_place=(" << driver->after_place.x
                          << ',' << driver->after_place.y << ")@"
                          << driver->after_place_pos
                          << " sink_after_place=(" << sink->after_place.x << ','
                          << sink->after_place.y << ")@"
                          << sink->after_place_pos
                          << " driver_final=(" << edge.driver->coord.x << ','
                          << edge.driver->coord.y << ')'
                          << " sink_final=(" << edge.sink->coord.x << ','
                          << edge.sink->coord.y << ')'
                          << " distances="
                          << distance(driver->generated, sink->generated) << '/'
                          << distance(driver->outline_target,
                                      sink->outline_target) << '/'
                          << distance(driver->after_place,
                                      sink->after_place) << '/'
                          << distance(edge.driver->coord, edge.sink->coord)
                          << " wire_delay_ns=" << edge.wire_delay_ns << '\n';
            }
        }
    }

    void checkFinalPlacement() const
    {
        size_t placed = 0;
        for (const PuzzleCell& cell : core_cells) {
            require(cell.inst->tile.peer,
                    "placing-puzzle finished with an unplaced core cell");
            require(cell.inst->coord.x == cell.inst->tile->coord.x
                        && cell.inst->coord.y == cell.inst->tile->coord.y,
                    "placing-puzzle cell coordinate disagrees with its Tile");
            require(cell.inst->pos >= 0,
                    "placing-puzzle cell has no final element position");
            ++placed;
        }
        require(placed == stats.core_cells,
                "placing-puzzle final placement count is inconsistent");
    }
};

void runPuzzle(PuzzleParameters parameters)
{
    auto generated_started = std::chrono::steady_clock::now();
    PlacementPuzzle puzzle(parameters);
    pnr::PlaceTimingAnalysis generated = puzzle.closeGeneratedTiming();
    require(generated.endpoints > 0,
            "placing-puzzle generated no timed register endpoints");
    require(generated.violated_endpoints == 0,
            "placing-puzzle baseline is not timing-clean");
    double generated_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - generated_started).count();

    size_t capacity = static_cast<size_t>(parameters.size*parameters.size)
        * kCellsPerTile;
    double actual_fullness = 100.0*puzzle.stats.core_cells/capacity;
    std::cout << "PLACING_PUZZLE_GENERATED size=" << parameters.size << 'x'
              << parameters.size
              << " requested_fullness=" << parameters.fullness_percent
              << " actual_fullness=" << actual_fullness
              << " cells=" << puzzle.stats.core_cells
              << " luts=" << puzzle.stats.luts
              << " registers=" << puzzle.stats.registers
              << " combs_per_register="
              << static_cast<double>(puzzle.stats.luts)/puzzle.stats.registers
              << " connections=" << puzzle.stats.data_connections
              << " horizontal=" << puzzle.stats.horizontal_connections
              << " vertical=" << puzzle.stats.vertical_connections
              << " dual_direction_cells=" << puzzle.stats.dual_direction_cells
              << " elapsed_s=" << generated_seconds << '\n';

    puzzle.clearPlacement();

    auto estimate_started = std::chrono::steady_clock::now();
    puzzle.tech.estimate.estimateDesign(puzzle.tech.design);
    double estimate_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - estimate_started).count();
    require(!puzzle.tech.estimate.data_outs.empty(),
            "Estimate produced no placing bunches");
    puzzle.printEstimateStats();
    std::cout << "PLACING_PUZZLE_STAGE stage=Estimate elapsed_s="
              << estimate_seconds << " roots="
              << puzzle.tech.estimate.data_outs.size() << '\n';

    auto outline_started = std::chrono::steady_clock::now();
    puzzle.tech.outline.placeIOBs(
        puzzle.tech.estimate.data_outs, puzzle.tech.assignments);
    puzzle.tech.outline.placeInstIOBs(
        puzzle.tech.design.top, puzzle.tech.assignments);
    puzzle.tech.outline.optimizeOutline(puzzle.tech.estimate.data_outs);
    puzzle.captureOutlineTargets();
    double outline_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - outline_started).count();
    std::cout << "\nPLACING_PUZZLE_STAGE stage=Outline elapsed_s="
              << outline_seconds << '\n';

    auto place_started = std::chrono::steady_clock::now();
    puzzle.tech.place.placeDesign(puzzle.tech.estimate.data_outs);
    double place_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - place_started).count();
    puzzle.checkFinalPlacement();
    puzzle.capturePlacedCells();
    std::cout << "PLACING_PUZZLE_STAGE stage=PlaceDesign elapsed_s="
              << place_seconds
              << " commits=" << puzzle.tech.place.place_commits
              << " tile_trials=" << puzzle.tech.place.place_tile_trials
              << '\n';

    pnr::PlaceTiming final_timing;
    final_timing.tech = &puzzle.tech;
    auto timing_started = std::chrono::steady_clock::now();
    puzzle.tech.timings.calculateTimings();
    pnr::PlaceTimingAnalysis final = final_timing.analyze(puzzle.tech.timings);
    if (final.violated_endpoints != 0) {
        size_t anchors_per_pass = std::clamp<size_t>(
            puzzle.stats.core_cells/8, 256, 8192);
        puzzle.tech.place.timing_refinement = puzzle.tech.place.refineTiming(
            puzzle.tech.timings, 100, anchors_per_pass);
        final = final_timing.analyze(puzzle.tech.timings);
    }
    double timing_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - timing_started).count();
    std::cout << "PLACING_PUZZLE_STAGE stage=PlaceTiming elapsed_s="
              << timing_seconds
              << " endpoints=" << final.endpoints
              << " violations=" << final.violated_endpoints
              << " worst_slack_ns=" << final.worst_slack_ns
              << " tns_ns=" << final.total_negative_slack_ns
              << " refinement_passes="
              << puzzle.tech.place.timing_refinement.passes
              << '\n';
    require(final.endpoints == generated.endpoints,
            "PlaceTiming lost endpoints after replacement");
    require(final.unplaced_edges == 0,
            "PlaceTiming found unplaced timing edges");
    if (final.violated_endpoints != 0) {
        puzzle.printViolationHistory(final, 5);
    }
    require(final.violated_endpoints == 0,
            "re-placed design does not meet its 1 ns clock");
    std::cout << "placing_puzzle_test passed\n";
}

}

int main(int argc, char** argv)
{
    try {
        require(argc == 3,
                "usage: placing_puzzle_test <tile-space-N> <fullness-percent>");
        PuzzleParameters parameters{
            .size = parsePositive(argv[1], "Tile-space size"),
            .fullness_percent = parsePositive(argv[2], "fullness"),
        };
        runPuzzle(parameters);
    }
    catch (const TestFailure& failure) {
        std::cerr << "placing_puzzle_test: " << failure.message << '\n';
        return 1;
    }
    return 0;
}
