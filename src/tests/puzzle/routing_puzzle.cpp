#include "RouteDesign.h"
#include "TimingPath.h"

#include "Cell.h"
#include "Conn.h"
#include "Device.h"
#include "Element.h"
#include "Module.h"
#include "Wire.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <utility>
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

NodeMask bit(int index)
{
    return NodeMask{0, 1} << index;
}

size_t countBits(const NodeMask& mask)
{
    size_t count = 0;
    mask.for_each_set_bit([&](int) {
        ++count;
        return false;
    });
    return count;
}

bool sameCoord(fpga::Coord left, fpga::Coord right)
{
    return left.x == right.x && left.y == right.y;
}

struct PuzzleParameters
{
    size_t design_cells = 0;
    fpga::Coord tile_space;
    int fullness_percent = 0;
    int fanout_merge_percent = 10;
    size_t fanout_tree_count = 1;

    int width() const { return tile_space.x; }
    int height() const { return tile_space.y; }
};

// This topology is deliberately fixed. New puzzle cases vary only the public
// scale, occupancy, and fanout dimensions, never per-tile routing structure.
constexpr int kLutsPerTile = 8;
constexpr int kRegistersPerTile = 8;
constexpr int kTracksPerDirection = 8;
constexpr int kDirectionCount = 8;
constexpr int kJumpLengthCount = 3;
constexpr int kSrcCount =
    kTracksPerDirection * kDirectionCount * kJumpLengthCount;
constexpr int kDstCount = kSrcCount;
constexpr int kLutOutputBase = 0;
constexpr int kLutInputBase = 8;
constexpr int kRegisterOutputBase = 16;
constexpr int kRegisterInputBase = 24;

constexpr uint64_t kPuzzleSeed = 0x5ca1e1234ULL;

const std::array<fpga::Coord, kDirectionCount> kUnitDirectionDeltas{{
    {0, -1},
    {1, -1},
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    {-1, 0},
    {-1, -1},
}};

const std::array<int, kJumpLengthCount> kJumpLengths{{1, 2, 4}};

int sourceNode(int direction, int length_index, int track)
{
    return ((length_index * kDirectionCount + direction)
        * kTracksPerDirection) + track;
}

int jumpLengthIndex(int length)
{
    for (int index = 0; index < kJumpLengthCount; ++index) {
        if (kJumpLengths[index] == length) {
            return index;
        }
    }
    return -1;
}

int directionForDelta(fpga::Coord delta)
{
    int length = std::max(std::abs(delta.x), std::abs(delta.y));
    if (jumpLengthIndex(length) < 0
        || (delta.x != 0 && std::abs(delta.x) != length)
        || (delta.y != 0 && std::abs(delta.y) != length)) {
        return -1;
    }
    fpga::Coord unit{
        delta.x == 0 ? 0 : (delta.x > 0 ? 1 : -1),
        delta.y == 0 ? 0 : (delta.y > 0 ? 1 : -1),
    };
    for (int direction = 0; direction < kDirectionCount; ++direction) {
        if (sameCoord(kUnitDirectionDeltas[direction], unit)) {
            return direction;
        }
    }
    return -1;
}

int transitJoint(int dst, int src)
{
    (void)dst;
    int track = src % kTracksPerDirection;
    if (track < kTracksPerDirection / 2) {
        return -1;
    }
    return src;
}

void rememberConnection(fpga::CBType& cb, fpga::CBNodeNameType from_type,
                        int from, fpga::CBNodeNameType to_type, int to)
{
    const std::string* from_name = cb.nodeName(from_type, from);
    const std::string* to_name = cb.nodeName(to_type, to);
    require(from_name && to_name,
        "routing-puzzle topology tried to connect an unnamed node");
    cb.rememberConnName(from_type, from, to_type, to, *from_name, *to_name);
}

void rememberJump(fpga::CBType& cb, int src, int dst, fpga::Coord delta)
{
    fpga::CBJumpState dsts{};
    dsts.jump = bit(dst);
    fpga::CBType::ResolvedJump jump{};
    jump.delta = delta;
    jump.target_cb_type_id = cb.type_id;
    jump.dsts = dsts;
    cb.dst_by_src[src].push_back(jump);
}

fpga::CBType makePuzzleCrossbar()
{
    fpga::CBType cb{};
    cb.name = "PUZZLE_MESH";
    cb.type_id = 0;

    for (int local = 0; local < 32; ++local) {
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, local,
            std::format("LOCAL_{}", local));
    }

    for (int length_index = 0; length_index < kJumpLengthCount;
         ++length_index) {
        int length = kJumpLengths[length_index];
        for (int direction = 0; direction < kDirectionCount; ++direction) {
            fpga::Coord unit = kUnitDirectionDeltas[direction];
            fpga::Coord delta{unit.x * length, unit.y * length};
            for (int track = 0; track < kTracksPerDirection; ++track) {
                int src = sourceNode(direction, length_index, track);
                int dst = src;
                cb.rememberNodeName(fpga::CB_NODE_SRC, src,
                    std::format("SRC_D{}_L{}_T{}", direction, length, track));
                cb.rememberNodeName(fpga::CB_NODE_DST, dst,
                    std::format("DST_D{}_L{}_T{}", direction, length, track));
                rememberJump(cb, src, dst, delta);
                rememberConnection(cb, fpga::CB_NODE_SRC, src,
                    fpga::CB_NODE_DST, dst);
            }
        }
    }

    NodeMask all_sources{};
    NodeMask all_destinations{};
    for (int node = 0; node < kSrcCount; ++node) {
        all_sources |= bit(node);
        all_destinations |= bit(node);
    }

    for (int local = kLutOutputBase;
         local < kLutOutputBase + kLutsPerTile; ++local) {
        cb.local_src[local].jump = all_sources;
        for (int src = 0; src < kSrcCount; ++src) {
            rememberConnection(cb, fpga::CB_NODE_LOCAL, local,
                fpga::CB_NODE_SRC, src);
        }
    }
    for (int local = kRegisterOutputBase;
         local < kRegisterOutputBase + kRegistersPerTile; ++local) {
        cb.local_src[local].jump = all_sources;
        for (int src = 0; src < kSrcCount; ++src) {
            rememberConnection(cb, fpga::CB_NODE_LOCAL, local,
                fpga::CB_NODE_SRC, src);
        }
    }

    for (int dst = 0; dst < kDstCount; ++dst) {
        for (int src = 0; src < kSrcCount; ++src) {
            int joint = transitJoint(dst, src);
            if (joint < 0) {
                cb.dst_src[dst].jump |= bit(src);
                rememberConnection(cb, fpga::CB_NODE_DST, dst,
                    fpga::CB_NODE_SRC, src);
                continue;
            }
            if (!cb.nodeName(fpga::CB_NODE_JOINT, joint)) {
                cb.rememberNodeName(fpga::CB_NODE_JOINT, joint,
                    std::format("JOINT_{}", joint));
            }
            cb.dst_joint[dst].joint |= bit(joint);
            cb.joint_src[joint].jump |= bit(src);
            rememberConnection(cb, fpga::CB_NODE_DST, dst,
                fpga::CB_NODE_JOINT, joint);
            rememberConnection(cb, fpga::CB_NODE_JOINT, joint,
                fpga::CB_NODE_SRC, src);
        }

        for (int local = kLutInputBase;
             local < kLutInputBase + kLutsPerTile; ++local) {
            cb.dst_local[dst].local |= bit(local);
            rememberConnection(cb, fpga::CB_NODE_DST, dst,
                fpga::CB_NODE_LOCAL, local);
        }
        for (int local = kRegisterInputBase;
             local < kRegisterInputBase + kRegistersPerTile; ++local) {
            cb.dst_local[dst].local |= bit(local);
            rememberConnection(cb, fpga::CB_NODE_DST, dst,
                fpga::CB_NODE_LOCAL, local);
        }
    }

    cb.local_output_nodes = {};
    cb.local_input_nodes = {};
    for (int lane = 0; lane < kLutsPerTile; ++lane) {
        cb.local_output_nodes |= bit(kLutOutputBase + lane);
        cb.local_input_nodes |= bit(kLutInputBase + lane);
    }
    for (int lane = 0; lane < kRegistersPerTile; ++lane) {
        cb.local_output_nodes |= bit(kRegisterOutputBase + lane);
        cb.local_input_nodes |= bit(kRegisterInputBase + lane);
    }
    cb.valid_dst_nodes = all_destinations;
    cb.rebuildOutgoingSrcs();
    cb.ensureDerivedMasks();
    return cb;
}

fpga::Element makeElement(const std::string& name, fpga::ElementType type,
                          int bitmap_pos)
{
    fpga::Element element{};
    element.name = name;
    element.type = type;
    element.bitmap_pos = static_cast<uint16_t>(bitmap_pos);
    element.elements_to_left = static_cast<int>(type);
    return element;
}

void rememberPinEndpoint(fpga::TileType& type, fpga::TilePinNameType direction,
                         int resource, int local, const std::string& pin)
{
    type.pin_map.rememberResourcePinName(direction, resource, pin);
    if (direction == fpga::TILE_PIN_INPUT) {
        type.pin_map.input_nodes[resource] = bit(local);
    } else {
        type.pin_map.output_nodes[resource] = bit(local);
    }
    type.pin_map.rememberLocalNames(direction, local,
        std::format("LOCAL_{}", local), std::format("RESOURCE_{}", resource),
        pin);
    type.pin_map.rememberEndpointNames(direction, resource, local,
        std::format("LOCAL_{}", local), std::format("RESOURCE_{}", resource),
        pin);
    type.pin_map.rememberEndpointRouteRef(direction, resource, local,
        "PUZZLE_MESH");
}

fpga::TileType makePuzzleTileType()
{
    fpga::TileType type{};
    type.name = "PUZZLE_LOGIC";
    type.num = 0;
    type.type = fpga::Tile::TILE_LUTS;

    for (int site = 0; site < 2; ++site) {
        type.sites.push_back(fpga::SiteModel{
            .name = std::format("LOGIC_SITE_{}", site),
            .type = "LOGIC_SITE",
            .pos = site,
        });
    }

    static constexpr std::array<const char*, 4> prefixes{{"A", "B", "C", "D"}};
    static constexpr std::array<int, 4> lut_output_resources{{16, 80, 144, 212}};
    static constexpr std::array<int, 4> lut_input_resources{{17, 81, 145, 213}};
    static constexpr std::array<int, 4> register_output_resources{{1, 65, 129, 197}};
    static constexpr std::array<int, 4> register_input_resources{{31, 95, 130, 198}};

    for (int site = 0; site < 2; ++site) {
        for (int bel = 0; bel < 4; ++bel) {
            int lane = site * 4 + bel;
            int resource_offset = site * 256;
            rememberPinEndpoint(type, fpga::TILE_PIN_OUTPUT,
                lut_output_resources[bel] + resource_offset,
                kLutOutputBase + lane, prefixes[bel]);
            rememberPinEndpoint(type, fpga::TILE_PIN_INPUT,
                lut_input_resources[bel] + resource_offset,
                kLutInputBase + lane, std::string(prefixes[bel]) + "1");
            rememberPinEndpoint(type, fpga::TILE_PIN_OUTPUT,
                register_output_resources[bel] + resource_offset,
                kRegisterOutputBase + lane,
                std::string(prefixes[bel]) + "Q");
            rememberPinEndpoint(type, fpga::TILE_PIN_INPUT,
                register_input_resources[bel] + resource_offset,
                kRegisterInputBase + lane,
                std::string(prefixes[bel]) + "X");

            type.elements.push_back(makeElement(
                std::format("LUT6_{}", lane), fpga::ELEMENT_LUT5, lane));
            int register_bit = site * 8 + bel;
            type.elements.push_back(makeElement(
                std::format("REG_{}", lane), fpga::ELEMENT_FD, register_bit));
        }
    }
    return type;
}

std::vector<fpga::Tile*> resetPuzzleDevice(const PuzzleParameters& parameters)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.tile_types.clear();
    device.tileconn_rules.clear();
    device.local_route_wire_mappings.clear();
    device.route_wire_graph.clear();
    device.grid_spec.size = parameters.tile_space;
    device.size_width = parameters.width();
    device.size_height = parameters.height();

    device.cb_types.push_back(makePuzzleCrossbar());
    device.tile_types.push_back(makePuzzleTileType());
    fpga::CBType* cb_type = &device.cb_types.front();
    fpga::TileType* tile_type = &device.tile_types.front();

    device.tile_grid.resize(
        static_cast<size_t>(parameters.width() * parameters.height()));
    std::vector<fpga::Tile*> tiles;
    tiles.reserve(device.tile_grid.size());
    for (int y = 0; y < parameters.height(); ++y) {
        for (int x = 0; x < parameters.width(); ++x) {
            fpga::Tile& tile =
                device.tile_grid[static_cast<size_t>(y * parameters.width() + x)];
            tile = {};
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = tile.coord;
            tile.type = fpga::Tile::TILE_LUTS;
            tile.cb_type = cb_type;
            tile.cb.type = cb_type;
            tile.tile_type = tile_type;
            tile.full_name = std::format("PUZZLE_TILE_X{}Y{}", x, y);
            tiles.push_back(&tile);
        }
    }
    device.rebuildIncomingDstMasks();
    return tiles;
}

struct PuzzleCell
{
    rtl::Inst* inst = nullptr;
    bool is_lut = false;
    int lane = -1;

    std::string inputPort() const
    {
        return is_lut ? "I0" : "D";
    }

    std::string outputPort() const
    {
        return is_lut ? "O" : "Q";
    }
};

struct PuzzleDesign
{
    Referable<rtl::Module> top_module;
    Referable<rtl::Module> primitive_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    int next_designator = 1;

    explicit PuzzleDesign(size_t design_size)
    {
        top_module.name = "routing_puzzle_top";
        top_module.is_blackbox = false;
        top_module.nets.reserve(design_size);
        primitive_module.name = "routing_puzzle_primitives";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&top_module);
        cells.reserve(design_size);
        insts.reserve(design_size);
        tasks.reserve(design_size);
    }

    PuzzleCell makeCell(const std::string& name, bool is_lut, int lane,
                        fpga::Tile& tile)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = is_lut ? "LUT6" : "FD";
        cell->module_ref.set(&primitive_module);

        rtl::Port input;
        input.name = is_lut ? "I0" : "D";
        input.type = rtl::Port::PORT_IN;
        cell->ports.emplace_back(std::move(input));
        rtl::Port output;
        output.name = is_lut ? "O" : "Q";
        output.type = rtl::Port::PORT_OUT;
        cell->ports.emplace_back(std::move(output));

        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(cell.get());
        inst->depth = 1;
        inst->height = 0;
        inst->cnt_inputs = is_lut ? 6 : 1;
        inst->cnt_outputs = 1;
        inst->pos = is_lut ? lane * 4 + 3 : lane * 4;
        inst->coord = tile.coord;
        inst->conns.reserve(2);
        for (auto& port : cell->ports) {
            auto& conn = inst->conns.emplace_back();
            conn.port_ref.set(&port);
            conn.inst_ref.set(inst.get());
        }
        tile.assign(inst.get());
        if (is_lut) {
            ++tile.luts6cnt;
        } else {
            ++tile.regs_cnt;
        }

        PuzzleCell result{inst.get(), is_lut, lane};
        cells.push_back(std::move(cell));
        insts.push_back(std::move(inst));
        return result;
    }

    Referable<rtl::Conn>* connection(rtl::Inst& inst,
                                     const std::string& port_name)
    {
        for (auto& conn : inst.conns) {
            if (conn.port_ref.peer && conn.port_ref->name == port_name) {
                return &conn;
            }
        }
        return nullptr;
    }

    rtl::Net& connect(const PuzzleCell& from, const PuzzleCell& to,
                      const std::string& net_name)
    {
        Referable<rtl::Conn>* output = connection(*from.inst, from.outputPort());
        Referable<rtl::Conn>* input = connection(*to.inst, to.inputPort());
        require(output && input,
            "routing-puzzle net references a missing cell port");
        require(output->getPeers().empty(),
            "routing-puzzle generator connected one output more than once");
        require(input->peer == nullptr,
            "routing-puzzle generator connected one input more than once");

        int designator = next_designator++;
        output->port_ref->designator = designator;
        input->port_ref->designator = designator;
        input->set(output);

        auto& net = top_module.nets.emplace_back();
        net.name = net_name;
        net.designators.push_back(designator);
        return net;
    }
};

int outputLocal(const PuzzleCell& cell)
{
    NodeMask nodes = cell.inst->tile->getOutputPinNodes(
        cell.inst->cell_ref->type, cell.outputPort(), cell.inst->pos);
    int local = nodes.firstSetBit();
    require(local >= 0 && countBits(nodes) == 1,
        "routing-puzzle source pin did not resolve to exactly one local node");
    return local;
}

int inputLocal(const PuzzleCell& cell)
{
    NodeMask nodes = cell.inst->tile->getPinNodes(
        cell.inst->cell_ref->type, cell.inputPort(), cell.inst->pos);
    int local = nodes.firstSetBit();
    require(local >= 0 && countBits(nodes) == 1,
        "routing-puzzle sink pin did not resolve to exactly one local node");
    return local;
}

std::string nodeName(const fpga::CBType& cb, fpga::CBNodeNameType type,
                     int node)
{
    const std::string* name = cb.nodeName(type, node);
    require(name != nullptr, "routing-puzzle route references an unnamed node");
    return *name;
}

struct GeneratedHop
{
    fpga::Coord from;
    fpga::Coord to;
    int src = -1;
    int dst = -1;
};

void installPerfectRoute(PuzzleDesign& design, const PuzzleCell& from,
                         const PuzzleCell& to,
                         const std::vector<GeneratedHop>& hops,
                         rtl::Net& net)
{
    require(!hops.empty(),
        "routing-puzzle perfect route contains no tile transition");
    fpga::Device& device = fpga::Device::current();
    std::vector<fpga::Wire> route;
    route.reserve(hops.size() + 2);
    int incoming_dst = -1;

    for (size_t step = 0; step < hops.size(); ++step) {
        const GeneratedHop& hop = hops[step];
        fpga::Tile* tile = device.getTile(hop.from.x, hop.from.y);
        fpga::Tile* landing = device.getTile(hop.to.x, hop.to.y);
        require(tile && landing && tile->cb_type,
            "routing-puzzle route traverses a missing tile");
        fpga::Coord delta{
            hop.to.x - hop.from.x,
            hop.to.y - hop.from.y,
        };
        int direction = directionForDelta(delta);
        int length_index = jumpLengthIndex(
            std::max(std::abs(delta.x), std::abs(delta.y)));
        require(direction >= 0,
            "routing-puzzle route contains a non-mesh transition");
        int src = hop.src;
        int dst = hop.dst;
        require(length_index >= 0 && src == dst
                && src == sourceNode(direction, length_index,
                    src % kTracksPerDirection),
            "routing-puzzle hop does not match its synthetic jump node");
        int joint = step == 0 ? -1 : transitJoint(incoming_dst, src);

        require(!tile->cb.src.jump.testBit(src),
            "routing-puzzle perfect route reused an occupied SRC node");
        tile->cb.src.jump.setBit(src);
        if (step == 0) {
            int local = outputLocal(from);
            require(!tile->cb.local.local.testBit(local),
                "routing-puzzle perfect route reused a source local node");
            tile->cb.local.local.setBit(local);
        } else {
            require(!tile->cb.dst.jump.testBit(incoming_dst),
                "routing-puzzle perfect route reused an occupied transit DST node");
            tile->cb.dst.jump.setBit(incoming_dst);
            if (joint >= 0) {
                require(!tile->cb.joint.jump.testBit(joint),
                    "routing-puzzle perfect route reused an occupied JOINT node");
                tile->cb.joint.jump.setBit(joint);
            }
        }

        fpga::Wire wire;
        wire.type = fpga::Wire::WIRE_CROSSBAR;
        wire.from = tile->coord;
        wire.to = landing->coord;
        wire.local = step == 0 ? outputLocal(from) : incoming_dst;
        wire.jump = src;
        wire.route_jump = src;
        wire.dst = dst;
        wire.joint = joint;
        wire.pos = step == 0 ? 0 : 1;
        wire.from_wire_name = nodeName(*tile->cb_type,
            step == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST, wire.local);
        wire.src_wire_name = nodeName(*tile->cb_type, fpga::CB_NODE_SRC, src);
        wire.dst_wire_name = nodeName(*landing->cb_type, fpga::CB_NODE_DST, dst);
        wire.net_name = net.name;
        route.push_back(std::move(wire));
        incoming_dst = dst;
    }

    fpga::Tile* sink_tile = device.getTile(
        hops.back().to.x, hops.back().to.y);
    require(sink_tile && sink_tile == to.inst->tile.peer,
        "routing-puzzle perfect route does not end at its sink cell");
    int pin_local = inputLocal(to);
    require(!sink_tile->cb.dst.jump.testBit(incoming_dst),
        "routing-puzzle perfect route reused a terminal DST node");
    require(!sink_tile->cb.local.local.testBit(pin_local)
            && !sink_tile->pin_state.leased_nodes.testBit(pin_local),
        "routing-puzzle perfect route reused a sink local node");
    sink_tile->cb.dst.jump.setBit(incoming_dst);
    sink_tile->cb.local.local.setBit(pin_local);
    sink_tile->pin_state.leased_nodes.setBit(pin_local);

    fpga::Wire enter;
    enter.type = fpga::Wire::WIRE_CROSSBAR;
    enter.from = sink_tile->coord;
    enter.to = sink_tile->coord;
    enter.local = incoming_dst;
    enter.jump = -1;
    enter.pos = 1;
    enter.from_wire_name = nodeName(*sink_tile->cb_type,
        fpga::CB_NODE_DST, incoming_dst);
    enter.dst_wire_name = enter.from_wire_name;
    enter.net_name = net.name;
    route.push_back(std::move(enter));

    fpga::Wire pin;
    pin.type = fpga::Wire::WIRE_TILE_PIN;
    pin.from = sink_tile->coord;
    pin.to = sink_tile->coord;
    pin.resource = sink_tile->coord;
    pin.local = pin_local;
    pin.pos = to.inst->pos;
    pin.resource_node = sink_tile->getResourceNodeNum(
        to.inst->cell_ref->type, to.inputPort(), to.inst->pos,
        fpga::TILE_PIN_INPUT, pin_local);
    pin.pin_dir = fpga::TILE_PIN_INPUT;
    pin.cell_type = to.inst->cell_ref->type;
    pin.port = to.inputPort();
    pin.net_name = net.name;
    route.push_back(std::move(pin));

    require(fpga::isRouteComplete(route),
        "routing-puzzle generator produced an incomplete perfect route");
    to.inst->wires.emplace_back(std::move(route));
    size_t route_index = to.inst->wires.size() - 1;
    fpga::attachNetRoute(net, *to.inst, route_index, from.inst, to.inst,
        from.outputPort(), to.inputPort(), net.name);
    fpga::registerNetRouteTiles(net, to.inst->wires.back());

    design.tasks.push_back(pnr::RouteDesign::RouteTask{
        .from = from.inst,
        .to = to.inst,
        .net = &net,
        .from_port = from.outputPort(),
        .to_port = to.inputPort(),
        .net_name = net.name,
    });
}

struct GeneratedPuzzle
{
    struct FanoutTree
    {
        size_t start_index = 0;
        size_t merged_net_count = 0;
        rtl::Net* net = nullptr;
        rtl::Inst* source = nullptr;
    };

    PuzzleParameters parameters;
    std::vector<fpga::Tile*> tiles;
    PuzzleDesign design;
    size_t route_capacity = 0;
    size_t routed_resources = 0;
    size_t minimum_tile_fullness = 0;
    size_t maximum_tile_fullness = 0;
    size_t merged_net_count = 0;
    size_t fanout_suffix_count = 0;
    std::vector<FanoutTree> fanout_trees;
    std::vector<std::pair<size_t, size_t>> route_group_ranges;

    explicit GeneratedPuzzle(PuzzleParameters input)
        : parameters(input)
        , tiles(resetPuzzleDevice(parameters))
        , design(parameters.design_cells)
    {
    }
};

struct MeshLink
{
    int from_tile = -1;
    int to_tile = -1;
    int direction = -1;
    int length_index = -1;
    int track = -1;
};

struct MeshArc
{
    int from_tile = -1;
    int to_tile = -1;
    int src = -1;
    int dst = -1;
};

fpga::Coord coordForTile(int tile, int width)
{
    return {tile % width, tile / width};
}

std::vector<fpga::Wire>& boundTaskRoute(
    const pnr::RouteDesign::RouteTask& task)
{
    require(task.net != nullptr,
        "routing-puzzle task has no RTL net during fanout merge");
    for (rtl::NetRouteBinding& binding : task.net->routes) {
        if (binding.from == task.from && binding.to == task.to
            && binding.from_port == task.from_port
            && binding.to_port == task.to_port
            && binding.route_name == task.net_name && binding.owner
            && binding.route_index < binding.owner->wires.size()) {
            return binding.owner->wires[binding.route_index];
        }
    }
    throw TestFailure{std::format(
        "routing-puzzle task '{}' has no physical binding", task.net_name)};
}

const fpga::Wire& lastIntertileHop(const std::vector<fpga::Wire>& route)
{
    for (auto wire = route.rbegin(); wire != route.rend(); ++wire) {
        if (wire->type == fpga::Wire::WIRE_CROSSBAR
            && !sameCoord(wire->from, wire->to)) {
            return *wire;
        }
    }
    throw TestFailure{
        "routing-puzzle fanout route has no inter-tile crossbar hop"};
}

GeneratedPuzzle::FanoutTree mergeGeneratedFanoutRange(
    GeneratedPuzzle& puzzle, size_t start, size_t merge_count)
{
    pnr::RouteDesign::RouteTask& trunk_task = puzzle.design.tasks[start];
    rtl::Net* fanout_net = trunk_task.net;
    rtl::Inst* fanout_source = trunk_task.from;
    require(fanout_net && fanout_source,
        "routing-puzzle selected an invalid fanout trunk");
    Referable<rtl::Conn>* fanout_output = puzzle.design.connection(
        *fanout_source, trunk_task.from_port);
    require(fanout_output,
        "routing-puzzle fanout trunk has no RTL source connection");

    const fpga::Wire* previous_hop =
        &lastIntertileHop(boundTaskRoute(trunk_task));
    for (size_t offset = 1; offset < merge_count; ++offset) {
        pnr::RouteDesign::RouteTask& task =
            puzzle.design.tasks[start + offset];
        rtl::Net* donor_net = task.net;
        rtl::Inst* donor_source = task.from;
        require(donor_net && donor_net != fanout_net && donor_source,
            "routing-puzzle fanout merge selected a duplicate net");
        require(task.to && task.to->tile.peer && donor_source->tile.peer,
            "routing-puzzle fanout suffix has an unplaced endpoint");

        std::vector<fpga::Wire>& suffix = boundTaskRoute(task);
        require(!suffix.empty()
                && suffix.front().type == fpga::Wire::WIRE_CROSSBAR
                && suffix.front().pos == 0
                && sameCoord(suffix.front().from, previous_hop->to),
            "routing-puzzle selected nets are not consecutive suffixes");
        fpga::Wire& first = suffix.front();
        fpga::Tile* branch_tile = fpga::Device::current().getTile(
            first.from.x, first.from.y);
        require(branch_tile && branch_tile->cb_type,
            "routing-puzzle suffix branch tile is missing");
        require(branch_tile->cb.local.local.testBit(first.local),
            "routing-puzzle suffix takeoff local was not leased");
        branch_tile->cb.local.local &= ~bit(first.local);
        int joint = transitJoint(previous_hop->dst, first.jump);
        if (joint >= 0) {
            require(!branch_tile->cb.joint.jump.testBit(joint),
                "routing-puzzle suffix merge reused a JOINT node");
            branch_tile->cb.joint.jump.setBit(joint);
        }
        first.local = previous_hop->dst;
        first.joint = joint;
        first.pos = 1;
        first.owns_dst = false;
        first.from_wire_name = nodeName(
            *branch_tile->cb_type, fpga::CB_NODE_DST, first.local);

        Referable<rtl::Conn>* donor_output = puzzle.design.connection(
            *donor_source, task.from_port);
        Referable<rtl::Conn>* sink_input = puzzle.design.connection(
            *task.to, task.to_port);
        require(donor_output && sink_input && sink_input->peer == donor_output,
            "routing-puzzle donor RTL connection is inconsistent");
        sink_input->set(fanout_output);
        donor_output->port_ref->designator = -1;

        fanout_net->designators.insert(fanout_net->designators.end(),
            donor_net->designators.begin(), donor_net->designators.end());
        donor_net->designators.clear();
        donor_net->void_net = true;
        donor_net->src_port.clear();
        donor_net->dst_port.clear();
        size_t retargeted = fpga::retargetNetRouteBindings(
            *donor_net, *fanout_net, donor_source, task.to,
            task.from_port, task.to_port, fanout_source, task.to,
            trunk_task.from_port, task.to_port, task.net_name);
        require(retargeted == 1 && donor_net->routes.empty(),
            "routing-puzzle failed to transfer a fanout suffix binding");

        task.from = fanout_source;
        task.from_port = trunk_task.from_port;
        task.net = fanout_net;
        previous_hop = &lastIntertileHop(suffix);
    }

    require(fanout_net->routes.size() == merge_count
            && fanout_net->designators.size() == merge_count
            && fanout_output->getPeers().size() == merge_count,
        "routing-puzzle RTL fanout merge has the wrong branch count");
    return GeneratedPuzzle::FanoutTree{
        start, merge_count, fanout_net, fanout_source};
}

void mergeGeneratedFanout(GeneratedPuzzle& puzzle)
{
    int percent = puzzle.parameters.fanout_merge_percent;
    require(percent >= 0 && percent <= 100,
        "routing-puzzle fanout merge percentage must be between 0 and 100");
    if (percent == 0) {
        return;
    }

    size_t task_count = puzzle.design.tasks.size();
    size_t merge_count = task_count * static_cast<size_t>(percent) / 100;
    require(merge_count * 100
            == task_count * static_cast<size_t>(percent),
        "routing-puzzle fanout percentage does not select whole nets");
    size_t tree_count = puzzle.parameters.fanout_tree_count;
    require(tree_count > 0 && merge_count >= tree_count * 2
            && merge_count <= task_count,
        "routing-puzzle fanout forest needs at least two nets per tree");

    // Assign trees to the largest circuit groups first. Each group is a
    // physical route ring, so disjoint consecutive ranges form valid shared
    // prefixes without inventing routing that the generated mesh cannot hold.
    std::vector<size_t> group_order(puzzle.route_group_ranges.size());
    std::iota(group_order.begin(), group_order.end(), 0);
    std::sort(group_order.begin(), group_order.end(), [&](size_t left,
                  size_t right) {
        return puzzle.route_group_ranges[left].second
            > puzzle.route_group_ranges[right].second;
    });
    std::vector<size_t> trees_by_group(puzzle.route_group_ranges.size(), 0);
    size_t assigned_trees = 0;
    for (size_t group_index : group_order) {
        if (assigned_trees == tree_count) {
            break;
        }
        size_t capacity = puzzle.route_group_ranges[group_index].second / 2;
        if (capacity == 0) {
            continue;
        }
        trees_by_group[group_index] = 1;
        ++assigned_trees;
    }
    while (assigned_trees < tree_count) {
        bool advanced = false;
        for (size_t group_index : group_order) {
            size_t capacity = puzzle.route_group_ranges[group_index].second / 2;
            if (trees_by_group[group_index] >= capacity) {
                continue;
            }
            ++trees_by_group[group_index];
            ++assigned_trees;
            advanced = true;
            if (assigned_trees == tree_count) {
                break;
            }
        }
        require(advanced,
            "routing-puzzle route groups cannot hold the requested forest");
    }

    size_t selected_capacity = 0;
    for (size_t group_index = 0; group_index < trees_by_group.size();
         ++group_index) {
        if (trees_by_group[group_index] != 0) {
            selected_capacity += puzzle.route_group_ranges[group_index].second;
        }
    }
    require(selected_capacity >= merge_count,
        "routing-puzzle selected route groups cannot hold merged fanouts");

    std::vector<size_t> merged_by_group(trees_by_group.size(), 0);
    for (size_t group_index = 0; group_index < trees_by_group.size();
         ++group_index) {
        merged_by_group[group_index] = trees_by_group[group_index] * 2;
    }
    size_t remaining = merge_count - tree_count * 2;
    while (remaining != 0) {
        bool advanced = false;
        for (size_t group_index : group_order) {
            if (trees_by_group[group_index] == 0) {
                continue;
            }
            size_t capacity = puzzle.route_group_ranges[group_index].second;
            if (merged_by_group[group_index] >= capacity) {
                continue;
            }
            ++merged_by_group[group_index];
            --remaining;
            advanced = true;
            if (remaining == 0) {
                break;
            }
        }
        require(advanced,
            "routing-puzzle fanout forest exhausted selected route groups");
    }

    size_t generated_tree_index = 0;
    for (size_t group_index = 0; group_index < trees_by_group.size();
         ++group_index) {
        size_t group_trees = trees_by_group[group_index];
        if (group_trees == 0) {
            continue;
        }
        size_t group_merged = merged_by_group[group_index];
        std::vector<size_t> tree_sizes(group_trees, 2);
        size_t group_extra = group_merged - group_trees * 2;
        auto grow_class = [&](size_t divisor, size_t target_size) {
            for (size_t tree = 0; tree < group_trees && group_extra != 0;
                 ++tree) {
                size_t serial = generated_tree_index + tree;
                if (serial % divisor != 0 || tree_sizes[tree] >= target_size) {
                    continue;
                }
                size_t growth = std::min(
                    group_extra, target_size - tree_sizes[tree]);
                tree_sizes[tree] += growth;
                group_extra -= growth;
            }
        };
        // A deterministic skew models a forest containing many tiny trees and
        // fewer regional trees. Grow rare classes first so they remain present
        // even when the requested average is only three or four sinks.
        grow_class(64, 32);
        grow_class(16, 16);
        grow_class(4, 8);
        grow_class(1, 4);
        for (size_t tree = 0; group_extra != 0; ++tree) {
            ++tree_sizes[tree % group_trees];
            --group_extra;
        }

        size_t cursor = puzzle.route_group_ranges[group_index].first;
        for (size_t tree = 0; tree < group_trees; ++tree) {
            size_t tree_size = tree_sizes[tree];
            require(tree_size >= 2,
                "routing-puzzle generated a fanout tree without a suffix");
            puzzle.fanout_trees.push_back(
                mergeGeneratedFanoutRange(puzzle, cursor, tree_size));
            cursor += tree_size;
        }
        generated_tree_index += group_trees;
    }
    puzzle.merged_net_count = merge_count;
    puzzle.fanout_suffix_count = merge_count - tree_count;
    require(puzzle.fanout_trees.size() == tree_count,
        "routing-puzzle generated the wrong number of fanout trees");
}

GeneratedPuzzle generatePerfectPuzzle(PuzzleParameters parameters)
{
    require(parameters.fullness_percent > 0
            && parameters.fullness_percent <= 100,
        "routing-puzzle fullness must be between 1 and 100 percent");

    GeneratedPuzzle puzzle(parameters);
    std::mt19937_64 random(kPuzzleSeed);
    const int tile_count = parameters.width() * parameters.height();
    require(tile_count > 0
            && parameters.design_cells % static_cast<size_t>(tile_count) == 0,
        "routing-puzzle design size must assign whole cells to every tile");
    const size_t cells_per_tile =
        parameters.design_cells / static_cast<size_t>(tile_count);
    const size_t luts_per_tile = (cells_per_tile + 1) / 2;
    const size_t registers_per_tile = cells_per_tile - luts_per_tile;
    require(cells_per_tile > 0 && luts_per_tile <= kLutsPerTile
            && registers_per_tile <= kRegistersPerTile,
        "routing-puzzle design exceeds the fixed per-tile logic capacity");

    std::vector<std::vector<PuzzleCell>> tile_cells(
        static_cast<size_t>(tile_count));
    size_t generated_cells = 0;
    for (int tile_index = 0; tile_index < tile_count; ++tile_index) {
        fpga::Tile& tile = *puzzle.tiles[static_cast<size_t>(tile_index)];
        std::vector<PuzzleCell>& cells =
            tile_cells[static_cast<size_t>(tile_index)];
        cells.reserve(cells_per_tile);
        for (size_t slot = 0; slot < cells_per_tile; ++slot) {
            bool is_lut = slot < luts_per_tile;
            int lane = static_cast<int>(
                is_lut ? slot : slot - luts_per_tile);
            PuzzleCell cell = puzzle.design.makeCell(
                std::format("puzzle_cell_{}", generated_cells),
                is_lut, lane, tile);
            cells.push_back(cell);
            ++generated_cells;
        }
        require(tile.luts6cnt == static_cast<int>(luts_per_tile)
                && tile.regs_cnt == static_cast<int>(registers_per_tile),
            "routing-puzzle placement did not keep the intended tile balance");
    }
    require(generated_cells == parameters.design_cells,
        "routing-puzzle generator produced the wrong design size");

    // Enumerate every valid undirected link once. Selecting both directed arcs
    // keeps selected indegree and outdegree equal at every tile, allowing the
    // random picture to be decomposed into feasible routes without loose ends.
    constexpr std::array<int, 4> canonical_directions{{2, 3, 4, 5}};
    std::vector<MeshLink> links;
    std::vector<size_t> tile_capacity(static_cast<size_t>(tile_count));
    for (int length_index = 0; length_index < kJumpLengthCount;
         ++length_index) {
        int length = kJumpLengths[length_index];
        for (int direction : canonical_directions) {
            fpga::Coord unit = kUnitDirectionDeltas[direction];
            fpga::Coord delta{unit.x * length, unit.y * length};
            for (int y = 0; y < parameters.height(); ++y) {
                for (int x = 0; x < parameters.width(); ++x) {
                    fpga::Coord to{x + delta.x, y + delta.y};
                    if (to.x < 0 || to.y < 0
                        || to.x >= parameters.width()
                        || to.y >= parameters.height()) {
                        continue;
                    }
                    int from_tile = y * parameters.width() + x;
                    int to_tile = to.y * parameters.width() + to.x;
                    for (int track = 0; track < kTracksPerDirection; ++track) {
                        links.push_back(MeshLink{
                            from_tile, to_tile, direction, length_index, track});
                        ++tile_capacity[static_cast<size_t>(from_tile)];
                        ++tile_capacity[static_cast<size_t>(to_tile)];
                    }
                }
            }
        }
    }
    require(!links.empty(), "routing-puzzle mesh has no valid links");
    puzzle.route_capacity = links.size() * 2;

    std::shuffle(links.begin(), links.end(), random);
    // Links are selected as bidirectional pairs so every track remains
    // balanced. Round to the nearest whole pair when the requested percentage
    // is not exactly representable by the finite mesh capacity.
    size_t requested_scaled = links.size()
        * static_cast<size_t>(parameters.fullness_percent);
    size_t selected_link_count = (requested_scaled + 50) / 100;
    size_t selected_scaled = selected_link_count * 100;
    size_t rounding_error = selected_scaled > requested_scaled
        ? selected_scaled - requested_scaled
        : requested_scaled - selected_scaled;
    require(rounding_error <= 50,
        "routing-puzzle fullness rounding exceeded half a link pair");

    std::vector<MeshArc> arcs;
    arcs.reserve(selected_link_count * 2);
    std::vector<size_t> tile_selected(static_cast<size_t>(tile_count));
    for (size_t index = 0; index < selected_link_count; ++index) {
        const MeshLink& link = links[index];
        int reverse_direction = (link.direction + kDirectionCount / 2)
            % kDirectionCount;
        int forward_src = sourceNode(
            link.direction, link.length_index, link.track);
        int reverse_src = sourceNode(
            reverse_direction, link.length_index, link.track);
        arcs.push_back(
            MeshArc{link.from_tile, link.to_tile, forward_src, forward_src});
        arcs.push_back(
            MeshArc{link.to_tile, link.from_tile, reverse_src, reverse_src});
        ++tile_selected[static_cast<size_t>(link.from_tile)];
        ++tile_selected[static_cast<size_t>(link.to_tile)];
    }
    puzzle.routed_resources = arcs.size();
    require(puzzle.routed_resources == selected_link_count * 2,
        "routing-puzzle generator missed its rounded fullness target");

    std::vector<std::vector<std::vector<size_t>>> outgoing(
        kTracksPerDirection,
        std::vector<std::vector<size_t>>(static_cast<size_t>(tile_count)));
    for (size_t index = 0; index < arcs.size(); ++index) {
        int track = arcs[index].src % kTracksPerDirection;
        outgoing[static_cast<size_t>(track)]
            [static_cast<size_t>(arcs[index].from_tile)].push_back(index);
    }
    for (auto& track_outgoing : outgoing) {
        for (std::vector<size_t>& choices : track_outgoing) {
            std::shuffle(choices.begin(), choices.end(), random);
        }
    }

    // The shuffled adjacency changes both direction and jump length as the
    // traversal advances. Each track is balanced independently, so every
    // component is a closed Euler traversal. This is only a construction
    // proof; the generic router still sees unrestricted cross-track transit.
    constexpr size_t no_edge = static_cast<size_t>(-1);
    struct WalkFrame
    {
        int tile = -1;
        size_t incoming_edge = no_edge;
    };
    std::vector<bool> used(arcs.size());
    std::vector<std::vector<size_t>> circuits;
    size_t traversed_edges = 0;
    for (int track = 0; track < kTracksPerDirection; ++track) {
        std::vector<size_t> cursor(static_cast<size_t>(tile_count));
        for (int start = 0; start < tile_count; ++start) {
            std::vector<size_t>& start_choices =
                outgoing[static_cast<size_t>(track)]
                    [static_cast<size_t>(start)];
            while (cursor[static_cast<size_t>(start)] < start_choices.size()
                   && used[start_choices[cursor[static_cast<size_t>(start)]]]) {
                ++cursor[static_cast<size_t>(start)];
            }
            if (cursor[static_cast<size_t>(start)] == start_choices.size()) {
                continue;
            }
            std::vector<WalkFrame> stack{{start, no_edge}};
            std::vector<size_t> reversed;
            while (!stack.empty()) {
                int tile = stack.back().tile;
                std::vector<size_t>& choices =
                    outgoing[static_cast<size_t>(track)]
                        [static_cast<size_t>(tile)];
                size_t& next = cursor[static_cast<size_t>(tile)];
                while (next < choices.size() && used[choices[next]]) {
                    ++next;
                }
                if (next < choices.size()) {
                    size_t edge = choices[next++];
                    used[edge] = true;
                    stack.push_back(WalkFrame{arcs[edge].to_tile, edge});
                    continue;
                }
                if (stack.back().incoming_edge != no_edge) {
                    reversed.push_back(stack.back().incoming_edge);
                }
                stack.pop_back();
            }
            std::reverse(reversed.begin(), reversed.end());
            traversed_edges += reversed.size();
            circuits.push_back(std::move(reversed));
        }
    }
    require(traversed_edges == arcs.size(),
        "routing-puzzle track traversals did not cover every selected hop");
    for (const std::vector<size_t>& circuit : circuits) {
        require(!circuit.empty(), "routing-puzzle produced an empty circuit");
        int circuit_track = arcs[circuit.front()].src % kTracksPerDirection;
        for (size_t index = 0; index < circuit.size(); ++index) {
            const MeshArc& current = arcs[circuit[index]];
            const MeshArc& next = arcs[circuit[(index + 1) % circuit.size()]];
            require(current.to_tile == next.from_tile
                    && current.src % kTracksPerDirection == circuit_track,
                "routing-puzzle Euler traversal lost track continuity");
        }
    }

    const size_t generated_route_count = parameters.design_cells;
    require(arcs.size() >= generated_route_count,
        "routing-puzzle selected mesh has fewer hops than generated cells");
    require(generated_route_count
            == static_cast<size_t>(tile_count) * cells_per_tile,
        "routing-puzzle endpoint generation lost per-tile cells");

    struct RouteBoundary
    {
        size_t circuit = 0;
        size_t position = 0;
        int tile = -1;
        size_t cell_slot = 0;
    };
    struct CircuitPosition
    {
        size_t circuit = 0;
        size_t position = 0;
    };
    std::vector<std::vector<CircuitPosition>> positions_by_tile(
        static_cast<size_t>(tile_count));
    for (size_t circuit_index = 0; circuit_index < circuits.size();
         ++circuit_index) {
        const std::vector<size_t>& circuit = circuits[circuit_index];
        for (size_t position = 0; position < circuit.size(); ++position) {
            int tile = arcs[circuit[position]].from_tile;
            positions_by_tile[static_cast<size_t>(tile)].push_back(
                CircuitPosition{circuit_index, position});
        }
    }

    std::vector<std::vector<RouteBoundary>> boundaries_by_circuit(
        circuits.size());
    for (int tile = 0; tile < tile_count; ++tile) {
        std::vector<CircuitPosition>& positions =
            positions_by_tile[static_cast<size_t>(tile)];
        require(positions.size() >= tile_cells[static_cast<size_t>(tile)].size(),
            "routing-puzzle tile has too few route visits for all its cells");
        std::shuffle(positions.begin(), positions.end(), random);
        for (size_t slot = 0;
             slot < tile_cells[static_cast<size_t>(tile)].size(); ++slot) {
            const CircuitPosition& selected = positions[slot];
            boundaries_by_circuit[selected.circuit].push_back(RouteBoundary{
                selected.circuit, selected.position, tile, slot});
        }
    }
    size_t boundary_count = 0;
    for (size_t circuit_index = 0; circuit_index < circuits.size();
         ++circuit_index) {
        std::vector<RouteBoundary>& boundaries =
            boundaries_by_circuit[circuit_index];
        require(!boundaries.empty(),
            "routing-puzzle circuit has no generated cell endpoint");
        std::sort(boundaries.begin(), boundaries.end(),
            [](const RouteBoundary& left, const RouteBoundary& right) {
                return left.position < right.position;
            });
        const std::vector<size_t>& circuit = circuits[circuit_index];
        size_t group_start = puzzle.design.tasks.size();
        for (size_t boundary_index = 0; boundary_index < boundaries.size();
             ++boundary_index) {
            const RouteBoundary& start_boundary = boundaries[boundary_index];
            const RouteBoundary& end_boundary =
                boundaries[(boundary_index + 1) % boundaries.size()];
            const PuzzleCell& from = tile_cells
                [static_cast<size_t>(start_boundary.tile)]
                [start_boundary.cell_slot];
            const PuzzleCell& to = tile_cells
                [static_cast<size_t>(end_boundary.tile)]
                [end_boundary.cell_slot];
            std::vector<GeneratedHop> hops;
            size_t position = start_boundary.position;
            do {
                const MeshArc& arc = arcs[circuit[position]];
                hops.push_back(GeneratedHop{
                    coordForTile(arc.from_tile, parameters.width()),
                    coordForTile(arc.to_tile, parameters.width()),
                    arc.src,
                    arc.dst,
                });
                position = (position + 1) % circuit.size();
            } while (position != end_boundary.position);
            size_t route_index = puzzle.design.tasks.size();
            rtl::Net& net = puzzle.design.connect(from, to,
                std::format("puzzle_net_{}", route_index));
            installPerfectRoute(puzzle.design, from, to, hops, net);
            ++boundary_count;
        }
        puzzle.route_group_ranges.emplace_back(
            group_start, boundaries.size());
    }
    require(boundary_count == generated_route_count,
        "routing-puzzle did not create one boundary per cell");
    require(puzzle.design.tasks.size() == generated_route_count,
        "routing-puzzle generator did not create one route per cell");
    for (const std::unique_ptr<Referable<rtl::Inst>>& inst :
         puzzle.design.insts) {
        Referable<rtl::Conn>* input = puzzle.design.connection(
            *inst, inst->cell_ref->type == "LUT6" ? "I0" : "D");
        Referable<rtl::Conn>* output = puzzle.design.connection(
            *inst, inst->cell_ref->type == "LUT6" ? "O" : "Q");
        require(input && input->peer && output
                && output->getPeers().size() == 1,
            "routing-puzzle did not use every generated cell in the RTL ring");
    }

    puzzle.minimum_tile_fullness = kSrcCount;
    for (int tile_index = 0; tile_index < tile_count; ++tile_index) {
        fpga::Tile* tile = puzzle.tiles[static_cast<size_t>(tile_index)];
        size_t occupied = countBits(tile->cb.src.jump);
        require(occupied == tile_selected[static_cast<size_t>(tile_index)],
            "routing-puzzle usage accounting disagrees with installed routes");
        require(tile_capacity[static_cast<size_t>(tile_index)] > 0,
            "routing-puzzle tile has no valid jump capacity");
        size_t percent = occupied * 100
            / tile_capacity[static_cast<size_t>(tile_index)];
        puzzle.minimum_tile_fullness =
            std::min(puzzle.minimum_tile_fullness, percent);
        puzzle.maximum_tile_fullness =
            std::max(puzzle.maximum_tile_fullness, percent);
        int minimum_percent = std::max(0, parameters.fullness_percent - 25);
        int maximum_percent = std::min(100, parameters.fullness_percent + 25);
        require(percent >= static_cast<size_t>(minimum_percent)
                && percent <= static_cast<size_t>(maximum_percent),
            std::format("random tile fullness {}% is outside tolerance", percent));
    }
    mergeGeneratedFanout(puzzle);
    return puzzle;
}

struct RouteDiversity
{
    std::set<int> directions;
    std::set<int> lengths;
    size_t hops = 0;
    size_t direction_changes = 0;
    size_t length_changes = 0;
};

RouteDiversity inspectGeneratedRoute(const std::vector<fpga::Wire>& route)
{
    RouteDiversity result;
    int previous_direction = -1;
    int previous_length = -1;
    for (const fpga::Wire& wire : route) {
        if (wire.type != fpga::Wire::WIRE_CROSSBAR
            || sameCoord(wire.from, wire.to)) {
            continue;
        }
        fpga::Coord delta{
            wire.to.x - wire.from.x,
            wire.to.y - wire.from.y,
        };
        int direction = directionForDelta(delta);
        int length = std::max(std::abs(delta.x), std::abs(delta.y));
        int length_index = jumpLengthIndex(length);
        require(direction >= 0 && length_index >= 0,
            "generated route diversity audit found an invalid jump");
        require(wire.jump == sourceNode(direction, length_index,
                    wire.jump % kTracksPerDirection),
            "generated route diversity audit found the wrong jump node");
        if (previous_direction >= 0 && previous_direction != direction) {
            ++result.direction_changes;
        }
        if (previous_length >= 0 && previous_length != length) {
            ++result.length_changes;
        }
        result.directions.insert(direction);
        result.lengths.insert(length);
        previous_direction = direction;
        previous_length = length;
        ++result.hops;
    }
    return result;
}

const std::vector<fpga::Wire>& generatedBindingRoute(
    const rtl::NetRouteBinding& binding)
{
    require(binding.owner && binding.route_index < binding.owner->wires.size(),
        "generated route diversity audit found an invalid binding");
    return binding.owner->wires[binding.route_index];
}

std::string joinedDirections(const std::set<int>& directions)
{
    constexpr std::array<const char*, kDirectionCount> names{{
        "N", "NE", "E", "SE", "S", "SW", "W", "NW",
    }};
    std::string result;
    for (int direction : directions) {
        if (!result.empty()) {
            result += ',';
        }
        result += names[static_cast<size_t>(direction)];
    }
    return result;
}

std::string joinedLengths(const std::set<int>& lengths)
{
    std::string result;
    for (int length : lengths) {
        if (!result.empty()) {
            result += ',';
        }
        result += std::to_string(length);
    }
    return result;
}

void auditAndPrintGeneratedDiversity(const GeneratedPuzzle& puzzle)
{
    std::set<int> all_directions;
    std::set<int> all_lengths;
    std::vector<const rtl::NetRouteBinding*> bindings;
    std::vector<std::pair<size_t, const rtl::NetRouteBinding*>> diverse_bindings;
    for (const auto& net : puzzle.design.top_module.nets) {
        for (const rtl::NetRouteBinding& binding : net.routes) {
            RouteDiversity diversity = inspectGeneratedRoute(
                generatedBindingRoute(binding));
            bool diverse = diversity.directions.size() > 1
                && diversity.lengths.size() > 1
                && diversity.direction_changes > 0
                && diversity.length_changes > 0;
            all_directions.insert(
                diversity.directions.begin(), diversity.directions.end());
            all_lengths.insert(
                diversity.lengths.begin(), diversity.lengths.end());
            bindings.push_back(&binding);
            if (diverse) {
                diverse_bindings.emplace_back(bindings.size() - 1, &binding);
            }
        }
    }
    require(bindings.size() == puzzle.design.tasks.size(),
        "generated route diversity audit missed physical bindings");
    require(all_directions.size() == kDirectionCount,
        "generated routes did not use all eight jump directions");
    require(all_lengths.size() == kJumpLengthCount,
        "generated routes did not use jump lengths 1, 2, and 4");
    require(diverse_bindings.size() * 100 >= bindings.size() * 80,
        std::format("only {} of {} generated routes change both direction "
                    "and jump length",
            diverse_bindings.size(), bindings.size()));

    std::mt19937_64 sample_random(kPuzzleSeed ^ 0xd3b6a55ULL);
    const auto& sampled = diverse_bindings[static_cast<size_t>(sample_random()
        % diverse_bindings.size())];
    size_t sample_index = sampled.first;
    const rtl::NetRouteBinding& sample = *sampled.second;
    const std::vector<fpga::Wire>& sample_route =
        generatedBindingRoute(sample);
    RouteDiversity diversity = inspectGeneratedRoute(sample_route);
    std::fprintf(stdout,
        "routing_puzzle generated_sample: net=%s index=%zu/%zu hops=%zu "
        "directions=%s lengths=%s direction_changes=%zu length_changes=%zu "
        "diverse_routes=%zu/%zu\n",
        sample.route_name.c_str(), sample_index, bindings.size(), diversity.hops,
        joinedDirections(diversity.directions).c_str(),
        joinedLengths(diversity.lengths).c_str(),
        diversity.direction_changes, diversity.length_changes,
        diverse_bindings.size(), bindings.size());
    std::fflush(stdout);

    if (std::getenv("SCALEPNR_PUZZLE_DEBUG") == nullptr) {
        return;
    }
    constexpr size_t debug_hop_limit = 200;
    size_t printed = 0;
    for (const fpga::Wire& wire : sample_route) {
        if (wire.type != fpga::Wire::WIRE_CROSSBAR
            || sameCoord(wire.from, wire.to)) {
            continue;
        }
        fpga::Coord delta{wire.to.x - wire.from.x, wire.to.y - wire.from.y};
        int direction = directionForDelta(delta);
        int length = std::max(std::abs(delta.x), std::abs(delta.y));
        std::fprintf(stdout,
            "  hop[%zu] (%d,%d)->(%d,%d) direction=%s length=%d src=%d "
            "track=%d\n",
            printed, wire.from.x, wire.from.y, wire.to.x, wire.to.y,
            joinedDirections(std::set<int>{direction}).c_str(), length,
            wire.jump, wire.jump % kTracksPerDirection);
        if (++printed == debug_hop_limit) {
            break;
        }
    }
    if (diversity.hops > printed) {
        std::fprintf(stdout, "  ... %zu additional hops omitted\n",
            diversity.hops - printed);
    }
}

void auditGeneratedFanout(GeneratedPuzzle& puzzle)
{
    if (puzzle.parameters.fanout_merge_percent == 0) {
        require(puzzle.merged_net_count == 0
                && puzzle.fanout_suffix_count == 0
                && puzzle.fanout_trees.empty(),
            "disabled routing-puzzle fanout merge changed the design");
        return;
    }
    require(!puzzle.fanout_trees.empty() && puzzle.merged_net_count > 1
            && puzzle.fanout_suffix_count
                == puzzle.merged_net_count - puzzle.fanout_trees.size(),
        "routing-puzzle fanout merge metadata is inconsistent");

    size_t minimum_tree = puzzle.merged_net_count;
    size_t maximum_tree = 0;
    std::vector<int> tree_spreads;
    std::vector<int> tree_maximum_distances;
    for (const GeneratedPuzzle::FanoutTree& tree : puzzle.fanout_trees) {
        require(tree.net && tree.source && tree.merged_net_count > 1
                && tree.net->routes.size() == tree.merged_net_count
                && tree.net->designators.size() == tree.merged_net_count,
            "routing-puzzle fanout-tree metadata is inconsistent");
        Referable<rtl::Conn>* source_output = puzzle.design.connection(
            *tree.source, puzzle.design.tasks[tree.start_index].from_port);
        require(source_output
                && source_output->getPeers().size() == tree.merged_net_count,
            "routing-puzzle fanout-tree source has the wrong sink count");

        const fpga::Wire* previous_hop = nullptr;
        fpga::Coord source_coord = tree.source->tile->coord;
        int minimum_x = source_coord.x;
        int maximum_x = source_coord.x;
        int minimum_y = source_coord.y;
        int maximum_y = source_coord.y;
        int maximum_distance = 0;
        for (size_t offset = 0; offset < tree.merged_net_count; ++offset) {
            const pnr::RouteDesign::RouteTask& task =
                puzzle.design.tasks[tree.start_index + offset];
            require(task.net == tree.net && task.from == tree.source,
                "routing-puzzle fanout task did not retain its common source");
            Referable<rtl::Conn>* sink_input = puzzle.design.connection(
                *task.to, task.to_port);
            require(sink_input && sink_input->peer == source_output,
                "routing-puzzle fanout sink was not updated in RTL");
            fpga::Coord sink_coord = task.to->tile->coord;
            minimum_x = std::min(minimum_x, sink_coord.x);
            maximum_x = std::max(maximum_x, sink_coord.x);
            minimum_y = std::min(minimum_y, sink_coord.y);
            maximum_y = std::max(maximum_y, sink_coord.y);
            maximum_distance = std::max(maximum_distance,
                std::abs(source_coord.x - sink_coord.x)
                    + std::abs(source_coord.y - sink_coord.y));
            std::vector<fpga::Wire>& route = boundTaskRoute(task);
            require(fpga::isRouteComplete(route),
                "routing-puzzle generated fanout branch is incomplete");
            if (offset == 0) {
                require(route.front().pos == 0,
                    "routing-puzzle fanout trunk lost its Takeoff");
            } else {
                require(previous_hop && route.front().pos != 0
                        && !route.front().owns_dst
                        && route.front().local == previous_hop->dst
                        && sameCoord(route.front().from, previous_hop->to),
                    "routing-puzzle fanout suffix is detached from predecessor");
            }
            previous_hop = &lastIntertileHop(route);
        }
        minimum_tree = std::min(minimum_tree, tree.merged_net_count);
        maximum_tree = std::max(maximum_tree, tree.merged_net_count);
        tree_spreads.push_back(
            maximum_x - minimum_x + maximum_y - minimum_y);
        tree_maximum_distances.push_back(maximum_distance);
    }
    std::sort(tree_spreads.begin(), tree_spreads.end());
    std::sort(tree_maximum_distances.begin(), tree_maximum_distances.end());
    int median_spread = tree_spreads[tree_spreads.size() / 2];
    int median_maximum_distance =
        tree_maximum_distances[tree_maximum_distances.size() / 2];

    size_t active_nets = static_cast<size_t>(std::count_if(
        puzzle.design.top_module.nets.begin(),
        puzzle.design.top_module.nets.end(),
        [](const rtl::Net& net) { return !net.routes.empty(); }));
    require(active_nets
            == puzzle.design.tasks.size() - puzzle.merged_net_count
                + puzzle.fanout_trees.size(),
        "routing-puzzle fanout merge retained donor physical nets");
    std::fprintf(stdout,
        "routing_puzzle generated_fanout: trees=%zu merged_nets=%zu "
        "suffixes=%zu tree_sinks=%zu..%zu median_spread=%d "
        "median_max_distance=%d route_tasks=%zu active_nets=%zu cells=%zu\n",
        puzzle.fanout_trees.size(), puzzle.merged_net_count,
        puzzle.fanout_suffix_count, minimum_tree, maximum_tree, median_spread,
        median_maximum_distance, puzzle.design.tasks.size(), active_nets,
        puzzle.design.insts.size());
    std::fflush(stdout);
}

void clearPerfectRouting(GeneratedPuzzle& puzzle)
{
    // The fixture owns every dynamic lease in this synthetic device. Clear all
    // route bindings and tile bookkeeping in one linear sweep; invoking the
    // production single-net unrouter thousands of times would repeatedly
    // rebuild overlapping tile registration lists. No route fragment or lease
    // is retained to assist the subsequent generic reroute.
    size_t cleared = 0;
    for (auto& net : puzzle.design.top_module.nets) {
        for (rtl::NetRouteBinding& binding : net.routes) {
            require(binding.owner
                    && binding.route_index < binding.owner->wires.size(),
                std::format("generated route '{}' has an invalid binding",
                    binding.route_name));
            std::vector<fpga::Wire>& route =
                binding.owner->wires[binding.route_index];
            require(!route.empty(),
                std::format("generated route '{}' was already empty",
                    binding.route_name));
            route.clear();
            ++cleared;
        }
    }
    require(cleared == puzzle.design.tasks.size(),
        "routing-puzzle did not clear every generated net");

    for (fpga::Tile* tile : puzzle.tiles) {
        tile->cb.src.jump = {};
        tile->cb.dst.jump = {};
        tile->cb.joint.jump = {};
        tile->cb.local.local = {};
        tile->cb.src_deadend.jump = {};
        tile->pin_state.leased_nodes = {};
        for (Ref<rtl::Net>& net : tile->routedNets) {
            net.clear();
        }
        tile->routedNets.clear();
        require(tile->cb.src.jump == NodeMask{}
                && tile->cb.dst.jump == NodeMask{}
                && tile->cb.joint.jump == NodeMask{}
                && tile->cb.local.local == NodeMask{}
                && tile->pin_state.leased_nodes == NodeMask{},
            "routing-puzzle clear left a dynamic routing lease behind");
        require(std::none_of(tile->routedNets.begin(), tile->routedNets.end(),
                    [](const Ref<rtl::Net>& net) { return net.peer != nullptr; }),
            "routing-puzzle clear left a routed-net tile reference behind");
    }
    for (const auto& net : puzzle.design.top_module.nets) {
        for (const rtl::NetRouteBinding& binding : net.routes) {
            require(binding.owner
                    && binding.route_index < binding.owner->wires.size()
                    && binding.owner->wires[binding.route_index].empty(),
                "routing-puzzle clear retained generated route fragments");
        }
    }
}

void compactCompleted(std::vector<pnr::RouteDesign::RouteTask>& tasks)
{
    std::erase_if(tasks, [](const pnr::RouteDesign::RouteTask& task) {
        return task.remove_after_pass;
    });
}

void collectRouterWork(pnr::RouteDesign& router,
                       std::vector<pnr::RouteDesign::RouteTask>& generic,
                       std::vector<pnr::RouteDesign::RouteTask>& fanout)
{
    auto collect = [&](std::vector<pnr::RouteDesign::RouteTask>& queued,
                       bool force_fanout) {
        for (pnr::RouteDesign::RouteTask& task : queued) {
            task.remove_after_pass = false;
            task.fanout = force_fanout || task.fanout;
            router.enqueueRouteTask(task, task.fanout ? fanout : generic);
        }
        queued.clear();
    };
    collect(router.pending_route_todo, false);
    collect(router.fanout_route_todo, true);
    collect(router.moving_deferred_todo, false);
}

bool taskRouteIsComplete(const pnr::RouteDesign::RouteTask& task)
{
    if (!task.net) {
        return false;
    }
    for (const rtl::NetRouteBinding& binding : task.net->routes) {
        if (binding.from != task.from || binding.to != task.to
            || binding.from_port != task.from_port
            || binding.to_port != task.to_port
            || binding.route_name != task.net_name || !binding.owner
            || binding.route_index >= binding.owner->wires.size()) {
            continue;
        }
        return fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
    }
    return false;
}

void collectIncompleteOriginalTasks(
    pnr::RouteDesign& router,
    const std::vector<pnr::RouteDesign::RouteTask>& original,
    std::vector<pnr::RouteDesign::RouteTask>& tasks)
{
    for (const pnr::RouteDesign::RouteTask& task : original) {
        if (taskRouteIsComplete(task)) {
            continue;
        }
        pnr::RouteDesign::RouteTask restored = task;
        restored.remove_after_pass = false;
        restored.no_progress_passes = 0;
        router.enqueueRouteTask(restored, tasks);
    }
}

struct StageCounts
{
    size_t basic_passes = 0;
    size_t fanout_passes = 0;
    size_t moving_passes = 0;
    size_t left_after_basic = 0;
    size_t left_after_fanout = 0;
};

void printPassStart(const char* stage, size_t pass, size_t generic_queued,
                    size_t fanout_queued)
{
    std::fprintf(stdout,
        "routing_puzzle pass_start: stage=%s pass=%zu generic=%zu "
        "fanout=%zu\n",
        stage, pass, generic_queued, fanout_queued);
    std::fflush(stdout);
}

void printPassResult(const char* stage, size_t pass,
                     const pnr::RouteDesign::RouteBatchResult& result,
                     const pnr::RouteDesign::RouteStats& before,
                     const pnr::RouteDesign::RouteStats& after,
                     size_t generic_queued, size_t fanout_queued,
                     std::chrono::steady_clock::time_point started)
{
    double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::fprintf(stdout,
        "routing_puzzle pass_result: stage=%s pass=%zu attempted=%zu "
        "completed=%zu active=%zu advanced=%zu changed=%zu deferred=%zu "
        "generic=%zu fanout=%zu searches=%zu pops=%zu trials=%zu "
        "accepted=%zu busy=%zu busy_src=%zu busy_dst=%zu deadend=%zu "
        "src_deadend_marks=%zu no_src=%zu preempt=%zu/%zu seconds=%.3f\n",
        stage, pass, result.attempted, result.completed, result.active,
        result.advanced, result.changed, result.deferred_fanout,
        generic_queued, fanout_queued,
        after.route_searches - before.route_searches,
        after.search_pops - before.search_pops,
        after.edge_trials - before.edge_trials,
        after.edge_accepted - before.edge_accepted,
        after.edge_rejected_busy - before.edge_rejected_busy,
        after.edge_rejected_busy_src - before.edge_rejected_busy_src,
        after.edge_rejected_busy_dst - before.edge_rejected_busy_dst,
        after.edge_rejected_deadend - before.edge_rejected_deadend,
        after.src_deadend_marks - before.src_deadend_marks,
        after.no_src_nodes - before.no_src_nodes,
        after.preempt_success - before.preempt_success,
        after.preempt_attempts - before.preempt_attempts, seconds);
    std::fflush(stdout);
}

void printUnroutedDecisionHistory(
    const char* stage,
    const std::vector<pnr::RouteDesign::RouteTask>& tasks,
    const pnr::RouteDesign& router)
{
    constexpr size_t history_limit = 5;
    size_t printed = 0;
    for (const pnr::RouteDesign::RouteTask& task : tasks) {
        if (taskRouteIsComplete(task)) {
            continue;
        }
        const std::vector<fpga::Wire>* route = nullptr;
        if (task.net) {
            for (const rtl::NetRouteBinding& binding : task.net->routes) {
                if (binding.from == task.from && binding.to == task.to
                    && binding.from_port == task.from_port
                    && binding.to_port == task.to_port
                    && binding.route_name == task.net_name && binding.owner
                    && binding.route_index < binding.owner->wires.size()) {
                    route = &binding.owner->wires[binding.route_index];
                    break;
                }
            }
        }
        size_t crossbars = route
            ? static_cast<size_t>(std::count_if(route->begin(), route->end(),
                [](const fpga::Wire& wire) {
                    return wire.type == fpga::Wire::WIRE_CROSSBAR;
                }))
            : 0;
        int from_x = task.from && task.from->tile.peer
            ? task.from->tile->coord.x : -1;
        int from_y = task.from && task.from->tile.peer
            ? task.from->tile->coord.y : -1;
        int to_x = task.to && task.to->tile.peer
            ? task.to->tile->coord.x : -1;
        int to_y = task.to && task.to->tile.peer
            ? task.to->tile->coord.y : -1;
        std::fprintf(stdout,
            "routing_puzzle decision_history[%zu]: stage=%s net=%s "
            "from=(%d,%d) to=(%d,%d) fanout=%d attempt=%zu "
            "branch_attempt=%zu branch_offset=%zu no_progress=%zu "
            "route_size=%zu crossbars=%zu src_deadends=%zu\n",
            printed, stage, task.net_name.c_str(), from_x, from_y, to_x, to_y,
            task.fanout, task.attempt, task.fanout_branch_attempt,
            task.fanout_branch_offset, task.no_progress_passes,
            route ? route->size() : 0, crossbars, task.src_deadends.size());
        if (++printed == history_limit) {
            break;
        }
    }
    const pnr::RouteDesign::RouteStats& stats = router.route_stats;
    std::fprintf(stdout,
        "routing_puzzle decision_summary: stage=%s unfinished=%zu "
        "searches=%zu pops=%zu trials=%zu accepted=%zu busy=%zu "
        "busy_src=%zu busy_dst=%zu deadend_rejected=%zu "
        "src_deadend_marks=%zu no_src=%zu failed=%zu "
        "preempt=%zu/%zu deadline_expired=%d\n",
        stage, tasks.size(), stats.route_searches, stats.search_pops,
        stats.edge_trials, stats.edge_accepted, stats.edge_rejected_busy,
        stats.edge_rejected_busy_src, stats.edge_rejected_busy_dst,
        stats.edge_rejected_deadend, stats.src_deadend_marks,
        stats.no_src_nodes, stats.failed, stats.preempt_success,
        stats.preempt_attempts, router.route_stage_deadline_expired);
    std::fflush(stdout);
}

StageCounts reroutePuzzle(GeneratedPuzzle& puzzle)
{
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.fpga_width = puzzle.parameters.width();
    router.fpga_height = puzzle.parameters.height();
    router.iteration_limit = 64;
    router.route_suffix_depth_limit = 5;
    router.move_attempt_limit = 32;
    router.route_stage_deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(270);
    router.route_stage_deadline_enabled = true;
    router.route_stage_deadline_expired = false;

    std::vector<pnr::RouteDesign::RouteTask> unscheduled = puzzle.design.tasks;
    std::vector<pnr::RouteDesign::RouteTask> basic;
    std::vector<pnr::RouteDesign::RouteTask> fanouts;
    pnr::RouteDesign::scheduleOneSeedPerSource(
        unscheduled, basic, fanouts);
    require(fanouts.size() == puzzle.fanout_suffix_count
            && basic.size() + fanouts.size() == puzzle.design.tasks.size(),
        "routing-puzzle source scheduling did not split trunks and fanouts");

    StageCounts counts;
    constexpr size_t max_basic_passes = 32;
    for (; counts.basic_passes < max_basic_passes && !basic.empty();
         ++counts.basic_passes) {
        size_t pass = counts.basic_passes + 1;
        printPassStart("Basic", pass, basic.size(), fanouts.size());
        auto pass_started = std::chrono::steady_clock::now();
        pnr::RouteDesign::RouteStats stats_before = router.route_stats;
        pnr::RouteDesign::RouteBatchResult result = router.routeTaskBatch(
            pnr::RouteDesign::RouteTaskMode::Generic, basic, basic.size(), 5);
        compactCompleted(basic);
        size_t queued_before = basic.size() + fanouts.size();
        collectRouterWork(router, basic, fanouts);
        printPassResult("Basic", pass, result, stats_before,
            router.route_stats, basic.size(), fanouts.size(), pass_started);
        if (router.route_stage_deadline_expired) {
            printUnroutedDecisionHistory("Basic", basic, router);
            throw TestFailure{
                "routing-puzzle Basic exceeded its routing deadline"};
        }
        bool queued_work = basic.size() + fanouts.size() > queued_before;
        if (result.completed == 0 && result.advanced == 0
            && result.changed == 0 && !queued_work) {
            break;
        }
    }
    counts.left_after_basic = basic.size();

    // Fanout is an explicit stage even when this single-sink puzzle supplies no
    // deferred branches. The same harness will route non-empty fanout queues in
    // subsequent parameterized cases.
    do {
        ++counts.fanout_passes;
        printPassStart(
            "Fanout", counts.fanout_passes, basic.size(), fanouts.size());
        auto pass_started = std::chrono::steady_clock::now();
        pnr::RouteDesign::RouteStats stats_before = router.route_stats;
        pnr::RouteDesign::RouteBatchResult result = router.routeTaskBatch(
            pnr::RouteDesign::RouteTaskMode::Fanout, fanouts, fanouts.size(), 5);
        compactCompleted(fanouts);
        collectRouterWork(router, basic, fanouts);
        printPassResult("Fanout", counts.fanout_passes, result, stats_before,
            router.route_stats, basic.size(), fanouts.size(), pass_started);
        if (router.route_stage_deadline_expired) {
            printUnroutedDecisionHistory("Fanout", fanouts, router);
            throw TestFailure{
                "routing-puzzle Fanout exceeded its routing deadline"};
        }
        if (fanouts.empty()
            || (result.completed == 0 && result.advanced == 0
                && result.changed == 0)) {
            break;
        }
    } while (counts.fanout_passes < 32);
    counts.left_after_fanout = basic.size() + fanouts.size();

    std::vector<pnr::RouteDesign::RouteTask> moving;
    moving.reserve(basic.size() + fanouts.size());
    moving.insert(moving.end(), basic.begin(), basic.end());
    moving.insert(moving.end(), fanouts.begin(), fanouts.end());
    collectIncompleteOriginalTasks(router, puzzle.design.tasks, moving);
    router.moving_stage = true;
    router.route_deadends_enabled = false;
    size_t previous_unfinished = moving.size();
    size_t stagnant_passes = 0;
    do {
        ++counts.moving_passes;
        printPassStart(
            "Moving", counts.moving_passes, moving.size(), 0);
        auto pass_started = std::chrono::steady_clock::now();
        pnr::RouteDesign::RouteStats stats_before = router.route_stats;
        pnr::RouteDesign::RouteBatchResult result = router.routeTaskBatch(
            pnr::RouteDesign::RouteTaskMode::Moving, moving, moving.size(), 5);
        compactCompleted(moving);
        std::vector<pnr::RouteDesign::RouteTask> queued_generic;
        std::vector<pnr::RouteDesign::RouteTask> queued_fanout;
        collectRouterWork(router, queued_generic, queued_fanout);
        for (pnr::RouteDesign::RouteTask& task : queued_generic) {
            router.enqueueRouteTask(task, moving);
        }
        for (pnr::RouteDesign::RouteTask& task : queued_fanout) {
            router.enqueueRouteTask(task, moving);
        }
        collectIncompleteOriginalTasks(router, puzzle.design.tasks, moving);
        printPassResult("Moving", counts.moving_passes, result, stats_before,
            router.route_stats, moving.size(), 0, pass_started);
        if (router.route_stage_deadline_expired) {
            printUnroutedDecisionHistory("Moving", moving, router);
            throw TestFailure{
                "routing-puzzle Moving exceeded its routing deadline"};
        }
        if (moving.empty()) {
            break;
        }
        stagnant_passes = moving.size() < previous_unfinished
            ? 0 : stagnant_passes + 1;
        previous_unfinished = moving.size();
        bool no_search_progress = result.completed == 0
            && result.advanced == 0 && result.changed == 0;
        if (no_search_progress || stagnant_passes >= 3) {
            bool moved = false;
            std::string last_failure;
            for (const pnr::RouteDesign::RouteTask& candidate : moving) {
                std::vector<pnr::RouteDesign::RouteTask> moved_tasks;
                pnr::RouteDesign::RouteTask focus = candidate;
                if (!router.moveUnfinishedCell(
                        focus, &moved_tasks, &candidate, &last_failure)) {
                    continue;
                }
                router.moving_focus_inst = focus.to;
                for (pnr::RouteDesign::RouteTask& moved_task : moved_tasks) {
                    router.enqueueRouteTask(moved_task, moving);
                }
                moved = true;
                stagnant_passes = 0;
                previous_unfinished = moving.size();
                break;
            }
            if (!moved) {
                throw TestFailure{std::format(
                    "Moving could not relocate any of {} unfinished routes: {}",
                    moving.size(), last_failure)};
            }
        }
    } while (counts.moving_passes < 64);

    require(moving.empty(),
        std::format("routing-puzzle reroute left {} tasks unfinished",
            moving.size()));
    return counts;
}

void auditReroutedPuzzle(const GeneratedPuzzle& puzzle)
{
    size_t complete = 0;
    for (const pnr::RouteDesign::RouteTask& task : puzzle.design.tasks) {
        require(taskRouteIsComplete(task),
            std::format("rerouted route '{}' is incomplete", task.net_name));
        ++complete;
    }
    require(complete == puzzle.design.tasks.size(),
        "routing-puzzle completion audit counted the wrong number of routes");
}

void routes_puzzle_case(PuzzleParameters parameters)
{
    std::fprintf(stdout,
        "routing_puzzle case_start: cells=%zu tiles=%dx%d fullness=%d%% "
        "fanout_merge=%d%% fanout_trees=%zu\n",
        parameters.design_cells, parameters.width(), parameters.height(),
        parameters.fullness_percent, parameters.fanout_merge_percent,
        parameters.fanout_tree_count);
    std::fflush(stdout);
    auto started = std::chrono::steady_clock::now();
    GeneratedPuzzle puzzle = generatePerfectPuzzle(parameters);
    require(puzzle.design.insts.size() == parameters.design_cells,
        "routing-puzzle fixture contains the wrong number of cells");
    require(puzzle.tiles.size()
            == static_cast<size_t>(parameters.width() * parameters.height()),
        "routing-puzzle fixture contains the wrong tile-space size");
    auditAndPrintGeneratedDiversity(puzzle);
    auditGeneratedFanout(puzzle);
    clearPerfectRouting(puzzle);
    StageCounts stages = reroutePuzzle(puzzle);
    auditReroutedPuzzle(puzzle);

    double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::fprintf(stdout,
        "routing_puzzle: cells=%zu tiles=%dx%d generated_fullness=%d%% "
        "resources=%zu/%zu tile_fullness=%zu..%zu%% "
        "fanout_merge=%d%% fanout_trees=%zu merged_nets=%zu suffixes=%zu "
        "passes=%zu/%zu/%zu remaining=%zu/%zu "
        "seconds=%.3f\n",
        parameters.design_cells, parameters.width(), parameters.height(),
        parameters.fullness_percent, puzzle.routed_resources,
        puzzle.route_capacity, puzzle.minimum_tile_fullness,
        puzzle.maximum_tile_fullness, parameters.fanout_merge_percent,
        puzzle.fanout_trees.size(), puzzle.merged_net_count,
        puzzle.fanout_suffix_count,
        stages.basic_passes, stages.fanout_passes, stages.moving_passes,
        stages.left_after_basic, stages.left_after_fanout, seconds);
}

} // namespace

int main(int argc, char** argv)
{
    try {
        PuzzleParameters parameters{
            .design_cells = 10'000,
            .tile_space = {50, 50},
            .fullness_percent = 50,
            .fanout_merge_percent = 10,
            .fanout_tree_count = 1,
        };
        auto parse_argument = [&](int index, const char* name, long minimum,
                                  long maximum) {
            char* end = nullptr;
            long parsed = std::strtol(argv[index], &end, 10);
            require(end && *end == '\0' && parsed >= minimum
                    && parsed <= maximum,
                std::format("invalid routing-puzzle {} '{}'", name,
                    argv[index]));
            return parsed;
        };
        if (argc == 2) {
            parameters.fullness_percent = static_cast<int>(
                parse_argument(1, "fullness", 1, 100));
        }
        else if (argc == 6 || argc == 7) {
            parameters.design_cells = static_cast<size_t>(
                parse_argument(1, "design size", 1, 1'000'000));
            parameters.tile_space.x = static_cast<int>(
                parse_argument(2, "tile width", 1, 1'000));
            parameters.tile_space.y = static_cast<int>(
                parse_argument(3, "tile height", 1, 1'000));
            parameters.fullness_percent = static_cast<int>(
                parse_argument(4, "fullness", 1, 100));
            parameters.fanout_merge_percent = static_cast<int>(
                parse_argument(5, "fanout merge", 0, 100));
            if (argc == 7) {
                parameters.fanout_tree_count = static_cast<size_t>(
                    parse_argument(6, "fanout trees", 1, 500'000));
            }
        }
        else {
            require(argc == 1,
                "routing-puzzle expects design, width, height, fullness, and "
                "fanout-merge percentage, optionally followed by fanout-tree "
                "count");
        }
        routes_puzzle_case(parameters);
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "routing_puzzle failure: %s\n",
            failure.message.c_str());
        return EXIT_FAILURE;
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "routing_puzzle exception: %s\n", error.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
