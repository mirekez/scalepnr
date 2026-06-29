#include "RouteDesign.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "Cell.h"
#include "Conn.h"
#include "Module.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <memory>
#include <random>
#include <sstream>
#include <string>
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

int encodeJump(int dx, int dy, int lane)
{
    auto encode = [](int value) {
        return value & 0xf;
    };
    return (encode(dx) << 7) | (encode(dy) << 3) | (lane & 0x7);
}

void rememberJumpTarget(fpga::CBType& cb, int src, int dst, fpga::Coord delta)
{
    fpga::CBJumpState dsts{};
    dsts.jump = bit(dst);
    fpga::CBType::ResolvedJump entry{};
    entry.delta = delta;
    entry.target_cb_type_id = cb.type_id;
    entry.dsts = dsts;
    cb.dst_by_src[src].push_back(entry);
}

void rememberConn(fpga::CBType& cb, fpga::CBNodeNameType from_type, int from,
                  fpga::CBNodeNameType to_type, int to)
{
    const std::string* from_name = cb.nodeName(from_type, from);
    const std::string* to_name = cb.nodeName(to_type, to);
    cb.rememberConnName(from_type, from, to_type, to,
        from_name ? *from_name : std::to_string(from),
        to_name ? *to_name : std::to_string(to));
}

fpga::CBType makeArenaCrossbar()
{
    fpga::CBType cb{};
    cb.name = "ARENA_CB";
    cb.type_id = 0;

    constexpr int source_local = 16; // LUT6/O fallback output local.
    constexpr int sink_local = 17;   // LUT6/I0 fallback input local.
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, source_local, "ARENA_OUT");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, sink_local, "ARENA_IN");

    std::vector<fpga::Coord> deltas{
        {0, -1}, {1, -1}, {1, 0}, {1, 1},
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
    };

    NodeMask all_srcs{};
    for (fpga::Coord delta : deltas) {
        for (int lane = 0; lane < 4; ++lane) {
            int src = encodeJump(delta.x, delta.y, lane);
            int dst = src;
            std::string suffix = std::format("{}_{}_{}", delta.x, delta.y, lane);
            cb.rememberNodeName(fpga::CB_NODE_SRC, src, "ARENA_SRC_" + suffix);
            cb.rememberNodeName(fpga::CB_NODE_DST, dst, "ARENA_DST_" + suffix);
            cb.local_src[source_local].jump |= bit(src);            cb.dst_local[dst].local |= bit(sink_local);
            rememberJumpTarget(cb, src, dst, delta);
            rememberConn(cb, fpga::CB_NODE_LOCAL, source_local, fpga::CB_NODE_SRC, src);
            rememberConn(cb, fpga::CB_NODE_SRC, src, fpga::CB_NODE_DST, dst);
            rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, sink_local);
            all_srcs |= bit(src);
        }
    }

    for (fpga::Coord delta : deltas) {
        for (int lane = 0; lane < 4; ++lane) {
            int dst = encodeJump(delta.x, delta.y, lane);
            cb.dst_src[dst].jump |= all_srcs;
            rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, encodeJump(0, -1, lane));
        }
    }

    cb.local_output_nodes = bit(source_local);
    cb.local_input_nodes = bit(sink_local);
    cb.rebuildOutgoingSrcs();
    cb.ensureDerivedMasks();
    return cb;
}

struct ArenaDesign
{
    Referable<rtl::Module> parent;
    Referable<rtl::Module> primitive_module;
    Referable<rtl::Cell> cell;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    std::vector<std::unique_ptr<Referable<rtl::Net>>> nets;

    ArenaDesign()
    {
        parent.name = "arena_top";
        parent.is_blackbox = false;
        primitive_module.name = "arena_primitive";
        primitive_module.is_blackbox = true;
        primitive_module.parent_ref.set(&parent);
        cell.name = "arena_cell";
        cell.type = "LUT6";
        cell.module_ref.set(&primitive_module);

        rtl::Port out;
        out.name = "O";
        out.type = rtl::Port::PORT_OUT;
        out.designator = -1;
        cell.ports.emplace_back(std::move(out));

        rtl::Port in;
        in.name = "I0";
        in.type = rtl::Port::PORT_IN;
        in.designator = -1;
        cell.ports.emplace_back(std::move(in));
    }

    Referable<rtl::Inst>* makeInst(const std::string& name, fpga::Tile& tile)
    {
        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(&cell);
        inst->pos = 0;
        inst->cnt_inputs = 1;
        inst->cnt_outputs = 1;
        inst->conns.reserve(cell.ports.size());
        for (auto& port : cell.ports) {
            auto& conn = inst->conns.emplace_back();
            conn.port_ref.set(&port);
            conn.inst_ref.set(inst.get());
        }
        tile.assign(inst.get());
        inst->coord = tile.coord;
        Referable<rtl::Inst>* raw = inst.get();
        insts.push_back(std::move(inst));
        (void)name;
        return raw;
    }

    rtl::Net* makeNet(const std::string& name)
    {
        auto net = std::make_unique<Referable<rtl::Net>>();
        net->name = name;
        rtl::Net* raw = net.get();
        nets.push_back(std::move(net));
        return raw;
    }
};

std::vector<fpga::Tile*> resetArenaGrid(int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.tile_types.clear();
    device.local_route_wire_mappings.clear();
    device.route_wire_graph.clear();
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;

    device.cb_types.push_back(makeArenaCrossbar());
    fpga::CBType* cb = &device.cb_types.front();
    device.tile_grid.resize(static_cast<size_t>(width * height));

    std::vector<fpga::Tile*> tiles;
    tiles.reserve(device.tile_grid.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y * width + x)];
            tile = {};
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = {x, y};
            tile.type = (x == 0 || y == 0 || x == width - 1 || y == height - 1)
                ? fpga::Tile::TILE_IO
                : fpga::Tile::TILE_LUTS;
            tile.cb_type = cb;
            tile.cb.type = cb;
            tiles.push_back(&tile);
        }
    }
    return tiles;
}

void addReferenceStateLoad(const std::vector<fpga::Tile*>& tiles, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> percent(0, 99);
    std::vector<int> lanes{0, 1, 2, 3};
    std::vector<fpga::Coord> deltas{{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    for (fpga::Tile* tile : tiles) {
        if (!tile || tile->type == fpga::Tile::TILE_IO) {
            continue;
        }
        if (percent(rng) < 16) {
            fpga::Coord delta = deltas[static_cast<size_t>(percent(rng)) % deltas.size()];
            int src = encodeJump(delta.x, delta.y, lanes[static_cast<size_t>(percent(rng)) % lanes.size()]);
            tile->cb.src.jump |= bit(src);
        }
        if (percent(rng) < 10) {
            fpga::Coord delta = deltas[static_cast<size_t>(percent(rng)) % deltas.size()];
            int dst = encodeJump(delta.x, delta.y, lanes[static_cast<size_t>(percent(rng)) % lanes.size()]);
            tile->cb.dst.jump |= bit(dst);
        }
        if (percent(rng) < 4) {
            tile->cb.local.local |= bit(17);
        }
    }
}

std::vector<pnr::RouteDesign::RouteTask> makeArenaTasks(ArenaDesign& design, int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    tasks.reserve(50);
    for (int i = 0; i < 50; ++i) {
        int side = i % 4;
        int offset = 1 + ((i * 7) % (width - 2));
        fpga::Coord src_coord;
        fpga::Coord dst_coord;
        if (side == 0) {
            src_coord = {offset, 0};
            dst_coord = {width - 1 - offset, height - 1};
        }
        else if (side == 1) {
            src_coord = {width - 1, offset};
            dst_coord = {0, height - 1 - offset};
        }
        else if (side == 2) {
            src_coord = {width - 1 - offset, height - 1};
            dst_coord = {offset, 0};
        }
        else {
            src_coord = {0, height - 1 - offset};
            dst_coord = {width - 1, offset};
        }
        fpga::Tile* src_tile = device.getTile(src_coord.x, src_coord.y);
        fpga::Tile* dst_tile = device.getTile(dst_coord.x, dst_coord.y);
        require(src_tile && dst_tile, "arena endpoint tile missing");
        Referable<rtl::Inst>* src = design.makeInst("arena_src_" + std::to_string(i), *src_tile);
        Referable<rtl::Inst>* dst = design.makeInst("arena_dst_" + std::to_string(i), *dst_tile);
        rtl::Net* net = design.makeNet("arena_net_" + std::to_string(i));
        tasks.push_back(pnr::RouteDesign::RouteTask{
            .from = src,
            .to = dst,
            .net = net,
            .from_port = "O",
            .to_port = "I0",
            .net_name = net->name,
        });
    }
    return tasks;
}

std::string taskSummary(const std::vector<pnr::RouteDesign::RouteTask>& tasks)
{
    std::ostringstream out;
    size_t limit = std::min<size_t>(tasks.size(), 8);
    for (size_t i = 0; i < limit; ++i) {
        const auto& task = tasks[i];
        if (i != 0) {
            out << "; ";
        }
        out << task.net_name << " "
            << "(" << task.from->tile->coord.x << "," << task.from->tile->coord.y << ")"
            << "->"
            << "(" << task.to->tile->coord.x << "," << task.to->tile->coord.y << ")"
            << " attempt=" << task.attempt;
    }
    if (tasks.size() > limit) {
        out << "; ...";
    }
    return out.str();
}

std::string routeTailSummary(const pnr::RouteDesign::RouteTask& task)
{
    std::ostringstream out;
    const std::vector<fpga::Wire>* route = nullptr;
    if (task.to) {
        for (const std::vector<fpga::Wire>& candidate : task.to->wires) {
            bool matches = std::any_of(candidate.begin(), candidate.end(), [&](const fpga::Wire& wire) {
                return wire.net_name == task.net_name;
            });
            if (matches) {
                route = &candidate;
                break;
            }
        }
    }
    if (!route) {
        return "route=none";
    }
    out << "route_size=" << route->size();
    size_t first = route->size() > 12 ? route->size() - 12 : 0;
    for (size_t i = first; i < route->size(); ++i) {
        const fpga::Wire& wire = (*route)[i];
        if (wire.type != fpga::Wire::WIRE_CROSSBAR) {
            continue;
        }
        out << " [" << i << ":"
            << wire.from.x << "," << wire.from.y
            << "->" << wire.to.x << "," << wire.to.y
            << " dst=" << wire.dst
            << " src=" << wire.jump
            << " local=" << wire.local
            << "]";
    }
    return out.str();
}

void generic_arena_routes_reference_load()
{
    constexpr int width = 20;
    constexpr int height = 20;
    std::vector<fpga::Tile*> tiles = resetArenaGrid(width, height);
    addReferenceStateLoad(tiles, 0x5eed);

    ArenaDesign design;
    std::vector<pnr::RouteDesign::RouteTask> tasks = makeArenaTasks(design, width, height);

    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.fpga_width = width;
    router.fpga_height = height;
    router.iteration_limit = 5;

    std::vector<std::string> pass_log;
    for (int pass = 1; pass <= 120 && !tasks.empty(); ++pass) {
        router.route_stats.clear();
        pnr::RouteDesign::RouteBatchResult result =
            router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Generic, tasks, tasks.size(), 5);
        pass_log.push_back(std::format(
            "pass {}: {} -> {}, completed={}, active={}, advanced={}, changed={}, searches={}, pops={}, accepted={}, busy={}, deadend={}",
            pass, result.before, result.after, result.completed, result.active, result.advanced, result.changed,
            router.route_stats.route_searches, router.route_stats.search_pops,
            router.route_stats.edge_accepted, router.route_stats.edge_rejected_busy,
            router.route_stats.edge_rejected_deadend));
        if (result.completed == 0 && result.advanced == 0 && result.changed == 0) {
            break;
        }
    }

    if (!tasks.empty()) {
        std::fprintf(stderr, "arena routing left %zu routes unfinished\n", tasks.size());
        for (const std::string& line : pass_log) {
            std::fprintf(stderr, "%s\n", line.c_str());
        }
        std::fprintf(stderr, "unfinished: %s\n", taskSummary(tasks).c_str());
        for (const auto& task : tasks) {
            std::fprintf(stderr, "unfinished route: %s %s\n",
                task.net_name.c_str(), routeTailSummary(task).c_str());
        }
    }
    require(tasks.empty(), "generic arena routing did not finish all 50 reference routes");
}

} // namespace

int main()
{
    try {
        generic_arena_routes_reference_load();
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "arena_test failure: %s\n", failure.message.c_str());
        return EXIT_FAILURE;
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "arena_test exception: %s\n", error.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
