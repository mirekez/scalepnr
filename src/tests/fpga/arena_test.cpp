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
    return (encode(dx) << 8) | (encode(dy) << 4) | (lane & 0xf);
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
            for (fpga::Coord src_delta : deltas) {
                for (int src_lane = 0; src_lane < 4; ++src_lane) {
                    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC,
                        encodeJump(src_delta.x, src_delta.y, src_lane));
                }
            }
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
    std::vector<fpga::Coord> perimeter;
    for (int x = 0; x < width; ++x) {
        perimeter.push_back({x, 0});
    }
    for (int y = 1; y < height; ++y) {
        perimeter.push_back({width - 1, y});
    }
    for (int x = width - 2; x >= 0; --x) {
        perimeter.push_back({x, height - 1});
    }
    for (int y = height - 2; y > 0; --y) {
        perimeter.push_back({0, y});
    }
    require(perimeter.size() >= 50, "arena perimeter cannot provide unique route endpoints");

    std::vector<pnr::RouteDesign::RouteTask> tasks;
    tasks.reserve(50);
    for (int i = 0; i < 50; ++i) {
        fpga::Coord src_coord = perimeter[static_cast<size_t>(i)];
        fpga::Coord dst_coord = perimeter[(static_cast<size_t>(i) + perimeter.size() / 2) % perimeter.size()];
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

void routing_retries_parent_after_blocked_docking_endpoint()
{
    // A legal ordinary exit must not hide an already reachable docking rail
    // in any stage. This reproduces a five-hop Basic suffix whose distances
    // enter radius 5 at depth 3, then leave it again before depth 5.
    std::vector<int> suffix_distances{15, 11, 2, 5, 6};
    std::vector<int> retained_depths;
    for (size_t index = 0; index < suffix_distances.size(); ++index) {
        int depth = static_cast<int>(index) + 1;
        if (pnr::rememberDockingCandidate(depth, suffix_distances[index], 5)) {
            retained_depths.push_back(depth);
        }
    }
    require(retained_depths == std::vector<int>({3, 4}),
        "routing lost an in-radius docking rail after the suffix moved away");
    require(!pnr::rememberDockingCandidate(0, 4, 5)
            && !pnr::rememberDockingCandidate(1, 6, 5),
        "routing retained a source local or an out-of-radius docking rail");

    auto run_mode = [&](pnr::RouteDesign::RouteTaskMode mode, bool moving) {
        constexpr int width = 10;
        constexpr int height = 9;
        resetArenaGrid(width, height);
        fpga::Device& device = fpga::Device::current();

        fpga::Tile* source_tile = device.getTile(2, 4);
        fpga::Tile* blocked_tile = device.getTile(3, 3);
        fpga::Tile* target_tile = device.getTile(7, 4);
        require(source_tile && blocked_tile && target_tile,
            "blocked-docking arena tiles are missing");

        // The first angle-priority hop reaches a saturated docking candidate.
        // Another source bit at its parent remains free and must be tried in
        // the same bounded search instead of committing the blocked child.
        for (fpga::Coord delta : std::vector<fpga::Coord>{
                 {0, -1}, {1, -1}, {1, 0}, {1, 1},
                 {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}}) {
            for (int lane = 0; lane < 4; ++lane) {
                blocked_tile->cb.src.jump |= bit(encodeJump(delta.x, delta.y, lane));
            }
        }
        const int busy_target_dst = encodeJump(1, 0, 0);
        target_tile->cb.dst.jump |= bit(busy_target_dst);

        ArenaDesign design;
        Referable<rtl::Inst>* source = design.makeInst("blocked_docking_source", *source_tile);
        Referable<rtl::Inst>* target = design.makeInst("blocked_docking_target", *target_tile);
        rtl::Net* net = design.makeNet("blocked_docking_alternate_entry");
        std::vector<pnr::RouteDesign::RouteTask> tasks{
            pnr::RouteDesign::RouteTask{
                .from = source,
                .to = target,
                .net = net,
                .from_port = "O",
                .to_port = "I0",
                .net_name = net->name,
            }
        };

        pnr::RouteDesign router;
        router.fpga = &device;
        router.fpga_width = width;
        router.fpga_height = height;
        router.iteration_limit = 5;
        router.moving_stage = moving;
        std::vector<std::string> retry_log;
        for (int pass = 0; pass < 8 && !tasks.empty(); ++pass) {
            pnr::RouteDesign::RouteBatchResult result =
                router.routeTaskBatch(mode, tasks, tasks.size(), 5);
            tasks.erase(std::remove_if(tasks.begin(), tasks.end(),
                [](const pnr::RouteDesign::RouteTask& task) {
                    return task.remove_after_pass;
                }), tasks.end());
            retry_log.push_back(std::format(
                "{} pass {}: {} -> {}, completed={}, advanced={}, changed={}, {}",
                moving ? "Moving" : "Generic", pass + 1, result.before,
                result.after, result.completed, result.advanced, result.changed,
                tasks.empty() ? "route=complete" : routeTailSummary(tasks.front())));
        }

        // A blocked docking endpoint is only a failed candidate. Basic and
        // Moving must return to its parent and finish through another exit.
        if (!tasks.empty()) {
            for (const std::string& line : retry_log) {
                std::fprintf(stderr, "%s\n", line.c_str());
            }
        }
        require(tasks.empty(), std::format(
            "{} routing committed the first blocked docking endpoint instead of retrying its parent",
            moving ? "Moving" : "Generic"));
        require(!target->wires.empty() && !target->wires.front().empty(),
            "alternate target entry produced no routed wire");
        bool used_blocked_tile = std::any_of(target->wires.front().begin(), target->wires.front().end(),
            [&](const fpga::Wire& wire) {
                return wire.type == fpga::Wire::WIRE_CROSSBAR
                    && ((wire.from.x == blocked_tile->coord.x && wire.from.y == blocked_tile->coord.y)
                        || (wire.to.x == blocked_tile->coord.x && wire.to.y == blocked_tile->coord.y));
            });
        require(!used_blocked_tile,
            "routing retained the blocked endpoint prefix after selecting an alternate target entry");
        const fpga::Wire* target_entry = nullptr;
        for (const fpga::Wire& wire : target->wires.front()) {
            if (wire.type == fpga::Wire::WIRE_CROSSBAR
                && wire.to.x == target_tile->coord.x && wire.to.y == target_tile->coord.y) {
                target_entry = &wire;
            }
        }
        require(target_entry && target_entry->dst != busy_target_dst,
            "routing did not select one of the free alternate destination entries");
        require(target->wires.front().back().type == fpga::Wire::WIRE_TILE_PIN,
            "alternate target entry did not ground at the destination pin");
    };

    run_mode(pnr::RouteDesign::RouteTaskMode::Generic, false);
    run_mode(pnr::RouteDesign::RouteTaskMode::Moving, true);
}

void fanout_tries_low_capacity_fork_after_preferred_forks_fail(int free_exits)
{
    constexpr int width = 7, height = 7;
    auto tiles = resetArenaGrid(width, height);
    auto& device = fpga::Device::current();
    auto* source_tile = device.getTile(2, 2);
    auto* fork_tile = device.getTile(3, 2);
    auto* seed_tile = device.getTile(4, 2);
    auto* sink_tile = device.getTile(3, 3);
    const int east = encodeJump(1, 0, 0);

    // Every other exit is occupied. The trunk's last landing has three free
    // exits, all ending in saturated tiles; its earlier landing has one or two
    // free exits, which reach the requested sink directly.
    const NodeMask all_srcs = source_tile->cb_type->local_src[16].jump;
    for (auto* tile : tiles) tile->cb.src.jump = all_srcs;
    for (int lane = 0; lane < free_exits; ++lane)
        fork_tile->cb.src.jump &= ~bit(encodeJump(0, 1, lane));
    for (int lane = 0; lane < 3; ++lane)
        seed_tile->cb.src.jump &= ~bit(encodeJump(0, -1, lane));
    source_tile->cb.local.local |= bit(16);
    source_tile->pin_state.leased_nodes |= bit(16);
    fork_tile->cb.dst.jump |= bit(east);
    seed_tile->cb.dst.jump |= bit(east);
    seed_tile->cb.local.local |= bit(17);
    seed_tile->pin_state.leased_nodes |= bit(17);

    ArenaDesign design;
    auto* source = design.makeInst("fork_source", *source_tile);
    auto* seed = design.makeInst("fork_seed", *seed_tile);
    auto* sink = design.makeInst("fork_sink", *sink_tile);
    auto* net = design.makeNet("fork_capacity_is_not_reachability");
    auto& trunk = seed->wires.emplace_back();
    fpga::Wire pin;
    pin.type = fpga::Wire::WIRE_TILE_PIN;
    pin.from = pin.to = source_tile->coord;
    pin.local = 16;
    trunk.push_back(pin);
    fpga::Wire hop;
    hop.from = source_tile->coord;
    hop.to = fork_tile->coord;
    hop.local = 16;
    hop.jump = hop.dst = east;
    hop.pos = 0;
    trunk.push_back(hop);
    hop.from = fork_tile->coord;
    hop.to = seed_tile->coord;
    hop.local = east;
    hop.pos = 1;
    trunk.push_back(hop);
    fpga::Wire grounding;
    grounding.from = grounding.to = seed_tile->coord;
    grounding.local = east;
    grounding.pos = 1;
    trunk.push_back(grounding);
    pin.from = pin.to = seed_tile->coord;
    pin.local = 17;
    trunk.push_back(pin);
    require(fpga::isRouteComplete(trunk), "fanout fixture has no complete trunk");
    auto binding = fpga::attachNetRoute(*net, *seed, 0, source, seed,
                                       "O", "I0", net->name);
    fpga::registerNetRouteTiles(*net, trunk, binding);

    pnr::RouteDesign router;
    router.fpga = &device;
    router.fpga_width = width;
    router.fpga_height = height;
    router.iteration_limit = 5;
    router.route_preemption_enabled = false;
    router.indexSourceRoute(net, source, "O");
    pnr::RouteDesign::RouteTask task{
        .from = source, .to = sink, .net = net,
        .from_port = "O", .to_port = "I0", .net_name = net->name,
        .fanout = true};
    router.route_recursion_budget = 5;
    require(router.routeFanoutTask(task),
        "Fanout discarded the reachable low-capacity fork because a three-exit fork existed");
    require(sink->wires.size() == 1 && fpga::isRouteComplete(sink->wires.front()),
        "low-capacity fanout did not complete at the sink");
    const auto& route = sink->wires.front();
    require(std::any_of(route.begin(), route.end(), [&](const fpga::Wire& fragment) {
        return !fragment.shared && fragment.jump >= 0
            && fragment.from.x == 3 && fragment.from.y == 2
            && fragment.to.x == 3 && fragment.to.y == 3;
    }), "Fanout did not use the only free direct fork");
    require(fpga::isRouteComplete(trunk) && fork_tile->cb.dst.jump.testBit(east),
        "Fanout damaged the shared trunk");
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
        tasks.erase(std::remove_if(tasks.begin(), tasks.end(),
            [](const pnr::RouteDesign::RouteTask& task) {
                return task.remove_after_pass;
            }), tasks.end());
        pass_log.push_back(std::format(
            "pass {}: {} -> {}, completed={}, active={}, advanced={}, changed={}, searches={}, pops={}, trials={}, accepted={}, no_src={}, no_name={}, no_target={}, busy={}, deadend={}",
            pass, result.before, result.after, result.completed, result.active, result.advanced, result.changed,
            router.route_stats.route_searches, router.route_stats.search_pops,
            router.route_stats.edge_trials, router.route_stats.edge_accepted,
            router.route_stats.no_src_nodes, router.route_stats.edge_rejected_no_name,
            router.route_stats.edge_rejected_no_target, router.route_stats.edge_rejected_busy,
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
        fanout_tries_low_capacity_fork_after_preferred_forks_fail(1);
        fanout_tries_low_capacity_fork_after_preferred_forks_fail(2);
        routing_retries_parent_after_blocked_docking_endpoint();
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
