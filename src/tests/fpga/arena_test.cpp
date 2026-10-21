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
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_set>
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

void nearby_source_routes_by_reverse_grounding()
{
    resetArenaGrid(9, 5);
    auto& device = fpga::Device::current();
    auto& source_tile = *device.getTile(1, 2);
    auto& target_tile = *device.getTile(6, 2);
    ArenaDesign design;
    auto* source = design.makeInst("short_source", source_tile);
    auto* target = design.makeInst("short_sink", target_tile);
    auto* net = design.makeNet("short_reverse_route");
    // LOCAL16 and DST16 are distinct. Takeoff must not lease this DST.
    source_tile.cb.dst.jump = bit(16);
    const auto old_dsts = source_tile.cb.dst.jump;
    std::vector<pnr::RouteDesign::RouteTask> tasks{
        {.from = source, .to = target, .net = net,
         .from_port = "O", .to_port = "I0", .net_name = net->name}};
    pnr::RouteDesign router;
    router.fpga = &device;
    router.fpga_width = 9;
    router.fpga_height = 5;
    auto result = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Generic,
                                        tasks, tasks.size(), 5);
    require(result.completed == 1 && router.route_stats.search_pops == 1 &&
                router.route_stats.edge_trials == 0,
            "nearby source used forward routing instead of immediate Grounding");
    require(!target->wires.empty() && fpga::isRouteComplete(target->wires.front()),
            "backwards-only short route did not commit a complete connection");
    require(source_tile.cb.dst.jump == old_dsts && source_tile.cb.src.jump != NodeMask{},
            "Grounding committed its source LOCAL as a transit DST");
}

void routing_retries_parent_after_blocked_docking_endpoint()
{
    require(pnr::shouldTryDocking(3, 2, 5) && pnr::shouldTryDocking(4, 5, 5),
        "routing did not try Docking as soon as it entered the window");
    require(pnr::shouldTryDocking(0, 4, 5) && pnr::shouldTryDocking(0, 5, 5) &&
            !pnr::shouldTryDocking(0, 6, 5) && !pnr::shouldTryDocking(1, 6, 5),
        "routing skipped nearby source-local Docking or tried outside the window");

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
        // A rejected child is not the final failure when another branch wins.
        require(router.route_stats.exhausted_coord.x < 0,
            "successful alternate branch retained a discarded failure location");
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

// Reproduce a high-fanout output consuming a new Takeoff for every suffix,
// despite a connected trunk with free downstream forks. All nodes use the
// abstract arena mesh. Success must spread forks, not merely move the star
// from the source to one downstream Tile.
std::string fanout_spreads_over_connected_tiles()
{
    resetArenaGrid(15, 10);
    auto& device = fpga::Device::current();
    ArenaDesign design;
    auto* source = design.makeInst("spread_source", *device.getTile(2, 2));
    auto* seed = design.makeInst("spread_seed", *device.getTile(5, 2));
    auto* net = design.makeNet("spread_signal");
    const int east = encodeJump(1, 0, 0);
    fpga::Wire pin;
    pin.type = fpga::Wire::WIRE_TILE_PIN;
    pin.from = pin.to = pin.resource = source->tile->coord;
    pin.cell_type = "LUT6";
    pin.port = "O";
    pin.pos = source->pos;
    pin.local = 16;
    pin.pin_dir = fpga::TILE_PIN_OUTPUT;
    auto& trunk = seed->wires.emplace_back();
    trunk.push_back(pin);
    for (int x = 2; x < 5; ++x) {
        fpga::Wire hop;
        hop.from = device.getTile(x, 2)->coord;
        hop.to = device.getTile(x + 1, 2)->coord;
        hop.local = x == 2 ? 16 : east;
        hop.jump = hop.dst = east;
        hop.pos = x == 2 ? 0 : 1;
        trunk.push_back(hop);
    }
    fpga::Wire terminal;
    terminal.from = terminal.to = seed->tile->coord;
    terminal.local = east;
    terminal.pos = 1;
    trunk.push_back(terminal);
    pin.from = pin.to = pin.resource = seed->tile->coord;
    pin.port = "I0";
    pin.local = 17;
    pin.pin_dir = fpga::TILE_PIN_INPUT;
    trunk.push_back(pin);
    require(pnr::commitPreparedRoute(trunk), "spread fixture failed to lease trunk");
    const auto binding = fpga::attachNetRoute(*net, *seed, 0, source, seed,
                                             "O", "I0", "spread_trunk");
    fpga::registerNetRouteTiles(*net, trunk, binding);
    pnr::RouteDesign router;
    router.fpga = &device;
    router.fpga_width = 15;
    router.fpga_height = 10;
    router.iteration_limit = 30;
    router.fanout_stage = router.complete_routes_only = true;
    router.route_preemption_enabled = false;
    router.indexSourceRoute(net, source, "O");
    std::vector<fpga::Tile*> fork_tiles;
    std::ostringstream route_signature;
    for (int i = 0; i < 13; ++i) {
        auto* sink = design.makeInst(std::format("spread_sink_{}", i),
                                    *device.getTile(7 + i % 4, 3 + i / 4));
        pnr::RouteDesign::RouteTask task{
            .from = source, .to = sink, .net = net,
            .from_port = "O", .to_port = "I0",
            .net_name = std::format("spread_suffix_{}", i),
            .fanout = true, .endpoints_prepared = true};
        // Pass boundaries and preemption invalidate this acceleration index;
        // rebuilding it must not forget where completed suffixes branched.
        if (i % 2 == 0) router.fanout_branch_indexes.clear();
        router.route_recursion_budget = 30;
        require(router.routeFanoutTask(task), "spread suffix failed to route");
        const auto& route = sink->wires.front();
        for (const auto& edge : route) {
            route_signature << edge.type << ':' << edge.from.x << ',' << edge.from.y
                << '>' << edge.to.x << ',' << edge.to.y << ':' << edge.local
                << ':' << edge.jump << ':' << edge.dst << ':' << edge.joint
                << ':' << edge.joint2 << ':' << edge.pos << ':' << edge.shared
                << ':' << edge.owns_dst << ':' << edge.owns_landing << ';';
        }
        route_signature << '\n';
        require(fpga::isRouteComplete(route), "spread suffix is incomplete");
        require(route.size() > 1 && route[1].shared && route[1].jump == east &&
                route[1].from.x == 2 && route[1].from.y == 2,
                "spread suffix did not retain the original shared Takeoff");
        auto fork = std::find_if(route.begin(), route.end(), [](const auto& w) {
            return w.type == fpga::Wire::WIRE_CROSSBAR && !w.shared;
        });
        require(fork != route.end(), "spread suffix has no private connection");
        auto* tile = device.getTile(fork->from.x, fork->from.y);
        fork_tiles.push_back(tile);
        std::printf("suffix %d: fork (%d,%d), position %d\n",
                    i, fork->from.x, fork->from.y, fork->pos);
    }
    require(source->tile->cb.src.jump == bit(east),
            "high-fanout tree consumed additional source Takeoffs despite usable downstream forks");
    for (int i = 0; i < 3; ++i)
        require(fork_tiles[i] == device.getTile(3 + i, 2),
                "successive suffixes did not advance through trunk Tiles");
    std::unordered_set<fpga::Tile*> distinct(fork_tiles.begin(), fork_tiles.end());
    require(distinct.size() == fork_tiles.size(),
            "fanouts clustered on one Tile instead of extending the connected tree");
    require(fpga::isRouteComplete(trunk), "spreading damaged the trunk");
    route_signature << router.route_stats.search_pops << ':'
        << router.route_stats.edge_trials << ':' << router.route_stats.edge_accepted;
    return route_signature.str();
}

void fanout_exit_classification_preserves_routing()
{
    const char* previous = std::getenv("SCALEPNR_DEBUG_FANOUT_NET");
    const std::string saved = previous ? previous : "";
    const bool had_previous = previous != nullptr;
    unsetenv("SCALEPNR_DEBUG_FANOUT_NET");
    const auto classified = fanout_spreads_over_connected_tiles();
    // Detailed diagnostics request an exact count of every available exit.
    // The ordinary classifier must select the same forks AND complete paths.
    setenv("SCALEPNR_DEBUG_FANOUT_NET", "spread_suffix", 1);
    const auto exact = fanout_spreads_over_connected_tiles();
    if (had_previous) setenv("SCALEPNR_DEBUG_FANOUT_NET", saved.c_str(), 1);
    else unsetenv("SCALEPNR_DEBUG_FANOUT_NET");
    require(classified == exact,
            "early exit classification changed fanout paths or search counters");
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

// A completed one-hop trunk must not hide either endpoint Tile from Fanouts:
// the driver can take another exit, and the landing can ground without any
// outgoing exit. The occupied controls must leave the original tree intact.
void fanout_search_includes_endpoint_tiles(bool source_start)
{
    if (std::getenv("SCALEPNR_DEBUG_FANOUT_NET")) debug_level = 1;
    for (int scenario = 0; scenario < 3; ++scenario) {
        const bool blocked = scenario == 1;
        auto tiles = resetArenaGrid(7, 7);
        auto& device = fpga::Device::current();
        auto& cb = device.cb_types.front();
        auto* source_tile = device.getTile(2, 2);
        auto* seed_tile = device.getTile(3, 2);
        auto* sink_tile = source_start ? device.getTile(2, 3) : seed_tile;
        const int east = encodeJump(1, 0, 0);
        const int south = encodeJump(0, 1, 1);
        constexpr int terminal_joint = 8;
        if (!source_start) {
            cb.rememberNodeName(fpga::CB_NODE_LOCAL, 18, "node_593");
            cb.rememberNodeName(fpga::CB_NODE_JOINT, terminal_joint, "node_827");
            cb.local_input_nodes |= bit(18);
            cb.dst_joint[east].joint |= bit(terminal_joint);
            cb.joint_local[terminal_joint].local |= bit(18);
            rememberConn(cb, fpga::CB_NODE_DST, east, fpga::CB_NODE_JOINT, terminal_joint);
            rememberConn(cb, fpga::CB_NODE_JOINT, terminal_joint, fpga::CB_NODE_LOCAL, 18);
            cb.rebuildOutgoingSrcs();
            cb.ensureDerivedMasks();
        }
        ArenaDesign design;
        auto& input = design.cell.ports.emplace_back();
        input.name = "I1";
        input.type = rtl::Port::PORT_IN;
        input.designator = -1;
        auto* source = design.makeInst("source_719", *source_tile);
        auto* seed = design.makeInst("sink_431", *seed_tile);
        auto* sink = design.makeInst("sink_683", *sink_tile);
        auto* net = design.makeNet("signal_947");
        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = pin.to = source_tile->coord;
        pin.resource = source_tile->coord;
        pin.cell_type = "LUT6";
        pin.port = "O";
        pin.pos = source->pos;
        pin.local = 16;
        pin.pin_dir = fpga::TILE_PIN_OUTPUT;
        fpga::Wire hop;
        hop.from = source_tile->coord; hop.to = seed_tile->coord;
        hop.local = 16; hop.jump = hop.dst = east; hop.pos = 0;
        fpga::Wire terminal;
        terminal.from = terminal.to = seed_tile->coord;
        terminal.local = east; terminal.pos = 1;
        auto& trunk = seed->wires.emplace_back(std::vector<fpga::Wire>{pin, hop, terminal});
        pin.from = pin.to = seed_tile->coord;
        pin.resource = seed_tile->coord;
        pin.port = "I0";
        pin.local = 17; pin.pin_dir = fpga::TILE_PIN_INPUT;
        trunk.push_back(pin);
        require(pnr::commitPreparedRoute(trunk), "endpoint fixture could not lease its trunk");
        const auto binding = fpga::attachNetRoute(*net, *seed, 0, source, seed,
                                                 "O", "I0", "seed_947");
        fpga::registerNetRouteTiles(*net, trunk, binding);
        source_tile->cb.local.local |= bit(16);
        const NodeMask all_srcs = cb.local_src[16].jump;
        for (auto* tile : tiles) tile->cb.src.jump = all_srcs;
        if (source_start && !blocked) source_tile->cb.src.jump &= ~bit(south);
        if (!source_start && blocked) seed_tile->cb.joint.jump |= bit(terminal_joint);
        std::vector<fpga::CBState> before;
        std::vector<NodeMask> pins_before;
        for (auto* tile : tiles) {
            before.push_back(tile->cb);
            pins_before.push_back(tile->pin_state.leased_nodes);
        }
        pnr::RouteDesign router;
        router.fpga = &device; router.fpga_width = router.fpga_height = 7;
        router.iteration_limit = router.route_recursion_budget = 5;
        router.fanout_stage = router.complete_routes_only = true;
        router.route_preemption_enabled = false;
        router.indexSourceRoute(net, source, "O");
        pnr::RouteDesign::RouteTask task{
            .from = source, .to = sink, .net = net,
            .from_port = "O", .to_port = source_start ? "I0" : "I1",
            .net_name = "suffix_947", .fanout = true, .endpoints_prepared = true};
        const bool routed = router.routeFanoutTask(task);
        require(routed == !blocked, std::format(
            "Fanout omitted endpoint Tile: source_start={} blocked={} routed={}",
            source_start, blocked, routed));
        require(fpga::isRouteComplete(trunk) && trunk.size() == 4 &&
                !trunk.front().shared && source_tile->cb.src.jump.testBit(east) &&
                seed_tile->cb.dst.jump.testBit(east) && seed_tile->isPinNodeLeased(17),
                "endpoint fanout damaged the completed trunk");
        if (routed) {
            require(sink->wires.size() == 1 && fpga::isRouteComplete(sink->wires.front()),
                    "endpoint fanout did not commit a complete suffix");
            const auto& route = sink->wires.front();
            require(route.front().shared && route.front().local == 16 &&
                    route.back().local == (source_start ? 17 : 18),
                    "endpoint fanout lost its shared driver or destination pin");
            require(std::count_if(route.begin(), route.end(), [](const fpga::Wire& w) {
                return w.type == fpga::Wire::WIRE_CROSSBAR && w.jump >= 0;
            }) == 1, "endpoint fanout inserted an unnecessary detour");
            if (scenario == 2) {
                require(fpga::unrouteNetRoute(*net, 0), "could not remove original trunk");
                require(fpga::isRouteComplete(route) &&
                        source_tile->cb.src.jump.testBit(source_start ? south : east) &&
                        sink_tile->isPinNodeLeased(source_start ? 17 : 18),
                        "trunk removal damaged the surviving endpoint branch");
                require(fpga::unrouteNetRoute(*net, 1), "could not remove surviving branch");
                require(!source_tile->cb.src.jump.testBit(east) &&
                        !source_tile->cb.local.local.testBit(16) &&
                        !source_tile->isPinNodeLeased(16) &&
                        !seed_tile->cb.dst.jump.testBit(east) &&
                        !sink_tile->isPinNodeLeased(source_start ? 17 : 18) &&
                        (!source_start || !source_tile->cb.src.jump.testBit(south)),
                        "last branch removal leaked endpoint routing resources");
            } else {
                require(fpga::discardNetBranch(*net, 1), "could not remove endpoint suffix");
                require(fpga::isRouteComplete(trunk) && seed_tile->cb.dst.jump.testBit(east) &&
                        seed_tile->isPinNodeLeased(17), "suffix removal released trunk resources");
                for (size_t i = 0; i < tiles.size(); ++i)
                    require(tiles[i]->cb.src.jump == before[i].src.jump &&
                            tiles[i]->cb.dst.jump == before[i].dst.jump &&
                            tiles[i]->cb.joint.jump == before[i].joint.jump &&
                            tiles[i]->cb.local.local == before[i].local.local &&
                            tiles[i]->pin_state.leased_nodes == pins_before[i],
                            "suffix removal did not restore the original Tile state");
            }
        } else {
            require(sink->wires.empty(), "failed endpoint attempt retained an incomplete suffix");
            for (size_t i = 0; i < tiles.size(); ++i)
                require(tiles[i]->cb.src.jump == before[i].src.jump &&
                        tiles[i]->cb.dst.jump == before[i].dst.jump &&
                        tiles[i]->cb.joint.jump == before[i].joint.jump &&
                        tiles[i]->cb.local.local == before[i].local.local &&
                        tiles[i]->pin_state.leased_nodes == pins_before[i],
                        "failed endpoint attempt changed live leases");
        }
    }
}

void failed_uncommitted_hop_reports_downstream_congestion()
{
    constexpr int width = 22, height = 7;
    resetArenaGrid(width, height);
    auto& device = fpga::Device::current();
    auto* source_tile = device.getTile(2, 3);
    auto* tail_tile = device.getTile(3, 3);
    auto* blocked_tile = device.getTile(4, 3);
    auto* sink_tile = device.getTile(20, 3);
    const int east = encodeJump(1, 0, 0);
    const NodeMask all_srcs = tail_tile->cb_type->local_src[16].jump;

    // The committed trunk ends at A; its only free exit reaches B, where
    // every outgoing source is filtered as occupied before lease attempts.
    source_tile->cb.local.local = bit(16);
    source_tile->pin_state.leased_nodes = bit(16);
    source_tile->cb.src.jump = bit(east);
    tail_tile->cb.dst.jump = bit(east);
    tail_tile->cb.src.jump = all_srcs & ~bit(east);
    blocked_tile->cb.src.jump = all_srcs;
    ArenaDesign design;
    auto* source = design.makeInst("diagnostic_driver", *source_tile);
    auto* sink = design.makeInst("diagnostic_sink", *sink_tile);
    auto* net = design.makeNet("blocked_uncommitted_suffix");
    auto& route = sink->wires.emplace_back();
    fpga::Wire pin;
    pin.type = fpga::Wire::WIRE_TILE_PIN;
    pin.from = pin.to = source_tile->coord;
    pin.local = 16;
    route.push_back(pin);
    fpga::Wire hop;
    hop.from = source_tile->coord;
    hop.to = tail_tile->coord;
    hop.local = 16;
    hop.jump = hop.dst = east;
    hop.pos = 0;
    route.push_back(hop);
    auto binding = fpga::attachNetRoute(*net, *sink, 0, source, sink,
                                       "O", "I0", net->name);
    fpga::registerNetRouteTiles(*net, route, binding);

    pnr::RouteDesign router;
    router.fpga = &device;
    router.fpga_width = width;
    router.fpga_height = height;
    router.iteration_limit = 5;
    router.route_recursion_budget = 5;
    router.route_preemption_enabled = false;
    router.route_deadends_enabled = false;
    router.moving_stage = true;
    pnr::RouteDesign::RouteTask task{
        .from = source, .to = sink, .net = net,
        .from_port = "O", .to_port = "I0", .net_name = net->name,
        .endpoints_prepared = true};

    require(!router.routeNetTask(task), "saturated downstream tile unexpectedly routed");
    // Check the real search, not a fabricated statistic: the free hop was tried.
    require(router.route_stats.edge_accepted > 0,
        "regression did not reach the downstream tile");
    require(!router.route_stats.has_last_no_src && !router.route_stats.has_last_busy,
        "fixture did not reproduce occupancy filtering before individual lease attempts");
    // Parent exhaustion must not replace B with saved endpoint A in the PNG.
    require(task.failure_coord == blocked_tile->coord
            && task.failure_node_type == fpga::CB_NODE_DST && task.failure_node == east,
        "failure visualization points to the committed tail instead of downstream congestion");
    // Recording an uncommitted node must neither append it nor lease its bits.
    require(route.size() == 2 && route.back().to == tail_tile->coord
            && !tail_tile->cb.src.jump.testBit(east)
            && !blocked_tile->cb.dst.jump.testBit(east)
            && blocked_tile->cb.src.jump == all_srcs,
        "failure diagnostics modified route fragments or occupancy");

    blocked_tile->cb.src.jump = {};
    router.route_recursion_budget = 5;
    router.routeNetTask(task);
    // A later successful suffix must discard the preceding attempt's blocker.
    require(route.size() > 2 && router.route_stats.exhausted_coord.x < 0
            && task.failure_coord == route.back().to,
        "successful continuation retained an obsolete congestion coordinate");
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

// An earlier terminal failure can leave a sticky SRC deadend, although a
// different input at the same target remains free. Search permits this final
// hop; commit must preserve that exception without bypassing occupied nodes.
void target_deadend_exception_survives_commit()
{
    for (int hops : {1, 2}) {
        for (int blocker = 0; blocker < 5; ++blocker) {
            auto tiles = resetArenaGrid(7, 5);
            auto& device = fpga::Device::current();
            auto& cb = device.cb_types.front();
            const int east = encodeJump(1, 0, 0);
            constexpr int joint = 7;
            cb.dst_local[east].local = {};
            cb.dst_joint[east].joint = bit(joint);
            cb.joint_local[joint].local = bit(17);
            cb.rememberNodeName(fpga::CB_NODE_JOINT, joint, "TERMINAL_LINK");
            rememberConn(cb, fpga::CB_NODE_DST, east, fpga::CB_NODE_JOINT, joint);
            rememberConn(cb, fpga::CB_NODE_JOINT, joint, fpga::CB_NODE_LOCAL, 17);
            cb.derived_masks_valid = false;
            cb.rebuildOutgoingSrcs();
            cb.ensureDerivedMasks();
            const NodeMask all_srcs = cb.local_src[16].jump;
            for (auto* tile : tiles) tile->cb.src.jump = all_srcs;
            auto* source_tile = device.getTile(2, 2);
            auto* last_tile = device.getTile(1 + hops, 2);
            auto* sink_tile = device.getTile(2 + hops, 2);
            for (int x = 2; x < 2 + hops; ++x)
                device.getTile(x, 2)->cb.src.jump &= ~bit(east);
            last_tile->cb.src_deadend.jump |= bit(east);

            // Negative controls: the deadend exception is not permission to
            // steal a SRC, landing DST, terminal JOINT, or input pin.
            if (blocker == 1) last_tile->cb.src.jump |= bit(east);
            if (blocker == 2) sink_tile->cb.dst.jump |= bit(east);
            if (blocker == 3) sink_tile->cb.joint.jump |= bit(joint);
            if (blocker == 4) {
                sink_tile->cb.local.local |= bit(17);
                sink_tile->pin_state.leased_nodes |= bit(17);
            }
            std::vector<fpga::CBState> before;
            std::vector<NodeMask> pins_before;
            for (auto* tile : tiles) {
                before.push_back(tile->cb);
                pins_before.push_back(tile->pin_state.leased_nodes);
            }
            ArenaDesign design;
            auto* source = design.makeInst("deadend_source", *source_tile);
            auto* sink = design.makeInst("different_terminal", *sink_tile);
            auto* net = design.makeNet("sticky_deadend_final_hop");
            pnr::RouteDesign router;
            router.fpga = &device;
            router.fpga_width = 7;
            router.fpga_height = 5;
            router.iteration_limit = 5;
            router.route_recursion_budget = 5;
            router.route_deadends_enabled = true;
            router.route_preemption_enabled = false;
            router.complete_routes_only = true;
            pnr::RouteDesign::RouteTask task{
                .from = source, .to = sink, .net = net,
                .from_port = "O", .to_port = "I0", .net_name = net->name,
                .endpoints_prepared = true};
            const bool routed = router.routeNetTask(task);
            require(routed == (blocker == 0), std::format(
                "final-hop deadend commit: hops={} blocker={} routed={}",
                hops, blocker, routed));
            require(last_tile->cb.src_deadend.jump.testBit(east),
                "terminal exception erased the sticky mark globally");
            if (routed) {
                require(sink->wires.size() == 1 && fpga::isRouteComplete(sink->wires.front()) &&
                        last_tile->cb.src.jump.testBit(east) &&
                        sink_tile->cb.dst.jump.testBit(east) &&
                        sink_tile->cb.joint.jump.testBit(joint) &&
                        sink_tile->isPinNodeLeased(17) && router.route_stats.commit_rollbacks == 0,
                        "accepted final hop was not committed with its terminal leases");
            } else {
                require(sink->wires.empty(), "blocked final hop left an unfinished route");
                for (size_t i = 0; i < tiles.size(); ++i) {
                    require(tiles[i]->cb.src.jump == before[i].src.jump &&
                            tiles[i]->cb.dst.jump == before[i].dst.jump &&
                            tiles[i]->cb.joint.jump == before[i].joint.jump &&
                            tiles[i]->cb.local.local == before[i].local.local &&
                            tiles[i]->pin_state.leased_nodes == pins_before[i],
                            "blocked terminal exception changed existing leases");
                }
            }
        }
    }
}

// The first reached DST cannot finish within Docking's depth limit. The next
// one can join a complete path across one foreign transit. Deferred Docking
// used to test free paths at both positions but preemption only at the first.
void docking_finishes_at_each_reached_position()
{
  // Private cut, shared cut, and a shared claimant with reciprocal historical
  // route-name records. History must not suppress a physically legal cut.
  for (int shared_mode : {0, 1, 2}) {
    for (bool second_obstruction : {false, true}) {
    for (bool free_alternative : {false, true}) {
    for (bool blocked_terminal : {false, true}) {
        auto tiles = resetArenaGrid(20, 17);
        auto& device = fpga::Device::current();
        auto& cb = device.cb_types.front();
        cb = {};
        cb.name = "box_719";
        cb.type_id = cb.base_type_id = 0;
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, 16, "node_823");
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, 17, "node_419");
        const std::vector<fpga::Coord> path{
            {7,7}, {8,7}, {9,7}, {10,7}, {11,7}, {11,8}, {10,8},
            {9,8}, {8,8}, {7,8}, {7,9}, {8,9}, {10,9}};
        auto edge = [&](fpga::Coord from, fpga::Coord to, int local,
                        int src, int dst, bool takeoff) {
            cb.rememberNodeName(fpga::CB_NODE_SRC, src, std::format("node_{}", src + 31));
            cb.rememberNodeName(fpga::CB_NODE_DST, dst, std::format("node_{}", dst + 43));
            if (takeoff) cb.local_src[local].jump |= bit(src);
            else cb.dst_src[local].jump |= bit(src);
            rememberConn(cb, takeoff ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST,
                         local, fpga::CB_NODE_SRC, src);
            rememberJumpTarget(cb, src, dst, to - from);
            device.getTile(to.x, to.y)->incoming_dst_nodes |= bit(dst);
            fpga::Wire wire;
            wire.from = from; wire.to = to; wire.local = local;
            wire.jump = src; wire.dst = dst;
            wire.pos = takeoff ? 0 : 1;
            return wire;
        };
        std::vector<fpga::Wire> corridor;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const auto delta = path[i + 1] - path[i];
            corridor.push_back(edge(path[i], path[i + 1], i ? 2999 + i : 16,
                encodeJump(delta.x, delta.y, i), 3000 + i, i == 0));
        }
        constexpr int terminal_joint = 9;
        cb.rememberNodeName(fpga::CB_NODE_JOINT, terminal_joint, "node_967");
        cb.dst_joint[3011].joint = bit(terminal_joint);
        cb.joint_local[terminal_joint].local = bit(17);
        rememberConn(cb, fpga::CB_NODE_DST, 3011, fpga::CB_NODE_JOINT, terminal_joint);
        rememberConn(cb, fpga::CB_NODE_JOINT, terminal_joint, fpga::CB_NODE_LOCAL, 17);

        ArenaDesign design;
        auto* source = design.makeInst("source_173", *device.getTile(7,7));
        auto* sink = design.makeInst("sink_587", *device.getTile(10,9));
        auto* victim_source = design.makeInst("source_991", *device.getTile(10,6));
        auto* victim_sink = design.makeInst("sink_613", *device.getTile(12,8));
        auto* net = design.makeNet("signal_271");
        auto* victim = design.makeNet("signal_853");
        auto first = edge({10,6}, path[6], 16, encodeJump(0,2,14), 1500, true);
        auto shared = edge(path[6], {12,8}, 1500, encodeJump(2,0,15), 1501, false);
        constexpr int bridge_joint = 23;
        cb.rememberNodeName(fpga::CB_NODE_JOINT, bridge_joint, "node_739");
        for (auto* wire : {&corridor[6], &shared}) {
            cb.dst_src[wire->local].jump &= ~bit(wire->jump);
            cb.dst_joint[wire->local].joint |= bit(bridge_joint);
            cb.joint_src[bridge_joint].jump |= bit(wire->jump);
            rememberConn(cb, fpga::CB_NODE_DST, wire->local, fpga::CB_NODE_JOINT, bridge_joint);
            rememberConn(cb, fpga::CB_NODE_JOINT, bridge_joint, fpga::CB_NODE_SRC, wire->jump);
            wire->joint = bridge_joint;
        }
        cb.dst_local[1501].local |= bit(17);
        rememberConn(cb, fpga::CB_NODE_DST, 1501, fpga::CB_NODE_LOCAL, 17);
        fpga::Wire terminal;
        terminal.from = terminal.to = {12,8}; terminal.local = 1501;
        terminal.pos = 1;
        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN; pin.from = pin.to = {12,8}; pin.local = 17;
        victim_sink->wires.push_back({first, shared, terminal, pin});
        auto* sibling = design.makeInst("sink_479", *device.getTile(12,8));
        if (shared_mode) {
            sibling->wires.push_back(victim_sink->wires.front());
            for (auto& wire : sibling->wires.front()) wire.shared = true;
        }
        // This sibling leaves the same takeoff landing by another SRC, before
        // the contested joint. It must survive even the multi-binding cut.
        auto* earlier = design.makeInst("sink_683", *device.getTile(11,8));
        auto separate = edge(path[6], {11,8}, 1500, encodeJump(1,0,13), 1502, false);
        separate.owns_dst = false; separate.pos = 2;
        cb.dst_local[1502].local |= bit(17);
        rememberConn(cb, fpga::CB_NODE_DST, 1502, fpga::CB_NODE_LOCAL, 17);
        auto early_first = first; early_first.shared = true;
        auto early_terminal = terminal; early_terminal.from = early_terminal.to = {11,8};
        early_terminal.local = 1502;
        auto early_pin = pin; early_pin.from = early_pin.to = {11,8};
        earlier->wires.push_back({early_first,separate,early_terminal,early_pin});
        // Two separated transit obstructions used to stop both search fronts:
        // neither single blocked bridge joined them. Search must cross both
        // provisionally, then cut them together only after terminal validation.
        auto* second_source = design.makeInst("source_617", *device.getTile(6,6));
        auto* second_sink = design.makeInst("sink_719", *device.getTile(6,9));
        auto* second_net = design.makeNet("signal_947");
        if (second_obstruction) {
            auto first2 = edge({6,6}, path[9], 16, encodeJump(1,2,10), 1600, true);
            auto middle2 = edge(path[9], {6,9}, 1600, encodeJump(-1,1,11), 1601, false);
            constexpr int joint2 = 24;
            cb.rememberNodeName(fpga::CB_NODE_JOINT, joint2, "node_983");
            for (auto* wire : {&corridor[9], &middle2}) {
                cb.dst_src[wire->local].jump &= ~bit(wire->jump);
                cb.dst_joint[wire->local].joint |= bit(joint2);
                cb.joint_src[joint2].jump |= bit(wire->jump);
                rememberConn(cb, fpga::CB_NODE_DST, wire->local, fpga::CB_NODE_JOINT, joint2);
                rememberConn(cb, fpga::CB_NODE_JOINT, joint2, fpga::CB_NODE_SRC, wire->jump);
                wire->joint = joint2;
            }
            cb.dst_local[1601].local |= bit(17);
            rememberConn(cb, fpga::CB_NODE_DST, 1601, fpga::CB_NODE_LOCAL, 17);
            auto end2 = terminal; end2.from = end2.to = {6,9}; end2.local = 1601;
            auto pin2 = pin; pin2.from = pin2.to = {6,9};
            second_sink->wires.push_back({first2, middle2, end2, pin2});
            source->tile->cb.src.jump |= bit(first2.jump);
        }
        if (free_alternative) {
            // A direct rail bypasses the contested JOINTs. The free phase
            // must select it without cutting either transit owner.
            cb.dst_src[corridor[6].local].jump |= bit(corridor[6].jump);
            if (second_obstruction)
                cb.dst_src[corridor[9].local].jump |= bit(corridor[9].jump);
        }
        cb.rebuildOutgoingSrcs();
        cb.ensureDerivedMasks();
        // Boundary boxes have a distinct type, as their outgoing connections
        // can leave the device. Interior boxes share the complete mesh type.
        auto boundary = cb;
        boundary.type_id = 1;
        device.cb_types.push_back(std::move(boundary));
        for (auto* tile : tiles) {
            const auto coord = tile->coord;
            const bool edge_tile = coord.x == 0 || coord.y == 0 || coord.x == 19 || coord.y == 16;
            tile->cb_type = tile->cb.type = &device.cb_types[edge_tile ? 1 : 0];
        }
        // Only the intended source exit is free. The victim source's output
        // mapping is shared by this synthetic CB type, not by its placement.
        source->tile->cb.src.jump |= bit(first.jump);
        require(pnr::commitPreparedRoute(victim_sink->wires.front()),
                "could not lease the reference transit route");
        fpga::attachNetRoute(*victim, *victim_sink, 0, victim_source, victim_sink,
                            "O", "I0", victim->name);
        fpga::registerNetRouteTiles(*victim, victim_sink->wires.front(), 0);
        if (second_obstruction) {
            require(pnr::commitPreparedRoute(second_sink->wires.front()), "second transit lease failed");
            fpga::attachNetRoute(*second_net, *second_sink, 0, second_source, second_sink,
                                "O", "I0", second_net->name);
            fpga::registerNetRouteTiles(*second_net, second_sink->wires.front(), 0);
        }
        if (shared_mode) {
            fpga::attachNetRoute(*victim,*sibling,0,victim_source,sibling,"O","I0","route_479");
            fpga::registerNetRouteTiles(*victim,sibling->wires.front(),1);
        }
        // The shared takeoff is already leased; lease only this private suffix.
        std::vector<fpga::Wire> early_suffix{separate,early_terminal,early_pin};
        require(pnr::commitPreparedRoute(early_suffix),"could not lease earlier sibling");
        const size_t early_binding = victim->routes.size();
        fpga::attachNetRoute(*victim,*earlier,0,victim_source,earlier,"O","I0","route_683");
        fpga::registerNetRouteTiles(*victim,earlier->wires.front(),early_binding);
        if (blocked_terminal) sink->tile->cb.joint.jump |= bit(terminal_joint);

        pnr::RouteDesign router;
        router.fpga = &device; router.fpga_width = 20; router.fpga_height = 17;
        router.iteration_limit = router.route_recursion_budget = 5;
        router.complete_routes_only = true;
        router.fanout_stage = true;
        if (shared_mode == 2) {
            pnr::rememberPreemptionBlocker(router.preempted_route_blockers, "route_479", net->name);
            pnr::rememberPreemptionBlocker(router.preempted_route_blockers, net->name, "route_479");
        }
        pnr::RouteDesign::RouteTask task{
            .from = source, .to = sink, .net = net, .from_port = "O", .to_port = "I0",
            .net_name = net->name, .endpoints_prepared = true};
        const bool routed = router.routeNetTask(task);
        const bool should_route = !blocked_terminal;
        require(routed == should_route, std::format(
                "immediate Docking missed a later reachable completion: blocked={} routed={}",
                blocked_terminal, routed));
        if (should_route && free_alternative) {
            require(fpga::isRouteComplete(sink->wires.front()) &&
                    fpga::isRouteComplete(victim_sink->wires.front()) &&
                    router.route_stats.preempt_success == 0 && router.pending_route_todo.empty(),
                    "free-first search unnecessarily cut a transit victim");
            if (second_obstruction) require(fpga::isRouteComplete(second_sink->wires.front()),
                    "free-first search damaged the second transit victim");
        } else if (should_route) {
            require(fpga::isRouteComplete(sink->wires.front()), "Docking published an incomplete route");
            require(router.route_stats.search_pops <= 3,
                    "Docking was deferred until forward exploration finished");
            const size_t count = (shared_mode ? 2 : 1) + second_obstruction;
            require(router.route_stats.preempt_success == count &&
                    router.pending_route_todo.size() == count &&
                    !fpga::isRouteComplete(victim_sink->wires.front()),
                    "Docking did not cut/requeue exactly the dependent bindings");
            if (shared_mode) require(!fpga::isRouteComplete(sibling->wires.front()),
                    "shared claimant kept a disconnected suffix");
            if (second_obstruction) require(!fpga::isRouteComplete(second_sink->wires.front()),
                    "second transit obstruction was not cut");
            require(device.getTile(10,8)->cb.joint.jump.testBit(bridge_joint) && sink->tile->isPinNodeLeased(17),
                    "replacement did not own its bridge and terminal");
        } else {
            require(fpga::isRouteComplete(victim_sink->wires.front()) && router.route_stats.preempt_success == 0,
                    "failed Docking damaged the victim before validating the terminal");
            require(sink->wires.empty(), "failed Docking retained an unfinished suffix");
            if (second_obstruction) require(fpga::isRouteComplete(second_sink->wires.front()),
                    "failed provisional search cut its second victim");
            if (shared_mode) require(fpga::isRouteComplete(sibling->wires.front()) &&
                    router.pending_route_todo.empty(),"failed shared preemption damaged a sibling");
        }
        require(fpga::isRouteComplete(earlier->wires.front()) &&
                device.getTile(10,8)->cb.src.jump.testBit(separate.jump) &&
                earlier->tile->isPinNodeLeased(17),"preemption damaged an earlier sibling branch");
    }
    }
    }
  }
}

} // namespace

// Real stage boundary regression: an unreachable first connection must report
// failure without leasing a partial path or trying the routable second task.
void routing_tasks_fail_fast()
{
    using Router = pnr::RouteDesign;
    std::string directory = (std::filesystem::temp_directory_path() /
                             "scalepnr-fail-fast-XXXXXX").string();
    require(mkdtemp(directory.data()) != nullptr, "cannot create diagnostic directory");
    setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", directory.c_str(), 1);
    for (int stage = 0; stage != 5; ++stage) {
        constexpr int width = 24, height = 9;
        auto tiles = resetArenaGrid(width, height);
        auto& device = fpga::Device::current();
        ArenaDesign design;
        auto* source = design.makeInst("source", *device.getTile(2, 4));
        auto* sink = design.makeInst("blocked", *device.getTile(20, 4));
        auto* next_source = design.makeInst("next_source", *device.getTile(2, 6));
        auto* next_sink = design.makeInst("next_sink", *device.getTile(20, 6));
        auto* failed_net = design.makeNet("first_must_fail");
        auto* next_net = design.makeNet("second_must_not_run");
        Router router;
        router.fpga = &device;
        router.fpga_width = width;
        router.fpga_height = height;
        router.iteration_limit = 5;
        router.route_suffix_depth_limit = 1;
        router.preserve_routes = true; // failRouting throws after writing artifacts.
        router.route_preemption_enabled = false;
        router.moving_stage = stage == 1 || stage == 3;
        router.moving_sources_stage = stage == 1;
        router.fanout_stage = stage == 2;
        Router::RouteTaskMode mode = stage == 4 ? Router::RouteTaskMode::Constant
            : stage == 2 ? Router::RouteTaskMode::Fanout
            : router.moving_stage ? Router::RouteTaskMode::Moving
            : Router::RouteTaskMode::Generic;
        if (stage == 2) {
            auto* trunk_sink = design.makeInst("trunk", *device.getTile(3, 4));
            auto* trunk_net = design.makeNet("complete_trunk");
            Router::RouteTask seed{source, trunk_sink, trunk_net, "O", "I0", trunk_net->name};
            router.routeTaskOrFail(seed, "test seed");
        }
        failed_net->distributed_source = stage == 4;
        // No constant root exists in this abstract mesh. Ordinary routes have
        // a blocked input, but plenty of free transit exits for wandering.
        sink->tile->pin_state.lease(17);
        sink->tile->cb.local.local |= bit(17);
        std::vector<fpga::CBState> before;
        for (auto* tile : tiles) before.push_back(tile->cb);
        std::vector<Router::RouteTask> tasks{
            {source, sink, failed_net, "O", "I0", failed_net->name},
            {next_source, next_sink, next_net, "O", "I0", next_net->name}
        };
        tasks.front().fanout = stage == 2;
        const size_t edges_before_failure = router.route_stats.edge_accepted;
        bool failed = false;
        try {
            router.routeTaskBatch(mode, tasks, tasks.size(), 1);
        } catch (const std::runtime_error& error) {
            failed = std::string(error.what()).find("current connection did not finish") != std::string::npos;
        }
        require(failed, "stage returned instead of failing its first unfinished task");
        require(router.route_stats.edge_accepted == edges_before_failure,
                "router explored transit despite every destination input being leased");
        require(sink->wires.empty(), "failed task committed an incomplete suffix");
        require(next_sink->wires.empty() && next_net->routes.empty(),
                "stage attempted a second task after failure");
        require(!router.complete_routes_only, "task scope leaked after failure");
        for (size_t i = 0; i < tiles.size(); ++i) {
            require(tiles[i]->cb.src.jump == before[i].src.jump &&
                    tiles[i]->cb.dst.jump == before[i].dst.jump &&
                    tiles[i]->cb.joint.jump == before[i].joint.jump &&
                    tiles[i]->cb.local.local == before[i].local.local,
                    "unsuccessful task left speculative CB leases");
        }
        std::ifstream report(std::filesystem::path(directory) / "routing_failure.txt");
        std::string text((std::istreambuf_iterator<char>(report)), {});
        require(text.find("net=first_must_fail") != std::string::npos,
                "failure report does not identify the first task");
        // Control: the second route really is routable, and spans more than
        // the one-hop slice configured above. It must finish in one task call.
        router.fanout_stage = false;
        router.routeTaskOrFail(tasks.back(), "positive control", 1);
        require(!next_sink->wires.empty() &&
                next_sink->wires.back().back().type == fpga::Wire::WIRE_TILE_PIN,
                "whole-task search stopped at a scheduler slice boundary");
    }
    unsetenv("SCALEPNR_FAILURE_ARTIFACT_DIR");
    std::printf("fail-fast diagnostic artifacts: %s\n", directory.c_str());
}

int main(int argc, char** argv)
{
    try {
        if (argc == 2 && std::string(argv[1]) == "fanout_spread") {
            fanout_exit_classification_preserves_routing();
            return EXIT_SUCCESS;
        }
        if (argc == 2 && std::string(argv[1]) == "fanout_source") {
            fanout_search_includes_endpoint_tiles(true);
            return EXIT_SUCCESS;
        }
        if (argc == 2 && std::string(argv[1]) == "fanout_destination") {
            fanout_search_includes_endpoint_tiles(false);
            return EXIT_SUCCESS;
        }
        if (argc == 2 && std::string(argv[1]) == "immediate_docking") {
            if (std::getenv("SCALEPNR_DEBUG_ROUTE_NET")) debug_level = 1;
            nearby_source_routes_by_reverse_grounding();
            docking_finishes_at_each_reached_position();
            return EXIT_SUCCESS;
        }
        if (argc == 2 && std::string(argv[1]) == "target_deadend") {
            target_deadend_exception_survives_commit();
            return EXIT_SUCCESS;
        }
        if (argc == 2 && std::string(argv[1]) == "fail_fast") {
            routing_tasks_fail_fast();
            return EXIT_SUCCESS;
        }
        failed_uncommitted_hop_reports_downstream_congestion();
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
