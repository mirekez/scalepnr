#include "RouteDesign.h"

#include "Device.h"
#include "Module.h"
#include "TimingPath.h"
#include "Wire.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

struct Failure : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw Failure(message);
    }
}

NodeMask bit(int node)
{
    return NodeMask{0, 1} << node;
}

std::string randomToken(std::mt19937& rng, const std::string& prefix)
{
    static constexpr char alphabet[] = "abcdefghijklmnopqrstuvwxyz";
    std::uniform_int_distribution<int> pick(0, 25);
    std::string result = prefix;
    for (int i = 0; i < 12; ++i) {
        result.push_back(alphabet[pick(rng)]);
    }
    return result;
}

void resetGrid(int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;
    device.tile_grid.resize(static_cast<size_t>(width * height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            device.tile_grid[static_cast<size_t>(y * width + x)].coord = {x, y};
        }
    }
}

fpga::Wire tilePin(fpga::Coord coord, int local, const std::string& node)
{
    fpga::Wire wire;
    wire.type = fpga::Wire::WIRE_TILE_PIN;
    wire.from = coord;
    wire.to = coord;
    wire.local = local;
    wire.src_wire_name = node;
    return wire;
}

fpga::Wire crossbar(fpga::Coord from, fpga::Coord to, int local, int src, int dst,
                    const std::string& from_name, const std::string& src_name,
                    const std::string& dst_name, bool source)
{
    fpga::Wire wire;
    wire.type = fpga::Wire::WIRE_CROSSBAR;
    wire.from = from;
    wire.to = to;
    wire.local = local;
    wire.jump = src;
    wire.dst = dst;
    wire.pos = source ? 0 : 1;
    wire.from_wire_name = from_name;
    wire.src_wire_name = src_name;
    wire.dst_wire_name = dst_name;
    return wire;
}

void leaseOwnedRoute(const std::vector<fpga::Wire>& route)
{
    for (size_t index = 0; index < route.size(); ++index) {
        const fpga::Wire& wire = route[index];
        if (wire.shared) {
            continue;
        }
        fpga::Tile* tile = fpga::Device::current().getTile(wire.from.x, wire.from.y);
        require(tile != nullptr, "route references a tile outside the test grid");
        if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
            if (index + 1 == route.size()) {
                tile->pin_state.leased_nodes |= bit(wire.local);
                tile->cb.local.local |= bit(wire.local);
            }
            continue;
        }
        tile->cb.src.jump |= bit(wire.jump);
        if (wire.pos == 0) {
            tile->cb.local.local |= bit(wire.local);
        }
        else if (wire.owns_dst) {
            tile->cb.dst.jump |= bit(wire.local);
        }
    }
}

struct TestDesign
{
    rtl::Design design;
    std::vector<std::unique_ptr<rtl::Inst>> instances;

    rtl::Module& addModule()
    {
        return design.modules.emplace_back();
    }

    rtl::Inst* addInst()
    {
        instances.push_back(std::make_unique<rtl::Inst>());
        return instances.back().get();
    }

    rtl::Net& addNet(rtl::Module& module, const std::string& name)
    {
        Referable<rtl::Net>& net = module.nets.emplace_back();
        net.name = name;
        return net;
    }

    void addRoute(rtl::Net& net, rtl::Inst* driver, const std::string& source_port,
                  std::vector<fpga::Wire> route, const std::string& route_name)
    {
        rtl::Inst* sink = addInst();
        rtl::Inst* owner = addInst();
        owner->wires.push_back(std::move(route));
        size_t route_index = owner->wires.size() - 1;
        net.routes.push_back(rtl::NetRouteBinding{
            owner, route_index, driver, sink, source_port, "input", route_name
        });
        leaseOwnedRoute(owner->wires[route_index]);
    }
};

struct BuiltTree
{
    rtl::Net* net = nullptr;
    rtl::Inst* driver = nullptr;
    std::string source_port;
    std::vector<std::string> route_names;
    size_t prefix_crossbars = 0;
};

BuiltTree addSharedTree(TestDesign& fixture, rtl::Module& module, std::mt19937& rng,
                        int base_x, int prefix_crossbars, int branch_count,
                        const std::string& label)
{
    BuiltTree tree;
    tree.net = &fixture.addNet(module, randomToken(rng, label + "_net_"));
    tree.driver = fixture.addInst();
    tree.source_port = randomToken(rng, label + "_port_");
    tree.prefix_crossbars = static_cast<size_t>(prefix_crossbars);

    int local = 10 + base_x;
    std::string current_node = randomToken(rng, label + "_origin_");
    std::vector<fpga::Wire> prefix;
    prefix.push_back(tilePin({base_x, 1}, local, current_node));
    for (int step = 0; step < prefix_crossbars; ++step) {
        int src = 40 + step;
        int dst = 100 + step;
        std::string src_name = randomToken(rng, label + "_src_");
        std::string dst_name = randomToken(rng, label + "_dst_");
        prefix.push_back(crossbar({base_x + step, 1}, {base_x + step + 1, 1},
            step == 0 ? local : 99 + step, src, dst,
            current_node, src_name, dst_name, step == 0));
        current_node = dst_name;
    }

    std::vector<fpga::Wire> trunk = prefix;
    trunk.push_back(tilePin({base_x + prefix_crossbars, 1}, 200,
        randomToken(rng, label + "_trunk_pin_")));
    tree.route_names.push_back(randomToken(rng, label + "_trunk_"));
    fixture.addRoute(*tree.net, tree.driver, tree.source_port,
        std::move(trunk), tree.route_names.back());

    for (int branch_index = 0; branch_index < branch_count; ++branch_index) {
        std::vector<fpga::Wire> branch = prefix;
        for (fpga::Wire& wire : branch) {
            wire.shared = true;
            wire.owns_dst = false;
        }
        int branch_src = 300 + branch_index;
        int branch_dst = 400 + branch_index;
        fpga::Coord branch_from{base_x + prefix_crossbars, 1};
        fpga::Coord branch_to{base_x + prefix_crossbars + 1, 2 + branch_index};
        branch.push_back(crossbar(branch_from, branch_to, 99 + prefix_crossbars,
            branch_src, branch_dst, current_node,
            randomToken(rng, label + "_branch_src_"),
            randomToken(rng, label + "_branch_dst_"), false));
        branch.push_back(tilePin(branch_to, 500 + branch_index,
            randomToken(rng, label + "_branch_pin_")));
        tree.route_names.push_back(randomToken(rng, label + "_branch_"));
        fixture.addRoute(*tree.net, tree.driver, tree.source_port,
            std::move(branch), tree.route_names.back());
    }
    return tree;
}

std::vector<fpga::Wire>& boundRoute(rtl::NetRouteBinding& binding)
{
    require(binding.owner != nullptr && binding.route_index < binding.owner->wires.size(),
        "invalid test route binding");
    return binding.owner->wires[binding.route_index];
}

bool regionHasLeases(int first_x, int last_x, int first_y, int last_y)
{
    for (int y = first_y; y <= last_y; ++y) {
        for (int x = first_x; x <= last_x; ++x) {
            const fpga::Tile* tile = fpga::Device::current().getTile(x, y);
            if (tile && (tile->cb.src.jump != NodeMask{} || tile->cb.dst.jump != NodeMask{}
                    || tile->cb.local.local != NodeMask{}
                    || tile->pin_state.leased_nodes != NodeMask{})) {
                return true;
            }
        }
    }
    return false;
}

void randomized_shared_prefix_repairs_return_to_source_state()
{
    std::mt19937 rng(0x5a17c0deU);
    std::uniform_int_distribution<int> prefix_length(2, 6);
    std::uniform_int_distribution<int> branch_count(2, 5);

    for (int test_index = 0; test_index < 10; ++test_index) {
        resetGrid(32, 12);
        TestDesign fixture;
        rtl::Module& module = fixture.addModule();
        module.nets.reserve(2);
        int damaged_length = prefix_length(rng);
        int damaged_branches = branch_count(rng);
        BuiltTree damaged = addSharedTree(fixture, module, rng, 1,
            damaged_length, damaged_branches, "damaged");
        BuiltTree control = addSharedTree(fixture, module, rng, 18,
            prefix_length(rng), branch_count(rng), "control");

        if (test_index == 0) {
            // Reproduce a stale ownership alias: two endpoint bindings point at
            // one physical route slot before the source tree is repaired.
            require(fpga::unrouteNetRoute(*damaged.net, 1),
                "failed to release branch before constructing route-slot alias");
            damaged.net->routes[1].owner = damaged.net->routes[0].owner;
            damaged.net->routes[1].route_index = damaged.net->routes[0].route_index;
        }

        std::vector<pnr::RouteDesign::RouteTask> tasks;
        require(pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design, tasks) == 0,
            "valid shared route tree was incorrectly classified as stale");
        require(tasks.empty(), "valid shared route tree generated repair tasks");

        std::uniform_int_distribution<int> corrupt_step(0, damaged_length - 1);
        size_t corrupt_index = 1 + static_cast<size_t>(corrupt_step(rng));
        std::vector<fpga::Wire>& trunk = boundRoute(damaged.net->routes.front());
        require(corrupt_index < trunk.size() - 1
                && trunk[corrupt_index].type == fpga::Wire::WIRE_CROSSBAR,
            "random corruption did not select an owned trunk fragment");
        trunk[corrupt_index].shared = true;

        size_t expected_routes = static_cast<size_t>(damaged_branches + 1);
        size_t repaired = pnr::RouteDesign::repairStaleSharedRoutePrefixes(
            fixture.design, tasks);
        require(repaired == expected_routes,
            "repair did not remove the complete damaged source tree");
        require(tasks.size() == expected_routes,
            "repair did not recreate every source-tree routing task");

        size_t generic_task_count = 0;
        std::unordered_set<std::string> task_names;
        for (const pnr::RouteDesign::RouteTask& task : tasks) {
            require(task.from == damaged.driver && task.from_port == damaged.source_port,
                "repair task was detached from the damaged source port");
            generic_task_count += task.fanout ? 0 : 1;
            task_names.insert(task.net_name);
        }
        require(generic_task_count == 1,
            "repair did not reduce the damaged tree to exactly one Generic seed");
        require(task_names == std::unordered_set<std::string>(
                damaged.route_names.begin(), damaged.route_names.end()),
            "repair task names do not reproduce the damaged source hierarchy");

        std::vector<pnr::RouteDesign::RouteTask> generic_tasks;
        std::vector<pnr::RouteDesign::RouteTask> fanout_tasks;
        size_t scheduled_seeds = pnr::RouteDesign::scheduleSharedPrefixRepairs(
            tasks, generic_tasks, fanout_tasks);
        require(tasks.empty(),
            "shared-prefix scheduling retained tasks in its mixed input queue");
        require(scheduled_seeds == 1 && generic_tasks.size() == 1,
            "shared-prefix scheduling did not create exactly one Generic seed");
        require(!generic_tasks.front().fanout,
            "shared-prefix scheduling mislabeled the Generic seed as Fanout");
        require(fanout_tasks.size() == static_cast<size_t>(damaged_branches),
            "shared-prefix scheduling did not defer every dependent branch");
        for (const pnr::RouteDesign::RouteTask& task : fanout_tasks) {
            require(task.fanout,
                "shared-prefix scheduling left a dependent branch in Generic mode");
            require(pnr::fanoutWaitsForGenericSeed(true, task.fanout, false, false),
                "Fanout without a repaired Generic seed was considered runnable");
        }
        require(!pnr::fanoutWaitsForGenericSeed(true, true, false, true),
            "Fanout remained deferred after its Generic seed completed");
        require(!pnr::fanoutWaitsForGenericSeed(true, true, true, false),
            "an already-complete Fanout was promoted to a destructive Generic repair");
        require(!pnr::fanoutStageCanStart(12, 1),
            "Fanout started while one physical source still lacked a Generic trunk");
        require(pnr::fanoutStageCanStart(12, 0),
            "Fanout did not start after every physical source had a Generic trunk");
        require(pnr::routingStageUsesDeadends(false, false),
            "a repaired Generic seed would run without Basic deadend learning");
        require(!pnr::routingStageUsesDeadends(true, false),
            "a repaired Fanout branch would incorrectly retain Basic deadends");

        for (rtl::NetRouteBinding& binding : damaged.net->routes) {
            require(binding.owner == nullptr
                    && binding.route_index == std::numeric_limits<size_t>::max(),
                "repair retained or aliased old route storage");
        }
        require(!regionHasLeases(1, 10, 0, 8),
            "repair retained leases owned by the damaged source tree");

        for (rtl::NetRouteBinding& binding : control.net->routes) {
            require(!boundRoute(binding).empty(),
                "repair modified an unrelated valid shared route tree");
        }
        require(regionHasLeases(18, 28, 0, 8),
            "repair released leases owned by an unrelated source tree");
    }
}

void repeated_stage_entries_share_one_timeout_budget()
{
    constexpr double budget = 300.0;
    double elapsed = 120.0;

    // Re-entering a logical stage keeps the time consumed by earlier passes.
    require(pnr::routeStageSecondsRemaining(elapsed, budget) == 180.0,
        "initial cumulative stage timeout accounting is incorrect");
    elapsed += 179.0;
    require(!pnr::routeStageTimeoutIsFatal(elapsed, budget, true),
        "stage timeout expired before its cumulative budget");
    elapsed += 1.0;
    require(pnr::routeStageTimeoutIsFatal(elapsed, budget, true),
        "stage re-entry incorrectly received a fresh timeout budget");
    require(!pnr::routeStageTimeoutIsFatal(elapsed, budget, false),
        "a completed stage was classified as timed out");

    // Scope cleanup must account an iteration that exits before pass reporting.
    double charged = 0.0;
    {
        pnr::RouteStageTimeCharge charge(charged);
    }
    require(charged > 0.0,
        "an early scheduler exit escaped cumulative timeout accounting");
}

} // namespace

int main()
{
    try {
        randomized_shared_prefix_repairs_return_to_source_state();
        repeated_stage_entries_share_one_timeout_budget();
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "repair_prefixes_test failed: %s\n", error.what());
        return 1;
    }
    return 0;
}
