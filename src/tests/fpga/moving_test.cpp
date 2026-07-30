#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Inst.h"
#include "Wire.h"
#include "route/RoutePassState.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
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

bool isSet(NodeMask mask, int node)
{
    return (mask & bit(node)) != NodeMask{};
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

fpga::Wire crossbar(fpga::Coord from, fpga::Coord to, int local,
                    int src, int dst, int pos, bool shared = false,
                    bool owns_dst = true)
{
    fpga::Wire wire;
    wire.type = fpga::Wire::WIRE_CROSSBAR;
    wire.from = from;
    wire.to = to;
    wire.local = local;
    wire.jump = src;
    wire.dst = dst;
    wire.pos = pos;
    wire.shared = shared;
    wire.owns_dst = owns_dst;
    wire.from_wire_name = "node_" + std::to_string(local);
    wire.src_wire_name = "node_" + std::to_string(src);
    wire.dst_wire_name = "node_" + std::to_string(dst);
    return wire;
}

fpga::Wire tilePin(fpga::Coord coord, int local, bool shared = false)
{
    fpga::Wire wire;
    wire.type = fpga::Wire::WIRE_TILE_PIN;
    wire.from = coord;
    wire.to = coord;
    wire.local = local;
    wire.shared = shared;
    wire.src_wire_name = "pin_" + std::to_string(local);
    return wire;
}

void leaseRoute(const std::vector<fpga::Wire>& route)
{
    for (size_t index = 0; index < route.size(); ++index) {
        const fpga::Wire& fragment = route[index];
        if (fragment.shared) {
            continue;
        }
        fpga::Tile* tile = fpga::Device::current().getTile(
            fragment.from.x, fragment.from.y);
        require(tile != nullptr, "test route references a missing tile");
        if (fragment.type == fpga::Wire::WIRE_TILE_PIN) {
            if (index + 1 == route.size()) {
                tile->pin_state.leased_nodes |= bit(fragment.local);
                tile->cb.local.local |= bit(fragment.local);
            }
            continue;
        }
        tile->cb.src.jump |= bit(fragment.jump);
        if (fragment.pos == 0) {
            tile->cb.local.local |= bit(fragment.local);
        }
        else if (fragment.owns_dst) {
            tile->cb.dst.jump |= bit(fragment.local);
        }
    }
}

bool sameFragment(const fpga::Wire& left, const fpga::Wire& right)
{
    return left.type == right.type
        && left.from.x == right.from.x && left.from.y == right.from.y
        && left.to.x == right.to.x && left.to.y == right.to.y
        && left.local == right.local && left.jump == right.jump
        && left.dst == right.dst && left.shared == right.shared
        && left.owns_dst == right.owns_dst;
}

void moving_one_fanout_releases_only_its_suffix()
{
    constexpr int branch_count = 24;
    constexpr int moved_branch = 11;
    resetGrid(branch_count + 8, 3);

    Referable<rtl::Net> net;
    net.name = "large_random_name_fanout_tree";
    rtl::Inst driver;
    std::vector<std::unique_ptr<rtl::Inst>> sinks;
    std::vector<std::unique_ptr<rtl::Inst>> owners;

    std::vector<fpga::Wire> trunk{
        tilePin({0, 1}, 10),
        crossbar({0, 1}, {1, 1}, 10, 100, 200, 0),
        crossbar({1, 1}, {2, 1}, 200, 101, 201, 1),
        crossbar({2, 1}, {3, 1}, 201, 102, 202, 1),
    };

    // The first binding owns the physical trunk; every later branch carries a
    // shared replica followed by a uniquely leased suffix to its sink.
    owners.push_back(std::make_unique<rtl::Inst>());
    sinks.push_back(std::make_unique<rtl::Inst>());
    std::vector<fpga::Wire> seed = trunk;
    seed.push_back(crossbar({3, 1}, {4, 1}, 202, 103, 203, 1));
    seed.push_back(crossbar({4, 1}, {4, 1}, 203, 104, 204, 1));
    seed.push_back(tilePin({4, 1}, 300));
    owners.back()->wires.push_back(seed);
    leaseRoute(owners.back()->wires.back());
    fpga::attachNetRoute(net, *owners.back(), 0, &driver, sinks.back().get(),
        "source", "sink", "seed_route");

    std::vector<std::vector<fpga::Wire>> sibling_snapshots;
    sibling_snapshots.reserve(branch_count);
    for (int branch = 0; branch < branch_count; ++branch) {
        owners.push_back(std::make_unique<rtl::Inst>());
        sinks.push_back(std::make_unique<rtl::Inst>());
        std::vector<fpga::Wire> route = trunk;
        for (fpga::Wire& fragment : route) {
            fragment.shared = true;
        }
        int target_x = branch + 5;
        int branch_src = 320 + branch;
        int arrival_src = 400 + branch;
        int arrival_dst = 500 + branch;
        int sink_local = 600 + branch;
        route.push_back(crossbar({3, 1}, {target_x, 1}, 202,
            branch_src, arrival_dst, 1, false, false));
        route.push_back(crossbar({target_x, 1}, {target_x, 1},
            arrival_dst, arrival_src, sink_local, 1));
        route.push_back(tilePin({target_x, 1}, sink_local));
        owners.back()->wires.push_back(route);
        leaseRoute(owners.back()->wires.back());
        fpga::attachNetRoute(net, *owners.back(), 0, &driver, sinks.back().get(),
            "source", "sink", "branch_" + std::to_string(branch));
        sibling_snapshots.push_back(route);
    }

    const size_t moved_binding = static_cast<size_t>(moved_branch + 1);
    const int moved_x = moved_branch + 5;
    const int moved_src = 320 + moved_branch;
    const int moved_arrival_src = 400 + moved_branch;
    const int moved_arrival_dst = 500 + moved_branch;
    const int moved_local = 600 + moved_branch;
    fpga::Tile* branch_tile = fpga::Device::current().getTile(3, 1);
    fpga::Tile* moved_tile = fpga::Device::current().getTile(moved_x, 1);
    require(branch_tile && moved_tile, "moving fanout test grid is incomplete");
    require(isSet(branch_tile->cb.src.jump, moved_src)
            && isSet(moved_tile->cb.src.jump, moved_arrival_src)
            && isSet(moved_tile->cb.dst.jump, moved_arrival_dst)
            && isSet(moved_tile->cb.local.local, moved_local),
        "moving fanout test did not lease the selected private suffix");

    require(fpga::invalidateMovedSinkRoute(net, moved_binding),
        "Moving could not invalidate the selected fanout sink");

    // Check: the moved route retains exactly its shared trunk replica.
    const std::vector<fpga::Wire>& moved_route = owners[moved_binding]->wires[0];
    require(moved_route.size() == trunk.size(),
        "Moving retained private suffix fragments or removed the shared trunk");
    for (size_t index = 0; index < moved_route.size(); ++index) {
        require(moved_route[index].shared
                && sameFragment(moved_route[index], sibling_snapshots[moved_branch][index]),
            "Moving changed the selected fanout's shared prefix");
    }

    // Check: all leases owned only by the selected branch suffix are free.
    require(!isSet(branch_tile->cb.src.jump, moved_src)
            && !isSet(moved_tile->cb.src.jump, moved_arrival_src)
            && !isSet(moved_tile->cb.dst.jump, moved_arrival_dst)
            && !isSet(moved_tile->cb.local.local, moved_local)
            && !isSet(moved_tile->pin_state.leased_nodes, moved_local),
        "Moving did not release every lease in the selected private suffix");

    // Check: the owning trunk and all other fanout vectors and leases remain intact.
    require(isSet(fpga::Device::current().getTile(0, 1)->cb.src.jump, 100)
            && isSet(fpga::Device::current().getTile(1, 1)->cb.src.jump, 101)
            && isSet(fpga::Device::current().getTile(2, 1)->cb.src.jump, 102),
        "Moving released a trunk lease while moving one fanout sink");
    for (int branch = 0; branch < branch_count; ++branch) {
        if (branch == moved_branch) {
            continue;
        }
        const std::vector<fpga::Wire>& route = owners[static_cast<size_t>(branch + 1)]->wires[0];
        require(route.size() == sibling_snapshots[branch].size()
                && std::equal(route.begin(), route.end(), sibling_snapshots[branch].begin(), sameFragment),
            "Moving changed a sibling fanout route");
        require(isSet(branch_tile->cb.src.jump, 320 + branch)
                && isSet(fpga::Device::current().getTile(branch + 5, 1)->cb.src.jump,
                    400 + branch),
            "Moving released a sibling fanout lease");
    }
}

void crossbar_destination_owner_uses_landing_node()
{
    resetGrid(3, 1);

    Referable<rtl::Net> net;
    net.name = "random_owner_identity";
    rtl::Inst owner;
    rtl::Inst driver;
    rtl::Inst sink;
    constexpr int source_incoming = 117;
    constexpr int destination_landing = 533;

    owner.wires.push_back({
        crossbar({0, 0}, {1, 0}, source_incoming, 301,
            destination_landing, 1),
    });
    fpga::attachNetRoute(net, owner, 0, &driver, &sink,
        "random_output", "random_input", "random_route");
    fpga::registerNetRouteTiles(net, owner.wires[0]);

    fpga::Tile* source = fpga::Device::current().getTile(0, 0);
    fpga::Tile* destination = fpga::Device::current().getTile(1, 0);
    require(source && destination, "destination owner test grid is incomplete");

    // Check: the source tile owns the incoming node used to select its exit.
    require(fpga::findNetByNode(*source, fpga::CB_NODE_DST,
                source_incoming, true) == &net,
        "source-side incoming destination node lost its owner");

    // Check: a jump alone does not own its landing node, and the destination
    // never aliases the source tile's incoming-node index.
    require(fpga::findNetByNode(*destination, fpga::CB_NODE_DST,
                destination_landing, true) == nullptr
            && fpga::findNetByNode(*destination, fpga::CB_NODE_DST,
                source_incoming, true) == nullptr,
        "jump fragment falsely owned or aliased its destination landing node");

    owner.wires[0].push_back(crossbar({1, 0}, {2, 0},
        destination_landing, 302, 701, 1));
    fpga::registerNetRouteTiles(net, owner.wires[0]);

    // Check: the following hop owns the resolved landing node because it
    // actually leases that node while exiting the destination tile.
    require(fpga::findNetByNode(*destination, fpga::CB_NODE_DST,
                destination_landing, true) == &net,
        "following crossbar hop did not own its leased incoming node");
}

void focused_moving_bounds_each_placement_while_routes_advance()
{
    int pending_routes = 3;
    int no_progress_passes = 0;
    int stagnant_passes = 0;
    constexpr int pass_limit = 5;

    auto run_pass = [&](size_t completed, size_t advanced) {
        bool made_progress = pnr::movingPassMadeProgress(completed, advanced);
        no_progress_passes = pnr::updateMovingPlacementNoProgressPasses(
            no_progress_passes, made_progress);
        stagnant_passes = made_progress ? 0 : stagnant_passes + 1;
        pending_routes -= static_cast<int>(completed);
        bool exhausted = pnr::movingPlacementPassesExhausted(
            no_progress_passes, pass_limit);
        return pnr::focusedMovingShouldRelocate(false, stagnant_passes,
            pass_limit, exhausted, exhausted);
    };

    // Reproduce the observed focused placement: two incident routes finish,
    // leaving one incremental route. Input-first or output-first ordering cannot
    // protect this work because relocating the cell invalidates both directions.
    require(!run_pass(1, 0) && !run_pass(1, 0) && pending_routes == 1,
        "Moving relocated while completing the focused cell's incident routes");

    // Sibling completion and suffix growth retain their route state, but they
    // cannot keep an ultimately unroutable placement active indefinitely.
    for (int pass = 2; pass < pass_limit - 1; ++pass) {
        require(!run_pass(0, 1),
            "Moving relocated before the focused placement routing slice ended");
    }
    require(run_pass(0, 1),
        "Moving let incremental sibling growth extend the placement routing slice");
}

void focused_moving_relocates_when_one_incident_route_stalls()
{
    size_t advancing_route_no_progress = 0;
    size_t stalled_route_no_progress = 0;
    constexpr int pass_limit = 5;

    // Reproduce the timeout after the placement-level fix: one incident route
    // grows every pass while another required route cannot add a fragment.
    // Aggregate progress is true, but the placement can never be accepted while
    // the stalled route remains incomplete, so each route needs its own budget.
    for (int pass = 0; pass < pass_limit; ++pass) {
        require(pnr::movingPassMadeProgress(0, 1),
            "test setup did not reproduce aggregate sibling progress");
        advancing_route_no_progress = pnr::updateMovingTaskNoProgressPasses(
            advancing_route_no_progress, true);
        stalled_route_no_progress = pnr::updateMovingTaskNoProgressPasses(
            stalled_route_no_progress, false);
        if (pass + 1 < pass_limit) {
            require(!pnr::movingTaskNoProgressExhausted(
                        stalled_route_no_progress, pass_limit),
                "Moving exhausted an incident route before its retry window");
        }
    }

    // Progress on the first route must not reset the second route's counter.
    // Relocating now avoids spending the entire stage extending siblings around
    // a placement whose required stalled route has exhausted its own tries.
    bool incident_route_exhausted = pnr::movingTaskNoProgressExhausted(
        stalled_route_no_progress, pass_limit);
    require(advancing_route_no_progress == 0 && incident_route_exhausted
            && pnr::focusedMovingShouldRelocate(false, 0, pass_limit,
                incident_route_exhausted, false),
        "productive sibling route hid a persistently stalled incident route");
}

void focused_moving_relocates_when_routes_wander_without_completion()
{
    constexpr int recursion_limit = 5;
    const int completion_limit = pnr::movingFocusNoCompletionLimit(recursion_limit);
    int no_completion_passes = 0;

    // Reproduce a focused placement whose incomplete routes append a bounded
    // suffix every pass without grounding. Growth keeps each committed prefix,
    // but one routing slice without a completion must try the next placement.
    for (int pass = 0; pass < completion_limit; ++pass) {
        require(pnr::movingPassMadeProgress(0, 1),
            "test setup did not reproduce a growing incomplete route");
        no_completion_passes = pnr::updateMovingNoCompletionPasses(
            no_completion_passes, true, 0);
        if (pass + 1 < completion_limit) {
            require(no_completion_passes < completion_limit,
                "Moving exhausted the completion window too early");
        }
    }
    require(no_completion_passes >= completion_limit
            && pnr::focusedMovingShouldRelocate(false, 0, recursion_limit,
                true, false),
        "Moving retained a placement whose routes grew without completing");

    // Completing any incident route starts a fresh window for the remaining
    // inputs, preserving useful work without permitting an unbounded focus.
    no_completion_passes = pnr::updateMovingNoCompletionPasses(
        no_completion_passes, true, 1);
    require(no_completion_passes == 0,
        "Moving did not reset its completion window after grounding a route");
}

void moving_scheduler_blocks_only_same_source_fanouts()
{
    // Check: a pending Generic seed blocks only branches from its own source pin.
    require(pnr::movingFanoutWaitsForSourceSeed(true, true, true),
        "Moving allowed a fanout to run before its own Generic seed");
    require(!pnr::movingFanoutWaitsForSourceSeed(true, true, false)
            && !pnr::movingFanoutWaitsForSourceSeed(false, true, true),
        "Moving blocked a fanout behind an unrelated source or another stage");

    // Check: outgoing-only work is handed to downstream loads instead of
    // relocating and invalidating the completed focused driver again.
    require(pnr::movingFocusHandsOffToLoads(false, true)
            && !pnr::movingFocusHandsOffToLoads(true, true)
            && !pnr::movingFocusHandsOffToLoads(false, false),
        "Moving selected the wrong focus handoff policy");

    // Check: a fresh placement receives its own no-progress budget instead of
    // inheriting the global Moving-stage pass count.
    require(!pnr::movingPlacementPassesExhausted(0, 5)
            && !pnr::focusedMovingShouldRelocate(false, 0, 5, false, false),
        "Moving relocated a fresh placement before its pass budget");

    // Check: a focus yields after a finite placement slice while a zero limit
    // remains the explicit representation of an unlimited atomic slice.
    require(!pnr::movingFocusSliceExhausted(15, 0, 16)
            && pnr::movingFocusSliceExhausted(16, 0, 16)
            && !pnr::movingFocusSliceExhausted(100, 0, 0),
        "Moving focus slicing did not bound a repeatedly relocated suffix");
}

void finished_focus_is_reopened_after_route_invalidation()
{
    // Check: successful relocation protects a cell only while every incident
    // route remains complete; a later tree repair must make it movable again.
    require(pnr::movingFinishedMarkIsValid(true, true),
        "Moving did not preserve a completed focus");
    require(!pnr::movingFinishedMarkIsValid(true, false),
        "Moving kept a stale completed-focus mark after route invalidation");

    // Check: route completion alone cannot invent a mark for a cell that has
    // not yet completed a focused Moving cycle.
    require(!pnr::movingFinishedMarkIsValid(false, true),
        "Moving invented a completed-focus mark");
}

void atomic_source_tree_requeues_already_empty_siblings()
{
    struct Task
    {
        int binding = 0;
        bool fanout = true;
    };

    bool generic_added = false;
    std::vector<Task> queue;
    std::vector<Task> already_empty{{1, true}, {2, true}};
    size_t empty_queued = pnr::enqueueInvalidatedSourceTreeTasks(
        already_empty, generic_added, false,
        [&](const Task& task) {
            queue.push_back(task);
            return true;
        });

    // Check: an earlier cleanup may have emptied these routes, but atomic tree
    // invalidation still returns both bindings to the scheduler.
    require(empty_queued == 2 && queue.size() == 2
            && !queue[0].fanout && queue[1].fanout,
        "Moving forgot already-empty source-tree siblings");

    std::vector<Task> newly_cleared{{3, true}};
    size_t cleared_queued = pnr::enqueueInvalidatedSourceTreeTasks(
        newly_cleared, generic_added, true,
        [&](const Task& task) {
            queue.push_back(task);
            return true;
        });

    // Check: subsequent logical nets from the same source remain Fanouts after
    // the first requeued binding established the replacement Generic seed.
    require(cleared_queued == 1 && queue.size() == 3 && queue[2].fanout,
        "Moving assigned multiple Generic seeds to one invalidated source tree");
}

} // namespace

int main()
{
    try {
        moving_one_fanout_releases_only_its_suffix();
        crossbar_destination_owner_uses_landing_node();
        focused_moving_bounds_each_placement_while_routes_advance();
        focused_moving_relocates_when_one_incident_route_stalls();
        focused_moving_relocates_when_routes_wander_without_completion();
        moving_scheduler_blocks_only_same_source_fanouts();
        finished_focus_is_reopened_after_route_invalidation();
        atomic_source_tree_requeues_already_empty_siblings();
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "moving_test failed: %s\n", error.what());
        return 1;
    }
    std::puts("moving_test passed");
    return 0;
}
