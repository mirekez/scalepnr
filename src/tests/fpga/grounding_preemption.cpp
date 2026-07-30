#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/RoutePassState.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <memory>
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

NodeMask bit(int node)
{
    return NodeMask{0, 1} << node;
}

bool leased(const fpga::Tile& tile, int dst)
{
    return (tile.cb.dst.jump & bit(dst)) != NodeMask{};
}

struct OwnedDst
{
    Referable<rtl::Net> net;
    rtl::Inst owner;
    int dst = -1;
};

struct Fixture
{
    static constexpr int local = 80;
    static constexpr int dst0 = 20;
    static constexpr int dst1 = 21;
    static constexpr int dst2 = 22;

    fpga::Tile* tile = nullptr;
    std::vector<std::unique_ptr<OwnedDst>> routes;

    Fixture()
    {
        fpga::Device& device = fpga::Device::current();
        device.tile_grid.clear();
        device.cb_types.clear();
        device.cb_types.emplace_back();
        fpga::CBType& type = device.cb_types.back();
        type.name = "matrix_alpha";
        type.type_id = 0;
        type.base_type_id = 0;
        type.dst_local[dst0].local |= bit(local);
        type.dst_local[dst1].local |= bit(local);
        type.dst_local[dst2].local |= bit(local);
        type.rebuildOutgoingSrcs();

        device.grid_spec.size = {1, 1};
        device.size_width = 1;
        device.size_height = 1;
        device.tile_grid.resize(1);
        tile = &device.tile_grid.front();
        tile->coord = {0, 0};
        tile->cb_coord = tile->coord;
        tile->name = tile->coord;
        tile->cb = {};
        tile->cb.type = &type;
        tile->cb_type = &type;
        tile->pin_state = {};
        tile->routedNets.clear();
    }

    OwnedDst& occupy(int dst, bool transit, const std::string& name)
    {
        auto route = std::make_unique<OwnedDst>();
        route->net.name = name;
        route->dst = dst;
        route->owner.wires.emplace_back();

        fpga::Wire fragment;
        fragment.type = fpga::Wire::WIRE_CROSSBAR;
        fragment.from = tile->coord;
        fragment.to = transit ? fpga::Coord{1, 0} : tile->coord;
        fragment.local = dst;
        fragment.jump = transit ? 100 + dst : -1;
        fragment.pos = 1;
        fragment.net_name = name;
        route->owner.wires.back().push_back(fragment);

        tile->cb.dst.jump |= bit(dst);
        if (transit) {
            tile->cb.src.jump |= bit(fragment.jump);
        }
        fpga::attachNetRoute(route->net, route->owner, 0, nullptr, nullptr, {}, {}, name);
        fpga::registerNetRouteTiles(route->net, route->owner.wires.back());
        routes.push_back(std::move(route));
        return *routes.back();
    }

    int select(rtl::Net*& victim, NodeMask incoming = {})
    {
        victim = nullptr;
        auto find_victim = [&](int dst) {
            rtl::Net* owner = fpga::findNetByNode(*tile, fpga::CB_NODE_DST, dst, true);
            if (!owner) {
                return false;
            }
            victim = owner;
            return true;
        };
        if (incoming == NodeMask{}) {
            return pnr::groundingPreemptionDst(*tile->cb_type, tile->cb, local, find_victim);
        }
        return pnr::groundingPreemptionDst(*tile->cb_type, tile->cb, local,
            incoming, find_victim);
    }
};

void free_destination_suppresses_preemption()
{
    Fixture fixture;
    OwnedDst& first = fixture.occupy(Fixture::dst0, true, "transit_first");
    OwnedDst& second = fixture.occupy(Fixture::dst1, true, "transit_second");
    rtl::Net* victim = nullptr;

    int selected = fixture.select(victim);

    // Check: one free destination leading to the local forbids grounding preemption.
    require(selected < 0 && victim == nullptr,
        "grounding selected a victim while a destination entry was free");
    require(!first.owner.wires.front().empty() && !second.owner.wires.front().empty(),
        "grounding unrouted transit despite a free destination entry");
    require(!leased(*fixture.tile, Fixture::dst2),
        "free destination entry became leased during candidate selection");
}

void endpoint_destinations_are_not_victims()
{
    Fixture fixture;
    fixture.occupy(Fixture::dst0, false, "endpoint_first");
    fixture.occupy(Fixture::dst1, false, "endpoint_second");
    fixture.occupy(Fixture::dst2, false, "endpoint_third");
    rtl::Net* victim = nullptr;

    int selected = fixture.select(victim);

    // Check: destination nodes grounding other local routes are never transit victims.
    require(selected < 0 && victim == nullptr,
        "grounding selected a destination used by a same-tile endpoint route");
    require(leased(*fixture.tile, Fixture::dst0)
            && leased(*fixture.tile, Fixture::dst1)
            && leased(*fixture.tile, Fixture::dst2),
        "grounding changed endpoint destination leases while inspecting owners");
}

void unreachable_free_destination_does_not_suppress_preemption()
{
    Fixture fixture;
    OwnedDst& first = fixture.occupy(Fixture::dst0, true, "transit_reachable");
    fixture.occupy(Fixture::dst1, false, "endpoint_reachable");
    rtl::Net* victim = nullptr;

    int selected = fixture.select(victim, bit(Fixture::dst0) | bit(Fixture::dst1));

    // Check: both physically incoming entries are occupied while the excluded logical entry stays free.
    require(leased(*fixture.tile, Fixture::dst0) && leased(*fixture.tile, Fixture::dst1)
            && !leased(*fixture.tile, Fixture::dst2),
        "physical-incoming grounding setup has incorrect destination leases");
    // Check: a free logical entry with no incoming physical mapping cannot suppress the transit victim.
    require(selected == Fixture::dst0 && victim == &first.net,
        "unreachable free destination suppressed the reachable transit victim");
    require(!leased(*fixture.tile, Fixture::dst2),
        "unreachable destination was modified while selecting a victim");
}

void endpoint_joint_owners_cannot_preempt_each_other()
{
    fpga::CBType type{"endpoint_joint_cycle"};
    type.dst_joint[20].joint |= bit(15);
    type.joint_local[15].local |= bit(80) | bit(81);
    type.rebuildOutgoingSrcs();
    fpga::CBState state;
    state.dst.jump |= bit(20);
    state.joint.jump |= bit(15);
    bool endpoint_owner_inspected = false;

    int selected = pnr::groundingPreemptionDst(type, state, 81, bit(20), [&](int dst) {
        endpoint_owner_inspected = dst == 20;
        return false; // The owner terminates at local 80, so it is not transit.
    });

    // Check: two local routes sharing one possible terminal joint cannot evict
    // each other and alternate forever; only a transit-owned dst is eligible.
    require(endpoint_owner_inspected && selected < 0,
        "grounding allowed an endpoint-joint preemption cycle");
}

void successful_preemption_retries_grounding_immediately()
{
    int retries = 0;
    require(!pnr::retryGroundingAfterPreemption(false, [&]() {
                ++retries;
                return true;
            }) && retries == 0,
        "grounding retried when no route was preempted");
    require(pnr::retryGroundingAfterPreemption(true, [&]() {
                ++retries;
                return true;
            }) && retries == 1,
        "grounding did not claim the terminal immediately after preemption");
    require(!pnr::retryGroundingAfterPreemption(true, [&]() {
                ++retries;
                return false;
            }) && retries == 2,
        "grounding reported completion when its immediate retry failed");
}

void all_transit_owners_are_removed_before_grounding_claim()
{
    Fixture fixture;
    OwnedDst& first = fixture.occupy(Fixture::dst0, true, "shared_owner_first");
    OwnedDst& second = fixture.occupy(Fixture::dst0, true, "shared_owner_second");

    std::vector<fpga::NetRouteRef> owners = fpga::findNetRoutesByNode(
        *fixture.tile, fpga::CB_NODE_DST, Fixture::dst0, true);
    require(owners.size() == 2,
        "grounding did not enumerate every transit binding on the destination");
    for (const fpga::NetRouteRef& owner : owners) {
        require(owner.net && fpga::unrouteNet(*owner.net),
            "grounding could not remove a transit owner of the selected destination");
    }

    // Check: all replicated route owners are gone before the current route
    // claims the single physical destination lease.
    require(first.owner.wires.front().empty() && second.owner.wires.front().empty()
            && !leased(*fixture.tile, Fixture::dst0)
            && fpga::findNetRoutesByNode(*fixture.tile, fpga::CB_NODE_DST,
                Fixture::dst0, true).empty(),
        "grounding retained a transit owner after atomic destination preemption");
}

void grounding_skips_a_victim_that_does_not_enable_docking()
{
    Fixture fixture;
    fixture.occupy(Fixture::dst0, true, "undockable_transit");
    fixture.occupy(Fixture::dst1, true, "dockable_transit");
    fixture.occupy(Fixture::dst2, false, "endpoint_owner");
    std::vector<int> probed;

    int selected = pnr::groundingPreemptionDst(
        *fixture.tile->cb_type, fixture.tile->cb, Fixture::local,
        [&](int dst) {
            probed.push_back(dst);
            // Emulate the bounded docking probe: releasing dst0 cannot connect
            // to the forward anchor, while releasing dst1 completes the path.
            return dst == Fixture::dst1;
        });

    // Check: grounding leaves an undockable owner intact and selects only the
    // transit destination whose temporary release creates a complete path.
    require(selected == Fixture::dst1
            && std::find(probed.begin(), probed.end(), Fixture::dst0) != probed.end()
            && std::find(probed.begin(), probed.end(), Fixture::dst1) != probed.end()
            && leased(*fixture.tile, Fixture::dst0),
        "grounding selected a victim without proving the released path docks");
}

void transit_source_tree_is_unrouted_and_requeued_atomically()
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.grid_spec.size = {3, 1};
    device.size_width = 3;
    device.size_height = 1;
    device.tile_grid.resize(3);
    fpga::Tile& source = device.tile_grid[0];
    fpga::Tile& transit = device.tile_grid[1];
    fpga::Tile& target = device.tile_grid[2];
    source.coord = source.cb_coord = {0, 0};
    transit.coord = transit.cb_coord = {1, 0};
    target.coord = target.cb_coord = {2, 0};

    Referable<rtl::Net> net;
    net.name = "tree_signal";
    rtl::Inst driver;
    std::array<rtl::Inst, 3> sinks;
    driver.wires.resize(sinks.size());
    std::vector<std::string> route_names;

    for (size_t index = 0; index < sinks.size(); ++index) {
        int source_local = 80 + static_cast<int>(index);
        int source_src = 10 + static_cast<int>(index);
        int transit_dst = 20 + static_cast<int>(index);
        int transit_src = 30 + static_cast<int>(index);
        int transit_joint = 40 + static_cast<int>(index);
        int target_dst = 50 + static_cast<int>(index);
        int target_joint = 60 + static_cast<int>(index);
        int target_local = 70 + static_cast<int>(index);
        std::string route_name = "tree_branch_" + std::to_string(index);
        route_names.push_back(route_name);

        fpga::Wire takeoff;
        takeoff.type = fpga::Wire::WIRE_CROSSBAR;
        takeoff.from = source.coord;
        takeoff.to = transit.coord;
        takeoff.local = source_local;
        takeoff.jump = source_src;
        takeoff.pos = 0;
        takeoff.net_name = route_name;

        fpga::Wire middle;
        middle.type = fpga::Wire::WIRE_CROSSBAR;
        middle.from = transit.coord;
        middle.to = target.coord;
        middle.local = transit_dst;
        middle.jump = transit_src;
        middle.joint = transit_joint;
        middle.pos = 1;
        middle.net_name = route_name;

        fpga::Wire entry;
        entry.type = fpga::Wire::WIRE_CROSSBAR;
        entry.from = target.coord;
        entry.to = target.coord;
        entry.local = target_dst;
        entry.joint = target_joint;
        entry.pos = 1;
        entry.net_name = route_name;

        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = target.coord;
        pin.to = target.coord;
        pin.local = target_local;
        pin.pos = 2;
        pin.net_name = route_name;

        driver.wires[index] = {takeoff, middle, entry, pin};
        source.cb.local.local |= bit(source_local);
        source.cb.src.jump |= bit(source_src);
        transit.cb.dst.jump |= bit(transit_dst);
        transit.cb.src.jump |= bit(transit_src);
        transit.cb.joint.jump |= bit(transit_joint);
        target.cb.dst.jump |= bit(target_dst);
        target.cb.joint.jump |= bit(target_joint);
        target.cb.local.local |= bit(target_local);
        target.pin_state.leased_nodes |= bit(target_local);
        fpga::attachNetRoute(net, driver, index, &driver, &sinks[index],
            "OUT", "IN", route_name);
        fpga::registerNetRouteTiles(net, driver.wires[index]);
    }

    require(fpga::unrouteNetRouteTree(net, {0, 1, 2}),
        "grounding failed to atomically unroute the transit source tree");

    // Check: atomic grounding preemption releases every branch before any reroute task is queued.
    require(std::all_of(driver.wires.begin(), driver.wires.end(),
                [](const std::vector<fpga::Wire>& route) { return route.empty(); })
            && source.cb.local.local == NodeMask{} && source.cb.src.jump == NodeMask{}
            && transit.cb.dst.jump == NodeMask{} && transit.cb.src.jump == NodeMask{}
            && transit.cb.joint.jump == NodeMask{} && target.cb.dst.jump == NodeMask{}
            && target.cb.joint.jump == NodeMask{} && target.cb.local.local == NodeMask{}
            && target.pin_state.leased_nodes == NodeMask{},
        "grounding left leases from the atomically removed transit source tree");

    struct RequeuedTask
    {
        std::string name;
        bool fanout = true;
    };
    std::vector<RequeuedTask> removed_tasks;
    std::vector<RequeuedTask> queue;
    for (const std::string& route_name : route_names) {
        removed_tasks.push_back({route_name, true});
    }
    bool generic_task_added = false;
    size_t queued = pnr::enqueuePreemptedSourceTasks(removed_tasks, generic_task_added,
        [&](const RequeuedTask& task) {
            queue.push_back(task);
            return true;
        });

    // Check: the removed tree is requeued as one Generic seed and all dependent Fanout branches.
    require(queued == 3 && queue.size() == 3 && !queue[0].fanout
            && queue[1].fanout && queue[2].fanout
            && queue[0].name == route_names[0] && queue[1].name == route_names[1]
            && queue[2].name == route_names[2],
        "grounding did not requeue one Generic seed followed by its Fanout siblings");
}

void only_transit_destination_is_preempted()
{
    Fixture fixture;
    OwnedDst& endpoint0 = fixture.occupy(Fixture::dst0, false, "endpoint_alpha");
    OwnedDst& transit = fixture.occupy(Fixture::dst1, true, "transit_selected");
    OwnedDst& endpoint2 = fixture.occupy(Fixture::dst2, false, "endpoint_beta");
    rtl::Net* victim = nullptr;

    int selected = fixture.select(victim);
    require(selected == Fixture::dst1 && victim == &transit.net,
        "grounding did not select the only transit-owned destination");
    require(fpga::unrouteNet(*victim),
        "grounding failed to unroute the selected transit route");

    // Check: exactly the transit route and its destination lease are released.
    require(transit.owner.wires.front().empty() && !leased(*fixture.tile, Fixture::dst1),
        "grounding retained the selected transit destination");
    require(!endpoint0.owner.wires.front().empty() && leased(*fixture.tile, Fixture::dst0),
        "grounding unrouted the first endpoint destination");
    require(!endpoint2.owner.wires.front().empty() && leased(*fixture.tile, Fixture::dst2),
        "grounding unrouted the second endpoint destination");
    require(fpga::findNetByNode(*fixture.tile, fpga::CB_NODE_DST, Fixture::dst1, true) == nullptr,
        "grounding left the preempted transit route registered on the tile");
}

}

int main()
{
    try {
        free_destination_suppresses_preemption();
        endpoint_destinations_are_not_victims();
        unreachable_free_destination_does_not_suppress_preemption();
        endpoint_joint_owners_cannot_preempt_each_other();
        successful_preemption_retries_grounding_immediately();
        all_transit_owners_are_removed_before_grounding_claim();
        grounding_skips_a_victim_that_does_not_enable_docking();
        transit_source_tree_is_unrouted_and_requeued_atomically();
        only_transit_destination_is_preempted();
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "grounding_preemption failed: %s\n", failure.message.c_str());
        return EXIT_FAILURE;
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "grounding_preemption exception: %s\n", error.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
