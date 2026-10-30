#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/RouteDesign.h"
#include "route/RoutePassState.h"
#include "outline/OutlineDesign.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

void partial_preemption_regressions();
void preserved_route_repair_regression();
void route_owner_query_regressions();
void fanout_trunk_handoff_regression(bool repair_control);
void fanout_takeoff_handoff_regression(bool repair_control);
void fanout_local_trunk_regression(bool blocked);
void fanout_search_budget_regression(int mode);
void fanout_transit_local_regression(int mode);
void basic_transit_local_regression(int mode);
void basic_search_budget_regression(int mode);
void fanout_source_priority_regression(int mode);
void fanout_numeric_seed_regression(int mode);
void fanout_fork_role_regression();
void fanout_numeric_grounding_regression(int mode);
void fanout_docking_owner_regression(int mode);
void fanout_shared_input_repair_regression(bool repair_control, bool blocked_control=false);
void fixed_endpoint_recovery_regression(int mode);
void fixed_endpoint_preemption_regression(int mode);
void fanout_shared_terminal_gap_regression(int mode);
void fanout_private_grounding_repair_regression(bool direct_input);
void docking_cut_checks_regression();
void source_free_budget_escalation_regression();
void source_grounding_budget_escalation_regression();
void source_frontier_reuse_regression();
void gap_source_frontier_reuse_regression();
void moving_fixed_local_cycle_regression();
void moving_source_probe_regression();
void moving_deferred_fixed_wire_regression();
void moving_source_boundary_continuation_regression();
void moving_source_candidate_input_work_regression(bool output_conflict);
void moving_source_current_wire_regression();
void moving_source_two_ended_regression();
void moving_source_resolved_local_regression();

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

void package_site_position_uses_tile_model_order()
{
    fpga::TileType type{"EDGE", 2};
    type.sites.push_back(fpga::SiteModel{.name = "X0Y0", .type = "PIN", .pos = 0});
    type.sites.push_back(fpga::SiteModel{.name = "X0Y1", .type = "PIN", .pos = 1});
    fpga::Tile tile;
    tile.tile_type = &type;
    tile.sites = {"PIN_X4Y102", "PIN_X4Y101"};

    // Global parity and physical-site vector ordering must not swap tile-local positions.
    require(pnr::packageSitePosition(tile, "PIN_X4Y101") == 0,
        "lower physical package site did not select modeled position zero");
    require(pnr::packageSitePosition(tile, "PIN_X4Y102") == 1,
        "upper physical package site did not select modeled position one");
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
        fragment.dst = transit ? dst : -1;
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
            if (!owner || !owner->routeCanBePreempted()) {
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

void protected_transit_destination_is_not_a_victim()
{
    Fixture fixture;
    OwnedDst& protected_route = fixture.occupy(
        Fixture::dst0, true, "reserved_infrastructure");
    protected_route.net.route_protected = true;
    fixture.occupy(Fixture::dst1, false, "endpoint_first");
    fixture.occupy(Fixture::dst2, false, "endpoint_second");
    rtl::Net* victim = nullptr;

    int selected = fixture.select(victim);

    // Check: a reserved infrastructure route cannot become a grounding victim.
    require(selected < 0 && victim == nullptr
            && !protected_route.owner.wires.front().empty()
            && leased(*fixture.tile, Fixture::dst0),
        "grounding preempted a protected infrastructure route");
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

void only_completed_destination_bindings_protect_terminal_resources()
{
    // Check: naming this tile as the sink cannot protect a partial route that
    // has occupied a terminal joint without reaching its local resource.
    require(!pnr::groundingOwnerProtectsEndpoint(true, false),
        "unfinished destination binding was protected from grounding preemption");
    // Check: a complete route grounded at this tile remains protected, while a
    // complete route ending elsewhere is still transit ownership here.
    require(pnr::groundingOwnerProtectsEndpoint(true, true)
            && !pnr::groundingOwnerProtectsEndpoint(false, true),
        "grounding endpoint protection did not require both sink identity and completion");
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

void free_dst_with_busy_joint_does_not_suppress_preemption()
{
    fpga::CBType type{"terminal_path_matrix"};
    constexpr int local = 80;
    constexpr int transit_dst = 20;
    constexpr int free_dst = 21;
    constexpr int free_joint = 15;
    constexpr int busy_joint = 16;
    type.dst_joint[transit_dst].joint |= bit(free_joint);
    type.dst_joint[free_dst].joint |= bit(busy_joint);
    type.joint_local[free_joint].local |= bit(local);
    type.joint_local[busy_joint].local |= bit(local);
    type.rebuildOutgoingSrcs();

    fpga::CBState state;
    state.dst.jump |= bit(transit_dst);
    state.joint.jump |= bit(busy_joint);
    std::vector<pnr::GroundingTerminalPath> inspected;
    pnr::GroundingTerminalPath selected = pnr::groundingPreemptionPath(
        type, state, local, bit(transit_dst) | bit(free_dst),
        [&](const pnr::GroundingTerminalPath& path) {
            inspected.push_back(path);
            return path.dst == transit_dst;
        });

    // Check: a free DST is not a free terminal path when its required joint is
    // occupied, so the reverse-reachable transit path remains preemptible.
    require(selected.dst == transit_dst && selected.joint == free_joint
            && inspected.size() == 1,
        "a free destination with a busy joint suppressed grounding preemption");
}

void completely_free_terminal_path_suppresses_preemption()
{
    fpga::CBType type{"free_terminal_path_matrix"};
    constexpr int local = 81;
    type.dst_joint[30].joint |= bit(17);
    type.dst_joint[31].joint |= bit(18);
    type.joint_local[17].local |= bit(local);
    type.joint_local[18].local |= bit(local);
    type.rebuildOutgoingSrcs();

    fpga::CBState state;
    state.dst.jump |= bit(30);
    bool inspected = false;
    pnr::GroundingTerminalPath selected = pnr::groundingPreemptionPath(
        type, state, local, bit(30) | bit(31),
        [&](const pnr::GroundingTerminalPath&) {
            inspected = true;
            return true;
        });

    // Check: dst 31 and joint 18 form a complete free incoming path, so no
    // occupied transit path may be inspected or displaced.
    require(selected.dst < 0 && !inspected,
        "grounding preempted while a complete terminal path was free");
}

void reserved_joint_path_does_not_suppress_preemption()
{
    fpga::CBType type{"reserved_terminal_path_matrix"};
    constexpr int local = 82;
    constexpr int transit_dst = 40;
    constexpr int reserved_dst = 41;
    constexpr int transit_joint = 19;
    constexpr int reserved_joint = 20;
    type.dst_joint[transit_dst].joint |= bit(transit_joint);
    type.dst_joint[reserved_dst].joint |= bit(reserved_joint);
    type.joint_local[transit_joint].local |= bit(local);
    type.joint_local[reserved_joint].local |= bit(local);
    type.rebuildOutgoingSrcs();

    fpga::CBState state;
    state.dst.jump |= bit(transit_dst);
    pnr::GroundingTerminalPath selected = pnr::groundingPreemptionPath(
        type, state, local, bit(transit_dst) | bit(reserved_dst),
        bit(reserved_joint), [&](const pnr::GroundingTerminalPath& path) {
            return path.dst == transit_dst;
        });

    // Check: a joint reserved by the packed endpoint is unavailable even when
    // its live lease bit is clear, so it cannot hide a preemptible transit path.
    require(selected.dst == transit_dst && selected.joint == transit_joint,
        "a packed-joint reservation suppressed grounding preemption");
}

void terminal_preemption_requires_the_exact_joint_path()
{
    const pnr::GroundingTerminalPath blocked{30, 17, 18};

    // Check: a direct path sharing the destination is not the jointed path
    // whose backward frontier docking proved reachable.
    require(!pnr::sameGroundingTerminalPath(blocked, {30, -1, -1}),
        "direct terminal path matched a blocked two-joint path");
    // Check: both joint positions participate in identity; preempting either
    // alternative would release resources unrelated to the proven path.
    require(!pnr::sameGroundingTerminalPath(blocked, {30, 17, 19})
            && !pnr::sameGroundingTerminalPath(blocked, {30, 19, 18})
            && pnr::sameGroundingTerminalPath(blocked, {30, 17, 18}),
        "terminal preemption did not compare the complete joint sequence");
}

// Full numeric routes exercise the production grounding cut and task scheduler,
// unlike selection-only fixtures which cannot detect a retained landing lease.
struct TerminalCutFixture
{
    struct Route {
        Referable<rtl::Net> net;
        rtl::Inst driver;
        rtl::Inst sink;
    };
    pnr::RouteDesign router;
    std::vector<std::unique_ptr<Route>> routes;
    rtl::Inst requester;
    rtl::Inst destination;
    pnr::RouteDesign::RouteTask task;
    fpga::Tile* target;
    fpga::CBType* type;
    static constexpr int local = 80;

    TerminalCutFixture()
    {
        auto& device = fpga::Device::current();
        device.tile_grid.clear();
        device.cb_types.clear();
        device.cb_types.emplace_back();
        type = &device.cb_types.back();
        type->name = "terminal_matrix";
        device.grid_spec.size = {4, 1};
        device.size_width = 4;
        device.size_height = 1;
        device.tile_grid.resize(4);
        for (int x = 0; x < 4; ++x) {
            auto& tile = device.tile_grid[x];
            tile.coord = tile.cb_coord = {x, 0};
            tile.cb_type = type;
            tile.cb.type = type;
        }
        target = &device.tile_grid[2];
        task.from = &requester;
        task.to = &destination;
        task.from_port = "OUTPUT";
        task.to_port = "INPUT";
        task.net_name = "requesting_route";
    }

    Route& occupy(int dst, bool takeoff_landing = false, int joint = -1, int joint2 = -1)
    {
        auto& device = fpga::Device::current();
        auto route = std::make_unique<Route>();
        const int lane = static_cast<int>(routes.size());
        route->net.name = "signal_" + std::to_string(lane);
        route->driver.tile.set(&device.tile_grid[takeoff_landing ? 1 : 0]);
        route->sink.tile.set(&device.tile_grid[3]);
        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = pin.to = {takeoff_landing ? 1 : 0, 0};
        pin.local = 300 + lane;
        pin.pin_dir = 1;
        fpga::Wire takeoff;
        takeoff.type = fpga::Wire::WIRE_CROSSBAR;
        takeoff.from = pin.from;
        takeoff.to = {takeoff_landing ? 2 : 1, 0};
        takeoff.local = pin.local;
        takeoff.jump = 100 + lane;
        takeoff.dst = takeoff_landing ? dst : 200 + lane;
        takeoff.pos = 0;
        fpga::Wire approach = takeoff;
        approach.from = {1, 0};
        approach.to = {2, 0};
        approach.local = 200 + lane;
        approach.jump = 120 + lane;
        approach.dst = dst;
        approach.pos = 1;
        fpga::Wire transit = approach;
        transit.from = {2, 0};
        transit.to = {3, 0};
        transit.local = dst;
        transit.jump = 140 + lane;
        transit.dst = 220 + lane;
        transit.joint = joint;
        transit.joint2 = joint2;
        fpga::Wire entry = transit;
        entry.from = entry.to = {3, 0};
        entry.local = transit.dst;
        entry.jump = entry.dst = entry.joint = entry.joint2 = -1;
        fpga::Wire sink_pin = pin;
        sink_pin.from = sink_pin.to = {3, 0};
        sink_pin.local = 320 + lane;
        sink_pin.pin_dir = 0;
        route->driver.wires.push_back({pin, takeoff});
        auto& wires = route->driver.wires[0];
        if (!takeoff_landing) wires.push_back(approach);
        wires.insert(wires.end(), {transit, entry, sink_pin});
        for (const auto& wire : wires) {
            auto& tile = *device.getTile(wire.from.x, wire.from.y);
            if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
                tile.cb.local.local |= bit(wire.local);
                tile.pin_state.leased_nodes |= bit(wire.local);
            } else {
                if (wire.pos) tile.cb.dst.jump |= bit(wire.local);
                if (wire.jump >= 0) tile.cb.src.jump |= bit(wire.jump);
                if (wire.joint >= 0) tile.cb.joint.jump |= bit(wire.joint);
                if (wire.joint2 >= 0) tile.cb.joint.jump |= bit(wire.joint2);
            }
        }
        fpga::attachNetRoute(route->net, route->driver, 0, &route->driver,
            &route->sink, "OUTPUT", "INPUT", "victim_" + std::to_string(lane));
        fpga::registerNetRouteTiles(route->net, wires);
        routes.push_back(std::move(route));
        return *routes.back();
    }

    pnr::GroundingTerminalPath preempt()
    {
        type->rebuildOutgoingSrcs();
        return router.preemptGroundingTerminal(*target, local,
            type->dsts_reaching_local[local].jump, {}, task);
    }
};

void grounding_releases_incoming_hop_not_just_its_continuation(bool preserved = false)
{
    TerminalCutFixture f;
    f.type->dst_local[20].local |= bit(f.local);
    auto& victim = f.occupy(20);
    // A retained run's transit must be selectable, cut and requeued just like
    // current-run transit. Its fixed endpoints must not be relocated.
    victim.net.pnr_preserved = preserved;
    f.router.preserve_routes = preserved;
    victim.driver.outline.fixed = victim.sink.outline.fixed = preserved;
    const auto driver_coord = victim.driver.tile->coord;
    const auto sink_coord = victim.sink.tile->coord;
    if (preserved) {
        victim.net.route_protected = true;
        require(f.preempt().dst < 0 && leased(*f.target, 20) &&
                f.router.pending_route_todo.empty(),
                "retained infrastructure became preemptible");
        victim.net.route_protected = false;
    }
    f.type->rebuildOutgoingSrcs();
    const size_t before = victim.driver.wires[0].size();
    std::vector<fpga::RouteCutNode> releases;
    const auto preview = f.router.preemptGroundingTerminal(*f.target, f.local,
        bit(20), {}, f.task, nullptr, &releases);
    require(preview.dst == 20 && leased(*f.target, 20) &&
            victim.driver.wires[0].size() == before && f.router.pending_route_todo.empty(),
        "terminal replacement preview mutated the victim or scheduled a retry");
    require(std::any_of(releases.begin(), releases.end(), [&](const auto& node) {
        return node.tile.x == f.target->coord.x && node.tile.y == f.target->coord.y &&
            node.type == fpga::CB_NODE_DST && node.node == 20;
    }), "terminal preview omitted the contested incoming landing");
    const auto path = f.preempt();
    // Check: success really frees the requested DST instead of retaining it as a landing.
    require(path.dst == 20 && !leased(*f.target, 20) &&
            fpga::findNetByNode(*f.target, fpga::CB_NODE_DST, 20, false) == nullptr,
        "grounding reported success but kept its contested landing");
    // Check: the source takeoff and its different landing remain a valid retry prefix.
    require(victim.driver.wires[0].size() == 4 &&
            fpga::hasRouteGaps(victim.driver.wires[0]) &&
            victim.driver.wires[0][2].to_node_type == fpga::CB_NODE_LOCAL &&
            victim.driver.wires[0][2].to_node == 320 &&
            victim.driver.wires[0][1].owns_landing &&
            fpga::Device::current().getTile(3, 0)->isPinNodeLeased(320) &&
            leased(*fpga::Device::current().getTile(1, 0), 200),
        "grounding removed the protected takeoff or its retry landing");
    // Check: the displaced binding is scheduled exactly once before returning success.
    require(f.router.pending_route_todo.size() == 1 &&
            f.router.pending_route_todo[0].net == &victim.net,
        "grounding lost or duplicated its displaced route task");
    require(victim.driver.tile->coord.x == driver_coord.x &&
            victim.driver.tile->coord.y == driver_coord.y &&
            victim.sink.tile->coord.x == sink_coord.x &&
            victim.sink.tile->coord.y == sink_coord.y,
            "transit preemption moved a fixed endpoint");
    if (preserved) {
        std::vector<pnr::RouteDesign::RouteTask> repairs;
        require(f.router.unrouteSourceTree(victim.net, &victim.driver,
                    victim.net.routes.front().from_port, &repairs, false) > 0 &&
                !repairs.empty(),
                "retained source tree could not be reset and requeued for recovery");
    }
}

void grounding_skips_takeoff_and_policy_rejections()
{
    for (bool protected_takeoff : {false, true}) {
        TerminalCutFixture f;
        f.type->dst_local[20].local |= bit(f.local);
        f.type->dst_local[21].local |= bit(f.local);
        auto& first = f.occupy(20, protected_takeoff);
        auto& second = f.occupy(21);
        const size_t first_size = first.driver.wires[0].size();
        if (!protected_takeoff) first.net.route_protected = true;
        const auto path = f.preempt();
        // Check: rejection of the first candidate must not hide another usable terminal.
        require(path.dst == 21 && leased(*f.target, 20) && !leased(*f.target, 21),
            "grounding stopped at a takeoff/policy rejection rather than trying another entry");
        // Check: the rejected owner's complete route is untouched and only the second is requeued.
        require(first.driver.wires[0].size() == first_size &&
                f.router.pending_route_todo.size() == 1 &&
                f.router.pending_route_todo[0].net == &second.net,
            "grounding changed a rejected victim or queued the wrong binding");
    }
}

void grounding_preflights_all_joint_owners_and_cuts_each_binding_once()
{
    for (bool reject : {false, true}) {
        TerminalCutFixture f;
        f.type->dst_joint[20].joint |= bit(40);
        f.type->joint_joint[40].joint |= bit(41);
        f.type->joint_local[41].local |= bit(f.local);
        auto& first = f.occupy(20);
        auto& second = f.occupy(21, false, 40, 41);
        const size_t old_size = first.driver.wires[0].size();
        if (reject) second.net.route_protected = true;
        const auto path = f.preempt();
        if (reject) {
            // Check: a protected joint owner prevents all cuts, including the otherwise removable DST.
            require(path.dst < 0 && leased(*f.target, 20) &&
                    first.driver.wires[0].size() == old_size && f.router.pending_route_todo.empty(),
                "grounding partially unrouted a terminal whose joint owner was protected");
        } else {
            // Check: one binding owning both joints is cut once; both joints and the DST are released.
            require(path.dst == 20 && !leased(*f.target, 20) &&
                    !f.target->cb.joint.jump.testBit(40) && !f.target->cb.joint.jump.testBit(41) &&
                    f.router.pending_route_todo.size() == 2,
                "grounding lost a multi-node owner or failed to release the complete terminal");
        }
    }
}

void grounding_shared_landing_is_released_only_after_all_owners_pass()
{
    for (bool protected_alias : {false, true}) {
        TerminalCutFixture f;
        f.type->dst_local[20].local |= bit(f.local);
        auto& first = f.occupy(20);
        auto alias = std::make_unique<TerminalCutFixture::Route>();
        alias->net.name = "shared_alias";
        alias->net.route_protected = protected_alias;
        alias->driver.wires.push_back(first.driver.wires[0]);
        for (auto& wire : alias->driver.wires[0]) wire.shared = true;
        fpga::attachNetRoute(alias->net, alias->driver, 0, &first.driver,
            &first.sink, "OUTPUT", "INPUT", "shared_alias_route");
        fpga::registerNetRouteTiles(alias->net, alias->driver.wires[0]);
        const size_t original_size = first.driver.wires[0].size();
        f.type->rebuildOutgoingSrcs();
        std::vector<fpga::RouteCutNode> releases;
        const auto preview = f.router.preemptGroundingTerminal(*f.target, f.local,
            bit(20), {}, f.task, nullptr, &releases);
        require((preview.dst >= 0) == !protected_alias && leased(*f.target, 20) &&
                first.driver.wires[0].size() == original_size &&
                alias->driver.wires[0].size() == original_size && f.router.pending_route_todo.empty(),
            "shared terminal preview cut a victim or missed its protected co-owner");
        if (!protected_alias) require(std::any_of(releases.begin(), releases.end(), [&](const auto& node) {
            return node.tile.x == f.target->coord.x && node.tile.y == f.target->coord.y &&
                node.type == fpga::CB_NODE_DST && node.node == 20;
        }), "shared terminal preview omitted the node released by all validated owners");
        const auto path = f.preempt();
        if (protected_alias) {
            // Check: discovering an ineligible shared owner must precede mutation of the eligible owner.
            require(path.dst < 0 && leased(*f.target, 20) &&
                    first.driver.wires[0].size() == original_size &&
                    alias->driver.wires[0].size() == original_size &&
                    f.router.pending_route_todo.empty(),
                "grounding cut only part of a protected shared landing");
        } else {
            // Check: every replica is trimmed and requeued before the shared DST is reported free.
            require(path.dst == 20 && !leased(*f.target, 20) &&
                    fpga::hasRouteGaps(first.driver.wires[0]) &&
                    fpga::hasRouteGaps(alias->driver.wires[0]) &&
                    fpga::Device::current().getTile(3, 0)->isPinNodeLeased(320) &&
                    f.router.pending_route_todo.size() == 2,
                "grounding retained a shared landing or lost a sibling retry task");
        }
    }
}

// Three sinks share the takeoff. Two also share the next hop; the third
// branches earlier. A private SRC cut must affect one sink, a shared SRC or
// arrival DST cut exactly two, never the earlier branch.
void numeric_cuts_follow_dependencies_not_lease_owner_flags()
{
    for (int kind : {0, 1, 2}) {
      for (bool reverse : {false, true}) {
        TerminalCutFixture f;
        auto& a = f.occupy(20);
        auto& device = fpga::Device::current();
        rtl::Inst b, c;
        b.tile.set(&device.tile_grid[3]); c.tile.set(&device.tile_grid[3]);
        b.wires.push_back(a.driver.wires[0]);
        auto& bw = b.wires[0];
        for (size_t i=0; i<3; ++i) bw[i].shared = true;
        bw[3].jump = 141; bw[3].dst = 221; bw[3].owns_dst = false; bw[3].pos = 2;
        bw[4].local = 221; bw[5].local = 321;
        c.wires.push_back({bw[0],bw[1],bw[3],bw[4],bw[5]});
        auto& cw = c.wires[0];
        cw[2].from = {1,0}; cw[2].local = 200; cw[2].jump = 180; cw[2].dst = 240;
        cw[3].local = 240; cw[4].local = 322;
        auto lease = [&](const std::vector<fpga::Wire>& wires) {
            for (const auto& w : wires) {
                auto& tile = *device.getTile(w.from.x,w.from.y);
                if (w.type == fpga::Wire::WIRE_TILE_PIN) {
                    tile.cb.local.local |= bit(w.local); tile.pin_state.leased_nodes |= bit(w.local);
                } else {
                    if (w.pos && w.owns_dst) tile.cb.dst.jump |= bit(w.local);
                    if (w.jump >= 0) tile.cb.src.jump |= bit(w.jump);
                }
            }
        };
        lease(bw); lease(cw);
        fpga::attachNetRoute(a.net,b,0,&a.driver,&b,"OUTPUT","INPUT","branch_beta");
        fpga::registerNetRouteTiles(a.net,bw,1);
        fpga::attachNetRoute(a.net,c,0,&a.driver,&c,"OUTPUT","INPUT","branch_gamma");
        fpga::registerNetRouteTiles(a.net,cw,2);
        if (reverse) for (auto& tile : device.tile_grid)
            std::reverse(tile.routed_bindings.begin(),tile.routed_bindings.end());
        std::vector<fpga::RouteCutNode> cuts = kind == 0
            ? std::vector<fpga::RouteCutNode>{{{2,0},fpga::CB_NODE_SRC,140}}
            : kind == 1
              ? std::vector<fpga::RouteCutNode>{{{1,0},fpga::CB_NODE_SRC,120}}
              : std::vector<fpga::RouteCutNode>{{{2,0},fpga::CB_NODE_DST,20}};
        auto victims = fpga::findNetRouteCutVictims(cuts);
        require(victims.size() == (kind == 0 ? 1 : 2),
            "cut selected unrelated siblings or missed a non-owning fork arrival");
        require(std::none_of(victims.begin(),victims.end(),[](const auto& v){return v.binding_index == 2;}),
            "cut selected a sibling branching before the contested hop");
        if (reverse) std::reverse(victims.begin(),victims.end());
        const auto released = fpga::previewNetRouteCuts(victims,cuts);
        auto contains = [&](int x, fpga::CBNodeNameType type, int node) {
          return std::any_of(released.begin(),released.end(),[&](const auto& r){
            return r.tile.x == x && r.tile.y == 0 && r.type == type && r.node == node; });
        };
        require(!contains(0,fpga::CB_NODE_SRC,100) && !contains(1,fpga::CB_NODE_DST,200) &&
                !contains(1,fpga::CB_NODE_SRC,180),"preview released a retained prefix or earlier branch");
        require(contains(2,fpga::CB_NODE_SRC,140) &&
                contains(2,fpga::CB_NODE_DST,20) == (kind != 0) &&
                contains(2,fpga::CB_NODE_SRC,141) == (kind != 0),
                "group preview omitted common landing or released an unaffected suffix");
        require(a.driver.wires[0].size()==6 && bw.size()==6 && cw.size()==5,
                "preview changed live routes");
        // Missing even the non-owning replica must reject without a partial cut.
        if (kind != 0) require(!fpga::unrouteNetRoutesFromNodes({victims.front()},cuts) &&
                a.driver.wires[0].size()==6 && bw.size()==6,
                "incomplete victim set partly cut shared ownership");
        require(fpga::unrouteNetRoutesFromNodes(victims,cuts),"valid dependency cut failed");
        require(a.driver.wires[0].size()==(kind==0 ? 3 : 2) &&
                bw.size()==(kind==0 ? 6 : 2) && cw.size()==5 && fpga::isRouteComplete(cw),
                "cut trimmed the wrong branches or lost a retained prefix");
        require(!device.getTile(2,0)->cb.src.jump.testBit(140) &&
                device.getTile(2,0)->cb.src.jump.testBit(141)==(kind==0) &&
                device.getTile(1,0)->cb.src.jump.testBit(180),"cut produced incorrect leases");
        for (auto& tile : device.tile_grid) {
            std::ostringstream proof;
            const auto audit = fpga::auditTileCongestion(tile,proof,{&a.net});
            require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                    "dependency cut corrupted ownership: " + proof.str());
        }
      }
    }
}

void grounding_production_free_entry_prevents_cuts()
{
    TerminalCutFixture f;
    f.type->dst_local[20].local |= bit(f.local);
    f.type->dst_local[21].local |= bit(f.local);
    auto& victim = f.occupy(20);
    const size_t original_size = victim.driver.wires[0].size();
    const auto path = f.preempt();
    // Check: production preemption preserves the victim when any complete terminal path is already free.
    require(path.dst < 0 && leased(*f.target, 20) && !leased(*f.target, 21) &&
            victim.driver.wires[0].size() == original_size && f.router.pending_route_todo.empty(),
        "grounding preempted despite an available free entry");
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

void mandatory_distributed_local_path_preserves_local_takeoff()
{
    constexpr int root_local = 4;
    constexpr int path_joint = 9;
    constexpr int target_local = 10;

    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    fpga::CBType& cb_type = device.cb_types.back();
    cb_type.name = "numeric_local_matrix";
    cb_type.type_id = 0;
    cb_type.base_type_id = 0;
    cb_type.constant_one_nodes |= bit(root_local);
    cb_type.local_joint[root_local].joint |= bit(path_joint);
    cb_type.joint_local[path_joint].local |= bit(target_local);

    fpga::TileType tile_type{"numeric_resource_tile", 0};
    tile_type.pin_map.nodes[{"sink_kind", "input", 0}] |= bit(target_local);
    device.grid_spec.size = {1, 1};
    device.size_width = 1;
    device.size_height = 1;
    device.tile_grid.resize(1);
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = tile.cb_coord = {0, 0};
    tile.name = tile.coord;
    tile.cb_type = &cb_type;
    tile.cb.type = &cb_type;
    tile.tile_type = &tile_type;

    Referable<rtl::Cell> source_cell;
    source_cell.type = "distributed_kind";
    Referable<rtl::Cell> sink_cell;
    sink_cell.type = "sink_kind";
    rtl::Inst distributed_source;
    distributed_source.cell_ref.set(&source_cell);
    rtl::Inst sink;
    sink.cell_ref.set(&sink_cell);
    sink.tile.set(&device.tile_grid.front());
    sink.pos = 0;

    Referable<rtl::Net> ordinary_net;
    rtl::Inst ordinary_driver;
    rtl::Inst ordinary_sink;
    rtl::Inst ordinary_owner;
    fpga::Wire blocker;
    blocker.type = fpga::Wire::WIRE_ROUTE_EDGE;
    blocker.from = tile.coord;
    blocker.to = tile.coord;
    blocker.from_node_type = fpga::CB_NODE_LOCAL;
    blocker.from_node = 7;
    blocker.to_node_type = fpga::CB_NODE_JOINT;
    blocker.to_node = path_joint;
    blocker.net_name = "ordinary_branch";
    fpga::Wire prefix = blocker;
    prefix.from_node = 6;
    prefix.to_node_type = fpga::CB_NODE_LOCAL;
    prefix.to_node = 7;
    ordinary_owner.wires.push_back({prefix, blocker});
    tile.cb.local.local |= bit(7);
    tile.cb.joint.jump |= bit(path_joint);
    fpga::attachNetRoute(ordinary_net, ordinary_owner, 0, &ordinary_driver,
                         &ordinary_sink, "out", "input", "ordinary_branch");
    fpga::registerNetRouteTiles(ordinary_net, ordinary_owner.wires.front());

    Referable<rtl::Net> distributed_net;
    distributed_net.route_protected = true;
    distributed_net.distributed_source = true;
    pnr::RouteDesign router;
    router.fpga = &device;
    pnr::RouteDesign::RouteTask task{&distributed_source, &sink,
        &distributed_net, "out", "input", "mandatory_local"};

    require(!router.routeDistributedLocalTask(task)
            && !ordinary_owner.wires.front().empty()
            && router.pending_route_todo.empty(),
        "Basic distributed routing displaced an established ordinary trunk");
    // This blocker starts at a LOCAL, so it is an endpoint path, not transit.
    // Moving must not turn constant priority into permission to steal it.
    router.moving_stage = true;
    require(!router.routeDistributedLocalTask(task),
        "mandatory local routing stole an endpoint joint");
    require(ordinary_owner.wires.front().size() == 2 && router.pending_route_todo.empty(),
        "mandatory local routing destroyed the protected endpoint prefix");
    require((tile.cb.joint.jump & bit(path_joint)) != NodeMask{}
            && !tile.isPinNodeLeased(target_local) && sink.wires.empty(),
        "failed mandatory local routing published a partial endpoint");
}

void mandatory_distributed_local_path_keeps_protected_blocker()
{
    constexpr int root_local = 14;
    constexpr int path_joint = 19;
    constexpr int target_local = 20;

    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    fpga::CBType& cb_type = device.cb_types.back();
    cb_type.name = "protected_numeric_matrix";
    cb_type.constant_one_nodes |= bit(root_local);
    cb_type.local_joint[root_local].joint |= bit(path_joint);
    cb_type.joint_local[path_joint].local |= bit(target_local);

    fpga::TileType tile_type{"protected_resource_tile", 0};
    tile_type.pin_map.nodes[{"sink_kind", "input", 0}] |= bit(target_local);
    device.grid_spec.size = {1, 1};
    device.size_width = 1;
    device.size_height = 1;
    device.tile_grid.resize(1);
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = tile.cb_coord = {0, 0};
    tile.name = tile.coord;
    tile.cb_type = &cb_type;
    tile.cb.type = &cb_type;
    tile.tile_type = &tile_type;

    Referable<rtl::Cell> source_cell;
    source_cell.type = "distributed_kind";
    Referable<rtl::Cell> sink_cell;
    sink_cell.type = "sink_kind";
    rtl::Inst distributed_source;
    distributed_source.cell_ref.set(&source_cell);
    rtl::Inst sink;
    sink.cell_ref.set(&sink_cell);
    sink.tile.set(&device.tile_grid.front());
    sink.pos = 0;

    Referable<rtl::Net> protected_net;
    protected_net.route_protected = true;
    rtl::Inst protected_driver;
    rtl::Inst protected_sink;
    rtl::Inst protected_owner;
    fpga::Wire blocker;
    blocker.type = fpga::Wire::WIRE_ROUTE_EDGE;
    blocker.from = tile.coord;
    blocker.to = tile.coord;
    blocker.from_node_type = fpga::CB_NODE_LOCAL;
    blocker.from_node = 17;
    blocker.to_node_type = fpga::CB_NODE_JOINT;
    blocker.to_node = path_joint;
    protected_owner.wires.push_back({blocker});
    tile.cb.joint.jump |= bit(path_joint);
    fpga::attachNetRoute(protected_net, protected_owner, 0, &protected_driver,
                         &protected_sink, "out", "input", "protected_branch");
    fpga::registerNetRouteTiles(protected_net, protected_owner.wires.front());

    Referable<rtl::Net> distributed_net;
    distributed_net.route_protected = true;
    distributed_net.distributed_source = true;
    pnr::RouteDesign router;
    router.fpga = &device;
    pnr::RouteDesign::RouteTask task{&distributed_source, &sink,
        &distributed_net, "out", "input", "mandatory_local"};

    require(!router.routeDistributedLocalTask(task),
        "mandatory distributed route displaced a protected local owner");
    // Check: protected infrastructure remains registered and no reroute task
    // is fabricated when the mandatory path is unavailable.
    require(!protected_owner.wires.front().empty()
            && router.pending_route_todo.empty()
            && (tile.cb.joint.jump & bit(path_joint)) != NodeMask{},
        "failed mandatory local attempt changed protected route ownership");
}

void mandatory_distributed_local_path_keeps_ordinary_endpoint()
{
    constexpr int root_local = 24;
    constexpr int path_joint = 29;
    constexpr int target_local = 30;

    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    fpga::CBType& cb_type = device.cb_types.back();
    cb_type.name = "endpoint_numeric_matrix";
    cb_type.constant_one_nodes |= bit(root_local);
    cb_type.local_joint[root_local].joint |= bit(path_joint);
    cb_type.joint_local[path_joint].local |= bit(target_local);

    fpga::TileType tile_type{"endpoint_resource_tile", 0};
    tile_type.pin_map.nodes[{"sink_kind", "input", 0}] |= bit(target_local);
    device.grid_spec.size = {1, 1};
    device.size_width = 1;
    device.size_height = 1;
    device.tile_grid.resize(1);
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = tile.cb_coord = {0, 0};
    tile.name = tile.coord;
    tile.cb_type = &cb_type;
    tile.cb.type = &cb_type;
    tile.tile_type = &tile_type;

    Referable<rtl::Cell> source_cell;
    source_cell.type = "distributed_kind";
    Referable<rtl::Cell> sink_cell;
    sink_cell.type = "sink_kind";
    rtl::Inst distributed_source;
    distributed_source.cell_ref.set(&source_cell);
    rtl::Inst distributed_sink;
    distributed_sink.cell_ref.set(&sink_cell);
    distributed_sink.tile.set(&device.tile_grid.front());
    distributed_sink.pos = 0;

    Referable<rtl::Net> ordinary_net;
    rtl::Inst ordinary_driver;
    rtl::Inst ordinary_sink;
    ordinary_sink.tile.set(&device.tile_grid.front());
    ordinary_sink.pos = 1;
    rtl::Inst ordinary_owner;
    fpga::Wire blocker;
    blocker.type = fpga::Wire::WIRE_ROUTE_EDGE;
    blocker.from = tile.coord;
    blocker.to = tile.coord;
    blocker.from_node_type = fpga::CB_NODE_LOCAL;
    blocker.from_node = 27;
    blocker.to_node_type = fpga::CB_NODE_JOINT;
    blocker.to_node = path_joint;
    ordinary_owner.wires.push_back({blocker});
    tile.cb.joint.jump |= bit(path_joint);
    fpga::attachNetRoute(ordinary_net, ordinary_owner, 0, &ordinary_driver,
                         &ordinary_sink, "out", "input", "endpoint_branch");
    fpga::registerNetRouteTiles(ordinary_net, ordinary_owner.wires.front());

    Referable<rtl::Net> distributed_net;
    distributed_net.route_protected = true;
    distributed_net.distributed_source = true;
    pnr::RouteDesign router;
    router.fpga = &device;
    router.moving_stage = true;
    pnr::RouteDesign::RouteTask task{&distributed_source, &distributed_sink,
        &distributed_net, "out", "input", "mandatory_endpoint_local"};

    // Check: Moving cannot steal a node used to terminate an ordinary route
    // on this tile; it must leave the distributed task for sink relocation.
    require(!router.routeDistributedLocalTask(task),
        "mandatory distributed route displaced an ordinary endpoint owner");
    // Check: rejecting endpoint preemption preserves both physical ownership
    // and the scheduler queue; no replacement task is fabricated.
    require(!ordinary_owner.wires.front().empty()
            && (tile.cb.joint.jump & bit(path_joint)) != NodeMask{}
            && router.pending_route_todo.empty(),
        "failed mandatory endpoint attempt changed ordinary route state");
}

void distributed_source_routes_to_attached_fixed_resource(bool one)
{
    constexpr int root_local = 34;
    constexpr int path_joint = 39;
    constexpr int target_local = 40;
    constexpr int incoming_dst = 44;

    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    fpga::CBType& cb_type = device.cb_types.back();
    cb_type.name = "attached_numeric_matrix";
    if (one) {
        cb_type.constant_one_nodes |= bit(root_local);
    } else {
        cb_type.constant_zero_nodes |= bit(root_local);
    }
    cb_type.local_joint[root_local].joint |= bit(path_joint);
    cb_type.joint_local[path_joint].local |= bit(target_local);
    cb_type.dst_local[incoming_dst].local |= bit(target_local);
    cb_type.rebuildOutgoingSrcs();

    fpga::TileType resource_type{"fixed_resource", 0};
    resource_type.pin_map.nodes[{"fixed_sink", "input", 0}] |= bit(target_local);
    device.grid_spec.size = {2, 1};
    device.size_width = 2;
    device.size_height = 1;
    device.tile_grid.resize(2);
    fpga::Tile& resource = device.tile_grid[0];
    fpga::Tile& route = device.tile_grid[1];
    resource.coord = resource.name = {0, 0};
    route.coord = route.name = {1, 0};
    resource.cb_coord = route.coord;
    resource.tile_type = &resource_type;
    route.cb_coord = route.coord;
    route.cb_type = &cb_type;
    route.cb.type = &cb_type;
    route.incoming_dst_nodes |= bit(incoming_dst);

    Referable<rtl::Cell> source_cell;
    source_cell.type = "distributed_kind";
    Referable<rtl::Cell> sink_cell;
    sink_cell.type = "fixed_sink";
    rtl::Inst distributed_source;
    distributed_source.cell_ref.set(&source_cell);
    rtl::Inst sink;
    sink.cell_ref.set(&sink_cell);
    sink.tile.set(&device.tile_grid[0]);
    sink.pos = 0;

    Referable<rtl::Net> distributed_net;
    distributed_net.route_protected = true;
    distributed_net.distributed_source = true;
    distributed_net.distributed_one = one;
    pnr::RouteDesign router;
    router.fpga = &device;
    pnr::RouteDesign::RouteTask task{&distributed_source, &sink,
        &distributed_net, "out", "input", "attached_mandatory_local"};
    task.distributed_one = one;

    // Moving can retain a shared-only marker after relocating this sink. It is
    // not a continuable distributed route and must be replaced from the root.
    fpga::Wire stale;
    stale.type = fpga::Wire::WIRE_ROUTE_EDGE;
    stale.from = route.coord;
    stale.to = route.coord;
    stale.shared = true;
    stale.net_name = task.net_name;
    sink.wires.push_back({stale});
    fpga::attachNetRoute(distributed_net, sink, 0, &distributed_source, &sink,
                         task.from_port, task.to_port, task.net_name);

    // Check: a fixed resource without its own crossbar is routed through its
    // numerically annotated adjacent route tile instead of entering Moving.
    require(router.routeDistributedLocalTask(task),
        "distributed source did not use the fixed resource's attached route tile");
    // Check: all leases and the terminal resource identity belong to the
    // attached route tile while the endpoint remains on the fixed tile.
    require((route.cb.joint.jump & bit(path_joint)) != NodeMask{}
            && route.isPinNodeLeased(target_local)
            && !sink.wires.empty() && fpga::isRouteComplete(sink.wires.back())
            && sink.wires.front().empty()
            && sink.wires.back().back().resource == resource.coord,
        "attached distributed route did not replace stale state or retain ownership");
}

void moving_rebuilds_constants_without_consuming_ordinary_tasks()
{
    constexpr int root_local = 50;
    constexpr int target_local = 51;

    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    fpga::CBType& cb_type = device.cb_types.back();
    cb_type.name = "stage_isolation_matrix";
    cb_type.constant_zero_nodes |= bit(root_local);
    cb_type.local_local[root_local].local |= bit(target_local);

    fpga::TileType tile_type{"stage_isolation_resource", 0};
    tile_type.pin_map.nodes[{"sink_kind", "input", 0}] |= bit(target_local);
    device.grid_spec.size = {1, 1};
    device.size_width = 1;
    device.size_height = 1;
    device.tile_grid.resize(1);
    fpga::Tile& tile = device.tile_grid.front();
    tile.coord = tile.cb_coord = tile.name = {0, 0};
    tile.cb_type = &cb_type;
    tile.cb.type = &cb_type;
    tile.tile_type = &tile_type;

    Referable<rtl::Cell> source_cell;
    source_cell.type = "constant_source";
    Referable<rtl::Cell> sink_cell;
    sink_cell.type = "sink_kind";
    rtl::Inst constant_source;
    constant_source.cell_ref.set(&source_cell);
    rtl::Inst constant_sink;
    constant_sink.cell_ref.set(&sink_cell);
    constant_sink.tile.set(&device.tile_grid.front());
    constant_sink.pos = 0;
    rtl::Inst ordinary_source;
    rtl::Inst ordinary_sink;

    Referable<rtl::Net> constant_net;
    constant_net.route_protected = true;
    constant_net.distributed_source = true;
    constant_net.distributed_one = false;
    Referable<rtl::Net> ordinary_net;
    pnr::RouteDesign::RouteTask ordinary{
        &ordinary_source, &ordinary_sink, &ordinary_net,
        "out", "input", "ordinary_after_move"};
    pnr::RouteDesign::RouteTask constant{
        &constant_source, &constant_sink, &constant_net,
        "out", "input", "constant_after_move"};
    std::vector<pnr::RouteDesign::RouteTask> tasks{ordinary, constant};

    pnr::RouteDesign router;
    router.fpga = &device;
    router.moving_stage = true;
    std::string failure;
    require(router.routeConstantTasksImmediately(tasks, &failure),
        "Moving could not synchronously rebuild its constant branch: " + failure);
    // Check: Const mode removes only the completed distributed task and leaves
    // the ordinary task for the Moving scheduler that owns it.
    require(tasks.size() == 1 && tasks.front().net == &ordinary_net,
        "synchronous constant repair consumed or retained the wrong task");
    // Check: the rebuilt constant reaches its exact local while the helper
    // restores the caller's Moving mode after the isolated transaction.
    require(!constant_sink.wires.empty()
            && fpga::isRouteComplete(constant_sink.wires.back())
            && tile.isPinNodeLeased(target_local)
            && router.moving_stage,
        "synchronous constant repair did not commit a complete isolated route");
}

void complete_source_tree_releases_both_endpoint_sides()
{
    constexpr int source_local = 12;
    constexpr int sink_local = 29;
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.grid_spec.size = {2, 1};
    device.size_width = 2;
    device.size_height = 1;
    device.tile_grid.resize(2);
    fpga::Tile& route_tile = device.tile_grid[0];
    fpga::Tile& resource_tile = device.tile_grid[1];
    route_tile.coord = route_tile.cb_coord = route_tile.name = {0, 0};
    resource_tile.coord = resource_tile.cb_coord = resource_tile.name = {1, 0};

    rtl::Inst owner;
    owner.wires.emplace_back();
    std::vector<fpga::Wire>& route = owner.wires.back();
    fpga::Wire source;
    source.type = fpga::Wire::WIRE_TILE_PIN;
    source.from = source.to = route_tile.coord;
    source.resource = resource_tile.coord;
    source.local = source_local;
    route.push_back(source);
    fpga::Wire sink = source;
    sink.local = sink_local;
    route.push_back(sink);

    route_tile.cb.local.local |= bit(source_local) | bit(sink_local);
    route_tile.pin_state.leased_nodes |= bit(source_local) | bit(sink_local);
    resource_tile.pin_state.leased_nodes |= bit(source_local) | bit(sink_local);
    Referable<rtl::Net> net;
    fpga::attachNetRoute(net, owner, 0, nullptr, nullptr, {}, {}, "protected_tree");

    require(fpga::unrouteSourceRouteTree({fpga::NetRouteRef{&net, 0}}),
        "complete source-tree teardown reported no removed route");
    // Check: atomic teardown releases source and sink locals on the routing
    // matrix as well as the endpoint leases on the attached resource tile.
    require(route.empty()
            && route_tile.cb.local.local == NodeMask{}
            && route_tile.pin_state.leased_nodes == NodeMask{}
            && resource_tile.pin_state.leased_nodes == NodeMask{},
        "complete source-tree teardown retained a source or resource endpoint lease");
}

}

void fanout_gap_replay_regression();
void fanout_island_landing_regression();
void fanout_cut_island_regression(bool reverse);
void basic_local_templates_regression(int mode);
void fanout_complete_input_regression(int mode);
void basic_remote_local_regression(int mode);
void basic_detour_regression(int mode);
void basic_free_detour_regression(int mode);
void basic_grounding_detour_regression(int mode);
void source_fallback_precedence_regression();
void fanout_island_audit_regression();
void fanout_gap_budget_regression(bool failure_location, bool wide_probe=false);
void fanout_gap_grounding_regression();
void fanout_retained_suffix_regression();
void fanout_private_grounding_regression();
void fanout_final_jump_regression();
void fanout_two_jump_grounding_regression();
void fanout_long_grounding_regression();
void fanout_remote_takeoff_regression(bool remote_blocked);
void fanout_gap_owner_scan_regression(bool source_only);
void fanout_retained_audit_regression();
void fanout_numeric_terminal_regression();
void resource_pin_audit_regression();
void fanout_retained_frontier_regression(bool bounded_failure,bool wide=false);
void unbound_completion_frontier_regression(bool deadline_capture=false);
void source_dominance_work_regression();

int main(int argc, char **argv)
{
    try {
        if (argc > 1) {
            const std::array<std::string,8> cases{"fixed_endpoint_transit",
                "fixed_endpoint_transit_control", "fixed_endpoint_takeoff_protected",
                "moving_wire_first_transit", "moving_wire_first_takeoff_protected",
                "moving_wire_first_full_fallback", "fixed_endpoint_source_probe",
                "fixed_endpoint_source_probe_full_fallback"};
            for (size_t i=0;i<cases.size();++i) if(argv[1]==cases[i]) {
                fixed_endpoint_preemption_regression(i);
                return 0;
            }
        }
        if (argc > 1 && std::string(argv[1]) == "fixed_endpoint_recovery") {
            fixed_endpoint_recovery_regression(0);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fixed_endpoint_control") {
            fixed_endpoint_recovery_regression(1);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fixed_endpoint_blocked") {
            fixed_endpoint_recovery_regression(2);
            return 0;
        }
        if(argc>1) {
            const std::array<std::string,4> modes{"free_detour","free_detour_budget",
                "free_detour_disconnected","free_detour_limited"};
            for(size_t i=0;i<modes.size();++i)if(argv[1]==modes[i]) {
                basic_free_detour_regression(i);return 0;
            }
        }
        if(argc>1 && std::string(argv[1])=="source_fallback_precedence") {
            source_fallback_precedence_regression();return 0;
        }
        if(argc>1 && std::string(argv[1])=="detour_failure_capture") {
            basic_detour_regression(4);return 0;
        }
        if (argc>1) {
            const std::string arg=argv[1];
            const std::array<std::string,5> modes{"grounding_detour","grounding_detour_budget",
                "grounding_detour_protected","grounding_detour_disconnected","grounding_detour_recovery"};
            for(size_t i=0;i<modes.size();++i)if(arg==modes[i]) {
                basic_grounding_detour_regression(i);return 0;
            }
        }
        if (argc > 1 && std::string(argv[1]) == "detour") {
            basic_detour_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "detour_budget") {
            basic_detour_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "detour_blocked") {
            basic_detour_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "detour_audit") {
            basic_detour_regression(3); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "remote_local") {
            basic_remote_local_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "remote_local_budget") {
            basic_remote_local_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "remote_local_blocked") {
            basic_remote_local_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "complete_input") {
            fanout_complete_input_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "complete_input_control") {
            fanout_complete_input_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "complete_input_other") {
            fanout_complete_input_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "local_templates") {
            basic_local_templates_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "local_templates_budget") {
            basic_local_templates_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "local_templates_blocked") {
            basic_local_templates_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "local_templates_control") {
            basic_local_templates_regression(3); return 0;
        }
        if (argc > 1 && (std::string(argv[1]) == "cut_island" || std::string(argv[1]) == "cut_island_reverse")) {
            fanout_cut_island_regression(std::string(argv[1]) == "cut_island_reverse");
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "island_audit") {
            fanout_island_audit_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "island_landing") {
            fanout_island_landing_regression();
            return 0;
        }
        if (argc > 1 && (std::string(argv[1]) == "local_trunk" || std::string(argv[1]) == "local_trunk_blocked")) {
            fanout_local_trunk_regression(std::string(argv[1]) == "local_trunk_blocked");
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fork_source_replacement") {
            fanout_docking_owner_regression(11);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_forward_preserve") {
            fanout_docking_owner_regression(10);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "retained_fork_landing") {
            fanout_docking_owner_regression(9);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_forward_blocked") {
            fanout_docking_owner_regression(8);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_forward") {
            fanout_docking_owner_regression(5);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_forward_free") {
            fanout_docking_owner_regression(6);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_forward_grounding") {
            fanout_docking_owner_regression(7);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_owner") {
            fanout_docking_owner_regression(0);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_owner_free") {
            fanout_docking_owner_regression(1);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_owner_transit") {
            fanout_docking_owner_regression(2);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_owner_removed") {
            fanout_docking_owner_regression(3);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_owner_grounding") {
            fanout_docking_owner_regression(4);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "transit_local") {
            fanout_transit_local_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "basic_transit_local") {
            basic_transit_local_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "basic_search_budget") {
            basic_search_budget_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_source_priority") {
            fanout_source_priority_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_numeric_seed") {
            fanout_numeric_seed_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_numeric_seed_control") {
            fanout_numeric_seed_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_numeric_seed_blocked") {
            fanout_numeric_seed_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_numeric_seed_local") {
            fanout_numeric_seed_regression(3); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_numeric_seed_arrival") {
            fanout_numeric_seed_regression(4); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_source_priority_control") {
            fanout_source_priority_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fanout_source_priority_blocked") {
            fanout_source_priority_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "basic_search_budget_control") {
            basic_search_budget_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "basic_search_budget_blocked") {
            basic_search_budget_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "basic_transit_local_direct") {
            basic_transit_local_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "basic_transit_local_blocked") {
            basic_transit_local_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "fork_role") {
            fanout_fork_role_regression(); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "numeric_grounding") {
            fanout_numeric_grounding_regression(0); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "numeric_grounding_protected") {
            fanout_numeric_grounding_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "numeric_grounding_disconnected") {
            fanout_numeric_grounding_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "transit_local_direct") {
            fanout_transit_local_regression(1); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "transit_local_blocked") {
            fanout_transit_local_regression(2); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "transit_local_reuse") {
            fanout_transit_local_regression(3); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "search_budget_blocked") {
            fanout_search_budget_regression(3);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "search_budget") {
            fanout_search_budget_regression(0);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "search_budget_control") {
            fanout_search_budget_regression(1);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "search_budget_short") {
            fanout_search_budget_regression(2);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "private_grounding_repair") {
            fanout_private_grounding_repair_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "private_grounding_repair_direct") {
            fanout_private_grounding_repair_regression(true);
            return 0;
        }
        if (argc > 1) {
            const std::array<const char *,7> cases{{"shared_terminal_gap", "shared_terminal_private",
                "shared_terminal_exact", "shared_terminal_blocked", "retained_src_rejoin", "retained_src_exact",
                "retained_src_protected"}};
            for (size_t i=0;i<cases.size();++i) if (std::string(argv[1])==cases[i]) {
                fanout_shared_terminal_gap_regression(i);
                return 0;
            }
        }
        if (argc > 1 && std::string(argv[1]) == "shared_input_repair_blocked") {
            fanout_shared_input_repair_regression(false, true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "shared_input_repair") {
            fanout_shared_input_repair_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "shared_input_repair_control") {
            fanout_shared_input_repair_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "trunk_handoff") {
            fanout_trunk_handoff_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "takeoff_handoff") {
            fanout_takeoff_handoff_regression(false); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "takeoff_handoff_control") {
            fanout_takeoff_handoff_regression(true); return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "trunk_handoff_control") {
            fanout_trunk_handoff_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "preserved_repair") {
            preserved_route_repair_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "owner_query") {
            route_owner_query_regressions();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_probe_budget") {
            fanout_gap_budget_regression(false,true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_budget") {
            fanout_gap_budget_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_failure_location") {
            fanout_gap_budget_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "final_jump") {
            fanout_final_jump_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "two_jump_grounding") {
            fanout_two_jump_grounding_regression();
            return 0;
        }
        if (argc > 1 && (std::string(argv[1]) == "remote_takeoff" ||
                         std::string(argv[1]) == "remote_takeoff_blocked")) {
            fanout_remote_takeoff_regression(std::string(argv[1]) == "remote_takeoff_blocked");
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "long_grounding") {
            fanout_long_grounding_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_owner_scan") {
            fanout_gap_owner_scan_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "retained_audit") {
            fanout_retained_audit_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "numeric_terminal") {
            fanout_numeric_terminal_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "resource_pin_audit") {
            resource_pin_audit_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "retained_frontier") {
            fanout_retained_frontier_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "retained_frontier_capacity") {
            fanout_retained_frontier_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "retained_wide") {
            fanout_retained_frontier_regression(false,true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "unbound_completion_frontier") {
            unbound_completion_frontier_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "unbound_deadline_frontier") {
            unbound_completion_frontier_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_dominance_work") {
            source_dominance_work_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_grounding_budget_escalation") {
            source_grounding_budget_escalation_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_free_budget_escalation") {
            source_free_budget_escalation_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "docking_cut_checks") {
            docking_cut_checks_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_frontier_reuse") {
            source_frontier_reuse_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_source_frontier_reuse") {
            gap_source_frontier_reuse_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "moving_fixed_local_cycle") {
            moving_fixed_local_cycle_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "moving_source_probe") {
            moving_source_probe_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "moving_deferred_fixed_wire") {
            moving_deferred_fixed_wire_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "moving_source_boundary_continuation") {
            moving_source_boundary_continuation_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_candidate_inputs") {
            moving_source_candidate_input_work_regression(false);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_current_wire") {
            moving_source_current_wire_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_two_ended") {
            moving_source_two_ended_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_resolved_local") {
            moving_source_resolved_local_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "source_candidate_inputs_output_conflict") {
            moving_source_candidate_input_work_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_owner_source") {
            fanout_gap_owner_scan_regression(true);
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "private_grounding") {
            fanout_private_grounding_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "retained_suffix") {
            fanout_retained_suffix_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_grounding") {
            fanout_gap_grounding_regression();
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "gap_replay") {
            fanout_gap_replay_regression();
            return 0;
        }
        package_site_position_uses_tile_model_order();
        partial_preemption_regressions();
        if (argc > 1 && std::string(argv[1]) == "partial") return 0;
        free_destination_suppresses_preemption();
        endpoint_destinations_are_not_victims();
        protected_transit_destination_is_not_a_victim();
        unreachable_free_destination_does_not_suppress_preemption();
        endpoint_joint_owners_cannot_preempt_each_other();
        only_completed_destination_bindings_protect_terminal_resources();
        successful_preemption_retries_grounding_immediately();
        all_transit_owners_are_removed_before_grounding_claim();
        grounding_skips_a_victim_that_does_not_enable_docking();
        free_dst_with_busy_joint_does_not_suppress_preemption();
        completely_free_terminal_path_suppresses_preemption();
        reserved_joint_path_does_not_suppress_preemption();
        terminal_preemption_requires_the_exact_joint_path();
        grounding_releases_incoming_hop_not_just_its_continuation();
        grounding_releases_incoming_hop_not_just_its_continuation(true);
        grounding_skips_takeoff_and_policy_rejections();
        grounding_preflights_all_joint_owners_and_cuts_each_binding_once();
        grounding_shared_landing_is_released_only_after_all_owners_pass();
        numeric_cuts_follow_dependencies_not_lease_owner_flags();
        grounding_production_free_entry_prevents_cuts();
        transit_source_tree_is_unrouted_and_requeued_atomically();
        only_transit_destination_is_preempted();
        mandatory_distributed_local_path_preserves_local_takeoff();
        mandatory_distributed_local_path_keeps_protected_blocker();
        mandatory_distributed_local_path_keeps_ordinary_endpoint();
        // Both distributed constant polarities use the same numeric endpoint
        // machinery but must select their independently declared root masks.
        distributed_source_routes_to_attached_fixed_resource(true);
        distributed_source_routes_to_attached_fixed_resource(false);
        moving_rebuilds_constants_without_consuming_ordinary_tasks();
        complete_source_tree_releases_both_endpoint_sides();
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
