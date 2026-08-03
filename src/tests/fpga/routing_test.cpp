#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "Cell.h"
#include "Conn.h"
#include "Module.h"
#include "Tile.h"
#include "route/RoutePassState.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <utility>

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

bool isSet(NodeMask value, int index)
{
    return (value & bit(index)) != NodeMask{};
}

void packed_void_state_is_per_net_designator()
{
    rtl::Net bus;
    bus.designators = {11, 12};

    require(bus.markDesignatorVoid(11),
        "first packed bus bit was not marked void");
    require(bus.designatorIsVoid(11) && !bus.designatorIsVoid(12),
        "marking one packed bus bit also hid an unrelated routable bit");
    require(!bus.void_net,
        "partially internal bus was incorrectly classified as fully void");

    require(bus.markDesignatorVoid(12) && bus.void_net,
        "net did not become fully void after every designator was internal");

    bus.clearVoidDesignators();
    require(!bus.void_net && !bus.designatorIsVoid(11)
            && bus.void_designators.empty(),
        "clearing design state retained per-designator void flags");
}

void packed_void_connection_unroutes_only_its_binding()
{
    rtl::Inst driver;
    rtl::Inst internal_sink;
    rtl::Inst external_sink;
    rtl::Inst internal_owner;
    rtl::Inst external_owner;
    internal_owner.wires.emplace_back(1);
    external_owner.wires.emplace_back(1);

    rtl::Net bus;
    bus.routes.push_back(rtl::NetRouteBinding{
        &internal_owner, 0, &driver, &internal_sink, "O", "I0", "internal"});
    bus.routes.push_back(rtl::NetRouteBinding{
        &external_owner, 0, &driver, &external_sink, "O", "I1", "external"});

    require(fpga::unrouteNetConnection(
                bus, &driver, &internal_sink, "O", "I0") == 1,
        "packed connection did not release its exact route binding");
    require(internal_owner.wires[0].empty(),
        "packed connection retained its former external route");
    require(external_owner.wires[0].size() == 1,
        "packing one designator unrouted another connection in the RTL net");
}

void packed_void_generic_connection_transfers_shared_prefix_ownership()
{
    rtl::Inst driver;
    rtl::Inst internal_sink;
    rtl::Inst external_sink;
    rtl::Inst generic_owner;
    rtl::Inst fanout_owner;

    fpga::Wire trunk_a;
    trunk_a.from = {2, 3};
    trunk_a.to = {3, 3};
    trunk_a.jump = 17;
    trunk_a.dst = 29;
    fpga::Wire trunk_b;
    trunk_b.from = {3, 3};
    trunk_b.to = {4, 3};
    trunk_b.local = 29;
    trunk_b.jump = 31;
    trunk_b.dst = 43;
    fpga::Wire internal_tail;
    internal_tail.from = {4, 3};
    internal_tail.to = {4, 4};
    internal_tail.local = 43;
    internal_tail.jump = 47;
    fpga::Wire external_tail = internal_tail;
    external_tail.to = {5, 3};
    external_tail.jump = 53;

    generic_owner.wires.push_back({trunk_a, trunk_b, internal_tail});
    trunk_a.shared = true;
    trunk_b.shared = true;
    fanout_owner.wires.push_back({trunk_a, trunk_b, external_tail});

    rtl::Net bus;
    bus.routes.push_back(rtl::NetRouteBinding{
        &generic_owner, 0, &driver, &internal_sink, "O", "I0", "internal"});
    bus.routes.push_back(rtl::NetRouteBinding{
        &fanout_owner, 0, &driver, &external_sink, "O", "I1", "external"});

    require(fpga::unrouteNetConnection(
                bus, &driver, &internal_sink, "O", "I0") == 1,
        "packing the Generic connection did not clear its exact binding");
    require(generic_owner.wires[0].empty(),
        "packed Generic connection retained its old route");
    require(fanout_owner.wires[0].size() == 3,
        "packing the Generic connection removed its surviving fanout");
    require(!fanout_owner.wires[0][0].shared
            && !fanout_owner.wires[0][1].shared,
        "surviving fanout did not take ownership of the shared source prefix");
    require(!fanout_owner.wires[0][2].shared,
        "fanout ownership transfer changed its private suffix");
}

void deadend_masks_are_basic_stage_only()
{
    NodeMask learned = bit(7) | bit(31);

    // Check: the initial Generic stage keeps learned deadends active.
    require(!pnr::routingIgnoresDeadends(true, false, false),
        "initial Generic routing unexpectedly ignored deadends");
    require(pnr::routingStageDeadends(learned, false, false) == learned,
        "initial Generic routing discarded its deadend mask");

    // Check: Fanout and Moving reconsider all source exits.
    require(pnr::routingIgnoresDeadends(true, true, false)
            && pnr::routingIgnoresDeadends(true, false, true),
        "later routing stage did not ignore deadends");
    require(pnr::routingStageDeadends(learned, true, false) == NodeMask{}
            && pnr::routingStageDeadends(learned, false, true) == NodeMask{},
        "later routing stage retained a deadend mask");

    // A failed edge learned inside the current bounded search must still be
    // excluded so Fanout/Moving can return to the parent and try another exit.
    NodeMask current_search = bit(19);
    require(pnr::effectiveSearchDeadends(current_search, learned, true) == current_search,
        "later routing stage discarded its current-search failed edge");
    require(pnr::effectiveSearchDeadends(current_search, learned, false)
            == (current_search | learned),
        "Basic routing did not combine current and persistent deadends");

    // Check: a Generic trunk repair scheduled after Basic stays under the
    // later-stage policy even though its task itself is not a Fanout task.
    require(pnr::routingIgnoresDeadends(false, false, false),
        "post-Basic Generic repair re-enabled deadends");
}

void failed_edge_persistence_is_basic_stage_only()
{
    pnr::FailedEdgePolicy basic = pnr::failedEdgePolicy(false, true, true, true);
    pnr::FailedEdgePolicy fanout = pnr::failedEdgePolicy(true, true, true, true);
    pnr::FailedEdgePolicy moving = pnr::failedEdgePolicy(true, true, true, true);

    // Basic remembers the failed edge across attempts; later stages must not
    // inherit this congestion decision after routes and placements change.
    require(basic.persist, "Basic failed edge was not made persistent");
    require(!fanout.persist, "Fanout failed edge was incorrectly made persistent");
    require(!moving.persist, "Moving failed edge was incorrectly made persistent");

    // A caller can explicitly request a temporary-only rejection in any stage.
    require(!pnr::failedEdgePolicy(false, false, true, true).persist,
        "temporary Basic rejection was incorrectly persisted");
}

void failed_edge_returns_to_parent_in_every_stage()
{
    pnr::FailedEdgePolicy basic = pnr::failedEdgePolicy(false, true, true, true);
    pnr::FailedEdgePolicy fanout = pnr::failedEdgePolicy(true, true, true, true);
    pnr::FailedEdgePolicy moving = pnr::failedEdgePolicy(true, true, true, true);

    // Retry is independent of persistence: every stage returns to the parent
    // and uses its current-search rejection to select another outgoing edge.
    require(basic.retry_parent, "Basic failed child did not return to its parent");
    require(fanout.retry_parent, "Fanout failed child did not return to its parent");
    require(moving.retry_parent, "Moving failed child did not return to its parent");

    // Re-entering the parent must suppress only the failed child and expose the
    // next exit, even though Moving ignores every persistent deadend mask.
    fpga::CBState live;
    NodeMask parent_exits = bit(11) | bit(23);
    NodeMask current_search_failure = bit(11);
    NodeMask stale_persistent = bit(23);
    NodeMask effective = pnr::effectiveSearchDeadends(
        current_search_failure, stale_persistent, true);
    NodeMask available = pnr::availableSourceCandidates(
        parent_exits, live, effective, false);
    require(!isSet(available, 11),
        "Moving parent retried the child that failed in this search");
    require(isSet(available, 23),
        "Moving parent did not expose an alternate exit hidden by a stale deadend");

    // A root or malformed step has no incoming edge and therefore no parent
    // transition that can be retried.
    require(!pnr::failedEdgePolicy(true, true, false, true).retry_parent,
        "root step incorrectly requested a parent retry");
    require(!pnr::failedEdgePolicy(true, true, true, false).retry_parent,
        "step without an incoming jump incorrectly requested a parent retry");
}

void fanout_branch_selection_skips_saturated_points()
{
    // Reproduce a routed trunk whose final landing is saturated while earlier
    // trunk and sibling landings still have usable exits.
    const std::vector<int> free_exits = {4, 1, 0, 2, 0};
    std::vector<size_t> preferred;
    std::vector<size_t> fallbacks;
    for (size_t index = 0; index < free_exits.size(); ++index) {
        if (pnr::fanoutBranchIsUsableFallback(free_exits[index])) {
            fallbacks.push_back(index);
        }
        if (pnr::fanoutBranchIsPreferred(free_exits[index])) {
            preferred.push_back(index);
        }
    }

    // Check: the well-connected earlier trunk point is preferred.
    require(preferred == std::vector<size_t>{0},
        "Fanout did not prefer the trunk point with more than two free exits");
    // Check: all usable trunk/sibling points remain available as fallbacks.
    require(fallbacks == (std::vector<size_t>{0, 1, 3}),
        "Fanout discarded a usable trunk or sibling fallback");
    // Check: the saturated final trunk landing is never selected as fallback.
    require(std::find(fallbacks.begin(), fallbacks.end(), 4) == fallbacks.end(),
        "Fanout selected the saturated final trunk landing");
}

void current_target_entry_failure_is_not_retried()
{
    // A prior sticky mark may be ignored for a direct target hop because the
    // target occupancy can have changed since the mark was learned.
    require(pnr::targetHopMayBypassDeadend(true, false),
        "direct target hop did not bypass an old sticky deadend");

    // A terminal-entry failure learned in this bounded search must exclude the
    // same hop so another destination entry candidate can be attempted.
    require(!pnr::targetHopMayBypassDeadend(true, true),
        "current target-entry deadend was immediately retried");
    require(!pnr::targetHopMayBypassDeadend(false, false),
        "non-target hop incorrectly bypassed deadend filtering");
}

void failed_near_target_docking_keeps_normal_suffix_depth()
{
    // A route already in the grounding window still needs the normal suffix
    // depth when docking is blocked, so it can route around local congestion.
    require(pnr::suffixDepthBeforeDocking(3, 5, 5) == 5,
        "near-target route lost its normal suffix depth");
    require(pnr::suffixDepthBeforeDocking(5, 5, 5) == 5,
        "docking-window boundary shortened its bounded suffix");

    // Outside the docking window, normal bounded incremental routing is unchanged.
    require(pnr::suffixDepthBeforeDocking(6, 5, 5) == 5,
        "far route unexpectedly shortened its bounded suffix");

    // A blocked routed endpoint inside the radius must survive normal expansion
    // so the caller can run backward/forward docking from its selected path.
    require(pnr::preserveBlockedEndpointForDocking(false, 3, 1, 5),
        "blocked near-target endpoint was discarded before docking");
    require(!pnr::preserveBlockedEndpointForDocking(true, 3, 1, 5),
        "endpoint with an accepted normal edge incorrectly forced docking");
    require(!pnr::preserveBlockedEndpointForDocking(false, 0, 1, 5),
        "unrouted source local was incorrectly retained as a docking endpoint");
    require(!pnr::preserveBlockedEndpointForDocking(false, 3, 6, 5),
        "blocked endpoint outside the docking window was retained");
}

uint16_t elementBit(int index)
{
    return static_cast<uint16_t>(1u << index);
}

fpga::Element makeElement(const std::string& name, fpga::ElementType type, int bit)
{
    fpga::Element element;
    element.name = name;
    element.type = type;
    element.bitmap_pos = static_cast<uint16_t>(bit);
    element.elements_to_left = static_cast<int>(type);
    return element;
}

void connectElements(fpga::TileType& tile_type, fpga::ElementType left_type, int left_bit,
                     fpga::ElementType right_type, int right_bit)
{
    for (fpga::Element& element : tile_type.elements) {
        if (element.type == left_type && element.bitmap_pos == left_bit) {
            element.right_blockers[right_bit] |= elementBit(left_bit);
        }
        if (element.type == right_type && element.bitmap_pos == right_bit) {
            element.left_blockers[left_bit] |= elementBit(right_bit);
        }
    }
}

int pick(std::mt19937& rng, std::vector<int>& values)
{
    require(!values.empty(), "random choice from empty vector");
    std::uniform_int_distribution<size_t> dist(0, values.size() - 1);
    size_t index = dist(rng);
    int value = values[index];
    values.erase(values.begin() + static_cast<std::ptrdiff_t>(index));
    return value;
}

std::vector<int> range(int first, int count)
{
    std::vector<int> values;
    values.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        values.push_back(first + i);
    }
    return values;
}

std::vector<fpga::Tile*> resetDeviceGrid(int width, int height)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;
    device.local_route_wire_mappings.clear();
    device.tile_grid.resize(static_cast<size_t>(width * height));

    std::vector<fpga::Tile*> tiles;
    tiles.reserve(device.tile_grid.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y * width + x)];
            tile.coord = {x, y};
            tile.name = {x, y};
            tile.routedNets.clear();
            tile.cb = {};
            tile.cb_type = nullptr;
            tile.pin_state = {};
            tiles.push_back(&tile);
        }
    }
    return tiles;
}

fpga::Tile& resetDevice()
{
    return *resetDeviceGrid(1, 1).front();
}

struct TestRoute
{
    Referable<rtl::Net> net;
    rtl::Inst owner;
    int exit = -1;
    int local = -1;
    int joint = -1;
    bool transit = false;
};

void addRoute(fpga::Tile& tile, TestRoute& route, const std::string& name)
{
    route.net.name = name;
    route.owner.wires.clear();
    route.owner.wires.emplace_back();
    std::vector<fpga::Wire>& wires = route.owner.wires.back();

    fpga::Wire fragment;
    fragment.type = fpga::Wire::WIRE_CROSSBAR;
    fragment.from = tile.coord;
    fragment.to = route.transit ? fpga::Coord{tile.coord.x + 1, tile.coord.y} : tile.coord;
    fragment.local = route.local;
    fragment.pos = route.transit ? 1 : 0;
    fragment.jump = route.exit;
    fragment.joint = route.joint;
    fragment.net_name = name;
    wires.push_back(fragment);

    tile.cb.src.jump |= bit(route.exit);
    if (route.transit) {
        tile.cb.dst.jump |= bit(route.local);
    }
    else {
        tile.cb.local.local |= bit(route.local);
    }

    fpga::attachNetRoute(route.net, route.owner, 0, nullptr, &route.owner, {}, {}, name);
    fpga::registerNetRouteTiles(route.net, wires);
}

void assertCanTakeExit(fpga::Tile& tile, int local, int exit)
{
    require(!isSet(tile.cb.src.jump, exit), "chosen exit is still busy");
    tile.cb.src.jump |= bit(exit);
    tile.cb.local.local |= bit(local);
    require(isSet(tile.cb.src.jump, exit), "new local did not lease exit");
}

void preemptTransitOnExit(fpga::Tile& tile, int exit)
{
    rtl::Net* victim = fpga::findNetByNode(tile, fpga::CB_NODE_SRC, exit, true);
    require(victim != nullptr, "did not find transit victim on busy exit");
    require(fpga::unrouteNet(*victim), "failed to unroute transit victim");
}

void local_and_transit_preemption(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::Tile& tile = resetDevice();

    std::vector<int> exits = range(8, 32);
    std::vector<int> locals = range(64, 40);
    std::vector<std::unique_ptr<TestRoute>> transit_routes;
    std::vector<std::unique_ptr<TestRoute>> local_routes;

    for (int i = 0; i < 8; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = true;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        addRoute(tile, *route, "transit_" + std::to_string(i));
        transit_routes.push_back(std::move(route));
    }
    for (int i = 0; i < 8; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = false;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        addRoute(tile, *route, "local_" + std::to_string(i));
        local_routes.push_back(std::move(route));
    }

    int new_local = pick(rng, locals);
    int transit_exit = transit_routes.front()->exit;
    int local_exit = local_routes.front()->exit;

    require(fpga::findNetByNode(tile, fpga::CB_NODE_SRC, local_exit, true) == nullptr,
        "local-to-exit route must not be a transit victim");
    require(isSet(tile.cb.src.jump, transit_exit), "transit exit was not busy before preemption");

    preemptTransitOnExit(tile, transit_exit);

    require(!isSet(tile.cb.src.jump, transit_exit), "transit exit still leased after unroute");
    require(isSet(tile.cb.src.jump, local_exit), "local-to-exit route was incorrectly unrouted");
    assertCanTakeExit(tile, new_local, transit_exit);
}

void joint_metadata_preemption(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::Tile& tile = resetDevice();

    std::vector<int> exits = range(40, 24);
    std::vector<int> locals = range(96, 48);
    std::vector<int> joints = range(4, 20);
    std::vector<std::unique_ptr<TestRoute>> transit_routes;
    std::vector<std::unique_ptr<TestRoute>> local_routes;

    for (int i = 0; i < 6; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = true;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        route->joint = pick(rng, joints);
        addRoute(tile, *route, "joint_transit_" + std::to_string(i));
        transit_routes.push_back(std::move(route));
    }
    for (int i = 0; i < 6; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = false;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        route->joint = pick(rng, joints);
        addRoute(tile, *route, "joint_local_" + std::to_string(i));
        local_routes.push_back(std::move(route));
    }

    int new_local = pick(rng, locals);
    int transit_exit = transit_routes.front()->exit;
    int transit_joint = transit_routes.front()->joint;
    int local_joint = local_routes.front()->joint;

    require(fpga::findNetByNode(tile, fpga::CB_NODE_JOINT, transit_joint, true) != nullptr,
        "transit joint use was not discoverable");
    require(fpga::findNetByNode(tile, fpga::CB_NODE_JOINT, local_joint, true) == nullptr,
        "local-to-exit joint use must not be a transit victim");

    preemptTransitOnExit(tile, transit_exit);
    assertCanTakeExit(tile, new_local, transit_exit);
}

void free_joint_exit_is_preferred(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::Tile& tile = resetDevice();

    std::vector<int> exits = range(80, 16);
    std::vector<int> locals = range(128, 40);
    std::vector<int> joints = range(32, 16);
    std::vector<std::unique_ptr<TestRoute>> transit_routes;

    for (int i = 0; i < 10; ++i) {
        auto route = std::make_unique<TestRoute>();
        route->transit = true;
        route->exit = pick(rng, exits);
        route->local = pick(rng, locals);
        route->joint = pick(rng, joints);
        addRoute(tile, *route, "free_joint_transit_" + std::to_string(i));
        transit_routes.push_back(std::move(route));
    }

    int free_exit = pick(rng, exits);
    int free_joint = pick(rng, joints);
    int new_local = pick(rng, locals);

    require(!isSet(tile.cb.src.jump, free_exit), "free joint exit setup accidentally occupied exit");
    require(fpga::findNetByNode(tile, fpga::CB_NODE_JOINT, free_joint, true) == nullptr,
        "free joint setup accidentally occupied joint");

    tile.cb.src.jump |= bit(free_exit);
    tile.cb.local.local |= bit(new_local);
    fpga::Wire fragment;
    fragment.from = tile.coord;
    fragment.to = tile.coord;
    fragment.local = new_local;
    fragment.pos = 0;
    fragment.jump = free_exit;
    fragment.joint = free_joint;
    require(isSet(tile.cb.src.jump, free_exit), "new local failed to occupy free joint exit");
}

void releasing_route_fragment_keeps_deadend_for_same_src()
{
    fpga::Tile& tile = resetDevice();
    fpga::Wire fragment;
    fragment.type = fpga::Wire::WIRE_CROSSBAR;
    fragment.from = tile.coord;
    fragment.to = {tile.coord.x + 1, tile.coord.y};
    fragment.local = 77;
    fragment.jump = 33;
    fragment.pos = 1;
    std::vector<fpga::Wire> route{fragment};

    tile.cb.src.jump |= bit(fragment.jump);
    tile.cb.src_deadend.jump |= bit(fragment.jump);
    fpga::releaseRouteFragmentLease(route, 0);

    // Check: rollback of the fragment frees the lease but keeps the sticky deadend mark.
    require(!isSet(tile.cb.src.jump, fragment.jump), "rollback release did not clear the source lease");
    require(isSet(tile.cb.src_deadend.jump, fragment.jump), "rollback release cleared the sticky source deadend mark");
}

void releasing_route_fragment_clears_source_and_transit_node_classes()
{
    fpga::Tile& tile = resetDevice();
    fpga::Wire source;
    source.type = fpga::Wire::WIRE_CROSSBAR;
    source.from = tile.coord;
    source.to = {tile.coord.x + 1, tile.coord.y};
    source.local = 82;
    source.jump = 34;
    source.pos = 0;

    tile.cb.local.local |= bit(source.local);
    tile.cb.src.jump |= bit(source.jump);
    fpga::releaseRouteFragmentLease(std::vector<fpga::Wire>{source}, 0);

    // Check: releasing a source fragment frees both the local takeoff and the outgoing source.
    require(!isSet(tile.cb.local.local, source.local), "source fragment release did not clear local lease");
    require(!isSet(tile.cb.src.jump, source.jump), "source fragment release did not clear source lease");

    fpga::Wire transit;
    transit.type = fpga::Wire::WIRE_CROSSBAR;
    transit.from = tile.coord;
    transit.to = {tile.coord.x + 1, tile.coord.y};
    transit.local = 91;
    transit.jump = 48;
    transit.joint = 17;
    transit.pos = 2;

    tile.cb.dst.jump |= bit(transit.local);
    tile.cb.src.jump |= bit(transit.jump);
    tile.cb.joint.jump |= bit(transit.joint);
    tile.cb.src_deadend.jump |= bit(transit.jump);
    fpga::releaseRouteFragmentLease(std::vector<fpga::Wire>{transit}, 0);

    // Check: releasing a transit/fork fragment frees incoming dst, outgoing src, and joint leases.
    require(!isSet(tile.cb.dst.jump, transit.local), "transit fragment release left stale dst lease");
    require(!isSet(tile.cb.src.jump, transit.jump), "transit fragment release did not clear source lease");
    require(!isSet(tile.cb.joint.jump, transit.joint), "transit fragment release did not clear joint lease");
    // Check: release still preserves sticky routing deadend marks.
    require(isSet(tile.cb.src_deadend.jump, transit.jump), "transit release cleared sticky source deadend mark");

    fpga::Wire double_joint = transit;
    double_joint.joint = 18;
    double_joint.joint2 = 19;
    tile.cb.dst.jump |= bit(double_joint.local);
    tile.cb.src.jump |= bit(double_joint.jump);
    tile.cb.joint.jump |= bit(double_joint.joint);
    tile.cb.joint.jump |= bit(double_joint.joint2);
    fpga::releaseRouteFragmentLease(std::vector<fpga::Wire>{double_joint}, 0);

    // Check: a two-joint route releases both physical joint nodes atomically.
    require(!isSet(tile.cb.joint.jump, double_joint.joint),
        "double-joint fragment release left the source-side joint leased");
    require(!isSet(tile.cb.joint.jump, double_joint.joint2),
        "double-joint fragment release left the destination-side joint leased");
}

void preempted_transit_unroute_keeps_deadend_for_same_src()
{
    fpga::Tile& tile = resetDevice();
    TestRoute transit;
    transit.transit = true;
    transit.exit = 45;
    transit.local = 91;
    addRoute(tile, transit, "deadend_preempted_transit");
    tile.cb.src_deadend.jump |= bit(transit.exit);

    rtl::Net* victim = fpga::findNetByNode(tile, fpga::CB_NODE_SRC, transit.exit, true);
    require(victim != nullptr, "deadend preemption test did not find transit victim");
    require(fpga::unrouteNet(*victim), "deadend preemption test failed to unroute victim");

    // Check: preempting a transit route frees the lease but keeps the sticky deadend on its exit.
    require(!isSet(tile.cb.src.jump, transit.exit), "preempted transit source lease was not cleared");
    require(isSet(tile.cb.src_deadend.jump, transit.exit), "preempted transit source deadend was incorrectly cleared");
}

void preemption_cycle_guards_expire_after_each_route_pass()
{
    std::unordered_set<std::string> names{"route_a"};
    std::unordered_map<std::string, std::string> blockers{{"route_a", "route_b"}};

    pnr::resetPassPreemptionContainers(names, blockers);

    // Check: duplicate-attempt names expire, while blocker ancestry remains across passes.
    require(names.empty(),
        "route-name preemption guard survived the pass boundary");
    require(blockers.size() == 1 && blockers.at("route_a") == "route_b",
        "route-blocker ancestry was lost at the pass boundary");

    // Check: direct and transitive reverse preemptions are rejected without blocking unrelated victims.
    blockers["route_b"] = "route_c";
    require(pnr::preemptionWouldCycle(blockers, "route_a", "route_b"),
        "direct reverse preemption cycle was accepted");
    require(pnr::preemptionWouldCycle(blockers, "route_a", "route_c"),
        "transitive reverse preemption cycle was accepted");
    require(!pnr::preemptionWouldCycle(blockers, "route_a", "route_d"),
        "unrelated preemption was rejected");
    pnr::rememberPreemptionBlocker(blockers, "route_a", "route_d");
    require(blockers.at("route_a") == "route_b",
        "later preemption overwrote the stable blocker parent");
}

void preemption_candidate_iteration_includes_busy_transit_exits()
{
    fpga::CBType type{"candidate_iteration"};
    fpga::CBState live;
    live.type = &type;
    constexpr int local = 10;
    constexpr int busy_src = 20;
    NodeMask busy_bit = bit(busy_src);
    type.local_src[local].jump |= busy_bit;
    live.src.jump |= busy_bit;

    require(live.iterate(false, local, {0, 0}, {1, 0}, -1) < 0,
        "normal candidate iteration exposed a leased source");
    fpga::CBState preemption_view = pnr::sourceCandidateIterationState(
        live, type.local_src[local].jump, true);
    NodeMask normal_candidates = pnr::availableSourceCandidates(
        type.local_src[local].jump, live, {}, false);
    NodeMask preemption_candidates = pnr::availableSourceCandidates(
        type.local_src[local].jump, live, {}, true);

    // Check: normal routing hides the lease, while preemption keeps it for owner inspection.
    require(normal_candidates == NodeMask{},
        "normal source candidate mask exposed a leased source");
    require((preemption_candidates & busy_bit) != NodeMask{},
        "preemption source candidate mask filtered the busy transit exit");
    require(preemption_view.iterate(false, local, {0, 0}, {1, 0}, -1) == busy_src,
        "preemption candidate iteration filtered the busy transit exit");
    require(isSet(live.src.jump, busy_src),
        "preemption candidate iteration modified the live source lease");

    // Check: a remembered busy candidate cannot displace transit when a later free path succeeded.
    require(!pnr::shouldPreemptTakeoff(true, true),
        "takeoff preempted transit despite accepting a free exit");
    require(!pnr::shouldPreemptTakeoff(false, false),
        "takeoff preempted transit when the route was not leaving its source local");
    require(pnr::shouldPreemptTakeoff(false, true),
        "takeoff did not preempt after exhausting free exits");

    // Check: ordinary routes may displace transit only at takeoff.
    require(pnr::transitPreemptionStepAllowed(true, false, true),
        "ordinary takeoff preemption was disabled");
    require(!pnr::transitPreemptionStepAllowed(true, false, false),
        "ordinary routing preempted transit at an intermediate step");

    // Check: protected infrastructure may inspect transit owners at any step,
    // while the separate free-path check above still prevents needless preemption.
    require(pnr::transitPreemptionStepAllowed(true, true, false),
        "protected routing could not preempt an intermediate transit blocker");
    require(!pnr::transitPreemptionStepAllowed(false, true, false),
        "protected routing bypassed the global preemption switch");
    require(pnr::preferPreemptionVictim(1, 12),
        "preemption did not prefer the smaller physical source tree");
    require(!pnr::preferPreemptionVictim(12, 1),
        "preemption preferred a larger physical source tree");

    // Check: Fanout can preempt a private suffix but never its shared Generic trunk.
    require(pnr::canPreemptFanoutSuffix(true, true, false),
        "Fanout routing rejected private suffix preemption");
    require(!pnr::canPreemptFanoutSuffix(true, true, true),
        "Fanout routing allowed shared trunk preemption");
    require(!pnr::canPreemptFanoutSuffix(true, false, false),
        "Fanout routing allowed preemption outside a shared route tree");
    require(pnr::canPreemptFanoutSuffix(false, false, true),
        "Generic routing incorrectly inherited the Fanout suffix restriction");

    // Check: Moving may rip unfinished work but cannot invalidate a completed moved endpoint.
    require(!pnr::canPreemptMovingRoute(true, true),
        "Moving routing allowed preemption of a finished endpoint");
    require(pnr::canPreemptMovingRoute(true, false),
        "Moving routing rejected preemption of an unfinished endpoint");
    require(pnr::canPreemptMovingRoute(false, true),
        "Generic/Fanout routing inherited the Moving endpoint restriction");

    // Check: partial Fanout advancement keeps the Fanout stage active.
    require(pnr::fanoutPassMadeProgress(0, 1, 0),
        "Fanout advancement was treated as a stagnant pass");
    require(pnr::fanoutPassMadeProgress(0, 0, 1),
        "Fanout route change was treated as a stagnant pass");
    require(!pnr::fanoutPassMadeProgress(0, 0, 0),
        "inactive Fanout pass was treated as progress");
    require(pnr::sourceCandidateInPhase(false, 0)
            && !pnr::sourceCandidateInPhase(true, 0),
        "free-source phase admitted a leased source");
    require(pnr::sourceCandidateInPhase(true, 1)
            && !pnr::sourceCandidateInPhase(false, 1),
        "preemption phase did not isolate leased sources");
    require(!pnr::fanoutShouldHandOff(39, 0, 5),
        "Fanout ended before its bounded low-progress window");
    require(pnr::fanoutShouldHandOff(40, 2, 5),
        "Fanout retained low-completion work after its bounded window");
    require(!pnr::fanoutShouldHandOff(40, 3, 5),
        "Fanout handed off a pass that still completed several branches");
    require(!pnr::routeStageTimeoutRequiresFailure(true, true),
        "Fanout deadline rejected a valid Moving handoff");
    require(pnr::routeStageTimeoutRequiresFailure(true, false)
            && !pnr::routeStageTimeoutRequiresFailure(false, false),
        "terminal stage timeout handling accepted unfinished work");

    // Check: Moving does not relocate an endpoint while its current route is growing.
    require(pnr::movingPassMadeProgress(0, 1),
        "Moving route advancement was treated as placement stagnation");
    require(pnr::movingPassMadeProgress(1, 0),
        "Moving route completion was treated as placement stagnation");
    require(!pnr::movingPassMadeProgress(0, 0),
        "Moving route without forward progress retained a blocked placement");
    require(!pnr::focusedMovingShouldRelocate(false, 0, 5, false, false),
        "focused Moving relocated a placement while routes were advancing");
    require(pnr::focusedMovingShouldRelocate(false, 0, 5, false, true),
        "focused Moving retained an incomplete route after its bounded pass count");
    require(pnr::focusedMovingShouldRelocate(true, 0, 5, false, false)
            && pnr::focusedMovingShouldRelocate(false, 5, 5, false, false)
            && pnr::focusedMovingShouldRelocate(false, 0, 5, true, false),
        "focused Moving ignored a blocked or persistently stagnant placement");
    int moving_no_progress = pnr::updateMovingPlacementNoProgressPasses(4, true);
    require(moving_no_progress == 5
            && pnr::movingPlacementPassesExhausted(moving_no_progress, 5),
        "Moving let suffix progress extend a placement beyond its bounded slice");

    moving_no_progress = 0;
    for (int pass = 0; pass < 5; ++pass) {
        moving_no_progress = pnr::updateMovingPlacementNoProgressPasses(
            moving_no_progress, false);
    }
    require(pnr::movingPlacementPassesExhausted(moving_no_progress, 5),
        "unfocused Moving did not select a cell after bounded no-progress passes");
    // Check: each moved placement tests the binding that rejected the previous
    // placement before spending work on the rest of the incident route set.
    std::vector<int> moving_tasks{4, 7, 2, 9};
    require(pnr::prioritizeMovingTrigger(moving_tasks, 2,
                [](int task, int trigger) { return task == trigger; })
            && moving_tasks == std::vector<int>({2, 4, 7, 9}),
        "Moving did not prioritize the failed trigger after relocation");
    require(!pnr::prioritizeMovingTrigger(moving_tasks, 8,
                [](int task, int trigger) { return task == trigger; })
            && moving_tasks == std::vector<int>({2, 4, 7, 9}),
        "Moving changed the task order for an unknown trigger");

    // Check: recovering an incomplete incident route gets one retry at the current placement.
    require(!pnr::relocateAfterIncidentRequeue(1, 0),
        "Moving relocated before trying its requeued incident route");
    // Check: rediscovering the same incomplete route advances the deterministic placement cycle.
    require(pnr::relocateAfterIncidentRequeue(1, 1),
        "Moving requeued the same incomplete incident route indefinitely");
    require(pnr::relocateAfterIncidentRequeue(0, 0),
        "Moving suppressed relocation when no incident route was recovered");

    // Check: Moving commits only the still-placed candidate position that was
    // connectivity- and route-checked during the deterministic scan.
    require(pnr::acceptMovingPlacedCandidate(true, false),
        "Moving rejected its still-placed untried candidate");
    require(!pnr::acceptMovingPlacedCandidate(false, false)
            && !pnr::acceptMovingPlacedCandidate(true, true),
        "Moving accepted a repacked or already-tried candidate position");

    // Check: a tile-local void binding is complete even though it owns no Wire route.
    require(!pnr::incidentBindingNeedsRouting(true, false, false),
        "Moving requeued a tile-local void connection");
    require(pnr::incidentBindingNeedsRouting(false, false, false),
        "Moving accepted a physical binding without a route");
    require(!pnr::incidentBindingNeedsRouting(false, true, true),
        "Moving requeued a complete physical route");
    require(pnr::discardOwnerlessDuplicateBinding(false, true),
        "Moving retained an ownerless duplicate of a complete physical binding");
    require(!pnr::discardOwnerlessDuplicateBinding(false, false)
            && !pnr::discardOwnerlessDuplicateBinding(true, true),
        "Moving discarded a unique or physically owned route binding");

    // Check: requeued work keeps the source tree role established by its siblings.
    require(pnr::incidentBindingIsFanout(true),
        "Moving restarted a source whose physical tree was already routed");
    require(!pnr::incidentBindingIsFanout(false),
        "Moving made a Fanout task without an existing physical source tree");

    // Check: an endpoint attached to a neighboring route tile is not transit there.
    require(pnr::endpointRouteTileMatches(Coord{10, 11}, Coord{9, 11}, Coord{9, 11}),
        "attached route tile was not recognized as the endpoint crossbar");
    require(!pnr::endpointRouteTileMatches(Coord{10, 11}, Coord{9, 11}, Coord{8, 11}),
        "unrelated transit tile was classified as an endpoint crossbar");

    // Check: the selected Moving focus may use takeoff or grounding preemption,
    // while background Moving repair cannot disturb unrelated route trees.
    require(pnr::canPreemptDuringFocusedMove(true, true),
        "Moving blocked transit preemption for its active focus");
    require(!pnr::canPreemptDuringFocusedMove(true, false),
        "unfocused Moving repair preempted an unrelated route tree");
    require(pnr::canPreemptDuringFocusedMove(false, false),
        "Generic/Fanout routing inherited the Moving preemption guard");

    // Check: a Moving Generic seed first releases stale partial source siblings.
    require(pnr::resetIncompleteSourceTree(true, false, false, true),
        "Moving retained an incomplete source tree ahead of its Generic seed");
    require(!pnr::resetIncompleteSourceTree(true, true, false, true),
        "Moving reset the source tree while processing a Fanout sibling");
    require(!pnr::resetIncompleteSourceTree(true, false, true, true),
        "Moving reset a source tree that already had a complete seed");
    require(!pnr::resetIncompleteSourceTree(false, false, false, true),
        "Generic/Fanout routing inherited Moving source-tree repair");
    require(!pnr::incompleteBindingBlocksMovingSeed(true, true, false),
        "Moving treated its current incremental prefix as a stale sibling");
    require(pnr::incompleteBindingBlocksMovingSeed(false, true, false),
        "Moving ignored an incomplete sibling holding the source tree");
    require(!pnr::incompleteBindingBlocksMovingSeed(false, true, true),
        "Moving treated a complete Fanout seed as stale state");
    require(pnr::movingFocusPlacementsExhausted(71, 71),
        "Moving did not defer a focus after exhausting its placement set");
    require(!pnr::movingFocusPlacementsExhausted(70, 71),
        "Moving deferred a focus before trying its final placement");
    // Check: a completed Moving focus remains fixed while later source repairs
    // reroute any temporarily invalidated branch to the same sink placement.
    require(pnr::movingFinishedMarkIsValid(true, true),
        "Moving discarded a valid completed-cell mark");
    require(!pnr::movingFinishedMarkIsValid(true, false),
        "Moving retained a stale mark after an incident route was invalidated");
    require(!pnr::movingFinishedMarkIsValid(false, true),
        "Moving invented an absent completed-cell mark");
    struct MovingSeedTask
    {
        int source = 0;
        bool fanout = true;
    };
    std::vector<MovingSeedTask> moving_seed_tasks{
        {1, false}, {1, true}, {1, false},
        {2, true}, {2, true},
        {3, false}, {3, true}
    };
    pnr::MovingSeedNormalization moving_seed_result = pnr::normalizeMovingSourceRoles(
        moving_seed_tasks,
        [](const MovingSeedTask& task) {
            return std::to_string(task.source);
        },
        [](const MovingSeedTask& task) {
            return task.source == 3;
        });
    // Check: Moving keeps one existing Generic seed, promotes one when absent,
    // and demotes every queued task when the source already has a routed seed.
    require(moving_seed_result.promoted == 1 && moving_seed_result.demoted == 2,
        "Moving source-role normalization reported incorrect changes");
    require(!moving_seed_tasks[0].fanout && moving_seed_tasks[1].fanout
            && moving_seed_tasks[2].fanout,
        "Moving retained duplicate Generic tasks for one source");
    require(!moving_seed_tasks[3].fanout && moving_seed_tasks[4].fanout,
        "Moving did not promote exactly one missing Generic seed");
    require(moving_seed_tasks[5].fanout && moving_seed_tasks[6].fanout,
        "Moving retained a Generic task beside a completed source seed");
    pnr::MovingSeedNormalization stable_seed_result = pnr::normalizeMovingSourceRoles(
        moving_seed_tasks,
        [](const MovingSeedTask& task) {
            return std::to_string(task.source);
        },
        [](const MovingSeedTask& task) {
            return task.source == 3;
        });
    require(stable_seed_result.promoted == 0 && stable_seed_result.demoted == 0,
        "Moving source-role normalization was not stable");

    std::vector<uint64_t> moving_cycle{11, 12, 13, 14};
    // Check: an atomic focus can restart its deterministic placement cycle only
    // after exhausting every candidate in that cycle.
    require(pnr::restartMovingPlacementCycle(moving_cycle, 4) && moving_cycle.empty(),
        "Moving did not restart its exhausted placement cycle");
    moving_cycle = {21, 22, 23};
    require(!pnr::restartMovingPlacementCycle(moving_cycle, 4)
            && moving_cycle.size() == 3,
        "Moving restarted a placement cycle before exhausting it");

    // Check: the cells/10 scheduler quantum does not truncate the deterministic
    // candidate universe to the same short prefix on every cycle.
    require(pnr::movingCandidateRetryLimit(50) == 500
            && pnr::movingCandidateRetryLimit(0) == 10,
        "Moving candidate history did not retain ten scheduler quanta");
    require(!pnr::movingFocusPlacementsExhausted(
                71, pnr::movingCandidateRetryLimit(71)),
        "Moving restarted after the first repeated candidate prefix");
    require(pnr::movingPlacementKey(0, 64, 7) != pnr::movingPlacementKey(1, 0, 7)
            && pnr::movingPlacementKey(1, 0, 7) != pnr::movingPlacementKey(1, 0, 8),
        "Moving placement history aliases coordinates or resource positions");

    // Check: bounded focus slices advance over retained placement history, but
    // a zero limit keeps an atomic Moving focus active until it is complete.
    require(!pnr::movingFocusSliceExhausted(15, 0, 16)
            && pnr::movingFocusSliceExhausted(16, 0, 16)
            && pnr::movingFocusSliceExhausted(32, 16, 16)
            && !pnr::movingFocusSliceExhausted(3, 0, 4)
            && pnr::movingFocusSliceExhausted(4, 0, 4)
            && pnr::movingFocusSliceExhausted(11, 7, 4)
            && !pnr::movingFocusSliceExhausted(1000, 0, 0),
        "Moving focus slices do not advance over retained placement history");

    // Check: every yielded focus resumes its deterministic candidate sequence,
    // while a fully exhausted design-sized cycle restarts from its beginning.
    require(pnr::retainMovingPlacementHistory(33, 32, false)
            && pnr::retainMovingPlacementHistory(32, 32, false)
            && !pnr::retainMovingPlacementHistory(5, 32, true),
        "Moving placement history restarted before exhausting its full cycle");

    // Check: relocation of a packed cluster uses every external route endpoint
    // as an anchor and ignores routes wholly inside or outside that cluster.
    int cluster_a = 1;
    int cluster_b = 2;
    int external_a = 3;
    int external_b = 4;
    auto in_cluster = [&](int* endpoint) {
        return endpoint == &cluster_a || endpoint == &cluster_b;
    };
    require(pnr::externalMoveAnchor(&cluster_a, &external_a, in_cluster) == &external_a,
        "Moving did not anchor a cluster output at its external sink");
    require(pnr::externalMoveAnchor(&external_b, &cluster_b, in_cluster) == &external_b,
        "Moving did not anchor a cluster input at its external driver");
    require(pnr::externalMoveAnchor(&cluster_a, &cluster_b, in_cluster) == nullptr,
        "Moving used an internal cluster route as a relocation anchor");
    require(pnr::externalMoveAnchor(&external_a, &external_b, in_cluster) == nullptr,
        "Moving used an unrelated route as a relocation anchor");

    std::vector<int*> packed_cluster{&cluster_b, &cluster_a};
    int* cluster_owner = reinterpret_cast<uintptr_t>(&cluster_a)
            < reinterpret_cast<uintptr_t>(&cluster_b)
        ? &cluster_a : &cluster_b;
    require(pnr::movingClusterOwner(packed_cluster, &external_a) == cluster_owner
            && pnr::movingClusterOwner(std::vector<int*>{}, &external_a) == &external_a,
        "Moving did not share one stable history owner across a packed cluster");

    // Check: an explicit stable identity chooses the same cluster owner even
    // when allocation addresses and cluster traversal order are different.
    struct NamedMovingMember
    {
        std::string name;
    } named_a{"a"}, named_b{"b"}, named_fallback{"z"};
    auto named_less = [](NamedMovingMember* left, NamedMovingMember* right) {
        return left->name < right->name;
    };
    require(pnr::movingClusterOwner(
                std::vector<NamedMovingMember*>{&named_b, &named_a},
                &named_fallback, named_less) == &named_a
            && pnr::movingClusterOwner(
                std::vector<NamedMovingMember*>{&named_a, &named_b},
                &named_fallback, named_less) == &named_a,
        "Moving cluster ownership depended on traversal or allocation order");

    // Check: a neighboring route tile with no endpoint translation must not
    // accept coincident resource-local bit numbers as a routable endpoint.
    require(pnr::mappedOutputCandidateNodes<uint16_t>(0, 0x20, false, true) == 0
            && pnr::mappedOutputCandidateNodes<uint16_t>(0, 0x20, true, true) == 0x20
            && pnr::mappedOutputCandidateNodes<uint16_t>(0x04, 0x20, false, true) == 0x04,
        "Moving candidate validation reused unmapped resource-local node numbers");

    // Check: Moving visits every coordinate once in expanding rings, while an
    // accepted candidate stops the scan before any later coordinate is built.
    std::vector<std::pair<int, int>> moving_coords;
    bool moving_scan_stopped = pnr::forEachMovingCandidateCoord(10, 20, 2,
                [&](int x, int y) {
                    moving_coords.emplace_back(x, y);
                    return false;
                });
    std::unordered_set<uint64_t> unique_moving_coords;
    for (const auto& [x, y] : moving_coords) {
        unique_moving_coords.insert((static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32)
            | static_cast<uint32_t>(y));
    }
    require(!moving_scan_stopped && moving_coords.size() == 25
            && moving_coords.front() == std::pair{10, 20}
            && unique_moving_coords.size() == moving_coords.size(),
        "Moving candidate rings omitted or repeated coordinates");
    size_t moving_visited = 0;
    require(pnr::forEachMovingCandidateCoord(10, 20, 8,
                [&](int, int) {
                    return ++moving_visited == 3;
                })
            && moving_visited == 3,
        "Moving candidate scan materialized coordinates after acceptance");


    // Check: replacing a focused queue keeps source-tree siblings that were
    // introduced after the original deferred snapshot was made.
    struct MovingQueueTask
    {
        int id = 0;
    };
    std::vector<MovingQueueTask> active_queue{{1}, {2}, {3}, {4}};
    std::vector<MovingQueueTask> replacement_queue{{2}, {4}, {5}};
    std::vector<int> displaced_ids;
    size_t displaced = pnr::deferDisplacedActiveTasks(active_queue, replacement_queue,
        [](const MovingQueueTask& left, const MovingQueueTask& right) {
            return left.id == right.id;
        },
        [&](const MovingQueueTask& task) {
            displaced_ids.push_back(task.id);
            return true;
        });
    require(displaced == 2 && displaced_ids == std::vector<int>({1, 3}),
        "Moving dropped active source-tree siblings while replacing its focus queue");

    struct AffectedMoveTask
    {
        int binding = 0;
        bool complete = false;
    };
    std::vector<AffectedMoveTask> affected_move_tasks{
        {0, true}, {1, false}, {2, false}, {3, true}, {4, false}
    };
    std::vector<int> queued_move_bindings;
    size_t queued_move_tasks = pnr::enqueueIncompleteAffectedTasks(
        affected_move_tasks,
        [](const AffectedMoveTask& task) { return task.complete; },
        [&](const AffectedMoveTask& task) {
            queued_move_bindings.push_back(task.binding);
            return true;
        });
    // Check: relocation schedules all incomplete incident bindings at once,
    // including routes that were empty before the focused branch was removed.
    require(queued_move_tasks == 3
            && queued_move_bindings == std::vector<int>({1, 2, 4}),
        "Moving forgot an already-empty incident route during relocation");

    // Check: a generated passthrough replaces the physical route endpoint instead of leaving stale work.
    rtl::Inst old_from;
    rtl::Inst old_to;
    rtl::Inst new_from;
    rtl::Inst new_to;
    rtl::Net retarget_net;
    retarget_net.routes.push_back(rtl::NetRouteBinding{
        &old_to, 0, &old_from, &old_to, "Q", "D", "retarget_route"});
    require(fpga::retargetNetRouteBindings(retarget_net, retarget_net,
                &old_from, &old_to, "Q", "D",
                &new_from, &new_to, "O", "I", "retarget_route") == 1,
        "passthrough endpoint did not retarget its route binding");
    require(retarget_net.routes.size() == 1
            && retarget_net.routes[0].from == &new_from
            && retarget_net.routes[0].to == &new_to
            && retarget_net.routes[0].from_port == "O"
            && retarget_net.routes[0].to_port == "I",
        "passthrough endpoint left an obsolete route binding");

    rtl::Inst sibling_to;
    rtl::Inst unrelated_from;
    rtl::Net source_tree_net;
    source_tree_net.routes.push_back(rtl::NetRouteBinding{
        &old_to, 0, &old_from, &old_to, "Q", "D", "source_branch_a"});
    source_tree_net.routes.push_back(rtl::NetRouteBinding{
        &sibling_to, 0, &old_from, &sibling_to, "Q", "D", "source_branch_b"});
    source_tree_net.routes.push_back(rtl::NetRouteBinding{
        &sibling_to, 1, &unrelated_from, &sibling_to, "Q", "D", "unrelated"});
    require(fpga::retargetNetRouteSourceBindings(
                source_tree_net, &old_from, "Q", &new_from, "O") == 2,
        "source passthrough did not retarget every sibling binding");
    // Both branches move atomically while an unrelated physical source remains unchanged.
    require(source_tree_net.routes[0].from == &new_from
            && source_tree_net.routes[1].from == &new_from
            && source_tree_net.routes[0].from_port == "O"
            && source_tree_net.routes[1].from_port == "O"
            && source_tree_net.routes[2].from == &unrelated_from,
        "source passthrough split one source tree across endpoint identities");

    // Check: moving a binding between nets also moves source-tile route ownership.
    fpga::Tile& retarget_tile = resetDevice();
    Referable<rtl::Net> old_retarget_net;
    Referable<rtl::Net> new_retarget_net;
    rtl::Inst retarget_owner;
    retarget_owner.wires.emplace_back();
    fpga::Wire retarget_fragment;
    retarget_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    retarget_fragment.from = retarget_tile.coord;
    retarget_fragment.to = {retarget_tile.coord.x + 1, retarget_tile.coord.y};
    retarget_fragment.jump = 7;
    retarget_owner.wires.back().push_back(retarget_fragment);
    old_retarget_net.routes.push_back(rtl::NetRouteBinding{
        &retarget_owner, 0, &old_from, &old_to, "Q", "D", "retarget_owned_route"});
    fpga::registerNetRouteTiles(old_retarget_net, retarget_owner.wires.back());
    require(fpga::retargetNetRouteBindings(old_retarget_net, new_retarget_net,
                &old_from, &old_to, "Q", "D",
                &new_from, &new_to, "O", "I", "retarget_owned_route") == 1,
        "cross-net passthrough retarget did not move its binding");
    bool old_tile_owner = false;
    bool new_tile_owner = false;
    for (auto& route_ref : retarget_tile.routedNets) {
        old_tile_owner = old_tile_owner || route_ref.peer == &old_retarget_net;
        new_tile_owner = new_tile_owner || route_ref.peer == &new_retarget_net;
    }
    require(!old_tile_owner && new_tile_owner,
        "passthrough retarget left the partial source prefix owned by the old net");

    // Check: only a new best unfinished count resets the Fanout plateau window.
    size_t best_remaining = 12;
    size_t plateau_passes = 7;
    require(pnr::updateFanoutPlateau(11, best_remaining, plateau_passes),
        "Fanout best-count improvement was not recorded");
    require(best_remaining == 11 && plateau_passes == 0,
        "Fanout plateau was not reset by a new best count");
    require(!pnr::updateFanoutPlateau(12, best_remaining, plateau_passes),
        "Fanout regression was treated as a best-count improvement");
    require(plateau_passes == 1,
        "Fanout plateau pass was not counted");

    // Check: Fanout transit preemption follows its stage policy without affecting Generic.
    require(!pnr::transitPreemptionEnabled(true, true, false),
        "Fanout free-path phase still enabled transit preemption");
    require(pnr::transitPreemptionEnabled(true, true, true),
        "Fanout grounding did not enable requested transit preemption");
    require(pnr::transitPreemptionEnabled(true, false, false),
        "Generic routing inherited the Fanout preemption restriction");
    require(!pnr::transitPreemptionEnabled(false, false, true),
        "unrequested transit preemption was enabled");

    // Check: Generic and Fanout grounding use the first routed node inside radius,
    // even when the remaining depth-limited suffix wanders back outside the radius.
    std::vector<std::pair<int, int>> grounding_path{{1, 1}, {2, 5}, {3, 8}, {4, 3}};
    require(pnr::earliestGroundingAnchor(grounding_path, 5) == 0,
        "grounding skipped the first route node already inside docking radius");
    require(pnr::earliestGroundingAnchor({{0, 1}, {1, 6}, {2, 4}}, 5) == 2,
        "grounding selected a source local or a node outside docking radius");

    // Check: diagonal offsets use the same square window as the docking search.
    require(pnr::dockingWindowDistance({122, 117}, {125, 114}) == 3,
        "docking gate incorrectly used Manhattan distance for a diagonal anchor");
}

void preempted_fanout_siblings_remain_deferred_during_basic_routing()
{
    // Check: Basic keeps secondary source-tree routes out of its Generic task queue.
    require(pnr::deferToFanoutStage(true, false, false, false),
        "Basic routing did not defer a preempted fanout sibling");
    require(!pnr::deferToFanoutStage(false, false, false, false),
        "Basic routing deferred the replacement Generic trunk");
    require(!pnr::deferToFanoutStage(true, true, false, false),
        "Fanout routing deferred its own active task");
    require(!pnr::deferToFanoutStage(true, false, true, false),
        "Moving routing deferred an incident fanout task");

    // Check: only actual Fanout work owns a removable branch suffix.
    require(pnr::failedContinuationOwnsOnlyBranch(false, true, true),
        "Fanout continuation failure selected whole-tree cleanup");
    require(!pnr::failedContinuationOwnsOnlyBranch(true, false, false),
        "Moving Generic continuation was mislabeled as a removable fanout branch");
    require(!pnr::failedContinuationOwnsOnlyBranch(false, false, false),
        "Generic trunk failure selected branch-only cleanup");

    // Check: a Generic repair cannot remain mislabeled inside the deferred Fanout queue.
    require(pnr::promoteGenericOutOfFanoutQueue(false, true),
        "Generic repair was not promoted out of deferred Fanout work");
    require(!pnr::promoteGenericOutOfFanoutQueue(true, true),
        "Fanout work was incorrectly promoted to Generic repair");
}

void failed_fanout_branch_advances_rotation_once()
{
    size_t branch_offset = 7;
    size_t branch_attempt = 4;

    // Check: discarding a failed private suffix consumes exactly its branch candidate.
    pnr::consumeFanoutBranch(branch_offset, branch_attempt);
    require(branch_offset == 8 && branch_attempt == 0,
        "failed Fanout branch did not advance to its immediate successor");

    // Check: the following shared-prefix cleanup cannot consume a second candidate.
    branch_attempt = 2;
    pnr::cleanFanoutSharedPrefix(branch_attempt);
    require(branch_offset == 8 && branch_attempt == 0,
        "shared-prefix cleanup skipped an untried Fanout branch candidate");

    // Check: a second real branch failure still advances by exactly one.
    pnr::consumeFanoutBranch(branch_offset, branch_attempt);
    require(branch_offset == 9,
        "successive Fanout branch failures did not preserve contiguous rotation order");
}


struct MuxPlacementFixture
{
    Referable<rtl::Module> parent;
    Referable<rtl::Module> cell_module;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    int next_designator = 1000;

    MuxPlacementFixture()
    {
        parent.name = "top";
        parent.is_blackbox = false;
        parent.nets.reserve(32);
        cell_module.name = "primitive";
        cell_module.is_blackbox = true;
        cell_module.parent_ref.set(&parent);
    }

    Referable<rtl::Cell>* makeCell(const std::string& name, const std::string& type,
                                   const std::vector<std::pair<std::string, int>>& ports)
    {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name;
        cell->type = type;
        cell->module_ref.set(&cell_module);
        cell->ports.reserve(ports.size());
        for (const auto& [port_name, port_type] : ports) {
            rtl::Port port;
            port.name = port_name;
            port.type = static_cast<decltype(port.type)>(port_type);
            port.designator = -1;
            cell->ports.emplace_back(std::move(port));
        }
        Referable<rtl::Cell>* raw = cell.get();
        cells.push_back(std::move(cell));
        return raw;
    }

    Referable<rtl::Inst>* makeInst(const std::string& name, const std::string& type,
                                   const std::vector<std::pair<std::string, int>>& ports)
    {
        auto inst = std::make_unique<Referable<rtl::Inst>>();
        inst->cell_ref.set(makeCell(name + "_cell", type, ports));
        inst->pos = -1;
        inst->conns.reserve(ports.size());
        for (auto& port : inst->cell_ref->ports) {
            auto& conn = inst->conns.emplace_back();
            conn.port_ref.set(&port);
            conn.inst_ref.set(inst.get());
        }
        Referable<rtl::Inst>* raw = inst.get();
        insts.push_back(std::move(inst));
        return raw;
    }

    Referable<rtl::Conn>* conn(Referable<rtl::Inst>* inst, const std::string& port_name)
    {
        for (auto& conn_ref : inst->conns) {
            if (conn_ref.port_ref.peer && conn_ref.port_ref->name == port_name) {
                return &conn_ref;
            }
        }
        return nullptr;
    }

    rtl::Net& connect(Referable<rtl::Inst>* driver, const std::string& driver_port,
                      Referable<rtl::Inst>* sink, const std::string& sink_port,
                      const std::string& net_name)
    {
        Referable<rtl::Conn>* out = conn(driver, driver_port);
        Referable<rtl::Conn>* in = conn(sink, sink_port);
        require(out && in, "mux placement test connection references a missing port");
        int designator = next_designator++;
        out->port_ref->designator = designator;
        in->port_ref->designator = designator;
        in->set(out);
        auto& net = parent.nets.emplace_back();
        net.name = net_name;
        net.designators.push_back(designator);
        return net;
    }
};

fpga::Tile& resetMuxPlacementTile(fpga::TileType& tile_type)
{
    fpga::Tile& tile = resetDevice();
    tile_type.name = "GENERIC_ROUTE_TEST";
    tile_type.num = 1;
    tile_type.sites.clear();
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE0", .type = "LOGIC", .pos = 0});
    tile_type.sites.push_back(fpga::SiteModel{.name = "SITE1", .type = "LOGIC", .pos = 1});
    tile_type.elements.clear();
    tile_type.elements.push_back(makeElement("LUT5_0", fpga::ELEMENT_LUT5, 0));
    tile_type.elements.push_back(makeElement("LUT5_1", fpga::ELEMENT_LUT5, 1));
    tile_type.elements.push_back(makeElement("LUT5_2", fpga::ELEMENT_LUT5, 2));
    tile_type.elements.push_back(makeElement("MUXF7_0", fpga::ELEMENT_MUXF7, 0));
    connectElements(tile_type, fpga::ELEMENT_LUT5, 0, fpga::ELEMENT_MUXF7, 0);
    connectElements(tile_type, fpga::ELEMENT_LUT5, 1, fpga::ELEMENT_MUXF7, 0);
    tile.tile_type = &tile_type;
    return tile;
}

struct LeafMuxShape
{
    Referable<rtl::Inst>* mux = nullptr;
    Referable<rtl::Inst>* lut0 = nullptr;
    Referable<rtl::Inst>* lut1 = nullptr;
};

LeafMuxShape makeLeafMux(MuxPlacementFixture& fixture, const std::string& prefix,
                         std::vector<rtl::Net*>& internal_nets)
{
    LeafMuxShape shape;
    shape.lut0 = fixture.makeInst(prefix + "_lut0", "LUT6", {{"O", rtl::Port::PORT_OUT}});
    shape.lut1 = fixture.makeInst(prefix + "_lut1", "LUT6", {{"O", rtl::Port::PORT_OUT}});
    shape.mux = fixture.makeInst(prefix + "_mux", "MUX2",
        {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN},
         {"S", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
    internal_nets.push_back(&fixture.connect(shape.lut0, "O", shape.mux, "I0", prefix + "_i0"));
    internal_nets.push_back(&fixture.connect(shape.lut1, "O", shape.mux, "I1", prefix + "_i1"));
    return shape;
}

void connected_mux_inputs_are_void_when_packed_by_element_rules()
{
    fpga::TileType tile_type{"GENERIC_ROUTE_TEST", 1};
    fpga::Tile& tile = resetMuxPlacementTile(tile_type);
    MuxPlacementFixture fixture;

    std::vector<rtl::Net*> mux0_nets;
    LeafMuxShape mux0 = makeLeafMux(fixture, "mux0", mux0_nets);

    int lut0_pos = tile.tryAdd(mux0.lut0);
    int lut1_pos = tile.tryAdd(mux0.lut1);
    rtl::Inst route_owner;
    route_owner.wires.emplace_back();
    fpga::Wire preexisting_route;
    preexisting_route.type = fpga::Wire::WIRE_CROSSBAR;
    preexisting_route.from = tile.coord;
    preexisting_route.to = tile.coord;
    preexisting_route.local = 11;
    preexisting_route.pos = 0;
    preexisting_route.jump = 27;
    route_owner.wires.back().push_back(preexisting_route);
    tile.cb.local.local |= bit(11);
    tile.cb.src.jump |= bit(27);
    fpga::attachNetRoute(*mux0_nets.front(), route_owner, 0, mux0.lut0, mux0.mux, "O", "I0", "preexisting_internal_route");
    fpga::registerNetRouteTiles(*mux0_nets.front(), route_owner.wires.back());
    require(!route_owner.wires.back().empty() && tile.routedNets.size() == 1,
        "test setup failed to register a preexisting routed internal net");

    int mux_pos = tile.tryAdd(mux0.mux);
    require(lut0_pos >= 0 && lut1_pos >= 0, "connected mux LUT drivers could not be placed");
    require(mux_pos >= 0, "connected mux could not be placed next to its LUT drivers");
    require(mux0.lut0->tile.peer && mux0.lut1->tile.peer && mux0.mux->tile.peer,
        "connected mux shape was not fully assigned to the tile");
    for (rtl::Net* net : mux0_nets) {
        require(net && net->void_net, "first packed mux input net was not marked void");
    }
    require(route_owner.wires.back().empty(), "voiding a packed mux input did not clear its old route");
    require(!isSet(tile.cb.local.local, 11) && !isSet(tile.cb.src.jump, 27),
        "voiding a packed mux input did not release its old route leases");
    require(std::none_of(tile.routedNets.begin(), tile.routedNets.end(), [&](const Ref<rtl::Net>& ref) {
        return ref.peer == mux0_nets.front();
    }), "voiding a packed mux input left a stale routed net reference in the tile");
}

void mux_selector_remains_routable_when_driver_shares_tile()
{
    fpga::TileType tile_type{"GENERIC_ROUTE_TEST", 1};
    fpga::Tile& tile = resetMuxPlacementTile(tile_type);
    MuxPlacementFixture fixture;

    std::vector<rtl::Net*> data_nets;
    LeafMuxShape shape = makeLeafMux(fixture, "selector_shape", data_nets);
    Referable<rtl::Inst>* selector = fixture.makeInst(
        "selector_driver", "LUT2", {{"O", rtl::Port::PORT_OUT}});
    rtl::Net& selector_net = fixture.connect(
        selector, "O", shape.mux, "S", "selector_must_use_fabric");

    require(tile.tryAdd(shape.lut0) >= 0 && tile.tryAdd(shape.lut1) >= 0,
        "selector regression could not place mux data LUTs");
    require(tile.tryAdd(shape.mux) >= 0 && tile.tryAdd(selector) >= 0,
        "selector regression could not place mux and selector driver");

    // Dedicated data arcs are internal, but the selector still needs a route
    // even when all four elements happen to occupy the same tile.
    require(std::all_of(data_nets.begin(), data_nets.end(), [](rtl::Net* net) {
                return net && net->void_net;
            }),
        "packed mux data arcs were not marked void");
    require(!selector_net.designatorIsVoid(
                fixture.conn(shape.mux, "S")->port_ref->designator),
        "packed mux selector was incorrectly removed from routing tasks");
}

void joint_mediated_src_nodes_are_indexed()
{
    fpga::CBType type{};
    type.local_joint[7].joint |= bit(3);
    type.joint_src[3].jump |= bit(42);
    type.dst_joint[9].joint |= bit(4);
    type.joint_joint[4].joint |= bit(5);
    type.joint_src[5].jump |= bit(43);

    require(type.srcNodes(fpga::CB_NODE_LOCAL, 7) == nullptr,
        "test setup expected no src index before rebuild");

    type.rebuildOutgoingSrcs();

    const std::vector<uint16_t>* local_srcs = type.srcNodes(fpga::CB_NODE_LOCAL, 7);
    require(local_srcs != nullptr, "local->joint->src path was not indexed as local outgoing src");
    require(std::find(local_srcs->begin(), local_srcs->end(), 42) != local_srcs->end(),
        "local->joint->src index missed source node");

    const std::vector<uint16_t>* dst_srcs = type.srcNodes(fpga::CB_NODE_DST, 9);
    require(dst_srcs != nullptr, "dst->joint->joint->src path was not indexed as dst outgoing src");
    require(std::find(dst_srcs->begin(), dst_srcs->end(), 43) != dst_srcs->end(),
        "dst->joint->joint->src index missed source node");
}

void can_in_rejects_unconnected_double_joint_paths()
{
    fpga::CBType type{};
    int joint = -1;
    int first_joint = -1;
    for (int i = 0; i < CB_MAX_NODES; ++i) {
        type.dst_local[i].local = NodeMask{};
        type.dst_joint[i].joint = NodeMask{};
        type.joint_local[i].local = NodeMask{};
        type.joint_joint[i].joint = NodeMask{};
    }

    type.dst_joint[10].joint |= bit(3);
    type.joint_joint[3].joint |= bit(4);
    type.joint_local[5].local |= bit(20);

    require(!type.canIn(10, 20, joint),
        "dst joint with no path to local joint was incorrectly accepted");

    type.joint_joint[3].joint |= bit(5);
    require(type.canIn(10, 20, joint, &first_joint),
        "dst->joint->joint->local path was not accepted after adding the missing joint link");
    require(first_joint == 3 && joint == 5,
        "double-joint input path did not preserve both joint node identities");
    require(!type.canInAvoidingJoint(20, 3) && !type.canInAvoidingJoint(20, 5),
        "terminal path incorrectly avoided one of its required joints");

    type.dst_joint[11].joint |= bit(6);
    type.joint_local[6].local |= bit(20);
    type.rebuildOutgoingSrcs();
    require(type.canInAvoidingJoint(20, 3) && type.canInAvoidingJoint(20, 5),
        "alternate terminal path was not found around a blocked joint");
    require(type.canInAvoidingJoint(20, 6),
        "original double-joint path was not retained as an alternate");
}

void loaded_crossbar_local_and_joint_masks_use_router_bit_numbering()
{
    CBTypeSpec spec;
    spec.nodes.emplace("OUT5", "IN90");
    spec.nodes.emplace("OUT6", "JOINT7");
    spec.nodes.emplace("JOINT7", "IN91");

    fpga::TechMap map;
    fpga::CBType type{"GENERIC_CB"};
    type.loadFromSpec(spec, map);

    int out5 = type.local_nodes_by_name.at("OUT5");
    int out6 = type.local_nodes_by_name.at("OUT6");
    int in90 = type.local_nodes_by_name.at("IN90");
    int in91 = type.local_nodes_by_name.at("IN91");
    int joint7 = type.joint_nodes_by_name.at("JOINT7");

    // Check: loaded local masks must use the same low-lane bit numbering as routing leases/tests.
    require(isSet(type.local_local[out5].local, in90), "loaded local-to-local mask used incompatible bit numbering");
    // Check: loaded joint masks must also use router bit numbering, otherwise joint-mediated paths are invisible.
    require(isSet(type.local_joint[out6].joint, joint7), "loaded local-to-joint mask used incompatible bit numbering");
    require(isSet(type.joint_local[joint7].local, in91), "loaded joint-to-local mask used incompatible bit numbering");
    type.rebuildOutgoingSrcs();
    require(type.local_input_nodes != NodeMask{} && type.local_output_nodes != NodeMask{},
        "loaded local input/output masks were not populated with router bit numbering");
}

void tile_type_mapping_models_all_16_ff_input_pins_per_clb_tile()
{
    constexpr const char* description =
        "The abstract TileType mapping must model all 16 FF input pins per CLB tile. "
        "If it only maps AFF/BFF/CFF/DFF and misses AFF2/BFF2/CFF2/DFF2, placement may "
        "appear legal while routing local nodes are incomplete or aliased.";

    auto ff_pos = [] {
        std::vector<int> positions;
        positions.reserve(16);
        for (int site = 0; site < 2; ++site) {
            for (int ff = 0; ff < 8; ++ff) {
                positions.push_back(site * 128 + ff);
            }
        }
        return positions;
    }();

    auto addFdInputPins = [](fpga::TileType& type, const std::vector<int>& positions) {
        for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
            int pos = positions[static_cast<size_t>(i)];
            int d_local = 32 + i;
            int sr_local = 96 + i;

            type.pin_map.nodes[fpga::TilePinKey{"FDRE", "D", pos}] = bit(d_local);
            type.pin_map.nodes[fpga::TilePinKey{"FDRE", "R", pos}] = bit(sr_local);
            type.pin_map.nodes[fpga::TilePinKey{"FDSE", "S", pos}] = bit(sr_local);
            type.pin_map.rememberLocalNames(fpga::TILE_PIN_INPUT, d_local,
                "FF_D_LOCAL_" + std::to_string(i), "FF" + std::to_string(i) + ".D", "D");
            type.pin_map.rememberLocalNames(fpga::TILE_PIN_INPUT, sr_local,
                "FF_SR_LOCAL_" + std::to_string(i), "FF" + std::to_string(i) + ".SR", "SR");
        }
    };

    auto mappedLocal = [](const fpga::TileType& tile_type, const std::string& type,
                          const std::string& port, int pos) {
        NodeMask nodes = tile_type.pin_map.getNodes(type, port, pos);
        return nodes.firstSetBit();
    };

    fpga::TileType complete{"CLB_WITH_16_FF_INPUTS", 1};
    addFdInputPins(complete, ff_pos);

    std::set<int> d_nodes;
    std::set<int> sr_nodes;
    for (int pos : ff_pos) {
        int d_local = mappedLocal(complete, "FDRE", "D", pos);
        int r_local = mappedLocal(complete, "FDRE", "R", pos);
        int s_local = mappedLocal(complete, "FDSE", "S", pos);
        require(d_local >= 0, std::string(description) + " Missing FDRE.D at pos " + std::to_string(pos));
        require(r_local >= 0, std::string(description) + " Missing FDRE.R at pos " + std::to_string(pos));
        require(s_local >= 0, std::string(description) + " Missing FDSE.S at pos " + std::to_string(pos));
        require(r_local == s_local,
            std::string(description) + " Reset/set aliases should resolve to the same slot at pos " + std::to_string(pos));
        d_nodes.insert(d_local);
        sr_nodes.insert(r_local);
        require(complete.pin_map.localResourceName(fpga::TILE_PIN_INPUT, d_local) != nullptr,
            std::string(description) + " Missing D resource annotation at pos " + std::to_string(pos));
        require(complete.pin_map.localResourceName(fpga::TILE_PIN_INPUT, r_local) != nullptr,
            std::string(description) + " Missing SR resource annotation at pos " + std::to_string(pos));
    }
    require(d_nodes.size() == 16, std::string(description) + " FD D inputs are incomplete or aliased");
    require(sr_nodes.size() == 16, std::string(description) + " FD SR inputs are incomplete or aliased");

    fpga::TileType incomplete{"CLB_WITH_ONLY_FOUR_FF_INPUTS", 2};
    addFdInputPins(incomplete, std::vector<int>(ff_pos.begin(), ff_pos.begin() + 4));
    int mapped = 0;
    for (int pos : ff_pos) {
        if (mappedLocal(incomplete, "FDRE", "D", pos) >= 0) {
            ++mapped;
        }
    }
    require(mapped == 4, "incomplete four-FF mapping test setup is invalid");
    require(mapped != 16, std::string(description) + " Incomplete AFF/BFF/CFF/DFF-only mapping was not detected");
}

enum class RegressionRoutingMode
{
    Generic,
    Fanout,
    Moving,
};

struct RegressionTask
{
    std::string name;
    rtl::Port* source_port = nullptr;
    fpga::Coord source_tile;
    fpga::Coord branch_tile;
    fpga::Coord sink_tile;
    TestRoute* old_route = nullptr;
    int source_local = 120;
    int source_exit = 24;
    bool routed = false;
    bool fanout = false;
    bool moved = false;
    std::vector<fpga::Wire> route;
};

struct RegressionBatchState
{
    std::vector<RegressionTask> deferred_fanouts;
    size_t generic_routed = 0;
    size_t fanout_routed = 0;
    size_t moved_cells = 0;
};

void leaseRegressionFragment(const fpga::Wire& fragment)
{
    fpga::Tile* tile = fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
    require(tile != nullptr, "regression route tried to lease a missing tile");
    tile->cb.src.jump |= bit(fragment.jump);
    if (fragment.pos == 0) {
        tile->cb.local.local |= bit(fragment.local);
    }
    else {
        tile->cb.dst.jump |= bit(fragment.local);
    }
}

void routeRegressionTasks(RegressionRoutingMode mode, std::vector<RegressionTask>& tasks,
                          RegressionBatchState& state)
{
    constexpr uint64_t routed_source_mark = 0xace551;
    if (mode == RegressionRoutingMode::Moving) {
        bool moved_any = false;
        for (RegressionTask& task : tasks) {
            if (task.old_route && fpga::unrouteNet(task.old_route->net)) {
                task.old_route = nullptr;
            }
            task.source_tile = {1, 0};
            task.branch_tile = {1, 0};
            task.moved = true;
            moved_any = true;
            if (task.source_port) {
                task.source_port->mark = 0;
            }
        }
        if (moved_any) {
            ++state.moved_cells;
        }
        routeRegressionTasks(RegressionRoutingMode::Generic, tasks, state);
        std::vector<RegressionTask> fanouts = std::move(state.deferred_fanouts);
        state.deferred_fanouts.clear();
        routeRegressionTasks(RegressionRoutingMode::Fanout, fanouts, state);
        return;
    }

    for (RegressionTask& task : tasks) {
        if (mode == RegressionRoutingMode::Generic) {
            if (task.source_port && task.source_port->mark == routed_source_mark) {
                task.fanout = true;
                state.deferred_fanouts.push_back(task);
                continue;
            }
            if (task.source_port) {
                task.source_port->mark = routed_source_mark;
            }
            fpga::Wire fragment;
            fragment.type = fpga::Wire::WIRE_CROSSBAR;
            fragment.from = task.source_tile;
            fragment.to = task.branch_tile;
            fragment.local = task.source_local;
            fragment.jump = task.source_exit;
            fragment.pos = 0;
            fragment.net_name = task.name;
            task.route = {fragment};
            leaseRegressionFragment(fragment);
            task.routed = true;
            ++state.generic_routed;
            continue;
        }

        fpga::Wire shared;
        shared.type = fpga::Wire::WIRE_CROSSBAR;
        shared.from = task.source_tile;
        shared.to = task.branch_tile;
        shared.local = task.source_local;
        shared.jump = task.source_exit;
        shared.pos = 1;
        shared.shared = true;
        shared.net_name = task.name;

        fpga::Wire branch;
        branch.type = fpga::Wire::WIRE_CROSSBAR;
        branch.from = task.branch_tile;
        branch.to = task.sink_tile;
        branch.local = task.source_local + 1;
        branch.jump = task.source_exit + 1;
        branch.pos = 1;
        branch.net_name = task.name;

        task.route = {shared, branch};
        leaseRegressionFragment(branch);
        task.routed = true;
        ++state.fanout_routed;
    }
}

void routing_mode_generic_routes_only_one_net_from_single_source_port()
{
    resetDeviceGrid(3, 1);
    rtl::Port source;
    source.name = "O";
    source.type = rtl::Port::PORT_OUT;

    std::vector<RegressionTask> tasks;
    for (int i = 0; i < 4; ++i) {
        tasks.push_back(RegressionTask{
            "source_fanout_" + std::to_string(i),
            &source,
            {0, 0},
            {1, 0},
            {2, 0},
            nullptr,
            120 + i,
            24 + i,
        });
    }

    RegressionBatchState state;
    routeRegressionTasks(RegressionRoutingMode::Generic, tasks, state);

    // Check: generic mode consumes exactly one physical route start from one logical source port.
    require(state.generic_routed == 1, "generic mode routed more than one net from the same source port");
    // Check: secondary loads of the same source port are deferred to fanout routing.
    require(state.deferred_fanouts.size() == 3, "generic mode did not defer secondary source-port fanouts");
    // Check: the selected generic route really leased its source tile exit.
    require(isSet(fpga::Device::current().tile_grid.front().cb.src.jump, 24),
        "generic mode did not lease the first source exit");
}

void routing_mode_fanout_branches_away_from_source_tile()
{
    resetDeviceGrid(4, 1);
    rtl::Port source;
    source.name = "O";
    source.type = rtl::Port::PORT_OUT;

    std::vector<RegressionTask> tasks;
    for (int i = 0; i < 3; ++i) {
        tasks.push_back(RegressionTask{
            "branch_fanout_" + std::to_string(i),
            &source,
            {0, 0},
            {1, 0},
            {2 + i % 2, 0},
            nullptr,
            100,
            28,
        });
    }

    RegressionBatchState state;
    routeRegressionTasks(RegressionRoutingMode::Generic, tasks, state);
    std::vector<RegressionTask> fanouts = std::move(state.deferred_fanouts);
    state.deferred_fanouts.clear();
    routeRegressionTasks(RegressionRoutingMode::Fanout, fanouts, state);

    // Check: fanout mode routed all deferred loads through branch points.
    require(state.fanout_routed == 2, "fanout mode did not route every deferred fanout");
    for (const RegressionTask& task : fanouts) {
        auto first_new = std::find_if(task.route.begin(), task.route.end(), [](const fpga::Wire& wire) {
            return !wire.shared;
        });
        // Check: fanout mode starts the new branch after the shared trunk, not at the original source tile.
        require(first_new != task.route.end() && first_new->from.x != task.source_tile.x,
            "fanout mode started a secondary fanout in the original source tile");
        // Check: the branch point is the tile selected from the existing routed trunk.
        require(first_new->from.x == task.branch_tile.x && first_new->from.y == task.branch_tile.y,
            "fanout mode did not start from the expected branch tile");
    }
}

void routing_mode_moving_unroutes_old_cell_tree_and_reroutes_hierarchy()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(2, 1);
    fpga::Tile& old_tile = *tiles[0];
    fpga::Tile& new_tile = *tiles[1];

    std::vector<std::unique_ptr<TestRoute>> local_blockers;
    for (int i = 0; i < 8; ++i) {
        auto blocker = std::make_unique<TestRoute>();
        blocker->transit = false;
        blocker->exit = 8 + i;
        blocker->local = 64 + i;
        addRoute(old_tile, *blocker, "old_local_blocker_" + std::to_string(i));
        local_blockers.push_back(std::move(blocker));
    }

    TestRoute previous_route;
    previous_route.transit = false;
    previous_route.exit = 32;
    previous_route.local = 120;
    addRoute(old_tile, previous_route, "moved_cell_previous_route");

    rtl::Port source;
    source.name = "O";
    source.type = rtl::Port::PORT_OUT;
    std::vector<RegressionTask> tasks;
    for (int i = 0; i < 3; ++i) {
        tasks.push_back(RegressionTask{
            "moving_hierarchy_" + std::to_string(i),
            &source,
            old_tile.coord,
            old_tile.coord,
            new_tile.coord,
            i == 0 ? &previous_route : nullptr,
            140 + i,
            40 + i,
        });
    }

    // Check: the old source tile has no modeled local exits left for another local start.
    for (int i = 0; i < 8; ++i) {
        require(isSet(old_tile.cb.src.jump, 8 + i), "moving test setup did not occupy every old local exit");
    }

    RegressionBatchState state;
    routeRegressionTasks(RegressionRoutingMode::Moving, tasks, state);

    // Check: moving mode removed the previous route owned by the moved cell.
    require(!isSet(old_tile.cb.src.jump, 32), "moving mode did not clear the old route source exit");
    // Check: moving mode cleared the old local lease as part of unrouting the moved cell tree.
    require(!isSet(old_tile.cb.local.local, 120), "moving mode did not clear the old route local lease");
    // Check: unrelated local-to-exit blockers were not unrouted by moving cleanup.
    for (int i = 0; i < 8; ++i) {
        require(isSet(old_tile.cb.src.jump, 8 + i), "moving mode removed an unrelated local blocker");
    }
    // Check: moving mode reran generic routing first and fanout routing for the same source hierarchy after relocation.
    require(state.moved_cells == 1 && state.generic_routed == 1 && state.fanout_routed == 2,
        "moving mode did not reroute the relocated hierarchy through generic then fanout stages");
    // Check: the relocated generic route leased the new tile, proving routing did not retry from the blocked old tile.
    require(new_tile.cb.src.jump != NodeMask{}, "moving mode did not lease any exit on the new tile");
}

bool tileIsBlocked(const fpga::Tile& tile)
{
    return isSet(tile.cb.src.jump, 1);
}

std::vector<fpga::Coord> reconstructSyntheticPath(const std::vector<int>& parent,
                                                  const std::vector<fpga::Coord>& nodes,
                                                  int end)
{
    std::vector<fpga::Coord> path;
    for (int index = end; index >= 0; index = parent[static_cast<size_t>(index)]) {
        path.push_back(nodes[static_cast<size_t>(index)]);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<fpga::Coord> findLimitedSyntheticRouteChunk(fpga::Coord start, fpga::Coord dst,
                                                        int width, int height, int depth_limit,
                                                        const std::set<std::pair<int, int>>& route_seen)
{
    auto distance = [](fpga::Coord a, fpga::Coord b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };
    auto inside = [&](fpga::Coord coord) {
        return coord.x >= 0 && coord.y >= 0 && coord.x < width && coord.y < height;
    };

    std::vector<fpga::Coord> nodes{start};
    std::vector<int> parent{-1};
    std::vector<int> depth{0};
    std::vector<size_t> queue{0};
    std::set<std::pair<int, int>> seen = route_seen;
    seen.erase({start.x, start.y});
    seen.insert({start.x, start.y});
    int best = 0;
    int fallback = 0;

    for (size_t head = 0; head < queue.size(); ++head) {
        int current = static_cast<int>(queue[head]);
        fpga::Coord coord = nodes[static_cast<size_t>(current)];
        if (coord.x == dst.x && coord.y == dst.y) {
            best = current;
            break;
        }
        if (distance(coord, dst) < distance(nodes[static_cast<size_t>(best)], dst)) {
            best = current;
        }
        if (depth[static_cast<size_t>(current)] > depth[static_cast<size_t>(fallback)]) {
            fallback = current;
        }
        if (depth[static_cast<size_t>(current)] >= depth_limit) {
            continue;
        }

        std::vector<fpga::Coord> next = {
            {coord.x + 1, coord.y},
            {coord.x, coord.y - 1},
            {coord.x, coord.y + 1},
            {coord.x - 1, coord.y},
        };
        std::stable_sort(next.begin(), next.end(), [&](fpga::Coord a, fpga::Coord b) {
            return distance(a, dst) < distance(b, dst);
        });
        for (fpga::Coord candidate : next) {
            if (!inside(candidate) || seen.contains({candidate.x, candidate.y})) {
                continue;
            }
            fpga::Tile* tile = fpga::Device::current().getTile(candidate.x, candidate.y);
            if (!tile || tileIsBlocked(*tile)) {
                continue;
            }
            seen.insert({candidate.x, candidate.y});
            nodes.push_back(candidate);
            parent.push_back(current);
            depth.push_back(depth[static_cast<size_t>(current)] + 1);
            queue.push_back(nodes.size() - 1);
        }
    }

    if (best == 0) {
        best = fallback;
    }
    require(best != 0, "limited synthetic router found no usable path fragment");
    return reconstructSyntheticPath(parent, nodes, best);
}

void limited_iterations_find_one_tile_escape_path_behind_source()
{
    constexpr int width = 13;
    constexpr int height = 11;
    fpga::Coord src{5, 5};
    fpga::Coord dst{11, 5};
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(width, height);

    std::set<std::pair<int, int>> free_trail;
    for (int x = 0; x <= src.x; ++x) {
        free_trail.insert({x, src.y});
    }
    for (int y = 0; y <= src.y; ++y) {
        free_trail.insert({0, y});
    }
    for (int x = 0; x <= dst.x; ++x) {
        free_trail.insert({x, 0});
    }
    for (int y = 0; y <= dst.y; ++y) {
        free_trail.insert({dst.x, y});
    }
    free_trail.insert({dst.x, dst.y});

    for (fpga::Tile* tile : tiles) {
        int dx = std::abs(tile->coord.x - src.x);
        int dy = std::abs(tile->coord.y - src.y);
        bool in_filled_radius = dx <= 5 && dy <= 5;
        bool is_free = free_trail.contains({tile->coord.x, tile->coord.y});
        if (in_filled_radius && !is_free) {
            tile->cb.src.jump |= bit(1);
            tile->cb.dst.jump |= bit(1);
            tile->cb.local.local |= bit(1);
        }
    }

    // Check: the direct direction to the destination is blocked, forcing the first useful step backwards.
    require(tileIsBlocked(*fpga::Device::current().getTile(src.x + 1, src.y)),
        "escape test setup did not block the direct source-to-destination direction");
    // Check: the only immediate escape from the source points opposite the destination direction.
    require(!tileIsBlocked(*fpga::Device::current().getTile(src.x - 1, src.y)),
        "escape test setup accidentally blocked the one-tile-wide backward trail");

    fpga::Coord current = src;
    std::vector<fpga::Coord> route{src};
    std::set<std::pair<int, int>> route_seen{{src.x, src.y}};
    for (int pass = 0; pass < 8 && !(current.x == dst.x && current.y == dst.y); ++pass) {
        std::vector<fpga::Coord> chunk = findLimitedSyntheticRouteChunk(current, dst, width, height, 5, route_seen);
        require(chunk.size() > 1, "limited synthetic router did not advance");
        route.insert(route.end(), std::next(chunk.begin()), chunk.end());
        for (fpga::Coord coord : chunk) {
            route_seen.insert({coord.x, coord.y});
        }
        current = route.back();
    }

    // Check: limited-depth routing eventually escapes the blocked source region and reaches the sink.
    require(current.x == dst.x && current.y == dst.y,
        "limited synthetic router did not find the one-tile-wide escape path in several iterations");
    // Check: the route starts by moving opposite the destination direction before going around the blocked region.
    require(route.size() > 1 && route[1].x == src.x - 1 && route[1].y == src.y,
        "limited synthetic router did not take the required backward first step");
    // Check: the route uses only the deliberately preserved one-tile-wide trail through the filled radius.
    for (fpga::Coord coord : route) {
        require(free_trail.contains({coord.x, coord.y}),
            "limited synthetic router left the one-tile-wide free trail");
    }
}

void limited_continuation_is_strictly_incremental_without_rollbacks()
{
    constexpr int width = 15;
    constexpr int height = 9;
    constexpr int depth_limit = 3;
    fpga::Coord src{7, 4};
    fpga::Coord dst{14, 4};
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(width, height);

    std::set<std::pair<int, int>> free_trail;
    for (int x = src.x; x >= 1; --x) {
        free_trail.insert({x, src.y});
    }
    for (int y = src.y; y >= 1; --y) {
        free_trail.insert({1, y});
    }
    for (int x = 1; x <= dst.x; ++x) {
        free_trail.insert({x, 1});
    }
    for (int y = 1; y <= dst.y; ++y) {
        free_trail.insert({dst.x, y});
    }
    free_trail.insert({dst.x, dst.y});

    for (fpga::Tile* tile : tiles) {
        bool is_free = free_trail.contains({tile->coord.x, tile->coord.y});
        if (!is_free) {
            tile->cb.src.jump |= bit(1);
            tile->cb.dst.jump |= bit(1);
            tile->cb.local.local |= bit(1);
        }
    }

    // Check: the obstacle model blocks the direct route and leaves only a narrow detour.
    require(tileIsBlocked(*fpga::Device::current().getTile(src.x + 1, src.y)),
        "incremental continuation setup did not block the direct route");
    require(!tileIsBlocked(*fpga::Device::current().getTile(src.x - 1, src.y)),
        "incremental continuation setup blocked the required first detour tile");

    fpga::Coord current = src;
    std::vector<fpga::Coord> route{src};
    std::set<std::pair<int, int>> route_seen{{src.x, src.y}};
    for (int pass = 0; pass < 16 && !(current.x == dst.x && current.y == dst.y); ++pass) {
        std::vector<fpga::Coord> before = route;
        std::vector<fpga::Coord> chunk = findLimitedSyntheticRouteChunk(current, dst, width, height,
            depth_limit, route_seen);

        // Check: each limited pass must find at least one committed forward fragment.
        require(chunk.size() > 1, "incremental continuation pass did not produce a commit fragment");

        route.insert(route.end(), std::next(chunk.begin()), chunk.end());
        for (fpga::Coord coord : chunk) {
            route_seen.insert({coord.x, coord.y});
        }
        current = route.back();

        // Check: previous prefix is preserved exactly; no committed coordinate may be erased or changed.
        require(route.size() > before.size(), "incremental continuation did not grow the route");
        require(std::equal(before.begin(), before.end(), route.begin()),
            "incremental continuation changed an already committed prefix");
        // Check: incremental growth must not end by returning to an already committed endpoint.
        std::set<std::pair<int, int>> old_points;
        for (fpga::Coord coord : before) {
            old_points.insert({coord.x, coord.y});
        }
        require(!old_points.contains({route.back().x, route.back().y}),
            "incremental continuation ended at an already committed endpoint");
        // Check: the newly committed suffix stays on the deliberately free trail.
        for (auto it = route.begin() + static_cast<std::ptrdiff_t>(before.size()); it != route.end(); ++it) {
            require(free_trail.contains({it->x, it->y}),
                "incremental continuation committed a tile outside the free trail");
        }
        // Check: bounded passes are really partial before the final pass, so this covers continuation behavior.
        if (current.x != dst.x || current.y != dst.y) {
            require(chunk.size() <= static_cast<size_t>(depth_limit + 1),
                "incremental continuation unexpectedly completed in one unbounded pass");
        }
    }

    // Check: repeated incremental commits eventually follow the detour to the destination.
    require(current.x == dst.x && current.y == dst.y,
        "incremental continuation did not reach the target through repeated committed partial routes");
    // Check: the first committed move goes away from the destination; this prevents distance-only rollback policy.
    require(route.size() > 1 && route[1].x == src.x - 1 && route[1].y == src.y,
        "incremental continuation did not preserve the required backward first step");
}

void unroute_net_clears_multifragment_route_state()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(3, 1);
    fpga::Tile& source = *tiles[0];
    fpga::Tile& transit = *tiles[1];
    fpga::Tile& sink = *tiles[2];

    Referable<rtl::Net> net;
    net.name = "multifragment_unroute";
    rtl::Inst owner;
    owner.wires.emplace_back();
    std::vector<fpga::Wire>& route = owner.wires.back();

    fpga::Wire source_fragment;
    source_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    source_fragment.from = source.coord;
    source_fragment.to = transit.coord;
    source_fragment.local = 64;
    source_fragment.jump = 10;
    source_fragment.pos = 0;
    source_fragment.net_name = net.name;
    route.push_back(source_fragment);

    fpga::Wire transit_fragment;
    transit_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    transit_fragment.from = transit.coord;
    transit_fragment.to = sink.coord;
    transit_fragment.local = 20;
    transit_fragment.jump = 11;
    transit_fragment.joint = 5;
    transit_fragment.pos = 1;
    transit_fragment.net_name = net.name;
    route.push_back(transit_fragment);

    fpga::Wire sink_fragment;
    sink_fragment.type = fpga::Wire::WIRE_CROSSBAR;
    sink_fragment.from = sink.coord;
    sink_fragment.to = sink.coord;
    sink_fragment.local = 21;
    sink_fragment.jump = -1;
    sink_fragment.joint = 6;
    sink_fragment.pos = 1;
    sink_fragment.net_name = net.name;
    route.push_back(sink_fragment);

    fpga::Wire pin_fragment;
    pin_fragment.type = fpga::Wire::WIRE_TILE_PIN;
    pin_fragment.from = sink.coord;
    pin_fragment.to = sink.coord;
    pin_fragment.local = 70;
    pin_fragment.pos = 2;
    pin_fragment.net_name = net.name;
    route.push_back(pin_fragment);

    source.cb.local.local |= bit(source_fragment.local);
    source.cb.src.jump |= bit(source_fragment.jump);
    transit.cb.dst.jump |= bit(transit_fragment.local);
    transit.cb.src.jump |= bit(transit_fragment.jump);
    transit.cb.joint.jump |= bit(transit_fragment.joint);
    sink.cb.dst.jump |= bit(sink_fragment.local);
    sink.cb.joint.jump |= bit(sink_fragment.joint);
    sink.cb.local.local |= bit(pin_fragment.local);
    sink.pin_state.leased_nodes |= bit(pin_fragment.local);

    fpga::attachNetRoute(net, owner, 0, nullptr, &owner, {}, {}, net.name);
    fpga::registerNetRouteTiles(net, route);
    require(!source.routedNets.empty() && !transit.routedNets.empty() && !sink.routedNets.empty(),
        "multifragment unroute setup did not register route tiles");

    require(fpga::unrouteNet(net), "multifragment unroute returned false");

    // Check: all source-side leases from the first route fragment are released.
    require(!isSet(source.cb.local.local, source_fragment.local), "source local lease survived unroute");
    require(!isSet(source.cb.src.jump, source_fragment.jump), "source jump lease survived unroute");
    // Check: all transit dst/src/joint leases are released.
    require(!isSet(transit.cb.dst.jump, transit_fragment.local), "transit dst lease survived unroute");
    require(!isSet(transit.cb.src.jump, transit_fragment.jump), "transit src lease survived unroute");
    require(!isSet(transit.cb.joint.jump, transit_fragment.joint), "transit joint lease survived unroute");
    // Check: final entry and resource pin leases are released together.
    require(!isSet(sink.cb.dst.jump, sink_fragment.local), "sink dst lease survived unroute");
    require(!isSet(sink.cb.joint.jump, sink_fragment.joint), "sink joint lease survived unroute");
    require(!isSet(sink.cb.local.local, pin_fragment.local), "sink local pin lease survived unroute");
    require(!isSet(sink.pin_state.leased_nodes, pin_fragment.local), "sink pin_state lease survived unroute");
    // Check: route storage and per-tile net references are empty after atomic net unroute.
    require(route.empty(), "owner route vector was not cleared by unroute");
    require(std::all_of(source.routedNets.begin(), source.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "source routedNets kept unrouted net");
    require(std::all_of(transit.routedNets.begin(), transit.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "transit routedNets kept unrouted net");
    require(std::all_of(sink.routedNets.begin(), sink.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "sink routedNets kept unrouted net");
}

void grounding_preemption_route_tree_unroute_frees_terminal_masks()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(3, 2);
    fpga::Tile& source = *tiles[0];
    fpga::Tile& transit0 = *tiles[1];
    fpga::Tile& target0 = *tiles[2];
    fpga::Tile& transit1 = *tiles[4];
    fpga::Tile& target1 = *tiles[5];

    Referable<rtl::Net> net;
    net.name = "grounding_preemption_tree";
    rtl::Inst driver;
    rtl::Inst sink0;
    rtl::Inst sink1;
    driver.wires.resize(2);

    auto add_source_fragment = [&](std::vector<fpga::Wire>& route, int local, int src_bit,
                                   fpga::Coord to, const std::string& name) {
        fpga::Wire fragment;
        fragment.type = fpga::Wire::WIRE_CROSSBAR;
        fragment.from = source.coord;
        fragment.to = to;
        fragment.local = local;
        fragment.jump = src_bit;
        fragment.pos = 0;
        fragment.net_name = name;
        route.push_back(fragment);
        source.cb.local.local |= bit(local);
        source.cb.src.jump |= bit(src_bit);
    };
    auto add_transit_fragment = [](std::vector<fpga::Wire>& route, fpga::Tile& tile, fpga::Coord to,
                                   int dst_bit, int src_bit, int joint_bit, const std::string& name) {
        fpga::Wire fragment;
        fragment.type = fpga::Wire::WIRE_CROSSBAR;
        fragment.from = tile.coord;
        fragment.to = to;
        fragment.local = dst_bit;
        fragment.jump = src_bit;
        fragment.joint = joint_bit;
        fragment.pos = 1;
        fragment.net_name = name;
        route.push_back(fragment);
        tile.cb.dst.jump |= bit(dst_bit);
        tile.cb.src.jump |= bit(src_bit);
        tile.cb.joint.jump |= bit(joint_bit);
    };
    auto add_terminal_fragment = [](std::vector<fpga::Wire>& route, fpga::Tile& tile,
                                    int dst_bit, int joint_bit, int pin_bit, const std::string& name) {
        fpga::Wire entry;
        entry.type = fpga::Wire::WIRE_CROSSBAR;
        entry.from = tile.coord;
        entry.to = tile.coord;
        entry.local = dst_bit;
        entry.jump = -1;
        entry.joint = joint_bit;
        entry.pos = 1;
        entry.net_name = name;
        route.push_back(entry);

        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = tile.coord;
        pin.to = tile.coord;
        pin.local = pin_bit;
        pin.pos = 2;
        pin.net_name = name;
        route.push_back(pin);

        tile.cb.dst.jump |= bit(dst_bit);
        tile.cb.joint.jump |= bit(joint_bit);
        tile.cb.local.local |= bit(pin_bit);
        tile.pin_state.leased_nodes |= bit(pin_bit);
    };

    std::vector<fpga::Wire>& route0 = driver.wires[0];
    std::vector<fpga::Wire>& route1 = driver.wires[1];
    add_source_fragment(route0, 80, 12, transit0.coord, net.name);
    add_transit_fragment(route0, transit0, target0.coord, 30, 13, 5, net.name);
    add_terminal_fragment(route0, target0, 31, 6, 90, net.name);

    add_source_fragment(route1, 81, 14, transit1.coord, net.name);
    add_transit_fragment(route1, transit1, target1.coord, 32, 15, 7, net.name);
    add_terminal_fragment(route1, target1, 33, 8, 91, net.name);

    source.cb.src_deadend.jump |= bit(12);
    source.cb.src_deadend.jump |= bit(14);

    fpga::attachNetRoute(net, driver, 0, &driver, &sink0, "O", "I", "grounding_preemption_tree_0");
    fpga::attachNetRoute(net, driver, 1, &driver, &sink1, "O", "I", "grounding_preemption_tree_1");
    fpga::registerNetRouteTiles(net, route0);
    fpga::registerNetRouteTiles(net, route1);
    require(net.routes.size() == 2, "grounding preemption tree setup did not register two bindings");
    require(!source.routedNets.empty() && !target0.routedNets.empty() && !target1.routedNets.empty(),
        "grounding preemption tree setup did not register route tiles");

    require(fpga::unrouteNetRouteTree(net, {0, 1}), "grounding preemption route-tree unroute returned false");

    // Check: source-tree unroute frees both source locals and both outgoing takeoff nodes.
    require(!isSet(source.cb.local.local, 80), "grounding preemption left first source local leased");
    require(!isSet(source.cb.local.local, 81), "grounding preemption left second source local leased");
    require(!isSet(source.cb.src.jump, 12), "grounding preemption left first source exit leased");
    require(!isSet(source.cb.src.jump, 14), "grounding preemption left second source exit leased");
    // Check: sticky deadend learning remains after preemption cleanup.
    require(isSet(source.cb.src_deadend.jump, 12), "grounding preemption cleared first sticky deadend");
    require(isSet(source.cb.src_deadend.jump, 14), "grounding preemption cleared second sticky deadend");
    // Check: transit nodes from every preempted branch are fully freed.
    require(!isSet(transit0.cb.dst.jump, 30), "grounding preemption left first transit dst leased");
    require(!isSet(transit0.cb.src.jump, 13), "grounding preemption left first transit src leased");
    require(!isSet(transit0.cb.joint.jump, 5), "grounding preemption left first transit joint leased");
    require(!isSet(transit1.cb.dst.jump, 32), "grounding preemption left second transit dst leased");
    require(!isSet(transit1.cb.src.jump, 15), "grounding preemption left second transit src leased");
    require(!isSet(transit1.cb.joint.jump, 7), "grounding preemption left second transit joint leased");
    // Check: final destination entries and resource pin leases are released for grounding preemption victims.
    require(!isSet(target0.cb.dst.jump, 31), "grounding preemption left first target dst leased");
    require(!isSet(target0.cb.joint.jump, 6), "grounding preemption left first target joint leased");
    require(!isSet(target0.cb.local.local, 90), "grounding preemption left first target pin local leased");
    require(!isSet(target0.pin_state.leased_nodes, 90), "grounding preemption left first target pin_state leased");
    require(!isSet(target1.cb.dst.jump, 33), "grounding preemption left second target dst leased");
    require(!isSet(target1.cb.joint.jump, 8), "grounding preemption left second target joint leased");
    require(!isSet(target1.cb.local.local, 91), "grounding preemption left second target pin local leased");
    require(!isSet(target1.pin_state.leased_nodes, 91), "grounding preemption left second target pin_state leased");
    // Check: selected source-tree route vectors and tile route references are removed atomically.
    require(route0.empty() && route1.empty(), "grounding preemption route tree did not clear route vectors");
    require(std::all_of(source.routedNets.begin(), source.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left source routedNets reference");
    require(std::all_of(transit0.routedNets.begin(), transit0.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left first transit routedNets reference");
    require(std::all_of(transit1.routedNets.begin(), transit1.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left second transit routedNets reference");
    require(std::all_of(target0.routedNets.begin(), target0.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left first target routedNets reference");
    require(std::all_of(target1.routedNets.begin(), target1.routedNets.end(), [](const Ref<rtl::Net>& ref) { return !ref.peer; }),
        "grounding preemption left second target routedNets reference");
}

void remote_endpoints_require_crossbar_fabric()
{
    auto endpoint = [](fpga::Coord coord, int local) {
        fpga::Wire wire;
        wire.type = fpga::Wire::WIRE_TILE_PIN;
        wire.from = coord;
        wire.to = coord;
        wire.resource = coord;
        wire.local = local;
        return wire;
    };

    std::vector<fpga::Wire> same_tile{endpoint({2, 3}, 4), endpoint({2, 3}, 5)};
    require(fpga::isRouteComplete(same_tile),
        "same-tile endpoint connection was not accepted as a complete local route");

    std::vector<fpga::Wire> remote{endpoint({2, 3}, 4), endpoint({8, 9}, 5)};
    require(!fpga::isRouteComplete(remote),
        "remote endpoint pins were accepted without a crossbar path");

    fpga::Wire transit;
    transit.type = fpga::Wire::WIRE_CROSSBAR;
    transit.from = {2, 3};
    transit.to = {8, 9};
    remote.insert(remote.begin() + 1, transit);
    require(fpga::isRouteComplete(remote),
        "remote endpoint connection with fabric transit was not complete");
}

void attached_resource_tiles_share_the_route_tile_state()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(2, 1);
    fpga::CBType cb_type;
    cb_type.name = "route_matrix_alpha";
    cb_type.type_id = 7;
    cb_type.base_type_id = 7;

    fpga::Tile& resource = *tiles[0];
    fpga::Tile& route = *tiles[1];
    resource.cb_type = &cb_type;
    resource.cb.type = &cb_type;
    resource.cb_coord = route.coord;
    route.cb_type = &cb_type;
    route.cb.type = &cb_type;
    route.cb_coord = route.coord;

    // Check: both resource and route views resolve to one owner for every lease bitmap.
    require(fpga::Device::current().routeTile(resource) == &route,
        "attached resource tile did not resolve to the physical route-state owner");
    require(fpga::Device::current().routeTile(route) == &route,
        "physical route tile did not resolve to itself");
    fpga::Device::current().routeTile(resource)->cb.src.jump |= bit(123);
    require(isSet(route.cb.src.jump, 123),
        "attached resource route lease did not update the canonical route tile state");
    require(!isSet(resource.cb.src.jump, 123),
        "attached resource retained an independent duplicate route lease");
}

void releasing_fanout_suffix_keeps_parent_destination_lease()
{
    fpga::Tile& tile = resetDevice();
    constexpr int trunk_dst = 42;
    constexpr int trunk_src = 73;
    constexpr int branch_src = 91;

    fpga::Wire trunk;
    trunk.from = tile.coord;
    trunk.to = {tile.coord.x + 1, tile.coord.y};
    trunk.local = trunk_dst;
    trunk.jump = trunk_src;
    trunk.pos = 1;

    fpga::Wire branch = trunk;
    branch.jump = branch_src;
    branch.owns_dst = false;

    tile.cb.dst.jump |= bit(trunk_dst);
    tile.cb.src.jump |= bit(trunk_src);
    tile.cb.src.jump |= bit(branch_src);
    std::vector<fpga::Wire> branch_route{branch};
    fpga::releaseRouteFragmentLease(branch_route, 0);

    // Check: branch cleanup frees its exit but cannot free the destination owned by the trunk.
    require(isSet(tile.cb.dst.jump, trunk_dst),
        "fanout suffix cleanup released its parent trunk destination");
    require(!isSet(tile.cb.src.jump, branch_src),
        "fanout suffix cleanup did not release its own outgoing source");
    require(isSet(tile.cb.src.jump, trunk_src),
        "fanout suffix cleanup released the parent trunk source");

    std::vector<fpga::Wire> trunk_route{trunk};
    fpga::releaseRouteFragmentLease(trunk_route, 0);
    require(!isSet(tile.cb.dst.jump, trunk_dst),
        "parent trunk cleanup did not release its owned destination");
    require(!isSet(tile.cb.src.jump, trunk_src),
        "parent trunk cleanup did not release its owned source");
}

void fanout_fork_requires_existing_trunk_destination()
{
    fpga::Tile& tile = resetDevice();
    constexpr int trunk_dst = 42;
    constexpr int branch_src = 91;

    // Check: a branch cannot manufacture a destination lease when no routed
    // trunk owns the destination at its selected branch point.
    require(!pnr::leaseExistingDestinationFork(tile.cb, trunk_dst, branch_src),
        "fanout fork succeeded without an existing trunk destination");
    require(!pnr::routeStartReusesDestination(tile.cb, true, true, trunk_dst),
        "unleased partial-route landing was classified as a shared trunk destination");
    require(tile.cb.dst.jump == NodeMask{} && tile.cb.src.jump == NodeMask{},
        "rejected fanout fork left an ownerless crossbar lease");

    tile.cb.dst.jump |= bit(trunk_dst);
    require(pnr::routeStartReusesDestination(tile.cb, true, true, trunk_dst),
        "leased fanout trunk destination was classified as a new route landing");
    require(pnr::leaseExistingDestinationFork(tile.cb, trunk_dst, branch_src),
        "fanout fork rejected an existing trunk destination");
    require(isSet(tile.cb.dst.jump, trunk_dst) && isSet(tile.cb.src.jump, branch_src),
        "valid fanout fork did not lease only its private exit");

    fpga::Wire branch;
    branch.from = tile.coord;
    branch.to = {tile.coord.x + 1, tile.coord.y};
    branch.local = trunk_dst;
    branch.jump = branch_src;
    branch.pos = 2;
    branch.owns_dst = false;
    std::vector<fpga::Wire> route{branch};
    fpga::releaseRouteFragmentLease(route, 0);

    // Check: branch removal frees its private source and leaves the destination
    // owned by the pre-existing trunk, so no ownerless bit can survive.
    require(isSet(tile.cb.dst.jump, trunk_dst) && !isSet(tile.cb.src.jump, branch_src),
        "fanout fork cleanup corrupted trunk ownership");
}

void fanout_exact_local_reuse_rejects_foreign_owner()
{
    // A sibling route may share its net's exact terminal local.
    require(pnr::fanoutMayReuseExactLocal(true, false),
        "fanout rejected an exact local owned only by its own route tree");

    // A protected static route or another ordinary net blocks that local.
    require(!pnr::fanoutMayReuseExactLocal(true, true),
        "fanout reused an exact local owned by another electrical net");
    require(!pnr::fanoutMayReuseExactLocal(false, false),
        "fanout reused a local that does not map to its destination pin");
}

void unrouting_binding_preserves_foreign_local_owner()
{
    fpga::Tile& tile = resetDevice();
    rtl::Inst ordinary_owner;
    rtl::Inst protected_owner;
    rtl::Inst ordinary_driver;
    rtl::Inst protected_driver;
    constexpr int local = 83;

    fpga::Wire ordinary_pin;
    ordinary_pin.type = fpga::Wire::WIRE_TILE_PIN;
    ordinary_pin.from = tile.coord;
    ordinary_pin.to = tile.coord;
    ordinary_pin.local = local;
    fpga::Wire protected_pin = ordinary_pin;
    ordinary_owner.wires.push_back({ordinary_pin});
    protected_owner.wires.push_back({protected_pin});

    Referable<rtl::Net> ordinary_net;
    Referable<rtl::Net> protected_net;
    protected_net.route_protected = true;
    fpga::attachNetRoute(ordinary_net, ordinary_owner, 0, &ordinary_driver,
                         &ordinary_owner, "O", "I", "ordinary");
    fpga::attachNetRoute(protected_net, protected_owner, 0, &protected_driver,
                         &protected_owner, "O", "I", "static");
    fpga::registerNetRouteTiles(ordinary_net, ordinary_owner.wires[0]);
    fpga::registerNetRouteTiles(protected_net, protected_owner.wires[0]);
    tile.cb.local.local |= bit(local);
    tile.pin_state.leased_nodes |= bit(local);

    require(fpga::unrouteNetRoute(ordinary_net, 0),
        "ordinary binding sharing a protected endpoint was not removed");
    // Check: removing one electrical net cannot clear a local still owned by
    // the protected route represented in the tile's live-owner index.
    require(isSet(tile.cb.local.local, local)
            && isSet(tile.pin_state.leased_nodes, local),
        "ordinary binding removal cleared a protected endpoint lease");
    require(fpga::findNetByNode(tile, fpga::CB_NODE_LOCAL, local, false)
            == &protected_net,
        "protected endpoint disappeared from the live-owner index");

    require(fpga::unrouteNetRoute(protected_net, 0),
        "protected binding was not removable by its owning router");
    require(!isSet(tile.cb.local.local, local)
            && !isSet(tile.pin_state.leased_nodes, local),
        "last endpoint owner removal retained a stale lease");
}

void protected_preemption_truncates_only_blocked_suffix()
{
    fpga::Tile& tile = resetDevice();
    rtl::Inst owner;
    rtl::Inst driver;
    Referable<rtl::Net> net;
    constexpr int prefix_joint = 17;
    constexpr int blocked_joint = 58;
    constexpr int terminal_local = 43;

    fpga::Wire prefix;
    prefix.type = fpga::Wire::WIRE_CROSSBAR;
    prefix.from = tile.coord;
    prefix.to = tile.coord;
    prefix.pos = 1;
    prefix.local = 5;
    prefix.joint = prefix_joint;
    fpga::Wire blocked = prefix;
    blocked.local = 6;
    blocked.joint = blocked_joint;
    fpga::Wire pin;
    pin.type = fpga::Wire::WIRE_TILE_PIN;
    pin.from = tile.coord;
    pin.to = tile.coord;
    pin.local = terminal_local;
    owner.wires.push_back({prefix, blocked, pin});
    fpga::attachNetRoute(net, owner, 0, &driver, &owner, "O", "I",
                         "ordinary");
    fpga::registerNetRouteTiles(net, owner.wires[0]);
    tile.cb.joint.jump |= bit(prefix_joint) | bit(blocked_joint);
    tile.cb.local.local |= bit(terminal_local);
    tile.pin_state.leased_nodes |= bit(terminal_local);

    require(fpga::unrouteNetRouteFromNode(
                net, 0, tile.coord, fpga::CB_NODE_JOINT, blocked_joint),
        "protected-node preemption did not truncate the conflicting suffix");
    // Check: the committed route before the blocked node remains available as
    // the continuation point for the requeued Generic task.
    require(owner.wires[0].size() == 1
            && isSet(tile.cb.joint.jump, prefix_joint),
        "protected-node preemption discarded or released the valid prefix");
    // Check: every lease from the blocked node through the endpoint is freed.
    require(!isSet(tile.cb.joint.jump, blocked_joint)
            && !isSet(tile.cb.local.local, terminal_local)
            && !isSet(tile.pin_state.leased_nodes, terminal_local),
        "protected-node preemption retained a blocked suffix lease");
}

void equal_route_names_keep_distinct_endpoint_bindings()
{
    rtl::Net net;
    rtl::Inst driver;
    rtl::Inst sink0;
    rtl::Inst sink1;
    rtl::Inst owner0;
    rtl::Inst owner1;

    fpga::attachNetRoute(net, owner0, 3, &driver, &sink0, "OUT", "IN0", "shared_text_name");
    fpga::attachNetRoute(net, owner1, 7, &driver, &sink1, "OUT", "IN1", "shared_text_name");

    // Check: a textual name is annotation, while endpoint identity owns each physical route binding.
    require(net.routes.size() == 2,
        "equal route names collapsed distinct source-to-sink route bindings");
    require(net.routes[0].to == &sink0 && net.routes[0].owner == &owner0 && net.routes[0].route_index == 3,
        "second endpoint binding overwrote the first route");
    require(net.routes[1].to == &sink1 && net.routes[1].owner == &owner1 && net.routes[1].route_index == 7,
        "second endpoint binding was not preserved independently");

    fpga::attachNetRoute(net, owner1, 9, &driver, &sink1, "OUT", "IN1", "shared_text_name");
    require(net.routes.size() == 2 && net.routes[1].route_index == 9,
        "reattaching the same exact endpoint created a duplicate binding");
}

void duplicate_endpoint_identity_updates_exact_physical_binding()
{
    rtl::Net net;
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst complete_owner;
    rtl::Inst incomplete_owner;

    complete_owner.wires.emplace_back();
    fpga::Wire complete_crossbar;
    complete_crossbar.type = fpga::Wire::WIRE_CROSSBAR;
    complete_owner.wires[0].push_back(complete_crossbar);
    fpga::Wire complete_pin;
    complete_pin.type = fpga::Wire::WIRE_TILE_PIN;
    complete_owner.wires[0].push_back(complete_pin);
    incomplete_owner.wires.emplace_back();
    incomplete_owner.wires[0].push_back(complete_crossbar);

    fpga::attachNetRoute(net, complete_owner, 0,
        &driver, &sink, "OUT", "IN", "same_identity");

    fpga::attachNetRoute(net, incomplete_owner, 0,
        &driver, &sink, "OUT", "IN", "same_identity");

    // Check: one physical endpoint identity has one binding. A later stale
    // attachment cannot create a second task or replace its completed route.
    require(net.routes.size() == 1
            && net.routes[0].owner == &complete_owner
            && net.routes[0].route_index == 0,
        "duplicate endpoint identity created a second physical binding");
}

void moving_sink_detaches_destination_but_keeps_unique_source_prefix()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(3, 1);
    fpga::Tile& source_tile = *tiles[0];
    fpga::Tile& middle_tile = *tiles[1];
    fpga::Tile& sink_tile = *tiles[2];
    Referable<rtl::Net> net;
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.emplace_back();
    std::vector<fpga::Wire>& route = owner.wires.back();

    fpga::Wire source_pin;
    source_pin.type = fpga::Wire::WIRE_TILE_PIN;
    source_pin.from = source_tile.coord;
    source_pin.to = source_tile.coord;
    source_pin.local = 10;
    route.push_back(source_pin);

    fpga::Wire jump;
    jump.type = fpga::Wire::WIRE_CROSSBAR;
    jump.from = source_tile.coord;
    jump.to = middle_tile.coord;
    jump.local = 10;
    jump.jump = 20;
    jump.dst = 30;
    jump.pos = 0;
    route.push_back(jump);

    fpga::Wire tail_jump;
    tail_jump.type = fpga::Wire::WIRE_CROSSBAR;
    tail_jump.from = middle_tile.coord;
    tail_jump.to = sink_tile.coord;
    tail_jump.local = 30;
    tail_jump.jump = 21;
    tail_jump.dst = 31;
    tail_jump.pos = 1;
    tail_jump.owns_dst = true;
    route.push_back(tail_jump);

    fpga::Wire terminal;
    terminal.type = fpga::Wire::WIRE_CROSSBAR;
    terminal.from = sink_tile.coord;
    terminal.to = sink_tile.coord;
    terminal.local = 31;
    terminal.joint = 40;
    terminal.pos = 1;
    terminal.owns_dst = true;
    route.push_back(terminal);

    fpga::Wire sink_pin;
    sink_pin.type = fpga::Wire::WIRE_TILE_PIN;
    sink_pin.from = sink_tile.coord;
    sink_pin.to = sink_tile.coord;
    sink_pin.local = 50;
    route.push_back(sink_pin);

    source_tile.cb.local.local |= bit(10);
    source_tile.cb.src.jump |= bit(20);
    middle_tile.cb.dst.jump |= bit(30);
    middle_tile.cb.src.jump |= bit(21);
    sink_tile.cb.dst.jump |= bit(31);
    sink_tile.cb.joint.jump |= bit(40);
    sink_tile.cb.local.local |= bit(50);
    sink_tile.pin_state.leased_nodes |= bit(50);
    fpga::attachNetRoute(net, owner, 0, &driver, &sink, "OUT", "IN", "route");

    require(fpga::detachNetRouteDestination(net, 0),
        "moving sink destination detach returned false");

    // Check: a private route has no reusable shared trunk, so relocation
    // removes the complete old route instead of guessing a takeoff boundary.
    require(route.empty(),
        "moving sink cleanup retained a private route without a live sibling");
    require(fpga::findNetByNode(middle_tile, fpga::CB_NODE_DST, 30) == nullptr,
        "moving sink cleanup retained a landing lease without a route owner");
    // Check: every lease in the old private path is released.
    require(!isSet(source_tile.cb.local.local, 10)
            && !isSet(source_tile.cb.src.jump, 20)
            && !isSet(middle_tile.cb.dst.jump, 30)
            && !isSet(middle_tile.cb.src.jump, 21)
            && !isSet(sink_tile.cb.dst.jump, 31) && !isSet(sink_tile.cb.joint.jump, 40)
            && !isSet(sink_tile.cb.local.local, 50) && !isSet(sink_tile.pin_state.leased_nodes, 50),
        "moving sink cleanup retained an obsolete destination lease");

    tail_jump.owns_dst = false;
    route.push_back(tail_jump);
    middle_tile.cb.src.jump |= bit(21);
    std::vector<fpga::Wire> committed_route = route;

    // Check: bounded Moving Generic failure is not permission to remove a
    // committed hop; placement relocation handles genuine stagnation.
    require(pnr::failedMovingGenericKeepsPrefix(true, false)
            && !pnr::failedMovingGenericKeepsPrefix(true, true)
            && !pnr::failedMovingGenericKeepsPrefix(false, false),
        "Moving Generic failed-prefix preservation policy selected the wrong mode");
    require(route.size() == committed_route.size()
            && std::equal(route.begin(), route.end(), committed_route.begin(),
                [](const fpga::Wire& left, const fpga::Wire& right) {
                    return left.type == right.type
                        && left.from.x == right.from.x && left.from.y == right.from.y
                        && left.to.x == right.to.x && left.to.y == right.to.y
                        && left.jump == right.jump
                        && left.dst == right.dst;
                })
            && isSet(middle_tile.cb.src.jump, 21),
        "failed Moving Generic continuation changed its committed prefix or lease");
}

void moving_fanout_sink_keeps_only_shared_trunk()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(3, 1);
    fpga::Tile& source_tile = *tiles[0];
    fpga::Tile& branch_tile = *tiles[1];
    fpga::Tile& sink_tile = *tiles[2];
    Referable<rtl::Net> net;
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.emplace_back();
    std::vector<fpga::Wire>& route = owner.wires.back();

    fpga::Wire source_pin;
    source_pin.type = fpga::Wire::WIRE_TILE_PIN;
    source_pin.from = source_tile.coord;
    source_pin.to = source_tile.coord;
    source_pin.local = 11;
    source_pin.shared = true;
    route.push_back(source_pin);

    fpga::Wire trunk;
    trunk.type = fpga::Wire::WIRE_CROSSBAR;
    trunk.from = source_tile.coord;
    trunk.to = branch_tile.coord;
    trunk.local = 11;
    trunk.jump = 22;
    trunk.dst = 32;
    trunk.shared = true;
    route.push_back(trunk);

    fpga::Wire branch;
    branch.type = fpga::Wire::WIRE_CROSSBAR;
    branch.from = branch_tile.coord;
    branch.to = sink_tile.coord;
    branch.local = 32;
    branch.jump = 23;
    branch.dst = 33;
    route.push_back(branch);

    fpga::Wire arrival;
    arrival.type = fpga::Wire::WIRE_CROSSBAR;
    arrival.from = sink_tile.coord;
    arrival.to = sink_tile.coord;
    arrival.local = 33;
    arrival.pos = 1;
    route.push_back(arrival);

    source_tile.cb.local.local |= bit(11);
    source_tile.cb.src.jump |= bit(22);
    branch_tile.cb.dst.jump |= bit(32);
    branch_tile.cb.src.jump |= bit(23);
    sink_tile.cb.dst.jump |= bit(33);
    fpga::attachNetRoute(net, owner, 0, &driver, &sink, "OUT", "IN", "fanout");

    rtl::Inst sibling_sink;
    rtl::Inst sibling_owner;
    sibling_owner.wires.emplace_back();
    fpga::Wire sibling_source = source_pin;
    sibling_source.shared = false;
    fpga::Wire sibling_trunk = trunk;
    sibling_trunk.shared = false;
    sibling_owner.wires[0].push_back(sibling_source);
    sibling_owner.wires[0].push_back(sibling_trunk);
    fpga::attachNetRoute(net, sibling_owner, 0, &driver, &sibling_sink,
        "OUT", "IN2", "sibling");

    require(fpga::detachNetRouteDestination(net, 0),
        "moving fanout destination detach returned false");

    // Check: the prefix proven by a live sibling remains, while only this
    // branch's private leases are released.
    require(route.size() == 2 && route[0].shared && route[1].shared && route.back().jump == 22,
        "moving fanout cleanup did not stop at the shared trunk boundary");
    require(isSet(source_tile.cb.local.local, 11) && isSet(source_tile.cb.src.jump, 22)
            && isSet(branch_tile.cb.dst.jump, 32),
        "moving fanout cleanup released shared trunk leases");
    require(!isSet(branch_tile.cb.src.jump, 23) && !isSet(sink_tile.cb.dst.jump, 33),
        "moving fanout cleanup retained its obsolete private suffix");
}

void moved_sink_binding_is_always_invalidated()
{
    Referable<rtl::Net> net;
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.emplace_back();
    fpga::attachNetRoute(net, owner, 0, &driver, &sink, "OUT", "IN", "empty");

    // Check: moving an endpoint with an empty route still creates pending work.
    require(fpga::invalidateMovedSinkRoute(net, 0),
        "empty moved-sink binding was incorrectly treated as already valid");

    fpga::Wire shared;
    shared.type = fpga::Wire::WIRE_CROSSBAR;
    shared.shared = true;
    shared.jump = 9;
    owner.wires[0].push_back(shared);
    fpga::Wire sink_pin;
    sink_pin.type = fpga::Wire::WIRE_TILE_PIN;
    owner.wires[0].push_back(sink_pin);

    require(fpga::invalidateMovedSinkRoute(net, 0),
        "shared moved-sink binding was not invalidated");
    // Check: a shared flag without a live sibling is stale metadata, so the
    // obsolete route is removed completely.
    require(owner.wires[0].empty(),
        "moved-sink invalidation retained an unowned shared fragment");
}

void shared_terminal_fanout_releases_only_its_local()
{
    std::vector<fpga::Tile*> tiles = resetDeviceGrid(1, 1);
    fpga::Tile& tile = *tiles[0];
    Referable<rtl::Net> net;
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.emplace_back();
    std::vector<fpga::Wire>& route = owner.wires.back();

    fpga::Wire shared_terminal;
    shared_terminal.type = fpga::Wire::WIRE_CROSSBAR;
    shared_terminal.from = tile.coord;
    shared_terminal.to = tile.coord;
    shared_terminal.local = 31;
    shared_terminal.joint = 40;
    shared_terminal.joint2 = 41;
    shared_terminal.pos = 1;
    shared_terminal.shared = true;
    shared_terminal.owns_dst = false;
    route.push_back(shared_terminal);

    fpga::Wire branch_pin;
    branch_pin.type = fpga::Wire::WIRE_TILE_PIN;
    branch_pin.from = tile.coord;
    branch_pin.to = tile.coord;
    branch_pin.local = 51;
    route.push_back(branch_pin);

    tile.cb.dst.jump |= bit(31);
    tile.cb.joint.jump |= bit(40) | bit(41);
    tile.cb.local.local |= bit(50) | bit(51);
    tile.pin_state.leased_nodes |= bit(50) | bit(51);
    fpga::attachNetRoute(net, owner, 0, &driver, &sink, "OUT", "IN", "shared-terminal");

    require(fpga::unrouteNetBranch(net, 0),
        "shared-terminal fanout branch could not be removed");

    // Check: removing the private sink local does not release its shared dst/joints.
    require(isSet(tile.cb.dst.jump, 31) && isSet(tile.cb.joint.jump, 40)
            && isSet(tile.cb.joint.jump, 41),
        "shared-terminal fanout released a trunk lease");
    // Check: only the branch's local and pin leases are released.
    require(isSet(tile.cb.local.local, 50) && isSet(tile.pin_state.leased_nodes, 50)
            && !isSet(tile.cb.local.local, 51) && !isSet(tile.pin_state.leased_nodes, 51),
        "shared-terminal fanout did not release only its private local");
}

void failed_generic_seed_rotates_to_another_sink()
{
    struct SeedTask
    {
        void* from = nullptr;
        int to = 0;
        std::string from_port;
        size_t attempt = 0;
        bool fanout = false;
    };

    int source = 0;
    SeedTask current{&source, 1, "OUT", 2, false};
    std::vector<SeedTask> deferred{
        SeedTask{&source, 2, "OUT", 0, true},
        SeedTask{&source, 3, "OTHER", 0, true},
    };
    std::vector<SeedTask> pending;
    std::vector<SeedTask> returned;

    // Check: the full routing loop rotates only an empty, failed Generic seed;
    // partial progress, completion, and later routing modes retain their task.
    require(pnr::shouldRotateFailedGenericSeed(true, false, false, true),
        "full Generic loop did not recognize an exhausted empty seed");
    require(!pnr::shouldRotateFailedGenericSeed(true, false, true, true)
            && !pnr::shouldRotateFailedGenericSeed(true, false, false, false)
            && !pnr::shouldRotateFailedGenericSeed(false, false, false, true),
        "Generic seed rotation accepted progress, a partial route, or another stage");

    bool rotated = pnr::rotateFailedGenericSeedTask(current, deferred, pending,
        [](const SeedTask& left, const SeedTask& right) {
            return left.from == right.from && left.to == right.to
                && left.from_port == right.from_port;
        },
        [&](SeedTask failed) {
            returned.push_back(std::move(failed));
        });

    // Check: Generic selects another sink from the same physical source port.
    require(rotated && current.to == 2 && !current.fanout && current.attempt == 0,
        "failed Generic seed did not rotate to its deferred sibling");
    // Check: the failed sink is deferred for Fanout and unrelated sources stay queued.
    require(returned.size() == 1 && returned[0].to == 1 && returned[0].fanout
            && deferred.size() == 1 && deferred[0].from_port == "OTHER",
        "Generic seed rotation lost or consumed the wrong deferred task");

    SeedTask early{&source, 4, "OUT", 1, false};
    require(!pnr::rotateFailedGenericSeedTask(early, deferred, pending,
                [](const SeedTask& left, const SeedTask& right) {
                    return left.to == right.to;
                },
                [&](SeedTask) {}),
        "Generic seed rotated before reaching the failed-attempt threshold");
}

void restored_moving_work_does_not_preempt_until_focused()
{
    // Check: ordinary routing and a focused relocation may preempt congestion.
    require(pnr::movingRouteMayPreempt(false, false),
        "Generic/Fanout routing unexpectedly disabled preemption");
    require(pnr::movingRouteMayPreempt(true, true),
        "focused Moving reroute unexpectedly disabled preemption");

    // Check: restored sibling work cannot invalidate another route tree.
    require(!pnr::movingRouteMayPreempt(true, false),
        "unfocused Moving work was allowed to preempt sibling routes");
}

void fanout_continuations_keep_grounding_preemption_enabled()
{
    // Check: both the initial Fanout branch and every committed continuation
    // may ground by preempting transit while ordinary unfocused Moving may not.
    require(pnr::movingRouteMayPreempt(false, false),
        "Fanout continuation lost grounding preemption after committing a partial suffix");
    require(!pnr::movingRouteMayPreempt(true, false),
        "unfocused Moving continuation unexpectedly enabled grounding preemption");
    require(pnr::movingRouteMayPreempt(true, true),
        "focused Moving continuation did not restore grounding preemption");
}

void moving_rehomes_complete_generated_endpoint_chain()
{
    struct Endpoint
    {
        int id = 0;
        bool generated = false;
        std::vector<std::pair<Endpoint*, bool>> neighbors;
    };

    Endpoint real{1, false, {}};
    Endpoint pass_a{2, true, {}};
    Endpoint pass_b{3, true, {}};
    Endpoint unrelated{4, true, {}};
    real.neighbors = {{&pass_a, true}, {&unrelated, false}};
    pass_a.neighbors = {{&real, true}, {&pass_b, true}};
    pass_b.neighbors = {{&pass_a, true}};

    std::vector<Endpoint*> cluster{&real};
    pnr::appendMovingEndpointChain(cluster, [](Endpoint* endpoint) {
        std::vector<Endpoint*> generated;
        for (auto [neighbor, void_net] : endpoint->neighbors) {
            if (void_net && neighbor && neighbor->generated) {
                generated.push_back(neighbor);
            }
        }
        return generated;
    });

    // Check: Moving includes the whole generated chain exactly once so no
    // physical endpoint remains placed at the cell's previous tile.
    require(cluster.size() == 3
            && std::count(cluster.begin(), cluster.end(), &pass_a) == 1
            && std::count(cluster.begin(), cluster.end(), &pass_b) == 1,
        "Moving did not include the complete generated endpoint chain");
    // Check: generated endpoints reached through an external routed net remain fixed.
    require(std::find(cluster.begin(), cluster.end(), &unrelated) == cluster.end(),
        "Moving absorbed an unrelated generated endpoint");

    std::vector<Endpoint*> reversed{&pass_b, &pass_a};
    std::set<int> placed{real.id};
    bool resolved = pnr::resolveMovingEndpointDependencies(reversed, [&](Endpoint* endpoint) {
        int required_driver = endpoint == &pass_a ? real.id : pass_a.id;
        if (!placed.contains(required_driver)) {
            return false;
        }
        placed.insert(endpoint->id);
        return true;
    });
    // Check: endpoint placement is dependency-driven, so reverse discovery
    // order still places the inner source endpoint before the outer endpoint.
    require(resolved && placed.contains(pass_a.id) && placed.contains(pass_b.id),
        "Moving failed to resolve a reversed generated endpoint chain");

    placed = {real.id};
    resolved = pnr::resolveMovingEndpointDependencies(reversed, [&](Endpoint* endpoint) {
        int required_driver = endpoint == &pass_a ? real.id : pass_a.id;
        if (!placed.contains(required_driver)) {
            return false;
        }
        placed.insert(endpoint->id);
        return true;
    });
    // Check: restoration rebuilds stale generated placement from the live
    // anchor chain instead of relying on the endpoints' former coordinates.
    require(resolved && placed.size() == 3,
        "Moving could not rebuild a stale generated endpoint chain from its anchor");
}

}

int main()
{
    try {
        packed_void_state_is_per_net_designator();
        packed_void_connection_unroutes_only_its_binding();
        packed_void_generic_connection_transfers_shared_prefix_ownership();
        deadend_masks_are_basic_stage_only();
        failed_edge_persistence_is_basic_stage_only();
        failed_edge_returns_to_parent_in_every_stage();
        fanout_branch_selection_skips_saturated_points();
        current_target_entry_failure_is_not_retried();
        failed_near_target_docking_keeps_normal_suffix_depth();
        joint_mediated_src_nodes_are_indexed();
        can_in_rejects_unconnected_double_joint_paths();
        loaded_crossbar_local_and_joint_masks_use_router_bit_numbering();
        tile_type_mapping_models_all_16_ff_input_pins_per_clb_tile();
        connected_mux_inputs_are_void_when_packed_by_element_rules();
        mux_selector_remains_routable_when_driver_shares_tile();
        routing_mode_generic_routes_only_one_net_from_single_source_port();
        routing_mode_fanout_branches_away_from_source_tile();
        routing_mode_moving_unroutes_old_cell_tree_and_reroutes_hierarchy();
        limited_iterations_find_one_tile_escape_path_behind_source();
        limited_continuation_is_strictly_incremental_without_rollbacks();
        unroute_net_clears_multifragment_route_state();
        grounding_preemption_route_tree_unroute_frees_terminal_masks();
        remote_endpoints_require_crossbar_fabric();
        attached_resource_tiles_share_the_route_tile_state();
        releasing_fanout_suffix_keeps_parent_destination_lease();
        fanout_fork_requires_existing_trunk_destination();
        fanout_exact_local_reuse_rejects_foreign_owner();
        unrouting_binding_preserves_foreign_local_owner();
        protected_preemption_truncates_only_blocked_suffix();
        equal_route_names_keep_distinct_endpoint_bindings();
        duplicate_endpoint_identity_updates_exact_physical_binding();
        moving_sink_detaches_destination_but_keeps_unique_source_prefix();
        moving_fanout_sink_keeps_only_shared_trunk();
        moved_sink_binding_is_always_invalidated();
        shared_terminal_fanout_releases_only_its_local();
        failed_generic_seed_rotates_to_another_sink();
        restored_moving_work_does_not_preempt_until_focused();
        fanout_continuations_keep_grounding_preemption_enabled();
        moving_rehomes_complete_generated_endpoint_chain();
        releasing_route_fragment_keeps_deadend_for_same_src();
        releasing_route_fragment_clears_source_and_transit_node_classes();
        preempted_transit_unroute_keeps_deadend_for_same_src();
        preemption_cycle_guards_expire_after_each_route_pass();
        preemption_candidate_iteration_includes_busy_transit_exits();
        preempted_fanout_siblings_remain_deferred_during_basic_routing();
        failed_fanout_branch_advances_rotation_once();
        for (unsigned seed = 1; seed <= 64; ++seed) {
            local_and_transit_preemption(seed);
            joint_metadata_preemption(seed + 1000);
            free_joint_exit_is_preferred(seed + 2000);
        }
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "routing_test failed: %s\n", failure.message.c_str());
        return EXIT_FAILURE;
    }
    catch (const std::exception& ex) {
        std::fprintf(stderr, "routing_test exception: %s\n", ex.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
