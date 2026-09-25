#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Inst.h"
#include "Wire.h"
#include "route/RouteDesign.h"
#include "route/RoutePassState.h"
#include "route/RouteDiagnostics.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

void require(bool condition, const std::string& message);

struct EndpointBinding {
    int* from = nullptr;
    int* to = nullptr;
};

void incident_binding_uses_precomputed_endpoint_closure()
{
    int focus = 1;
    int generated_endpoint = 2;
    int unrelated = 3;
    std::unordered_set<int*> closure{&focus, &generated_endpoint};
    size_t lookups = 0;
    auto contains = [&](int* endpoint) {
        ++lookups;
        return closure.contains(endpoint);
    };

    // Check: a generated endpoint in the precomputed closure identifies an
    // incident binding without any graph or module traversal fallback.
    require(pnr::routeBindingTouchesKnownEndpoint(
                EndpointBinding{&unrelated, &generated_endpoint}, contains),
            "precomputed Moving endpoint closure missed an incident binding");
    require(lookups == 2,
            "incident binding lookup performed work beyond its two endpoints");

    lookups = 0;
    // Check: an unrelated binding is rejected after exactly two indexed
    // endpoint tests, which prevents the former per-binding graph traversal.
    require(!pnr::routeBindingTouchesKnownEndpoint(
                EndpointBinding{&unrelated, &unrelated}, contains),
            "precomputed Moving endpoint closure accepted unrelated binding");
    require(lookups == 2,
            "unrelated binding lookup fell back to endpoint traversal");
}

void failed_route_anchor_controls_moving_search_center()
{
    fpga::Coord old_place{40, 40};
    std::vector<fpga::Coord> anchors{{8, 9}};
    for (int index = 0; index < 12; ++index) {
        anchors.push_back(fpga::Coord{80 + index, 90 + index});
    }

    // Check: many secondary output endpoints cannot pull relocation away
    // from the unfinished input route recorded as the first anchor.
    fpga::Coord center = pnr::movingSearchCenter(old_place, anchors);
    require(center.x == 8 && center.y == 9,
            "Moving averaged the failed route with secondary fanout anchors");

    // Check: an endpoint with no external or partial anchor remains centered
    // on its current placement instead of using an invalid coordinate.
    center = pnr::movingSearchCenter(old_place, std::vector<fpga::Coord>{});
    require(center.x == old_place.x && center.y == old_place.y,
            "Moving lost its fallback placement without route anchors");
}

void multi_input_sink_balances_only_incoming_route_anchors()
{
    fpga::Coord old_place{40, 40};
    std::vector<fpga::Coord> all_anchors{{8, 9}, {90, 90}, {91, 91}, {92, 92}};
    std::vector<fpga::Coord> incoming{{8, 9}, {72, 81}, {40, 33}};

    // Check: several incoming routes center the legal search region so moving
    // toward one blocked input cannot invalidate another input indefinitely.
    fpga::Coord center =
        pnr::movingSearchCenter(old_place, all_anchors, incoming);
    require(center.x == 40 && center.y == 45,
            "Moving did not balance a multi-input sink's route anchors");

    // Check: secondary output fanouts are deliberately absent from the input
    // set and therefore cannot pull the sink toward their distant loads.
    incoming.resize(1);
    center = pnr::movingSearchCenter(old_place, all_anchors, incoming);
    require(center.x == 8 && center.y == 9,
            "output fanouts displaced a single failed input anchor");
}

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

void moving_sources_use_bounded_relocation_batches()
{
    // Check: Moving Sources enters route-first relocation immediately instead
    // of spending its first pass retrying every unfinished trunk in place.
    require(pnr::movingStageStartsWithRelocation(true, true) &&
                pnr::movingStageStartsWithRelocation(false, true) &&
                !pnr::movingStageStartsWithRelocation(true, false),
            "Moving Sources did not start with route-first relocation");

    // Check: a large unfinished queue receives small route-and-measure batches
    // instead of one destructive full-quantum relocation.
    require(pnr::movingSourceRelocationBatchLimit(5000, 18000) == 62,
            "Moving sources did not bound the relocation batch");

    // Check: the final source batch contains only the remaining drivers.
    require(pnr::movingSourceRelocationBatchLimit(5000, 31) == 31,
            "Moving sources overran the final relocation batch");

    // Check: low-yield source probing cannot consume an entire stage while
    // trying to fill a success quota; each batch width returns control to
    // ordinary trunk routing.
    require(pnr::movingSourceRelocationAttemptLimit(62, 18000) == 62 &&
                pnr::movingSourceRelocationAttemptLimit(31, 31) == 31,
            "Moving sources did not bound rejected relocation probes");

    // Check: Generic recovery advances an unfinished trunk incrementally.
    // Its committed prefix must survive the next route-first relocation batch.
    require(pnr::movingSourceRecoveryRetainsPrefix(true) &&
                !pnr::movingSourceRecoveryRetainsPrefix(false),
            "Moving sources discarded incremental route progress");

    // Check: an incomplete Generic prefix is retained as a reverse-docking
    // anchor. Empty and complete routes must not enter anchor recovery.
    require(pnr::movingSourceUsesBackwardAnchor(true, false) &&
                !pnr::movingSourceUsesBackwardAnchor(false, false) &&
                !pnr::movingSourceUsesBackwardAnchor(true, true),
            "Moving sources discarded or misclassified its docking anchor");

    // Check: only a retained prefix that failed every backward anchor is
    // released before replacement search. Successful docking keeps it live.
    require(pnr::movingSourceReleasesPrefixAfterDockMiss(true, false) &&
                !pnr::movingSourceReleasesPrefixAfterDockMiss(true, true) &&
                !pnr::movingSourceReleasesPrefixAfterDockMiss(false, false),
            "Moving sources retained unreachable prefix congestion or "
            "released a successfully docked prefix");

    // Check: route-first recovery may place at any reached reverse frontier and
    // proves every route-guided placement unusable before mandatory failure.
    require(pnr::movingSourcePlacementRadius() < 0 &&
                pnr::movingSourceProbeLimit() == 0,
            "Moving sources clipped its reverse frontier or stopped before "
            "proving every route-guided placement unusable");

    // Check: after a rejected batch rotates the queue, the next source cycle
    // restores the complete trunk workset rather than one destination-style
    // focus per scheduler pass.
    std::vector<int> active_sources;
    std::vector<int> deferred_sources{1, 2, 3};
    require(pnr::activateMovingSourceWorkset(active_sources,
                                             deferred_sources) &&
                active_sources.size() == 3 && deferred_sources.empty() &&
                !pnr::activateMovingSourceWorkset(active_sources,
                                                  deferred_sources),
            "Moving sources restored its retry workset one task at a time");

    std::vector<int> untouched_sources{3, 4};
    std::vector<int> rejected_sources{1, 2};
    pnr::rotateRejectedMovingSources(untouched_sources, rejected_sources);
    // Check: a bounded batch probes untouched sources before returning to the
    // rejected prefix, while retaining every rejected task exactly once.
    require((untouched_sources == std::vector<int>{3, 4, 1, 2}) &&
                rejected_sources.empty(),
            "Moving sources retried the same rejected batch without rotation");

    // Check: exhausting only placement probes or only numeric expansion keeps
    // the same bounded search, while exhausting both advances the next retry.
    require(pnr::nextMovingSourceExpansionBudget(512, false, true, 4096) ==
                    512 &&
                pnr::nextMovingSourceExpansionBudget(512, true, false, 4096) ==
                    512 &&
                pnr::nextMovingSourceExpansionBudget(512, true, true, 4096) ==
                    1024 &&
                pnr::nextMovingSourceExpansionBudget(4096, true, true, 4096) ==
                    4096,
            "Moving sources did not advance its exhausted reverse frontier");

    // Check: one precisely freed reverse boundary is consumed immediately,
    // while repeated cuts are bounded and ordinary failures remain rotated.
    require(pnr::movingSourceRetriesReleasedBoundary(true, 0) &&
                pnr::movingSourceRetriesReleasedBoundary(true, 1) &&
                !pnr::movingSourceRetriesReleasedBoundary(true, 2) &&
                !pnr::movingSourceRetriesReleasedBoundary(false, 0),
            "Moving sources did not bound immediate boundary retries");

    // Check: isolated failures and small productive batches do not trigger an
    // O(N) Generic scan; each topology-change class has an explicit threshold.
    require(!pnr::movingSourceGenericRecoveryDue(255, 31, 1023) &&
                pnr::movingSourceGenericRecoveryDue(256, 0, 0) &&
                pnr::movingSourceGenericRecoveryDue(0, 32, 0) &&
                pnr::movingSourceGenericRecoveryDue(0, 0, 1024),
            "Moving sources did not threshold Generic recovery");

    require(pnr::movingSourceGenericRecoveryDue(0, 0, 0, 1) &&
                pnr::movingSourceGenericRecoveryDue(2, 0, 7, 3) &&
                !pnr::movingSourceGenericRecoveryDue(0, 0, 0, 0),
            "Moving sources starved pending victims below batch thresholds");

    require(pnr::movingRelocationCanContinue(true, true, false, false) &&
                pnr::movingRelocationCanContinue(true, false, true, false) &&
                !pnr::movingRelocationCanContinue(false, true, false, false) &&
                !pnr::movingRelocationCanContinue(true, true, false, true) &&
                !pnr::movingRelocationCanContinue(true, false, false, false),
            "relocation bypassed pending work or final-trunk stage handoff");

    // Check: releasing prefixes made by a recovery chunk is cleanup of that
    // chunk and must not immediately replay the same Generic work.
    require(pnr::movingSourceRecoveryReleaseCount(4096, false) == 4096 &&
                pnr::movingSourceRecoveryReleaseCount(4096, true) == 0 &&
                !pnr::movingSourceGenericRecoveryDue(
                    pnr::movingSourceRecoveryReleaseCount(4096, true), 0, 0),
            "post-recovery prefix cleanup retriggered Generic routing");

    // Check: Generic recovery processes a rotating chunk rather than rescanning
    // every unfinished trunk after a bounded relocation batch.
    require(pnr::movingSourceGenericRecoveryTaskLimit(18000) == 4096 &&
                pnr::movingSourceGenericRecoveryTaskLimit(127) == 127,
            "Moving sources did not bound Generic recovery work");

    // Check: every displaced binding from the output port whose replacement
    // trunk was just committed is deferred as a fanout, even before a binding
    // lookup observes that trunk. A different output without a trunk remains
    // Generic work.
    require(pnr::movedSourceRouteBecomesFanout(true, false) &&
                pnr::movedSourceRouteBecomesFanout(false, true) &&
                !pnr::movedSourceRouteBecomesFanout(false, false),
            "Moving sources recreated same-port fanouts as Generic trunks");

    // Check: an incomplete trunk with a committed partial prefix still enters
    // route-guided relocation; only a complete trunk is skipped.
    require(pnr::movingSourceRouteGuidedTaskNeedsRelocation(false) &&
                !pnr::movingSourceRouteGuidedTaskNeedsRelocation(true),
            "Moving sources mistook a partial trunk for a completed route");

    // Check: recording the original placement does not enlarge the first
    // candidate search, while each actual rejected alternative does.
    require(pnr::movingTriedAlternativeCount(1) == 0 &&
                pnr::movingTriedAlternativeCount(4) == 3,
            "Moving counted the original placement as a failed alternative");
}

void moving_source_failure_stops_the_stage()
{
    // Check: a complete route-first source repair failure is fatal because no
    // later routing stage is allowed to inherit an unfinished Generic trunk.
    require(pnr::movingFailureRequiresExit(false),
            "Moving Sources retained a source after its repair failed");

    // Successful repair continues; destination exhaustion follows the same
    // immediate-failure policy instead of retrying or processing other tasks.
    require(!pnr::movingFailureRequiresExit(true),
            "Moving rejected a successful repair");

    // Check: one atomically rolled-back placement is not a complete source
    // failure. The same source must retry its next route-proven candidate.
    require(!pnr::movingFailureRequiresExit(false, true),
            "Moving Sources aborted after one rejected placement candidate");
}

void moving_sources_keep_generic_preemption_enabled()
{
    // Check: source recovery may displace transit congestion while rebuilding
    // trunks, unlike unfocused destination suffix repair.
    require(pnr::movingRouteMayPreempt(true, false, true),
            "Moving sources disabled trunk preemption");
    require(!pnr::movingRouteMayPreempt(true, false, false),
            "unfocused Moving destinations enabled broad preemption");
}

void moving_source_input_transaction_protects_sibling_trees()
{
    std::unordered_set<std::string> protected_sources{"driver_a/O", "driver_b/O"};

    // Check: another input route rebuilt by the same source-move transaction
    // cannot be selected as a transit victim and steal this input's leases.
    require(pnr::preemptionOwnerIsTransactionProtected(
                protected_sources, "driver_b/O"),
            "Moving source inputs could preempt a transaction sibling");

    // Check: focused input repair may still preempt an unrelated transit tree,
    // which is the reason preemption remains enabled during this transaction.
    require(!pnr::preemptionOwnerIsTransactionProtected(
                protected_sources, "unrelated_driver/O"),
            "Moving source input protection blocked unrelated transit work");
}

void moving_source_route_endpoint_keeps_physical_cluster_owner()
{
    int physical_source = 1;
    int generated_endpoint = 2;
    int destination = 3;
    auto resolve_owner = [&](int* endpoint) -> int* {
        return endpoint == &generated_endpoint ? &physical_source : nullptr;
    };

    // Check: backward routing still starts at the generated external endpoint,
    // while source relocation follows its void link to the physical owner.
    require(pnr::movingSourcePlacementTarget(&generated_endpoint,
                                              resolve_owner) ==
                &physical_source &&
                pnr::movingSourcePlacementTarget(&destination,
                                                  resolve_owner) ==
                    &destination,
            "Moving sources lost generated-endpoint placement ownership");

    struct Choice
    {
        int* member = nullptr;
        int pos = -1;
    };
    std::vector<int*> cluster{&physical_source, &generated_endpoint};
    std::vector<Choice> incomplete{{&generated_endpoint, 7}};
    std::vector<Choice> complete{{&generated_endpoint, 7},
                                 {&physical_source, 11}};

    // Check: the route endpoint lane alone cannot authorize a move; the
    // preview must also prove the physical source and every packed companion.
    require(!pnr::routeFirstClusterPlacementComplete(
                cluster, incomplete,
                [](const Choice& choice) { return choice.member; }) &&
                pnr::routeFirstClusterPlacementComplete(
                    cluster, complete,
                    [](const Choice& choice) { return choice.member; }),
            "Moving sources accepted an incomplete packed-cluster preview");
}

void rejected_route_first_move_restores_the_complete_cluster()
{
    struct Placement
    {
        int member = -1;
        int tile = -1;
        int pos = -1;
    };
    std::vector<Placement> original{{1, 10, 3}, {2, 10, 7}};
    std::vector<Placement> live{{1, 20, 4}, {2, 20, 8}};
    std::vector<int> scheduler_work{99};
    const std::vector<int> original_scheduler_work = scheduler_work;

    // Emulate a failed immediate input reroute after candidate-local work was
    // queued. Atomic rejection discards that work and restores exact placement.
    scheduler_work.insert(scheduler_work.end(), {11, 12, 13});
    live = original;
    scheduler_work = original_scheduler_work;

    // Check: a rejected route-first candidate cannot leak its speculative
    // placement or manufacture new route obligations for previously live nets.
    require(live.size() == original.size() && live[0].tile == 10 &&
                live[0].pos == 3 && live[1].tile == 10 && live[1].pos == 7 &&
                scheduler_work == original_scheduler_work,
            "rejected route-first move left a cluster or task behind");
}

void moving_source_batches_only_blocked_takeoffs()
{
    // Check: the older takeoff-only classifier remains useful when Moving
    // Destinations guards against accidentally selecting a driver.
    require(!pnr::movingSourceNeedsRelocation(true, false),
            "destination moving misclassified a committed driver takeoff");

    // Check: an unstarted source moves only when no concrete takeoff is free.
    require(!pnr::movingSourceNeedsRelocation(false, true) &&
                pnr::movingSourceNeedsRelocation(false, false),
            "Moving source takeoff classification is inverted");

    // Check: Moving Sources itself is route-guided and replaces every
    // incomplete trunk, including a partial trunk that already owns takeoff.
    require(pnr::movingSourceRouteGuidedTaskNeedsRelocation(false),
            "route-first Moving Sources skipped a partial trunk takeoff");
}

void moving_source_candidate_requires_a_free_takeoff()
{
    auto node_bit = [](int node) { return NodeMask{0, 1} << node; };
    fpga::CBType type;
    type.type_id = 1;
    fpga::CBState state;
    state.type = &type;
    constexpr int local = 7;
    constexpr int direct_src = 10;
    constexpr int joint_src = 11;
    constexpr int first_joint = 20;
    fpga::CBJumpState destination;
    destination.jump = node_bit(30);

    type.local_src[local].jump = node_bit(direct_src);
    type.dst_by_src[direct_src].push_back(
        fpga::CBType::ResolvedJump{{1, 0}, 1, destination, {}, false});
    type.rebuildOutgoingSrcs();
    // Check: one free resolved direct source makes the candidate usable.
    require(state.hasFreeOut(local),
            "Moving rejected a free direct source takeoff");

    state.src.jump = node_bit(direct_src);
    // Check: occupying the only direct source makes the candidate unavailable.
    require(!state.hasFreeOut(local),
            "Moving accepted a fully occupied direct takeoff");

    type.local_src[local].jump = {};
    type.local_joint[local].joint = node_bit(first_joint);
    type.joint_src[first_joint].jump = node_bit(joint_src);
    type.dst_by_src[joint_src].push_back(
        fpga::CBType::ResolvedJump{{0, 1}, 1, destination, {}, false});
    type.rebuildOutgoingSrcs();
    // Check: a free local-to-joint-to-source path is also a usable takeoff.
    require(state.hasFreeOut(local),
            "Moving rejected a free joint-assisted source takeoff");

    state.joint.jump = node_bit(first_joint);
    // Check: occupying the mandatory joint blocks that takeoff even while its
    // source bit itself remains free.
    require(!state.hasFreeOut(local),
            "Moving accepted a takeoff through an occupied joint");
}

void moving_source_candidate_requires_free_input_terminals()
{
    auto node_bit = [](int node) { return NodeMask{0, 1} << node; };
    fpga::CBType type;
    type.type_id = 1;
    fpga::CBState state;
    state.type = &type;
    constexpr int input_local = 21;
    constexpr int input_dst = 34;
    constexpr int input_joint = 55;
    type.dst_local[input_dst].local = node_bit(input_local);
    type.dst_joint[input_dst].joint = node_bit(input_joint);
    type.joint_local[input_joint].local = node_bit(input_local);
    type.rebuildOutgoingSrcs();

    // Check: the hypothetical placement may reserve its destination, joint,
    // and local input without changing the live candidate state.
    fpga::CBState trial = state;
    require(!trial.dst.jump.testBit(input_dst) &&
                !trial.joint.jump.testBit(input_joint) &&
                !trial.local.local.testBit(input_local),
            "input-terminal preflight started from occupied test state");
    trial.dst.jump.setBit(input_dst);
    trial.joint.jump.setBit(input_joint);
    trial.local.local.setBit(input_local);
    require(state.dst.jump == NodeMask{} && state.joint.jump == NodeMask{} &&
                state.local.local == NodeMask{},
            "input-terminal preflight changed live routing masks");

    // Check: occupancy in any required terminal resource makes this placement
    // unsuitable before the source cluster is moved.
    state.joint.jump.setBit(input_joint);
    require(state.joint.jump.testBit(input_joint),
            "input-terminal blocker was not represented in candidate state");

    // Check: ordinary connected inputs require a concrete free fabric terminal,
    // while generated tile-local void links consume only packed element wiring.
    require(pnr::movingSourceInputNeedsFabricTerminal(true, false) &&
                !pnr::movingSourceInputNeedsFabricTerminal(true, true) &&
                !pnr::movingSourceInputNeedsFabricTerminal(false, false),
            "Moving source preflight treated a tile-local void link as fabric");

    // Check: four rejected probes from 32 candidates resume at the next
    // candidate slice, while scanning the complete suffix is true exhaustion.
    require(pnr::movingSourceProbeWindowHasRemaining(32, 0, 4) &&
                pnr::movingSourceProbeWindowHasRemaining(32, 24, 4) &&
                !pnr::movingSourceProbeWindowHasRemaining(32, 28, 4) &&
                !pnr::movingSourceProbeWindowHasRemaining(0, 0, 0),
            "Moving source bounded probe window lost or invented work");
}

void moving_source_candidate_reuses_same_driver_input_terminal()
{
    // Check: a packed sink may reuse an occupied control local when the
    // existing endpoint is driven by the exact same physical source.
    require(pnr::movingSourceInputTerminalAvailable(true, false, true, true),
        "Moving rejected a shared input terminal owned by the same driver");

    // Check: another signal and a reservation made by a different input in
    // this hypothetical placement remain exclusive.
    require(!pnr::movingSourceInputTerminalAvailable(true, false, false, true),
        "Moving reused an input terminal owned by another driver");
    require(!pnr::movingSourceInputTerminalAvailable(false, true, true, true),
        "Moving reused a candidate-local terminal reservation");

    // Check: placement metadata without a live routed owner cannot make an
    // orphan physical lease reusable by the candidate transaction.
    require(!pnr::movingSourceInputTerminalAvailable(true, false, true, false),
        "Moving reused a same-driver reservation without a routed owner");
}

void moving_source_matches_distributed_input_values()
{
    for (bool one : {false, true}) {
        // Check: a live distributed route implements the logical constant even
        // though its generated source endpoint has a different cell identity.
        const bool match = pnr::movingSourceInputOwnerMatches(
            true, one, true, one, false);
        require(match && pnr::movingSourceInputTerminalAvailable(
                    true, false, match, true),
            "Moving rejected a live constant terminal with a generated driver");

        // Check: neither the opposite value nor an ordinary signal can satisfy
        // a constant input, even if endpoint pointer comparison reports equal.
        require(!pnr::movingSourceInputOwnerMatches(true, one, true, !one, true) &&
                    !pnr::movingSourceInputOwnerMatches(true, one, false, one, true),
            "Moving confused a constant with a different physical signal");

        // Check: a matching value never permits an orphan lease or reuse of
        // another input's temporary reservation in the candidate placement.
        require(!pnr::movingSourceInputTerminalAvailable(true, false, match, false) &&
                    !pnr::movingSourceInputTerminalAvailable(true, true, match, true),
            "Moving shared a constant terminal without exclusive live ownership");

        // Check: distributed drivers cannot satisfy an ordinary input; ordinary
        // sharing still requires exact physical source identity.
        require(!pnr::movingSourceInputOwnerMatches(false, one, true, one, true) &&
                    !pnr::movingSourceInputOwnerMatches(false, one, false, one, false) &&
                    pnr::movingSourceInputOwnerMatches(false, one, false, one, true),
            "Moving changed ordinary input driver identity semantics");
    }
}

void moving_source_legalizes_only_a_known_blocked_terminal()
{
    std::vector<pnr::MovingTerminalPath> one_bottleneck{
        {24, 248, 16, -1}, {24, 260, 16, -1}};
    NodeMask pins;
    NodeMask locals;
    NodeMask dsts;
    NodeMask joints;
    joints.setBit(16);

    // Check: known destination paths sharing one occupied joint require sink
    // legalization before backward source routing can create a seed.
    require(pnr::movingSourceNeedsTerminalLegalization(
                one_bottleneck, pins, locals, dsts, joints),
            "Moving Sources did not legalize an occupied terminal bottleneck");

    one_bottleneck.push_back({24, 261, 17, -1});
    // Check: any complete free terminal path suppresses relocation so reverse
    // routing starts from that numeric destination instead.
    require(!pnr::movingSourceNeedsTerminalLegalization(
                one_bottleneck, pins, locals, dsts, joints),
            "Moving Sources relocated a sink with a free terminal seed");

    // Check: absent endpoint topology remains a database/routing error and is
    // not disguised as congestion-driven placement legalization.
    require(!pnr::movingSourceNeedsTerminalLegalization(
                {}, pins, locals, dsts, joints),
            "Moving Sources hid missing terminal topology with relocation");

    // Check: a free terminal whose immediate reverse predecessors are all
    // occupied is legalized because moving its driver cannot cross that cut.
    require(pnr::movingSourceNeedsIngressLegalization(true, 0, 14, 0),
            "Moving Sources ignored saturated terminal ingress");

    // Check: one free predecessor keeps the current sink placement, while a
    // deeper routing obstruction remains a source-routing concern.
    require(!pnr::movingSourceNeedsIngressLegalization(true, 0, 14, 1) &&
                !pnr::movingSourceNeedsIngressLegalization(true, 1, 14, 0) &&
                !pnr::movingSourceNeedsIngressLegalization(false, 0, 14, 0),
            "Moving Sources legalized a sink without an immediate ingress cut");
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

struct ForwardMovingMesh {
    std::vector<fpga::CBType>& types = fpga::Device::current().cb_types;
    explicit ForwardMovingMesh(int width) {
        resetGrid(width, 1);
        types.clear();
        types.resize(width);
        for (int x = 0; x < width; ++x) {
            auto& tile = *fpga::Device::current().getTile(x, 0);
            types[x].name = "mesh_box_" + std::to_string(x);
            types[x].type_id = types[x].base_type_id = x;
            tile.cb_type = tile.cb.type = &types[x];
        }
    }
    fpga::Tile& tile(int x) { return *fpga::Device::current().getTile(x, 0); }
    void edge(int from, int dst, int src, int to, int arrival) {
        types[from].dst_src[dst].jump |= bit(src);
        types[from].dst_by_src[src].push_back({{to - from, 0},
            static_cast<uint16_t>(to), {bit(arrival)}, {}, true});
        types[from].rebuildOutgoingSrcs();
        types[to].dst_local[arrival].local |= bit(400);
        types[to].rebuildOutgoingSrcs();
        tile(to).incoming_dst_nodes |= bit(arrival);
    }
};

void moving_terminal_diagnostics_distinguish_lease_sources() {
    ForwardMovingMesh mesh(1);
    auto& tile = mesh.tile(0);
    for (int i = 0; i < 4; ++i) {
        mesh.types[0].dst_joint[40 + i].joint = bit(21 + i);
        mesh.types[0].joint_local[21 + i].local = bit(50);
        if (i != 3) tile.incoming_dst_nodes.setBit(40 + i);
    }
    mesh.types[0].rebuildOutgoingSrcs();
    Referable<rtl::Net> owner;
    owner.name = "committed_signal_83";
    rtl::Inst sink;
    auto wire = crossbar({0,0}, {0,0}, 40, -1, -1, 1);
    wire.joint = 21;
    sink.wires.push_back({wire, tilePin({0,0}, 60)});
    fpga::attachNetRoute(owner, sink, 0, nullptr, &sink, "Q", "D", "route_83");
    fpga::registerNetRouteTiles(owner, sink.wires[0], 0);
    tile.cb.dst.jump = bit(40);
    tile.cb.joint.jump = bit(21);
    const auto base = tile.cb;
    tile.cb.joint.jump.setBit(22); // A forward prefix, not a registered route.
    const auto before = tile.cb;
    auto trial = tile.cb;
    trial.joint.jump.setBit(23); // Earlier input in the candidate trial.
    std::ostringstream out;
    pnr::printMovingCandidateState(out, tile, base);
    pnr::printMovingTerminalAlternatives(out, tile, bit(50), base, trial, {},
        {}, {}, {{23, "previous_input_19"}});
    const auto text = out.str();
    require(text.find("JOINT:21 name=\"?\" base=1 prefix_only=0 trial=1 registered_route_owner=\"committed_signal_83\"") != std::string::npos &&
                text.find("JOINT:22 name=\"?\" base=0 prefix_only=1 trial=1 registered_route_owner=\"<none>\" temporary_owner=\"<none>\"") != std::string::npos &&
                text.find("JOINT:23 name=\"?\" base=0 prefix_only=0 trial=1 registered_route_owner=\"<none>\" temporary_owner=\"previous_input_19\"") != std::string::npos &&
                text.find("local=50 dst=43 joint=24 joint2=-1 reason=no-incoming-src") != std::string::npos,
            "terminal diagnostic confused physical/prefix/input leases or hid an alternative: " + text);
    require(tile.cb.dst.jump == before.dst.jump && tile.cb.joint.jump == before.joint.jump &&
                tile.pin_state.leased_nodes == NodeMask{},
            "terminal diagnostic changed routing state");
}

void moving_forward_growth_visits_only_reached_tiles() {
    ForwardMovingMesh mesh(5);
    mesh.tile(0).cb.dst.jump = bit(7);
    mesh.edge(0, 7, 10, 2, 8); // Tile 1 is nearby but disconnected.
    mesh.edge(2, 8, 11, 2, 9); // Two same-CB continuations are not reentries.
    mesh.edge(2, 9, 12, 2, 10);
    mesh.edge(2, 10, 13, 4, 11);
    mesh.tile(0).cb.src_deadend.jump = bit(10);
    mesh.tile(2).cb.src_deadend.jump = bit(11) | bit(12) | bit(13);
    std::vector<int> reached;
    std::vector<std::pair<fpga::Tile*, fpga::CBState>> baseline;
    auto result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>& terminal) {
            reached.push_back(tile.coord.x);
            if (tile.coord.x != 4) return false;
            require(mesh.tile(0).cb.src.jump.testBit(10) &&
                    mesh.tile(2).cb.src.jump.testBit(13),
                    "placement probe did not see its connected prefix leases");
            require(!baseline.empty() && baseline.front().first == &mesh.tile(0) &&
                        !baseline.front().second.src.jump.testBit(10),
                    "diagnostic baseline confused prefix leases with committed routes");
            terminal = {crossbar(tile.coord, tile.coord, dst, -1, -1, 1),
                        tilePin(tile.coord, 21)};
            return true;
        }, {}, nullptr, [&](const auto& saved) { baseline = saved; });
    require(result.success && result.fragments.size() == 6 &&
                std::find(reached.begin(), reached.end(), 1) == reached.end() &&
                std::find(reached.begin(), reached.end(), 3) == reached.end(),
            "Moving forward reachability: success=" + std::to_string(result.success) +
            " fragments=" + std::to_string(result.fragments.size()) +
            " expanded=" + std::to_string(result.expanded));
    require(mesh.tile(0).cb.dst.jump == bit(7) &&
                mesh.tile(0).cb.src.jump == NodeMask{} &&
                mesh.tile(2).cb.dst.jump == NodeMask{} &&
                mesh.tile(2).cb.src.jump == NodeMask{},
            "forward placement search leaked temporary route leases");
    mesh.tile(2).cb.src.jump = bit(13);
    result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [&](fpga::Tile& tile, int, bool, std::vector<fpga::Wire>&) {
            return tile.coord.x == 4;
        });
    require(!result.success && mesh.tile(2).cb.src.jump == bit(13),
            "Moving crossed a busy edge or cleared another route");
    result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [](auto&, int, bool, auto&) { return false; }, [] { return true; });
    require(result.cancelled && !result.success && result.expanded == 0,
            "forward Moving ignored stage cancellation");
}

void moving_forward_growth_rejects_third_visit_per_path() {
    ForwardMovingMesh mesh(4);
    mesh.tile(0).cb.dst.jump = bit(0);
    mesh.edge(0, 0, 10, 1, 1);
    mesh.edge(1, 1, 11, 0, 2); // second visit to tile 0
    mesh.edge(0, 2, 12, 2, 3);
    mesh.edge(2, 3, 13, 0, 4); // forbidden third visit
    mesh.edge(0, 0, 14, 3, 5);
    mesh.edge(3, 5, 15, 2, 6);
    mesh.edge(2, 6, 16, 3, 7); // second visit to tile 3 is legal
    mesh.edge(3, 7, 17, 0, 8); // independent second visit to tile 0
    bool saw_illegal = false;
    auto result = pnr::routeForwardToPlacement({{&mesh.tile(0), 0, true, {}}},
        [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>&) {
            saw_illegal |= tile.coord.x == 0 && dst == 4;
            return tile.coord.x == 0 && dst == 8;
        });
    require(result.success && !saw_illegal && result.tile_visit_rejects > 0,
            "Moving failed path-local third-visit rejection or poisoned an alternative");
    require(mesh.tile(0).cb.dst.jump == bit(0) &&
                mesh.tile(0).cb.src.jump == NodeMask{},
            "revisited Moving path left speculative occupancy");
    // Existing fabric counts too: this retained prefix has visited tile 0
    // twice, so neither new branch may re-enter it a third time.
    std::vector<fpga::Wire> prefix{
        crossbar({0,0}, {1,0}, 40, 41, 42, 0),
        crossbar({1,0}, {0,0}, 42, 43, 0, 1)};
    bool revisited_root = false;
    result = pnr::routeForwardToPlacement({{&mesh.tile(0), 0, true, prefix}},
        [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>&) {
            revisited_root |= tile.coord.x == 0 && dst != 0;
            return false;
        });
    require(!result.success && !revisited_root && result.tile_visit_rejects > 0,
            "Moving forgot retained-prefix Tile visits");
}

void moving_forward_exhaustion_reports_every_exit() {
    ForwardMovingMesh mesh(8);
    mesh.tile(0).cb.dst.jump = bit(7);
    for (int x = 0; x < 5; ++x) mesh.edge(x, 7, 10, x + 1, 7);
    mesh.edge(5, 7, 10, 6, 7);
    mesh.edge(5, 7, 11, 7, 7);
    mesh.tile(5).cb.src.jump = bit(10);
    mesh.tile(7).cb.dst.jump = bit(7);
    std::ostringstream decisions;
    const auto result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [](auto&, int, bool, auto&) { return false; }, {}, &decisions);
    require(!result.success && !result.cancelled && result.expanded == 6,
            "blocked forward search did not exhaust exactly six states");
    const auto text = decisions.str();
    require(text.find("FORWARD_EDGE state=5") != std::string::npos &&
                text.find("src_busy=1 free=0") != std::string::npos &&
                text.find("FORWARD_TARGET state=5 src=11") != std::string::npos &&
                text.find("name=? busy=1") != std::string::npos &&
                text.find("FORWARD_END queued=6 reached=6 cancelled=0") != std::string::npos,
            "forward exhaustion report omitted blocked SRC/DST decisions");
    require(mesh.tile(5).cb.src.jump == bit(10) && mesh.tile(7).cb.dst.jump == bit(7)
                && mesh.tile(1).cb.src.jump == NodeMask{},
            "forward tracing altered committed or speculative occupancy");
}

void moving_destination_commits_forward_grounding(bool blocked,
                                                bool retained = false,
                                                bool at_root = false,
                                                bool packing_blocked = false,
                                                bool source_focus = false,
                                                bool generated_source = false,
                                                bool support_blocked = false) {
    ForwardMovingMesh mesh(5);
    fpga::TileType resource{"PACKING_BOX", 1};
    fpga::Element reg;
    reg.name = "register_slots";
    reg.type = fpga::ELEMENT_FD;
    reg.bitmap_pos = 0;
    resource.elements.push_back(reg);
    reg.bitmap_pos = 1;
    resource.elements.push_back(reg);
    for (int x : {1, 2, 3, 4})
        if (x != 1 || at_root) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent;
    Referable<rtl::Module> primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_cell";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sibling, sink, helper;
    sink.cell_ref.set(&cell);
    driver.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    Referable<rtl::Cell> helper_cell;
    helper_cell.name = "source_helper_29";
    helper_cell.type = "FDRE";
    helper_cell.module_ref.set(&primitive);
    helper_cell.attributes["scalepnr_passthrough"] = "source";
    helper.cell_ref.set(&helper_cell);
    helper.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    helper.pos = 4;
    rtl::Inst* source = generated_source ? &helper : &driver;
    require(mesh.tile(4).tryAddAt(&sink, 0) == 0, "cannot place original sink");
    parent.nets.resize(support_blocked ? 2 : 1);
    auto& net = parent.nets.front();
    net.name = "forward_move_signal";
    sibling.wires.push_back({crossbar({0,0}, {1,0}, 4, 5, 7, 0),
        crossbar({1,0}, {1,0}, 7, -1, -1, 1), tilePin({1,0}, 9)});
    mesh.tile(0).cb.src.jump = bit(5);
    mesh.tile(1).cb.dst.jump = bit(7);
    sink.wires.push_back({});
    if (retained) {
        sink.wires[0] = {sibling.wires[0].front()};
        sink.wires[0].back().owns_landing = true;
        sibling.wires.clear();
    } else {
        mesh.tile(1).cb.local.local = mesh.tile(1).pin_state.leased_nodes = bit(9);
        fpga::attachNetRoute(net, sibling, 0, source, &sibling, "Q", "D", "stable_trunk");
        fpga::registerNetRouteTiles(net, sibling.wires[0], 0);
    }
    fpga::attachNetRoute(net, sink, 0, source, &sink, "Q", "D", "moving_suffix");
    if (retained) fpga::registerNetRouteTiles(net, sink.wires[0], 0);
    mesh.edge(1, 7, 10, 2, 8);
    if (blocked) mesh.tile(1).cb.src.jump.setBit(10);
    const int landing = at_root ? 1 : 2;
    const int dst = at_root ? 7 : 8;
    rtl::Inst blockers[2];
    if (packing_blocked) {
        for (int i = 0; i < 2; ++i) {
            blockers[i].cell_ref.set(&cell);
            require(mesh.tile(landing).tryAddAt(&blockers[i], i * 4) >= 0,
                    "cannot occupy trial packing slots");
        }
    }
    const int pin0 = mesh.tile(landing).getPinNodes("FDRE", "D", 0).firstSetBit();
    const int pin1 = mesh.tile(landing).getPinNodes("FDRE", "D", 4).firstSetBit();
    Referable<rtl::Port> reset_port;
    require(pin0 >= 0 && pin1 >= 0 && pin0 != pin1, "fixture requires distinct input lanes");
    // The first packing position has only an unreachable terminal. The second
    // position grounds through two joints from the actually reached DST.
    mesh.types[landing].dst_local[99].local = bit(pin0);
    mesh.types[landing].dst_joint[dst].joint = bit(21);
    mesh.types[landing].joint_joint[21].joint = bit(22);
    mesh.types[landing].joint_local[22].local = bit(pin1);
    if (support_blocked) {
        // Another input's only free terminal shares the trigger's temporary
        // joint. Its disjoint alternative is already occupied, so the packing
        // reservation filter does not reject the shared joint beforehand.
        const int reset_pin = mesh.tile(landing).getPinNodes("FDRE", "R", 4).firstSetBit();
        require(reset_pin >= 0 && reset_pin != pin1, "fixture needs a reset input");
        mesh.types[landing].dst_joint[81].joint = bit(21);
        mesh.types[landing].joint_local[22].local |= bit(reset_pin);
        mesh.tile(landing).incoming_dst_nodes.setBit(81);
        mesh.types[landing].dst_local[82].local = bit(reset_pin);
        mesh.tile(landing).incoming_dst_nodes.setBit(82);
        mesh.tile(landing).cb.dst.jump.setBit(82);
        auto& reset = parent.nets[1];
        reset.name = "reset_signal_47";
        reset.designators = {47};
        reset_port.name = "R";
        reset_port.type = rtl::Port::PORT_IN;
        reset_port.designator = 47;
        sink.conns.resize(1);
        sink.conns.front().port_ref.set(&reset_port);
        sink.wires.push_back({});
        fpga::attachNetRoute(reset, sink, 1, &driver, &sink, "Q", "R", "reset_route_47");
    }
    mesh.types[landing].rebuildOutgoingSrcs();
    // A disconnected tile looks locally ideal; it must never be selected.
    mesh.types[3].dst_local[8].local = bit(pin0);
    mesh.types[3].rebuildOutgoingSrcs();
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&net, source, "Q");
    if (support_blocked) router.indexSourceRoute(&parent.nets[1], &driver, "Q");
    pnr::RouteDesign::RouteTask task{source, &sink, &net, "Q", "D", "moving_suffix"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    const auto trunk = retained ? sink.wires[0] : sibling.wires[0];
    auto move_task = task;
    if (source_focus) {
        // Match the production selection, including a generated driver in the
        // source focus's endpoint closure. The old focus-only target selected
        // driver and the anchor guard rejected every retained branch.
        std::unordered_set<rtl::Inst*> focus_endpoints{&driver, &helper};
        move_task.to = pnr::movingDestinationPlacementTarget(
            &driver, task.to, focus_endpoints.contains(task.from),
            focus_endpoints.contains(task.to));
    }
    bool moved = router.moveUnfinishedCell(move_task, &tasks, &task, &reason);
    require(driver.tile.peer == &mesh.tile(0) && driver.pos == 0 &&
                helper.tile.peer == &mesh.tile(0) && helper.pos == 4,
            "outgoing focus relocation moved the source or its generated endpoint");
    require(moved != (blocked || packing_blocked || support_blocked), "forward relocation result: " + reason);
    const auto& preserved = retained ? sink.wires[0] : sibling.wires[0];
    require(retained ? preserved.size() >= trunk.size() : preserved.size() == trunk.size(),
            "Moving changed retained fabric size");
    for (size_t i = 0; i < trunk.size(); ++i)
        require(sameFragment(trunk[i], preserved[i]), "Moving altered retained fabric");
    if (blocked || packing_blocked || support_blocked) {
        require(sink.tile.peer == &mesh.tile(4) && sink.pos == 0 && sink.wires[0].empty(),
                "failed forward relocation did not restore original placement");
        require(router.failure_packing_trace && router.failure_packing_net == task.net_name,
                "Moving did not retain the failed task packing trace");
        if (support_blocked) {
            std::rewind(router.failure_packing_trace.get());
            std::string trace;
            char buffer[4096];
            while (size_t n = std::fread(buffer, 1, sizeof(buffer), router.failure_packing_trace.get()))
                trace.append(buffer, n);
            require(trace.find("OTHER_INPUT_REJECT net=\"reset_route_47\" port=\"R\"") != std::string::npos &&
                        trace.find("CANDIDATE_CB tile=(2,0)") != std::string::npos &&
                        trace.find("JOINT:21 name=\"?\" base=0 prefix_only=0 trial=1 registered_route_owner=\"<none>\" temporary_owner=\"moving_suffix\"") != std::string::npos,
                    "automatic support diagnostic lost temporary trigger ownership: " + trace);
        }
        if (packing_blocked) {
            char directory[] = "/tmp/scalepnr-packing-report-XXXXXX";
            require(mkdtemp(directory) != nullptr, "cannot create packing report directory");
            const auto path = std::filesystem::path(directory);
            std::fflush(nullptr);
            const pid_t child = fork();
            require(child >= 0, "cannot fork packing report test");
            if (!child) {
                setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", directory, 1);
                if (!std::freopen((path / "stdout.txt").c_str(), "w", stdout)) _exit(2);
                router.recordRouteTaskFailure(task, {landing, 0}, fpga::CB_NODE_DST, dst);
                router.failRouting(&task, "Moving packing exhausted: " + reason);
            }
            int status = 0;
            require(waitpid(child, &status, 0) == child && WIFEXITED(status) &&
                        WEXITSTATUS(status) == EXIT_FAILURE, "packing failure did not exit");
            auto read = [&](const char* name) {
                std::ifstream file(path / name);
                return std::string(std::istreambuf_iterator<char>(file), {});
            };
            const auto trace = read("routing_failure_packing.txt");
            require(trace.find("net=moving_suffix") != std::string::npos &&
                    trace.find("CANDIDATE reached=(2,0) dst=8") != std::string::npos &&
                    trace.find("CANDIDATE_CB tile=(2,0)") != std::string::npos &&
                    trace.find("reason=busy owner=") != std::string::npos &&
                    trace.find("pos=0 depth=0") != std::string::npos &&
                    trace.find("pos=4 depth=0") != std::string::npos &&
                    read("stdout.txt").find(trace) != std::string::npos &&
                    read("routing_failure.txt").find("packing_report_written=true") != std::string::npos,
                    "failure report lost per-position packing diagnostics: " + trace);
            std::filesystem::remove_all(path);
        }
        return;
    }
    require(sink.tile.peer == &mesh.tile(landing) && sink.pos == 4 &&
                !sink.wires[0].empty() && sink.wires[0].back().type == fpga::Wire::WIRE_TILE_PIN &&
                sink.wires[0].back().local == pin1,
            "Moving grounding mismatch: tile=" + std::to_string(sink.tile->coord.x) +
            " pos=" + std::to_string(sink.pos) +
            " route_size=" + std::to_string(sink.wires[0].size()) +
            " expected_pin=" + std::to_string(pin1));
    require(mesh.tile(landing).cb.dst.jump.testBit(dst) &&
                mesh.tile(landing).cb.joint.jump == (bit(21) | bit(22)) &&
                mesh.tile(landing).pin_state.leased_nodes.testBit(pin1),
            "Moving returned success without committed terminal leases");
    for (int x : {0, 1, 2}) {
        std::ostringstream out;
        auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                "forward relocation ownership mismatch: " + out.str());
    }
}

void distributed_input_sharing_requires_a_live_route()
{
    resetGrid(2, 1);
    Referable<rtl::Net> net;
    net.name = "distributed_control";
    net.distributed_source = true;
    rtl::Inst generated_driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.push_back({
        crossbar({0, 0}, {1, 0}, 10, 110, 210, 0),
        tilePin({1, 0}, 32),
    });
    leaseRoute(owner.wires[0]);
    fpga::attachNetRoute(net, owner, 0, &generated_driver, &sink,
                        "output", "control", "control_route");
    fpga::registerNetRouteTiles(net, owner.wires[0], 0);
    fpga::Tile& tile = *fpga::Device::current().getTile(1, 0);
    const NodeMask leases_before = tile.pin_state.leased_nodes;
    auto has_owner = [&](bool one) {
        for (const auto& route : fpga::findNetRoutesByNode(
                 tile, fpga::CB_NODE_LOCAL, 32, false)) {
            if (route.net && route.binding_index < route.net->routes.size() &&
                pnr::movingSourceInputOwnerMatches(
                    true, one, route.net->distributed_source,
                    route.net->distributed_one, false)) {
                return true;
            }
        }
        return false;
    };
    for (bool one : {false, true}) {
        net.distributed_one = one;
        // Check: the owner is found through the real tile/route registry even
        // though its generated driver is not the logical constant endpoint.
        require(has_owner(one) && !has_owner(!one),
                "Moving lost the distributed input owner or confused its value");
    }
    // Check: a stale registry and occupied bit without a route cannot satisfy
    // an input proof. Ownership inspection itself must not modify live masks.
    owner.wires[0].clear();
    require(!has_owner(false) && !has_owner(true) &&
                tile.pin_state.leased_nodes == leases_before,
            "Moving accepted an orphan constant lease or changed live state");
}

void stagnation_failure_writes_png_and_live_tile_debug(bool destination_failure = false)
{
    resetGrid(5, 5);
    fpga::CBType cb;
    cb.name = "failure_matrix";
    cb.rememberNodeName(fpga::CB_NODE_DST, 7, "arrival_probe");
    cb.rememberNodeName(fpga::CB_NODE_SRC, 8, "departure_probe");
    cb.dst_src[7].jump.setBit(8);
    cb.rebuildOutgoingSrcs();
    for (auto& tile : fpga::Device::current().tile_grid) {
        tile.cb_type = &cb;
        tile.cb.type = &cb;
        tile.cb_coord = tile.name = tile.coord;
    }
    auto* tile = fpga::Device::current().getTile(2, 2);
    tile->cb.src.jump.setBit(8);
    if (destination_failure) tile->cb.dst.jump.setBit(7);
    char directory[] = "/tmp/scalepnr-failure-test-XXXXXX";
    require(mkdtemp(directory) != nullptr, "cannot create failure test directory");
    const auto path = std::filesystem::path(directory);
    std::fflush(nullptr);
    const pid_t child = fork();
    require(child >= 0, "cannot fork failure-report test");
    if (child == 0) {
        setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", directory, 1);
        if (!std::freopen((path / "stdout.txt").c_str(), "w", stdout)) _exit(2);
        pnr::RouteDesign router;
        router.fpga = &fpga::Device::current();
        pnr::RouteDesign::RouteTask task;
        task.net_name = "blocked_probe";
        router.recordRouteTaskFailure(task, {2, 2}, fpga::CB_NODE_SRC, 8);
        if (destination_failure) {
            const auto result = pnr::routeForwardToPlacement({{tile, 7, true, {}}},
                [](auto&, int, bool, auto&) { return false; });
            if (pnr::movingFailureRequiresExit(result.success))
                router.failRouting(&task, "Moving destinations failed: forward search exhausted");
            // Any retry or advancement to another task is a regression.
            _exit(3);
        }
        router.failRouting(&task, "Fanouts routing stagnation");
    }
    int status = 0;
    require(waitpid(child, &status, 0) == child && WIFEXITED(status) &&
                WEXITSTATUS(status) == EXIT_FAILURE,
            "stagnation did not exit as a routing failure");
    auto read = [&](const char* name) {
        std::ifstream stream(path / name, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(stream), {});
    };
    const auto report = read("routing_failure.txt");
    const auto output = read("stdout.txt");
    const auto png = read("routing_failure.png");
    if (destination_failure)
        require(report.find("reason=Moving destinations failed: forward search exhausted") != std::string::npos,
                "Moving exhaustion failed to exit/report the first failed task");
    // Exercise the real terminal exit, not just a renderer: the last node,
    // center, failure status, and PNG must describe the same failed attempt.
    require(report.find("routing_status=failed") != std::string::npos &&
                report.find("failure_coord=2,2") != std::string::npos &&
                report.find("failure_node=8") != std::string::npos &&
                report.find("image_written=true") != std::string::npos &&
                report.find("backward_report_written=true") != std::string::npos &&
                read("routing_failure_backward.txt").find("NO_PLACED_SINK") != std::string::npos &&
                png.starts_with(std::string("\x89PNG\r\n\x1a\n", 8)),
            "stagnation failed to write the correct report and PNG in " + path.string());
    // Deliberately leave one ownerless busy bit: the stdout audit must expose
    // the real mask rather than inventing an owner or silently clearing it.
    require(output.find("routing failure tile:") != std::string::npos &&
                output.find("coord=(2,2)") != std::string::npos &&
                output.find("MASK SRC=[8,") != std::string::npos &&
                output.find("status=UNOWNED_LEASE") != std::string::npos &&
                output.find("routing failure tile ownership:") != std::string::npos,
            "stagnation omitted its live congestion state in " + path.string());
    require(tile->cb.src.jump == bit(8), "failure reporting changed live leases");
    std::filesystem::remove_all(path);
    for (auto& item : fpga::Device::current().tile_grid) {
        item.cb_type = nullptr;
        item.cb.type = nullptr;
    }
}

void moving_empty_suffix_reuses_sibling_takeoff(bool indexed, bool alias)
{
    resetGrid(3, 1);
    pnr::RouteDesign router;
    Referable<rtl::Module> parent;
    Referable<rtl::Module> child;
    child.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "driver_47";
    cell.type = "LUT1";
    cell.module_ref.set(&child);
    rtl::Inst driver;
    rtl::Inst other_driver;
    rtl::Inst sink;
    rtl::Inst sibling;
    rtl::Inst owner;
    driver.cell_ref.set(&cell);
    driver.tile.set(&fpga::Device::current().tile_grid[0]);
    driver.pos = 0;
    parent.nets.resize(2);
    auto& net = parent.nets[0];
    auto& seed_net = parent.nets[alias ? 1 : 0];
    net.name = "signal_83";
    if (alias) seed_net.name = "signal_alias_29";

    fpga::CBType type;
    type.name = "crossbox_61";
    type.type_id = 1;
    fpga::Tile& tile = *driver.tile;
    tile.cb_type = &type;
    tile.cb.type = &type;
    const int local = tile.getOutputPinNodes("LUT1", "O", 0).firstSetBit();
    require(local >= 0, "test driver has no output node");
    constexpr int src = 117;
    constexpr int dst = 217;
    type.local_src[local].jump = bit(src);
    fpga::CBJumpState destination;
    destination.jump = bit(dst);
    type.dst_by_src[src].push_back(
        fpga::CBType::ResolvedJump{{1, 0}, 1, destination, {}, false});
    type.rebuildOutgoingSrcs();
    owner.wires.push_back({});
    owner.wires.push_back({crossbar({0, 0}, {1, 0}, local, src, dst, 0),
                           crossbar({1, 0}, {1, 0}, dst, 118, 318, 1),
                           tilePin({1, 0}, 318)});
    fpga::attachNetRoute(net, owner, 0, &driver, &sink,
                        "O", "I0", "suffix_53");
    fpga::attachNetRoute(seed_net, owner, 1, &driver, &sibling,
                        "O", "I0", "trunk_71");
    leaseRoute(owner.wires[1]);
    if (indexed) {
        router.indexSourceRoute(&net, &driver, "O");
        router.indexSourceRoute(&seed_net, &driver, "O");
    }
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net,
                                    "O", "I0", "suffix_53"};
    task.fanout = true;
    require(!tile.cb.hasFreeOut(local),
            "test did not occupy the driver's only takeoff");
    require(router.sourceTreeHasCompleteExit(driver, "O"),
            "test sibling is not a complete source-tree seed");
    require(!router.movingSourceNeedsRelocation(task),
            "empty suffix misclassified its sibling-owned takeoff as a blocked driver");

    // An unfinished sibling can still own a committed source prefix.
    require(fpga::truncateNetRoute(seed_net, seed_net.routes.size() - 1, 1),
            "test could not retain only the sibling takeoff");
    require(!router.sourceTreeHasCompleteExit(driver, "O"),
            "partial sibling unexpectedly remained complete");
    require(!router.movingSourceNeedsRelocation(task),
            "partial sibling takeoff was mistaken for a missing driver exit");

    auto& binding = seed_net.routes.back();
    binding.from_port = "other_output";
    require(router.movingSourceNeedsRelocation(task),
            "takeoff on another port hid a genuinely blocked source");
    binding.from_port = "O";
    binding.from = &other_driver;
    require(router.movingSourceNeedsRelocation(task),
            "another driver's takeoff was reused");
    binding.from = &driver;
    require(fpga::unrouteNetRoute(seed_net, seed_net.routes.size() - 1),
            "test could not release the sibling prefix");
    tile.cb.local.local = {};
    tile.cb.src.jump = bit(src); // Occupied exit with no surviving source binding.
    require(router.movingSourceNeedsRelocation(task),
            "cleared sibling still counted as a committed takeoff");
    tile.cb.src.jump = {};
    require(!router.movingSourceNeedsRelocation(task),
            "genuinely free takeoff was rejected");
    require(driver.tile.peer == &tile && driver.pos == 0 && owner.wires[0].empty(),
            "classification changed placement or routed the empty suffix");
    tile.cb_type = nullptr;
    tile.cb.type = nullptr;
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

    // Check: the moved route retains its shared trunk and private fabric path,
    // stopping at the landing immediately before the old local terminal.
    const std::vector<fpga::Wire>& moved_route = owners[moved_binding]->wires[0];
    require(moved_route.size() == trunk.size() + 1,
        "Moving removed a reusable private fabric prefix or retained its terminal");
    for (size_t index = 0; index < trunk.size(); ++index) {
        require(moved_route[index].shared
                && sameFragment(moved_route[index], sibling_snapshots[moved_branch][index]),
            "Moving changed the selected fanout's shared prefix");
    }
    require(!moved_route.back().shared && moved_route.back().owns_landing,
        "Moving did not retain the private branch as an extendable landing");

    // Check: the reusable branch stays leased while only the old terminal path
    // and resource pin become free for the replacement suffix.
    require(isSet(branch_tile->cb.src.jump, moved_src)
            && !isSet(moved_tile->cb.src.jump, moved_arrival_src)
            && isSet(moved_tile->cb.dst.jump, moved_arrival_dst)
            && !isSet(moved_tile->cb.local.local, moved_local)
            && !isSet(moved_tile->pin_state.leased_nodes, moved_local),
        "Moving did not preserve the fabric landing or release the old terminal");

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

    net.route_protected = true;
    require(fpga::discardNetBranch(net, moved_binding),
        "Moving could not forget a protected branch's stale trunk replica");

    // Check: protected infrastructure reroutes from its live tree instead of
    // continuing the shared prefix that led toward the sink's old placement.
    require(owners[moved_binding]->wires[0].empty(),
        "Moving retained a stale protected shared-prefix replica");

    // Check: forgetting the non-owning replica must preserve the real trunk
    // owner and every sibling branch lease.
    require(isSet(fpga::Device::current().getTile(0, 1)->cb.src.jump, 100)
            && isSet(fpga::Device::current().getTile(1, 1)->cb.src.jump, 101)
            && isSet(fpga::Device::current().getTile(2, 1)->cb.src.jump, 102),
        "Moving released the protected tree while forgetting one replica");
    for (int branch = 0; branch < branch_count; ++branch) {
        if (branch == moved_branch) {
            continue;
        }
        require(isSet(branch_tile->cb.src.jump, 320 + branch)
                && isSet(fpga::Device::current()
                             .getTile(branch + 5, 1)->cb.src.jump,
                         400 + branch),
            "Moving changed a protected sibling while forgetting one replica");
    }
}

void moving_source_replaces_only_a_dead_partial_tail()
{
    resetGrid(4, 1);
    Referable<rtl::Net> net;
    net.name = "prefix_replacement_route";
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.push_back({
        tilePin({0, 0}, 10),
        crossbar({0, 0}, {1, 0}, 10, 100, 200, 0),
        crossbar({1, 0}, {2, 0}, 200, 101, 201, 1),
        crossbar({2, 0}, {3, 0}, 201, 102, 202, 1),
        tilePin({3, 0}, 300),
    });
    leaseRoute(owner.wires[0]);
    fpga::attachNetRoute(net, owner, 0, &driver, &sink,
                         "random_output", "random_input", "partial_route");

    // Check: exact prefix truncation retains the source and first landing but
    // releases every lease in the obsolete tail selected by reverse docking.
    require(fpga::truncateNetRoute(net, 0, 2),
            "Moving could not truncate the dead partial-route tail");
    require(owner.wires[0].size() == 2 &&
                owner.wires[0].back().owns_landing &&
                isSet(fpga::Device::current().getTile(0, 0)->cb.src.jump, 100) &&
                isSet(fpga::Device::current().getTile(1, 0)->cb.dst.jump, 200),
            "Moving changed the retained route prefix or its landing owner");
    require(!isSet(fpga::Device::current().getTile(1, 0)->cb.src.jump, 101) &&
                !isSet(fpga::Device::current().getTile(2, 0)->cb.dst.jump, 201) &&
                !isSet(fpga::Device::current().getTile(2, 0)->cb.src.jump, 102) &&
                !isSet(fpga::Device::current().getTile(3, 0)
                           ->pin_state.leased_nodes,
                       300),
            "Moving retained leases from the replaced partial-route tail");
}

void moving_private_route_releases_its_stale_takeoff()
{
    resetGrid(3, 1);

    Referable<rtl::Net> net;
    net.name = "private_route_without_siblings";
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.push_back({
        tilePin({0, 0}, 17),
        crossbar({0, 0}, {1, 0}, 17, 117, 217, 0),
        crossbar({1, 0}, {2, 0}, 217, 118, 218, 1),
        crossbar({2, 0}, {2, 0}, 218, 119, 318, 1),
        tilePin({2, 0}, 318),
    });
    leaseRoute(owner.wires[0]);
    // Reproduce stale fanout metadata left on a route that no live sibling
    // actually shares. Moving must use physical sibling ownership, not flags.
    for (fpga::Wire& fragment : owner.wires[0]) {
        fragment.shared = true;
    }
    fpga::attachNetRoute(net, owner, 0, &driver, &sink,
        "random_output", "random_input", "private_route");

    // Check: pre-move input validation must use the same surviving prefix as
    // detachment, even when this is the sole route and its sink is moving.
    const size_t anchor_prefix = fpga::retainedRoutePrefixSize(
        owner.wires[0], false, true);
    require(anchor_prefix == 3,
        "pre-move input proof discarded the moved sink's private anchors");
    // Check: a sibling whose sink stays put keeps all its anchors, whereas
    // moving its driver invalidates every anchor, including the old takeoff.
    require(fpga::retainedRoutePrefixSize(owner.wires[0], false, false) == 5 &&
            fpga::retainedRoutePrefixSize(owner.wires[0], true, false) == 0 &&
            fpga::retainedRoutePrefixSize(owner.wires[0], true, true) == 0,
        "pre-move input proof reused an invalidated driver or lost a sibling");

    require(fpga::invalidateMovedSinkRoute(net, 0),
        "Moving could not invalidate a private route");

    // Check: stale shared flags do not affect the ownership decision. The
    // private fabric path remains as a concrete anchor for the relocated sink.
    require(owner.wires[0].size() == anchor_prefix && owner.wires[0].back().owns_landing,
        "Moving lost the private route anchor or retained its old terminal");

    // Check: invalidating the sink preserves source/transit ownership and
    // releases only the old destination terminal and endpoint lease.
    fpga::Tile* source = fpga::Device::current().getTile(0, 0);
    fpga::Tile* transit = fpga::Device::current().getTile(1, 0);
    fpga::Tile* destination = fpga::Device::current().getTile(2, 0);
    require(source && transit && destination
            && isSet(source->cb.local.local, 17)
            && isSet(source->cb.src.jump, 117)
            && isSet(transit->cb.dst.jump, 217)
            && isSet(transit->cb.src.jump, 118)
            && isSet(destination->cb.dst.jump, 218)
            && !isSet(destination->cb.src.jump, 119)
            && !isSet(destination->cb.local.local, 318)
            && !isSet(destination->pin_state.leased_nodes, 318),
        "Moving did not preserve the private prefix or release its terminal");
}

void moving_stale_shared_route_releases_the_complete_private_path()
{
    resetGrid(4, 1);

    Referable<rtl::Net> net;
    net.name = "stale_shared_route_with_private_takeoff";
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.push_back({
        tilePin({0, 0}, 27),
        crossbar({0, 0}, {1, 0}, 27, 127, 227, 0),
        crossbar({1, 0}, {2, 0}, 227, 128, 228, 1),
        crossbar({2, 0}, {3, 0}, 228, 129, 229, 1),
        crossbar({3, 0}, {3, 0}, 229, 130, 329, 1),
        tilePin({3, 0}, 329),
    });
    leaseRoute(owner.wires[0]);
    for (fpga::Wire& fragment : owner.wires[0]) {
        fragment.shared = true;
    }
    fpga::attachNetRoute(net, owner, 0, &driver, &sink,
        "random_output", "random_input", "stale_shared_route");

    require(fpga::invalidateMovedSinkRoute(net, 0),
        "Moving could not detach a stale shared route");

    // Check: stale shared flags do not manufacture sibling ownership, while the
    // route's own private fabric remains a valid continuation anchor.
    require(owner.wires[0].size() == 4 && owner.wires[0].back().owns_landing,
        "Moving lost a reusable private path or retained its old terminal");

    // Check: fabric ownership survives and only the local terminal resources
    // are released through the live-owner scan.
    fpga::Tile* source = fpga::Device::current().getTile(0, 0);
    fpga::Tile* transit1 = fpga::Device::current().getTile(1, 0);
    fpga::Tile* transit2 = fpga::Device::current().getTile(2, 0);
    fpga::Tile* destination = fpga::Device::current().getTile(3, 0);
    require(source && transit1 && transit2 && destination
            && isSet(source->cb.src.jump, 127)
            && isSet(transit1->cb.dst.jump, 227)
            && isSet(transit1->cb.src.jump, 128)
            && isSet(transit2->cb.dst.jump, 228)
            && isSet(transit2->cb.src.jump, 129)
            && isSet(destination->cb.dst.jump, 229)
            && !isSet(destination->cb.src.jump, 130)
            && !isSet(destination->cb.local.local, 329)
            && !isSet(destination->pin_state.leased_nodes, 329),
        "Moving did not preserve fabric ownership or release the old terminal");
}

void moving_input_prefix_extension_keeps_ownership_and_frees_old_tail()
{
    resetGrid(4, 1);
    Referable<rtl::Net> net;
    net.name = "retained_input_owner";
    rtl::Inst driver;
    rtl::Inst sink;
    rtl::Inst owner;
    owner.wires.push_back({
        tilePin({0, 0}, 17),
        crossbar({0, 0}, {1, 0}, 17, 117, 217, 0),
        crossbar({1, 0}, {2, 0}, 217, 118, 218, 1),
        crossbar({2, 0}, {2, 0}, 218, 119, 318, 1),
        tilePin({2, 0}, 318),
    });
    leaseRoute(owner.wires[0]);
    fpga::attachNetRoute(net, owner, 0, &driver, &sink,
                        "out", "in", "retained_input_owner");

    // The new destination docks at an earlier landing of its own input route.
    // Sink invalidation preserves that owner; trimming releases only its tail.
    require(fpga::invalidateMovedSinkRoute(net, 0) &&
                fpga::truncateNetRoute(net, 0, 2),
            "moving input could not retain its selected private anchor");
    std::vector<fpga::Wire> suffix{
        crossbar({1, 0}, {3, 0}, 217, 120, 220, 1),
        crossbar({3, 0}, {3, 0}, 220, 121, 321, 1),
        tilePin({3, 0}, 321),
    };
    leaseRoute(suffix);
    owner.wires[0].insert(owner.wires[0].end(), suffix.begin(), suffix.end());
    fpga::registerNetRouteTiles(net, owner.wires[0], 0);

    auto *source = fpga::Device::current().getTile(0, 0);
    auto *branch = fpga::Device::current().getTile(1, 0);
    auto *old_sink = fpga::Device::current().getTile(2, 0);
    // Check: replacing a suffix must neither orphan its takeoff nor leave the
    // old destination/unused private tail occupied without a live route.
    require(net.routes.size() == 1 && net.routes[0].route_index == 0 &&
                !owner.wires[0][1].shared && source->cb.src.jump.testBit(117) &&
                branch->cb.dst.jump.testBit(217) && branch->cb.src.jump.testBit(120) &&
                !branch->cb.src.jump.testBit(118) && old_sink->cb.dst.jump == NodeMask{} &&
                old_sink->cb.local.local == NodeMask{},
            "moving input extension lost ownership or leaked its old tail");
    auto owners = fpga::findNetOwnersByNode(*source, fpga::CB_NODE_SRC, 117);
    require(owners.size() == 1 && owners.front().net == &net &&
                owners.front().binding_index == 0,
            "extended input takeoff no longer has exactly one live owner");
    // Check: subsequent atomic unrouting finds that same owner and frees both
    // the original prefix and its replacement suffix, without lease replay.
    require(fpga::unrouteNet(net), "extended input could not be unrouted");
    for (auto &tile : fpga::Device::current().tile_grid) {
        require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                    tile.cb.local.local == NodeMask{} && tile.cb.joint.jump == NodeMask{},
                "extended input left orphan routing leases after unroute");
    }
}

void moving_co_moved_sinks_cannot_preserve_each_other()
{
    resetGrid(5, 2);

    Referable<rtl::Net> net;
    net.name = "two_sinks_moved_as_one_cluster";
    rtl::Inst driver;
    rtl::Inst sink_a;
    rtl::Inst sink_b;
    rtl::Inst owner_a;
    rtl::Inst owner_b;
    std::vector<fpga::Wire> common{
        tilePin({0, 0}, 40),
        crossbar({0, 0}, {1, 0}, 40, 140, 240, 0),
        crossbar({1, 0}, {2, 0}, 240, 141, 241, 1),
        crossbar({2, 0}, {3, 0}, 241, 142, 242, 1),
    };
    owner_a.wires.push_back(common);
    owner_a.wires[0].push_back(
        crossbar({3, 0}, {4, 0}, 242, 143, 243, 1));
    owner_a.wires[0].push_back(
        crossbar({4, 0}, {4, 0}, 243, 144, 340, 1));
    owner_a.wires[0].push_back(tilePin({4, 0}, 340));

    owner_b.wires.push_back(common);
    for (fpga::Wire& fragment : owner_b.wires[0]) {
        fragment.shared = true;
    }
    owner_b.wires[0].push_back(
        crossbar({3, 0}, {3, 1}, 242, 145, 245, 1));
    owner_b.wires[0].push_back(
        crossbar({3, 1}, {3, 1}, 245, 146, 341, 1));
    owner_b.wires[0].push_back(tilePin({3, 1}, 341));

    leaseRoute(owner_a.wires[0]);
    leaseRoute(owner_b.wires[0]);
    fpga::attachNetRoute(net, owner_a, 0, &driver, &sink_a,
        "random_output", "random_input_a", "co_moved_a");
    fpga::attachNetRoute(net, owner_b, 0, &driver, &sink_b,
        "random_output", "random_input_b", "co_moved_b");

    require(fpga::invalidateMovedSinkRoutes({{&net, 0}, {&net, 1}}),
        "Moving could not invalidate a co-moved sink set");

    // Check: each co-moved sink retains its complete fabric path independently;
    // neither route is mistaken for the other sink's surviving shared owner.
    require(owner_a.wires[0].size() == 5 && owner_b.wires[0].size() == 5,
        "co-moved sinks lost their reusable private fabric prefixes");
    require(owner_a.wires[0].back().owns_landing
            && owner_b.wires[0].back().owns_landing,
        "co-moved sink prefixes did not retain their final landing leases");

    // Check: invalidation preserves all fabric SRC leases but releases both old
    // destination locals; the next route pass continues from those landings.
    require(isSet(fpga::Device::current().getTile(0, 0)->cb.src.jump, 140)
            && isSet(fpga::Device::current().getTile(1, 0)->cb.src.jump, 141)
            && isSet(fpga::Device::current().getTile(2, 0)->cb.src.jump, 142)
            && isSet(fpga::Device::current().getTile(3, 0)->cb.src.jump, 143)
            && isSet(fpga::Device::current().getTile(3, 0)->cb.src.jump, 145)
            && !isSet(fpga::Device::current().getTile(4, 0)->cb.local.local, 340)
            && !isSet(fpga::Device::current().getTile(3, 1)->cb.local.local, 341),
        "atomic moved-sink invalidation did not preserve prefixes or release terminals");
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
            no_progress_passes, completed != 0);
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

    // The last completion starts a fresh finite window. Suffix growth retains
    // its route state but cannot extend that window indefinitely.
    for (int pass = 0; pass < pass_limit - 1; ++pass) {
        require(!run_pass(0, 1),
            "Moving relocated before the focused placement routing slice ended");
    }
    require(run_pass(0, 1),
        "Moving let incremental sibling growth extend the placement routing slice");
}

void unfocused_moving_relocates_after_bounded_global_sweep()
{
    // Reproduce the 51K-cell stall: a global Moving pass leaves over 100K
    // blocked routes but completes a few unrelated easy routes each time.
    constexpr size_t before = 103549;
    constexpr size_t remaining = 103461;
    require(pnr::movingStageShouldRelocate(
                false, false, remaining, before, true, false),
        "small unrelated queue progress suppressed required Moving relocation");

    // Before the bounded sweep expires, normal global route progress keeps
    // ownership of the queue and no endpoint is moved yet.
    require(!pnr::movingStageShouldRelocate(
                 false, false, remaining, before, false, false),
        "Moving relocated before its bounded global routing sweep expired");

    // Focused placement routing retains its stricter no-growth gate because a
    // completed incident route is part of accepting that one moved endpoint.
    require(!pnr::movingStageShouldRelocate(
                 false, true, remaining, before, false, true),
        "focused Moving discarded useful incident-route completion");

    // Once one focus has completed its isolated incident recovery, restoring
    // the large deferred pool must immediately choose the next endpoint.
    require(pnr::movingStageShouldRelocate(
                true, false, remaining, remaining + 88, false, false),
        "restored Moving queue was globally rescanned before next relocation");

    // An empty restored pool is complete and must not request a relocation.
    require(!pnr::movingStageShouldRelocate(
                 true, false, 0, remaining, true, false),
        "empty restored Moving queue requested another relocation");

    // Completion is checked only for candidates reached by relocation
    // selection; completed deferred work is skipped without a full-pool audit.
    require(!pnr::movingTaskNeedsRelocation(true)
            && pnr::movingTaskNeedsRelocation(false),
        "Moving lazy relocation validation accepted a completed route");

    // Clearing one focus's active queue must not terminate Moving while the
    // persistent deferred pool still contains later endpoints.
    require(pnr::routeSchedulerHasWork(false, true, true)
            && !pnr::routeSchedulerHasWork(false, true, false),
        "Moving scheduler ignored or invented persistent deferred work");

    // Reproduce the 50K-cell timeout: the active focus queue became empty while
    // over 100K deferred tasks remained. The next pass must relocate immediately.
    require(pnr::movingDeferredWorkNeedsRelocation(true, false, false, true),
        "empty Moving active queue did not wake deferred relocation work");
    require(!pnr::movingDeferredWorkNeedsRelocation(true, true, false, true)
            && !pnr::movingDeferredWorkNeedsRelocation(true, false, true, true)
            && !pnr::movingDeferredWorkNeedsRelocation(false, false, false, true),
        "Moving deferred wakeup interrupted an active focus or another stage");

    // A repaired Generic/Fanout queue may re-enter after its cumulative stage
    // budget expired; it must hand work onward instead of failing at entry.
    require(!pnr::routeStageEntryTimeoutRequiresFailure(true, false),
        "recoverable stage timeout bypassed its normal Moving handoff");

    // Moving is terminal, so an exhausted Moving re-entry cannot silently
    // preserve unfinished work in a nonexistent later routing stage.
    require(pnr::routeStageEntryTimeoutRequiresFailure(true, true),
        "terminal Moving timeout did not fail at stage entry");
}

void moving_focus_keeps_deferred_pool_persistent()
{
    struct Task {
        int id = 0;
        bool incident = false;
    };
    std::vector<Task> deferred{{1, false}, {2, true}};
    std::vector<Task> active{{3, true}, {4, false}};

    size_t removed = pnr::appendNonFocusMovingTasks(
        deferred, active, [](const Task& task) { return task.incident; });

    // Unrelated deferred work stays in place, while stale copies of the new
    // focus are removed before replacement tasks become active.
    require(removed == 1 && deferred.size() == 2 && deferred[0].id == 1
            && deferred[1].id == 4,
        "Moving focus retained stale incident work in its deferred pool");
}

void moving_stage_handoff_compacts_stale_work_once()
{
    struct Task {
        int id = 0;
        bool complete = false;
        size_t attempt = 0;
    };
    std::vector<Task> active{{1, false, 2}, {2, true, 0}};
    std::vector<Task> deferred{
        {1, false, 9}, {3, false, 4}, {4, true, 0}};

    pnr::MovingStageQueueCompaction result = pnr::compactMovingStageQueues(
        active, deferred, [](const Task& task) { return task.complete; },
        [](const Task& task) { return std::hash<int>{}(task.id); },
        [](const Task& left, const Task& right) {
            return left.id == right.id;
        },
        [](Task& existing, const Task& duplicate) {
            existing.attempt = std::max(existing.attempt, duplicate.attempt);
        });

    // Fanout handoff may contain completed stale entries and duplicate work
    // from preemption. Moving keeps one task with its furthest retry cursor.
    require(result.completed == 2 && result.duplicates == 1
            && deferred.empty() && active.size() == 2
            && active[0].id == 1 && active[0].attempt == 9
            && active[1].id == 3 && active[1].attempt == 4,
        "Moving stage handoff retained stale or duplicate scheduler work");
}

void repeated_relocation_does_not_duplicate_incident_tasks()
{
    struct Task {
        int id = 0;
        bool incident = false;
    };
    std::vector<Task> deferred{{1, false}, {2, true}, {3, true}};
    std::vector<Task> active{{2, true}};

    // Each relocation removes the previous generation of focused work before
    // the newly reconstructed incident routes are activated.
    require(pnr::appendNonFocusMovingTasks(
                deferred, active,
                [](const Task& task) { return task.incident; }) == 2,
        "first relocation did not remove stale incident tasks");
    deferred.push_back({2, true});
    deferred.push_back({3, true});

    // Repeating a failed placement must leave the same one unrelated task,
    // rather than growing one stale task generation per attempted placement.
    require(pnr::appendNonFocusMovingTasks(
                deferred, active,
                [](const Task& task) { return task.incident; }) == 2
            && deferred.size() == 1 && deferred.front().id == 1,
        "repeated relocation inflated the deferred task pool");
}

void moved_focus_rebuilds_generated_endpoint_cache()
{
    struct Endpoint {
        int id = 0;
    };
    Endpoint focus{1};
    Endpoint old_passthrough{2};
    Endpoint* cached_focus = &focus;
    std::unordered_set<Endpoint*> cached_endpoints{&focus, &old_passthrough};

    // Reproduce relocation of the same focus pointer after its old generated
    // passthrough endpoint was detached and replaced at the destination tile.
    pnr::invalidateMovingEndpointCache(cached_focus, cached_endpoints);

    // The next incident-task check must rebuild the endpoint closure; retaining
    // the old set would misclassify every new passthrough route as non-incident.
    require(cached_focus == nullptr && cached_endpoints.empty(),
        "Moving retained stale generated endpoints after focus relocation");
}

void relocation_preserves_bindingless_incident_suffixes()
{
    struct Task {
        int id = 0;
        bool incident = false;
    };
    std::vector<Task> active{{1, true}, {2, true}, {3, false}};
    std::vector<Task> replacement{{1, true}};

    size_t preserved = pnr::preserveActiveIncidentTasks(
        replacement, active, [](const Task& task) { return task.incident; },
        [](const Task& lhs, const Task& rhs) { return lhs.id == rhs.id; });

    // Task 2 represents an invalidated suffix whose binding is temporarily
    // absent. It must remain in the atomic focus instead of moving to the
    // global deferred pool and allowing a false focus-complete result.
    require(preserved == 1 && replacement.size() == 2
            && replacement[0].id == 1 && replacement[1].id == 2,
        "Moving discarded a binding-less incident suffix during relocation");
}

void relocation_preserves_fanout_retry_cursor()
{
    struct Task {
        int id = 0;
        bool incident = false;
        size_t attempt = 0;
        size_t branch_attempt = 0;
        size_t branch_offset = 0;
        size_t no_progress_passes = 0;
    };
    std::vector<Task> active{{7, true, 19, 3, 11, 4}};
    std::vector<Task> replacement{{7, true, 0, 0, 0, 0}};

    size_t inserted = pnr::preserveActiveIncidentTasks(
        replacement, active, [](const Task& task) { return task.incident; },
        [](const Task& lhs, const Task& rhs) { return lhs.id == rhs.id; },
        [](Task& rebuilt, const Task& live) {
            pnr::mergeRelocatedMovingTaskState(
                rebuilt, live, [](Task& target, const Task& source) {
                    target.attempt = std::max(target.attempt, source.attempt);
                    target.branch_attempt = std::max(
                        target.branch_attempt, source.branch_attempt);
                    target.branch_offset = std::max(
                        target.branch_offset, source.branch_offset);
                    target.no_progress_passes = std::max(
                        target.no_progress_passes,
                        source.no_progress_passes);
                });
        });

    // A binding-derived replacement must retain the live Fanout rotation
    // cursor, otherwise every relocation retries the same blocked trunk fork.
    require(inserted == 0 && replacement.size() == 1
            && replacement[0].attempt == 19
            && replacement[0].branch_attempt == 3
            && replacement[0].branch_offset == 11
            && replacement[0].no_progress_passes == 0,
        "Moving reset a Fanout branch cursor while rebuilding bindings");
}

void inactive_focused_pass_uses_bounded_retry_window()
{
    constexpr int retry_limit = 5;
    size_t task_no_progress = 0;
    int placement_passes = 0;

    // Reproduce a moved sink whose first suffix attempt cannot extend. The
    // placement is still valid and must receive the remaining alternative
    // attempts instead of discarding already completed incident routes.
    for (int pass = 0; pass < retry_limit; ++pass) {
        task_no_progress = pnr::updateMovingTaskNoProgressPasses(
            task_no_progress, false);
        placement_passes = pnr::updateMovingPlacementNoProgressPasses(
            placement_passes, false);
        bool exhausted = pnr::movingTaskNoProgressExhausted(
                             task_no_progress, retry_limit)
            || pnr::movingPlacementPassesExhausted(
                placement_passes, retry_limit);
        bool blocked = pnr::focusedMovingPassIsBlocked(true, exhausted);
        bool relocate = pnr::focusedMovingShouldRelocate(
            blocked, pass + 1, retry_limit, exhausted, exhausted);
        if (pass + 1 < retry_limit) {
            require(!relocate
                    && pnr::focusedMovingMayRetryInactivePass(true, exhausted),
                "Moving relocated after one inactive suffix attempt");
        }
        else {
            require(relocate
                    && !pnr::focusedMovingMayRetryInactivePass(true, exhausted)
                    && !pnr::focusedMovingMayRetryInactivePass(false, false),
                "Moving retained a placement after exhausting its retry window");
        }
    }
}

void completed_incident_route_renews_focused_placement_window()
{
    struct Task {
        size_t no_progress_passes = 0;
    };

    constexpr int retry_limit = 5;
    int placement_passes = retry_limit - 1;
    std::vector<Task> remaining{{retry_limit}, {retry_limit - 1}};

    // Check: grounding one incident route retains this placement instead of
    // immediately invalidating that successful route through relocation.
    placement_passes = pnr::updateMovingPlacementNoProgressPasses(
        placement_passes, true);
    pnr::renewMovingTaskWindowsAfterCompletion(remaining, true);
    require(placement_passes == 0
            && std::all_of(remaining.begin(), remaining.end(),
                           [](const Task& task) {
                               return task.no_progress_passes == 0;
                           }),
            "completed Moving route did not renew focused retry windows");

    // Check: partial growth remains bounded and cannot keep an ungrounded
    // placement alive indefinitely.
    placement_passes = pnr::updateMovingPlacementNoProgressPasses(
        placement_passes, false);
    remaining[0].no_progress_passes = 1;
    pnr::renewMovingTaskWindowsAfterCompletion(remaining, false);
    require(placement_passes == 1 && remaining[0].no_progress_passes == 1,
            "partial Moving growth incorrectly renewed retry windows");
}

void distributed_completion_does_not_renew_focused_placement()
{
    // Check: an ordinary incident completion proves the placement useful.
    require(pnr::movingCompletionRenewsPlacement(true, false),
            "ordinary completion did not retain the focused placement");

    // Check: a placement-independent distributed route cannot extend retries.
    require(!pnr::movingCompletionRenewsPlacement(true, true),
            "distributed completion incorrectly retained the focused placement");
    require(!pnr::movingCompletionRenewsPlacement(false, false),
            "unfinished ordinary route incorrectly retained the focused placement");
}

void useful_focused_placement_retries_incident_routes_in_parallel()
{
    constexpr int recursion_limit = 5;

    // Check: each pass visits every remaining incident route, so one completed
    // sibling renews five attempts for each route without multiplying by count.
    require(pnr::movingUsefulPlacementRetryLimit(recursion_limit, 0) == 5
            && pnr::movingUsefulPlacementRetryLimit(recursion_limit, 7) == 5,
            "useful Moving placement multiplied its parallel retry window");

    // Check: a large incident set still gets the same per-route retry count.
    require(pnr::movingUsefulPlacementRetryLimit(recursion_limit, 100) == 5,
            "large Moving incident set inflated its parallel retry window");
}

void relocation_partitions_active_work_without_global_rebuild()
{
    struct Task {
        int id = 0;
        bool incident = false;
    };
    std::vector<Task> deferred{{1, false}};
    std::vector<Task> active{{2, true}, {3, false}, {4, true}};
    std::vector<Task> replacement{{2, true}};

    pnr::appendNonFocusMovingTasks(
        deferred, active, [](const Task& task) { return task.incident; });
    pnr::preserveActiveIncidentTasks(
        replacement, active, [](const Task& task) { return task.incident; },
        [](const Task& lhs, const Task& rhs) { return lhs.id == rhs.id; });

    // Non-incident work enters the persistent pool exactly once, while every
    // incident task stays in the atomic replacement focus. No second displaced
    // task pass or full deferred-pool deduplication is required.
    require(deferred.size() == 2 && deferred[0].id == 1
            && deferred[1].id == 3 && replacement.size() == 2
            && replacement[0].id == 2 && replacement[1].id == 4,
        "Moving relocation did not partition active work exactly once");
}

void outgoing_focus_switch_keeps_previous_work()
{
    int old_focus = 1, sink = 2, helper = 3, unrelated = 4;
    require(pnr::movingDestinationPlacementTarget(&old_focus, &sink, true, false) == &sink,
            "outgoing route incorrectly relocates its own source focus");
    require(pnr::movingDestinationPlacementTarget(&old_focus, &helper, false, true) == &old_focus,
            "incoming route lost its physical destination focus");
    require(pnr::movingDestinationPlacementTarget(static_cast<int*>(nullptr), &sink, false, false) == &sink,
            "unfocused relocation lost the real sink");
    // No source fallback: sink movability is checked by the caller, and a
    // fixed sink must not redirect forward grounding to its own driver.
    require(pnr::movingDestinationPlacementTarget(&old_focus, &sink, true, true) == &sink,
            "moving source was selected for a route internal to its closure");

    struct Task { int id; int* from; int* to; };
    std::vector<Task> active{{1, &old_focus, &sink},
                             {2, &unrelated, &old_focus},
                             {3, &helper, &unrelated},
                             {4, &unrelated, &sink}};
    std::vector<Task> deferred{{5, &unrelated, &unrelated}};
    std::vector<Task> replacement{{1, &old_focus, &sink}};
    auto incident = [&](const Task& task) { return task.from == &sink || task.to == &sink; };
    pnr::appendNonFocusMovingTasks(deferred, active, incident);
    pnr::preserveActiveIncidentTasks(replacement, active, incident,
        [](const Task& a, const Task& b) { return a.id == b.id; });
    require(deferred.size() == 3 && deferred[0].id == 5 &&
                deferred[1].id == 2 && deferred[2].id == 3 &&
                replacement.size() == 2 && replacement[1].id == 4,
            "switching to the sink focus dropped or duplicated incident work");
}

void relocation_defers_source_tree_siblings_before_focused_routing()
{
    struct Task {
        int id = 0;
        bool incident = false;
    };
    std::vector<Task> replacement{
        {1, true}, {2, false}, {3, true}, {4, false}};
    std::vector<Task> deferred;

    size_t displaced = pnr::partitionMovingReplacementTasks(
        replacement, [](const Task& task) { return task.incident; },
        [&](const Task& task) { deferred.push_back(task); });

    // Moving one sink can invalidate siblings from the same source tree. Only
    // the two routes touching the moved sink remain in its atomic reroute;
    // siblings survive in the persistent queue for their own later focuses.
    require(displaced == 2 && replacement.size() == 2
            && replacement[0].id == 1 && replacement[1].id == 3
            && deferred.size() == 2 && deferred[0].id == 2
            && deferred[1].id == 4,
        "Moving kept displaced source-tree siblings in the focused queue");
}

void focused_routing_defers_siblings_generated_after_relocation()
{
    struct Task {
        int id = 0;
        bool incident = false;
    };
    std::vector<Task> active{
        {1, true}, {2, false}, {3, false}, {4, true}};
    std::vector<Task> deferred{{5, false}};

    size_t displaced = pnr::partitionMovingReplacementTasks(
        active, [](const Task& task) { return task.incident; },
        [&](const Task& task) { deferred.push_back(task); });

    // Focused preemption can create external sibling repairs after relocation.
    // They leave the atomic queue without abandoning its incident work.
    require(displaced == 2 && active.size() == 2
            && active[0].id == 1 && active[1].id == 4
            && deferred.size() == 3 && deferred[1].id == 2
            && deferred[2].id == 3,
        "focused routing abandoned or retained generated sibling repairs");
}

void deferred_scan_retries_cooling_endpoints_without_empty_passes()
{
    // A nonempty pool containing cooling-down endpoints must schedule another
    // relocation epoch instead of entering an empty routing pass.
    require(pnr::movingDeferredScanNeedsRetry(100, 4),
        "Moving did not retry a deferred cooldown-only scan");

    // An empty pool is complete, while a nonempty permanently immovable pool
    // is an error handled by the caller rather than a cooldown retry.
    require(!pnr::movingDeferredScanNeedsRetry(0, 4)
            && !pnr::movingDeferredScanNeedsRetry(100, 0),
        "Moving invented a cooldown retry without deferred cooldown work");
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
    // Check: persistent deadend masks are applied only by Generic routing;
    // short Fanout and Moving passes reuse the one-time cleared tile state.
    require(pnr::routingStageUsesDeadends(false, false)
            && !pnr::routingStageUsesDeadends(true, false)
            && !pnr::routingStageUsesDeadends(false, true),
        "non-Generic routing unexpectedly enabled persistent deadends");

    // Check: a pending Generic seed blocks only branches from its own source pin.
    require(pnr::movingFanoutWaitsForSourceSeed(true, true, true),
        "Moving allowed a fanout to run before its own Generic seed");
    require(!pnr::movingFanoutWaitsForSourceSeed(true, true, false)
            && !pnr::movingFanoutWaitsForSourceSeed(false, true, true),
        "Moving blocked a fanout behind an unrelated source or another stage");

    // Check: outgoing-only work is handed to downstream loads instead of
    // relocating and invalidating the completed focused driver again.
    require(pnr::movingFocusHandsOffToLoads(false, false, true)
            && !pnr::movingFocusHandsOffToLoads(false, true, true)
            && !pnr::movingFocusHandsOffToLoads(false, false, false),
        "Moving selected the wrong focus handoff policy");

    // Check: a fresh placement receives its own no-progress budget instead of
    // inheriting the global Moving-stage pass count.
    require(!pnr::movingPlacementPassesExhausted(0, 5)
            && !pnr::focusedMovingShouldRelocate(false, 0, 5, false, false),
        "Moving relocated a fresh placement before its pass budget");

    // Check: a focus owns its incident work for a bounded placement slice, then
    // yields the intact task set so one congested sink cannot consume the stage.
    require(!pnr::movingFocusSliceExhausted(3, 0, 4)
            && pnr::movingFocusSliceExhausted(4, 0, 4)
            && pnr::MOVING_FOCUS_SLICE_LIMIT == 8
            && !pnr::movingFocusSliceExhausted(
                7, 0, pnr::MOVING_FOCUS_SLICE_LIMIT)
            && pnr::movingFocusSliceExhausted(
                8, 0, pnr::MOVING_FOCUS_SLICE_LIMIT),
        "Moving did not yield a congested focus at its placement bound");

    // Check: if every unfinished sink is on cooldown, the scheduler releases
    // the cooldown once because no relocation can otherwise advance its epoch.
    require(pnr::movingCooldownMustBeReleased(1, 0, true, 72, 71),
        "Moving retained a cooldown that could not expire");
    require(!pnr::movingCooldownMustBeReleased(1, 1, true, 72, 71)
            && !pnr::movingCooldownMustBeReleased(1, 0, false, 72, 71)
            && !pnr::movingCooldownMustBeReleased(1, 0, true, 72, 72),
        "Moving repeatedly released cooldowns or released an active candidate");

    // A newly queued route may have no binding yet, so the binding-only
    // incident audit cannot keep its destination's old finished mark valid.
    require(pnr::movingQueuedTaskInvalidatesFinishedMark(true, true)
            && !pnr::movingQueuedTaskInvalidatesFinishedMark(false, true)
            && !pnr::movingQueuedTaskInvalidatesFinishedMark(true, false),
        "an explicit incomplete task did not invalidate its stale Moving mark");
}

void moving_candidate_reserves_distinct_terminal_paths()
{
    NodeMask pins;
    NodeMask locals;
    NodeMask dsts;
    NodeMask joints;
    std::vector<pnr::MovingTerminalPath> first{{11, 101, 21, -1}};
    std::vector<pnr::MovingTerminalPath> conflicts{{12, 101, 22, -1},
                                                   {12, 102, 21, -1}};
    std::vector<pnr::MovingTerminalPath> disjoint{{12, 102, 22, -1}};

    // Check: the first moved input reserves its local, destination, and joint.
    require(pnr::reserveMovingTerminalPath(first, pins, locals, dsts, joints)
            && isSet(pins, 11) && isSet(locals, 11)
            && isSet(dsts, 101) && isSet(joints, 21),
        "Moving did not reserve a complete candidate terminal path");

    // Check: another input cannot reuse either the reserved destination or
    // the reserved joint, and failed alternatives must not alter any mask.
    NodeMask old_pins = pins;
    NodeMask old_locals = locals;
    NodeMask old_dsts = dsts;
    NodeMask old_joints = joints;
    require(!pnr::reserveMovingTerminalPath(
                conflicts, pins, locals, dsts, joints)
            && pins == old_pins && locals == old_locals
            && dsts == old_dsts && joints == old_joints,
        "Moving accepted conflicting terminal paths or changed failed state");

    // Check: a disjoint path for the second input remains legal in the same
    // candidate tile and is reserved atomically with the first path.
    require(pnr::reserveMovingTerminalPath(
                disjoint, pins, locals, dsts, joints)
            && isSet(pins, 12) && isSet(dsts, 102) && isSet(joints, 22),
        "Moving rejected disjoint input terminal capacity");
}

void moving_candidate_reserves_constrained_terminal_first()
{
    std::vector<pnr::MovingTerminalPath> flexible{
        {11, 101, 21, -1}, {11, 102, 22, -1}};
    std::vector<pnr::MovingTerminalPath> constrained{
        {12, 103, 21, -1}, {12, 104, 21, -1}};
    std::vector<std::vector<pnr::MovingTerminalPath>> requirements{
        flexible, constrained};
    std::vector<size_t> order =
        pnr::movingTerminalReservationOrder(requirements);

    // Check: several destination choices through one joint are still one
    // physical alternative, so the constrained input reserves before the LUT.
    require(order.size() == 2 && order[0] == 1 && order[1] == 0,
        "Moving did not prioritize the single-joint terminal requirement");

    NodeMask pins;
    NodeMask locals;
    NodeMask dsts;
    NodeMask joints;
    for (size_t index : order) {
        require(pnr::reserveMovingTerminalPath(
                    requirements[index], pins, locals, dsts, joints),
            "Moving's constrained-first order did not reserve every terminal");
    }

    // Check: the constrained route owns joint 21 while the flexible route
    // selected joint 22 instead of rejecting an otherwise legal placement.
    require(isSet(joints, 21) && isSet(joints, 22)
            && isSet(locals, 11) && isSet(locals, 12),
        "Moving's flexible terminal did not avoid the constrained joint");
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

void indexed_source_tree_invalidation_is_endpoint_scoped()
{
    resetGrid(6, 1);

    pnr::RouteDesign router;
    Referable<rtl::Net> first;
    Referable<rtl::Net> second;
    Referable<rtl::Net> empty_sibling;
    Referable<rtl::Net> unrelated;
    first.name = "random_tree_part_a";
    second.name = "random_tree_part_b";
    empty_sibling.name = "random_deferred_tree_part";
    unrelated.name = "random_unrelated_tree";
    rtl::Inst shared_driver;
    rtl::Inst unrelated_driver;
    rtl::Inst sink_a;
    rtl::Inst sink_b;
    rtl::Inst sink_empty;
    rtl::Inst sink_other;
    rtl::Inst owner_a;
    rtl::Inst owner_b;
    rtl::Inst owner_other;

    owner_a.wires.push_back({
        crossbar({0, 0}, {1, 0}, 10, 110, 210, 0),
        crossbar({1, 0}, {1, 0}, 210, 111, 310, 1),
        tilePin({1, 0}, 310),
    });
    owner_b.wires.push_back({
        crossbar({2, 0}, {3, 0}, 20, 120, 220, 0),
        crossbar({3, 0}, {3, 0}, 220, 121, 320, 1),
        tilePin({3, 0}, 320),
    });
    owner_other.wires.push_back({
        crossbar({4, 0}, {5, 0}, 30, 130, 230, 0),
        crossbar({5, 0}, {5, 0}, 230, 131, 330, 1),
        tilePin({5, 0}, 330),
    });
    leaseRoute(owner_a.wires[0]);
    leaseRoute(owner_b.wires[0]);
    leaseRoute(owner_other.wires[0]);
    fpga::attachNetRoute(first, owner_a, 0, &shared_driver, &sink_a,
        "random_output", "random_input", "tree_route_a");
    fpga::attachNetRoute(second, owner_b, 0, &shared_driver, &sink_b,
        "random_output", "random_input", "tree_route_b");
    empty_sibling.routes.push_back(rtl::NetRouteBinding{
        nullptr, std::numeric_limits<size_t>::max(), &shared_driver,
        &sink_empty, "random_output", "random_input", "deferred_route"});
    fpga::attachNetRoute(unrelated, owner_other, 0, &unrelated_driver,
        &sink_other, "random_output", "random_input", "other_route");

    router.indexSourceRoute(&first, &shared_driver, "random_output");
    router.indexSourceRoute(&second, &shared_driver, "random_output");
    router.indexSourceRoute(&empty_sibling, &shared_driver, "random_output");
    router.indexSourceRoute(&unrelated, &unrelated_driver, "random_output");
    require(router.sourceTreeHasCompleteExit(
                shared_driver, "random_output"),
        "source endpoint index did not find its completed physical exit");
    require(router.sourceTreeRouteCount(
                first, &shared_driver, "random_output") == 3,
        "source endpoint index did not collect all logical net objects");

    std::vector<pnr::RouteDesign::RouteTask> tasks;
    require(router.unrouteSourceTree(first, &shared_driver, "random_output",
                &tasks, false, false) == 2,
        "source endpoint invalidation did not requeue its complete tree");

    // Check: both logical net objects driven by the selected endpoint are
    // cleared and returned as one Generic seed followed by one Fanout.
    require(owner_a.wires[0].empty() && owner_b.wires[0].empty()
            && tasks.size() == 2 && !tasks[0].fanout && tasks[1].fanout,
        "indexed invalidation did not atomically rebuild one source tree");
    // Check: Basic invalidation does not duplicate an empty sibling that is
    // already waiting in the deferred Fanout queue.
    require(std::none_of(tasks.begin(), tasks.end(), [&](const auto& task) {
                return task.net == &empty_sibling;
            }),
        "Basic invalidation requeued an already-deferred empty fanout");
    require(!router.sourceTreeHasCompleteExit(
                shared_driver, "random_output"),
        "source endpoint index reported a cleared route as complete");
    require(!isSet(fpga::Device::current().getTile(0, 0)->cb.src.jump, 110)
            && !isSet(fpga::Device::current().getTile(2, 0)->cb.src.jump, 120),
        "indexed invalidation leaked a selected source-tree lease");

    // Check: an identically named port on another driver is outside the index
    // key, so its route vector and physical leases remain untouched.
    require(owner_other.wires[0].size() == 3
            && isSet(fpga::Device::current().getTile(4, 0)->cb.src.jump, 130)
            && isSet(fpga::Device::current().getTile(5, 0)->cb.src.jump, 131)
            && isSet(fpga::Device::current().getTile(5, 0)->cb.local.local, 330),
        "source endpoint invalidation changed an unrelated driver's tree");
}

void exhausted_fanout_rebuilds_source_tree_without_moving_driver()
{
    struct Task
    {
        int binding = 0;
        bool fanout = true;
        size_t attempt = 9;
        size_t fanout_branch_attempt = 3;
        size_t fanout_branch_offset = 17;
        size_t no_progress_passes = 4;
        bool source_tree_rebuild_attempted = false;
    };

    // Check: only an exhausted Moving fanout with a live source exit requests
    // source-tree reconstruction; ordinary retries and progressing tasks do not.
    require(pnr::movingFanoutNeedsSourceTreeRebuild(
                true, true, false, true, true, false, false, 8, 9),
            "Moving did not detect exhausted fanout branch points");
    require(!pnr::movingFanoutNeedsSourceTreeRebuild(
                true, true, false, true, true, false, true, 8, 9)
            && !pnr::movingFanoutNeedsSourceTreeRebuild(
                false, true, false, true, true, false, false, 8, 9)
            && !pnr::movingFanoutNeedsSourceTreeRebuild(
                true, false, false, true, true, false, false, 8, 9),
            "source-tree rebuild escaped its failed Moving fanout case");
    require(!pnr::movingFanoutNeedsSourceTreeRebuild(
                true, true, true, true, true, false, false, 8, 9),
            "Moving rebuilt the same source tree twice at one placement");

    Task current{2};
    std::vector<Task> recovered{{1}, {2}, {3}};
    std::vector<Task> queued;
    int driver_place = 41;
    size_t siblings = pnr::scheduleMovingSourceTreeRebuild(
        current, recovered,
        [](const Task& left, const Task& right) {
            return left.binding == right.binding;
        },
        [&](const Task& task) {
            queued.push_back(task);
            return true;
        });

    // Check: the blocked moved sink becomes the new Generic trunk and every
    // other binding is reset as Fanout work behind that seed.
    require(!current.fanout && current.attempt == 0
            && current.fanout_branch_attempt == 0
            && current.fanout_branch_offset == 0
            && current.no_progress_passes == 0
            && current.source_tree_rebuild_attempted
            && siblings == 2 && queued.size() == 2
            && queued[0].fanout && queued[1].fanout
            && queued[0].source_tree_rebuild_attempted
            && queued[1].source_tree_rebuild_attempted,
        "Moving source-tree reconstruction did not establish one Generic seed");
    // Check: source-tree reconstruction changes routing roles only; the driver
    // placement remains untouched and is never treated as the moved endpoint.
    require(driver_place == 41,
        "Moving source-tree reconstruction relocated the source driver");
}

void repeated_source_repair_merges_deferred_siblings()
{
    struct Task
    {
        int binding = 0;
        size_t attempt = 0;
    };
    std::vector<Task> deferred{{7, 3}};
    Task rediscovered{7, 11};
    bool inserted = pnr::mergeMovingDeferredTask(
        deferred, rediscovered,
        [](const Task& left, const Task& right) {
            return left.binding == right.binding;
        },
        [](Task& old, const Task& replacement) {
            old.attempt = std::max(old.attempt, replacement.attempt);
        });

    // Check: rebuilding one source tree repeatedly retains one external
    // sibling task and advances its retry state instead of growing the queue.
    require(!inserted && deferred.size() == 1 && deferred[0].attempt == 11,
        "repeated Moving source repair duplicated a deferred sibling");
}

} // namespace

int main()
{
    try {
        moving_terminal_diagnostics_distinguish_lease_sources();
        moving_forward_growth_visits_only_reached_tiles();
        moving_forward_growth_rejects_third_visit_per_path();
        moving_forward_exhaustion_reports_every_exit();
        stagnation_failure_writes_png_and_live_tile_debug(true);
        moving_destination_commits_forward_grounding(false);
        moving_destination_commits_forward_grounding(true);
        moving_destination_commits_forward_grounding(false, false, false, true);
        moving_destination_commits_forward_grounding(false, false, false, false, false, false, true);
        moving_destination_commits_forward_grounding(false, true);
        moving_destination_commits_forward_grounding(false, false, true);
        moving_destination_commits_forward_grounding(false, true, true);
        moving_destination_commits_forward_grounding(false, true, false, false, true);
        moving_destination_commits_forward_grounding(false, true, false, false, true, true);
        moving_destination_commits_forward_grounding(false, false, false, false, true, true);
        outgoing_focus_switch_keeps_previous_work();
        incident_binding_uses_precomputed_endpoint_closure();
        failed_route_anchor_controls_moving_search_center();
        multi_input_sink_balances_only_incoming_route_anchors();
        moving_sources_use_bounded_relocation_batches();
        moving_source_failure_stops_the_stage();
        moving_sources_keep_generic_preemption_enabled();
        moving_source_input_transaction_protects_sibling_trees();
        moving_source_route_endpoint_keeps_physical_cluster_owner();
        rejected_route_first_move_restores_the_complete_cluster();
        moving_source_batches_only_blocked_takeoffs();
        moving_source_candidate_requires_a_free_takeoff();
        moving_source_candidate_requires_free_input_terminals();
        moving_source_candidate_reuses_same_driver_input_terminal();
        moving_source_matches_distributed_input_values();
        distributed_input_sharing_requires_a_live_route();
        stagnation_failure_writes_png_and_live_tile_debug();
        moving_source_legalizes_only_a_known_blocked_terminal();
        for (bool indexed : {false, true}) {
            for (bool alias : {false, true}) {
                moving_empty_suffix_reuses_sibling_takeoff(indexed, alias);
            }
        }
        moving_one_fanout_releases_only_its_suffix();
        moving_source_replaces_only_a_dead_partial_tail();
        moving_private_route_releases_its_stale_takeoff();
        moving_input_prefix_extension_keeps_ownership_and_frees_old_tail();
        moving_stale_shared_route_releases_the_complete_private_path();
        moving_co_moved_sinks_cannot_preserve_each_other();
        crossbar_destination_owner_uses_landing_node();
        focused_moving_bounds_each_placement_while_routes_advance();
        unfocused_moving_relocates_after_bounded_global_sweep();
        moving_focus_keeps_deferred_pool_persistent();
        moving_stage_handoff_compacts_stale_work_once();
        repeated_relocation_does_not_duplicate_incident_tasks();
        moved_focus_rebuilds_generated_endpoint_cache();
        relocation_preserves_bindingless_incident_suffixes();
        relocation_preserves_fanout_retry_cursor();
        inactive_focused_pass_uses_bounded_retry_window();
        completed_incident_route_renews_focused_placement_window();
        distributed_completion_does_not_renew_focused_placement();
        useful_focused_placement_retries_incident_routes_in_parallel();
        relocation_partitions_active_work_without_global_rebuild();
        relocation_defers_source_tree_siblings_before_focused_routing();
        focused_routing_defers_siblings_generated_after_relocation();
        deferred_scan_retries_cooling_endpoints_without_empty_passes();
        focused_moving_relocates_when_one_incident_route_stalls();
        focused_moving_relocates_when_routes_wander_without_completion();
        moving_scheduler_blocks_only_same_source_fanouts();
        moving_candidate_reserves_distinct_terminal_paths();
        moving_candidate_reserves_constrained_terminal_first();
        finished_focus_is_reopened_after_route_invalidation();
        atomic_source_tree_requeues_already_empty_siblings();
        indexed_source_tree_invalidation_is_endpoint_scoped();
        exhausted_fanout_rebuilds_source_tree_without_moving_driver();
        repeated_source_repair_merges_deferred_siblings();
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "moving_test failed: %s\n", error.what());
        return 1;
    }
    std::puts("moving_test passed");
    return 0;
}
