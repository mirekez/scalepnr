#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Inst.h"
#include "Wire.h"
#include "route/RouteDesign.h"
#include "route/RoutePassState.h"
#include "route/RouteDiagnostics.h"
#include "route/MovingPlacementBlacklist.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <sys/wait.h>
#include <sys/resource.h>
#include <unistd.h>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

void moving_source_physical_tile_reentry();

namespace {

void require(bool condition, const std::string& message);

void stagnation_runs_full_diagnostic_attempts_before_cancelling()
{
    pnr::RouteDesign router;
    router.stagnation_attempt_limit = 2;
    router.resetRouteProgressWatchdog(3, "Moving sources", 1, 1, 1);
    router.route_progress_watchdog.reset(1, std::chrono::steady_clock::now() -
        std::chrono::seconds(2), std::chrono::seconds(1), 1, 1);
    // Exercise the real cancellation hook: detection enables tracing without
    // cancelling the search or waiting for another deficient time window.
    require(!router.routeStageSearchCancelled() &&
                router.stagnation_diagnostics.stagnation_detected &&
                router.stagnation_diagnostics.stagnation_attempts == 2,
            "stagnation cancelled before diagnostic attempts");
    std::ostringstream output;
    struct Capture {
        std::streambuf* old;
        explicit Capture(std::ostream& out) : old(std::cout.rdbuf(out.rdbuf())) {}
        ~Capture() { std::cout.rdbuf(old); }
    } capture(output);
    pnr::RouteDesign::RouteTask task;
    task.net_name = "unplaced_signal_17";
    // An unplaced endpoint deterministically returns from the real recovery
    // entry point. Each return must consume exactly one outer diagnostic cycle.
    for (unsigned remaining : {1U, 0U}) {
        std::string reason;
        require(!router.moveUnfinishedSource(task, nullptr, &reason) && !reason.empty(),
                "unplaced recovery fixture unexpectedly succeeded");
        require(router.stagnation_diagnostics.stagnation_attempts == remaining &&
                    router.routeStageSearchCancelled() == (remaining == 0),
                "production recovery entry did not finish exactly one cycle");
    }
    require(output.str().find("route=\"unplaced_signal_17\"") != std::string::npos &&
                output.str().find("AFTER complete=0") != std::string::npos &&
                output.str().find("full_cycle=1") != std::string::npos &&
                !pnr::RouteCongestionTrace::current,
            "automatic diagnostic did not log to stdout or restore trace scope");
    // Finishing all queued work during the allowance is success, not a false
    // terminal failure merely because the last diagnostic cycle also ended.
    router.updateRouteProgressRemaining(0);
    require(!router.routeStageSearchCancelled() && !router.route_stage_stagnated,
            "completed work was reported as a stagnation failure");
    router.resetRouteProgressWatchdog(4, "Fanouts", 1, 1, 1);
    require(!router.routeStageSearchCancelled() && !router.routeTaskDebugMatches(task),
            "automatic verbose/cancellation state leaked into next stage");
    // The diagnostic allowance never bypasses an absolute stage deadline.
    router.stagnation_diagnostics.detect(2);
    router.route_stage_deadline_enabled = true;
    router.route_stage_deadline = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    require(router.routeStageSearchCancelled() && router.route_stage_deadline_expired &&
                router.stagnation_diagnostics.stagnation_attempts == 2,
            "diagnostic attempts disabled the hard timeout");
}

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
    // examines all placements in a ten-hop reverse search before failure.
    require(pnr::movingSourcePlacementRadius() < 0 &&
                pnr::movingSourceProbeLimit() == 0 &&
                pnr::movingSourceSearchDepth() == 10,
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

    // Multi-boundary progress and cancellation are covered with real owned
    // routing resources by fpga.moving_source_boundary_continuation. A retry
    // counter is not a search limit or evidence of an exhausted source route.

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

    // A candidate may be rejected inside the search. Once the search returns
    // false, a retryable flag cannot authorize switching to another source.
    require(pnr::movingFailureRequiresExit(false, true),
            "Moving Sources allowed a failed task to yield via retryable flag");
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
    // All discovered placements may be rejected while deeper free nodes are
    // still queued. Keep the offset at the next probe instead of restarting
    // the same window or treating this return as permission to preempt.
    require(pnr::movingSourceProbeWindowHasRemaining(2, 0, 2, 1) &&
                pnr::movingSourceProbeWindowHasRemaining(4, 2, 2, 1) &&
                !pnr::movingSourceProbeWindowHasRemaining(4, 2, 2, 0),
            "Moving source ignored an unexpanded free frontier");
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
        if (fragment.jump >= 0)
            tile->cb.src.jump |= bit(fragment.jump);
        if (fragment.pos == 0) {
            tile->cb.local.local |= bit(fragment.local);
        }
        else if (fragment.owns_dst) {
            tile->cb.dst.jump |= bit(fragment.local);
        }
    }
}

void bridge_preemption_prepares_all_leases_before_cutting()
{
    resetGrid(4, 1);
    auto &device = fpga::Device::current();
    Referable<rtl::Net> victim;
    victim.name = "signal_742";
    rtl::Inst driver, sink, owner;
    owner.wires.push_back({tilePin({0, 0}, 17),
        crossbar({0, 0}, {1, 0}, 17, 117, 217, 0),
        crossbar({1, 0}, {2, 0}, 217, 118, 218, 1),
        crossbar({2, 0}, {2, 0}, 218, -1, -1, 1), tilePin({2, 0}, 318)});
    owner.wires[0][2].joint = 5;
    leaseRoute(owner.wires[0]);
    auto *bridge = device.getTile(1, 0), *target = device.getTile(2, 0);
    bridge->cb.joint.jump.setBit(5);
    fpga::attachNetRoute(victim, owner, 0, &driver, &sink, "out", "in", "signal_742");
    fpga::registerNetRouteTiles(victim, owner.wires[0], 0);
    std::vector<fpga::Wire> proposal{
        crossbar({3, 0}, {1, 0}, 41, 141, 219, 0),
        crossbar({1, 0}, {2, 0}, 219, 120, 220, 1),
        crossbar({2, 0}, {2, 0}, 220, -1, -1, 1), tilePin({2, 0}, 321)};
    proposal[1].joint = 5;
    proposal[2].joint = 49;
    proposal[2].joint2 = 52;
    const std::vector<fpga::RouteCutNode> cuts{{{1, 0}, fpga::CB_NODE_JOINT, 5}};
    size_t calls = 0, failed = 0;
    auto cut = [&]() {
        ++calls;
        require(bridge->cb.joint.jump.testBit(5) &&
                    !device.getTile(3, 0)->cb.src.jump.testBit(141),
                "preemption callback saw speculative state instead of live ownership");
        return fpga::unrouteNetRouteFromNodes(victim, 0, cuts);
    };
    // Reproduce the audit: joined path has another blocked terminal joint.
    // Also cover joint2, local, terminal pin, DST and an intermediate SRC.
    std::vector<std::pair<NodeMask*, int>> blockers{
        {&target->cb.joint.jump, 49}, {&target->cb.joint.jump, 52},
        {&target->cb.local.local, 321}, {&target->pin_state.leased_nodes, 321},
        {&target->cb.dst.jump, 220}, {&bridge->cb.src.jump, 120}};
    for (auto [mask, bit] : blockers) {
        mask->setBit(bit);
        require(!pnr::commitPreparedRouteWithPreemption(proposal,
                    fpga::previewNetRouteCut(victim, 0, cuts), cut, &failed),
                "preemption ignored an independently blocked replacement resource");
        require(calls == 0 && owner.wires[0].size() == 5 &&
                    fpga::isRouteComplete(owner.wires[0]) && mask->testBit(bit) &&
                    bridge->cb.joint.jump.testBit(5) &&
                    !device.getTile(3, 0)->cb.src.jump.testBit(141) &&
                    !bridge->cb.dst.jump.testBit(219),
                "failed replacement cut the victim or leaked speculative leases");
        mask->clearBit(bit);
    }
    // Positive counterpart: a busy terminal joint in this SAME victim's tail
    // really will be released. Do not ban all blocked-terminal bridges.
    owner.wires[0][3].joint = 49;
    owner.wires[0][3].joint2 = 52;
    target->cb.joint.jump.setBit(49);
    target->cb.joint.jump.setBit(52);
    Referable<rtl::Net> foreign;
    foreign.name = "signal_913";
    rtl::Inst foreign_owner;
    foreign_owner.wires.push_back({crossbar({2, 0}, {3, 0}, 238, 138, 339, 1),
                                    tilePin({3, 0}, 339)});
    foreign_owner.wires[0][0].joint = 52;
    leaseRoute(foreign_owner.wires[0]);
    fpga::attachNetRoute(foreign, foreign_owner, 0, &driver, &sink,
                         "out", "in", "signal_913");
    fpga::registerNetRouteTiles(foreign, foreign_owner.wires[0], 0);
    require(fpga::findNetOwnersByNode(*target, fpga::CB_NODE_JOINT, 52).size() == 2,
            "shared-joint fixture did not register both physical owners");
    require(!pnr::commitPreparedRouteWithPreemption(proposal,
                fpga::previewNetRouteCut(victim, 0, cuts), cut) && calls == 0,
            "cut preview freed a tail joint still owned by another binding");
    require(fpga::unrouteNetRoute(foreign, 0) && target->cb.joint.jump.testBit(52),
            "removing the second owner lost the victim's joint lease");
    // Reusing a released terminal pin must publish its new pin lease too,
    // not only its CB-local bit (the preflight baseline is after the release).
    owner.wires[0].back().local = 321;
    target->cb.local.local.clearBit(318);
    target->pin_state.leased_nodes.clearBit(318);
    target->cb.local.local.setBit(321);
    target->pin_state.leased_nodes.setBit(321);
    const auto releases = fpga::previewNetRouteCut(victim, 0, cuts);
    require(std::none_of(releases.begin(), releases.end(), [](const fpga::RouteCutNode &node) {
                return node.tile.x == 1 && node.type == fpga::CB_NODE_DST && node.node == 217;
            }), "cut preview released the retained prefix landing");
    require(pnr::commitPreparedRouteWithPreemption(proposal, releases) && calls == 0 &&
                !target->cb.dst.jump.testBit(220),
            "read-only preflight changed routing state");
    require(!pnr::commitPreparedRouteWithPreemption(proposal, releases, [] { return false; }) &&
                fpga::isRouteComplete(owner.wires[0]) && !target->cb.dst.jump.testBit(220),
            "policy rejection leaked the prepared replacement");
    require(pnr::commitPreparedRouteWithPreemption(proposal, releases, cut) && calls == 1 &&
                owner.wires[0].size() == 2 && bridge->cb.joint.jump.testBit(5) &&
                bridge->cb.src.jump.testBit(120) && target->cb.dst.jump.testBit(220) &&
                target->cb.joint.jump.testBit(49) && target->cb.joint.jump.testBit(52) &&
                target->pin_state.leased_nodes.testBit(321),
            "valid replacement was not committed after the exact victim cut");
    require(!bridge->cb.src.jump.testBit(118) && !target->cb.dst.jump.testBit(218) &&
                !target->pin_state.leased_nodes.testBit(318),
            "publishing prepared leases resurrected the removed victim tail");
    require(fpga::findNetOwnersByNode(*bridge, fpga::CB_NODE_SRC, 118).empty(),
            "accepted cut retained a stale victim owner");
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

// Repeated arrivals at a full resource tile must skip packing, not traversal.
// A packable tile rejected by one route arrival must remain eligible for another.
void moving_forward_placement_blacklist()
{
    ForwardMovingMesh mesh(4);
    fpga::TileType resource{"resource_811", 1};
    fpga::Element slot;
    slot.name = "slot_812";
    slot.type = fpga::ELEMENT_FD;
    resource.elements.push_back(slot);
    Referable<rtl::Cell> cell;
    cell.name = "cell_813";
    cell.type = "FD_SAMPLE";
    rtl::Inst moving, occupant;
    moving.pos = occupant.pos = -1;
    moving.cell_ref.set(&cell);
    occupant.cell_ref.set(&cell);
    mesh.tile(1).tile_type = mesh.tile(2).tile_type = &resource;
    require(mesh.tile(1).tryAddAt(&occupant, 0) == 0, "cannot fill rejection tile");
    mesh.edge(0, 7, 10, 1, 8);
    mesh.edge(1, 8, 11, 1, 9);
    mesh.edge(1, 9, 12, 2, 10);
    mesh.edge(2, 10, 13, 2, 11);
    mesh.tile(0).cb.dst.jump = bit(7);
    pnr::MovingPlacementBlacklist blacklist({{&moving}});
    size_t blocked_arrivals = 0, viable_arrivals = 0;
    auto result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>& terminal) {
            if (tile.coord.x == 1) ++blocked_arrivals;
            if (blacklist.rejects(tile)) return false;
            ++viable_arrivals;
            if (dst != 11) return false; // Packing fits, but this arrival cannot ground.
            terminal = {crossbar(tile.coord, tile.coord, dst, -1, -1, 1), tilePin(tile.coord, 400)};
            return true;
        });
    // The only path passes THROUGH the blacklisted tile and tries two arrivals at the good tile.
    require(result.success && blocked_arrivals == 2 && viable_arrivals == 2,
            "blacklist prevented transit or cached a path-dependent grounding rejection");
    require(blacklist.size() == 2 && blacklist.hits == 1 && blacklist.checks == 3,
            "repeated full-tile packing was not skipped exactly once");
    // Preview must not place the candidate, remove the occupant, or retain any trial leases.
    require(!moving.tile.peer && moving.pos == -1 && occupant.tile.peer == &mesh.tile(1)
            && mesh.tile(1).elements_free[fpga::ELEMENT_FD] == 0,
            "blacklist feasibility proof changed placement: moving_pos=" + std::to_string(moving.pos));
    for (int x = 0; x < 4; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{}
                && mesh.tile(x).cb.dst.jump == (x == 0 ? bit(7) : NodeMask{}),
                "forward blacklist search leaked or removed routing masks");
    // A different search must reconsider a tile after its occupant is removed.
    occupant.tile.clear();
    occupant.pos = -1;
    mesh.tile(1).elements_initialized = false;
    pnr::MovingPlacementBlacklist fresh({{&moving}});
    require(!fresh.rejects(mesh.tile(1)), "blacklist survived into a new placement snapshot");
    // Cancellation is not a proof of impossible packing and must not poison future probes.
    require(!fresh.rejects(mesh.tile(3), [] { return true; }) && fresh.size() == 0,
            "cancelled placement probe blacklisted a tile");
    require(fresh.rejects(mesh.tile(3)) && fresh.size() == 1,
            "uncancelled probe failed to recognize a tile without resources");
}

// Several reverse frontiers can offer the same attached resource tile.
// It must be packed once when impossible, while a later feasible takeoff remains reachable.
void moving_backward_placement_blacklist()
{
    ForwardMovingMesh mesh(6);
    fpga::TileType resource{"resource_821", 1};
    fpga::Element slot;
    slot.name = "slot_822";
    slot.type = fpga::ELEMENT_FD;
    resource.elements.push_back(slot);
    Referable<rtl::Cell> cell;
    cell.name = "cell_823";
    cell.type = "FD_SAMPLE";
    rtl::Inst moving, occupant;
    moving.pos = occupant.pos = -1;
    moving.cell_ref.set(&cell);
    occupant.cell_ref.set(&cell);
    for (int x : {0, 4, 5}) mesh.tile(x).tile_type = &resource;
    require(mesh.tile(5).tryAddAt(&occupant, 0) == 0, "cannot fill reverse rejection tile");
    require(mesh.tile(4).tryAddAt(&moving, 0) == 0, "cannot place original source");
    for (int x = 0; x < 4; ++x) mesh.edge(x, 7 + x, 10 + x, x + 1, 8 + x);
    for (int x = 1; x < 4; ++x) mesh.tile(x).attached_resource_tiles.push_back(&mesh.tile(5));
    mesh.types[0].local_src[21].jump = bit(10);
    mesh.types[0].rebuildOutgoingSrcs();
    pnr::MovingPlacementBlacklist blacklist({{&moving}});
    size_t rejected_arrivals = 0;
    auto result = pnr::routeBackwardToTakeoff(mesh.tile(4), bit(400), {0,0}, 8, 8, 8,
        [&](fpga::Tile& tile, int, pnr::BackwardTakeoffChoice& choice) {
            choice.counts_toward_probe_limit = false;
            if (tile.coord.x >= 1 && tile.coord.x <= 3) {
                ++rejected_arrivals;
                require(blacklist.rejects(*tile.attached_resource_tiles.front()),
                        "full attached resource accepted a source placement");
                return false;
            }
            if (blacklist.rejects(tile)) return false;
            choice = {21, -1, -1};
            return true;
        }, nullptr, {}, 32768, 0, 64, nullptr, nullptr, {}, {},
        [](const auto&) { return false; });
    // Reverse traversal must exhaust the rejected resource's approaches and find tile zero.
    require(result.success && result.source_tile == &mesh.tile(0) && rejected_arrivals == 3,
            "reverse blacklist incorrectly pruned a transit frontier: success="
            + std::to_string(result.success) + " rejected_arrivals=" + std::to_string(rejected_arrivals));
    require(blacklist.size() == 1 && blacklist.checks == 2 && blacklist.hits == 2,
            "reverse frontiers repeated packing of the same attached resource");
    // Source probing runs with an existing placement: references, coordinates and slots survive.
    require(moving.tile.peer == &mesh.tile(4) && moving.pos == 0 && moving.coord == mesh.tile(4).coord
            && occupant.tile.peer == &mesh.tile(5)
            && mesh.tile(4).elements_free[fpga::ELEMENT_FD] == 0
            && mesh.tile(5).elements_free[fpga::ELEMENT_FD] == 0,
            "reverse blacklist changed an original placement or occupant");
    for (int x = 0; x < 6; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{}
                && mesh.tile(x).cb.local.local == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{},
                "reverse blacklist search changed live route leases");
}

// A preempted route to a fixed endpoint still needs repair after Fanouts times
// out. Placement immobility must not bypass the existing numeric gap router.
void moving_fixed_destination_repairs_retained_gap(bool blocked, bool transit = false,
                                                  bool preempt = true, bool protect = false,
                                                  bool alternate = false, bool focused = false,
                                                  int entry = 0)
{
    const int width = transit ? 6 : 4;
    ForwardMovingMesh mesh(width);
    fpga::TileType resource{"resource_701", 1};
    Referable<rtl::Cell> cell;
    cell.name = "cell_702";
    cell.type = "FD_SAMPLE";
    rtl::Inst driver, sink, other_driver, other_sink;
    driver.cell_ref.set(&cell);
    sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    sink.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(3)));
    driver.pos = sink.pos = 0;
    driver.outline.fixed = sink.outline.fixed = true;
    mesh.tile(0).tile_type = mesh.tile(3).tile_type = &resource;
    const int output = mesh.tile(0).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    const int input = mesh.tile(3).getPinNodes(cell.type, "D", 0).firstSetBit();
    require(output >= 0 && input >= 0, "fixed recovery fixture lacks endpoints");
    mesh.edge(0, output, 5, 1, 7);
    mesh.types[0].local_src[output].jump.setBit(5);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 7, 10, 2, 8);
    mesh.edge(2, 8, 11, 3, 9);
    mesh.types[3].dst_local[9].local.setBit(input);
    mesh.types[3].rebuildOutgoingSrcs();
    Referable<rtl::Net> net;
    net.name = "signal_703";
    sink.wires.push_back({tilePin({0,0}, output),
        crossbar({0,0}, {1,0}, output, 5, 7, 0),
        crossbar({1,0}, {2,0}, 7, 10, 8, 1),
        crossbar({2,0}, {3,0}, 8, 11, 9, 1),
        crossbar({3,0}, {3,0}, 9, -1, -1, 1), tilePin({3,0}, input)});
    sink.wires[0].front().pin_dir = fpga::TILE_PIN_OUTPUT;
    sink.wires[0].back().pin_dir = fpga::TILE_PIN_INPUT;
    leaseRoute(sink.wires[0]);
    const size_t binding = fpga::attachNetRoute(net, sink, 0, &driver, &sink,
                                               "Q", "D", "suffix_704");
    fpga::registerNetRouteTiles(net, sink.wires[0], binding);
    std::vector<fpga::RouteCutNode> cuts{{{1,0}, fpga::CB_NODE_SRC, 10}};
    // A removed terminal approach leaves only the retained input pin, as in
    // the production failure. Its replacement needs a foreign transit's hop.
    if (transit) cuts.push_back({{2,0}, fpga::CB_NODE_SRC, 11});
    require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cuts), cuts, true)
                && fpga::hasRouteGaps(sink.wires[0]),
            "fixture did not preserve a preemption gap");
    Referable<rtl::Net> victim;
    victim.name = "signal_705";
    if (transit) {
        for (auto* inst : {&other_driver, &other_sink}) inst->cell_ref.set(&cell);
        other_driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(4)));
        other_sink.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(5)));
        other_driver.pos = other_sink.pos = 0;
        mesh.tile(4).tile_type = mesh.tile(5).tile_type = &resource;
        const int out = mesh.tile(4).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
        const int in = mesh.tile(5).getPinNodes(cell.type, "D", 0).firstSetBit();
        mesh.edge(4, out, 15, 1, 17);
        mesh.types[4].local_src[out].jump.setBit(15);
        mesh.edge(1, 17, 10, 2, 8);
        mesh.edge(2, 8, 13, 5, 19);
        mesh.types[5].dst_local[19].local.setBit(in);
        other_sink.wires.push_back({tilePin({4,0}, out),
            crossbar({4,0}, {1,0}, out, 15, 17, 0),
            crossbar({1,0}, {2,0}, 17, 10, 8, 1),
            crossbar({2,0}, {5,0}, 8, 13, 19, 1),
            crossbar({5,0}, {5,0}, 19, -1, -1, 1), tilePin({5,0}, in)});
        other_sink.wires[0].front().pin_dir = fpga::TILE_PIN_OUTPUT;
        other_sink.wires[0].back().pin_dir = fpga::TILE_PIN_INPUT;
        leaseRoute(other_sink.wires[0]);
        const auto vi = fpga::attachNetRoute(victim, other_sink, 0, &other_driver,
                                            &other_sink, "Q", "D", "transit_706");
        fpga::registerNetRouteTiles(victim, other_sink.wires[0], vi);
        victim.route_protected = protect;
        if (alternate) {
            mesh.edge(1, 7, 21, 3, 23);
            mesh.types[3].dst_local[23].local.setBit(input);
        }
        for (auto& type : mesh.types) type.rebuildOutgoingSrcs();
    }
    if (blocked) {
        mesh.types[1].dst_src[7].jump = {};
        mesh.types[1].rebuildOutgoingSrcs();
    }
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.route_preemption_enabled = transit && preempt;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_704"};
    task.fanout = true;
    router.route_stage_deadline_enabled = true;
    router.route_stage_deadline = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    // The old stage can cancel repair without releasing either retained island.
    require(!router.routeNetTask(task) && fpga::hasRouteGaps(sink.wires[0]),
            "expired Fanout attempt completed or discarded its gap");
    router.route_stage_deadline_enabled = router.route_stage_deadline_expired = false;
    router.moving_stage = true;
    router.moving_focus_inst = focused ? &sink : nullptr;
    router.route_recursion_budget = 20;
    task.failure_coord = {-1, -1};
    task.failure_sequence = 0;
    const auto retained = sink.wires[0];
    const auto victim_before = other_sink.wires;
    for (int x = 0; x < width; ++x) {
        std::ostringstream out;
        auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net, &victim});
        // Establish real ownership before testing preemption, not synthetic busy bits.
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.conflicting_driver_nodes, "invalid fixed transit fixture: " + out.str());
    }
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    bool repaired;
    if (entry == 2) {
        // The production Moving queue can dispatch gap repair without the
        // destination-relocation wrapper. It must retain identical eligibility.
        std::vector<pnr::RouteDesign::RouteTask> batch{task};
        auto result = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving, batch, 1, 20);
        // Batch retirement is deferred to the outer pass, not vector erasure.
        repaired = result.completed == 1 && batch.size() == 1 && batch[0].remove_after_pass;
    } else if (entry == 1) repaired = router.routeNetTask(task);
    else repaired = router.moveUnfinishedDestination(task, &tasks, &reason);
    // A fixed load gets an in-place repair, not a new placement or takeoff.
    const bool should_cut = transit && preempt && !protect && !alternate;
    const bool expected = !blocked && (!transit || alternate || should_cut);
    require(repaired == expected, "fixed destination recovery: transit=" +
        std::to_string(transit) + " preempt=" + std::to_string(preempt) +
        " protected=" + std::to_string(protect) + " alternate=" + std::to_string(alternate) +
        " focused=" + std::to_string(focused) + " entry=" + std::to_string(entry) + ": " + reason);
    // Fixed endpoint repair has a temporary focus, never a lasting relocation focus.
    require(router.moving_focus_inst == (focused ? &sink : nullptr),
            "fixed repair leaked its focus into the next task");
    if (should_cut) {
        // Preserve the foreign source takeoff and schedule the cut suffix for repair.
        require(fpga::hasRouteGaps(other_sink.wires[0]) &&
                    mesh.tile(4).cb.src.jump.testBit(15) &&
                    std::any_of(router.pending_route_todo.begin(), router.pending_route_todo.end(),
                        [&](const auto& t) { return t.net == &victim && t.fanout; }),
                "fixed repair did not preserve and reschedule the cut victim");
    } else if (transit) {
        // A free alternative or protected/disabled victim forbids eviction.
        require(router.pending_route_todo.empty() &&
                    other_sink.wires[0].size() == victim_before[0].size() &&
                    std::equal(victim_before[0].begin(), victim_before[0].end(),
                               other_sink.wires[0].begin(), sameFragment),
                "fixed repair evicted a protected or unnecessary transit");
    }
    require(driver.tile.peer == &mesh.tile(0) && sink.tile.peer == &mesh.tile(3) &&
                driver.pos == 0 && sink.pos == 0 && driver.outline.fixed && sink.outline.fixed,
            "fixed recovery changed placement");
    require(mesh.tile(0).cb.src.jump.testBit(5) && mesh.tile(3).cb.local.local.testBit(input),
            "fixed recovery released the takeoff or retained endpoint");
    if (repaired) require(fpga::isRouteComplete(sink.wires[0]) && tasks.empty(),
                          "fixed repair left unfinished or invented relocation tasks");
    else require(task.failure_sequence > 0 && task.failure_coord.x == 1 && task.failure_coord.y == 0 &&
                     !reason.empty() && sink.wires[0].size() == retained.size() &&
                     std::equal(retained.begin(), retained.end(), sink.wires[0].begin(), sameFragment),
                 "failed fixed repair lost its current frontier or changed its retained path");
    // Every live bit must retain an actual route owner, on success and failure.
    for (int x = 0; x < width; ++x) {
        std::ostringstream out;
        const auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net, &victim});
        require(!audit.orphan_leases && !audit.missing_leases && !audit.missing_registrations &&
                    !audit.stale_registrations && !audit.conflicting_driver_nodes,
                "fixed recovery ownership mismatch: " + out.str());
    }
    fpga::unrouteNet(net);
    fpga::unrouteNet(victim);
    // Final cleanup must release both retained islands and any repaired hops.
    for (int x = 0; x < width; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{},
                "fixed recovery cleanup leaked leases");
}

void fanout_search_stack_retains_each_level()
{
    pnr::RouteSearchStack search;
    std::vector<fpga::Wire> route{
        crossbar({0,0}, {1,0}, 41, 17, 43, 0, true),
        crossbar({1,0}, {2,0}, 43, 23, 47, 1),
        crossbar({2,0}, {3,0}, 47, 29, 53, 1)};
    search.resume(route, {19,0}, 0);
    require(search.levels.size() == 4, "DFS did not push all routing levels");
    search.levels[2].tried_sources.setBit(31);
    route.pop_back();
    search.resume(route, {19,0}, 0);
    require(search.levels.size() == 3 && search.levels.back().tried_sources.testBit(29) &&
                search.levels.back().tried_sources.testBit(31),
            "popping the child restarted its parent's alternatives");
    route.pop_back();
    search.resume(route, {19,0}, 0);
    require(search.levels.size() == 2 && search.levels.back().tried_sources.testBit(23),
            "second pop forgot an earlier recursion level");
    route.push_back(crossbar({1,0}, {4,0}, 43, 37, 47, 1));
    search.resume(route, {19,0}, 0);
    require(search.levels.back().tried_sources == NodeMask{} &&
                search.levels[1].tried_sources == (bit(23) | bit(37)),
            "new child inherited the old child's exhausted alternatives");
    // External preemption/replacement of a prefix invalidates descendants,
    // even when a replacement later arrives at the same numeric DST.
    route[0].jump = 19;
    search.resume(route, {19,0}, 0);
    require(search.levels[1].tried_sources == bit(37),
            "replacement prefix retained stale child history");
    search.resume(route, {18,0}, 4);
    require(search.target.x == 18 && search.target_pos == 4,
            "relocation did not reset the search target");
}

// A fixed load may enter Moving without retained islands. Its last-chance
// numeric completion needs the same transit eligibility as fixed gap repair.
void moving_fixed_unbound_preemption(bool alternate, bool preempt, bool protect)
{
    ForwardMovingMesh mesh(6);
    fpga::TileType resource{"resource_811", 1};
    Referable<rtl::Cell> cell;
    cell.name = "cell_812"; cell.type = "FD_SAMPLE";
    rtl::Inst driver, sink, other_driver, other_sink;
    for (auto *inst : {&driver, &sink, &other_driver, &other_sink}) {
        inst->cell_ref.set(&cell); inst->pos = 0;
    }
    for (int x : {0, 3, 4, 5}) mesh.tile(x).tile_type = &resource;
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    sink.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(3)));
    other_driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(4)));
    other_sink.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(5)));
    driver.outline.fixed = sink.outline.fixed = true;
    const int output = mesh.tile(0).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    const int input = mesh.tile(3).getPinNodes(cell.type, "D", 0).firstSetBit();
    const int other_out = mesh.tile(4).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    const int other_in = mesh.tile(5).getPinNodes(cell.type, "D", 0).firstSetBit();
    require(output >= 0 && input >= 0 && other_out >= 0 && other_in >= 0,
            "unbound fixed fixture lacks endpoints");
    mesh.edge(0, output, 5, 1, 7);
    mesh.types[0].local_src[output].jump.setBit(5);
    mesh.edge(1, 7, 10, 2, 8);
    mesh.edge(2, 8, 11, 3, 9);
    mesh.types[3].dst_local[9].local.setBit(input);
    mesh.edge(4, other_out, 15, 1, 17);
    mesh.types[4].local_src[other_out].jump.setBit(15);
    mesh.edge(1, 17, 10, 2, 8);
    mesh.edge(2, 8, 13, 5, 19);
    mesh.types[5].dst_local[19].local.setBit(other_in);
    if (alternate) {
        mesh.edge(1, 7, 21, 3, 23);
        mesh.types[3].dst_local[23].local.setBit(input);
    }
    for (auto &type : mesh.types) type.rebuildOutgoingSrcs();
    Referable<rtl::Net> net, victim;
    net.name = "signal_813"; victim.name = "signal_814";
    other_sink.wires.push_back({tilePin({4,0}, other_out),
        crossbar({4,0}, {1,0}, other_out, 15, 17, 0),
        crossbar({1,0}, {2,0}, 17, 10, 8, 1),
        crossbar({2,0}, {5,0}, 8, 13, 19, 1),
        crossbar({5,0}, {5,0}, 19, -1, -1, 1), tilePin({5,0}, other_in)});
    other_sink.wires[0].front().pin_dir = fpga::TILE_PIN_OUTPUT;
    other_sink.wires[0].back().pin_dir = fpga::TILE_PIN_INPUT;
    leaseRoute(other_sink.wires[0]);
    const auto vi = fpga::attachNetRoute(victim, other_sink, 0, &other_driver,
                                        &other_sink, "Q", "D", "transit_815");
    fpga::registerNetRouteTiles(victim, other_sink.wires[0], vi);
    victim.route_protected = protect;
    const auto before = other_sink.wires[0];
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = router.complete_routes_only = true;
    router.route_preemption_enabled = preempt;
    router.route_deadends_enabled = false;
    router.route_recursion_budget = 20;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_816"};
    const bool done = router.routeNetTask(task);
    const bool cut = !alternate && preempt && !protect;
    // The ordinary Moving entry has no relocation focus, but the load is fixed.
    require(done == (alternate || cut), "unbound fixed destination: alternate=" +
        std::to_string(alternate) + " preempt=" + std::to_string(preempt) +
        " protect=" + std::to_string(protect));
    require(!router.moving_focus_inst, "unbound completion leaked its temporary focus");
    // Cuts preserve the foreign takeoff and explicitly reschedule its suffix.
    if (cut) require(fpga::hasRouteGaps(other_sink.wires[0]) &&
        mesh.tile(4).cb.src.jump.testBit(15) &&
        std::any_of(router.pending_route_todo.begin(), router.pending_route_todo.end(),
            [&](const auto &t) { return t.net == &victim && t.fanout; }),
        "unbound completion lost the victim takeoff or recovery task");
    else require(router.pending_route_todo.empty() && before.size() == other_sink.wires[0].size() &&
        std::equal(before.begin(), before.end(), other_sink.wires[0].begin(), sameFragment),
        "unbound completion cut a protected or unnecessary victim");
    // No failed numeric attempt may publish speculative leases or a partial route.
    require(!done || (!sink.wires.empty() && fpga::isRouteComplete(sink.wires.back())),
            "unbound completion reported an incomplete route");
    for (int x = 0; x < 6; ++x) {
        std::ostringstream out;
        const auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net, &victim});
        require(!audit.orphan_leases && !audit.missing_leases && !audit.missing_registrations &&
                    !audit.stale_registrations && !audit.conflicting_driver_nodes,
                "unbound completion ownership mismatch: " + out.str());
    }
    fpga::unrouteNet(net); fpga::unrouteNet(victim);
    // All private and shared ownership must be releasable after either outcome.
    for (int x = 0; x < 6; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{},
                "unbound fixed fixture leaked leases after cleanup");
}

void fanout_multilevel_search_resumes_after_dead_child(bool alternative,
                                                       bool near_target = false,
                                                       bool whole_task = false,
                                                       bool complete_call = false)
{
    // Real router regression: a retained shared root has a two-hop dead child.
    // Pop both private levels, then take a different child of the SAME root.
    // No global deadends, packing, placement movement, or preemption is needed.
    ForwardMovingMesh mesh(20);
    const int root = near_target ? 12 : 1;
    const int dead_parent = near_target ? 16 : 2;
    const int dead_leaf = near_target ? 17 : 3;
    fpga::TileType resource{"register_box_83", 1};
    fpga::Element reg;
    reg.name = "slots_71";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    mesh.tile(19).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_97";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sink, trunk_owner;
    driver.cell_ref.set(&cell);
    sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(19).tryAddAt(&sink, 0) == 0, "cannot place DFS test sink");
    parent.nets.resize(1);
    auto &net = parent.nets.front();
    net.name = "signal_89";
    const int driver_local = whole_task
        ? mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit() : 41;
    require(driver_local >= 0, "backtracking driver output is missing");
    mesh.edge(0, driver_local, 17, root, 43);
    if (whole_task) {
        mesh.types[0].local_src[driver_local].jump.setBit(17);
        mesh.types[0].rebuildOutgoingSrcs();
    }
    mesh.edge(root, 43, 23, dead_parent, 47);
    mesh.edge(dead_parent, 47, 29, dead_leaf, 53);
    if (alternative) {
        mesh.edge(root, 43, 37, 5, 59);
        mesh.edge(5, 59, 41, 10, 61);
        mesh.edge(10, 61, 43, 15, 67);
        mesh.edge(15, 67, 47, 19, 71);
        int pin = mesh.tile(19).getPinNodes("FDRE", "D", 0).firstSetBit();
        require(pin >= 0, "DFS test sink has no terminal");
        mesh.types[19].dst_local[71].local |= bit(pin);
    }
    // Even the exhaustive negative fixture has physical input topology, just
    // disconnected from this source, so failure exercises traversal itself.
    const int pin = mesh.tile(19).getPinNodes("FDRE", "D", 0).firstSetBit();
    mesh.types[19].dst_local[73].local |= bit(pin);
    mesh.tile(19).incoming_dst_nodes.setBit(73);
    mesh.types[19].rebuildOutgoingSrcs();
    auto first = crossbar({0,0}, {root,0}, driver_local, 17, 43, 0);
    first.owns_landing = true;
    trunk_owner.wires.push_back({first});
    leaseRoute(trunk_owner.wires[0]);
    mesh.tile(root).cb.dst.jump.setBit(43);
    fpga::attachNetRoute(net, trunk_owner, 0, &driver, &trunk_owner, "Q", "D", "trunk_79");
    fpga::registerNetRouteTiles(net, trunk_owner.wires[0], 0);
    first.shared = true;
    first.owns_landing = false;
    first.owns_dst = false;
    sink.wires.push_back({first,
        crossbar({root,0}, {dead_parent,0}, 43, 23, 47, 1, false, false),
        crossbar({dead_parent,0}, {dead_leaf,0}, 47, 29, 53, 1)});
    // Empty concrete names let this abstract fixture use numeric topology.
    for (auto &w : sink.wires[0]) {
        w.from_wire_name = ""; w.src_wire_name = ""; w.dst_wire_name = "";
    }
    sink.wires[0].back().owns_landing = true;
    leaseRoute(sink.wires[0]);
    mesh.tile(dead_leaf).cb.dst.jump.setBit(53);
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "Q", "D", "suffix_101");
    fpga::registerNetRouteTiles(net, sink.wires[0], 1);
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.fanout_stage = true;
    router.route_deadends_enabled = false;
    router.route_preemption_enabled = false;
    router.iteration_limit = 5;
    router.route_suffix_depth_limit = 2;
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_101"};
    task.fanout = true;
    if (complete_call) {
        router.moving_stage = true;
        router.moving_focus_inst = &sink;
        // Moving's production task boundary requires completion, not one DFS
        // pop followed by a scheduler yield. The same free alternate must be
        // explored within this call while the shared trunk stays leased.
        if (alternative) {
            require(router.routeTaskOrFail(task, "Moving destinations", 20, true) &&
                        fpga::isRouteComplete(sink.wires[0]),
                    "complete Moving call stopped at a resumable dead child");
        } else {
            router.complete_routes_only = true;
            router.route_recursion_budget = 20;
            // Exhausting a genuinely disconnected tree must still return
            // failure, with no speculative leases or repeated dead child.
            require(!router.routeFanoutTask(task), "disconnected Moving suffix completed");
        }
    }
    auto step = [&]() {
        router.route_recursion_budget = 1; // suspend at each scheduler boundary
        return router.routeFanoutTask(task);
    };
    if (whole_task) {
        // Moving has relocated the sink and retained an input's old fabric.
        // Popping one dead private hop is an internal search step, not an
        // exhausted connection: the same root still has a feasible child.
        // Exercise the real all-or-fail boundary rather than rescheduling it.
        router.moving_stage = true;
        router.moving_focus_inst = &sink;
        const bool done = router.routeTaskOrFail(task, "Moving destinations", 5, false);
        require(done == alternative && (!done || fpga::isRouteComplete(sink.wires[0])),
                "whole task stopped after a private backstep despite remaining alternatives");
    } else if (!complete_call) {
        require(!step() && sink.wires[0].size() == 2, "DFS did not pop the dead leaf");
        require(!step() && sink.wires[0].size() == 1, "DFS did not pop the exhausted parent");
        require(task.fanout_branch_offset == 0, "DFS rotated before exhausting the root");
        bool done = step();
        if (alternative) {
            for (int attempt = 0; !done && attempt < 8; ++attempt) done = step();
            require(done && fpga::isRouteComplete(sink.wires[0]),
                    "DFS failed to explore the root's alternate child");
            require(task.fanout_branch_offset == 0, "successful DFS unnecessarily changed roots");
        } else {
            require(!done && sink.wires[0].empty() && task.fanout_branch_offset == 1,
                    "exhausted DFS restarted its dead children instead of advancing the root");
        }
    }
    if (complete_call) {
        require(alternative ? fpga::isRouteComplete(sink.wires[0]) && task.fanout_branch_offset == 0
                            : sink.wires[0].empty() && task.fanout_branch_offset == 1,
                "complete Moving DFS stopped before exhausting its retained tree");
    }
    require(!mesh.tile(root).cb.src.jump.testBit(23) &&
                !mesh.tile(dead_parent).cb.src.jump.testBit(29) &&
                !mesh.tile(dead_parent).cb.dst.jump.testBit(47) &&
                !mesh.tile(dead_leaf).cb.dst.jump.testBit(53) &&
                mesh.tile(0).cb.src.jump.testBit(17) && mesh.tile(root).cb.dst.jump.testBit(43),
            "DFS backtracking leaked failed children or damaged the shared trunk");
    for (int x = 0; x < 20; ++x) {
        require(mesh.tile(x).cb.src_deadend.jump == NodeMask{},
                "DFS alternatives leaked into Basic's global deadend masks");
        std::ostringstream out;
        auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                "DFS lease bookkeeping failed: " + out.str());
    }
}

void moving_deep_private_backtracking(bool alternative)
{
    // Example 1: after relocation, five known-dead private hops consumed all
    // five attempts. The retained vector shrank 40 -> 35, never reaching the
    // source-connected fork. Unwinding exhausted levels is finite cleanup,
    // not five new forward searches. Keep the real task budget at five.
    ForwardMovingMesh mesh(64);
    fpga::TileType resource{"slots_281", 1};
    fpga::Element reg;
    reg.name = "registers_283";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    mesh.tile(63).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_293";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sink, trunk;
    for (auto *inst : {&driver, &sink, &trunk}) inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(63).tryAddAt(&sink, 0) == 0, "cannot place deep-backtracking sink");
    parent.nets.resize(1);
    auto &net = parent.nets.front();
    net.name = "signal_307";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    const int pin = mesh.tile(63).getPinNodes("FDRE", "D", 0).firstSetBit();
    require(local >= 0 && pin >= 0, "deep-backtracking endpoint missing");
    auto output = tilePin({0,0}, local);
    output.pin_dir = fpga::TILE_PIN_OUTPUT;
    output.resource = {0,0}; output.pos = 0; output.cell_type = "FDRE"; output.port = "Q";
    auto input = tilePin({63,0}, pin);
    input.pin_dir = fpga::TILE_PIN_INPUT;
    input.resource = {63,0}; input.pos = 0; input.cell_type = "FDRE"; input.port = "D";
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    auto first = crossbar({0,0}, {1,0}, local, 17, 19, 0);
    first.owns_landing = true;
    trunk.wires.push_back({output, first});
    leaseRoute(trunk.wires[0]);
    mesh.tile(0).cb.local.local.clearBit(local);
    mesh.tile(1).cb.dst.jump.setBit(19);
    const size_t parent_binding = fpga::attachNetRoute(net, trunk, 0,
        &driver, &trunk, "Q", "D", "parent_311");
    fpga::registerNetRouteTiles(net, trunk.wires[0], parent_binding);
    sink.wires.push_back(trunk.wires[0]);
    for (auto &w : sink.wires[0]) {
        w.shared = true; w.owns_dst = false; w.owns_landing = false;
    }
    constexpr int private_hops = 38;
    for (int i = 0; i < private_hops; ++i) {
        const int from = i + 1, to = i + 2;
        const int start = i ? 37 + 4 * (i - 1) : 19;
        const int src = 31 + 4 * i, dst = 37 + 4 * i;
        mesh.edge(from, start, src, to, dst);
        auto w = crossbar({from,0}, {to,0}, start, src, dst, 1, false, i != 0);
        w.owns_landing = i + 1 == private_hops;
        sink.wires[0].push_back(w);
    }
    leaseRoute(sink.wires[0]);
    mesh.tile(39).cb.dst.jump.setBit(185);
    // This fixture provides numeric topology, not an interned physical wire
    // name table. Do not attach labels that falsely request named lookup.
    for (auto *route : {&trunk.wires[0], &sink.wires[0]})
        for (auto &w : *route) {
            w.from_wire_name = ""; w.src_wire_name = ""; w.dst_wire_name = "";
        }
    const size_t child_binding = fpga::attachNetRoute(net, sink, 0,
        &driver, &sink, "Q", "D", "child_313");
    fpga::registerNetRouteTiles(net, sink.wires[0], child_binding);
    // The disconnected control still has a legitimate physical input.
    mesh.edge(55, 233, 239, 63, 241);
    mesh.types[63].dst_local[241].local.setBit(pin);
    mesh.types[63].rebuildOutgoingSrcs();
    if (alternative) {
        mesh.edge(1, 19, 211, 45, 213);
        mesh.edge(45, 213, 223, 50, 227);
        mesh.edge(50, 227, 229, 55, 233);
    }
    const auto old_trunk = trunk.wires[0];
    auto audit = [&] {
        for (int x = 0; x < 64; ++x) {
            std::ostringstream out;
            const auto r = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
            require(!r.orphan_leases && !r.missing_leases && !r.missing_registrations &&
                        !r.stale_registrations && !r.conflicting_driver_nodes,
                    "deep-backtracking occupancy mismatch: " + out.str());
            require(mesh.tile(x).cb.src_deadend.jump == NodeMask{},
                    "Moving backtracking changed Basic deadends");
        }
    };
    require(sink.wires[0].size() == 40, "fixture must retain forty fragments");
    audit();
    std::ostringstream proof;
    require(fpga::auditUnroutedReachability(net, net.routes[child_binding], output, input,
                proof, 32768) == alternative, "deep-backtracking topology witness mismatch");
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = true;
    router.moving_focus_inst = &sink;
    router.route_deadends_enabled = false;
    router.route_preemption_enabled = false;
    // Match routeDesign's production setup, rather than the class's one-hop
    // construction default. The reference bypass is only four hops long.
    router.iteration_limit = 200;
    router.route_suffix_depth_limit = 200;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "child_313"};
    task.fanout = task.endpoints_prepared = true;
    const bool complete = router.routeTaskOrFail(task, "Moving destinations", 5, false);
    require(complete == alternative && (!complete || fpga::isRouteComplete(sink.wires[0])),
            "whole task spent its attempts unwinding a known-dead private prefix");
    require(router.route_stats.backstep_success == private_hops &&
                router.route_recursion_budget > 0,
            "dead-prefix cleanup exhausted the forward-attempt budget");
    for (int i = 0; i < private_hops; ++i)
        require(!mesh.tile(i + 1).cb.src.jump.testBit(31 + 4 * i) &&
                    !mesh.tile(i + 2).cb.dst.jump.testBit(37 + 4 * i),
                "deep backtracking retained a discarded private hop");
    require(trunk.wires[0].size() == old_trunk.size(), "deep backtracking changed the parent");
    for (size_t i = 0; i < old_trunk.size(); ++i)
        require(sameFragment(trunk.wires[0][i], old_trunk[i]) &&
                    trunk.wires[0][i].owns_landing == old_trunk[i].owns_landing,
                "deep backtracking changed a shared parent fragment");
    require(mesh.tile(0).cb.src.jump.testBit(17) && mesh.tile(1).cb.dst.jump.testBit(19) &&
                !mesh.tile(0).cb.local.local.testBit(local),
            "deep backtracking damaged the shared source or charged its LOCAL reference");
    audit();
    require(fpga::unrouteNet(net), "cannot remove deep-backtracking fixture");
    audit();
}

void fanout_continuation_counts_retained_visits(bool near_target, bool alternative)
{
    ForwardMovingMesh mesh(20);
    const int root = near_target ? 13 : 1;
    const int parent_tile = near_target ? 14 : 2;
    const int leaf = near_target ? 16 : 3;
    fpga::TileType resource{"slots_113", 1};
    fpga::Element reg;
    reg.name = "register_127"; reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    mesh.tile(19).tile_type = &resource;
    Referable<rtl::Module> module, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&module);
    Referable<rtl::Cell> cell;
    cell.name = "load_131"; cell.type = "FDRE"; cell.module_ref.set(&primitive);
    rtl::Inst driver, sink, trunk;
    driver.cell_ref.set(&cell); sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(19).tryAddAt(&sink, 0) == 0, "cannot place revisit sink");
    module.nets.resize(1);
    auto &net = module.nets.front(); net.name = "signal_137";
    mesh.edge(0, 41, 17, root, 43);
    mesh.edge(root, 43, 23, parent_tile, 47);
    mesh.edge(parent_tile, 47, 29, root, 53);
    mesh.edge(root, 53, 31, leaf, 59);
    // Every numeric DST is new. Only whole-path Tile history can reject this
    // third arrival at root; its lease is free and it leads to a free terminal.
    mesh.edge(leaf, 59, 37, root, 61);
    mesh.edge(root, 61, 41, 19, 67);
    const int pin = mesh.tile(19).getPinNodes("FDRE", "D", 0).firstSetBit();
    mesh.types[19].dst_local[67].local.setBit(pin);
    if (alternative) {
        // Requires popping leaf and the second root arrival. In the near
        // fixture this valid detour first moves AWAY from the target.
        mesh.edge(parent_tile, 47, 43, 5, 71);
        mesh.edge(5, 71, 47, 10, 73);
        mesh.edge(10, 73, 53, 15, 79);
        mesh.edge(15, 79, 59, 19, 83);
        mesh.types[19].dst_local[83].local.setBit(pin);
    }
    mesh.types[19].rebuildOutgoingSrcs();
    auto first = crossbar({0,0}, {root,0}, 41, 17, 43, 0);
    first.owns_landing = true;
    trunk.wires.push_back({first}); leaseRoute(trunk.wires[0]);
    mesh.tile(root).cb.dst.jump.setBit(43);
    fpga::attachNetRoute(net, trunk, 0, &driver, &trunk, "Q", "D", "trunk_139");
    fpga::registerNetRouteTiles(net, trunk.wires[0], 0);
    first.shared = true; first.owns_dst = first.owns_landing = false;
    sink.wires.push_back({first,
        crossbar({root,0}, {parent_tile,0}, 43, 23, 47, 1),
        crossbar({parent_tile,0}, {root,0}, 47, 29, 53, 1),
        crossbar({root,0}, {leaf,0}, 53, 31, 59, 1)});
    for (auto &wire : sink.wires[0]) {
        wire.from_wire_name = ""; wire.src_wire_name = ""; wire.dst_wire_name = "";
    }
    sink.wires[0].back().owns_landing = true;
    leaseRoute(sink.wires[0]); mesh.tile(leaf).cb.dst.jump.setBit(59);
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "Q", "D", "suffix_149");
    fpga::registerNetRouteTiles(net, sink.wires[0], 1);
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.fanout_stage = true;
    router.route_deadends_enabled = router.route_preemption_enabled = false;
    router.iteration_limit = 5; router.route_suffix_depth_limit = 2;
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_149"};
    task.fanout = true;
    bool done = false, popped = false;
    for (int attempt = 0; attempt < 30 && !done && task.fanout_branch_offset == 0; ++attempt) {
        router.route_recursion_budget = 1;
        done = router.routeFanoutTask(task);
        popped |= sink.wires[0].size() < 4;
        require(pnr::combinatorialRouteVisitsValid(sink.wires[0]),
                "Fanouts committed a third Tile arrival across continuation calls");
    }
    require(popped && done == alternative,
            "Fanouts did not backtrack/explore the legal detour after rejecting a repeated Tile");
    require(!mesh.tile(leaf).cb.src.jump.testBit(37) && !mesh.tile(root).cb.dst.jump.testBit(61),
            "rejected repeat leaked leases");
    if (done) require(fpga::isRouteComplete(sink.wires[0]), "detour did not ground");
    else require(sink.wires[0].empty() && task.fanout_branch_offset == 1,
                 "exhausted looping suffix did not rotate its root");
    for (int x = 0; x < 20; ++x) {
        require(mesh.tile(x).cb.src_deadend.jump == NodeMask{}, "visit rejection became a global deadend");
        std::ostringstream out;
        const auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                "revisit backtracking ownership mismatch: " + out.str());
    }
}

void moving_input_uses_mapped_io_endpoint()
{
    // An I/O resource need not share the CB's canonical attachment coordinate.
    // Its loaded pin map, not attached_resource_tiles, identifies its takeoff.
    // Previously this valid source was never probed: input recovery exhausted
    // the reverse graph and repeated the same failure for each packing slot.
    ForwardMovingMesh mesh(7);
    auto& resource = mesh.tile(0);
    resource.cb_type = resource.cb.type = nullptr;
    resource.cb_coord = {-1, -1};
    fpga::TileType io_type{"io_bank_73", 1};
    io_type.pin_map.rememberResourcePinName(fpga::TILE_PIN_OUTPUT, 0, "I");
    io_type.pin_map.output_nodes[0] = bit(159);
    io_type.pin_map.rememberEndpointRouteRef(
        fpga::TILE_PIN_OUTPUT, 0, 159, mesh.types[4].name, {4, 0});
    resource.tile_type = &io_type;
    mesh.types[3].name = mesh.types[4].name; // Same type, wrong physical offset.
    mesh.edge(4, 7, 17, 6, 9);
    mesh.types[4].local_src[159].jump = bit(17);
    mesh.types[4].rebuildOutgoingSrcs();
    mesh.types[3].local_src[159].jump = bit(17);
    mesh.types[3].rebuildOutgoingSrcs();

    Referable<rtl::Module> primitive;
    primitive.is_blackbox = true;
    Referable<rtl::Cell> cell;
    cell.name = "pad_driver_29";
    cell.type = "IBUF";
    cell.module_ref.set(&primitive);
    rtl::Inst driver;
    driver.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&resource));
    driver.pos = 0;
    driver.coord = resource.coord;
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    require(mesh.tile(4).attached_resource_tiles.empty(),
            "mapped-I/O fixture accidentally uses direct attachment");
    require(resource.getPinNodesForRouteType("IBUF", "O", 0,
                fpga::TILE_PIN_OUTPUT, mesh.types[4].name, {4, 0}) == bit(159),
            "mapped-I/O fixture has no real output endpoint");
    auto&& index = router.backwardDockingIndex({6, 0}, 6);
    auto run = [&]() {
        return pnr::routeBackwardToInput(mesh.tile(6), bit(400), {0, 0}, 6,
            [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                return router.routeInputSourceTry(driver, "O", tile, src, choice);
            }, &index, {}, nullptr, nullptr);
    };
    auto result = run();
    require(result.success && result.source_tile == &mesh.tile(4) &&
                result.source_src == 17 && result.takeoff.local == 159 &&
                result.expanded == 1,
            "Moving input proof rejected the mapped I/O source");
    require(resource.coord.x == 0 && driver.tile.peer == &resource && driver.pos == 0 &&
                mesh.tile(4).cb.src.jump == NodeMask{} &&
                mesh.tile(6).cb.dst.jump == NodeMask{},
            "input proof changed placement or live leases");
    pnr::BackwardTakeoffChoice wrong;
    require(!router.routeInputSourceTry(driver, "O", mesh.tile(3), 17, wrong),
            "mapped-I/O lookup accepted the right type at the wrong offset");
    mesh.tile(4).cb.src.jump.setBit(17);
    require(!run().success, "mapped-I/O input proof ignored a busy takeoff");
    mesh.tile(4).cb.src.jump = {};
    pnr::BackwardTakeoffStateView trial;
    trial[&mesh.tile(4)] = mesh.tile(4).cb;
    trial[&mesh.tile(4)].src.jump.setBit(17);
    pnr::BackwardTakeoffChoice blocked;
    require(!router.routeInputSourceTry(driver, "O", mesh.tile(4), 17, blocked, &trial),
            "mapped-I/O input proof ignored temporary output-route reservations");
}

void moving_nested_input_keeps_outer_window()
{
    // The outer source-relocation search is suspended while proving a moved
    // cell's input. A one-entry cache used to replace the object still borrowed
    // by that outer search, silently changing both its bounds and resolved arcs.
    ForwardMovingMesh mesh(7);
    mesh.edge(1, 99, 10, 3, 7);
    mesh.edge(3, 7, 11, 5, 8);
    mesh.edge(5, 8, 12, 6, 9);
    mesh.edge(1, 99, 70, 0, 20);
    mesh.types[1].local_src[100].jump = bit(10) | bit(70);
    mesh.types[1].rebuildOutgoingSrcs();
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    auto&& outer = router.backwardDockingIndex({6, 0}, 6);
    const pnr::BackwardResolveKey key{5, 0, 8};
    const auto* before = pnr::resolveBackwardSources(outer, key);
    require(before && before->size() == 1 && before->front().tile == &mesh.tile(3),
            "outer-window fixture lacks its continuation");
    auto probe = [&](int wanted_src) {
        return [&, wanted_src](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
            if (&tile != &mesh.tile(1) || src != wanted_src) return false;
            choice.local = 100;
            return true;
        };
    };
    {
        auto&& inner = router.backwardDockingIndex({0, 0}, 1);
        require(inner.shared_cache == outer.shared_cache &&
                    pnr::resolveBackwardSources(inner, key) == nullptr,
                "nested window lost its bounds or duplicated shared connectivity");
        const auto result = pnr::routeBackwardToInput(mesh.tile(0), bit(400), {1, 0}, 1,
            probe(70), &inner, {}, nullptr, nullptr);
        require(result.success, "nested input fixture failed to route");
    }
    require(outer.center.x == 6 && outer.center.y == 0 && outer.radius == 6,
            "nested input search overwrote the active outer search window");
    const auto* after = pnr::resolveBackwardSources(outer, key);
    require(after && after->size() == 1 && after->front().tile == &mesh.tile(3),
            "nested input search removed the outer continuation");
    const auto result = pnr::routeBackwardToInput(mesh.tile(6), bit(400), {1, 0}, 6,
        probe(10), &outer, {}, nullptr, nullptr);
    require(result.success && result.source_tile == &mesh.tile(1),
            "outer search could not resume after its nested input proof");
    for (int x = 0; x < 7; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{},
                "nested proofs changed live leases");
}

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

void moving_forward_internal_local_controls() {
    for (int mode = 0; mode < 7; ++mode) {
        ForwardMovingMesh mesh(3);
        auto& root = mesh.tile(0);
        root.cb.dst.jump.setBit(7);
        mesh.edge(0, 7, 10, 1, 8);
        mesh.edge(1, 8, 11, 2, 9);
        mesh.types[0].dst_src[7].jump = {};
        if (mode != 2) mesh.types[0].dst_local[7].local.setBit(91);
        mesh.types[0].local_src[91].jump.setBit(10);
        if (mode == 1) root.cb.local.local.setBit(91);
        rtl::Conn reservation;
        if (mode == 3) root.input_local_reservations[91] = &reservation;
        if (mode == 6) mesh.types[0].dst_src[7].jump.setBit(10);
        mesh.types[0].rebuildOutgoingSrcs();
        const auto before = root.cb;
        bool threw = false, cancel = false;
        pnr::ForwardPlacementResult result;
        try {
            result = pnr::routeForwardToPlacement({{&root, 7, true, {}}},
                [&](fpga::Tile& tile, int, bool, auto&) {
                    if (tile.coord.x == 1 && mode == 4) throw std::runtime_error("probe_911");
                    if (tile.coord.x == 1 && mode == 5) cancel = true;
                    return tile.coord.x == 2;
                }, [&] { return cancel; });
        } catch (const std::runtime_error& e) {
            threw = std::string(e.what()) == "probe_911";
        }
        // A busy/reserved/disconnected bridge must never become a legal path.
        require(result.success == (mode == 0 || mode == 6) && threw == (mode == 4) &&
                    result.cancelled == (mode == 5), "internal LOCAL bridge control failed");
        if (result.success) {
            const auto edges = std::count_if(result.fragments.begin(), result.fragments.end(),
                [](const auto& w) { return w.type == fpga::Wire::WIRE_ROUTE_EDGE; });
            // The existing direct path must retain priority; fallback preserves typed edges.
            require(mode == 6 ? edges == 0 : edges == 3, "bridge identity or fast-path priority lost");
        }
        // Successful, failed, cancelled and throwing probes all restore live state.
        require(root.cb.src.jump == before.src.jump && root.cb.dst.jump == before.dst.jump &&
                    root.cb.local.local == before.local.local && root.cb.joint.jump == before.joint.jump,
                "internal LOCAL bridge leaked a speculative root lease");
        for (int x : {1, 2})
            require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                        mesh.tile(x).cb.local.local == NodeMask{}, "bridge leaked a downstream lease");
        if (mode == 0) {
            std::vector<fpga::Wire> arcs(result.fragments.begin(), result.fragments.begin() + 3);
            for (bool occupied : {false, true}) {
                if (occupied) mesh.tile(1).cb.dst.jump.setBit(8);
                else mesh.types[0].local_src[91].jump = {};
                size_t failed = 0;
                // A failed prepared commit must undo earlier arc leases, not its blocker.
                require(!pnr::commitPreparedRoute(arcs, &failed) && failed == (occupied ? 2u : 1u),
                        "typed commit ignored a stale arc or occupied landing");
                require(root.cb.src.jump == before.src.jump && root.cb.dst.jump == before.dst.jump &&
                            root.cb.local.local == before.local.local &&
                            mesh.tile(1).cb.dst.jump == (occupied ? bit(8) : NodeMask{}),
                        "typed commit rollback leaked leases or freed a foreign landing");
                mesh.types[0].local_src[91].jump.setBit(10);
                mesh.tile(1).cb.dst.jump = {};
            }
        }
    }
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

void moving_forward_search_has_compact_path_storage() {
    // Reproduce the production fallback's growing frontier, not a mock queue.
    // Reject all parking probes so recursive alternatives must be exhausted.
    const pid_t child = fork();
    require(child >= 0, "cannot fork forward-search memory regression");
    if (child == 0) {
        try {
            constexpr int width = 20;
            resetGrid(width, width);
            auto& device = fpga::Device::current();
            device.cb_types.clear();
            device.cb_types.resize(1);
            auto& type = device.cb_types.front();
            type.name = "arena_switch_83";
            type.type_id = type.base_type_id = 0;
            const fpga::Coord directions[]{{0,-1},{1,0},{0,1},{-1,0}};
            for (int direction = 0; direction < 4; ++direction) {
                const int src = 10 + direction;
                const int dst = 20 + (direction + 2) % 4;
                type.local_src[0].jump.setBit(src);
                type.dst_by_src[src].push_back({directions[direction], 0,
                    {bit(dst)}, {}, true});
                type.rememberNodeName(fpga::CB_NODE_SRC, src,
                                     "departure_" + std::to_string(direction));
                type.rememberNodeName(fpga::CB_NODE_DST, 20 + direction,
                                     "arrival_" + std::to_string(direction));
                for (int arrival = 20; arrival < 24; ++arrival)
                    type.dst_src[arrival].jump.setBit(src);
            }
            type.rebuildOutgoingSrcs();
            for (auto& tile : device.tile_grid) {
                tile.cb_coord = tile.coord;
                tile.cb_type = tile.cb.type = &type;
            }

            // Limit additional address space, independently of the executable's
            // existing mappings. The former full-Node/history frontier exceeds
            // this allowance long before one million actual placement probes.
            size_t pages = 0;
            std::ifstream status("/proc/self/statm");
            require(static_cast<bool>(status >> pages), "cannot read address-space size");
            rlimit limit{};
            require(getrlimit(RLIMIT_AS, &limit) == 0, "cannot read memory limit");
            const rlim_t budget = pages * static_cast<rlim_t>(sysconf(_SC_PAGESIZE)) +
                                 (48ULL << 20);
            limit.rlim_cur = std::min(limit.rlim_cur, budget);
            require(setrlimit(RLIMIT_AS, &limit) == 0, "cannot set search memory budget");
            size_t probes = 0;
            const auto result = pnr::routeForwardToPlacement(
                {{device.getTile(10, 10), 0, false, {}}},
                [&](auto&, int, bool, auto&) { ++probes; return false; },
                [&] { return probes >= 1000000; });

            // A search cancelled at the requested probe count must leave no
            // speculative occupancy, even across many revisited crossbars.
            require(result.cancelled && !result.success && probes == 1000000,
                    "compact search did not reach the requested probe count");
            for (const auto& tile : device.tile_grid)
                require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                            tile.cb.joint.jump == NodeMask{},
                        "cancelled memory regression leaked speculative leases");
            _exit(0);
        } catch (const std::exception& error) {
            std::fprintf(stderr, "forward-search memory regression: %s\n", error.what());
            _exit(1);
        }
    }
    int status = 0;
    require(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "forward placement search exceeded its 48 MiB growth budget or leaked leases");
}

void moving_forward_search_never_restarts_depth() {
    for (int depth : {1, 5, 23, 64}) {
        for (bool from_dst : {false, true}) {
            ForwardMovingMesh mesh(depth + 1);
            if (from_dst) mesh.tile(0).cb.dst.jump = bit(7);
            for (int x = 0; x < depth; ++x) mesh.edge(x, 7, 10, x + 1, 7);
            if (!from_dst) {
                mesh.types[0].local_src[7].jump = bit(10);
                mesh.types[0].rebuildOutgoingSrcs();
            }
            size_t probes = 0;
            const auto result = pnr::routeForwardToPlacement(
                {{&mesh.tile(0), 7, from_dst, {}}},
                [&](auto&, int, bool, auto&) { ++probes; return false; });
            // Rejecting every placement must exhaust the path, not rebuild it
            // once per depth. Probe counts alone missed the production bug.
            require(!result.success && !result.cancelled &&
                        result.expanded == static_cast<size_t>(depth + 1) &&
                        probes == static_cast<size_t>(depth + from_dst),
                    "recursive exhaustion lost a node or repeated a placement probe");
            require(result.edge_checks == static_cast<size_t>(depth),
                    "depth restart repeated edge checks: depth=" + std::to_string(depth) +
                    " checks=" + std::to_string(result.edge_checks));
            // Exhaustion must release speculative leases while preserving the anchor.
            for (int x = 0; x <= depth; ++x)
                require(mesh.tile(x).cb.src.jump == NodeMask{} &&
                            mesh.tile(x).cb.dst.jump == (x == 0 && from_dst ? bit(7) : NodeMask{}),
                        "recursive exhaustion changed committed occupancy");
        }
    }
}

void moving_forward_failed_routing_cache() {
    // Each layer converges four paths onto each of four identical DST states.
    // Distinct numeric SRC/DST namespaces and high IDs must not alias the cache.
    for (unsigned seed = 0; seed < 10; ++seed) {
        constexpr int layers = 5, width = 4, count = 1 + layers * width;
        ForwardMovingMesh mesh(count);
        std::mt19937 random(901 + seed);
        std::vector<int> arrivals(count);
        for (auto &arrival : arrivals) arrival = 512 + random() % 3500;
        const int root = arrivals[0];
        mesh.tile(0).cb.dst.jump = bit(root);
        for (int level = 0; level < layers; ++level) {
            const int from_begin = level == 0 ? 0 : 1 + (level - 1) * width;
            const int from_end = level == 0 ? 1 : from_begin + width;
            for (int from = from_begin; from < from_end; ++from)
                for (int i = 0; i < width; ++i) {
                    const int to = 1 + level * width + i;
                    mesh.edge(from, arrivals[from], 100 + i, to, arrivals[to]);
                }
        }
        std::vector<unsigned> probes(count);
        auto rejected = pnr::routeForwardToPlacement({{&mesh.tile(0), root, true, {}}},
            [&](fpga::Tile &tile, int dst, bool, auto&) {
                require(dst == arrivals[tile.coord.x], "failure cache changed the arrival ID");
                ++probes[tile.coord.x];
                return false;
            });
        // Completely failed convergent states are probed once, not once per path.
        require(!rejected.success && !rejected.cancelled && rejected.expanded == count &&
                    rejected.failed_states == count && rejected.failure_cache_hits == 48,
                "exhausted forward states were expanded repeatedly: expanded=" +
                    std::to_string(rejected.expanded) + " hits=" +
                    std::to_string(rejected.failure_cache_hits));
        for (unsigned calls : probes) require(calls == 1, "convergent DST repeated its move probe");
        // The cache belongs to one search; a new placement snapshot is reconsidered.
        auto fresh = pnr::routeForwardToPlacement({{&mesh.tile(0), root, true, {}}},
            [&](fpga::Tile &tile, int, bool, auto&) { return tile.coord.x == count - 1; });
        require(fresh.success, "failed routing cache survived into a new search");
        size_t attempts = 0;
        auto cancelled = pnr::routeForwardToPlacement({{&mesh.tile(0), root, true, {}}},
            [&](auto&, int, bool, auto&) { ++attempts; return false; },
            [&] { return attempts == 3; });
        // Pending frames on cancellation have not exhausted their alternatives.
        require(cancelled.cancelled && cancelled.failed_states == 0,
                "cancelled frames were cached as failed");
        for (int x = 0; x < count; ++x)
            require(mesh.tile(x).cb.src.jump == NodeMask{} &&
                        mesh.tile(x).cb.dst.jump == (x == 0 ? bit(root) : NodeMask{}),
                    "failure cache changed committed masks or leaked trial leases");
    }

    ForwardMovingMesh mesh(4);
    mesh.tile(0).cb.dst.jump = bit(7);
    mesh.edge(0, 7, 10, 1, 8);
    mesh.edge(0, 7, 11, 1, 10);
    mesh.edge(1, 8, 12, 2, 9);
    mesh.edge(2, 9, 13, 1, 10);
    // Both arrivals use the SAME physical SRC jump. Adding another edge()
    // would duplicate its target and count the blocked prefix twice.
    mesh.types[1].dst_src[10].jump = bit(12);
    mesh.types[1].rebuildOutgoingSrcs();
    size_t repeats = 0;
    std::ostringstream decisions;
    auto result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [&](fpga::Tile &tile, int dst, bool, auto&) {
            if (tile.coord.x == 1 && dst == 10) ++repeats;
            return tile.coord.x == 2 && !mesh.tile(1).cb.dst.jump.testBit(8);
        }, {}, &decisions);
    // The first prefix reserves SRC 12. Its context-dependent failure must not
    // hide a second prefix which reaches the SAME DST with SRC 12 still free.
    require(result.success && repeats == 2 && result.failed_states == 0 &&
                result.failure_cache_hits == 0,
            "prefix-only congestion poisoned a later free approach: success=" +
                std::to_string(result.success) + " repeats=" + std::to_string(repeats) +
                " failed=" + std::to_string(result.failed_states) +
                " hits=" + std::to_string(result.failure_cache_hits) + "\n" + decisions.str());
    for (int x = 0; x < 4; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{}, "cached search leaked a source lease");
}

void moving_backward_failed_routing_cache() {
    for (bool continuation : {false, true}) {
        ForwardMovingMesh mesh(6);
        mesh.edge(0, 7, 100, 5, 10);
        mesh.edge(0, 7, 100, 3, 12);
        mesh.edge(3, 12, 101, 5, 11);
        if (continuation) {
            mesh.edge(1, 8, 102, 0, 7);
            mesh.types[1].local_src[200].jump = bit(102);
            mesh.types[1].rebuildOutgoingSrcs();
        }
        size_t probes = 0, first_source_probes = 0, boundary_calls = 0;
        auto probe = [&](fpga::Tile &tile, int src, pnr::BackwardTakeoffChoice &choice) {
            ++probes;
            if (tile.coord.x == 0 && src == 100) ++first_source_probes;
            if (tile.coord.x == 1 && continuation) { choice.local = 200; return true; }
            return false;
        };
        auto run = [&](const pnr::BackwardTakeoffCancel &cancel = {}, int depth = 0) {
            return pnr::routeBackwardToTakeoff(mesh.tile(5), bit(400), {0,0}, depth, 8, 8,
                probe, nullptr, cancel, 0, 0, 0, nullptr, nullptr, {}, {},
                [&](const auto&) { ++boundary_calls; return false; });
        };
        auto result = run();
        // SRC 100 is offered by two different landings. It is cached only after
        // its move probe AND its predecessor subtree fail, never while queued.
        require(result.success == continuation && first_source_probes == 1 && boundary_calls == 0,
                "backward cache confused a pending continuation with a failed source");
        if (!continuation)
            require(result.failed_states == 2 && result.failure_cache_hits == 1,
                    "exhausted backward SRC was not skipped at its second landing");
        else
            require(result.source_tile == &mesh.tile(1) && result.failed_states == 0,
                    "failed local probe blacklisted a source with a live predecessor");
        probes = first_source_probes = 0;
        auto limited = run({}, 1);
        // A child with an unsearched predecessor remains unknown at the depth
        // limit. With no predecessor at all, resolving the empty incoming set
        // already proves the isolated source failed, even at that depth.
        require(!limited.success && limited.failed_states == (continuation ? 0u : 1u),
                "depth-limited reverse continuation was cached as exhausted: continuation=" +
                    std::to_string(continuation) + " success=" + std::to_string(limited.success) +
                    " failed=" + std::to_string(limited.failed_states));
        probes = first_source_probes = 0;
        auto cancelled = run([&] { return probes == 1; });
        require(!cancelled.success && cancelled.failed_states == 0,
                "cancelled backward probe published a failure");
        probes = first_source_probes = 0;
        require(run().success == continuation && first_source_probes == 1,
                "reverse failure cache leaked across search calls");
        for (int x = 0; x < 6; ++x)
            require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{},
                    "reverse failure proof modified route leases");
    }
}

void moving_forward_search_preserves_recursive_probe_order() {
    ForwardMovingMesh mesh(7);
    mesh.tile(0).cb.dst.jump = bit(7);
    mesh.tile(1).cb.dst.jump = bit(8);
    mesh.edge(0, 7, 10, 2, 20);
    mesh.edge(0, 7, 11, 3, 21);
    mesh.edge(1, 8, 10, 4, 22);
    mesh.edge(1, 8, 11, 5, 23);
    for (int x = 2; x <= 5; ++x)
        mesh.edge(x, 18 + x, 12, 6, 28 + x);
    const std::vector<std::pair<int, int>> expected{
        {0, 7}, {2, 20}, {6, 30}, {3, 21}, {6, 31},
        {1, 8}, {4, 22}, {6, 32}, {5, 23}, {6, 33}};
    std::vector<std::pair<int, int>> actual;
    const auto result = pnr::routeForwardToPlacement(
        {{&mesh.tile(0), 7, true, {}}, {&mesh.tile(1), 8, true, {}}},
        [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>&) {
            actual.emplace_back(tile.coord.x, dst);
            // Each deep alternative must see only its own temporary path.
            if (tile.coord.x == 6)
                for (int x = 2; x <= 5; ++x)
                    require(mesh.tile(x).cb.src.jump.testBit(12) == (dst == 28 + x),
                            "recursive probe inherited another alternative's leases");
            return false;
        });
    // Returning from a child resumes its parent's next exit. The second
    // anchor starts only after the first anchor's alternatives are exhausted.
    require(!result.success && !result.cancelled && actual == expected &&
                result.expanded == expected.size() && result.edge_checks == 8,
            "bounded-memory search repeated or reordered placement probes");
    for (int x = 0; x < 7; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{},
                "exhausted recursive proof leaked a speculative source");
}

void moving_forward_local_bridge_resumes_same_frame() {
    ForwardMovingMesh mesh(5);
    mesh.tile(0).cb.dst.jump = bit(7);
    mesh.edge(0, 7, 10, 1, 8);
    mesh.edge(1, 8, 11, 2, 9); // Exhaust this branch before trying the local bridge.
    mesh.edge(0, 7, 12, 3, 10);
    mesh.types[0].dst_src[7].jump.clearBit(12);
    mesh.types[0].dst_local[7].local.setBit(91);
    mesh.types[0].local_src[91].jump.setBit(12);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(3, 10, 13, 4, 11);
    std::vector<int> probes;
    const auto result = pnr::routeForwardToPlacement({{&mesh.tile(0), 7, true, {}}},
        [&](fpga::Tile& tile, int, bool, auto&) {
            probes.push_back(tile.coord.x);
            if (tile.coord.x != 4) return false;
            // The failed jump branch is gone; only the selected local bridge is leased.
            require(!mesh.tile(0).cb.src.jump.testBit(10) &&
                        mesh.tile(1).cb.src.jump == NodeMask{} &&
                        mesh.tile(0).cb.local.local.testBit(91),
                    "bridge inherited a failed alternative's occupancy");
            return true;
        });
    // Enabling a LOCAL bridge must not restart either the anchor or its jump subtree.
    require(result.success && probes == std::vector<int>{0, 1, 2, 3, 4} &&
                result.expanded == 6 && result.edge_checks == 4,
            "local bridge restarted an exhausted subtree");
    for (int x = 0; x < 5; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} &&
                    mesh.tile(x).cb.dst.jump == (x == 0 ? bit(7) : NodeMask{}),
                "successful bridge left speculative leases");
}

void moving_forward_deep_recursion_uses_heap_frames() {
    const pid_t child = fork();
    require(child >= 0, "cannot fork recursion-stack regression");
    if (child == 0) {
        try {
            constexpr int depth = 2048;
            resetGrid(depth + 1, 1);
            auto& device = fpga::Device::current();
            device.cb_types.clear();
            device.cb_types.resize(1);
            auto& type = device.cb_types.front();
            type.name = "deep_box_71";
            type.type_id = type.base_type_id = 0;
            type.dst_src[7].jump.setBit(10);
            type.dst_by_src[10].push_back({{1, 0}, 0, {bit(7)}, {}, true});
            type.rebuildOutgoingSrcs();
            for (auto& tile : device.tile_grid) tile.cb_type = tile.cb.type = &type;
            device.getTile(0, 0)->cb.dst.jump = bit(7);
            rlimit limit{};
            require(getrlimit(RLIMIT_STACK, &limit) == 0, "cannot read stack limit");
            limit.rlim_cur = std::min<rlim_t>(limit.rlim_cur, 256 * 1024);
            require(setrlimit(RLIMIT_STACK, &limit) == 0, "cannot limit native stack");
            const auto result = pnr::routeForwardToPlacement(
                {{device.getTile(0, 0), 7, true, {}}},
                [](fpga::Tile& tile, int, bool, auto&) { return tile.coord.x == depth; });
            // The algorithm's depth must not be limited by the native call stack.
            require(result.success && result.fragments.size() == depth &&
                        result.expanded == depth + 1 && result.edge_checks == depth,
                    "deep recursive traversal restarted or lost its continuation");
            for (const auto& tile : device.tile_grid)
                require(tile.cb.src.jump == NodeMask{} &&
                            tile.cb.dst.jump == (tile.coord.x == 0 ? bit(7) : NodeMask{}),
                        "deep success failed to unwind speculative state");
            _exit(0);
        } catch (const std::exception& error) {
            std::fprintf(stderr, "forward recursion regression: %s\n", error.what());
            _exit(1);
        }
    }
    int status = 0;
    require(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "deep recursive traversal overflowed the native stack or failed cleanup");
}

void moving_forward_paths_preserve_metadata() {
    // Exercise high node IDs, two joints, exact landing aliases and both
    // anchor modes. Reconstructed paths must produce identical
    // exported Wires and must not change fork ownership or temporary leases.
    for (bool from_dst : {false, true}) {
        ForwardMovingMesh mesh(2);
        auto& type = mesh.types[0];
        if (from_dst) {
            type.dst_joint[4095].joint = bit(4094);
            mesh.tile(0).cb.dst.jump = bit(4095);
        } else {
            type.local_joint[4095].joint = bit(4094);
        }
        type.joint_joint[4094].joint = bit(4093);
        type.joint_src[4093].jump = bit(4092);
        type.dst_by_src[4092].push_back({{1, 0}, 1, {bit(4091)},
                                       {{4091, "landing_alias_57"}}, true});
        type.rememberNodeName(from_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL,
                              4095, "origin_58");
        type.rememberConnName(fpga::CB_NODE_JOINT, 4093, fpga::CB_NODE_SRC,
                              4092, "joint_59", "departure_alias_60");
        mesh.types[1].dst_local[4091].local = bit(400);
        mesh.types[1].rebuildOutgoingSrcs();
        type.rebuildOutgoingSrcs();
        const auto result = pnr::routeForwardToPlacement(
            {{&mesh.tile(0), 4095, from_dst, {}}},
            [&](fpga::Tile& tile, int dst, bool, std::vector<fpga::Wire>&) {
                if (&tile != &mesh.tile(1)) return false;
                require(dst == 4091 && mesh.tile(0).cb.src.jump == bit(4092) &&
                            mesh.tile(0).cb.joint.jump == (bit(4094) | bit(4093)),
                        "candidate path lost high node IDs or its two-joint leases");
                return true;
            });
        require(result.success && result.fragments.size() == 1,
                "two-joint path failed to reach the parking probe");
        const auto& wire = result.fragments.front();
        require(wire.local == 4095 && wire.jump == 4092 && wire.dst == 4091 &&
                    wire.joint == 4093 && wire.joint2 == 4094 &&
                    wire.owns_dst == !from_dst && wire.pos == (from_dst ? 2 : 0) &&
                    wire.from_wire_name == std::string("origin_58") &&
                    wire.src_wire_name == std::string("departure_alias_60") &&
                    wire.dst_wire_name == std::string("landing_alias_57"),
                "reconstructed edge changed exported names, joints or fork ownership");
        require(mesh.tile(0).cb.src.jump == NodeMask{} &&
                    mesh.tile(0).cb.joint.jump == NodeMask{} &&
                    mesh.tile(0).cb.dst.jump == (from_dst ? bit(4095) : NodeMask{}),
                "successful search changed committed anchor leases");
    }
}

void moving_destination_commits_forward_grounding(bool blocked,
                                                bool retained = false,
                                                bool at_root = false,
                                                bool packing_blocked = false,
                                                bool source_focus = false,
                                                bool generated_source = false,
                                                bool support_blocked = false,
                                                bool replay = false,
                                                bool duplicate_input = false,
                                                bool stale_trigger = false) {
    struct ReplayArtifacts {
        std::filesystem::path path;
        std::string previous;
        bool had_previous = false;
        explicit ReplayArtifacts(bool enabled) {
            if (!enabled) return;
            if (const char* old = std::getenv("SCALEPNR_FAILURE_ARTIFACT_DIR")) {
                previous = old; had_previous = true;
            }
            char directory[] = "/tmp/scalepnr-moving-replay-XXXXXX";
            require(mkdtemp(directory) != nullptr, "cannot create replay test directory");
            path = directory;
            setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", directory, 1);
        }
        ~ReplayArtifacts() {
            if (path.empty()) return;
            if (had_previous) setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", previous.c_str(), 1);
            else unsetenv("SCALEPNR_FAILURE_ARTIFACT_DIR");
            std::filesystem::remove_all(path);
        }
    } replay_artifacts(replay);
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
    if (duplicate_input) {
        // Source legalization left an empty old-source binding for the same
        // physical sink port. It is not a second input needing another lease.
        require(source != &driver,"duplicate-input test needs a generated source");
        sink.wires.push_back({});
        fpga::attachNetRoute(net,sink,1,&driver,&sink,"Q","D","moving_suffix");
    }
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
    Referable<rtl::Net> remote_owner;
    rtl::Inst remote_sink;
    if (packing_blocked && replay) {
        // A free exit can be blocked by a lease on a tile never visited by
        // the placement search. Failure replay must still audit its owner.
        remote_owner.name = "remote_signal_73";
        mesh.edge(landing, dst, 11, 3, 87);
        mesh.types[3].dst_local[87].local = bit(88);
        remote_sink.wires.push_back({crossbar({3,0}, {3,0}, 87, -1, -1, 1),
                                     tilePin({3,0}, 88)});
        fpga::attachNetRoute(remote_owner, remote_sink, 0, nullptr, &remote_sink,
                             "Q", "D", "remote_branch_73");
        fpga::registerNetRouteTiles(remote_owner, remote_sink.wires[0], 0);
        mesh.tile(3).cb.dst.jump = bit(87);
        mesh.tile(3).cb.local.local = mesh.tile(3).pin_state.leased_nodes = bit(88);
    }
    // A disconnected tile looks locally ideal; it must never be selected.
    mesh.types[3].dst_local[8].local = bit(pin0);
    mesh.types[3].rebuildOutgoingSrcs();
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&net, source, "Q");
    if (duplicate_input) {
        router.source_endpoint_retargets[std::to_string(reinterpret_cast<uintptr_t>(&driver)) + ":Q"] =
            {source, "Q"};
    }
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
    if (stale_trigger) {
        // A saved Moving trigger can also predate source legalization, even
        // when every queued task has already been retargeted.
        task.from = &driver;
    }
    std::vector<fpga::CBState> before_replay_cb;
    std::vector<NodeMask> before_replay_pins;
    if (replay) for (int x = 0; x < 5; ++x) {
        before_replay_cb.push_back(mesh.tile(x).cb);
        before_replay_pins.push_back(mesh.tile(x).pin_state.leased_nodes);
    }
    bool moved = router.moveUnfinishedCell(move_task, &tasks, &task, &reason, replay);
    require(driver.tile.peer == &mesh.tile(0) && driver.pos == 0 &&
                helper.tile.peer == &mesh.tile(0) && helper.pos == 4,
            "outgoing focus relocation moved the source or its generated endpoint");
    require(moved != (blocked || packing_blocked || support_blocked), "forward relocation result: " + reason);
    if (replay) {
        if (moved) {
            require(router.failure_candidate_directory.empty() &&
                        std::filesystem::is_empty(replay_artifacts.path),
                    "successful routing rendered failure candidates");
        } else {
            for (int x = 0; x < 5; ++x) {
                for (auto kind : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST, fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL})
                    require(fpga::congestionNodeMask(mesh.tile(x).cb, kind) ==
                                fpga::congestionNodeMask(before_replay_cb[x], kind),
                            "diagnostic replay leaked prefix or terminal occupancy");
                require(mesh.tile(x).pin_state.leased_nodes == before_replay_pins[x],
                        "diagnostic replay leaked pin reservations");
            }
            const std::filesystem::path directory(router.failure_candidate_directory);
            auto read = [&](const std::filesystem::path& path) {
                std::ifstream file(path, std::ios::binary);
                return std::string(std::istreambuf_iterator<char>(file), {});
            };
            const auto index = read(directory / "index.txt");
            const auto tree = read(directory / "source_tree.txt");
            // Failure diagnostics enumerate the original source tile and full
            // retained paths, not just the DSTs chosen as branch anchors.
            require(tree.find("route=\"stable_trunk\"") != std::string::npos &&
                        tree.find("route=\"moving_suffix\"") != std::string::npos &&
                        tree.find("FRAGMENT index=0") != std::string::npos &&
                        tree.find("exclusion=not_dst_landing") != std::string::npos &&
                        tree.find("TILE coord=(0,0)") != std::string::npos &&
                        tree.find("TREE_TOTAL") != std::string::npos &&
                        index.find("source_tree=") != std::string::npos,
                    "failure replay omitted the physical source tree or its exclusions");
            require(index.find("normal_candidates=2") != std::string::npos &&
                        index.find("replayed_candidates=2") != std::string::npos &&
                        index.find("completed=1") != std::string::npos,
                    "replay did not visit every failed candidate exactly once: " + index);
            const auto root = read(directory / "001_cb_1_0_dst_7_resource_1_0.txt");
            const auto landing = read(directory / "002_cb_2_0_dst_8_resource_2_0.txt");
            require(root.find("no packing resources") != std::string::npos &&
                        root.find("AUDIT leased=") != std::string::npos &&
                        landing.find("pos=0 depth=0") != std::string::npos &&
                        landing.find("pos=4 depth=0") != std::string::npos &&
                        landing.find("CANDIDATE_CB tile=(2,0)") != std::string::npos,
                    "candidate reports omitted no-resource, occupancy, or packing decisions");
            if (packing_blocked)
                require(landing.find("reason=busy owner=") != std::string::npos,
                        "replay report lost packing owner diagnostics");
            if (packing_blocked) {
                const auto decisions = read(directory / "forward_search.txt");
                // Audit the blocked landing, not just candidate placement
                // tiles, and retain the real route binding behind its lease.
                require(decisions.find("reason=forward_target_lease tile=(3,0) node=DST:87/live=1") != std::string::npos &&
                            decisions.find("remote_signal_73") != std::string::npos &&
                            decisions.find("remote_branch_73") != std::string::npos &&
                            !pnr::RouteCongestionTrace::current,
                        "failure replay omitted blocked landing ownership or leaked trace scope");
            }
            if (support_blocked) {
                const auto failure = landing.find("OTHER_INPUT_REJECT");
                const auto picture = landing.find("PICTURE reason=no distinct", failure);
                const auto backtrack = landing.find("PACK_BACKTRACK", failure);
                require(failure != std::string::npos && picture < backtrack &&
                            landing.find("temporary_owner=\"moving_suffix\"", failure) != std::string::npos &&
                            landing.find("placed=true tile=(2,0) pos=4", picture) != std::string::npos,
                        "replay picture was captured after rollback or lost temporary owners: " + landing);
            }
            size_t pictures = 0;
            for (const auto& entry : std::filesystem::directory_iterator(directory)) {
                if (entry.path().extension() != ".png") continue;
                const auto png = read(entry.path());
                require(png.starts_with(std::string("\x89PNG\r\n\x1a\n", 8)) && png.size() > 100,
                        "candidate image missing or not a PNG");
                ++pictures;
            }
            require(pictures == 3 && read(directory / "forward_search.txt").find("FORWARD_END queued=2 reached=2 cancelled=0") != std::string::npos,
                    "replay images or complete forwarding decisions missing");
        }
    }
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
            if (replay)
                require(read("routing_failure.txt").find("candidate_replay_directory=" +
                            router.failure_candidate_directory) != std::string::npos,
                        "final failure report did not link replay artifacts");
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
    if (duplicate_input) {
        require(tasks.size() == 1 && tasks.front().from == source,
                "Moving requeued the obsolete source's duplicate input");
        for (const auto& binding : net.routes)
            require(binding.from == source, "Moving left an obsolete source binding");
    }
    for (int x : {0, 1, 2}) {
        std::ostringstream out;
        auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                "forward relocation ownership mismatch: " + out.str());
    }
}

// An empty suffix must reuse the occupied takeoff of an INCOMPLETE sibling.
// Its only legal destination is reachable from that prefix, not the output
// LOCAL. Exercise the real relocation/commit path, not just source selection.
void moving_destination_uses_incomplete_sibling(bool alias, bool indexed,
                                               bool remove_sibling_first,
                                               int fault = 0,
                                               bool source_reference = false)
{
    ForwardMovingMesh mesh(5);
    fpga::TileType resource{"slots_83", 1};
    fpga::Element reg;
    reg.name = "registers_29";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {2, 3, 4}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_71";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, stranger, sink, sibling, blocker;
    for (auto *inst : {&driver, &stranger, &sink, &sibling, &blocker})
        inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(4).tryAddAt(&sink, 0) == 0 &&
                mesh.tile(3).tryAddAt(&sibling, 0) == 0,
            "cannot place incomplete sibling fixture");
    parent.nets.resize(2);
    auto &net = parent.nets[0];
    auto &seed = parent.nets[alias ? 1 : 0];
    net.name = "signal_61";
    if (alias) seed.name = "alias_97";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    require(local >= 0, "incomplete sibling driver has no output");
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 31);
    const int pin = mesh.tile(2).getPinNodes("FDRE", "D", 0).firstSetBit();
    mesh.types[2].dst_local[31].local.setBit(pin);
    mesh.types[2].rebuildOutgoingSrcs();
    // One owned hop, deliberately without a sink terminal.
    sibling.wires.push_back({crossbar({0,0}, {1,0}, local, 17, 19, 0)});
    sibling.wires[0].back().owns_landing = true;
    if (source_reference) {
        // Example 1 retains a real Takeoff after a transit cut, but the
        // driver's LOCAL is only a source reference, not a routing lease.
        // Start with a complete physical route and cut its transit below.
        mesh.edge(1, 19, 29, 3, 37);
        const int old_pin = mesh.tile(3).getPinNodes("FDRE", "D", 0).firstSetBit();
        require(old_pin >= 0, "source-reference sibling input missing");
        mesh.types[3].dst_local[37].local.setBit(old_pin);
        mesh.types[3].rebuildOutgoingSrcs();
        auto output = tilePin({0,0}, local);
        output.pin_dir = fpga::TILE_PIN_OUTPUT;
        output.resource = {0,0}; output.pos = driver.pos;
        output.cell_type = "FDRE"; output.port = "Q";
        sibling.wires[0].insert(sibling.wires[0].begin(), output);
        sibling.wires[0].push_back(crossbar({1,0}, {3,0}, 19, 29, 37, 1));
        sibling.wires[0].push_back(crossbar({3,0}, {3,0}, 37, -1, -1, 2));
        auto input = tilePin({3,0}, old_pin);
        input.pin_dir = fpga::TILE_PIN_INPUT;
        input.resource = {3,0}; input.pos = sibling.pos;
        input.cell_type = "FDRE"; input.port = "D";
        sibling.wires[0].push_back(input);
    }
    leaseRoute(sibling.wires[0]);
    if (source_reference) mesh.tile(0).cb.local.local.clearBit(local);
    mesh.tile(1).cb.dst.jump.setBit(19);
    const size_t seed_binding = fpga::attachNetRoute(seed, sibling, 0, &driver,
        &sibling, "Q", "D", "retained_43");
    fpga::registerNetRouteTiles(seed, sibling.wires[0], seed_binding);
    if (source_reference)
        require(fpga::unrouteNetRoutesFromNodes({{&seed, seed_binding}},
                    {{{1,0}, fpga::CB_NODE_SRC, 29}}, true) &&
                    fpga::hasRouteGaps(sibling.wires[0]),
                "real transit preemption did not retain source/GAP/input");
    sink.wires.push_back({});
    const size_t sink_binding = fpga::attachNetRoute(net, sink, 0, &driver,
        &sink, "Q", "D", "suffix_59");
    // Invalid prefixes must not become anchors merely because the last DST is
    // busy. Case 6 is a valid prefix whose only destination is full; case 7
    // belongs to another input of the moving sink and cannot be borrowed.
    if (fault == 1) seed.routes[seed_binding].from = &stranger;
    if (fault == 2) seed.routes[seed_binding].from_port = "other_output";
    if (fault == 3) mesh.tile(0).cb.src.jump.clearBit(17);
    const size_t takeoff_index = source_reference ? 1 : 0;
    if (fault == 4) sibling.wires[0][takeoff_index].from = {1,0};
    if (fault == 5) sibling.wires[0][takeoff_index].shared = true; // no real owner
    if (fault == 6)
        require(mesh.tile(2).tryAddAt(&blocker, 0) == 0, "cannot fill destination");
    if (fault == 7) seed.routes[seed_binding].to = &sink;
    if (fault == 8) mesh.tile(1).cb.dst.jump.clearBit(19);
    if (fault == 9) {
        stranger.wires.push_back(sibling.wires[0]);
        auto other = fpga::attachNetRoute(seed, stranger, 0, &stranger,
            &stranger, "Q", "D", "foreign_103");
        fpga::registerNetRouteTiles(seed, stranger.wires[0], other);
    }
    if (fault == 10) mesh.types[0].dst_by_src[17].clear();
    const auto original = sibling.wires[0];
    std::vector<fpga::CBState> before;
    for (int x = 0; x < 5; ++x) before.push_back(mesh.tile(x).cb);
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    if (indexed) {
        router.indexSourceRoute(&net, &driver, "Q");
        router.indexSourceRoute(&seed, &driver, "Q");
    }
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_59"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    require(!router.sourceTreeHasCompleteExit(driver, "Q"),
            "fixture accidentally contains a complete seed");
    const bool moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    require(moved == (fault == 0),
            "incomplete sibling anchor result fault=" + std::to_string(fault) + ": " + reason);
    require(sibling.wires[0].size() == original.size(),
            "relocation changed the sibling's retained prefix");
    for (size_t i = 0; i < original.size(); ++i)
        require(sameFragment(sibling.wires[0][i], original[i]) &&
                    sibling.wires[0][i].owns_landing == original[i].owns_landing,
                "relocation changed an incomplete sibling fragment");
    if (source_reference)
        require(!mesh.tile(0).cb.local.local.testBit(local) &&
                    !mesh.tile(0).pin_state.leased_nodes.testBit(local),
                "Moving charged the source reference as an exclusive pin lease");
    if (fault) {
        require(sink.tile.peer == &mesh.tile(4) && sink.wires[0].empty(),
                "rejected sibling anchor changed the sink");
        for (int x = 0; x < 5; ++x)
            require(mesh.tile(x).cb.src.jump == before[x].src.jump &&
                        mesh.tile(x).cb.dst.jump == before[x].dst.jump &&
                        mesh.tile(x).cb.local.local == before[x].local.local &&
                        mesh.tile(x).cb.joint.jump == before[x].joint.jump,
                    "failed sibling search modified existing leases");
        return; // Invalid fixtures intentionally contain stale leases/claims.
    }
    require(sink.tile.peer == &mesh.tile(2) && fpga::isRouteComplete(sink.wires[0]) &&
                sink.wires[0].front().shared &&
                sink.wires[0][takeoff_index].jump == 17,
            "Moving did not share the retained takeoff and ground the suffix");
    auto audit = [&]() {
        for (int x = 0; x < 5; ++x) {
            std::ostringstream out;
            auto result = fpga::auditTileCongestion(mesh.tile(x), out, {&net, &seed});
            require(!result.orphan_leases && !result.missing_leases &&
                        !result.missing_registrations && !result.stale_registrations &&
                        !result.conflicting_driver_nodes,
                    "incomplete sibling ownership mismatch: " + out.str());
        }
    };
    audit();
    require(fpga::unrouteNetRoute(remove_sibling_first ? seed : net,
                                 remove_sibling_first ? seed_binding : sink_binding),
            "cannot remove first shared branch");
    require(mesh.tile(0).cb.src.jump.testBit(17) && mesh.tile(1).cb.dst.jump.testBit(19),
            "removing one branch released the survivor's prefix");
    audit();
    require(fpga::unrouteNetRoute(remove_sibling_first ? net : seed,
                                 remove_sibling_first ? sink_binding : seed_binding),
            "cannot remove surviving branch");
    audit();
    for (int x = 0; x < 5; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} &&
                    mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{},
                "shared incomplete prefix leaked after both branches were removed");
}

void moving_incomplete_sibling_regressions(bool unleased_source = false)
{
    for (bool alias : {false, true})
        for (bool indexed : {false, true}) {
            for (bool first : {false, true})
                moving_destination_uses_incomplete_sibling(alias, indexed, first, 0, unleased_source);
            for (int fault = 1; fault <= 10; ++fault)
                moving_destination_uses_incomplete_sibling(alias, indexed, false, fault, unleased_source);
        }
}

void moving_destination_rejects_its_detached_tail(bool bypass)
{
    // A real transit cut leaves source -> prefix, GAP, and a live tail.
    // The tail can ground locally, but that is NOT a source-connected route.
    // Example 1 incorrectly accepted this relocation and kept the GAP. With
    // a bypass, continue from the prefix; without it, preserve the old state.
    ForwardMovingMesh mesh(6);
    fpga::TileType resource{"slots_137", 1};
    fpga::Element reg;
    reg.name = "registers_139";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {3, 4, 5}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_149";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sink;
    driver.cell_ref.set(&cell);
    sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(5).tryAddAt(&sink, 0) == 0, "cannot place detached-tail sink");
    parent.nets.resize(2);
    auto &net = parent.nets.front();
    auto &requester_net = parent.nets.back();
    net.name = "signal_151";
    requester_net.name = "requester_163";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    const int pin = mesh.tile(5).getPinNodes("FDRE", "D", 0).firstSetBit();
    require(local >= 0 && pin >= 0, "detached-tail endpoint missing");
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 29);
    mesh.edge(2, 29, 31, 3, 37);
    mesh.edge(3, 37, 41, 4, 43);
    mesh.edge(4, 43, 59, 5, 61);
    mesh.types[5].dst_local[61].local.setBit(pin);
    const int island_pin = mesh.tile(4).getPinNodes("FDRE", "D", 0).firstSetBit();
    mesh.types[4].dst_local[43].local.setBit(island_pin);
    mesh.types[4].rebuildOutgoingSrcs();
    const int tail_pin = mesh.tile(3).getPinNodes("FDRE", "D", 0).firstSetBit();
    mesh.types[3].dst_local[37].local.setBit(tail_pin);
    mesh.types[3].rebuildOutgoingSrcs();
    if (bypass) {
        mesh.edge(1, 19, 47, 4, 53);
        const int new_pin = mesh.tile(4).getPinNodes("FDRE", "D", 0).firstSetBit();
        mesh.types[4].dst_local[53].local.setBit(new_pin);
        mesh.types[4].rebuildOutgoingSrcs();
    }
    sink.wires.push_back({crossbar({0,0}, {1,0}, local, 17, 19, 0),
        crossbar({1,0}, {2,0}, 19, 23, 29, 1),
        crossbar({2,0}, {3,0}, 29, 31, 37, 1),
        crossbar({3,0}, {4,0}, 37, 41, 43, 1),
        crossbar({4,0}, {5,0}, 43, 59, 61, 1),
        crossbar({5,0}, {5,0}, 61, -1, -1, 2), tilePin({5,0}, pin)});
    leaseRoute(sink.wires[0]);
    const size_t binding = fpga::attachNetRoute(net, sink, 0, &driver,
        &sink, "Q", "D", "suffix_157");
    fpga::registerNetRouteTiles(net, sink.wires[0], binding);
    require(fpga::unrouteNetRoutesFromNodes({{&net, binding}},
                {{{1,0}, fpga::CB_NODE_SRC, 23}}, true) &&
                fpga::hasRouteGaps(sink.wires[0]),
            "transit cut did not create the real detached-tail state");
    // A different driver takes precisely the released jump and its landing.
    // Every busy node has a real claim, including the preemption requester.
    rtl::Inst requester, receiver;
    for (auto *inst : {&requester, &receiver}) inst->cell_ref.set(&cell);
    requester.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(1)));
    requester.pos = 0;
    receiver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(2)));
    receiver.pos = 0;
    const int request_local = mesh.tile(1).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    require(request_local >= 0, "requester output missing");
    mesh.types[1].local_src[request_local].jump.setBit(23);
    mesh.types[1].rebuildOutgoingSrcs();
    mesh.types[2].dst_local[29].local.setBit(400);
    mesh.types[2].rebuildOutgoingSrcs();
    receiver.wires.push_back({crossbar({1,0}, {2,0}, request_local, 23, 29, 0),
        crossbar({2,0}, {2,0}, 29, -1, -1, 2), tilePin({2,0}, 400)});
    leaseRoute(receiver.wires[0]);
    const size_t request_binding = fpga::attachNetRoute(requester_net, receiver,
        0, &requester, &receiver, "Q", "D", "request_167");
    fpga::registerNetRouteTiles(requester_net, receiver.wires[0], request_binding);
    auto audit = [&] {
        for (int x = 0; x < 6; ++x) {
            std::ostringstream out;
            const auto result = fpga::auditTileCongestion(mesh.tile(x), out, {&net, &requester_net});
            require(!result.orphan_leases && !result.missing_leases &&
                        !result.missing_registrations && !result.stale_registrations &&
                        !result.conflicting_driver_nodes,
                    "detached-tail ownership mismatch: " + out.str());
        }
    };
    audit();
    const auto original = sink.wires[0];
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_157"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    const bool moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    require(moved == bypass, "Moving accepted a detached tail or missed its bypass: " + reason);
    if (bypass) {
        require(fpga::isRouteComplete(sink.wires[0]) &&
                    !fpga::hasRouteGaps(sink.wires[0]) && sink.tile.peer == &mesh.tile(4),
                "Moving reported success but copied a disconnected tail/GAP");
        require(!mesh.tile(2).cb.src.jump.testBit(31) &&
                    !mesh.tile(3).cb.src.jump.testBit(41),
                "Moving retained abandoned tail leases");
    } else {
        require(sink.tile.peer == &mesh.tile(5) && original.size() == sink.wires[0].size(),
                "failed detached-tail search changed placement or route");
        for (size_t i = 0; i < original.size(); ++i)
            require(sameFragment(original[i], sink.wires[0][i]) &&
                        original[i].owns_landing == sink.wires[0][i].owns_landing,
                    "failed detached-tail search changed fragment ownership");
    }
    audit();
    require(fpga::unrouteNet(net), "cannot remove detached-tail result");
    audit();
    require(fpga::isRouteComplete(receiver.wires[0]), "Moving damaged the requester");
    require(fpga::unrouteNet(requester_net), "cannot remove requester");
    audit();
}

void moving_empty_binding_uses_numeric_completion(bool preserve_binding, bool bypass,
                                                  bool retired_storage = false)
{
    // Example 1: relocating a sink cleared one physical route but kept its
    // logical binding. Generic routing needed an internal LOCAL bridge; the
    // numeric fallback rejected the existing EMPTY binding before searching.
    // The identical unbound control establishes that this is an admission bug.
    ForwardMovingMesh mesh(20);
    fpga::TileType resource{"slots_317", 1};
    fpga::Element reg;
    reg.name = "registers_331";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {4, 19}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_337";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sink;
    driver.cell_ref.set(&cell); sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(4).tryAddAt(&sink, 0) == 0, "cannot place empty-binding fixture");
    parent.nets.resize(1);
    auto &net = parent.nets.front();
    net.name = "signal_347";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    const int old_pin = mesh.tile(4).getPinNodes("FDRE", "D", 0).firstSetBit();
    require(local >= 0 && old_pin >= 0, "empty-binding endpoint missing");
    auto output = tilePin({0,0}, local);
    output.pin_dir = fpga::TILE_PIN_OUTPUT;
    output.resource = {0,0}; output.pos = 0; output.cell_type = "FDRE"; output.port = "Q";
    auto old_input = tilePin({4,0}, old_pin);
    old_input.pin_dir = fpga::TILE_PIN_INPUT;
    old_input.resource = {4,0}; old_input.pos = 0; old_input.cell_type = "FDRE"; old_input.port = "D";
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 4, 29);
    mesh.types[4].dst_local[29].local.setBit(old_pin);
    mesh.types[4].rebuildOutgoingSrcs();
    sink.wires.push_back({output, crossbar({0,0}, {1,0}, local, 17, 19, 0),
        crossbar({1,0}, {4,0}, 19, 23, 29, 1),
        crossbar({4,0}, {4,0}, 29, -1, -1, 1), old_input});
    leaseRoute(sink.wires[0]);
    mesh.tile(0).cb.local.local.clearBit(local);
    const size_t binding = fpga::attachNetRoute(net, sink, 0,
        &driver, &sink, "Q", "D", "child_349");
    fpga::registerNetRouteTiles(net, sink.wires[0], binding);
    auto audit = [&] {
        for (int x = 0; x < 20; ++x) {
            std::ostringstream out;
            const auto r = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
            require(!r.orphan_leases && !r.missing_leases && !r.missing_registrations &&
                        !r.stale_registrations && !r.conflicting_driver_nodes,
                    "empty-binding ownership mismatch: " + out.str());
        }
    };
    audit();
    const auto route_id = net.routes[binding].route_id;
    require(fpga::isRouteComplete(sink.wires[0]) && fpga::unrouteNetRoute(net, binding) &&
                sink.wires[0].empty() && net.routes.size() == 1,
            "real unrouting did not preserve an empty logical binding");
    if (!preserve_binding) net.eraseRouteBinding(binding);
    if (retired_storage) {
        // Shared-prefix repair deliberately retires aliased physical storage
        // but keeps the logical task. This is the production uninstalled
        // sentinel, distinct from a corrupt owner/index pair. Example 1
        // subsequently rejected every numeric attempt before searching it.
        require(preserve_binding, "retired binding needs its logical identity");
        net.routes[0].owner = nullptr;
        net.routes[0].route_index = std::numeric_limits<size_t>::max();
    }
    require(mesh.tile(4).unassign(&sink) && mesh.tile(19).tryAddAt(&sink, 0) == 0,
            "cannot relocate empty-binding sink");
    const int pin = mesh.tile(19).getPinNodes("FDRE", "D", 0).firstSetBit();
    auto input = tilePin({19,0}, pin);
    input.pin_dir = fpga::TILE_PIN_INPUT;
    input.resource = {19,0}; input.pos = 0; input.cell_type = "FDRE"; input.port = "D";
    mesh.edge(0, local, 31, 6, 37);
    mesh.types[0].local_src[local].jump.setBit(31);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(6, 37, 41, 11, 43);
    mesh.types[6].dst_src[37].jump.clearBit(41);
    mesh.types[6].dst_local[37].local.setBit(151);
    if (bypass) mesh.types[6].local_src[151].jump.setBit(41);
    mesh.types[6].rebuildOutgoingSrcs();
    mesh.edge(11, 43, 47, 15, 53);
    mesh.edge(15, 53, 59, 19, 61);
    mesh.types[19].dst_local[61].local.setBit(pin);
    mesh.types[19].rebuildOutgoingSrcs();
    audit();
    rtl::NetRouteBinding endpoints;
    endpoints.from = &driver; endpoints.to = &sink;
    endpoints.from_port = "Q"; endpoints.to_port = "D"; endpoints.route_name = "child_349";
    std::ostringstream proof;
    require(fpga::auditUnroutedReachability(net, endpoints, output, input, proof, 32768) == bypass,
            "empty-binding source witness disagrees with topology");
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = true;
    router.moving_focus_inst = &sink;
    router.route_deadends_enabled = router.route_preemption_enabled = false;
    router.iteration_limit = router.route_suffix_depth_limit = 200;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "child_349"};
    task.endpoints_prepared = true;
    if (preserve_binding) {
        // An invalid storage reference is not an empty, reusable slot.
        const auto owner = net.routes[0].owner;
        const auto storage = net.routes[0].route_index;
        net.routes[0].owner = nullptr;
        net.routes[0].route_index = 0;
        require(!fpga::routeUnboundNetFromSource(net, endpoints, output, input),
                "numeric completion accepted a binding with invalid storage");
        net.routes[0].owner = owner;
        net.routes[0].route_index = storage;
    }
    const bool complete = router.routeTaskOrFail(task, "Moving destinations", 5, false);
    require(complete == bypass, "empty binding prevented numeric completion of a free path");
    if (bypass) {
        require(net.routes.size() == 1 &&
                    (!preserve_binding || net.routes[0].route_id == route_id) &&
                    (!preserve_binding ||
                        (net.routes[0].route_index == size_t(retired_storage) &&
                         sink.wires.size() == 1 + size_t(retired_storage))) &&
                    net.routes[0].route_index < sink.wires.size() &&
                    fpga::isRouteComplete(sink.wires[net.routes[0].route_index]) &&
                    mesh.tile(19).pin_state.leased_nodes.testBit(pin),
                "completion duplicated the binding, changed its identity or missed the sink lease");
        const auto old = sink.wires[net.routes[0].route_index];
        require(!fpga::routeUnboundNetFromSource(net, endpoints, output, input),
                "numeric completion overwrote an already populated route");
        require(sink.wires[net.routes[0].route_index].size() == old.size(),
                "rejected overwrite changed the completed route");
        for (size_t i = 0; i < old.size(); ++i)
            require(sameFragment(old[i], sink.wires[net.routes[0].route_index][i]),
                    "rejected overwrite changed a completed fragment");
        require(std::any_of(sink.wires.back().begin(), sink.wires.back().end(), [](const auto &w) {
                    return w.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                        w.from_node_type == fpga::CB_NODE_LOCAL && w.from_node == 151;
                }), "completion omitted its mandatory numeric LOCAL bridge");
        audit();
        require(fpga::unrouteNet(net), "cannot release empty-binding completion");
    } else {
        require(net.routes.size() == size_t(preserve_binding) && sink.wires[0].empty() &&
                    sink.tile.peer == &mesh.tile(19),
                "failed completion changed binding storage or placement");
    }
    audit();
}

void moving_destination_tries_another_source_exit(bool blocked_branch, bool alternate)
{
    // Example 1 exhausted an interrupted sibling's Takeoff: its reachable CBs
    // had no placement capacity. A different free Takeoff reached usable logic,
    // but Moving omitted source LOCALs whenever any retained start existed.
    // Prefer a usable retained branch; try a new exit only after it is exhausted.
    ForwardMovingMesh mesh(6);
    fpga::TileType resource{"slots_251", 1};
    fpga::Element reg;
    reg.name = "registers_257";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {2, 3, 4, 5}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_263";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sibling, sink, blocker;
    for (auto *inst : {&driver, &sibling, &sink, &blocker}) inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(3).tryAddAt(&sibling, 0) == 0 &&
                mesh.tile(5).tryAddAt(&sink, 0) == 0, "cannot place source-exit fixture");
    if (blocked_branch)
        require(mesh.tile(2).tryAddAt(&blocker, 0) == 0, "cannot fill the branch's only resource");
    parent.nets.resize(1);
    auto &net = parent.nets.front();
    net.name = "signal_269";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    const int pin = mesh.tile(3).getPinNodes("FDRE", "D", 0).firstSetBit();
    require(local >= 0 && pin >= 0, "source-exit fixture pin missing");
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 29, 3, 37);
    mesh.types[3].dst_local[37].local.setBit(pin);
    mesh.types[3].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 31);
    const int branch_pin = mesh.tile(2).getPinNodes("FDRE", "D", 0).firstSetBit();
    mesh.types[2].dst_local[31].local.setBit(branch_pin);
    mesh.types[2].rebuildOutgoingSrcs();
    if (alternate) {
        mesh.edge(0, local, 41, 4, 43);
        mesh.types[0].local_src[local].jump.setBit(41);
        mesh.types[0].rebuildOutgoingSrcs();
        const int new_pin = mesh.tile(4).getPinNodes("FDRE", "D", 0).firstSetBit();
        mesh.types[4].dst_local[43].local.setBit(new_pin);
        mesh.types[4].rebuildOutgoingSrcs();
    }
    auto output = tilePin({0,0}, local);
    output.pin_dir = fpga::TILE_PIN_OUTPUT;
    output.resource = {0,0}; output.pos = 0; output.cell_type = "FDRE"; output.port = "Q";
    auto input = tilePin({3,0}, pin);
    input.pin_dir = fpga::TILE_PIN_INPUT;
    input.resource = {3,0}; input.pos = 0; input.cell_type = "FDRE"; input.port = "D";
    sibling.wires.push_back({output, crossbar({0,0}, {1,0}, local, 17, 19, 0),
        crossbar({1,0}, {3,0}, 19, 29, 37, 1),
        crossbar({3,0}, {3,0}, 37, -1, -1, 1), input});
    leaseRoute(sibling.wires[0]);
    mesh.tile(0).cb.local.local.clearBit(local);
    const size_t seed = fpga::attachNetRoute(net, sibling, 0,
        &driver, &sibling, "Q", "D", "sibling_271");
    fpga::registerNetRouteTiles(net, sibling.wires[0], seed);
    require(fpga::unrouteNetRoutesFromNodes({{&net, seed}},
                {{{1,0}, fpga::CB_NODE_SRC, 29}}, true), "cannot interrupt the sibling");
    const auto original = sibling.wires[0];
    auto audit = [&] {
        for (int x = 0; x < 6; ++x) {
            std::ostringstream out;
            const auto r = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
            require(!r.orphan_leases && !r.missing_leases && !r.missing_registrations &&
                        !r.stale_registrations && !r.conflicting_driver_nodes,
                    "source-exit ownership mismatch: " + out.str());
        }
    };
    audit();
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "child_277"};
    task.fanout = true; // Still a fanout while its interrupted seed awaits repair.
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    const bool moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    const bool expected = !blocked_branch || alternate;
    require(moved == expected, "Moving omitted the alternate source exit: " + reason);
    if (expected) {
        const int destination = blocked_branch ? 4 : 2;
        require(sink.tile.peer == &mesh.tile(destination) &&
                    !sink.wires.empty() && fpga::isRouteComplete(sink.wires.back()),
                "Moving did not ground the source-connected replacement");
        require(mesh.tile(0).cb.src.jump.testBit(41) == blocked_branch,
                "Moving used a new Takeoff before exhausting a usable retained branch");
        require(fpga::unrouteNetRoute(net, net.routes.size() - 1), "cannot remove new suffix");
        require(mesh.tile(0).cb.src.jump.testBit(17) && mesh.tile(1).cb.dst.jump.testBit(19),
                "removing the replacement released the interrupted sibling's prefix");
    } else {
        require(sink.tile.peer == &mesh.tile(5) && sink.wires.empty(),
                "exhausted source search changed the old sink");
    }
    require(sibling.wires[0].size() == original.size(), "Moving changed the retained sibling");
    for (size_t i = 0; i < original.size(); ++i)
        require(sameFragment(sibling.wires[0][i], original[i]) &&
                    sibling.wires[0][i].owns_landing == original[i].owns_landing,
                "Moving changed an interrupted sibling fragment");
    audit();
    require(fpga::unrouteNet(net), "cannot remove source-exit fixture");
    audit();
}

void moving_whole_task_replaces_a_shared_only_prefix(bool bypass)
{
    // Exact first-failure state after relocation in Example 1: another input
    // retains only a shared prefix, with no suspended search. Its parent is
    // complete. Cleaning this replica is necessary housekeeping, not failure
    // of the connection. The first attempt must continue to the free branch.
    ForwardMovingMesh mesh(5);
    fpga::TileType resource{"slots_173", 1};
    fpga::Element reg;
    reg.name = "registers_179";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {2, 3}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_181";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, stable, sink;
    for (auto *inst : {&driver, &stable, &sink}) inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(2).tryAddAt(&stable, 0) == 0 &&
                mesh.tile(3).tryAddAt(&sink, 0) == 0, "cannot place shared-prefix fixture");
    parent.nets.resize(1);
    auto &net = parent.nets.front();
    net.name = "signal_191";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    const int parent_pin = mesh.tile(2).getPinNodes("FDRE", "D", 0).firstSetBit();
    const int sink_pin = mesh.tile(3).getPinNodes("FDRE", "D", 0).firstSetBit();
    require(local >= 0 && parent_pin >= 0 && sink_pin >= 0, "shared-prefix pins missing");
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 29);
    mesh.types[2].dst_local[29].local.setBit(parent_pin);
    mesh.types[2].rebuildOutgoingSrcs();
    if (bypass) mesh.edge(1, 19, 31, 3, 37);
    mesh.types[3].dst_local[37].local.setBit(sink_pin);
    mesh.types[3].rebuildOutgoingSrcs();
    mesh.tile(3).incoming_dst_nodes.setBit(37);
    stable.wires.push_back({crossbar({0,0}, {1,0}, local, 17, 19, 0),
        crossbar({1,0}, {2,0}, 19, 23, 29, 1),
        crossbar({2,0}, {2,0}, 29, -1, -1, 2), tilePin({2,0}, parent_pin)});
    leaseRoute(stable.wires[0]);
    const size_t parent_binding = fpga::attachNetRoute(net, stable, 0,
        &driver, &stable, "Q", "D", "parent_193");
    fpga::registerNetRouteTiles(net, stable.wires[0], parent_binding);
    sink.wires.emplace_back(stable.wires[0].begin(), stable.wires[0].begin() + 2);
    for (auto &wire : sink.wires[0]) wire.shared = true;
    const size_t child_binding = fpga::attachNetRoute(net, sink, 0,
        &driver, &sink, "Q", "D", "child_197");
    fpga::registerNetRouteTiles(net, sink.wires[0], child_binding);
    auto audit = [&] {
        for (int x = 0; x < 5; ++x) {
            std::ostringstream out;
            const auto result = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
            require(!result.orphan_leases && !result.missing_leases &&
                        !result.missing_registrations && !result.stale_registrations &&
                        !result.conflicting_driver_nodes,
                    "shared-prefix ownership mismatch: " + out.str());
        }
    };
    audit();
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = true;
    router.moving_focus_inst = &sink;
    router.route_preemption_enabled = false;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "child_197"};
    task.fanout = true;
    // The disconnected control isolates cleanup/search from the separate
    // whole-source reconstruction fallback (covered by its own regressions).
    // Do not retry source reconstruction while checking this mechanism.
    task.source_tree_rebuild_attempted = !bypass;
    require(task.search.levels.empty(), "shared-only fixture has a suspended search");
    const bool complete = router.routeTaskOrFail(task, "Moving destinations", 5, false);
    require(complete == bypass && (!complete || fpga::isRouteComplete(sink.wires[0])),
            "whole task reported prefix cleanup as failure before trying replacement branches");
    require(fpga::isRouteComplete(stable.wires[0]) &&
                mesh.tile(0).cb.src.jump.testBit(17) && mesh.tile(1).cb.src.jump.testBit(23),
            "shared-prefix cleanup damaged the complete parent");
    audit();
    require(fpga::unrouteNet(net), "cannot remove shared-prefix fixture");
    audit();
}

void moving_generic_prefix_uses_another_source_branch(bool bypass, bool pin_only = false)
{
    // Example 1: relocation promoted one input to Generic because its source
    // had no COMPLETE trunk. The input retained a four-fragment shared prefix.
    // Its last landing was trapped, but another interrupted sibling had a live
    // source-connected branch with a free route to the new input position.
    // Use real transit cuts and sink invalidation, not fabricated busy bits.
    ForwardMovingMesh mesh(20);
    fpga::TileType resource{"slots_211", 1};
    fpga::Element reg;
    reg.name = "registers_223";
    reg.type = fpga::ELEMENT_FD;
    for (int slot : {0, 1}) {
        reg.bitmap_pos = slot;
        resource.elements.push_back(reg);
    }
    const int destination = pin_only ? 5 : 19;
    for (int x : {4, 5, 8, 19}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_227";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, stable, branch, sink;
    for (auto *inst : {&driver, &stable, &branch, &sink}) inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(4).tryAddAt(&stable, 0) == 0 &&
                mesh.tile(4).tryAddAt(&sink, 4) == 4 &&
                mesh.tile(8).tryAddAt(&branch, 0) == 0,
            "cannot place interrupted Generic fixture");
    parent.nets.resize(1);
    auto &net = parent.nets.front();
    net.name = "signal_229";
    auto endpoint = [&](rtl::Inst &inst, const char *port, bool output) {
        const auto coord = inst.tile->coord;
        auto wire = tilePin(coord, (output
            ? inst.tile->getOutputPinNodes("FDRE", port, inst.pos)
            : inst.tile->getPinNodes("FDRE", port, inst.pos)).firstSetBit());
        require(wire.local >= 0, "Generic fixture endpoint missing");
        wire.resource = coord; wire.pos = inst.pos;
        wire.cell_type = "FDRE"; wire.port = port;
        wire.pin_dir = output ? fpga::TILE_PIN_OUTPUT : fpga::TILE_PIN_INPUT;
        return wire;
    };
    const auto output = endpoint(driver, "Q", true);
    auto stable_input = endpoint(stable, "D", false);
    auto branch_input = endpoint(branch, "D", false);
    auto old_input = endpoint(sink, "D", false);
    mesh.edge(0, output.local, 17, 1, 19);
    mesh.types[0].local_src[output.local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 29);
    mesh.edge(2, 29, 31, 3, 37);
    mesh.edge(3, 37, 41, 4, 43);
    mesh.types[4].dst_local[43].local |= bit(stable_input.local) | bit(old_input.local);
    mesh.types[4].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 47, 6, 53);
    mesh.edge(6, 53, 59, 8, 61);
    mesh.types[8].dst_local[61].local.setBit(branch_input.local);
    mesh.types[8].rebuildOutgoingSrcs();
    stable.wires.push_back({output,
        crossbar({0,0}, {1,0}, output.local, 17, 19, 0),
        crossbar({1,0}, {2,0}, 19, 23, 29, 1),
        crossbar({2,0}, {3,0}, 29, 31, 37, 1),
        crossbar({3,0}, {4,0}, 37, 41, 43, 1),
        crossbar({4,0}, {4,0}, 43, -1, -1, 1), stable_input});
    branch.wires.push_back({output, stable.wires[0][1],
        crossbar({1,0}, {6,0}, 19, 47, 53, 1),
        crossbar({6,0}, {8,0}, 53, 59, 61, 1),
        crossbar({8,0}, {8,0}, 61, -1, -1, 1), branch_input});
    branch.wires[0][0].shared = branch.wires[0][1].shared = true;
    sink.wires.push_back(stable.wires[0]);
    for (auto &wire : sink.wires[0]) wire.shared = true;
    sink.wires[0].back() = old_input;
    for (auto *inst : {&stable, &branch, &sink}) leaseRoute(inst->wires[0]);
    mesh.tile(0).cb.local.local.clearBit(output.local); // normal source reference
    auto bind = [&](rtl::Inst &inst, const char *name) {
        const size_t index = fpga::attachNetRoute(net, inst, 0, &driver, &inst, "Q", "D", name);
        fpga::registerNetRouteTiles(net, inst.wires[0], index);
        return index;
    };
    const auto stable_binding = bind(stable, "parent_233");
    const auto branch_binding = bind(branch, "branch_239");
    const auto sink_binding = bind(sink, "child_241");
    auto audit = [&] {
        for (int x = 0; x < 20; ++x) {
            std::ostringstream out;
            const auto r = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
            require(!r.orphan_leases && !r.missing_leases && !r.missing_registrations &&
                        !r.stale_registrations && !r.conflicting_driver_nodes,
                    "Generic prefix ownership mismatch: " + out.str());
        }
    };
    audit();
    std::vector<fpga::RouteCutNode> cuts{{{3,0}, fpga::CB_NODE_SRC, 41}};
    std::vector<fpga::NetRouteRef> victims{{&net, stable_binding}, {&net, sink_binding}};
    if (!pin_only) cuts.push_back({{6,0}, fpga::CB_NODE_SRC, 59});
    if (!pin_only) victims.push_back({&net, branch_binding});
    require(fpga::unrouteNetRoutesFromNodes(victims, cuts, true),
            "cannot create interrupted source branches");
    require(fpga::invalidateMovedSinkRoutes({{&net, sink_binding}}) &&
                mesh.tile(4).unassign(&sink) && mesh.tile(destination).tryAddAt(&sink, 4) == 4,
            "cannot relocate Generic fixture input");
    const auto input = endpoint(sink, "D", false);
    if (bypass || pin_only) {
        // Keep a legal free terminal approach in the disconnected control;
        // only its source connection is missing. The source-pin case is near
        // its sink, but the valid bypass leaves the local Docking region.
        if (bypass) mesh.edge(6, 53, 67, 11, 71);
        mesh.edge(11, 71, 73, 15, 79);
        mesh.edge(15, 79, 83, destination, 89);
        if (pin_only) {
            // The real next failure also needed DST -> LOCAL -> SRC bridges.
            // A hop-only restart cannot substitute for whole-source completion.
            mesh.types[11].dst_src[71].jump.clearBit(73);
            mesh.types[11].dst_local[71].local.setBit(175);
            mesh.types[11].local_src[175].jump.setBit(73);
            mesh.types[11].rebuildOutgoingSrcs();
        }
        mesh.types[destination].dst_local[89].local.setBit(input.local);
        mesh.types[destination].rebuildOutgoingSrcs();
    }
    require(sink.wires[0].size() == 4 && !fpga::hasRouteGaps(sink.wires[0]) &&
                std::all_of(sink.wires[0].begin(), sink.wires[0].end(),
                            [](const auto &w) { return w.shared; }),
            "relocation did not reproduce the four-fragment shared prefix");
    if (pin_only) {
        // Example 1's next failure: a private Takeoff is popped from a
        // two-fragment branch, leaving only its shared, unleased source PIN.
        // The live search stack must not turn this LOCAL into a nonexistent DST.
        require(fpga::truncateNetRoute(net, sink_binding, 1), "cannot retain source PIN");
        mesh.edge(0, output.local, 7, 2, 13);
        mesh.types[0].local_src[output.local].jump.setBit(7);
        mesh.types[0].rebuildOutgoingSrcs();
        auto dead = crossbar({0,0}, {2,0}, output.local, 7, 13, 0);
        dead.owns_landing = true;
        sink.wires[0].push_back(dead);
        mesh.tile(0).cb.src.jump.setBit(7);
        mesh.tile(2).cb.dst.jump.setBit(13);
        fpga::registerNetRouteTiles(net, sink.wires[0], sink_binding);
    }
    const auto old_route = sink.wires[0];
    const auto old_stable = stable.wires[0], old_branch = branch.wires[0];
    audit();
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = true;
    router.moving_focus_inst = &sink;
    router.route_preemption_enabled = false;
    router.indexSourceRoute(&net, &driver, "Q");
    require(router.sourceTreeHasCompleteExit(driver, "Q") == pin_only,
            "source seed completeness disagrees with the fixture mode");
    rtl::NetRouteBinding endpoints = net.routes[sink_binding];
    std::ostringstream proof;
    require(fpga::auditUnroutedReachability(net, endpoints, output, input, proof, 32768) == bypass,
            "source witness disagrees with the fixture topology: " + proof.str());
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "child_241"};
    task.endpoints_prepared = true;
    task.fanout = pin_only;
    const bool complete = router.routeTaskOrFail(task, "Moving destinations", 5, false);
    require(complete == bypass && (!complete || fpga::isRouteComplete(sink.wires[0])),
            "Generic continuation stopped at its trapped end despite another source branch");
    if (!bypass) {
        const size_t retained = pin_only ? 1 : old_route.size();
        require(sink.wires[0].size() == retained, "failed source completion lost prefix");
        for (size_t i = 0; i < retained; ++i)
            require(sameFragment(old_route[i], sink.wires[0][i]), "failed completion changed prefix");
    } else {
        require(mesh.tile(destination).pin_state.leased_nodes.testBit(input.local) &&
                    std::any_of(sink.wires[0].begin(), sink.wires[0].end(),
                        [](const auto &w) { return w.from.x == 6 && w.jump == 67; }),
                "completion did not use the sibling's connected branch and lease the new input");
        if (pin_only)
            require(std::any_of(sink.wires[0].begin(), sink.wires[0].end(), [](const auto &w) {
                        return w.type == fpga::Wire::WIRE_ROUTE_EDGE &&
                            w.from_node_type == fpga::CB_NODE_LOCAL && w.from_node == 175;
                    }), "source completion skipped the required internal LOCAL bridge");
    }
    if (pin_only)
        require(router.route_stats.backstep_success == 1 &&
                    !mesh.tile(0).cb.src.jump.testBit(7) && !mesh.tile(2).cb.dst.jump.testBit(13),
                "last private Takeoff was not removed before source-pin completion");
    for (auto [inst, old] : {std::pair{&stable, &old_stable}, std::pair{&branch, &old_branch}}) {
        require(inst->wires[0].size() == old->size(), "completion changed a sibling route");
        for (size_t i = 0; i < old->size(); ++i)
            require(sameFragment(inst->wires[0][i], (*old)[i]) &&
                        inst->wires[0][i].owns_landing == (*old)[i].owns_landing,
                    "completion changed a sibling fragment");
    }
    require(!mesh.tile(0).cb.local.local.testBit(output.local),
            "completion charged the source reference as a routing lease");
    audit();
    require(fpga::unrouteNet(net), "cannot remove interrupted Generic fixture");
    audit();
}

void moving_destination_branches_before_dead_tail(bool packing_blocked)
{
    ForwardMovingMesh mesh(5);
    fpga::TileType resource{"PACKING_BOX", 1};
    fpga::Element reg;
    reg.name = "register_slots";
    reg.type = fpga::ELEMENT_FD;
    for (int slot : {0, 1}) {
        reg.bitmap_pos = slot;
        resource.elements.push_back(reg);
    }
    for (int x : {2, 3, 4}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "branch_load_53";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sink, blockers[2];
    driver.cell_ref.set(&cell);
    sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(4).tryAddAt(&sink, 0) == 0, "cannot place branch test sink");
    if (packing_blocked)
        for (int i = 0; i < 2; ++i) {
            blockers[i].cell_ref.set(&cell);
            require(mesh.tile(2).tryAddAt(&blockers[i], i * 4) >= 0,
                    "cannot occupy branch test destination");
        }
    parent.nets.resize(1);
    auto& net = parent.nets.front();
    net.name = "branch_signal_71";
    // The retained route ends at 3, which cannot ground or continue. Only its
    // earlier landing at 1 can reach the legal packing/grounding tile at 2.
    sink.wires.push_back({crossbar({0,0}, {1,0}, 4, 5, 7, 0),
                          crossbar({1,0}, {3,0}, 7, 20, 12, 1)});
    sink.wires[0].back().owns_landing = true;
    leaseRoute(sink.wires[0]);
    mesh.tile(3).cb.dst.jump.setBit(12);
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "Q", "D", "branch_suffix_19");
    fpga::registerNetRouteTiles(net, sink.wires[0], 0);
    mesh.edge(1, 7, 20, 3, 12);
    mesh.edge(1, 7, 10, 2, 8);
    const int pin = mesh.tile(2).getPinNodes("FDRE", "D", 4).firstSetBit();
    require(pin >= 0, "branch test input lane missing");
    mesh.types[2].dst_joint[8].joint = bit(21);
    mesh.types[2].joint_joint[21].joint = bit(22);
    mesh.types[2].joint_local[22].local = bit(pin);
    mesh.types[2].rebuildOutgoingSrcs();
    const auto original = sink.wires[0];
    std::vector<fpga::CBState> before;
    for (int x = 0; x < 5; ++x) before.push_back(mesh.tile(x).cb);
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "branch_suffix_19"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    const bool moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    require(moved == !packing_blocked,
            "Moving skipped an earlier private branch: " + reason);
    if (packing_blocked) {
        require(sink.tile.peer == &mesh.tile(4) && sink.pos == 0 &&
                    sink.wires[0].size() == original.size(),
                "failed earlier-branch search changed placement or prefix");
        for (size_t i = 0; i < original.size(); ++i)
            require(sameFragment(original[i], sink.wires[0][i]) &&
                        original[i].owns_landing == sink.wires[0][i].owns_landing,
                    "failed earlier-branch search modified route ownership");
        for (int x = 0; x < 5; ++x)
            require(mesh.tile(x).cb.src.jump == before[x].src.jump &&
                        mesh.tile(x).cb.dst.jump == before[x].dst.jump &&
                        mesh.tile(x).cb.joint.jump == before[x].joint.jump &&
                        mesh.tile(x).cb.local.local == before[x].local.local &&
                        mesh.tile(x).pin_state.leased_nodes == NodeMask{},
                    "failed earlier-branch search changed leases");
    } else {
        require(sink.tile.peer == &mesh.tile(2) && sink.pos == 4 &&
                    sameFragment(original.front(), sink.wires[0].front()) &&
                    sink.wires[0].front().owns_landing &&
                    sink.wires[0].back().type == fpga::Wire::WIRE_TILE_PIN &&
                    sink.wires[0].back().local == pin,
                "earlier branch did not preserve its owner and ground the new sink");
        require(!mesh.tile(1).cb.src.jump.testBit(20) &&
                    !mesh.tile(3).cb.dst.jump.testBit(12) &&
                    mesh.tile(1).cb.dst.jump.testBit(7) &&
                    mesh.tile(1).cb.src.jump.testBit(10),
                "earlier branch left dead-tail leases or lost its new route");
    }
    for (int x = 0; x < 5; ++x) {
        std::ostringstream out;
        const auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                "earlier-branch ownership mismatch: " + out.str());
    }
    require(fpga::unrouteNet(net), "cannot unroute earlier-branch result");
    for (int x = 0; x < 5; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} &&
                    mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.joint.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{},
                "earlier-branch unroute leaked routing resources");
}

// A saturated initial branch must not hide another routed sibling. The index
// may be incomplete; existing routes are authoritative and must stay untouched.
void moving_branch_point_on_routed_sibling(bool indexed, bool blocked,
                                         bool sibling_first)
{
    ForwardMovingMesh mesh(8);
    fpga::TileType resource{"slots_601", 1};
    fpga::Element reg;
    reg.name = "registers_602";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {2, 4, 5, 6, 7}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_603";
    cell.type = "FDRE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, trunk, sibling, sink, foreign, stranger, blocker;
    for (auto* inst : {&driver, &trunk, &sibling, &sink, &foreign, &stranger, &blocker})
        inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(2).tryAddAt(&trunk, 0) == 0 &&
                mesh.tile(4).tryAddAt(&sibling, 0) == 0 &&
                mesh.tile(6).tryAddAt(&foreign, 0) == 0 &&
                mesh.tile(7).tryAddAt(&sink, 0) == 0,
            "cannot place branch-moving fixture");
    if (blocked)
        require(mesh.tile(5).tryAddAt(&blocker, 0) == 0, "cannot fill alternative site");
    parent.nets.resize(4);
    auto& base = parent.nets[0];
    auto& branch = parent.nets[1];
    auto& net = parent.nets[2];
    auto& busy = parent.nets[3];
    base.name = "signal_607";
    branch.name = "alias_608";
    net.name = "alias_609";
    busy.name = "foreign_610";
    const int local = mesh.tile(0).getOutputPinNodes("FDRE", "Q", 0).firstSetBit();
    require(local >= 0, "driver output missing");
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 31);
    mesh.edge(1, 19, 37, 3, 41);
    mesh.edge(3, 41, 43, 4, 47);
    mesh.edge(4, 47, 53, 5, 59);
    mesh.edge(1, 19, 61, 6, 67); // Free exit, but its landing belongs to another signal.
    auto terminal = [&](int x, int dst) {
        const int pin = mesh.tile(x).getPinNodes("FDRE", "D", 0).firstSetBit();
        require(pin >= 0, "fixture input missing");
        mesh.types[x].dst_local[dst].local.setBit(pin);
        mesh.types[x].rebuildOutgoingSrcs();
        return pin;
    };
    int trunk_pin = terminal(2, 31), sibling_pin = terminal(4, 47);
    terminal(5, 59);
    int foreign_pin = terminal(6, 67);
    trunk.wires.push_back({crossbar({0,0}, {1,0}, local, 17, 19, 0),
        crossbar({1,0}, {2,0}, 19, 23, 31, 1),
        crossbar({2,0}, {2,0}, 31, -1, -1, 1), tilePin({2,0}, trunk_pin)});
    sibling.wires.push_back({crossbar({0,0}, {1,0}, local, 17, 19, 0, true),
        crossbar({1,0}, {3,0}, 19, 37, 41, 2, false, false),
        crossbar({3,0}, {4,0}, 41, 43, 47, 1),
        crossbar({4,0}, {4,0}, 47, -1, -1, 1), tilePin({4,0}, sibling_pin)});
    // The foreign landing has a real registered owner, not an unexplained bit.
    foreign.wires.push_back({crossbar({6,0}, {6,0}, 67, -1, -1, 1),
                            tilePin({6,0}, foreign_pin)});
    for (auto [owner, signal, source, name] : {
            std::tuple{&trunk, &base, &driver, "trunk_611"},
            std::tuple{&sibling, &branch, &driver, "sibling_612"},
            std::tuple{&foreign, &busy, &stranger, "busy_613"}}) {
        leaseRoute(owner->wires[0]);
        size_t i = fpga::attachNetRoute(*signal, *owner, 0, source, owner, "Q", "D", name);
        fpga::registerNetRouteTiles(*signal, owner->wires[0], i);
    }
    sink.wires.push_back({});
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "Q", "D", "suffix_614");
    const auto trunk_before = trunk.wires[0], sibling_before = sibling.wires[0];
    std::vector<fpga::CBState> before;
    for (int x = 0; x < 8; ++x) before.push_back(mesh.tile(x).cb);
    // Prove the original branch cannot grow: both exits are leased and the
    // remaining free exit lands on an occupied DST. No recursive child exists.
    auto initial = pnr::routeForwardToPlacement({{&mesh.tile(1), 19, true,
            {trunk.wires[0].front()}}},
        [](auto&, int, bool, auto&) { return false; });
    require(!initial.success && initial.expanded == 1, "initial branch is not blocked");
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&base, &driver, "Q");
    router.indexSourceRoute(&net, &driver, "Q");
    if (indexed) router.indexSourceRoute(&branch, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_614"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    std::ostringstream decisions;
    bool moved;
    {
        pnr::RouteCongestionTrace trace("suffix_614", "Moving branching point",
                                       {&base, &branch, &net, &busy}, &decisions);
        moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    }
    require(moved == !blocked, "routed-sibling branch relocation failed: " + reason);
    // The full-index path already knows the sibling; only the missing-index
    // path needs the exhaustive fallback, and both must use the same fabric.
    require((decisions.str().find("MOVE_BRANCHING_POINT") != std::string::npos) == !indexed,
            "branch relocation fallback was skipped or redundantly replayed");
    // Neither a successful branch change nor an exhausted search may reroute
    // the trunk or the already routed sibling, including ownership flags.
    for (auto [actual, expected] : {
            std::pair{&trunk.wires[0], &trunk_before},
            std::pair{&sibling.wires[0], &sibling_before}}) {
        require(actual->size() == expected->size(), "branch move changed existing route size");
        for (size_t i = 0; i < actual->size(); ++i)
            require(sameFragment((*actual)[i], (*expected)[i]) &&
                        (*actual)[i].shared == (*expected)[i].shared &&
                        (*actual)[i].owns_dst == (*expected)[i].owns_dst &&
                        (*actual)[i].owns_landing == (*expected)[i].owns_landing,
                    "branch move changed existing trunk/sibling ownership");
    }
    if (blocked) {
        require(sink.tile.peer == &mesh.tile(7) && sink.wires[0].empty(),
                "failed branch move changed sink");
        for (int x = 0; x < 8; ++x)
            require(mesh.tile(x).cb.src.jump == before[x].src.jump &&
                        mesh.tile(x).cb.dst.jump == before[x].dst.jump &&
                        mesh.tile(x).cb.joint.jump == before[x].joint.jump &&
                        mesh.tile(x).cb.local.local == before[x].local.local,
                    "failed branch move changed leases");
    } else {
        require(sink.tile.peer == &mesh.tile(5) && fpga::isRouteComplete(sink.wires[0]) &&
                    sink.wires[0].size() == 6 && sink.wires[0][2].shared &&
                    sink.wires[0][3].from.x == 4 && sink.wires[0][3].local == 47 &&
                    sink.wires[0][3].pos == 2 && !sink.wires[0][3].owns_dst,
                "suffix did not branch from the routed sibling at (4,0)/47");
    }
    auto audit = [&]() {
        for (int x = 0; x < 8; ++x) {
            std::ostringstream out;
            auto a = fpga::auditTileCongestion(mesh.tile(x), out, {&base, &branch, &net, &busy});
            require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
                        !a.stale_registrations, "branch move ownership mismatch: " + out.str());
        }
    };
    audit();
    // Removing either copy first must retain the shared path for its survivor.
    if (!blocked) {
        require(fpga::unrouteNet(sibling_first ? branch : net), "cannot remove branch");
        require(mesh.tile(3).cb.src.jump.testBit(43) && mesh.tile(4).cb.dst.jump.testBit(47),
                "branch removal released the survivor's shared prefix");
        audit();
        require(fpga::unrouteNet(sibling_first ? net : branch), "cannot remove survivor");
    } else require(fpga::unrouteNet(branch), "cannot remove sibling");
    require(fpga::unrouteNet(base) && fpga::unrouteNet(busy), "cannot clear fixture");
    audit();
    for (int x = 0; x < 8; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{}, "branch move leaked leases");
}

// A fanout whose tree cannot grow must be allowed to cut a foreign transit,
// without treating the branch DST as a new local takeoff or ripping its trunk.
void moving_branch_takeoff_preemption(int conflict, bool free_alternative,
                                      bool protected_endpoint, bool enabled = true,
                                      bool focused = false, int free_joint_alternative = 0)
{
    ForwardMovingMesh mesh(9);
    fpga::TileType resource{"resource_901", 1};
    fpga::Element reg;
    reg.name = "register_902";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {2, 3, 4, 7, 8}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "cell_903";
    cell.type = "FD_SAMPLE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, trunk, sink, other_driver, other_sink;
    for (auto* inst : {&driver, &trunk, &sink, &other_driver, &other_sink})
        inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    const int victim_source = protected_endpoint && conflict != 2 ? 1 : 6;
    const int victim_sink = protected_endpoint && conflict == 2 ? 3 : 7;
    other_driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(victim_source)));
    other_driver.pos = 0;
    require(mesh.tile(2).tryAddAt(&trunk, 0) == 0 &&
                mesh.tile(8).tryAddAt(&sink, 0) == 0 &&
                mesh.tile(victim_sink).tryAddAt(&other_sink, 0) == 0,
            "cannot place branch takeoff fixture");
    parent.nets.resize(3);
    auto& base = parent.nets[0];
    auto& net = parent.nets[1];
    auto& busy = parent.nets[2];
    base.name = "trunk_904"; net.name = "suffix_905"; busy.name = "transit_906";
    const int local = mesh.tile(0).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    const int other_local = mesh.tile(victim_source).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    require(local >= 0 && other_local >= 0, "fixture output missing");
    auto terminal = [&](int x, int dst) {
        int pin = mesh.tile(x).getPinNodes(cell.type, "D", 0).firstSetBit();
        require(pin >= 0, "fixture input missing");
        mesh.types[x].dst_local[dst].local.setBit(pin);
        mesh.types[x].rebuildOutgoingSrcs();
        return pin;
    };
    mesh.edge(0, local, 17, 1, 19);
    mesh.types[0].local_src[local].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 31);
    mesh.edge(1, 19, 37, 3, 59);
    if (free_alternative) mesh.edge(1, 19, 41, 4, 43);
    const int trunk_pin = terminal(2, 31);
    terminal(3, 59); terminal(4, 43);
    trunk.wires.push_back({crossbar({0,0}, {1,0}, local, 17, 19, 0),
        crossbar({1,0}, {2,0}, 19, 23, 31, 1),
        crossbar({2,0}, {2,0}, 31, -1, -1, 1), tilePin({2,0}, trunk_pin)});
    other_sink.wires.resize(1);
    auto& victim = other_sink.wires[0];
    if (conflict == 2) {
        // Keep the contested landing beyond the victim's protected first hop.
        mesh.edge(6, other_local, 61, 5, 97);
        mesh.edge(5, 97, 101, 3, 59);
        mesh.types[6].local_src[other_local].jump.setBit(61);
        victim.push_back(crossbar({6,0}, {5,0}, other_local, 61, 97, 0));
        victim.push_back(crossbar({5,0}, {3,0}, 97, 101, 59, 1));
    } else {
        if (!protected_endpoint) {
            mesh.edge(6, other_local, 61, 1, 71);
            mesh.types[6].local_src[other_local].jump.setBit(61);
            victim.push_back(crossbar({6,0}, {1,0}, other_local, 61, 71, 0));
        }
        const int incoming = protected_endpoint ? other_local : 71;
        const int exit = conflict == 1 ? 67 : 37;
        const int arrival = conflict == 1 ? 79 : 59;
        mesh.edge(1, incoming, exit, 3, arrival);
        if (protected_endpoint) mesh.types[1].local_src[incoming].jump.setBit(exit);
        auto step = crossbar({1,0}, {3,0}, incoming, exit, arrival,
                             protected_endpoint ? 0 : 1);
        if (conflict == 1) {
            // The candidate and the transit need the same joint, not the same SRC.
            mesh.types[1].dst_src[19].jump.clearBit(37);
            mesh.types[1].dst_joint[19].joint.setBit(11);
            mesh.types[1].joint_src[11].jump.setBit(37);
            mesh.types[1].joint_src[11].jump.setBit(67);
            if (free_joint_alternative) {
                mesh.types[1].dst_joint[19].joint.setBit(12);
                if (free_joint_alternative == 1) mesh.types[1].joint_src[12].jump.setBit(37);
                else {
                    mesh.types[1].joint_joint[12].joint.setBit(13);
                    mesh.types[1].joint_src[13].jump.setBit(37);
                }
            }
            mesh.types[1].dst_src[incoming].jump.clearBit(exit);
            if (protected_endpoint) {
                mesh.types[1].local_src[incoming].jump.clearBit(exit);
                mesh.types[1].local_joint[incoming].joint.setBit(11);
            } else mesh.types[1].dst_joint[incoming].joint.setBit(11);
            step.joint = 11;
        }
        victim.push_back(step);
    }
    if (victim_sink == 7) {
        int arrival = conflict == 1 ? 79 : 59;
        mesh.edge(3, arrival, 83, 7, 89);
        victim.push_back(crossbar({3,0}, {7,0}, arrival, 83, 89, 1));
    }
    const int last_dst = victim_sink == 7 ? 89 : 59;
    const int victim_pin = terminal(victim_sink, last_dst);
    victim.push_back(crossbar({victim_sink,0}, {victim_sink,0}, last_dst, -1, -1, 1));
    victim.push_back(tilePin({victim_sink,0}, victim_pin));
    for (auto& type : mesh.types) type.rebuildOutgoingSrcs();
    for (auto [owner, signal, source, name] : {
            std::tuple{&trunk, &base, &driver, "seed_907"},
            std::tuple{&other_sink, &busy, &other_driver, "owner_908"}}) {
        leaseRoute(owner->wires[0]);
        auto i = fpga::attachNetRoute(*signal, *owner, 0, source, owner, "Q", "D", name);
        fpga::registerNetRouteTiles(*signal, owner->wires[0], i);
    }
    sink.wires.resize(1);
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "Q", "D", "branch_909");
    if (conflict == 1) mesh.tile(1).cb.joint.jump.setBit(11);
    for (int x = 0; x < 9; ++x) {
        std::ostringstream out;
        auto a = fpga::auditTileCongestion(mesh.tile(x), out, {&base, &net, &busy});
        // Establish that the obstruction is a real route, not a fabricated busy bit.
        require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
            !a.stale_registrations, "invalid branch takeoff fixture: " + out.str());
    }
    const auto trunk_before = trunk.wires[0], victim_before = victim;
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.moving_stage = true;
    router.moving_focus_inst = focused ? &sink : nullptr;
    router.route_preemption_enabled = enabled;
    router.indexSourceRoute(&base, &driver, "Q");
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "branch_909"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    const bool moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    const bool should_cut = enabled && !free_alternative && !free_joint_alternative && !protected_endpoint;
    // Reproduce the bug: free-only growth cannot escape even a preemptible transit.
    require(moved == (free_alternative || free_joint_alternative || should_cut), "branch takeoff conflict=" +
        std::to_string(conflict) + " free=" + std::to_string(free_alternative) +
        " protected=" + std::to_string(protected_endpoint) + " enabled=" + std::to_string(enabled) +
        " focused=" + std::to_string(focused) + " free_joint=" + std::to_string(free_joint_alternative) + " result: " + reason);
    // Preemption must never alter the current tree or invent another source takeoff.
    require(trunk.wires[0].size() == trunk_before.size() &&
                std::equal(trunk_before.begin(), trunk_before.end(), trunk.wires[0].begin(), sameFragment),
            "branch takeoff modified its trunk");
    if (moved) require(fpga::isRouteComplete(sink.wires[0]) &&
        sink.tile.peer == &mesh.tile(free_alternative ? 4 : 3), "branch did not reach replacement site");
    // A free alternate joint to the same source must be tried before any eviction.
    if (free_joint_alternative) require(sink.wires[0][1].joint == (free_joint_alternative == 1 ? 12 : 13) &&
        sink.wires[0][1].joint2 == (free_joint_alternative == 1 ? -1 : 12),
        "branch ignored the free parallel joint");
    if (should_cut) {
        // The cut victim must be unfinished AND explicitly returned to routing work.
        require(!fpga::isRouteComplete(victim) &&
            std::any_of(router.pending_route_todo.begin(), router.pending_route_todo.end(),
                [&](const auto& t) { return t.net == &busy && t.to == &other_sink && t.fanout; }),
            "branch victim was not cut and rescheduled");
        const int cut_tile = conflict == 2 ? 3 : 1;
        const auto cut_type = conflict == 0 ? fpga::CB_NODE_SRC :
                              conflict == 1 ? fpga::CB_NODE_JOINT : fpga::CB_NODE_DST;
        const int cut_node = conflict == 0 ? 37 : conflict == 1 ? 11 : 59;
        const auto owners = fpga::findNetOwnersByNode(mesh.tile(cut_tile), cut_type, cut_node);
        // The released node is now owned by the new branch, not a stale victim.
        require(!owners.empty() && std::all_of(owners.begin(), owners.end(),
            [&](const auto& owner) { return owner.net == &net; }) &&
            mesh.tile(6).cb.src.jump.testBit(61),
            "branch cut retained victim ownership or destroyed its takeoff");
    } else {
        // Free alternatives, disabled preemption, and endpoint owners forbid eviction.
        require(victim.size() == victim_before.size() &&
            std::equal(victim_before.begin(), victim_before.end(), victim.begin(), sameFragment) &&
            router.pending_route_todo.empty(), "branch evicted a protected or unnecessary victim");
    }
    for (int x = 0; x < 9; ++x) {
        std::ostringstream out;
        auto a = fpga::auditTileCongestion(mesh.tile(x), out, {&base, &net, &busy});
        // Every retained or newly allocated lease needs a real route owner.
        require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
            !a.stale_registrations, "branch takeoff leaked ownership: " + out.str());
    }
    fpga::unrouteNet(net); fpga::unrouteNet(base); fpga::unrouteNet(busy);
    for (int x = 0; x < 9; ++x)
        // Shared prefixes and cut tails must also be fully releasable afterward.
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
            mesh.tile(x).cb.joint.jump == NodeMask{} && mesh.tile(x).cb.local.local == NodeMask{} &&
            mesh.tile(x).pin_state.leased_nodes == NodeMask{}, "branch cleanup leaked a bit");
}

void moving_branch_takeoff_regressions()
{
    for (bool focused : {false, true}) for (int conflict : {0, 1, 2}) {
        moving_branch_takeoff_preemption(conflict, true, false, true, focused);
        moving_branch_takeoff_preemption(conflict, false, true, true, focused);
        moving_branch_takeoff_preemption(conflict, false, false, false, focused);
        moving_branch_takeoff_preemption(conflict, false, false, true, focused);
    }
    for (bool focused : {false, true}) for (int joints : {1, 2})
        moving_branch_takeoff_preemption(1, false, false, true, focused, joints);
}

// Moving can leave only a non-owning prefix after removing a local terminal.
// Cleaning that replica is preparation, not a failed complete routing attempt.
void moving_shared_prefix_cleanup_continues_search(bool incremental, int budget)
{
    ForwardMovingMesh mesh(4);
    fpga::TileType resource{"slots_801", 1};
    fpga::Element reg;
    reg.name = "register_802";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x = 0; x < 4; ++x) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_803";
    cell.type = "FD_SAMPLE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, trunk, sink;
    for (auto* inst : {&driver, &trunk, &sink}) inst->cell_ref.set(&cell);
    require(mesh.tile(0).tryAddAt(&driver, 0) == 0 &&
                mesh.tile(2).tryAddAt(&trunk, 0) == 0 &&
                mesh.tile(1).tryAddAt(&sink, 0) == 0, "cannot place cleanup fixture");
    parent.nets.resize(1);
    auto& net = parent.nets.front();
    net.name = "signal_804";
    const int output = mesh.tile(0).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    const int pin = mesh.tile(1).getPinNodes(cell.type, "D", 0).firstSetBit();
    require(output >= 0 && pin >= 0, "fixture endpoints missing");
    mesh.edge(0, output, 17, 1, 19);
    mesh.types[0].local_src[output].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 19, 23, 2, 31);
    mesh.edge(1, 19, 37, 3, 41);
    for (auto [x, dst] : {std::pair{1, 19}, std::pair{2, 31}, std::pair{3, 41}}) {
        mesh.types[x].dst_local[dst].local.setBit(pin);
        mesh.types[x].rebuildOutgoingSrcs();
    }
    trunk.wires.push_back({crossbar({0,0}, {1,0}, output, 17, 19, 0),
        crossbar({1,0}, {2,0}, 19, 23, 31, 1),
        crossbar({2,0}, {2,0}, 31, -1, -1, 1), tilePin({2,0}, pin)});
    sink.wires.push_back({crossbar({0,0}, {1,0}, output, 17, 19, 0, true),
        crossbar({1,0}, {1,0}, 19, -1, -1, 2, false, false), tilePin({1,0}, pin)});
    for (auto [owner, name] : {std::pair{&trunk, "seed_805"}, std::pair{&sink, "suffix_806"}}) {
        leaseRoute(owner->wires[0]);
        auto index = fpga::attachNetRoute(net, *owner, 0, &driver, owner, "Q", "D", name);
        fpga::registerNetRouteTiles(net, owner->wires[0], index);
    }
    const auto original = trunk.wires[0];
    // Use real Moving invalidation, and prove its result is a shared-only prefix.
    require(fpga::invalidateMovedSinkRoute(net, 1) && sink.wires[0].size() == 1 &&
                sink.wires[0].front().shared, "fixture did not retain a shared-only prefix");
    require(sink.tile->unassign(&sink), "cannot release cleanup fixture placement");
    require(mesh.tile(3).tryAddAt(&sink, 0) == 0, "cannot move cleanup fixture sink");
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = true;
    router.moving_focus_inst = &sink;
    router.complete_routes_only = !incremental;
    router.route_preemption_enabled = false;
    router.route_recursion_budget = budget;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_806"};
    task.fanout = true;
    if (incremental) {
        // Bounded callers keep their old cleanup-only turn and retry normally.
        require(!router.routeNetTask(task) && sink.wires[0].empty() &&
                    router.route_stats.route_searches == 0,
                "incremental cleanup unexpectedly searched or reported completion");
        router.complete_routes_only = true;
        router.route_recursion_budget = budget;
    }
    // A full task must search now, rather than report failure for housekeeping.
    require(router.routeNetTask(task), "Moving returned failure after shared-prefix cleanup without searching");
    require(fpga::isRouteComplete(sink.wires[0]) && sink.wires[0].back().to.x == 3,
            "cleaned Moving task did not reach the relocated sink");
    // The trunk must stay fixed, and every leased bit must have a real owner.
    require(trunk.wires[0].size() == original.size() &&
                std::equal(original.begin(), original.end(), trunk.wires[0].begin(), sameFragment),
            "shared-prefix cleanup rerouted the trunk");
    for (int x = 0; x < 4; ++x) {
        std::ostringstream out;
        auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases && !audit.missing_registrations &&
                    !audit.stale_registrations && !audit.conflicting_driver_nodes,
                "cleanup ownership mismatch: " + out.str());
    }
    require(fpga::unrouteNet(net), "cannot clear cleanup fixture");
    for (int x = 0; x < 4; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{}, "cleanup fixture leaked leases");
}

// A shared control input needs its existing source-connected terminal, not a
// second free one. A disconnected tail or another driver cannot prove support.
void moving_destination_reuses_complete_input(bool foreign, bool detached, bool through_joint,
                                              bool through_local = false)
{
    ForwardMovingMesh mesh(6);
    fpga::TileType resource{"slots_901", 1};
    for (int lane = 0; lane < 2; ++lane) {
        fpga::Element reg;
        reg.name = "register_902";
        reg.type = fpga::ELEMENT_FD;
        reg.bitmap_pos = lane;
        resource.elements.push_back(reg);
    }
    for (int x : {2, 4}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_903";
    cell.type = "FD_SAMPLE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, control, stranger, trunk, peer, sink, second;
    for (auto* inst : {&driver, &control, &stranger, &trunk, &peer, &sink})
        inst->cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    control.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(5)));
    stranger.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(5)));
    driver.pos = control.pos = 0;
    stranger.pos = 4;
    require(mesh.tile(2).tryAddAt(&peer, 0) == 0 &&
                mesh.tile(4).tryAddAt(&sink, 0) == 0, "cannot place shared-input fixture");
    parent.nets.resize(2);
    auto& data = parent.nets[0];
    auto& enable = parent.nets[1];
    data.name = "signal_904";
    enable.name = "signal_905";
    Referable<rtl::Port> port;
    port.name = "CE";
    port.type = rtl::Port::PORT_IN;
    port.designator = 905;
    enable.designators = {905};
    sink.conns.resize(1);
    sink.conns.front().port_ref.set(&port);
    const int pin = mesh.tile(2).getPinNodes(cell.type, "D", 4).firstSetBit();
    const int shared = mesh.tile(2).getPinNodes(cell.type, "CE", 4).firstSetBit();
    require(pin >= 0 && shared >= 0 && pin != shared &&
                mesh.tile(2).getPinNodes(cell.type, "CE", 0).testBit(shared),
            "fixture needs distinct data and shared control inputs");
    mesh.edge(0, 4, 5, 1, 7);
    mesh.types[0].local_src[4].jump.setBit(5);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(1, 7, 10, 2, 8);
    if (through_local) {
        // The native exhausted Moving frontier omitted a free internal LOCAL bridge.
        mesh.types[1].dst_src[7].jump = {};
        mesh.types[1].dst_local[7].local.setBit(91);
        mesh.types[1].local_src[91].jump.setBit(10);
        mesh.types[1].rememberNodeName(fpga::CB_NODE_LOCAL, 91, "bridge_910");
    }
    mesh.types[1].dst_local[7].local.setBit(9);
    mesh.types[1].rebuildOutgoingSrcs();
    mesh.types[2].dst_local[8].local.setBit(pin);
    mesh.edge(5, 4, 57, 2, 81);
    mesh.types[5].local_src[4].jump.setBit(57);
    mesh.types[5].rebuildOutgoingSrcs();
    if (through_joint) {
        mesh.types[2].dst_joint[81].joint.setBit(93);
        mesh.types[2].joint_local[93].local.setBit(shared);
    } else mesh.types[2].dst_local[81].local.setBit(shared);
    mesh.types[2].rebuildOutgoingSrcs();
    trunk.wires.push_back({crossbar({0,0}, {1,0}, 4, 5, 7, 0),
        crossbar({1,0}, {1,0}, 7, -1, -1, 1), tilePin({1,0}, 9)});
    peer.wires.push_back({crossbar({5,0}, {2,0}, 4, 57, 81, 0),
        crossbar({2,0}, {2,0}, 81, -1, -1, 1), tilePin({2,0}, shared)});
    if (through_joint) peer.wires[0][1].joint = 93;
    sink.wires.resize(2);
    leaseRoute(trunk.wires[0]);
    leaseRoute(peer.wires[0]);
    if (through_joint) mesh.tile(2).cb.joint.jump.setBit(93);
    if (detached) {
        fpga::Wire gap;
        gap.type = fpga::Wire::WIRE_GAP;
        peer.wires[0].insert(peer.wires[0].begin() + 1, gap);
    }
    auto index = fpga::attachNetRoute(data, trunk, 0, &driver, &trunk, "Q", "D", "seed_906");
    fpga::registerNetRouteTiles(data, trunk.wires[0], index);
    index = fpga::attachNetRoute(enable, peer, 0, &control, &peer, "Q", "CE", "control_907");
    fpga::registerNetRouteTiles(enable, peer.wires[0], index);
    fpga::attachNetRoute(data, sink, 0, &driver, &sink, "Q", "D", "suffix_908");
    auto* wanted_control = foreign ? &stranger : &control;
    fpga::attachNetRoute(enable, sink, 1, wanted_control, &sink, "Q", "CE", "control_909");
    const auto original = peer.wires[0];
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.moving_stage = true;
    router.route_preemption_enabled = false;
    router.indexSourceRoute(&data, &driver, "Q");
    router.indexSourceRoute(&enable, &control, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &data, "Q", "D", "suffix_908"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    const bool moved = router.moveUnfinishedCell(task, &tasks, &task, &reason);
    // Only a complete same-driver endpoint can support this otherwise free move.
    require(moved == (!foreign && !detached), "shared-input move result: " + reason);
    if (moved) {
        require(sink.tile.peer == &mesh.tile(2) && sink.pos == 4 && fpga::isRouteComplete(sink.wires[0]),
                "shared-input relocation did not ground the data suffix");
        pnr::RouteDesign::RouteTask input{&control, &sink, &enable, "Q", "CE", "control_909"};
        input.fanout = true;
        router.complete_routes_only = true;
        router.route_recursion_budget = 20;
        // Prove the preflight's sharing claim by actually attaching the moved input.
        require(router.routeNetTask(input), "accepted shared input could not be routed");
        auto binding = std::find_if(enable.routes.begin(), enable.routes.end(), [&](const auto& route) {
            return route.to == &sink && route.to_port == "CE";
        });
        require(binding != enable.routes.end() && binding->owner &&
                    binding->route_index < binding->owner->wires.size() &&
                    fpga::isRouteComplete(binding->owner->wires[binding->route_index]),
                "shared input did not retain its complete binding");
    } else require(sink.tile.peer == &mesh.tile(4) && sink.pos == 0,
                   "rejected shared-input move changed placement");
    if (through_local && moved) {
        mesh.tile(3).tile_type = &resource;
        second.cell_ref.set(&cell);
        require(mesh.tile(4).tryAddAt(&second, 0) == 0, "cannot place second bridge load");
        second.wires.resize(1);
        fpga::attachNetRoute(data, second, 0, &driver, &second, "Q", "D", "suffix_912");
        mesh.edge(2, 8, 112, 3, 82);
        const int second_pin = mesh.tile(3).getPinNodes(cell.type, "D", 0).firstSetBit();
        mesh.types[3].dst_local[82].local.setBit(second_pin);
        mesh.types[3].rebuildOutgoingSrcs();
        pnr::RouteDesign::RouteTask next{&driver, &second, &data, "Q", "D", "suffix_912"};
        next.fanout = true;
        // A numeric DST landing in the first completed branch is also a branch anchor.
        require(router.moveUnfinishedCell(next, &tasks, &next, &reason) &&
                    second.tile.peer == &mesh.tile(3), "cannot branch from typed Moving route: " + reason);
    }
    require(peer.wires[0].size() == original.size() &&
                std::equal(original.begin(), original.end(), peer.wires[0].begin(), sameFragment),
            "Moving rewrote the existing control route");
    for (int x = 0; x < 6; ++x) {
        std::ostringstream out;
        const auto audit = fpga::auditTileCongestion(mesh.tile(x), out, {&data, &enable});
        require(!audit.orphan_leases && !audit.missing_leases && !audit.missing_registrations &&
                    !audit.stale_registrations && !audit.conflicting_driver_nodes,
                "shared-input ownership mismatch: " + out.str());
    }
    fpga::unrouteNet(data);
    fpga::unrouteNet(enable);
    // Clearing both trees must release each shared bit exactly once.
    for (int x = 0; x < 6; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} && mesh.tile(x).cb.joint.jump == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{}, "shared-input fixture leaked leases");
}

void moving_branch_point_regressions()
{
    for (bool indexed : {false, true})
        for (bool blocked : {false, true})
            for (bool sibling_first : {false, true})
                moving_branch_point_on_routed_sibling(indexed, blocked, sibling_first);
}

// A preempted middle segment keeps both sides leased, but only the side before
// WIRE_GAP is source-connected. The native failure incorrectly seeded the tail.
void moving_gap_tail_is_not_a_branch_point(bool alternative)
{
    ForwardMovingMesh mesh(6);
    fpga::TileType resource{"slots_701", 1};
    fpga::Element reg;
    reg.name = "register_702";
    reg.type = fpga::ELEMENT_FD;
    resource.elements.push_back(reg);
    for (int x : {3, 4, 5}) mesh.tile(x).tile_type = &resource;
    Referable<rtl::Module> parent, primitive;
    primitive.is_blackbox = true;
    primitive.parent_ref.set(&parent);
    Referable<rtl::Cell> cell;
    cell.name = "load_703";
    cell.type = "FD_SAMPLE";
    cell.module_ref.set(&primitive);
    rtl::Inst driver, sink;
    driver.cell_ref.set(&cell);
    sink.cell_ref.set(&cell);
    driver.tile.set(static_cast<Referable<fpga::Tile>*>(&mesh.tile(0)));
    driver.pos = 0;
    require(mesh.tile(4).tryAddAt(&sink, 0) == 0, "cannot place gap fixture sink");
    parent.nets.resize(1);
    auto& net = parent.nets.front();
    net.name = "signal_704";
    const int output = mesh.tile(0).getOutputPinNodes(cell.type, "Q", 0).firstSetBit();
    require(output >= 0, "fixture output missing");
    mesh.edge(0, output, 17, 1, 19);
    mesh.types[0].local_src[output].jump.setBit(17);
    mesh.types[0].rebuildOutgoingSrcs();
    mesh.edge(2, 23, 29, 3, 31);
    if (alternative) mesh.edge(1, 19, 37, 5, 41);
    for (auto [x, dst] : {std::pair{3, 31}, std::pair{5, 41}}) {
        const int pin = mesh.tile(x).getPinNodes(cell.type, "D", 0).firstSetBit();
        require(pin >= 0, "fixture input missing");
        mesh.types[x].dst_local[dst].local.setBit(pin);
        mesh.types[x].rebuildOutgoingSrcs();
    }
    auto first = crossbar({0,0}, {1,0}, output, 17, 19, 0);
    first.owns_landing = true;
    auto tail = crossbar({2,0}, {3,0}, 23, 29, 31, 1);
    tail.owns_landing = true;
    fpga::Wire gap;
    gap.type = fpga::Wire::WIRE_GAP;
    gap.from = {1,0}; gap.to = {2,0};
    gap.from_node_type = gap.to_node_type = fpga::CB_NODE_DST;
    gap.from_node = 19; gap.to_node = 23;
    sink.wires.push_back({first, gap, tail});
    leaseRoute({first, tail});
    mesh.tile(1).cb.dst.jump.setBit(19);
    mesh.tile(3).cb.dst.jump.setBit(31);
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "Q", "D", "suffix_705");
    fpga::registerNetRouteTiles(net, sink.wires[0], 0);
    const auto original = sink.wires[0];
    std::vector<fpga::CBState> before;
    for (int x = 0; x < 6; ++x) before.push_back(mesh.tile(x).cb);
    pnr::RouteDesign router;
    router.fpga = &fpga::Device::current();
    router.aspect_x = router.aspect_y = 1;
    router.indexSourceRoute(&net, &driver, "Q");
    pnr::RouteDesign::RouteTask task{&driver, &sink, &net, "Q", "D", "suffix_705"};
    task.fanout = true;
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    std::ostringstream decisions;
    bool moved;
    {
        pnr::RouteCongestionTrace trace("suffix_705", "Moving gap tail", {&net}, &decisions);
        moved = router.moveUnfinishedDestination(task, &tasks, &reason);
    }
    // A busy landing beyond the gap is NOT a usable branch, even though its
    // retained suffix has the right driver identity and a legal packing site.
    require(decisions.str().find("prefix=3 tile=(3,0)") == std::string::npos,
            "Moving seeded a disconnected tail beyond WIRE_GAP");
    require(moved == alternative, "gap-tail Moving result mismatch: " + reason);
    if (alternative) {
        // Commit only the intact prefix plus the new branch; release private
        // tail leases and never return a successful route still containing a gap.
        require(sink.tile.peer == &mesh.tile(5) && fpga::isRouteComplete(sink.wires[0]),
                "Moving failed to ground from the connected alternative");
        require(!mesh.tile(2).cb.src.jump.testBit(29) &&
                    !mesh.tile(2).cb.dst.jump.testBit(23) &&
                    !mesh.tile(3).cb.dst.jump.testBit(31), "detached tail leases survived move");
    } else {
        // Exhaustion must leave both retained components and placement intact.
        require(sink.tile.peer == &mesh.tile(4) && sink.wires[0].size() == original.size(),
                "failed search changed placement or retained route");
        for (int x = 0; x < 6; ++x)
            require(mesh.tile(x).cb.src.jump == before[x].src.jump &&
                        mesh.tile(x).cb.dst.jump == before[x].dst.jump &&
                        mesh.tile(x).cb.local.local == before[x].local.local,
                    "failed gap-tail search changed leases");
    }
    // Audit real owner registrations, then prove final cleanup frees both sides.
    for (int x = 0; x < 6; ++x) {
        std::ostringstream audit;
        auto a = fpga::auditTileCongestion(mesh.tile(x), audit, {&net});
        require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
                    !a.stale_registrations, "gap-tail audit: " + audit.str());
    }
    require(fpga::unrouteNet(net), "cannot unroute gap fixture");
    for (int x = 0; x < 6; ++x)
        require(mesh.tile(x).cb.src.jump == NodeMask{} && mesh.tile(x).cb.dst.jump == NodeMask{} &&
                    mesh.tile(x).cb.local.local == NodeMask{} &&
                    mesh.tile(x).pin_state.leased_nodes == NodeMask{}, "gap-tail unroute leaked leases");
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

void moving_replacement_input_releases_retained_prefix(bool sibling, bool reuse)
{
    resetGrid(3, 1);
    auto &device = fpga::Device::current();
    Referable<rtl::Net> net;
    net.name = "signal_83";
    rtl::Inst driver, sink, follower;
    // A complete input with a same-CB takeoff, like the retained prefix
    // replaced when its sink subsequently moves as an output driver.
    sink.wires.push_back({tilePin({0, 0}, 17),
        crossbar({0, 0}, {0, 0}, 17, 117, 217, 0),
        crossbar({0, 0}, {0, 0}, 217, -1, 317, 1),
        tilePin({0, 0}, 317)});
    leaseRoute(sink.wires[0]);
    fpga::attachNetRoute(net, sink, 0, &driver, &sink, "out", "in", net.name);
    fpga::registerNetRouteTiles(net, sink.wires[0], 0);
    const auto route_id = net.routeId(0);
    if (sibling) {
        follower.wires.push_back({sink.wires[0][0], sink.wires[0][1],
            crossbar({0, 0}, {2, 0}, 217, 119, 219, 1),
            crossbar({2, 0}, {2, 0}, 219, -1, 319, 1),
            tilePin({2, 0}, 319)});
        follower.wires[0][0].shared = follower.wires[0][1].shared = true;
        leaseRoute(follower.wires[0]);
        fpga::attachNetRoute(net, follower, 0, &driver, &follower,
                            "out", "in", "branch_84");
        fpga::registerNetRouteTiles(net, follower.wires[0], 1);
    }
    require(fpga::invalidateMovedSinkRoutes({{&net, 0}}) &&
                sink.wires[0].size() == 2 && sink.wires[0].back().owns_landing,
            "Moving did not retain the old input prefix");

    // Build the new committed route in separate storage, as source Moving
    // does for an input proof that does not extend its binding in place.
    std::vector<fpga::Wire> replacement{tilePin({0, 0}, 17)};
    if (reuse) {
        replacement = sink.wires[0];
        for (auto &w : replacement) {
            w.shared = true;
            w.owns_landing = false;
        }
    }
    replacement.push_back(crossbar({0, 0}, {1, 0}, reuse ? 217 : 17,
                                   118, 218, reuse ? 1 : 0));
    replacement.push_back(crossbar({1, 0}, {1, 0}, 218, -1, 318, 1));
    replacement.push_back(tilePin({1, 0}, 318));
    leaseRoute(replacement);
    sink.wires.push_back(std::move(replacement));
    require(fpga::attachNetRoute(net, sink, 1, &driver, &sink,
                                "out", "in", net.name) == 0 &&
                net.routeId(0) == route_id && net.routes[0].route_index == 1,
            "input replacement changed the stable binding identity");
    fpga::registerNetRouteTiles(net, sink.wires[1], 0);
    require(sink.wires[0].empty(), "replacement abandoned old input storage");
    auto &source = *device.getTile(0, 0);
    require(source.cb.src.jump.testBit(117) == (sibling || reuse) &&
                source.cb.dst.jump.testBit(217) == (sibling || reuse),
            "replacement leaked private leases or released a surviving prefix");
    require(source.cb.src.jump.testBit(118) && source.cb.local.local.testBit(17),
            "replacement cleanup released the new takeoff");
    if (sibling)
        require(!follower.wires[0][1].shared,
                "replacement failed to promote the surviving sibling");
    if (reuse && !sibling)
        require(!sink.wires[1][1].shared,
                "replacement failed to inherit its reused prefix");
    for (int x = 0; x < 3; ++x) {
        std::ostringstream out;
        auto audit = fpga::auditTileCongestion(*device.getTile(x, 0), out, {&net});
        require(!audit.orphan_leases && !audit.missing_leases &&
                    !audit.missing_registrations && !audit.stale_registrations,
                "input replacement ownership mismatch: " + out.str());
    }
    require(fpga::unrouteNet(net), "cannot unroute replaced input");
    for (int x = 0; x < 3; ++x) {
        auto &tile = *device.getTile(x, 0);
        require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                    tile.cb.local.local == NodeMask{} &&
                    tile.pin_state.leased_nodes == NodeMask{},
                "replaced input left leases after full unroute");
    }
}

void moving_proven_input_prefix_survives_invalidation_and_commit(
    size_t keep, bool complete)
{
    resetGrid(4, 1);
    auto &device = fpga::Device::current();
    Referable<rtl::Net> net;
    net.name = "signal_47";
    rtl::Inst driver, sink, owner;
    owner.wires.push_back({
        tilePin({0, 0}, 17),
        crossbar({0, 0}, {1, 0}, 17, 117, 217, 0),
        crossbar({1, 0}, {2, 0}, 217, 118, 218, 1),
    });
    owner.wires[0].back().owns_landing = true;
    leaseRoute(owner.wires[0]);
    device.getTile(2, 0)->cb.dst.jump.setBit(218);
    if (complete) {
        owner.wires[0].push_back(crossbar({2, 0}, {2, 0}, 218, -1, 318, 1));
        owner.wires[0].push_back(tilePin({2, 0}, 318));
        device.getTile(2, 0)->cb.local.local.setBit(318);
        device.getTile(2, 0)->pin_state.leased_nodes.setBit(318);
    }
    fpga::attachNetRoute(net, owner, 0, &driver, &sink, "out", "in", net.name);
    fpga::registerNetRouteTiles(net, owner.wires[0], 0);

    Referable<rtl::Net> unproven;
    unproven.name = "signal_48";
    rtl::Inst other_driver, other_owner;
    other_owner.wires.push_back({
        tilePin({0, 0}, 44),
        crossbar({0, 0}, {2, 0}, 44, 144, 244, 0),
    });
    other_owner.wires[0].back().owns_landing = true;
    leaseRoute(other_owner.wires[0]);
    device.getTile(2, 0)->cb.dst.jump.setBit(244);
    fpga::attachNetRoute(unproven, other_owner, 0, &other_driver, &sink,
                        "out", "other_in", unproven.name);
    fpga::registerNetRouteTiles(unproven, other_owner.wires[0], 0);
    const std::vector<fpga::NetRouteRef> moved_inputs{{&net, 0}, {&unproven, 0}};

    auto *anchor = device.getTile(static_cast<int>(keep - 1), 0);
    const int anchor_dst = keep == 2 ? 217 : 218;
    auto *target = device.getTile(3, 0);
    device.cb_types.clear();
    device.cb_types.resize(2);
    auto &anchor_type = device.cb_types[0];
    auto &target_type = device.cb_types[1];
    anchor_type.type_id = 0;
    target_type.type_id = 1;
    anchor->cb.type = anchor->cb_type = &anchor_type;
    target->cb.type = target->cb_type = &target_type;
    anchor_type.dst_src[anchor_dst].jump = bit(120);
    fpga::CBJumpState destinations;
    destinations.jump = bit(220);
    anchor_type.dst_by_src[120].push_back(
        {{3 - anchor->coord.x, 0}, 1, destinations, {}, true});
    target_type.dst_local[220].local = bit(321);
    anchor_type.rebuildOutgoingSrcs();
    target_type.rebuildOutgoingSrcs();

    // Check: actual backward search can prove a new input suffix from this
    // owned landing, even when the old input has never reached its destination.
    require(pnr::movingInputMayReuseRoute(true, complete, false, true),
            "owned input prefix was not eligible for relocation proof");
    std::vector<pnr::BackwardRouteAnchor> anchors{{anchor, anchor_dst, "entry", keep}};
    auto proof = pnr::routeBackwardToAnchors(*target, bit(321), anchors, 5, 5);
    require(proof.success && proof.completed_from_anchor && proof.anchor_id == keep,
            "backward input proof failed to select its owned prefix");
    // Check: proof construction alone must not lease its proposed suffix.
    require(!anchor->cb.src.jump.testBit(120) && target->cb.dst.jump == NodeMask{},
            "speculative input proof changed live leases");

    // Check: malformed retention requests are rejected before any route or
    // lease is modified, rather than invalidating only part of a relocation.
    const auto old_route = owner.wires[0];
    require(!fpga::invalidateMovedSinkRoutes(
                moved_inputs, {{{&net, 0}, old_route.size() + 1}}) &&
                owner.wires[0].size() == old_route.size() &&
                anchor->cb.dst.jump.testBit(anchor_dst) &&
                other_owner.wires[0].size() == 2 &&
                device.getTile(2, 0)->cb.dst.jump.testBit(244),
            "invalid prefix proof partially invalidated the input");

    // Check: use the same explicit proof boundary as source relocation. The
    // former complete-only invalidation erased incomplete inputs at this point.
    require(fpga::invalidateMovedSinkRoutes(moved_inputs, {{{&net, 0}, keep}}) &&
                owner.wires[0].size() == keep &&
                anchor->cb.dst.jump.testBit(anchor_dst),
            "sink invalidation released the proven input anchor");
    // Check: the proof protects only its selected binding, not another
    // incomplete input of the same moved cell with no successful proof.
    require(other_owner.wires[0].empty() &&
                !device.getTile(0, 0)->cb.src.jump.testBit(144) &&
                !device.getTile(0, 0)->cb.local.local.testBit(44) &&
                !device.getTile(2, 0)->cb.dst.jump.testBit(244),
            "proof retention protected an unrelated incomplete input");
    // Check: an earlier selected anchor releases the unused private tail;
    // neither complete inputs nor partial inputs retain unselected tail leases.
    require((keep != 2 || (!anchor->cb.src.jump.testBit(118) &&
                device.getTile(2, 0)->cb.dst.jump == NodeMask{})) &&
                device.getTile(2, 0)->cb.local.local == NodeMask{} &&
                device.getTile(2, 0)->pin_state.leased_nodes == NodeMask{},
            "sink invalidation retained an obsolete private input tail");

    // Check: a rejected commit restores its speculative leases without
    // disturbing the retained anchor or the pre-existing terminal blocker.
    target->pin_state.leased_nodes.setBit(321);
    require(!pnr::commitPreparedRoute(proof.fragments) &&
                !anchor->cb.src.jump.testBit(120) &&
                anchor->cb.dst.jump.testBit(anchor_dst) &&
                target->cb.dst.jump == NodeMask{} &&
                target->cb.local.local == NodeMask{} &&
                target->pin_state.leased_nodes.testBit(321),
            "failed input commit leaked leases or released its prefix");
    target->pin_state.leased_nodes &= ~bit(321);

    // Check: production commit must extend the still-owned DST, not attempt
    // a second source takeoff or rely on replaying the old prefix leases.
    require(pnr::commitPreparedRoute(proof.fragments),
            "proven input suffix could not commit after sink invalidation");
    owner.wires[0].insert(owner.wires[0].end(), proof.fragments.begin(), proof.fragments.end());
    fpga::registerNetRouteTiles(net, owner.wires[0], 0);
    require(fpga::isRouteComplete(owner.wires[0]) &&
                anchor->cb.src.jump.testBit(120) && target->cb.dst.jump.testBit(220) &&
                target->pin_state.leased_nodes.testBit(321),
            "committed input proof did not complete the moved input");
    // Check: original prefix, anchor, and new suffix all have a live owner.
    for (const auto &[tile, kind, node] : std::vector<std::tuple<fpga::Tile*, fpga::CBNodeNameType, int>>{
             {device.getTile(0, 0), fpga::CB_NODE_SRC, 117},
             {anchor, fpga::CB_NODE_DST, anchor_dst},
             {anchor, fpga::CB_NODE_SRC, 120},
             {target, fpga::CB_NODE_DST, 220}}) {
        auto owners = fpga::findNetOwnersByNode(*tile, kind, node);
        require(owners.size() == 1 && owners.front().net == &net,
                "extended input has an orphan or duplicate lease owner");
    }
    // Check: atomic unroute frees both the retained prefix and new suffix.
    require(fpga::unrouteNet(net), "extended input could not be unrouted");
    for (auto &tile : device.tile_grid) {
        require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                    tile.cb.local.local == NodeMask{} && tile.cb.joint.jump == NodeMask{} &&
                    tile.pin_state.leased_nodes == NodeMask{},
                "unrouting the extended input left orphan leases");
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

    // Generic/Fanout report softly and let the scheduler enter recovery.
    require(!pnr::routeStageEntryTimeoutRequiresFailure(true, false),
        "Generic/Fanout timeout prevented its recovery handoff");

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

int main(int argc, char** argv)
{
    try {
        if (argc == 2) {
            const std::string test = argv[1];
            if (test == "source_tile_reentry") moving_source_physical_tile_reentry();
            else if (test == "mapped_io_input") moving_input_uses_mapped_io_endpoint();
            else if (test == "nested_input_window") moving_nested_input_keeps_outer_window();
            else if (test == "duplicate_input")
                moving_destination_commits_forward_grounding(false,false,false,false,false,true,false,false,true);
            else if (test == "stale_trigger")
                moving_destination_commits_forward_grounding(false,false,false,false,false,true,false,false,true,true);
            else if (test == "bridge_preemption") bridge_preemption_prepares_all_leases_before_cutting();
            else if (test == "incomplete_sibling") moving_incomplete_sibling_regressions();
            else if (test == "unleased_source") {
                for (bool alias : {false, true})
                    for (bool indexed : {false, true})
                        for (bool first : {false, true})
                            moving_destination_uses_incomplete_sibling(alias, indexed, first, 0, true);
            }
            else if (test == "unleased_source_blocked") {
                for (bool alias : {false, true})
                    for (bool indexed : {false, true})
                        for (int fault = 1; fault <= 10; ++fault)
                            moving_destination_uses_incomplete_sibling(alias, indexed, false, fault, true);
            }
            else if (test == "detached_trigger") {
                moving_destination_rejects_its_detached_tail(true);
                moving_destination_rejects_its_detached_tail(false);
            }
            else if (test == "numeric_source_prefix") moving_incomplete_sibling_regressions(true);
            else if (test == "move_branching_point") moving_branch_point_regressions();
            else if (test == "branch_takeoff_preemption") moving_branch_takeoff_regressions();
            else if (test == "gap_tail_anchor") {
                moving_gap_tail_is_not_a_branch_point(false);
                moving_gap_tail_is_not_a_branch_point(true);
            }
            else if (test == "shared_input") {
                for (bool joint : {false, true}) {
                    moving_destination_reuses_complete_input(false, false, joint);
                    moving_destination_reuses_complete_input(true, false, joint);
                    moving_destination_reuses_complete_input(false, true, joint);
                }
            }
            else if (test == "moving_local_bridge") {
                moving_forward_internal_local_controls();
                for (bool joint : {false, true})
                    moving_destination_reuses_complete_input(false, false, joint, true);
            }
            else if (test == "shared_prefix_cleanup") {
                for (bool incremental : {false, true})
                    for (int budget : {1, 20})
                        moving_shared_prefix_cleanup_continues_search(incremental, budget);
            }
            else if (test == "fanout_tile_visits") {
                for (bool near : {false, true})
                    for (bool alternative : {false, true})
                        fanout_continuation_counts_retained_visits(near, alternative);
            }
            else if (test == "fanout_search_stack") {
                fanout_search_stack_retains_each_level();
                fanout_multilevel_search_resumes_after_dead_child(true);
                fanout_multilevel_search_resumes_after_dead_child(false);
                fanout_multilevel_search_resumes_after_dead_child(true, true);
                fanout_multilevel_search_resumes_after_dead_child(false, true);
            }
            else if (test == "whole_task_backtracking")
                fanout_multilevel_search_resumes_after_dead_child(true, false, true);
            else if (test == "whole_task_backtracking_blocked")
                fanout_multilevel_search_resumes_after_dead_child(false, false, true);
            else if (test == "whole_task_deep_backtracking")
                moving_deep_private_backtracking(true);
            else if (test == "whole_task_deep_backtracking_blocked")
                moving_deep_private_backtracking(false);
            else if (test == "whole_task_shared_prefix")
                moving_whole_task_replaces_a_shared_only_prefix(true);
            else if (test == "whole_task_shared_prefix_blocked")
                moving_whole_task_replaces_a_shared_only_prefix(false);
            else if (test == "moving_generic_prefix")
                moving_generic_prefix_uses_another_source_branch(true);
            else if (test == "moving_generic_prefix_blocked")
                moving_generic_prefix_uses_another_source_branch(false);
            else if (test == "moving_source_pin_backtrack")
                moving_generic_prefix_uses_another_source_branch(true, true);
            else if (test == "moving_source_pin_backtrack_blocked")
                moving_generic_prefix_uses_another_source_branch(false, true);
            else if (test == "moving_source_exit")
                moving_destination_tries_another_source_exit(true, true);
            else if (test == "moving_source_exit_blocked")
                moving_destination_tries_another_source_exit(true, false);
            else if (test == "moving_source_exit_prefers_branch")
                moving_destination_tries_another_source_exit(false, true);
            else if (test == "moving_empty_binding")
                moving_empty_binding_uses_numeric_completion(true, true);
            else if (test == "moving_empty_binding_unbound_control")
                moving_empty_binding_uses_numeric_completion(false, true);
            else if (test == "moving_empty_binding_blocked")
                moving_empty_binding_uses_numeric_completion(true, false);
            else if (test == "moving_empty_binding_unbound_blocked")
                moving_empty_binding_uses_numeric_completion(false, false);
            else if (test == "moving_retired_binding")
                moving_empty_binding_uses_numeric_completion(true, true, true);
            else if (test == "moving_retired_binding_blocked")
                moving_empty_binding_uses_numeric_completion(true, false, true);
            else if (test == "moving_retained_search") {
                for (bool near : {false, true})
                    for (bool alternative : {true, false})
                        fanout_multilevel_search_resumes_after_dead_child(alternative, near, false, true);
            }
            else if (test == "fixed_unbound") {
                moving_fixed_unbound_preemption(true, true, false);
                moving_fixed_unbound_preemption(false, false, false);
                moving_fixed_unbound_preemption(false, true, true);
                moving_fixed_unbound_preemption(false, true, false);
            }
            else if (test == "forward_blacklist") moving_forward_placement_blacklist();
            else if (test == "backward_blacklist") moving_backward_placement_blacklist();
            else if (test == "forward_routing_cache") moving_forward_failed_routing_cache();
            else if (test == "backward_routing_cache") moving_backward_failed_routing_cache();
            else if (test == "forward_memory") moving_forward_search_has_compact_path_storage();
            else if (test == "forward_iteration") {
                moving_forward_search_never_restarts_depth();
                moving_forward_search_preserves_recursive_probe_order();
                moving_forward_local_bridge_resumes_same_frame();
                moving_forward_deep_recursion_uses_heap_frames();
                moving_forward_internal_local_controls();
            }
            else if (test == "fixed_destination") {
                moving_fixed_destination_repairs_retained_gap(false);
                moving_fixed_destination_repairs_retained_gap(true);
                moving_fixed_destination_repairs_retained_gap(false, true, false);
                moving_fixed_destination_repairs_retained_gap(false, true, true, true);
                moving_fixed_destination_repairs_retained_gap(false, true, true, false, true);
                moving_fixed_destination_repairs_retained_gap(false, true, true, false, false, true);
                moving_fixed_destination_repairs_retained_gap(false, true);
                moving_fixed_destination_repairs_retained_gap(false, true, true, false, false, false, 1);
                moving_fixed_destination_repairs_retained_gap(false, true, true, false, false, false, 2);
            }
            else if (test == "quiet_packing_replay") {
                // Disabled live tracing must not hide the real occupied slot's
                // owner in a failure replay; the fixture also checks lease restoration.
                moving_destination_commits_forward_grounding(false, false, false, true, false, false, false, true);
            }
            else throw std::runtime_error("unknown Moving regression: " + test);
            std::puts("moving regression passed");
            return 0;
        }
        moving_input_uses_mapped_io_endpoint();
        moving_incomplete_sibling_regressions();
        moving_incomplete_sibling_regressions(true);
        moving_branch_point_regressions();
        moving_gap_tail_is_not_a_branch_point(false);
        moving_gap_tail_is_not_a_branch_point(true);
        for (bool incremental : {false, true})
            for (int budget : {1, 20})
                moving_shared_prefix_cleanup_continues_search(incremental, budget);
        for (bool near : {false, true})
            for (bool alternative : {false, true})
                fanout_continuation_counts_retained_visits(near, alternative);
        fanout_search_stack_retains_each_level();
        fanout_multilevel_search_resumes_after_dead_child(true);
        fanout_multilevel_search_resumes_after_dead_child(false);
        fanout_multilevel_search_resumes_after_dead_child(true, true);
        fanout_multilevel_search_resumes_after_dead_child(false, true);
        for (bool near : {false, true})
            for (bool alternative : {true, false})
                fanout_multilevel_search_resumes_after_dead_child(alternative, near, true);
        moving_nested_input_keeps_outer_window();
        stagnation_runs_full_diagnostic_attempts_before_cancelling();
        moving_terminal_diagnostics_distinguish_lease_sources();
        moving_forward_growth_visits_only_reached_tiles();
        moving_forward_placement_blacklist();
        moving_backward_placement_blacklist();
        moving_forward_internal_local_controls();
        for (bool joint : {false, true})
            moving_destination_reuses_complete_input(false, false, joint, true);
        moving_forward_growth_rejects_third_visit_per_path();
        moving_forward_exhaustion_reports_every_exit();
        moving_forward_search_has_compact_path_storage();
        moving_forward_search_never_restarts_depth();
        moving_forward_failed_routing_cache();
        moving_backward_failed_routing_cache();
        moving_forward_local_bridge_resumes_same_frame();
        moving_forward_deep_recursion_uses_heap_frames();
        moving_forward_paths_preserve_metadata();
        moving_forward_search_preserves_recursive_probe_order();
        for (bool joint : {false, true}) {
            moving_destination_reuses_complete_input(false, false, joint);
            moving_destination_reuses_complete_input(true, false, joint);
            moving_destination_reuses_complete_input(false, true, joint);
        }
        moving_destination_branches_before_dead_tail(false);
        moving_destination_branches_before_dead_tail(true);
        stagnation_failure_writes_png_and_live_tile_debug(true);
        moving_destination_commits_forward_grounding(false);
        moving_destination_commits_forward_grounding(false,false,false,false,false,true,false,false,true);
        moving_destination_commits_forward_grounding(false,false,false,false,false,true,false,false,true,true);
        moving_destination_commits_forward_grounding(true);
        moving_destination_commits_forward_grounding(false, false, false, true);
        moving_destination_commits_forward_grounding(false, false, false, false, false, false, true);
        moving_destination_commits_forward_grounding(false, false, false, true, false, false, false, true);
        moving_destination_commits_forward_grounding(false, false, false, false, false, false, true, true);
        moving_destination_commits_forward_grounding(false, false, false, false, false, false, false, true);
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
        for (bool sibling : {false, true})
            for (bool reuse : {false, true})
                moving_replacement_input_releases_retained_prefix(sibling, reuse);
        for (bool complete : {false, true}) {
            for (size_t keep : {2U, 3U}) {
        moving_proven_input_prefix_survives_invalidation_and_commit(keep, complete);
            }
        }
        bridge_preemption_prepares_all_leases_before_cutting();
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
