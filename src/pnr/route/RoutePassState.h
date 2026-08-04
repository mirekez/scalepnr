#pragma once

#include "Crossbar.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pnr
{

// Expand a Moving placement cluster through generated tile-local endpoint links.
// The growing-index traversal includes complete chains without duplicating members.
template<typename Member, typename NeighborFn>
void appendMovingEndpointChain(std::vector<Member>& cluster, NeighborFn&& neighbors)
{
    for (size_t member_index = 0; member_index < cluster.size(); ++member_index) {
        for (Member neighbor : neighbors(cluster[member_index])) {
            if (neighbor != Member{}
                && std::find(cluster.begin(), cluster.end(), neighbor) == cluster.end()) {
                cluster.push_back(neighbor);
            }
        }
    }
}

// Rehome generated endpoint chains in dependency order without sorting them.
// A full pass with no successful placement proves the remaining chain blocked.
template<typename Member, typename TryFn>
bool resolveMovingEndpointDependencies(std::vector<Member>& endpoints, TryFn&& try_endpoint)
{
    std::vector<bool> done(endpoints.size(), false);
    size_t remaining = endpoints.size();
    while (remaining != 0) {
        bool progress = false;
        for (size_t index = 0; index < endpoints.size(); ++index) {
            if (done[index] || !try_endpoint(endpoints[index])) {
                continue;
            }
            done[index] = true;
            --remaining;
            progress = true;
        }
        if (!progress) {
            return false;
        }
    }
    return true;
}

// Route attempts are pass-local; blocker ancestry persists to prevent later reverse cycles.
inline void resetPassPreemptionContainers(std::unordered_set<std::string>& names,
                                           std::unordered_map<std::string, std::string>& blockers)
{
    names.clear();
    (void)blockers;
}

// Reject a victim already present in the current route's preemption ancestry.
inline bool preemptionWouldCycle(const std::unordered_map<std::string, std::string>& blockers,
                                 const std::string& current, const std::string& victim)
{
    std::string cursor = current;
    std::unordered_set<std::string> visited;
    while (!cursor.empty() && visited.insert(cursor).second) {
        if (cursor == victim) {
            return true;
        }
        auto it = blockers.find(cursor);
        if (it == blockers.end()) {
            return false;
        }
        cursor = it->second;
    }
    return !cursor.empty();
}

// Preserve the first blocker parent so later preemptions cannot erase cycle history.
inline void rememberPreemptionBlocker(std::unordered_map<std::string, std::string>& blockers,
                                      const std::string& victim, const std::string& blocker)
{
    if (!victim.empty() && !blocker.empty()) {
        blockers.try_emplace(victim, blocker);
    }
}

// Build an ordering-only state that can expose leased sources to preemption checks.
inline fpga::CBState sourceCandidateIterationState(const fpga::CBState& live,
                                                   NodeMask candidate_sources,
                                                   bool include_leased_sources)
{
    fpga::CBState result = live;
    if (include_leased_sources) {
        result.src.jump &= ~candidate_sources;
    }
    return result;
}

// Deadend learning guides Generic trunks only; later stages operate after
// preemption or placement changes and must reconsider every unleased source.
inline bool routingStageIgnoresDeadends(bool fanout_stage, bool moving_stage)
{
    return fanout_stage || moving_stage;
}

inline bool routingStageUsesDeadends(bool fanout_stage, bool moving_stage)
{
    return !routingStageIgnoresDeadends(fanout_stage, moving_stage);
}

// A structural fanout deadend can skip ordinary continuation only when there
// is no nearby endpoint from which the grounding search can still recover.
inline bool structuralDeadendStopsBeforeDocking(bool start_from_dst,
                                                bool final_step_missing,
                                                bool structural_deadend,
                                                bool has_docking_candidate)
{
    return start_from_dst && final_step_missing && structural_deadend
        && !has_docking_candidate;
}

// Charge every scheduler iteration to its logical stage, including iterations
// that leave through an early continue before pass statistics are emitted.
class RouteStageTimeCharge
{
public:
    explicit RouteStageTimeCharge(double& stage_seconds)
        : stage_seconds(stage_seconds), start(std::chrono::steady_clock::now())
    {
    }

    ~RouteStageTimeCharge()
    {
        finish();
    }

    double finish()
    {
        if (!active) {
            return elapsed;
        }
        elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        stage_seconds += elapsed;
        active = false;
        return elapsed;
    }

private:
    double& stage_seconds;
    std::chrono::steady_clock::time_point start;
    double elapsed = 0.0;
    bool active = true;
};

inline double routeStageSecondsRemaining(double elapsed, double budget)
{
    return elapsed < budget ? budget - elapsed : 0.0;
}

// A logical stage has one cumulative budget across all of its re-entries.
inline bool routeStageTimeoutIsFatal(double elapsed, double budget, bool tasks_remain)
{
    return elapsed >= budget && tasks_remain;
}

// Fanout may hand unfinished work to Moving at its deadline; terminal stages
// must fail rather than silently passing unfinished work onward.
inline bool routeStageTimeoutRequiresFailure(bool timeout_reached, bool can_handoff)
{
    return timeout_reached && !can_handoff;
}

// Generic needs one completed route per physical source, not one particular
// sink. Rotate a repeatedly failed seed to another deferred sibling.
inline bool shouldRotateFailedGenericSeed(bool generic_mode, bool complete,
                                          bool progress, bool route_empty)
{
    return generic_mode && !complete && !progress && route_empty;
}

template<typename Task, typename SameTask, typename AppendFailed>
bool rotateFailedGenericSeedTask(Task& task, std::vector<Task>& primary_deferred,
                                 std::vector<Task>& secondary_deferred,
                                 SameTask same_task, AppendFailed append_failed,
                                 size_t attempt_limit = 2)
{
    if (task.fanout || !task.from || task.attempt < attempt_limit) {
        return false;
    }
    auto rotate_from = [&](std::vector<Task>& queue) {
        auto alternate = std::find_if(queue.begin(), queue.end(), [&](const Task& candidate) {
            return candidate.from == task.from
                && candidate.from_port == task.from_port
                && !same_task(candidate, task);
        });
        if (alternate == queue.end()) {
            return false;
        }
        Task failed = task;
        failed.fanout = true;
        failed.attempt = 0;
        Task replacement = std::move(*alternate);
        queue.erase(alternate);
        replacement.fanout = false;
        replacement.attempt = 0;
        append_failed(std::move(failed));
        task = std::move(replacement);
        return true;
    };
    return rotate_from(primary_deferred) || rotate_from(secondary_deferred);
}

inline bool routingIgnoresDeadends(bool deadends_enabled, bool fanout_stage, bool moving_stage)
{
    return !deadends_enabled || routingStageIgnoresDeadends(fanout_stage, moving_stage);
}

// A Fanout task cannot run until one route from the same physical source port
// has completed the Generic stage and provides a branchable source tree.
inline bool fanoutWaitsForGenericSeed(bool fanout_stage, bool task_is_fanout,
                                      bool task_is_complete, bool has_complete_seed)
{
    return fanout_stage && task_is_fanout && !task_is_complete && !has_complete_seed;
}

// Fanout starts only after every physical source has a branchable Generic trunk.
inline bool fanoutStageCanStart(size_t ready_fanouts, size_t missing_seed_fanouts)
{
    return ready_fanouts != 0 && missing_seed_fanouts == 0;
}

inline NodeMask routingStageDeadends(NodeMask deadends, bool fanout_stage, bool moving_stage)
{
    return routingStageIgnoresDeadends(fanout_stage, moving_stage) ? NodeMask{} : deadends;
}

// Every bounded search must remember edges it has already proved unusable.
// Later stages ignore only deadends persisted by earlier routing attempts.
inline NodeMask effectiveSearchDeadends(NodeMask current_search, NodeMask persistent,
                                        bool ignore_persistent)
{
    return current_search | (ignore_persistent ? NodeMask{} : persistent);
}

struct FailedEdgePolicy
{
    bool persist = false;
    bool retry_parent = false;
};

// Persistence is stage-specific, but retrying the parent depends only on the
// failed child having a valid incoming edge. These decisions must stay independent.
inline FailedEdgePolicy failedEdgePolicy(bool ignore_persistent, bool persist_requested,
                                         bool has_parent, bool has_incoming_jump)
{
    bool retry_parent = has_parent && has_incoming_jump;
    return {
        .persist = retry_parent && persist_requested && !ignore_persistent,
        .retry_parent = retry_parent,
    };
}

// Keep leased source bits visible only when the caller will inspect their route owners.
inline NodeMask availableSourceCandidates(NodeMask candidates, const fpga::CBState& live,
                                          NodeMask deadends, bool include_leased_sources)
{
    NodeMask available = candidates & ~deadends;
    if (!include_leased_sources) {
        available &= ~live.src.jump;
    }
    return available;
}

// Free candidates are always tried before the optional leased-source preemption phase.
inline bool sourceCandidateInPhase(bool leased, int phase)
{
    return phase == 0 ? !leased : leased;
}

// A route starting at a destination reuses it only when a committed trunk
// already owns the bit. A partial-route landing is unleased and must own it.
inline bool routeStartReusesDestination(const fpga::CBState& state,
                                        bool branch_from_existing,
                                        bool start_from_dst, int dst)
{
    return branch_from_existing && start_from_dst
        && dst >= 0 && dst < CB_MAX_NODES
        && (state.dst.jump & (NodeMask{0,1} << dst)) != NodeMask{};
}

// An exact terminal local may be shared inside one net's route tree. A route
// from another net owns a different electrical signal and blocks that reuse.
inline bool fanoutMayReuseExactLocal(bool maps_to_target,
                                     bool has_foreign_owner)
{
    return maps_to_target && !has_foreign_owner;
}

// A fanout fork reuses a destination leased by its existing trunk. It may
// lease a new source or joints, but it must never create an unowned dst lease.
inline bool leaseExistingDestinationFork(fpga::CBState& state, int dst, int src,
                                         int joint = -1, bool ignore_deadend = false,
                                         int joint2 = -1)
{
    if (dst < 0 || dst >= CB_MAX_NODES || src < 0 || src >= CB_MAX_NODES) {
        return false;
    }
    NodeMask dst_bit = NodeMask{0,1} << dst;
    NodeMask src_bit = NodeMask{0,1} << src;
    NodeMask joint_bit = joint >= 0 ? (NodeMask{0,1} << joint) : NodeMask{};
    NodeMask joint2_bit = joint2 >= 0 ? (NodeMask{0,1} << joint2) : NodeMask{};
    if ((state.dst.jump & dst_bit) == NodeMask{}
        || (!ignore_deadend && (state.src_deadend.jump & src_bit) != NodeMask{})
        || (state.src.jump & src_bit) != NodeMask{}
        || (joint >= 0 && (state.joint.jump & joint_bit) != NodeMask{})
        || (joint2 >= 0 && (state.joint.jump & joint2_bit) != NodeMask{})) {
        return false;
    }
    state.src.jump |= src_bit;
    state.joint.jump |= joint_bit | joint2_bit;
    return true;
}

// A terminal path must not consume joints required by another packed input.
inline bool terminalEntryAvoidsReservedJoints(int joint, int joint2, NodeMask reserved)
{
    NodeMask used{};
    if (joint >= 0) {
        used |= NodeMask{0,1} << joint;
    }
    if (joint2 >= 0) {
        used |= NodeMask{0,1} << joint2;
    }
    return (used & reserved) == NodeMask{};
}

// A direct target hop may retry an old sticky deadend, but not an edge that
// already failed terminal entry during the current bounded search.
inline bool targetHopMayBypassDeadend(bool reaches_target, bool search_deadend)
{
    return reaches_target && !search_deadend;
}

// Keep the normal incremental search depth inside the docking window.  The
// earliest in-window node is still selected as the docking anchor afterward.
inline int suffixDepthBeforeDocking(int start_distance, int docking_radius,
                                    int normal_depth)
{
    (void)start_distance;
    (void)docking_radius;
    return normal_depth;
}

// A routed endpoint inside the docking window remains useful even when none of
// its ordinary forward exits can be leased; preserve it for bidirectional docking.
inline bool preserveBlockedEndpointForDocking(bool accepted_edge, int depth,
                                              int distance, int docking_radius)
{
    return !accepted_edge && depth > 0 && distance <= docking_radius;
}

// Moving retains every nearby routed rail as a docking candidate even when
// ordinary forward expansion is legal, because that exit may lead away.
inline bool rememberMovingDockingCandidate(bool moving_stage, int depth,
                                           int distance, int docking_radius)
{
    return moving_stage && depth > 0 && distance <= docking_radius;
}

// Takeoff may preempt a remembered transit victim only after every free exit failed.
inline bool shouldPreemptTakeoff(bool accepted_free_path, bool takeoff)
{
    return !accepted_free_path && takeoff;
}

// Ordinary transit displacement is limited to takeoff. A protected
// infrastructure route may also displace transit after an intermediate step.
inline bool transitPreemptionStepAllowed(bool preemption_enabled,
                                         bool protected_route,
                                         bool first_source_step)
{
    return preemption_enabled && (first_source_step || protected_route);
}

// Grounding may evict a transit destination only when every physically incoming
// destination node that can reach the requested local is already leased.
template<typename IsTransitOwned>
int groundingPreemptionDst(fpga::CBType& type, const fpga::CBState& state, int local,
                           NodeMask incoming_dsts, IsTransitOwned&& is_transit_owned)
{
    if (local < 0 || local >= CB_MAX_NODES) {
        return -1;
    }
    type.ensureDerivedMasks();
    NodeMask candidates = type.dsts_reaching_local[local].jump & incoming_dsts;
    if (candidates == NodeMask{} || (candidates & ~state.dst.jump) != NodeMask{}) {
        return -1;
    }

    int victim_dst = -1;
    candidates.for_each_set_bit([&](int dst) {
        if (!is_transit_owned(dst)) {
            return false;
        }
        victim_dst = dst;
        return true;
    });
    return victim_dst;
}

template<typename IsTransitOwned>
int groundingPreemptionDst(fpga::CBType& type, const fpga::CBState& state, int local,
                           IsTransitOwned&& is_transit_owned)
{
    if (local < 0 || local >= CB_MAX_NODES) {
        return -1;
    }
    type.ensureDerivedMasks();
    return groundingPreemptionDst(type, state, local,
        type.dsts_reaching_local[local].jump,
        std::forward<IsTransitOwned>(is_transit_owned));
}

// A successful grounding preemption must claim the freed terminal path before
// the displaced victim can be routed again by the outer pass scheduler.
template<typename RetryGrounding>
bool retryGroundingAfterPreemption(bool preempted, RetryGrounding&& retry_grounding)
{
    return preempted && retry_grounding();
}

// Requeue an atomically removed source tree as one Generic seed followed by
// Fanout siblings, preserving this ordering across all nets from the source port.
template<typename Task, typename Enqueue>
size_t enqueuePreemptedSourceTasks(std::vector<Task>& source_tasks,
                                   bool& generic_task_added, Enqueue&& enqueue)
{
    size_t queued = 0;
    for (Task& task : source_tasks) {
        task.fanout = generic_task_added;
        generic_task_added = true;
        if (enqueue(task)) {
            ++queued;
        }
    }
    return queued;
}

// Atomic source-tree invalidation requeues every binding, including a sibling
// whose route was already empty before this particular cleanup call.
template<typename Task, typename Enqueue>
size_t enqueueInvalidatedSourceTreeTasks(std::vector<Task>& source_tasks,
                                         bool& generic_task_added,
                                         bool route_state_changed,
                                         Enqueue&& enqueue)
{
    // Physical cleanup can report false when an earlier operation already emptied
    // a sibling, but that binding still needs a new routing task.
    (void)route_state_changed;
    // Restore one Generic seed followed by Fanout siblings for the whole source tree.
    return enqueuePreemptedSourceTasks(source_tasks, generic_task_added,
        // Preserve the scheduler supplied by the caller.
        std::forward<Enqueue>(enqueue));
}

// Relocation schedules every affected incomplete binding immediately, including
// bindings that were already empty before the selected route was unrouted.
template<typename Task, typename IsComplete, typename Enqueue>
size_t enqueueIncompleteAffectedTasks(const std::vector<Task>& affected_tasks,
                                      IsComplete&& is_complete, Enqueue&& enqueue)
{
    size_t queued = 0;
    for (const Task& task : affected_tasks) {
        if (is_complete(task)) {
            continue;
        }
        queued += enqueue(task) ? 1 : 0;
    }
    return queued;
}

// A relocation exists to repair its triggering binding. Put that binding first
// so an impossible candidate is rejected before rebuilding unrelated routes.
template<typename Task, typename SameTask>
bool prioritizeMovingTrigger(std::vector<Task>& tasks, const Task& trigger,
                             SameTask&& same_task)
{
    auto found = std::find_if(tasks.begin(), tasks.end(),
        [&](const Task& task) { return same_task(task, trigger); });
    if (found == tasks.end()) {
        return false;
    }
    std::rotate(tasks.begin(), found, found + 1);
    return true;
}

// Replacing a focused Moving queue must defer active source-tree siblings that
// are not present in the replacement set; otherwise their empty routes vanish.
template<typename Task, typename SameTask, typename Enqueue>
size_t deferDisplacedActiveTasks(const std::vector<Task>& active_tasks,
                                 const std::vector<Task>& replacement_tasks,
                                 SameTask&& same_task, Enqueue&& enqueue)
{
    size_t deferred = 0;
    for (const Task& active : active_tasks) {
        bool replaced = std::any_of(replacement_tasks.begin(), replacement_tasks.end(),
            [&](const Task& replacement) { return same_task(active, replacement); });
        if (!replaced && enqueue(active)) {
            ++deferred;
        }
    }
    return deferred;
}

// Atomic preemption should invalidate the fewest already-routed branches.
inline bool preferPreemptionVictim(size_t candidate_tree_size, size_t current_tree_size)
{
    return candidate_tree_size < current_tree_size;
}

// Fanout routing may remove only a private suffix; its Generic trunk is immutable.
inline bool canPreemptFanoutSuffix(bool fanout_stage, bool route_has_shared_suffix,
                                   bool blocking_node_is_shared)
{
    return !fanout_stage || (route_has_shared_suffix && !blocking_node_is_shared);
}

// Moving may rip unfinished trees, but a completed moved endpoint is immutable.
inline bool canPreemptMovingRoute(bool moving_stage, bool endpoint_finished)
{
    return !moving_stage || !endpoint_finished;
}

// A Fanout pass is still productive when an existing branch advances or changes.
inline bool fanoutPassMadeProgress(size_t completed, size_t advanced, size_t changed)
{
    return completed != 0 || advanced != 0 || changed != 0;
}

// Consuming a failed Fanout branch advances its rotation exactly once and
// resets the retry count for the next branch candidate.
inline void consumeFanoutBranch(size_t& branch_offset, size_t& branch_attempt)
{
    ++branch_offset;
    branch_attempt = 0;
}

// Removing a shared-prefix replica is cleanup for an already-consumed branch;
// it must not skip another branch candidate.
inline void cleanFanoutSharedPrefix(size_t& branch_attempt)
{
    branch_attempt = 0;
}

// Fanout prefers a well-connected fork, but may use any fork with a free exit.
inline bool fanoutBranchIsPreferred(int free_exits)
{
    return free_exits > 2;
}

inline bool fanoutBranchIsUsableFallback(int free_exits)
{
    return free_exits > 0;
}

// Fanout partial movement is bounded; persistent low completion belongs to Moving.
inline bool fanoutShouldHandOff(int stage_pass, size_t completed, int recursion_limit)
{
    return stage_pass >= std::max(32, recursion_limit * 8) && completed <= 2;
}

// Moving keeps the current placement while an incident route completes or grows.
inline bool movingPassMadeProgress(size_t completed, size_t advanced)
{
    return completed != 0 || advanced != 0;
}

struct MovingTerminalPath
{
    int local = -1;
    int dst = -1;
    int joint = -1;
    int joint2 = -1;
};

// Count distinct bottleneck resources for a candidate terminal. Requirements
// with fewer alternatives must reserve first so flexible inputs cannot take them.
inline size_t movingTerminalPathFlexibility(
    const std::vector<MovingTerminalPath>& paths)
{
    std::unordered_set<uint64_t> resources;
    for (const MovingTerminalPath& path : paths) {
        if (path.local < 0 || path.dst < 0) {
            continue;
        }
        if (path.joint >= 0) {
            uint64_t joint = static_cast<uint32_t>(path.joint);
            uint64_t joint2 = path.joint2 >= 0
                ? static_cast<uint32_t>(path.joint2)
                : std::numeric_limits<uint32_t>::max();
            resources.insert((joint << 32) | joint2);
        }
        else {
            resources.insert((uint64_t{1} << 63)
                | static_cast<uint32_t>(path.dst));
        }
    }
    return resources.empty() ? std::numeric_limits<size_t>::max()
                             : resources.size();
}

// Return a stable most-constrained-first order without changing caller data.
inline std::vector<size_t> movingTerminalReservationOrder(
    const std::vector<std::vector<MovingTerminalPath>>& requirements)
{
    std::vector<size_t> order;
    std::vector<bool> selected(requirements.size(), false);
    order.reserve(requirements.size());
    while (order.size() < requirements.size()) {
        size_t best = requirements.size();
        size_t best_flexibility = std::numeric_limits<size_t>::max();
        size_t best_paths = std::numeric_limits<size_t>::max();
        for (size_t index = 0; index < requirements.size(); ++index) {
            if (selected[index]) {
                continue;
            }
            size_t flexibility =
                movingTerminalPathFlexibility(requirements[index]);
            size_t path_count = requirements[index].size();
            if (best == requirements.size() ||
                std::pair{flexibility, path_count} <
                    std::pair{best_flexibility, best_paths}) {
                best = index;
                best_flexibility = flexibility;
                best_paths = path_count;
            }
        }
        if (best == requirements.size()) {
            break;
        }
        selected[best] = true;
        order.push_back(best);
    }
    return order;
}

// Reserve one complete input path in temporary masks while validating a
// candidate placement. Failed alternatives leave every mask unchanged.
inline bool reserveMovingTerminalPath(
    const std::vector<MovingTerminalPath>& paths, NodeMask& leased_pins,
    NodeMask& leased_locals, NodeMask& leased_dsts, NodeMask& leased_joints,
    MovingTerminalPath* selected = nullptr)
{
    for (const MovingTerminalPath& path : paths) {
        if (path.local < 0 || path.dst < 0) {
            continue;
        }
        NodeMask local_bit = NodeMask{0, 1} << path.local;
        NodeMask dst_bit = NodeMask{0, 1} << path.dst;
        NodeMask joint_bit = path.joint >= 0
            ? NodeMask{0, 1} << path.joint : NodeMask{};
        NodeMask joint2_bit = path.joint2 >= 0
            ? NodeMask{0, 1} << path.joint2 : NodeMask{};
        if ((leased_pins & local_bit) != NodeMask{}
            || (leased_locals & local_bit) != NodeMask{}
            || (leased_dsts & dst_bit) != NodeMask{}
            || (leased_joints & (joint_bit | joint2_bit)) != NodeMask{}) {
            continue;
        }
        leased_pins |= local_bit;
        leased_locals |= local_bit;
        leased_dsts |= dst_bit;
        leased_joints |= joint_bit | joint2_bit;
        if (selected) {
            *selected = path;
        }
        return true;
    }
    return false;
}

// Count passes since the last completed incident route.  Prefix growth alone
// cannot prove that the current placement will ever route all of its pins.
inline int updateMovingNoCompletionPasses(int no_completion_passes,
                                          bool moving_mode,
                                          size_t completed)
{
    if (!moving_mode || completed != 0) {
        return 0;
    }
    return no_completion_passes + 1;
}

// Give one bounded routing slice to a focused placement. Prefix growth is
// retained, but no completed incident route means the next placement is due.
inline int movingFocusNoCompletionLimit(int recursion_limit)
{
    return std::max(1, recursion_limit);
}

// Moving delays a fanout only while the Generic seed for that same physical
// source pin is pending; an unrelated seed cannot block the fanout queue.
inline bool movingFanoutWaitsForSourceSeed(bool moving_mode, bool task_is_fanout,
                                           bool same_source_seed_pending)
{
    return moving_mode && task_is_fanout && same_source_seed_pending;
}

// Once every unfinished route targets a downstream load, the current focus is
// stable and Moving must hand the work off instead of moving that driver again.
inline bool movingFocusHandsOffToLoads(bool has_route_into_focus,
                                       bool has_route_out_of_focus)
{
    return !has_route_into_focus && has_route_out_of_focus;
}

// Every focused placement receives one bounded routing slice. Completed and
// incremental routes are retained, but sibling growth cannot extend the slice.
inline int updateMovingPlacementNoProgressPasses(int no_progress_passes,
                                                 bool pass_made_progress)
{
    (void)pass_made_progress;
    return no_progress_passes + 1;
}

// Track each required incident route independently; progress on one sibling
// must not hide another route that is persistently unable to grow.
inline size_t updateMovingTaskNoProgressPasses(size_t no_progress_passes,
                                               bool task_made_progress)
{
    return task_made_progress ? 0 : no_progress_passes + 1;
}

inline bool movingTaskNoProgressExhausted(size_t no_progress_passes,
                                          int recursion_limit)
{
    return no_progress_passes >= static_cast<size_t>(std::max(1, recursion_limit));
}

// A placement gets a bounded number of unproductive passes before relocation.
inline bool movingPlacementPassesExhausted(int placement_passes, int recursion_limit)
{
    return placement_passes >= std::max(1, recursion_limit);
}

// A focused placement gets one bounded Generic/Fanout pass slice. If its
// incident routes remain incomplete, Moving tries the next sink placement.
inline bool focusedMovingShouldRelocate(bool blocked, int stagnant_passes,
                                        int stagnation_limit,
                                        bool no_completion_exhausted,
                                        bool placement_passes_exhausted)
{
    return blocked || stagnant_passes >= stagnation_limit
        || no_completion_exhausted || placement_passes_exhausted;
}

// Newly recovered incident routes run once at the current placement. If the
// same incomplete bindings return, Moving must try the next placement.
inline bool relocateAfterIncidentRequeue(size_t requeued, size_t prior_recoveries)
{
    return requeued == 0 || prior_recoveries != 0;
}

// Candidate validation and commit must refer to the same still-placed resource
// position; repacking can silently select an unvalidated position.
inline bool acceptMovingPlacedCandidate(bool candidate_remains_placed,
                                        bool final_placement_was_tried)
{
    return candidate_remains_placed && !final_placement_was_tried;
}

// Tile-local void connections are complete without an inter-tile Wire route.
inline bool incidentBindingNeedsRouting(bool void_net, bool has_route, bool route_complete)
{
    return !void_net && (!has_route || !route_complete);
}

// An ownerless duplicate is stale only when an identical complete physical binding exists.
inline bool discardOwnerlessDuplicateBinding(bool has_route, bool has_complete_twin)
{
    return !has_route && has_complete_twin;
}

// Recovered incident work branches from an existing physical source tree.
inline bool incidentBindingIsFanout(bool has_other_complete_source_binding)
{
    return has_other_complete_source_binding;
}

// Resource endpoints occupy both their resource tile and attached route tile.
inline bool endpointRouteTileMatches(const Coord& resource, const Coord& attached,
                                     const Coord& query)
{
    return (resource.x == query.x && resource.y == query.y)
        || (attached.x == query.x && attached.y == query.y);
}

// Non-Moving routing and the active Moving focus may preempt transit routes.
// Unfocused Moving repair must not disturb unrelated completed route trees.
inline bool canPreemptDuringFocusedMove(bool moving_stage,
                                        bool has_moving_focus)
{
    return !moving_stage || has_moving_focus;
}

// Moving must rebuild an incomplete source tree before selecting its new Generic seed.
inline bool resetIncompleteSourceTree(bool moving_mode, bool task_is_fanout,
                                      bool has_complete_seed, bool has_incomplete_binding)
{
    return moving_mode && !task_is_fanout && !has_complete_seed && has_incomplete_binding;
}

// The active task's partial prefix is incremental state, not a stale sibling tree.
inline bool incompleteBindingBlocksMovingSeed(bool binding_is_current, bool route_nonempty,
                                              bool route_complete)
{
    return !binding_is_current && route_nonempty && !route_complete;
}

// A bounded Moving Generic failure preserves its committed prefix.  Genuine
// stagnation is resolved by relocating the focused sink, not by hop rollback.
inline bool failedMovingGenericKeepsPrefix(bool moving_mode, bool task_is_fanout)
{
    return moving_mode && !task_is_fanout;
}

// Exhausting placements defers a Moving focus; it is not proof that its routes are impossible.
inline bool movingFocusPlacementsExhausted(size_t tried, size_t retry_limit)
{
    return retry_limit != 0 && tried >= retry_limit;
}

// The scheduler receives roughly cells/10 as its attempt quantum. Retain ten
// quanta so a focus explores roughly one candidate per design cell before cycling.
inline size_t movingCandidateRetryLimit(int move_attempt_limit)
{
    return static_cast<size_t>(std::max(1, move_attempt_limit)) * 10;
}

// Moving placement history uses independent 16-bit coordinates and a 32-bit
// resource position; coordinate fields must never alias one another.
inline uint64_t movingPlacementKey(int x, int y, int pos)
{
    return (static_cast<uint64_t>(static_cast<uint16_t>(x)) << 48)
        | (static_cast<uint64_t>(static_cast<uint16_t>(y)) << 32)
        | static_cast<uint32_t>(pos);
}

// Optional focus slicing retains candidate history across scheduler visits. A
// zero slice limit keeps an atomic focus active until all its routes complete.
inline bool movingFocusSliceExhausted(size_t tried, size_t slice_start,
                                      size_t slice_limit)
{
    return slice_limit != 0 && tried >= slice_start
        && tried - slice_start >= slice_limit;
}

// A sliced focus resumes its deterministic candidate sequence after yielding.
// A complete design-sized cycle may restart only after full exhaustion.
inline bool retainMovingPlacementHistory(size_t unfinished_tasks,
                                         size_t tail_threshold,
                                         bool full_cycle_exhausted)
{
    (void)unfinished_tasks;
    (void)tail_threshold;
    return !full_cycle_exhausted;
}

// A cooldown cannot expire when every unfinished candidate is blocked because
// relocation epochs advance only after a candidate is selected for relocation.
inline bool movingCooldownMustBeReleased(size_t unfinished_tasks,
                                         size_t relocation_attempts,
                                         bool has_blocked_candidates,
                                         int relocation_epoch,
                                         int last_release_epoch)
{
    return unfinished_tasks != 0 && relocation_attempts == 0
        && has_blocked_candidates && relocation_epoch != last_release_epoch;
}

// A moved packing cluster is positioned from every route that crosses its
// boundary; connections internal to the cluster do not supply an anchor.
template<typename Endpoint, typename IsMoved>
Endpoint* externalMoveAnchor(Endpoint* from, Endpoint* to, IsMoved is_moved)
{
    bool from_moved = from && is_moved(from);
    bool to_moved = to && is_moved(to);
    if (from_moved == to_moved) {
        return nullptr;
    }
    return from_moved ? to : from;
}

// Every member of a strict packing cluster shares one Moving history owner, so
// selecting another member cannot restart the same placement sequence.
template<typename Endpoint, typename Less>
Endpoint* movingClusterOwner(const std::vector<Endpoint*>& cluster, Endpoint* fallback,
                             Less less)
{
    Endpoint* owner = fallback;
    for (Endpoint* member : cluster) {
        if (member && (!owner || less(member, owner))) {
            owner = member;
        }
    }
    return owner;
}

template<typename Endpoint>
Endpoint* movingClusterOwner(const std::vector<Endpoint*>& cluster, Endpoint* fallback)
{
    return movingClusterOwner(cluster, fallback, [](Endpoint* left, Endpoint* right) {
        return reinterpret_cast<uintptr_t>(left) < reinterpret_cast<uintptr_t>(right);
    });
}

// A completed focus stays fixed while its incident routes remain complete.
// Later route-tree invalidation makes the mark stale and permits relocation.
inline bool movingFinishedMarkIsValid(bool marked, bool incident_routes_complete)
{
    // A finished mark protects placement only while all routes touching it survive.
    return marked && incident_routes_complete;
}

// An explicitly queued incomplete endpoint invalidates a prior finished mark,
// including when no route binding exists yet for the missing connection.
inline bool movingQueuedTaskInvalidatesFinishedMark(bool marked,
                                                    bool task_is_incomplete)
{
    return marked && task_is_incomplete;
}

struct MovingSeedNormalization
{
    size_t promoted = 0;
    size_t demoted = 0;
};

// Keep exactly one Generic task for every source without a completed seed.
// Sources that already own a completed seed contain Fanout tasks only.
template<typename Task, typename SourceKey, typename HasCompleteSeed>
MovingSeedNormalization normalizeMovingSourceRoles(std::vector<Task>& tasks,
                                                   SourceKey source_key,
                                                   HasCompleteSeed has_complete_seed)
{
    std::unordered_map<std::string, size_t> first_task;
    std::unordered_map<std::string, size_t> preferred_generic;
    std::unordered_set<std::string> completed_sources;
    for (size_t index = 0; index < tasks.size(); ++index) {
        std::string key = source_key(tasks[index]);
        if (key.empty()) {
            continue;
        }
        first_task.try_emplace(key, index);
        if (has_complete_seed(tasks[index])) {
            completed_sources.insert(key);
        }
        if (!tasks[index].fanout) {
            preferred_generic.try_emplace(key, index);
        }
    }

    MovingSeedNormalization result;
    for (size_t index = 0; index < tasks.size(); ++index) {
        std::string key = source_key(tasks[index]);
        if (key.empty()) {
            continue;
        }
        size_t generic_index = preferred_generic.contains(key)
            ? preferred_generic.at(key) : first_task.at(key);
        bool should_be_fanout = completed_sources.contains(key) || index != generic_index;
        if (tasks[index].fanout == should_be_fanout) {
            continue;
        }
        tasks[index].fanout = should_be_fanout;
        if (should_be_fanout) {
            ++result.demoted;
        }
        else {
            ++result.promoted;
        }
    }
    return result;
}

// A placement cycle is bounded, but exhaustion is not permanent: congestion
// may change while other focused cells are moved before the next cycle.
template<typename Placement>
bool restartMovingPlacementCycle(std::vector<Placement>& tried, size_t retry_limit)
{
    if (!movingFocusPlacementsExhausted(tried.size(), retry_limit)) {
        return false;
    }
    tried.clear();
    return true;
}

// Track whether a Fanout pass improved the best unfinished-task count.
inline bool updateFanoutPlateau(size_t remaining, size_t& best_remaining,
                                size_t& passes_without_improvement)
{
    if (remaining < best_remaining) {
        best_remaining = remaining;
        passes_without_improvement = 0;
        return true;
    }
    ++passes_without_improvement;
    return false;
}

// Fanout transit preemption follows its stage policy without changing Generic preemption.
inline bool transitPreemptionEnabled(bool requested, bool fanout_stage,
                                     bool fanout_preemption_enabled)
{
    return requested && (!fanout_stage || fanout_preemption_enabled);
}

// Unfocused Moving retries restored work without disturbing other route trees.
// Preemption resumes after Moving isolates and relocates one endpoint focus.
inline bool movingRouteMayPreempt(bool moving_stage, bool has_moving_focus)
{
    return !moving_stage || has_moving_focus;
}

// Resource-local node numbers are a valid route-tile fallback only when the
// resource and its crossbar occupy the same grid coordinate.
template<typename Mask>
Mask mappedOutputCandidateNodes(Mask mapped_nodes, Mask resource_nodes,
                                bool same_coord, bool resource_nodes_supported)
{
    if (mapped_nodes == Mask{} && same_coord && resource_nodes_supported) {
        return resource_nodes;
    }
    return mapped_nodes;
}

// Visit Moving candidates in deterministic expanding-square order and stop as
// soon as the caller accepts one; later rings are never materialized.
template<typename Visit>
bool forEachMovingCandidateCoord(int center_x, int center_y, int radius, Visit visit)
{
    if (visit(center_x, center_y)) {
        return true;
    }
    for (int dist = 1; dist <= radius; ++dist) {
        for (int dx = -dist; dx <= dist; ++dx) {
            if (visit(center_x + dx, center_y - dist)
                || visit(center_x + dx, center_y + dist)) {
                return true;
            }
        }
        for (int dy = -dist + 1; dy <= dist - 1; ++dy) {
            if (visit(center_x - dist, center_y + dy)
                || visit(center_x + dist, center_y + dy)) {
                return true;
            }
        }
    }
    return false;
}

// Grounding starts at the first routed destination node that enters its radius.
inline int earliestGroundingAnchor(const std::vector<std::pair<int, int>>& depth_distance,
                                   int radius)
{
    for (size_t index = 0; index < depth_distance.size(); ++index) {
        if (depth_distance[index].first > 0 && depth_distance[index].second <= radius) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

// Docking uses a square coordinate window, matching the bounded search in Docking.cpp.
inline int dockingWindowDistance(const fpga::Coord& from, const fpga::Coord& to)
{
    return std::max(std::abs(from.x - to.x), std::abs(from.y - to.y));
}

// Secondary source-tree routes stay deferred until Fanout routing owns the queue.
inline bool deferToFanoutStage(bool task_is_fanout, bool fanout_stage,
                               bool moving_stage, bool moving_focus)
{
    return task_is_fanout && !fanout_stage && !moving_stage && !moving_focus;
}

// Only Fanout continuations own a removable branch suffix; failed Moving Generic routes restart.
inline bool failedContinuationOwnsOnlyBranch(bool moving_stage, bool fanout_stage,
                                             bool task_is_fanout)
{
    (void)moving_stage;
    return fanout_stage || task_is_fanout;
}

// A Generic repair matching deferred Fanout work must move to the active repair queue.
inline bool promoteGenericOutOfFanoutQueue(bool incoming_is_fanout, bool deferred_match)
{
    return !incoming_is_fanout && deferred_match;
}

// Source passthrough insertion must not restore a promoted Generic task's old
// Fanout role; that task becomes the one physical takeoff for the new source.
inline bool retargetedCurrentTaskIsFanout(bool preserve_generic_seed,
                                         bool recovered_current,
                                         bool recovered_fanout,
                                         bool recovered_has_generic)
{
    return preserve_generic_seed
        ? false
        : (recovered_current ? recovered_fanout : recovered_has_generic);
}

// When a promoted Generic task changes source identity, every recovered
// sibling remains deferred Fanout work for the replacement source tree.
inline bool retargetedSiblingTaskIsFanout(bool preserve_generic_seed,
                                         bool recovered_fanout)
{
    return preserve_generic_seed || recovered_fanout;
}

}
