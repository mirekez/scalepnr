#pragma once

#include "Crossbar.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pnr
{

// Failure identity comes from live queues, never a retained failed-search
// snapshot: that search may have succeeded on a later attempt.
template<typename Task, typename CompleteFn>
const Task* firstUnfinishedRouteTask(
    std::initializer_list<const std::vector<Task>*> queues,
    CompleteFn&& complete)
{
    for (const auto* queue : queues) {
        for (const Task& task : *queue) {
            if (!task.remove_after_pass && !complete(task)) {
                return &task;
            }
        }
    }
    return nullptr;
}

// Diagnose the newest failed attempt that still belongs to live unfinished
// work. Old completed failures must not replace the current net identity.
template<typename Task, typename CompleteFn>
const Task* latestUnfinishedRouteTask(
    std::initializer_list<const std::vector<Task>*> queues,
    CompleteFn&& complete)
{
    const Task* latest = nullptr;
    for (const auto* queue : queues) {
        for (const Task& task : *queue) {
            if (!task.remove_after_pass && !complete(task) &&
                (!latest || task.failure_sequence > latest->failure_sequence)) {
                latest = &task;
            }
        }
    }
    return latest;
}

struct RouteProgressSample
{
    bool sampled = false;
    bool stagnated = false;
    size_t start_tasks = 0;
    size_t remaining_tasks = 0;
    size_t completed_tasks = 0;
    size_t required_tasks = 0;
    unsigned stagnant_windows = 0;
};

// Measure committed queue reduction in fixed windows. Several deficient
// windows are tolerated so one expensive route probe does not abort a stage.
class RouteProgressWatchdog
{
public:
    using Clock = std::chrono::steady_clock;

    void reset(size_t remaining_tasks, Clock::time_point now,
               std::chrono::seconds window = std::chrono::seconds(60),
               unsigned required_percent = 1,
               unsigned allowed_stagnant_windows = 3)
    {
        baseline_tasks = remaining_tasks;
        current_tasks = remaining_tasks;
        window_started = now;
        window_duration = window;
        progress_percent = std::max(1U, required_percent);
        stagnant_window_limit = std::max(1U, allowed_stagnant_windows);
        stagnant_windows = 0;
        stagnation_latched = false;
        active = remaining_tasks != 0;
    }

    RouteProgressSample observe(size_t remaining_tasks,
                                Clock::time_point now)
    {
        current_tasks = remaining_tasks;
        RouteProgressSample sample;
        if (stagnation_latched) {
            sample.stagnated = true;
            sample.remaining_tasks = current_tasks;
            sample.stagnant_windows = stagnant_windows;
            return sample;
        }
        if (!active || now - window_started < window_duration) {
            return sample;
        }

        sample.sampled = true;
        sample.start_tasks = baseline_tasks;
        sample.remaining_tasks = current_tasks;
        sample.completed_tasks = baseline_tasks > current_tasks
                                     ? baseline_tasks - current_tasks
                                     : 0;
        sample.required_tasks = std::max<size_t>(
            1, (baseline_tasks * progress_percent + 99) / 100);

        const auto elapsed_windows = static_cast<unsigned>(std::max<int64_t>(
            1, std::chrono::duration_cast<std::chrono::seconds>(
                   now - window_started).count()
                   / std::max<int64_t>(1, window_duration.count())));
        if (sample.completed_tasks >= sample.required_tasks) {
            stagnant_windows = 0;
        } else {
            stagnant_windows += elapsed_windows;
        }
        sample.stagnant_windows = stagnant_windows;
        sample.stagnated = current_tasks != 0 &&
                           stagnant_windows >= stagnant_window_limit;
        stagnation_latched = sample.stagnated;

        baseline_tasks = current_tasks;
        window_started = now;
        active = current_tasks != 0;
        return sample;
    }

    bool enabled() const { return active; }

private:
    size_t baseline_tasks = 0;
    size_t current_tasks = 0;
    Clock::time_point window_started{};
    std::chrono::seconds window_duration{60};
    unsigned progress_percent = 1;
    unsigned stagnant_window_limit = 3;
    unsigned stagnant_windows = 0;
    bool stagnation_latched = false;
    bool active = false;
};

// Follow lazily recorded physical-source replacements. Queue-wide callers can
// canonicalize each task once instead of rescanning every queue per insertion.
template<typename Endpoint, typename Retargets, typename KeyFn>
bool resolveSourceRetarget(Endpoint& endpoint, const Retargets& retargets,
                           KeyFn&& key_fn)
{
    bool changed = false;
    for (size_t hop = 0; hop <= retargets.size(); ++hop) {
        auto found = retargets.find(key_fn(endpoint));
        if (found == retargets.end() || found->second == endpoint) {
            return changed;
        }
        endpoint = found->second;
        changed = true;
    }
    return changed;
}

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

// Relocation can replace generated tile-local endpoints while retaining the
// same focus pointer, so pointer identity alone cannot validate this cache.
template<typename Focus, typename EndpointSet>
void invalidateMovingEndpointCache(Focus*& cached_focus,
                                   EndpointSet& cached_endpoints)
{
    cached_focus = nullptr;
    cached_endpoints.clear();
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

// Logical aliases of one physical source tree must never preempt each other.
// Net identity is sufficient, while a canonical source key joins split nets.
inline bool preemptionOwnerIsCurrentTree(bool same_net,
                                         const std::string& current_source_key,
                                         const std::string& owner_source_key)
{
    return same_net || (!current_source_key.empty()
        && current_source_key == owner_source_key);
}

// Routes participating in one atomic relocation transaction cannot preempt
// each other while their endpoint connectivity is rebuilt.
inline bool preemptionOwnerIsTransactionProtected(
    const std::unordered_set<std::string>& protected_source_keys,
    const std::string& owner_source_key)
{
    return !owner_source_key.empty()
        && protected_source_keys.contains(owner_source_key);
}

// Synchronous input repair belongs to the active Moving transaction and may
// displace transit congestion under the same focused preemption policy.
inline bool immediateMovingInputMayPreempt(bool preemption_enabled,
                                           bool moving_sources,
                                           bool has_moving_focus)
{
    return preemption_enabled && (moving_sources || has_moving_focus);
}

struct RouteAttemptPreemption
{
    bool transit = false;
    bool docking = false;
};

// An initial Generic attempt must carry the same focused preemption policy to
// ordinary transit expansion and to the final docking boundary.
inline RouteAttemptPreemption genericRouteAttemptPreemption(bool enabled)
{
    return RouteAttemptPreemption{enabled, enabled};
}

// Roll back work created by a rejected relocation but retain foreign victim
// tasks produced by transit preemption so no displaced route is forgotten.
template<typename Task, typename IsAffected>
size_t preserveExternalPreemptionTasks(std::vector<Task>& tasks,
                                       size_t transaction_start,
                                       IsAffected&& is_affected)
{
    if (transaction_start >= tasks.size()) {
        return 0;
    }
    std::vector<Task> external;
    external.reserve(tasks.size() - transaction_start);
    for (size_t index = transaction_start; index < tasks.size(); ++index) {
        if (!is_affected(tasks[index])) {
            external.push_back(std::move(tasks[index]));
        }
    }
    tasks.resize(transaction_start);
    tasks.insert(tasks.end(), std::make_move_iterator(external.begin()),
                 std::make_move_iterator(external.end()));
    return external.size();
}

// Failed speculative Generic suffixes own no live leases, so the committed
// prefix remains the continuation frontier without any rollback.
inline bool failedGenericContinuationKeepsPrefix(bool fanout_stage,
                                                 bool moving_stage)
{
    return !fanout_stage && !moving_stage;
}

// Basic treats every blocked committed frontier as a deadend. Whether the
// frontier lacks topology or only lacks a free resource does not change retry.
inline bool failedBasicRootNeedsBackstep(bool fanout_stage, bool moving_stage,
                                         bool root_blocked)
{
    return !fanout_stage && !moving_stage && root_blocked;
}

// Pass one reserves one takeoff. Later passes retain a bounded retry budget;
// the caller stops the current turn after its first successful Generic suffix.
inline int routeTaskAttemptBudget(bool generic_stage, bool takeoff_sweep,
                                  int configured_budget)
{
    if (generic_stage) {
        return takeoff_sweep ? 1 : std::max(1, configured_budget);
    }
    return std::max(1, configured_budget);
}

// Only the initial takeoff reservation is one hop; all continuation passes
// retain the normal bounded five-hop incremental search.
inline int routeSuffixDepthForPass(bool generic_stage, int stage_pass,
                                   int normal_depth)
{
    if (generic_stage && stage_pass == 1) {
        return 1;
    }
    return std::max(1, normal_depth);
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

// Moving computes its complete endpoint closure once per focus. Binding
// audits then use only constant-time membership checks over that closure.
template<typename Binding, typename Contains>
bool routeBindingTouchesKnownEndpoint(const Binding& binding, Contains&& contains)
{
    return (binding.from && contains(binding.from))
        || (binding.to && contains(binding.to));
}

inline double routeStageSecondsRemaining(double elapsed, double budget)
{
    return elapsed < budget ? budget - elapsed : 0.0;
}

// A logical stage has one cumulative budget across all of its re-entries.
inline bool routeStageTimeoutIsFatal(double elapsed, double budget, bool tasks_remain)
{
    return elapsed >= budget && tasks_remain;
}

// An exhausted recoverable stage must reach its existing handoff path even
// when re-entered later. Mandatory Moving sources and terminal Moving
// destinations have no legal nonzero handoff and therefore fail.
inline bool routeStageEntryTimeoutRequiresFailure(bool timeout_reached,
                                                  bool terminal_stage)
{
    return timeout_reached && terminal_stage;
}

// Basic and Fanout may hand unfinished work to their recovery successors;
// Moving sources and Moving destinations must fail on unfinished timeout.
inline bool routeStageTimeoutRequiresFailure(bool timeout_reached, bool can_handoff)
{
    return timeout_reached && !can_handoff;
}

// Stagnation is a terminal routing failure in every stage, not a handoff.
inline bool routeStageStagnationRequiresFailure(bool stagnated, bool /*fanout_stage*/)
{
    return stagnated;
}

inline bool fanoutStageBudgetRequiresHandoff(bool fanout_stage,
                                            bool timed_out, bool stagnated)
{
    return fanout_stage && timed_out && !stagnated;
}

// Suppress all large routing-state files when either the legacy timeout-only
// switch or the general diagnostics switch requests stdout-only operation.
inline bool routeStateDumpEnabled(bool skip_timeout_dump, bool skip_state_dump)
{
    return !skip_timeout_dump && !skip_state_dump;
}

// Every nonterminal Generic stop conserves its unfinished tasks for the later
// recovery stages instead of aborting the complete routing transaction.
inline bool basicStageRequiresHandoff(bool timeout_reached,
                                      bool congestion_growth,
                                      bool routing_blocked)
{
    return timeout_reached || congestion_growth || routing_blocked;
}

// Basic hands only unresolved trunks to Moving sources. Deferred suffixes stay
// parked until every trunk has a complete physical source exit.
template<typename Task>
size_t prepareMovingSourceTasks(std::vector<Task>& trunk_tasks)
{
    for (Task& task : trunk_tasks) {
        task.fanout = false;
    }
    return trunk_tasks.size();
}

// Moving Sources docks its reverse search to an incomplete Generic prefix.
// Complete routes need no recovery and empty routes provide no anchor.
inline bool movingSourceUsesBackwardAnchor(bool has_route,
                                           bool route_complete)
{
    return has_route && !route_complete;
}

// An incomplete prefix is released only after reverse routing has failed to
// dock to every retained anchor; a successful dock preserves it.
inline bool movingSourceReleasesPrefixAfterDockMiss(bool has_anchor,
                                                     bool docked)
{
    return has_anchor && !docked;
}

// Moving sources is a mandatory barrier: neither a focused relocation nor a
// deferred trunk may survive when the scheduler releases Fanout work.
inline bool movingSourcesReachedZero(size_t active_trunks,
                                     size_t deferred_trunks,
                                     bool has_focus)
{
    return active_trunks == 0 && deferred_trunks == 0 && !has_focus;
}

// Fanout can start only after the source-moving barrier and after validating
// that every deferred suffix still has a completed source trunk.
inline bool fanoutMayStartAfterMovingSources(bool source_stage_complete,
                                             size_t suffixes_without_trunk)
{
    return source_stage_complete && suffixes_without_trunk == 0;
}

// Both moving stages begin with relocation. Moving Sources must run its
// destination-to-source route probe before another generic trunk retry.
inline bool movingStageStartsWithRelocation(bool moving_sources,
                                            bool has_tasks)
{
    (void)moving_sources;
    return has_tasks;
}

// Both moving stages finish one focused endpoint before selecting the next.
inline bool movingRelocatesImmediatelyAfterFocus(bool moving_sources)
{
    (void)moving_sources;
    return true;
}

// Source relocation uses small route-first batches so Generic routing can
// consume newly freed capacity before another large group is displaced.
inline size_t movingSourceRelocationBatchLimit(int move_attempt_limit,
                                               size_t pending_sources)
{
    size_t design_quantum = static_cast<size_t>(std::max(1, move_attempt_limit));
    size_t batch_quantum =
        std::max<size_t>(8, std::min<size_t>(64, design_quantum / 80));
    return std::min(pending_sources, batch_quantum);
}

// Sparse late-stage queues may require many rejected probes per accepted move.
// Bound attempts as well as successes so routing passes retain stage time.
inline size_t movingSourceRelocationAttemptLimit(size_t relocation_limit,
                                                 size_t pending_sources)
{
    return std::min(pending_sources, relocation_limit);
}

// Basic handoff removes stale prefixes once; incremental Moving Sources
// recovery must retain every newly committed prefix across later passes.
inline bool movingSourceRecoveryRetainsPrefix(bool has_partial_prefix)
{
    return has_partial_prefix;
}

// Route-first source placement may use any tile reached by the bounded reverse
// frontier; candidate buckets still examine positions nearest the old source first.
inline int movingSourcePlacementRadius()
{
    return -1;
}

// Moving Sources must prove that no route-guided placement works before its
// mandatory stage can fail. Zero requests exhaustive probing until deadline.
inline size_t movingSourceProbeLimit()
{
    return 0;
}

// Generic recovery is amortized over meaningful topology changes instead of
// rescanning the complete unfinished queue after one rejected relocation batch.
inline bool movingSourceGenericRecoveryDue(size_t released_prefixes,
                                           size_t source_moves,
                                           size_t changed_routes,
                                           size_t pending_tasks = 0)
{
    // Preempted work must reach the normal queue merge even when fewer than
    // one batch of sources remains. The Generic pass is still chunk-bounded.
    return pending_tasks != 0 || released_prefixes >= 256 || source_moves >= 32 ||
           changed_routes >= 1024;
}

// Relocation may finish the last trunk without a routing pass. Fall through
// once in that case so stage handoff releases deferred fanouts before exit.
inline bool movingRelocationCanContinue(bool requested, bool active_work,
                                        bool deferred_work, bool pending_work)
{
    return requested && !pending_work && (active_work || deferred_work);
}

// Prefixes rejected immediately after their producing recovery chunk are
// cleanup from that chunk; only an initial independent sweep exposes new work.
inline size_t movingSourceRecoveryReleaseCount(size_t released_prefixes,
                                               bool follows_recovery)
{
    return follows_recovery ? 0 : released_prefixes;
}

// Process a rotating Generic recovery chunk so large Moving Sources worksets
// return promptly to reverse-anchor and route-guided placement probes.
inline size_t movingSourceGenericRecoveryTaskLimit(size_t pending_tasks)
{
    return std::min<size_t>(pending_tasks, 4096);
}

// A moved source's same-port routes share the newly committed physical trunk;
// only routes from another output port can become new Generic trunk work.
inline bool movedSourceRouteBecomesFanout(bool same_source_port,
                                          bool source_has_complete_exit)
{
    return same_source_port || source_has_complete_exit;
}

// Moving Sources rebuilds every unfinished trunk from its destination. A
// committed partial prefix is not completion and must not suppress relocation.
inline bool movingSourceRouteGuidedTaskNeedsRelocation(bool route_complete)
{
    return !route_complete;
}

// Moving Sources is the mandatory trunk-recovery stage. Once its complete
// route-first relocation attempt fails, later sources cannot repair that task.
inline bool movingSourceFailureRequiresExit(bool moving_sources_stage,
                                            bool move_succeeded,
                                            bool candidate_retryable = false)
{
    return moving_sources_stage && !move_succeeded && !candidate_retryable;
}

// Consume a precisely released reverse boundary before unrelated work can
// claim it; the bound prevents a chain of transit cuts monopolizing a batch.
inline bool movingSourceRetriesReleasedBoundary(bool boundary_released,
                                                size_t retries,
                                                size_t retry_limit = 2)
{
    return boundary_released && retries < retry_limit;
}

// A tile-local void connection is implemented by the packed element chain and
// must not consume a fabric input terminal during source-placement preflight.
inline bool movingSourceInputNeedsFabricTerminal(bool has_driver,
                                                 bool is_void_net)
{
    return has_driver && !is_void_net;
}

// An input owns its retained prefix even before it reaches the sink. Sibling
// anchors still need complete routes, and moving a driver invalidates both.
inline bool movingInputMayReuseRoute(bool own_input, bool complete,
                                    bool source_moves, bool sink_moves)
{
    return (own_input || complete) && !source_moves && (!sink_moves || own_input);
}

// Generated distributed endpoints implement a constant value, not the
// logical constant's cell identity. Ordinary drivers retain exact identity.
inline bool movingSourceInputOwnerMatches(bool constant_input, bool input_one,
                                          bool distributed_owner, bool owner_one,
                                          bool same_physical_source)
{
    return constant_input ? distributed_owner && input_one == owner_one
                          : !distributed_owner && same_physical_source;
}

// A live pin lease may be reused only when a routed binding from the same
// physical driver owns it; placement reservation metadata alone is not enough.
inline bool movingSourceInputTerminalAvailable(bool pin_leased,
                                               bool candidate_reserved,
                                               bool same_driver,
                                               bool live_same_driver_owner)
{
    return !candidate_reserved &&
        (!pin_leased || (same_driver && live_same_driver_owner));
}

// Bounded reverse placement probing is incomplete until every candidate after
// the current offset has been examined; the scheduler must resume that window.
inline bool movingSourceProbeWindowHasRemaining(size_t candidate_count,
                                                size_t offset,
                                                size_t scanned)
{
    return offset < candidate_count &&
        scanned < candidate_count - offset;
}

// The original placement occupies one history entry but is not a failed
// alternative and therefore must not expand the first relocation radius.
inline size_t movingTriedAlternativeCount(size_t tried_placements)
{
    return tried_placements == 0 ? 0 : tried_placements - 1;
}

// A source placement needs batch relocation only when it owns no committed
// takeoff and its current output local exposes no free concrete takeoff.
inline bool movingSourceNeedsRelocation(bool has_committed_takeoff,
                                        bool has_free_takeoff)
{
    return !has_committed_takeoff && !has_free_takeoff;
}

// Park an exhausted source focus for the next fair scheduler cycle. Current
// deferred sources remain ahead of it and therefore receive one slice first.
template<typename Task>
void deferMovingSourceRetry(std::vector<Task>& retry_tasks,
                            std::vector<Task>& active_tasks)
{
    retry_tasks.insert(retry_tasks.end(),
                       std::make_move_iterator(active_tasks.begin()),
                       std::make_move_iterator(active_tasks.end()));
    active_tasks.clear();
}

// Begin another source cycle only after the current deferred source pool has
// been consumed, preventing a blocked focus from starving unrelated sources.
template<typename Task>
bool activateMovingSourceRetryCycle(std::vector<Task>& deferred_tasks,
                                    std::vector<Task>& retry_tasks)
{
    if (!deferred_tasks.empty() || retry_tasks.empty()) {
        return false;
    }
    deferred_tasks.swap(retry_tasks);
    return true;
}

// Source recovery operates on the complete trunk workset. Destination-style
// one-focus activation would turn a fair retry cycle into thousands of passes.
template<typename Task>
bool activateMovingSourceWorkset(std::vector<Task>& active_tasks,
                                      std::vector<Task>& deferred_tasks)
{
    if (!active_tasks.empty() || deferred_tasks.empty()) {
        return false;
    }
    active_tasks.swap(deferred_tasks);
    return true;
}

// A bounded route-first batch must probe new sources before retrying rejected
// ones; append rejected tasks behind all untouched/rebuilt work without loss.
template<typename Task>
void rotateRejectedMovingSources(std::vector<Task>& work,
                                 std::vector<Task>& rejected)
{
    work.insert(work.end(), std::make_move_iterator(rejected.begin()),
                std::make_move_iterator(rejected.end()));
    rejected.clear();
}

// Widen route-first reverse search only after both its numeric frontier and
// placement window are exhausted; every retry remains independently bounded.
inline size_t nextMovingSourceExpansionBudget(size_t current,
                                              bool frontier_exhausted,
                                              bool probes_exhausted,
                                              size_t maximum)
{
    if (!frontier_exhausted || !probes_exhausted || current >= maximum) {
        return current;
    }
    return std::min(maximum, current * 2);
}

// Starting a focused move replaces incident routes, but every unrelated task
// from both the old deferred queue and active queue must survive the rebuild.
template<typename Task, typename IsIncident>
void retainNonFocusMovingTasks(std::vector<Task>& deferred_tasks,
                               const std::vector<Task>& active_tasks,
                               IsIncident&& is_incident)
{
    std::vector<Task> retained;
    retained.reserve(deferred_tasks.size() + active_tasks.size());
    for (Task& task : deferred_tasks) {
        if (!is_incident(task)) {
            retained.push_back(std::move(task));
        }
    }
    for (const Task& task : active_tasks) {
        if (!is_incident(task)) {
            retained.push_back(task);
        }
    }
    deferred_tasks = std::move(retained);
}

// A persistent Moving deferred pool is compacted in place when a focus starts.
// Remove stale copies of that focus, then append only unrelated active work.
template<typename Task, typename IsIncident>
size_t appendNonFocusMovingTasks(std::vector<Task>& deferred_tasks,
                                 const std::vector<Task>& active_tasks,
                                 IsIncident&& is_incident)
{
    size_t removed = std::erase_if(deferred_tasks, is_incident);
    for (const Task& task : active_tasks) {
        if (!is_incident(task)) {
            deferred_tasks.push_back(task);
        }
    }
    return removed;
}

struct MovingStageQueueCompaction
{
    size_t completed = 0;
    size_t duplicates = 0;
};

// Consolidate the queues once when Moving begins. Completed stale entries are
// discarded and duplicate scheduler state is merged in expected linear time.
template<typename Task, typename IsComplete, typename Hash, typename Same,
         typename Merge>
MovingStageQueueCompaction compactMovingStageQueues(
    std::vector<Task>& active_tasks, std::vector<Task>& deferred_tasks,
    IsComplete&& is_complete, Hash&& hash, Same&& same, Merge&& merge)
{
    std::vector<Task> pending;
    pending.reserve(active_tasks.size() + deferred_tasks.size());
    pending.insert(pending.end(),
                   std::make_move_iterator(active_tasks.begin()),
                   std::make_move_iterator(active_tasks.end()));
    pending.insert(pending.end(),
                   std::make_move_iterator(deferred_tasks.begin()),
                   std::make_move_iterator(deferred_tasks.end()));
    active_tasks.clear();
    deferred_tasks.clear();

    std::vector<Task> unique;
    unique.reserve(pending.size());
    std::unordered_map<size_t, std::vector<size_t>> buckets;
    buckets.reserve(pending.size());
    MovingStageQueueCompaction result;
    for (Task& task : pending) {
        if (is_complete(task)) {
            ++result.completed;
            continue;
        }
        std::vector<size_t>& candidates = buckets[hash(task)];
        auto existing = std::find_if(
            candidates.begin(), candidates.end(), [&](size_t index) {
                return same(unique[index], task);
            });
        if (existing != candidates.end()) {
            merge(unique[*existing], task);
            ++result.duplicates;
            continue;
        }
        candidates.push_back(unique.size());
        unique.push_back(std::move(task));
    }
    active_tasks = std::move(unique);
    return result;
}

// Source-tree invalidation may return both routes incident to the moved cell
// and displaced sibling branches. Keep only incident work in the atomic focus.
template<typename Task, typename IsIncident, typename Defer>
size_t partitionMovingReplacementTasks(std::vector<Task>& replacement_tasks,
                                       IsIncident&& is_incident,
                                       Defer&& defer)
{
    std::vector<Task> incident_tasks;
    incident_tasks.reserve(replacement_tasks.size());
    size_t deferred = 0;
    for (Task& task : replacement_tasks) {
        if (is_incident(task)) {
            incident_tasks.push_back(std::move(task));
        } else {
            defer(task);
            ++deferred;
        }
    }
    replacement_tasks = std::move(incident_tasks);
    return deferred;
}

// A deferred scan that found only cooling-down endpoints must advance the
// relocation epoch and retry; otherwise Moving emits empty passes forever.
inline bool movingDeferredScanNeedsRetry(size_t remaining_tasks,
                                         size_t cooldown_tasks)
{
    return remaining_tasks != 0 && cooldown_tasks != 0;
}

// Moving receives queues accumulated by earlier stages. Remove bindings that
// became complete before relocation so the persistent pool represents work.
template<typename Task, typename IsComplete>
size_t removeCompletedMovingTasks(std::vector<Task>& tasks,
                                  IsComplete&& is_complete)
{
    size_t before = tasks.size();
    std::erase_if(tasks, is_complete);
    return before - tasks.size();
}

// Fanout timeout hands both its active residue and pass-deferred branches to
// Moving; none may remain parked in the inactive Fanout queue.
template<typename Task, typename Append>
size_t deferFanoutTimeoutTasks(std::vector<Task>& fanout_tasks,
                               std::vector<Task>& moving_tasks,
                               Append&& append)
{
    size_t deferred = fanout_tasks.size();
    for (Task& task : fanout_tasks) {
        append(moving_tasks, task);
    }
    fanout_tasks.clear();
    return deferred;
}

// Keep one Generic seed per physical source port and defer its siblings.
template<typename Task, typename SourceKeyFn, typename AppendFn>
void scheduleOneSeedPerSourcePort(std::vector<Task>& tasks,
                                  std::vector<Task>& generic_tasks,
                                  std::vector<Task>& fanout_tasks,
                                  SourceKeyFn&& source_key,
                                  AppendFn&& append_task)
{
    std::unordered_set<std::string> sources;
    for (const Task& task : generic_tasks) {
        sources.insert(source_key(task));
    }
    for (Task& task : tasks) {
        if (sources.insert(source_key(task)).second) {
            task.fanout = false;
            append_task(generic_tasks, task);
        } else {
            task.fanout = true;
            append_task(fanout_tasks, task);
        }
    }
}

// Choose the nearest placed sink as the one Generic trunk for each physical
// source. All other sinks remain Fanout tasks; no route-search ordering changes.
template<typename Task, typename SourceKeyFn, typename DistanceFn>
size_t selectNearestGenericSeeds(std::vector<Task>& generic_tasks,
                                 std::vector<Task>& fanout_tasks,
                                 SourceKeyFn&& source_key,
                                 DistanceFn&& distance)
{
    std::unordered_map<std::string, size_t> generic_by_source;
    generic_by_source.reserve(generic_tasks.size());
    for (size_t index = 0; index < generic_tasks.size(); ++index) {
        generic_by_source.emplace(source_key(generic_tasks[index]), index);
    }
    size_t replacements = 0;
    for (Task& fanout : fanout_tasks) {
        auto generic = generic_by_source.find(source_key(fanout));
        if (generic == generic_by_source.end()) {
            continue;
        }
        Task& seed = generic_tasks[generic->second];
        if (distance(fanout) >= distance(seed)) {
            continue;
        }
        std::swap(seed, fanout);
        seed.fanout = false;
        fanout.fanout = true;
        ++replacements;
    }
    return replacements;
}

// Generic needs one completed route per physical source, not one particular
// sink. Rotate a repeatedly failed seed to another deferred sibling.
inline bool shouldRotateFailedGenericSeed(bool generic_mode, bool complete,
                                          bool progress, bool route_empty)
{
    return generic_mode && !complete && !progress && route_empty;
}

template<typename Task, typename SameTask, typename AppendFailed, typename DistanceFn>
bool rotateFailedGenericSeedTaskNearest(Task& task,
                                        std::vector<Task>& primary_deferred,
                                        std::vector<Task>& secondary_deferred,
                                        SameTask same_task,
                                        AppendFailed append_failed,
                                        DistanceFn distance,
                                        size_t attempt_limit = 2)
{
    if (task.fanout || !task.from || task.attempt < attempt_limit) {
        return false;
    }
    std::vector<Task>* best_queue = nullptr;
    size_t best_index = 0;
    int best_distance = std::numeric_limits<int>::max();
    auto consider = [&](std::vector<Task>& queue) {
        for (size_t index = 0; index < queue.size(); ++index) {
            Task& candidate = queue[index];
            if (candidate.from != task.from
                || candidate.from_port != task.from_port
                || same_task(candidate, task)) {
                continue;
            }
            int candidate_distance = distance(candidate);
            if (!best_queue || candidate_distance < best_distance) {
                best_queue = &queue;
                best_index = index;
                best_distance = candidate_distance;
            }
        }
    };
    consider(primary_deferred);
    consider(secondary_deferred);
    if (!best_queue) {
        return false;
    }
    Task failed = task;
    failed.fanout = true;
    failed.attempt = 0;
    Task replacement = std::move((*best_queue)[best_index]);
    best_queue->erase(best_queue->begin() + static_cast<std::ptrdiff_t>(best_index));
    replacement.fanout = false;
    replacement.attempt = 0;
    append_failed(std::move(failed));
    task = std::move(replacement);
    return true;
}

template<typename Task, typename SameTask, typename AppendFailed>
bool rotateFailedGenericSeedTask(Task& task, std::vector<Task>& primary_deferred,
                                 std::vector<Task>& secondary_deferred,
                                 SameTask same_task, AppendFailed append_failed,
                                 size_t attempt_limit = 2)
{
    return rotateFailedGenericSeedTaskNearest(
        task, primary_deferred, secondary_deferred, same_task, append_failed,
        [](const Task&) { return 0; }, attempt_limit);
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

// Constant tasks and ordinary tasks occupy disjoint scheduler modes. This is
// the stage barrier that prevents distributed roots becoming trunks/fanouts.
inline bool constantTaskModeIsValid(bool constant_mode,
                                    bool distributed_source)
{
    return constant_mode == distributed_source;
}

// Moving preserves ordinary protected trees for their dedicated owner. A
// distributed constant branch is instead rebuilt synchronously in Const mode.
inline bool movementDefersProtectedTree(bool route_protected,
                                        bool distributed_source)
{
    return route_protected && !distributed_source;
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

// Keep the normal incremental search depth inside the docking window. The
// newest accepted in-window node is selected as the docking anchor afterward.
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

// Every routing stage retains a nearby rail before ordinary expansion because
// later steps in the same bounded suffix may leave the docking window.
inline bool rememberDockingCandidate(int depth, int distance,
                                     int docking_radius)
{
    return depth > 0 && distance <= docking_radius;
}

// Takeoff may preempt a remembered transit victim only after every free exit failed.
inline bool shouldPreemptTakeoff(bool accepted_free_path, bool takeoff)
{
    return !accepted_free_path && takeoff;
}

// A complete docking bridge wins immediately. An incomplete reverse boundary
// may be cut only after the bounded search found no free suffix to commit.
inline bool dockingBoundaryMayPreempt(bool joins_frontiers,
                                      bool has_free_partial)
{
    return joins_frontiers || !has_free_partial;
}

inline bool transitPreemptionStepAllowed(bool preemption_enabled,
                                         bool protected_route,
                                         bool first_committed_step)
{
    return preemption_enabled && (first_committed_step || protected_route);
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

struct GroundingTerminalPath
{
    int dst = -1;
    int joint = -1;
    int joint2 = -1;
};

// A logical sink assignment protects its terminal resources only after the
// route actually reaches that sink; an unfinished prefix remains preemptible.
inline bool groundingOwnerProtectsEndpoint(bool binding_targets_tile,
                                            bool route_complete)
{
    return binding_targets_tile && route_complete;
}

// Grounding may preempt only the exact terminal path proven reachable by
// docking; sharing its destination node is not sufficient.
inline bool sameGroundingTerminalPath(const GroundingTerminalPath& lhs,
                                      const GroundingTerminalPath& rhs)
{
    return lhs.dst == rhs.dst && lhs.joint == rhs.joint &&
        lhs.joint2 == rhs.joint2;
}

// Select an exact blocked terminal path only when no physically incoming path
// to the requested local has every required DST and joint resource free.
template<typename IsPreemptible>
GroundingTerminalPath groundingPreemptionPath(
    fpga::CBType& type, const fpga::CBState& state, int local,
    NodeMask incoming_dsts, NodeMask unavailable_joints,
    IsPreemptible&& is_preemptible)
{
    GroundingTerminalPath none;
    if (local < 0 || local >= CB_MAX_NODES) {
        return none;
    }

    const std::vector<fpga::CBType::TerminalEntry>& entries =
        type.terminalEntries(local);
    auto leased = [&](const fpga::CBType::TerminalEntry& entry) {
        return state.dst.jump.testBit(entry.dst) ||
            (entry.joint >= 0 && (state.joint.jump.testBit(entry.joint) ||
                                  unavailable_joints.testBit(entry.joint))) ||
            (entry.joint2 >= 0 && (state.joint.jump.testBit(entry.joint2) ||
                                   unavailable_joints.testBit(entry.joint2)));
    };
    for (const fpga::CBType::TerminalEntry& entry : entries) {
        if (incoming_dsts.testBit(entry.dst) && !leased(entry)) {
            return none;
        }
    }

    for (const fpga::CBType::TerminalEntry& entry : entries) {
        if (!incoming_dsts.testBit(entry.dst) || !leased(entry)) {
            continue;
        }
        GroundingTerminalPath path{entry.dst, entry.joint, entry.joint2};
        if (is_preemptible(path)) {
            return path;
        }
    }
    return none;
}

template<typename IsPreemptible>
GroundingTerminalPath groundingPreemptionPath(
    fpga::CBType& type, const fpga::CBState& state, int local,
    NodeMask incoming_dsts, IsPreemptible&& is_preemptible)
{
    return groundingPreemptionPath(type, state, local, incoming_dsts,
        NodeMask{}, std::forward<IsPreemptible>(is_preemptible));
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

// A failed Moving fanout has exhausted its current physical branch choices
// only when the fanout attempt cursor advanced without routing progress.
inline bool movingFanoutNeedsSourceTreeRebuild(
    bool moving_mode, bool focused_move, bool rebuild_already_attempted,
    bool task_is_fanout, bool has_complete_source_exit, bool task_complete,
    bool task_progress, size_t attempt_before, size_t attempt_after)
{
    return moving_mode && focused_move && !rebuild_already_attempted
        && task_is_fanout && has_complete_source_exit && !task_complete
        && !task_progress && attempt_after > attempt_before;
}

// Rebuild an exhausted source tree toward the currently moved sink. The
// current binding becomes its Generic seed while every sibling remains Fanout.
template<typename Task, typename Same, typename Enqueue>
size_t scheduleMovingSourceTreeRebuild(Task& current,
                                       std::vector<Task>& source_tasks,
                                       Same&& same, Enqueue&& enqueue)
{
    size_t queued = 0;
    for (Task& sibling : source_tasks) {
        if (same(current, sibling)) {
            continue;
        }
        sibling.fanout = true;
        sibling.source_tree_rebuild_attempted = true;
        sibling.attempt = 0;
        sibling.fanout_branch_attempt = 0;
        sibling.fanout_branch_offset = 0;
        sibling.no_progress_passes = 0;
        if (enqueue(sibling)) {
            ++queued;
        }
    }
    current.fanout = false;
    current.source_tree_rebuild_attempted = true;
    current.attempt = 0;
    current.fanout_branch_attempt = 0;
    current.fanout_branch_offset = 0;
    current.no_progress_passes = 0;
    return queued;
}

// Repeated focused source-tree repairs may rediscover an external sibling.
// Keep one deferred task and merge its live retry state in place.
template<typename Task, typename Same, typename Merge>
bool mergeMovingDeferredTask(std::vector<Task>& deferred, const Task& task,
                             Same&& same, Merge&& merge)
{
    for (Task& old : deferred) {
        if (!same(old, task)) {
            continue;
        }
        merge(old, task);
        return false;
    }
    deferred.push_back(task);
    return true;
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

// Relocation rebuilds tasks from route bindings, but a previously invalidated
// incident suffix may temporarily exist only in the active scheduler queue.
template<typename Task, typename IsIncident, typename SameTask, typename MergeState>
size_t preserveActiveIncidentTasks(std::vector<Task>& replacement_tasks,
                                   const std::vector<Task>& active_tasks,
                                   IsIncident&& is_incident,
                                   SameTask&& same_task,
                                   MergeState&& merge_state)
{
    size_t preserved = 0;
    for (const Task& active : active_tasks) {
        if (!is_incident(active)) {
            continue;
        }
        auto replacement = std::find_if(
            replacement_tasks.begin(), replacement_tasks.end(),
            [&](const Task& candidate) { return same_task(active, candidate); });
        if (replacement == replacement_tasks.end()) {
            replacement_tasks.push_back(active);
            ++preserved;
        }
        else {
            // Binding reconstruction restores endpoint identity; the active
            // task remains authoritative for its bounded-search retry cursor.
            merge_state(*replacement, active);
        }
    }
    return preserved;
}

// Relocation preserves route-choice cursors but starts a fresh no-progress
// window because the rebuilt task now targets a different physical placement.
template<typename Task, typename MergeState>
void mergeRelocatedMovingTaskState(Task& replacement, const Task& active,
                                   MergeState&& merge_state)
{
    merge_state(replacement, active);
    replacement.no_progress_passes = 0;
}

// Compatibility overload for task types that carry no scheduler state.
template<typename Task, typename IsIncident, typename SameTask>
size_t preserveActiveIncidentTasks(std::vector<Task>& replacement_tasks,
                                   const std::vector<Task>& active_tasks,
                                   IsIncident&& is_incident,
                                   SameTask&& same_task)
{
    return preserveActiveIncidentTasks(
        replacement_tasks, active_tasks, std::forward<IsIncident>(is_incident),
        std::forward<SameTask>(same_task), [](Task&, const Task&) {});
}

// Reorder Generic tasks with stable linear buckets. The initial takeoff sweep
// uses source-first order; later passes protect already-committed prefixes.
template<typename Task, typename RouteClass>
std::array<size_t, 3> prioritizeGenericRouteTasks(std::vector<Task>& tasks,
                                                  RouteClass&& route_class,
                                                  bool prefixes_first = false)
{
    std::array<std::vector<Task>, 3> buckets;
    std::array<size_t, 3> counts{};
    for (std::vector<Task>& bucket : buckets) {
        bucket.reserve(tasks.size());
    }
    for (Task& task : tasks) {
        size_t index = std::min<size_t>(2, route_class(task));
        ++counts[index];
        buckets[index].push_back(std::move(task));
    }
    tasks.clear();
    tasks.reserve(counts[0] + counts[1] + counts[2]);
    constexpr std::array<size_t, 3> source_first{0, 1, 2};
    constexpr std::array<size_t, 3> progress_first{2, 1, 0};
    const std::array<size_t, 3>& order =
        prefixes_first ? progress_first : source_first;
    for (size_t index : order) {
        std::vector<Task>& bucket = buckets[index];
        tasks.insert(tasks.end(), std::make_move_iterator(bucket.begin()),
                     std::make_move_iterator(bucket.end()));
    }
    return counts;
}

// Candidate iteration already encodes angle and wire-length priority. Keep
// the first candidate in each completion class, preferring a partial victim
// so one takeoff does not destroy completed work unnecessarily.
inline bool selectPreemptionCandidate(bool already_selected,
                                      bool selected_complete,
                                      bool candidate_complete)
{
    return !already_selected || (selected_complete && !candidate_complete);
}

// One bridge claim completes one route task. It may exchange that task with
// one completed victim, but must not create two or more unfinished tasks.
inline bool bridgePreemptionConservesTasks(size_t complete_victims)
{
    return complete_victims <= 1;
}

// Inspect partial victims first, then allow one completed foreign transit
// victim in every stage. The caller separately protects the current source
// tree, endpoint owners, shared bridges, and reciprocal preemption cycles.
inline bool bridgePreemptionPhaseAccepts(bool fanout_stage, bool moving_stage,
                                         bool allow_complete_victim,
                                         size_t complete_victims)
{
    (void)fanout_stage;
    (void)moving_stage;
    return complete_victims == 0 ||
           (complete_victims == 1 && allow_complete_victim);
}

// Mandatory source recovery may exchange its blocked reverse boundary with
// one completed transit route; other Moving modes retain completed work.
inline bool movingSourceBoundaryMayExchangeComplete(bool moving_sources_stage)
{
    return moving_sources_stage;
}

// A bridge cut must be private to one route binding. Cutting a shared transit
// node invalidates many partial fanouts and loses more work than one claim adds.
inline bool bridgePreemptionHasSingleOwner(size_t victims)
{
    return victims == 1;
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

// Source-tree reset state is relevant only while Moving rebuilds an affected
// hierarchy. Generic and Fanout batches must not scan every source binding.
inline bool routeBatchNeedsSourceTreeResetState(bool moving_mode)
{
    return moving_mode;
}

// A Fanout pass is still productive when an existing branch advances or changes.
inline bool fanoutPassMadeProgress(size_t completed, size_t advanced, size_t changed)
{
    return completed != 0 || advanced != 0 || changed != 0;
}

// Consuming a failed Fanout branch advances both branch rotation and source-tree
// selection, then resets the retry count for the next branch candidate.
inline void consumeFanoutBranch(size_t& source_attempt, size_t& branch_offset,
                                size_t& branch_attempt)
{
    ++source_attempt;
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

enum class BlockedFanoutAction
{
    retry,
    backstep,
    rotate
};

// A blocked private suffix must retry from its committed parent. If no private
// parent remains, rotate to another branch point instead of probing it again.
inline BlockedFanoutAction blockedFanoutAction(bool root_blocked,
                                               size_t private_crossbars)
{
    if (!root_blocked) {
        return BlockedFanoutAction::retry;
    }
    return private_crossbars > 1 ? BlockedFanoutAction::backstep
                                : BlockedFanoutAction::rotate;
}

// Start each Fanout task from any usable Generic-trunk fork. Routed siblings
// are considered only after that fork fails or when the trunk has none.
inline bool fanoutShouldInspectSiblingTrees(bool trunk_has_preferred_branch,
                                            bool trunk_has_fallback_branch,
                                            size_t prior_failed_attempts)
{
    return (!trunk_has_preferred_branch && !trunk_has_fallback_branch)
        || prior_failed_attempts != 0;
}

// Each retry selects one complete sibling tree. Wrap over the currently
// complete trees because additional siblings may finish after earlier retries.
inline size_t fanoutSiblingBindingOrdinal(size_t prior_failed_attempts,
                                          size_t complete_binding_count)
{
    if (complete_binding_count <= 1) {
        return 0;
    }
    return 1 + (prior_failed_attempts % (complete_binding_count - 1));
}

// Fanout-demoted Generic seeds get a bounded repair window. A seed still
// blocked after two recursion windows is conserved for Moving with its tree.
inline bool fanoutSeedRepairPassesExhausted(bool repair_active, int stage_pass,
                                            int recursion_limit)
{
    return repair_active
        && stage_pass >= std::max(2, recursion_limit * 2);
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

// A known endpoint whose every numeric terminal path is occupied cannot seed
// reverse trunk routing; relocating that sink is the only useful next action.
inline bool movingSourceNeedsTerminalLegalization(
    const std::vector<MovingTerminalPath>& paths, const NodeMask& leased_pins,
    const NodeMask& leased_locals, const NodeMask& leased_dsts,
    const NodeMask& leased_joints)
{
    if (paths.empty()) {
        return false;
    }
    NodeMask pins = leased_pins;
    NodeMask locals = leased_locals;
    NodeMask dsts = leased_dsts;
    NodeMask joints = leased_joints;
    return !reserveMovingTerminalPath(paths, pins, locals, dsts, joints);
}

// A free terminal DST is still unusable when reverse routing cannot cross its
// immediate ingress boundary. Deeper congestion must not relocate the sink.
inline bool movingSourceNeedsIngressLegalization(
    bool has_terminal_paths, int diagnostic_depth,
    size_t predecessor_paths, size_t free_predecessor_paths)
{
    return has_terminal_paths && diagnostic_depth == 0 &&
        predecessor_paths != 0 && free_predecessor_paths == 0;
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
inline bool movingFocusHandsOffToLoads(bool moving_sources,
                                       bool has_route_into_focus,
                                       bool has_route_out_of_focus)
{
    return !moving_sources && !has_route_into_focus && has_route_out_of_focus;
}

// A distributed source can reach the moved sink independently of placement.
// Only an ordinary incident completion proves this placement worth retaining.
inline bool movingCompletionRenewsPlacement(bool route_completed,
                                             bool distributed_source)
{
    return route_completed && !distributed_source;
}

// Moving normally relocates a route's load. A physically fixed load has no
// legal placement candidate, so its movable source is the only useful focus.
inline bool movingUsesSourceForFixedSink(bool sink_is_fixed,
                                         bool source_is_movable)
{
    return sink_is_fixed && source_is_movable;
}

// Completing an incident route proves the current placement useful and starts
// a fresh bounded slice. Partial-prefix growth alone cannot extend the slice.
inline int updateMovingPlacementNoProgressPasses(int no_progress_passes,
                                                 bool route_completed)
{
    return route_completed ? 0 : no_progress_passes + 1;
}

// A completed sibling changes the focused incident set. Give every remaining
// sibling a fresh bounded retry window before considering another relocation.
template<typename Task>
void renewMovingTaskWindowsAfterCompletion(std::vector<Task>& tasks,
                                           bool route_completed)
{
    if (!route_completed) {
        return;
    }
    for (Task& task : tasks) {
        task.no_progress_passes = 0;
    }
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

// Every focused pass attempts every remaining incident route. A completed
// sibling renews one ordinary retry window; task count must not multiply it.
inline int movingUsefulPlacementRetryLimit(int recursion_limit,
                                           size_t /*remaining_routes*/)
{
    return std::max(1, recursion_limit);
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

// An inactive pass proves the placement blocked only after its bounded task or
// placement retry window is exhausted; one failed suffix must try alternatives.
inline bool focusedMovingPassIsBlocked(bool no_active_work,
                                       bool retry_window_exhausted)
{
    return no_active_work && retry_window_exhausted;
}

// The global no-progress invariant must allow an active focus to consume its
// bounded retry window; outside that case a zero-work pass remains an error.
inline bool focusedMovingMayRetryInactivePass(bool has_focus,
                                               bool retry_window_exhausted)
{
    return has_focus && !retry_window_exhausted;
}

// A bounded focused retry may finish by scheduling a new placement rather
// than advancing a wire; that pending relocation is valid scheduler progress.
inline bool movingRelocationSatisfiesProgress(bool moving_stage,
                                              bool relocation_pending)
{
    return moving_stage && relocation_pending;
}

// Resolve generated source adapters to the physical element whose placement
// owns them; a malformed ownership cycle leaves the last stable endpoint.
template <typename Endpoint, typename ResolveOwner>
Endpoint* movingSourcePlacementTarget(Endpoint* source,
                                      ResolveOwner&& resolve_owner)
{
    Endpoint* current = source;
    for (int depth = 0; current && depth < 16; ++depth) {
        Endpoint* owner = resolve_owner(current);
        if (!owner || owner == current) {
            return current;
        }
        current = owner;
    }
  return current;
}

// A route-first placement is usable only when its preview assigns every
// member of the physical source cluster, including generated route endpoints.
template <typename Member, typename Choice, typename ChoiceMember>
bool routeFirstClusterPlacementComplete(const std::vector<Member>& cluster,
                                        const std::vector<Choice>& choices,
                                        ChoiceMember&& choice_member)
{
    return std::all_of(cluster.begin(), cluster.end(), [&](Member member) {
        return member != Member{} &&
            std::any_of(choices.begin(), choices.end(), [&](const Choice& choice) {
                return choice_member(choice) == member;
            });
    });
}

// A restored Moving queue selects its next endpoint immediately; the previous
// focus already completed its isolated Generic/Fanout recovery.
// A bounded initial sweep also selects a blocked endpoint even when a few
// unrelated tasks complete and make the large global queue slightly smaller.
inline bool movingStageShouldRelocate(bool restored_focus, bool has_focus,
                                      size_t remaining, size_t before,
                                      bool should_move_unfocused,
                                      bool should_move_focus)
{
    if (remaining == 0) {
        return false;
    }
    if (restored_focus) {
        return true;
    }
    if (!has_focus) {
        return should_move_unfocused;
    }
    return remaining >= before && should_move_focus;
}

// Moving validates the selected deferred task lazily; a completed route is not
// a relocation candidate and must be skipped without auditing the whole queue.
inline bool movingTaskNeedsRelocation(bool route_complete)
{
    return !route_complete;
}

// A persistent deferred pool is executable Moving work even when the focused
// active queue has just been cleared between cells.
inline bool routeSchedulerHasWork(bool active_work, bool moving_stage,
                                  bool deferred_moving_work)
{
    return active_work || (moving_stage && deferred_moving_work);
}

// An empty active Moving queue with deferred work is a scheduler handoff, not
// an empty routing pass. Wake relocation immediately when no focus owns work.
inline bool movingDeferredWorkNeedsRelocation(bool moving_stage,
                                              bool has_focus,
                                              bool active_work,
                                              bool deferred_moving_work)
{
    return moving_stage && !has_focus && !active_work && deferred_moving_work;
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

// Generic trunk recovery and an active destination focus may preempt transit
// routes. Only unfocused destination repair must preserve unrelated trees.
inline bool canPreemptDuringFocusedMove(bool moving_stage,
                                        bool has_moving_focus,
                                        bool moving_sources = false)
{
    return !moving_stage || moving_sources || has_moving_focus;
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

// Moving owns one sink atomically for a bounded placement slice. Yielding keeps
// its complete incident task set and placement history intact for the next visit.
inline constexpr size_t MOVING_FOCUS_SLICE_LIMIT = 8;

// Focus slicing retains candidate history across scheduler visits. A zero
// limit remains available to explicitly request an unbounded atomic focus.
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

// Moving starts its deterministic candidate walk at the failed trigger route.
// Secondary incident endpoints constrain legality but cannot pull that origin away.
template<typename Coord>
Coord movingSearchCenter(const Coord& fallback, const std::vector<Coord>& anchors)
{
    return anchors.empty() ? fallback : anchors.front();
}

// A sink with several incoming routes must be reachable from all of them.
// Center their bounding box, while excluding output fanouts from the balance.
template<typename Coord>
Coord movingSearchCenter(const Coord& fallback, const std::vector<Coord>& anchors,
                         const std::vector<Coord>& incoming_anchors)
{
    Coord primary = movingSearchCenter(fallback, anchors);
    if (incoming_anchors.size() < 2) {
        return primary;
    }
    int min_x = incoming_anchors.front().x;
    int max_x = incoming_anchors.front().x;
    int min_y = incoming_anchors.front().y;
    int max_y = incoming_anchors.front().y;
    for (const Coord& anchor : incoming_anchors) {
        min_x = std::min(min_x, anchor.x);
        max_x = std::max(max_x, anchor.x);
        min_y = std::min(min_y, anchor.y);
        max_y = std::max(max_y, anchor.y);
    }
    return Coord{(min_x + max_x) / 2, (min_y + max_y) / 2};
}

// Every member of a strict packing cluster shares one Moving history owner, so
// selecting another member cannot restart the same placement sequence.
template<typename Endpoint, typename Less>
Endpoint* movingClusterOwner(const std::vector<Endpoint*>& cluster, Endpoint* fallback,
                             Less less)
{
    Endpoint* owner = nullptr;
    for (Endpoint* member : cluster) {
        if (member && (!owner || less(member, owner))) {
            owner = member;
        }
    }
    return owner ? owner : fallback;
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
template<typename Task, typename SourceKey, typename HasCompleteSeed,
         typename IsIndependentSource>
MovingSeedNormalization normalizeMovingSourceRoles(std::vector<Task>& tasks,
                                                   SourceKey source_key,
                                                   HasCompleteSeed has_complete_seed,
                                                   IsIndependentSource is_independent_source)
{
    std::unordered_map<std::string, size_t> first_task;
    std::unordered_map<std::string, size_t> preferred_generic;
    std::unordered_set<std::string> completed_sources;
    std::vector<std::string> source_keys(tasks.size());
    for (size_t index = 0; index < tasks.size(); ++index) {
        // Distributed roots have no shared physical takeoff and therefore do
        // not participate in one-Generic-plus-fanouts source normalization.
        if (is_independent_source(tasks[index])) {
            continue;
        }
        std::string& key = source_keys[index];
        key = source_key(tasks[index]);
        if (key.empty()) {
            continue;
        }
        bool inserted = first_task.try_emplace(key, index).second;
        if (inserted && has_complete_seed(tasks[index])) {
            completed_sources.insert(key);
        }
        if (!tasks[index].fanout) {
            preferred_generic.try_emplace(key, index);
        }
    }

    MovingSeedNormalization result;
    for (size_t index = 0; index < tasks.size(); ++index) {
        if (is_independent_source(tasks[index])) {
            if (tasks[index].fanout) {
                tasks[index].fanout = false;
                ++result.promoted;
            }
            continue;
        }
        const std::string& key = source_keys[index];
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

    // Moving executes the recovered Generic seeds before any dependent
    // branches. This is a stage-level stable partition, not route-edge order.
    std::vector<Task> ordered;
    ordered.reserve(tasks.size());
    for (Task& task : tasks) {
        if (!task.fanout) {
            ordered.push_back(std::move(task));
        }
    }
    for (Task& task : tasks) {
        if (task.fanout) {
            ordered.push_back(std::move(task));
        }
    }
    tasks = std::move(ordered);
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

// Track sustained Basic queue growth independently from a one-pass preemption
// fluctuation. Leaving Basic or shrinking the queue resets the streak.
inline size_t updateBasicGrowthPasses(bool basic_stage, size_t before,
                                      size_t after, size_t growth_passes)
{
    return basic_stage && after > before ? growth_passes + 1 : 0;
}

// One productive growth pass is tolerated. Two consecutive growth passes, or
// one growth pass with no completion/advance, prove preemption is diverging.
inline bool basicGrowthRequiresHandoff(bool basic_stage, size_t before,
                                       size_t after, size_t completed,
                                       size_t advanced,
                                       size_t growth_passes)
{
    return basic_stage && after > before
        && ((completed == 0 && advanced == 0) || growth_passes >= 2);
}

// Fanout transit preemption follows its stage policy without changing Generic preemption.
inline bool transitPreemptionEnabled(bool requested, bool fanout_stage,
                                     bool fanout_preemption_enabled)
{
    return requested && (!fanout_stage || fanout_preemption_enabled);
}

// Source recovery is Generic trunk routing and may preempt transit globally;
// destination recovery requires an isolated focus before it may do so.
inline bool movingRouteMayPreempt(bool moving_stage, bool has_moving_focus,
                                  bool moving_sources = false)
{
    return !moving_stage || moving_sources || has_moving_focus;
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

// Grounding starts at the newest routed destination node inside its radius.
// This preserves every accepted suffix hop instead of repeatedly docking from
// the stale point where the route first entered the docking window.
inline int latestGroundingAnchor(const std::vector<std::pair<int, int>>& depth_distance,
                                 int radius)
{
    for (size_t index = depth_distance.size(); index-- > 0;) {
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

// Once Fanout owns the active queue, a demoted Generic victim belongs to
// Moving recovery; it must not re-enter Basic and interrupt every branch.
inline bool deferFanoutRepairToMoving(bool fanout_stage, bool task_is_fanout)
{
    return fanout_stage && !task_is_fanout;
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
