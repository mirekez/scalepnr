#pragma once

#include "MovingPlacementBlacklist.h"
#include "RouteDesign.h"

namespace pnr {

// The scheduler and regression use the same complete boundary-retry loop.
// Exhaust full wire recovery once. Each cut then gets a free-only wire check;
// repeat full preempting recovery only if relocation has no continuation.
inline bool recoverMovingSourceBoundaries(
    RouteDesign& router, RouteDesign::RouteTask& task,
    MovingPlacementBlacklist& placements,
    std::vector<RouteDesign::RouteTask>* moved_tasks, std::string& fail_reason,
    size_t& released_prefixes, bool& source_moved, bool& candidate_retryable,
    size_t& boundary_retries, bool current_wire_exhausted = false,
    bool retry_probe_windows = false) {
    // A fixed older sink may invoke this after its new source's trunk has
    // completed. It is still Source recovery: apply the same boundary and
    // input-proof rules, then return to the caller's Destination stage.
    struct SourceStageScope {
        bool& value;
        bool saved;
        ~SourceStageScope() { value = saved; }
    } scope{router.moving_sources_stage,router.moving_sources_stage};
    router.moving_sources_stage = true;
    bool completed = false, boundary_released = false;
    bool skip_full_wire = current_wire_exhausted, free_check_pending = false;
    bool graph_changed = false;
    const int retry_budget = std::max(1, router.route_recursion_budget);
    do {
        candidate_retryable = false;
        boundary_released = false;
        if (free_check_pending) {
            free_check_pending = false;
            completed = router.recoverMovingSourceWires(task, true);
            if (completed) break;
            if (router.routeStageSearchCancelled()) {
                candidate_retryable = true;
                fail_reason = "source recovery cancelled during free current-position wire check";
                break;
            }
        }
        router.route_recursion_budget = retry_budget;
        const size_t before = boundary_retries;
        completed = router.moveUnfinishedSource(
            task, moved_tasks, &fail_reason, &released_prefixes, &source_moved,
            &candidate_retryable, &boundary_released, skip_full_wire, &placements,
            &boundary_retries);
        graph_changed |= before != boundary_retries;
        skip_full_wire = true;
        if (completed) break;
        if (boundary_released || released_prefixes) {
            graph_changed = true;
            free_check_pending = true;
        }
        if (movingSourceRetriesReleasedBoundary(boundary_released)) {
            ++boundary_retries;
            PNR_LOG1("ROUT", "Moving source boundary continuation: net='{}', "
                "topology_revisions={}, previous_boundary=({},{})/{}:{}",
                task.net_name, boundary_retries, task.failure_coord.x, task.failure_coord.y,
                static_cast<int>(task.failure_node_type), task.failure_node);
        } else if (!retry_probe_windows || (!candidate_retryable && !released_prefixes)) {
            break;
        }
    } while (!router.routeStageSearchCancelled());
    // Released fabric can also enable a route requiring a new Takeoff or
    // Grounding cut. Preserve that final chance without spending it after
    // every intermediate reverse-search boundary.
    if (!completed && graph_changed && !router.routeStageSearchCancelled())
        completed = router.recoverMovingSourceWires(task);
    if (completed) {
        fail_reason.clear();
        candidate_retryable = false;
    }
    return completed;
}

} // namespace pnr
