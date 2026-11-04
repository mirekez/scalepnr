#pragma once

#include <array>
#include <cstdint>
#include <iosfwd>
#include <string_view>

namespace fpga {
// Opt-in observations only: counters and numeric expansion identities never
// participate in routing decisions or survive as a routing cache.
enum class RepairCounter : unsigned {
  gap_calls, source_calls, two_ended_calls, gap_success, source_success,
  cancelled, gap_expanded, source_expanded, gap_unique, source_unique,
  gap_reexpanded, source_reexpanded, gap_source_overlap, source_cross_call_repeat,
  gap_edges, source_edges, gap_seen, source_seen, gap_limit, source_limit,
  gap_parent_steps, source_parent_steps, shared_edge_checks, owner_checks,
  gap_busy_src, gap_busy_dst, gap_labels, source_labels, reused_frontiers,
  stale_source_labels, shared_binding_visits, shared_source_checks,
  shared_fragment_checks, source_probe_expanded, source_probe_edges,
  source_probe_parent_steps, count
};
inline constexpr std::array<std::string_view, unsigned(RepairCounter::count)> repairCounterNames{
  "gap_calls", "source_calls", "two_ended_calls", "gap_success", "source_success",
  "cancelled", "gap_expanded", "source_expanded", "gap_unique", "source_unique",
  "gap_reexpanded", "source_reexpanded", "gap_source_overlap", "source_cross_call_repeat",
  "gap_edges", "source_edges", "gap_seen", "source_seen", "gap_limit", "source_limit",
  "gap_parent_steps", "source_parent_steps", "shared_edge_checks", "owner_checks",
  "gap_busy_src", "gap_busy_dst", "gap_labels", "source_labels", "reused_frontiers",
  "stale_source_labels", "shared_binding_visits", "shared_source_checks",
  "shared_fragment_checks", "source_probe_expanded", "source_probe_edges",
  "source_probe_parent_steps"
};
inline constexpr std::array<std::string_view, 9> repairPhaseNames{
  "exact_gap_search", "source_fallback", "exact_gap_commit", "source_setup",
  "source_free_search", "source_short_completion", "source_full_completion",
  "source_takeoff_completion", "source_commit"
};
// Phase seconds are exclusive of nested repair calls; totals are cumulative
// in this thread. Differences between reports describe each reporting interval.
struct RepairStatistics {
  std::array<uint64_t, unsigned(RepairCounter::count)> counters{};
  std::array<double, repairPhaseNames.size()> seconds{};
  uint64_t operator[](RepairCounter counter) const { return counters[unsigned(counter)]; }
};
const RepairStatistics& repairStatistics();
void resetRepairStatistics(); // Only between searches, chiefly for regressions.
void printRepairStatistics(std::ostream& out, std::string_view context);
}
