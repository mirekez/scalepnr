#pragma once

#include "PlaceTiming.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace technology {
struct Tech;
}

namespace rtl {
struct Inst;
}

namespace pnr {

struct PlaceSwappingConfig {
  int strip_width = 5;
  int search_length = 10;
  int critical_strip_width = 19;
  int critical_search_length = 26;
  int placement_radius = 5;
  int replacement_search_radius = 10;
  int placement_core_radius = 3;
  int replacement_core_radius = 5;
  double minimum_improvement = 0.05;
  double maximum_challenger_timing_degradation = 0.05;
  double strong_improvement = 0.80;
  double maximum_global_regression = 0.05;
  double slack_tolerance_ns = 0.10;
  // Optional stage-completion target independent of the per-endpoint
  // actionable-slack tolerance. This lets a stress regression accept a
  // specified WNS without changing which paths guide swapping.
  double completion_worst_slack_ns =
      -std::numeric_limits<double>::infinity();
  // Abandon a degraded core-search tail and expand from the saved best state
  // once WNS has fallen this far below it.
  double maximum_best_wns_regression_before_expansion_ns = 0.30;
  size_t maximum_swaps_per_bunch = 2;
  // Repeat the complete bounded violation traversal. Timing is rebuilt
  // after every accepted swap and each pass receives a fresh attempt budget.
  size_t maximum_passes = 48;
  size_t maximum_violated_endpoints = 32;
  size_t maximum_critical_edges_per_endpoint = 3;
  // Retain this many challengers from every fixed-width axial search band.
  // The band width never depends on the total stripe length, so extending a
  // stripe appends candidates without evicting any shorter-stripe candidate.
  size_t maximum_candidates_per_edge = 2;
  int candidate_band_length = 2;
  // Added after the complete budget of every preceding band. Set to zero
  // when maximum_attempts must remain a strict total per-pass cap.
  size_t additional_attempts_per_band = 4;
  // Base per-pass limit; it is not shared by all passes.
  size_t maximum_attempts = 32;
};

struct PlaceSwappingResult {
  PlaceTimingAnalysis before;
  PlaceTimingAnalysis after;
  // Saved after the complete original production geometry has exhausted and
  // immediately before any configured wider/farther candidates are admitted.
  // The final result is guaranteed to be no worse than this checkpoint under
  // PlaceSwapping's WNS-first, TNS-second ordering.
  PlaceTimingAnalysis baseline_scope_best;
  bool expanded_beyond_baseline_scope = false;
  size_t passes = 0;
  size_t improving_passes = 0;
  size_t scope_expansions = 0;
  size_t actionable_violations_before = 0;
  size_t actionable_violations_after = 0;
  size_t violated_endpoints_examined = 0;
  size_t candidates_examined = 0;
  size_t attempts = 0;
  size_t accepted_swaps = 0;
  size_t accepted_relaxed_swaps = 0;
  size_t rolled_back_tail_swaps = 0;
  size_t skipped_reused_bunches = 0;
  size_t skipped_fixed_moving_sides = 0;
  size_t skipped_missing_bunch_edges = 0;
  size_t skipped_same_bunch_edges = 0;
  size_t candidate_groups_seen = 0;
  size_t farther_candidates_admitted = 0;
  size_t farther_candidates_attempted = 0;
  size_t farther_candidates_accepted = 0;
  size_t candidate_shortlist_discarded = 0;
  size_t rejected_visited_placements = 0;
  size_t rejected_pack = 0;
  size_t rejected_challenger_timing = 0;
  size_t rejected_improvement = 0;
  size_t rejected_endpoint_improvement = 0;
  size_t rejected_global_timing = 0;
  size_t rolled_back_cells = 0;
  double elapsed_ms = 0;
};

struct PlaceSwapping {
  technology::Tech *tech = nullptr;
  PlaceSwappingConfig config;

  PlaceSwappingResult run(clk::Timings &timings);
  PlaceSwappingResult run(clk::Timings &timings,
                          const std::vector<rtl::Inst *> &cells);
  bool acceptsTimingTradeoff(
      double endpoint_improvement, const PlaceTimingAnalysis &current,
      const PlaceTimingAnalysis &candidate, bool *used_relaxed_rule = nullptr,
      const PlaceTimingAnalysis *run_baseline = nullptr) const;
};

} // namespace pnr
