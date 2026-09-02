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
  double deficite_slack_ns = -0.10;
  double preferred_proficite_slack_ns = 1.0;
  double minimum_proficite_slack_ns = 0.5;
  // A 50x50 spatial mesh keeps each timing-reserve container local. Swap
  // search visits every container in the endpoints' padded axis-aligned
  // rectangle, in straightforward y-then-x order.
  size_t proficite_regions_per_axis = 50;
  int proficite_rectangle_margin_tiles = 10;
  size_t minimum_proficite_cells_per_region = 50;
  int placement_radius = 5;
  int replacement_search_radius = 10;
  double minimum_improvement = 0.05;
  double strong_improvement = 0.80;
  double maximum_global_regression = 0.05;
  double slack_tolerance_ns = 0.10;
  // Optional stage-completion target independent of the per-endpoint
  // actionable-slack tolerance. This lets a stress regression accept a
  // specified WNS without changing which paths guide swapping.
  double completion_worst_slack_ns =
      -std::numeric_limits<double>::infinity();
  size_t maximum_swaps_per_bunch = 2;
  // Repeat complete DEFICITE traversals using one frozen timing/map snapshot
  // per pass. Provisional swaps use fast cached A/B/C setup correction; the
  // full timing graph and both maps are rebuilt once at the pass boundary.
  size_t maximum_passes = std::numeric_limits<size_t>::max();
  double maximum_runtime_seconds = 300.0;
  size_t maximum_critical_edges_per_endpoint = 3;
};

struct PlaceSwappingResult {
  PlaceTimingAnalysis before;
  PlaceTimingAnalysis after;
  size_t passes = 0;
  size_t improving_passes = 0;
  bool timed_out = false;
  size_t deficite_cells = 0;
  size_t proficite_cells = 0;
  size_t proficite_regions = 0;
  size_t proficite_regions_relaxed = 0;
  size_t actionable_violations_before = 0;
  size_t actionable_violations_after = 0;
  size_t violated_endpoints_examined = 0;
  size_t candidates_examined = 0;
  size_t attempts = 0;
  size_t accepted_swaps = 0;
  size_t accepted_relaxed_swaps = 0;
  size_t pass_timing_analyses = 0;
  size_t locally_corrected_endpoints = 0;
  size_t rejected_local_timing = 0;
  size_t rolled_back_pass_swaps = 0;
  size_t rolled_back_tail_swaps = 0;
  size_t skipped_reused_bunches = 0;
  size_t skipped_fixed_moving_sides = 0;
  size_t skipped_missing_bunch_edges = 0;
  size_t skipped_same_bunch_edges = 0;
  size_t candidate_groups_seen = 0;
  size_t farther_candidates_admitted = 0;
  size_t farther_candidates_attempted = 0;
  size_t farther_candidates_accepted = 0;
  size_t rejected_visited_placements = 0;
  size_t rejected_pack = 0;
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
