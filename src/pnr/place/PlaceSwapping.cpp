#include "PlaceSwapping.h"

#include "Device.h"
#include "RegBunch.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <limits>
#include <print>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace {

using fpga::Coord;

constexpr size_t full_name_limit = std::numeric_limits<size_t>::max();

void collectInsts(rtl::Inst &inst, std::vector<rtl::Inst *> &result) {
  result.push_back(&inst);
  for (rtl::Inst &child : inst.insts)
    collectInsts(child, result);
}

bool isClocked(const rtl::Inst &inst) {
  return inst.cell_ref.peer &&
         technology::Tech::clocked_ports.contains(inst.cell_ref.peer->type);
}

std::vector<rtl::Inst *> timingPeers(rtl::Inst &inst, technology::Tech *tech) {
  std::vector<rtl::Inst *> peers;
  for (rtl::Conn &conn : inst.conns) {
    if (!conn.port_ref.peer)
      continue;
    if (conn.port_ref->type == rtl::Port::PORT_IN) {
      if (conn.port_ref->is_global ||
          (tech &&
           tech->check_clocked(inst.cell_ref->type, conn.port_ref->name))) {
        continue;
      }
      rtl::Conn *driver = conn.follow();
      if (driver && driver->inst_ref.peer) {
        peers.push_back(driver->inst_ref.peer);
      }
    } else if (conn.port_ref->type == rtl::Port::PORT_OUT) {
      for (RefBase<Referable<rtl::Conn>> *sink_ref :
           rtl::Conn::getSinks(conn)) {
        rtl::Conn *sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (sink && sink->inst_ref.peer && sink->port_ref.peer &&
            !sink->port_ref->is_global &&
            (!tech || !tech->check_clocked(sink->inst_ref->cell_ref->type,
                                           sink->port_ref->name))) {
          peers.push_back(sink->inst_ref.peer);
        }
      }
    }
  }
  std::ranges::sort(peers);
  peers.erase(std::unique(peers.begin(), peers.end()), peers.end());
  return peers;
}

int manhattan(Coord left, Coord right) {
  return std::abs(left.x - right.x) + std::abs(left.y - right.y);
}

struct BunchInfo {
  pnr::RegBunch *bunch = nullptr;
  rtl::Inst *anchor = nullptr;
  std::vector<rtl::Inst *> members;
  bool fixed = false;
};

struct PlacementSnapshot {
  rtl::Inst *inst = nullptr;
  fpga::Tile *tile = nullptr;
  Coord coord{-1, -1};
  int pos = -1;
  float outline_x = 0;
  float outline_y = 0;
};

struct BunchSnapshot {
  pnr::RegBunch *bunch = nullptr;
  float x = 0;
  float y = 0;
};

struct ProtectedBunchLimit {
  pnr::RegBunch *bunch = nullptr;
  double slack_ns = -std::numeric_limits<double>::infinity();
};

struct SwapTimingScore {
  double worst_slack_ns = -std::numeric_limits<double>::infinity();
  double tns_delta_ns = std::numeric_limits<double>::infinity();
  double endpoint_slack_ns = -std::numeric_limits<double>::infinity();
  size_t stable_order = std::numeric_limits<size_t>::max();

  auto rank() const {
    // Balance every affected input/output, not just the selected path. TNS
    // uses a delta because different challengers touch different endpoint sets.
    // Exact ordering also keeps alternative sorting strictly transitive.
    return std::tuple{-worst_slack_ns, tns_delta_ns, -endpoint_slack_ns,
                      stable_order};
  }
};

struct SwapAlternative {
  size_t endpoint_index = 0;
  pnr::RegBunch *moving_bunch = nullptr;
  pnr::RegBunch *challenger_bunch = nullptr;
  bool compact_combs = false;
  double endpoint_improvement = 0;
  SwapTimingScore score;
  std::array<ProtectedBunchLimit, 3> protected_limits{};
};

struct AcceptedSwapSnapshot {
  std::vector<PlacementSnapshot> placements;
  std::vector<BunchSnapshot> bunches;
  std::uint64_t fingerprint_before = 0;
  std::vector<SwapAlternative> alternatives;
};

struct SwapAttemptResult {
  bool packed = false;
  std::vector<PlacementSnapshot> snapshots;
  struct PackingFailure {
    rtl::Inst *inst = nullptr;
    Coord desired{-1, -1};
    int radius = 0;
    bool moving_group = false;
    bool anchor = false;
  };
  std::vector<PackingFailure> failures;
};

const pnr::PlaceTimingEndpoint *
findEndpoint(const pnr::PlaceTimingAnalysis &analysis,
             const rtl::Conn *data_in) {
  auto found = std::ranges::find(analysis.endpoint_details, data_in,
                                 &pnr::PlaceTimingEndpoint::data_in);
  return found == analysis.endpoint_details.end() ? nullptr : &*found;
}

} // namespace

pnr::PlaceSwappingResult pnr::PlaceSwapping::run(clk::Timings &timings) {
  std::vector<rtl::Inst *> cells;
  if (tech)
    collectInsts(tech->design.top, cells);
  return run(timings, cells);
}

double pnr::PlaceSwapping::temperatureForPass(
    double initial_temperature, size_t pass_index) const {
  return std::max(
      0.0, std::max(0.0, initial_temperature) -
               config.temperature_cooling_per_pass_ns *
                   static_cast<double>(pass_index));
}

double pnr::PlaceSwapping::criticalSlackLimitForPass(
    double before_slack_ns, double initial_temperature,
    size_t pass_index) const {
  return std::min(-temperatureForPass(initial_temperature, pass_index),
                  before_slack_ns);
}

double pnr::PlaceSwapping::challengerSlackLimitForPass(
    double before_slack_ns, double initial_temperature,
    size_t pass_index) const {
  return before_slack_ns -
         temperatureForPass(initial_temperature, pass_index);
}

bool pnr::PlaceSwapping::acceptsTimingTradeoff(
    double endpoint_improvement, const PlaceTimingAnalysis &current,
    const PlaceTimingAnalysis &candidate, bool *used_relaxed_rule,
    const PlaceTimingAnalysis *run_baseline) const {
  constexpr double epsilon = 1e-9;
  if (used_relaxed_rule)
    *used_relaxed_rule = false;
  bool strict = candidate.total_negative_slack_ns <=
                    current.total_negative_slack_ns + epsilon &&
                candidate.worst_slack_ns >= current.worst_slack_ns - epsilon;
  if (strict)
    return true;
  if (endpoint_improvement + epsilon < config.strong_improvement) {
    return false;
  }

  const PlaceTimingAnalysis &baseline = run_baseline ? *run_baseline : current;
  double scale = 1.0 + config.maximum_global_regression;
  auto boundedLimit = [&](double current_value, double baseline_value) {
    // Intermediate strict improvements may be traded back, but never by
    // more than the configured fraction below the supplied reference state.
    // PlaceSwapping supplies its best accepted state here. Using run entry
    // allowed a late 80%-endpoint move to turn -0.189 ns back into -0.454 ns
    // while still claiming it was inside a 5% global regression allowance.
    return std::min(std::max(current_value * scale, baseline_value),
                    baseline_value * scale);
  };
  bool tns_within = candidate.total_negative_slack_ns <=
                    boundedLimit(current.total_negative_slack_ns,
                                 baseline.total_negative_slack_ns) +
                        epsilon;
  double current_wns_deficit =
      std::max(0.0, -current.worst_slack_ns - config.slack_tolerance_ns);
  double candidate_wns_deficit =
      std::max(0.0, -candidate.worst_slack_ns - config.slack_tolerance_ns);
  double baseline_wns_deficit =
      std::max(0.0, -baseline.worst_slack_ns - config.slack_tolerance_ns);
  bool wns_within =
      candidate_wns_deficit <=
      boundedLimit(current_wns_deficit, baseline_wns_deficit) + epsilon;
  bool relaxed = tns_within && wns_within;
  if (used_relaxed_rule)
    *used_relaxed_rule = relaxed;
  return relaxed;
}

pnr::PlaceSwappingResult
pnr::PlaceSwapping::run(clk::Timings &timings,
                        const std::vector<rtl::Inst *> &cells) {
  auto started = std::chrono::steady_clock::now();
  PlaceSwappingResult result;
  if (!tech || cells.empty())
    return result;

  fpga::Device &device = fpga::Device::current();
  int width = device.size_width;
  int height = device.size_height;
  if (width <= 0 || height <= 0 || device.tile_grid.empty())
    return result;

  PlaceTiming timing;
  timing.tech = tech;
  timings.calculateTimings();
  timing.preparePlacementGuide(timings);
  result.before = timing.analyze(timings);
  PlaceTimingAnalysis current = result.before;
  PlaceTimingPrepared prepared_timing(timing);
  const bool reference_local_timing =
      std::getenv("SCALEPNR_PLACE_SWAP_REFERENCE_LOCAL_TIMING") != nullptr;
  size_t local_timing_calls = 0;
  double local_timing_ms = 0;
  auto evaluateLocalTiming = [&](const std::vector<PlaceTimingEndpoint*>& endpoints) {
    const auto begin = std::chrono::steady_clock::now();
    if (reference_local_timing) timing.evaluateSetupTiming(endpoints);
    else prepared_timing.evaluate(endpoints);
    ++local_timing_calls;
    local_timing_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
  };
  result.initial_temperature =
      std::max(0.0, -result.before.worst_slack_ns);
  std::unordered_map<const rtl::Inst *, size_t> stable_cell_order;
  stable_cell_order.reserve(cells.size());
  for (size_t index = 0; index < cells.size(); ++index) {
    if (cells[index])
      stable_cell_order.emplace(cells[index], index);
  }
  auto isActionable = [&](const PlaceTimingEndpoint &endpoint) {
    return endpoint.slack_ns < config.deficite_slack_ns;
  };
  auto countActionable = [&](const PlaceTimingAnalysis &analysis) {
    return static_cast<size_t>(
        std::ranges::count_if(analysis.endpoint_details, isActionable));
  };
  auto completionReached = [&](const PlaceTimingAnalysis &analysis) {
    return std::isfinite(config.completion_worst_slack_ns) &&
           analysis.worst_slack_ns + 1e-9 >= config.completion_worst_slack_ns;
  };
  auto timedOut = [&] {
    return config.maximum_runtime_seconds > 0.0 &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         started)
                   .count() >= config.maximum_runtime_seconds;
  };
  auto recoveryReserveReached = [&] {
    if (config.maximum_runtime_seconds <= 0.0 ||
        config.recovery_runtime_reserve_seconds <= 0.0)
      return false;
    double reserve = std::min(config.recovery_runtime_reserve_seconds,
                              0.25 * config.maximum_runtime_seconds);
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         started)
               .count() >= config.maximum_runtime_seconds - reserve;
  };
  result.actionable_violations_before = countActionable(result.before);
  const char *trace_a = std::getenv("SCALEPNR_PLACE_SWAP_TRACE_A");
  const char *trace_b = std::getenv("SCALEPNR_PLACE_SWAP_TRACE_B");
  double marker_target_wns = std::numeric_limits<double>::quiet_NaN();
  if (const char *value = std::getenv("SCALEPNR_PLACE_SWAP_MARK_WNS")) {
    char *end = nullptr;
    double parsed = std::strtod(value, &end);
    if (end != value && *end == '\0' && std::isfinite(parsed))
      marker_target_wns = parsed;
  }
  bool marker_target_captured = false;
  bool marker_target_frame_pending = false;
  auto captureMarkerTarget = [&](const PlaceTimingAnalysis &analysis) {
    if (marker_target_captured || !std::isfinite(marker_target_wns) ||
        std::round(analysis.worst_slack_ns * 1000.0) !=
            std::round(marker_target_wns * 1000.0)) {
      return;
    }
    const PlaceTimingEndpoint *worst_endpoint = nullptr;
    for (const PlaceTimingEndpoint &endpoint : analysis.endpoint_details) {
      if (!worst_endpoint || endpoint.slack_ns < worst_endpoint->slack_ns)
        worst_endpoint = &endpoint;
    }
    if (!worst_endpoint || worst_endpoint->critical_edges.empty())
      return;
    const PlaceTimingEdge &marked_edge = *std::ranges::max_element(
        worst_endpoint->critical_edges, {}, &PlaceTimingEdge::wire_delay_ns);
    if (!marked_edge.driver || !marked_edge.sink)
      return;
    tech->place.movement_marker_a = marked_edge.driver;
    tech->place.movement_marker_b = marked_edge.sink;
    marker_target_captured = true;
    marker_target_frame_pending = true;
    std::print(
        "\nPLACE_SWAPPING_TARGET_WNS_MARKERS target_wns_ns={:.3f} "
        "actual_wns_ns={:.3f} endpoint='{}' A='{}' coord=({},{}) "
        "B='{}' coord=({},{}) wire_delay_ns={:.3f}",
        marker_target_wns, analysis.worst_slack_ns,
        worst_endpoint->data_in && worst_endpoint->data_in->inst_ref.peer
            ? worst_endpoint->data_in->inst_ref.peer->makeName(full_name_limit)
            : std::string{"<none>"},
        marked_edge.driver->makeName(full_name_limit),
        marked_edge.driver->coord.x, marked_edge.driver->coord.y,
        marked_edge.sink->makeName(full_name_limit), marked_edge.sink->coord.x,
        marked_edge.sink->coord.y, marked_edge.wire_delay_ns);
  };
  // A requested ALPHA target may be the PlaceSwapping entry WNS. Previously
  // target capture only ran after an accepted pass, so diagnostics silently
  // switched to the final WNS pair when the requested state preceded all
  // swaps.
  captureMarkerTarget(current);
  auto report = [&] {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - started)
                            .count();
    std::print(
        "\nPLACE_SWAPPING_SUMMARY endpoints={} violations={}->{} "
        "actionable_violations={}->{} worst_slack_ns={:.3f}->{:.3f} "
        "tns_ns={:.3f}->{:.3f} passes={} improving_passes={} timed_out={} "
        "DEFICITE={} PROFICITE={} regions={} relaxed_regions={} "
        "candidates={} "
        "attempts={} accepted={} accepted_relaxed={} "
        "pass_timing_analyses={} local_recovery_evaluations={} "
        "acceptance_capped_passes={} recovery_reserved_passes={} "
        "locally_corrected_endpoints={} "
        "rejected_local_timing={} "
        "rolled_back_pass_swaps={} rolled_back_tail_swaps={} "
        "skipped_fixed_moving_sides={} "
        "skipped_missing_bunch_edges={} skipped_same_bunch_edges={} "
        "candidate_groups_seen={} farther_candidates_admitted={} "
        "farther_candidates_attempted={} farther_candidates_accepted={} "
        "rejected_visited_placements={} "
        "rejected_pack={} "
        "rejected_improvement={} "
        "rejected_endpoint_improvement={} rejected_global_timing={} "
        "rolled_back_cells={} initial_TEMPERATURE={:.3f} "
        "cooling_per_pass={:.3f} "
        "deficite_slack_ns={:.3f} "
        "preferred_proficite_slack_ns={:.3f} "
        "minimum_proficite_slack_ns={:.3f} "
        "proficite_regions_per_axis={} "
        "proficite_rectangle_margin_tiles={} "
        "minimum_proficite_cells_per_region={} "
        "placement_radius={} replacement_search_radius={} "
        "repack_combinational_fallback={} "
        "maximum_accepted_swaps_per_pass={} "
        "recovery_runtime_reserve_seconds={:.1f} "
        "edge_limit={} "
        "minimum_improvement={:.1f}% "
        "strong_improvement={:.1f}% "
        "maximum_global_regression={:.1f}% slack_tolerance_ns={:.3f} "
        "completion_worst_slack_ns={:.3f} maximum_runtime_seconds={:.1f} "
        "elapsed_ms={:.3f}\n",
        result.after.endpoints, result.before.violated_endpoints,
        result.after.violated_endpoints, result.actionable_violations_before,
        result.actionable_violations_after, result.before.worst_slack_ns,
        result.after.worst_slack_ns, result.before.total_negative_slack_ns,
        result.after.total_negative_slack_ns, result.passes,
        result.improving_passes, result.timed_out, result.deficite_cells,
        result.proficite_cells,
        result.proficite_regions, result.proficite_regions_relaxed,
        result.candidates_examined, result.attempts, result.accepted_swaps,
        result.accepted_relaxed_swaps, result.pass_timing_analyses,
        result.local_recovery_evaluations,
        result.acceptance_capped_passes, result.recovery_reserved_passes,
        result.locally_corrected_endpoints, result.rejected_local_timing,
        result.rolled_back_pass_swaps, result.rolled_back_tail_swaps,
        result.skipped_fixed_moving_sides,
        result.skipped_missing_bunch_edges, result.skipped_same_bunch_edges,
        result.candidate_groups_seen, result.farther_candidates_admitted,
        result.farther_candidates_attempted, result.farther_candidates_accepted,
        result.rejected_visited_placements, result.rejected_pack,
        result.rejected_improvement,
        result.rejected_endpoint_improvement, result.rejected_global_timing,
        result.rolled_back_cells, result.initial_temperature,
        config.temperature_cooling_per_pass_ns, config.deficite_slack_ns,
        config.preferred_proficite_slack_ns, config.minimum_proficite_slack_ns,
        config.proficite_regions_per_axis,
        config.proficite_rectangle_margin_tiles,
        config.minimum_proficite_cells_per_region, config.placement_radius,
        config.replacement_search_radius,
        config.repack_combinational_fallback,
        config.maximum_accepted_swaps_per_pass,
        config.recovery_runtime_reserve_seconds,
        config.maximum_critical_edges_per_endpoint,
        100.0 * config.minimum_improvement, 100.0 * config.strong_improvement,
        100.0 * config.maximum_global_regression, config.slack_tolerance_ns,
        config.completion_worst_slack_ns, config.maximum_runtime_seconds,
        result.elapsed_ms);
  };
  if (current.violated_endpoints == 0) {
    result.after = current;
    result.actionable_violations_after = 0;
    report();
    return result;
  }

  std::unordered_map<RegBunch *, BunchInfo> groups;
  groups.reserve(cells.size() / 2);
  for (rtl::Inst *inst : cells) {
    if (!inst || !inst->tile.peer || !inst->bunch_ref.peer ||
        !fpga::isPlaceableElement(*inst)) {
      continue;
    }
    BunchInfo &group = groups[inst->bunch_ref.peer];
    group.bunch = inst->bunch_ref.peer;
    group.members.push_back(inst);
    group.fixed |= inst->outline.fixed;
  }
  for (auto &[bunch, group] : groups) {
    group.anchor =
        bunch && bunch->reg && bunch->reg->tile.peer ? bunch->reg : nullptr;
    if (!group.anchor) {
      auto anchor = std::ranges::find_if(group.members, [](rtl::Inst *inst) {
        return inst && isClocked(*inst);
      });
      if (anchor != group.members.end())
        group.anchor = *anchor;
    }
    if (!group.anchor && !group.members.empty()) {
      group.anchor = group.members.front();
    }
    group.fixed |=
        !bunch || bunch->fixed || !group.anchor || group.anchor->outline.fixed;
  }

  // Bunches remain eligible after any number of swaps. Reject repeated full
  // placements, not repeated participation by an individual bunch.
  auto placementToken = [](const rtl::Inst *inst, Coord coord, int pos) {
    auto mix = [](std::uint64_t value) {
      value += 0x9e3779b97f4a7c15ULL;
      value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
      value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
      return value ^ (value >> 31);
    };
    std::uint64_t value =
        static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(inst));
    value ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(coord.x))
             << 1;
    value ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(coord.y))
             << 22;
    value ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(pos))
             << 43;
    return mix(value);
  };
  auto calculatePlacementFingerprint = [&] {
    std::uint64_t hash = 0;
    for (rtl::Inst *inst : cells) {
      if (!inst || !inst->tile.peer || !fpga::isPlaceableElement(*inst))
        continue;
      hash ^= placementToken(inst, inst->coord, inst->pos);
    }
    return hash;
  };
  std::uint64_t placement_fingerprint = calculatePlacementFingerprint();
  std::unordered_set<std::uint64_t> visited_placements{
      placement_fingerprint};
  auto fingerprintAfter = [&](const std::vector<PlacementSnapshot> &snapshots) {
    std::uint64_t fingerprint = placement_fingerprint;
    for (const PlacementSnapshot &snapshot : snapshots) {
      fingerprint ^= placementToken(snapshot.inst, snapshot.coord,
                                    snapshot.pos);
      fingerprint ^= placementToken(snapshot.inst, snapshot.inst->coord,
                                    snapshot.inst->pos);
    }
    return fingerprint;
  };

  auto restore = [&](const std::vector<PlacementSnapshot> &snapshots,
                     const std::vector<BunchSnapshot> &bunch_snapshots) {
    for (const PlacementSnapshot &snapshot : snapshots) {
      if (snapshot.inst && snapshot.inst->tile.peer) {
        snapshot.inst->tile->unassign(snapshot.inst);
      }
    }
    std::vector<const PlacementSnapshot *> pending;
    pending.reserve(snapshots.size());
    for (const PlacementSnapshot &snapshot : snapshots) {
      pending.push_back(&snapshot);
    }
    while (!pending.empty()) {
      size_t before = pending.size();
      for (auto it = pending.begin(); it != pending.end();) {
        const PlacementSnapshot &snapshot = **it;
        int restored =
            snapshot.tile ? snapshot.tile->tryAddAt(snapshot.inst, snapshot.pos)
                          : -1;
        if (restored == snapshot.pos) {
          snapshot.inst->coord = snapshot.coord;
          snapshot.inst->pos = snapshot.pos;
          snapshot.inst->outline.x = snapshot.outline_x;
          snapshot.inst->outline.y = snapshot.outline_y;
          it = pending.erase(it);
        } else {
          ++it;
        }
      }
      PNR_ASSERT(pending.size() < before,
                 "PlaceSwapping could not restore {} original placements",
                 pending.size());
    }
    for (const BunchSnapshot &snapshot : bunch_snapshots) {
      if (!snapshot.bunch)
        continue;
      snapshot.bunch->x = snapshot.x;
      snapshot.bunch->y = snapshot.y;
    }
  };

  constexpr double epsilon = 1e-9;
  std::vector<AcceptedSwapSnapshot> accepted_history;
  double best_worst_slack_ns = current.worst_slack_ns;
  double best_tns_ns = current.total_negative_slack_ns;
  PlaceTimingAnalysis best_timing_limits;
  best_timing_limits.worst_slack_ns = best_worst_slack_ns;
  best_timing_limits.total_negative_slack_ns = best_tns_ns;
  size_t best_history_size = 0;
  auto retainsBetterFinalTiming = [&](const PlaceTimingAnalysis &candidate) {
    if (candidate.worst_slack_ns > best_worst_slack_ns + 1e-9)
      return true;
    return std::abs(candidate.worst_slack_ns - best_worst_slack_ns) <= 1e-9 &&
           candidate.total_negative_slack_ns < best_tns_ns - 1e-9;
  };
  auto rollbackTailToBest = [&] {
    bool restored_any = false;
    while (accepted_history.size() > best_history_size) {
      AcceptedSwapSnapshot &snapshot = accepted_history.back();
      result.rolled_back_cells += snapshot.placements.size();
      placement_fingerprint = snapshot.fingerprint_before;
      restore(snapshot.placements, snapshot.bunches);
      accepted_history.pop_back();
      ++result.rolled_back_tail_swaps;
      restored_any = true;
    }
    if (restored_any)
      current = timing.analyze(timings);
  };

  struct WeightedPeer {
    rtl::Inst *inst = nullptr;
    double weight = 0;
  };
  std::unordered_map<rtl::Inst *, std::vector<WeightedPeer>> peer_cache;
  peer_cache.reserve(cells.size());
  auto weightedPeers =
      [&](rtl::Inst &inst) -> const std::vector<WeightedPeer> & {
    auto found = peer_cache.find(&inst);
    if (found != peer_cache.end())
      return found->second;
    std::vector<WeightedPeer> peers;
    for (rtl::Inst *peer : timingPeers(inst, tech)) {
      if (peer)
        peers.push_back({peer, timing.placementNetWeight(inst, *peer)});
    }
    return peer_cache.emplace(&inst, std::move(peers)).first->second;
  };

  auto replacementOrigin = [&](const BunchInfo &group, Coord fallback) {
    double sum_x = 0;
    double sum_y = 0;
    double sum_weight = 0;
    for (rtl::Inst *inst : group.members) {
      if (!inst)
        continue;
      Coord member_offset = inst->coord - group.anchor->coord;
      for (const WeightedPeer &weighted_peer : weightedPeers(*inst)) {
        rtl::Inst *peer = weighted_peer.inst;
        if (!peer || !peer->tile.peer || peer->bunch_ref.peer == group.bunch)
          continue;
        double weight = weighted_peer.weight;
        Coord desired_anchor = peer->coord - member_offset;
        sum_x += weight * desired_anchor.x;
        sum_y += weight * desired_anchor.y;
        sum_weight += weight;
      }
    }
    if (sum_weight <= 0)
      return fallback;
    return Coord{
        std::clamp(static_cast<int>(std::lround(sum_x / sum_weight)), 0,
                   width - 1),
        std::clamp(static_cast<int>(std::lround(sum_y / sum_weight)), 0,
                   height - 1),
    };
  };

  bool explain_packing =
      std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_PACKING") != nullptr;
  auto attemptRelocation = [&](BunchInfo &first, BunchInfo &second,
                               bool place_challenger = true,
                               bool compact_combs = false) {
    SwapAttemptResult attempt;
    std::vector<PlacementSnapshot> snapshots;
    snapshots.reserve(first.members.size() + second.members.size());
    std::unordered_set<rtl::Inst *> moving;
    std::vector<BunchSnapshot> bunch_snapshots{
        {first.bunch, first.bunch->x, first.bunch->y},
        {second.bunch, second.bunch->x, second.bunch->y},
    };
    auto save = [&](BunchInfo &group) {
      for (rtl::Inst *inst : group.members) {
        if (!inst || !inst->tile.peer || !moving.insert(inst).second) {
          continue;
        }
        snapshots.push_back(PlacementSnapshot{
            .inst = inst,
            .tile = inst->tile.peer,
            .coord = inst->coord,
            .pos = inst->pos,
            .outline_x = inst->outline.x,
            .outline_y = inst->outline.y,
        });
      }
    };
    save(first);
    save(second);
    if (snapshots.size() != first.members.size() + second.members.size()) {
      attempt.snapshots = std::move(snapshots);
      return attempt;
    }

    Coord first_origin = first.anchor->coord;
    Coord second_origin = second.anchor->coord;
    Coord second_replacement = replacementOrigin(second, first_origin);
    std::unordered_map<rtl::Inst *, Coord> desired;
    desired.reserve(snapshots.size());
    auto setDesired = [&](BunchInfo &group, Coord origin, Coord target) {
      for (rtl::Inst *inst : group.members) {
        Coord coordinate = target + (inst->coord - origin);
        if (compact_combs && &group == &first && inst != first.anchor &&
            !isClocked(*inst))
          coordinate = target;
        coordinate.x = std::clamp(coordinate.x, 0, width - 1);
        coordinate.y = std::clamp(coordinate.y, 0, height - 1);
        desired[inst] = coordinate;
      }
    };
    setDesired(first, first_origin, second_origin);
    if (place_challenger)
      setDesired(second, second_origin, second_replacement);

    for (const PlacementSnapshot &snapshot : snapshots) {
      snapshot.inst->tile->unassign(snapshot.inst);
    }

    std::vector<rtl::Inst *> pending;
    pending.reserve(snapshots.size());
    pending.insert(pending.end(), first.members.begin(), first.members.end());
    if (place_challenger) {
      pending.insert(pending.end(), second.members.begin(),
                     second.members.end());
    }
    std::ranges::stable_sort(pending, [&](rtl::Inst *left, rtl::Inst *right) {
      auto priority = [&](rtl::Inst *inst) {
        if (inst == first.anchor || inst == second.anchor)
          return 0;
        return isClocked(*inst) ? 1 : 2;
      };
      return priority(left) < priority(right);
    });

    auto placeOne = [&](rtl::Inst &inst) {
      struct Candidate {
        Coord coord;
        double cost = 0;
        int radius = 0;
      };
      const std::vector<WeightedPeer> &weighted_peers = weightedPeers(inst);
      Coord target = desired.at(&inst);
      // The critical anchor claims the challenger's Tile exactly. The
      // challenger gets an independent timing-directed replacement search;
      // the smaller radius legalizes both bunches' followers around the
      // anchors selected by those two steps.
      int radius_limit =
          &inst == first.anchor
              ? 0
              : (&inst == second.anchor ? config.replacement_search_radius
                                        : config.placement_radius);
      std::vector<Coord> search_origins{target};
      if (&inst == second.anchor && first_origin != target) {
        // A swap has just vacated first_origin. The timing centroid is still
        // preferred, but it must not be the only packing search: it can be
        // surrounded by incompatible/full Tiles even though the literal swap
        // destination is guaranteed to have just lost the moving anchor.
        // Append this fallback so adding possibilities cannot perturb any
        // relocation which already succeeds around the timing target.
        search_origins.push_back(first_origin);
      }
      for (Coord search_origin : search_origins) {
        for (int band_radius = 0; band_radius <= radius_limit; ++band_radius) {
          std::vector<Candidate> candidates;
          for (int dy = -band_radius; dy <= band_radius; ++dy) {
            for (int dx = -band_radius; dx <= band_radius; ++dx) {
              int radius = std::abs(dx) + std::abs(dy);
              if (radius != band_radius)
                continue;
              Coord coordinate = search_origin + Coord{dx, dy};
              if (coordinate.x < 0 || coordinate.x >= width ||
                  coordinate.y < 0 || coordinate.y >= height) {
                continue;
              }
              fpga::Tile &tile =
                  device.tile_grid[coordinate.y * width + coordinate.x];
              if (!tile.tile_type)
                continue;
              double cost = 0.01 * radius;
              for (const WeightedPeer &weighted_peer : weighted_peers) {
                rtl::Inst *peer = weighted_peer.inst;
                Coord peer_coord{-1, -1};
                if (peer->tile.peer)
                  peer_coord = peer->coord;
                else if (desired.contains(peer))
                  peer_coord = desired[peer];
                if (peer_coord.x < 0)
                  continue;
                cost +=
                    weighted_peer.weight * manhattan(coordinate, peer_coord);
              }
              candidates.push_back({coordinate, cost, radius});
            }
          }
          std::ranges::sort(candidates,
                            [](const Candidate &left, const Candidate &right) {
                              if (left.cost != right.cost)
                                return left.cost < right.cost;
                              if (left.radius != right.radius)
                                return left.radius < right.radius;
                              if (left.coord.y != right.coord.y)
                                return left.coord.y < right.coord.y;
                              return left.coord.x < right.coord.x;
                            });
          for (const Candidate &candidate : candidates) {
            fpga::Tile &tile = device.tile_grid[candidate.coord.y * width +
                                               candidate.coord.x];
            int pos = tile.tryAdd(&inst, false);
            if (pos < 0)
              continue;
            float aspect_x = std::max(tech->place.aspect_x, 0.0001F);
            float aspect_y = std::max(tech->place.aspect_y, 0.0001F);
            inst.outline.x =
                (inst.coord.x + 0.25F * (pos % 4)) / aspect_x;
            inst.outline.y =
                (inst.coord.y + 0.25F * (pos / 4)) / aspect_y;
            if (&inst == second.anchor) {
              // Followers use the replacement anchor actually selected by the
              // second search, not merely its timing-centroid seed.
              setDesired(second, second_origin, inst.coord);
            }
            return true;
          }
        }
      }
      return false;
    };

    while (!pending.empty()) {
      size_t before = pending.size();
      for (auto it = pending.begin(); it != pending.end();) {
        if (placeOne(**it))
          it = pending.erase(it);
        else
          ++it;
      }
      if (pending.size() == before)
        break;
    }
    if (!pending.empty()) {
      if (explain_packing) {
        for (rtl::Inst *inst : pending) {
          bool is_first = std::ranges::find(first.members, inst) !=
                          first.members.end();
          bool is_anchor = inst == first.anchor || inst == second.anchor;
          int radius = inst == first.anchor
                           ? 0
                           : (inst == second.anchor
                                  ? config.replacement_search_radius
                                  : config.placement_radius);
          attempt.failures.push_back({inst, desired.at(inst), radius, is_first,
                                      is_anchor});
        }
      }
      restore(snapshots, bunch_snapshots);
      attempt.snapshots = std::move(snapshots);
      return attempt;
    }

    float aspect_x = std::max(tech->place.aspect_x, 0.0001F);
    float aspect_y = std::max(tech->place.aspect_y, 0.0001F);
    first.bunch->x = first.anchor->coord.x / aspect_x;
    first.bunch->y = first.anchor->coord.y / aspect_y;
    if (place_challenger) {
      second.bunch->x = second.anchor->coord.x / aspect_x;
      second.bunch->y = second.anchor->coord.y / aspect_y;
    }
    attempt.packed = true;
    attempt.snapshots = std::move(snapshots);
    return attempt;
  };

  struct DeficiteCell {
    rtl::Inst *cell = nullptr;
    PlaceTimingEndpoint *endpoint = nullptr;
    size_t stable_order = 0;
  };
  struct ProficiteCell {
    BunchInfo *group = nullptr;
    double slack_ns = 0;
    size_t stable_order = 0;
  };
  struct TimingCellMaps {
    std::vector<DeficiteCell> deficite;
    std::vector<std::vector<ProficiteCell>> proficite;
    size_t relaxed_regions = 0;
    size_t proficite_cells = 0;
  };

  size_t regions_per_axis =
      std::max<size_t>(1, config.proficite_regions_per_axis);
  size_t region_count = regions_per_axis * regions_per_axis;
  auto regionCoordinate = [&](Coord coordinate) {
    int region_x =
        std::clamp(static_cast<int>(static_cast<long long>(coordinate.x) *
                                    static_cast<long long>(regions_per_axis) /
                                    std::max(1, width)),
                   0, static_cast<int>(regions_per_axis) - 1);
    int region_y =
        std::clamp(static_cast<int>(static_cast<long long>(coordinate.y) *
                                    static_cast<long long>(regions_per_axis) /
                                    std::max(1, height)),
                   0, static_cast<int>(regions_per_axis) - 1);
    return Coord{region_x, region_y};
  };
  auto regionIndex = [&](Coord region) {
    return static_cast<size_t>(region.y) * regions_per_axis +
           static_cast<size_t>(region.x);
  };

  auto buildTimingCellMaps = [&](PlaceTimingAnalysis &analysis) {
    TimingCellMaps maps;
    maps.proficite.resize(region_count);

    std::unordered_map<rtl::Inst *, PlaceTimingEndpoint *> worst_deficite;
    std::unordered_map<RegBunch *, double> group_setup_slack;
    auto accountGroupSlack = [&](rtl::Inst *inst, double slack_ns) {
      if (!inst || !inst->bunch_ref.peer ||
          !groups.contains(inst->bunch_ref.peer))
        return;
      auto [found, inserted] =
          group_setup_slack.emplace(inst->bunch_ref.peer, slack_ns);
      if (!inserted)
        found->second = std::min(found->second, slack_ns);
    };

    for (PlaceTimingEndpoint &endpoint : analysis.endpoint_details) {
      rtl::Inst *endpoint_cell =
          endpoint.data_in ? endpoint.data_in->inst_ref.peer : nullptr;
      accountGroupSlack(endpoint_cell, endpoint.slack_ns);
      for (const PlaceTimingEdge &edge : endpoint.critical_edges) {
        accountGroupSlack(edge.driver, endpoint.slack_ns);
        accountGroupSlack(edge.sink, endpoint.slack_ns);
      }

      if (!endpoint_cell ||
          endpoint.slack_ns > config.deficite_slack_ns + epsilon)
        continue;
      auto [found, inserted] = worst_deficite.emplace(endpoint_cell, &endpoint);
      if (!inserted && endpoint.slack_ns < found->second->slack_ns)
        found->second = &endpoint;
    }

    maps.deficite.reserve(worst_deficite.size());
    for (const auto &[cell, endpoint] : worst_deficite) {
      auto order = stable_cell_order.find(cell);
      maps.deficite.push_back({cell, endpoint,
                               order == stable_cell_order.end()
                                   ? std::numeric_limits<size_t>::max()
                                   : order->second});
    }
    std::ranges::sort(
        maps.deficite, [](const DeficiteCell &left, const DeficiteCell &right) {
          if (left.endpoint->slack_ns != right.endpoint->slack_ns)
            return left.endpoint->slack_ns < right.endpoint->slack_ns;
          return left.stable_order < right.stable_order;
        });

    for (auto &[bunch, group] : groups) {
      auto slack = group_setup_slack.find(bunch);
      if (group.fixed || !group.anchor || !group.anchor->tile.peer ||
          slack == group_setup_slack.end() ||
          slack->second + epsilon < config.minimum_proficite_slack_ns) {
        continue;
      }
      Coord region = regionCoordinate(group.anchor->coord);
      auto order = stable_cell_order.find(group.anchor);
      maps.proficite[regionIndex(region)].push_back(
          {&group, slack->second,
           order == stable_cell_order.end() ? std::numeric_limits<size_t>::max()
                                            : order->second});
    }

    for (std::vector<ProficiteCell> &container : maps.proficite) {
      std::ranges::sort(
          container, [](const ProficiteCell &left, const ProficiteCell &right) {
            if (left.slack_ns != right.slack_ns)
              return left.slack_ns > right.slack_ns;
            return left.stable_order < right.stable_order;
          });
      size_t preferred_count = static_cast<size_t>(
          std::ranges::count_if(container, [&](const ProficiteCell &entry) {
            return entry.slack_ns + epsilon >=
                   config.preferred_proficite_slack_ns;
          }));
      if (preferred_count >= config.minimum_proficite_cells_per_region) {
        // The preferred threshold has enough alternatives in this region.
        // Keep every cell which satisfies it: the minimum count controls only
        // threshold relaxation, not the size of the candidate container.
        container.resize(preferred_count);
      } else {
        // Recalculate this sparse region at the lower threshold.  The initial
        // population already contains every cell satisfying that floor.
        ++maps.relaxed_regions;
      }
      maps.proficite_cells += container.size();
    }
    return maps;
  };

  size_t timing_map_frame = 0;
  bool log_proficite_cells =
      std::getenv("SCALEPNR_PLACE_SWAP_LOG_PROFICITE") != nullptr;
  bool defer_timing_map_png =
      std::getenv("SCALEPNR_PLACE_SWAP_DEFER_PNG") != nullptr;
  auto captureTimingMapFrame = [&](const TimingCellMaps &maps,
                                   const std::string &label) {
    PlaceDesign &place = tech->place;
    bool render = !place.movement_png_prefix.empty() &&
                  !place.movement_snapshot_cells.empty();
    size_t frame_index = timing_map_frame++;
    if (render) {
      place.movement_deficite_cells.clear();
      place.movement_proficite_cells.clear();
      for (const DeficiteCell &entry : maps.deficite) {
        if (entry.cell)
          place.movement_deficite_cells.insert(entry.cell);
      }
    }
    for (size_t region_index = 0; region_index < maps.proficite.size();
         ++region_index) {
      const std::vector<ProficiteCell> &container =
          maps.proficite[region_index];
      for (const ProficiteCell &entry : container) {
        if (entry.group && entry.group->anchor) {
          if (render)
            place.movement_proficite_cells.insert(entry.group->anchor);
          if (log_proficite_cells) {
            Coord region{
                static_cast<int>(region_index % regions_per_axis),
                static_cast<int>(region_index / regions_per_axis),
            };
            std::print(
                "\nPLACE_SWAPPING_PROFICITE frame={} label={} region=({},{})"
                " cell='{}' coord=({},{}) slack_ns={:.3f} bunch_cells={}",
                frame_index, label, region.x, region.y,
                entry.group->anchor->makeName(full_name_limit),
                entry.group->anchor->coord.x, entry.group->anchor->coord.y,
                entry.slack_ns, entry.group->members.size());
          }
        }
      }
    }
    if (!render)
      return;
    std::string filename = place.movementPngFilename(
        std::format("150_swap_{:03d}_{}", frame_index, label));
    // Diagnostic rendering must not consume the placement algorithm's runtime
    // budget. The puzzle test redraws every retained frame after it has selected
    // the final WNS markers.
    if (!defer_timing_map_png)
      place.drawPlacementSnapshot(place.movement_snapshot_cells, filename);
    place.captureMovementSnapshot(place.movement_snapshot_cells, filename);
    std::print("\nPLACE_SWAPPING_MAP_PNG file='{}' DEFICITE={} PROFICITE={}"
               " deferred={}",
               filename, place.movement_deficite_cells.size(),
               place.movement_proficite_cells.size(), defer_timing_map_png);
  };

  auto visitRectangleProficite = [&](const TimingCellMaps &maps, Coord first,
                                      Coord second, auto &&visitor) {
    int margin = std::max(0, config.proficite_rectangle_margin_tiles);
    Coord minimum_tile{
        std::max(0, std::min(first.x, second.x) - margin),
        std::max(0, std::min(first.y, second.y) - margin),
    };
    Coord maximum_tile{
        std::min(width - 1, std::max(first.x, second.x) + margin),
        std::min(height - 1, std::max(first.y, second.y) + margin),
    };
    Coord minimum_region = regionCoordinate(minimum_tile);
    Coord maximum_region = regionCoordinate(maximum_tile);
    int minimum_x = minimum_region.x;
    int maximum_x = maximum_region.x;
    int minimum_y = minimum_region.y;
    int maximum_y = maximum_region.y;
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const std::vector<ProficiteCell> &container =
            maps.proficite[regionIndex({x, y})];
        for (const ProficiteCell &entry : container) {
          if (entry.group && visitor(entry))
            return true;
        }
      }
    }
    return false;
  };

  auto minimumBunchSetupSlack = [](const PlaceTimingAnalysis &analysis,
                                   const RegBunch *bunch) {
    double minimum_slack = std::numeric_limits<double>::infinity();
    for (const PlaceTimingEndpoint &endpoint : analysis.endpoint_details) {
      bool touches = endpoint.data_in && endpoint.data_in->inst_ref.peer &&
                     endpoint.data_in->inst_ref->bunch_ref.peer == bunch;
      if (!touches) {
        touches = std::ranges::any_of(
            endpoint.critical_edges, [&](const PlaceTimingEdge &edge) {
              return (edge.driver && edge.driver->bunch_ref.peer == bunch) ||
                     (edge.sink && edge.sink->bunch_ref.peer == bunch);
            });
      }
      if (touches)
        minimum_slack = std::min(minimum_slack, endpoint.slack_ns);
    }
    return minimum_slack;
  };

  // Differential diagnostics only: retain the old filter on an identical
  // packed design without changing timing acceptance or candidate ordering.
  const bool rigid_projection_only =
      std::getenv("SCALEPNR_PLACE_SWAP_RIGID_PROJECTION_ONLY") != nullptr;
  auto projectedEndpointSlack = [&](const PlaceTimingEndpoint &endpoint,
                                    const BunchInfo &moving_group,
                                    Coord target_anchor,
                                    bool allow_follower_packing = false,
                                    bool compact_combs = false) {
    Coord displacement = target_anchor - moving_group.anchor->coord;
    auto projected = [&](rtl::Inst *inst) {
      Coord coordinate = inst ? inst->coord : Coord{-1, -1};
      if (inst && inst->bunch_ref.peer == moving_group.bunch) {
        coordinate = coordinate + displacement;
        if (compact_combs && inst != moving_group.anchor && !isClocked(*inst))
          coordinate = target_anchor;
        coordinate.x = std::clamp(coordinate.x, 0, width - 1);
        coordinate.y = std::clamp(coordinate.y, 0, height - 1);
      }
      return coordinate;
    };
    auto geometryDelay = [&](Coord sink, Coord driver) {
      int dx = std::abs(sink.x - driver.x);
      int dy = std::abs(sink.y - driver.y);
      return timing.calibration.local_wire_ns +
             dx * timing.calibration.horizontal_ns_per_tile +
             dy * timing.calibration.vertical_ns_per_tile +
             (dx != 0 && dy != 0 ? timing.calibration.bend_ns : 0.0);
    };
    double arrival_ns = endpoint.arrival_ns;
    for (const PlaceTimingEdge &edge : endpoint.critical_edges) {
      if (!edge.driver || !edge.sink)
        continue;
      Coord sink = projected(edge.sink), driver = projected(edge.driver);
      double projected_delay = geometryDelay(sink, driver);
      if (allow_follower_packing) {
        // The anchor is exact, but each follower may pack within an L1
        // radius of its translated target. Rigid translation is not a safe
        // rejection test for this actual relocation operation. Bound the
        // best possible wire delay before paying for a packing/timing trial.
        auto freedom = [&](rtl::Inst *inst) {
          return inst->bunch_ref.peer == moving_group.bunch &&
                         inst != moving_group.anchor
                     ? std::max(0, config.placement_radius) : 0;
        };
        int radius = freedom(edge.sink) + freedom(edge.driver);
        if (radius > 0) {
          int dx = std::abs(sink.x - driver.x);
          int dy = std::abs(sink.y - driver.y);
          int budget = radius;
          auto shorten = [&](int &distance) {
            int reduction = std::min(distance, budget);
            distance -= reduction;
            budget -= reduction;
          };
          bool unavoidable_bend = dx > radius && dy > radius;
          if (timing.calibration.horizontal_ns_per_tile >=
              timing.calibration.vertical_ns_per_tile) {
            shorten(dx);
            shorten(dy);
          } else {
            shorten(dy);
            shorten(dx);
          }
          projected_delay = timing.calibration.local_wire_ns +
              dx * timing.calibration.horizontal_ns_per_tile +
              dy * timing.calibration.vertical_ns_per_tile +
              (unavoidable_bend ? timing.calibration.bend_ns : 0.0);
        }
      }
      arrival_ns += projected_delay - geometryDelay(edge.sink->coord, edge.driver->coord);
    }
    return endpoint.required_ns - arrival_ns;
  };

  auto explainMidpointProficite = [&](const TimingCellMaps &maps,
                                      const PlaceTimingAnalysis &analysis,
                                      const PlaceTimingEndpoint *forced_endpoint = nullptr,
                                      rtl::Inst *forced_a = nullptr,
                                      rtl::Inst *forced_b = nullptr) {
    if (!std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_MIDPOINT"))
      return;
    const char *a_name = std::getenv("SCALEPNR_PLACING_MARKER_A");
    const char *b_name = std::getenv("SCALEPNR_PLACING_MARKER_B");
    auto findNamed = [&](const char *name) -> rtl::Inst * {
      if (!name || !*name)
        return nullptr;
      auto found = std::ranges::find_if(cells, [&](rtl::Inst *inst) {
        return inst && inst->makeName(full_name_limit) == name;
      });
      return found == cells.end() ? nullptr : *found;
    };
    rtl::Inst *a = forced_a ? forced_a : findNamed(a_name);
    rtl::Inst *b = forced_b ? forced_b : findNamed(b_name);
    if (!a && !b) {
      // ALPHA diagnostics must bind to the final WNS objects selected inside
      // this process. Requiring names from an earlier run made the diagnostic
      // vulnerable to a different (but valid) placement trajectory.
      a = tech->place.movement_marker_a;
      b = tech->place.movement_marker_b;
    }
    if (!a || !b || !a->tile.peer || !b->tile.peer) {
      std::print(
          "\nPLACE_SWAPPING_MIDPOINT result=missing_marker A='{}' B='{}'",
          a_name ? a_name : "", b_name ? b_name : "");
      return;
    }
    auto a_group = groups.find(a->bunch_ref.peer);
    auto b_group = groups.find(b->bunch_ref.peer);
    if (a_group == groups.end() && b_group == groups.end()) {
      std::print("\nPLACE_SWAPPING_MIDPOINT result=no_movable_marker_bunch");
      return;
    }
    RegBunch *a_bunch =
        a_group == groups.end() ? nullptr : a_group->second.bunch;
    RegBunch *b_bunch =
        b_group == groups.end() ? nullptr : b_group->second.bunch;

    Coord midpoint{(a->coord.x + b->coord.x) / 2,
                   (a->coord.y + b->coord.y) / 2};
    struct MiddleCandidate {
      const ProficiteCell *cell = nullptr;
      int midpoint_distance = 0;
    };
    std::vector<MiddleCandidate> selected;
    const bool explain_all =
        std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_ALL") != nullptr;
    visitRectangleProficite(maps, a->coord, b->coord,
                            [&](const ProficiteCell &entry) {
      if (entry.group->anchor && entry.group->bunch != a_bunch &&
          entry.group->bunch != b_bunch) {
        selected.push_back(
            {&entry, manhattan(entry.group->anchor->coord, midpoint)});
      }
      return false;
    });
    std::ranges::sort(selected, [](const MiddleCandidate &left,
                                   const MiddleCandidate &right) {
      if (left.midpoint_distance != right.midpoint_distance)
        return left.midpoint_distance < right.midpoint_distance;
      if (left.cell->slack_ns != right.cell->slack_ns)
        return left.cell->slack_ns > right.cell->slack_ns;
      return left.cell->stable_order < right.cell->stable_order;
    });
    if (const char *limit = std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_LIMIT")) {
      size_t count = std::strtoull(limit, nullptr, 10);
      if (count > 0 && selected.size() > count)
        selected.resize(count);
    }
    std::vector<ProficiteCell> forced_candidates;
    if (const char *requested =
            std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_CANDIDATES")) {
      std::vector<MiddleCandidate> requested_candidates;
      forced_candidates.reserve(64);
      std::string names = requested;
      for (size_t begin = 0; begin <= names.size();) {
        size_t end = names.find(',', begin);
        std::string name = names.substr(begin, end - begin);
        const ProficiteCell *found = nullptr;
        for (const std::vector<ProficiteCell> &container : maps.proficite) {
          auto entry = std::ranges::find_if(
              container, [&](const ProficiteCell &candidate) {
                return candidate.group && candidate.group->anchor &&
                       candidate.group->anchor->makeName(full_name_limit) ==
                           name;
              });
          if (entry != container.end()) {
            found = &*entry;
            break;
          }
        }
        if (found) {
          Coord coordinate = found->group->anchor->coord;
          requested_candidates.push_back(
              {found, manhattan(coordinate, midpoint)});
        } else {
          rtl::Inst *requested_cell = findNamed(name.c_str());
          auto requested_group =
              requested_cell
                  ? groups.find(requested_cell->bunch_ref.peer)
                  : groups.end();
          if (requested_group != groups.end() && requested_group->second.anchor &&
              !requested_group->second.fixed) {
            auto stable = stable_cell_order.find(requested_group->second.anchor);
            forced_candidates.push_back(
                {&requested_group->second,
                 minimumBunchSetupSlack(analysis,
                                        requested_group->second.bunch),
                 stable == stable_cell_order.end()
                     ? std::numeric_limits<size_t>::max()
                     : stable->second});
            found = &forced_candidates.back();
            Coord coordinate = found->group->anchor->coord;
            requested_candidates.push_back(
                {found, manhattan(coordinate, midpoint)});
            std::print(
                "\nPLACE_SWAPPING_MIDPOINT requested_C='{}' result="
                "forced_non_PROFICITE setup_slack_ns={:.3f}",
                name, found->slack_ns);
          } else {
            std::print(
                "\nPLACE_SWAPPING_MIDPOINT requested_C='{}' result="
                "missing_or_fixed",
                name);
          }
        }
        if (end == std::string::npos)
          break;
        begin = end + 1;
      }
      selected = std::move(requested_candidates);
    }
    constexpr size_t candidates_to_explain = 6;
    if (!explain_all && selected.size() > candidates_to_explain)
      selected.resize(candidates_to_explain);
    if (selected.empty()) {
      std::print("\nPLACE_SWAPPING_MIDPOINT result=no_PROFICITE_candidate");
      return;
    }

    const PlaceTimingEndpoint *endpoint = forced_endpoint;
    if (!endpoint) {
      for (const PlaceTimingEndpoint &candidate : analysis.endpoint_details) {
        bool contains_pair = std::ranges::any_of(
            candidate.critical_edges, [&](const PlaceTimingEdge &edge) {
              return (edge.driver == a && edge.sink == b) ||
                     (edge.driver == b && edge.sink == a);
            });
        if (contains_pair &&
            (!endpoint || candidate.slack_ns < endpoint->slack_ns)) {
          endpoint = &candidate;
        }
      }
    }
    if (!endpoint) {
      for (const PlaceTimingEndpoint &candidate : analysis.endpoint_details) {
      rtl::Inst *sink =
          candidate.data_in ? candidate.data_in->inst_ref.peer : nullptr;
      if (sink != b)
        continue;
      bool contains_a = std::ranges::any_of(
          candidate.critical_edges, [&](const PlaceTimingEdge &edge) {
            return edge.driver == a || edge.sink == a;
          });
      if (!endpoint || contains_a || candidate.slack_ns < endpoint->slack_ns)
        endpoint = &candidate;
      if (contains_a)
        break;
      }
    }

    rtl::Inst *search_path_a = a;
    rtl::Inst *search_path_b = b;
    Coord actual_search_from = search_path_a->coord;
    Coord actual_search_to = search_path_b->coord;
    std::print(
        "\nPLACE_SWAPPING_MIDPOINT_SEARCH from='{}' coord=({},{}) to='{}' "
        "coord=({},{}) A_B=({},{})=>({},{})",
        search_path_a ? search_path_a->makeName(full_name_limit)
                      : std::string{"<none>"},
        actual_search_from.x, actual_search_from.y,
        search_path_b ? search_path_b->makeName(full_name_limit)
                      : std::string{"<none>"},
        actual_search_to.x, actual_search_to.y, a->coord.x, a->coord.y,
        b->coord.x, b->coord.y);

    // Use the same all-branch dependency index as exact local recovery when
    // reporting A/B/C setup limits, including formerly noncritical paths.
    PlaceTimingAnalysis diagnostic_state = analysis;
    PlaceTimingIncremental diagnostic_index(timing, diagnostic_state);
    struct ScanCounts {
      size_t tested = 0, packed = 0, path_improved = 0;
      size_t strict_good = 0, safe_good = 0, policy_good = 0, full_verified = 0;
    } scan_counts[4];
    const auto original_fingerprint = calculatePlacementFingerprint();
    auto diagnosticBunchSlack = [&](const PlaceTimingAnalysis &state,
                                     RegBunch *bunch) {
      double slack = std::numeric_limits<double>::infinity();
      auto group = groups.find(bunch);
      if (group == groups.end()) return slack;
      for (rtl::Inst *cell : group->second.members) {
        auto found = diagnostic_index.endpoints_by_cell.find(cell);
        if (found == diagnostic_index.endpoints_by_cell.end()) continue;
        for (size_t index : found->second)
          slack = std::min(slack, state.endpoint_details[index].slack_ns);
      }
      return slack;
    };
    auto explainOne = [&](const char *label, BunchInfo &moving,
                          rtl::Inst *marker,
                          const ProficiteCell &selected_cell,
                          bool compact_combs = false) {
      BunchInfo &challenger = *selected_cell.group;
      ScanCounts &counts = scan_counts[(label[0] == 'A' ? 0 : 1) +
                                      (compact_combs ? 2 : 0)];
      ++counts.tested;
      if (!endpoint) {
        std::print(
            "\nPLACE_SWAPPING_MIDPOINT_SWAP side={} result=no_B_endpoint",
            label);
        return;
      }
      bool marker_on_path = std::ranges::any_of(
          endpoint->critical_edges, [&](const PlaceTimingEdge &edge) {
            return edge.driver == marker || edge.sink == marker;
          });
      rtl::Inst *adjacent = nullptr;
      for (const PlaceTimingEdge &edge : endpoint->critical_edges) {
        if (edge.driver == marker)
          adjacent = edge.sink;
        else if (edge.sink == marker)
          adjacent = edge.driver;
        if (adjacent)
          break;
      }
      Coord timing_target =
          replacementOrigin(moving, adjacent ? adjacent->coord : b->coord);
      bool search_visible = false;
      visitRectangleProficite(
          maps, actual_search_from, actual_search_to,
          [&](const ProficiteCell &entry) {
            search_visible = entry.group &&
                             entry.group->bunch == challenger.bunch;
            return search_visible;
          });
      if (moving.fixed || challenger.fixed ||
          moving.bunch == challenger.bunch) {
        std::print("\nPLACE_SWAPPING_MIDPOINT_SWAP side={} result=ineligible"
                   " marker_on_path={} moving_fixed={} C_fixed={} same_bunch={}"
                   " timing_target=({},{}) C_search_visible={}",
                   label, marker_on_path, moving.fixed, challenger.fixed,
                   moving.bunch == challenger.bunch,
                   timing_target.x,
                   timing_target.y, search_visible);
        return;
      }

      std::vector<BunchSnapshot> original_bunches{
          {moving.bunch, moving.bunch->x, moving.bunch->y},
          {challenger.bunch, challenger.bunch->x, challenger.bunch->y},
      };
      Coord marker_before = marker->coord;
      Coord c_before = challenger.anchor->coord;
      double projected_slack_ns =
          projectedEndpointSlack(*endpoint, moving, c_before, false, compact_combs);
      double packing_bound_slack_ns =
          projectedEndpointSlack(*endpoint, moving, c_before, true, compact_combs);
      SwapAttemptResult attempt = attemptRelocation(moving, challenger, true, compact_combs);
      if (!attempt.packed) {
        std::print(
            "\nPLACE_SWAPPING_MIDPOINT_SWAP side={} result=pack_failed"
            " marker_on_path={} timing_target=({},{}) C_search_visible={}"
            " moving_bunch_cells={} C_bunch_cells={}",
            label, marker_on_path, timing_target.x, timing_target.y,
            search_visible, moving.members.size(), challenger.members.size());
        for (const SwapAttemptResult::PackingFailure &failure :
             attempt.failures) {
          std::print(
              "\nPLACE_SWAPPING_PACK_FAILURE side={} C='{}' failed='{}'"
              " type='{}' role={} anchor={} desired=({},{}) radius={}",
              label,
              challenger.anchor->makeName(full_name_limit),
              failure.inst ? failure.inst->makeName(full_name_limit)
                           : std::string{"<none>"},
              failure.inst && failure.inst->cell_ref.peer
                  ? failure.inst->cell_ref->type
                  : std::string{"<none>"},
              failure.moving_group ? "moving" : "challenger",
              failure.anchor, failure.desired.x, failure.desired.y,
              failure.radius);
        }

        // First answer the literal question: with only marker and C removed,
        // can this individual A/B cell occupy any compatible element in C's
        // original Tile? The rest of both bunches stays untouched.
        std::vector<PlacementSnapshot> direct_snapshots;
        std::unordered_set<rtl::Inst *> direct_cells;
        auto saveDirect = [&](rtl::Inst *inst) {
          if (!inst || !inst->tile.peer || !direct_cells.insert(inst).second)
            return;
          direct_snapshots.push_back({
              .inst = inst,
              .tile = inst->tile.peer,
              .coord = inst->coord,
              .pos = inst->pos,
              .outline_x = inst->outline.x,
              .outline_y = inst->outline.y,
          });
        };
        saveDirect(marker);
        saveDirect(challenger.anchor);
        fpga::Tile *c_tile = challenger.anchor->tile.peer;
        int c_pos = challenger.anchor->pos;
        for (const PlacementSnapshot &snapshot : direct_snapshots)
          snapshot.inst->tile->unassign(snapshot.inst);
        int direct_pos = c_tile ? c_tile->tryAdd(marker, false) : -1;
        bool direct_packed = direct_pos >= 0;
        Coord direct_coord = direct_packed ? marker->coord : Coord{-1, -1};
        restore(direct_snapshots, {});
        std::print(
            "\nPLACE_SWAPPING_C_ONLY_PROBE side={} marker='{}' type='{}'"
            " C='{}' C_type='{}' C_original=({},{}) C_pos={}"
            " individual_packed={} packed_coord=({},{}) packed_pos={}",
            label, marker->makeName(full_name_limit), marker->cell_ref->type,
            challenger.anchor->makeName(full_name_limit),
            challenger.anchor->cell_ref->type, c_before.x, c_before.y, c_pos,
            direct_packed, direct_coord.x, direct_coord.y, direct_pos);

        // Then remove C and test the real movable unit: all cells in A/B's
        // bunch translated together to C. C is deliberately left unplaced;
        // this separates moving-bunch capacity from replacement failure.
        SwapAttemptResult moving_only =
            attemptRelocation(moving, challenger, false, compact_combs);
        std::print(
            "\nPLACE_SWAPPING_C_ONLY_GROUP_PROBE side={} marker='{}' C='{}'"
            " moving_bunch_cells={} packed={}",
            label, marker->makeName(full_name_limit),
            challenger.anchor->makeName(full_name_limit),
            moving.members.size(), moving_only.packed);
        for (const SwapAttemptResult::PackingFailure &failure :
             moving_only.failures) {
          std::print(
              "\nPLACE_SWAPPING_C_ONLY_GROUP_FAILURE side={} failed='{}'"
              " type='{}' role={} anchor={} desired=({},{}) radius={}",
              label,
              failure.inst ? failure.inst->makeName(full_name_limit)
                           : std::string{"<none>"},
              failure.inst && failure.inst->cell_ref.peer
                  ? failure.inst->cell_ref->type
                  : std::string{"<none>"},
              failure.moving_group ? "moving" : "challenger",
              failure.anchor, failure.desired.x, failure.desired.y,
              failure.radius);
        }
        if (moving_only.packed)
          restore(moving_only.snapshots, original_bunches);
        return;
      }

      double estimated_arrival_ns = endpoint->arrival_ns;
      for (const PlaceTimingEdge &edge : endpoint->critical_edges) {
        if (!edge.sink_input || !edge.driver_output)
          continue;
        estimated_arrival_ns +=
            timing.estimateWireDelay(*edge.sink_input, *edge.driver_output) -
            edge.wire_delay_ns;
      }
      double estimated_slack_ns = endpoint->required_ns - estimated_arrival_ns;
      size_t diagnostic_pass = result.passes == 0 ? 0 : result.passes - 1;
      double diagnostic_temperature =
          temperatureForPass(result.initial_temperature, diagnostic_pass);
      double before_deficit = std::max(0.0, -endpoint->slack_ns);
      double estimated_after_deficit =
          std::max(0.0, -estimated_slack_ns);
      double estimated_improvement =
          before_deficit > epsilon
              ? (before_deficit - estimated_after_deficit) / before_deficit
              : 0;
      std::uint64_t fingerprint = calculatePlacementFingerprint();
      bool previously_visited = visited_placements.contains(fingerprint);
      ++counts.packed;
      PlaceTimingIncremental::Transaction transaction;
      PlaceTimingAnalysis full_candidate;
      if (explain_all) {
        std::vector<rtl::Inst *> changed;
        for (const auto &snapshot : attempt.snapshots)
          changed.push_back(snapshot.inst);
        transaction = diagnostic_index.update(changed);
      } else {
        full_candidate = timing.analyze(timings);
      }
      const PlaceTimingAnalysis &candidate =
          explain_all ? diagnostic_state : full_candidate;
      const PlaceTimingEndpoint *candidate_endpoint =
          findEndpoint(candidate, endpoint->data_in);
      double after_deficit =
          candidate_endpoint ? std::max(0.0, -candidate_endpoint->slack_ns)
                             : before_deficit;
      double improvement =
          before_deficit > epsilon
              ? (before_deficit - after_deficit) / before_deficit
              : 0;
      double challenger_slack =
          diagnosticBunchSlack(candidate, challenger.bunch);
      double first_slack_before = a_bunch
                                      ? diagnosticBunchSlack(analysis, a_bunch)
                                      : std::numeric_limits<double>::infinity();
      double second_slack_before = b_bunch
                                       ? diagnosticBunchSlack(analysis, b_bunch)
                                       : std::numeric_limits<double>::infinity();
      double challenger_slack_before =
          diagnosticBunchSlack(analysis, challenger.bunch);
      double first_slack_after = a_bunch
                                     ? diagnosticBunchSlack(candidate, a_bunch)
                                     : first_slack_before;
      double second_slack_after = b_bunch
                                      ? diagnosticBunchSlack(candidate, b_bunch)
                                      : second_slack_before;
      double first_slack_limit =
          criticalSlackLimitForPass(first_slack_before,
                                    result.initial_temperature,
                                    diagnostic_pass);
      double second_slack_limit =
          criticalSlackLimitForPass(second_slack_before,
                                    result.initial_temperature,
                                    diagnostic_pass);
      double challenger_slack_limit =
          challengerSlackLimitForPass(challenger_slack_before,
                                      result.initial_temperature,
                                      diagnostic_pass);
      bool relaxed = false;
      bool global_ok = acceptsTimingTradeoff(improvement, analysis, candidate,
                                             &relaxed, &best_timing_limits);
      double projected_improvement = before_deficit > epsilon
          ? (before_deficit - std::max(0.0, -projected_slack_ns)) / before_deficit
          : 0;
      bool projected_ok =
          projected_improvement + epsilon >= config.minimum_improvement;
      double packing_bound_improvement = before_deficit > epsilon
          ? (before_deficit - std::max(0.0, -packing_bound_slack_ns)) / before_deficit
          : 0;
      bool packing_bound_ok = packing_bound_improvement + epsilon >=
                              config.minimum_improvement;
      bool challenger_ok = challenger_slack + epsilon >= challenger_slack_limit;
      bool endpoints_temperature_ok =
          first_slack_after + epsilon >= first_slack_limit &&
          second_slack_after + epsilon >= second_slack_limit;
      bool endpoint_ok = improvement + epsilon >= config.minimum_improvement;
      // A stable minimum can hide degradation of another C-dependent path.
      // The broad diagnostic checks every affected endpoint independently.
      std::unordered_set<size_t> c_endpoints;
      for (rtl::Inst *cell : challenger.members) {
        auto found = diagnostic_index.endpoints_by_cell.find(cell);
        if (found != diagnostic_index.endpoints_by_cell.end())
          c_endpoints.insert(found->second.begin(), found->second.end());
      }
      size_t c_degraded = 0, c_new_violations = 0, c_worsened_violations = 0;
      for (size_t index : c_endpoints) {
        double before = analysis.endpoint_details[index].slack_ns;
        double after = candidate.endpoint_details[index].slack_ns;
        c_degraded += after + epsilon < before;
        c_new_violations += before >= -epsilon && after < -epsilon;
        c_worsened_violations += after < -epsilon && after + epsilon < before;
      }
      bool strict_good = endpoint_ok && endpoints_temperature_ok &&
          c_degraded == 0 &&
          candidate.worst_slack_ns + epsilon >= analysis.worst_slack_ns &&
          candidate.total_negative_slack_ns <=
              analysis.total_negative_slack_ns + epsilon;
      bool policy_good = endpoint_ok && endpoints_temperature_ok &&
                         challenger_ok && global_ok;
      bool safe_good = endpoint_ok && endpoints_temperature_ok &&
          c_worsened_violations == 0 &&
          candidate.worst_slack_ns + epsilon >= analysis.worst_slack_ns &&
          candidate.total_negative_slack_ns <=
              analysis.total_negative_slack_ns + epsilon;
      counts.path_improved += endpoint_ok;
      counts.strict_good += strict_good;
      counts.safe_good += safe_good;
      counts.policy_good += policy_good;
      bool full_verified = !explain_all;
      if (explain_all && safe_good && counts.full_verified < 8) {
        auto exact = timing.analyze(timings);
        PNR_ASSERT(exact.endpoint_details.size() == candidate.endpoint_details.size(),
                   "broad diagnostic endpoint count changed");
        for (size_t index = 0; index < exact.endpoint_details.size(); ++index)
          PNR_ASSERT(std::abs(exact.endpoint_details[index].slack_ns -
                             candidate.endpoint_details[index].slack_ns) < 1e-7,
                     "broad diagnostic local endpoint timing differs from full analysis");
        PNR_ASSERT(std::abs(exact.worst_slack_ns - candidate.worst_slack_ns) < 1e-7 &&
                       std::abs(exact.total_negative_slack_ns -
                                candidate.total_negative_slack_ns) < 1e-7,
                   "broad diagnostic timing totals differ from full analysis");
        ++counts.full_verified;
        full_verified = true;
      }
      const char *reason = !endpoints_temperature_ok ? "A_B_temperature"
                           : !packing_bound_ok ? "projected_endpoint_improvement"
                           : previously_visited ? "visited_placement"
                           : !challenger_ok     ? "C_timing"
                           : !endpoint_ok       ? "endpoint_improvement"
                           : !global_ok         ? "global_timing"
                                                : "would_accept";
      if (std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_PATHS")) {
        auto originalCoord = [&](rtl::Inst *inst) {
          auto saved = std::ranges::find(attempt.snapshots, inst,
                                         &PlacementSnapshot::inst);
          return saved == attempt.snapshots.end() ? inst->coord : saved->coord;
        };
        auto printPath = [&](const char *phase,
                             const PlaceTimingEndpoint *path_endpoint,
                             bool original_geometry) {
          if (!path_endpoint)
            return;
          double edge_sum_ns = 0;
          for (size_t edge_index = 0;
               edge_index < path_endpoint->critical_edges.size();
               ++edge_index) {
            const PlaceTimingEdge &path_edge =
                path_endpoint->critical_edges[edge_index];
            if (!path_edge.driver || !path_edge.sink)
              continue;
            Coord driver_coord = original_geometry
                                     ? originalCoord(path_edge.driver)
                                     : path_edge.driver->coord;
            Coord sink_coord = original_geometry
                                   ? originalCoord(path_edge.sink)
                                   : path_edge.sink->coord;
            int dx = std::abs(driver_coord.x - sink_coord.x);
            int dy = std::abs(driver_coord.y - sink_coord.y);
            edge_sum_ns += path_edge.wire_delay_ns;
            std::print(
                "\nPLACE_SWAPPING_EXACT_PATH side={} C='{}' phase={} "
                "edge={} driver='{}' coord=({},{}) sink='{}' "
                "coord=({},{}) dx={} dy={} manhattan={} wire_delay_ns={:.3f}",
                label, challenger.anchor->makeName(full_name_limit), phase,
                edge_index,
                path_edge.driver->makeName(full_name_limit), driver_coord.x,
                driver_coord.y, path_edge.sink->makeName(full_name_limit),
                sink_coord.x, sink_coord.y, dx, dy, dx + dy,
                path_edge.wire_delay_ns);
          }
          std::print(
              "\nPLACE_SWAPPING_EXACT_PATH_SUMMARY side={} C='{}' phase={} "
              "required_ns={:.3f} arrival_ns={:.3f} slack_ns={:.3f} "
              "critical_edges={} edge_wire_sum_ns={:.3f}",
              label, challenger.anchor->makeName(full_name_limit), phase,
              path_endpoint->required_ns, path_endpoint->arrival_ns,
              path_endpoint->slack_ns, path_endpoint->critical_edges.size(),
              edge_sum_ns);
        };
        printPath("before", endpoint, true);
        printPath("after", candidate_endpoint, false);
      }
      std::print(
          "\nPLACE_SWAPPING_MIDPOINT_SWAP side={} result={} compact_combs={}"
          " marker_on_path={} timing_target=({},{}) C_search_visible={}"
          " marker=({},{})=>({},{}) C=({},{})=>({},{})"
          " endpoint_slack_ns={:.3f}->{:.3f} estimated_slack_ns={:.3f}"
          " endpoint_improvement={:.1f}% estimated_improvement={:.1f}%"
          " projected_slack_ns={:.3f} projected_improvement={:.1f}% projected_ok={}"
          " packing_bound_slack_ns={:.3f} packing_bound_ok={}"
          " A_slack_ns={:.3f} A_limit_ns={:.3f}"
          " B_slack_ns={:.3f} B_limit_ns={:.3f}"
          " TEMPERATURE={:.3f} C_slack_ns={:.3f} C_limit_ns={:.3f}"
          " C_before_slack_ns={:.3f} C_no_degradation={}"
          " WNS_ns={:.3f}->{:.3f} TNS_ns={:.3f}->{:.3f}"
          " global_ok={} relaxed={} C_name='{}'"
          " C_degraded_paths={} C_new_violations={} strict_good={}"
          " C_worsened_violations={} safe_good={} policy_good={} full_verified={}",
          label, reason, compact_combs, marker_on_path, timing_target.x, timing_target.y,
          search_visible, marker_before.x, marker_before.y, marker->coord.x,
          marker->coord.y, c_before.x, c_before.y, challenger.anchor->coord.x,
          challenger.anchor->coord.y, endpoint->slack_ns,
          candidate_endpoint ? candidate_endpoint->slack_ns
                             : endpoint->slack_ns,
          estimated_slack_ns, 100.0 * improvement,
          100.0 * estimated_improvement, projected_slack_ns,
          100.0 * projected_improvement, projected_ok,
          packing_bound_slack_ns, packing_bound_ok,
          first_slack_after, first_slack_limit,
          second_slack_after, second_slack_limit, diagnostic_temperature,
          challenger_slack, challenger_slack_limit,
          challenger_slack_before,
          challenger_slack + epsilon >= challenger_slack_before,
          analysis.worst_slack_ns, candidate.worst_slack_ns,
          analysis.total_negative_slack_ns, candidate.total_negative_slack_ns,
          global_ok, relaxed, challenger.anchor->makeName(full_name_limit),
          c_degraded, c_new_violations, strict_good,
          c_worsened_violations, safe_good, policy_good, full_verified);
      if (explain_all)
        diagnostic_index.restore(std::move(transaction));
      restore(attempt.snapshots, original_bunches);
    };

    for (size_t index = 0; index < selected.size(); ++index) {
      const MiddleCandidate &candidate = selected[index];
      BunchInfo &challenger = *candidate.cell->group;
      std::print(
          "\nPLACE_SWAPPING_MIDPOINT candidate={} A='{}' coord=({},{})"
          " B='{}' coord=({},{}) midpoint=({},{}) C='{}' coord=({},{})"
          " midpoint_distance={}"
          " C_PROFICITE_slack_ns={:.3f} C_bunch_cells={}"
          " endpoint_slack_ns={:.3f}",
          index + 1, a->makeName(full_name_limit), a->coord.x, a->coord.y,
          b->makeName(full_name_limit), b->coord.x, b->coord.y, midpoint.x,
          midpoint.y, challenger.anchor->makeName(full_name_limit),
          challenger.anchor->coord.x, challenger.anchor->coord.y,
          candidate.midpoint_distance, candidate.cell->slack_ns,
          challenger.members.size(),
          endpoint ? endpoint->slack_ns
                   : std::numeric_limits<double>::quiet_NaN());
      if (a_group != groups.end())
        explainOne("A", a_group->second, a, *candidate.cell);
      else
        std::print("\nPLACE_SWAPPING_MIDPOINT_SWAP side=A "
                   "result=fixed_or_non_bunch_endpoint");
      if (b_group != groups.end())
        explainOne("B", b_group->second, b, *candidate.cell);
      else
        std::print("\nPLACE_SWAPPING_MIDPOINT_SWAP side=B "
                   "result=fixed_or_non_bunch_endpoint");
      if (std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_COMPACT")) {
        if (a_group != groups.end())
          explainOne("A", a_group->second, a, *candidate.cell, true);
        if (b_group != groups.end())
          explainOne("B", b_group->second, b, *candidate.cell, true);
      }
    }
    if (explain_all) {
      PNR_ASSERT(calculatePlacementFingerprint() == original_fingerprint,
                 "broad diagnostic did not restore placement");
      auto restored = timing.analyze(timings);
      for (size_t index = 0; index < analysis.endpoint_details.size(); ++index)
        PNR_ASSERT(std::abs(restored.endpoint_details[index].slack_ns -
                           analysis.endpoint_details[index].slack_ns) < 1e-7,
                   "broad diagnostic did not restore timing");
      for (int side = 0; side < 4; ++side) {
        const auto &counts = scan_counts[side];
        if (!counts.tested)
          continue;
        std::print("\nPLACE_SWAPPING_BROAD_SCAN side={} candidates={} tested={}"
                   " packed={} path_improved={} strict_good={} safe_good={} policy_good={}"
                   " full_verified={} restored=true compact_combs={}",
                   side % 2 == 0 ? "A" : "B", selected.size(), counts.tested,
                   counts.packed, counts.path_improved, counts.strict_good, counts.safe_good,
                   counts.policy_good, counts.full_verified, side >= 2);
      }
    }
  };

  for (size_t pass = 0;
       pass < config.maximum_passes && current.violated_endpoints != 0 &&
       !completionReached(current) && !timedOut();
       ++pass) {
    ++result.passes;
    size_t pass_attempts_before = result.attempts;
    size_t pass_accepted_before = result.accepted_swaps;
    double pass_worst_before = current.worst_slack_ns;
    double pass_tns_before = current.total_negative_slack_ns;
    bool pass_improved = false;
    bool pass_has_provisional_swaps = false;
    bool pass_used_relaxed_rule = false;
    bool pass_acceptance_cap_reached = false;
    bool pass_recovery_reserve_reached = false;
    bool pass_exact_temperature_ok = true;
    double pass_temperature =
        temperatureForPass(result.initial_temperature, pass);
    double pass_ab_slack_limit = -pass_temperature;
    double pass_strongest_improvement = 0;
    size_t pass_history_start = accepted_history.size();
    std::unordered_map<RegBunch *, double> pass_temperature_limits;
    auto acceptedSwapCapReached = [&] {
      return accepted_history.size() - pass_history_start >=
             config.maximum_accepted_swaps_per_pass;
    };
    auto finishProvisionalWork = [&] {
      if (!recoveryReserveReached())
        return false;
      pass_recovery_reserve_reached = true;
      return true;
    };

    // One timing/map snapshot drives the complete pass. Accepted swaps update
    // only timing cones touching A, B, or C; the timing forest and
    // maps are not rebuilt until the complete pass is validated.
    PlaceTimingAnalysis local_analysis = current;
    TimingCellMaps maps = buildTimingCellMaps(local_analysis);
    std::unordered_map<RegBunch *, std::vector<PlaceTimingEndpoint *>>
        setup_endpoints_by_bunch;
    setup_endpoints_by_bunch.reserve(groups.size());
    for (PlaceTimingEndpoint &setup_endpoint :
         local_analysis.endpoint_details) {
      std::unordered_set<RegBunch *> endpoint_bunches;
      auto account = [&](rtl::Inst *inst) {
        if (inst && inst->bunch_ref.peer)
          endpoint_bunches.insert(inst->bunch_ref.peer);
      };
      account(setup_endpoint.data_in
                  ? setup_endpoint.data_in->inst_ref.peer
                  : nullptr);
      for (rtl::Inst *dependency : timing.setupDependencies(setup_endpoint))
        account(dependency);
      for (RegBunch *bunch : endpoint_bunches)
        setup_endpoints_by_bunch[bunch].push_back(&setup_endpoint);
    }
    struct LocalEndpointSnapshot {
      PlaceTimingEndpoint *endpoint = nullptr;
      double arrival_ns = 0;
      double slack_ns = 0;
      std::vector<PlaceTimingEdge> critical_edges;
    };
    auto affectedSetupEndpoints =
        [&](RegBunch *first, RegBunch *second, RegBunch *third) {
          std::vector<PlaceTimingEndpoint *> affected;
          std::unordered_set<PlaceTimingEndpoint *> seen;
          for (RegBunch *bunch : {first, second, third}) {
            auto found = setup_endpoints_by_bunch.find(bunch);
            if (found == setup_endpoints_by_bunch.end())
              continue;
            for (PlaceTimingEndpoint *setup_endpoint : found->second) {
              if (seen.insert(setup_endpoint).second)
                affected.push_back(setup_endpoint);
            }
          }
          return affected;
        };
    auto minimumSetupSlack = [&](RegBunch *bunch) {
      double slack = std::numeric_limits<double>::infinity();
      auto found = setup_endpoints_by_bunch.find(bunch);
      if (found == setup_endpoints_by_bunch.end())
        return slack;
      for (const PlaceTimingEndpoint *setup_endpoint : found->second)
        slack = std::min(slack, setup_endpoint->slack_ns);
      return slack;
    };
    auto snapshotLocalTiming = [](const std::vector<PlaceTimingEndpoint *> &eps) {
      std::vector<LocalEndpointSnapshot> snapshots;
      snapshots.reserve(eps.size());
      for (PlaceTimingEndpoint *setup_endpoint : eps) {
        LocalEndpointSnapshot &snapshot = snapshots.emplace_back();
        snapshot.endpoint = setup_endpoint;
        snapshot.arrival_ns = setup_endpoint->arrival_ns;
        snapshot.slack_ns = setup_endpoint->slack_ns;
        snapshot.critical_edges = setup_endpoint->critical_edges;
      }
      return snapshots;
    };
    auto restoreLocalTiming = [](const std::vector<LocalEndpointSnapshot> &ss) {
      for (const LocalEndpointSnapshot &snapshot : ss) {
        snapshot.endpoint->arrival_ns = snapshot.arrival_ns;
        snapshot.endpoint->slack_ns = snapshot.slack_ns;
        snapshot.endpoint->critical_edges = snapshot.critical_edges;
      }
    };
    if (timing_map_frame == 0)
      captureTimingMapFrame(maps, "initial");
    result.deficite_cells = maps.deficite.size();
    result.proficite_cells = maps.proficite_cells;
    result.proficite_regions = maps.proficite.size();
    result.proficite_regions_relaxed = maps.relaxed_regions;
    std::print("\nPLACE_SWAPPING_MAP pass={} DEFICITE={} PROFICITE={} "
               "regions={} relaxed_to_{:.3f}ns={}",
               pass + 1, maps.deficite.size(), maps.proficite_cells,
               maps.proficite.size(), config.minimum_proficite_slack_ns,
               maps.relaxed_regions);

    if (maps.proficite_cells == 0)
      break;

    if (maps.deficite.empty()) {
      std::print("\nPLACE_SWAPPING_PASS pass={} TEMPERATURE={:.3f} "
                 "A_B_limit_ns={:.3f} attempts=0 provisional=0 accepted=0 "
                 "DEFICITE=0 PROFICITE={} worst_slack_ns={:.3f}->{:.3f} "
                 "tns_ns={:.3f}->{:.3f} full_analysis=false "
                 "temperature_ok=true relaxed=false "
                 "acceptance_cap_reached=false",
                 pass + 1, pass_temperature, pass_ab_slack_limit,
                 maps.proficite_cells, pass_worst_before, pass_worst_before,
                 pass_tns_before, pass_tns_before);
      break;
    }

    if (trace_a && trace_b) {
      auto traced =
          std::ranges::find_if(maps.deficite, [&](const DeficiteCell &entry) {
            return std::ranges::any_of(
                entry.endpoint->critical_edges,
                [&](const PlaceTimingEdge &edge) {
                  if (!edge.driver || !edge.sink)
                    return false;
                  std::string driver = edge.driver->makeName(full_name_limit);
                  std::string sink = edge.sink->makeName(full_name_limit);
                  return (driver.find(trace_a) != std::string::npos &&
                          sink.find(trace_b) != std::string::npos) ||
                         (driver.find(trace_b) != std::string::npos &&
                          sink.find(trace_a) != std::string::npos);
                });
          });
      if (traced != maps.deficite.end())
        std::rotate(maps.deficite.begin(), traced, std::next(traced));
    }
    size_t pass_deficite = maps.deficite.size();
    for (const DeficiteCell &deficite : maps.deficite) {
      if (finishProvisionalWork())
        break;
      if (acceptedSwapCapReached()) {
        pass_acceptance_cap_reached = true;
        break;
      }
      rtl::Inst *deficite_cell = deficite.cell;
      if (timedOut()) {
        result.timed_out = true;
        break;
      }
      PlaceTimingEndpoint *endpoint = deficite.endpoint;
      // The worklist is frozen for this pass. Timing for its endpoints is
      // corrected as cells move. TEMPERATURE affects acceptance limits only;
      // it never filters membership in the DEFICITE worklist.
      bool endpoint_accepted = false;
      ++result.violated_endpoints_examined;
      if (log_proficite_cells || std::getenv("SCALEPNR_PLACE_SWAP_AUDIT_WORST")) {
        std::print("\nPLACE_SWAPPING_ALPHA_VISIT pass={} endpoint='{}' "
                   "slack_ns={:.3f}", pass + 1,
                   deficite_cell->makeName(full_name_limit), endpoint->slack_ns);
      }
      std::vector<PlaceTimingEdge> edges;
      for (const PlaceTimingEdge &edge : endpoint->critical_edges) {
        if (edge.driver && edge.sink)
          edges.push_back(edge);
      }
      std::ranges::sort(edges, std::greater{}, &PlaceTimingEdge::wire_delay_ns);
      if (edges.size() > config.maximum_critical_edges_per_endpoint)
        edges.resize(config.maximum_critical_edges_per_endpoint);
      // Exhaust existing translations first. Only a still-unrepaired
      // endpoint reaches the second traversal, which can repack its combs
      // and therefore also repair edges internal to a single bunch.
      const size_t search_modes = config.repack_combinational_fallback ? 2 : 1;
      for (size_t edge_visit = 0; edge_visit < edges.size() * search_modes; ++edge_visit) {
        bool compact_combs = edge_visit >= edges.size();
        if (compact_combs && (pass_recovery_reserve_reached || result.timed_out))
          break;
        const PlaceTimingEdge &edge_copy = edges[edge_visit % edges.size()];
        const PlaceTimingEdge *edge = &edge_copy;
        if (endpoint_accepted || pass_acceptance_cap_reached ||
            finishProvisionalWork() || timedOut())
          break;
        auto first_group = groups.find(edge->driver->bunch_ref.peer);
        auto second_group = groups.find(edge->sink->bunch_ref.peer);
        if (first_group == groups.end() || second_group == groups.end()) {
          ++result.skipped_missing_bunch_edges;
          continue;
        }
        if (first_group == second_group && !compact_combs) {
          ++result.skipped_same_bunch_edges;
          continue;
        }

        std::string driver_name = edge->driver->makeName(full_name_limit);
        std::string sink_name = edge->sink->makeName(full_name_limit);
        bool trace_edge = trace_a && trace_b &&
                          ((driver_name.find(trace_a) != std::string::npos &&
                            sink_name.find(trace_b) != std::string::npos) ||
                           (driver_name.find(trace_b) != std::string::npos &&
                            sink_name.find(trace_a) != std::string::npos));

        bool available_sides[2] = {true, true};
        for (int side = 0; side < 2; ++side) {
          BunchInfo &moving_group =
              side == 0 ? first_group->second : second_group->second;
          if (moving_group.fixed) {
            ++result.skipped_fixed_moving_sides;
            available_sides[side] = false;
          }
          if (compact_combs &&
              ((side == 1 && first_group == second_group) ||
               !std::ranges::any_of(moving_group.members, [&](rtl::Inst *inst) {
                 return inst != moving_group.anchor && !isClocked(*inst) &&
                        inst->coord != moving_group.anchor->coord;
               })))
            available_sides[side] = false;
          if (log_proficite_cells && !available_sides[side]) {
            std::print("\nPLACE_SWAPPING_ALPHA_SKIP_SIDE pass={} endpoint='{}' "
                       "side={} anchor='{}' fixed={}",
                       pass + 1, deficite_cell->makeName(full_name_limit), side,
                       moving_group.anchor->makeName(full_name_limit),
                       moving_group.fixed);
          }
        }
        if (!available_sides[0] && !available_sides[1])
          continue;

        struct SideProjection {
          int side = 0;
          double predicted_slack_ns = -std::numeric_limits<double>::infinity();
          int predicted_distance = 0;
          bool farther = false;
          bool eligible = false;
          bool packing_rescue = false;
        };
        struct BestSwap {
          const ProficiteCell *proficite = nullptr;
          SideProjection projection;
          SwapTimingScore score;
        } best_swap;
        std::vector<SwapAlternative> alternatives;

        Coord search_from = edge->driver->coord;
        Coord search_to = edge->sink->coord;
        visitRectangleProficite(
            maps, search_from, search_to, [&](const ProficiteCell &proficite) {
              if (finishProvisionalWork())
                return true;
              if (acceptedSwapCapReached()) {
                pass_acceptance_cap_reached = true;
                return true;
              }
              if (timedOut()) {
                result.timed_out = true;
                return true;
              }
              ++result.candidate_groups_seen;
              if (!proficite.group || proficite.group == &first_group->second ||
                  proficite.group == &second_group->second ||
                  proficite.group->fixed) {
                return false;
              }

              SideProjection projections[2];
              double before_deficit = std::max(0.0, -endpoint->slack_ns);
              for (int side = 0; side < 2; ++side) {
                projections[side].side = side;
                if (!available_sides[side])
                  continue;
                BunchInfo &moving_group =
                    side == 0 ? first_group->second : second_group->second;
                rtl::Inst *moving_endpoint =
                    side == 0 ? edge->driver : edge->sink;
                rtl::Inst *other_endpoint =
                    side == 0 ? edge->sink : edge->driver;
                Coord predicted_endpoint_coord =
                    proficite.group->anchor->coord +
                    (moving_endpoint->coord - moving_group.anchor->coord);
                if (compact_combs && moving_endpoint != moving_group.anchor &&
                    !isClocked(*moving_endpoint))
                  predicted_endpoint_coord = proficite.group->anchor->coord;
                predicted_endpoint_coord.x =
                    std::clamp(predicted_endpoint_coord.x, 0, width - 1);
                predicted_endpoint_coord.y =
                    std::clamp(predicted_endpoint_coord.y, 0, height - 1);
                int old_distance =
                    manhattan(moving_endpoint->coord, other_endpoint->coord);
                projections[side].predicted_distance =
                    manhattan(predicted_endpoint_coord, other_endpoint->coord);
                projections[side].farther =
                    projections[side].predicted_distance > old_distance;
                projections[side].predicted_slack_ns = projectedEndpointSlack(
                    *endpoint, moving_group, proficite.group->anchor->coord,
                    false, compact_combs);
                double after_deficit = std::max(
                    0.0, -projections[side].predicted_slack_ns);
                double predicted_improvement =
                    before_deficit > epsilon
                        ? (before_deficit - after_deficit) / before_deficit
                        : 0;
                projections[side].eligible = predicted_improvement + epsilon >=
                                             config.minimum_improvement;
                if (!rigid_projection_only && !projections[side].eligible &&
                    config.placement_radius > 0 &&
                    moving_group.members.size() > 1) {
                  double packing_bound = projectedEndpointSlack(
                      *endpoint, moving_group, proficite.group->anchor->coord,
                      true, compact_combs);
                  double best_deficit = std::max(0.0, -packing_bound);
                  double possible_improvement = before_deficit > epsilon
                      ? (before_deficit - best_deficit) / before_deficit : 0;
                  projections[side].eligible = possible_improvement + epsilon >=
                                               config.minimum_improvement;
                  projections[side].packing_rescue = projections[side].eligible;
                }
                if (!projections[side].eligible) {
                  ++result.rejected_improvement;
                  ++result.rejected_endpoint_improvement;
                }
              }
              if (projections[1].eligible &&
                  (!projections[0].eligible ||
                   projections[1].predicted_slack_ns >
                       projections[0].predicted_slack_ns)) {
                std::swap(projections[0], projections[1]);
              }

              for (const SideProjection &projection : projections) {
                if (!projection.eligible)
                  continue;
                ++result.candidates_examined;
                int side = projection.side;
                BunchInfo &moving_group =
                    side == 0 ? first_group->second : second_group->second;
                rtl::Inst *other_endpoint =
                    side == 0 ? edge->sink : edge->driver;
                if (trace_edge) {
                  std::print(
                      "\nPLACE_SWAPPING_TRACE_PROFICITE side={} moving='{}' "
                      "other='{}' challenger='{}' "
                      "DEFICITE_slack_ns={:.3f}",
                      side == 0 ? "driver" : "sink",
                      moving_group.anchor->makeName(full_name_limit),
                      other_endpoint->makeName(full_name_limit),
                      proficite.group->anchor->makeName(full_name_limit),
                      endpoint->slack_ns);
                }
                ++result.attempts;
                BunchInfo &challenger = *proficite.group;
                bool farther = projection.farther;
                result.farther_candidates_admitted += farther;
                result.farther_candidates_attempted += farther;

                std::vector<BunchSnapshot> original_bunches{
                    {moving_group.bunch, moving_group.bunch->x,
                     moving_group.bunch->y},
                    {challenger.bunch, challenger.bunch->x,
                     challenger.bunch->y},
                };
                SwapAttemptResult swap_attempt =
                    attemptRelocation(moving_group, challenger, true, compact_combs);
                std::vector<PlacementSnapshot> &snapshots =
                    swap_attempt.snapshots;
                if (!swap_attempt.packed) {
                  ++result.rejected_pack;
                  continue;
                }
                std::uint64_t candidate_fingerprint =
                    fingerprintAfter(snapshots);
                if (visited_placements.contains(candidate_fingerprint)) {
                  ++result.rejected_visited_placements;
                  result.rolled_back_cells += snapshots.size();
                  restore(snapshots, original_bunches);
                  continue;
                }

                double endpoint_slack_before = endpoint->slack_ns;
                double first_slack_before =
                    minimumSetupSlack(first_group->second.bunch);
                double second_slack_before =
                    minimumSetupSlack(second_group->second.bunch);
                double challenger_slack_before =
                    minimumSetupSlack(challenger.bunch);
                std::vector<PlaceTimingEndpoint *> affected =
                    affectedSetupEndpoints(first_group->second.bunch,
                                           second_group->second.bunch,
                                           challenger.bunch);
                std::vector<LocalEndpointSnapshot> local_snapshots =
                    snapshotLocalTiming(affected);
                evaluateLocalTiming(affected);
                result.locally_corrected_endpoints += affected.size();

                double before_deficit =
                    std::max(0.0, -endpoint_slack_before);
                double after_deficit = std::max(0.0, -endpoint->slack_ns);
                double local_improvement =
                    before_deficit > epsilon
                        ? (before_deficit - after_deficit) / before_deficit
                        : 0;
                double first_slack_after =
                    minimumSetupSlack(first_group->second.bunch);
                double second_slack_after =
                    minimumSetupSlack(second_group->second.bunch);
                double challenger_slack_after =
                    minimumSetupSlack(challenger.bunch);
                double first_slack_limit =
                    criticalSlackLimitForPass(first_slack_before,
                                              result.initial_temperature,
                                              pass);
                double second_slack_limit =
                    criticalSlackLimitForPass(second_slack_before,
                                              result.initial_temperature,
                                              pass);
                double challenger_slack_limit =
                    challengerSlackLimitForPass(challenger_slack_before,
                                                result.initial_temperature,
                                                pass);
                bool local_timing_ok =
                    local_improvement + epsilon >= config.minimum_improvement &&
                    first_slack_after + epsilon >= first_slack_limit &&
                    second_slack_after + epsilon >= second_slack_limit &&
                    challenger_slack_after + epsilon >=
                        challenger_slack_limit;
                if (!local_timing_ok) {
                  ++result.rejected_local_timing;
                  ++result.rejected_improvement;
                  ++result.rejected_endpoint_improvement;
                  result.rolled_back_cells += snapshots.size();
                  restoreLocalTiming(local_snapshots);
                  restore(snapshots, original_bunches);
                  continue;
                }

                SwapTimingScore score{
                    .worst_slack_ns = std::min(
                        {first_slack_after, second_slack_after,
                         challenger_slack_after}),
                    .tns_delta_ns = 0,
                    .endpoint_slack_ns = endpoint->slack_ns,
                    .stable_order = proficite.stable_order,
                };
                // Reuse the exact affected-cone evaluation and its existing
                // before values: no additional timing walk or global analysis.
                for (const auto &snapshot : local_snapshots)
                  score.tns_delta_ns +=
                      std::max(0.0, -snapshot.endpoint->slack_ns) -
                      std::max(0.0, -snapshot.slack_ns);
                bool better = !best_swap.proficite ||
                              score.rank() < best_swap.score.rank();
                if (better) {
                  best_swap.proficite = &proficite;
                  best_swap.projection = projection;
                  best_swap.score = score;
                }
                alternatives.push_back(SwapAlternative{
                    .endpoint_index = static_cast<size_t>(
                        endpoint - local_analysis.endpoint_details.data()),
                    .moving_bunch = moving_group.bunch,
                    .challenger_bunch = challenger.bunch,
                    .compact_combs = compact_combs,
                    .endpoint_improvement = local_improvement,
                    .score = score,
                    .protected_limits =
                        {{{first_group->second.bunch, first_slack_limit},
                          {second_group->second.bunch, second_slack_limit},
                          {challenger.bunch, challenger_slack_limit}}},
                });
                // Candidate evaluation is speculative. Continue the direct
                // square traversal so exact recovery can fall back through
                // every locally valid choice rather than losing the endpoint
                // when its strongest approximation is rejected.
                restoreLocalTiming(local_snapshots);
                restore(snapshots, original_bunches);
              }
              return false;
            });

        std::stable_sort(
            alternatives.begin(), alternatives.end(),
            [](const SwapAlternative &left, const SwapAlternative &right) {
              return left.score.rank() < right.score.rank();
            });

        if (best_swap.proficite && !result.timed_out &&
            !pass_acceptance_cap_reached) {
          const ProficiteCell &proficite = *best_swap.proficite;
          int side = best_swap.projection.side;
          BunchInfo &moving_group =
              side == 0 ? first_group->second : second_group->second;
          BunchInfo &challenger = *proficite.group;
          rtl::Inst *moving_endpoint =
              side == 0 ? edge->driver : edge->sink;
          rtl::Inst *other_endpoint =
              side == 0 ? edge->sink : edge->driver;
          int old_endpoint_distance =
              manhattan(moving_endpoint->coord, other_endpoint->coord);
          std::vector<BunchSnapshot> original_bunches{
              {moving_group.bunch, moving_group.bunch->x,
               moving_group.bunch->y},
              {challenger.bunch, challenger.bunch->x,
               challenger.bunch->y},
          };
          SwapAttemptResult swap_attempt =
              attemptRelocation(moving_group, challenger, true, compact_combs);
          PNR_ASSERT(swap_attempt.packed,
                     "best PlaceSwapping candidate no longer packs");
          std::vector<PlacementSnapshot> &snapshots = swap_attempt.snapshots;
          std::uint64_t candidate_fingerprint = fingerprintAfter(snapshots);
          PNR_ASSERT(!visited_placements.contains(candidate_fingerprint),
                     "best PlaceSwapping candidate became visited");
          PNR_ASSERT(!alternatives.empty() &&
                         alternatives.front().moving_bunch ==
                             moving_group.bunch &&
                         alternatives.front().challenger_bunch ==
                             challenger.bunch,
                     "PlaceSwapping fallback ranking lost its strongest "
                     "candidate");

          double endpoint_slack_before = endpoint->slack_ns;
          double first_slack_before =
              minimumSetupSlack(first_group->second.bunch);
          double second_slack_before =
              minimumSetupSlack(second_group->second.bunch);
          double challenger_slack_before =
              minimumSetupSlack(challenger.bunch);
          std::vector<PlaceTimingEndpoint *> affected =
              affectedSetupEndpoints(first_group->second.bunch,
                                     second_group->second.bunch,
                                     challenger.bunch);
          evaluateLocalTiming(affected);
          result.locally_corrected_endpoints += affected.size();
          double before_deficit = std::max(0.0, -endpoint_slack_before);
          double after_deficit = std::max(0.0, -endpoint->slack_ns);
          double local_improvement =
              before_deficit > epsilon
                  ? (before_deficit - after_deficit) / before_deficit
                  : 0;
          double first_slack_limit = criticalSlackLimitForPass(
              first_slack_before, result.initial_temperature, pass);
          double second_slack_limit = criticalSlackLimitForPass(
              second_slack_before, result.initial_temperature, pass);
          double challenger_slack_limit = challengerSlackLimitForPass(
              challenger_slack_before, result.initial_temperature, pass);
          PNR_ASSERT(local_improvement + epsilon >=
                         config.minimum_improvement &&
                     minimumSetupSlack(first_group->second.bunch) + epsilon >=
                         first_slack_limit &&
                     minimumSetupSlack(second_group->second.bunch) + epsilon >=
                         second_slack_limit &&
                     minimumSetupSlack(challenger.bunch) + epsilon >=
                         challenger_slack_limit,
                     "best PlaceSwapping candidate changed during commit");

          ++result.accepted_swaps;
          result.farther_candidates_accepted += best_swap.projection.farther;
          auto recordTemperatureLimit = [&](RegBunch *bunch, double limit) {
            auto [entry, inserted] =
                pass_temperature_limits.emplace(bunch, limit);
            if (!inserted)
              entry->second = std::max(entry->second, limit);
          };
          recordTemperatureLimit(first_group->second.bunch,
                                 first_slack_limit);
          recordTemperatureLimit(second_group->second.bunch,
                                 second_slack_limit);
          recordTemperatureLimit(challenger.bunch, challenger_slack_limit);
          visited_placements.insert(candidate_fingerprint);
          accepted_history.push_back(AcceptedSwapSnapshot{
              .placements = std::move(snapshots),
              .bunches = std::move(original_bunches),
              .fingerprint_before = placement_fingerprint,
              .alternatives = std::move(alternatives),
          });
          placement_fingerprint = candidate_fingerprint;
          std::print("\nPLACE_SWAPPING_MOVE pass={} endpoint='{}' "
                     "DEFICITE_slack_ns={:.3f} edge='{}'->'{}' "
                     "side={} challenger='{}' PROFICITE_slack_ns={:.3f} "
                     "distance={}->{} improvement={:.1f}% "
                     "projected_slack_ns={:.3f} corrected_slack_ns={:.3f} "
                     "packing_projection_rescue={} compact_combs={} "
                     "corrected_endpoints={} affected_worst_slack_ns={:.3f} "
                     "affected_tns_delta_ns={:.3f} selection=affected_worst_then_tns",
                     pass + 1, deficite_cell->makeName(full_name_limit),
                     endpoint_slack_before, driver_name, sink_name,
                     side == 0 ? "driver" : "sink",
                     challenger.anchor->makeName(full_name_limit),
                     proficite.slack_ns, old_endpoint_distance,
                     manhattan(moving_endpoint->coord, other_endpoint->coord),
                     100.0 * local_improvement,
                     best_swap.projection.predicted_slack_ns,
                     endpoint->slack_ns, best_swap.projection.packing_rescue,
                     compact_combs, affected.size(),
                     best_swap.score.worst_slack_ns,
                     best_swap.score.tns_delta_ns);
          endpoint_accepted = true;
          pass_has_provisional_swaps = true;
          pass_strongest_improvement = std::max(
              pass_strongest_improvement, local_improvement);
          if (acceptedSwapCapReached())
            pass_acceptance_cap_reached = true;
        }

        if (pass_acceptance_cap_reached)
          break;
        if (pass_recovery_reserve_reached)
          break;
        if (timedOut())
          result.timed_out = true;
      }

      // Do not rebuild timing or maps here. The pass-start snapshot remains
      // frozen, and moved bunches invalidate stale C entries through counts.
      // New timing information becomes available at the pass boundary.
      if (result.timed_out)
        break;
      if (pass_recovery_reserve_reached)
        break;
    }

    if (pass_acceptance_cap_reached)
      ++result.acceptance_capped_passes;
    if (pass_recovery_reserve_reached)
      ++result.recovery_reserved_passes;

    size_t provisional_swaps = accepted_history.size() - pass_history_start;
    if (pass_has_provisional_swaps) {
      ++result.pass_timing_analyses;
      PlaceTimingAnalysis candidate = timing.analyze(timings);
      bool relaxed_acceptance = false;
      const PlaceTimingAnalysis *regression_reference =
          best_history_size == 0 ? &result.before : &best_timing_limits;
      pass_exact_temperature_ok = true;
      for (const auto &[bunch, limit_ns] : pass_temperature_limits) {
        double exact_slack_ns = minimumBunchSetupSlack(candidate, bunch);
        if (exact_slack_ns + epsilon >= limit_ns)
          continue;
        pass_exact_temperature_ok = false;
        auto group = groups.find(bunch);
        std::string bunch_name =
            group != groups.end() && group->second.anchor
                ? group->second.anchor->makeName(full_name_limit)
                : std::string{"<unknown>"};
        std::print(
            "\nPLACE_SWAPPING_TEMPERATURE_FAILURE pass={} bunch='{}' "
            "exact_slack_ns={:.3f} required_limit_ns={:.3f} "
            "shortfall_ns={:.3f}",
            pass + 1, bunch_name, exact_slack_ns, limit_ns,
            limit_ns - exact_slack_ns);
      }
      bool global_ok = pass_exact_temperature_ok && acceptsTimingTradeoff(
          pass_strongest_improvement, current, candidate,
          &relaxed_acceptance, regression_reference);
      if (global_ok) {
        for (size_t index = pass_history_start;
             index < accepted_history.size(); ++index) {
          std::vector<SwapAlternative>{}.swap(
              accepted_history[index].alternatives);
        }
        current = std::move(candidate);
        result.accepted_relaxed_swaps +=
            relaxed_acceptance ? provisional_swaps : 0;
        pass_used_relaxed_rule = relaxed_acceptance;
        pass_improved = true;
        captureMarkerTarget(current);
        if (retainsBetterFinalTiming(current)) {
          best_worst_slack_ns = current.worst_slack_ns;
          best_tns_ns = current.total_negative_slack_ns;
          best_timing_limits.worst_slack_ns = best_worst_slack_ns;
          best_timing_limits.total_negative_slack_ns = best_tns_ns;
          best_history_size = accepted_history.size();
        }
      } else {
        struct ReplayProposal {
          std::vector<SwapAlternative> alternatives;
        };
        std::vector<ReplayProposal> proposals;
        proposals.reserve(provisional_swaps);
        for (size_t index = pass_history_start;
             index < accepted_history.size(); ++index) {
          AcceptedSwapSnapshot &snapshot = accepted_history[index];
          proposals.push_back({std::move(snapshot.alternatives)});
        }

        // Restore the exact pass-start placement first. Selective rollback in
        // place is unsafe because a later provisional move may occupy a Tile
        // vacated by an earlier one. Replay preserves proposal order while
        // giving every swap its own exact timing decision.
        while (accepted_history.size() > pass_history_start) {
          AcceptedSwapSnapshot &snapshot = accepted_history.back();
          result.rolled_back_cells += snapshot.placements.size();
          placement_fingerprint = snapshot.fingerprint_before;
          restore(snapshot.placements, snapshot.bunches);
          accepted_history.pop_back();
        }
        result.accepted_swaps -= provisional_swaps;

        size_t recovered = 0;
        size_t recovery_evaluations = 0;
        PlaceTimingIncremental incremental(timing, current);
        auto changedCells = [](const SwapAttemptResult &attempt) {
          std::vector<rtl::Inst *> changed;
          changed.reserve(attempt.snapshots.size());
          for (const auto &snapshot : attempt.snapshots)
            changed.push_back(snapshot.inst);
          return changed;
        };
        for (const ReplayProposal &proposal : proposals) {
          if (timedOut()) {
            result.timed_out = true;
            break;
          }
          size_t best_rank = std::numeric_limits<size_t>::max();
          double best_wns = -std::numeric_limits<double>::infinity();
          double best_tns = std::numeric_limits<double>::infinity();
          bool best_relaxed = false;
          std::uint64_t best_fingerprint = 0;
          PlaceTimingAnalysis reference;
          reference.worst_slack_ns = current.worst_slack_ns;
          reference.total_negative_slack_ns = current.total_negative_slack_ns;
          reference.violated_endpoints = current.violated_endpoints;

          // Every locally valid alternative remains eligible. A trial only
          // reevaluates endpoint cones touched by moved cells, including
          // previously noncritical branches. No whole-design analyze here.
          for (size_t rank = 0; rank < proposal.alternatives.size(); ++rank) {
            if (timedOut()) {
              result.timed_out = true;
              break;
            }
            const SwapAlternative &alternative = proposal.alternatives[rank];
            auto moving = groups.find(alternative.moving_bunch);
            auto challenger = groups.find(alternative.challenger_bunch);
            if (moving == groups.end() || challenger == groups.end() ||
                moving->second.fixed || challenger->second.fixed)
              continue;
            std::vector<BunchSnapshot> original_bunches{
                {moving->second.bunch, moving->second.bunch->x, moving->second.bunch->y},
                {challenger->second.bunch, challenger->second.bunch->x, challenger->second.bunch->y},
            };
            double before_deficit = std::max(
                0.0, -current.endpoint_details[alternative.endpoint_index].slack_ns);
            SwapAttemptResult replay = attemptRelocation(
                moving->second, challenger->second, true, alternative.compact_combs);
            if (!replay.packed) {
              ++result.rejected_pack;
              continue;
            }
            auto candidate_fingerprint = fingerprintAfter(replay.snapshots);
            if (visited_placements.contains(candidate_fingerprint)) {
              restore(replay.snapshots, original_bunches);
              continue;
            }
            auto transaction = incremental.update(changedCells(replay));
            ++recovery_evaluations;
            ++result.local_recovery_evaluations;
            double after_deficit = std::max(
                0.0, -current.endpoint_details[alternative.endpoint_index].slack_ns);
            double improvement = before_deficit > epsilon
                ? (before_deficit - after_deficit) / before_deficit : 0;
            bool temperature_ok = true;
            for (const auto &limit : alternative.protected_limits) {
              auto group = groups.find(limit.bunch);
              if (group != groups.end())
                temperature_ok &= incremental.minimumSlack(group->second.members) +
                                      epsilon >= limit.slack_ns;
            }
            bool relaxed = false;
            bool accepted = temperature_ok &&
                improvement + epsilon >= config.minimum_improvement &&
                acceptsTimingTradeoff(improvement, reference, current, &relaxed,
                    best_history_size == 0 ? &result.before : &best_timing_limits);
            if (accepted && (best_rank == std::numeric_limits<size_t>::max() ||
                current.worst_slack_ns > best_wns + epsilon ||
                (std::abs(current.worst_slack_ns - best_wns) <= epsilon &&
                 current.total_negative_slack_ns < best_tns - epsilon))) {
              best_rank = rank;
              best_wns = current.worst_slack_ns;
              best_tns = current.total_negative_slack_ns;
              best_relaxed = relaxed;
              best_fingerprint = candidate_fingerprint;
            }
            incremental.restore(std::move(transaction));
            result.rolled_back_cells += replay.snapshots.size();
            restore(replay.snapshots, original_bunches);
          }

          if (best_rank != std::numeric_limits<size_t>::max()) {
            const auto &alternative = proposal.alternatives[best_rank];
            BunchInfo &moving = groups.at(alternative.moving_bunch);
            BunchInfo &challenger = groups.at(alternative.challenger_bunch);
            std::vector<BunchSnapshot> original_bunches{
                {moving.bunch, moving.bunch->x, moving.bunch->y},
                {challenger.bunch, challenger.bunch->x, challenger.bunch->y},
            };
            SwapAttemptResult replay = attemptRelocation(
                moving, challenger, true, alternative.compact_combs);
            PNR_ASSERT(replay.packed &&
                           fingerprintAfter(replay.snapshots) == best_fingerprint,
                       "exact local recovery placement changed during commit");
            incremental.update(changedCells(replay));
            PNR_ASSERT(std::abs(current.worst_slack_ns - best_wns) < 1e-7 &&
                           std::abs(current.total_negative_slack_ns - best_tns) < 1e-7,
                       "exact local recovery timing changed during commit");
            ++result.accepted_swaps;
            result.accepted_relaxed_swaps += best_relaxed;
            visited_placements.insert(best_fingerprint);
            accepted_history.push_back(AcceptedSwapSnapshot{
                .placements = std::move(replay.snapshots),
                .bunches = std::move(original_bunches),
                .fingerprint_before = placement_fingerprint,
                .alternatives = {},
            });
            placement_fingerprint = best_fingerprint;
            pass_used_relaxed_rule |= best_relaxed;
            pass_improved = true;
            ++recovered;
            captureMarkerTarget(current);
            if (retainsBetterFinalTiming(current)) {
              best_worst_slack_ns = current.worst_slack_ns;
              best_tns_ns = current.total_negative_slack_ns;
              best_timing_limits.worst_slack_ns = best_worst_slack_ns;
              best_timing_limits.total_negative_slack_ns = best_tns_ns;
              best_history_size = accepted_history.size();
            }
            std::print(
                "\nPLACE_SWAPPING_SELECTIVE_KEEP pass={} rank={} "
                "moving='{}' challenger='{}' WNS_ns={:.3f} TNS_ns={:.3f}",
                pass + 1, best_rank + 1, moving.anchor->makeName(full_name_limit),
                challenger.anchor->makeName(full_name_limit),
                current.worst_slack_ns, current.total_negative_slack_ns);
          }
          if (result.timed_out) break;
        }
        // One independent full analysis checks all incremental decisions,
        // refreshes force data and detects any missed dependency immediately.
        ++result.pass_timing_analyses;
        auto exact = timing.analyze(timings);
        PNR_ASSERT(std::abs(exact.worst_slack_ns - current.worst_slack_ns) < 1e-7 &&
                       std::abs(exact.total_negative_slack_ns -
                                current.total_negative_slack_ns) < 1e-6 &&
                       exact.violated_endpoints == current.violated_endpoints,
                   "incremental recovery disagrees with full timing analysis");
        current = std::move(exact);
        result.rejected_global_timing += provisional_swaps - recovered;
        result.rolled_back_pass_swaps += provisional_swaps - recovered;
        std::print(
            "\nPLACE_SWAPPING_SELECTIVE_SUMMARY pass={} provisional={} "
            "kept={} rejected={} local_evaluations={} full_analyses=1",
            pass + 1, provisional_swaps, recovered,
            provisional_swaps - recovered, recovery_evaluations);
      }
      maps = buildTimingCellMaps(current);
      result.deficite_cells = maps.deficite.size();
      result.proficite_cells = maps.proficite_cells;
      result.proficite_regions = maps.proficite.size();
      result.proficite_regions_relaxed = maps.relaxed_regions;
      if (marker_target_frame_pending) {
        captureTimingMapFrame(maps, "target_wns");
        explainMidpointProficite(maps, current);
        marker_target_frame_pending = false;
      } else if (!tech->place.movement_png_prefix.empty() ||
                 log_proficite_cells) {
        // With map rebuilding deferred to pass boundaries, retain one exact
        // DEFICITE/PROFICITE snapshot per completed pass. Name rolled-back
        // batches explicitly so ALPHA logs cannot present them as accepted.
        captureTimingMapFrame(
            maps, std::format("pass_{:03d}_{}", pass + 1,
                              pass_improved ? "accepted" : "rolled_back"));
      }
    }
    if (pass_improved)
      ++result.improving_passes;
    std::print("\nPLACE_SWAPPING_PASS pass={} TEMPERATURE={:.3f} "
               "A_B_limit_ns={:.3f} attempts={} provisional={} "
               "accepted={} DEFICITE={} PROFICITE={} "
               "worst_slack_ns={:.3f}->{:.3f} tns_ns={:.3f}->{:.3f} "
               "full_analysis={} temperature_ok={} relaxed={} "
               "acceptance_cap_reached={} recovery_reserve_reached={}",
               pass + 1, pass_temperature, pass_ab_slack_limit,
               result.attempts - pass_attempts_before,
               provisional_swaps,
               result.accepted_swaps - pass_accepted_before, pass_deficite,
               maps.proficite_cells, pass_worst_before,
               current.worst_slack_ns, pass_tns_before,
               current.total_negative_slack_ns,
               pass_has_provisional_swaps, pass_exact_temperature_ok,
               pass_used_relaxed_rule,
               pass_acceptance_cap_reached,
               pass_recovery_reserve_reached);
    if (result.timed_out || pass_recovery_reserve_reached ||
        (!pass_improved && pass_temperature <= epsilon))
      break;
  }

  if (!result.timed_out && timedOut())
    result.timed_out = true;

  rollbackTailToBest();
  if (std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_MIDPOINT")) {
    const PlaceTimingEndpoint *worst_endpoint = nullptr;
    for (const PlaceTimingEndpoint &endpoint : current.endpoint_details) {
      if (!worst_endpoint || endpoint.slack_ns < worst_endpoint->slack_ns)
        worst_endpoint = &endpoint;
    }
    if (worst_endpoint && !worst_endpoint->critical_edges.empty()) {
      const PlaceTimingEdge &worst_edge = *std::ranges::max_element(
          worst_endpoint->critical_edges, {},
          &PlaceTimingEdge::wire_delay_ns);
      if (worst_edge.driver && worst_edge.sink) {
        tech->place.movement_marker_a = worst_edge.driver;
        tech->place.movement_marker_b = worst_edge.sink;
        std::print(
            "\nPLACE_SWAPPING_ALPHA_WNS slack_ns={:.3f} A='{}' "
            "coord=({},{}) B='{}' coord=({},{}) wire_delay_ns={:.3f}",
            worst_endpoint->slack_ns,
            worst_edge.driver->makeName(full_name_limit),
            worst_edge.driver->coord.x, worst_edge.driver->coord.y,
            worst_edge.sink->makeName(full_name_limit),
            worst_edge.sink->coord.x, worst_edge.sink->coord.y,
            worst_edge.wire_delay_ns);
      }
    }
  }
  TimingCellMaps final_maps = buildTimingCellMaps(current);
  result.deficite_cells = final_maps.deficite.size();
  result.proficite_cells = final_maps.proficite_cells;
  result.proficite_regions = final_maps.proficite.size();
  result.proficite_regions_relaxed = final_maps.relaxed_regions;
  if (!tech->place.movement_png_prefix.empty() || log_proficite_cells ||
      std::getenv("SCALEPNR_PLACE_SWAP_EXPLAIN_MIDPOINT")) {
    captureTimingMapFrame(final_maps, "final");
    if (std::getenv("SCALEPNR_PLACE_SWAP_AUDIT_WORST")) {
      std::vector<const PlaceTimingEndpoint *> worst;
      for (const auto &endpoint : current.endpoint_details)
        if (endpoint.slack_ns < config.deficite_slack_ns &&
            endpoint.data_in && endpoint.data_in->inst_ref.peer &&
            !endpoint.critical_edges.empty())
          worst.push_back(&endpoint);
      std::ranges::stable_sort(worst, {}, &PlaceTimingEndpoint::slack_ns);
      worst.resize(std::min<size_t>(3, worst.size()));
      // Emit every selected path before potentially expensive forced trials.
      for (size_t rank = 0; rank < worst.size(); ++rank) {
        const auto *endpoint = worst[rank];
        for (const auto &edge : endpoint->critical_edges) {
          if (!edge.driver || !edge.sink)
            continue;
          std::print(
              "\nPLACE_SWAPPING_AUDIT_PATH rank={} endpoint={} slack={:.9f} "
              "A={} coord=({},{}) B={} coord=({},{}) same_bunch={} wire={:.9f}",
              rank, endpoint->data_in->inst_ref.peer->makeName(full_name_limit),
              endpoint->slack_ns, edge.driver->makeName(full_name_limit),
              edge.driver->coord.x, edge.driver->coord.y,
              edge.sink->makeName(full_name_limit), edge.sink->coord.x,
              edge.sink->coord.y,
              edge.driver->bunch_ref.peer == edge.sink->bunch_ref.peer,
              edge.wire_delay_ns);
        }
      }
      for (const auto *endpoint : worst) {
        const auto &edge = *std::ranges::max_element(
            endpoint->critical_edges, {}, &PlaceTimingEdge::wire_delay_ns);
        explainMidpointProficite(final_maps, current, endpoint,
                                edge.driver, edge.sink);
      }
    } else {
      explainMidpointProficite(final_maps, current);
    }
  }
  result.after = std::move(current);
  result.actionable_violations_after = countActionable(result.after);
  std::print("\nPLACE_SWAPPING_LOCAL_TIMING prepared={} calls={} elapsed_ms={:.3f}",
             !reference_local_timing, local_timing_calls, local_timing_ms);
  report();
  return result;
}
