#include "PlaceSwapping.h"

#include "Device.h"
#include "RegBunch.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <print>
#include <ranges>
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

struct AcceptedSwapSnapshot {
  std::vector<PlacementSnapshot> placements;
  std::vector<BunchSnapshot> bunches;
  pnr::RegBunch *first_bunch = nullptr;
  pnr::RegBunch *second_bunch = nullptr;
};

struct SwapAttemptResult {
  bool packed = false;
  std::vector<PlacementSnapshot> snapshots;
  double challenger_timing_before_ns = 0;
  double challenger_timing_after_ns = 0;
  double challenger_timing_degradation = 0;
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
  std::unordered_map<const rtl::Inst *, size_t> stable_cell_order;
  stable_cell_order.reserve(cells.size());
  for (size_t index = 0; index < cells.size(); ++index) {
    if (cells[index])
      stable_cell_order.emplace(cells[index], index);
  }
  auto isActionable = [&](const PlaceTimingEndpoint &endpoint) {
    return endpoint.slack_ns < -config.slack_tolerance_ns;
  };
  auto countActionable = [&](const PlaceTimingAnalysis &analysis) {
    return static_cast<size_t>(
        std::ranges::count_if(analysis.endpoint_details, isActionable));
  };
  result.actionable_violations_before = countActionable(result.before);
  const char *trace_a = std::getenv("SCALEPNR_PLACE_SWAP_TRACE_A");
  const char *trace_b = std::getenv("SCALEPNR_PLACE_SWAP_TRACE_B");
  auto report = [&] {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - started)
                            .count();
    std::print(
        "\nPLACE_SWAPPING_SUMMARY endpoints={} violations={}->{} "
        "actionable_violations={}->{} worst_slack_ns={:.3f}->{:.3f} "
        "tns_ns={:.3f}->{:.3f} passes={} improving_passes={} "
        "scope_expansions={} candidates={} "
        "attempts={} accepted={} accepted_relaxed={} rolled_back_tail_swaps={} "
        "skipped_reused_bunches={} skipped_fixed_moving_sides={} "
        "skipped_missing_bunch_edges={} skipped_same_bunch_edges={} "
        "candidate_groups_seen={} farther_candidates_admitted={} "
        "farther_candidates_attempted={} farther_candidates_accepted={} "
        "candidate_shortlist_discarded={} "
        "rejected_visited_placements={} "
        "rejected_pack={} rejected_challenger_timing={} "
        "rejected_improvement={} "
        "rejected_endpoint_improvement={} rejected_global_timing={} "
        "rolled_back_cells={} "
        "strip_width={} search_length={} critical_strip_width={} "
        "critical_search_length={} placement_radius={} "
        "replacement_search_radius={} "
        "placement_core_radius={} replacement_core_radius={} "
        "maximum_swaps_per_bunch={} endpoint_limit={} edge_limit={} "
        "candidate_limit={} candidate_band_length={} "
        "additional_attempts_per_band={} base_attempt_limit_per_pass={} "
        "minimum_improvement={:.1f}% "
        "maximum_challenger_timing_degradation={:.1f}% "
        "strong_improvement={:.1f}% "
        "maximum_global_regression={:.1f}% slack_tolerance_ns={:.3f} "
        "elapsed_ms={:.3f}\n",
        result.after.endpoints, result.before.violated_endpoints,
        result.after.violated_endpoints, result.actionable_violations_before,
        result.actionable_violations_after, result.before.worst_slack_ns,
        result.after.worst_slack_ns, result.before.total_negative_slack_ns,
        result.after.total_negative_slack_ns, result.passes,
        result.improving_passes, result.scope_expansions,
        result.candidates_examined, result.attempts,
        result.accepted_swaps, result.accepted_relaxed_swaps,
        result.rolled_back_tail_swaps, result.skipped_reused_bunches,
        result.skipped_fixed_moving_sides, result.skipped_missing_bunch_edges,
        result.skipped_same_bunch_edges, result.candidate_groups_seen,
        result.farther_candidates_admitted,
        result.farther_candidates_attempted,
        result.farther_candidates_accepted,
        result.candidate_shortlist_discarded,
        result.rejected_visited_placements, result.rejected_pack,
        result.rejected_challenger_timing, result.rejected_improvement,
        result.rejected_endpoint_improvement,
        result.rejected_global_timing, result.rolled_back_cells,
        config.strip_width, config.search_length, config.critical_strip_width,
        config.critical_search_length, config.placement_radius,
        config.replacement_search_radius,
        config.placement_core_radius, config.replacement_core_radius,
        config.maximum_swaps_per_bunch, config.maximum_violated_endpoints,
        config.maximum_critical_edges_per_endpoint,
        config.maximum_candidates_per_edge, config.candidate_band_length,
        config.additional_attempts_per_band, config.maximum_attempts,
        100.0 * config.minimum_improvement,
        100.0 * config.maximum_challenger_timing_degradation,
        100.0 * config.strong_improvement,
        100.0 * config.maximum_global_regression, config.slack_tolerance_ns,
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

  // A violated endpoint may repair itself twice, but a passive challenger is
  // borrowed only while untouched. Reusing passive challengers produced many
  // exact cycles without improving timing; placement fingerprints remain a
  // second guard against longer cycles.
  std::unordered_map<RegBunch *, size_t> bunch_swap_counts;
  auto placementFingerprint = [&] {
    constexpr std::uint64_t offset = 1469598103934665603ULL;
    constexpr std::uint64_t prime = 1099511628211ULL;
    std::uint64_t hash = offset;
    auto mix = [&](std::uint64_t value) {
      hash ^= value;
      hash *= prime;
    };
    for (rtl::Inst *inst : cells) {
      if (!inst || !inst->tile.peer || !fpga::isPlaceableElement(*inst))
        continue;
      mix(static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(inst)));
      mix(static_cast<std::uint32_t>(inst->coord.x));
      mix(static_cast<std::uint32_t>(inst->coord.y));
      mix(static_cast<std::uint32_t>(inst->pos));
    }
    return hash;
  };
  std::unordered_set<std::uint64_t> visited_placements{placementFingerprint()};

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
    auto releaseBunch = [&](RegBunch *bunch) {
      auto found = bunch_swap_counts.find(bunch);
      if (found == bunch_swap_counts.end())
        return;
      if (found->second > 1)
        --found->second;
      else
        bunch_swap_counts.erase(found);
    };
    while (accepted_history.size() > best_history_size) {
      AcceptedSwapSnapshot &snapshot = accepted_history.back();
      result.rolled_back_cells += snapshot.placements.size();
      restore(snapshot.placements, snapshot.bunches);
      releaseBunch(snapshot.first_bunch);
      releaseBunch(snapshot.second_bunch);
      accepted_history.pop_back();
      ++result.rolled_back_tail_swaps;
      restored_any = true;
    }
    if (restored_any)
      current = timing.analyze(timings);
  };

  int active_placement_radius =
      std::min(config.placement_radius, config.placement_core_radius);
  int active_replacement_radius = std::min(
      config.replacement_search_radius, config.replacement_core_radius);

  struct WeightedPeer {
    rtl::Inst *inst = nullptr;
    double weight = 0;
  };
  std::unordered_map<rtl::Inst *, std::vector<WeightedPeer>> peer_cache;
  peer_cache.reserve(cells.size());
  auto weightedPeers = [&](rtl::Inst &inst)
      -> const std::vector<WeightedPeer> & {
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

  auto bunchWireDelay = [&](const BunchInfo &group) {
    double delay_ns = 0;
    for (rtl::Inst *inst : group.members) {
      if (!inst)
        continue;
      for (rtl::Conn &conn : inst->conns) {
        if (!conn.port_ref.peer || conn.port_ref->is_global)
          continue;
        if (conn.port_ref->type == rtl::Port::PORT_IN) {
          if (tech->check_clocked(inst->cell_ref->type,
                                  conn.port_ref->name))
            continue;
          rtl::Conn *driver = conn.follow();
          if (!driver || !driver->inst_ref.peer ||
              driver->inst_ref->bunch_ref.peer == group.bunch)
            continue;
          delay_ns += timing.estimateWireDelay(conn, *driver);
        } else if (conn.port_ref->type == rtl::Port::PORT_OUT) {
          for (RefBase<Referable<rtl::Conn>> *sink_ref :
               rtl::Conn::getSinks(conn)) {
            rtl::Conn *sink =
                sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            if (!sink || !sink->inst_ref.peer || !sink->port_ref.peer ||
                sink->port_ref->is_global ||
                tech->check_clocked(sink->inst_ref->cell_ref->type,
                                    sink->port_ref->name) ||
                sink->inst_ref->bunch_ref.peer == group.bunch)
              continue;
            delay_ns += timing.estimateWireDelay(*sink, conn);
          }
        }
      }
    }
    return delay_ns;
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
        if (!peer || !peer->tile.peer ||
            peer->bunch_ref.peer == group.bunch)
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

  auto attemptRelocation = [&](BunchInfo &first, BunchInfo &second) {
    SwapAttemptResult attempt;
    attempt.challenger_timing_before_ns = bunchWireDelay(second);
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
        coordinate.x = std::clamp(coordinate.x, 0, width - 1);
        coordinate.y = std::clamp(coordinate.y, 0, height - 1);
        desired[inst] = coordinate;
      }
    };
    setDesired(first, first_origin, second_origin);
    setDesired(second, second_origin, second_replacement);

    for (const PlacementSnapshot &snapshot : snapshots) {
      snapshot.inst->tile->unassign(snapshot.inst);
    }

    std::vector<rtl::Inst *> pending;
    pending.reserve(snapshots.size());
    pending.insert(pending.end(), first.members.begin(), first.members.end());
    pending.insert(pending.end(), second.members.begin(), second.members.end());
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
      int radius_limit = &inst == first.anchor
                             ? 0
                             : (&inst == second.anchor
                                    ? active_replacement_radius
                                    : active_placement_radius);
      int core_radius =
          &inst == first.anchor
              ? 0
              : (&inst == second.anchor
                     ? std::min(radius_limit,
                                config.replacement_core_radius)
                     : std::min(radius_limit, config.placement_core_radius));
      int outer_rings = radius_limit - core_radius;
      for (int search_band = 0; search_band <= outer_rings; ++search_band) {
        int band_radius = search_band == 0 ? core_radius
                                           : core_radius + search_band;
        std::vector<Candidate> candidates;
        for (int dy = -band_radius; dy <= band_radius; ++dy) {
          for (int dx = -band_radius; dx <= band_radius; ++dx) {
            int radius = std::abs(dx) + std::abs(dy);
            if ((search_band == 0 && radius > core_radius) ||
                (search_band != 0 && radius != band_radius))
              continue;
            Coord coordinate = target + Coord{dx, dy};
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
              cost += weighted_peer.weight *
                      manhattan(coordinate, peer_coord);
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
          fpga::Tile &tile =
              device.tile_grid[candidate.coord.y * width + candidate.coord.x];
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
      restore(snapshots, bunch_snapshots);
      attempt.snapshots = std::move(snapshots);
      return attempt;
    }

    float aspect_x = std::max(tech->place.aspect_x, 0.0001F);
    float aspect_y = std::max(tech->place.aspect_y, 0.0001F);
    first.bunch->x = first.anchor->coord.x / aspect_x;
    first.bunch->y = first.anchor->coord.y / aspect_y;
    second.bunch->x = second.anchor->coord.x / aspect_x;
    second.bunch->y = second.anchor->coord.y / aspect_y;
    attempt.packed = true;
    attempt.snapshots = std::move(snapshots);
    attempt.challenger_timing_after_ns = bunchWireDelay(second);
    if (attempt.challenger_timing_before_ns <= epsilon) {
      attempt.challenger_timing_degradation =
          attempt.challenger_timing_after_ns <= epsilon
              ? 0
              : std::numeric_limits<double>::infinity();
    } else {
      attempt.challenger_timing_degradation =
          (attempt.challenger_timing_after_ns -
           attempt.challenger_timing_before_ns) /
          attempt.challenger_timing_before_ns;
    }
    return attempt;
  };

  bool traced_same_bunch_samples = false;
  for (size_t pass = 0;
       pass < config.maximum_passes && countActionable(current) != 0; ++pass) {
    ++result.passes;
    size_t pass_attempts_before = result.attempts;
    size_t pass_accepted_before = result.accepted_swaps;
    double pass_worst_before = current.worst_slack_ns;
    double pass_tns_before = current.total_negative_slack_ns;
    bool pass_accepted = false;
    int band_length = std::max(1, config.candidate_band_length);
    auto axialBand = [&](int axis_offset) {
      return axis_offset == 0 ? 0 : 1 + (axis_offset - 1) / band_length;
    };
    int maximum_band = axialBand(
        std::max(config.search_length, config.critical_search_length));
    size_t attempts_per_added_band = config.additional_attempts_per_band;
    size_t maximum_attempts_this_pass =
        config.maximum_attempts +
        static_cast<size_t>(maximum_band) * attempts_per_added_band;
    while (countActionable(current) != 0 &&
           result.attempts - pass_attempts_before <
               maximum_attempts_this_pass) {
      bool accepted = false;
      std::vector<std::vector<BunchInfo *>> groups_by_tile(
          static_cast<size_t>(width * height));
      for (auto &[bunch, group] : groups) {
        (void)bunch;
        if (group.fixed || !group.anchor || !group.anchor->tile.peer) {
          continue;
        }
        Coord coordinate = group.anchor->coord;
        if (coordinate.x < 0 || coordinate.x >= width || coordinate.y < 0 ||
            coordinate.y >= height) {
          continue;
        }
        groups_by_tile[static_cast<size_t>(coordinate.y * width + coordinate.x)]
            .push_back(&group);
      }
      std::vector<const PlaceTimingEndpoint *> violations;
      for (const PlaceTimingEndpoint &endpoint : current.endpoint_details) {
        if (isActionable(endpoint))
          violations.push_back(&endpoint);
      }
      std::ranges::sort(violations, {}, &PlaceTimingEndpoint::slack_ns);
      if (trace_a && trace_b) {
        auto traced = std::ranges::find_if(
            violations, [&](const PlaceTimingEndpoint *candidate) {
              return std::ranges::any_of(
                  candidate->critical_edges, [&](const PlaceTimingEdge &edge) {
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
        if (traced != violations.end()) {
          size_t original_rank =
              static_cast<size_t>(std::distance(violations.begin(), traced)) +
              1;
          std::rotate(violations.begin(), traced, std::next(traced));
          std::print("\nPLACE_SWAPPING_TRACE_PRIORITY "
                     "original_endpoint_rank={} endpoint_limit={}",
                     original_rank, config.maximum_violated_endpoints);
        }
      }
      if (violations.size() > config.maximum_violated_endpoints) {
        violations.resize(config.maximum_violated_endpoints);
      }

      // Traverse breadth-first by distance band. Increasing the stripe
      // length can only append outer work after every opportunity that
      // a shorter stripe would have tried. Every added band also adds a
      // small trial allowance, so outer candidates cannot steal the
      // fixed budget from inner candidates or merely remain untried.
      for (int active_band = 0; active_band <= maximum_band && !accepted;
           ++active_band) {
        size_t band_attempt_limit =
            config.maximum_attempts +
            static_cast<size_t>(active_band) * attempts_per_added_band;
        if (result.attempts - pass_attempts_before >= band_attempt_limit)
          continue;
        for (const PlaceTimingEndpoint *endpoint : violations) {
          if (accepted ||
              result.attempts - pass_attempts_before >= band_attempt_limit)
            break;
          ++result.violated_endpoints_examined;
          std::vector<const PlaceTimingEdge *> edges;
          for (const PlaceTimingEdge &edge : endpoint->critical_edges) {
            if (edge.driver && edge.sink)
              edges.push_back(&edge);
          }
          std::ranges::sort(edges, std::greater{},
                            &PlaceTimingEdge::wire_delay_ns);
          if (edges.size() > config.maximum_critical_edges_per_endpoint) {
            edges.resize(config.maximum_critical_edges_per_endpoint);
          }
          for (const PlaceTimingEdge *edge : edges) {
            if (accepted)
              break;
            std::string driver_name =
                edge->driver ? edge->driver->makeName(full_name_limit) : "";
            std::string sink_name =
                edge->sink ? edge->sink->makeName(full_name_limit) : "";
            bool trace_edge =
                trace_a && trace_b &&
                ((driver_name.find(trace_a) != std::string::npos &&
                  sink_name.find(trace_b) != std::string::npos) ||
                 (driver_name.find(trace_b) != std::string::npos &&
                  sink_name.find(trace_a) != std::string::npos));
            auto first_group = groups.find(edge->driver->bunch_ref.peer);
            auto second_group = groups.find(edge->sink->bunch_ref.peer);
            if (first_group == groups.end() || second_group == groups.end()) {
              ++result.skipped_missing_bunch_edges;
              if (trace_edge) {
                std::print("\nPLACE_SWAPPING_TRACE_SKIPPED "
                           "reason=missing_bunch driver_group={} "
                           "sink_group={}",
                           first_group != groups.end(),
                           second_group != groups.end());
              }
              continue;
            }
            if (first_group == second_group) {
              ++result.skipped_same_bunch_edges;
              if (trace_edge) {
                std::print(
                    "\nPLACE_SWAPPING_TRACE_SKIPPED "
                    "reason=same_bunch "
                    "anchor='{}' bunch_size={} fixed={}",
                    first_group->second.anchor
                        ? first_group->second.anchor->makeName(full_name_limit)
                        : "<none>",
                    first_group->second.members.size(),
                    first_group->second.fixed);
              }
              if (trace_edge && !traced_same_bunch_samples) {
                traced_same_bunch_samples = true;
                int pair_dx = edge->sink->coord.x - edge->driver->coord.x;
                int pair_dy = edge->sink->coord.y - edge->driver->coord.y;
                bool pair_horizontal = std::abs(pair_dx) >= std::abs(pair_dy);
                auto sampleSide = [&](rtl::Inst *moving_inst,
                                      rtl::Inst *other_inst,
                                      const char *side_name) {
                  struct Sample {
                    BunchInfo *group = nullptr;
                    int predicted_distance = 0;
                  };
                  std::vector<Sample> samples;
                  std::unordered_set<BunchInfo *> seen;
                  Coord moving = moving_inst->coord;
                  Coord other = other_inst->coord;
                  int old_distance = manhattan(moving, other);
                  int half_width = config.strip_width / 2;
                  for (int axis = -config.search_length;
                       axis <= config.search_length; ++axis) {
                    for (int cross = -half_width; cross <= half_width;
                         ++cross) {
                      Coord coordinate = pair_horizontal
                                             ? moving + Coord{axis, cross}
                                             : moving + Coord{cross, axis};
                      if (coordinate.x < 0 || coordinate.x >= width ||
                          coordinate.y < 0 || coordinate.y >= height) {
                        continue;
                      }
                      auto &tile_groups = groups_by_tile[static_cast<size_t>(
                          coordinate.y * width + coordinate.x)];
                      for (BunchInfo *candidate : tile_groups) {
                        if (!candidate || candidate->fixed ||
                            candidate == &first_group->second ||
                            !candidate->anchor ||
                            !isClocked(*candidate->anchor) ||
                            !seen.insert(candidate).second) {
                          continue;
                        }
                        int predicted =
                            manhattan(candidate->anchor->coord, other);
                        if (predicted >= old_distance)
                          continue;
                        samples.push_back({candidate, predicted});
                      }
                    }
                  }
                  std::ranges::sort(samples, [](const Sample &left,
                                                const Sample &right) {
                    return left.predicted_distance < right.predicted_distance;
                  });
                  if (samples.size() > 6)
                    samples.resize(6);
                  std::print("\nPLACE_SWAPPING_TRACE_INDIVIDUAL_SIDE "
                             "side={} "
                             "moving='{}' coord=({},{}) other='{}' "
                             "coord=({},{}) old_distance={} samples={}",
                             side_name, moving_inst->makeName(full_name_limit),
                             moving.x, moving.y,
                             other_inst->makeName(full_name_limit), other.x,
                             other.y, old_distance, samples.size());
                  for (const Sample &sample : samples) {
                    rtl::Inst *challenger = sample.group->anchor;
                    std::vector<PlacementSnapshot> snapshots{
                        {moving_inst, moving_inst->tile.peer,
                         moving_inst->coord, moving_inst->pos,
                         moving_inst->outline.x, moving_inst->outline.y},
                        {challenger, challenger->tile.peer, challenger->coord,
                         challenger->pos, challenger->outline.x,
                         challenger->outline.y},
                    };
                    moving_inst->tile->unassign(moving_inst);
                    challenger->tile->unassign(challenger);
                    int moved_pos = snapshots[1].tile->tryAddAt(
                        moving_inst, snapshots[1].pos);
                    int challenger_pos = snapshots[0].tile->tryAddAt(
                        challenger, snapshots[0].pos);
                    bool packed = moved_pos == snapshots[1].pos &&
                                  challenger_pos == snapshots[0].pos;
                    if (!packed) {
                      restore(snapshots, {});
                      std::print("\n  "
                                 "PLACE_SWAPPING_TRACE_INDIVIDUAL "
                                 "challenger='{}' coord=({},{}) "
                                 "bunch_size={} "
                                 "predicted_distance={} "
                                 "result=pack_failed",
                                 challenger->makeName(full_name_limit),
                                 snapshots[1].coord.x, snapshots[1].coord.y,
                                 sample.group->members.size(),
                                 sample.predicted_distance);
                      continue;
                    }
                    PlaceTimingAnalysis trial = timing.analyze(timings);
                    const PlaceTimingEndpoint *trial_endpoint =
                        findEndpoint(trial, endpoint->data_in);
                    std::print("\n  PLACE_SWAPPING_TRACE_INDIVIDUAL "
                               "challenger='{}' "
                               "coord=({},{}) bunch_size={} "
                               "predicted_distance={} "
                               "result=packed "
                               "endpoint_slack_ns={:.3f}->{:.3f} "
                               "worst_slack_ns={:.3f}->{:.3f} "
                               "tns_ns={:.3f}->{:.3f}",
                               challenger->makeName(full_name_limit),
                               snapshots[1].coord.x, snapshots[1].coord.y,
                               sample.group->members.size(),
                               sample.predicted_distance, endpoint->slack_ns,
                               trial_endpoint ? trial_endpoint->slack_ns
                                              : endpoint->slack_ns,
                               current.worst_slack_ns, trial.worst_slack_ns,
                               current.total_negative_slack_ns,
                               trial.total_negative_slack_ns);
                    restore(snapshots, {});
                  }
                };
                sampleSide(edge->driver, edge->sink, "A/driver");
                sampleSide(edge->sink, edge->driver, "B/sink");
              }
              continue;
            }
            int delta_x = edge->sink->coord.x - edge->driver->coord.x;
            int delta_y = edge->sink->coord.y - edge->driver->coord.y;
            bool horizontal = std::abs(delta_x) >= std::abs(delta_y);
            bool use_critical_geometry =
                endpoint->slack_ns <= current.worst_slack_ns + epsilon;
            int selected_strip_width = use_critical_geometry
                                           ? config.critical_strip_width
                                           : config.strip_width;
            int selected_search_length = use_critical_geometry
                                             ? config.critical_search_length
                                             : config.search_length;
            if (trace_edge) {
              std::print("\nPLACE_SWAPPING_TRACE_EDGE pass={} "
                         "endpoint_slack_ns={:.3f} driver='{}' "
                         "coord=({},{}) "
                         "bunch_size={} sink='{}' coord=({},{}) "
                         "bunch_size={} "
                         "distance={} axis={} wire_delay_ns={:.3f} "
                         "search_band={}",
                         pass + 1, endpoint->slack_ns, driver_name,
                         edge->driver->coord.x, edge->driver->coord.y,
                         first_group->second.members.size(), sink_name,
                         edge->sink->coord.x, edge->sink->coord.y,
                         second_group->second.members.size(),
                         manhattan(edge->driver->coord, edge->sink->coord),
                         horizontal ? "horizontal" : "vertical",
                         edge->wire_delay_ns, active_band);
            }

            for (int side = 0; side < 2 && !accepted; ++side) {
              BunchInfo &moving_group =
                  side == 0 ? first_group->second : second_group->second;
              BunchInfo &other_group =
                  side == 0 ? second_group->second : first_group->second;
              // The opposite endpoint is only a timing reference; it is not
              // part of this exchange. In particular, a fixed I/O must remain
              // available as an anchor that a movable bunch can approach.
              if (moving_group.fixed) {
                ++result.skipped_fixed_moving_sides;
                continue;
              }
              if (bunch_swap_counts[moving_group.bunch] >=
                  config.maximum_swaps_per_bunch) {
                ++result.skipped_reused_bunches;
                continue;
              }
              Coord moving = moving_group.anchor->coord;
              Coord other = other_group.anchor->coord;
              rtl::Inst *moving_endpoint =
                  side == 0 ? edge->driver : edge->sink;
              rtl::Inst *other_endpoint =
                  side == 0 ? edge->sink : edge->driver;
              int old_distance = manhattan(moving, other);
              int old_endpoint_distance =
                  manhattan(moving_endpoint->coord, other_endpoint->coord);

              struct Challenger {
                BunchInfo *group = nullptr;
                int predicted_distance = 0;
                int predicted_endpoint_distance = 0;
                int axis_offset = 0;
                size_t stable_order = 0;
                bool farther = false;
              };
              std::vector<Challenger> challengers;
              int half_width = selected_strip_width / 2;
              for (int axis = -selected_search_length;
                   axis <= selected_search_length; ++axis) {
                if (axialBand(std::abs(axis)) != active_band)
                  continue;
                for (int cross = -half_width; cross <= half_width; ++cross) {
                  Coord coordinate = horizontal ? moving + Coord{axis, cross}
                                                : moving + Coord{cross, axis};
                  if (coordinate.x < 0 || coordinate.x >= width ||
                      coordinate.y < 0 || coordinate.y >= height) {
                    continue;
                  }
                  auto &tile_groups = groups_by_tile[static_cast<size_t>(
                      coordinate.y * width + coordinate.x)];
                  for (BunchInfo *candidate : tile_groups) {
                    ++result.candidate_groups_seen;
                    if (!candidate || candidate == &moving_group ||
                        candidate == &other_group ||
                        bunch_swap_counts[candidate->bunch] != 0) {
                      if (candidate &&
                          bunch_swap_counts[candidate->bunch] != 0) {
                        ++result.skipped_reused_bunches;
                      }
                      continue;
                    }
                    Coord predicted_endpoint_coord =
                        candidate->anchor->coord +
                        (moving_endpoint->coord - moving_group.anchor->coord);
                    predicted_endpoint_coord.x =
                        std::clamp(predicted_endpoint_coord.x, 0, width - 1);
                    predicted_endpoint_coord.y =
                        std::clamp(predicted_endpoint_coord.y, 0, height - 1);
                    int predicted_endpoint_distance = manhattan(
                        predicted_endpoint_coord, other_endpoint->coord);
                    int predicted =
                        manhattan(candidate->anchor->coord, other);
                    // Distance is a ranking heuristic, not a proof about the
                    // complete endpoint path. Even a farther selected edge can
                    // be outweighed by improvements on the bunch's other
                    // timing connections; full analysis decides acceptance.
                    if (predicted_endpoint_distance > old_endpoint_distance) {
                      ++result.farther_candidates_admitted;
                    }
                    challengers.push_back(
                        {candidate, predicted, predicted_endpoint_distance,
                         std::abs(axis),
                         stable_cell_order.at(candidate->anchor),
                         predicted_endpoint_distance >
                             old_endpoint_distance});
                  }
                }
              }
              result.candidates_examined += challengers.size();
              size_t eligible_challengers = challengers.size();
              auto candidateBand = [&](const Challenger &candidate) {
                return axialBand(candidate.axis_offset);
              };
              std::ranges::sort(challengers, [&](const Challenger &left,
                                                 const Challenger &right) {
                if (left.predicted_distance != right.predicted_distance) {
                  return left.predicted_distance < right.predicted_distance;
                }
                if (left.predicted_endpoint_distance !=
                    right.predicted_endpoint_distance) {
                  return left.predicted_endpoint_distance <
                         right.predicted_endpoint_distance;
                }
                if (left.axis_offset != right.axis_offset) {
                  return left.axis_offset < right.axis_offset;
                }
                return left.stable_order < right.stable_order;
              });
              if (challengers.size() > config.maximum_candidates_per_edge) {
                result.candidate_shortlist_discarded +=
                    challengers.size() - config.maximum_candidates_per_edge;
                challengers.resize(config.maximum_candidates_per_edge);
              }
              if (trace_edge) {
                std::print("\nPLACE_SWAPPING_TRACE_SIDE side={} "
                           "moving='{}' "
                           "coord=({},{}) bunch_size={} other='{}' "
                           "coord=({},{}) old_distance={} eligible={} "
                           "selected={} band_length={}",
                           side == 0 ? "driver" : "sink",
                           moving_group.anchor->makeName(full_name_limit),
                           moving.x, moving.y, moving_group.members.size(),
                           other_group.anchor->makeName(full_name_limit),
                           other.x, other.y, old_distance, eligible_challengers,
                           challengers.size(), band_length);
                size_t trace_limit = std::min<size_t>(challengers.size(), 12);
                for (size_t index = 0; index < trace_limit; ++index) {
                  const Challenger &candidate = challengers[index];
                  std::print("\n  PLACE_SWAPPING_TRACE_CANDIDATE "
                             "rank={} "
                             "anchor='{}' coord=({},{}) bunch_size={} "
                             "predicted_distance={} reduction={} "
                             "band={}",
                             index + 1,
                             candidate.group->anchor->makeName(full_name_limit),
                             candidate.group->anchor->coord.x,
                             candidate.group->anchor->coord.y,
                             candidate.group->members.size(),
                             candidate.predicted_distance,
                             old_distance - candidate.predicted_distance,
                             candidateBand(candidate));
                }
              }

              for (Challenger &challenger : challengers) {
                if (result.attempts - pass_attempts_before >=
                    band_attempt_limit)
                  break;
                ++result.attempts;
                result.farther_candidates_attempted += challenger.farther;
                std::vector<BunchSnapshot> original_bunches{
                    {moving_group.bunch, moving_group.bunch->x,
                     moving_group.bunch->y},
                    {challenger.group->bunch, challenger.group->bunch->x,
                     challenger.group->bunch->y},
                };
                SwapAttemptResult swap_attempt =
                    attemptRelocation(moving_group, *challenger.group);
                std::vector<PlacementSnapshot> &snapshots =
                    swap_attempt.snapshots;
                if (!swap_attempt.packed) {
                  ++result.rejected_pack;
                  if (trace_edge) {
                    std::print(
                        "\n  PLACE_SWAPPING_TRACE_RESULT "
                        "challenger='{}' "
                        "result=pack_failed "
                        "moved_bunch_size={} "
                        "challenger_bunch_size={}",
                        challenger.group->anchor->makeName(full_name_limit),
                        moving_group.members.size(),
                        challenger.group->members.size());
                  }
                  continue;
                }
                if (swap_attempt.challenger_timing_degradation >
                    config.maximum_challenger_timing_degradation + epsilon) {
                  ++result.rejected_challenger_timing;
                  result.rolled_back_cells += snapshots.size();
                  restore(snapshots, original_bunches);
                  if (trace_edge) {
                    std::print(
                        "\n  PLACE_SWAPPING_TRACE_RESULT "
                        "challenger='{}' "
                        "result=challenger_timing_rejected "
                        "challenger_wire_delay_ns={:.3f}->{:.3f} "
                        "degradation={:.1f}% limit={:.1f}%",
                        challenger.group->anchor->makeName(full_name_limit),
                        swap_attempt.challenger_timing_before_ns,
                        swap_attempt.challenger_timing_after_ns,
                        100.0 * swap_attempt.challenger_timing_degradation,
                        100.0 *
                            config.maximum_challenger_timing_degradation);
                  }
                  continue;
                }
                std::uint64_t candidate_fingerprint = placementFingerprint();
                if (visited_placements.contains(candidate_fingerprint)) {
                  ++result.rejected_visited_placements;
                  result.rolled_back_cells += snapshots.size();
                  restore(snapshots, original_bunches);
                  if (trace_edge) {
                    std::print(
                        "\n  PLACE_SWAPPING_TRACE_RESULT "
                        "challenger='{}' "
                        "result=visited_placement_rejected",
                        challenger.group->anchor->makeName(full_name_limit));
                  }
                  continue;
                }
                PlaceTimingAnalysis candidate = timing.analyze(timings);
                const PlaceTimingEndpoint *candidate_endpoint =
                    findEndpoint(candidate, endpoint->data_in);
                double before_deficit = std::max(
                    0.0, -endpoint->slack_ns - config.slack_tolerance_ns);
                double after_deficit =
                    candidate_endpoint
                        ? std::max(0.0, -candidate_endpoint->slack_ns -
                                            config.slack_tolerance_ns)
                        : before_deficit;
                double improvement =
                    before_deficit > epsilon
                        ? (before_deficit - after_deficit) / before_deficit
                        : 0;
                bool relaxed_acceptance = false;
                // Core search needs the wider run-entry envelope to cross
                // the basin which precedes the first best-state rollback.
                // Expanded search starts from that restored best state; from
                // there, measure its 5% allowance against the moving best so
                // a late local repair cannot throw away the breakthrough.
                const PlaceTimingAnalysis* regression_reference =
                    result.scope_expansions == 0
                        ? &result.before : &best_timing_limits;
                bool global_ok = acceptsTimingTradeoff(
                    improvement, current, candidate, &relaxed_acceptance,
                    regression_reference);
                bool endpoint_ok = improvement + epsilon >=
                                   config.minimum_improvement;
                if (!endpoint_ok || !global_ok) {
                  ++result.rejected_improvement;
                  result.rejected_endpoint_improvement += !endpoint_ok;
                  result.rejected_global_timing += endpoint_ok && !global_ok;
                  if (trace_edge) {
                    std::print(
                        "\n  PLACE_SWAPPING_TRACE_RESULT "
                        "challenger='{}' "
                        "result=timing_rejected "
                        "endpoint_slack_ns={:.3f}->{:.3f} "
                        "improvement={:.1f}% "
                        "violations={}->{} "
                        "worst_slack_ns={:.3f}->{:.3f} "
                        "tns_ns={:.3f}->{:.3f} global_ok={} "
                        "relaxed_ok={}",
                        challenger.group->anchor->makeName(full_name_limit),
                        endpoint->slack_ns,
                        candidate_endpoint ? candidate_endpoint->slack_ns
                                           : endpoint->slack_ns,
                        100.0 * improvement, current.violated_endpoints,
                        candidate.violated_endpoints, current.worst_slack_ns,
                        candidate.worst_slack_ns,
                        current.total_negative_slack_ns,
                        candidate.total_negative_slack_ns, global_ok,
                        relaxed_acceptance);
                  }
                  result.rolled_back_cells += snapshots.size();
                  restore(snapshots, original_bunches);
                  continue;
                }

                ++result.accepted_swaps;
                result.farther_candidates_accepted += challenger.farther;
                result.accepted_relaxed_swaps += relaxed_acceptance;
                pass_accepted = true;
                ++bunch_swap_counts[moving_group.bunch];
                ++bunch_swap_counts[challenger.group->bunch];
                visited_placements.insert(candidate_fingerprint);
                accepted_history.push_back(AcceptedSwapSnapshot{
                    .placements = std::move(snapshots),
                    .bunches = std::move(original_bunches),
                    .first_bunch = moving_group.bunch,
                    .second_bunch = challenger.group->bunch,
                });
                std::print(
                    "\nPLACE_SWAPPING_MOVE pass={} move={} "
                    "endpoint='{}' "
                    "edge='{}'->'{}' axis={} challenger='{}' "
                    "improvement={:.1f}% violations={}->{} "
                    "worst_slack_ns={:.3f}->{:.3f} "
                    "tns_ns={:.3f}->{:.3f} relaxed={}",
                    pass + 1, result.accepted_swaps - pass_accepted_before,
                    endpoint->data_in && endpoint->data_in->inst_ref.peer
                        ? endpoint->data_in->inst_ref->makeName(full_name_limit)
                        : "<none>",
                    edge->driver->makeName(full_name_limit),
                    edge->sink->makeName(full_name_limit),
                    horizontal ? "horizontal" : "vertical",
                    challenger.group->anchor->makeName(full_name_limit),
                    100.0 * improvement, current.violated_endpoints,
                    candidate.violated_endpoints, current.worst_slack_ns,
                    candidate.worst_slack_ns, current.total_negative_slack_ns,
                    candidate.total_negative_slack_ns, relaxed_acceptance);
                current = std::move(candidate);
                if (retainsBetterFinalTiming(current)) {
                  best_worst_slack_ns = current.worst_slack_ns;
                  best_tns_ns = current.total_negative_slack_ns;
                  best_timing_limits.worst_slack_ns = best_worst_slack_ns;
                  best_timing_limits.total_negative_slack_ns = best_tns_ns;
                  best_history_size = accepted_history.size();
                }
                accepted = true;
                break;
              }
            }
          }
        }
      }
      if (!accepted)
        break;
    }
    if (pass_accepted)
      ++result.improving_passes;
    std::print("\nPLACE_SWAPPING_PASS pass={} attempts={} accepted={} "
               "violations={} "
               "worst_slack_ns={:.3f}->{:.3f} tns_ns={:.3f}->{:.3f}",
               pass + 1, result.attempts - pass_attempts_before,
               result.accepted_swaps - pass_accepted_before,
               current.violated_endpoints, pass_worst_before,
               current.worst_slack_ns, pass_tns_before,
               current.total_negative_slack_ns);
    bool can_expand_scope =
        active_placement_radius < config.placement_radius ||
        active_replacement_radius < config.replacement_search_radius;
    if (!pass_accepted && can_expand_scope) {
      rollbackTailToBest();
      active_placement_radius = config.placement_radius;
      active_replacement_radius = config.replacement_search_radius;
      ++result.scope_expansions;
      std::print("\nPLACE_SWAPPING_SCOPE_EXPANDED pass={} "
                 "placement_radius={} replacement_search_radius={} "
                 "worst_slack_ns={:.3f} tns_ns={:.3f}",
                 pass + 1, active_placement_radius,
                 active_replacement_radius, current.worst_slack_ns,
                 current.total_negative_slack_ns);
      continue;
    }
    if (!pass_accepted)
      break;
  }

  rollbackTailToBest();
  result.after = std::move(current);
  result.actionable_violations_after = countActionable(result.after);
  report();
  return result;
}
