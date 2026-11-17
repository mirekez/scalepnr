#include "ForwardRouteDiagnostic.h"
#include "Device.h"
#include "RouteDiagnostics.h"
#include "RouteSearch.h"
#include "TraceRouteCommon.h"
#include "TraceRouteForward.h"

#include <algorithm>
#include <cstdlib>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

namespace pnr
{

// Attach a deadline log that becomes active during recursion.
void ForwardRouteDiagnostic::attach_deadline_stream()
{

    if (!decisions && RouteCongestionTrace::current)
        decisions = &RouteCongestionTrace::current->stream();
}

void ForwardRouteDiagnostic::attempt(size_t index)
{
    if (decisions)
        *decisions << "FORWARD_ATTEMPT anchor=" << index << '\n';
}

void ForwardRouteDiagnostic::begin_anchor(const RoutingAnchor &anchor)
{
    saved.clear();
    saved_index.clear();
    path.clear();
    seen.clear();
    toward = away = level = jump_prints = source_reports = 0;
    speculative_deadend_skips = 0;
    max_same_tile_frames = 0;
    tile_chain_printed = false;
    if (!decisions || !anchor.tile || !anchor.tile->cb_type)
        return;
    *decisions << "FORWARD_ANCHOR prefix=" << anchor.prefix.size();
    describe(decisions, *anchor.tile,
             anchor.starts_at_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL,
             anchor.node);
    *decisions << '\n';
}

void ForwardRouteDiagnostic::finish_anchor(size_t queued,
                                           const TraceRouteForward &trace)
{
    if (decisions)
        *decisions << "FORWARD_END queued=" << queued
                   << " reached=" << trace.expanded
                   << " cancelled=" << trace.cancelled
                   << " failed_states=" << trace.failed_states
                   << " failure_cache_hits=" << trace.failure_cache_hits
                   << '\n';
    if (audit)
        *audit << "FORWARD_AUDIT_END expanded=" << trace.expanded
               << " unique=" << seen.size()
               << " max_same_tile_frames=" << max_same_tile_frames
               << " physical_toward=" << toward << " physical_away=" << away
               << " physical_level=" << level
               << " speculative_deadend_skips=" << speculative_deadend_skips
               << " cancelled=" << trace.cancelled << '\n';
}

bool ForwardRouteDiagnostic::remember(fpga::Tile &tile,
                                      TraceRouteForward &trace)
{
    if (!observer && !RouteCongestionTrace::current)
        return false;
    ++trace.snapshot_lookup_checks;
    if (saved_index.find(&tile) != saved_index.end())
        return false;
    saved.emplace_back(&tile, tile.cb);
    try
    {
        saved_index.emplace(&tile, saved.size() - 1);
    }

    catch (...)
    {
        saved.pop_back();
        throw;
    }

    return true;
}

void ForwardRouteDiagnostic::forget(fpga::Tile &tile, bool added)
{
    if (added)
    {
        saved_index.erase(&tile);
        saved.pop_back();
    }

}

void ForwardRouteDiagnostic::before_probe(fpga::CBNodeNameType type,
                                          bool has_local_probe)
{
    if (observer && *observer && (type == fpga::CB_NODE_DST || has_local_probe))
        (*observer)(saved);
}

// Emit decisions only when a log was requested.
void ForwardRouteDiagnostic::reject(size_t cursor)
{

    if (decisions)
        *decisions << "FORWARD_REJECT state=" << cursor
                   << " reason=arrival_lease\n";
}

void ForwardRouteDiagnostic::state(size_t cursor, long long parent,
                                   fpga::Tile &tile, fpga::CBNodeNameType type,
                                   int node)
{

    if (!decisions)
        return;
    *decisions << "FORWARD_STATE index=" << cursor << " parent=" << parent;
    describe(decisions, tile, type, node);
    *decisions << '\n';
}

void ForwardRouteDiagnostic::grounded(size_t cursor, bool local)
{

    if (decisions)
        *decisions << (local ? "FORWARD_GROUNDED_LOCAL state="
                             : "FORWARD_GROUNDED state=")
                   << cursor << '\n';
}

void ForwardRouteDiagnostic::source_count(size_t cursor, size_t count)
{

    if (decisions)
        *decisions << "FORWARD_SOURCES state=" << cursor << " count=" << count
                   << '\n';
}

void ForwardRouteDiagnostic::target(size_t cursor, int src,
                                    const fpga::TileJumpTarget &target)
{

    if (!decisions || !target.tile || !target.tile->cb_type)
        return;
    *decisions << "FORWARD_TARGET state=" << cursor << " src=" << src;
    describe(decisions, *target.tile, fpga::CB_NODE_DST, target.dst_node);
    *decisions << " busy="
               << (target.dst_node >= 0 &&
                   target.tile->cb.dst.jump.testBit(target.dst_node))
               << '\n';
}

void ForwardRouteDiagnostic::descend(size_t parent, size_t child)
{

    if (decisions)
        *decisions << "FORWARD_DESCEND parent=" << parent << " child=" << child
                   << '\n';
}

void ForwardRouteDiagnostic::edge(size_t cursor, fpga::Tile &tile, int src,
                                  int joint, int joint2, bool free)
{

    if (!decisions)
        return;
    *decisions << "FORWARD_EDGE state=" << cursor;
    describe(decisions, tile, fpga::CB_NODE_SRC, src);
    *decisions << " joint=" << joint << " joint2=" << joint2
               << " src_busy=" << tile.cb.src.jump.testBit(src)
               << " free=" << free << '\n';
}

void ForwardRouteDiagnostic::target_count(size_t cursor, int src, size_t count)
{

    if (decisions)
        *decisions << "FORWARD_TARGETS state=" << cursor << " src=" << src
                   << " count=" << count << '\n';
}

void ForwardRouteDiagnostic::local_target(size_t cursor,
                                          const fpga::TileLocalTarget &target,
                                          bool busy)
{

    if (!decisions)
        return;
    *decisions << "FORWARD_LOCAL_TARGET state=" << cursor;
    describe(decisions, *target.tile, target.node_type, target.node);
    *decisions << " busy=" << busy << '\n';
}

void ForwardRouteDiagnostic::local(size_t cursor, fpga::Tile &tile, int local)
{

    if (!decisions)
        return;
    *decisions << "FORWARD_LOCAL state=" << cursor;
    describe(decisions, tile, fpga::CB_NODE_LOCAL, local);
    *decisions << " busy=" << tile.cb.local.local.testBit(local)
               << " pin_busy=" << tile.isPinNodeLeased(local)
               << " reserved=" << inputLocalReserved(tile, local)
               << " enabled=1\n";
}

// Record the active typed-node path and bounded repeated-visit evidence.
void ForwardRouteDiagnostic::enter(fpga::Tile &tile, fpga::CBNodeNameType type,
                                   int dst, fpga::Coord aim, size_t cursor,
                                   long long parent, size_t depth,
                                   size_t same_tile_depth,
                                   const TraceRouteForward &trace)
{

    if (audit)
    {
        auto &visit = seen[{&tile, type, dst}];
        ++visit.visits;
        max_same_tile_frames = std::max(max_same_tile_frames, same_tile_depth);
        if (!tile_chain_printed &&
            (same_tile_depth >= 64 || visit.visits == 128))
        {
            tile_chain_printed = true;
            const size_t begin = depth - std::min<size_t>(same_tile_depth, 64);
            std::ostringstream line;
            line << "FORWARD_AUDIT_TILE_CHAIN expanded=" << trace.expanded
                 << " depth=" << depth << " tile=(" << tile.coord.x << ','
                 << tile.coord.y << ')'
                 << " goal_distance=" << distance(aim, tile.coord)
                 << " same_tile_frames=" << same_tile_depth
                 << " visit_count=" << visit.visits << " trigger="
                 << (same_tile_depth >= 64 ? "same_tile_64" : "revisit_128");
            if (begin > 0)
            {
                const auto &[previous, previous_cursor] = path[begin - 1];
                line << " predecessor=" << previous_cursor << '@'
                     << previous.tile->coord.x << ',' << previous.tile->coord.y
                     << ':' << static_cast<int>(previous.type) << ':'
                     << previous.node;
            }

            line << " path=";
            for (size_t i = begin; i < path.size(); ++i)
            {
                const auto &[step, step_cursor] = path[i];
                const auto *exits =
                    step.tile->cb_type->srcNodes(step.type, step.node);
                const auto locals =
                    step.type == fpga::CB_NODE_DST
                        ? step.tile->cb_type->dst_local.get(step.node).local
                    : step.type == fpga::CB_NODE_JOINT
                        ? step.tile->cb_type->joint_local.get(step.node).local
                    : step.type == fpga::CB_NODE_LOCAL
                        ? step.tile->cb_type->local_local.get(step.node).local
                        : NodeMask{};
                const auto joints =
                    step.type == fpga::CB_NODE_DST
                        ? step.tile->cb_type->dst_joint.get(step.node).joint
                    : step.type == fpga::CB_NODE_JOINT
                        ? step.tile->cb_type->joint_joint.get(step.node).joint
                    : step.type == fpga::CB_NODE_SRC
                        ? step.tile->cb_type->src_joint.get(step.node).joint
                        : step.tile->cb_type->local_joint.get(step.node).joint;
                line << (i == begin ? "" : ";") << step_cursor << ':'
                     << static_cast<int>(step.type) << ':' << step.node
                     << "/src=" << (exits ? exits->size() : 0)
                     << "/local=" << locals.firstSetBit()
                     << "/joint=" << joints.firstSetBit();
                if (step.type == fpga::CB_NODE_LOCAL)
                    line << "/link="
                         << fpga::Device::current()
                                .resolveLocalTargets(*step.tile, step.node)
                                .size()
                         << "/input="
                         << step.tile->cb_type->local_input_nodes.testBit(
                                step.node)
                         << "/output="
                         << step.tile->cb_type->local_output_nodes.testBit(
                                step.node);
            }

            *audit << line.str() << '\n';
        }

        if (visit.visits == 1)
        {
            visit.first = cursor;
            visit.first_parent = parent;
        }

        else if (visit.visits == 2 || visit.visits == 128 ||
                 (visit.visits % 1000000) == 0)
        {
            const auto *sources = tile.cb_type->srcNodes(type, dst);
            *audit << "FORWARD_AUDIT_REENTRY first=" << visit.first
                   << " first_parent=" << visit.first_parent
                   << " current=" << cursor << " parent=" << parent
                   << " visits=" << visit.visits << " tile=(" << tile.coord.x
                   << ',' << tile.coord.y << ')'
                   << " type=" << static_cast<int>(type) << " node=" << dst
                   << " depth=" << depth
                   << " goal_distance=" << distance(aim, tile.coord)
                   << " src_exits=" << (sources ? sources->size() : 0);
            if (type == fpga::CB_NODE_LOCAL)
                *audit << " local_input="
                       << tile.cb_type->local_input_nodes.testBit(dst)
                       << " local_output="
                       << tile.cb_type->local_output_nodes.testBit(dst)
                       << " local_next="
                       << tile.cb_type->local_local.get(dst).local.firstSetBit()
                       << " joint_next="
                       << tile.cb_type->local_joint.get(dst).joint.firstSetBit()
                       << " linked_targets="
                       << fpga::Device::current()
                              .resolveLocalTargets(tile, dst)
                              .size();
            else if (type == fpga::CB_NODE_JOINT)
                *audit
                    << " local_next="
                    << tile.cb_type->joint_local.get(dst).local.firstSetBit()
                    << " joint_next="
                    << tile.cb_type->joint_joint.get(dst).joint.firstSetBit();
            *audit << " path_tail=";
            const size_t begin = depth > 12 ? depth - 12 : 0;
            for (size_t i = begin; i < path.size(); ++i)
            {
                const auto &[step, step_cursor] = path[i];
                *audit << (i == begin ? "" : ";") << step_cursor << '@'
                       << step.tile->coord.x << ',' << step.tile->coord.y << ':'
                       << static_cast<int>(step.type) << ':' << step.node;
            }

            *audit << '\n';
        }

        if (trace.expanded % 1000000 == 0)
            *audit << "FORWARD_AUDIT_PROGRESS expanded=" << trace.expanded
                   << " unique=" << seen.size() << " depth=" << depth
                   << " physical_toward=" << toward << " physical_away=" << away
                   << " physical_level=" << level
                   << " failed_src=" << trace.failed_states << '\n';
    }

}

void ForwardRouteDiagnostic::jump(fpga::Coord aim, fpga::Tile &from,
                                  fpga::Tile &to, size_t cursor, int src,
                                  size_t source_choice, size_t depth)
{

    if (!audit)
        return;
    const int before = distance(aim, from.coord);
    const int after = distance(aim, to.coord);
    if (after < before)
        ++toward;
    else if (after > before)
        ++away;
    else
        ++level;
    if (jump_prints++ < 64)
        *audit << "FORWARD_AUDIT_JUMP state=" << cursor << " src=" << src
               << " source_choice=" << source_choice << " from=("
               << from.coord.x << ',' << from.coord.y << ") to=(" << to.coord.x
               << ',' << to.coord.y << ") goal_before=" << before
               << " goal_after=" << after << " depth=" << depth << '\n';
}

// Record why each angle-ranked exit did or did not descend.
void ForwardRouteDiagnostic::source_start(Step &step, int src)
{

    if (audit)
        step.sources.push_back(Source{.src = src});
}

void ForwardRouteDiagnostic::preexisting_deadend(Step &step)
{

    if (audit)
        step.sources.back().preexisting_deadend = true;
}

void ForwardRouteDiagnostic::blocked_target(Step &step)
{

    if (audit && !step.sources.empty())
        ++step.sources.back().blocked_targets;
}

void ForwardRouteDiagnostic::target_descent(Step &step)
{

    if (audit && !step.sources.empty())
        ++step.sources.back().descents;
}

void ForwardRouteDiagnostic::source_targets(Step &step, size_t count)
{

    if (audit)
        step.sources.back().targets = count;
}

void ForwardRouteDiagnostic::source_choice(Step &step, fpga::Tile &tile,
                                           fpga::CBNodeNameType type, int dst,
                                           int src, int joint, int joint2,
                                           bool free, bool root)
{

    if (!audit)
        return;
    auto &choice = step.sources.back();
    choice.joint = joint;
    choice.joint2 = joint2;
    choice.missing_topology = joint == -2;
    choice.source_busy = tile.cb.src.jump.testBit(src);
    choice.destination_busy =
        !root && type == fpga::CB_NODE_DST && tile.cb.dst.jump.testBit(dst);
    choice.joint_busy = (joint >= 0 && tile.cb.joint.jump.testBit(joint)) ||
                        (joint2 >= 0 && tile.cb.joint.jump.testBit(joint2));
    choice.free = free;
}

void ForwardRouteDiagnostic::source_outcomes(Step &step, fpga::Tile &tile,
                                             fpga::CBNodeNameType type, int dst,
                                             fpga::Coord aim, size_t cursor,
                                             size_t depth, size_t source_count)
{

    if (audit && type == fpga::CB_NODE_DST && source_count >= 8 &&
        !step.reported && source_reports < 64)
    {
        step.reported = true;
        ++source_reports;
        size_t deadend_count = 0, source_busy_count = 0;
        size_t joint_busy_count = 0, missing_topology_count = 0;
        size_t available_count = 0, descended_count = 0;
        for (const auto &choice : step.sources)
        {
            deadend_count += choice.preexisting_deadend;
            source_busy_count += choice.source_busy;
            joint_busy_count += choice.joint_busy;
            missing_topology_count += choice.missing_topology;
            available_count += choice.free;
            descended_count += choice.descents > 0;
        }

        std::ostringstream line;
        line << "FORWARD_AUDIT_SOURCE_OUTCOMES state=" << cursor << " tile=("
             << tile.coord.x << ',' << tile.coord.y << ')' << " dst=" << dst
             << " depth=" << depth
             << " goal_distance=" << distance(aim, tile.coord)
             << " ordered_sources=" << source_count
             << " attempts=" << step.sources.size()
             << " deadend=" << deadend_count
             << " source_busy=" << source_busy_count
             << " joint_busy=" << joint_busy_count
             << " topology_missing=" << missing_topology_count
             << " available=" << available_count
             << " descended=" << descended_count << " choices=";
        for (const auto &choice : step.sources)
        {
            const char *outcome = choice.preexisting_deadend ? "deadend"
                                  : !choice.free        ? "blocked_before_jump"
                                  : choice.targets == 0 ? "no_targets"
                                  : choice.descents == 0
                                      ? "no_free_landing"
                                      : "descended_then_failed";
            line << choice.src << ':' << outcome
                 << "/topology_missing=" << choice.missing_topology
                 << "/src_busy=" << choice.source_busy
                 << "/dst_busy=" << choice.destination_busy
                 << "/joint=" << choice.joint << "/joint2=" << choice.joint2
                 << "/joint_busy=" << choice.joint_busy
                 << "/targets=" << choice.targets
                 << "/blocked_targets=" << choice.blocked_targets
                 << "/descents=" << choice.descents << ';';
        }

        *audit << line.str() << '\n';
    }

}

void ForwardRouteDiagnostic::speculative_skip(fpga::Tile &tile, int src,
                                              size_t depth, size_t cursor)
{

    ++speculative_deadend_skips;
    if (audit && speculative_deadend_skips <= 16)
        *audit << "FORWARD_AUDIT_SPECULATIVE_DEADEND_SKIP tile=("
               << tile.coord.x << ',' << tile.coord.y << ") src=" << src
               << " depth=" << depth << " state=" << cursor << '\n';
}

// Cross-check the shared fast selector against the loaded angle order.
void ForwardRouteDiagnostic::audit_candidates(
    fpga::CBType &type, fpga::CBNodeNameType from_type, int from_node,
    fpga::Coord angle, const NodeMask &allowed,
    const std::vector<uint16_t> &active,
    const std::vector<uint16_t> &unresolved)
{

    static const bool enabled = std::getenv("SCALEPNR_AUDIT_FORWARD_SELECTION");
    if (!enabled)
        return;
    std::vector<uint16_t> expected_active, expected_unresolved;
    for (uint16_t src : type.orderedSrcNodes(from_type, from_node, angle))
    {
        if (!allowed.testBit(src))
            continue;
        if (src >= CB_MAX_NODES || type.dst_by_src.get(src).empty())
            expected_unresolved.push_back(src);
        else
            expected_active.push_back(src);
    }

    PNR_ASSERT(active == expected_active && unresolved == expected_unresolved,
               "angle-ranked forward source selection changed for node {}",
               from_node);
}

int ForwardRouteDiagnostic::distance(fpga::Coord aim, fpga::Coord point)
{
    return aim.x < 0 || aim.y < 0
               ? -1
               : std::abs(aim.x - point.x) + std::abs(aim.y - point.y);
}

void ForwardRouteDiagnostic::describe(std::ostream *out, fpga::Tile &tile,
                                      fpga::CBNodeNameType type, int id)
{
    *out << " tile=(" << tile.coord.x << ',' << tile.coord.y
         << ") type=" << static_cast<int>(type) << " node=" << id;
    const auto *name =
        tile.cb_type && id >= 0 ? tile.cb_type->nodeName(type, id) : nullptr;
    *out << " name=" << (name ? *name : "?");
}

void ForwardRouteDiagnostic::blocker(
    fpga::Tile &tile, const char *reason,
    std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes,
    TraceRouteForward &trace)
{
    if (!RouteCongestionTrace::current)
        return;
    fpga::CBState search_state = tile.cb;
    ++trace.snapshot_lookup_checks;
    const auto found = saved_index.find(&tile);
    if (found != saved_index.end())
        tile.cb.restoreLeases(saved[found->second].second);
    RestoreState restore{tile, search_state};
    RouteCongestionTrace::current->blocked(tile, reason, nodes, &search_state);
}

} // namespace pnr
