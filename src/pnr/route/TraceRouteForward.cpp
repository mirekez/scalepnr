#include "TraceRouteForward.h"
#include "Device.h"
#include "InputRepairBudget.h"
#include "MovingSourceReachability.h"
#include "RouteDiagnostics.h"
#include "RouteSearch.h"
#include "RouteTaskTiming.h"
#include "TraceRouteCommon.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iterator>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>

namespace pnr
{
namespace
{
constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;
} // namespace

// Drive the Moving destination search from one source-connected anchor.
bool TraceRouteForward::run(const RoutingAnchor &anchor, fpga::Coord aim)
{
    assert(docking);
    success = cancelled = budget_exhausted = false;
    expanded = edge_checks = failed_states = failure_cache_hits = 0;
    tile_visit_rejects = snapshot_lookup_checks = 0;
    fragments.clear();
    std::vector<fpga::Wire> &path = fragments;
    size_t queued = 0;
    size_t unproven_exits = 0;
    if (diagnostic)
        diagnostic->begin_anchor(anchor);
    if (!anchor.tile || !anchor.tile->cb_type || anchor.node < 0 ||
        (anchor.starts_at_dst &&
         !anchor.tile->cb.dst.jump.testBit(anchor.node)))
        return false; // A free DST is not a source-connected branch point.
    // Each recursive call owns the cursor for one typed node.
    next(anchor, aim, path, queued, unproven_exits, nullptr, *anchor.tile,
         anchor.node,
         anchor.starts_at_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL, -1, 1,
         1);
    if (diagnostic)
        diagnostic->finish_anchor(queued, *this);
    return success;
}

// Recursively explore one node, resuming its local cursors after each child.
bool TraceRouteForward::next(const RoutingAnchor &anchor, fpga::Coord aim,
                             std::vector<fpga::Wire> &path, size_t &queued,
                             size_t &unproven_exits, LeaseScope *active,
                             fpga::Tile &tile, int dst,
                             fpga::CBNodeNameType type, long long parent,
                             size_t depth, size_t same_tile_depth)
{
    if (expansions_used >= max_expansions)
    {
        cancelled = true;
        budget_exhausted = true;
        return false;
    }

    ++expansions_used;
    // A deadline diagnostic can also activate during the docking callback.
    if (diagnostic)
        diagnostic->attach_deadline_stream();
    const size_t cursor = queued++;
    const bool root = depth == 1;
    ForwardRouteDiagnostic::PathScope diagnostic_scope{diagnostic, tile, type,
                                                       dst, cursor};
    ForwardRouteDiagnostic::Step diagnostic_step;
    const std::vector<uint16_t> *sources = nullptr;
    size_t next_source = 0;
    fpga::CBType::FastSearchCursor source_cursor;
    std::vector<fpga::TileJumpTarget> targets;
    size_t next_target = 0;
    std::vector<fpga::TileLocalTarget> local_targets;
    int src = -1, joint = -1, joint2 = -1;
    // An ancestor cannot prove its exit exhausted after a descendant failed
    // only because that ancestor's own speculative prefix occupied a SRC.
    size_t source_proof_start = unproven_exits;
    const NodeMask *locals = nullptr, *joints = nullptr;
    int local_cursor = 0, joint_cursor = 0;
    // Enter a typed node once, reject leased arrivals, and probe for a
    // placement before collecting its outgoing choices.
    {
        if (type == fpga::CB_NODE_SRC && tile.cb.src_deadend.jump.testBit(dst))
        {
            ++failure_cache_hits;
            return false;
        }

        if (!root &&
            (node_mask(tile, type).testBit(dst) ||
             (type == fpga::CB_NODE_LOCAL && tile.isPinNodeLeased(dst))))
        {
            if (active && active->owns(node_mask(tile, type), dst))
                ++unproven_exits;
            if (diagnostic)
                diagnostic->reject(cursor);
            blocked(tile, "forward_arrival_lease", {{type, dst}});
            return false;
        }

        ++expanded;
        // Opt-in audit records repeated typed nodes and long same-tile
        // chains.
        if (diagnostic)
        {
            diagnostic->enter(tile, type, dst, aim, cursor, parent, depth,
                              same_tile_depth, *this);
            diagnostic->state(cursor, parent, tile, type, dst);
            diagnostic->before_probe(type, docking->hasLocalTerminal());
        }

        std::vector<fpga::Wire> terminal;
        if (docking->tryTerminal(tile, type, dst, root && anchor.starts_at_dst,
                                 terminal))
        {
            success = true;
            path.insert(path.end(), terminal.begin(), terminal.end());
            if (diagnostic)
                diagnostic->grounded(cursor, type == fpga::CB_NODE_LOCAL);
            return true;
        }
        // A failed endpoint probe depends on this sink and on the caller's
        // current docking policy. It cannot prove a global SRC deadend.
        if ((docking->blocked_arrival || docking->local_terminal) &&
            ((type == fpga::CB_NODE_DST && docking->sink) ||
             (type == fpga::CB_NODE_LOCAL && docking->local_terminal)))
            ++unproven_exits;

        // A packed input reserves its own terminal LOCAL. The caller's
        // terminal probe decides ownership; a declined reservation cannot
        // become an intermediate routing node.
        if (type == fpga::CB_NODE_LOCAL &&
            docking->blocksReservedTransit(tile, dst, root))
        {
            blocked(tile, "forward_local_reservation", {{type, dst}});
            return false;
        }

        // The loaded numeric priority keeps angle first and short hops
        // next. A resolved SRC is already the selected exit; continue its
        // numeric jump rather than interpreting its index as a DST or
        // LOCAL.
        if (type == fpga::CB_NODE_SRC)
        {
            src = dst;
            targets = fpga::Device::current().resolveJumpTargets(tile, dst);
        }

        else
        {
            sources = &tile.cb_type->orderedSrcNodes(type, dst,
                                                     aim.x >= 0 && aim.y >= 0
                                                         ? aim - tile.coord
                                                         : fpga::Coord{0, 0});
        }

        locals = type == fpga::CB_NODE_DST
                     ? &tile.cb_type->dst_local.get(dst).local
                 : type == fpga::CB_NODE_JOINT
                     ? &tile.cb_type->joint_local.get(dst).local
                 : type == fpga::CB_NODE_LOCAL
                     ? &tile.cb_type->local_local.get(dst).local
                     : nullptr;
        joints = type == fpga::CB_NODE_DST
                     ? &tile.cb_type->dst_joint.get(dst).joint
                 : type == fpga::CB_NODE_JOINT
                     ? &tile.cb_type->joint_joint.get(dst).joint
                 : type == fpga::CB_NODE_SRC
                     ? &tile.cb_type->src_joint.get(dst).joint
                     : &tile.cb_type->local_joint.get(dst).joint;
        if (type == fpga::CB_NODE_LOCAL)
            local_targets =
                fpga::Device::current().resolveLocalTargets(tile, dst);
        if (diagnostic)
            diagnostic->source_count(cursor, sources ? sources->size() : 0);
    }

    while (!budget_exhausted)
    {
        // Follow resolved physical jumps before selecting another source exit.
        if (next_target < targets.size())
        {
            const auto target = targets[next_target++];
            if (diagnostic)
                diagnostic->target(cursor, src, target);
            if (!target.tile || !target.tile->cb_type || target.dst_node < 0 ||
                target.tile->cb.dst.jump.testBit(target.dst_node))
            {
                if (diagnostic)
                    diagnostic->blocked_target(diagnostic_step);
                if (target.tile && target.tile->cb_type && target.dst_node >= 0)
                {
                    ++unproven_exits;
                    blocked(*target.tile, "forward_target_lease",
                            {{fpga::CB_NODE_DST, target.dst_node}});
                    docking->recordBlockedArrival(*target.tile,
                                                  target.dst_node);
                }

                continue;
            }

            if (type == fpga::CB_NODE_DST && target.tile == &tile &&
                target.dst_node == dst)
                continue;
            if (diagnostic)
            {
                diagnostic->target_descent(diagnostic_step);
                diagnostic->jump(aim, tile, *target.tile, cursor, src,
                                 next_source, depth);
            }

            fpga::Wire wire;
            wire.from = tile.coord;
            wire.to = target.tile->coord;
            wire.local = dst;
            wire.jump = src;
            wire.dst = target.dst_node;
            wire.route_jump = target.jump_node;
            wire.joint = joint;
            wire.joint2 = joint2;
            wire.pos = root ? (anchor.starts_at_dst ? ROUTE_POS_FORK : 0)
                            : ROUTE_POS_TRANSIT;
            wire.owns_dst = !(root && anchor.starts_at_dst);
            if (const auto *name = tile.cb_type->nodeName(type, dst))
                wire.from_wire_name = *name;
            wire.src_wire_name = srcWireName(tile, type, dst, src, joint,
                                             std::string(wire.from_wire_name));
            wire.dst_wire_name = target.dst_wire;
            const size_t path_begin = path.size();
            LeaseScope leases{tile, active, diagnostic, *this};
            if (diagnostic)
                diagnostic->descend(cursor, queued);
            leases.lease(tile.cb.src.jump, src);
            if (!root || type == fpga::CB_NODE_DST)
                leases.lease(node_mask(tile, type), dst);
            if (joint >= 0)
                leases.lease(tile.cb.joint.jump, joint);
            if (joint2 >= 0)
                leases.lease(tile.cb.joint.jump, joint2);
            if (type == fpga::CB_NODE_SRC)
            {
                append_arc(path, tile.coord, fpga::CB_NODE_SRC, dst,
                           target.tile->coord, fpga::CB_NODE_DST,
                           target.dst_node);
            }

            else if (type == fpga::CB_NODE_JOINT ||
                     (type == fpga::CB_NODE_LOCAL && !root))
            {
                auto from_type = type;
                int from_node = dst;
                for (int joint : {joint2, joint})
                    if (joint >= 0)
                    {
                        append_arc(path, tile.coord, from_type, from_node,
                                   tile.coord, fpga::CB_NODE_JOINT, joint);
                        from_type = fpga::CB_NODE_JOINT;
                        from_node = joint;
                    }

                append_arc(path, tile.coord, from_type, from_node, tile.coord,
                           fpga::CB_NODE_SRC, src);
                append_arc(path, tile.coord, fpga::CB_NODE_SRC, src,
                           target.tile->coord, fpga::CB_NODE_DST,
                           target.dst_node);
            }

            else
                path.push_back(wire);
            const bool found =
                next(anchor, aim, path, queued, unproven_exits, &leases,
                     *target.tile, target.dst_node, fpga::CB_NODE_DST, cursor,
                     depth + 1, target.tile == &tile ? same_tile_depth + 1 : 1);
            if (found)
                return true;
            path.resize(path_begin);
            if (budget_exhausted)
                return false;
            continue;
        }

        // Consume this call's angle-ranked SRC cursor without restarting it.
        if (sources && next_source < sources->size())
        {
            if (src >= 0 && unproven_exits == source_proof_start)
                mark_deadend(tile, src, depth, cursor, active, unproven_exits);
            source_proof_start = unproven_exits;
            src = tile.cb_type->fastSearchNextHop(
                aim.x >= 0 && aim.y >= 0 ? aim - tile.coord : fpga::Coord{0, 0},
                source_cursor, type, dst);
            ++next_source;
            if (diagnostic)
                diagnostic->source_start(diagnostic_step, src);
            if (tile.cb.src_deadend.jump.testBit(src))
            {
                if (diagnostic)
                    diagnostic->preexisting_deadend(diagnostic_step);
                ++failure_cache_hits;
                continue;
            }

            ++edge_checks;
            joint = selectJointToSrc(tile, type, dst, src, &joint2, true);
            const bool free =
                joint != -2 && canLeaseJump(tile.cb, dst, src, joint, joint2,
                                            root || type != fpga::CB_NODE_DST);
            if (joint == -2)
                joint = selectJointToSrc(tile, type, dst, src, &joint2);
            if (diagnostic)
                diagnostic->source_choice(diagnostic_step, tile, type, dst, src,
                                          joint, joint2, free, root);
            if (diagnostic)
                diagnostic->edge(cursor, tile, src, joint, joint2, free);
            if (!free)
            {
                blocked(tile,
                        joint == -2 ? "forward_missing_topology"
                                    : "forward_exit_lease",
                        {{type, dst},
                         {fpga::CB_NODE_SRC, src},
                         {fpga::CB_NODE_JOINT, joint},
                         {fpga::CB_NODE_JOINT, joint2}});
                if (joint != -2)
                    ++unproven_exits;
                else if (unproven_exits == source_proof_start)
                    mark_deadend(tile, src, depth, cursor, active,
                                 unproven_exits);
                src = -1;
                continue;
            }

            targets = fpga::Device::current().resolveJumpTargets(tile, src);
            if (diagnostic)
                diagnostic->source_targets(diagnostic_step, targets.size());
            next_target = 0;
            if (diagnostic)
                diagnostic->target_count(cursor, src, targets.size());
            continue;
        }

        if (sources && src >= 0)
        {
            if (unproven_exits == source_proof_start)
                mark_deadend(tile, src, depth, cursor, active, unproven_exits);
            src = -1;
        }

        // After jump exits, try numeric LOCAL and JOINT bridges in this call.
        // Resume its parent only when every continuation has failed.
        const int local = locals ? locals->nextSetBit(local_cursor) : -1;
        if (local < 0)
        {
            // Internal joints may lead to LOCALs rather than jump sources.
            // Retain their cursor instead of relying on compact terminal
            // tables.
            const int joint = joints ? joints->nextSetBit(joint_cursor) : -1;
            if (joint >= 0)
            {
                if (diagnostic)
                    diagnostic->source_outcomes(diagnostic_step, tile, type,
                                                dst, aim, cursor, depth,
                                                sources ? sources->size() : 0);
                joint_cursor = joint + 1;
                if (tile.cb.joint.jump.testBit(joint) ||
                    (type == fpga::CB_NODE_JOINT && joint == dst))
                {
                    if (tile.cb.joint.jump.testBit(joint))
                        blocked(tile, "forward_joint_lease",
                                {{fpga::CB_NODE_JOINT, joint}});
                    continue;
                }

                const size_t path_begin = path.size();
                LeaseScope leases{tile, active, diagnostic, *this};
                if (diagnostic)
                    diagnostic->descend(cursor, queued);
                leases.lease(node_mask(tile, type), dst);
                bool restore_landing = false;
                bool previous_landing = false;
                if (!path.empty() &&
                    path.back().type == fpga::Wire::WIRE_CROSSBAR)
                {
                    restore_landing = true;
                    previous_landing = path.back().owns_landing;
                    path.back().owns_landing = true;
                }

                append_arc(path, tile.coord, type, dst, tile.coord,
                           fpga::CB_NODE_JOINT, joint);
                const bool found =
                    next(anchor, aim, path, queued, unproven_exits, &leases,
                         tile, joint, fpga::CB_NODE_JOINT, cursor, depth + 1,
                         same_tile_depth + 1);
                if (found)
                    return true;
                path.resize(path_begin);
                if (restore_landing && !path.empty())
                    path.back().owns_landing = previous_landing;
                if (budget_exhausted)
                    return false;
                continue;
            }

            // Rank physical LOCAL links toward the placement before descending.
            if (type == fpga::CB_NODE_LOCAL && !local_targets.empty())
            {
                const int wanted =
                    aim.x >= 0 && aim.y >= 0 ? direction(aim - tile.coord) : -1;
                size_t preferred = 0;
                int preferred_angle = 9, preferred_distance = 1 << 30;
                for (size_t i = 0; i < local_targets.size(); ++i)
                {
                    const auto &candidate = local_targets[i];
                    if (!candidate.tile || !candidate.tile->cb_type)
                        continue;
                    const auto delta = candidate.tile->coord - tile.coord;
                    const int actual = direction(delta);
                    int angle = 0;
                    if (actual >= 0 && wanted >= 0)
                    {
                        angle = 8;
                        for (int offset = 0; offset < 8; ++offset)
                            if (actual == ((wanted + offsets[offset] + 8) & 7))
                            {
                                angle = offset;
                                break;
                            }

                    }

                    const int distance = std::abs(delta.x) + std::abs(delta.y);
                    if (angle < preferred_angle ||
                        (angle == preferred_angle &&
                         distance < preferred_distance))
                    {
                        preferred = i;
                        preferred_angle = angle;
                        preferred_distance = distance;
                    }

                }

                const auto target = local_targets[preferred];
                local_targets.erase(local_targets.begin() + preferred);
                if (!target.tile || !target.tile->cb_type || target.node < 0 ||
                    target.node >= CB_MAX_NODES ||
                    (target.node_type != fpga::CB_NODE_LOCAL &&
                     target.node_type != fpga::CB_NODE_JOINT &&
                     target.node_type != fpga::CB_NODE_SRC &&
                     target.node_type != fpga::CB_NODE_DST) ||
                    (target.tile == &tile && target.node_type == type &&
                     target.node == dst))
                    continue;
                ++edge_checks;
                const bool busy = node_mask(*target.tile, target.node_type)
                                      .testBit(target.node) ||
                                  (target.node_type == fpga::CB_NODE_LOCAL &&
                                   (target.tile->isPinNodeLeased(target.node) ||
                                    !docking->canEnterReservedLocal(
                                        *target.tile, target.node)));
                if (diagnostic)
                    diagnostic->local_target(cursor, target, busy);
                if (busy)
                {
                    blocked(*target.tile, "forward_local_target_lease",
                            {{target.node_type, target.node}});
                    continue;
                }

                if (target.node_type == fpga::CB_NODE_SRC &&
                    target.tile->cb.src_deadend.jump.testBit(target.node))
                {
                    ++failure_cache_hits;
                    continue;
                }

                const size_t path_begin = path.size();
                LeaseScope leases{tile, active, diagnostic, *this};
                if (diagnostic)
                    diagnostic->descend(cursor, queued);
                if (!root)
                    leases.lease(tile.cb.local.local, dst);
                append_arc(path, tile.coord, fpga::CB_NODE_LOCAL, dst,
                           target.tile->coord, target.node_type, target.node);
                const bool found = next(
                    anchor, aim, path, queued, unproven_exits, &leases,
                    *target.tile, target.node, target.node_type, cursor,
                    depth + 1, target.tile == &tile ? same_tile_depth + 1 : 1);
                if (found)
                    return true;
                path.resize(path_begin);
                if (budget_exhausted)
                    return false;
                continue;
            }

            // This call is exhausted; release its suffix and resume the
            // parent.
            if (type == fpga::CB_NODE_SRC &&
                unproven_exits == source_proof_start)
                mark_deadend(tile, dst, depth, cursor, active, unproven_exits);
            return false;
        }

        // A same-tile LOCAL may lead to another exit or to the terminal probe.
        local_cursor = local + 1;
        if (diagnostic)
            diagnostic->local(cursor, tile, local);
        if (tile.cb.local.local.testBit(local) || tile.isPinNodeLeased(local))
        {
            blocked(tile, "forward_local_lease",
                    {{fpga::CB_NODE_LOCAL, local}});
            continue;
        }

        if (!docking->canEnterReservedLocal(tile, local) ||
            (type == fpga::CB_NODE_LOCAL && local == dst))
            continue;
        const auto *exits = tile.cb_type->srcNodes(fpga::CB_NODE_LOCAL, local);
        if ((!exits || exits->empty()) &&
            tile.cb_type->local_local.get(local).local == NodeMask{} &&
            tile.cb_type->local_joint.get(local).joint == NodeMask{} &&
            tile.cb_type->local_by_local.get(local).empty() &&
            !docking->hasLocalTerminal())
            continue;
        const size_t path_begin = path.size();
        LeaseScope leases{tile, active, diagnostic, *this};
        if (diagnostic)
            diagnostic->descend(cursor, queued);
        leases.lease(node_mask(tile, type), dst);
        bool restore_landing = false;
        bool previous_landing = false;
        if (!path.empty() && path.back().type == fpga::Wire::WIRE_CROSSBAR)
        {
            restore_landing = true;
            previous_landing = path.back().owns_landing;
            path.back().owns_landing = true;
        }

        append_arc(path, tile.coord, type, dst, tile.coord, fpga::CB_NODE_LOCAL,
                   local);
        const bool found = next(anchor, aim, path, queued, unproven_exits,
                                &leases, tile, local, fpga::CB_NODE_LOCAL,
                                cursor, depth + 1, same_tile_depth + 1);
        if (found)
            return true;
        path.resize(path_begin);
        if (restore_landing && !path.empty())
            path.back().owns_landing = previous_landing;
        if (budget_exhausted)
            return false;
    }

    return false;
}

// Forward blockers to the optional log without changing the search proof.
void TraceRouteForward::blocked(
    fpga::Tile &tile, const char *reason,
    std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes)
{
    if (diagnostic)
        diagnostic->blocker(tile, reason, nodes, *this);
}

// Try source-connected anchors until one forward trace completes.
TraceRouteForward
routeForwardToPlacement(const std::vector<RoutingAnchor> &anchors,
                        const Docking &docking, size_t max_expansions,
                        std::ostream *decisions,
                        const ForwardRouteDiagnostic::Observer &observe,
                        fpga::Coord aim, std::ostream *audit)
{
    TraceRouteForward trace;
    trace.docking = &docking;
    trace.max_expansions =
        max_expansions ? max_expansions : trace.max_expansions;
    ForwardRouteDiagnostic diagnostic;
    diagnostic.decisions = decisions;
    diagnostic.audit = audit;
    diagnostic.observer = observe ? &observe : nullptr;
    trace.diagnostic = &diagnostic;
    size_t total_expanded = 0;
    size_t total_edge_checks = 0;
    size_t total_failed_states = 0;
    size_t total_failure_cache_hits = 0;
    size_t total_tile_visit_rejects = 0;
    size_t total_snapshot_lookup_checks = 0;
    // Each candidate starts one atomic forward trace for this connection.
    for (size_t i = 0; i < anchors.size(); ++i)
    {
        diagnostic.attempt(i);
        trace.run(anchors[i], aim);
        trace.anchor = i;
        total_expanded += trace.expanded;
        total_edge_checks += trace.edge_checks;
        total_failed_states += trace.failed_states;
        total_failure_cache_hits += trace.failure_cache_hits;
        total_tile_visit_rejects += trace.tile_visit_rejects;
        total_snapshot_lookup_checks += trace.snapshot_lookup_checks;
        if (trace.success || trace.budget_exhausted)
            break;
    }

    trace.expanded = total_expanded;
    trace.edge_checks = total_edge_checks;
    trace.failed_states = total_failed_states;
    trace.failure_cache_hits = total_failure_cache_hits;
    trace.tile_visit_rejects = total_tile_visit_rejects;
    trace.snapshot_lookup_checks = total_snapshot_lookup_checks;
    trace.docking = nullptr;
    trace.diagnostic = nullptr;
    return trace;
}

} // namespace pnr

// Share angle-ranked candidate selection with Basic and Fanouts routing.
void pnr::TraceRouteForward::candidates(fpga::CBType &type,
                                        fpga::CBNodeNameType from_type,
                                        int from_node, fpga::Coord angle,
                                        const NodeMask &allowed,
                                        std::vector<uint16_t> &active,
                                        std::vector<uint16_t> &unresolved)
{
    active.clear();
    unresolved.clear();
    fpga::CBType::FastSearchCursor cursor;
    for (int src; (src = type.fastSearchNextHop(angle, cursor, from_type,
                                                from_node)) >= 0;)
    {
        if (!allowed.testBit(src))
            continue;
        if (src >= CB_MAX_NODES || type.dst_by_src.get(src).empty())
            unresolved.push_back(static_cast<uint16_t>(src));
        else
            active.push_back(static_cast<uint16_t>(src));
    }

    ForwardRouteDiagnostic::audit_candidates(type, from_type, from_node, angle,
                                             allowed, active, unresolved);
}
