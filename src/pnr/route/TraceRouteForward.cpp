#include "TraceRouteForward.h"
#include "Device.h"
#include "Docking.h"
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

// A resource Tile and its mapped routing CB identify the same destination.
bool destinationTile(const fpga::Tile &landing, fpga::Coord destination)
{
    if (landing.coord == destination)
        return true;
    const auto *resource =
        fpga::Device::current().getTile(destination.x, destination.y);
    return resource && resource->cb_coord == landing.coord;
}

} // namespace

// A terminal landing does not need another SRC and may bypass a learned mark.
bool TraceRouteForward::isDeadend(fpga::Tile &tile, int src,
                                  fpga::Coord destination)
{
    if (src < 0 || !tile.cb.src_deadend.jump.testBit(src))
        return false;
    fpga::Device::JumpTargetCursor cursor;
    fpga::TileJumpTarget target;
    while (fpga::Device::current().nextJumpTarget(tile, src, cursor, target))
        if (target.tile && destinationTile(*target.tile, destination))
            return false;
    return true;
}

// Moving checks the reached Tile's placement eligibility, not its angle hint.
// Fixed-endpoint tracing keeps the mapped destination exception.
bool TraceRouteForward::canFinishJump(fpga::Tile &tile, int src,
                                      fpga::Coord aim) const
{
    fpga::Device::JumpTargetCursor cursor;
    fpga::TileJumpTarget target;
    while (fpga::Device::current().nextJumpTarget(tile, src, cursor, target))
    {
        if (!target.tile)
            continue;
        if (docking->sink && docking->terminal_tiles.empty())
        {
            if (docking->canPlaceIn(*target.tile))
                return true;
        }

        else if (destinationTile(*target.tile, aim))
            return true;
    }

    return false;
}

// Only the next Tile's complete SRC mask can establish forward exhaustion.
bool TraceRouteForward::markDeadend(fpga::Tile &tile, int src)
{
    if (src < 0 || tile.cb.src_deadend.jump.testBit(src))
        return false;
    fpga::Device::JumpTargetCursor cursor;
    fpga::TileJumpTarget target;
    bool reached = false;
    while (fpga::Device::current().nextJumpTarget(tile, src, cursor, target))
    {
        reached = true;
        if (!target.tile || !target.tile->cb_type)
            return false;
        target.tile->cb_type->ensureDerivedMasks();
        const NodeMask eligible = target.tile->cb_type->valid_src_nodes;
        // Only persistent marks can prove a persistent failure. Occupancy,
        // including this prefix's leases, may be released or preempted.
        if ((eligible & ~target.tile->cb.src_deadend.jump) != NodeMask{})
            return false;
    }

    if (!reached)
        return false;
    tile.cb.src_deadend.jump.setBit(src);
    return true;
}

// Report a newly learned mark without deriving it from failed placement probes.
void TraceRouteForward::recordDeadend(fpga::Tile &tile, int src, size_t depth,
                                      size_t cursor)
{
    if (markDeadend(tile, src))
    {
        ++failed_states;
        if (std::getenv("SCALEPNR_DEBUG_FORWARD_DEADEND"))
            std::fprintf(
                stderr,
                "FORWARD_DEADEND tile=(%d,%d) src=%d depth=%zu cursor=%zu\n",
                tile.coord.x, tile.coord.y, src, depth, cursor);
    }

}

// Drive the Moving destination search from one source-connected anchor.
bool TraceRouteForward::run(const RoutingAnchor &anchor, fpga::Coord aim)
{
    assert(docking);
    success = cancelled = budget_exhausted = false;
    expanded = edge_checks = failed_states = failure_cache_hits = 0;
    tile_visit_rejects = snapshot_lookup_checks = 0;
    fragments.clear();
    size_t queued = 0;
    if (diagnostic)
        diagnostic->begin_anchor(anchor);
    if (!anchor.tile || !anchor.tile->cb_type || anchor.node < 0 ||
        (anchor.starts_at_dst &&
         !anchor.tile->cb.dst.jump.testBit(anchor.node)))
        return false; // A free DST is not a source-connected branch point.
    // Each recursive call owns one Tile-hop cursor.
    next(anchor, aim, queued, *anchor.tile, anchor.node,
         anchor.starts_at_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL, -1, 1,
         1);
    if (diagnostic)
        diagnostic->finish_anchor(queued, *this);
    return success;
}

// Recurse only for a physical hop, then resume this Tile's exit cursor.
bool TraceRouteForward::next(const RoutingAnchor &anchor, fpga::Coord aim,
                             size_t &queued, fpga::Tile &tile, int arrival,
                             fpga::CBNodeNameType type, long long parent,
                             size_t depth, size_t same_tile_depth,
                             bool terminal_only)
{
    if (expansions_used >= max_expansions)
    {
        cancelled = budget_exhausted = true;
        return false;
    }

    ++expansions_used;
    const size_t cursor = queued++;
    const bool root = depth == 1;
    if (!root && node_mask(tile, type).testBit(arrival))
        return false;
    if (!tile.cb_type)
        return false;
    tile.cb_type->ensureDerivedMasks();
    ++expanded;
    ForwardRouteDiagnostic::PathScope diagnostic_scope{diagnostic, tile, type,
                                                       arrival, cursor};
    if (diagnostic)
    {
        diagnostic->attach_deadline_stream();
        diagnostic->enter(tile, type, arrival, aim, cursor, parent, depth,
                          same_tile_depth, *this);
        diagnostic->state(cursor, parent, tile, type, arrival);
    }

    // Docking owns terminal enumeration; this call owns only one Tile hop.
    if (diagnostic &&
        ((type == fpga::CB_NODE_DST && docking->sink &&
          docking->canPlaceIn(tile)) ||
         (type == fpga::CB_NODE_LOCAL && docking->hasLocalTerminal() &&
          !tile.cb.local.local.testBit(arrival))))
        diagnostic->before_probe(type, docking->hasLocalTerminal());
    if (docking->appendTerminal(tile, type, arrival,
                                root && anchor.starts_at_dst, terminal_only,
                                fragments))
    {
        success = true;
        if (diagnostic)
            diagnostic->grounded(cursor, type == fpga::CB_NODE_LOCAL);
        return true;
    }

    if (!docking->canContinueIn(tile))
        return false;
    if (type == fpga::CB_NODE_LOCAL &&
        docking->blocksReservedTransit(tile, arrival, root))
        return false;

    // Borrow the loaded masks; scalar cursors remember one untried bit.
    // Recursion starts only after a physical SRC-to-DST jump.
    static const NodeMask empty_mask;
    const NodeMask *reachable = type == fpga::CB_NODE_DST
                                    ? &tile.cb_type->dst_src.get(arrival).jump
                                : type == fpga::CB_NODE_LOCAL
                                    ? &tile.cb_type->local_src.get(arrival).jump
                                    : &empty_mask;
    const NodeMask *locals = type == fpga::CB_NODE_DST
                                 ? &tile.cb_type->dst_local.get(arrival).local
                             : type == fpga::CB_NODE_LOCAL
                                 ? &tile.cb_type->local_local.get(arrival).local
                                 : &empty_mask;
    const NodeMask *joints = type == fpga::CB_NODE_DST
                                 ? &tile.cb_type->dst_joint.get(arrival).joint
                             : type == fpga::CB_NODE_LOCAL
                                 ? &tile.cb_type->local_joint.get(arrival).joint
                                 : &empty_mask;
    fpga::CBType::FastSearchCursor exits;
    fpga::Device::JumpTargetCursor landings;
    int src = -1, local = -1, joint = -1;
    int next_local = 0, next_joint = 0;
    unsigned bridge_phase = 0;
    bool use_local = false, use_joint = false;
    const fpga::Coord angle =
        aim.x >= 0 && aim.y >= 0 ? aim - tile.coord : fpga::Coord{0, 0};

    // This single cursor tries physical exits; recursion starts only after a
    // resolved SRC-to-DST jump into the next Tile/CB.
    while (!budget_exhausted)
    {
        if (src < 0)
        {
            // Marked exits remain candidates only for direct endpoint docking.
            // Occupied exits are handed to the caller's validated preemption.
            src = tile.cb_type->fastSearchNextHop(angle, exits, *reachable,
                                                  tile.cb.src.jump);
            if (src < 0)
            {
                exits = {};
                if (bridge_phase == 0)
                    bridge_phase = 1;
                if (bridge_phase == 1)
                {
                    local = locals->nextSetBit(next_local);
                    if (local >= 0)
                    {
                        next_local = local + 1;
                        if ((type == fpga::CB_NODE_LOCAL && local == arrival) ||
                            tile.cb.local.local.testBit(local) ||
                            (tile.pin_state.leased_nodes.testBit(local) &&
                             !docking->hasLocalTerminal()))
                            continue;
                        use_local = true;
                        use_joint = false;
                        if (tryLocalTerminal(tile, type, arrival, local,
                                             terminal_only, cursor))
                            return true;
                        reachable =
                            tile.pin_state.leased_nodes.testBit(local) ||
                                    inputLocalReserved(tile, local)
                                ? &empty_mask
                                : &tile.cb_type->local_src.get(local).jump;
                        continue;
                    }

                    bridge_phase = 2;
                }

                joint = joints->nextSetBit(next_joint);
                if (joint < 0)
                    return tryFullInternal(anchor, aim, queued, tile, type,
                                           arrival, cursor, depth,
                                           same_tile_depth, terminal_only);
                next_joint = joint + 1;
                if (tile.cb.joint.jump.testBit(joint))
                    continue;
                use_local = false;
                use_joint = true;
                reachable = &tile.cb_type->joint_src.get(joint).jump;
                continue;
            }

            if (tile.cb.src_deadend.jump.testBit(src) &&
                !canFinishJump(tile, src, aim))
            {
                ++failure_cache_hits;
                src = -1;
                continue;
            }

            landings = {};
            ++edge_checks;
            if (diagnostic)
                diagnostic->edge(cursor, tile, src, use_joint ? joint : -1, -1,
                                 true);
        }

        fpga::TileJumpTarget target;
        if (!fpga::Device::current().nextJumpTarget(tile, src, landings,
                                                    target))
        {
            recordDeadend(tile, src, depth, cursor);
            src = -1;
            continue;
        }

        const bool marked = tile.cb.src_deadend.jump.testBit(src);
        if ((terminal_only && target.tile != &tile) ||
            (marked && (docking->sink && docking->terminal_tiles.empty()
                            ? !docking->canPlaceIn(*target.tile)
                            : !destinationTile(*target.tile, aim))))
            continue;
        if (target.tile->cb.dst.jump.testBit(target.dst_node))
        {
            blocked(*target.tile, "forward_target_lease",
                    {{fpga::CB_NODE_DST, target.dst_node}});
            docking->recordBlockedArrival(*target.tile, target.dst_node);
            continue;
        }

        if (type == fpga::CB_NODE_DST && target.tile == &tile &&
            target.dst_node == arrival)
            continue;
        const size_t path_begin = fragments.size();
        LeaseScope leases{tile, diagnostic, *this};
        leases.lease(tile.cb.src.jump, src);
        if (!root || type == fpga::CB_NODE_DST)
            leases.lease(node_mask(tile, type), arrival);
        if (use_joint)
            leases.lease(tile.cb.joint.jump, joint);
        if (use_local)
            leases.lease(tile.cb.local.local, local);
        if (use_local)
        {
            append_arc(fragments, tile.coord, type, arrival, tile.coord,
                       fpga::CB_NODE_LOCAL, local);
            append_arc(fragments, tile.coord, fpga::CB_NODE_LOCAL, local,
                       tile.coord, fpga::CB_NODE_SRC, src);
            append_arc(fragments, tile.coord, fpga::CB_NODE_SRC, src,
                       target.tile->coord, fpga::CB_NODE_DST, target.dst_node);
        }

        else if (type == fpga::CB_NODE_SRC)
            append_arc(fragments, tile.coord, type, arrival, target.tile->coord,
                       fpga::CB_NODE_DST, target.dst_node);
        else
        {
            fpga::Wire wire;
            wire.from = tile.coord;
            wire.to = target.tile->coord;
            wire.local = arrival;
            wire.jump = src;
            wire.dst = target.dst_node;
            wire.route_jump = target.jump_node;
            wire.joint = use_joint ? joint : -1;
            wire.pos = root ? (anchor.starts_at_dst ? ROUTE_POS_FORK : 0)
                            : ROUTE_POS_TRANSIT;
            wire.owns_dst = !(root && anchor.starts_at_dst);
            if (const auto *name = tile.cb_type->nodeName(type, arrival))
                wire.from_wire_name = *name;
            wire.src_wire_name =
                srcWireName(tile, type, arrival, src, wire.joint,
                            std::string(wire.from_wire_name));
            wire.dst_wire_name = target.dst_wire;
            fragments.push_back(wire);
        }

        if (diagnostic)
        {
            diagnostic->target(cursor, src, target);
            diagnostic->descend(cursor, queued);
            diagnostic->jump(aim, tile, *target.tile, cursor, src, 0, depth);
        }

        if (next(anchor, aim, queued, *target.tile, target.dst_node,
                 fpga::CB_NODE_DST, cursor, depth + 1,
                 target.tile == &tile ? same_tile_depth + 1 : 1,
                 terminal_only || marked))
            return true;
        fragments.resize(path_begin);
    }

    return false;
}

// Preserve only the bits and diagnostic Tile snapshots touched by one
// Docking-supplied internal path.
void TraceRouteForward::InternalLeaseScope::lease(
    fpga::Tile &tile, fpga::CBNodeNameType type, int node)
{
    if (trace.diagnostic)
        if (trace.diagnostic->remember(tile, trace))
            remembered.push_back(&tile);
    NodeMask &mask = trace.node_mask(tile, type);
    bits.push_back({&mask, node, mask.testBit(node)});
    mask.setBit(node);
}

TraceRouteForward::InternalLeaseScope::~InternalLeaseScope()
{
    for (auto bit = bits.rbegin(); bit != bits.rend(); ++bit)
    {
        if (bit->was_set)
            bit->mask->setBit(bit->node);
        else
            bit->mask->clearBit(bit->node);
    }
    if (trace.diagnostic)
        for (auto tile = remembered.rbegin(); tile != remembered.rend(); ++tile)
            trace.diagnostic->forget(**tile, true);
}

bool TraceRouteForward::InternalPathAttempt::operator()(
    const std::vector<DockingForwardNode> &path)
{
    return trace.tryInternalPath(*this, path);
}

// Hand long internal chains to Docking without adding a LOCAL/JOINT frame to
// the angle-guided recursion.
bool TraceRouteForward::tryFullInternal(
    const RoutingAnchor &anchor, fpga::Coord aim, size_t &queued,
    fpga::Tile &tile, fpga::CBNodeNameType type, int arrival, size_t cursor,
    size_t depth, size_t same_tile_depth, bool terminal_only)
{
    InternalPathAttempt attempt{*this, anchor, aim, queued, cursor, depth,
                                same_tile_depth, terminal_only};
    bool limited = false;
    const bool found = docking->searchForwardInternal(
        tile, type, arrival, std::ref(attempt), limited);
    if (limited && !found)
        cancelled = budget_exhausted = true;
    return found;
}

// Materialize one full-search path and recurse only after its physical jump.
bool TraceRouteForward::tryInternalPath(
    const InternalPathAttempt &attempt,
    const std::vector<DockingForwardNode> &path)
{
    if (path.size() < 2)
        return false;
    const DockingForwardNode &end = path.back();
    if (!end.tile ||
        (end.type != fpga::CB_NODE_LOCAL && end.type != fpga::CB_NODE_SRC))
        return false;

    const size_t path_begin = fragments.size();
    InternalLeaseScope leases{*this, path.size()};
    const size_t lease_count =
        end.type == fpga::CB_NODE_LOCAL ? path.size() - 1 : path.size();
    for (size_t index = 0; index < lease_count; ++index)
        leases.lease(*path[index].tile, path[index].type, path[index].node);
    for (size_t index = 1; index < path.size(); ++index)
        append_arc(fragments, path[index - 1].tile->coord,
                   path[index - 1].type, path[index - 1].node,
                   path[index].tile->coord, path[index].type,
                   path[index].node);

    if (end.type == fpga::CB_NODE_LOCAL)
    {
        if (diagnostic && !end.tile->cb.local.local.testBit(end.node))
            diagnostic->before_probe(fpga::CB_NODE_LOCAL, true);
        if (docking->appendTerminal(*end.tile, end.type, end.node, false,
                                    attempt.terminal_only, fragments))
        {
            success = true;
            if (diagnostic)
                diagnostic->grounded(attempt.cursor, true);
            return true;
        }
        fragments.resize(path_begin);
        return false;
    }

    if (end.tile->cb.src_deadend.jump.testBit(end.node) &&
        !canFinishJump(*end.tile, end.node, attempt.aim))
    {
        fragments.resize(path_begin);
        return false;
    }

    ++edge_checks;
    fpga::Device::JumpTargetCursor landings;
    fpga::TileJumpTarget target;
    while (fpga::Device::current().nextJumpTarget(
        *end.tile, end.node, landings, target))
    {
        if (!target.tile)
            continue;
        const bool marked = end.tile->cb.src_deadend.jump.testBit(end.node);
        if ((attempt.terminal_only && target.tile != end.tile) ||
            (marked && (docking->sink && docking->terminal_tiles.empty()
                            ? !docking->canPlaceIn(*target.tile)
                            : !destinationTile(*target.tile, attempt.aim))))
            continue;
        if (target.tile->cb.dst.jump.testBit(target.dst_node))
        {
            blocked(*target.tile, "forward_target_lease",
                    {{fpga::CB_NODE_DST, target.dst_node}});
            docking->recordBlockedArrival(*target.tile, target.dst_node);
            continue;
        }
        if (target.tile == path.front().tile &&
            path.front().type == fpga::CB_NODE_DST &&
            target.dst_node == path.front().node)
            continue;

        append_arc(fragments, end.tile->coord, fpga::CB_NODE_SRC,
                   end.node, target.tile->coord, fpga::CB_NODE_DST,
                   target.dst_node);
        if (diagnostic)
        {
            diagnostic->target(attempt.cursor, end.node, target);
            diagnostic->descend(attempt.cursor, attempt.queued);
            diagnostic->jump(attempt.aim, *end.tile, *target.tile,
                             attempt.cursor, end.node, 0, attempt.depth);
        }
        if (next(attempt.anchor, attempt.aim, attempt.queued, *target.tile,
                 target.dst_node, fpga::CB_NODE_DST, attempt.cursor,
                 attempt.depth + 1,
                 target.tile == path.front().tile
                     ? attempt.same_tile_depth + 1
                     : 1,
                 attempt.terminal_only || marked))
            return true;
        fragments.resize(path_begin + path.size() - 1);
    }

    recordDeadend(*end.tile, end.node, attempt.depth, attempt.cursor);
    fragments.resize(path_begin);
    return false;
}

// Try this one LOCAL as an endpoint, leaving combinatorial completion to
// Docking.
bool TraceRouteForward::tryLocalTerminal(fpga::Tile &tile,
                                         fpga::CBNodeNameType arrival_type,
                                         int arrival, int local,
                                         bool terminal_only, size_t cursor)
{
    if (!docking->hasLocalTerminal())
        return false;
    const size_t path_begin = fragments.size();
    LeaseScope leases{tile, diagnostic, *this};
    leases.lease(node_mask(tile, arrival_type), arrival);
    append_arc(fragments, tile.coord, arrival_type, arrival, tile.coord,
               fpga::CB_NODE_LOCAL, local);
    if (diagnostic && !tile.cb.local.local.testBit(local))
        diagnostic->before_probe(fpga::CB_NODE_LOCAL, true);
    if (docking->appendTerminal(tile, fpga::CB_NODE_LOCAL, local, false,
                                terminal_only, fragments))
    {
        success = true;
        if (diagnostic)
            diagnostic->grounded(cursor, true);
        return true;
    }

    fragments.resize(path_begin);
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
    for (int src; (src = type.fastSearchNextHop(angle, cursor, allowed)) >= 0;)
    {
        if (src >= CB_MAX_NODES || type.dst_by_src.get(src).empty())
            unresolved.push_back(static_cast<uint16_t>(src));
        else
            active.push_back(static_cast<uint16_t>(src));
    }

    ForwardRouteDiagnostic::audit_candidates(type, from_type, from_node, angle,
                                             allowed, active, unresolved);
}
