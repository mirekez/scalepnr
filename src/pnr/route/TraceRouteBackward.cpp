#include "TraceRouteBackward.h"
#include "Device.h"
#include "Docking.h"
#include "InputRepairBudget.h"
#include "MovingSourceReachability.h"
#include "RouteDiagnostics.h"
#include "RoutePreemption.h"
#include "RouteSearch.h"
#include "RouteTaskTiming.h"
#include "TraceRouteCommon.h"

#include <algorithm>
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

// Identify persistent reverse marks in focused routing failure reproductions.
void logBackwardDeadend(const fpga::Tile &tile, int dst)
{
    if (std::getenv("SCALEPNR_DEBUG_BACKWARD_DEADEND"))
        std::fprintf(stderr, "BACKWARD_DEADEND tile=(%d,%d) dst=%d\n",
                     tile.coord.x, tile.coord.y, dst);
}

} // namespace

// Reverse jumps may terminate in the source Tile without another DST.
bool TraceRouteBackward::isDeadend(fpga::Tile &tile, int dst,
                                   fpga::Coord destination,
                                   BackwardResolveIndex &index)
{
    if (dst < 0 || !tile.cb.dst_deadend.jump.testBit(dst))
        return false;
    Docking::prepareBackwardTrace(index);
    auto incoming =
        Docking::incomingCursor(index, {tile.coord.x, tile.coord.y, dst});
    const auto *resource =
        fpga::Device::current().getTile(destination.x, destination.y);
    while (const auto *source = Docking::nextIncoming(incoming, {0, 0}))
        if (source->tile->coord == destination ||
            (resource && resource->cb_coord == source->tile->coord))
            return false;
    return true;
}

// Propagate only persistent reverse DST marks; occupancy may later change.
bool TraceRouteBackward::markDeadend(fpga::Tile &tile, int dst,
                                     BackwardResolveIndex &index)
{
    if (dst < 0 || tile.cb.dst_deadend.jump.testBit(dst))
        return false;
    Docking::prepareBackwardTrace(index);
    auto incoming =
        Docking::incomingCursor(index, {tile.coord.x, tile.coord.y, dst});
    bool reached = false;
    while (const auto *source = Docking::nextIncoming(incoming, {0, 0}))
    {
        reached = true;
        source->tile->cb_type->ensureDerivedMasks();
        const NodeMask eligible = source->tile->cb_type->valid_dst_nodes |
                                  source->tile->incoming_dst_nodes;
        // Reverse occupancy can also unwind or change after preemption.
        if ((eligible & ~source->tile->cb.dst_deadend.jump) != NodeMask{})
            return false;
    }

    if (!reached)
        return false;
    tile.cb.dst_deadend.jump.setBit(dst);
    logBackwardDeadend(tile, dst);
    return true;
}

void TraceRouteBackward::debug_trace(std::ostream *out, fpga::Tile &tile,
                                     int dst, fpga::Coord source_hint)
{
    if (out)
        *out << "BACKWARD_TRACE tile=(" << tile.coord.x << ',' << tile.coord.y
             << ") dst=" << dst << " aim=(" << source_hint.x << ','
             << source_hint.y << ")\n";
}

void TraceRouteBackward::debug_probe(std::ostream *out, fpga::Tile &tile,
                                     int src, bool accepted)
{
    if (out)
        *out << "BACKWARD_TRACE_PROBE tile=(" << tile.coord.x << ','
             << tile.coord.y << ") src=" << src << " accepted=" << accepted
             << '\n';
}

// Preserve the deepest failed route suffix without affecting search.
void TraceRouteBackward::retain_failure(
    BackwardTakeoffRoute &result, fpga::Tile &tile, int dst,
    const std::vector<fpga::Wire> &reverse_edges,
    const TerminalIngress &ingress)
{
    if (reverse_edges.size() + ingress.suffix.size() >=
        result.diagnostic_fragments.size())
    {
        writeSuffix(result.diagnostic_fragments, reverse_edges, ingress);
        result.failure_tile = &tile;
        result.failure_dst = dst;
    }

}

// Recurse through one reverse physical hop; local bridge checks are mask pops.
bool TraceRouteBackward::next(fpga::Tile &arrival_tile, int arrival_dst,
                              fpga::Coord source_hint,
                              BackwardResolveIndex &index,
                              std::ostream *decisions,
                              BackwardTakeoffRoute &result, bool &stopped,
                              const TerminalIngress &ingress)
{
    if (docking->driverStopped() || remaining_expansions == 0)
    {
        result.expansion_limit_reached = remaining_expansions == 0;
        stopped = true;
        retain_failure(result, arrival_tile, arrival_dst, reverse_edges,
                       ingress);
        return true;
    }

    --remaining_expansions;
    ++result.expanded;
    debug_trace(decisions, arrival_tile, arrival_dst, source_hint);
    auto incoming = Docking::incomingCursor(
        index, {arrival_tile.coord.x, arrival_tile.coord.y, arrival_dst});
    const BackwardResolveSource *source = nullptr;
    const BackwardCrossbarMasks *masks = nullptr;
    NodeMask destinations, blocked_destinations, locals, joints;
    Docking::DstCursor exits;
    unsigned bridge_phase = 0;
    int local = -1, joint = -1;

    // Each iteration consumes one physical source, DST, LOCAL or JOINT bit.
    // Only the physical jump below invokes this function recursively.
    while (!stopped)
    {
        if (!source)
        {
            source = Docking::nextIncoming(incoming,
                                           source_hint - arrival_tile.coord);
            if (!source)
            {
                if (markDeadend(arrival_tile, arrival_dst, index))
                    ++result.failed_states;
                retain_failure(result, arrival_tile, arrival_dst, reverse_edges,
                               ingress);
                return false;
            }

            ++result.incoming_edges;
            auto &tile = *source->tile;
            const bool placement_allowed = docking->canPlaceIn(tile);
            if (tile.cb.src.jump.testBit(source->src))
            {
                if (placement_allowed)
                    docking->recordBackwardSourceBlock(result, tile,
                                                       source->src);
                source = nullptr;
                continue;
            }

            const bool marked =
                arrival_tile.cb.dst_deadend.jump.testBit(arrival_dst);
            if (marked && !placement_allowed)
            {
                ++result.failure_cache_hits;
                source = nullptr;
                continue;
            }

            BackwardTakeoffChoice takeoff;
            bool accepted = false;
            if (placement_allowed)
            {
                ++result.probe_calls;
                accepted = docking->tryDriver(tile, source->src, takeoff);
                debug_probe(decisions, tile, source->src, accepted);
            }

            if (accepted && takeoff.local >= 0 &&
                canUsePhysicalJump(tile.cb, takeoff.local, source->src,
                                   takeoff.joint, takeoff.joint2, true))
            {
                writeSuffix(result.fragments, reverse_edges, ingress);
                result.fragments.insert(
                    result.fragments.begin(),
                    EdgeBuilder{arrival_tile, arrival_dst, *source}(
                        takeoff.local, takeoff.joint, takeoff.joint2, true));
                markGraphArrivals(result.fragments);
                result.source_tile = &tile;
                result.source_src = source->src;
                result.takeoff = takeoff;
                result.success = true;
                return true;
            }

            if (marked)
            {
                source = nullptr;
                continue;
            }

            masks = Docking::reverseMasks(index, *tile.cb_type);
            if (!masks)
            {
                source = nullptr;
                continue;
            }

            destinations = masks->source_dsts.get(source->src).jump;
            blocked_destinations = destinations & tile.cb.dst.jump;
            locals = masks->source_locals.get(source->src).local &
                     ~(tile.cb.local.local | tile.pin_state.leased_nodes);
            joints = masks->source_joints.get(source->src).joint &
                     ~tile.cb.joint.jump;
            exits = {};
            bridge_phase = 0;
            local = joint = -1;
        }

        auto &tile = *source->tile;
        const int dst =
            Docking::nextBackwardDst(index, tile, source_hint - tile.coord,
                                     exits, destinations & ~tile.cb.dst.jump);
        if (dst < 0)
        {
            const int blocked = blocked_destinations.firstSetBit();
            if (blocked >= 0)
            {
                blocked_destinations.clearBit(blocked);
                docking->recordBackwardDstBlock(result, tile, blocked, index);
                continue;
            }

            exits = {};
            if (bridge_phase == 0)
                bridge_phase = 1;
            if (bridge_phase == 1)
            {
                local = locals.firstSetBit();
                if (local >= 0)
                {
                    locals.clearBit(local);
                    joint = -1;
                    destinations = inputLocalReserved(tile, local)
                                       ? NodeMask{}
                                       : masks->local_dsts.get(local).jump;
                    blocked_destinations = destinations & tile.cb.dst.jump;
                    continue;
                }

                bridge_phase = 2;
            }

            joint = joints.firstSetBit();
            if (joint < 0)
            {
                source = nullptr;
                continue;
            }

            joints.clearBit(joint);
            local = -1;
            destinations = masks->joint_dsts.get(joint).jump;
            blocked_destinations = destinations & tile.cb.dst.jump;
            continue;
        }

        if (tile.cb.dst_deadend.jump.testBit(dst) &&
            !docking->canPlaceAtReverseLanding(tile, dst, index))
        {
            ++result.failure_cache_hits;
            continue;
        }

        const size_t path_size = reverse_edges.size();
        HopLeases leases;
        leases.lease(tile.cb.src.jump, source->src);
        leases.lease(tile.cb.dst.jump, dst);
        if (local >= 0)
        {
            leases.lease(tile.cb.local.local, local);
            arc(reverse_edges, tile.coord, fpga::CB_NODE_SRC, source->src,
                arrival_tile.coord, fpga::CB_NODE_DST, arrival_dst);
            arc(reverse_edges, tile.coord, fpga::CB_NODE_LOCAL, local,
                tile.coord, fpga::CB_NODE_SRC, source->src);
            arc(reverse_edges, tile.coord, fpga::CB_NODE_DST, dst, tile.coord,
                fpga::CB_NODE_LOCAL, local);
        }

        else
        {
            leases.lease(tile.cb.joint.jump, joint);
            reverse_edges.push_back(EdgeBuilder{arrival_tile, arrival_dst,
                                                *source}(dst, joint, -1));
        }

        if (next(tile, dst, source_hint, index, decisions, result, stopped,
                 ingress))
            return true;
        reverse_edges.resize(path_size);
    }

    return false;
}

// Docking leases terminal ingress; each visitor starts one Tile-hop trace.
bool TraceRouteBackward::IngressVisitor::operator()(
    const BackwardTerminalIngress &ingress) const
{
    trace.reverse_edges.clear();
    return trace.next(*ingress.tile, ingress.dst, source_hint, index, decisions,
                      result, stopped, ingress);
}

// Establish one attempt context; all search cursors live in recursive calls.
BackwardTakeoffRoute TraceRouteBackward::run(fpga::Tile &target, NodeMask pins,
                                             fpga::Coord source_hint,
                                             BackwardResolveIndex &index,
                                             std::ostream *decisions)
{
    BackwardTakeoffRoute result;
    bool stopped = false;
    result.failure_tile = &target;
    if (!target.cb_type || !docking || !docking->hasDriver())
        return result;
    Docking::prepareBackwardTrace(index);
    remaining_expansions = max_expansions;
    docking->visitBackwardTerminals(
        target, pins, source_hint, index, decisions, result, stopped,
        remaining_expansions,
        IngressVisitor{*this, source_hint, index, decisions, result, stopped});
    return result;
}

// Retain one angle-led suffix; placement probes run only along that trace.
// Speculative leases are restored before the caller commits or moves a cell.
BackwardTakeoffRoute traceBackwardToTakeoff(
    fpga::Tile &target, NodeMask pins, fpga::Coord source_hint,
    BackwardResolveIndex &index, const BackwardTakeoffProbe &probe,
    const BackwardTakeoffCancel &cancel,
    const BackwardTakeoffStateView *state_view, std::ostream *decisions,
    size_t max_expansions, const Docking::PlacementAllowed &placement_allowed)
{
    Docking docking;
    docking.placement_allowed = placement_allowed;
    docking.driver_probe = &probe;
    docking.driver_cancel = &cancel;
    docking.driver_state = state_view;
    TraceRouteBackward trace;
    trace.docking = &docking;
    trace.max_expansions = max_expansions;
    return trace.run(target, pins, source_hint, index, decisions);
}

} // namespace pnr
