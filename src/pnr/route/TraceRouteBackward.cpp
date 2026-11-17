#include "TraceRouteBackward.h"
#include "Device.h"
#include "InputRepairBudget.h"
#include "MovingSourceReachability.h"
#include "RouteDiagnostics.h"
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

void TraceRouteBackward::debug_ingress(std::ostream *out, fpga::Tile &tile,
                                       fpga::CBNodeNameType type, int node)
{
    if (out)
        *out << "BACKWARD_INGRESS tile=(" << tile.coord.x << ',' << tile.coord.y
             << ") type=" << type << " node=" << node << '\n';
}

// Preserve the deepest failed route suffix without affecting search.
void TraceRouteBackward::IngressTrace::retain_failure(
    BackwardTakeoffRoute &result, fpga::Tile &tile, int dst,
    const std::vector<fpga::Wire> &reverse_edges,
    const TerminalIngress &ingress)
{
    auto diagnostic = suffix(reverse_edges, ingress);
    if (diagnostic.size() >= result.diagnostic_fragments.size())
    {
        result.diagnostic_fragments = std::move(diagnostic);
        result.failure_tile = &tile;
        result.failure_dst = dst;
    }

}

// Lease one terminal suffix and walk backward toward a movable source.
bool TraceRouteBackward::IngressTrace::run(
    const TerminalIngress &ingress, fpga::Tile &target, fpga::Coord source_hint,
    BackwardResolveIndex &index, std::ostream *decisions,
    BackwardTakeoffRoute &result, bool &stopped)
{
    Trial trial;
    std::vector<fpga::Wire> reverse_edges;
    // Present caller-supplied state while retaining a rollback snapshot per
    // tile.
    if (docking->driver_state)
        for (const auto &[tile, state] : *docking->driver_state)
        {
            if (!tile)
                continue;
            trial.remember(*tile);
            tile->cb.restoreLeases(state);
        }

    if (ingress.tile->incoming_dst_nodes != NodeMask{} &&
        !ingress.tile->incoming_dst_nodes.testBit(ingress.dst))
        return false;
    // Reject occupied terminal nodes before leasing the entire ingress chain.
    for (const auto &node : ingress.nodes)
    {
        trial.remember(*node.tile);
        if (node.type == fpga::CB_NODE_LOCAL)
        {
            if (node.tile->cb.local.local.testBit(node.node) ||
                node.tile->isPinNodeLeased(node.node) ||
                (!(node.tile == &target && node.node == ingress.pin) &&
                 inputLocalReserved(*node.tile, node.node)))
                return false;
        }

        else if (node.type == fpga::CB_NODE_JOINT)
        {
            if (node.tile->cb.joint.jump.testBit(node.node))
                return false;
        }

        else if (node.type == fpga::CB_NODE_DST)
        {
            if (node.tile->cb.dst.jump.testBit(node.node))
                return false;
        }

    }

    for (const auto &node : ingress.nodes)
    {
        if (node.type == fpga::CB_NODE_LOCAL)
            node.tile->cb.local.local.setBit(node.node);
        else if (node.type == fpga::CB_NODE_JOINT)
            node.tile->cb.joint.jump.setBit(node.node);
        else if (node.type == fpga::CB_NODE_DST)
            node.tile->cb.dst.jump.setBit(node.node);
    }

    const auto *arrival = resolveBackwardSources(
        index, {ingress.tile->coord.x, ingress.tile->coord.y, ingress.dst},
        true);
    // The caller may distinguish a free terminal from a failure farther
    // upstream, including for an ordinary DST->LOCAL terminal.
    if ((arrival && !arrival->empty()) ||
        ingress.tile->incoming_dst_nodes.testBit(ingress.dst))
        result.free_terminal_ingress = true;
    // Each recursive call resumes its local choices after a child fails.
    return next(*ingress.tile, ingress.dst, source_hint, index, decisions,
                result, stopped, ingress, trial, reverse_edges);
}

// Apply the forward hop-shape order to physical arrivals in reverse direction.
void TraceRouteBackward::IngressTrace::order_incoming(
    fpga::Tile &arrival_tile, fpga::Coord source_hint,
    const std::vector<BackwardResolveSource> *incoming,
    NodeMask &remaining_source_ranks, std::array<uint32_t, 225> &first_by_rank,
    std::vector<uint32_t> &next_source)
{
    constexpr uint32_t empty = std::numeric_limits<uint32_t>::max();
    first_by_rank.fill(empty);
    if (!incoming)
        return;
    next_source.assign(incoming->size(), empty);
    const fpga::Coord aim = source_hint - arrival_tile.coord;
    // The index is already short-first. Prepending in reverse index order
    // preserves that order for sources with the same angle rank.
    for (size_t i = incoming->size(); i > 0; --i)
    {
        const auto &source = (*incoming)[i - 1];
        if (!source.tile || !source.tile->cb_type)
            continue;
        const auto rank = fpga::CBType::anglePriorityRank(
            aim, source.tile->coord - arrival_tile.coord);
        next_source[i - 1] = first_by_rank[rank];
        first_by_rank[rank] = static_cast<uint32_t>(i - 1);
        remaining_source_ranks.setBit(rank);
    }

}

// Pop the lowest remaining angle rank without rescanning other arrivals.
const BackwardResolveSource *TraceRouteBackward::IngressTrace::next_incoming(
    const std::vector<BackwardResolveSource> *incoming,
    NodeMask &remaining_source_ranks, std::array<uint32_t, 225> &first_by_rank,
    const std::vector<uint32_t> &next_source)
{
    constexpr uint32_t empty = std::numeric_limits<uint32_t>::max();
    const int rank = remaining_source_ranks.firstSetBit();
    if (rank < 0)
        return nullptr;
    const uint32_t index = first_by_rank[rank];
    first_by_rank[rank] = next_source[index];
    if (first_by_rank[rank] == empty)
        remaining_source_ranks.clearBit(rank);
    return &(*incoming)[index];
}

// Recurse through one arrival while retaining its local reverse cursors.
bool TraceRouteBackward::IngressTrace::next(
    fpga::Tile &arrival_tile, int arrival_dst, fpga::Coord source_hint,
    BackwardResolveIndex &index, std::ostream *decisions,
    BackwardTakeoffRoute &result, bool &stopped, const TerminalIngress &ingress,
    Trial &trial, std::vector<fpga::Wire> &reverse_edges)
{
    const auto *incoming = resolveBackwardSources(
        index, {arrival_tile.coord.x, arrival_tile.coord.y, arrival_dst}, true);
    NodeMask remaining_source_ranks;
    std::array<uint32_t, 225> first_by_rank{};
    std::vector<uint32_t> next_source;
    order_incoming(arrival_tile, source_hint, incoming, remaining_source_ranks,
                   first_by_rank, next_source);
    Phase phase = Phase::source;
    BackwardResolveSource selected;
    NodeMask reachable, previous, locals, active_locals;
    std::vector<LocalStep> local_walk;
    Row row{};
    bool row_ready = false;
    if (docking->driverStopped())
    {
        stopped = true;
        retain_failure(result, arrival_tile, arrival_dst, reverse_edges,
                       ingress);
        return true;
    }

    ++result.expanded;
    debug_trace(decisions, arrival_tile, arrival_dst, source_hint);
    for (;;)
    {
        bool advanced = false;
        fpga::Tile *child_tile = nullptr;
        int child_dst = -1;
        fpga::Tile *changed_tile = nullptr;
        std::unique_ptr<fpga::CBState> before;
        size_t path_size = reverse_edges.size();
        Descent descend{advanced, child_tile, child_dst,    changed_tile,
                        before,   path_size,  reverse_edges};
        while (!advanced)
        {
            // Visit incoming physical sources in angle and length order and
            // probe a takeoff.
            if (phase == Phase::source)
            {
                bool found = false;
                while (const auto *candidate =
                           next_incoming(incoming, remaining_source_ranks,
                                         first_by_rank, next_source))
                {
                    const auto source = *candidate;
                    ++result.incoming_edges;
                    auto &tile = *source.tile;
                    trial.remember(tile);
                    if (tile.cb.src.jump.testBit(source.src))
                    {
                        if (visible_state(tile, trial)
                                .src.jump.testBit(source.src))
                            retain_blocked(
                                result,
                                {{tile.coord, fpga::CB_NODE_SRC, source.src}});
                        continue;
                    }

                    if (tile.cb.src_deadend.jump.testBit(source.src))
                    {
                        ++result.failure_cache_hits;
                        continue;
                    }

                    BackwardTakeoffChoice takeoff;
                    ++result.probe_calls;
                    const bool accepted =
                        docking->tryDriver(tile, source.src, takeoff);
                    debug_probe(decisions, tile, source.src, accepted);
                    EdgeBuilder edge{arrival_tile, arrival_dst, source};
                    if (accepted && takeoff.local >= 0 &&
                        canLeaseJump(tile.cb, takeoff.local, source.src,
                                     takeoff.joint, takeoff.joint2, true))
                    {
                        result.fragments = suffix(reverse_edges, ingress);
                        result.fragments.insert(result.fragments.begin(),
                                                edge(takeoff.local,
                                                     takeoff.joint,
                                                     takeoff.joint2, true));
                        markGraphArrivals(result.fragments);
                        result.source_tile = &tile;
                        result.source_src = source.src;
                        result.takeoff = takeoff;
                        result.success = true;
                        return true;
                    }

                    tile.cb_type->ensureDerivedMasks();
                    selected = source;
                    reachable =
                        tile.cb_type->dsts_reaching_src.get(source.src).jump;
                    previous = reachable & ~tile.cb.dst.jump;
                    phase = Phase::ordinary;
                    found = true;
                    break;
                }

                if (!found)
                    break;
            }

            const auto source = selected;
            auto &tile = *source.tile;
            EdgeBuilder edge{arrival_tile, arrival_dst, source};
            // Try a free DST-to-SRC transit before involving LOCAL nodes.
            if (phase == Phase::ordinary)
            {
                for (int node = previous.firstSetBit(); node >= 0;
                     node = previous.firstSetBit())
                {
                    previous.clearBit(node);
                    int joint2 = -1;
                    const int joint =
                        selectJointToSrc(tile, fpga::CB_NODE_DST, node,
                                         source.src, &joint2, true);
                    if (joint == -2 ||
                        !canLeaseJump(tile.cb, node, source.src, joint, joint2))
                        continue;
                    descend(tile, node);
                    reverse_edges.push_back(edge(node, joint, joint2));
                    tile.cb.src.jump.setBit(source.src);
                    tile.cb.dst.jump.setBit(node);
                    for (int used : {joint, joint2})
                        if (used >= 0)
                            tile.cb.joint.jump.setBit(used);
                    break;
                }

                if (advanced)
                    break;
                phase = Phase::direct_local;
                row = tile.cb_type->dst_local.values.begin();
                row_ready = false;
            }

            // Try a single LOCAL bridge from an upstream DST to the chosen SRC.
            if (phase == Phase::direct_local)
            {
                const auto end = tile.cb_type->dst_local.values.end();
                while (row != end)
                {
                    if (!row_ready)
                    {
                        locals = tile.cb.dst.jump.testBit(row->first)
                                     ? NodeMask{}
                                     : row->second.local & ~tile.cb.local.local;
                        row_ready = true;
                    }

                    for (int local = locals.firstSetBit(); local >= 0;
                         local = locals.firstSetBit())
                    {
                        locals.clearBit(local);
                        if (tile.isPinNodeLeased(local) ||
                            inputLocalReserved(tile, local))
                            continue;
                        int joint2 = -1;
                        const int joint =
                            selectJointToSrc(tile, fpga::CB_NODE_LOCAL, local,
                                             source.src, &joint2, true);
                        if (joint == -2 ||
                            !canLeaseJump(tile.cb, local, source.src, joint,
                                          joint2, true))
                            continue;
                        const int node = row->first;
                        descend(tile, node);
                        arc(reverse_edges, tile.coord, fpga::CB_NODE_SRC,
                            source.src, arrival_tile.coord, fpga::CB_NODE_DST,
                            arrival_dst);
                        auto kind = fpga::CB_NODE_SRC;
                        int next = source.src;
                        for (int used : {joint, joint2})
                            if (used >= 0)
                            {
                                arc(reverse_edges, tile.coord,
                                    fpga::CB_NODE_JOINT, used, tile.coord, kind,
                                    next);
                                kind = fpga::CB_NODE_JOINT;
                                next = used;
                                tile.cb.joint.jump.setBit(used);
                            }

                        arc(reverse_edges, tile.coord, fpga::CB_NODE_LOCAL,
                            local, tile.coord, kind, next);
                        arc(reverse_edges, tile.coord, fpga::CB_NODE_DST, node,
                            tile.coord, fpga::CB_NODE_LOCAL, local);
                        tile.cb.src.jump.setBit(source.src);
                        tile.cb.local.local.setBit(local);
                        tile.cb.dst.jump.setBit(node);
                        break;
                    }

                    if (advanced)
                        break;
                    ++row;
                    row_ready = false;
                }

                if (advanced)
                    break;
                phase = Phase::local_chain;
                row = tile.cb_type->dst_local.values.begin();
                row_ready = false;
            }

            // Follow longer LOCAL relay chains with a path-local visited mask.
            if (phase == Phase::local_chain)
            {
                const auto end = tile.cb_type->dst_local.values.end();
                while (row != end)
                {
                    if (!row_ready)
                    {
                        locals = tile.cb.dst.jump.testBit(row->first)
                                     ? NodeMask{}
                                     : row->second.local;
                        row_ready = true;
                    }

                    for (;;)
                    {
                        if (local_walk.empty())
                        {
                            const int first = locals.firstSetBit();
                            if (first < 0)
                                break;
                            locals.clearBit(first);
                            if (tile.cb.local.local.testBit(first) ||
                                tile.isPinNodeLeased(first) ||
                                inputLocalReserved(tile, first))
                                continue;
                            local_walk.push_back(
                                {first,
                                 tile.cb_type->local_local.get(first).local});
                            active_locals.setBit(first);
                        }

                        auto &step = local_walk.back();
                        const int last = step.node;
                        if (!step.direct_checked)
                        {
                            step.direct_checked = true;
                            int joint2 = -1;
                            const int joint = selectJointToSrc(
                                tile, fpga::CB_NODE_LOCAL, last, source.src,
                                &joint2, true);
                            if (local_walk.size() > 1 && joint != -2 &&
                                canLeaseJump(tile.cb, last, source.src, joint,
                                             joint2, true))
                            {
                                const int node = row->first;
                                const int first = local_walk.front().node;
                                descend(tile, node);
                                arc(reverse_edges, tile.coord,
                                    fpga::CB_NODE_SRC, source.src,
                                    arrival_tile.coord, fpga::CB_NODE_DST,
                                    arrival_dst);
                                auto kind = fpga::CB_NODE_SRC;
                                int next = source.src;
                                for (int used : {joint, joint2})
                                    if (used >= 0)
                                    {
                                        arc(reverse_edges, tile.coord,
                                            fpga::CB_NODE_JOINT, used,
                                            tile.coord, kind, next);
                                        kind = fpga::CB_NODE_JOINT;
                                        next = used;
                                        tile.cb.joint.jump.setBit(used);
                                    }

                                for (auto it = local_walk.rbegin();
                                     it != local_walk.rend(); ++it)
                                {
                                    arc(reverse_edges, tile.coord,
                                        fpga::CB_NODE_LOCAL, it->node,
                                        tile.coord, kind, next);
                                    kind = fpga::CB_NODE_LOCAL;
                                    next = it->node;
                                    tile.cb.local.local.setBit(it->node);
                                }

                                arc(reverse_edges, tile.coord,
                                    fpga::CB_NODE_DST, node, tile.coord,
                                    fpga::CB_NODE_LOCAL, first);
                                tile.cb.src.jump.setBit(source.src);
                                tile.cb.dst.jump.setBit(node);
                                break;
                            }

                        }

                        int next = -1;
                        for (int candidate = step.remaining.firstSetBit();
                             candidate >= 0;
                             candidate = step.remaining.firstSetBit())
                        {
                            step.remaining.clearBit(candidate);
                            if (active_locals.testBit(candidate) ||
                                tile.cb.local.local.testBit(candidate) ||
                                tile.isPinNodeLeased(candidate) ||
                                inputLocalReserved(tile, candidate))
                                continue;
                            next = candidate;
                            break;
                        }

                        if (next >= 0)
                        {
                            local_walk.push_back(
                                {next,
                                 tile.cb_type->local_local.get(next).local});
                            active_locals.setBit(next);
                            continue;
                        }

                        active_locals.clearBit(last);
                        local_walk.pop_back();
                    }

                    if (advanced)
                        break;
                    ++row;
                    row_ready = false;
                }

                if (advanced)
                    break;
                phase = Phase::blocked;
            }

            // Preserve committed blockers and mark only the exhausted source
            // exit.
            if (phase == Phase::blocked)
            {
                NodeMask blocked_reachable = reachable & tile.cb.dst.jump;
                for (int node = blocked_reachable.firstSetBit(); node >= 0;
                     node = blocked_reachable.firstSetBit())
                {
                    blocked_reachable.clearBit(node);
                    if (!visible_state(tile, trial).dst.jump.testBit(node))
                        continue;
                    std::vector<fpga::RouteCutNode> cuts{
                        {tile.coord, fpga::CB_NODE_DST, node}};
                    const auto *upstream = resolveBackwardSources(
                        index, {tile.coord.x, tile.coord.y, node}, true);
                    if (upstream)
                        for (const auto &incoming : *upstream)
                        {
                            if (!incoming.tile ||
                                !visible_state(*incoming.tile, trial)
                                     .src.jump.testBit(incoming.src))
                                continue;
                            cuts.insert(cuts.begin(),
                                        fpga::RouteCutNode{incoming.tile->coord,
                                                           fpga::CB_NODE_SRC,
                                                           incoming.src});
                            break;
                        }

                    retain_blocked(result, cuts);
                }

                if (!tile.cb.src_deadend.jump.testBit(source.src))
                {
                    tile.cb.src_deadend.jump.setBit(source.src);
                    ++result.failed_states;
                }

                phase = Phase::source;
            }

        }

        if (advanced)
        {
            if (next(*child_tile, child_dst, source_hint, index, decisions,
                     result, stopped, ingress, trial, reverse_edges))
                return true;
            changed_tile->cb.restoreLeases(*before);
            reverse_edges.resize(path_size);
            continue;
        }

        retain_failure(result, arrival_tile, arrival_dst, reverse_edges,
                       ingress);
        return false;
    }

}

// Extend one pin through typed predecessors with a local recursive cursor.
bool TraceRouteBackward::TypedIngressTrace::run(fpga::Tile &target,
                                                fpga::Coord source_hint,
                                                BackwardResolveIndex &index,
                                                std::ostream *decisions,
                                                BackwardTakeoffRoute &result,
                                                bool &stopped, int pin)
{
    TerminalIngress ingress{&target, -1, pin};
    ingress.nodes.push_back({&target, fpga::CB_NODE_LOCAL, pin});
    std::vector<fpga::Wire> reversed;
    std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> seen;
    seen.insert({target.coord.x, target.coord.y, fpga::CB_NODE_LOCAL, pin});
    return trace(target, source_hint, index, decisions, result, stopped, pin,
                 ingress, reversed, seen);
}

// Try each free typed predecessor in source-angle order, resuming after
// failure.
bool TraceRouteBackward::TypedIngressTrace::trace(
    fpga::Tile &target, fpga::Coord source_hint, BackwardResolveIndex &index,
    std::ostream *decisions, BackwardTakeoffRoute &result, bool &stopped,
    int pin, TerminalIngress &ingress, std::vector<fpga::Wire> &reversed,
    std::unordered_set<BackwardLocalKey, BackwardLocalKeyHash> &seen)
{
    if (docking->driverStopped())
    {
        stopped = true;
        return true;
    }

    const TerminalNode current = ingress.nodes.back();
    if (current.type == fpga::CB_NODE_DST)
    {
        const auto *incoming = resolveBackwardSources(
            index, {current.tile->coord.x, current.tile->coord.y, current.node},
            true);
        if ((incoming && !incoming->empty()) ||
            current.tile->incoming_dst_nodes.testBit(current.node))
        {
            ingress.tile = current.tile;
            ingress.dst = current.node;
            ingress.suffix.assign(reversed.rbegin(), reversed.rend());
            fpga::Wire endpoint;
            endpoint.type = fpga::Wire::WIRE_TILE_PIN;
            endpoint.from = endpoint.to = target.coord;
            endpoint.local = pin;
            endpoint.pin_dir = fpga::TILE_PIN_INPUT;
            ingress.suffix.push_back(endpoint);
            if (ingress_trace->run(ingress, target, source_hint, index,
                                   decisions, result, stopped))
                return true;
        }

    }

    // This call keeps its own ordered alternatives while a child recurses.
    std::vector<Candidate> choices;
    TerminalNode blocked{nullptr, fpga::CB_NODE_LOCAL, -1};
    int blocked_angle = 225;
    int blocked_distance = 1 << 30;
    const auto &cb = *current.tile->cb_type;
    if (current.type == fpga::CB_NODE_LOCAL)
    {
        for (const auto &[dst, row] : cb.dst_local.values)
            if (row.local.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_DST, dst, choices,
                         blocked, blocked_angle, blocked_distance, target,
                         source_hint, pin, ingress, seen);
        for (const auto &[joint, row] : cb.joint_local.values)
            if (row.local.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_JOINT, joint,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
        for (const auto &[local, row] : cb.local_local.values)
            if (row.local.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_LOCAL, local,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
    }

    else if (current.type == fpga::CB_NODE_JOINT)
    {
        for (const auto &[dst, row] : cb.dst_joint.values)
            if (row.joint.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_DST, dst, choices,
                         blocked, blocked_angle, blocked_distance, target,
                         source_hint, pin, ingress, seen);
        for (const auto &[joint, row] : cb.joint_joint.values)
            if (row.joint.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_JOINT, joint,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
        for (const auto &[local, row] : cb.local_joint.values)
            if (row.joint.testBit(current.node))
                consider(current, current.tile, fpga::CB_NODE_LOCAL, local,
                         choices, blocked, blocked_angle, blocked_distance,
                         target, source_hint, pin, ingress, seen);
    }

    visitBackwardPhysicalSources(
        index, current.tile->coord, current.type, current.node,
        PhysicalSourceVisitor{*this, current, choices, blocked, blocked_angle,
                              blocked_distance, target, source_hint, pin,
                              ingress, seen});
    std::stable_sort(choices.begin(), choices.end());
    choices.erase(std::unique(choices.begin(), choices.end()), choices.end());
    for (const auto &choice : choices)
    {
        const auto &node = choice.node;
        const BackwardLocalKey key{node.tile->coord.x, node.tile->coord.y,
                                   node.type, node.node};
        if (!seen.insert(key).second)
            continue;
        debug_ingress(decisions, *node.tile, node.type, node.node);
        reversed.push_back(typed_edge(node, current));
        ingress.nodes.push_back(node);
        if (trace(target, source_hint, index, decisions, result, stopped, pin,
                  ingress, reversed, seen))
            return true;
        ingress.nodes.pop_back();
        reversed.pop_back();
        seen.erase(key);
    }

    retain_typed_diagnostic(current, blocked.tile ? &blocked : nullptr,
                            reversed, target, pin, result);
    return false;
}

// Try short terminal entries first, then angle-led typed ingress for each pin.
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
    IngressTrace ingress_walk{docking};
    TypedIngressTrace typed_walk{docking, &ingress_walk};
    // Preserve the compact ordinary terminal representation and its existing
    // angle/short-hop behavior before considering longer typed ingress chains.
    NodeMask compact_pins = pins;
    bool compact_finished = false;
    for (int pin = compact_pins.firstSetBit(); pin >= 0 && !compact_finished;
         pin = compact_pins.firstSetBit())
    {
        compact_pins.clearBit(pin);
        for (const auto &entry : target.cb_type->terminalEntries(pin))
        {
            TerminalIngress ingress{&target, entry.dst, pin};
            ingress.nodes.push_back({&target, fpga::CB_NODE_DST, entry.dst});
            ingress.nodes.push_back({&target, fpga::CB_NODE_LOCAL, pin});
            for (int joint : {entry.joint, entry.joint2})
                if (joint >= 0)
                    ingress.nodes.push_back(
                        {&target, fpga::CB_NODE_JOINT, joint});
            fpga::Wire enter;
            enter.from = enter.to = target.coord;
            enter.local = entry.dst;
            enter.joint = entry.joint;
            enter.joint2 = entry.joint2;
            enter.pos = ROUTE_POS_TRANSIT;
            fpga::Wire endpoint;
            endpoint.type = fpga::Wire::WIRE_TILE_PIN;
            endpoint.from = endpoint.to = target.coord;
            endpoint.local = pin;
            endpoint.pin_dir = fpga::TILE_PIN_INPUT;
            ingress.suffix = {enter, endpoint};
            if (ingress_walk.run(ingress, target, source_hint, index, decisions,
                                 result, stopped))
            {
                compact_finished = true;
                break;
            }

        }

    }

    if (result.success || stopped)
        return result;

    // TerminalEntries covers only short local paths. Continue one free typed
    // predecessor at a time toward the source angle; a dead end ends this pin's
    // trace, without materializing alternative paths or a source frontier.
    NodeMask remaining_pins = pins;
    for (int pin = remaining_pins.firstSetBit(); pin >= 0;
         pin = remaining_pins.firstSetBit())
    {
        remaining_pins.clearBit(pin);
        if (typed_walk.run(target, source_hint, index, decisions, result,
                           stopped, pin))
            break;
    }

    return result;
}

// Retain one angle-led suffix; placement probes run only along that trace.
// Speculative leases are restored before the caller commits or moves a cell.
BackwardTakeoffRoute traceBackwardToTakeoff(
    fpga::Tile &target, NodeMask pins, fpga::Coord source_hint,
    BackwardResolveIndex &index, const BackwardTakeoffProbe &probe,
    const BackwardTakeoffCancel &cancel,
    const BackwardTakeoffStateView *state_view, std::ostream *decisions)
{
    Docking docking;
    docking.driver_probe = &probe;
    docking.driver_cancel = &cancel;
    docking.driver_state = state_view;
    TraceRouteBackward trace;
    trace.docking = &docking;
    return trace.run(target, pins, source_hint, index, decisions);
}

// Visit the exact incoming physical arcs used by reverse tracing.
void visitBackwardPhysicalSources(
    BackwardResolveIndex &index, fpga::Coord at, fpga::CBNodeNameType type,
    int node, const fpga::RepairIncomingSourceVisitor &visit)
{
    if (type == fpga::CB_NODE_DST)
    {
        if (const auto *sources =
                resolveBackwardSources(index, {at.x, at.y, node}, true))
            for (const auto &source : *sources)
                if (source.tile)
                    visit(source.tile->coord, fpga::CB_NODE_SRC, source.src);
    }

    const auto &locals = index.shared_cache ? index.shared_cache->local_incoming
                                            : index.local_incoming;
    const auto sources = locals.find({at.x, at.y, type, node});
    if (sources != locals.end())
        for (const auto &source : sources->second)
            if (source.tile)
                visit(source.tile->coord, fpga::CB_NODE_LOCAL, source.local);
}

} // namespace pnr
