#pragma once

#include "Docking.h"
#include "ForwardRouteDiagnostic.h"
#include "RouteDiagnostics.h"
#include "TraceRouteCommon.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace pnr
{

// A source-connected starting node for one atomic routing attempt.
struct RoutingAnchor
{
    fpga::Tile *tile = nullptr;
    int node = -1;
    bool starts_at_dst = true;
    // Existing, owned source-to-anchor fabric retained by the caller.
    std::vector<fpga::Wire> prefix;
};

// Trace one source-connected anchor toward a movable destination.
// Moving callers try anchors per connection; Basic and Fanouts use the
// forward candidate selector below.
struct TraceRouteForward
{
    // Select angle-ranked candidates for the shared Basic/Fanouts trace.
    // The caller owns leasing, deadend marking, and unresolved diagnostics.
    static void candidates(fpga::CBType &type, fpga::CBNodeNameType from_type,
                           int from_node, fpga::Coord angle,
                           const NodeMask &allowed,
                           std::vector<uint16_t> &active,
                           std::vector<uint16_t> &unresolved);

    // Each descent leases exact bits in the live tile and restores only those
    // bits as its recursive call unwinds. Committed SRC deadends remain set.
    struct LeaseScope
    {
        fpga::Tile &tile;
        ForwardRouteDiagnostic *diagnostic;

        struct Bit
        {
            NodeMask *mask = nullptr;
            int node = -1;
            bool was_set = false;
        };

        std::array<Bit, 4> bits{};
        size_t count = 0;
        bool diagnostic_added = false;

        LeaseScope(fpga::Tile &tile, ForwardRouteDiagnostic *diagnostic,
                   TraceRouteForward &trace)
            : tile(tile), diagnostic(diagnostic)
        {
            if (diagnostic)
                diagnostic_added = diagnostic->remember(tile, trace);
        }

        void lease(NodeMask &mask, int node)
        {
            if (node < 0)
                return;
            bits[count++] = {&mask, node, mask.testBit(node)};
            mask.setBit(node);
        }

        ~LeaseScope()
        {
            while (count)
            {
                auto &bit = bits[--count];
                if (bit.was_set)
                    bit.mask->setBit(bit.node);
                else
                    bit.mask->clearBit(bit.node);
            }

            if (diagnostic)
                diagnostic->forget(tile, diagnostic_added);
        }

    };

    // Lease the exact typed nodes in one Docking path during its recursive
    // attempt, then release them when that attempt returns.
    struct InternalLeaseScope
    {
        TraceRouteForward &trace;
        struct Bit
        {
            NodeMask *mask;
            int node;
            bool was_set;
        };
        std::vector<Bit> bits;
        std::vector<fpga::Tile *> remembered;

        InternalLeaseScope(TraceRouteForward &trace, size_t count)
            : trace(trace)
        {
            bits.reserve(count);
            remembered.reserve(count);
        }

        void lease(fpga::Tile &tile, fpga::CBNodeNameType type, int node);
        ~InternalLeaseScope();
    };

    // Hold the physical-hop cursor while Docking visits internal alternatives.
    struct InternalPathAttempt
    {
        TraceRouteForward &trace;
        const RoutingAnchor &anchor;
        fpga::Coord aim;
        size_t &queued;
        size_t cursor;
        size_t depth;
        size_t same_tile_depth;
        bool terminal_only;

        bool operator()(const std::vector<DockingForwardNode> &path);
    };

    static constexpr int offsets[] = {0, -1, 1, -2, 2, -3, 3, 4};

    // The caller owns sink decisions for the lifetime of one forward trace.
    const Docking *docking = nullptr;
    ForwardRouteDiagnostic *diagnostic = nullptr;
    size_t max_expansions = 1000000;
    size_t expansions_used = 0;
    bool success = false;
    bool cancelled = false;
    bool budget_exhausted = false;
    size_t anchor = 0;
    size_t expanded = 0;
    size_t edge_checks = 0;
    size_t failed_states = 0;
    size_t failure_cache_hits = 0;
    size_t tile_visit_rejects = 0;
    size_t snapshot_lookup_checks = 0;
    // The recursive path becomes the returned route on success.
    std::vector<fpga::Wire> fragments;

    bool run(const RoutingAnchor &anchor, fpga::Coord aim);

    // Map a coordinate delta to the octant used for LOCAL link priority.
    static int direction(fpga::Coord delta)
    {
        const int x = (delta.x > 0) - (delta.x < 0);
        const int y = (delta.y > 0) - (delta.y < 0);
        static constexpr int octants[3][3] = {{7, 6, 5}, {0, -1, 4}, {1, 2, 3}};
        return octants[y + 1][x + 1];
    }

    // Select the mutable lease mask for a typed node.
    static NodeMask &node_mask(fpga::Tile &tile, fpga::CBNodeNameType type)
    {
        return type == fpga::CB_NODE_LOCAL   ? tile.cb.local.local
               : type == fpga::CB_NODE_JOINT ? tile.cb.joint.jump
               : type == fpga::CB_NODE_SRC   ? tile.cb.src.jump
                                             : tile.cb.dst.jump;
    }

    // Filter SRC choices; a jump directly into the destination may terminate.
    static bool isDeadend(fpga::Tile &tile, int src, fpga::Coord destination);

    // Mark an incoming SRC only after all successor SRCs are persistently dead.
    // Used only by this trace after a failed continuation.
    static bool markDeadend(fpga::Tile &tile, int src);

    // A marked exit remains a candidate only for an eligible next-hop endpoint.
    bool canFinishJump(fpga::Tile &tile, int src, fpga::Coord aim) const;

    void recordDeadend(fpga::Tile &tile, int src, size_t depth, size_t cursor);

    // Report an occupied node without changing deadend proof.
    void
    blocked(fpga::Tile &tile, const char *reason,
            std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes);

    // Add a typed graph edge to the route under construction.
    static void append_arc(std::vector<fpga::Wire> &path, fpga::Coord from,
                           fpga::CBNodeNameType from_type, int from_node,
                           fpga::Coord to, fpga::CBNodeNameType to_type,
                           int to_node)
    {
        fpga::Wire arc;
        arc.type = fpga::Wire::WIRE_ROUTE_EDGE;
        arc.from = from;
        arc.to = to;
        arc.from_node_type = from_type;
        arc.from_node = from_node;
        arc.to_node_type = to_type;
        arc.to_node = to_node;
        path.push_back(arc);
    }

    // Check one selected LOCAL terminal without a same-Tile recursive call.
    bool tryLocalTerminal(fpga::Tile &tile, fpga::CBNodeNameType arrival_type,
                          int arrival, int local, bool terminal_only,
                          size_t cursor);

    // Ask bounded Docking full search for longer LOCAL/JOINT paths.
    bool tryFullInternal(const RoutingAnchor &anchor, fpga::Coord aim,
                         size_t &queued, fpga::Tile &tile,
                         fpga::CBNodeNameType type, int arrival, size_t cursor,
                         size_t depth, size_t same_tile_depth,
                         bool terminal_only);
    bool tryInternalPath(const InternalPathAttempt &attempt,
                         const std::vector<DockingForwardNode> &path);

    // Each call owns one Tile hop; LOCAL/JOINT bridges use fixed mask checks.
    bool next(const RoutingAnchor &anchor, fpga::Coord aim, size_t &queued,
              fpga::Tile &tile, int node, fpga::CBNodeNameType type,
              long long parent, size_t depth, size_t same_tile_depth,
              bool terminal_only = false);
};

// Count recursive node visits across all anchors. Zero selects the default
// budget; reaching the budget leaves the attempt incomplete.
TraceRouteForward routeForwardToPlacement(
    const std::vector<RoutingAnchor> &anchors, const Docking &docking,
    size_t max_expansions = 0, std::ostream *decisions = nullptr,
    const ForwardRouteDiagnostic::Observer &observe = {},
    fpga::Coord aim = {-1, -1}, std::ostream *audit = nullptr);

} // namespace pnr
