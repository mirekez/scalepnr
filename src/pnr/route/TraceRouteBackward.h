#pragma once

#include "Device.h"
#include "Docking.h"
#include "TraceRouteCommon.h"

#include <array>
#include <ostream>
#include <vector>

namespace pnr
{

// Reverse angle-guided routing: one recursive invocation per Tile/CB hop.
struct TraceRouteBackward
{
    using TerminalIngress = BackwardTerminalIngress;
    static constexpr int ROUTE_POS_TRANSIT = 1;
    const Docking *docking = nullptr;
    size_t max_expansions = 32768;
    size_t remaining_expansions = 0;

    static bool isDeadend(fpga::Tile &tile, int dst, fpga::Coord destination,
                          BackwardResolveIndex &index);
    static bool markDeadend(fpga::Tile &tile, int dst,
                            BackwardResolveIndex &index);
    static void debug_trace(std::ostream *out, fpga::Tile &tile, int dst,
                            fpga::Coord source_hint);
    static void debug_probe(std::ostream *out, fpga::Tile &tile, int src,
                            bool accepted);

    // One route buffer belongs to the attempt, never to a recursive frame.
    std::vector<fpga::Wire> reverse_edges;

    // Release exactly the nodes leased by this physical hop on return.
    struct HopLeases
    {
        struct Bit
        {
            NodeMask *mask;
            int node;
        };

        std::array<Bit, 4> bits{};
        size_t count = 0;

        void lease(NodeMask &mask, int node)
        {
            if (node >= 0 && !mask.testBit(node))
            {
                mask.setBit(node);
                bits[count++] = {&mask, node};
            }

        }

        ~HopLeases()
        {
            while (count)
            {
                const auto bit = bits[--count];
                bit.mask->clearBit(bit.node);
            }

        }

    };

    // Write the one accepted or diagnostic route into its caller-owned buffer.
    static void writeSuffix(std::vector<fpga::Wire> &path,
                            const std::vector<fpga::Wire> &reverse_edges,
                            const TerminalIngress &ingress)
    {
        path.assign(reverse_edges.rbegin(), reverse_edges.rend());
        path.insert(path.end(), ingress.suffix.begin(), ingress.suffix.end());
    }

    // Retain the longest failed suffix for diagnostics and cancellation.
    static void retain_failure(BackwardTakeoffRoute &result, fpga::Tile &tile,
                               int dst,
                               const std::vector<fpga::Wire> &reverse_edges,
                               const TerminalIngress &ingress);

    // Append a typed edge in reverse traversal order.
    static void arc(std::vector<fpga::Wire> &reverse_edges, fpga::Coord from,
                    fpga::CBNodeNameType from_type, int from_node,
                    fpga::Coord to, fpga::CBNodeNameType to_type, int to_node)
    {
        fpga::Wire wire;
        wire.type = fpga::Wire::WIRE_ROUTE_EDGE;
        wire.from = from;
        wire.to = to;
        wire.from_node_type = from_type;
        wire.from_node = from_node;
        wire.to_node_type = to_type;
        wire.to_node = to_node;
        reverse_edges.push_back(std::move(wire));
    }

    // Build one physical jump with the chosen takeoff or transit endpoint.
    struct EdgeBuilder
    {
        fpga::Tile &arrival_tile;
        int arrival_dst;
        const BackwardResolveSource &source;

        fpga::Wire operator()(int local, int joint, int joint2,
                              bool takeoff_edge = false) const
        {
            auto &tile = *source.tile;
            fpga::Wire wire;
            wire.from = tile.coord;
            wire.to = arrival_tile.coord;
            wire.local = local;
            wire.jump = source.src;
            wire.route_jump = source.route_jump;
            wire.dst = arrival_dst;
            wire.joint = joint;
            wire.joint2 = joint2;
            wire.pos = takeoff_edge ? 0 : ROUTE_POS_TRANSIT;
            const auto type =
                takeoff_edge ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
            wire.from_wire_name =
                fromWireName(tile, type, local, source.src, joint, {});
            wire.src_wire_name =
                srcWireName(tile, type, local, source.src, joint, {});
            if (const auto *name = arrival_tile.cb_type->nodeName(
                    fpga::CB_NODE_DST, arrival_dst))
                wire.dst_wire_name = *name;
            return wire;
        }

    };

    // Recurse for one physical reverse hop, using direct inverse masks.
    bool next(fpga::Tile &arrival_tile, int arrival_dst,
              fpga::Coord source_hint, BackwardResolveIndex &index,
              std::ostream *decisions, BackwardTakeoffRoute &result,
              bool &stopped, const TerminalIngress &ingress);

    // Bind the attempt context once at Docking's ingress boundary.
    struct IngressVisitor
    {
        TraceRouteBackward &trace;
        fpga::Coord source_hint;
        BackwardResolveIndex &index;
        std::ostream *decisions;
        BackwardTakeoffRoute &result;
        bool &stopped;
        bool operator()(const BackwardTerminalIngress &ingress) const;
    };

    BackwardTakeoffRoute run(fpga::Tile &target, NodeMask pins,
                             fpga::Coord source_hint,
                             BackwardResolveIndex &index,
                             std::ostream *decisions);
};

// Direction-guided relocation trace: keep the first free prioritized hop.
// Recursive ingress attempts share one explicit expansion allowance.
BackwardTakeoffRoute traceBackwardToTakeoff(
    fpga::Tile &target, NodeMask pins, fpga::Coord source_hint,
    BackwardResolveIndex &index, const BackwardTakeoffProbe &probe,
    const BackwardTakeoffCancel &cancel = {},
    const BackwardTakeoffStateView *state_view = nullptr,
    std::ostream *decisions = nullptr, size_t max_expansions = 32768,
    const Docking::PlacementAllowed &placement_allowed = {});

} // namespace pnr
