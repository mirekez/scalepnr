#include "Wire.h"
#include "Device.h"
#include "Tile.h"

#include <algorithm>
#include <ostream>

using namespace fpga;

void Wire::assign(rtl::Net *net)
{
    PNR_ASSERT(net->wire.peer == nullptr,
               "assigning wire {}:{}-{}:{} to already assigned net {}", from.x,
               from.y, to.x, to.y, net->makeName());
    net->wire.set(static_cast<Referable<Wire> *>(this));
}

bool fpga::isRouteComplete(const std::vector<Wire> &route)
{
    if (route.empty() || route.back().type != Wire::WIRE_TILE_PIN)
    {
        return false;
    }

    bool has_crossbar = false;
    size_t tile_pin_count = 0;
    Coord endpoint_coord{-1, -1};
    bool has_endpoint = false;
    bool distinct_endpoints = false;
    for (const Wire &fragment : route)
    {
        if (fragment.type == Wire::WIRE_GAP)
            return false;
        if (fragment.type == Wire::WIRE_CROSSBAR ||
            fragment.type == Wire::WIRE_ROUTE_EDGE)
        {
            has_crossbar = true;
            continue;
        }

        if (fragment.type != Wire::WIRE_TILE_PIN)
        {
            continue;
        }

        ++tile_pin_count;
        Coord coord = fragment.resource.x >= 0 && fragment.resource.y >= 0
                          ? fragment.resource
                          : fragment.to;
        if (!has_endpoint)
        {
            endpoint_coord = coord;
            has_endpoint = true;
        }

        else if (endpoint_coord.x != coord.x || endpoint_coord.y != coord.y)
        {
            distinct_endpoints = true;
        }

    }

    return has_crossbar || (!distinct_endpoints && tile_pin_count >= 2);
}

bool fpga::hasRouteGaps(const std::vector<Wire> &route)
{
    return std::any_of(route.begin(), route.end(),
                       [](const Wire &w) { return w.type == Wire::WIRE_GAP; });
}

fpga::RouteState fpga::routeState(const std::vector<Wire> &route)
{
    return route.empty()            ? RouteState::Unrouted
           : isRouteComplete(route) ? RouteState::Routed
                                    : RouteState::PartiallyRouted;
}

size_t fpga::WireNodeHash::operator()(const WireNode &node) const noexcept
{
    const auto [x, y, kind, id] = node;
    size_t hash = 0;
    for (int value : {x, y, kind, id})
        hash ^= std::hash<int>{}(value) + size_t{0x9e3779b9} + (hash << 6) +
                (hash >> 2);
    return hash;
}

// Enumerate numeric connectivity for angle tracing and retained-route checks.
void fpga::WireNodeSearch::successors(
    const WireNode &node,
    const std::function<void(const WireNode &)> &visit) const
{
    const auto [x, y, kind, id] = node;
    auto *tile = device.getTile(x, y);
    if (!tile || !tile->cb_type || id < 0 || id >= CB_MAX_NODES)
        return;
    const auto &cb = *tile->cb_type;
    auto mask = [&](const NodeMask &bits, int type)
    {
        bits.for_each_set_bit(
            [&](int n)
            {
                visit(WireNode{x, y, type, n});
                return false;
            });
    };

    switch (kind)
    {
    case CB_NODE_LOCAL:
        mask(cb.local_src[id].jump, CB_NODE_SRC);
        mask(cb.local_joint[id].joint, CB_NODE_JOINT);
        mask(cb.local_local[id].local, CB_NODE_LOCAL);
        for (const auto &t : device.resolveLocalTargets(*tile, id))
            if (t.tile)
                visit(WireNode{t.tile->coord.x, t.tile->coord.y, t.node_type,
                               t.node});
        break;
    case CB_NODE_DST:
        mask(cb.dst_src[id].jump, CB_NODE_SRC);
        mask(cb.dst_joint[id].joint, CB_NODE_JOINT);
        mask(cb.dst_local[id].local, CB_NODE_LOCAL);
        break;
    case CB_NODE_JOINT:
        mask(cb.joint_src[id].jump, CB_NODE_SRC);
        mask(cb.joint_joint[id].joint, CB_NODE_JOINT);
        mask(cb.joint_local[id].local, CB_NODE_LOCAL);
        break;
    case CB_NODE_SRC:
        mask(cb.src_joint[id].joint, CB_NODE_JOINT);
        for (const auto &t : device.resolveJumpTargets(*tile, id))
            if (t.tile && t.dst_node >= 0)
                visit(WireNode{t.tile->coord.x, t.tile->coord.y, CB_NODE_DST,
                               t.dst_node});
        break;
    }

}

// Order physical alternatives through Crossbar; callers decide resource
// eligibility.
std::vector<std::pair<WireNode, int>>
fpga::WireNodeSearch::orderedSuccessors(const WireNode &node, int selected_src)
{
    std::vector<WireNode> edges;
    successors(node, [&](const WireNode &next) { edges.push_back(next); });
    std::vector<std::pair<WireNode, int>> ordered;
    ordered.reserve(edges.size());
    // A reached terminal is complete before any speculative angle exit.
    const auto terminal_edge = std::find(edges.begin(), edges.end(), sink);
    if (terminal_edge != edges.end())
    {
        ordered.push_back({sink, -1});
        edges.erase(terminal_edge);
    }

    const auto [x, y, kind, id] = node;
    auto *tile = device.getTile(x, y);
    if (!tile || !tile->cb_type)
        return ordered;
    if (kind == CB_NODE_SRC)
    {
        for (const auto &next : edges)
            if (std::get<2>(next) == CB_NODE_DST)
                ordered.push_back({next, -1});
        for (const auto &next : edges)
            if (std::get<2>(next) != CB_NODE_DST)
                ordered.push_back({next, -1});
        return ordered;
    }

    auto &cb = *tile->cb_type;
    std::vector<NodeMask> exits(edges.size());
    for (size_t i = 0; i < edges.size(); ++i)
    {
        if (std::get<0>(edges[i]) != x || std::get<1>(edges[i]) != y)
            continue;
        auto [entry, inserted] = local_exits.try_emplace(edges[i]);
        if (inserted)
        {
            std::vector<WireNode> pending{edges[i]};
            std::unordered_set<WireNode, WireNodeHash> seen;
            while (!pending.empty())
            {
                const auto at = pending.back();
                pending.pop_back();
                if (!seen.insert(at).second)
                    continue;
                if (std::get<2>(at) == CB_NODE_SRC)
                {
                    entry->second.setBit(std::get<3>(at));
                    continue;
                }

                successors(at,
                           [&](const WireNode &next)
                           {
                               if (std::get<0>(next) == x &&
                                   std::get<1>(next) == y)
                                   pending.push_back(next);
                           });
            }

        }

        exits[i] = entry->second;
    }

    if (selected_src >= 0)
    {
        for (size_t i = 0; i < edges.size(); ++i)
            if (exits[i].testBit(selected_src))
                ordered.push_back({edges[i], selected_src});
        return ordered;
    }

    CBType::FastSearchCursor source_cursor;
    std::vector<bool> used(edges.size(), false);
    for (int src;
         (src = cb.fastSearchNextHop(
              Coord{std::get<0>(sink), std::get<1>(sink)} - tile->coord,
              source_cursor)) >= 0;)
    {
        if (diagnostic && node == source && kind == CB_NODE_LOCAL)
            *diagnostic << "SOURCE_ORDER src=" << src
                        << " deadend=" << tile->cb.src_deadend.jump.testBit(src)
                        << '\n';
        if (tile->cb.src_deadend.jump.testBit(src))
            continue;
        for (size_t i = 0; i < edges.size(); ++i)
        {
            if (exits[i].testBit(src))
            {
                used[i] = true;
                ordered.push_back({edges[i], src});
            }

        }

    }

    for (size_t i = 0; i < edges.size(); ++i)
        if (!used[i])
            ordered.push_back({edges[i], -1});
    return ordered;
}
