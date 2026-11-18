#include "Wire.h"

#include <algorithm>

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
