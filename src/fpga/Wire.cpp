#include "Wire.h"
#include "Device.h"
#include "Tile.h"

#include <algorithm>

using namespace fpga;

void Wire::assign(rtl::Net* net)
{
    PNR_ASSERT(net->wire.peer == nullptr, "assigning wire {}:{}-{}:{} to already assigned net {}", from.x, from.y, to.x, to.y, net->makeName());
    net->wire.set(static_cast<Referable<Wire>*>(this));
}

namespace {

bool sameCoord(const Coord& a, const Coord& b)
{
    return a.x == b.x && a.y == b.y;
}

bool routeUsesNodeOnTile(const std::vector<Wire>& route, const Tile& tile,
                         CBNodeNameType node_type, int node, bool transit_only)
{
    for (const Wire& fragment : route) {
        if (!sameCoord(fragment.from, tile.coord)) {
            continue;
        }

        if (node_type == CB_NODE_SRC) {
            if (fragment.type == Wire::WIRE_CROSSBAR && fragment.jump == node) {
                if (!transit_only || fragment.pos != 0) {
                    return true;
                }
            }
            continue;
        }

        if (node_type == CB_NODE_DST) {
            if (fragment.type == Wire::WIRE_CROSSBAR && fragment.local == node && fragment.pos != 0) {
                if (transit_only && sameCoord(fragment.from, fragment.to)) {
                    continue;
                }
                return true;
            }
            continue;
        }

        if (node_type == CB_NODE_LOCAL) {
            if (fragment.local == node && (!transit_only || fragment.pos != 0)) {
                return true;
            }
            continue;
        }

        if (node_type == CB_NODE_JOINT) {
            if (fragment.type == Wire::WIRE_CROSSBAR && fragment.joint == node) {
                if (!transit_only || fragment.pos != 0) {
                    return true;
                }
            }
            continue;
        }
    }
    return false;
}

std::vector<Wire>* bindingRoute(rtl::NetRouteBinding& binding)
{
    if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
        return nullptr;
    }
    return &binding.owner->wires[binding.route_index];
}

void clearTileNetRef(Tile& tile, rtl::Net& net)
{
    auto& refs = tile.routedNets;
    for (auto& ref : refs) {
        if (ref.peer == &net) {
            ref.clear();
        }
    }
}

void addTileNetRef(Tile& tile, rtl::Net& net)
{
    for (auto& ref : tile.routedNets) {
        if (ref.peer == &net) {
            return;
        }
    }
    for (auto& ref : tile.routedNets) {
        if (!ref.peer) {
            ref.set(static_cast<Referable<rtl::Net>*>(&net));
            return;
        }
    }
    Ref<rtl::Net>& ref = tile.routedNets.emplace_back();
    ref.set(static_cast<Referable<rtl::Net>*>(&net));
}

void clearRouteLeases(const std::vector<Wire>& route)
{
    for (size_t i = 0; i < route.size(); ++i) {
        const Wire& fragment = route[i];
        if (fragment.shared) {
            continue;
        }
        if (fragment.type == Wire::WIRE_TILE_PIN) {
            if (i + 1 != route.size()) {
                continue;
            }
            Tile* tile = Device::current().getTile(fragment.from.x, fragment.from.y);
            if (!tile || fragment.local < 0) {
                continue;
            }
            tile->pin_state.leased_nodes &= ~(u256{0,1} << fragment.local);
            tile->cb.local.local &= ~(u256{0,1} << fragment.local);
            if (i > 0 && route[i - 1].type == Wire::WIRE_CROSSBAR && route[i - 1].local >= 0) {
                tile->cb.dst.jump &= ~(u256{0,1} << route[i - 1].local);
            }
            continue;
        }

        if (fragment.type != Wire::WIRE_CROSSBAR) {
            continue;
        }
        Tile* tile = Device::current().getTile(fragment.from.x, fragment.from.y);
        if (!tile) {
            continue;
        }
        if (fragment.jump >= 0) {
            tile->cb.src.jump &= ~(u256{0,1} << fragment.jump);
        }
        if (fragment.pos == 1 && fragment.local >= 0) {
            tile->cb.dst.jump &= ~(u256{0,1} << fragment.local);
        }
    }
}

}

void fpga::attachNetRoute(rtl::Net& net, rtl::Inst& owner, size_t route_index,
                          rtl::Inst* from, rtl::Inst* to,
                          const std::string& from_port, const std::string& to_port,
                          const std::string& route_name)
{
    for (rtl::NetRouteBinding& binding : net.routes) {
        if (binding.route_name == route_name && binding.to == to) {
            binding.owner = &owner;
            binding.route_index = route_index;
            binding.from = from;
            binding.to = to;
            binding.from_port = from_port;
            binding.to_port = to_port;
            return;
        }
    }
    net.routes.push_back(rtl::NetRouteBinding{
        &owner,
        route_index,
        from,
        to,
        from_port,
        to_port,
        route_name
    });
}

void fpga::registerNetRouteTiles(rtl::Net& net, const std::vector<Wire>& route)
{
    for (const Wire& fragment : route) {
        Tile* from_tile = Device::current().getTile(fragment.from.x, fragment.from.y);
        if (from_tile) {
            addTileNetRef(*from_tile, net);
        }
        Tile* to_tile = Device::current().getTile(fragment.to.x, fragment.to.y);
        if (to_tile) {
            addTileNetRef(*to_tile, net);
        }
    }
}

rtl::Net* fpga::findNetByNode(Tile& tile, CBNodeNameType node_type, int node, bool transit_only)
{
    for (auto& ref : tile.routedNets) {
        rtl::Net* net = ref.peer;
        if (!net) {
            continue;
        }
        for (rtl::NetRouteBinding& binding : net->routes) {
            std::vector<Wire>* route = bindingRoute(binding);
            if (!route || route->empty()) {
                continue;
            }
            if (routeUsesNodeOnTile(*route, tile, node_type, node, transit_only)) {
                return net;
            }
        }
    }
    return nullptr;
}

bool fpga::unrouteNetRoute(rtl::Net& net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size()) {
        return false;
    }
    rtl::NetRouteBinding& binding = net.routes[route_binding_index];
    std::vector<Wire>* route = bindingRoute(binding);
    if (!route || route->empty()) {
        return false;
    }
    clearRouteLeases(*route);
    route->clear();

    for (auto& tile_ref : Device::current().tile_grid) {
        clearTileNetRef(tile_ref, net);
    }
    for (rtl::NetRouteBinding& remaining : net.routes) {
        std::vector<Wire>* remaining_route = bindingRoute(remaining);
        if (remaining_route && !remaining_route->empty()) {
            registerNetRouteTiles(net, *remaining_route);
        }
    }
    return true;
}

bool fpga::unrouteNet(rtl::Net& net)
{
    bool changed = false;
    for (rtl::NetRouteBinding& binding : net.routes) {
        std::vector<Wire>* route = bindingRoute(binding);
        if (!route || route->empty()) {
            continue;
        }
        clearRouteLeases(*route);
        route->clear();
        changed = true;
    }

    if (changed) {
        for (auto& tile_ref : Device::current().tile_grid) {
            clearTileNetRef(tile_ref, net);
        }
    }
    return changed;
}
