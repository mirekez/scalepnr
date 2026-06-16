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
                if (transit_only && sameCoord(fragment.from, fragment.to)) {
                    continue;
                }
                return true;
            }
            continue;
        }

        if (node_type == CB_NODE_JOINT) {
            if (fragment.type == Wire::WIRE_CROSSBAR && fragment.joint == node) {
                if (!transit_only || fragment.pos != 0) {
                    if (transit_only && sameCoord(fragment.from, fragment.to)) {
                        continue;
                    }
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

void rebuildNetRouteTiles(rtl::Net& net)
{
    for (auto& tile_ref : Device::current().tile_grid) {
        clearTileNetRef(tile_ref, net);
    }
    for (rtl::NetRouteBinding& remaining : net.routes) {
        std::vector<Wire>* remaining_route = bindingRoute(remaining);
        if (remaining_route && !remaining_route->empty()) {
            registerNetRouteTiles(net, *remaining_route);
        }
    }
}

// Resolve a placed instance port to a stable net endpoint reference.
Referable<rtl::Port>* findInstPort(rtl::Inst* inst, const std::string& port_name)
{
    if (!inst || port_name.empty()) {
        return nullptr;
    }
    for (auto& conn : inst->conns) {
        if (conn.port_ref.peer && conn.port_ref->name == port_name) {
            return conn.port_ref.peer;
        }
    }
    return nullptr;
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


void clearRouteLeases(const std::vector<Wire>& route, bool clear_shared = false)
{
    for (size_t i = 0; i < route.size(); ++i) {
        const Wire& fragment = route[i];
        if (fragment.shared && !clear_shared) {
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
            tile->pin_state.leased_nodes &= ~(NodeMask{0,1} << fragment.local);
            tile->cb.local.local &= ~(NodeMask{0,1} << fragment.local);
            if (i > 0 && route[i - 1].type == Wire::WIRE_CROSSBAR && route[i - 1].local >= 0) {
                tile->cb.dst.jump &= ~(NodeMask{0,1} << route[i - 1].local);
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
            tile->cb.src.jump &= ~(NodeMask{0,1} << fragment.jump);
            tile->cb.src_deadend.jump &= ~(NodeMask{0,1} << fragment.jump);
        }
        if (fragment.joint >= 0) {
            tile->cb.joint.jump &= ~(NodeMask{0,1} << fragment.joint);
        }
        if (fragment.pos == 0 && fragment.local >= 0) {
            tile->cb.local.local &= ~(NodeMask{0,1} << fragment.local);
        }
        if (fragment.pos == 1 && fragment.local >= 0) {
            tile->cb.dst.jump &= ~(NodeMask{0,1} << fragment.local);
        }
    }
}

}

void fpga::releaseRouteFragmentLease(const std::vector<Wire>& route, size_t fragment_index)
{
    if (fragment_index >= route.size()) {
        return;
    }
    const Wire& fragment = route[fragment_index];
    if (fragment.type == Wire::WIRE_TILE_PIN) {
        if (fragment_index + 1 != route.size()) {
            return;
        }
        Tile* tile = Device::current().getTile(fragment.from.x, fragment.from.y);
        if (!tile || fragment.local < 0) {
            return;
        }
        tile->pin_state.leased_nodes &= ~(NodeMask{0,1} << fragment.local);
        tile->cb.local.local &= ~(NodeMask{0,1} << fragment.local);
        if (fragment_index > 0 && route[fragment_index - 1].type == Wire::WIRE_CROSSBAR
            && route[fragment_index - 1].local >= 0) {
            tile->cb.dst.jump &= ~(NodeMask{0,1} << route[fragment_index - 1].local);
        }
        return;
    }

    if (fragment.type != Wire::WIRE_CROSSBAR) {
        return;
    }
    Tile* tile = Device::current().getTile(fragment.from.x, fragment.from.y);
    if (!tile) {
        return;
    }
    if (fragment.jump >= 0) {
        tile->cb.src.jump &= ~(NodeMask{0,1} << fragment.jump);
        tile->cb.src_deadend.jump &= ~(NodeMask{0,1} << fragment.jump);
    }
    if (fragment.joint >= 0) {
        tile->cb.joint.jump &= ~(NodeMask{0,1} << fragment.joint);
    }
    if (fragment.pos == 1 && fragment.local >= 0) {
        tile->cb.dst.jump &= ~(NodeMask{0,1} << fragment.local);
    }
}

void fpga::attachNetRoute(rtl::Net& net, rtl::Inst& owner, size_t route_index,
                          rtl::Inst* from, rtl::Inst* to,
                          const std::string& from_port, const std::string& to_port,
                          const std::string& route_name)
{
    if (!net.src_port.peer) {
        if (Referable<rtl::Port>* port = findInstPort(from, from_port)) {
            net.src_port.set(port);
        }
    }
    if (!net.dst_port.peer) {
        if (Referable<rtl::Port>* port = findInstPort(to, to_port)) {
            net.dst_port.set(port);
        }
    }

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
    registerNetRouteTilesFrom(net, route, 0);
}

void fpga::registerNetRouteTilesFrom(rtl::Net& net, const std::vector<Wire>& route, size_t first_fragment)
{
    for (size_t index = first_fragment; index < route.size(); ++index) {
        const Wire& fragment = route[index];
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

    rebuildNetRouteTiles(net);
    return true;
}

bool fpga::unrouteNetBranch(rtl::Net& net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size()) {
        return false;
    }
    rtl::NetRouteBinding& binding = net.routes[route_binding_index];
    std::vector<Wire>* route = bindingRoute(binding);
    if (!route || route->empty()) {
        return false;
    }

    size_t branch_start = 0;
    while (branch_start < route->size() && (*route)[branch_start].shared) {
        ++branch_start;
    }
    if (branch_start == route->size()) {
        return false;
    }

    std::vector<Wire> branch(route->begin() + static_cast<std::ptrdiff_t>(branch_start), route->end());
    clearRouteLeases(branch, true);
    route->resize(branch_start);
    rebuildNetRouteTiles(net);
    return true;
}

bool fpga::unrouteBrunch(rtl::Net& net, size_t route_binding_index)
{
    return unrouteNetBranch(net, route_binding_index);
}

bool fpga::discardNetBranch(rtl::Net& net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size()) {
        return false;
    }
    rtl::NetRouteBinding& binding = net.routes[route_binding_index];
    std::vector<Wire>* route = bindingRoute(binding);
    if (!route || route->empty()) {
        return false;
    }

    size_t branch_start = 0;
    while (branch_start < route->size() && (*route)[branch_start].shared) {
        ++branch_start;
    }
    if (branch_start < route->size()) {
        std::vector<Wire> branch(route->begin() + static_cast<std::ptrdiff_t>(branch_start), route->end());
        clearRouteLeases(branch, true);
    }
    route->clear();
    rebuildNetRouteTiles(net);
    return true;
}

// Clear an atomic route tree, including shared fanout fragments owned by the tree.
bool fpga::unrouteNetRouteTree(rtl::Net& net, const std::vector<size_t>& route_binding_indices)
{
    bool changed = false;
    for (size_t route_binding_index : route_binding_indices) {
        if (route_binding_index >= net.routes.size()) {
            continue;
        }
        std::vector<Wire>* route = bindingRoute(net.routes[route_binding_index]);
        if (!route || route->empty()) {
            continue;
        }
        clearRouteLeases(*route, true);
        route->clear();
        changed = true;
    }
    if (!changed) {
        return false;
    }
    rebuildNetRouteTiles(net);
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
        clearRouteLeases(*route, true);
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
