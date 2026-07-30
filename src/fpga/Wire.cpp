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
        bool from_tile = sameCoord(fragment.from, tile.coord);
        bool to_tile = sameCoord(fragment.to, tile.coord);
        if (!from_tile && !to_tile) {
            continue;
        }

        if (node_type == CB_NODE_SRC) {
            if (from_tile && fragment.type == Wire::WIRE_CROSSBAR && fragment.jump == node) {
                if (!transit_only || fragment.pos != 0) {
                    return true;
                }
            }
            continue;
        }

        if (node_type == CB_NODE_DST) {
            // A fragment leases only its source-tile incoming node. Its landing
            // node is owned by the following fragment that exits or grounds there.
            if (from_tile && fragment.type == Wire::WIRE_CROSSBAR
                && fragment.pos != 0 && fragment.owns_dst
                && fragment.local == node
                && (!transit_only || !sameCoord(fragment.from, fragment.to))) {
                return true;
            }
            continue;
        }

        if (node_type == CB_NODE_LOCAL) {
            if (transit_only) {
                continue;
            }
            if (fragment.type == Wire::WIRE_TILE_PIN && from_tile && fragment.local == node) {
                return true;
            }
            if (fragment.type == Wire::WIRE_CROSSBAR && from_tile && fragment.pos == 0 && fragment.local == node) {
                return true;
            }
            continue;
        }

        if (node_type == CB_NODE_JOINT) {
            if (from_tile && fragment.type == Wire::WIRE_CROSSBAR
                && (fragment.joint == node || fragment.joint2 == node)) {
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
            if (i > 0 && route[i - 1].type == Wire::WIRE_CROSSBAR
                && route[i - 1].local >= 0 && route[i - 1].owns_dst) {
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
        }
        if (fragment.joint >= 0) {
            tile->cb.joint.jump &= ~(NodeMask{0,1} << fragment.joint);
        }
        if (fragment.joint2 >= 0) {
            tile->cb.joint.jump &= ~(NodeMask{0,1} << fragment.joint2);
        }
        if (fragment.pos == 0 && fragment.local >= 0) {
            tile->cb.local.local &= ~(NodeMask{0,1} << fragment.local);
        }
        if (fragment.pos != 0 && fragment.local >= 0 && fragment.owns_dst) {
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
            && route[fragment_index - 1].local >= 0 && route[fragment_index - 1].owns_dst) {
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
    }
    if (fragment.joint >= 0) {
        tile->cb.joint.jump &= ~(NodeMask{0,1} << fragment.joint);
    }
    if (fragment.joint2 >= 0) {
        tile->cb.joint.jump &= ~(NodeMask{0,1} << fragment.joint2);
    }
    if (fragment.pos == 0 && fragment.local >= 0) {
        tile->cb.local.local &= ~(NodeMask{0,1} << fragment.local);
    }
    if (fragment.pos != 0 && fragment.local >= 0 && fragment.owns_dst) {
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

    // Search existing physical bindings before creating another route branch.
    for (rtl::NetRouteBinding& binding : net.routes) {
        // Match the complete endpoint identity; a display name alone is not unique.
        if (binding.route_name == route_name
            && binding.from == from && binding.to == to
            && binding.from_port == from_port && binding.to_port == to_port) {
            // A repeated attachment to the same storage requires no update.
            if (binding.owner == &owner && binding.route_index == route_index) {
                // Preserve the existing exact binding.
                return;
            }
            // Inspect the physical route currently owned by the exact binding.
            std::vector<Wire>* route = bindingRoute(binding);
            // Exact endpoint identity denotes one physical branch. Preserve an
            // already-complete owner; otherwise transfer its pending binding.
            if (route && isRouteComplete(*route)) {
                // Never replace completed physical routing with a pending owner.
                return;
            }
            // Transfer an incomplete binding to its current route-vector owner.
            binding.owner = &owner;
            // Record the route index within the new owner.
            binding.route_index = route_index;
            // The exact identity has been updated, so no new binding is needed.
            return;
        }
    }
    // No exact endpoint identity exists; create one independent physical branch.
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

size_t fpga::retargetNetRouteBindings(rtl::Net& old_net, rtl::Net& new_net,
                                      rtl::Inst* old_from, rtl::Inst* old_to,
                                      const std::string& old_from_port, const std::string& old_to_port,
                                      rtl::Inst* new_from, rtl::Inst* new_to,
                                      const std::string& new_from_port, const std::string& new_to_port,
                                      const std::string& route_name)
{
    size_t changed = 0;
    for (size_t index = 0; index < old_net.routes.size();) {
        rtl::NetRouteBinding& binding = old_net.routes[index];
        if (binding.from != old_from || binding.to != old_to
            || binding.from_port != old_from_port || binding.to_port != old_to_port
            || binding.route_name != route_name) {
            ++index;
            continue;
        }
        rtl::NetRouteBinding replacement = binding;
        replacement.from = new_from;
        replacement.to = new_to;
        replacement.from_port = new_from_port;
        replacement.to_port = new_to_port;
        if (&old_net == &new_net) {
            binding = std::move(replacement);
            ++index;
        }
        else {
            old_net.routes.erase(old_net.routes.begin() + static_cast<std::ptrdiff_t>(index));
            new_net.routes.push_back(std::move(replacement));
        }
        ++changed;
    }
    // Retargeting changes the Net found through each routed tile, including an existing partial prefix.
    if (changed != 0) {
        rebuildNetRouteTiles(old_net);
        if (&new_net != &old_net) {
            rebuildNetRouteTiles(new_net);
        }
    }
    return changed;
}

// Keep every branch of one physical source tree on the same endpoint identity.
size_t fpga::retargetNetRouteSourceBindings(rtl::Net& net, rtl::Inst* old_from,
                                            const std::string& old_from_port,
                                            rtl::Inst* new_from, const std::string& new_from_port)
{
    size_t changed = 0;
    for (rtl::NetRouteBinding& binding : net.routes) {
        if (binding.from != old_from || binding.from_port != old_from_port) {
            continue;
        }
        binding.from = new_from;
        binding.from_port = new_from_port;
        ++changed;
    }
    return changed;
}

bool fpga::isRouteComplete(const std::vector<Wire>& route)
{
    if (route.empty() || route.back().type != Wire::WIRE_TILE_PIN) {
        return false;
    }
    bool has_crossbar = false;
    size_t tile_pin_count = 0;
    Coord endpoint_coord{-1, -1};
    bool has_endpoint = false;
    for (const Wire& fragment : route) {
        if (fragment.type == Wire::WIRE_CROSSBAR) {
            has_crossbar = true;
            continue;
        }
        if (fragment.type != Wire::WIRE_TILE_PIN) {
            continue;
        }
        ++tile_pin_count;
        Coord coord = fragment.resource.x >= 0 && fragment.resource.y >= 0
            ? fragment.resource
            : fragment.to;
        if (!has_endpoint) {
            endpoint_coord = coord;
            has_endpoint = true;
        }
        else if (endpoint_coord.x != coord.x || endpoint_coord.y != coord.y) {
            return has_crossbar;
        }
    }
    return has_crossbar || tile_pin_count >= 2;
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
    std::vector<NetRouteRef> routes = findNetRoutesByNode(tile, node_type, node, transit_only);
    return routes.empty() ? nullptr : routes.front().net;
}

std::vector<NetRouteRef> fpga::findNetRoutesByNode(
    Tile& tile, CBNodeNameType node_type, int node, bool transit_only)
{
    std::vector<NetRouteRef> result;
    for (auto& ref : tile.routedNets) {
        rtl::Net* net = ref.peer;
        if (!net) {
            continue;
        }
        for (size_t binding_index = 0; binding_index < net->routes.size(); ++binding_index) {
            rtl::NetRouteBinding& binding = net->routes[binding_index];
            std::vector<Wire>* route = bindingRoute(binding);
            if (!route || route->empty()) {
                continue;
            }
            if (routeUsesNodeOnTile(*route, tile, node_type, node, transit_only)) {
                result.push_back(NetRouteRef{net, binding_index});
            }
        }
    }
    return result;
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

bool fpga::unrouteLastRouteStep(rtl::Net& net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size()) {
        return false;
    }
    std::vector<Wire>* route = bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty()) {
        return false;
    }

    size_t step_index = route->size();
    while (step_index > 0 && (*route)[step_index - 1].type == Wire::WIRE_TILE_PIN) {
        --step_index;
    }
    if (step_index == 0 || (*route)[step_index - 1].shared) {
        return false;
    }
    --step_index;
    for (size_t fragment_index = route->size(); fragment_index > step_index; --fragment_index) {
        releaseRouteFragmentLease(*route, fragment_index - 1);
    }
    route->resize(step_index);
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

// Release a moved sink endpoint while retaining its shared trunk or source takeoff.
bool fpga::detachNetRouteDestination(rtl::Net& net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size()) {
        return false;
    }
    rtl::NetRouteBinding& binding = net.routes[route_binding_index];
    std::vector<Wire>* route = bindingRoute(binding);
    if (!route || route->empty()) {
        return false;
    }

    size_t keep = 0;
    while (keep < route->size() && (*route)[keep].shared) {
        ++keep;
    }
    if (keep == route->size() && keep > 1 && (*route)[keep - 1].type == Wire::WIRE_TILE_PIN) {
        --keep;
    }
    if (keep == 0) {
        for (size_t fragment_index = 0; fragment_index < route->size(); ++fragment_index) {
            const Wire& fragment = (*route)[fragment_index];
            if (fragment.type == Wire::WIRE_CROSSBAR && fragment.jump >= 0) {
                keep = fragment_index + 1;
                break;
            }
        }
    }

    // The retained prefix owns a landing reused by the first removed terminal fragment.
    if (keep > 0 && keep < route->size()
        && (*route)[keep - 1].type == Wire::WIRE_CROSSBAR
        && (*route)[keep].type == Wire::WIRE_CROSSBAR
        && (*route)[keep - 1].dst == (*route)[keep].local) {
        (*route)[keep].owns_dst = false;
    }

    for (size_t fragment_index = route->size(); fragment_index > keep; --fragment_index) {
        const size_t removed_index = fragment_index - 1;
        if ((*route)[removed_index].shared) {
            continue;
        }
        if (removed_index == keep && keep > 0
            && (*route)[keep - 1].type == Wire::WIRE_CROSSBAR
            && (*route)[removed_index].type == Wire::WIRE_CROSSBAR
            && (*route)[keep - 1].dst == (*route)[removed_index].local) {
            Wire boundary = (*route)[removed_index];
            boundary.owns_dst = false;
            releaseRouteFragmentLease(std::vector<Wire>{boundary}, 0);
        }
        else {
            releaseRouteFragmentLease(*route, removed_index);
        }
    }
    route->resize(keep);
    rebuildNetRouteTiles(net);
    return true;
}

// A moved sink always needs routing again; preserve only the reusable prefix
// when its binding currently owns physical route fragments.
bool fpga::invalidateMovedSinkRoute(rtl::Net& net, size_t route_binding_index)
{
    if (route_binding_index >= net.routes.size()) {
        return false;
    }
    std::vector<Wire>* route = bindingRoute(net.routes[route_binding_index]);
    if (route && !route->empty()) {
        detachNetRouteDestination(net, route_binding_index);
    }
    return true;
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
