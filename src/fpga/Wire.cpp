#include "Wire.h"
#include "Device.h"
#include "Tile.h"

#include <algorithm>
#include <unordered_set>
#include <map>
#include <set>
#include <span>
#include <ostream>
#include <iomanip>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <stdexcept>

using namespace fpga;

void Wire::assign(rtl::Net *net) {
  PNR_ASSERT(net->wire.peer == nullptr,
             "assigning wire {}:{}-{}:{} to already assigned net {}", from.x,
             from.y, to.x, to.y, net->makeName());
  net->wire.set(static_cast<Referable<Wire> *>(this));
}

namespace {

bool sameCoord(const Coord &a, const Coord &b) {
  return a.x == b.x && a.y == b.y;
}

// Compare only routed resource identity; ownership and annotation fields may
// differ.
bool samePhysicalFragment(const Wire &left, const Wire &right) {
  return left.type == right.type && sameCoord(left.from, right.from) &&
         sameCoord(left.to, right.to) && left.local == right.local &&
         left.pos == right.pos && left.jump == right.jump &&
         left.route_jump == right.route_jump && left.dst == right.dst &&
         left.joint == right.joint && left.joint2 == right.joint2 &&
         left.from_node_type == right.from_node_type &&
         left.from_node == right.from_node &&
         left.to_node_type == right.to_node_type &&
         left.to_node == right.to_node;
}

std::vector<Wire> *bindingRoute(rtl::NetRouteBinding &binding);

// Return the prefix that is physically shared with another live binding from
// the same source port; stale Wire::shared flags alone do not prove ownership.
size_t liveSharedPrefixLength(
    rtl::Net &net, size_t binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded =
        nullptr) {
  if (binding_index >= net.routes.size()) {
    return 0;
  }
  rtl::NetRouteBinding &binding = net.routes[binding_index];
  std::vector<Wire> *route = bindingRoute(binding);
  if (!route || route->empty() || !binding.from ||
      binding.from_port.empty()) {
    return 0;
  }

  size_t longest = 0;
  std::unordered_set<rtl::Net *> visited;
  auto inspect_net = [&](rtl::Net &candidate_net) {
    if (!visited.insert(&candidate_net).second) {
      return;
    }
    for (size_t candidate_index = 0;
         candidate_index < candidate_net.routes.size(); ++candidate_index) {
      if (&candidate_net == &net && candidate_index == binding_index) {
        continue;
      }
      rtl::NetRouteBinding &candidate_binding =
          candidate_net.routes[candidate_index];
      if (excluded && excluded->contains(&candidate_binding)) {
        continue;
      }
      if (candidate_binding.from != binding.from ||
          candidate_binding.from_port != binding.from_port) {
        continue;
      }
      std::vector<Wire> *candidate = bindingRoute(candidate_binding);
      if (!candidate || candidate->empty()) {
        continue;
      }
      size_t common = 0;
      while (common < route->size() && common < candidate->size() &&
             (*route)[common].shared &&
             samePhysicalFragment((*route)[common], (*candidate)[common])) {
        ++common;
      }
      longest = std::max(longest, common);
    }
  };

  inspect_net(net);
  Tile *source_tile = Device::current().getTile(route->front().from.x,
                                                route->front().from.y);
  if (source_tile) {
    for (Ref<rtl::Net> &ref : source_tile->routedNets) {
      if (ref.peer) {
        inspect_net(*ref.peer);
      }
    }
  }
  return longest;
}

// Transfer a removed route's owned prefix to surviving branches before
// releasing leases; a fanout can itself be the parent of other fanouts.
void promoteSurvivingSourcePrefix(
    rtl::Net &net, size_t removed_binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded =
        nullptr) {
  RouteHistoryScope history(&net, __func__);
  if (removed_binding_index >= net.routes.size()) {
    return;
  }
  rtl::NetRouteBinding &removed_binding = net.routes[removed_binding_index];
  std::vector<Wire> *removed = bindingRoute(removed_binding);
  if (!removed || removed->empty() || !removed_binding.from ||
      removed_binding.from_port.empty()) {
    return;
  }

  auto promote_from_net = [&](rtl::Net &candidate_net) {
    size_t transferred_count = 0;
    for (size_t sibling_index = 0; sibling_index < candidate_net.routes.size();
         ++sibling_index) {
      if (&candidate_net == &net && sibling_index == removed_binding_index) {
        continue;
      }
      rtl::NetRouteBinding &sibling_binding =
          candidate_net.routes[sibling_index];
      if (excluded && excluded->contains(&sibling_binding)) {
        continue;
      }
      if (sibling_binding.from != removed_binding.from ||
          sibling_binding.from_port != removed_binding.from_port) {
        continue;
      }
      std::vector<Wire> *sibling = bindingRoute(sibling_binding);
      if (!sibling || sibling->empty() || !sibling->front().shared) {
        continue;
      }

      size_t common = 0;
      while (common < removed->size() && common < sibling->size() &&
             (*sibling)[common].shared &&
             samePhysicalFragment((*removed)[common], (*sibling)[common])) {
        ++common;
      }
      if (common == 0) {
        continue;
      }

      for (size_t index = 0; index < common; ++index) {
        if ((*removed)[index].owns_landing) {
          (*sibling)[index].owns_landing = true;
          (*removed)[index].owns_landing = false;
          ++transferred_count;
        }
        if ((*removed)[index].shared) {
          continue;
        }
        (*sibling)[index].shared = false;
        (*sibling)[index].owns_dst = (*removed)[index].owns_dst;
        (*removed)[index].shared = true;
        ++transferred_count;
      }
    }
    return transferred_count;
  };

  promote_from_net(net);

  // One physical source port may be represented by several RTL nets. Every
  // surviving sibling sharing this prefix is registered on its first prefix
  // tile.
  std::unordered_set<rtl::Net *> visited{&net};
  Tile *source_tile = Device::current().getTile(removed->front().from.x,
                                                removed->front().from.y);
  if (source_tile) {
    for (Ref<rtl::Net> &ref : source_tile->routedNets) {
      rtl::Net *candidate = ref.peer;
      if (!candidate || !visited.insert(candidate).second) {
        continue;
      }
      promote_from_net(*candidate);
    }
  }
}

bool routeUsesNodeOnTile(std::span<const Wire> route, const Tile &tile,
                         CBNodeNameType node_type, int node, bool transit_only,
                         bool owned_only) {
  for (const Wire &fragment : route) {
    if (owned_only && fragment.shared &&
        !(fragment.type == Wire::WIRE_CROSSBAR && node_type == CB_NODE_DST &&
          fragment.owns_landing && sameCoord(fragment.to, tile.coord) &&
          fragment.dst == node)) {
      continue;
    }
    bool from_tile = sameCoord(fragment.from, tile.coord);
    bool to_tile = sameCoord(fragment.to, tile.coord);
    if (!from_tile && !to_tile) {
      continue;
    }

    if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
      if (from_tile && fragment.from_node_type == node_type &&
          fragment.from_node == node) {
        return true;
      }
      if (to_tile && fragment.to_node_type == node_type &&
          fragment.to_node == node) {
        return true;
      }
      continue;
    }

    if (node_type == CB_NODE_SRC) {
      if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
          fragment.jump == node) {
        if (!transit_only ||
            (fragment.pos != 0 && !sameCoord(fragment.from, fragment.to))) {
          return true;
        }
      }
      continue;
    }

    if (node_type == CB_NODE_DST) {
      // A fragment leases only its source-tile incoming node. Its landing
      // is normally owned by the following fragment, except when a partial
      // route explicitly reserves that landing for its next continuation.
      if (to_tile && fragment.type == Wire::WIRE_CROSSBAR &&
          fragment.owns_landing && fragment.dst == node &&
          (!transit_only || !sameCoord(fragment.from, fragment.to))) {
        return true;
      }
      if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
          fragment.pos != 0 && fragment.owns_dst && fragment.local == node &&
          (!transit_only || !sameCoord(fragment.from, fragment.to))) {
        return true;
      }
      continue;
    }

    if (node_type == CB_NODE_LOCAL) {
      if (transit_only) {
        continue;
      }
      if (fragment.type == Wire::WIRE_TILE_PIN && from_tile &&
          fragment.local == node) {
        return true;
      }
      if (fragment.type == Wire::WIRE_CROSSBAR && from_tile &&
          fragment.pos == 0 && fragment.local == node) {
        return true;
      }
      continue;
    }

    if (node_type == CB_NODE_JOINT) {
      if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
          (fragment.joint == node || fragment.joint2 == node)) {
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

std::vector<Wire> *bindingRoute(rtl::NetRouteBinding &binding) {
  if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
    return nullptr;
  }
  return &binding.owner->wires[binding.route_index];
}

void clearTileNetRef(Tile &tile, rtl::Net &net) {
  tile.removeRoutedBindings(&net);
}

void appendRouteTiles(const std::vector<Wire> &route,
                      std::unordered_set<Tile *> &tiles) {
  for (const Wire &fragment : route) {
    if (Tile *tile =
            Device::current().getTile(fragment.from.x, fragment.from.y)) {
      tiles.insert(tile);
    }
    if (Tile *tile = Device::current().getTile(fragment.to.x, fragment.to.y)) {
      tiles.insert(tile);
    }
  }
}

// Rebuild only tile registrations touched by removed or retargeted routes.
// A local route edit must not scan the complete device grid.
void rebuildNetRouteTiles(rtl::Net &net,
                          const std::vector<std::vector<Wire>> &affected) {
  std::unordered_set<Tile *> affected_tiles;
  for (const std::vector<Wire> &route : affected) {
    appendRouteTiles(route, affected_tiles);
  }
  for (Tile *tile : affected_tiles) {
    clearTileNetRef(*tile, net);
  }
  for (size_t binding_index = 0; binding_index < net.routes.size();
       ++binding_index) {
    rtl::NetRouteBinding &remaining = net.routes[binding_index];
    std::vector<Wire> *remaining_route = bindingRoute(remaining);
    if (remaining_route && !remaining_route->empty()) {
      registerNetRouteTiles(net, *remaining_route, binding_index);
    }
  }
}

// Resolve a placed instance port to a stable net endpoint reference.
Referable<rtl::Port> *findInstPort(rtl::Inst *inst,
                                   const std::string &port_name) {
  if (!inst || port_name.empty()) {
    return nullptr;
  }
  for (auto &conn : inst->conns) {
    if (conn.port_ref.peer && conn.port_ref->name == port_name) {
      return conn.port_ref.peer;
    }
  }
  return nullptr;
}

void addTileNetRef(Tile &tile, rtl::Net &net) {
  tile.addRoutedNet(&net);
}

void clearRouteLeases(const std::vector<const std::vector<Wire> *> &routes,
                      bool clear_shared = false) {
  if (clear_shared) {
    // Shared metadata may be stale after route-tree edits. Release through
    // the live-owner index so sibling-owned nodes remain leased.
    for (const std::vector<Wire> *route : routes) {
      if (!route) {
        continue;
      }
      for (size_t fragment_index = 0; fragment_index < route->size();
           ++fragment_index) {
        releaseRouteFragmentLease(*route, fragment_index);
      }
    }
    return;
  }
  for (const std::vector<Wire> *route : routes) {
    if (!route) {
      continue;
    }
    for (size_t i = 0; i < route->size(); ++i) {
      const Wire &fragment = (*route)[i];
      if (fragment.type == Wire::WIRE_CROSSBAR && fragment.owns_landing &&
          fragment.dst >= 0) {
        Tile *landing_tile =
            Device::current().getTile(fragment.to.x, fragment.to.y);
        if (landing_tile) {
          landing_tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.dst);
        }
      }
      if (fragment.shared) {
        continue;
      }
      if (fragment.type == Wire::WIRE_TILE_PIN) {
        Tile *route_tile =
            Device::current().getTile(fragment.from.x, fragment.from.y);
        if (!route_tile || fragment.local < 0) {
          continue;
        }
        NodeMask keep = ~(NodeMask{0, 1} << fragment.local);
        route_tile->pin_state.leased_nodes &= keep;
        route_tile->cb.local.local &= keep;
        // A resource endpoint may attach to a distinct route tile. Full-tree
        // teardown releases both sides so its dedicated router can rebuild it.
        Tile *resource_tile = Device::current().getTile(
            fragment.resource.x, fragment.resource.y);
        if (resource_tile) {
          resource_tile->pin_state.leased_nodes &= keep;
        }
        continue;
      }

      if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
        Tile *tile = Device::current().getTile(fragment.to.x, fragment.to.y);
        if (!tile || fragment.to_node < 0) {
          continue;
        }
        NodeMask clear = ~(NodeMask{0, 1} << fragment.to_node);
        switch (fragment.to_node_type) {
        case CB_NODE_LOCAL: tile->cb.local.local &= clear; break;
        case CB_NODE_JOINT: tile->cb.joint.jump &= clear; break;
        case CB_NODE_SRC: tile->cb.src.jump &= clear; break;
        case CB_NODE_DST: tile->cb.dst.jump &= clear; break;
        default: break;
        }
        continue;
      }

      if (fragment.type != Wire::WIRE_CROSSBAR) {
        continue;
      }
      Tile *tile = Device::current().getTile(fragment.from.x, fragment.from.y);
      if (!tile) {
        continue;
      }
      if (fragment.jump >= 0) {
        tile->cb.src.jump &= ~(NodeMask{0, 1} << fragment.jump);
      }
      if (fragment.joint >= 0) {
        tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint);
      }
      if (fragment.joint2 >= 0) {
        tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint2);
      }
      if (fragment.pos == 0 && fragment.local >= 0) {
        tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
      }
      if (fragment.pos != 0 && fragment.local >= 0 && fragment.owns_dst) {
        tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.local);
      }
    }
  }
}

void clearRouteLeases(const std::vector<Wire> &route,
                      bool clear_shared = false) {
  clearRouteLeases(std::vector<const std::vector<Wire> *>{&route},
                   clear_shared);
}

} // namespace

void fpga::releaseRouteFragmentLease(const std::vector<Wire> &route,
                                     size_t fragment_index) {
  if (fragment_index >= route.size()) {
    return;
  }
  const Wire &fragment = route[fragment_index];
  if (fragment.type == Wire::WIRE_CROSSBAR && fragment.owns_landing &&
      fragment.dst >= 0) {
    Tile *landing_tile =
        Device::current().getTile(fragment.to.x, fragment.to.y);
    if (landing_tile &&
        findNetOwnersByNode(*landing_tile, CB_NODE_DST, fragment.dst).empty()) {
      landing_tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.dst);
    }
  }
  if (fragment.type == Wire::WIRE_TILE_PIN) {
    if (fragment_index + 1 != route.size()) {
      return;
    }
    Tile *tile = Device::current().getTile(fragment.from.x, fragment.from.y);
    if (!tile || fragment.local < 0) {
      return;
    }
    if (findNetOwnersByNode(*tile, CB_NODE_LOCAL, fragment.local).empty()) {
      tile->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
      tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
    }
    if (fragment_index > 0 &&
        route[fragment_index - 1].type == Wire::WIRE_CROSSBAR &&
        route[fragment_index - 1].local >= 0 &&
        route[fragment_index - 1].owns_dst &&
        findNetOwnersByNode(*tile, CB_NODE_DST, route[fragment_index - 1].local)
            .empty()) {
      tile->cb.dst.jump &= ~(NodeMask{0, 1} << route[fragment_index - 1].local);
    }
    return;
  }

  if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
    Tile *tile = Device::current().getTile(fragment.to.x, fragment.to.y);
    if (!tile || fragment.to_node < 0 ||
        !findNetOwnersByNode(*tile,
            static_cast<CBNodeNameType>(fragment.to_node_type), fragment.to_node).empty()) {
      return;
    }
    NodeMask clear = ~(NodeMask{0, 1} << fragment.to_node);
    switch (fragment.to_node_type) {
    case CB_NODE_LOCAL: tile->cb.local.local &= clear; break;
    case CB_NODE_JOINT: tile->cb.joint.jump &= clear; break;
    case CB_NODE_SRC: tile->cb.src.jump &= clear; break;
    case CB_NODE_DST: tile->cb.dst.jump &= clear; break;
    default: break;
    }
    return;
  }

  if (fragment.type != Wire::WIRE_CROSSBAR) {
    return;
  }
  Tile *tile = Device::current().getTile(fragment.from.x, fragment.from.y);
  if (!tile) {
    return;
  }
  if (fragment.jump >= 0) {
    if (findNetOwnersByNode(*tile, CB_NODE_SRC, fragment.jump).empty()) {
      tile->cb.src.jump &= ~(NodeMask{0, 1} << fragment.jump);
    }
  }
  if (fragment.joint >= 0) {
    if (findNetOwnersByNode(*tile, CB_NODE_JOINT, fragment.joint).empty()) {
      tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint);
    }
  }
  if (fragment.joint2 >= 0) {
    if (findNetOwnersByNode(*tile, CB_NODE_JOINT, fragment.joint2).empty()) {
      tile->cb.joint.jump &= ~(NodeMask{0, 1} << fragment.joint2);
    }
  }
  if (fragment.pos == 0 && fragment.local >= 0) {
    if (findNetOwnersByNode(*tile, CB_NODE_LOCAL, fragment.local).empty()) {
      tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
    }
  }
  if (fragment.pos != 0 && fragment.local >= 0 && fragment.owns_dst &&
      findNetOwnersByNode(*tile, CB_NODE_DST, fragment.local).empty()) {
    tile->cb.dst.jump &= ~(NodeMask{0, 1} << fragment.local);
  }
}

void fpga::releaseRouteLeases(const std::vector<Wire> &route) {
  clearRouteLeases(route, true);
}

size_t fpga::attachNetRoute(rtl::Net &net, rtl::Inst &owner,
                            size_t route_index, rtl::Inst *from,
                            rtl::Inst *to, const std::string &from_port,
                            const std::string &to_port,
                            const std::string &route_name) {
  RouteHistoryScope history(&net, __func__);
  if (!net.src_port.peer) {
    if (Referable<rtl::Port> *port = findInstPort(from, from_port)) {
      net.src_port.set(port);
    }
  }
  if (!net.dst_port.peer) {
    if (Referable<rtl::Port> *port = findInstPort(to, to_port)) {
      net.dst_port.set(port);
    }
  }

  // Resolve the complete endpoint identity through the net's stable index.
  rtl::NetRouteLookup lookup =
      net.findRouteBinding(from, to, from_port, to_port, route_name);
  if (lookup.index != std::numeric_limits<size_t>::max()) {
    rtl::NetRouteBinding &binding = net.routes[lookup.index];
    // A repeated attachment to the same storage requires no update.
    if (binding.owner == &owner && binding.route_index == route_index) {
      return lookup.index;
    }
    // Preserve a completed physical owner; otherwise transfer the binding.
    std::vector<Wire> *route = bindingRoute(binding);
    if (route && isRouteComplete(*route)) {
      return lookup.index;
    }
    binding.owner = &owner;
    binding.route_index = route_index;
    return lookup.index;
  }
  // No exact endpoint identity exists; create one independent physical branch.
  net.appendRouteBinding(rtl::NetRouteBinding{
      &owner, route_index, from, to, from_port, to_port, route_name});
  return net.routes.size() - 1;
}

size_t fpga::retargetNetRouteBindings(rtl::Net &old_net, rtl::Net &new_net,
                                      rtl::Inst *old_from, rtl::Inst *old_to,
                                      const std::string &old_from_port,
                                      const std::string &old_to_port,
                                      rtl::Inst *new_from, rtl::Inst *new_to,
                                      const std::string &new_from_port,
                                      const std::string &new_to_port,
                                      const std::string &route_name) {
  size_t changed = 0;
  std::vector<std::vector<Wire>> affected_routes;
  for (size_t index = 0; index < old_net.routes.size();) {
    rtl::NetRouteBinding &binding = old_net.routes[index];
    if (binding.from != old_from || binding.to != old_to ||
        binding.from_port != old_from_port || binding.to_port != old_to_port ||
        binding.route_name != route_name) {
      ++index;
      continue;
    }
    if (std::vector<Wire> *route = bindingRoute(binding);
        route && !route->empty()) {
      affected_routes.push_back(*route);
    }
    rtl::NetRouteBinding replacement = binding;
    replacement.from = new_from;
    replacement.to = new_to;
    replacement.from_port = new_from_port;
    replacement.to_port = new_to_port;
    if (&old_net == &new_net) {
      binding = std::move(replacement);
      ++index;
    } else {
      old_net.eraseRouteBinding(index);
      replacement.route_id = 0;
      new_net.appendRouteBinding(std::move(replacement));
    }
    ++changed;
  }
  // Retargeting changes the Net found through each routed tile, including an
  // existing partial prefix.
  if (changed != 0) {
    old_net.invalidateRouteLookup();
    new_net.invalidateRouteLookup();
    rebuildNetRouteTiles(old_net, affected_routes);
    if (&new_net != &old_net) {
      rebuildNetRouteTiles(new_net, affected_routes);
    }
  }
  return changed;
}

// Keep every branch of one physical source tree on the same endpoint identity.
size_t fpga::retargetNetRouteSourceBindings(rtl::Net &net, rtl::Inst *old_from,
                                            const std::string &old_from_port,
                                            rtl::Inst *new_from,
                                            const std::string &new_from_port) {
  size_t changed = 0;
  for (rtl::NetRouteBinding &binding : net.routes) {
    if (binding.from != old_from || binding.from_port != old_from_port) {
      continue;
    }
    binding.from = new_from;
    binding.from_port = new_from_port;
    ++changed;
  }
  if (changed != 0) {
    net.invalidateRouteLookup();
  }
  return changed;
}

bool fpga::isRouteComplete(const std::vector<Wire> &route) {
  if (route.empty() || route.back().type != Wire::WIRE_TILE_PIN) {
    return false;
  }
  bool has_crossbar = false;
  size_t tile_pin_count = 0;
  Coord endpoint_coord{-1, -1};
  bool has_endpoint = false;
  for (const Wire &fragment : route) {
    if (fragment.type == Wire::WIRE_CROSSBAR || fragment.type == Wire::WIRE_ROUTE_EDGE) {
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
    } else if (endpoint_coord.x != coord.x || endpoint_coord.y != coord.y) {
      return has_crossbar;
    }
  }
  return has_crossbar || tile_pin_count >= 2;
}

void fpga::registerNetRouteTiles(rtl::Net &net,
                                 const std::vector<Wire> &route,
                                 size_t binding_index) {
  registerNetRouteTilesFrom(net, route, 0, binding_index);
}

void fpga::registerNetRouteTilesFrom(rtl::Net &net,
                                     const std::vector<Wire> &route,
                                     size_t first_fragment,
                                     size_t binding_index) {
  RouteHistoryScope history(&net, __func__);
  if (binding_index == std::numeric_limits<size_t>::max()) {
    for (size_t index = 0; index < net.routes.size(); ++index) {
      if (bindingRoute(net.routes[index]) == &route) {
        binding_index = index;
        break;
      }
    }
  }
  uint64_t route_id = net.routeId(binding_index);
  for (size_t index = first_fragment; index < route.size(); ++index) {
    const Wire &fragment = route[index];
    Tile *from_tile =
        Device::current().getTile(fragment.from.x, fragment.from.y);
    if (from_tile) {
      addTileNetRef(*from_tile, net);
      from_tile->addRoutedBinding(&net, route_id);
    }
    Tile *to_tile = Device::current().getTile(fragment.to.x, fragment.to.y);
    if (to_tile) {
      addTileNetRef(*to_tile, net);
      to_tile->addRoutedBinding(&net, route_id);
    }
  }
}

rtl::Net *fpga::findNetByNode(Tile &tile, CBNodeNameType node_type, int node,
                              bool transit_only) {
  std::vector<NetRouteRef> routes =
      findNetRoutesByNode(tile, node_type, node, transit_only);
  return routes.empty() ? nullptr : routes.front().net;
}

NodeMask fpga::congestionNodeMask(const CBState& state, CBNodeNameType type) {
  switch (type) {
  case CB_NODE_SRC: return state.src.jump;
  case CB_NODE_DST: return state.dst.jump;
  case CB_NODE_JOINT: return state.joint.jump;
  case CB_NODE_LOCAL: return state.local.local;
  default: return {};
  }
}

CongestionAudit fpga::auditTileCongestion(
    Tile& tile, std::ostream& out, const std::vector<rtl::Net*>& design_nets) {
  CongestionAudit audit;
  auto inst_name = [](rtl::Inst* inst) {
    return inst && inst->cell_ref.peer ? inst->makeName(1000000) : std::string("<untyped-or-null>");
  };
  out << "TILE coord=(" << tile.coord.x << ',' << tile.coord.y << ") name="
      << std::quoted(tile.full_name) << " cb="
      << std::quoted(tile.cb_type ? tile.cb_type->name : "")
      << " authoritative=" << tile.routed_bindings_authoritative << '\n';
  auto mask = [&](const char* name, NodeMask bits) {
    out << "MASK " << name << "=[";
    bits.for_each_set_bit([&](int node) { out << node << ','; return false; });
    out << "]\n";
  };
  mask("SRC", tile.cb.src.jump);
  mask("DST", tile.cb.dst.jump);
  mask("JOINT", tile.cb.joint.jump);
  mask("LOCAL", tile.cb.local.local);
  mask("PIN", tile.pin_state.leased_nodes);
  mask("DEADEND", tile.cb.src_deadend.jump);
  mask("INCOMING", tile.incoming_dst_nodes);
  for (const auto& [node, conn] : tile.input_local_reservations) {
    out << "PACKED_LOCAL node=" << node << " driver="
        << std::quoted(inst_name(conn ? conn->inst_ref.peer : nullptr))
        << " port=" << std::quoted(conn && conn->port_ref.peer ? conn->port_ref->makeName() : "") << '\n';
  }
  for (const auto& [conn, bits] : tile.input_joint_reservations) {
    out << "PACKED_JOINT driver="
        << std::quoted(inst_name(conn ? conn->inst_ref.peer : nullptr)) << '\n';
    mask("reserved_joints", bits);
  }
  struct Claim { rtl::Net* net; size_t binding; size_t fragment; bool owner; bool requires_lease; };
  std::map<std::pair<int, int>, std::vector<Claim>> claims;
  std::unordered_set<rtl::Net*> nets(design_nets.begin(), design_nets.end());
  // The design scope is the authority for object existence. Do not dereference
  // an indexed pointer that has disappeared from that scope.
  for (const auto& ref : tile.routedNets) {
    if (ref.peer && !nets.contains(ref.peer)) {
      ++audit.stale_registrations;
      out << "STALE_NET_REF net_exists=0\n";
    }
  }
  for (rtl::Net* net : nets) {
    if (!net) continue;
    for (size_t b = 0; b < net->routes.size(); ++b) {
      auto& binding = net->routes[b];
      auto* route = bindingRoute(binding);
      if (!route) continue;
      bool touches = false;
      for (size_t f = 0; f < route->size(); ++f) {
        const Wire& wire = (*route)[f];
        if (!sameCoord(wire.from, tile.coord) && !sameCoord(wire.to, tile.coord)) continue;
        touches = true;
        std::set<std::pair<int, int>> nodes{
            {CB_NODE_SRC, wire.jump}, {CB_NODE_DST, wire.local},
            {CB_NODE_DST, wire.dst}, {CB_NODE_LOCAL, wire.local},
            {CB_NODE_JOINT, wire.joint}, {CB_NODE_JOINT, wire.joint2},
            {wire.from_node_type, wire.from_node}, {wire.to_node_type, wire.to_node}};
        for (auto [kind, node] : nodes) {
          if (node < 0 || node >= CB_MAX_NODES) continue;
          auto type = static_cast<CBNodeNameType>(kind);
          if (!routeUsesNodeOnTile({&wire, 1}, tile, type, node, false, false)) continue;
          // Recheck the owning fragment flags directly, not the owner lookup.
          // A shared hop can own its landing, never its source-side DST.
          bool owns = !wire.shared ||
              (wire.type == Wire::WIRE_CROSSBAR && type == CB_NODE_DST &&
               wire.owns_landing && sameCoord(wire.to, tile.coord) && wire.dst == node);
          // A directed graph edge leases only its destination. Its source
          // is owned by a preceding edge or is an unleased resource root.
          if (wire.type == Wire::WIRE_ROUTE_EDGE)
            owns = !wire.shared && sameCoord(wire.to, tile.coord) &&
                   wire.to_node_type == kind && wire.to_node == node;
          // Numeric takeoff deliberately does not lease its LOCAL source:
          // several branches may leave the same physical output. Dedicated
          // routers can lease it, but its absence alone is not an error.
          bool source_reference = type == CB_NODE_LOCAL &&
              ((wire.type == Wire::WIRE_CROSSBAR && wire.pos == 0) ||
               (wire.type == Wire::WIRE_TILE_PIN && wire.pin_dir == TILE_PIN_OUTPUT));
          claims[{kind, node}].push_back({net, b, f, owns, !source_reference});
        }
      }
      if (touches) {
        bool registered = tile.routed_bindings_authoritative
            ? std::any_of(tile.routed_bindings.begin(), tile.routed_bindings.end(),
                [&](const auto& ref) { return ref.net == net && net->findRouteBindingById(ref.route_id) == b; })
            : std::any_of(tile.routedNets.begin(), tile.routedNets.end(),
                [&](const auto& ref) { return ref.peer == net; });
        if (!registered) {
          ++audit.missing_registrations;
          out << "MISSING_REGISTRATION net=" << std::quoted(net->makeName(1000000))
              << " binding=" << b << " route=" << std::quoted(binding.route_name) << '\n';
        }
      }
    }
  }
  for (const auto& ref : tile.routed_bindings) {
    if (!ref.net || !nets.contains(ref.net)) {
      ++audit.stale_registrations;
      out << "INDEX route_id=" << ref.route_id << " net_exists=0 valid=0\n";
      continue;
    }
    size_t b = ref.net ? ref.net->findRouteBindingById(ref.route_id) : SIZE_MAX;
    auto* route = ref.net && b < ref.net->routes.size() ? bindingRoute(ref.net->routes[b]) : nullptr;
    bool valid = route && std::any_of(route->begin(), route->end(), [&](const Wire& w) {
      return sameCoord(w.from, tile.coord) || sameCoord(w.to, tile.coord);
    });
    out << "INDEX net=" << std::quoted(ref.net ? ref.net->makeName(1000000) : "")
        << " route_id=" << ref.route_id << " binding=" << b << " net_exists=1 valid=" << valid << '\n';
    if (!valid) ++audit.stale_registrations;
  }
  for (auto type : {CB_NODE_SRC, CB_NODE_DST, CB_NODE_JOINT, CB_NODE_LOCAL}) {
    NodeMask leased = congestionNodeMask(tile.cb, type);
    if (type == CB_NODE_LOCAL) leased |= tile.pin_state.leased_nodes;
    leased.for_each_set_bit([&](int n) { claims[{type, n}]; return false; });
  }
  for (const auto& [key, users] : claims) {
    auto [kind, node] = key;
    auto type = static_cast<CBNodeNameType>(kind);
    bool leased = congestionNodeMask(tile.cb, type).testBit(node) ||
                  (type == CB_NODE_LOCAL && tile.pin_state.leased_nodes.testBit(node));
    size_t owners = std::count_if(users.begin(), users.end(), [](const Claim& c) { return c.owner; });
    bool requires_lease = std::any_of(users.begin(), users.end(),
        [](const Claim& c) { return c.owner && c.requires_lease; });
    const char* label = type == CB_NODE_SRC ? "SRC" : type == CB_NODE_DST ? "DST" :
                        type == CB_NODE_JOINT ? "JOINT" : "LOCAL";
    const std::string* name = tile.cb_type ? tile.cb_type->nodeName(type, node) : nullptr;
    const char* status = leased ? (owners ? "VERIFIED" : "UNOWNED_LEASE")
                               : (requires_lease ? "MISSING_LEASE" : owners ? "SOURCE_REFERENCE" : "NONOWNING_REFERENCE");
    audit.leased_nodes += leased;
    audit.orphan_leases += leased && !owners;
    audit.missing_leases += !leased && requires_lease;
    out << "NODE type=" << label << " id=" << node << " name=" << std::quoted(name ? *name : "")
        << " leased=" << leased << " claims=" << users.size() << " owners=" << owners
        << " status=" << status << '\n';
    for (const Claim& claim : users) {
      const auto& binding = claim.net->routes[claim.binding];
      const Wire& w = binding.owner->wires[binding.route_index][claim.fragment];
      out << "PROOF net=" << std::quoted(claim.net->makeName(1000000))
          << " route=" << std::quoted(binding.route_name) << " binding=" << claim.binding
          << " storage=" << binding.route_index << " fragment=" << claim.fragment
          << " owns=" << claim.owner << " shared=" << w.shared << " owns_dst=" << w.owns_dst
          << " owns_landing=" << w.owns_landing << " pos=" << w.pos
          << " type=" << w.type << " pin_dir=" << w.pin_dir
          << " from_node=" << w.from_node_type << ':' << w.from_node
          << " to_node=" << w.to_node_type << ':' << w.to_node
          << " from=(" << w.from.x << ',' << w.from.y << ") to=(" << w.to.x << ',' << w.to.y << ")"
          << " local=" << w.local << " src=" << w.jump << " dst=" << w.dst
          << " joint=" << w.joint << " joint2=" << w.joint2
          << " driver=" << std::quoted(inst_name(binding.from))
          << " sink=" << std::quoted(inst_name(binding.to))
          << " complete=" << isRouteComplete(binding.owner->wires[binding.route_index]) << '\n';
    }
  }
  out << "AUDIT leased=" << audit.leased_nodes << " unowned=" << audit.orphan_leases
      << " missing_lease=" << audit.missing_leases << " missing_registration=" << audit.missing_registrations
      << " stale_registration=" << audit.stale_registrations << '\n';
  return audit;
}

void fpga::dumpNetRouteHistory(rtl::Net& net, std::ostream& out) {
  auto name = [](rtl::Inst* inst) {
    return inst && inst->cell_ref.peer ? inst->makeName(1000000) : std::string("<untyped-or-null>");
  };
  auto leased = [](Coord coord, CBNodeNameType type, int node) {
    Tile* tile = Device::current().getTile(coord.x, coord.y);
    return tile && node >= 0 && node < CB_MAX_NODES &&
        (congestionNodeMask(tile->cb, type).testBit(node) ||
         (type == CB_NODE_LOCAL && tile->isPinNodeLeased(node)));
  };
  out << "TREE net=" << std::quoted(net.makeName(1000000)) << " bindings=" << net.routes.size() << '\n';
  for (size_t b = 0; b < net.routes.size(); ++b) {
    auto& binding = net.routes[b];
    auto* route = bindingRoute(binding);
    out << "BINDING index=" << b << " id=" << net.routeId(b)
        << " route=" << std::quoted(binding.route_name) << " storage=" << binding.route_index
        << " driver=" << std::quoted(name(binding.from)) << " from_port=" << std::quoted(binding.from_port)
        << " sink=" << std::quoted(name(binding.to)) << " to_port=" << std::quoted(binding.to_port)
        << " fragments=" << (route ? route->size() : 0)
        << " complete=" << (route && isRouteComplete(*route)) << '\n';
    if (!route) continue;
    for (size_t f = 0; f < route->size(); ++f) {
      const Wire& w = (*route)[f];
      out << "FRAGMENT index=" << f << " type=" << w.type
          << " from=(" << w.from.x << ',' << w.from.y << ") to=(" << w.to.x << ',' << w.to.y << ')'
          << " local=" << w.local << " src=" << w.jump << " dst=" << w.dst
          << " joint=" << w.joint << " joint2=" << w.joint2 << " pos=" << w.pos
          << " shared=" << w.shared << " owns_dst=" << w.owns_dst << " owns_landing=" << w.owns_landing
          << " live_local=" << leased(w.from, w.pos == 0 || w.type == Wire::WIRE_TILE_PIN ? CB_NODE_LOCAL : CB_NODE_DST, w.local)
          << " live_src=" << leased(w.from, CB_NODE_SRC, w.jump)
          << " live_landing=" << leased(w.to, CB_NODE_DST, w.dst)
          << " live_joint=" << leased(w.from, CB_NODE_JOINT, w.joint)
          << " live_joint2=" << leased(w.from, CB_NODE_JOINT, w.joint2)
          << " from_node=" << w.from_node_type << ':' << w.from_node
          << " to_node=" << w.to_node_type << ':' << w.to_node << " pin_dir=" << w.pin_dir << '\n';
    }
  }
}

namespace {
const std::string& routeHistoryNet() {
  static const std::string selected = [] {
    const char* value = std::getenv("SCALEPNR_ROUTE_HISTORY_NET");
    return value ? std::string(value) : std::string{};
  }();
  return selected;
}
thread_local std::string_view route_history_actor;
thread_local size_t route_history_sequence = 0;
std::ostream& routeHistoryOutput() {
  static std::ofstream out([] {
    const char* value = std::getenv("SCALEPNR_ROUTE_HISTORY_LOG");
    return value && *value ? value : "routing_node_history.log";
  }());
  if (!out) throw std::runtime_error("cannot write route ownership history");
  return out;
}
void traceHistoryOwnerLookup(Tile& tile, CBNodeNameType type, int node,
                             const std::vector<NetRouteRef>& result) {
  struct Watch { int x = -1, y = -1, node = -1; CBNodeNameType type = CB_NODE_DST; };
  static const Watch watch = [] {
    Watch w;
    const char* value = std::getenv("SCALEPNR_ROUTE_HISTORY_NODE");
    char kind[8] = {};
    if (!value || std::sscanf(value, "%d,%d,%7[^,],%d", &w.x, &w.y, kind, &w.node) != 4)
      return Watch{};
    std::string_view label(kind);
    if (label == "SRC") w.type = CB_NODE_SRC;
    else if (label == "DST") w.type = CB_NODE_DST;
    else if (label == "JOINT") w.type = CB_NODE_JOINT;
    else if (label == "LOCAL") w.type = CB_NODE_LOCAL;
    else return Watch{};
    return w;
  }();
  if (node != watch.node || type != watch.type || tile.coord.x != watch.x || tile.coord.y != watch.y)
    return;
  auto& out = routeHistoryOutput();
  out << "OWNER_LOOKUP tile=(" << watch.x << ',' << watch.y << ") kind=" << static_cast<int>(type)
      << " node=" << node << " leased=" << congestionNodeMask(tile.cb, type).testBit(node)
      << " returned=" << result.size() << " actor=" << std::quoted(std::string(route_history_actor)) << '\n';
  for (const auto& ref : result) {
    auto& binding = ref.net->routes[ref.binding_index];
    auto* route = bindingRoute(binding);
    out << "LOOKUP_OWNER net=" << std::quoted(ref.net->makeName(1000000))
        << " binding=" << ref.binding_index << " route=" << std::quoted(binding.route_name) << '\n';
    if (!route) continue;
    for (size_t f = 0; f < route->size(); ++f) {
      const Wire& w = (*route)[f];
      if (!routeUsesNodeOnTile({&w, 1}, tile, type, node, false, false)) continue;
      bool owns = !w.shared || (w.type == Wire::WIRE_CROSSBAR && type == CB_NODE_DST &&
          w.owns_landing && sameCoord(w.to, tile.coord) && w.dst == node);
      if (w.type == Wire::WIRE_ROUTE_EDGE)
        owns = !w.shared && sameCoord(w.to, tile.coord) && w.to_node_type == type && w.to_node == node;
      out << "LOOKUP_PROOF fragment=" << f << " verified_owner=" << owns
          << " from=(" << w.from.x << ',' << w.from.y << ") to=(" << w.to.x << ',' << w.to.y << ')'
          << " local=" << w.local << " dst=" << w.dst << " shared=" << w.shared
          << " owns_dst=" << w.owns_dst << " owns_landing=" << w.owns_landing << '\n';
    }
  }
  out.flush();
}
}

fpga::RouteHistoryScope::RouteHistoryScope(rtl::Net* net, const char* operation,
                                         std::string_view actor) : operation(operation) {
  if (routeHistoryNet().empty()) return;
  active = true;
  previous_actor = route_history_actor;
  if (!actor.empty()) route_history_actor = actor;
  if (!net || net->makeName(1000000) != routeHistoryNet()) return;
  selected = net;
  id = ++route_history_sequence;
  auto& out = routeHistoryOutput();
  out << "HISTORY_BEGIN id=" << id << " operation=" << operation
      << " actor=" << std::quoted(std::string(route_history_actor)) << '\n';
  dumpNetRouteHistory(*selected, out);
  out.flush();
}

fpga::RouteHistoryScope::~RouteHistoryScope() {
  if (selected) {
    auto& out = routeHistoryOutput();
    out << "HISTORY_AFTER id=" << id << " operation=" << operation
        << " actor=" << std::quoted(std::string(route_history_actor)) << '\n';
    dumpNetRouteHistory(*selected, out);
    out << "HISTORY_END id=" << id << '\n';
    out.flush();
  }
  if (active) route_history_actor = previous_actor;
}

std::vector<NetRouteRef> fpga::findNetRoutesByNode(Tile &tile,
                                                   CBNodeNameType node_type,
                                                   int node,
                                                   bool transit_only) {
  std::vector<NetRouteRef> result;
  if (tile.routed_bindings_authoritative) {
    for (const Tile::RoutedBinding &ref : tile.routed_bindings) {
      if (!ref.net) {
        continue;
      }
      size_t binding_index = ref.net->findRouteBindingById(ref.route_id);
      if (binding_index == std::numeric_limits<size_t>::max() ||
          binding_index >= ref.net->routes.size()) {
        continue;
      }
      rtl::NetRouteBinding &binding = ref.net->routes[binding_index];
      std::vector<Wire> *route = bindingRoute(binding);
      if (route && !route->empty() &&
          routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                              false)) {
        result.push_back(NetRouteRef{ref.net, binding_index});
      }
    }
    return result;
  }
  for (auto &ref : tile.routedNets) {
    rtl::Net *net = ref.peer;
    if (!net) {
      continue;
    }
    for (size_t binding_index = 0; binding_index < net->routes.size();
         ++binding_index) {
      rtl::NetRouteBinding &binding = net->routes[binding_index];
      std::vector<Wire> *route = bindingRoute(binding);
      if (!route || route->empty()) {
        continue;
      }
      if (routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                              false)) {
        result.push_back(NetRouteRef{net, binding_index});
      }
    }
  }
  return result;
}

std::vector<NetRouteRef> fpga::findNetOwnersByNode(Tile &tile,
                                                   CBNodeNameType node_type,
                                                   int node,
                                                   bool transit_only) {
  std::vector<NetRouteRef> result;
  if (tile.routed_bindings_authoritative) {
    for (const Tile::RoutedBinding &ref : tile.routed_bindings) {
      if (!ref.net) {
        continue;
      }
      size_t binding_index = ref.net->findRouteBindingById(ref.route_id);
      if (binding_index == std::numeric_limits<size_t>::max() ||
          binding_index >= ref.net->routes.size()) {
        continue;
      }
      rtl::NetRouteBinding &binding = ref.net->routes[binding_index];
      std::vector<Wire> *route = bindingRoute(binding);
      if (route && !route->empty() &&
          routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                              true)) {
        result.push_back(NetRouteRef{ref.net, binding_index});
      }
    }
    traceHistoryOwnerLookup(tile, node_type, node, result);
    return result;
  }
  for (auto &ref : tile.routedNets) {
    rtl::Net *net = ref.peer;
    if (!net) {
      continue;
    }
    for (size_t binding_index = 0; binding_index < net->routes.size();
         ++binding_index) {
      rtl::NetRouteBinding &binding = net->routes[binding_index];
      std::vector<Wire> *route = bindingRoute(binding);
      if (!route || route->empty()) {
        continue;
      }
      if (routeUsesNodeOnTile(*route, tile, node_type, node, transit_only,
                              true)) {
        result.push_back(NetRouteRef{net, binding_index});
      }
    }
  }
  traceHistoryOwnerLookup(tile, node_type, node, result);
  return result;
}

bool fpga::unrouteNetRoute(rtl::Net &net, size_t route_binding_index) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  rtl::NetRouteBinding &binding = net.routes[route_binding_index];
  std::vector<Wire> *route = bindingRoute(binding);
  if (!route || route->empty()) {
    return false;
  }
  // Preserve one owner whenever another branch of this source still uses the
  // prefix.
  promoteSurvivingSourcePrefix(net, route_binding_index);
  std::vector<Wire> removed = *route;
  route->clear();
  // A physical node may still be owned by another net, including protected
  // infrastructure. Release only nodes with no remaining live route owner.
  clearRouteLeases(removed, true);

  rebuildNetRouteTiles(net, {removed});
  return true;
}

bool fpga::unrouteNetRouteFromNode(rtl::Net &net, size_t route_binding_index,
                                   Coord tile_coord,
                                   CBNodeNameType node_type, int node) {
  return unrouteNetRouteFromNodes(
      net, route_binding_index, {RouteCutNode{tile_coord, node_type, node}});
}

bool fpga::truncateNetRoute(rtl::Net &net, size_t route_binding_index,
                            size_t keep_fragments) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (!route || keep_fragments == 0 || keep_fragments >= route->size()) {
    return false;
  }
  std::vector<Wire> removed(
      route->begin() + static_cast<std::ptrdiff_t>(keep_fragments),
      route->end());
  if ((*route)[keep_fragments - 1].type == Wire::WIRE_CROSSBAR &&
      removed.front().type == Wire::WIRE_CROSSBAR &&
      (*route)[keep_fragments - 1].dst == removed.front().local) {
    (*route)[keep_fragments - 1].owns_landing = true;
    removed.front().owns_dst = false;
  }
  route->resize(keep_fragments);
  clearRouteLeases(removed, true);
  rebuildNetRouteTiles(net, {removed});
  return true;
}

bool fpga::unrouteNetRouteFromNodes(
    rtl::Net &net, size_t route_binding_index,
    const std::vector<RouteCutNode> &nodes) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size() || nodes.empty()) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (!route || route->empty()) {
    return false;
  }

  auto fragment_uses_node = [&](const Wire &fragment,
                                const RouteCutNode &cut) {
    if (cut.node < 0) {
      return false;
    }
    bool from_tile = sameCoord(fragment.from, cut.tile);
    bool to_tile = sameCoord(fragment.to, cut.tile);
    if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
      return (from_tile && fragment.from_node_type == cut.type &&
              fragment.from_node == cut.node) ||
             (to_tile && fragment.to_node_type == cut.type &&
              fragment.to_node == cut.node);
    }
    if (fragment.type == Wire::WIRE_TILE_PIN) {
      return cut.type == CB_NODE_LOCAL && from_tile &&
             fragment.local == cut.node;
    }
    if (fragment.type != Wire::WIRE_CROSSBAR) {
      return false;
    }
    if (cut.type == CB_NODE_SRC) {
      return from_tile && fragment.jump == cut.node;
    }
    if (cut.type == CB_NODE_DST) {
      return (from_tile && fragment.pos != 0 && fragment.local == cut.node) ||
             (to_tile && fragment.owns_landing && fragment.dst == cut.node);
    }
    if (cut.type == CB_NODE_LOCAL) {
      return from_tile && fragment.pos == 0 && fragment.local == cut.node;
    }
    return cut.type == CB_NODE_JOINT && from_tile &&
           (fragment.joint == cut.node || fragment.joint2 == cut.node);
  };

  size_t cut = 0;
  while (cut < route->size() &&
         std::none_of(nodes.begin(), nodes.end(), [&](const RouteCutNode &node) {
           return fragment_uses_node((*route)[cut], node);
         })) {
    ++cut;
  }
  if (cut == route->size()) {
    return false;
  }
  if (cut == 0) {
    return unrouteNetRoute(net, route_binding_index);
  }

  std::vector<Wire> removed(
      route->begin() + static_cast<std::ptrdiff_t>(cut), route->end());
  if (!removed.empty() && (*route)[cut - 1].type == Wire::WIRE_CROSSBAR &&
      removed.front().type == Wire::WIRE_CROSSBAR &&
      (*route)[cut - 1].dst == removed.front().local) {
    (*route)[cut - 1].owns_landing = true;
    removed.front().owns_dst = false;
  }
  route->resize(cut);
  clearRouteLeases(removed, true);
  rebuildNetRouteTiles(net, {removed});
  return true;
}

namespace {

bool trimNetRouteToTakeoff(rtl::Net &net, size_t route_binding_index) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (!route || route->empty()) {
    return false;
  }

  size_t keep = 0;
  while (keep < route->size()) {
    const Wire &fragment = (*route)[keep++];
    if (fragment.type == Wire::WIRE_CROSSBAR && fragment.jump >= 0 &&
        !sameCoord(fragment.from, fragment.to)) {
      break;
    }
  }
  if (keep == route->size()) {
    return false;
  }

  std::vector<Wire> removed(
      route->begin() + static_cast<std::ptrdiff_t>(keep), route->end());
  if (!removed.empty() && (*route)[keep - 1].type == Wire::WIRE_CROSSBAR &&
      removed.front().type == Wire::WIRE_CROSSBAR &&
      (*route)[keep - 1].dst == removed.front().local) {
    (*route)[keep - 1].owns_landing = true;
    removed.front().owns_dst = false;
  }
  route->resize(keep);
  clearRouteLeases(removed, true);
  rebuildNetRouteTiles(net, {removed});
  return true;
}

} // namespace

bool fpga::unrouteNetRouteToTakeoff(rtl::Net &net,
                                    size_t route_binding_index) {
  return trimNetRouteToTakeoff(net, route_binding_index);
}

bool fpga::unrouteNetRouteToTakeoffFromNode(
    rtl::Net &net, size_t route_binding_index, Coord tile_coord,
    CBNodeNameType node_type, int node) {
  if (route_binding_index >= net.routes.size() || node < 0) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (!route || route->empty()) {
    return false;
  }

  auto fragment_uses_node = [&](const Wire &fragment) {
    bool from_tile = sameCoord(fragment.from, tile_coord);
    bool to_tile = sameCoord(fragment.to, tile_coord);
    if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
      return (from_tile && fragment.from_node_type == node_type &&
              fragment.from_node == node) ||
             (to_tile && fragment.to_node_type == node_type &&
              fragment.to_node == node);
    }
    if (fragment.type != Wire::WIRE_CROSSBAR) {
      return node_type == CB_NODE_LOCAL &&
             fragment.type == Wire::WIRE_TILE_PIN && from_tile &&
             fragment.local == node;
    }
    if (node_type == CB_NODE_SRC) {
      return from_tile && fragment.jump == node;
    }
    if (node_type == CB_NODE_DST) {
      return (from_tile && fragment.pos != 0 && fragment.local == node) ||
             (to_tile && fragment.owns_landing && fragment.dst == node);
    }
    if (node_type == CB_NODE_LOCAL) {
      return from_tile && fragment.pos == 0 && fragment.local == node;
    }
    return node_type == CB_NODE_JOINT && from_tile &&
           (fragment.joint == node || fragment.joint2 == node);
  };

  size_t conflict = 0;
  while (conflict < route->size() && !fragment_uses_node((*route)[conflict])) {
    ++conflict;
  }
  if (conflict == route->size()) {
    return false;
  }

  size_t keep = 0;
  while (keep < route->size()) {
    const Wire &fragment = (*route)[keep++];
    if (fragment.type == Wire::WIRE_CROSSBAR && fragment.jump >= 0 &&
        !sameCoord(fragment.from, fragment.to)) {
      break;
    }
  }
  // A conflict on the takeoff itself cannot preserve that takeoff.
  if (keep == route->size() || conflict < keep) {
    return false;
  }
  return trimNetRouteToTakeoff(net, route_binding_index);
}

bool fpga::unrouteLastRouteStep(rtl::Net &net, size_t route_binding_index) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (!route || route->empty()) {
    return false;
  }

  size_t step_index = route->size();
  while (step_index > 0 &&
         (*route)[step_index - 1].type == Wire::WIRE_TILE_PIN) {
    --step_index;
  }
  if (step_index == 0 || (*route)[step_index - 1].shared) {
    return false;
  }
  --step_index;
  std::vector<Wire> removed = *route;
  route->resize(step_index);
  removed.erase(removed.begin(),
                removed.begin() + static_cast<std::ptrdiff_t>(step_index));
  clearRouteLeases(removed, true);
  rebuildNetRouteTiles(net, {removed});
  return true;
}

bool fpga::unrouteNetBranch(rtl::Net &net, size_t route_binding_index) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  rtl::NetRouteBinding &binding = net.routes[route_binding_index];
  std::vector<Wire> *route = bindingRoute(binding);
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

  std::vector<Wire> branch(
      route->begin() + static_cast<std::ptrdiff_t>(branch_start), route->end());
  route->resize(branch_start);
  clearRouteLeases(branch, true);
  rebuildNetRouteTiles(net, {branch});
  return true;
}

bool fpga::unrouteBrunch(rtl::Net &net, size_t route_binding_index) {
  return unrouteNetBranch(net, route_binding_index);
}

// Release a moved sink endpoint while retaining only a prefix physically
// shared with another live route binding.
static bool detachNetRouteDestinationImpl(
    rtl::Net &net, size_t route_binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded,
    bool retain_private_prefix) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  rtl::NetRouteBinding &binding = net.routes[route_binding_index];
  std::vector<Wire> *route = bindingRoute(binding);
  if (!route || route->empty()) {
    return false;
  }

  size_t keep = liveSharedPrefixLength(net, route_binding_index, excluded);

  if (retain_private_prefix) {
    // A completed moved sink can reuse its own private route as well as a
    // sibling-shared trunk. Incomplete search paths are not proven anchors.
    size_t terminal = route->size();
    while (terminal > 0 && (*route)[terminal - 1].type == Wire::WIRE_TILE_PIN) {
      --terminal;
    }
    if (terminal > 0 && (*route)[terminal - 1].type == Wire::WIRE_CROSSBAR) {
      const Wire &entry = (*route)[terminal - 1];
      if (entry.jump < 0 || (entry.from.x == entry.to.x &&
                             entry.from.y == entry.to.y)) {
        --terminal;
      }
    }
    keep = std::max(keep, terminal);
  }

  if (keep == route->size() && keep > 1 &&
      (*route)[keep - 1].type == Wire::WIRE_TILE_PIN) {
    --keep;
  }
  // The retained prefix owns a landing reused by the first removed terminal
  // fragment.
  if (keep > 0 && keep < route->size() &&
      (*route)[keep - 1].type == Wire::WIRE_CROSSBAR &&
      (*route)[keep].type == Wire::WIRE_CROSSBAR &&
      (*route)[keep - 1].dst == (*route)[keep].local) {
    (*route)[keep - 1].owns_landing = true;
    (*route)[keep].owns_dst = false;
  }

  std::vector<Wire> removed(route->begin() + static_cast<std::ptrdiff_t>(keep),
                            route->end());
  if (!removed.empty() && keep > 0 &&
      (*route)[keep - 1].type == Wire::WIRE_CROSSBAR &&
      removed.front().type == Wire::WIRE_CROSSBAR &&
      (*route)[keep - 1].dst == removed.front().local) {
    removed.front().owns_dst = false;
  }
  route->resize(keep);
  clearRouteLeases(removed, true);
  rebuildNetRouteTiles(net, {removed});
  return true;
}

bool fpga::detachNetRouteDestination(rtl::Net &net,
                                     size_t route_binding_index) {
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  return detachNetRouteDestinationImpl(
      net, route_binding_index, nullptr, route && isRouteComplete(*route));
}

// A moved sink always needs routing again; preserve only the reusable prefix
// when its binding currently owns physical route fragments.
bool fpga::invalidateMovedSinkRoute(rtl::Net &net,
                                    size_t route_binding_index) {
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (route && !route->empty()) {
    // A downstream fanout may branch from this route's private suffix.
    // Transfer that physical prefix before truncating the moved sink.
    promoteSurvivingSourcePrefix(net, route_binding_index);
    bool retain_private = isRouteComplete(*route);
    detachNetRouteDestinationImpl(net, route_binding_index, nullptr,
                                  retain_private);
  }
  return true;
}

bool fpga::invalidateMovedSinkRoutes(const std::vector<NetRouteRef> &routes) {
  std::unordered_set<const rtl::NetRouteBinding *> excluded;
  std::unordered_set<const rtl::NetRouteBinding *> complete;
  for (const NetRouteRef &ref : routes) {
    if (ref.net && ref.binding_index < ref.net->routes.size()) {
      rtl::NetRouteBinding *binding = &ref.net->routes[ref.binding_index];
      excluded.insert(binding);
      std::vector<Wire> *route = bindingRoute(*binding);
      if (route && isRouteComplete(*route)) {
        complete.insert(binding);
      }
    }
  }

  // Transfer ownership only to bindings that survive the complete atomic
  // relocation set.
  for (const NetRouteRef &ref : routes) {
    if (!ref.net || ref.binding_index >= ref.net->routes.size()) {
      continue;
    }
    std::vector<Wire> *route = bindingRoute(ref.net->routes[ref.binding_index]);
    if (route && !route->empty()) {
      promoteSurvivingSourcePrefix(*ref.net, ref.binding_index, &excluded);
    }
  }

  bool changed = false;
  for (const NetRouteRef &ref : routes) {
    if (!ref.net || ref.binding_index >= ref.net->routes.size()) {
      continue;
    }
    const rtl::NetRouteBinding *binding = &ref.net->routes[ref.binding_index];
    changed = detachNetRouteDestinationImpl(
                  *ref.net, ref.binding_index, &excluded,
                  complete.contains(binding)) ||
              changed;
  }
  return changed;
}

bool fpga::discardNetBranch(rtl::Net &net, size_t route_binding_index) {
  RouteHistoryScope history(&net, __func__);
  // A private suffix may already be a shared prefix of surviving children.
  // Promote their ownership before releasing leases, and rebuild registrations
  // on every touched Tile, including the discarded route's shared prefix.
  return unrouteNetRoute(net, route_binding_index);
}

// Clear an atomic route tree, including shared fanout fragments owned by the
// tree.
bool fpga::unrouteNetRouteTree(
    rtl::Net &net, const std::vector<size_t> &route_binding_indices) {
  RouteHistoryScope history(&net, __func__);
  bool changed = false;
  std::vector<std::vector<Wire>> removed_routes;
  for (size_t route_binding_index : route_binding_indices) {
    if (route_binding_index >= net.routes.size()) {
      continue;
    }
    std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
    if (!route || route->empty()) {
      continue;
    }
    // Transfer every retained shared prefix before removing its current owner.
    promoteSurvivingSourcePrefix(net, route_binding_index);
    removed_routes.push_back(*route);
    route->clear();
    changed = true;
  }
  if (!changed) {
    return false;
  }
  std::vector<const std::vector<Wire> *> removed_refs;
  removed_refs.reserve(removed_routes.size());
  for (const std::vector<Wire> &removed : removed_routes) {
    removed_refs.push_back(&removed);
  }
  clearRouteLeases(removed_refs, true);
  rebuildNetRouteTiles(net, removed_routes);
  return true;
}

bool fpga::unrouteSourceRouteTree(const std::vector<NetRouteRef> &routes) {
  std::unordered_map<rtl::Net *, std::vector<std::vector<Wire>>>
      removed_routes_by_net;
  std::unordered_set<const rtl::NetRouteBinding *> removed_bindings;
  for (const NetRouteRef &ref : routes) {
    if (!ref.net || ref.binding_index >= ref.net->routes.size()) {
      continue;
    }
    rtl::NetRouteBinding &binding = ref.net->routes[ref.binding_index];
    if (!removed_bindings.insert(&binding).second) {
      continue;
    }
    std::vector<Wire> *route = bindingRoute(binding);
    if (!route || route->empty()) {
      continue;
    }
    std::vector<Wire> &removed =
        removed_routes_by_net[ref.net].emplace_back(*route);
    // The caller supplies the entire source tree, so no shared replica survives.
    for (Wire &fragment : removed) {
      fragment.shared = false;
    }
    // Prefix retries can replace this route many times. Return its old backing
    // storage now instead of retaining every historical peak allocation.
    std::vector<Wire>().swap(*route);
  }
  if (removed_routes_by_net.empty()) {
    return false;
  }
  std::vector<const std::vector<Wire> *> removed_refs;
  removed_refs.reserve(routes.size());
  for (const auto &[net, removed_routes] : removed_routes_by_net) {
    (void)net;
    for (const std::vector<Wire> &route : removed_routes) {
      removed_refs.push_back(&route);
    }
  }
  // Atomic source ownership makes per-fragment live-owner scans unnecessary.
  clearRouteLeases(removed_refs, false);
  // Rebuild each logical net only from tiles touched by that net's removed
  // bindings; comparing every net with the complete batch is quadratic.
  for (auto &[net, removed_routes] : removed_routes_by_net) {
    rebuildNetRouteTiles(*net, removed_routes);
  }
  return true;
}

bool fpga::unrouteNet(rtl::Net &net) {
  bool changed = false;
  std::vector<std::vector<Wire>> removed_routes;
  for (size_t binding_index = 0; binding_index < net.routes.size();
       ++binding_index) {
    rtl::NetRouteBinding &binding = net.routes[binding_index];
    std::vector<Wire> *route = bindingRoute(binding);
    if (!route || route->empty()) {
      continue;
    }
    // A cross-net fanout can survive whole-net removal and inherit ownership.
    promoteSurvivingSourcePrefix(net, binding_index);
    removed_routes.push_back(*route);
    route->clear();
    changed = true;
  }

  if (changed) {
    std::vector<const std::vector<Wire> *> removed_refs;
    removed_refs.reserve(removed_routes.size());
    for (const std::vector<Wire> &removed : removed_routes) {
      removed_refs.push_back(&removed);
    }
    clearRouteLeases(removed_refs, true);
    rebuildNetRouteTiles(net, removed_routes);
  }
  return changed;
}

// Release every binding for one exact driver-to-sink connection while
// preserving unrelated connections represented by the same rtl::Net object.
size_t fpga::unrouteNetConnection(rtl::Net &net, rtl::Inst *from, rtl::Inst *to,
                                  const std::string &from_port,
                                  const std::string &to_port) {
  size_t cleared = 0;
  for (size_t binding_index = 0; binding_index < net.routes.size();
       ++binding_index) {
    const rtl::NetRouteBinding &binding = net.routes[binding_index];
    if (binding.from != from || binding.to != to ||
        binding.from_port != from_port || binding.to_port != to_port) {
      continue;
    }
    if (unrouteNetRoute(net, binding_index)) {
      ++cleared;
    }
  }
  return cleared;
}
