#include "Wire.h"
#include "Device.h"
#include "Tile.h"
#include "RouteSource.h"
#include "RepairStatistics.h"
#include "InputRepairBudget.h"

#include <absl/container/flat_hash_map.h>

#include <algorithm>
#include <array>
#include <unordered_set>
#include <map>
#include <memory>
#include <set>
#include <queue>
#include <span>
#include <tuple>
#include <ostream>
#include <iomanip>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <charconv>
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace fpga;

void Wire::assign(rtl::Net *net) {
  PNR_ASSERT(net->wire.peer == nullptr,
             "assigning wire {}:{}-{}:{} to already assigned net {}", from.x,
             from.y, to.x, to.y, net->makeName());
  net->wire.set(static_cast<Referable<Wire> *>(this));
}

namespace {

thread_local uint64_t route_mutation_revision = 0;
thread_local RepairStatistics repair_statistics;

bool sameCoord(const Coord &a, const Coord &b) {
  return a.x == b.x && a.y == b.y;
}

// Compare only routed resource identity; ownership and annotation fields may
// differ. FORK and TRANSIT both consume a DST at the same physical hop;
// their distinct pos values describe ownership, not different connectivity.
bool samePhysicalFragment(const Wire &left, const Wire &right) {
  return left.type == right.type && sameCoord(left.from, right.from) &&
         sameCoord(left.to, right.to) && left.local == right.local &&
         (left.pos == right.pos ||
          (left.type == Wire::WIRE_CROSSBAR &&
           ((left.pos == 1 && right.pos == 2) || (left.pos == 2 && right.pos == 1)))) &&
         left.jump == right.jump &&
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
      if (!rtl::sameRouteSource(candidate_binding.from, candidate_binding.from_port,
                                binding.from, binding.from_port)) {
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

// Transfer a removed Generic prefix to one surviving branch before releasing
// leases.
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
      if (!rtl::sameRouteSource(sibling_binding.from, sibling_binding.from_port,
                                removed_binding.from, removed_binding.from_port)) {
        continue;
      }
      std::vector<Wire> *sibling = bindingRoute(sibling_binding);
      if (!sibling || sibling->empty()) {
        continue;
      }

      size_t common = 0;
      // Partial repairs can make the source prefix private while a later
      // island still borrows its branching DST. Physical continuity and the
      // canonical source above establish sharing; the first fragment's flag
      // cannot decide whether that island needs its owner's lease.
      // A partial cut gives each binding its own non-shared GAP record and
      // may promote the first retained fragment after it. Neither flag ends
      // the physically common path. Continue through identical gaps/islands
      // so their landing ownership survives a sibling's source replacement.
      while (common < removed->size() && common < sibling->size() &&
             samePhysicalFragment((*removed)[common], (*sibling)[common])) {
        ++common;
      }
      if (common == 0) {
        continue;
      }

      // A sibling can fork immediately after the common prefix. Its private
      // FORK does not own the incoming DST: the departing route's next
      // TRANSIT fragment did. Preserve that boundary as a landing of the
      // surviving prefix before source replacement removes the old fragment.
      // Transferring only explicit owns_landing flags misses this owner.
      Wire &tip = (*sibling)[common - 1];
      if (common < removed->size() && tip.type == Wire::WIRE_CROSSBAR &&
          tip.jump >= 0 && tip.dst >= 0) {
        Wire &next = (*removed)[common];
        if (next.type == Wire::WIRE_CROSSBAR && !next.shared &&
            next.pos != 0 && next.owns_dst && next.local == tip.dst &&
            sameCoord(next.from, tip.to)) {
          tip.owns_landing = true;
          next.owns_dst = false;
          ++transferred_count;
        }
      }

      for (size_t index = 0; index < common; ++index) {
        if ((*removed)[index].type == Wire::WIRE_GAP) continue;
        if ((*removed)[index].owns_landing) {
          (*sibling)[index].owns_landing = true;
          (*removed)[index].owns_landing = false;
          ++transferred_count;
        }
        if ((*removed)[index].shared) {
          continue;
        }
        (*sibling)[index].shared = false;
        (*sibling)[index].owns_dst |= (*removed)[index].owns_dst;
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
                         bool owned_only, OwnershipQueryWork *work = nullptr) {
  for (const Wire &fragment : route) {
    if (work) ++work->fragment_visits;
    // Most fragments in a Tile-registered route belong to other nodes (often
    // other Tiles). Reject by numeric ID before coordinate/ownership work.
    // This is only a negative filter; all existing lease rules follow below.
    if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
      if (!(fragment.to_node_type == node_type && fragment.to_node == node) &&
          !(!owned_only && fragment.from_node_type == node_type && fragment.from_node == node))
        continue;
    } else if (fragment.type == Wire::WIRE_CROSSBAR) {
      switch (node_type) {
        case CB_NODE_SRC: if (fragment.jump != node) continue; break;
        case CB_NODE_DST:
          if (fragment.local != node && !(fragment.owns_landing && fragment.dst == node)) continue;
          break;
        case CB_NODE_JOINT:
          if (fragment.joint != node && fragment.joint2 != node) continue;
          break;
        case CB_NODE_LOCAL: if (fragment.local != node) continue; break;
        default: continue;
      }
    } else if (fragment.type == Wire::WIRE_TILE_PIN) {
      if (node_type != CB_NODE_LOCAL || fragment.local != node) continue;
    } else continue;
    // A shared replica can own an explicitly reserved landing, but that
    // says nothing about its incoming DST on the other end of the hop.
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
      if (!owned_only && from_tile && fragment.from_node_type == node_type &&
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
            (fragment.pos != 0 && fragment.jump >= 0 && fragment.dst >= 0)) {
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
          (!transit_only || (fragment.jump >= 0 && fragment.dst >= 0))) {
        return true;
      }
      if (from_tile && fragment.type == Wire::WIRE_CROSSBAR &&
          fragment.pos != 0 && fragment.owns_dst && fragment.local == node &&
          (!transit_only || (fragment.jump >= 0 && fragment.dst >= 0))) {
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
          // A SRC -> DST hop can stay inside one CB. Only a terminal
          // fragment (no SRC/landing DST), not zero distance, is Grounding.
          if (transit_only && (fragment.jump < 0 || fragment.dst < 0)) {
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
  ++route_mutation_revision;
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
        case CB_NODE_LOCAL:
          // A removed sink may have retained its pin bit while this LOCAL
          // was still used as transit. Release both bits with its last owner.
          tile->cb.local.local &= clear;
          tile->pin_state.leased_nodes &= clear;
          break;
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
        tile->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
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
    // Ordinary takeoff LOCAL leases are released by their CROSSBAR fragment.
    // Dedicated trees have only ROUTE_EDGE fragments, which release their
    // destination nodes, never the root. Release that output pin here too.
    const bool dedicated_root = fragment_index == 0 && route.size() > 1 &&
        route[1].type == Wire::WIRE_ROUTE_EDGE;
    if (fragment_index + 1 != route.size() && !dedicated_root) {
      return;
    }
    Tile *tile = Device::current().getTile(fragment.from.x, fragment.from.y);
    if (!tile || fragment.local < 0) {
      return;
    }
    if (findNetOwnersByNode(*tile, CB_NODE_LOCAL, fragment.local).empty()) {
      tile->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
      tile->cb.local.local &= ~(NodeMask{0, 1} << fragment.local);
      if (Tile* resource = Device::current().getTile(
              fragment.resource.x, fragment.resource.y)) {
        resource->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
      }
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
    case CB_NODE_LOCAL:
          // A removed sink may have retained its pin bit while this LOCAL
          // was still used as transit. Release both bits with its last owner.
          tile->cb.local.local &= clear;
          tile->pin_state.leased_nodes &= clear;
          break;
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
      tile->pin_state.leased_nodes &= ~(NodeMask{0, 1} << fragment.local);
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
    std::vector<Wire> removed;
    if (route && !route->empty()) {
      // Moving can replace a retained input prefix with a different takeoff.
      // Transfer shared ownership before losing the old binding/storage.
      promoteSurvivingSourcePrefix(net, lookup.index);
      if (route_index < owner.wires.size()) {
        auto &replacement = owner.wires[route_index];
        for (size_t i = 0; i < route->size() && i < replacement.size() &&
                           replacement[i].shared &&
                           samePhysicalFragment((*route)[i], replacement[i]); ++i) {
          Wire &old = (*route)[i];
          if (!old.shared) {
            replacement[i].shared = false;
            replacement[i].owns_dst = old.owns_dst;
          }
          replacement[i].owns_landing |= old.owns_landing;
        }
      }
      removed.swap(*route);
    }
    binding.owner = &owner;
    binding.route_index = route_index;
    ++route_mutation_revision;
    if (!removed.empty()) {
      // Publish the replacement first: it may reuse the old source LOCAL or
      // prefix. Owner-aware release must see it as well as surviving siblings.
      if (auto *replacement = bindingRoute(binding)) {
        registerNetRouteTiles(net, *replacement, lookup.index);
      }
      clearRouteLeases(removed, true);
      rebuildNetRouteTiles(net, {removed});
    }
    return lookup.index;
  }
  // No exact endpoint identity exists; create one independent physical branch.
  ++route_mutation_revision;
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
      // Existing receiver bindings and their stable IDs are unchanged. Only
      // the transferred branch needs registering; rebuilding the receiver
      // here revisits its entire growing fanout tree for every suffix.
      auto &transferred = new_net.routes.back();
      if (auto *route = bindingRoute(transferred); route && !route->empty()) {
        registerNetRouteTiles(new_net, *route, new_net.routes.size() - 1);
      }
    }
    ++changed;
  }
  // Retargeting changes the Net found through each routed tile, including an
  // existing partial prefix.
  if (changed != 0) {
    old_net.invalidateRouteLookup();
    new_net.invalidateRouteLookup();
    rebuildNetRouteTiles(old_net, affected_routes);
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
    ++route_mutation_revision;
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
  bool distinct_endpoints = false;
  for (const Wire &fragment : route) {
    if (fragment.type == Wire::WIRE_GAP) return false;
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
      distinct_endpoints = true;
    }
  }
  return has_crossbar || (!distinct_endpoints && tile_pin_count >= 2);
}

bool fpga::hasRouteGaps(const std::vector<Wire> &route) {
  return std::any_of(route.begin(), route.end(), [](const Wire &w) {
    return w.type == Wire::WIRE_GAP;
  });
}

fpga::RouteState fpga::routeState(const std::vector<Wire> &route) {
  return route.empty() ? RouteState::Unrouted :
      isRouteComplete(route) ? RouteState::Routed : RouteState::PartiallyRouted;
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
  if (first_fragment < route.size()) ++route_mutation_revision;
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

namespace {
uint64_t auditCoordKey(Coord coord) {
  return (uint64_t(uint32_t(coord.x)) << 32) | uint32_t(coord.y);
}
}

// Bindings are processed contiguously, so the last entry deduplicates repeated
// visits without a second per-tile hash table or copies of route fragments.
fpga::CongestionAuditIndex::CongestionAuditIndex(const std::vector<rtl::Net*>& nets)
    : design_nets(nets.begin(), nets.end()) {
  for (auto* net : design_nets) {
    if (!net) continue;
    for (size_t b = 0; b < net->routes.size(); ++b) {
      auto* route = bindingRoute(net->routes[b]);
      if (!route) continue;
      for (const auto& wire : *route) {
        auto index_coord = [&](Coord coord) {
          auto& refs = by_tile[auditCoordKey(coord)];
          if (refs.empty() || refs.back().net != net || refs.back().binding_index != b)
            refs.push_back({net, b});
        };
        index_coord(wire.from);
        index_coord(wire.to);
        // Dedicated endpoints may also lease their resource-side pin.
        if (wire.type == Wire::WIRE_TILE_PIN && wire.local >= 0 &&
            wire.resource.x >= 0 && wire.resource.y >= 0) index_coord(wire.resource);
      }
    }
  }
}

// Return an immutable empty range for coordinates untouched by any live route.
const std::vector<fpga::NetRouteRef>& fpga::CongestionAuditIndex::bindings(Coord tile) const {
  static const std::vector<NetRouteRef> empty;
  auto found = by_tile.find(auditCoordKey(tile));
  return found == by_tile.end() ? empty : found->second;
}

static fpga::CongestionAudit auditTileCongestionImpl(
    fpga::Tile& tile, std::ostream& out, const std::unordered_set<rtl::Net*>& nets,
    const fpga::CongestionAuditIndex* index) {
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
  NodeMask required_tip_landings;
  NodeMask required_incoming_dsts;
  // The design scope is the authority for object existence. Do not dereference
  // an indexed pointer that has disappeared from that scope.
  for (const auto& ref : tile.routedNets) {
    if (ref.peer && !nets.contains(ref.peer)) {
      ++audit.stale_registrations;
      out << "STALE_NET_REF net_exists=0\n";
    }
  }
  auto inspect_binding = [&](rtl::Net* net, size_t b) {
      auto& binding = net->routes[b];
      auto* route = bindingRoute(binding);
      if (!route) return;
      bool touches = false;
      for (size_t f = 0; f < route->size(); ++f) {
        const Wire& wire = (*route)[f];
        // Resource pin leases use the endpoint's route-local number. They
        // do not require CB registrations or a second lease in generic routes.
        if (wire.type == Wire::WIRE_TILE_PIN && wire.local >= 0 && wire.local < CB_MAX_NODES &&
            sameCoord(wire.resource, tile.coord) && !sameCoord(wire.from, tile.coord))
          claims[{CB_NODE_LOCAL, wire.local}].push_back({net, b, f, !wire.shared, false});
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
          // A retained unfinished hop physically needs its landing even if
          // buggy cleanup lost owns_landing. Do not let the same corrupted
          // flag hide the missing lease from this independent audit.
          bool tip_landing = f + 1 == route->size() &&
              wire.type == Wire::WIRE_CROSSBAR && wire.jump >= 0 &&
              !sameCoord(wire.from, wire.to) && sameCoord(wire.to, tile.coord) &&
              type == CB_NODE_DST && wire.dst == node;
          const bool incoming_dst = wire.type == Wire::WIRE_CROSSBAR &&
              wire.pos != 0 && sameCoord(wire.from, tile.coord) &&
              type == CB_NODE_DST && wire.local == node;
          if (!tip_landing && !incoming_dst &&
              !routeUsesNodeOnTile({&wire, 1}, tile, type, node, false, false)) continue;
          if (tip_landing) required_tip_landings.setBit(node);
          // A FORK borrows its incoming DST; owns_dst=false does not make
          // that physical dependency optional. Audit it independently of the
          // exact ownership flag whose corruption can lose the lease.
          if (incoming_dst) required_incoming_dsts.setBit(node);
          // Recheck the owning fragment flags directly, not the owner lookup.
          // A shared hop can own its landing, never its source-side DST.
          bool owns = !wire.shared ||
              (wire.type == Wire::WIRE_CROSSBAR && type == CB_NODE_DST &&
               wire.owns_landing && sameCoord(wire.to, tile.coord) && wire.dst == node);
          if (tip_landing) owns = wire.owns_landing;
          if (incoming_dst && !wire.owns_dst &&
              !(wire.owns_landing && sameCoord(wire.to,tile.coord) && wire.dst==node))
            owns = false;
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
  };
  if (index) {
    for (const auto& ref : index->bindings(tile.coord)) inspect_binding(ref.net, ref.binding_index);
  } else {
    for (rtl::Net* net : nets) {
      if (!net) continue;
      for (size_t b = 0; b < net->routes.size(); ++b) inspect_binding(net, b);
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
    bool requires_lease = (type == CB_NODE_DST &&
        (required_tip_landings.testBit(node) || required_incoming_dsts.testBit(node))) ||
        std::any_of(users.begin(), users.end(),
        [](const Claim& c) { return c.owner && c.requires_lease; });
    const char* label = type == CB_NODE_SRC ? "SRC" : type == CB_NODE_DST ? "DST" :
                        type == CB_NODE_JOINT ? "JOINT" : "LOCAL";
    const std::string* name = tile.cb_type ? tile.cb_type->nodeName(type, node) : nullptr;
    // Multiple same-source replicas/RTL aliases are valid. Different driver
    // ports are not, even when every lease and registration has a live owner.
    const rtl::NetRouteBinding *source = nullptr;
    bool conflicting = false;
    for (const auto &claim : users) {
      const auto &binding = claim.net->routes[claim.binding];
      if (!binding.from) continue; // no source identity available to compare
      if (!source) source = &binding;
      else if (!rtl::sameRouteSource(source->from, source->from_port, binding.from, binding.from_port))
        conflicting = true;
    }
    audit.conflicting_driver_nodes += conflicting;
    const char* status = leased ? (conflicting ? "CONFLICTING_DRIVERS" : owners ? "VERIFIED" : "UNOWNED_LEASE")
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
      << " stale_registration=" << audit.stale_registrations
      << " conflicting_drivers=" << audit.conflicting_driver_nodes << '\n';
  return audit;
}

// The exhaustive entry remains the reference implementation for audit checks.
fpga::CongestionAudit fpga::auditTileCongestion(
    Tile& tile, std::ostream& out, const std::vector<rtl::Net*>& design_nets) {
  std::unordered_set<rtl::Net*> nets(design_nets.begin(), design_nets.end());
  return auditTileCongestionImpl(tile, out, nets, nullptr);
}

// The indexed entry shares all lease and registration verification logic.
fpga::CongestionAudit fpga::auditTileCongestion(
    Tile& tile, std::ostream& out, const CongestionAuditIndex& index) {
  return auditTileCongestionImpl(tile, out, index.nets(), &index);
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
        << " complete=" << (route && isRouteComplete(*route))
        << " state=" << (!route || route->empty() ? "unrouted" :
                            isRouteComplete(*route) ? "routed" : "partially_routed") << '\n';
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
struct RouteHistoryWatch {
  int x = -1, y = -1, node = -1;
  CBNodeNameType type = CB_NODE_DST;
};
const RouteHistoryWatch& routeHistoryWatch() {
  static const RouteHistoryWatch watch = [] {
    RouteHistoryWatch w;
    const char* value = std::getenv("SCALEPNR_ROUTE_HISTORY_NODE");
    char kind[8] = {};
    if (!value || std::sscanf(value, "%d,%d,%7[^,],%d", &w.x, &w.y, kind, &w.node) != 4)
      return RouteHistoryWatch{};
    std::string_view label(kind);
    if (label == "SRC") w.type = CB_NODE_SRC;
    else if (label == "DST") w.type = CB_NODE_DST;
    else if (label == "JOINT") w.type = CB_NODE_JOINT;
    else if (label == "LOCAL") w.type = CB_NODE_LOCAL;
    else return RouteHistoryWatch{};
    if (w.x < 0 || w.y < 0 || w.node < 0 || w.node >= CB_MAX_NODES)
      return RouteHistoryWatch{};
    return w;
  }();
  return watch;
}
std::unordered_set<rtl::Net*> route_history_discovered;
bool watchedLease(const Tile& tile, const RouteHistoryWatch& watch) {
  return congestionNodeMask(tile.cb, watch.type).testBit(watch.node) ||
      (watch.type == CB_NODE_LOCAL && tile.isPinNodeLeased(watch.node));
}
void traceHistoryOwnerLookup(Tile& tile, CBNodeNameType type, int node,
                             const std::vector<NetRouteRef>& result) {
  const auto& watch = routeHistoryWatch();
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
  const auto& watch = routeHistoryWatch();
  if (routeHistoryNet().empty() && watch.node < 0) return;
  active = true;
  previous_actor = route_history_actor;
  if (!actor.empty()) route_history_actor = actor;
  if (watch.node >= 0) {
    if (Tile* tile = Device::current().getTile(watch.x, watch.y)) {
      watch_valid = true;
      watched_net = net;
      watched_lease = watchedLease(*tile, watch);
    }
  }
  if (!net || (!route_history_discovered.contains(net) &&
               (routeHistoryNet().empty() || net->makeName(1000000) != routeHistoryNet()))) return;
  selected = net;
  id = ++route_history_sequence;
  auto& out = routeHistoryOutput();
  out << "HISTORY_BEGIN id=" << id << " operation=" << operation
      << " actor=" << std::quoted(std::string(route_history_actor)) << '\n';
  dumpNetRouteHistory(*selected, out);
  out.flush();
}

uint64_t fpga::routeMutationRevision() { return route_mutation_revision; }

const RepairStatistics& fpga::repairStatistics() { return repair_statistics; }
void fpga::resetRepairStatistics() { repair_statistics = {}; }
void fpga::printRepairStatistics(std::ostream& out, std::string_view context) {
  if (!std::getenv("SCALEPNR_PROFILE_REPAIR")) return;
  out << "REPAIR_TOTAL context=" << std::quoted(std::string(context));
  for (size_t i=0;i<repair_statistics.counters.size();++i)
    out << ' ' << repairCounterNames[i] << '=' << repair_statistics.counters[i];
  for (size_t i=0;i<repair_statistics.seconds.size();++i)
    out << ' ' << repairPhaseNames[i] << "_s=" << repair_statistics.seconds[i];
  out << '\n';
}

fpga::RouteHistoryScope::~RouteHistoryScope() {
  if (watch_valid) {
    const auto& watch = routeHistoryWatch();
    Tile* tile = Device::current().getTile(watch.x, watch.y);
    const bool after = tile && watchedLease(*tile, watch);
    if (after != watched_lease) {
      auto& out = routeHistoryOutput();
      out << "WATCH_TRANSITION operation=" << operation
          << " actor=" << std::quoted(std::string(route_history_actor))
          << " net=" << std::quoted(watched_net ? watched_net->makeName(1000000) : "<none>")
          << " tile=(" << watch.x << ',' << watch.y << ") kind=" << static_cast<int>(watch.type)
          << " node=" << watch.node << " before=" << watched_lease << " after=" << after << '\n';
      // Discover the actor even when no owner survives. Thereafter record this
      // net's full before/after trees, including mutations that leave bits set.
      if (watched_net) {
        route_history_discovered.insert(watched_net);
        if (!selected) {
          selected = watched_net;
          id = ++route_history_sequence;
          out << "HISTORY_DISCOVERED id=" << id << " operation=" << operation
              << " before_tree_unavailable=1\n";
        }
      }
      out.flush();
    }
  }
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

namespace {
template<class Accept>
bool visitNodeOwners(Tile &tile, CBNodeNameType node_type, int node,
                     bool transit_only, Accept &&accept) {
  bool found = false;
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
        found = true;
        if (!accept(NetRouteRef{ref.net, binding_index})) return false;
      }
    }
    return found;
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
        found = true;
        if (!accept(NetRouteRef{net, binding_index})) return false;
      }
    }
  }
  return found;
}
} // namespace

bool fpga::RouteOwnerQueries::allMatch(Tile &tile, CBNodeNameType type, int node,
    const std::function<bool(const NetRouteRef&)> &accept, bool transit_only) {
  ++counts.queries;
  if (node < 0 || node >= CB_MAX_NODES || type > CB_NODE_DST) return false;
  // Keep opt-in owner tracing on its exhaustive, uncached diagnostic path.
  const auto &watch = routeHistoryWatch();
  if (node == watch.node && type == watch.type &&
      tile.coord.x == watch.x && tile.coord.y == watch.y)
    return allNetOwnersMatch(tile, type, node, accept, transit_only);
  // Preemption/publication invalidates every stored binding index and owner.
  if (revision != routeMutationRevision()) {
    tiles.clear();
    revision = routeMutationRevision();
  }
  auto found = tiles.find(&tile);
  if (found == tiles.end()) {
    // Bound memory even when a failed repair explores the entire device.
    if (tiles.size() >= 64) tiles.clear();
    auto &indexed = tiles[&tile];
    ++counts.tile_builds;
    auto add_binding = [&](rtl::Net &net, size_t index) {
      if (index >= net.routes.size()) return;
      const auto *route = bindingRoute(net.routes[index]);
      if (!route) return;
      std::array<NodeMask, CB_NODE_DST + 1> owned{}, transit{};
      for (const Wire &w : *route) {
        ++counts.fragment_visits;
        const bool from = sameCoord(w.from, tile.coord);
        const bool to = sameCoord(w.to, tile.coord);
        if (!from && !to) continue;
        // Reuse the lease predicate on a single fragment instead of duplicating
        // shared landing, terminal, fork and numeric-edge ownership rules.
        auto remember = [&](CBNodeNameType kind, int id) {
          if (id < 0 || id >= CB_MAX_NODES || kind > CB_NODE_DST) return;
          const std::span<const Wire> one(&w, 1);
          if (routeUsesNodeOnTile(one, tile, kind, id, false, true, &counts)) {
            owned[kind].setBit(id);
            if (routeUsesNodeOnTile(one, tile, kind, id, true, true, &counts))
              transit[kind].setBit(id);
          }
        };
        if (w.type == Wire::WIRE_ROUTE_EDGE) {
          if (to && w.to_node_type >= 0 && w.to_node_type <= CB_NODE_DST)
            remember(static_cast<CBNodeNameType>(w.to_node_type), w.to_node);
        } else if (w.type == Wire::WIRE_CROSSBAR) {
          if (from) {
            remember(CB_NODE_SRC, w.jump);
            remember(CB_NODE_DST, w.local);
            remember(CB_NODE_JOINT, w.joint);
            remember(CB_NODE_JOINT, w.joint2);
            remember(CB_NODE_LOCAL, w.local);
          }
          if (to && w.owns_landing) remember(CB_NODE_DST, w.dst);
        } else if (w.type == Wire::WIRE_TILE_PIN && from) {
          remember(CB_NODE_LOCAL, w.local);
        }
      }
      // Emit each binding only once per typed node, preserving registration
      // order even when its vector repeats fragments or revisits the tile.
      for (unsigned kind = 0; kind <= CB_NODE_DST; ++kind)
        owned[kind].for_each_set_bit([&](int id) {
          indexed[kind * CB_MAX_NODES + id].push_back(
              {{&net, index}, transit[kind].testBit(id)});
          return false;
        });
    };
    if (tile.routed_bindings_authoritative) {
      for (const auto &ref : tile.routed_bindings) if (ref.net) {
        ++counts.binding_resolutions;
        add_binding(*ref.net, ref.net->findRouteBindingById(ref.route_id));
      }
    } else {
      for (const auto &ref : tile.routedNets) if (ref.peer)
        for (size_t i = 0; i < ref.peer->routes.size(); ++i) {
          ++counts.binding_resolutions;
          add_binding(*ref.peer, i);
        }
    }
    found = tiles.find(&tile);
  }
  const auto owners = found->second.find(unsigned(type) * CB_MAX_NODES + node);
  if (owners == found->second.end()) return false;
  bool any = false;
  for (const auto &owner : owners->second) {
    if (transit_only && !owner.transit) continue;
    any = true;
    if (!accept(owner.ref)) return false;
  }
  return any;
}

std::vector<NetRouteRef> fpga::findNetOwnersByNode(Tile &tile,
                                                CBNodeNameType node_type,
                                                int node, bool transit_only) {
  std::vector<NetRouteRef> result;
  visitNodeOwners(tile, node_type, node, transit_only, [&](const NetRouteRef &ref) {
    result.push_back(ref);
    return true;
  });
  traceHistoryOwnerLookup(tile, node_type, node, result);
  return result;
}

bool fpga::allNetOwnersMatch(Tile &tile, CBNodeNameType node_type, int node,
                             const std::function<bool(const NetRouteRef &)> &accept,
                             bool transit_only) {
  const auto &watch = routeHistoryWatch();
  if (node == watch.node && node_type == watch.type &&
      tile.coord.x == watch.x && tile.coord.y == watch.y) {
    // Keep opt-in ownership proofs exhaustive even for a short-circuit query.
    const auto owners = findNetOwnersByNode(tile, node_type, node, transit_only);
    return !owners.empty() && std::all_of(owners.begin(), owners.end(), accept);
  }
  return visitNodeOwners(tile, node_type, node, transit_only, accept);
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

// Include the incoming hop of a contested DST, not just its outgoing fragment.
static size_t routeNodeCut(std::span<const Wire> route,
                           const std::vector<fpga::RouteCutNode> &nodes, OwnershipQueryWork *work = nullptr) {
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
             (to_tile && fragment.dst == cut.node);
    }
    if (cut.type == CB_NODE_LOCAL) {
      return from_tile && fragment.pos == 0 && fragment.local == cut.node;
    }
    return cut.type == CB_NODE_JOINT && from_tile &&
           (fragment.joint == cut.node || fragment.joint2 == cut.node);
  };

  size_t cut = 0;
  while (cut < route.size() &&
         std::none_of(nodes.begin(), nodes.end(), [&](const RouteCutNode &node) {
           if (work) ++work->fragment_visits;
           return fragment_uses_node(route[cut], node);
         })) {
    ++cut;
  }
  return cut;
}

std::vector<fpga::NetRouteRef> fpga::findNetRouteCutVictims(
    const std::vector<RouteCutNode> &cuts, OwnershipQueryWork *work) {
  if (work) ++work->queries;
  std::vector<NetRouteRef> result;
  auto inspect = [&](rtl::Net *net, size_t index) {
    if (!net || index >= net->routes.size()) return;
    if (std::any_of(result.begin(), result.end(), [&](const auto &old) {
          return old.net == net && old.binding_index == index;
        })) return;
    const auto *route = bindingRoute(net->routes[index]);
    if (route && routeNodeCut(*route, cuts, work) < route->size())
      result.push_back({net, index});
  };
  for (const auto &node : cuts) {
    auto *tile = Device::current().getTile(node.tile.x, node.tile.y);
    if (!tile || node.node < 0) continue;
    if (tile->routed_bindings_authoritative) {
      for (const auto &ref : tile->routed_bindings)
        if (ref.net) {
          if (work) ++work->binding_resolutions;
          inspect(ref.net, ref.net->findRouteBindingById(ref.route_id));
        }
    } else {
      for (const auto &ref : tile->routedNets)
        if (ref.peer)
          for (size_t i = 0; i < ref.peer->routes.size(); ++i) {
            if (work) ++work->binding_resolutions;
            inspect(ref.peer, i);
          }
    }
  }
  return result;
}

// The smallest contiguous hole containing every conflict. A contested DST
// includes both its arriving hop and its departing hop; otherwise the retained
// suffix would keep that very DST leased. Direct endpoint conflicts remain
// protected. If cutting a transit reaches the terminal tail, keep the LOCAL
// pin, not an unusable old arrival: repair that victim's input path as well.
static size_t transitCutEnd(std::span<const Wire> route, size_t &first,
                            const std::vector<RouteCutNode> &cuts) {
  auto graph_transit = [&](size_t i) {
    // Explicit intra-CB edges can precede a physical jump, including
    // DST -> LOCAL -> SRC. A terminal-only graph path is Grounding.
    for (; i < route.size() && route[i].type == Wire::WIRE_ROUTE_EDGE; ++i)
      if (!sameCoord(route[i].from, route[i].to) &&
          route[i].from_node_type == CB_NODE_SRC && route[i].to_node_type == CB_NODE_DST)
        return true;
    return false;
  };
  size_t end = first;
  for (size_t i = first; i < route.size(); ++i) {
    if (routeNodeCut(route.subspan(i, 1), cuts) != 0) continue;
    const auto &w = route[i];
    if (w.type == Wire::WIRE_ROUTE_EDGE) {
      if (!graph_transit(i)) return first;
    } else if (w.type != Wire::WIRE_CROSSBAR || w.pos == 0 ||
               w.jump < 0 || w.dst < 0) return first;
    end = i + 1;
  }
  // Reusing a SRC also reuses its physical landing. Do not retain a foreign
  // signal on that landing: include the departing transit fragment as well.
  if (end > first && end < route.size()) {
    const auto &last = route[end - 1];
    if (std::any_of(cuts.begin(), cuts.end(), [&](const auto &cut) {
      return cut.type == CB_NODE_SRC && sameCoord(cut.tile, last.from) &&
          (last.type == Wire::WIRE_ROUTE_EDGE
              ? last.from_node_type == CB_NODE_SRC && cut.node == last.from_node
              : cut.node == last.jump);
    })) ++end;
  }
  // Open a hole at DST boundaries, not midway through a LOCAL bridge.
  // Graph edges own only their destination; retain a CROSSBAR at the right
  // boundary so its incoming DST still has a real lease owner.
  while (first > 0 && route[first].type == Wire::WIRE_ROUTE_EDGE &&
         route[first].from_node_type != CB_NODE_DST) --first;
  while (end < route.size() && route[end].type == Wire::WIRE_ROUTE_EDGE) ++end;
  // Repeated displacement can extend an existing hole in either direction.
  if (first > 0 && route[first - 1].type == Wire::WIRE_GAP) --first;
  if (end < route.size() && route[end].type == Wire::WIRE_GAP) ++end;
  // Do not freeze a disconnected Grounding at the end of the cut. Its LOCAL
  // endpoint remains leased; the affected branch can choose a different DST.
  if (end + 2 == route.size() && route[end].type == Wire::WIRE_CROSSBAR &&
      route[end].pos != 0 && route[end].jump < 0 &&
      sameCoord(route[end].from, route[end].to) &&
      route.back().type == Wire::WIRE_TILE_PIN && route.back().pin_dir == 0)
    ++end;
  if (first == 0 || end <= first || end >= route.size()) return first;
  const bool terminal_tail = end + 1 == route.size() &&
      route[end].type == Wire::WIRE_TILE_PIN && route[end].pin_dir == 0;
  for (size_t i = first; i < end; ++i) {
    const auto &w = route[i];
    if (w.type == Wire::WIRE_GAP) continue;
    if (w.type == Wire::WIRE_ROUTE_EDGE) continue;
    if (terminal_tail && i + 1 == end && w.type == Wire::WIRE_CROSSBAR &&
        w.pos != 0 && w.jump < 0 && sameCoord(w.from, w.to)) continue;
    if (w.type != Wire::WIRE_CROSSBAR || w.pos == 0 || w.jump < 0 ||
        w.dst < 0) return first;
  }
  const auto &left = route[first], &right = route[end - 1], &suffix = route[end];
  if (terminal_tail) {
    if (!sameCoord(right.to, suffix.from) ||
        (right.type == Wire::WIRE_GAP &&
         (right.to_node_type != CB_NODE_LOCAL || right.to_node != suffix.local))) return first;
    return end;
  }
  const int last_dst = right.type == Wire::WIRE_GAP || right.type == Wire::WIRE_ROUTE_EDGE
      ? right.to_node : right.dst;
  if ((left.type != Wire::WIRE_CROSSBAR && left.type != Wire::WIRE_GAP &&
       !(left.type == Wire::WIRE_ROUTE_EDGE && left.from_node_type == CB_NODE_DST)) ||
      (right.type != Wire::WIRE_CROSSBAR && right.type != Wire::WIRE_GAP &&
       !(right.type == Wire::WIRE_ROUTE_EDGE && right.to_node_type == CB_NODE_DST)) ||
      suffix.type != Wire::WIRE_CROSSBAR ||
      !sameCoord(right.to, suffix.from) || last_dst != suffix.local) return first;
  return end;
}

std::vector<fpga::RouteCutNode> fpga::previewNetRouteCuts(
    const std::vector<NetRouteRef> &victims, const std::vector<RouteCutNode> &cuts,
    bool preserve_downstream, OwnershipQueryWork *work) {
  struct Suffix { NetRouteRef ref; const std::vector<Wire> *route; size_t cut, end; };
  std::vector<Suffix> suffixes;
  for (const auto &ref : victims) {
    if (!ref.net || ref.binding_index >= ref.net->routes.size()) return {};
    const auto *route = bindingRoute(ref.net->routes[ref.binding_index]);
    if (!route || route->empty()) return {};
    size_t cut = routeNodeCut(*route, cuts);
    if (cut == route->size()) return {};
    const size_t end = preserve_downstream ? transitCutEnd(*route, cut, cuts) : route->size();
    if (end == cut) return {};
    suffixes.push_back({ref, route, cut, end});
  }
  std::vector<RouteCutNode> released;
  auto add = [&](Coord coord, CBNodeNameType type, int node) {
    Tile *tile = Device::current().getTile(coord.x, coord.y);
    if (!tile || node < 0) return;
    const RouteCutNode candidate{coord, type, node};
    if (std::any_of(released.begin(), released.end(), [&](const auto &old) {
          return sameCoord(old.tile, coord) && old.type == type && old.node == node;
        })) return;
    released.push_back(candidate);
  };
  for (const auto &suffix : suffixes) {
    const auto *route = suffix.route;
    const size_t cut = suffix.cut;
    const bool preserve_landing = cut > 0 &&
        (*route)[cut - 1].type == Wire::WIRE_CROSSBAR &&
        (*route)[cut].type == Wire::WIRE_CROSSBAR &&
        sameCoord((*route)[cut - 1].to, (*route)[cut].from) &&
        (*route)[cut - 1].dst == (*route)[cut].local;
    for (size_t i = cut; i < suffix.end; ++i) {
      const Wire &fragment = (*route)[i];
      if (fragment.type == Wire::WIRE_TILE_PIN) {
        if (i + 1 == route->size()) add(fragment.from, CB_NODE_LOCAL, fragment.local);
      } else if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
        add(fragment.to, static_cast<CBNodeNameType>(fragment.to_node_type), fragment.to_node);
      } else if (fragment.type == Wire::WIRE_CROSSBAR) {
        add(fragment.from, CB_NODE_SRC, fragment.jump);
        add(fragment.from, CB_NODE_JOINT, fragment.joint);
        add(fragment.from, CB_NODE_JOINT, fragment.joint2);
        if (fragment.pos == 0) add(fragment.from, CB_NODE_LOCAL, fragment.local);
        else if (fragment.owns_dst && !(preserve_landing && i == cut))
          add(fragment.from, CB_NODE_DST, fragment.local);
        if (fragment.owns_landing) add(fragment.to, CB_NODE_DST, fragment.dst);
      }
    }
  }
  // Check all proposed releases together. The old per-node query rescanned
  // the same Tile registrations and complete routes for every released SRC,
  // DST and JOINT. Only surviving fragments can prevent release: inspect each
  // live binding once, omitting precisely its selected cut interval.
  // This is a read-only, invocation-local calculation, never a cached policy.
  using Key = std::tuple<int, int, int, int>;
  auto key = [](Coord at, CBNodeNameType type, int node) {
    return Key{at.x, at.y, int(type), node};
  };
  std::map<Key, size_t> candidates;
  std::array<NodeMask, 5> requested;
  for (size_t i = 0; i < released.size(); ++i) {
    const auto &n = released[i];
    candidates.emplace(key(n.tile, n.type, n.node), i);
    if (unsigned(n.type) < requested.size()) requested[n.type].setBit(n.node);
  }
  std::vector<bool> retained(released.size());
  auto mark = [&](Coord at, CBNodeNameType type, int node) {
    if (node < 0 || unsigned(type) >= requested.size() || !requested[type].testBit(node)) return;
    const auto found = candidates.find(key(at, type, node));
    if (found != candidates.end()) retained[found->second] = true;
  };
  std::unordered_set<const rtl::NetRouteBinding *> inspected;
  auto inspect = [&](rtl::Net *net, size_t index) {
    if (!net || index >= net->routes.size()) return;
    auto &binding = net->routes[index];
    if (!inspected.insert(&binding).second) return;
    const auto *route = bindingRoute(binding);
    if (!route) return;
    const auto selected = std::find_if(suffixes.begin(), suffixes.end(), [&](const auto &s) {
      return s.ref.net == net && s.ref.binding_index == index;
    });
    for (size_t i = 0; i < route->size(); ++i) {
      if (selected != suffixes.end() && i >= selected->cut && i < selected->end) continue;
      if (work) ++work->fragment_visits;
      const auto &w = (*route)[i];
      // Match routeNodeCut's physical dependencies, including shared replicas,
      // non-owning landings and both endpoints of explicit graph edges.
      if (w.type == Wire::WIRE_ROUTE_EDGE) {
        mark(w.from, static_cast<CBNodeNameType>(w.from_node_type), w.from_node);
        mark(w.to, static_cast<CBNodeNameType>(w.to_node_type), w.to_node);
      } else if (w.type == Wire::WIRE_TILE_PIN) {
        mark(w.from, CB_NODE_LOCAL, w.local);
      } else if (w.type == Wire::WIRE_CROSSBAR) {
        mark(w.from, CB_NODE_SRC, w.jump);
        mark(w.from, w.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST, w.local);
        mark(w.to, CB_NODE_DST, w.dst);
        mark(w.from, CB_NODE_JOINT, w.joint);
        mark(w.from, CB_NODE_JOINT, w.joint2);
      }
    }
  };
  std::unordered_set<Tile *> visited_tiles;
  std::unordered_set<rtl::Net *> visited_fallback_nets;
  for (const auto &node : released) {
    auto *tile = Device::current().getTile(node.tile.x, node.tile.y);
    if (!visited_tiles.insert(tile).second) continue;
    if (work) ++work->queries;
    if (tile->routed_bindings_authoritative) {
      for (const auto &ref : tile->routed_bindings) if (ref.net) {
        if (work) ++work->binding_resolutions;
        inspect(ref.net, ref.net->findRouteBindingById(ref.route_id));
      }
    } else {
      for (const auto &ref : tile->routedNets)
        if (ref.peer && visited_fallback_nets.insert(ref.peer).second)
          for (size_t i = 0; i < ref.peer->routes.size(); ++i) {
            if (work) ++work->binding_resolutions;
            inspect(ref.peer, i);
          }
    }
  }
  size_t next = 0;
  for (size_t i = 0; i < released.size(); ++i)
    if (!retained[i]) released[next++] = released[i];
  released.resize(next);
  return released;
}

std::vector<fpga::RouteCutNode> fpga::previewNetRouteCut(
    rtl::Net &net, size_t binding_index, const std::vector<RouteCutNode> &cuts) {
  return previewNetRouteCuts({{&net, binding_index}}, cuts);
}

// Preemption cannot release the source endpoint or its first physical hop.
bool fpga::canPreemptNetRouteFromNodes(
    rtl::Net &net, size_t route_binding_index,
    const std::vector<RouteCutNode> &nodes) {
  if (!net.routeCanBePreempted() || route_binding_index >= net.routes.size() || nodes.empty()) {
    return false;
  }
  const std::vector<Wire> *route = bindingRoute(net.routes[route_binding_index]);
  if (!route || route->empty()) {
    return false;
  }
  const size_t cut = routeNodeCut(*route, nodes);
  for (size_t i = 0; i < cut; ++i) {
    const Wire &wire = (*route)[i];
    if (wire.type == Wire::WIRE_CROSSBAR && wire.jump >= 0 &&
        !sameCoord(wire.from, wire.to)) {
      return cut < route->size();
    }
  }
  return false;
}

// Cut before every requested resource while preserving a nonconflicting landing.
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
  const size_t cut = routeNodeCut(*route, nodes);
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
      sameCoord((*route)[cut - 1].to, removed.front().from) &&
      (*route)[cut - 1].dst == removed.front().local) {
    (*route)[cut - 1].owns_landing = true;
    removed.front().owns_dst = false;
  }
  route->resize(cut);
  clearRouteLeases(removed, true);
  rebuildNetRouteTiles(net, {removed});
  return true;
}

bool fpga::unrouteNetRoutesFromNodes(const std::vector<NetRouteRef> &victims,
                                    const std::vector<RouteCutNode> &cuts,
                                    bool preserve_downstream) {
  const auto dependent = findNetRouteCutVictims(cuts);
  if (victims.empty() || victims.size() != dependent.size()) return false;
  struct Removal { rtl::Net *net; std::vector<Wire> *route; size_t cut, end;
                   std::vector<Wire> removed; };
  std::vector<Removal> removals;
  for (const auto &ref : dependent) {
    if (std::none_of(victims.begin(), victims.end(), [&](const auto &v) {
          return v.net == ref.net && v.binding_index == ref.binding_index;
        }) || !canPreemptNetRouteFromNodes(*ref.net, ref.binding_index, cuts)) return false;
    auto *route = bindingRoute(ref.net->routes[ref.binding_index]);
    size_t cut = routeNodeCut(*route, cuts);
    const size_t end = preserve_downstream ? transitCutEnd(*route, cut, cuts) : route->size();
    if (end == cut) return false;
    removals.push_back({ref.net, route, cut, end,
        std::vector<Wire>(route->begin() + static_cast<std::ptrdiff_t>(cut),
                          route->begin() + static_cast<std::ptrdiff_t>(end))});
  }
  // Every check and suffix copy precedes mutation. Remove all claims before
  // releasing any shared resource, so binding enumeration order cannot matter.
  std::vector<std::unique_ptr<RouteHistoryScope>> history;
  if (!routeHistoryNet().empty() || routeHistoryWatch().node >= 0) {
    std::unordered_set<rtl::Net *> watched;
    for (const auto &r : removals)
      if (watched.insert(r.net).second)
        history.push_back(std::make_unique<RouteHistoryScope>(r.net, __func__));
  }
  for (auto &r : removals) {
    auto &previous = (*r.route)[r.cut - 1];
    auto &first = r.removed.front();
    const int first_dst = first.type == Wire::WIRE_ROUTE_EDGE && first.from_node_type == CB_NODE_DST
        ? first.from_node : first.local;
    if (previous.type == Wire::WIRE_CROSSBAR &&
        (first.type == Wire::WIRE_CROSSBAR || first.type == Wire::WIRE_ROUTE_EDGE) &&
        sameCoord(previous.to, first.from) && previous.dst == first_dst) {
      previous.owns_landing = true;
      first.owns_dst = false;
    }
    if (preserve_downstream) {
      Wire gap;
      gap.type = Wire::WIRE_GAP;
      gap.from = first.from;
      gap.to = r.removed.back().to;
      gap.from_node_type = gap.to_node_type = CB_NODE_DST;
      gap.from_node = first.type == Wire::WIRE_GAP || first.type == Wire::WIRE_ROUTE_EDGE
          ? first.from_node : first.local;
      gap.to_node = r.removed.back().type == Wire::WIRE_GAP || r.removed.back().type == Wire::WIRE_ROUTE_EDGE
          ? r.removed.back().to_node : r.removed.back().dst;
      if ((*r.route)[r.end].type == Wire::WIRE_TILE_PIN) {
        gap.to_node_type = CB_NODE_LOCAL;
        gap.to_node = (*r.route)[r.end].local;
      }
      // The retained tail must own its resources independently. Another
      // victim can have a LONGER cut, removing owners inside this tail, not
      // just at its first DST. In particular a retained FORK may borrow its
      // input from that other victim. Promoting only route[end] silently
      // releases the fork's DST during clearRouteLeases(). All conflicting
      // occurrences have already been removed by transitCutEnd; these are
      // genuine surviving same-source dependencies, including later islands.
      for (size_t i = r.end; i < r.route->size(); ++i) {
        auto &retained = (*r.route)[i];
        if (retained.type == Wire::WIRE_GAP) continue;
        retained.shared = false;
        if (retained.type == Wire::WIRE_CROSSBAR) {
          retained.owns_dst = retained.pos != 0;
          retained.owns_landing = retained.jump >= 0 && retained.dst >= 0;
        }
      }
      r.route->erase(r.route->begin() + r.cut, r.route->begin() + r.end);
      r.route->insert(r.route->begin() + r.cut, std::move(gap));
    } else {
      r.route->resize(r.cut);
    }
  }
  std::vector<const std::vector<Wire> *> removed_refs;
  for (const auto &r : removals) removed_refs.push_back(&r.removed);
  clearRouteLeases(removed_refs, true);
  std::unordered_map<rtl::Net *, std::vector<std::vector<Wire>>> affected;
  for (auto &r : removals) affected[r.net].push_back(std::move(r.removed));
  for (const auto &[net, routes] : affected) rebuildNetRouteTiles(*net, routes);
  return true;
}

namespace {
// Ownership alone is insufficient: shared fragments can reference a resource
// without owning its lease. Only the current binding's private terminal nodes
// may change input; the pin stays reserved and no other binding may use them.
// The caller limits this exception to the sink LOCAL and sink-Tile JOINTs.
bool canReplacePrivateTerminalInput(rtl::Net &net, size_t index, Tile &tile,
                               fpga::CBNodeNameType kind, int node,
                               const std::vector<fpga::NetRouteRef> &owners) {
  if (owners.empty() || std::any_of(owners.begin(),owners.end(),[&](const auto &o) {
        return o.net!=&net || o.binding_index!=index;
      })) return false;
  std::unordered_set<rtl::Net *> nets{&net};
  for (const auto &ref:tile.routedNets) if (ref.peer) nets.insert(ref.peer);
  for (const auto &ref:tile.routed_bindings) if (ref.net) nets.insert(ref.net);
  for (auto *n:nets) for (size_t i=0;i<n->routes.size();++i) {
    if (n==&net && i==index) continue;
    const auto *other=bindingRoute(n->routes[i]);
    if (other && routeUsesNodeOnTile(*other,tile,kind,node,false,false)) return false;
  }
  return true;
}
} // namespace

static bool auditRouteReachability(rtl::Net &net, size_t binding_index,
                                    const rtl::NetRouteBinding &binding,
                                    const std::vector<Wire> *route,
                                    std::ostream &out, bool preserve_inputs,
                                    size_t max_states) {
  if (!route || route->empty() || route->front().type != Wire::WIRE_TILE_PIN ||
      route->back().type != Wire::WIRE_TILE_PIN) return false;
  using Node = std::tuple<int, int, int, int>;
  auto node = [](Coord c, int kind, int id) { return Node{c.x,c.y,kind,id}; };
  auto &device = Device::current();
  const Node sink = node(route->back().from,CB_NODE_LOCAL,route->back().local);
  auto same_source = [&](const rtl::NetRouteBinding &b) {
    return rtl::sameRouteSource(b.from, b.from_port, binding.from, binding.from_port);
  };
  auto print_node = [&](const Node &n) {
    auto [x,y,kind,id] = n;
    out << "tile=" << x << ',' << y << " kind=" << kind << " node=" << id;
  };
  // Recover physical input arcs, including replicas and downstream islands.
  // A same-source busy bit is not permission to short two distinct inputs.
  std::map<Node, std::set<Node>> inputs;
  std::unordered_set<rtl::Net *> nets{&net};
  for (auto &tile : device.tile_grid)
    for (auto &ref : tile.routedNets) if (ref.peer) nets.insert(ref.peer);
  for (auto *n : nets) for (auto &b : n->routes) {
    if (!same_source(b)) continue;
    const auto *wires = bindingRoute(b);
    if (!wires) continue;
    for (size_t i = 0; i < wires->size(); ++i) {
      const auto &w = (*wires)[i];
      if (w.type == Wire::WIRE_ROUTE_EDGE) {
        inputs[node(w.to,w.to_node_type,w.to_node)].insert(
            node(w.from,w.from_node_type,w.from_node));
      } else if (w.type == Wire::WIRE_CROSSBAR) {
        Node previous = node(w.from,w.pos == 0 ? CB_NODE_LOCAL : CB_NODE_DST,w.local);
        auto arc = [&](Node next) { inputs[next].insert(previous); previous = next; };
        if (w.joint2 >= 0) arc(node(w.from,CB_NODE_JOINT,w.joint2));
        if (w.joint >= 0) arc(node(w.from,CB_NODE_JOINT,w.joint));
        if (w.jump >= 0) {
          arc(node(w.from,CB_NODE_SRC,w.jump));
          arc(node(w.to,CB_NODE_DST,w.dst));
        } else if (i + 1 < wires->size() && (*wires)[i+1].type == Wire::WIRE_TILE_PIN) {
          arc(node((*wires)[i+1].from,CB_NODE_LOCAL,(*wires)[i+1].local));
        }
      }
    }
  }
  struct Lease { bool busy=false, foreign=false, private_sink=false; std::string owners; };
  std::map<Node, Lease> leases;
  auto lease = [&](const Node &n) -> const Lease & {
    auto [it,inserted] = leases.try_emplace(n);
    if (!inserted) return it->second;
    auto [x,y,kind,id] = n;
    auto *tile = device.getTile(x,y);
    auto &l = it->second;
    if (!tile || !tile->cb_type || id < 0) { l.foreign=true; return l; }
    switch (kind) {
      case CB_NODE_LOCAL: l.busy=tile->cb.local.local.testBit(id) || tile->isPinNodeLeased(id); break;
      case CB_NODE_DST: l.busy=tile->cb.dst.jump.testBit(id); break;
      case CB_NODE_SRC: l.busy=tile->cb.src.jump.testBit(id); break;
      case CB_NODE_JOINT: l.busy=tile->cb.joint.jump.testBit(id); break;
      default: l.foreign=true; return l;
    }
    if (!l.busy) return l;
    const auto owners = findNetOwnersByNode(*tile,static_cast<CBNodeNameType>(kind),id);
    l.foreign = owners.empty();
    for (const auto &ref : owners) {
      if (!l.owners.empty()) l.owners += ';';
      l.owners += ref.net->name + ":" + std::to_string(ref.binding_index);
      l.foreign |= !same_source(ref.net->routes[ref.binding_index]);
    }
    if (binding_index < net.routes.size() && !l.foreign && (n==sink || (kind==CB_NODE_JOINT &&
        x==std::get<0>(sink) && y==std::get<1>(sink))))
      l.private_sink=canReplacePrivateTerminalInput(net,binding_index,*tile,
          static_cast<CBNodeNameType>(kind),id,owners);
    return l;
  };
  const Node source = node(route->front().from,CB_NODE_LOCAL,route->front().local);
  std::vector<Node> goals;
  for (const auto &w : *route) if (w.type == Wire::WIRE_GAP)
    goals.push_back(node(w.to,w.to_node_type,w.to_node));
  if (std::find(goals.begin(),goals.end(),sink) == goals.end()) goals.push_back(sink);
  struct Step { Node node; int parent; };
  std::vector<Step> steps{{source,-1}};
  std::map<Node,int> seen{{source,0}};
  std::vector<int> reached(goals.size(),-1);
  size_t foreign=0, conflicting_input=0, packed=0;
  bool limited=false;
  out << "SOURCE_REACH begin net=" << net.name << " binding=" << binding_index
      << " preserve_inputs=" << preserve_inputs << " max_states=" << max_states << '\n';
  out.flush();
  for (size_t i=0; i<steps.size(); ++i) {
    const Node current=steps[i].node;
    for (size_t g=0;g<goals.size();++g) if (current==goals[g]) reached[g]=i;
    if (std::all_of(reached.begin(),reached.end(),[](int r){return r>=0;})) break;
    auto [x,y,kind,id]=current;
    auto *tile=device.getTile(x,y);
    if (!tile || !tile->cb_type) continue;
    const auto &cb=*tile->cb_type; // const tables: no lazy insertions into topology
    auto add = [&](Node next) {
      const auto [nx,ny,nkind,nid]=next;
      if (nkind==CB_NODE_JOINT || (nkind==CB_NODE_LOCAL && next!=sink && next!=source)) {
        auto *at=device.getTile(nx,ny);
        if (!at || !at->cb_type) return;
        const bool terminal=nx==std::get<0>(sink) && ny==std::get<1>(sink);
        const auto reserved=packedInputJointReservations(*at,
            terminal?binding.to:nullptr,terminal?binding.to_port:std::string{});
        if ((nkind==CB_NODE_JOINT && reserved.testBit(nid)) ||
            (nkind==CB_NODE_LOCAL && at->input_local_reservations.count(nid))) {
          ++packed; return;
        }
      }
      const auto &l=lease(next);
      if (l.foreign) { ++foreign; return; }
      if (preserve_inputs && l.busy && !l.private_sink) {
        auto old=inputs.find(next);
        if (old!=inputs.end() && (old->second.size()!=1 || !old->second.count(current))) {
          ++conflicting_input; return;
        }
      }
      if (seen.count(next)) return;
      if (steps.size()>=max_states) { limited=true; return; }
      seen[next]=steps.size(); steps.push_back({next,static_cast<int>(i)});
    };
    auto mask = [&](const NodeMask &bits, int to_kind) {
      bits.for_each_set_bit([&](int to){add(Node{x,y,to_kind,to}); return false;});
    };
    switch (kind) {
      case CB_NODE_LOCAL:
        mask(cb.local_src[id].jump,CB_NODE_SRC);
        mask(cb.local_joint[id].joint,CB_NODE_JOINT);
        mask(cb.local_local[id].local,CB_NODE_LOCAL);
        for (const auto &t : device.resolveLocalTargets(*tile,id))
          if (t.tile) add(node(t.tile->coord,t.node_type,t.node));
        break;
      case CB_NODE_DST:
        mask(cb.dst_src[id].jump,CB_NODE_SRC);
        mask(cb.dst_joint[id].joint,CB_NODE_JOINT);
        mask(cb.dst_local[id].local,CB_NODE_LOCAL);
        break;
      case CB_NODE_JOINT:
        mask(cb.joint_src[id].jump,CB_NODE_SRC);
        mask(cb.joint_joint[id].joint,CB_NODE_JOINT);
        mask(cb.joint_local[id].local,CB_NODE_LOCAL);
        break;
      case CB_NODE_SRC:
        mask(cb.src_joint[id].joint,CB_NODE_JOINT);
        for (const auto &t : device.resolveJumpTargets(*tile,id))
          if (t.tile && t.dst_node>=0) add(node(t.tile->coord,CB_NODE_DST,t.dst_node));
        break;
    }
  }
  out << "SOURCE_REACH end states=" << steps.size() << " limited=" << limited
      << " foreign_rejections=" << foreign << " input_rejections=" << conflicting_input
      << " packed_rejections=" << packed << '\n';
  // A bounded router's frontier is not the whole free component. Preserve
  // witnesses from this independent audit before its temporary state goes
  // away, so a failed sink does not hide reachable Grounding approaches.
  int closest = 0, closest_distance = std::numeric_limits<int>::max();
  std::map<std::pair<int,int>, std::pair<size_t,int>> region;
  for (size_t i=0;i<steps.size();++i) {
    const auto [x,y,kind,id]=steps[i].node;
    const int distance=std::max(std::abs(x-std::get<0>(sink)),
                                std::abs(y-std::get<1>(sink)));
    if (distance<closest_distance) { closest_distance=distance; closest=i; }
    if (distance<=GROUNDING_RADIUS) {
      auto [it,inserted]=region.try_emplace({x,y},std::pair<size_t,int>{0,int(i)});
      ++it->second.first;
      // Prefer a reached DST over an internal bridge as the Tile witness.
      if (kind==CB_NODE_DST && std::get<2>(steps[it->second.second].node)!=CB_NODE_DST)
        it->second.second=i;
    }
  }
  out << "SOURCE_CLOSEST distance=" << closest_distance << ' ';
  print_node(steps[closest].node); out << " grounding_tiles=" << region.size() << '\n';
  auto witness=[&](int end, const char *tag) {
    std::vector<int> path;
    for (int p=end;p>=0;p=steps[p].parent) path.push_back(p);
    std::reverse(path.begin(),path.end());
    for (size_t p=0;p<path.size();++p) {
      out << tag << " step=" << p << ' '; print_node(steps[path[p]].node); out << '\n';
    }
  };
  witness(closest,"SOURCE_CLOSEST_PATH");
  for (const auto &[coord,entry]:region) {
    out << "SOURCE_REGION tile=" << coord.first << ',' << coord.second
        << " states=" << entry.first << " witness=" << entry.second << '\n';
    witness(entry.second,"SOURCE_REGION_PATH");
  }
  for (size_t g=0;g<goals.size();++g) {
    out << "SOURCE_GOAL goal=" << g << " reached=" << (reached[g]>=0) << ' ';
    print_node(goals[g]); out << '\n';
    if (reached[g]<0) continue;
    std::vector<int> path;
    for (int p=reached[g];p>=0;p=steps[p].parent) path.push_back(p);
    std::reverse(path.begin(),path.end());
    for (size_t p=0;p<path.size();++p) {
      const auto &n=steps[path[p]].node;
      const auto &l=lease(n);
      out << "SOURCE_PATH goal=" << g << " step=" << p << ' '; print_node(n);
      out << " busy=" << l.busy << " foreign=" << l.foreign << " owners=" << l.owners << '\n';
    }
  }
  out.flush();
  for (size_t g=0;g<goals.size();++g)
    if (goals[g]==sink && reached[g]>=0) return true;
  return false;
}

bool fpga::auditNetRouteReachability(rtl::Net &net, size_t binding_index,
                                    std::ostream &out, bool preserve_inputs,
                                    size_t max_states) {
  if (binding_index >= net.routes.size()) return false;
  auto &binding = net.routes[binding_index];
  return auditRouteReachability(net, binding_index, binding, bindingRoute(binding),
                                out, preserve_inputs, max_states);
}

bool fpga::auditUnroutedReachability(rtl::Net &net,
                                    const rtl::NetRouteBinding &endpoints,
                                    const Wire &source, const Wire &sink,
                                    std::ostream &out, size_t max_states) {
  // Diagnostic endpoints are not registered or leased. In particular, do not
  // append a fake binding to the live net just to inspect an unrouted task.
  const std::vector<Wire> route{source, sink};
  return auditRouteReachability(net, std::numeric_limits<size_t>::max(),
                                endpoints, &route, out, true, max_states);
}

namespace {
using RepairNode = std::tuple<int, int, int, int>;

// Read the active trace at each decision so a deadline window can start mid-search.
void traceRepairDecision(std::string_view phase, const RepairNode& from,
                         const RepairNode& to, std::string_view result) {
  auto* trace = fpga::RouteSearchTrace::current;
  if (!trace) return;
  auto print = [&](const RepairNode& node) {
    const auto [x,y,kind,id] = node;
    trace->stream() << '(' << x << ',' << y << ")/" << kind << ':' << id;
  };
  trace->stream() << "REPAIR_DECISION phase=" << phase << " from=";
  print(from); trace->stream() << " to="; print(to);
  trace->stream() << " result=" << result << '\n';
  for (const auto& node : {from,to}) {
    auto* tile = fpga::Device::current().getTile(std::get<0>(node),std::get<1>(node));
    if (tile) trace->snapshot(*tile);
  }
}
// Search labels are looked up by exact numeric identity, never key order.
struct RepairNodeHash {
  size_t operator()(const RepairNode &node) const noexcept {
    const auto [x,y,kind,id]=node;
    size_t hash=0;
    for(int value:{x,y,kind,id})
      hash^=std::hash<int>{}(value)+size_t{0x9e3779b9}+(hash<<6)+(hash>>2);
    return hash;
  }
};
// Dense lookup storage for numeric search labels. No iteration order enters
// the search: its queue and stable label indices determine expansion order.
// Use the tuple hash's full mixing; the old integer-combine hash clusters
// poorly in a flat table for neighbouring routing coordinates and node IDs.
using RepairLabelMap = absl::flat_hash_map<RepairNode,int>;
// Free-only failure leaves immutable numeric labels for the later preemption
// phase of this SAME request. No routes or leases are retained in this object.
struct RepairFreeFrontier {
  struct Step { RepairNode node; int parent,hops,cost; };
  std::vector<Step> steps;
  RepairLabelMap best;
  bool budget_limited=false;
};
// One immutable request's exhausted free searches, never committed leases.
// Bound both labels and entries; different pin pairs and policies stay separate.
struct RepairFreeCache {
  using Key=std::tuple<RepairNode,RepairNode,size_t,int,bool,int>;
  struct Entry { Key key; RepairFreeFrontier frontier; };
  static constexpr size_t label_limit=262144, entry_limit=16;
  std::vector<Entry> entries;
  size_t labels=0;
  uint64_t revision=routeMutationRevision();
  void clear() { entries.clear(); labels=0; revision=routeMutationRevision(); }
  const RepairFreeFrontier* get(const Key &key) {
    if(revision!=routeMutationRevision()) clear();
    for(auto it=entries.begin();it!=entries.end();++it) if(it->key==key) {
      std::rotate(it,it+1,entries.end());
      return &entries.back().frontier;
    }
    return nullptr;
  }
  void put(const Key &key,RepairFreeFrontier frontier) {
    const size_t count=frontier.steps.size();
    if(count>label_limit || !count) return;
    while(!entries.empty() && (labels+count>label_limit || entries.size()>=entry_limit)) {
      labels-=entries.front().frontier.steps.size(); entries.erase(entries.begin());
    }
    entries.push_back({key,std::move(frontier)}); labels+=count;
  }
};
// Exclusive phase timing and per-call numeric counters. The overlap map is
// temporary diagnostic memory, never an eligibility filter or routing cache.
struct RepairPhaseTimer {
  static thread_local RepairPhaseTimer *current;
  static thread_local uint64_t sequence;
  bool enabled;
  const rtl::Net &net;
  size_t binding;
  const char *phase;
  bool gap, success=false, cancelled=false;
  RepairPhaseTimer *parent=nullptr;
  uint64_t id=0, revision=0, route_id=0;
  std::string route_name;
  RepairStatistics stats;
  std::unordered_map<RepairNode,uint8_t,RepairNodeHash> expanded_nodes;
  std::chrono::steady_clock::time_point start, began;
  RepairPhaseTimer(const rtl::Net &n,size_t b,const char *p,
                   const rtl::NetRouteBinding *unbound=nullptr,bool observe=true)
      :enabled(observe && std::getenv("SCALEPNR_PROFILE_REPAIR") != nullptr),
       net(n),binding(b),phase(p),gap(std::string_view(p)=="exact_gap_search") {
    if(!enabled) return;
    began=start=std::chrono::steady_clock::now();
    parent=current;
    if(parent) parent->pause(start);
    current=this; id=++sequence; revision=routeMutationRevision();
    const auto *bnd=unbound ? unbound : b<n.routes.size() ? &n.routes[b] : nullptr;
    if(bnd) { route_id=bnd->route_id; route_name=bnd->route_name; }
    add(gap?RepairCounter::gap_calls:RepairCounter::source_calls);
  }
  void add(RepairCounter counter,uint64_t amount=1) {
    if(enabled) stats.counters[unsigned(counter)]+=amount;
  }
  void expand(const RepairNode &node) {
    if(!enabled) return;
    add(gap?RepairCounter::gap_expanded:RepairCounter::source_expanded);
    auto &bits=expanded_nodes[node];
    const uint8_t own=gap?1:2;
    const bool fresh=!(bits&own);
    add(gap ? (fresh?RepairCounter::gap_unique:RepairCounter::gap_reexpanded)
            : (fresh?RepairCounter::source_unique:RepairCounter::source_reexpanded));
    if(gap && fresh && (bits&2)) add(RepairCounter::gap_source_overlap);
    bits|=own;
    if(!gap && fresh && parent && parent->gap) {
      auto &shared=parent->expanded_nodes[node];
      if(shared&2) parent->add(RepairCounter::source_cross_call_repeat);
      else if(shared&1) parent->add(RepairCounter::gap_source_overlap);
      shared|=2;
    }
  }
  void pause(std::chrono::steady_clock::time_point now) {
    const auto found=std::find(repairPhaseNames.begin(),repairPhaseNames.end(),phase);
    if(found!=repairPhaseNames.end())
      stats.seconds[found-repairPhaseNames.begin()]+=std::chrono::duration<double>(now-start).count();
  }
  void next(const char *p) {
    if(enabled) { const auto now=std::chrono::steady_clock::now(); pause(now); start=now; }
    phase=p;
  }
  ~RepairPhaseTimer() {
    if(!enabled) return;
    const auto end=std::chrono::steady_clock::now(); pause(end);
    if(cancelled) add(RepairCounter::cancelled);
    if(success) add(gap?RepairCounter::gap_success:RepairCounter::source_success);
    for(size_t i=0;i<stats.counters.size();++i) repair_statistics.counters[i]+=stats.counters[i];
    for(size_t i=0;i<stats.seconds.size();++i) repair_statistics.seconds[i]+=stats.seconds[i];
    std::ostringstream out;
    out << "REPAIR_CALL id=" << id << " parent=" << (parent?parent->id:0)
        << " kind=" << (gap?"gap":"source") << " net=" << std::quoted(net.name)
        << " route=" << std::quoted(route_name) << " binding=" << binding << " route_id=" << route_id
        << " revision=" << revision << " end_revision=" << routeMutationRevision()
        << " result=" << (success?"success":cancelled?"cancelled":"failed")
        << " wall_s=" << std::chrono::duration<double>(end-began).count();
    for(size_t i=0;i<stats.counters.size();++i)
      if(stats.counters[i]) out << ' ' << repairCounterNames[i] << '=' << stats.counters[i];
    for(size_t i=0;i<stats.seconds.size();++i)
      if(stats.seconds[i]) out << ' ' << repairPhaseNames[i] << "_s=" << stats.seconds[i];
    out << '\n'; const auto line=out.str(); std::fwrite(line.data(),1,line.size(),stderr);
    current=parent;
    if(parent) parent->start=std::chrono::steady_clock::now();
  }
};
thread_local RepairPhaseTimer *RepairPhaseTimer::current=nullptr;
thread_local uint64_t RepairPhaseTimer::sequence=0;
RepairNode repairNode(Coord at, int type, int node) {
  return {at.x,at.y,type,node};
}
Coord repairCoord(const RepairNode &node) { return {std::get<0>(node),std::get<1>(node)}; }

// Numeric expansion needs one lease bit, not a copy of the 4096-bit mask.
bool repairNodeLeased(const CBState &state,int kind,int id) {
  switch(kind) {
    case CB_NODE_SRC:return state.src.jump.testBit(id);
    case CB_NODE_DST:return state.dst.jump.testBit(id);
    case CB_NODE_JOINT:return state.joint.jump.testBit(id);
    case CB_NODE_LOCAL:return state.local.local.testBit(id);
    default:return false;
  }
}

// Read the current Tile's bindings once for this expanded incoming DST.
// Results live only until the next expansion; no occupancy survives a commit
// or a later arrival at this Tile. Keep the last matching fragment just as the
// per-SRC scan did, including canonical aliases and real SRC ownership.
using SharedRepairEdges = std::array<const Wire *, CB_MAX_NODES>;
void collectSharedRepairEdges(Tile &tile,int dst,
                              const rtl::NetRouteBinding &source,
                              NodeMask &found,SharedRepairEdges &edges) {
  found = {};
  auto *observations=RepairPhaseTimer::current;
  auto inspect=[&](rtl::Net &net,size_t index) {
    if(observations) observations->add(RepairCounter::shared_binding_visits);
    if(index>=net.routes.size()) return;
    auto &b=net.routes[index];
    if(observations) observations->add(RepairCounter::shared_source_checks);
    if(!rtl::sameRouteSource(b.from,b.from_port,source.from,source.from_port)) return;
    const auto *route=bindingRoute(b);
    if(!route) return;
    for(const auto &w:*route) {
      if(observations) observations->add(RepairCounter::shared_fragment_checks);
      if(w.type!=Wire::WIRE_CROSSBAR || w.pos==0 || w.local!=dst ||
          w.jump<0 || w.jump>=CB_MAX_NODES || !sameCoord(w.from,tile.coord)) continue;
      if(!routeUsesNodeOnTile(*route,tile,CB_NODE_SRC,w.jump,false,true)) continue;
      found.setBit(w.jump);
      edges[w.jump]=&w;
    }
  };
  if(tile.routed_bindings_authoritative) {
    for(const auto &ref:tile.routed_bindings)
      if(ref.net) inspect(*ref.net,ref.net->findRouteBindingById(ref.route_id));
  } else {
    for(const auto &ref:tile.routedNets) if(ref.peer)
      for(size_t i=0;i<ref.peer->routes.size();++i) inspect(*ref.peer,i);
  }
}

// The configured cap bounds fast free probes, not a later explicit-budget retry.
size_t sourceFreeStateLimit(size_t max_states) {
  if (const char *value=std::getenv("SCALEPNR_FREE_SEARCH_STATES")) {
    const std::string_view setting(value);
    size_t requested=0;
    const auto parsed=std::from_chars(setting.data(),setting.data()+setting.size(),requested);
    if (parsed.ec!=std::errc{} || parsed.ptr!=setting.data()+setting.size() || !requested)
      throw std::runtime_error("SCALEPNR_FREE_SEARCH_STATES must be a positive integer");
    return std::min(max_states,requested);
  }
  return max_states;
}

// A failed exact hole can have a trapped boundary even though another branch
// reaches the sink. Search from the real driver, never from a detached island.
// Used for a bounded free-only probe and after exact-gap search fails.
bool repairFromSource(rtl::Net &net, size_t index,
                      const std::function<bool()> &cancel,
                      size_t max_states, int max_hops,
                      const fpga::RepairGroundingPreemption &grounding_preemption,
                      const rtl::NetRouteBinding *unbound = nullptr,
                      const std::vector<Wire> *endpoints = nullptr,
                      bool existing_takeoff_only = false,
                      bool diagnostic_only = false, int cost_scale = 1,
                      std::ostream *diagnostic_log = nullptr,
                      bool diagnostic_frontier = false,
                      RepairFreeFrontier *free_frontier = nullptr,
                      bool fast_free_probe = true,
                      const RepairIncomingSources &incoming_sources = {},
                      bool *budget_limited = nullptr,
                      RepairFreeCache *retry_cache = nullptr,
                      const RouteCutNode *required_branch = nullptr,
                      bool preview_relocated_source = false) {
  RepairPhaseTimer timer(net,index,"source_setup",unbound,!diagnostic_only);
  const std::function<bool()> observed_cancel=timer.enabled && cancel ? std::function<bool()>([&] {
    const bool stop=cancel(); timer.cancelled|=stop; return stop;
  }) : std::function<bool()>{};
  const auto &cancelled=observed_cancel ? observed_cancel : cancel;
  const auto &binding=unbound ? *unbound : net.routes[index];
  const auto *route=endpoints ? endpoints : bindingRoute(net.routes[index]);
  if (!route || route->size()<2 || !max_states || max_hops<0 ||
      route->front().type!=Wire::WIRE_TILE_PIN ||
      route->back().type!=Wire::WIRE_TILE_PIN || (cancelled && cancelled())) return false;
  // Limit only free-source labels; terminal preemption keeps its own budget.
  // Failure diagnostics retain the caller's limit to distinguish budget exhaustion.
  const size_t free_states=!diagnostic_only && fast_free_probe
      ? sourceFreeStateLimit(max_states) : max_states;
  auto &device=Device::current();
  const auto source=repairNode(route->front().from,CB_NODE_LOCAL,route->front().local);
  const auto sink=repairNode(route->back().from,CB_NODE_LOCAL,route->back().local);
  const char *audit_filter=std::getenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY");
  const bool trace=diagnostic_log || (audit_filter && *audit_filter && net.name.find(audit_filter)!=std::string::npos);
  const bool trace_frontier=trace && (!diagnostic_only || diagnostic_frontier);
  const bool debug_repair=std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR")!=nullptr;
  std::ofstream search_file;
  std::ostream &search_log=diagnostic_log?*diagnostic_log:search_file;
  if (trace) {
    if (!diagnostic_log) {
      const char *path=std::getenv("SCALEPNR_SOURCE_REACHABILITY_LOG");
      search_file.open(path && *path?path:"source_reachability.log",std::ios::app);
    }
    search_log << "SOURCE_SEARCH begin net=" << net.name << " binding=" << index
        << " diagnostic_only=" << diagnostic_only << " max_states=" << max_states
        << " free_states=" << free_states << " cost_scale=" << cost_scale << '\n';
  }
  auto same_source=[&](const rtl::NetRouteBinding &other) {
    return rtl::sameRouteSource(other.from,other.from_port,binding.from,binding.from_port);
  };
  // Source-tile registrations contain the source tree, including aliases.
  // No device-wide scan or persistent search cache is needed.
  std::unordered_set<rtl::Net *> nets{&net};
  auto *source_tile=device.getTile(std::get<0>(source),std::get<1>(source));
  if (!source_tile) return false;
  for (auto &ref:source_tile->routedNets) if (ref.peer) nets.insert(ref.peer);
  std::map<RepairNode,std::set<RepairNode>> inputs;
  // An interrupted physical input can have several RTL consumers. Replace
  // those bindings together if a new path changes an incoming arc in their
  // disconnected tails. Never repoint a resource used by a live prefix, a
  // completed route, or a different destination's retained tail.
  std::vector<NetRouteRef> terminal_group;
  std::set<RepairNode> detached, protected_nodes;
  for (auto *n:nets) for (size_t bi=0;bi<n->routes.size();++bi) {
    auto &b=n->routes[bi];
    if (!same_source(b)) continue;
    const auto *old=bindingRoute(b);
    if (!old) continue;
    size_t last_gap=old->size();
    for (size_t i=0;i<old->size();++i)
      if ((*old)[i].type==Wire::WIRE_GAP) last_gap=i;
    const bool replaceable=!unbound && last_gap<old->size() && old->size()>=2 &&
        old->front().type==Wire::WIRE_TILE_PIN && old->back().type==Wire::WIRE_TILE_PIN &&
        repairNode(old->front().from,CB_NODE_LOCAL,old->front().local)==source &&
        repairNode(old->back().from,CB_NODE_LOCAL,old->back().local)==sink;
    if (replaceable) terminal_group.push_back({n,bi});
    for (size_t i=0;i<old->size();++i) {
      const auto &w=(*old)[i];
      auto mark=[&](const RepairNode &node) {
        (replaceable && i>last_gap?detached:protected_nodes).insert(node);
      };
      if (w.type==Wire::WIRE_ROUTE_EDGE) {
        const auto from=repairNode(w.from,w.from_node_type,w.from_node);
        const auto to=repairNode(w.to,w.to_node_type,w.to_node);
        inputs[to].insert(from); mark(from); mark(to);
      } else if (w.type==Wire::WIRE_CROSSBAR) {
        auto previous=repairNode(w.from,w.pos==0?CB_NODE_LOCAL:CB_NODE_DST,w.local);
        mark(previous);
        auto add=[&](RepairNode next) { inputs[next].insert(previous); mark(next); previous=next; };
        if (w.joint2>=0) add(repairNode(w.from,CB_NODE_JOINT,w.joint2));
        if (w.joint>=0) add(repairNode(w.from,CB_NODE_JOINT,w.joint));
        if (w.jump>=0) {
          add(repairNode(w.from,CB_NODE_SRC,w.jump));
          add(repairNode(w.to,CB_NODE_DST,w.dst));
        } else if (i+1<old->size() && (*old)[i+1].type==Wire::WIRE_TILE_PIN) {
          add(repairNode((*old)[i+1].from,CB_NODE_LOCAL,(*old)[i+1].local));
        }
      } else if (w.type==Wire::WIRE_TILE_PIN) {
        mark(repairNode(w.from,CB_NODE_LOCAL,w.local));
      }
    }
  }
  for (const auto &node:protected_nodes) detached.erase(node);
  size_t terminal_leaf_rejections=0;
  // A terminal pin may be hidden behind a short JOINT/LOCAL chain. Prove
  // these dead branches locally instead of spending frontier slots on each
  // intermediary. An unresolved/deeper branch stays eligible; this is not
  // a congestion heuristic and never rejects a potentially useful exit.
  auto terminal_branch=[&](const RepairNode &start) {
    std::array<RepairNode,8> visiting;
    auto continues=[&](auto &&self,const RepairNode &node,size_t depth)->bool {
      if(node==source || node==sink) return true;
      for(size_t i=0;i<depth;++i) if(visiting[i]==node) return false;
      if(depth==visiting.size()) return true;
      const auto [x,y,kind,id]=node;
      auto *at=device.getTile(x,y);
      if(!at || !at->cb_type || id<0 || id>=CB_MAX_NODES) return false;
      if(kind!=CB_NODE_LOCAL && kind!=CB_NODE_JOINT) return true;
      visiting[depth]=node;
      const auto &cb=*at->cb_type;
      auto follow=[&](const NodeMask &bits,int next_kind) {
        return bits.for_each_set_bit([&](int next) {
          return self(self,RepairNode{x,y,next_kind,next},depth+1);
        });
      };
      if(kind==CB_NODE_JOINT) {
        return cb.joint_src[id].jump.firstSetBit()>=0 ||
            follow(cb.joint_joint[id].joint,CB_NODE_JOINT) ||
            follow(cb.joint_local[id].local,CB_NODE_LOCAL);
      }
      if(cb.local_src[id].jump.firstSetBit()>=0 ||
          follow(cb.local_joint[id].joint,CB_NODE_JOINT) ||
          follow(cb.local_local[id].local,CB_NODE_LOCAL)) return true;
      for(const auto &target:device.resolveLocalTargets(*at,id))
        if(target.tile && self(self,repairNode(target.tile->coord,target.node_type,target.node),depth+1))
          return true;
      return false;
    };
    return !continues(continues,start,0);
  };
  RouteOwnerQueries owner_queries;
  auto allowed=[&](const RepairNode &from,const RepairNode &to,
                   const std::vector<fpga::RouteCutNode> &released = std::vector<fpga::RouteCutNode>{}) {
    auto decision = [&](bool ok, std::string_view reason) {
      if (auto* trace = RouteSearchTrace::current)
        trace->stream() << "ELIGIBILITY accepted=" << ok << '\n';
      traceRepairDecision("source_eligibility", from, to, reason);
      return ok;
    };
    auto [x,y,kind,id]=to;
    auto *tile=device.getTile(x,y);
    if (!tile || !tile->cb_type || id<0 || id>=CB_MAX_NODES) return decision(false,"invalid_node");
    bool busy=repairNodeLeased(tile->cb,kind,id);
    if (kind==CB_NODE_LOCAL) busy|=tile->isPinNodeLeased(id);
    // Reject an unrelated occupied node before doing structural lookahead.
    // Explicit cut previews and same-source resources still use all checks.
    if (busy && released.empty() && !protected_nodes.contains(to) && !detached.contains(to))
      return decision(false,"foreign_busy");
    if ((kind==CB_NODE_LOCAL || kind==CB_NODE_JOINT) && to!=sink && to!=source) {
      // An unrelated terminal pin with no outgoing physical arc cannot
      // complete this connection. Do not let these leaves consume the
      // bounded search frontier. Internal LOCAL bridges and dedicated
      // cross-Tile LOCAL links must remain searchable; this is topology,
      // not a congestion/deadend heuristic or an input-pin name check.
      if (terminal_branch(to)) {
        // CB templates may require a neighbour type absent at this Tile.
        // Only a resolved physical continuation can make this leaf useful.
        ++terminal_leaf_rejections;
        return decision(false,"terminal_leaf");
      }
    }
    if (kind==CB_NODE_JOINT || (kind==CB_NODE_LOCAL && to!=sink && to!=source)) {
      const bool terminal_tile=sameCoord(tile->coord,repairCoord(sink));
      // This also initializes the placement-owned LOCAL reservations. Protect
      // packed inputs on intermediate tiles, not only at the final sink.
      const auto reserved=packedInputJointReservations(*tile,
          terminal_tile?binding.to:nullptr,terminal_tile?binding.to_port:std::string{});
      if ((kind==CB_NODE_JOINT && reserved.testBit(id)) ||
          (kind==CB_NODE_LOCAL && tile->input_local_reservations.count(id))) return decision(false,"packed_reservation");
    }
    if (busy) {
      auto provisionally_released=[&] {
        return std::any_of(released.begin(),released.end(),[&](const auto &cut) {
          return sameCoord(cut.tile,tile->coord) && cut.type==kind && cut.node==id;
        });
      };
      // These two sets already describe every node in the same-source
      // fragments above. An absent busy node cannot be a reusable source
      // resource. Do not scan unrelated owners just to rediscover that fact.
      // Explicit provisional cuts still go through full preemption validation.
      if(!protected_nodes.contains(to) && !detached.contains(to))
        return decision(provisionally_released(),"provisional_cut_check");
      const bool terminal_input=to==sink || (kind==CB_NODE_JOINT &&
          sameCoord(tile->coord,repairCoord(sink)));
      std::vector<NetRouteRef> terminal_owners;
      if (debug_repair) std::fprintf(stderr,"REPAIR verify_source_owner=(%d,%d)/%d:%d\n",x,y,kind,id);
      timer.add(RepairCounter::owner_checks);
      if (!owner_queries.allMatch(*tile,static_cast<CBNodeNameType>(kind),id,[&](const auto &o) {
            if (!same_source(o.net->routes[o.binding_index])) return false;
            if (terminal_input) terminal_owners.push_back(o);
            return true;
          })) return decision(provisionally_released(),"owner_mismatch_cut_check");
      const auto old=inputs.find(to);
      if (!preview_relocated_source && old!=inputs.end() &&
          (old->second.size()!=1 || !old->second.count(from))) {
        // The detached suffix may be rejoined at a SRC as well as at its
        // terminal JOINT/LOCAL. This is provisional: all affected consumers
        // are replaced only after a complete source-to-input path is valid.
        if (!detached.contains(to) && (unbound || !terminal_input ||
            !canReplacePrivateTerminalInput(net,index,*tile,
                static_cast<CBNodeNameType>(kind),id,terminal_owners))) return decision(false,"different_incoming_arc");
      }
    }
    return decision(true,"eligible");
  };
  auto *sink_tile=device.getTile(std::get<0>(sink),std::get<1>(sink));
  if (!sink_tile || !sink_tile->cb_type) return false;
  // Destination Moving has not invalidated the moved source tree yet. Its
  // owned resources will be released at commit, including the old sink pin.
  // The preview may reuse/repoint those nodes, but still checks every live
  // owner in allowed(); unrelated routes and candidate input leases stay busy.
  if (unbound && !(preview_relocated_source && protected_nodes.contains(sink)) &&
                (sink_tile->isPinNodeLeased(std::get<3>(sink)) ||
                 sink_tile->cb.local.local.testBit(std::get<3>(sink)))) return false;
  // Enumerate physical edges identically for fabric search and retained-tail
  // validation. A stored fragment alone is not proof of physical continuity.
  auto successors=[&](const RepairNode &node,auto &&visit) {
    const auto [x,y,kind,id]=node;
    auto *tile=device.getTile(x,y);
    if (!tile || !tile->cb_type || id<0 || id>=CB_MAX_NODES) return;
    const auto &cb=*tile->cb_type;
    auto mask=[&](const NodeMask &bits,int type) {
      bits.for_each_set_bit([&](int n) { visit(RepairNode{x,y,type,n}); return false; });
    };
    switch (kind) {
      case CB_NODE_LOCAL:
        mask(cb.local_src[id].jump,CB_NODE_SRC);
        mask(cb.local_joint[id].joint,CB_NODE_JOINT);
        mask(cb.local_local[id].local,CB_NODE_LOCAL);
        for (const auto &t:device.resolveLocalTargets(*tile,id))
          if (t.tile) visit(repairNode(t.tile->coord,t.node_type,t.node));
        break;
      case CB_NODE_DST:
        mask(cb.dst_src[id].jump,CB_NODE_SRC);
        mask(cb.dst_joint[id].joint,CB_NODE_JOINT);
        mask(cb.dst_local[id].local,CB_NODE_LOCAL);
        break;
      case CB_NODE_JOINT:
        mask(cb.joint_src[id].jump,CB_NODE_SRC);
        mask(cb.joint_joint[id].joint,CB_NODE_JOINT);
        mask(cb.joint_local[id].local,CB_NODE_LOCAL);
        break;
      case CB_NODE_SRC:
        mask(cb.src_joint[id].joint,CB_NODE_JOINT);
        for (const auto &t:device.resolveJumpTargets(*tile,id))
          if (t.tile && t.dst_node>=0) visit(repairNode(t.tile->coord,CB_NODE_DST,t.dst_node));
        break;
    }
  };
  timer.next("source_free_search");
  // Only the contiguous tail after the LAST gap can complete this binding.
  // Decode it once, locally to this repair; do not search from detached islands.
  size_t tail_begin=route->size();
  for (size_t i=0;i<route->size();++i)
    if ((*route)[i].type==Wire::WIRE_GAP) tail_begin=i;
  std::vector<RepairNode> suffix;
  if (tail_begin<route->size()) {
    const auto &gap=(*route)[tail_begin];
    suffix.push_back(repairNode(gap.to,gap.to_node_type,gap.to_node));
    bool valid=true;
    auto append=[&](RepairNode from,RepairNode to) {
      bool connected=false;
      successors(from,[&](const RepairNode &next) { connected|=next==to; });
      if (suffix.back()!=from || !connected) valid=false;
      suffix.push_back(to);
    };
    for (size_t i=tail_begin+1;i<route->size() && valid;++i) {
      const auto &w=(*route)[i];
      if (w.type==Wire::WIRE_ROUTE_EDGE) {
        append(repairNode(w.from,w.from_node_type,w.from_node),
               repairNode(w.to,w.to_node_type,w.to_node));
      } else if (w.type==Wire::WIRE_CROSSBAR) {
        auto previous=repairNode(w.from,w.pos==0?CB_NODE_LOCAL:CB_NODE_DST,w.local);
        auto add=[&](RepairNode next) { append(previous,next); previous=next; };
        if (w.joint2>=0) add(repairNode(w.from,CB_NODE_JOINT,w.joint2));
        if (w.joint>=0) add(repairNode(w.from,CB_NODE_JOINT,w.joint));
        if (w.jump>=0) {
          add(repairNode(w.from,CB_NODE_SRC,w.jump));
          add(repairNode(w.to,CB_NODE_DST,w.dst));
        } else if (i+2==route->size()) add(sink);
        else valid=false;
      } else if (w.type!=Wire::WIRE_TILE_PIN || i+1!=route->size() || suffix.back()!=sink) {
        valid=false;
      }
    }
    if (!valid || suffix.back()!=sink) suffix.clear();
  }
  using Step = RepairFreeFrontier::Step;
  // Do not cache two-ended, diagnostic, or explicit uncapped-budget searches.
  const bool cacheable=retry_cache && !diagnostic_only && fast_free_probe &&
      !incoming_sources && free_states<=RepairFreeCache::label_limit;
  const RepairFreeCache::Key cache_key{source,sink,free_states,max_hops,existing_takeoff_only,cost_scale};
  const auto *cached=cacheable ? retry_cache->get(cache_key) : nullptr;
  const uint64_t cache_revision=routeMutationRevision();
  const bool local_reuse=free_frontier && !free_frontier->steps.empty();
  const bool reused_frontier = local_reuse || cached;
  // Reusing labels must retain the cap rejection, not report graph exhaustion.
  bool free_budget_limited=local_reuse ? free_frontier->budget_limited
      : cached && cached->budget_limited;
  if(budget_limited && free_budget_limited) *budget_limited=true;
  timer.add(reused_frontier ? RepairCounter::reused_frontiers : RepairCounter::source_labels);
  std::vector<Step> steps = local_reuse ? std::move(free_frontier->steps)
      : cached ? cached->steps : std::vector<Step>{{source,-1,0,0}};
  RepairLabelMap best = local_reuse
      ? std::move(free_frontier->best) : cached ? cached->best
      : RepairLabelMap{{source,0}};
  using Work=std::pair<int,int>;
  std::priority_queue<Work,std::vector<Work>,std::greater<Work>> work;
  if (!reused_frontier && !incoming_sources) work.push({0,0});
  int reached=-1;
  int completed_hops=-1;
  std::vector<RepairNode> path;
  size_t expanded=0, limited=0, first_limit_expanded=0, first_limit_queue=0;
  size_t eligibility_checks=0, parent_checks=0, dominated=0;
  std::unordered_set<int> traced_expanded;
  if (incoming_sources && free_states>=2) {
    timer.add(RepairCounter::two_ended_calls);
    timer.add(RepairCounter::source_labels);
    // A source-only frontier can spend its whole budget in unrelated free
    // fabric while just one thin approach reaches the sink. Expand the
    // smaller of the two pending frontiers. Both sides SHARE the old label
    // bound; the joined path still uses the ordinary transaction below.
    std::vector<Step> reverse{{sink,-1,0,0}};
    RepairLabelMap reverse_best{{sink,0}};
    size_t forward_head=0, reverse_head=0;
    auto join=[&](int forward,int backward) {
      std::vector<RepairNode> candidate;
      for(int i=forward;i>=0;i=steps[i].parent) candidate.push_back(steps[i].node);
      std::reverse(candidate.begin(),candidate.end());
      for(int i=backward;i>=0;i=reverse[i].parent) candidate.push_back(reverse[i].node);
      std::unordered_set<RepairNode,RepairNodeHash> visited;
      int hops=0;
      for(size_t i=0;i<candidate.size();++i) {
        if(cancelled && cancelled()) return false;
        if(!visited.insert(candidate[i]).second) return false;
        if(i) {
          bool connected=false;
          successors(candidate[i-1],[&](const RepairNode &next) { connected|=next==candidate[i]; });
          if(!connected || !allowed(candidate[i-1],candidate[i])) return false;
        }
        // This is a complete free path, not a growing speculative suffix.
        // Distinct nodes can require several crossings of one switchbox.
        // Both frontiers already expand each numeric node once; reject an
        // actual node cycle above and bound inter-tile hops, rather than
        // discarding a valid join for reusing a tile through other ports.
        if(i && !sameCoord(repairCoord(candidate[i-1]),repairCoord(candidate[i])) &&
            ++hops>max_hops) return false;
      }
      path=std::move(candidate); reached=forward; completed_hops=hops;
      return true;
    };
    auto predecessors=[&](const RepairNode &node,auto &&visit) {
      const auto [x,y,kind,id]=node;
      auto *tile=device.getTile(x,y);
      if(!tile || !tile->cb_type) return;
      const auto &cb=*tile->cb_type;
      // Read exact single arcs, including arbitrary LOCAL/JOINT chains.
      // Derived reachability masks would hide the intervening resources.
      auto scan=[&](const auto &table,auto bits,int from_kind) {
        for(const auto &[from,row]:table.values)
          if(bits(row).testBit(id)) visit(RepairNode{x,y,from_kind,from});
      };
      auto locals=[](const auto &row)->const NodeMask& { return row.local; };
      auto joints=[](const auto &row)->const NodeMask& { return row.joint; };
      auto jumps=[](const auto &row)->const NodeMask& { return row.jump; };
      if(kind==CB_NODE_LOCAL) {
        scan(cb.dst_local,locals,CB_NODE_DST);
        scan(cb.joint_local,locals,CB_NODE_JOINT);
        scan(cb.local_local,locals,CB_NODE_LOCAL);
      } else if(kind==CB_NODE_JOINT) {
        scan(cb.dst_joint,joints,CB_NODE_DST);
        scan(cb.local_joint,joints,CB_NODE_LOCAL);
        scan(cb.joint_joint,joints,CB_NODE_JOINT);
        scan(cb.src_joint,joints,CB_NODE_SRC);
      } else if(kind==CB_NODE_SRC) {
        scan(cb.dst_src,jumps,CB_NODE_DST);
        scan(cb.local_src,jumps,CB_NODE_LOCAL);
        scan(cb.joint_src,jumps,CB_NODE_JOINT);
      }
      incoming_sources({x,y},static_cast<CBNodeNameType>(kind),id,
          [&](Coord at,CBNodeNameType type,int input) { visit(repairNode(at,type,input)); });
    };
    // A fully exhausted frontier proves disconnection. A quota-truncated
    // frontier does not: let the other end reach one of its existing labels,
    // including a join on the final edge without another allocation.
    bool forward_limited=false, reverse_limited=false;
    while(path.empty() && (forward_head<steps.size() || reverse_head<reverse.size())) {
      if ((forward_head==steps.size() && !forward_limited) ||
          (reverse_head==reverse.size() && !reverse_limited)) break;
      if(cancelled && cancelled()) return false;
      const bool backwards=forward_head==steps.size() ||
          (reverse_head<reverse.size() && reverse.size()-reverse_head<steps.size()-forward_head);
      auto &own=backwards?reverse:steps;
      auto &own_best=backwards?reverse_best:best;
      auto &other_best=backwards?best:reverse_best;
      const int current=backwards?int(reverse_head++):int(forward_head++);
      const Step step=own[current];
      if (!pnr::InputRepairBudget::expand()) return false;
      timer.expand(step.node);
      traceRepairDecision(backwards ? "source_reverse_pop" : "source_forward_pop",
                          step.node, sink, "expand");
      ++expanded;
      auto add=[&](const RepairNode &next) {
        timer.add(RepairCounter::source_edges);
        auto decision = [&](std::string_view why) {
          traceRepairDecision(backwards ? "reverse_edge" : "forward_edge", step.node, next, why);
        };
        if(!path.empty()) { decision("joined_or_seen"); return; }
        if(own_best.contains(next)) {
          timer.add(RepairCounter::source_seen); decision("joined_or_seen"); return;
        }
        const int hops=step.hops+!sameCoord(repairCoord(step.node),repairCoord(next));
        if(hops>max_hops) { decision("hop_limit"); return; }
        if(backwards) {
          const auto [x,y,kind,id]=next;
          auto *at=device.getTile(x,y);
          if(!at || !at->cb_type || id<0 || id>=CB_MAX_NODES) { decision("invalid_node"); return; }
          bool busy=repairNodeLeased(at->cb,kind,id);
          if(kind==CB_NODE_LOCAL) busy|=at->isPinNodeLeased(id);
          if(busy && !protected_nodes.contains(next) && !detached.contains(next)) { decision("foreign_busy"); return; }
        }
        if(!allowed(backwards?next:step.node,backwards?step.node:next)) return;
        // Join across the discovered edge without allocating a duplicate of
        // the other frontier's node. A meeting at the exact label limit must
        // not be discarded as if it needed another state.
        const auto other=other_best.find(next);
        if(other!=other_best.end() && join(backwards?other->second:current,
            backwards?current:other->second)) { decision("joined"); return; }
        // One broad expansion must not consume the other frontier's room.
        // Keep expanding existing labels (and checking joins) at the quota;
        // a thin reverse approach can then meet a broad source frontier.
        // At tiny budgets leave room for one complete LOCAL/SRC/DST hop;
        // the combined limit below still includes the opposite endpoint.
        if(steps.size()+reverse.size()>=free_states ||
            own.size()>=std::max<size_t>(3,(free_states+1)/2)) {
          timer.add(RepairCounter::source_limit); ++limited;
          (backwards ? reverse_limited : forward_limited)=true;
          free_budget_limited=true;
          if (budget_limited) *budget_limited = true;
          decision("state_limit"); return;
        }
        const int n=own.size();
        own_best.emplace(next,n); own.push_back({next,current,hops,step.cost+1});
        timer.add(RepairCounter::source_labels);
        decision("enqueue");
      };
      if(backwards) predecessors(step.node,add);
      else successors(step.node,add);
    }
    if(trace) search_log << "SOURCE_TWO_ENDED forward=" << steps.size() << " reverse=" << reverse.size()
        << " expanded=" << expanded << " reached=" << !path.empty() << " budget=" << free_states << '\n';
    if(timer.enabled) std::fprintf(stderr,
        "REPAIR_TWO_ENDED net=%s forward=%zu reverse=%zu expanded=%zu reached=%d budget=%zu\n",
        net.name.c_str(),steps.size(),reverse.size(),expanded,!path.empty(),free_states);
  }
  while (!work.empty()) {
    if (cancelled && cancelled()) return false;
    const int current=work.top().second; work.pop();
    const auto step=steps[current];
    traceRepairDecision("source_pop",step.node,sink,"expand");
    if (best[step.node]!=current) { timer.add(RepairCounter::stale_source_labels); continue; }
    if (!pnr::InputRepairBudget::expand()) return false;
    timer.expand(step.node);
    ++expanded;
    if (trace_frontier) traced_expanded.insert(current);
    if (step.node==sink) { reached=current; completed_hops=step.hops; break; }
    const auto join=std::find(suffix.begin(),suffix.end(),step.node);
    if (join!=suffix.end()) {
      std::vector<RepairNode> candidate;
      for (int p=current;p>=0;p=steps[p].parent) candidate.push_back(steps[p].node);
      std::reverse(candidate.begin(),candidate.end());
      int hops=step.hops;
      bool valid=true;
      for (auto next=join+1;next!=suffix.end();++next) {
        if ((cancelled && cancelled()) || !allowed(candidate.back(),*next) ||
            std::find(candidate.begin(),candidate.end(),*next)!=candidate.end()) { valid=false; break; }
        if (!sameCoord(repairCoord(candidate.back()),repairCoord(*next))) {
          unsigned visits=0;
          for (size_t i=0;i<candidate.size();++i)
            if (sameCoord(repairCoord(candidate[i]),repairCoord(*next)) &&
                (i==0 || !sameCoord(repairCoord(candidate[i-1]),repairCoord(*next)))) ++visits;
          if (++hops>max_hops || visits>=2) { valid=false; break; }
        }
        candidate.push_back(*next);
      }
      if (valid) {
        path=std::move(candidate); reached=current; completed_hops=hops;
        if (trace) {
          const auto [x,y,kind,id]=step.node;
          search_log << "SOURCE_SEARCH_SUFFIX tile=" << x << ',' << y << " kind=" << kind
              << " node=" << id << " prefix_hops=" << step.hops
              << " suffix_hops=" << hops-step.hops << " total_hops=" << hops
              << " states=" << steps.size() << '\n';
        }
        break;
      }
    }
    // After one admissible continuation was rejected by the full label
    // table, no queued state's outgoing arc can store another label. Still
    // check every queued sink and retained-tail join above. Keep the limit
    // flag (so larger-budget retries remain necessary), the reached labels
    // used by preemption below, and full edge exports for diagnostic replay.
    if (!trace_frontier && free_budget_limited && steps.size() >= free_states)
      continue;
    auto add=[&](RepairNode next) {
      timer.add(RepairCounter::source_edges);
      const bool jump=!sameCoord(repairCoord(step.node),repairCoord(next));
      const int hops=step.hops+jump;
      // New fanouts must first reuse the connected source tree. Restrict only
      // the initial departure, not a later transit through the source Tile.
      if (existing_takeoff_only && step.hops==0 && std::get<2>(next)==CB_NODE_SRC &&
          !protected_nodes.contains(next)) { traceRepairDecision("source_edge",step.node,next,"new_takeoff_forbidden"); return; }
      if (hops>max_hops) { traceRepairDecision("source_edge",step.node,next,"hop_limit"); return; }
      const int cost=step.cost+1+4*jump;
      auto found=best.find(next);
      const bool redundant=found!=best.end() && steps[found->second].cost<=cost &&
          steps[found->second].hops<=hops;
      // This exact dominance test used to follow ownership/topology checks
      // and an entire parent walk. Reject the same labels before that work;
      // frontier diagnostics still record every eligible edge for replay.
      if (redundant && !trace_frontier) { timer.add(RepairCounter::source_seen); ++dominated; traceRepairDecision("source_edge",step.node,next,"dominated"); return; }
      ++eligibility_checks;
      if (!allowed(step.node,next)) return;
      if (trace_frontier) {
        const auto [nx,ny,nkind,nid]=next;
        // Preserve all eligible topology, not just chosen parents: dominance
        // and path-repetition pruning otherwise hide edges needed to replay
        // the exact frontier in an isolated regression.
        search_log << "SOURCE_SEARCH_EDGE from=" << current << " tile=" << nx << ',' << ny
            << " kind=" << nkind << " node=" << nid << '\n';
      }
      if (redundant) { timer.add(RepairCounter::source_seen); ++dominated; traceRepairDecision("source_edge",step.node,next,"dominated"); return; }
      unsigned visits=0;
      for (int p=current;p>=0;p=steps[p].parent) {
        ++parent_checks;
        timer.add(RepairCounter::source_parent_steps);
        if (steps[p].node==next) { traceRepairDecision("source_edge",step.node,next,"prefix_node"); return; }
        if (jump && sameCoord(repairCoord(steps[p].node),repairCoord(next)) &&
            (steps[p].parent<0 || !sameCoord(repairCoord(steps[steps[p].parent].node),repairCoord(next)))) ++visits;
      }
      if (visits>=2) { traceRepairDecision("source_edge",step.node,next,"tile_visit_limit"); return; }
      if (steps.size()>=free_states) {
        timer.add(RepairCounter::source_limit);
        free_budget_limited=true;
        if (budget_limited) *budget_limited = true;
        traceRepairDecision("source_edge",step.node,next,"state_limit");
        if (limited++==0) { first_limit_expanded=expanded; first_limit_queue=work.size(); }
        if (trace_frontier) {
          auto [nx,ny,nkind,nid]=next;
          search_log << "SOURCE_SEARCH_LIMIT from=" << current << " tile=" << nx << ',' << ny
              << " kind=" << nkind << " node=" << nid << " cost=" << cost << " hops=" << hops << '\n';
        }
        return;
      }
      int n=steps.size(); best[next]=n; steps.push_back({next,current,hops,cost});
      timer.add(RepairCounter::source_labels);
      traceRepairDecision("source_edge",step.node,next,"enqueue");
      const int distance=std::abs(std::get<0>(next)-std::get<0>(sink))+
                         std::abs(std::get<1>(next)-std::get<1>(sink));
      // A congested corridor can require first moving away from the sink.
      // Overweighting geometric distance consumes the bounded frontier in
      // nearer dead corridors before such a detour is explored.
      int remaining=distance;
      if (std::get<2>(next)==CB_NODE_SRC) {
        // A SRC still has its departure coordinates. Ranking only those
        // coordinates makes short and long exits indistinguishable, so a
        // useful long jump can sit queued while shorter branches exhaust
        // the frontier. Look through the physical jump when ranking it;
        // occupancy/input checks still happen normally when it is expanded.
        auto *at=device.getTile(std::get<0>(next),std::get<1>(next));
        const auto &cb=*at->cb_type;
        int onward=std::numeric_limits<int>::max();
        if (cb.src_joint[std::get<3>(next)].joint.firstSetBit()>=0)
          onward=remaining; // Preserve the intra-CB alternative as well.
        for (const auto &target:device.resolveJumpTargets(*at,std::get<3>(next))) {
          if (!target.tile || target.dst_node<0) continue;
          const auto c=target.tile->coord;
          const int edge=sameCoord(c,at->coord)?1:5;
          onward=std::min(onward,cost_scale*edge+std::abs(c.x-std::get<0>(sink))+
                                       std::abs(c.y-std::get<1>(sink)));
        }
        if (onward!=std::numeric_limits<int>::max()) remaining=onward;
      }
      work.push({cost_scale*cost+remaining,n});
    };
    successors(step.node,add);
  }
  if (std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR"))
    std::fprintf(stderr,"REPAIR source states=%zu limit=%zu reached=%d hops=%d eligibility_checks=%zu parent_checks=%zu dominated=%zu reused_frontier=%d\n",
        steps.size(),free_states,reached>=0,completed_hops,eligibility_checks,parent_checks,dominated,reused_frontier);
  if (trace) {
    search_log << "SOURCE_SEARCH end states=" << steps.size() << " unique=" << best.size()
        << " expanded=" << expanded << " budget_rejections=" << limited
        << " first_limit_expanded=" << first_limit_expanded
        << " first_limit_queue=" << first_limit_queue << " reached=" << (reached>=0)
        << " terminal_leaf_rejections=" << terminal_leaf_rejections
        << " reused_frontier=" << reused_frontier << '\n';
    for (size_t i=0;i<steps.size() && trace_frontier;++i) {
      const auto &s=steps[i]; const auto [x,y,kind,id]=s.node;
      search_log << "SOURCE_SEARCH_NODE index=" << i << " parent=" << s.parent << " tile=" << x << ',' << y
          << " kind=" << kind << " node=" << id << " cost=" << s.cost << " hops=" << s.hops
          << " expanded=" << traced_expanded.count(i) << " best=" << (best.at(s.node)==int(i)) << '\n';
    }
    search_log.flush();
  }
  std::vector<fpga::RouteCutNode> grounding_cuts, released;
  timer.next("source_short_completion");
  // Cancellation exits above; only a fully exhausted, unchanged free search
  // may survive the next budget trial. Completion still uses live owner checks.
  if(cacheable && !cached && reached<0 && cache_revision==routeMutationRevision())
    retry_cache->put(cache_key,RepairFreeFrontier{steps,best,free_budget_limited});
  // Only exhausted free-only searches are reusable. Cancellation returns
  // above without caching, and a successful free path still commits normally.
  if (reached<0 && free_frontier && !grounding_preemption) {
    free_frontier->steps=std::move(steps);
    free_frontier->best=std::move(best);
    free_frontier->budget_limited=free_budget_limited;
    return false;
  }
  if (reached>=0 && path.empty()) {
    for (int p=reached;p>=0;p=steps[p].parent) path.push_back(steps[p].node);
    std::reverse(path.begin(),path.end());
  } else if (reached<0 && grounding_preemption) {
    // The free search is finished. Reuse its reached states, without another
    // fabric search. Only the final Grounding may preempt transit, including
    // its bounded multi-Tile approach. Never authorize arbitrary remote
    // transit cuts simply because the source search reached that position.
    auto try_terminal=[&](int current,std::vector<RepairNode> tail) {
      if (cancelled && cancelled()) return false;
      std::vector<RepairNode> candidate;
      for (int p=current;p>=0;p=steps[p].parent) candidate.push_back(steps[p].node);
      std::reverse(candidate.begin(),candidate.end());
      int hops=steps[current].hops;
      std::vector<fpga::RouteCutNode> cuts;
      for (const auto &next:tail) {
        if (std::find(candidate.begin(),candidate.end(),next)!=candidate.end()) return false;
        const bool jump=!sameCoord(repairCoord(candidate.back()),repairCoord(next));
        if (jump) {
          unsigned visits=0;
          for (size_t i=0;i<candidate.size();++i)
            if (sameCoord(repairCoord(candidate[i]),repairCoord(next)) &&
                (i==0 || !sameCoord(repairCoord(candidate[i-1]),repairCoord(next)))) ++visits;
          if (visits>=2) return false;
        }
        hops+=jump;
        if (hops>max_hops) return false;
        if (!allowed(candidate.back(),next)) {
          const auto [x,y,kind,id]=next;
          if (kind!=CB_NODE_DST && kind!=CB_NODE_JOINT && kind!=CB_NODE_SRC) return false;
          cuts.push_back({{x,y},static_cast<CBNodeNameType>(kind),id});
          // Placement reservations and conflicting same-source input arcs
          // remain forbidden even when foreign transit is provisionally free.
          if (!allowed(candidate.back(),next,cuts)) return false;
        }
        candidate.push_back(next);
      }
      if (cuts.empty()) return false;
      if (required_branch && std::find(candidate.begin(),candidate.end(),
          repairNode(required_branch->tile,required_branch->type,required_branch->node))==candidate.end())
        return false;
      std::vector<fpga::RouteCutNode> preview;
      if (!grounding_preemption(cuts,false,preview)) return false;
      for (size_t i=1;i<candidate.size();++i)
        if (!allowed(candidate[i-1],candidate[i],preview)) return false;
      if (trace) {
        search_log << "SOURCE_GROUNDING_WITNESS current=" << current
            << " hops=" << hops << " cuts=" << cuts.size() << '\n';
        for (const auto &node:candidate) {
          const auto [x,y,kind,id]=node;
          search_log << "SOURCE_GROUNDING_NODE tile=" << x << ',' << y
              << " kind=" << kind << " node=" << id << '\n';
        }
        for (const auto &cut:cuts)
          search_log << "SOURCE_GROUNDING_CUT tile=" << cut.tile.x << ',' << cut.tile.y
              << " kind=" << int(cut.type) << " node=" << cut.node << '\n';
        std::unordered_set<rtl::Net *> logged;
        for (const auto &owner:findNetRouteCutVictims(cuts))
          if (logged.insert(owner.net).second) dumpNetRouteHistory(*owner.net,search_log);
        search_log.flush();
      }
      path=std::move(candidate); grounding_cuts=std::move(cuts); released=std::move(preview);
      return true;
    };
    const auto &cb=*sink_tile->cb_type;
    const int local=std::get<3>(sink);
    auto terminal=[&](int current,int dst,bool append_dst,
                      std::vector<RepairNode> tail=std::vector<RepairNode>{}) {
      if (append_dst) tail.push_back(repairNode(sink_tile->coord,CB_NODE_DST,dst));
      auto finish=[&](int first=-1,int second=-1) {
        auto complete=tail;
        if (first>=0) complete.push_back(repairNode(sink_tile->coord,CB_NODE_JOINT,first));
        if (second>=0) complete.push_back(repairNode(sink_tile->coord,CB_NODE_JOINT,second));
        complete.push_back(sink);
        return try_terminal(current,std::move(complete));
      };
      if (cb.dst_local[dst].local.testBit(local) && finish()) return true;
      return cb.dst_joint[dst].joint.for_each_set_bit([&](int j) {
        if (cb.joint_local[j].local.testBit(local) && finish(j)) return true;
        return cb.joint_joint[j].joint.for_each_set_bit([&](int k) {
          return k!=j && cb.joint_local[k].local.testBit(local) && finish(j,k);
        });
      });
    };
    auto in_grounding_window=[&](Coord at) {
      return std::abs(at.x-sink_tile->coord.x)<=GROUNDING_RADIUS &&
          std::abs(at.y-sink_tile->coord.y)<=GROUNDING_RADIUS;
    };
    std::vector<int> grounding_entries;
    for (size_t i=0;i<steps.size() && path.empty();++i) {
      if (cancelled && cancelled()) return false;
      const auto [x,y,kind,id]=steps[i].node;
      if (best.at(steps[i].node)!=int(i)) continue;
      if (kind==CB_NODE_DST && sameCoord({x,y},sink_tile->coord)) terminal(i,id,false);
      if (kind!=CB_NODE_SRC) continue;
      auto *tile=device.getTile(x,y);
      const bool outside=!in_grounding_window(tile->coord);
      bool enters_window=false;
      for (const auto &target:device.resolveJumpTargets(*tile,id)) {
        // The free SRC is the boundary, not a resource to preempt. Its
        // landing may enter Grounding from outside the window. Reuse this
        // existing jump scan instead of resolving the frontier again later.
        if (outside && target.tile && in_grounding_window(target.tile->coord))
          enters_window=true;
        if (target.tile==sink_tile && target.dst_node>=0 && terminal(i,target.dst_node,true)) break;
      }
      if (enters_window) grounding_entries.push_back(int(i));
    }
    // Preserve the existing free-SRC/terminal choices first. A foreign SRC
    // never entered the free search, even when its jump is the final approach
    // to this sink. Extend reached DSTs across that one physical jump here;
    // try_terminal validates the complete path and previews all owners before
    // any lease changes. Takeoff/Grounding victims remain protected by the
    // same callback as every other terminal preemption.
    for (size_t i=0;i<steps.size() && path.empty();++i) {
      if (cancelled && cancelled()) return false;
      const auto [x,y,kind,id]=steps[i].node;
      if (kind!=CB_NODE_DST || best.at(steps[i].node)!=int(i)) continue;
      auto *tile=device.getTile(x,y);
      const auto &type=*tile->cb_type;
      type.dst_src[id].jump.for_each_set_bit([&](int src) {
        // Free SRCs were handled by the free search and the preceding scan.
        if (!tile->cb.src.jump.testBit(src)) return false;
        for (const auto &target:device.resolveJumpTargets(*tile,src)) {
          if (target.tile==sink_tile && target.dst_node>=0 &&
              terminal(i,target.dst_node,true,{repairNode(tile->coord,CB_NODE_SRC,src)}))
            return true;
        }
        return false;
      });
    }
    // A reached DST can be separated from the sink by TWO occupied transit
    // jumps, both inside the Grounding window. The free search cannot enter
    // the first SRC, so the one-jump scan above never sees the second one.
    // Enumerate these short approaches only after the existing choices fail;
    // validate the WHOLE prefix/tail and preview all cuts before any mutation.
    // This is a local completion, not permission to preempt remote transit.
    if (path.empty()) {
      timer.next("source_full_completion");
      bool found=false;
      auto nearby=[&](Coord c) {
        return std::abs(c.x-sink_tile->coord.x)<=GROUNDING_RADIUS &&
            std::abs(c.y-sink_tile->coord.y)<=GROUNDING_RADIUS;
      };
      for (size_t i=0;i<steps.size() && !found;++i) {
        if (cancelled && cancelled()) break;
        const auto [x,y,kind,id]=steps[i].node;
        if (kind!=CB_NODE_DST || best.at(steps[i].node)!=int(i) || !nearby({x,y})) continue;
        auto *tile=device.getTile(x,y);
        const auto &type=*tile->cb_type;
        found=type.dst_src[id].jump.for_each_set_bit([&](int src) {
          for (const auto &middle:device.resolveJumpTargets(*tile,src)) {
            if (!middle.tile || middle.dst_node<0 || !middle.tile->cb_type || !nearby(middle.tile->coord)) continue;
            const auto &mid_type=*middle.tile->cb_type;
            if (mid_type.dst_src[middle.dst_node].jump.for_each_set_bit([&](int last) {
              for (const auto &target:device.resolveJumpTargets(*middle.tile,last))
                if (target.tile==sink_tile && target.dst_node>=0 && terminal(i,target.dst_node,true,
                    {repairNode(tile->coord,CB_NODE_SRC,src),
                     repairNode(middle.tile->coord,CB_NODE_DST,middle.dst_node),
                     repairNode(middle.tile->coord,CB_NODE_SRC,last)})) return true;
              return false;
            })) return true;
          }
          return false;
        });
      }
      if (trace) {
        search_log << "SOURCE_TWO_HOP_RESULT found=" << found << '\n';
        search_log.flush();
      }
    }
    // The short completions above cannot cross an occupied Takeoff exit or
    // an arbitrary chain of JOINTs. Continue numerically from reached states
    // within the Grounding window, at most twice its radius in Tile hops.
    // Only preview transit cuts during search; try_terminal validates the
    // entire connected prefix/completion and the combined cuts before commit.
    if (path.empty()) {
      struct CompletionStep { RepairNode node; int parent,hops,cost,root; };
      std::vector<CompletionStep> probe;
      std::map<RepairNode,int> cheapest;
      // Local to this immutable search: reuse owner-policy answers, never
      // occupancy across a commit. Incoming-arc/reservation checks stay per edge.
      std::map<RepairNode,bool> eligibility;
      std::priority_queue<Work,std::vector<Work>,std::greater<Work>> pending;
      // This optional retry extends one just-opened branch. Seeding another
      // existing approach would exchange an unrelated transit with a competing
      // repair. Derive eligibility from this search's parent chain; no retained
      // per-route bans or preemption history are involved.
      std::vector<bool> through_branch;
      if (required_branch) {
        through_branch.resize(steps.size());
        const auto required=repairNode(required_branch->tile,required_branch->type,required_branch->node);
        for (size_t i=0;i<steps.size();++i)
          through_branch[i]=steps[i].node==required ||
              (steps[i].parent>=0 && through_branch[steps[i].parent]);
      }
      auto seed=[&](size_t i) {
        if (required_branch && !through_branch[i]) return;
        const int n=probe.size();
        cheapest[steps[i].node]=n;
        probe.push_back({steps[i].node,-1,0,0,int(i)});
        pending.push({0,n});
      };
      for(size_t i=0;i<steps.size();++i) {
        const auto [x,y,kind,id]=steps[i].node;
        if(best.at(steps[i].node)!=int(i) || std::abs(x-sink_tile->coord.x)>GROUNDING_RADIUS ||
            std::abs(y-sink_tile->coord.y)>GROUNDING_RADIUS) continue;
        seed(i);
      }
      for(int i:grounding_entries) seed(i);
      size_t examined=0, rejected_complete=0;
      bool witness=false, budget=false;
      while (!pending.empty() && !(cancelled && cancelled())) {
        const int current=pending.top().second; pending.pop();
        const auto step=probe[current];
        if (cheapest.at(step.node)!=current) continue;
        if (!pnr::InputRepairBudget::expand()) return false;
        ++examined;
        if (step.node==sink) {
          std::vector<RepairNode> tail;
          for (int p=current;probe[p].parent>=0;p=probe[p].parent) tail.push_back(probe[p].node);
          std::reverse(tail.begin(),tail.end());
          witness=try_terminal(step.root,std::move(tail));
          if (witness) break;
          ++rejected_complete;
          continue;
        }
        successors(step.node,[&](const RepairNode &next) {
          const auto [x,y,kind,id]=next;
          if (std::abs(x-sink_tile->coord.x)>GROUNDING_RADIUS ||
              std::abs(y-sink_tile->coord.y)>GROUNDING_RADIUS) return;
          const bool jump=!sameCoord(repairCoord(step.node),repairCoord(next));
          const int hops=step.hops+jump;
          if (hops>2*GROUNDING_RADIUS || hops+steps[step.root].hops>max_hops) return;
          unsigned visits=0;
          for(int p=current;p>=0;p=probe[p].parent) {
            if (probe[p].node==next) return;
            if (jump && sameCoord(repairCoord(probe[p].node),repairCoord(next)) &&
                (probe[p].parent<0 || !sameCoord(repairCoord(probe[probe[p].parent].node),repairCoord(next)))) ++visits;
          }
          if (visits>=2) return;
          bool needs_cut=!allowed(step.node,next);
          const int cost=step.cost+1+4*jump+20*needs_cut;
          auto old=cheapest.find(next);
          if (old!=cheapest.end() && probe[old->second].cost<=cost) return;
          if (needs_cut) {
            if (kind!=CB_NODE_DST && kind!=CB_NODE_SRC && kind!=CB_NODE_JOINT) return;
            std::vector<fpga::RouteCutNode> cut{{{x,y},static_cast<CBNodeNameType>(kind),id}};
            if (!allowed(step.node,next,cut)) return;
            auto [it,inserted]=eligibility.try_emplace(next,false);
            if (inserted) {
              std::vector<fpga::RouteCutNode> preview;
              it->second=grounding_preemption(cut,false,preview);
              if (trace) search_log << "SOURCE_PREEMPT_NODE tile=" << x << ',' << y << " kind=" << kind
                  << " node=" << id << " eligible=" << it->second << '\n';
            }
            if (!it->second) return;
          }
          if (probe.size()>=max_states) {
            budget=true;
            if (budget_limited) *budget_limited = true;
            return;
          }
          const int n=probe.size();
          cheapest[next]=n; probe.push_back({next,current,hops,cost,step.root});
          pending.push({cost,n});
        });
      }
      if (trace) {
        search_log << "SOURCE_GROUNDING_SEARCH states=" << probe.size() << " expanded=" << examined
          << " limited=" << budget << " rejected_complete=" << rejected_complete
          << " found=" << witness << '\n';
        search_log.flush();
      }
    }
  }
  // A retained source prefix can be trapped OUTSIDE the Grounding window.
  // After free departures and local completion fail, allow a new Takeoff:
  // preview transit cuts only before/at its first landing, then require free
  // transit until bounded Grounding. The common commit below still validates
  // the complete source-to-input path before changing any victim leases.
  if (path.empty() && grounding_preemption && !required_branch && !existing_takeoff_only && cost_scale==1) {
    timer.next("source_takeoff_completion");
    struct ProbeStep { RepairNode node; int parent,hops,cost,grounding_start; };
    std::vector<ProbeStep> probe{{source,-1,0,0,-1}};
    RepairLabelMap best_probe{{source,0}};
    std::priority_queue<Work,std::vector<Work>,std::greater<Work>> pending;
    pending.push({0,0});
    bool found=false, limited=false;
    auto nearby=[&](const RepairNode &n) {
      return std::abs(std::get<0>(n)-std::get<0>(sink))<=GROUNDING_RADIUS &&
          std::abs(std::get<1>(n)-std::get<1>(sink))<=GROUNDING_RADIUS;
    };
    while (!pending.empty() && !(cancelled && cancelled())) {
      const int current=pending.top().second; pending.pop();
      const auto step=probe[current];
      if (best_probe.at(step.node)!=current) continue;
      if (!pnr::InputRepairBudget::expand()) return false;
      timer.add(RepairCounter::source_probe_expanded);
      if (step.node==sink) {
        std::vector<RepairNode> candidate;
        for (int p=current;p>=0;p=probe[p].parent) candidate.push_back(probe[p].node);
        std::reverse(candidate.begin(),candidate.end());
        std::vector<fpga::RouteCutNode> cuts, preview;
        for(size_t i=1;i<candidate.size();++i) if(!allowed(candidate[i-1],candidate[i])) {
          const auto [x,y,kind,id]=candidate[i];
          cuts.push_back({{x,y},static_cast<CBNodeNameType>(kind),id});
        }
        if ((cuts.empty() || grounding_preemption(cuts,false,preview)) &&
            std::equal(candidate.begin()+1,candidate.end(),candidate.begin(),
                [&](const auto &to,const auto &from) { return allowed(from,to,preview); })) {
          found=true;
          if (trace) {
            search_log << "SOURCE_TAKEOFF_WITNESS hops=" << step.hops << " cuts=" << cuts.size() << '\n';
            for(const auto &n:candidate) {
              const auto [x,y,kind,id]=n;
              search_log << "SOURCE_TAKEOFF_NODE tile=" << x << ',' << y << " kind=" << kind << " node=" << id << '\n';
            }
            for(const auto &cut:cuts) search_log << "SOURCE_TAKEOFF_CUT tile=" << cut.tile.x << ',' << cut.tile.y
                << " kind=" << int(cut.type) << " node=" << cut.node << '\n';
            std::unordered_set<rtl::Net *> logged;
            for(const auto &owner:findNetRouteCutVictims(cuts))
              if(logged.insert(owner.net).second) dumpNetRouteHistory(*owner.net,search_log);
          }
          path=std::move(candidate); grounding_cuts=std::move(cuts); released=std::move(preview);
          break;
        }
      }
      successors(step.node,[&](const RepairNode &next) {
        timer.add(RepairCounter::source_probe_edges);
        const bool jump=!sameCoord(repairCoord(step.node),repairCoord(next));
        const int hops=step.hops+jump;
        if(hops>max_hops) return;
        // A cut can only add cost. Reject already cheaper labels before
        // ownership checks and ancestry, as in the main free search.
        const int base_cost=step.cost+1+4*jump;
        const auto old=best_probe.find(next);
        if(old!=best_probe.end() && probe[old->second].cost<=base_cost) return;
        int grounding_start=step.grounding_start;
        if(grounding_start>=0 && (!nearby(next) || hops-grounding_start>2*GROUNDING_RADIUS)) return;
        const bool cut=!allowed(step.node,next);
        const int cost=base_cost+20*cut;
        if(old!=best_probe.end() && probe[old->second].cost<=cost) return;
        unsigned visits=0;
        for(int p=current;p>=0;p=probe[p].parent) {
          timer.add(RepairCounter::source_probe_parent_steps);
          if(probe[p].node==next) return;
          if(jump && sameCoord(repairCoord(probe[p].node),repairCoord(next)) &&
              (probe[p].parent<0 || !sameCoord(repairCoord(probe[probe[p].parent].node),repairCoord(next)))) ++visits;
        }
        if(visits>=2) return;
        if(cut) {
          const auto [x,y,kind,id]=next;
          if(kind!=CB_NODE_SRC && kind!=CB_NODE_DST && kind!=CB_NODE_JOINT) return;
          const bool takeoff=step.hops==0 && sameCoord(repairCoord(step.node),repairCoord(source));
          // A free approach may jump into an occupied Grounding landing
          // from outside. Only the resource being cut must be in the window;
          // the existing hop/window checks bound every subsequent step.
          if(!takeoff && !nearby(next)) return;
          if(!takeoff && grounding_start<0) grounding_start=step.hops;
          std::vector<fpga::RouteCutNode> cuts{{{x,y},static_cast<CBNodeNameType>(kind),id}}, preview;
          const bool eligible=allowed(step.node,next,cuts) && grounding_preemption(cuts,false,preview);
          if(trace && takeoff) search_log << "SOURCE_TAKEOFF_EXIT tile=" << x << ',' << y
              << " kind=" << kind << " node=" << id << " eligible=" << eligible << '\n';
          if(!eligible) return;
        }
        if(probe.size()>=max_states) {
          limited=true;
          if (budget_limited) *budget_limited = true;
          return;
        }
        const int n=probe.size(); best_probe[next]=n;
        probe.push_back({next,current,hops,cost,grounding_start});
        const int distance=std::abs(std::get<0>(next)-std::get<0>(sink))+
            std::abs(std::get<1>(next)-std::get<1>(sink));
        pending.push({cost+2*distance,n});
      });
    }
    if (trace) {
      search_log << "SOURCE_TAKEOFF_SEARCH states=" << probe.size() << " limited=" << limited << " found=" << found << '\n';
      search_log.flush();
    }
  }
  if (path.empty() || (cancelled && cancelled())) return false;
  // An opt-in failed-task probe uses exactly the production owner policy and
  // full replacement validation, but must stop before cutting or leasing.
  if (diagnostic_only) {
    if (trace) {
      search_log << "SOURCE_DIAGNOSTIC_COMPLETE nodes=" << path.size()
          << " cuts=" << grounding_cuts.size() << " cost_scale=" << cost_scale
          << " max_states=" << max_states << '\n';
      for (const auto &n:path) {
        const auto [x,y,kind,id]=n;
        search_log << "SOURCE_DIAGNOSTIC_NODE tile=" << x << ',' << y
            << " kind=" << kind << " node=" << id << '\n';
      }
    }
    return true;
  }
  timer.next("source_commit");
  // Compress ordinary hops into the existing representation so branch
  // discovery and preemption retain their normal transit/Takeoff distinction.
  // Unusual intra-CB continuations use existing explicit numeric graph edges.
  std::vector<Wire> replacement{route->front()};
  for (size_t i=0;i+1<path.size();) {
    size_t end=i+1;
    std::vector<int> joints;
    while (end<path.size() && sameCoord(repairCoord(path[end]),repairCoord(path[i])) &&
           std::get<2>(path[end])==CB_NODE_JOINT) {
      joints.push_back(std::get<3>(path[end++]));
    }
    const bool input=std::get<2>(path[i])==CB_NODE_DST || (i==0 && path[i]==source);
    const bool hop=end+1<path.size() && std::get<2>(path[end])==CB_NODE_SRC &&
        sameCoord(repairCoord(path[end]),repairCoord(path[i])) && std::get<2>(path[end+1])==CB_NODE_DST;
    const bool terminal=end+1==path.size() && path[end]==sink &&
        sameCoord(repairCoord(path[end]),repairCoord(path[i]));
    if (input && joints.size()<=2 && (hop || terminal)) {
      Wire w;
      w.from=repairCoord(path[i]); w.local=std::get<3>(path[i]); w.pos=i==0?0:1;
      if (!joints.empty()) w.joint=joints.back();
      if (joints.size()==2) w.joint2=joints.front();
      if (hop) {
        w.jump=w.route_jump=std::get<3>(path[end]); w.dst=std::get<3>(path[end+1]);
        w.to=repairCoord(path[end+1]); w.owns_landing=true;
        i=end+1;
      } else { w.to=w.from; i=end; }
      w.net_name=binding.route_name;
      replacement.push_back(w);
    } else {
      Wire w;
      w.type=Wire::WIRE_ROUTE_EDGE;
      w.from=repairCoord(path[i]); w.to=repairCoord(path[i+1]);
      w.from_node_type=std::get<2>(path[i]); w.from_node=std::get<3>(path[i]);
      w.to_node_type=std::get<2>(path[i+1]); w.to_node=std::get<3>(path[i+1]);
      w.net_name=binding.route_name;
      replacement.push_back(w); ++i;
    }
  }
  replacement.push_back(route->back());
  auto lease_path = [&] {
    for (const auto &n:path) {
      auto [x,y,kind,id]=n; auto &cb=device.getTile(x,y)->cb;
      switch (kind) {
        case CB_NODE_LOCAL: if (n!=source) cb.local.local.setBit(id); break;
        case CB_NODE_JOINT: cb.joint.jump.setBit(id); break;
        case CB_NODE_SRC: cb.src.jump.setBit(id); break;
        case CB_NODE_DST: cb.dst.jump.setBit(id); break;
      }
    }
  };
  if (unbound) {
    // A new task uses the same bounded numeric search, without manufacturing
    // an incomplete registered route. The complete projected path must pass
    // before any transit cuts, just as for a retained-route repair. There is
    // no detached tail to repoint, and no mutation on a failed preview.
    size_t storage=binding.to->wires.size();
    const auto existing=net.findRouteBinding(binding.from,binding.to,binding.from_port,
        binding.to_port,binding.route_name);
    if (existing.index<net.routes.size()) {
      auto &old=net.routes[existing.index];
      const auto *slot=bindingRoute(old);
      const bool uninstalled=!old.owner &&
          old.route_index==std::numeric_limits<size_t>::max();
      if (slot ? !slot->empty() : !uninstalled) return false;
      // Unrouting keeps the logical binding. Reuse its empty physical slot
      // when owned by this sink, rather than accumulating abandoned vectors.
      if (old.owner==binding.to) storage=old.route_index;
    }
    for (size_t i=1;i<path.size();++i) if (!allowed(path[i-1],path[i],released)) return false;
    if (cancelled && cancelled()) return false;
    if (!grounding_cuts.empty() && !grounding_preemption(grounding_cuts,true,released)) return false;
    RouteHistoryScope history(&net,__func__);
    for (auto &w:replacement) w.net_name=binding.route_name;
    replacement.front().shared=true;
    if (storage==binding.to->wires.size()) binding.to->wires.push_back(std::move(replacement));
    else binding.to->wires[storage]=std::move(replacement);
    lease_path();
    sink_tile->pin_state.leased_nodes.setBit(std::get<3>(sink));
    const size_t installed=attachNetRoute(net,*binding.to,storage,binding.from,binding.to,
        binding.from_port,binding.to_port,binding.route_name);
    registerNetRouteTiles(net,binding.to->wires[storage],installed);
    timer.success=true;
    return true;
  }
  bool rejoined=false;
  for (size_t i=1;i<path.size();++i) {
    const auto old=inputs.find(path[i]);
    rejoined |= detached.contains(path[i]) && old!=inputs.end() &&
        (old->second.size()!=1 || !old->second.count(path[i-1]));
  }
  if (!rejoined) terminal_group={{&net,index}};
  struct Edit { rtl::Net *net; size_t index; std::vector<Wire> *route; std::vector<Wire> next; };
  std::vector<Edit> edits;
  std::unordered_set<const rtl::NetRouteBinding *> excluded;
  for (const auto &ref:terminal_group) {
    auto &b=ref.net->routes[ref.binding_index];
    auto *old=bindingRoute(b);
    if (!old || old->empty()) return false;
    excluded.insert(&b);
    // Aliases can register the same stored vector more than once.
    if (std::any_of(edits.begin(),edits.end(),[&](const auto &e) { return e.route==old; })) continue;
    auto next=replacement;
    // Keep source ownership only when the replacement uses that same pin.
    // A prefix repair may select another mapped output LOCAL; copying the old
    // endpoint then records a disconnected route and leases the wrong pin.
    if (sameCoord(old->front().from,route->front().from) &&
        old->front().local==route->front().local)
      next.front()=old->front();
    else
      next.front().shared=false;
    // A relocated input can retain only a source prefix, with no old sink
    // endpoint. Keep the newly resolved input in that case, not its last hop.
    if (old->size()>1 && old->back().type==Wire::WIRE_TILE_PIN) next.back()=old->back();
    for (auto &w:next) {
      w.net_name=b.route_name;
      if (!edits.empty()) w.shared=true;
    }
    edits.push_back({ref.net,ref.binding_index,old,std::move(next)});
  }
  if (edits.empty()) return false;
  if (rejoined) {
    // Keep exactly one owner for the common input even when an RTL alias
    // happens to be enumerated first. Preserve borrowed source pins: taking
    // ownership of one would spuriously promote siblings on a later repair.
    edits.front().next.back().shared=false;
    edits.front().next.front().shared=std::all_of(edits.begin(),edits.end(),
        [&](const auto &e) { return e.route->front().shared &&
            sameCoord(e.route->front().from,route->front().from) &&
            e.route->front().local==route->front().local; });
  }
  // All validation precedes mutation. In particular, no incomplete wandering
  // suffix is installed. A terminal transit cut is applied only after the
  // complete replacement passes projected occupancy and packed-input checks.
  for (size_t i=1;i<path.size();++i) if (!allowed(path[i-1],path[i],released)) return false;
  if (cancelled && cancelled()) return false;
  if (!grounding_cuts.empty() && !grounding_preemption(grounding_cuts,true,released)) return false;
  RouteHistoryScope history(&net,__func__);
  for (const auto &e:edits) promoteSurvivingSourcePrefix(*e.net,e.index,&excluded);
  for (auto &e:edits) e.route->swap(e.next);
  lease_path();
  if (endpoints) sink_tile->pin_state.leased_nodes.setBit(std::get<3>(sink));
  // Register the replacement before releasing old leases, including old
  // islands: live owners in other bindings must keep every shared resource.
  for (const auto &ref:terminal_group)
    registerNetRouteTiles(*ref.net,*bindingRoute(ref.net->routes[ref.binding_index]),ref.binding_index);
  std::vector<const std::vector<Wire> *> removed;
  for (const auto &e:edits) removed.push_back(&e.next);
  clearRouteLeases(removed,true);
  for (const auto &e:edits) {
    if (sameCoord(e.next.front().from,e.route->front().from) &&
        e.next.front().local==e.route->front().local) continue;
    // Ordinary Takeoff cleanup releases CB usage but retains the output PIN
    // reservation. This endpoint changed, so release that old reservation as
    // well, through the live-owner check that protects surviving siblings.
    releaseRouteFragmentLease(std::vector<Wire>{e.next.front()},0);
  }
  for (const auto &ref:terminal_group) for (const auto &e:edits)
    if (bindingRoute(ref.net->routes[ref.binding_index])==e.route)
      rebuildNetRouteTiles(*ref.net,{e.next});
  if (!grounding_cuts.empty() && std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR"))
    std::fprintf(stderr,"REPAIR source grounding net=%s binding=%zu cuts=%zu complete=1\n",
        net.name.c_str(),index,grounding_cuts.size());
  timer.success=true;
  return true;
}
} // namespace

bool fpga::auditRouteCompletion(rtl::Net &net,
                                  const rtl::NetRouteBinding &binding,
                                  const Wire &source, const Wire &sink,
                                  std::ostream &out,
                                  const RepairGroundingPreemption &preemption,
                                  size_t frontier_states, size_t probe_states,
                                  int max_hops, bool capture_probe_frontier) {
  if (!binding.from || !binding.to || binding.from_port.empty() || binding.to_port.empty() ||
      !frontier_states || !probe_states || frontier_states>8000000 || probe_states>8000000) return false;
  const auto found=net.findRouteBinding(binding.from,binding.to,binding.from_port,
      binding.to_port,binding.route_name);
  const auto *stored=found.index<net.routes.size() ? bindingRoute(net.routes[found.index]) : nullptr;
  // A relocated sink can retain only a source prefix (or an empty slot).
  // Such a binding has no stored destination endpoint to audit. Inspect a
  // complete source-to-current-sink alternative without registering endpoints
  // or mutating that retained prefix, just as for an unbound diagnostic.
  const bool retained=stored && stored->size()>=2 &&
      stored->front().type==Wire::WIRE_TILE_PIN && stored->back().type==Wire::WIRE_TILE_PIN;
  // A retained task must use its real tree, including its already-owned sink.
  if (retained) {
    const auto *route=bindingRoute(net.routes[found.index]);
    if (!route || route->size()<2 || route->front().type!=Wire::WIRE_TILE_PIN ||
        route->back().type!=Wire::WIRE_TILE_PIN ||
        !sameCoord(route->front().from,source.from) || route->front().local!=source.local ||
        !sameCoord(route->back().from,sink.from) || route->back().local!=sink.local) return false;
    // Keep the retained fragments beside the numeric frontier for offline replay.
    dumpNetRouteHistory(net,out);
  }
  const std::vector<Wire> endpoints{source,sink};
  auto probe=[&](size_t states,int scale,bool frontier) {
    return repairFromSource(net,retained?found.index:std::numeric_limits<size_t>::max(),{},
        states,max_hops,preemption,retained?nullptr:&binding,retained?nullptr:&endpoints,
        false,true,scale,&out,frontier);
  };
  // These are all diagnostic_only calls: even a fully validated free route
  // or legal combined cut stops before touching leases, bindings or victims.
  for (int scale:{1,2})
    if (probe(frontier_states,scale,true)) return true;
  // Large failure captures are opt-in; keep their eligible arcs as well as
  // the witness so a replay does not turn a cut-only path into a free path.
  return probe_states>frontier_states && probe(probe_states,1,capture_probe_frontier);
}

bool fpga::previewUnboundNetRoute(rtl::Net &net, const rtl::NetRouteBinding &binding,
                           const Wire &source, const Wire &sink,
                           const std::function<bool()> &cancelled,
                           size_t max_states, int max_hops,
                           const RepairGroundingPreemption &preemption,
                           bool *budget_limited) {
  if (budget_limited) *budget_limited = false;
  const std::vector<Wire> endpoints{source, sink};
  return repairFromSource(net, std::numeric_limits<size_t>::max(), cancelled,
      max_states, max_hops, preemption, &binding, &endpoints, false, true,
      1, nullptr, false, nullptr, false, {}, budget_limited, nullptr, nullptr, true);
}

namespace {
bool routeUnboundWithCache(rtl::Net &net,
                                    const rtl::NetRouteBinding &binding,
                                    const Wire &source, const Wire &sink,
                                    const std::function<bool()> &cancelled,
                                    size_t max_states, int max_hops,
                                    const RepairGroundingPreemption &grounding_preemption,
                                    const RepairIncomingSources &incoming_sources,
                                    RepairFreeCache *retry_cache,
                                    bool *budget_limited,
                                    const RouteCutNode *required_branch = nullptr) {
  if (budget_limited) *budget_limited = false;
  if (!binding.from || !binding.to || binding.from_port.empty() || binding.to_port.empty()) return false;
  const auto existing=net.findRouteBinding(binding.from,binding.to,binding.from_port,
      binding.to_port,binding.route_name);
  if (existing.index<net.routes.size()) {
    auto &old=net.routes[existing.index];
    const auto *slot=bindingRoute(old);
    // Shared-prefix repair retires aliased storage without deleting the
    // logical connection. Its explicit uninstalled sentinel needs a new slot.
    // Other dangling indices/owners remain corruption, never free fabric.
    const bool uninstalled=!old.owner &&
        old.route_index==std::numeric_limits<size_t>::max();
    if (slot ? !slot->empty() : !uninstalled) return false;
  }
  const std::vector<Wire> endpoints{source,sink};
  if(incoming_sources) {
    // The two-ended probe is free-only. A miss leaves all bindings and leases
    // unchanged; its caller retains the normal preemption/relocation fallback.
    PNR_ASSERT(!grounding_preemption,"two-ended wire probe cannot preempt");
    bool limited = false;
    const bool complete = repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
        max_states,max_hops,{},&binding,&endpoints,false,false,1,nullptr,false,nullptr,true,incoming_sources,
        &limited);
    if (budget_limited) *budget_limited = limited;
    if (complete) return true;
    // Escalated recovery must honor its caller's budget after the fast cap;
    // otherwise every later free-only attempt repeats the same truncated walk.
    if (!limited || (cancelled && cancelled()) || sourceFreeStateLimit(max_states)>=max_states)
      return false;
    if (budget_limited) *budget_limited = false;
    return repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
        max_states,max_hops,{},&binding,&endpoints,false,false,1,nullptr,false,nullptr,false,incoming_sources,
        budget_limited);
  }
  RepairFreeFrontier retained_free, unrestricted_free;
  // Do not hide the caller's larger recovery budget behind the fast cap.
  // Run only after existing choices fail. Grounding must see the expanded
  // free frontier too, using the same owner policy and atomic commit path.
  auto full_budget_retry=[&] {
    if ((cancelled && cancelled()) || sourceFreeStateLimit(max_states)>=max_states)
      return false;
    return repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
        max_states,max_hops,grounding_preemption,&binding,&endpoints,false,false,1,nullptr,false,nullptr,false,{},
        budget_limited,nullptr,required_branch);
  };
  if (repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
                       max_states,max_hops,{},&binding,&endpoints,true,false,1,nullptr,false,
                       grounding_preemption?&retained_free:nullptr,true,{},budget_limited,retry_cache)) return true;
  if (repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
                       max_states,max_hops,{},&binding,&endpoints,false,false,1,nullptr,false,
                       grounding_preemption?&unrestricted_free:nullptr,true,{},budget_limited,retry_cache)) return true;
  if (!grounding_preemption || (cancelled && cancelled())) return full_budget_retry();
  // A numeric prefix can reach Grounding when hop-based search cannot cross
  // an internal LOCAL. Do not discard that complete approach merely because
  // its terminal transit needs the same legal preemption as ordinary Docking.
  if (repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
                       max_states,max_hops,grounding_preemption,&binding,&endpoints,true,false,1,nullptr,false,
                       &retained_free,true,{},budget_limited,retry_cache,required_branch)) return true;
  if (repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
                      max_states,max_hops,grounding_preemption,&binding,&endpoints,false,false,1,nullptr,false,
                      &unrestricted_free,true,{},budget_limited,retry_cache,required_branch)) return true;
  // Preserve every success of the original search, including its selected
  // victims. Only after all original attempts fail, retry free-prefix and
  // Grounding completion with less geometric bias and the SAME per-attempt
  // bounds. Do not repeat the unchanged Takeoff probe in this alternate run.
  if (repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
                      max_states,max_hops,grounding_preemption,&binding,&endpoints,false,false,2,
                      nullptr,false,nullptr,true,{},budget_limited,retry_cache,required_branch)) return true;
  if (full_budget_retry()) return true;
  // Diagnostic only: a larger probe cannot rescue this task, mutate victims,
  // or change the normal routing budget. It establishes whether the same
  // terminal-preemption rules admit a complete path in the actual snapshot.
  const char *filter=std::getenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY");
  const char *value=std::getenv("SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES");
  if (filter && *filter && net.name.find(filter)!=std::string::npos && value) {
    char *end=nullptr;
    const auto requested=std::strtoul(value,&end,10);
    if (end!=value && *end=='\0' && requested>0 && requested<=8000000) {
      bool witnessed=false;
      // Inspect priorities in the SAME failed snapshot, without rescuing it.
      // Small trials obey both the requested and normal per-attempt bounds.
      for(int scale:{4,8})
        if(repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
            std::min<size_t>(requested,max_states),max_hops,grounding_preemption,
            &binding,&endpoints,false,true,scale)) { witnessed=true;break; }
      if(!witnessed)repairFromSource(net,std::numeric_limits<size_t>::max(),cancelled,
          requested,max_hops,grounding_preemption,&binding,&endpoints,false,true);
    }
  }
  return false;
}
} // namespace

bool fpga::routeUnboundNetFromSource(rtl::Net &net,const rtl::NetRouteBinding &binding,
    const Wire &source,const Wire &sink,const std::function<bool()> &cancelled,
    size_t max_states,int max_hops,const RepairGroundingPreemption &preemption,
    const RepairIncomingSources &incoming_sources,bool *budget_limited) {
  return routeUnboundWithCache(net,binding,source,sink,cancelled,max_states,max_hops,
      preemption,incoming_sources,nullptr,budget_limited);
}

// Fixed logical identity prevents sharing frontiers between different consumers.
struct fpga::UnboundRouteSearch::Impl {
  rtl::Net &net;
  rtl::NetRouteBinding endpoints;
  RepairFreeCache cache;
  Impl(rtl::Net &n,const rtl::NetRouteBinding &b):net(n),endpoints(b) {}
};
fpga::UnboundRouteSearch::UnboundRouteSearch(rtl::Net &net,const rtl::NetRouteBinding &binding)
    :impl(std::make_unique<Impl>(net,binding)) {}
fpga::UnboundRouteSearch::~UnboundRouteSearch()=default;
void fpga::UnboundRouteSearch::clear() { impl->cache.clear(); }
// Scope reuse to read-only trials; a preemption commit invalidates all snapshots.
bool fpga::UnboundRouteSearch::route(const Wire &source,const Wire &sink,
    const std::function<bool()> &cancelled,size_t max_states,int max_hops,
    const RepairGroundingPreemption &preemption,const RepairIncomingSources &incoming_sources,
    bool *budget_limited,const RouteCutNode *required_branch) {
  RepairGroundingPreemption guarded;
  if(preemption) guarded=[&](const auto &cuts,bool commit,auto &released) {
    if(commit) clear();
    return preemption(cuts,commit,released);
  };
  return routeUnboundWithCache(impl->net,impl->endpoints,source,sink,cancelled,
      max_states,max_hops,guarded,incoming_sources,&impl->cache,budget_limited,required_branch);
}

bool fpga::completeNetRoutePrefixFromSource(rtl::Net &net, size_t binding_index,
                                            const Wire &source, const Wire &sink,
                                            const std::function<bool()> &cancelled,
                                            size_t max_states, int max_hops,
                                            const RepairGroundingPreemption &preemption,
                                            const RouteCutNode *required_branch) {
  if (binding_index>=net.routes.size()) return false;
  auto &binding=net.routes[binding_index];
  const auto *prefix=bindingRoute(binding);
  if (!binding.from || !binding.to || !prefix || prefix->empty() || hasRouteGaps(*prefix) ||
      prefix->front().type!=Wire::WIRE_TILE_PIN ||
      (prefix->size()>1 && prefix->back().type==Wire::WIRE_TILE_PIN) ||
      !sameCoord(prefix->front().from,source.from))
    return false;
  // The caller enumerates the packed output port's physical LOCALs. A retained
  // prefix fixes its old departure, not every legal departure of that port.
  // Let the atomic source-tree replacement prove another LOCAL in this CB;
  // failure still preserves the prefix and every sibling/victim lease.
  const std::vector<Wire> endpoints{source,sink};
  RepairFreeFrontier retained_free, unrestricted_free;
  auto search=[&](bool takeoff_only, const RepairGroundingPreemption &cuts,
                  RepairFreeFrontier *frontier, int scale=1, bool fast=true) {
    return repairFromSource(net,binding_index,cancelled,max_states,max_hops,cuts,
        nullptr,&endpoints,takeoff_only,false,scale,nullptr,false,frontier,fast,{},nullptr,nullptr,required_branch);
  };
  // Preserve the fast end-continuation's successes. This is its last-chance
  // source-tree search, with exactly the existing numeric completion budgets.
  if (search(true,{},preemption?&retained_free:nullptr) ||
      search(false,{},preemption?&unrestricted_free:nullptr)) return true;
  if (preemption && (search(true,preemption,&retained_free) ||
      search(false,preemption,&unrestricted_free) ||
      search(false,preemption,nullptr,2))) return true;
  return sourceFreeStateLimit(max_states)<max_states &&
      search(false,preemption,nullptr,1,false);
}

bool fpga::completeNetRouteFromSource(rtl::Net &net, size_t binding_index,
                                     const std::function<bool()> &cancelled,
                                     size_t max_states, int max_hops,
                                     const RepairGroundingPreemption &preemption,
                                     const RepairIncomingSources &incoming_sources,
                                     const RouteCutNode *required_branch) {
  if (binding_index >= net.routes.size()) return false;
  const auto *route = bindingRoute(net.routes[binding_index]);
  if (!route || !hasRouteGaps(*route)) return false;
  // A free dedicated approach can be hidden behind a broad source frontier.
  // Probe both ends before a short preempting repair starts exchanging the
  // same transit resources with its neighbours. Keep the old fallback intact.
  if (incoming_sources && repairFromSource(net,binding_index,cancelled,max_states,
          max_hops,{},nullptr,nullptr,false,false,1,nullptr,false,nullptr,true,
          incoming_sources)) return true;
  RepairFreeFrontier frontier;
  if (repairFromSource(net,binding_index,cancelled,max_states,max_hops,{},
          nullptr,nullptr,false,false,1,nullptr,false,preemption ? &frontier : nullptr)) return true;
  if (preemption && !(cancelled && cancelled()) &&
      repairFromSource(net,binding_index,cancelled,max_states,max_hops,preemption,
          nullptr,nullptr,false,false,1,nullptr,false,&frontier,true,{},nullptr,nullptr,required_branch)) return true;
  // Retained gaps need the caller's recovery budget too, not only the fast cap.
  return !(cancelled && cancelled()) && sourceFreeStateLimit(max_states)<max_states &&
      repairFromSource(net,binding_index,cancelled,max_states,max_hops,preemption,
          nullptr,nullptr,false,false,1,nullptr,false,nullptr,false,{},nullptr,nullptr,required_branch);
}

bool fpga::repairNetRouteGaps(rtl::Net &net, size_t binding_index,
                             const std::function<bool()> &cancel,
                             size_t max_states, int max_hops,
                             const std::function<void(Tile &)> &audit_failure,
                             const RepairGroundingPreemption &grounding_preemption,
                             const RepairIncomingSources &incoming_sources) {
  RouteHistoryScope history(&net, __func__);
  RepairPhaseTimer timer(net,binding_index,"exact_gap_search");
  const std::function<bool()> observed_cancel=timer.enabled && cancel ? std::function<bool()>([&] {
    const bool stop=cancel(); timer.cancelled|=stop; return stop;
  }) : std::function<bool()>{};
  const auto &cancelled=observed_cancel ? observed_cancel : cancel;
  if (binding_index >= net.routes.size()) return false;
  const auto &binding = net.routes[binding_index];
  auto *route = bindingRoute(net.routes[binding_index]);
  if (!route) return false;
  RouteOwnerQueries owner_queries;
  while (hasRouteGaps(*route)) {
    if (cancelled && cancelled()) return false;
    const Wire gap = *std::find_if(route->begin(), route->end(), [](const auto &w) {
      return w.type == Wire::WIRE_GAP;
    });
    Tile *start = Device::current().getTile(gap.from.x, gap.from.y);
    Tile *finish = Device::current().getTile(gap.to.x, gap.to.y);
    const bool debug_repair = std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR") != nullptr;
    if (debug_repair) std::fprintf(stderr, "REPAIR net=%s gap=(%d,%d)/DST%d->(%d,%d)/DST%d\n",
        net.name.c_str(), gap.from.x, gap.from.y, gap.from_node, gap.to.x, gap.to.y, gap.to_node);
    if (!start || !finish || gap.from_node < 0 || gap.to_node < 0) return false;
    const bool terminal_tail = gap.to_node_type == CB_NODE_LOCAL;
    const NodeMask reserved_terminal_joints = terminal_tail
        ? packedInputJointReservations(*finish, binding.to, binding.to_port) : NodeMask{};
    // The only boundary exemptions are the retained start and destination.
    auto same_source = [&](const rtl::NetRouteBinding &other) {
      return rtl::sameRouteSource(binding.from,binding.from_port,other.from,other.from_port);
    };
    for (const auto &[tile, type, node] : {
          std::tuple{start, CB_NODE_DST, gap.from_node},
          std::tuple{finish, terminal_tail ? CB_NODE_LOCAL : CB_NODE_DST, gap.to_node}}) {
      auto owners = findNetOwnersByNode(*tile, type, node);
      if (owners.empty() || std::any_of(owners.begin(), owners.end(), [&](const auto &r) {
        return !r.net || !same_source(r.net->routes[r.binding_index]);
      })) return false;
    }
    // A free-only gap search cannot enter a terminal whose every incoming
    // DST belongs to another signal. Keep free, retained and sibling entries;
    // only a proven foreign cut lets us go straight to the existing fallback.
    bool foreign_terminal_cut = false;
    if (terminal_tail && finish->cb_type) {
      finish->cb_type->ensureDerivedMasks();
      const auto entries = finish->cb_type->dsts_reaching_local.get(gap.to_node).jump;
      if (entries != NodeMask{} && (entries & ~finish->cb.dst.jump) == NodeMask{}) {
        foreign_terminal_cut = !entries.for_each_set_bit([&](int dst) {
          const auto owners = findNetOwnersByNode(*finish, CB_NODE_DST, dst);
          return owners.empty() || std::any_of(owners.begin(), owners.end(), [&](const auto &owner) {
            return !owner.net || owner.binding_index >= owner.net->routes.size() ||
                same_source(owner.net->routes[owner.binding_index]);
          });
        });
      }
    }
    // A retained DST can also have a proven foreign cut on every incoming SRC.
    // Unknown owners, same-signal arcs and free alternatives forbid this shortcut.
    if (!terminal_tail && incoming_sources &&
        !(start == finish && gap.from_node == gap.to_node)) {
      bool any = false, blocked = true;
      incoming_sources(finish->coord, CB_NODE_DST, gap.to_node,
          [&](Coord at, CBNodeNameType kind, int node) {
        any = true;
        auto *tile = Device::current().getTile(at.x, at.y);
        if (kind != CB_NODE_SRC || !tile || node < 0 ||
            !tile->cb.src.jump.testBit(node)) { blocked = false; return; }
        const auto owners = findNetOwnersByNode(*tile, kind, node);
        if (owners.empty() || std::any_of(owners.begin(), owners.end(), [&](const auto &owner) {
          return !owner.net || owner.binding_index >= owner.net->routes.size() ||
              same_source(owner.net->routes[owner.binding_index]);
        })) blocked = false;
      });
      foreign_terminal_cut = any && blocked;
    }
    if (foreign_terminal_cut)
      traceRepairDecision("gap_terminal", repairNode(gap.from, CB_NODE_DST, gap.from_node),
          repairNode(gap.to, gap.to_node_type, gap.to_node), "all_entries_foreign_owned");
    struct Step { Tile *tile; int dst, parent, depth; Wire edge; };
    std::vector<Step> steps{{start, gap.from_node, -1, 0, {}}};
    timer.add(RepairCounter::gap_labels);
    std::set<std::tuple<int, int, int>> seen{{gap.from.x, gap.from.y, gap.from_node}};
    int reached = -1;
    Wire grounding;
    // The early source probe and final Grounding fallback see the same
    // leases: exact-gap expansion is read-only until it succeeds. Retain
    // only this attempt's exhausted free frontier, just as prefix/unbound
    // completion does, instead of traversing that source graph twice.
    RepairFreeFrontier source_frontier;
    // A failed completion is read-only, so do not repeat it after unchanged
    // exact-gap exploration. A committed gap starts a fresh attempt below.
    bool source_completion_attempted = false;
    // Scratch for one expanded path, never shared occupancy or search pruning.
    std::unordered_map<Tile *, unsigned> path_visits;
    SharedRepairEdges shared_edges{};
    NodeMask shared_sources;
    for (size_t i = 0; !foreign_terminal_cut && i < steps.size(); ++i) {
      if (cancelled && cancelled()) return false;
      // A detached boundary can open into a large irrelevant region while
      // another source exit is already close to the sink. Do not consume the
      // whole stage deadline flooding that region before trying the driver.
      // Use the caller's normal state budget, not a smaller arbitrary cap:
      // truncating this source search just sends us back into the expensive
      // exact-gap flood before repeating the same source search at full size.
      // Try FREE ONLY first, then reuse its frontier for validated completion.
      // A free but unreachable ingress is not a reason to postpone a legal
      // completion behind the entire gap flood. Failed attempts keep leases
      // intact and resume this search; cancellation and hop limits still apply.
      if (i == 64) {
        if (repairFromSource(net, binding_index, cancelled,
                max_states, max_hops, {}, nullptr, nullptr, false, false, 1,
                nullptr, false, grounding_preemption ? &source_frontier : nullptr)) {
          timer.success=true; return true;
        }
        if (cancelled && cancelled()) return false;
        if (grounding_preemption) {
          source_completion_attempted = true;
          // The early completion must retain the free reverse alternative:
          // a source-only budget can miss a bypass behind a broad frontier.
          if (incoming_sources && repairFromSource(net, binding_index, cancelled,
                  max_states, max_hops, {}, nullptr, nullptr, false, false, 1,
                  nullptr, false, nullptr, true, incoming_sources)) {
            timer.success=true; return true;
          }
          if (cancelled && cancelled()) return false;
          if (repairFromSource(net, binding_index, cancelled, max_states, max_hops,
                  grounding_preemption, nullptr, nullptr, false, false, 1,
                  nullptr, false, &source_frontier)) {
            timer.success=true; return true;
          }
          if (cancelled && cancelled()) return false;
        }
      }
      const Step step = steps[i];
      const auto trace_from = repairNode(step.tile->coord, CB_NODE_DST, step.dst);
      if (!pnr::InputRepairBudget::expand()) return false;
      timer.expand(trace_from);
      traceRepairDecision("gap_pop", trace_from,
          repairNode(gap.to, gap.to_node_type, gap.to_node), "expand");
      if (debug_repair) std::fprintf(stderr, "REPAIR expand=(%d,%d)/DST%d depth=%d\n",
          step.tile->coord.x, step.tile->coord.y, step.dst, step.depth);
      if (!terminal_tail && step.tile == finish && step.dst == gap.to_node) { reached = i; break; }
      if (!step.tile->cb_type) continue;
      // A full label table cannot admit another hop. Still inspect every
      // queued goal: an exact boundary was checked above, and a queued
      // arrival at a terminal Tile may Ground without adding a label.
      // Other queued states need no outgoing topology/owner/ancestry scan.
      if (steps.size()>=max_states && (!terminal_tail || step.tile!=finish)) continue;
      auto &type = *step.tile->cb_type;
      type.ensureDerivedMasks();
      // Reconstruct the path once, instead of repeating it for every SRC and
      // every landing. Preserve the original consecutive-Tile visit semantics.
      NodeMask occupied = step.tile->cb.joint.jump, path_sources;
      path_visits.clear();
      for (int p = static_cast<int>(i); p >= 0; p = steps[p].parent) {
        timer.add(RepairCounter::gap_parent_steps);
        const auto &ancestor = steps[p];
        if (ancestor.parent < 0 || steps[ancestor.parent].tile != ancestor.tile)
          ++path_visits[ancestor.tile];
        if (ancestor.parent < 0 || !sameCoord(ancestor.edge.from, step.tile->coord)) continue;
        const auto &w = ancestor.edge;
        if (w.jump >= 0) path_sources.setBit(w.jump);
        if (w.joint >= 0) occupied.setBit(w.joint);
        if (w.joint2 >= 0) occupied.setBit(w.joint2);
      }
      if (terminal_tail && step.tile == finish) {
        const NodeMask terminal_occupied = occupied | reserved_terminal_joints;
        int joint = -1, joint2 = -1;
        bool connected = type.dst_local[step.dst].local.testBit(gap.to_node);
        if (!connected) {
          const auto first = type.dst_joint[step.dst].joint & ~terminal_occupied;
          const auto last = type.local_reachable_joints[gap.to_node].joint & ~terminal_occupied;
          joint = (first & last).firstSetBit();
          if (joint < 0) first.for_each_set_bit([&](int node) {
            joint = (type.joint_joint[node].joint & last).firstSetBit();
            if (joint < 0) return false;
            joint2 = node;
            return true;
          });
          connected = joint >= 0;
        }
        if (connected) {
          grounding.from = grounding.to = finish->coord;
          grounding.pos = 1; grounding.local = step.dst;
          grounding.joint = joint; grounding.joint2 = joint2;
          grounding.owns_dst = i != 0;
          reached = i;
          break;
        }
      }
      if (step.depth >= max_hops || steps.size()>=max_states) continue;
      const auto *sources = type.srcNodes(CB_NODE_DST, step.dst);
      if (!sources) continue;
      bool shared_edges_ready = false;
      for (int src : *sources) {
        timer.add(RepairCounter::gap_edges);
        const auto trace_src = repairNode(step.tile->coord, CB_NODE_SRC, src);
        auto reject = [&](std::string_view reason) {
          traceRepairDecision("gap_exit", trace_from, trace_src, reason);
        };
        if (debug_repair) std::fprintf(stderr, "REPAIR src=%d busy=%d\n", src, step.tile->cb.src.jump.testBit(src));
        // A prior repair may already have restored a common portion of this
        // tree. Reuse only that exact same-source DST->SRC->DST fragment,
        // never a foreign lease or another incoming connection to this SRC.
        const Wire *shared_edge = nullptr;
        if (step.tile->cb.src.jump.testBit(src)) {
          timer.add(RepairCounter::shared_edge_checks);
          if (!shared_edges_ready) {
            collectSharedRepairEdges(*step.tile,step.dst,binding,shared_sources,shared_edges);
            shared_edges_ready = true;
          }
          if (shared_sources.testBit(src)) shared_edge = shared_edges[src];
          if(!shared_edge) { timer.add(RepairCounter::gap_busy_src); reject("busy_src_no_shared_edge"); continue; }
          if (debug_repair) std::fprintf(stderr,"REPAIR verify_shared_src=(%d,%d)/%d\n",
              step.tile->coord.x,step.tile->coord.y,src);
          timer.add(RepairCounter::owner_checks);
          const bool same_tree = owner_queries.allMatch(*step.tile, CB_NODE_SRC, src, [&](const auto &ref) {
            return same_source(ref.net->routes[ref.binding_index]);
          });
          if (!same_tree) { timer.add(RepairCounter::gap_busy_src); reject("foreign_src_owner"); continue; }
        }
        // Select an actually free direct/one-joint/two-joint connection.
        if (path_sources.testBit(src)) { reject("prefix_src"); continue; }
        int joint = -1, joint2 = -1;
        if (shared_edge) {
          joint = shared_edge->joint; joint2 = shared_edge->joint2;
        } else if (!type.dst_src[step.dst].jump.testBit(src)) {
          const NodeMask first = type.dst_joint[step.dst].joint & ~occupied;
          const NodeMask last = type.src_reachable_joints[src].joint & ~occupied;
          joint = (first & last).firstSetBit();
          if (joint < 0) first.for_each_set_bit([&](int node) {
            joint = (type.joint_joint[node].joint & last).firstSetBit();
            if (joint < 0) return false;
            joint2 = node;
            return true;
          });
          if (joint < 0) { reject("joint_blocked"); continue; }
        }
        reject("exit_eligible");
        for (const auto &target : Device::current().resolveJumpTargets(*step.tile, src)) {
          if (debug_repair && target.tile) std::fprintf(stderr, "REPAIR target=(%d,%d)/DST%d busy=%d\n",
              target.tile->coord.x, target.tile->coord.y, target.dst_node,
              target.dst_node >= 0 && target.tile->cb.dst.jump.testBit(target.dst_node));
          if (!target.tile || target.dst_node < 0) continue;
          if (shared_edge && (!sameCoord(shared_edge->to, target.tile->coord) ||
                              shared_edge->dst != target.dst_node)) continue;
          const bool terminal = !terminal_tail && target.tile == finish && target.dst_node == gap.to_node;
          const auto trace_dst = repairNode(target.tile->coord, CB_NODE_DST, target.dst_node);
          if (!terminal && !shared_edge && target.tile->cb.dst.jump.testBit(target.dst_node)) {
            timer.add(RepairCounter::gap_busy_dst);
            traceRepairDecision("gap_jump",trace_src,trace_dst,"busy_dst"); continue;
          }
          const auto visits = path_visits.find(target.tile);
          if (target.tile != step.tile && visits != path_visits.end() && visits->second >= 2) {
            traceRepairDecision("gap_jump",trace_src,trace_dst,"tile_visit_limit"); continue;
          }
          if (steps.size() >= max_states) { timer.add(RepairCounter::gap_limit); traceRepairDecision("gap_jump",trace_src,trace_dst,"state_limit"); continue; }
          const auto key = std::tuple{target.tile->coord.x, target.tile->coord.y, target.dst_node};
          if (!seen.insert(key).second) { timer.add(RepairCounter::gap_seen); traceRepairDecision("gap_jump",trace_src,trace_dst,"already_seen"); continue; }
          Wire w;
          w.from = step.tile->coord; w.to = target.tile->coord;
          w.pos = 1; w.local = step.dst; w.jump = src;
          w.dst = target.dst_node; w.route_jump = target.jump_node;
          w.joint = joint; w.joint2 = joint2;
          w.shared = shared_edge != nullptr;
          w.owns_dst = i != 0; // retained prefix owns the first boundary
          if (const auto *name = type.nodeName(CB_NODE_DST, step.dst)) w.from_wire_name = *name;
          if (const auto *name = type.nodeName(CB_NODE_SRC, src)) w.src_wire_name = *name;
          w.dst_wire_name = target.dst_wire;
          steps.push_back({target.tile, target.dst_node, static_cast<int>(i), step.depth + 1, w});
          timer.add(RepairCounter::gap_labels);
          traceRepairDecision("gap_jump",trace_src,trace_dst,"enqueue");
        }
      }
    }
    if (reached < 0) {
      timer.next("source_fallback");
      if (debug_repair) std::fprintf(stderr, "REPAIR failed states=%zu limit=%zu\n", steps.size(), max_states);
      // The DST-only gap walk cannot follow arbitrary LOCAL bridges. Its
      // bounded source frontier can also miss a free detour behind broad
      // fabric. Try the existing two-ended free search before authorizing
      // Grounding cuts; otherwise adjacent repairs can evict each other
      // despite a free source-to-input bypass. Both ends share this same
      // label budget, and ordinary lease/arc validation still applies.
      if (incoming_sources && !source_completion_attempted) {
        // Preserve the source-only search's existing successes, including
        // direct reconnection to an intact retained tail. Reuse the failed
        // frontier for preemption rather than expanding it a second time.
        if (source_frontier.steps.empty() &&
            repairFromSource(net,binding_index,cancelled,max_states,max_hops,
                {},nullptr,nullptr,false,false,1,nullptr,false,&source_frontier)) {
          timer.success=true; return true;
        }
        if (repairFromSource(net,binding_index,cancelled,max_states,max_hops,
                {},nullptr,nullptr,false,false,1,nullptr,false,nullptr,true,
                incoming_sources)) {
          timer.success=true; return true;
        }
      }
      if (!source_completion_attempted && repairFromSource(net,binding_index,cancelled,max_states,max_hops,
              grounding_preemption,nullptr,nullptr,false,false,1,nullptr,false,
              &source_frontier)) { timer.success=true; return true; }
      // The early completion exhausted only the fast frontier; honor a larger
      // explicit budget before declaring a fixed retained connection unroutable.
      if (!(cancelled && cancelled()) && sourceFreeStateLimit(max_states)<max_states &&
          repairFromSource(net,binding_index,cancelled,max_states,max_hops,grounding_preemption,
              nullptr,nullptr,false,false,1,nullptr,false,nullptr,false)) {
        timer.success=true; return true;
      }
      if (const char *filter=std::getenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY");
          filter && *filter && net.name.find(filter)!=std::string::npos) {
        const char *path=std::getenv("SCALEPNR_SOURCE_REACHABILITY_LOG");
        std::ofstream out(path && *path ? path : "source_reachability.log",std::ios::app);
        if (!out) throw std::runtime_error("cannot open source reachability audit log");
        if (!auditNetRouteReachability(net,binding_index,out))
          auditNetRouteReachability(net,binding_index,out,false);
      }
      // Opt-in, read-only evidence at the actual repair frontier. The normal
      // task failure location may instead fall back to the retained sink pin.
      if (audit_failure) {
        std::unordered_set<Tile *> audited;
        auto audit = [&](Tile *tile) {
          if (tile && audited.insert(tile).second)
            audit_failure(*tile);
        };
        audit(start);
        audit(finish);
        // Bound diagnostic output when an unrelated failure exhausts a large
        // search. Include every reached Tile for the small-frontier case.
        for (size_t i = 0; i < std::min<size_t>(steps.size(), 16); ++i) {
          const auto &step = steps[i];
          audit(step.tile);
          if (!step.tile->cb_type) continue;
          const auto *sources = step.tile->cb_type->srcNodes(CB_NODE_DST, step.dst);
          if (!sources) continue;
          for (int src : *sources) {
            if (step.tile->cb.src.jump.testBit(src)) continue;
            for (const auto &target : Device::current().resolveJumpTargets(*step.tile, src))
              audit(target.tile);
          }
        }
      }
      return false;
    }
    std::vector<Wire> replacement;
    timer.next("exact_gap_commit");
    for (int i = reached; steps[i].parent >= 0; i = steps[i].parent)
      replacement.push_back(steps[i].edge);
    std::reverse(replacement.begin(), replacement.end());
    if (terminal_tail) replacement.push_back(grounding);
    if (replacement.empty()) return false;

    // Discover replicas through the retained boundary, including aliases in
    // other RTL nets. Construct all new vectors before committing any lease.
    struct Edit { rtl::Net *net; std::vector<Wire> *route; std::vector<Wire> next; };
    std::vector<Edit> edits;
    std::unordered_set<rtl::Net *> nets{&net};
    for (auto &ref : start->routedNets) if (ref.peer) nets.insert(ref.peer);
    for (auto *other_net : nets) for (auto &other : other_net->routes) {
      if (!same_source(other)) continue;
      auto *wires = bindingRoute(other);
      if (!wires || std::any_of(edits.begin(), edits.end(), [&](const auto &e) { return e.route == wires; })) continue;
      auto found = std::find_if(wires->begin(), wires->end(), [&](const auto &w) {
        return w.type == Wire::WIRE_GAP && sameCoord(w.from, gap.from) &&
            sameCoord(w.to, gap.to) && w.from_node == gap.from_node &&
            w.to_node_type == gap.to_node_type && w.to_node == gap.to_node;
      });
      if (found == wires->end()) continue;
      std::vector<Wire> next(wires->begin(), found);
      for (auto w : replacement) {
        // A donor or another repaired replica can have a different prefix
        // (including an equivalent numeric-edge representation). Common-prefix
        // promotion cannot protect these later hops when that route is removed.
        // Every repaired segment therefore keeps independent lease claims.
        w.shared = false;
        next.push_back(std::move(w));
      }
      next.insert(next.end(), found + 1, wires->end());
      edits.push_back({other_net, wires, std::move(next)});
    }
    if (edits.empty() || (cancelled && cancelled())) return false;
    for (const auto &w : replacement) {
      auto *tile = Device::current().getTile(w.from.x, w.from.y);
      if (w.jump >= 0) tile->cb.src.jump.setBit(w.jump);
      if (w.owns_dst) tile->cb.dst.jump.setBit(w.local);
      if (w.joint >= 0) tile->cb.joint.jump.setBit(w.joint);
      if (w.joint2 >= 0) tile->cb.joint.jump.setBit(w.joint2);
    }
    for (auto &edit : edits) edit.route->swap(edit.next);
    std::unordered_map<rtl::Net *, std::vector<std::vector<Wire>>> affected;
    for (auto &edit : edits) affected[edit.net].push_back(std::move(edit.next));
    for (const auto &[n, old] : affected) rebuildNetRouteTiles(*n, old);
    timer.next("exact_gap_search");
  }
  timer.success=true;
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
  // The removed hop owns its incoming DST. Transfer that lease back to the
  // retained prefix before releasing the hop, just as for any other cut.
  // Otherwise a retry prefix ends on an unreserved node another net can take.
  return step_index == 0 ? unrouteNetRoute(net, route_binding_index)
                         : truncateNetRoute(net, route_binding_index, step_index);
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

// Match pre-move anchor selection to the fabric retained during sink detachment.
size_t fpga::retainedRoutePrefixSize(const std::vector<Wire> &route,
                                   bool source_moves, bool destination_moves) {
  if (source_moves) {
    return 0;
  }
  if (!destination_moves) {
    return route.size();
  }
  size_t keep = route.size();
  while (keep > 0 && route[keep - 1].type == Wire::WIRE_TILE_PIN) {
    --keep;
  }
  if (keep > 0 && route[keep - 1].type == Wire::WIRE_CROSSBAR) {
    const Wire &entry = route[keep - 1];
    if (entry.jump < 0 || sameCoord(entry.from, entry.to)) {
      --keep;
    }
  }
  return keep;
}

// Release the terminal while preserving proven private and live shared fabric.
static bool detachNetRouteDestinationImpl(
    rtl::Net &net, size_t route_binding_index,
    const std::unordered_set<const rtl::NetRouteBinding *> *excluded,
    bool retain_private_prefix, size_t proven_prefix = 0) {
  RouteHistoryScope history(&net, __func__);
  if (route_binding_index >= net.routes.size()) {
    return false;
  }
  rtl::NetRouteBinding &binding = net.routes[route_binding_index];
  std::vector<Wire> *route = bindingRoute(binding);
  if (!route || route->empty()) {
    return false;
  }

  size_t keep = std::max(proven_prefix,
                         liveSharedPrefixLength(net, route_binding_index, excluded));

  if (retain_private_prefix) {
    // Without an explicit proof, only a completed route supplies a reusable
    // private prefix. Unproven incomplete tails must still be released.
    keep = std::max(keep, retainedRoutePrefixSize(*route, false, true));
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

bool fpga::invalidateMovedSinkRoutes(
    const std::vector<NetRouteRef> &routes,
    const std::vector<ProvenRoutePrefix> &proven_prefixes) {
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

  // Validate every proof before changing ownership or releasing any leases.
  std::unordered_map<const rtl::NetRouteBinding *, size_t> proven;
  for (const ProvenRoutePrefix &prefix : proven_prefixes) {
    const NetRouteRef &ref = prefix.route;
    if (!ref.net || ref.binding_index >= ref.net->routes.size()) {
      return false;
    }
    auto *binding = &ref.net->routes[ref.binding_index];
    const auto *route = bindingRoute(*binding);
    if (!excluded.contains(binding) || !route || prefix.fragments == 0 ||
        prefix.fragments > retainedRoutePrefixSize(*route, false, true) ||
        (*route)[prefix.fragments - 1].type != Wire::WIRE_CROSSBAR) {
      return false;
    }
    proven[binding] = std::max(proven[binding], prefix.fragments);
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
    // A selected anchor replaces the default complete-route retention boundary.
    auto prefix = proven.find(binding);
    changed = detachNetRouteDestinationImpl(
                  *ref.net, ref.binding_index, &excluded,
                  prefix == proven.end() && complete.contains(binding),
                  prefix == proven.end() ? 0 : prefix->second) ||
              changed;
  }
  return changed;
}

bool fpga::discardNetBranch(rtl::Net &net, size_t route_binding_index) {
  RouteHistoryScope history(&net, __func__);
  // A branch's private suffix may now be another branch's shared prefix.
  // Preserve those owners before clearing this binding, release only unused
  // leases, and remove registrations on its entire old path (shared part too).
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
