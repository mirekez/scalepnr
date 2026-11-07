#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/DockingCutChecks.h"
#include "route/RouteDesign.h"

#include <array>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}


// Long registered branches make per-node full-fragment scans expensive even
// though the requested resources all belong to just one transit crossbar.
struct Fixture {
  static constexpr int branches = 32, hops = 128;
  fpga::Device &device = fpga::Device::current();
  Referable<rtl::Net> net;
  rtl::Inst driver;
  std::array<rtl::Inst, branches> sinks;
  Fixture() {
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    auto &cb = device.cb_types.back();
    cb.name = "matrix_pattern_17";
    device.grid_spec.size = {hops + 1, 1};
    device.size_width = hops + 1;
    device.size_height = 1;
    device.tile_grid.resize(hops + 1);
    net.name = "signal_pattern_91";
    for (int x = 0; x <= hops; ++x) {
      auto &tile = device.tile_grid[x];
      tile.coord = tile.cb_coord = {x, 0};
      tile.cb_type = &cb;
      tile.cb.type = &cb;
    }
    for (int i = 0; i < branches; ++i) {
      const int id = i == branches - 1 ? 1 : i;
      for (auto type : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST,
                        fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL})
        cb.rememberNodeName(type, i, "pattern_" + std::to_string(type) + "_" + std::to_string(i));
      auto &route = sinks[i].wires.emplace_back();
      for (int x = 0; x < hops; ++x) {
        fpga::Wire w;
        w.from = {x, 0}; w.to = {x + 1, 0};
        w.pos = x != 0;
        w.local = w.jump = w.dst = id;
        w.joint = id; w.joint2 = id + branches;
        // Include shared replicas, explicit landing owners, and non-owning
        // fork arrivals. The index must match the actual lease semantics.
        w.shared = i % 7 == 0;
        w.owns_landing = i % 5 == 0;
        w.owns_dst = i % 3 != 0;
        route.push_back(w);
      }
      fpga::Wire edge;
      edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
      edge.from = edge.to = {64, 0};
      edge.from_node_type = fpga::CB_NODE_DST; edge.from_node = i + 64;
      edge.to_node_type = fpga::CB_NODE_LOCAL; edge.to_node = id;
      edge.shared = i % 2 == 0;
      route.push_back(edge);
      // Duplicate fragments must not duplicate a binding in the owner list.
      route.push_back(edge);
      const auto binding = fpga::attachNetRoute(net, sinks[i], 0, &driver,
          &sinks[i], "OUT", "IN", "branch_pattern_" + std::to_string(i));
      fpga::registerNetRouteTiles(net, route, binding);
      for (const auto &w : route) {
        auto &at = device.tile_grid[w.from.x];
        if (w.type == fpga::Wire::WIRE_CROSSBAR && w.owns_landing)
          device.tile_grid[w.to.x].cb.dst.jump.setBit(w.dst);
        if (w.shared) continue;
        if (w.type == fpga::Wire::WIRE_ROUTE_EDGE) {
          at.cb.local.local.setBit(w.to_node);
        } else {
          at.cb.src.jump.setBit(w.jump);
          at.cb.joint.jump.setBit(w.joint);
          at.cb.joint.jump.setBit(w.joint2);
          if (!w.pos) at.cb.local.local.setBit(w.local);
          else if (w.owns_dst) at.cb.dst.jump.setBit(w.local);
        }
      }
    }
  }
};
}

void docking_ownership_work_regression() {
  Fixture f;
  pnr::RouteDesign router;
  rtl::Inst requester;
  f.driver.tile.set(&f.device.tile_grid.front());
  for (auto &sink : f.sinks) sink.tile.set(&f.device.tile_grid.back());
  auto &tile = f.device.tile_grid[64];
  const auto before = tile.cb;
  const auto revision = fpga::routeMutationRevision();
  fpga::RouteOwnerQueries owners;
  size_t checks = 0, accepted = 0, rejected = 0;
  // Different candidate edges query different nodes on the same congested
  // crossbar. Per-cut memoization alone cannot avoid those route rescans.
  for (int attempt = 0; attempt < 8; ++attempt) {
    pnr::DockingCutChecks cuts([&](const fpga::RouteCutNode &cut) {
      ++checks;
      return pnr::provisionalDockingCutsEligible(&router, {&cut, 1},
          nullptr, &requester, "OUT", owners);
    });
    for (auto type : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST, fpga::CB_NODE_JOINT})
      for (int node = 0; node < 96; ++node) {
        const fpga::RouteCutNode cut{tile.coord, type, node};
        const auto expected = fpga::findNetOwnersByNode(tile, type, node);
        bool eligible = !expected.empty();
        for (const auto &owner : expected)
          eligible &= fpga::canPreemptNetRouteFromNodes(*owner.net, owner.binding_index, {cut});
        for (int repeat = 0; repeat < 3; ++repeat) {
          // Actual production preflight must agree with the exhaustive audit,
          // including occupied transit resources and nodes without an owner.
          require(cuts.eligible({&cut, 1}) == eligible, "docking ownership policy changed");
        }
        eligible ? ++accepted : ++rejected;
      }
  }
  const auto work = owners.work();
  std::cout << "docking ownership checks=" << checks << " queries=" << work.queries
            << " bindings=" << work.binding_resolutions << " fragments=" << work.fragment_visits
            << " builds=" << work.tile_builds << std::endl;
  // Preflight must not release victims or alter the live congestion state.
  require(fpga::routeMutationRevision() == revision && tile.cb.src.jump == before.src.jump &&
          tile.cb.dst.jump == before.dst.jump && tile.cb.joint.jump == before.joint.jump,
          "docking preflight modified leases");
  require(accepted && rejected && checks == 2304, "fixture missed busy/free cut cases");
  // The original caller bypassed the index entirely (queries=0); native CPU
  // samples attributed 19.9% to its repeated exhaustive ownership scans.
  require(work.queries == checks && work.tile_builds == 1 &&
          work.binding_resolutions <= Fixture::branches && work.fragment_visits <= 4529,
          "docking preflight bypassed the shared ownership index");
  f.net.route_protected = true;
  const fpga::RouteCutNode protected_cut{tile.coord, fpga::CB_NODE_SRC, 2};
  // Index only ownership, never the policy: changing protection must reject a
  // previously eligible cut even without a route publication/revision change.
  require(!pnr::provisionalDockingCutsEligible(&router, {&protected_cut, 1},
              nullptr, &requester, "OUT", owners), "cached policy allowed protected victim");
}

void ownership_query_work_regression() {
  Fixture f;
  auto &tile = f.device.tile_grid[64];
  bool bounded_work = true;
  for (bool authoritative : {true, false}) {
    tile.routed_bindings_authoritative = authoritative;
    fpga::RouteOwnerQueries queries;
    size_t query_count = 0;
    for (bool transit : {false, true})
      for (auto kind : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST,
                        fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL})
        for (int node = 0; node < 96; ++node) {
          const auto expected = fpga::findNetOwnersByNode(tile, kind, node, transit);
          for (int repeat = 0; repeat < 8; ++repeat) {
            size_t visited = 0;
            const bool found = queries.allMatch(tile, kind, node, [&](const auto &owner) {
              // Every matching alias must remain visible in registration order.
              require(visited < expected.size() && owner.net == expected[visited].net &&
                      owner.binding_index == expected[visited].binding_index,
                      "indexed owner identity/order differs from exhaustive scan");
              ++visited;
              return true;
            }, transit);
            require(found == !expected.empty() && visited == expected.size(),
                    "indexed query lost an owner or accepted an unowned node");
            ++query_count;
          }
          size_t rejected = 0;
          require(!queries.allMatch(tile, kind, node, [&](const auto &) {
            ++rejected; return false;
          }, transit) && rejected == size_t(!expected.empty()),
                  "cached ownership incorrectly cached predicate acceptance");
          ++query_count;
        }
    const auto work = queries.work();
    std::cout << "ownership authoritative=" << authoritative << " queries=" << work.queries
              << " bindings=" << work.binding_resolutions << " fragments=" << work.fragment_visits
              << " tile_builds=" << work.tile_builds << std::endl;
    require(work.queries == query_count, "ownership query counter lost work");
    // Red baseline: 217590 binding resolutions and 28167468 fragment visits
    // for 6912 queries. The fixed implementation needs 32 and 4529; preserve
    // those improved bounds, including singleton lease-predicate inspections.
    bounded_work &= work.binding_resolutions <= Fixture::branches &&
                    work.fragment_visits <= 4529 &&
                    work.tile_builds == 1;
  }
  require(bounded_work, "per-node queries repeatedly resolved bindings and scanned route fragments");
  tile.routed_bindings_authoritative = true;
  fpga::RouteOwnerQueries queries;
  auto accept = [](const auto &) { return true; };
  require(queries.allMatch(tile, fpga::CB_NODE_SRC, 2, accept),
          "missing initial owner before mutation test");
  // A production unroute changes the revision and clears the lease; the same
  // query object must not keep the released owner alive.
  require(fpga::unrouteNetRoute(f.net, 2), "test route was not unrouted");
  require(!queries.allMatch(tile, fpga::CB_NODE_SRC, 2, accept) &&
          !tile.cb.src.jump.testBit(2), "unroute left a cached owner or lease");
  // Re-publication can change vector storage and binding lookup indexes.
  // An existing query must see the new owner without remembering pointers.
  f.sinks[2].wires[0] = f.sinks[3].wires[0];
  fpga::registerNetRouteTiles(f.net, f.sinks[2].wires[0], 2);
  size_t owners = 0;
  require(queries.allMatch(tile, fpga::CB_NODE_SRC, 3, [&](const auto &) {
    ++owners; return true;
  }) && owners == 2, "route publication did not invalidate cached ownership");
  require(queries.work().tile_builds == 3, "mutation did not rebuild exactly once per revision");
  // Acceptance must still examine a later foreign owner, even after a prior
  // query accepted every owner of the shared node.
  rtl::Inst foreign_driver;
  f.net.routes.back().from = &foreign_driver;
  fpga::registerNetRouteTiles(f.net, f.sinks.back().wires[0], Fixture::branches - 1);
  size_t visited = 0;
  require(!queries.allMatch(tile, fpga::CB_NODE_SRC, 1, [&](const auto &owner) {
    ++visited;
    return owner.net->routes[owner.binding_index].from == &f.driver;
  }) && visited == 2, "cached query hid a later foreign driver");
  // A broad search must retain unchanged owners across more than 64 Tiles.
  // Revisiting an earlier Tile must preserve the answer without rebuilding it.
  for (int x = 0; x < 100; ++x)
    require(queries.allMatch(f.device.tile_grid[x], fpga::CB_NODE_SRC, 3, accept),
            "query index lost a live owner on a different tile");
  const auto builds_before_revisit = queries.work().tile_builds;
  require(queries.allMatch(f.device.tile_grid[0], fpga::CB_NODE_SRC, 3, accept) &&
          queries.work().tile_builds == builds_before_revisit,
          "revisiting an unchanged tile rebuilt its ownership");
  // Terminal joints/DSTs and source-local takeoffs are owners but not transit
  // victims. Include a same-tile hop and a pin fragment in this smaller control.
  auto &route = f.sinks[2].wires[0];
  const size_t first = route.size();
  fpga::Wire terminal;
  terminal.from = terminal.to = tile.coord;
  terminal.pos = 1; terminal.local = 110; terminal.joint = 111;
  route.push_back(terminal);
  fpga::Wire pin;
  pin.type = fpga::Wire::WIRE_TILE_PIN;
  pin.from = pin.to = tile.coord; pin.local = 112;
  route.push_back(pin);
  fpga::Wire takeoff;
  takeoff.from = takeoff.to = tile.coord;
  takeoff.pos = 0; takeoff.local = 113; takeoff.joint = 114;
  takeoff.jump = 115; takeoff.dst = 116; takeoff.owns_landing = true;
  route.push_back(takeoff);
  tile.cb.dst.jump.setBit(110); tile.cb.dst.jump.setBit(116);
  tile.cb.joint.jump.setBit(111); tile.cb.joint.jump.setBit(114);
  tile.cb.local.local.setBit(112); tile.cb.local.local.setBit(113);
  tile.cb.src.jump.setBit(115);
  fpga::registerNetRouteTilesFrom(f.net, route, first, 2);
  for (bool transit : {false, true})
    for (auto kind : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST,
                      fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL})
      for (int node = 110; node <= 116; ++node) {
        const auto expected = fpga::findNetOwnersByNode(tile, kind, node, transit);
        size_t seen = 0;
        const bool found = queries.allMatch(tile, kind, node, [&](const auto &owner) {
          require(seen < expected.size() && owner.net == expected[seen].net &&
                  owner.binding_index == expected[seen].binding_index,
                  "terminal/takeoff query changed an owner");
          ++seen; return true;
        }, transit);
        require(found == !expected.empty() && seen == expected.size(),
                "terminal/takeoff transit eligibility differs from lease predicate");
      }
}
