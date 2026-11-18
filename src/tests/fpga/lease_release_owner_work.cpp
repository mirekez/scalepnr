#include "RoutePreemption.h"
#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"

#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

struct Fixture {
  static constexpr int branches = 33;
  static constexpr int private_hops = 112;
  static constexpr int shared_hops = 16;
  static constexpr int shared_base = 3800;

  fpga::Device& device = fpga::Device::current();
  Referable<rtl::Net> net;
  rtl::Inst driver;
  std::array<rtl::Inst, branches> sinks;
  std::array<size_t, branches> bindings{};
  std::array<bool, CB_MAX_NODES> mapped_sources{};

  Fixture() {
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.emplace_back();
    auto& cb = device.cb_types.back();
    cb.name = "release_owner_matrix";
    cb.type_id = cb.base_type_id = 0;
    device.grid_spec.size = {2, 1};
    device.size_width = 2;
    device.size_height = 1;
    device.tile_grid.resize(2);
    for (int x = 0; x < 2; ++x) {
      auto& tile = device.tile_grid[x];
      tile.coord = tile.cb_coord = {x, 0};
      tile.cb_type = &cb;
      tile.cb.type = &cb;
    }
    net.name = "release_owner_signal";
    for (int branch = 0; branch < branches; ++branch) {
      auto& route = sinks[branch].wires.emplace_back();
      const int base = privateBase(branch);
      for (int hop = 0; hop < private_hops + shared_hops; ++hop) {
        fpga::Wire wire;
        wire.from = {hop % 2, 0};
        wire.to = {1 - hop % 2, 0};
        wire.pos = 1;
        if (hop < private_hops) {
          wire.local = wire.jump = base + hop;
          wire.dst = base + hop + 1;
        } else {
          const int tail = hop - private_hops;
          wire.local = tail == 0 ? base + private_hops : shared_base + tail;
          wire.jump = shared_base + tail;
          wire.dst = shared_base + tail + 1;
          wire.shared = branch != 0 && tail != 0;
          wire.owns_landing = branch == 0 && tail + 1 == shared_hops;
        }
        cb.dst_src[wire.local].jump.setBit(wire.jump);
        cb.rememberNodeName(fpga::CB_NODE_DST, wire.local,
            "release_dst_" + std::to_string(wire.local));
        cb.rememberNodeName(fpga::CB_NODE_SRC, wire.jump,
            "release_src_" + std::to_string(wire.jump));
        cb.rememberNodeName(fpga::CB_NODE_DST, wire.dst,
            "release_dst_" + std::to_string(wire.dst));
        if (!mapped_sources[wire.jump]) {
          fpga::CBType::ResolvedJump jump{};
          jump.delta = wire.to - wire.from;
          jump.target_cb_type_id = 0;
          jump.dsts.jump.setBit(wire.dst);
          cb.dst_by_src[wire.jump].push_back(jump);
          mapped_sources[wire.jump] = true;
        }
        device.tile_grid[wire.to.x].incoming_dst_nodes.setBit(wire.dst);
        route.push_back(wire);
        if (!wire.shared) {
          auto& from = device.tile_grid[wire.from.x];
          from.cb.dst.jump.setBit(wire.local);
          from.cb.src.jump.setBit(wire.jump);
        }
        if (wire.owns_landing)
          device.tile_grid[wire.to.x].cb.dst.jump.setBit(wire.dst);
      }
      bindings[branch] = fpga::attachNetRoute(net, sinks[branch], 0,
          &driver, &sinks[branch], "OUT", "IN",
          "branch_" + std::to_string(branch));
      fpga::registerNetRouteTiles(net, route, bindings[branch]);
    }
    cb.rebuildOutgoingSrcs();
  }

  static int privateBase(int branch) { return 20 + branch * (private_hops + 1); }

  void audit() {
    for (auto& tile : device.tile_grid) {
      std::ostringstream proof;
      const auto state = fpga::auditTileCongestion(tile, proof, {&net});
      require(!state.orphan_leases && !state.missing_leases &&
          !state.missing_registrations && !state.stale_registrations &&
          !state.conflicting_driver_nodes, proof.str());
    }
  }

  void checkSurvivors(const std::array<std::vector<fpga::Wire>, branches>& before,
                      int first_cut_branch) {
    for (int branch = 0; branch < branches; ++branch) {
      if (branch >= first_cut_branch) continue;
      const auto& route = sinks[branch].wires.front();
      require(route.size() == before[branch].size(), "survivor length changed");
      for (size_t i = 0; i < route.size(); ++i) {
        const auto& a = route[i];
        const auto& b = before[branch][i];
        require(a.type == b.type && a.from.x == b.from.x && a.from.y == b.from.y &&
            a.to.x == b.to.x && a.to.y == b.to.y && a.pos == b.pos &&
            a.local == b.local && a.jump == b.jump && a.dst == b.dst &&
            a.joint == b.joint && a.joint2 == b.joint2 &&
            a.shared == b.shared && a.owns_dst == b.owns_dst &&
            a.owns_landing == b.owns_landing,
            "survivor geometry or ownership changed");
      }
      require(device.tile_grid[0].cb.src.jump.testBit(privateBase(branch) + 50),
          "survivor private transit lease cleared");
    }
    require(device.tile_grid[0].cb.src.jump.testBit(shared_base) &&
        device.tile_grid[0].cb.dst.jump.testBit(shared_base + shared_hops),
        "surviving shared tail lost a lease");
  }

  fpga::OwnershipQueryWork cut(int branch, bool preserve_downstream = false) {
    const auto before_revision = fpga::routeMutationRevision();
    const int base = privateBase(branch);
    std::vector<fpga::RouteCutNode> cuts{{{0, 0}, fpga::CB_NODE_SRC, base + 2}};
    if (preserve_downstream)
      cuts.push_back({{0, 0}, fpga::CB_NODE_SRC, base + 110});
    const auto victims = fpga::findNetRouteCutVictims(cuts);
    require(victims.size() == 1 && victims.front().net == &net &&
        victims.front().binding_index == bindings[branch],
        "private transit cut selected another binding");
    fpga::OwnershipQueryWork work;
    require(fpga::unrouteNetRoutesFromNodes(victims, cuts, preserve_downstream, &work),
        "private transit unroute failed");
    require(fpga::routeMutationRevision() > before_revision,
        "unroute did not invalidate owner revision");
    const auto& route = sinks[branch].wires.front();
    require(route.size() == size_t(preserve_downstream ? 3 + shared_hops : 2),
        "private transit did not retain the expected prefix and tail");
    if (preserve_downstream) {
      require(route[2].type == fpga::Wire::WIRE_GAP &&
          route[2].from_node == base + 2 &&
          route[2].to_node == base + private_hops &&
          route[3].local == base + private_hops &&
          route.back().dst == shared_base + shared_hops &&
          fpga::hasRouteGaps(route),
          "preserved long tail is not connected to the intended gap boundaries");
      require(!device.tile_grid[0].cb.src.jump.testBit(base + 50) &&
          !device.tile_grid[0].cb.src.jump.testBit(base + 110) &&
          device.tile_grid[0].cb.dst.jump.testBit(base + private_hops),
          "preserved tail kept private removed nodes or lost its landing");
    }
    require(device.tile_grid[0].cb.dst.jump.testBit(base + 2) &&
        !device.tile_grid[0].cb.src.jump.testBit(base + 2) &&
        !device.tile_grid[1].cb.src.jump.testBit(base + 3),
        "private lease release lost the prefix or retained removed nodes");
    require(fpga::findNetOwnersByNode(device.tile_grid[0], fpga::CB_NODE_SRC,
        base + 2).empty(), "removed private node still has an owner");
    return work;
  }
};
}

void lease_release_owner_work_regression() {
  fpga::OwnershipQueryWork first, second;
  {
    Fixture fixture;
    fixture.audit();
    std::array<std::vector<fpga::Wire>, Fixture::branches> original;
    for (int i = 0; i < Fixture::branches; ++i)
      original[i] = fixture.sinks[i].wires.front();

    first = fixture.cut(Fixture::branches - 1);
    fixture.checkSurvivors(original, Fixture::branches - 1);
    fixture.audit();
    const auto first_prefix = fixture.sinks.back().wires.front();
    second = fixture.cut(Fixture::branches - 2);
    fixture.checkSurvivors(original, Fixture::branches - 2);
    const auto& prefix = fixture.sinks.back().wires.front();
    require(prefix.size() == first_prefix.size() &&
        prefix.back().dst == first_prefix.back().dst &&
        prefix.back().owns_landing == first_prefix.back().owns_landing,
        "later release changed the earlier retained prefix");
    fixture.audit();
  }

  Fixture retained;
  retained.audit();
  std::array<std::vector<fpga::Wire>, Fixture::branches> retained_original;
  for (int i = 0; i < Fixture::branches; ++i)
    retained_original[i] = retained.sinks[i].wires.front();
  const auto retained_first = retained.cut(Fixture::branches - 1, true);
  retained.checkSurvivors(retained_original, Fixture::branches - 1);
  retained.audit();
  const auto retained_prefix = retained.sinks.back().wires.front();
  const auto retained_second = retained.cut(Fixture::branches - 2, true);
  retained.checkSurvivors(retained_original, Fixture::branches - 2);
  const auto& still_retained = retained.sinks.back().wires.front();
  require(still_retained.size() == retained_prefix.size() &&
      still_retained[2].type == fpga::Wire::WIRE_GAP &&
      still_retained[2].to_node == retained_prefix[2].to_node &&
      still_retained.back().dst == retained_prefix.back().dst,
      "later retained-tail release changed the earlier gap or tail");
  retained.audit();

  std::cout << "LEASE_RELEASE_WORK first_queries=" << first.queries
      << " first_bindings=" << first.binding_resolutions
      << " first_fragments=" << first.fragment_visits
      << " first_tile_builds=" << first.tile_builds
      << " second_queries=" << second.queries
      << " second_bindings=" << second.binding_resolutions
      << " second_fragments=" << second.fragment_visits
      << " second_tile_builds=" << second.tile_builds
      << " retained_first_queries=" << retained_first.queries
      << " retained_first_bindings=" << retained_first.binding_resolutions
      << " retained_first_fragments=" << retained_first.fragment_visits
      << " retained_first_tile_builds=" << retained_first.tile_builds
      << " retained_second_queries=" << retained_second.queries
      << " retained_second_bindings=" << retained_second.binding_resolutions
      << " retained_second_fragments=" << retained_second.fragment_visits
      << " retained_second_tile_builds=" << retained_second.tile_builds << std::endl;
  for (const auto& work : {first, second, retained_first, retained_second}) {
    // Keep the actual release workload, not a shortcut that omits ownership checks.
    require(work.queries >= 100, "fixture omitted release owner queries");
    // Each transaction may inspect the two Tiles and their registered bindings once.
    require(work.tile_builds == 2 && work.binding_resolutions <= 2 * Fixture::branches,
        "lease release rebuilt unchanged Tile ownership");
    // Baseline: 877,314-1,027,908 visits; indexed release needs 26,910-27,906.
    require(work.fragment_visits <= 30000,
        "lease release repeated full-fragment owner scans");
  }
}
