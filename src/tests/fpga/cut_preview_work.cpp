#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace {
using Node = std::tuple<int, int, int, int>;
void check(bool ok, const char *why) {
  if (!ok) throw std::runtime_error(why);
}
Node key(const fpga::RouteCutNode &n) {
  return {n.tile.x, n.tile.y, int(n.type), n.node};
}
}

// Example 066 timed out in Destination Moving, repeatedly scanning registered
// routes while previewing Grounding cuts. Its last witness crosses SRC1030 at
// (49,10) and DST1027 at (53,10), owned by binding 16 of this captured tree.
// Preserve all 17 sibling vectors, including shared prefixes and landing flags.
// This tests the actual read-only cut preview, not routing on a toy shortcut.
// Other chip nets are omitted; even this one tree exposes per-resource rescans.
void cut_preview_work_regression() {
  auto &device = fpga::Device::current();
  device.tile_grid.clear(); device.cb_types.clear();
  device.size_width = 100; device.size_height = 40;
  device.grid_spec.size = {100, 40}; device.tile_grid.resize(100 * 40);
  auto &cb = device.cb_types.emplace_back(); cb.name = "pattern_matrix_607";
  for (int y = 0; y < 40; ++y) for (int x = 0; x < 100; ++x) {
    auto &t = *device.getTile(x, y); t.coord = t.cb_coord = {x, y};
    t.cb_type = t.cb.type = &cb;
  }
  Referable<rtl::Net> net; net.name = "pattern_signal_613";
  rtl::Inst driver;
  std::array<rtl::Inst, 17> sinks;
  std::array<std::vector<fpga::Wire>, 17> captured;
  std::ifstream file(std::filesystem::path(__FILE__).parent_path() / "cut_preview_snapshot.txt");
  check(bool(file), "missing cut preview snapshot");
  for (std::string line; std::getline(file, line);) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream in(line); char tag; int index, type; fpga::Wire w;
    in >> tag >> index >> w.from.x >> w.from.y >> w.to.x >> w.to.y
       >> w.resource.x >> w.resource.y >> type >> w.local >> w.pos >> w.jump
       >> w.route_jump >> w.dst >> w.joint >> w.joint2 >> w.from_node_type
       >> w.from_node >> w.to_node_type >> w.to_node >> w.resource_node
       >> w.pin_dir >> w.shared >> w.owns_dst >> w.owns_landing;
    check(bool(in) && tag == 'W' && index >= 0 && index < 17, "invalid captured fragment");
    w.type = fpga::Wire::Type(type); captured[index].push_back(w);
    for (auto [kind, node] : {std::pair{fpga::CB_NODE_SRC, w.jump},
                            {fpga::CB_NODE_DST, w.dst}, {fpga::CB_NODE_JOINT, w.joint},
                            {fpga::CB_NODE_JOINT, w.joint2},
                            {w.pos == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST, w.local}})
      if (node >= 0) cb.rememberNodeName(kind, node,
          "pattern_node_" + std::to_string(kind) + "_" + std::to_string(node));
  }
  size_t fragments = 0;
  for (size_t i = 0; i < sinks.size(); ++i) {
    check(fpga::isRouteComplete(captured[i]), "captured completed sibling changed");
    fragments += captured[i].size(); sinks[i].wires.push_back(captured[i]);
    auto b = fpga::attachNetRoute(net, sinks[i], 0, &driver, &sinks[i],
        "OUT", "IN", "pattern_branch_" + std::to_string(i));
    fpga::registerNetRouteTiles(net, sinks[i].wires[0], b);
  }
  // Install only leases with real owners, including explicitly owned landings.
  auto lease = [&](fpga::Coord at, fpga::CBNodeNameType kind, int node) {
    if (node < 0) return;
    auto *t = device.getTile(at.x, at.y); check(t, "captured node outside grid");
    if (kind == fpga::CB_NODE_LOCAL) t->cb.local.local.setBit(node);
    else if (kind == fpga::CB_NODE_JOINT) t->cb.joint.jump.setBit(node);
    else if (kind == fpga::CB_NODE_SRC) t->cb.src.jump.setBit(node);
    else if (kind == fpga::CB_NODE_DST) t->cb.dst.jump.setBit(node);
  };
  std::set<std::pair<int,int>> used_tiles;
  for (const auto &route : captured) for (const auto &w : route) {
    used_tiles.emplace(w.from.x, w.from.y); used_tiles.emplace(w.to.x, w.to.y);
    if (w.type == fpga::Wire::WIRE_CROSSBAR && w.owns_landing)
      lease(w.to, fpga::CB_NODE_DST, w.dst);
    if (w.shared) continue;
    if (w.type == fpga::Wire::WIRE_TILE_PIN) {
      lease(w.from, fpga::CB_NODE_LOCAL, w.local);
      device.getTile(w.from.x, w.from.y)->pin_state.leased_nodes.setBit(w.local);
    } else if (w.type == fpga::Wire::WIRE_CROSSBAR) {
      lease(w.from, fpga::CB_NODE_SRC, w.jump);
      lease(w.from, fpga::CB_NODE_JOINT, w.joint);
      lease(w.from, fpga::CB_NODE_JOINT, w.joint2);
      if (w.pos == 0 || w.owns_dst)
        lease(w.from, w.pos == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST, w.local);
    } else if (w.type == fpga::Wire::WIRE_ROUTE_EDGE)
      lease(w.to, fpga::CBNodeNameType(w.to_node_type), w.to_node);
  }
  auto audit = [&] {
    fpga::CongestionAuditIndex index({&net});
    for (auto [x,y] : used_tiles) {
      std::ostringstream report;
      const auto state = fpga::auditTileCongestion(*device.getTile(x,y), report, index);
      if (state.orphan_leases || state.missing_leases || state.missing_registrations ||
          state.stale_registrations || state.conflicting_driver_nodes)
        throw std::runtime_error(report.str());
    }
  };
  audit();
  const std::vector<fpga::RouteCutNode> cuts{
      {{49,10}, fpga::CB_NODE_SRC, 1030}, {{53,10}, fpga::CB_NODE_DST, 1027}};
  const auto victims = fpga::findNetRouteCutVictims(cuts);
  check(victims.size() == 1 && victims[0].net == &net && victims[0].binding_index == 16,
        "captured cut must select exactly binding 16");
  bool bounded = true;
  const auto revision = fpga::routeMutationRevision();
  for (bool authoritative : {true, false}) {
    for (auto &t : device.tile_grid) t.routed_bindings_authoritative = authoritative;
    for (bool partial : {true, false}) {
      fpga::OwnershipQueryWork work;
      const auto preview = fpga::previewNetRouteCuts(victims, cuts, partial, &work);
      std::set<Node> nodes; for (const auto &n : preview) nodes.insert(key(n));
      check(nodes.size() == preview.size() && nodes.contains({49,10,3,1030}) &&
            nodes.contains({53,10,4,1027}), "preview lost the actual contested resources");
      check(!nodes.contains({49,10,4,240}), "preview released the retained prefix landing");
      if (partial) {
        const std::set<Node> expected{{49,10,3,1030},{53,10,3,819},{53,10,4,1027}};
        check(nodes == expected, "partial cut changed its exact release footprint");
      }
      std::cout << "cut preview authoritative=" << authoritative << " partial=" << partial
                << " nodes=" << preview.size() << " bindings=" << work.binding_resolutions
                << " fragments=" << work.fragment_visits << " captured=" << fragments << '\n';
      // Each candidate Tile registration may be read once, and each distinct
      // binding's retained fragments once, independent of released-node count.
      bounded &= work.fragment_visits <= (authoritative ? captured[16].size() : fragments) &&
                 work.binding_resolutions <= (authoritative ? 6 : sinks.size());
    }
  }
  check(fpga::routeMutationRevision() == revision, "read-only preview changed routing");
  for (size_t i = 0; i < sinks.size(); ++i) {
    const auto &live = sinks[i].wires[0];
    check(live.size() == captured[i].size(), "preview resized a completed sibling");
    for (size_t j = 0; j < live.size(); ++j) {
      const auto &a = live[j], &b = captured[i][j];
      check(a.type == b.type && a.from == b.from && a.to == b.to && a.resource == b.resource &&
            a.local == b.local && a.pos == b.pos && a.jump == b.jump && a.dst == b.dst &&
            a.route_jump == b.route_jump && a.joint == b.joint && a.joint2 == b.joint2 &&
            a.from_node_type == b.from_node_type && a.from_node == b.from_node &&
            a.to_node_type == b.to_node_type && a.to_node == b.to_node &&
            a.resource_node == b.resource_node && a.pin_dir == b.pin_dir &&
            a.shared == b.shared && a.owns_dst == b.owns_dst && a.owns_landing == b.owns_landing,
            "read-only preview rewrote a captured fragment");
    }
  }
  // A live shared replica is a dependency even though it owns no lease. A
  // partial victim set must not release its transit resources. Publish the
  // alias between previews to ensure no answer outlives this invocation.
  rtl::Inst alias;
  alias.wires.push_back(captured[16]);
  for (auto &w : alias.wires[0]) { w.shared = true; w.owns_landing = false; }
  auto alias_index = fpga::attachNetRoute(net, alias, 0, &driver, &alias,
      "OUT", "IN", "pattern_alias_619");
  fpga::registerNetRouteTiles(net, alias.wires[0], alias_index);
  check(fpga::previewNetRouteCuts(victims, cuts, true).empty(),
        "preview released resources still referenced by an unselected alias");
  auto both = victims; both.push_back({&net, alias_index});
  const auto combined = fpga::previewNetRouteCuts(both, cuts, true);
  std::set<Node> combined_nodes;
  for (const auto &n : combined) combined_nodes.insert(key(n));
  const std::set<Node> expected{{49,10,3,1030},{53,10,3,819},{53,10,4,1027}};
  check(combined_nodes == expected && combined.size() == expected.size(),
        "selecting both shared dependencies changed the release footprint");
  audit();
  check(bounded, "cut preview rescanned route fragments separately for every released node");
}
