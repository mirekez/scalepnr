#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Docking.h"
#include "Wire.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

NodeMask bit(int pos) {
  NodeMask mask{};
  mask.setBit(pos);
  return mask;
}

// Moving must not blacklist a candidate using a smaller graph than the router.
// This corridor has exactly one path, optionally containing LOCAL relays. The
// former DST-only backward frontier rejects it while the real router commits
// it. Names are annotations only; every connection below is numeric.
void checkCorridor(unsigned seed, int direction, int relays, bool joint_sink,
                   bool disconnected) {
  std::mt19937 rng(seed);
  std::array<int, 64> positions{};
  std::iota(positions.begin(), positions.end(), 32);
  std::shuffle(positions.begin(), positions.end(), rng);
  const int output = positions[0], input = positions[1];
  const int relay0 = positions[2], relay1 = positions[3];
  const int landing = positions[4], finish = positions[5], joint = positions[6];
  const int candidate_input = positions[7];
  const int relay2 = positions[8];
  const std::array<fpga::Coord, 4> deltas{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};
  const auto delta = deltas.at(direction);
  const int lane = rng() % 8;
  const int first = ((delta.x & 15) << 8) | ((delta.y & 15) << 4) | lane;
  const int second = first + 8;
  fpga::CBType cb{};
  cb.name = "PATTERN_MATRIX_" + std::to_string(seed);
  cb.type_id = cb.base_type_id = 0;
  auto name = [&](fpga::CBNodeNameType kind, int node, const char *role) {
    cb.rememberNodeName(kind, node, "PATTERN_" + std::to_string(seed) + "_" +
                                      role + "_" + std::to_string(node));
  };
  name(fpga::CB_NODE_LOCAL, output, "OUTPUT");
  name(fpga::CB_NODE_LOCAL, input, "INPUT");
  name(fpga::CB_NODE_LOCAL, candidate_input, "MOVED_INPUT");
  name(fpga::CB_NODE_LOCAL, relay0, "RELAY_A");
  name(fpga::CB_NODE_LOCAL, relay1, "RELAY_B");
  name(fpga::CB_NODE_LOCAL, relay2, "RELAY_C");
  name(fpga::CB_NODE_DST, landing, "ENTRY_A");
  name(fpga::CB_NODE_DST, finish, "ENTRY_B");
  name(fpga::CB_NODE_SRC, first, "EXIT_A");
  name(fpga::CB_NODE_SRC, second, "EXIT_B");
  name(fpga::CB_NODE_JOINT, joint, "JOIN");
  for (auto [src, dst] : {std::pair{first, landing}, std::pair{second, finish}}) {
    fpga::CBType::ResolvedJump jump{};
    jump.delta = delta;
    jump.target_cb_type_id = cb.type_id;
    jump.dsts.jump = bit(dst);
    cb.dst_by_src[src].push_back(jump);
  }
  cb.local_src[output].jump.setBit(first);
  if (relays == 0) {
    if (!disconnected) cb.dst_src[landing].jump.setBit(second);
  } else {
    cb.dst_local[landing].local.setBit(relay0);
    if (relays == 2) cb.local_local[relay0].local.setBit(relay1);
    if (relays == 3) {
      fpga::CBType::ResolvedLocal link{};
      link.delta = delta;
      link.target_cb_type_id = cb.type_id;
      link.target_nodes.setBit(relay1);
      cb.local_by_local[relay0].push_back(link);
      cb.local_local[relay1].local.setBit(relay2);
    }
    if (!disconnected)
      cb.local_src[relays == 3 ? relay2 : relays == 2 ? relay1 : relay0].jump.setBit(second);
  }
  if (joint_sink) {
    cb.dst_joint[finish].joint.setBit(joint);
    cb.joint_local[joint].local.setBit(input);
    cb.joint_local[joint].local.setBit(candidate_input);
  } else {
    cb.dst_local[finish].local.setBit(input);
    cb.dst_local[finish].local.setBit(candidate_input);
  }
  cb.rebuildOutgoingSrcs();

  auto &device = fpga::Device::current();
  // Grid lookup caches use allocation identity. Allocate before releasing the
  // previous grid so each rotated topology is seen as a new device instance.
  const int length = relays == 3 ? 4 : 3;
  std::vector<Referable<fpga::Tile>> fresh_grid(length);
  device.tile_grid.swap(fresh_grid);
  fresh_grid.clear();
  device.cb_types.clear();
  device.cb_types.push_back(cb);
  const int width = delta.x == 0 ? 1 : length;
  const int height = delta.y == 0 ? 1 : length;
  device.grid_spec.size = {width, height};
  device.size_width = width;
  device.size_height = height;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto &tile = device.tile_grid[y * width + x];
      tile.coord = tile.cb_coord = {x, y};
      tile.name = {x, y};
      tile.cb_type = &device.cb_types[0];
      tile.cb = {};
      tile.pin_state = {};
      tile.routedNets.clear();
    }
  }
  auto &source = *device.getTile(delta.x < 0 ? length - 1 : 0,
                                 delta.y < 0 ? length - 1 : 0);
  auto &target = *device.getTile(source.coord.x + (length - 1) * delta.x,
                                 source.coord.y + (length - 1) * delta.y);
  auto index = pnr::buildBackwardResolveIndex(device, target.coord, 5);
  auto probe = [&](fpga::Tile &tile, int src, pnr::BackwardTakeoffChoice &choice) {
    if (&tile != &source || src != first) return false;
    choice = {output, -1, -1};
    return true;
  };
  auto requireUnchanged = [&] {
    // A feasibility query must not lease nodes, mark deadends, or register nets.
    for (const auto &tile : device.tile_grid) {
      require(tile.cb.src.jump == NodeMask{} && tile.cb.dst.jump == NodeMask{} &&
                  tile.cb.joint.jump == NodeMask{} && tile.cb.local.local == NodeMask{} &&
                  tile.cb.src_deadend.jump == NodeMask{} && tile.routedNets.empty(),
              "read-only proof or failed atomic route changed tile state");
    }
  };
  const bool proof = pnr::inputMayReachSource(target, bit(input), source.coord, 5,
      probe, &index, {}, nullptr, nullptr, 200);
  requireUnchanged();

  Referable<rtl::Net> net;
  net.name = "PATTERN_SIGNAL_" + std::to_string(seed);
  Referable<rtl::Cell> source_cell, target_cell;
  source_cell.name = "PATTERN_DRIVER";
  target_cell.name = "PATTERN_RECEIVER";
  rtl::Inst driver, sink;
  driver.cell_ref.set(&source_cell);
  sink.cell_ref.set(&target_cell);
  rtl::NetRouteBinding binding;
  binding.from = &driver;
  binding.to = &sink;
  binding.from_port = "OUTPUT";
  binding.to_port = "INPUT";
  binding.route_name = "PATTERN_ROUTE_" + std::to_string(seed);
  fpga::Wire from, to;
  from.type = to.type = fpga::Wire::WIRE_TILE_PIN;
  from.from = from.to = source.coord;
  from.local = output;
  from.pin_dir = fpga::TILE_PIN_OUTPUT;
  to.from = to.to = target.coord;
  to.local = input;
  to.pin_dir = fpga::TILE_PIN_INPUT;
  std::ostringstream reachability;
  const bool reachable = fpga::auditUnroutedReachability(net, binding, from, to,
                                                        reachability, 512);
  // The explicit topology, not the backward search, is the reference oracle.
  require(reachable == !disconnected, "numeric graph disagrees with corridor topology: " +
                                         reachability.str());
  requireUnchanged();
  const bool routed = fpga::routeUnboundNetFromSource(net, binding, from, to, {}, 512, 200);
  // Demonstrate actual production completion, not just permissive reachability.
  require(routed == !disconnected, "production router disagrees with corridor topology");
  if (routed) {
    require(sink.wires.size() == 1 && fpga::isRouteComplete(sink.wires[0]),
            "production router did not commit a complete route");
    // A completed real owner now occupies the corridor. A new unanchored
    // proof must not reuse its leases, even though connectivity is unchanged.
    require(!pnr::inputMayReachSource(target, bit(input), source.coord, 5,
        probe, &index, {}, nullptr, nullptr, 200), "preflight ignored real occupied resources");
    pnr::BackwardTakeoffStateView live, released_suffix;
    for (auto &tile : device.tile_grid) {
      live.emplace(&tile, tile.cb);
      if (&tile != &source) released_suffix.emplace(&tile, fpga::CBState{});
    }
    auto &anchor = *device.getTile(source.coord.x + delta.x, source.coord.y + delta.y);
    released_suffix.at(&anchor).dst.jump.setBit(landing);
    std::vector<pnr::BackwardRouteAnchor> anchors{{&anchor, landing, {}, 1}};
    // Simulate Moving's private suffix release without changing the live tree.
    // The driver stays leased; only its retained landing can prove the input.
    require(pnr::inputMayReachSource(target, bit(candidate_input), source.coord, 5,
        {}, &index, {}, &released_suffix, &anchors, 200),
        "preflight failed to reuse retained anchor with a released suffix view");
    require(!pnr::inputMayReachSource(target, bit(candidate_input), source.coord, 5,
        {}, &index, [] { return true; }, &released_suffix, &anchors, 200),
        "preflight ignored cancellation");
    if (relays) {
      // Keep one real LOCAL lease in the speculative view. A derived
      // reachability shortcut must not bypass this occupied intermediate node.
      released_suffix.at(&anchor).local.local.setBit(relay0);
      require(!pnr::inputMayReachSource(target, bit(candidate_input), source.coord, 5,
          {}, &index, {}, &released_suffix, &anchors, 200),
          "preflight skipped an occupied LOCAL relay");
    } else if (joint_sink) {
      released_suffix.at(&target).joint.jump.setBit(joint);
      require(!pnr::inputMayReachSource(target, bit(candidate_input), source.coord, 5,
          {}, &index, {}, &released_suffix, &anchors, 200),
          "preflight skipped an occupied terminal JOINT");
    }
    for (auto &tile : device.tile_grid) {
      const auto &before = live.at(&tile);
      require(tile.cb.src.jump == before.src.jump && tile.cb.dst.jump == before.dst.jump &&
                  tile.cb.joint.jump == before.joint.jump && tile.cb.local.local == before.local.local &&
                  tile.cb.src_deadend.jump == before.src_deadend.jump,
              "speculative input proof changed live occupancy");
      std::ostringstream details;
      const auto audit = fpga::auditTileCongestion(tile, details, {&net});
      // Every committed lease must have a real, consistently registered owner.
      require(!audit.orphan_leases && !audit.missing_leases &&
                  !audit.missing_registrations && !audit.stale_registrations &&
                  !audit.conflicting_driver_nodes, details.str());
    }
  } else {
    requireUnchanged();
  }
  std::cout << "seed=" << seed << " direction=" << direction << " relays=" << relays
            << " joint_sink=" << joint_sink << " disconnected=" << disconnected
            << " preflight=" << proof << " reachable=" << reachable
            << " committed=" << routed << '\n';
  // This assertion intentionally fails before the backward LOCAL traversal fix.
  // Both positive and disconnected cases prevent an always-true workaround.
  require(proof == !disconnected,
          proof ? "preflight accepted a disconnected corridor"
                : "preflight rejected a corridor the production router completed");
}

} // namespace

int runInputPreflightRegression(int relays) {
  if (relays < 0 || relays > 3) return 2;
  int failures = 0, cases = 0;
  for (unsigned seed : {17u, 93u, 511u, 4099u})
    for (int direction = 0; direction < 4; ++direction)
      for (bool joint_sink : {false, true})
        for (bool disconnected : {false, true}) {
          ++cases;
          try {
            checkCorridor(seed, direction, relays, joint_sink, disconnected);
          } catch (const std::exception &error) {
            ++failures;
            std::cerr << "FAIL seed=" << seed << " direction=" << direction
                      << " relays=" << relays << " joint_sink=" << joint_sink
                      << " disconnected=" << disconnected << ": " << error.what() << '\n';
          }
        }
  // Destroy singleton-held route references while no subsequent test uses them.
  fpga::Device::current().tile_grid.clear();
  std::cout << "input_preflight cases=" << cases << " failures=" << failures << '\n';
  return failures ? 1 : 0;
}
