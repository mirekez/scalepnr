#include "Device.h"
#include "Docking.h"
#include "RoutePassState.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct TestFailure {
  std::string message;
};

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw TestFailure{message};
  }
}

NodeMask bit(int index) { return NodeMask{0, 1} << index; }

void docking_duplicate_checks_only_recent_entries() {
  // Suppression is bounded by stored entries, not by routing hops. More than
  // sixteen older entries must neither increase scan work nor hide a candidate.
  for (size_t size : {0u, 1u, 15u, 16u, 17u, 1024u}) {
    std::vector<pnr::DockingFrontierNode> frontier;
    for (size_t i = 0; i < size; ++i)
      frontier.push_back({{static_cast<int>(i), 0}, 7, 4});
    auto recent = pnr::recentDockingEntries(frontier);
    const size_t count = std::min(size, size_t{16});
    require(recent.size() == count, "Docking duplicate lookback exceeds sixteen");
    if (size == 0) continue;
    require(&recent.front() == &frontier[size - count] &&
                &recent.back() == &frontier.back(),
            "Docking duplicate lookback does not end at the latest entry");
    size_t comparisons = 0;
    auto find = [&](int x) {
      comparisons = 0;
      return std::find_if(recent.begin(), recent.end(), [&](const auto &entry) {
        ++comparisons;
        return entry.coord.x == x && entry.coord.y == 0 && entry.dst == 7;
      });
    };
    require(find(static_cast<int>(size - count)) == recent.begin(),
            "Docking lost the sixteenth-most-recent duplicate");
    auto last = find(static_cast<int>(size - 1));
    require(last != recent.end(), "Docking missed its latest duplicate");
    last->depth = 2;
    require(frontier.back().depth == 2,
            "Docking shorter-depth update did not reach the stored frontier");
    if (size > 16) {
      require(find(static_cast<int>(size - 17)) == recent.end() && comparisons == 16,
              "Docking still scans old frontier history");
    }
  }
  std::vector<pnr::DockingBridgeBlocker> bridges(1024);
  for (size_t i = 0; i < bridges.size(); ++i) bridges[i].src = static_cast<int>(i);
  auto recent = pnr::recentDockingEntries(bridges);
  require(recent.size() == 16 && recent.front().src == 1008 && recent.back().src == 1023,
          "Docking bridge suppression did not use the same bounded window");
}

void terminal_entry_cache_refreshes_after_topology_growth() {
  constexpr int dst = 17;
  constexpr int local = 23;
  fpga::CBType cb;
  cb.derived_masks_valid = true;

  // Cache the initially absent endpoint before the remaining topology arrives.
  require(cb.terminalEntries(local).empty(),
          "empty terminal-entry cache setup failed");

  // Grow the numeric topology without rebuilding unrelated derived tables.
  cb.dst_local[dst].local.setBit(local);
  cb.dsts_reaching_local[local].jump.setBit(dst);

  // Check: a formerly empty cache must expose the newly loaded direct path.
  const auto &entries = cb.terminalEntries(local);
  require(entries.size() == 1 && entries.front().dst == dst &&
              entries.front().joint < 0,
          "terminal-entry cache hid topology loaded after its first lookup");
}

int encodedJump(int dx, int dy, int num = 0) {
  auto encode = [](int value) { return value & 0xf; };
  return (encode(dx) << 8) | (encode(dy) << 4) | (num & 0xf);
}

void rememberJumpTarget(fpga::CBType &cb, int src, int dst, fpga::Coord delta) {
  fpga::CBJumpState dsts{};
  dsts.jump = bit(dst);
  fpga::CBType::ResolvedJump entry{};
  entry.delta = delta;
  entry.target_cb_type_id = cb.type_id;
  entry.dsts = dsts;
  cb.dst_by_src[src].push_back(entry);
}

void rememberJumpTarget(fpga::CBType &cb, int src, int dst, fpga::Coord delta,
                        int target_cb_type_id) {
  fpga::CBJumpState dsts{};
  dsts.jump = bit(dst);
  fpga::CBType::ResolvedJump entry{};
  entry.delta = delta;
  entry.target_cb_type_id = target_cb_type_id;
  entry.dsts = dsts;
  entry.target_tile_coord = true;
  cb.dst_by_src[src].push_back(entry);
}

void rememberConn(fpga::CBType &cb, fpga::CBNodeNameType from_type, int from,
                  fpga::CBNodeNameType to_type, int to) {
  const std::string *from_name = cb.nodeName(from_type, from);
  const std::string *to_name = cb.nodeName(to_type, to);
  cb.rememberConnName(from_type, from, to_type, to,
                      from_name ? *from_name : std::to_string(from),
                      to_name ? *to_name : std::to_string(to));
}

fpga::CBType makeDockingCrossbar() {
  fpga::CBType cb{};
  cb.name = "DOCK";
  cb.type_id = 0;
  constexpr int dst = 0;
  constexpr int pin = 20;
  std::vector<std::pair<int, fpga::Coord>> srcs{
      {encodedJump(0, -1), fpga::Coord{0, -1}},
      {encodedJump(1, 0), fpga::Coord{1, 0}},
      {encodedJump(0, 1), fpga::Coord{0, 1}},
      {encodedJump(-1, 0), fpga::Coord{-1, 0}},
  };

  cb.rememberNodeName(fpga::CB_NODE_DST, dst, "D0");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN0");
  for (const auto &[src, delta] : srcs) {
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "S" + std::to_string(src));
    cb.dst_src[dst].jump |= bit(src);
    rememberJumpTarget(cb, src, dst, delta);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
  }
  cb.dst_local[dst].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeLinearDockingCrossbar() {
  fpga::CBType cb{};
  cb.name = "LINEAR_DOCK";
  cb.type_id = 0;
  constexpr int dst = 0;
  constexpr int pin = 20;
  int east = encodedJump(1, 0);

  cb.rememberNodeName(fpga::CB_NODE_DST, dst, "ENTRY");
  cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN");
  cb.dst_src[dst].jump |= bit(east);
  rememberJumpTarget(cb, east, dst, fpga::Coord{1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, east);
  cb.dst_local[dst].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeRevisitingBackwardCrossbar() {
  fpga::CBType cb{};
  cb.name = "REVISIT";
  cb.type_id = 0;
  constexpr int pin = 20;
  const int east0 = encodedJump(1, 0, 0);
  const int west0 = encodedJump(-1, 0, 0);
  const int east1 = encodedJump(1, 0, 1);
  const int west1 = encodedJump(-1, 0, 1);
  struct Link {
    int from_dst;
    int src;
    int target_dst;
    fpga::Coord delta;
  };
  const std::array<Link, 4> links{{
      {1, east0, 0, {1, 0}},
      {2, west0, 1, {-1, 0}},
      {3, east1, 2, {1, 0}},
      {4, west1, 3, {-1, 0}},
  }};
  for (int dst = 0; dst <= 4; ++dst) {
    cb.rememberNodeName(fpga::CB_NODE_DST, dst,
                        "DST_" + std::to_string(dst));
  }
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN");
  for (const Link &link : links) {
    cb.rememberNodeName(fpga::CB_NODE_SRC, link.src,
                        "SRC_" + std::to_string(link.src));
    cb.dst_src[link.from_dst].jump |= bit(link.src);
    rememberJumpTarget(cb, link.src, link.target_dst, link.delta);
    rememberConn(cb, fpga::CB_NODE_DST, link.from_dst, fpga::CB_NODE_SRC,
                 link.src);
  }
  cb.dst_local[0].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, 0, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeTargetStepOutCrossbar() {
  fpga::CBType cb{};
  cb.name = "DOCK_STEP_OUT";
  cb.type_id = 0;
  constexpr int enter_dst = 0;
  constexpr int blocked_dst = 1;
  constexpr int pin = 20;
  int east = encodedJump(1, 0);
  int west = encodedJump(-1, 0);

  cb.rememberNodeName(fpga::CB_NODE_DST, enter_dst, "ENTER_DST");
  cb.rememberNodeName(fpga::CB_NODE_DST, blocked_dst, "BLOCKED_DST");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN0");
  cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST");
  cb.rememberNodeName(fpga::CB_NODE_SRC, west, "WEST");

  cb.dst_src[blocked_dst].jump |= bit(east);
  rememberJumpTarget(cb, east, blocked_dst, fpga::Coord{1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, blocked_dst, fpga::CB_NODE_SRC, east);

  cb.dst_src[blocked_dst].jump |= bit(west);
  rememberJumpTarget(cb, west, enter_dst, fpga::Coord{-1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, blocked_dst, fpga::CB_NODE_SRC, west);

  cb.dst_local[enter_dst].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, enter_dst, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeTwoJointTerminalCrossbar() {
  fpga::CBType cb{};
  cb.name = "TWO_JOINT_TERMINAL";
  cb.type_id = 0;
  constexpr int dst = 0;
  constexpr int first_joint = 11;
  constexpr int second_joint = 12;
  constexpr int pin = 20;

  cb.rememberNodeName(fpga::CB_NODE_DST, dst, "ENTRY");
  cb.rememberNodeName(fpga::CB_NODE_JOINT, first_joint, "FIRST_JOINT");
  cb.rememberNodeName(fpga::CB_NODE_JOINT, second_joint, "SECOND_JOINT");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN");
  cb.dst_joint[dst].joint |= bit(first_joint);
  cb.joint_joint[first_joint].joint |= bit(second_joint);
  cb.joint_local[second_joint].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_JOINT, first_joint);
  rememberConn(cb, fpga::CB_NODE_JOINT, first_joint, fpga::CB_NODE_JOINT,
               second_joint);
  rememberConn(cb, fpga::CB_NODE_JOINT, second_joint, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeForwardNamespaceCrossbar() {
  fpga::CBType cb{};
  cb.name = "FORWARD_NS";
  cb.type_id = 0;
  constexpr int anchor_dst = 0;
  constexpr int target_dst = 7;
  int east = encodedJump(1, 0);

  cb.rememberNodeName(fpga::CB_NODE_DST, anchor_dst, "ANCHOR_DST");
  cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST_SRC");
  cb.dst_src[anchor_dst].jump |= bit(east);
  fpga::CBJumpState target_dsts{};
  target_dsts.jump = bit(target_dst);
  cb.dst_by_src[east].push_back(
      fpga::CBType::ResolvedJump{fpga::Coord{1, 0}, 1, target_dsts, {}, false});
  rememberConn(cb, fpga::CB_NODE_DST, anchor_dst, fpga::CB_NODE_SRC, east);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeTargetNamespaceCrossbar() {
  fpga::CBType cb{};
  cb.name = "TARGET_NS";
  cb.type_id = 1;
  constexpr int target_dst = 7;
  constexpr int pin = 20;

  cb.rememberNodeName(fpga::CB_NODE_DST, target_dst, "TARGET_DST");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "TARGET_PIN");
  cb.dst_local[target_dst].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, target_dst, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

fpga::CBType makeRandomBackwardCrossbar(int type_id, int dst,
                                        int first_target_dst,
                                        int second_target_dst) {
  fpga::CBType cb{};
  cb.name = "RANDOM_BACKWARD_" + std::to_string(type_id);
  cb.type_id = type_id;
  constexpr int pin = 20;
  std::vector<std::pair<int, fpga::Coord>> srcs{
      {encodedJump(0, -1), fpga::Coord{0, -1}},
      {encodedJump(1, 0), fpga::Coord{1, 0}},
      {encodedJump(0, 1), fpga::Coord{0, 1}},
      {encodedJump(-1, 0), fpga::Coord{-1, 0}},
  };

  cb.rememberNodeName(fpga::CB_NODE_DST, dst, "D" + std::to_string(type_id));
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "P" + std::to_string(type_id));
  for (const auto &[src, delta] : srcs) {
    cb.rememberNodeName(fpga::CB_NODE_SRC, src,
                        "S" + std::to_string(type_id) + "_" +
                            std::to_string(src));
    cb.dst_src[dst].jump |= bit(src);
    rememberJumpTarget(cb, src, first_target_dst, delta, 0);
    rememberJumpTarget(cb, src, second_target_dst, delta, 1);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
  }
  cb.dst_local[dst].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  return cb;
}

std::vector<fpga::Tile *> resetTwoTypeGrid(int width, int height,
                                           fpga::CBType &forward_cb,
                                           fpga::CBType &target_cb,
                                           fpga::Coord target_coord) {
  fpga::Device &device = fpga::Device::current();
  device.tile_grid.clear();
  device.cb_types.clear();
  forward_cb.type_id = 0;
  target_cb.type_id = 1;
  device.cb_types.push_back(forward_cb);
  device.cb_types.push_back(target_cb);
  device.grid_spec.size = {width, height};
  device.size_width = width;
  device.size_height = height;
  device.tile_grid.resize(static_cast<size_t>(width * height));

  std::vector<fpga::Tile *> result;
  result.reserve(device.tile_grid.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      fpga::Tile &tile = device.tile_grid[static_cast<size_t>(y * width + x)];
      tile.coord = {x, y};
      tile.cb_coord = tile.coord;
      tile.name = {x, y};
      tile.cb = {};
      tile.pin_state = {};
      tile.cb_type = (x == target_coord.x && y == target_coord.y)
                         ? &device.cb_types[1]
                         : &device.cb_types[0];
      tile.routedNets.clear();
      result.push_back(&tile);
    }
  }
  return result;
}

std::vector<fpga::Tile *> resetGrid(int width, int height, fpga::CBType &cb) {
  fpga::Device &device = fpga::Device::current();
  device.tile_grid.clear();
  device.cb_types.clear();
  cb.type_id = 0;
  device.cb_types.push_back(cb);
  device.grid_spec.size = {width, height};
  device.size_width = width;
  device.size_height = height;
  device.tile_grid.resize(static_cast<size_t>(width * height));

  std::vector<fpga::Tile *> result;
  result.reserve(device.tile_grid.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      fpga::Tile &tile = device.tile_grid[static_cast<size_t>(y * width + x)];
      tile.coord = {x, y};
      tile.cb_coord = tile.coord;
      tile.name = {x, y};
      tile.cb = {};
      tile.pin_state = {};
      tile.cb_type = &cb;
      tile.routedNets.clear();
      result.push_back(&tile);
    }
  }
  return result;
}

std::vector<fpga::Tile *> resetRandomTwoTypeGrid(int width, int height,
                                                 fpga::CBType &first_cb,
                                                 fpga::CBType &second_cb,
                                                 std::mt19937 &rng) {
  fpga::Device &device = fpga::Device::current();
  device.tile_grid.clear();
  device.cb_types.clear();
  first_cb.type_id = 0;
  second_cb.type_id = 1;
  device.cb_types.push_back(first_cb);
  device.cb_types.push_back(second_cb);
  device.grid_spec.size = {width, height};
  device.size_width = width;
  device.size_height = height;
  device.tile_grid.resize(static_cast<size_t>(width * height));

  std::vector<fpga::Tile *> result;
  result.reserve(device.tile_grid.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      fpga::Tile &tile = device.tile_grid[static_cast<size_t>(y * width + x)];
      tile.coord = {x, y};
      tile.cb_coord = tile.coord;
      tile.name = {x, y};
      tile.cb = {};
      tile.pin_state = {};
      tile.cb_type = &device.cb_types[rng() & 1u];
      tile.routedNets.clear();
      result.push_back(&tile);
    }
  }
  return result;
}

std::vector<fpga::Coord> makeCorridor(fpga::Coord source, fpga::Coord target,
                                      bool horizontal_first) {
  std::vector<fpga::Coord> path;
  fpga::Coord curr = source;
  path.push_back(curr);
  auto step_x = [&]() {
    while (curr.x != target.x) {
      curr.x += curr.x < target.x ? 1 : -1;
      path.push_back(curr);
    }
  };
  auto step_y = [&]() {
    while (curr.y != target.y) {
      curr.y += curr.y < target.y ? 1 : -1;
      path.push_back(curr);
    }
  };
  if (horizontal_first) {
    step_x();
    step_y();
  } else {
    step_y();
    step_x();
  }
  return path;
}

void occupyBlockedTile(fpga::Tile &tile) {
  tile.cb.dst.jump |= bit(0);
  tile.cb.src.jump |= bit(encodedJump(0, -1)) | bit(encodedJump(1, 0)) |
                      bit(encodedJump(0, 1)) | bit(encodedJump(-1, 0));
  tile.cb.local.local |= bit(20);
}

void docking_finds_one_random_free_path(unsigned seed) {
  std::mt19937 rng(seed);
  fpga::CBType cb = makeDockingCrossbar();
  std::vector<fpga::Tile *> tiles = resetGrid(13, 13, cb);

  std::uniform_int_distribution<int> coord_dist(3, 9);
  fpga::Coord source{coord_dist(rng), coord_dist(rng)};
  fpga::Coord target;
  do {
    target = {coord_dist(rng), coord_dist(rng)};
  } while ((std::abs(target.x - source.x) + std::abs(target.y - source.y)) ==
               0 ||
           (std::abs(target.x - source.x) + std::abs(target.y - source.y)) > 5);

  std::vector<fpga::Coord> corridor =
      makeCorridor(source, target, (seed & 1u) != 0);
  std::set<std::pair<int, int>> free_tiles;
  for (fpga::Coord coord : corridor) {
    free_tiles.insert({coord.x, coord.y});
  }

  for (fpga::Tile *tile : tiles) {
    if (std::abs(tile->coord.x - target.x) <= 5 &&
        std::abs(tile->coord.y - target.y) <= 5 &&
        !free_tiles.contains({tile->coord.x, tile->coord.y})) {
      occupyBlockedTile(*tile);
    }
  }

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile, "source or target tile missing");
  // Reverse topology is independent of leases and may be reused by every
  // suffix targeting the same tile during a Moving placement attempt.
  pnr::BackwardResolveIndex backward_index =
      pnr::buildBackwardResolveIndex(fpga::Device::current(), target, 5);
  pnr::DockingResult result =
      pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5,
                         false, {}, &backward_index);
  require(
      result.success,
      "dockGrounding did not find the deliberately freed corridor for seed " +
          std::to_string(seed) + " source=(" + std::to_string(source.x) + "," +
          std::to_string(source.y) + ")" + " target=(" +
          std::to_string(target.x) + "," + std::to_string(target.y) + ")" +
          " target_seeds=" + std::to_string(result.target_seed_count) +
          " fpop=" + std::to_string(result.forward_pop_count) +
          " fpush=" + std::to_string(result.forward_push_count) +
          " bpop=" + std::to_string(result.backward_pop_count) +
          " bpush=" + std::to_string(result.backward_push_count));
  require(result.fragments.size() >= 2,
          "dockGrounding returned an incomplete route suffix");
  require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
          "dockGrounding did not finish at a tile pin");
  require(result.fragments.back().local == 20,
          "dockGrounding finished at the wrong local input");

  for (const fpga::Wire &fragment : result.fragments) {
    if (fragment.type != fpga::Wire::WIRE_CROSSBAR || fragment.jump < 0) {
      continue;
    }
    require(free_tiles.contains({fragment.from.x, fragment.from.y}),
            "dockGrounding used a blocked source-side tile");
    require(free_tiles.contains({fragment.to.x, fragment.to.y}),
            "dockGrounding used a blocked destination-side tile");
  }
}

void docking_extends_from_existing_anchor_dst() {
  fpga::CBType cb = makeDockingCrossbar();
  std::vector<fpga::Tile *> tiles = resetGrid(13, 13, cb);
  fpga::Coord source{4, 4};
  fpga::Coord target{8, 4};
  std::vector<fpga::Coord> corridor = makeCorridor(source, target, true);
  std::set<std::pair<int, int>> free_tiles;
  for (fpga::Coord coord : corridor) {
    free_tiles.insert({coord.x, coord.y});
  }

  for (fpga::Tile *tile : tiles) {
    if (std::abs(tile->coord.x - target.x) <= 5 &&
        std::abs(tile->coord.y - target.y) <= 5 &&
        !free_tiles.contains({tile->coord.x, tile->coord.y})) {
      occupyBlockedTile(*tile);
    }
  }

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile, "source or target tile missing");

  // A continued partial route already owns its anchor dst; docking must only
  // allocate the new outgoing src from that anchor instead of rejecting it.
  source_tile->cb.dst.jump |= bit(0);
  pnr::DockingResult result =
      pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
  require(result.success,
          "dockGrounding rejected an already-leased anchor dst");
  require(!result.fragments.empty(),
          "dockGrounding returned no continuation fragments");
  require(result.fragments.front().from.x == source.x &&
              result.fragments.front().from.y == source.y,
          "dockGrounding did not start from the leased anchor tile");
}

void docking_uses_mask_angle_priority_before_numeric_node_order() {
  fpga::CBType cb{};
  cb.name = "DIRECTION_PRIORITY";
  cb.type_id = 0;
  constexpr int dst = 0;
  constexpr int pin = 20;
  int north = encodedJump(0, -1);
  int west = encodedJump(-1, 0);
  require(north < west, "priority regression requires the wrong direction to "
                        "have the lower node number");

  cb.rememberNodeName(fpga::CB_NODE_DST, dst, "ENTRY");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN");
  cb.rememberNodeName(fpga::CB_NODE_SRC, north, "NORTH");
  cb.rememberNodeName(fpga::CB_NODE_SRC, west, "WEST");
  cb.dst_src[dst].jump |= bit(north) | bit(west);
  cb.dst_local[dst].local |= bit(pin);
  rememberJumpTarget(cb, north, dst, {0, -1});
  rememberJumpTarget(cb, west, dst, {-1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, north);
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, west);
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();

  resetGrid(9, 9, cb);
  fpga::Tile *source = fpga::Device::current().getTile(5, 5);
  fpga::Tile *target = fpga::Device::current().getTile(4, 5);
  require(source && target, "priority docking tiles are missing");

  pnr::DockingResult result =
      pnr::dockGrounding(*source, dst, "ENTRY", *target, bit(pin), 5, 5);
  require(result.success, "priority docking did not reach the adjacent target");
  require(!result.fragments.empty() && result.fragments.front().jump == west,
          "docking ignored mask angle priority and selected the lower numeric "
          "north source");
  require(
      result.forward_push_count <= 1,
      "docking expanded a wrong-direction source before the direct target hop");
}

void docking_backward_meets_an_existing_anchor_dst() {
  fpga::CBType cb{};
  cb.name = "LEASED_ANCHOR";
  cb.type_id = 0;
  constexpr int dst = 0;
  constexpr int pin = 20;
  int east = encodedJump(1, 0);
  cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST_SRC");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "TARGET_PIN");
  cb.dst_src[dst].jump |= bit(east);
  rememberJumpTarget(cb, east, dst, fpga::Coord{1, 0});
  for (int entry = 0; entry < 8; ++entry) {
    cb.rememberNodeName(fpga::CB_NODE_DST, entry,
                        "ENTRY_" + std::to_string(entry));
    cb.dst_local[entry].local |= bit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, entry, fpga::CB_NODE_LOCAL, pin);
  }
  rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, east);
  cb.rebuildOutgoingSrcs();
  fpga::Coord source{5, 6};
  fpga::Coord target{6, 6};
  resetGrid(13, 13, cb);

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile,
          "leased-anchor docking source or target tile missing");

  // Reproduce a continued route: its final destination node is already
  // leased by the same route before backward grounding starts.
  source_tile->cb.dst.jump |= bit(dst);
  pnr::DockingResult result = pnr::dockGrounding(
      *source_tile, dst, "ANCHOR_DST", *target_tile, bit(pin), 5, 5, true);

  require(
      result.success,
      "backward docking could not reach the already-leased route anchor"
      " seeds=" +
          std::to_string(result.target_seed_count) +
          " bpop=" + std::to_string(result.backward_pop_count) +
          " bpush=" + std::to_string(result.backward_push_count) +
          " busy=" + std::to_string(result.backward_busy_reject_count) +
          " missing=" + std::to_string(result.backward_missing_prev_dst_count) +
          " reaches=" + std::to_string(result.backward_mapping_reaches_count));
  require(result.target_seed_count == 8,
          "leased-anchor regression did not create all eight free destination "
          "entries");
  require(result.backward_push_count == 0,
          "direct backward docking queued an alternative after meeting the "
          "route anchor");
  require(result.forward_push_count == 0,
          "docking bypassed the leased-anchor regression through forward "
          "expansion");
  require(result.fragments.size() == 3,
          "leased-anchor backward docking returned an unexpected route length");

  // Focused tracing must retain every free target seed and the concrete
  // backward path that met the existing forward anchor.
  std::array<bool, 8> traced_seeds{};
  bool traced_meeting_path = false;
  for (const pnr::DockingBackwardAttempt &attempt : result.backward_attempts) {
    if (attempt.result == "seed" && attempt.target_dst >= 0 &&
        attempt.target_dst < static_cast<int>(traced_seeds.size())) {
      traced_seeds[attempt.target_dst] = true;
    }
    if (attempt.result == "meet" && attempt.target_dst == dst &&
        attempt.fragments.size() == 3) {
      traced_meeting_path = true;
    }
  }
  require(std::all_of(traced_seeds.begin(), traced_seeds.end(),
                      [](bool traced) { return traced; }),
          "backward docking trace omitted a free target destination seed");
  require(
      traced_meeting_path,
      "backward docking trace omitted the path meeting the existing anchor");
}

void docking_late_terminal_entry_meets_anchor_after_width_limit() {
  fpga::CBType forward_cb{};
  forward_cb.name = "WIDTH_FORWARD";
  forward_cb.type_id = 0;
  fpga::CBType target_cb{};
  target_cb.name = "WIDTH_TARGET";
  target_cb.type_id = 1;
  constexpr int anchor_dst = 0;
  constexpr int pin = 200;
  constexpr int decoy_entry = 0;
  constexpr int useful_entry = 1;
  constexpr int decoy_count = 32;
  constexpr int middle_dst = 80;
  constexpr int middle_src = 300;
  constexpr int anchor_src = 301;

  forward_cb.rememberNodeName(fpga::CB_NODE_DST, anchor_dst, "ANCHOR");
  forward_cb.rememberNodeName(fpga::CB_NODE_DST, middle_dst, "MIDDLE");
  forward_cb.rememberNodeName(fpga::CB_NODE_SRC, anchor_src, "ANCHOR_SRC");
  forward_cb.rememberNodeName(fpga::CB_NODE_SRC, middle_src, "MIDDLE_SRC");
  forward_cb.dst_src[anchor_dst].jump |= bit(anchor_src);
  forward_cb.dst_src[middle_dst].jump |= bit(middle_src);
  rememberConn(forward_cb, fpga::CB_NODE_DST, anchor_dst, fpga::CB_NODE_SRC,
               anchor_src);
  rememberConn(forward_cb, fpga::CB_NODE_DST, middle_dst, fpga::CB_NODE_SRC,
               middle_src);
  target_cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN");
  for (int entry : {decoy_entry, useful_entry}) {
    target_cb.rememberNodeName(fpga::CB_NODE_DST, entry,
                               "ENTRY_" + std::to_string(entry));
    target_cb.dst_local[entry].local |= bit(pin);
    rememberConn(target_cb, fpga::CB_NODE_DST, entry, fpga::CB_NODE_LOCAL, pin);
  }
  for (int index = 0; index < decoy_count; ++index) {
    int decoy_dst = 100 + index;
    int decoy_src = 400 + index;
    forward_cb.rememberNodeName(fpga::CB_NODE_DST, decoy_dst,
                                "DECOY_DST_" + std::to_string(index));
    forward_cb.rememberNodeName(fpga::CB_NODE_SRC, decoy_src,
                                "DECOY_SRC_" + std::to_string(index));
    forward_cb.dst_src[decoy_dst].jump |= bit(decoy_src);
    rememberConn(forward_cb, fpga::CB_NODE_DST, decoy_dst, fpga::CB_NODE_SRC,
                 decoy_src);
  }
  forward_cb.rebuildOutgoingSrcs();
  target_cb.rebuildOutgoingSrcs();

  fpga::Coord source{4, 6};
  fpga::Coord middle{5, 6};
  fpga::Coord target{6, 6};
  resetTwoTypeGrid(13, 13, forward_cb, target_cb, target);
  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *middle_tile = fpga::Device::current().getTile(middle.x, middle.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && middle_tile && target_tile,
          "width-starvation docking tiles are missing");

  pnr::BackwardResolveIndex backward_index;
  backward_index.center = target;
  backward_index.radius = 5;
  std::vector<fpga::Tile *> decoy_tiles;
  for (fpga::Tile &tile : fpga::Device::current().tile_grid) {
    if (&tile != source_tile && &tile != middle_tile && &tile != target_tile) {
      decoy_tiles.push_back(&tile);
    }
  }
  require(decoy_tiles.size() >= decoy_count,
          "width-starvation grid has too few decoy tiles");
  for (int index = 0; index < decoy_count; ++index) {
    backward_index.sources[{target.x, target.y, decoy_entry}].push_back(
        {decoy_tiles[index], 400 + index, 400 + index});
  }
  backward_index.sources[{target.x, target.y, useful_entry}].push_back(
      {middle_tile, middle_src, middle_src});
  backward_index.sources[{middle.x, middle.y, middle_dst}].push_back(
      {source_tile, anchor_src, anchor_src});

  // The first terminal seed exhausts all 32 shared depth-one slots with dead
  // branches. The second seed is the only valid path and needs two backward
  // hops, so every terminal seed must retain a fair part of the bounded beam.
  source_tile->cb.dst.jump |= bit(anchor_dst);
  pnr::DockingResult result =
      pnr::dockGrounding(*source_tile, anchor_dst, "ANCHOR", *target_tile,
                         bit(pin), 5, 5, false, {}, &backward_index);
  require(result.success,
          "a late terminal entry was starved by earlier backward candidates"
          " fpush=" +
              std::to_string(result.forward_push_count) +
              " bpush=" + std::to_string(result.backward_push_count));
  require(result.target_seed_count == 2,
          "width-starvation regression did not create both terminal seeds");
  require(result.backward_push_count > 1,
          "late-entry regression did not require multi-hop backward expansion");
  require(
      result.fragments.size() == 4,
      "late-entry docking did not return two jumps plus terminal fragments");
  require(result.fragments.front().jump == anchor_src &&
              result.fragments[1].jump == middle_src &&
              result.fragments[1].dst == useful_entry,
          "late-entry docking did not use the useful seed's two-hop path");
}

void docking_reuses_beam_slots_from_dead_terminal_seeds() {
  fpga::CBType route_cb{};
  route_cb.name = "BEAM_ROUTE";
  route_cb.type_id = 0;
  fpga::CBType target_cb{};
  target_cb.name = "BEAM_TARGET";
  target_cb.type_id = 1;
  constexpr int anchor_dst = 0;
  constexpr int anchor_src = 301;
  constexpr int landing_src = 300;
  constexpr int first_predecessor = 100;
  constexpr int useful_predecessor = 106;
  constexpr int terminal_count = 13;
  constexpr int pin = 200;

  route_cb.rememberNodeName(fpga::CB_NODE_DST, anchor_dst, "ANCHOR");
  route_cb.rememberNodeName(fpga::CB_NODE_SRC, anchor_src, "ANCHOR_SRC");
  route_cb.rememberNodeName(fpga::CB_NODE_SRC, landing_src, "LANDING_SRC");
  route_cb.dst_src[anchor_dst].jump |= bit(anchor_src);
  rememberConn(route_cb, fpga::CB_NODE_DST, anchor_dst, fpga::CB_NODE_SRC,
               anchor_src);
  for (int index = 0; index < 7; ++index) {
    int predecessor = first_predecessor + index;
    route_cb.rememberNodeName(fpga::CB_NODE_DST, predecessor,
                              "PREDECESSOR_" + std::to_string(index));
    route_cb.dst_src[predecessor].jump |= bit(landing_src);
    rememberConn(route_cb, fpga::CB_NODE_DST, predecessor, fpga::CB_NODE_SRC,
                 landing_src);
  }
  route_cb.rebuildOutgoingSrcs();

  target_cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN");
  for (int entry = 0; entry < terminal_count; ++entry) {
    target_cb.rememberNodeName(fpga::CB_NODE_DST, entry,
                               "TERMINAL_" + std::to_string(entry));
    target_cb.dst_local[entry].local |= bit(pin);
    rememberConn(target_cb, fpga::CB_NODE_DST, entry, fpga::CB_NODE_LOCAL, pin);
  }
  target_cb.rebuildOutgoingSrcs();

  fpga::Coord source{4, 6};
  fpga::Coord middle{5, 6};
  fpga::Coord target{6, 6};
  resetTwoTypeGrid(13, 13, route_cb, target_cb, target);
  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *middle_tile = fpga::Device::current().getTile(middle.x, middle.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && middle_tile && target_tile,
          "dead-seed beam regression tiles are missing");

  pnr::BackwardResolveIndex backward_index;
  backward_index.center = target;
  backward_index.radius = 5;
  backward_index.sources[{target.x, target.y, 0}].push_back(
      {middle_tile, landing_src, landing_src});
  backward_index.sources[{middle.x, middle.y, useful_predecessor}].push_back(
      {source_tile, anchor_src, anchor_src});

  // Thirteen free terminal seeds previously divided the 32-node beam into
  // three slots each. Only seed zero has incoming arcs, and its valid seventh
  // predecessor was discarded after three dead predecessors consumed its
  // rigid quota. Dead seeds must release their reserved beam capacity.
  source_tile->cb.dst.jump |= bit(anchor_dst);
  pnr::DockingResult result = pnr::dockGrounding(
      *source_tile, anchor_dst, "ANCHOR", *target_tile, bit(pin), 5, 5, false,
      {}, &backward_index);
  require(result.success,
          "dead terminal seeds stranded usable backward beam capacity");
  require(result.target_seed_count == terminal_count,
          "dead-seed regression did not create every terminal seed");
  require(result.fragments.size() == 4,
          "dead-seed beam recovery returned an unexpected route length");
  require(result.fragments.front().jump == anchor_src &&
              result.fragments[1].local == useful_predecessor &&
              result.fragments[1].jump == landing_src,
          "dead-seed beam recovery did not use the seventh predecessor");
}

void docking_ignores_src_deadends() {
  fpga::CBType cb = makeDockingCrossbar();
  resetGrid(13, 13, cb);
  fpga::Coord source{4, 4};
  fpga::Coord target{8, 4};

  for (fpga::Tile &tile : fpga::Device::current().tile_grid) {
    if (std::abs(tile.coord.x - target.x) <= 5 &&
        std::abs(tile.coord.y - target.y) <= 5) {
      tile.cb.src_deadend.jump = tile.cb_type->dst_src[0].jump;
    }
  }

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile,
          "deadend docking source or target tile missing");

  // Docking is a bounded final-entry search; sticky deadends from earlier
  // forward attempts must not block an otherwise free local docking path.
  pnr::DockingResult result =
      pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
  require(result.success,
          "dockGrounding incorrectly treated src_deadend as real occupancy");
  require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
          "deadend-ignoring docking did not finish at a tile pin");
}

void docking_steps_out_from_non_enterable_target_dst() {
  fpga::CBType cb = makeTargetStepOutCrossbar();
  resetGrid(13, 13, cb);
  fpga::Coord target{6, 6};
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(target_tile, "target tile missing");

  // The forward route has already reached the destination tile, but on a dst
  // rail that cannot reach the required local pin. Docking must step out.
  target_tile->cb.dst.jump |= bit(1);
  pnr::DockingResult result = pnr::dockGrounding(*target_tile, 1, "BLOCKED_DST",
                                                 *target_tile, bit(20), 5, 5);
  require(result.success,
          "dockGrounding could not step out from a non-enterable target dst");
  require(result.fragments.size() >= 4,
          "dockGrounding returned too few fragments for step-out docking");
  require(result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR,
          "step-out docking did not start with a crossbar step");
  require(result.fragments.front().from.x == target.x &&
              result.fragments.front().from.y == target.y,
          "step-out docking did not start on the target tile");
  require(result.fragments.front().to.x != target.x ||
              result.fragments.front().to.y != target.y,
          "step-out docking did not leave the target tile before entering");
  require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
          "step-out docking did not finish at a tile pin");
  require(result.fragments.back().local == 20,
          "step-out docking finished at the wrong local input");
}

void docking_iob_uses_wider_endpoint_window() {
  fpga::CBType cb = makeDockingCrossbar();
  std::vector<fpga::Tile *> tiles = resetGrid(25, 25, cb);
  fpga::Coord source{6, 12};
  fpga::Coord target{18, 12};
  std::vector<fpga::Coord> corridor = makeCorridor(source, target, true);
  std::set<std::pair<int, int>> free_tiles;
  for (fpga::Coord coord : corridor) {
    free_tiles.insert({coord.x, coord.y});
  }

  for (fpga::Tile *tile : tiles) {
    if (std::abs(tile->coord.x - target.x) <= 12 &&
        std::abs(tile->coord.y - target.y) <= 12 &&
        !free_tiles.contains({tile->coord.x, tile->coord.y})) {
      occupyBlockedTile(*tile);
    }
  }

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile,
          "source or target tile missing for IOB docking");

  // Generic CLB grounding is intentionally radius 5 and must not cover this
  // edge-style endpoint distance.
  pnr::DockingResult clb_result =
      pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
  require(!clb_result.success,
          "generic docking unexpectedly crossed the IOB-sized gap");

  // IOB docking uses the same bitmask transitions, but with the wider edge
  // endpoint window needed for I/O route tiles.
  pnr::DockingResult iob_result =
      pnr::dockIOB(*source_tile, 0, "D0", *target_tile, bit(20));
  require(iob_result.success,
          "dockIOB did not find the deliberately freed I/O corridor");
  require(iob_result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
          "dockIOB did not finish at a tile pin");
  require(iob_result.fragments.back().local == 20,
          "dockIOB finished at the wrong local input");
  require(iob_result.forward_push_count < 160 &&
              iob_result.backward_push_count < 160,
          "dockIOB searched too many side branches for a one-tile-wide "
          "directed corridor");
}

void docking_backward_uses_resolved_target_dst_namespace() {
  fpga::CBType forward_cb = makeForwardNamespaceCrossbar();
  fpga::CBType target_cb = makeTargetNamespaceCrossbar();
  fpga::Coord source{5, 6};
  fpga::Coord target{6, 6};
  resetTwoTypeGrid(13, 13, forward_cb, target_cb, target);

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile,
          "namespace docking source or target tile missing");

  // Check: the previous tile has only local dst 0 for the outgoing source,
  // while the resolved destination tile uses dst 7 for the target pin.
  require((source_tile->cb_type->dsts_reaching_src[encodedJump(1, 0)].jump &
           bit(0)) != NodeMask{},
          "namespace docking setup lost previous-tile dst 0");
  require(
      (source_tile->cb_type->dsts_reaching_src[encodedJump(1, 0)].jump &
       bit(7)) == NodeMask{},
      "namespace docking setup accidentally made previous-tile dst 7 valid");

  pnr::DockingResult result = pnr::dockGrounding(*source_tile, 0, "ANCHOR_DST",
                                                 *target_tile, bit(20), 5, 5);
  require(result.success, "dockGrounding failed when previous dst and target "
                          "dst used different numeric namespaces");
  require(result.backward_push_count == 0,
          "namespace docking queued a node after target dst 7 met previous "
          "dst 0 directly");
  require(result.fragments.size() == 3,
          "namespace docking returned an unexpected route length");
  require(result.fragments.front().from.x == source.x &&
              result.fragments.front().to.x == target.x,
          "namespace docking did not use the direct resolved jump");
  require(result.fragments.front().local == 0 &&
              result.fragments.front().jump == encodedJump(1, 0),
          "namespace docking used the wrong previous-tile dst/src");
  require(result.fragments[1].local == 7 && result.fragments.back().local == 20,
          "namespace docking did not enter the resolved target dst/local");
}

void docking_reports_only_the_reachable_blocked_terminal() {
  fpga::CBType forward_cb = makeForwardNamespaceCrossbar();
  fpga::CBType target_cb = makeTargetNamespaceCrossbar();
  constexpr int reachable_dst = 7;
  constexpr int unreachable_dst = 9;
  constexpr int pin = 20;
  target_cb.rememberNodeName(fpga::CB_NODE_DST, unreachable_dst,
                             "UNREACHABLE_DST");
  target_cb.dst_local[unreachable_dst].local |= bit(pin);
  rememberConn(target_cb, fpga::CB_NODE_DST, unreachable_dst,
               fpga::CB_NODE_LOCAL, pin);
  target_cb.rebuildOutgoingSrcs();

  fpga::Coord source{5, 6};
  fpga::Coord target{6, 6};
  resetTwoTypeGrid(13, 13, forward_cb, target_cb, target);
  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile, "blocked-terminal test tiles missing");

  // Both terminal rails are occupied, but dst_by_src physically reaches only
  // dst 7. Docking must report dst 7 rather than an arbitrary busy entry.
  target_tile->cb.dst.jump |= bit(reachable_dst) | bit(unreachable_dst);
  pnr::DockingResult result = pnr::dockGrounding(*source_tile, 0, "ANCHOR_DST",
                                                 *target_tile, bit(pin), 5, 5);
  require(!result.success,
          "blocked-terminal docking unexpectedly leased an occupied entry");
  require(result.blocked_terminal_reachable,
          "docking did not report the occupied entry reached by its forward "
          "search");
  require(
      result.blocked_dst == reachable_dst,
      "docking reported an occupied entry that the forward route cannot reach");
  require(result.blocked_pin == pin, "docking reported the wrong terminal pin");
  require(result.blocked_terminal_fragments.size() >= 3 &&
              result.blocked_terminal_fragments.front().from == source &&
              result.blocked_terminal_fragments.back().to == target &&
              result.blocked_terminal_fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
              result.blocked_terminal_fragments.back().local == pin,
          "blocked-terminal report lost the complete path needed for safe replacement validation");
  require(target_tile->cb.dst.jump.testBit(reachable_dst) &&
              target_tile->cb.dst.jump.testBit(unreachable_dst) &&
              !target_tile->isPinNodeLeased(pin),
          "materializing a blocked terminal path changed live leases");
}

void docking_builds_random_multi_hop_backward_routes(unsigned seed) {
  std::mt19937 rng(seed);
  constexpr int first_dst = 3;
  constexpr int second_dst = 41;
  fpga::CBType first_cb =
      makeRandomBackwardCrossbar(0, first_dst, first_dst, second_dst);
  fpga::CBType second_cb =
      makeRandomBackwardCrossbar(1, second_dst, first_dst, second_dst);
  std::vector<fpga::Tile *> tiles =
      resetRandomTwoTypeGrid(13, 13, first_cb, second_cb, rng);

  std::uniform_int_distribution<int> coord_dist(3, 9);
  fpga::Coord source{coord_dist(rng), coord_dist(rng)};
  fpga::Coord target;
  do {
    target = {coord_dist(rng), coord_dist(rng)};
  } while (std::abs(target.x - source.x) + std::abs(target.y - source.y) < 4 ||
           std::abs(target.x - source.x) + std::abs(target.y - source.y) > 5);

  std::vector<fpga::Coord> corridor =
      makeCorridor(source, target, (seed & 1u) != 0);
  std::set<std::pair<int, int>> free_tiles;
  for (size_t i = 0; i < corridor.size(); ++i) {
    fpga::Coord coord = corridor[i];
    free_tiles.insert({coord.x, coord.y});
    fpga::Tile *tile = fpga::Device::current().getTile(coord.x, coord.y);
    require(tile, "random backward corridor tile missing");
    tile->cb_type = &fpga::Device::current().cb_types[(i + seed) & 1u];
  }
  for (fpga::Tile *tile : tiles) {
    if (!free_tiles.contains({tile->coord.x, tile->coord.y})) {
      occupyBlockedTile(*tile);
    }
  }

  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile,
          "random backward source or target tile missing");
  int source_dst = source_tile->cb_type->type_id == 0 ? first_dst : second_dst;

  pnr::DockingResult result = pnr::dockGrounding(*source_tile, source_dst, "",
                                                 *target_tile, bit(20), 5, 5);
  require(result.success,
          "random multi-hop backward docking failed for seed " +
              std::to_string(seed) +
              " bpop=" + std::to_string(result.backward_pop_count) +
              " bpush=" + std::to_string(result.backward_push_count));
  require(
      result.backward_push_count >= 2,
      "random docking did not exercise multi-hop backward expansion for seed " +
          std::to_string(seed));
  require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
              result.fragments.back().local == 20,
          "random backward docking did not finish at the requested pin");

  int cross_type_edges = 0;
  for (const fpga::Wire &fragment : result.fragments) {
    if (fragment.type != fpga::Wire::WIRE_CROSSBAR || fragment.jump < 0) {
      continue;
    }
    fpga::Tile *from_tile =
        fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
    fpga::Tile *to_tile =
        fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
    require(from_tile && to_tile && from_tile->cb_type && to_tile->cb_type,
            "random backward route references a missing tile");
    require((from_tile->cb_type->dsts_reaching_src[fragment.jump].jump &
             bit(fragment.local)) != NodeMask{},
            "random backward route used a source unreachable from its previous "
            "dst");
    std::vector<fpga::TileJumpTarget> targets =
        fpga::Device::current().resolveJumpTargets(*from_tile, fragment.jump);
    bool resolved = std::any_of(targets.begin(), targets.end(),
                                [&](const fpga::TileJumpTarget &candidate) {
                                  return candidate.tile == to_tile &&
                                         candidate.dst_node == fragment.dst;
                                });
    require(resolved, "random backward route edge is absent from dst_by_src");
    if (from_tile->cb_type != to_tile->cb_type) {
      ++cross_type_edges;
    }
  }
  require(cross_type_edges >= 2, "random backward route did not cross enough "
                                 "distinct destination namespaces");
}

void docking_memoizes_failed_backward_positions() {
  fpga::CBType cb = makeDockingCrossbar();
  resetGrid(9, 9, cb);
  constexpr int radius = 3;
  constexpr int max_depth = 3;
  fpga::Coord source{1, 1};
  fpga::Coord target{4, 4};
  fpga::Tile *source_tile = fpga::Device::current().getTile(source.x, source.y);
  fpga::Tile *target_tile = fpga::Device::current().getTile(target.x, target.y);
  require(source_tile && target_tile,
          "memoized docking source or target tile missing");

  // Block every forward exit at the anchor so the backward search exhausts
  // a cyclic grid and must encounter already-expanded (coord,dst) positions.
  source_tile->cb.src.jump = source_tile->cb_type->dst_src[0].jump;
  pnr::DockingResult result = pnr::dockGrounding(
      *source_tile, 0, "D0", *target_tile, bit(20), max_depth, radius);
  require(!result.success,
          "memoized docking unexpectedly escaped the blocked anchor");
  require(result.backward_pop_count > 4,
          "memoized docking test did not expand enough backward positions");
  require(result.backward_deadend_count == result.backward_pop_count,
          "docking did not memoize every exhausted backward position");
  require(
      result.backward_deadend_reject_count > 0,
      "docking did not prune a branch returning to a memoized failed position");
  // Traced search materializes rejected edges for diagnostics. The numeric
  // fast path must preserve its search and rejection accounting exactly.
  auto traced = pnr::dockGrounding(*source_tile, 0, "D0", *target_tile,
                                  bit(20), max_depth, radius, true);
  require(traced.success == result.success &&
              traced.backward_pop_count == result.backward_pop_count &&
              traced.backward_push_count == result.backward_push_count &&
              traced.backward_busy_reject_count == result.backward_busy_reject_count &&
              traced.backward_deadend_reject_count == result.backward_deadend_reject_count &&
              traced.backward_seen_reject_count == result.backward_seen_reject_count,
          "numeric early rejection changed the fully materialized search");

  // Standalone grounding indexes the complete topology once, so incoming
  // long wires are present even when their source is outside the window.
  int one_device_scan_limit = 0;
  for (int y = 0; y < fpga::Device::current().size_height; ++y) {
    for (int x = 0; x < fpga::Device::current().size_width; ++x) {
      fpga::Tile *tile = fpga::Device::current().getTile(x, y);
      if (tile && tile->cb_type) {
        one_device_scan_limit +=
            static_cast<int>(tile->cb_type->dst_by_src.values.size());
      }
    }
  }
  require(result.backward_mapping_scan_count <= one_device_scan_limit,
          "docking rescanned the reverse jump database after its temporary "
          "index was built");
}

void docking_reports_exact_busy_bridge_between_separate_frontiers() {
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(3, 1, cb);
  fpga::Tile *source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *bridge = fpga::Device::current().getTile(1, 0);
  fpga::Tile *target = fpga::Device::current().getTile(2, 0);
  require(source && bridge && target, "linear docking grid is incomplete");
  int east = encodedJump(1, 0);

  // The forward side can reach the bridge tile and the backward side can
  // reach the target side, but one live transit source separates them.
  bridge->cb.src.jump |= bit(east);
  pnr::DockingResult result =
      pnr::dockGrounding(*source, 0, "ENTRY", *target, bit(20), 5, 5);
  require(!result.success,
          "docking crossed an occupied edge between its two frontiers");
  require(std::any_of(
              result.forward_frontier.begin(), result.forward_frontier.end(),
              [](const pnr::DockingFrontierNode &node) {
                return node.coord.x == 1 && node.coord.y == 0 && node.dst == 0;
              }),
          "forward docking frontier did not retain the bridge position");
  require(std::any_of(
              result.backward_frontier.begin(), result.backward_frontier.end(),
              [](const pnr::DockingFrontierNode &node) {
                return node.coord.x == 2 && node.coord.y == 0 && node.dst == 0;
              }),
          "backward docking frontier did not retain the target position");
  auto blocker = std::find_if(
      result.blocked_bridges.begin(), result.blocked_bridges.end(),
      [&](const pnr::DockingBridgeBlocker &candidate) {
        return candidate.valid && candidate.tile.x == 1 &&
               candidate.tile.y == 0 && candidate.dst == 0 &&
               candidate.src == east && candidate.landing_tile.x == 2 &&
               candidate.landing_tile.y == 0 && candidate.landing_dst == 0;
      });
  require(blocker != result.blocked_bridges.end(),
          "docking did not expose the exact edge separating its frontiers");
  require(blocker->src_busy && !blocker->dst_busy && !blocker->joint_busy &&
              !blocker->joint2_busy,
          "docking attributed the bridge failure to the wrong node lease");
  require(blocker->forward_prefix.size() == 1 &&
              blocker->backward_suffix.size() >= 2,
          "docking did not preserve both sides of the blocked bridge");
  pnr::DockingResult materialized;
  require(pnr::materializeDockingBridge(*blocker, materialized) &&
              materialized.success && materialized.fragments.size() ==
                                          blocker->forward_prefix.size() +
                                              blocker->backward_suffix.size(),
          "docking could not materialize its proven bridge path");
  require(materialized.fragments.front().from.x == source->coord.x &&
              materialized.fragments.back().to.x == target->coord.x,
          "materialized bridge path does not span both docking frontiers");
}

void docking_reports_busy_bridge_at_committed_forward_anchor() {
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(2, 1, cb);
  fpga::Tile *source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *target = fpga::Device::current().getTile(1, 0);
  require(source && target, "anchor-blocked docking grid is incomplete");
  int east = encodedJump(1, 0);

  // A continued route already owns the anchor destination, while another
  // transit route owns its only exit. Docking must still resolve that exit's
  // numeric landing and expose it as the exact separator to the free target.
  source->cb.dst.jump |= bit(0);
  source->cb.src.jump |= bit(east);
  pnr::DockingResult result =
      pnr::dockGrounding(*source, 0, "ENTRY", *target, bit(20), 5, 5);
  require(!result.success,
          "docking crossed an occupied source at its committed anchor");
  auto blocker = std::find_if(
      result.blocked_bridges.begin(), result.blocked_bridges.end(),
      [&](const pnr::DockingBridgeBlocker &candidate) {
        return candidate.valid && candidate.tile.x == 0 &&
               candidate.tile.y == 0 && candidate.dst == 0 &&
               candidate.src == east && candidate.landing_tile.x == 1 &&
               candidate.landing_tile.y == 0 && candidate.landing_dst == 0;
      });
  require(blocker != result.blocked_bridges.end(),
          "docking discarded the occupied bridge at its forward anchor");
  require(blocker->src_busy && !blocker->dst_busy && !blocker->joint_busy &&
              !blocker->joint2_busy,
          "anchor bridge blocker did not identify the occupied source bit");
  require(blocker->forward_prefix.empty() &&
              blocker->backward_suffix.size() >= 3,
          "anchor bridge blocker did not preserve the complete target suffix");
  pnr::DockingResult materialized;
  require(pnr::materializeDockingBridge(*blocker, materialized) &&
              materialized.fragments.size() == blocker->backward_suffix.size() &&
              materialized.fragments.front().from.x == source->coord.x &&
              materialized.fragments.front().jump == east &&
              materialized.fragments.back().to.x == target->coord.x,
          "anchor bridge could not reuse its proven target suffix");
}

void docking_reports_busy_reverse_boundary_before_frontiers_join() {
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(4, 1, cb);
  fpga::Tile *source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *blocker_tile = fpga::Device::current().getTile(2, 0);
  fpga::Tile *target = fpga::Device::current().getTile(3, 0);
  require(source && blocker_tile && target,
          "reverse-boundary docking grid is incomplete");
  int east = encodedJump(1, 0);

  // A one-layer search cannot join the two frontiers. It must still expose
  // the exact busy reverse-boundary edge instead of silently discarding it.
  blocker_tile->cb.src.jump |= bit(east);
  pnr::DockingResult blocked =
      pnr::dockGrounding(*source, 0, "ENTRY", *target, bit(20), 1, 5);
  require(!blocked.success,
          "bounded docking crossed an occupied reverse-boundary edge");
  auto blocker = std::find_if(
      blocked.blocked_bridges.begin(), blocked.blocked_bridges.end(),
      [&](const pnr::DockingBridgeBlocker &candidate) {
        return candidate.valid && !candidate.joins_frontiers &&
               candidate.tile.x == 2 && candidate.tile.y == 0 &&
               candidate.dst == 0 && candidate.src == east &&
               candidate.landing_tile.x == 3 &&
               candidate.landing_tile.y == 0 && candidate.landing_dst == 0;
      });
  require(blocker != blocked.blocked_bridges.end(),
          "docking discarded the exact occupied reverse-frontier boundary");
  pnr::DockingResult materialized;
  require(!pnr::materializeDockingBridge(*blocker, materialized),
          "an incomplete boundary blocker was materialized as a full route");

  // Once the caller removes that exact transit lease, the same numeric path
  // becomes routable without changing endpoint or crossbar topology.
  blocker_tile->cb.src.jump &= ~bit(east);
  pnr::DockingResult retried =
      pnr::dockGrounding(*source, 0, "ENTRY", *target, bit(20), 2, 5);
  require(retried.success,
          "docking did not use the boundary path after its exact cut");
}

void docking_preserves_bridges_from_blocked_terminal_search() {
  fpga::CBType route_cb{};
  route_cb.name = "ROUTE_PHASE";
  route_cb.type_id = 0;
  fpga::CBType target_cb{};
  target_cb.name = "TARGET_PHASE";
  target_cb.type_id = 1;
  constexpr int route_dst = 0;
  constexpr int blocked_entry = 0;
  constexpr int free_entry = 2;
  constexpr int pin = 20;
  int east = encodedJump(1, 0);

  route_cb.rememberNodeName(fpga::CB_NODE_DST, route_dst, "ROUTE_DST");
  route_cb.rememberNodeName(fpga::CB_NODE_SRC, east, "ROUTE_EAST");
  route_cb.dst_src[route_dst].jump |= bit(east);
  rememberConn(route_cb, fpga::CB_NODE_DST, route_dst, fpga::CB_NODE_SRC,
               east);
  rememberJumpTarget(route_cb, east, route_dst, {1, 0}, 0);
  rememberJumpTarget(route_cb, east, blocked_entry, {1, 0}, 1);
  route_cb.rebuildOutgoingSrcs();

  target_cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "TARGET_PIN");
  for (int entry : {blocked_entry, free_entry}) {
    target_cb.rememberNodeName(fpga::CB_NODE_DST, entry,
                               "TARGET_" + std::to_string(entry));
    target_cb.dst_local[entry].local |= bit(pin);
    rememberConn(target_cb, fpga::CB_NODE_DST, entry, fpga::CB_NODE_LOCAL,
                 pin);
  }
  target_cb.rebuildOutgoingSrcs();

  resetTwoTypeGrid(3, 1, route_cb, target_cb, {2, 0});
  fpga::Tile *source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *bridge = fpga::Device::current().getTile(1, 0);
  fpga::Tile *target = fpga::Device::current().getTile(2, 0);
  require(source && bridge && target,
          "two-phase docking grid is incomplete");

  // The free terminal has no incoming route. The occupied terminal is probed
  // only in docking's second backward phase, where the busy bridge is found.
  target->cb.dst.jump |= bit(blocked_entry);
  bridge->cb.src.jump |= bit(east);
  pnr::BackwardResolveIndex backward_index;
  backward_index.center = target->coord;
  backward_index.radius = 5;
  backward_index.sources[{2, 0, blocked_entry}].push_back(
      {bridge, east, east});

  pnr::DockingResult result = pnr::dockGrounding(
      *source, route_dst, "ROUTE_DST", *target, bit(pin), 5, 5, false, {},
      &backward_index);
  require(!result.success,
          "two-phase docking crossed an occupied terminal bridge");
  require(result.target_seed_count == 1 && result.target_busy_count == 1,
          "two-phase docking did not exercise both terminal seed classes");
  // dockGrounding combines the bounded and expanded searches, so each
  // position may appear once in each snapshot, but not once per seed phase.
  std::multiset<std::array<int, 3>> forward_positions;
  for (const auto &node : result.forward_frontier) {
    std::array key{node.coord.x, node.coord.y, node.dst};
    forward_positions.insert(key);
    require(forward_positions.count(key) <= 2,
            "second backward phase duplicated the unchanged forward frontier");
  }
  for (int entry : {free_entry, blocked_entry})
    require(std::any_of(result.backward_frontier.begin(), result.backward_frontier.end(),
                       [&](const auto &node) {
                         return node.coord.x == 2 && node.coord.y == 0 && node.dst == entry;
                       }),
            "two-phase frontier lost a free or occupied terminal seed");
  auto blocker = std::find_if(
      result.blocked_bridges.begin(), result.blocked_bridges.end(),
      [&](const pnr::DockingBridgeBlocker &candidate) {
        return candidate.valid && candidate.tile.x == 1 &&
               candidate.tile.y == 0 && candidate.dst == route_dst &&
               candidate.src == east && candidate.landing_tile.x == 2 &&
               candidate.landing_tile.y == 0 &&
               candidate.landing_dst == blocked_entry;
      });
  require(blocker != result.blocked_bridges.end(),
          "docking discarded a bridge found by blocked-terminal search");
  require(blocker->src_busy,
          "blocked-terminal bridge did not retain its exact busy source");
  // This geometry still has a blocked terminal. Materialization alone is not
  // an ownership-aware lease proof: acceptance must check the victim footprint.
  pnr::DockingResult proposed;
  require(blocker->blocked_terminal &&
              pnr::materializeDockingBridge(*blocker, proposed),
          "blocked-terminal bridge lost its classification or numeric path");
  require(bridge->cb.src.jump.testBit(east) &&
              target->cb.dst.jump.testBit(blocked_entry),
          "rejecting the two-blocker bridge changed live leases");
}

void docking_finds_blocked_bridge_after_full_backward_beam() {
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(13, 13, cb);
  fpga::Coord source_coord{4, 6};
  fpga::Coord middle_coord{5, 6};
  fpga::Coord target_coord{6, 6};
  fpga::Tile *source = fpga::Device::current().getTile(source_coord.x,
                                                       source_coord.y);
  fpga::Tile *middle = fpga::Device::current().getTile(middle_coord.x,
                                                       middle_coord.y);
  fpga::Tile *target = fpga::Device::current().getTile(target_coord.x,
                                                       target_coord.y);
  require(source && middle && target,
          "late blocked-bridge docking grid is incomplete");
  int east = encodedJump(1, 0);

  pnr::BackwardResolveIndex backward_index;
  backward_index.center = target_coord;
  backward_index.radius = 5;
  std::vector<fpga::Tile *> decoys;
  for (fpga::Tile &tile : fpga::Device::current().tile_grid) {
    if (&tile != source && &tile != middle && &tile != target) {
      decoys.push_back(&tile);
    }
  }
  require(decoys.size() >= 33,
          "late blocked-bridge test has too few decoy source tiles");
  for (int index = 0; index < 33; ++index) {
    backward_index.sources[{target_coord.x, target_coord.y, 0}].push_back(
        {decoys[index], east, east});
  }
  // Put the useful incoming source after the complete 32-entry search beam.
  backward_index.sources[{target_coord.x, target_coord.y, 0}].push_back(
      {middle, east, east});

  // The forward anchor's only edge is occupied. Backward search must inspect
  // all numeric incoming sources and retain the bridge landing even when no
  // additional backward queue slot remains.
  source->cb.dst.jump |= bit(0);
  source->cb.src.jump |= bit(east);
  pnr::DockingResult result = pnr::dockGrounding(
      *source, 0, "ANCHOR", *target, bit(20), 5, 5, false, {},
      &backward_index);
  require(!result.success,
          "docking crossed the occupied late bridge source");
  auto blocker = std::find_if(
      result.blocked_bridges.begin(), result.blocked_bridges.end(),
      [&](const pnr::DockingBridgeBlocker &candidate) {
        return candidate.valid && candidate.tile.x == source_coord.x &&
               candidate.tile.y == source_coord.y && candidate.dst == 0 &&
               candidate.src == east &&
               candidate.landing_tile.x == middle_coord.x &&
               candidate.landing_tile.y == middle_coord.y &&
               candidate.landing_dst == 0;
      });
  require(blocker != result.blocked_bridges.end(),
          "backward beam truncation hid the occupied frontier bridge");
  require(blocker->src_busy,
          "late blocked bridge did not retain its occupied source bit");
}

void docking_preserves_and_checks_both_terminal_joints() {
  constexpr int dst = 0;
  constexpr int first_joint = 11;
  constexpr int second_joint = 12;
  constexpr int pin = 20;

  fpga::CBType cb = makeTwoJointTerminalCrossbar();
  resetGrid(1, 1, cb);
  fpga::Tile *tile = fpga::Device::current().getTile(0, 0);
  require(tile, "two-joint docking tile missing");

  // A successful terminal route must retain both intermediate nodes so the
  // committed route leases and exports the complete physical connection.
  pnr::DockingResult result =
      pnr::dockGrounding(*tile, dst, "ENTRY", *tile, bit(pin), 5, 5);
  require(result.success, "two-joint terminal route was not found");
  require(result.fragments.size() >= 2,
          "two-joint terminal route omitted its endpoint fragments");
  require(result.fragments[0].joint == second_joint,
          "two-joint terminal route lost the joint adjacent to the local pin");
  require(result.fragments[0].joint2 == first_joint,
          "two-joint terminal route lost the joint adjacent to the destination "
          "node");

  // Occupying either intermediate node must reject the same terminal path;
  // otherwise two routes can pass unit tests but conflict in the real fabric.
  for (int occupied_joint : {first_joint, second_joint}) {
    cb = makeTwoJointTerminalCrossbar();
    resetGrid(1, 1, cb);
    tile = fpga::Device::current().getTile(0, 0);
    require(tile, "occupied two-joint docking tile missing");
    tile->cb.joint.jump |= bit(occupied_joint);
    result = pnr::dockGrounding(*tile, dst, "ENTRY", *tile, bit(pin), 5, 5);
    require(!result.success,
            "two-joint terminal route ignored an occupied intermediate node");
    require(result.target_busy_count > 0,
            "two-joint terminal occupancy was not classified as busy");
    // Source-moving reverse proofs must agree: a free DST and final joint
    // do not make an entry free when its other required joint is occupied.
    const auto reverse = pnr::routeBackwardToInput(
        *tile, bit(pin), tile->coord, 5,
        [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; },
        nullptr, {}, nullptr, nullptr);
    require(!reverse.success && reverse.expanded == 0 &&
                reverse.diagnostic_fragments.size() == 2 &&
                reverse.diagnostic_fragments.front().joint == second_joint &&
                reverse.diagnostic_fragments.front().joint2 == first_joint &&
                tile->cb.joint.jump == bit(occupied_joint),
            "reverse input proof lost a busy joint or mutated its lease");
  }

  // Packed endpoint reservations are topology constraints rather than live
  // leases. Docking must avoid either reserved joint just like direct entry.
  for (int reserved_joint : {-1, first_joint, second_joint, 4095}) {
    for (bool allow_preemption : {false, true}) {
      cb = makeTwoJointTerminalCrossbar();
      resetGrid(1, 1, cb);
      tile = fpga::Device::current().getTile(0, 0);
      require(tile, "reserved two-joint docking tile missing");
      pnr::DockingPreemptionSearch policy;
      policy.eligible_edge = [](const auto&) { return true; };
      policy.accept_complete = [](const auto&) { return true; };
      result = pnr::dockGrounding(*tile, dst, "ENTRY", *tile, bit(pin), 5, 5,
          false, reserved_joint < 0 ? NodeMask{} : bit(reserved_joint),
          nullptr, {}, false, allow_preemption ? &policy : nullptr);
      const bool blocked = reserved_joint == first_joint || reserved_joint == second_joint;
      require(result.success == !blocked,
              "two-joint docking reservation check changed terminal admission");
      require(tile->cb.joint.jump == NodeMask{},
              "reserved-joint docking changed the live joint leases");
    }
  }
}

void backward_input_avoids_proposed_output(bool input_can_alternate) {
  constexpr int output_pin = 20, input_pin = 21, local = 31;
  int east = encodedJump(1, 0), alternate = encodedJump(1, 0, 1);
  int return_src = encodedJump(-2, 0);
  auto cb = makeLinearDockingCrossbar();
  cb.local_src[local].jump = bit(east) | bit(alternate);
  cb.dst_src[0].jump |= bit(return_src);
  cb.dst_src[1].jump = bit(return_src);
  cb.dst_local[2].local = bit(input_pin);
  if (!input_can_alternate) {
    // Only the output can avoid the collision; the placement itself is valid.
    cb.dst_src[1].jump |= bit(alternate);
    cb.dst_local[1].local |= bit(output_pin);
  }
  cb.rememberNodeName(fpga::CB_NODE_DST, 1, "ALTERNATE_ENTRY");
  cb.rememberNodeName(fpga::CB_NODE_DST, 2, "INPUT_ENTRY");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, local, "DRIVER");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, input_pin, "INPUT");
  cb.rememberNodeName(fpga::CB_NODE_SRC, alternate, "ALTERNATE_SRC");
  cb.rememberNodeName(fpga::CB_NODE_SRC, return_src, "RETURN_SRC");
  rememberJumpTarget(cb, alternate, 1, {1, 0});
  rememberJumpTarget(cb, return_src, 2, {-2, 0});
  cb.rebuildOutgoingSrcs();
  resetGrid(4, 1, cb);
  auto &device = fpga::Device::current();
  auto *source = device.getTile(0, 0);
  auto *input_driver = device.getTile(1, 0);
  auto *target = device.getTile(3, 0);
  auto input_probe = [&](fpga::Tile &tile, int src,
                         pnr::BackwardTakeoffChoice &choice) {
    if (&tile != input_driver ||
        (src != east && (!input_can_alternate || src != alternate))) {
      choice.counts_toward_probe_limit = false;
      return false;
    }
    choice = {local, -1, -1};
    return true;
  };
  auto independent = pnr::routeBackwardToTakeoff(
      *source, bit(input_pin), input_driver->coord, 8, 4, 0, input_probe);
  require(independent.success && independent.source_src == east,
          "collision fixture did not select the shared SRC independently");

  size_t full_path_checks = 0;
  pnr::BackwardTakeoffRoute input;
  auto output = pnr::routeBackwardToTakeoff(
      *target, bit(output_pin), source->coord, 8, 4, 0, {},
      nullptr, {}, 32768, 0, 8, nullptr, nullptr, {},
      [&](fpga::Tile &tile, int src, pnr::BackwardTakeoffChoice &choice,
          const pnr::BackwardTakeoffPath &path) {
        if (&tile != source ||
            (src != east && (input_can_alternate || src != alternate))) {
          choice.counts_toward_probe_limit = false;
          return false;
        }
        choice = {local, -1, -1};
        const auto &proposed = path(choice);
        ++full_path_checks;
        require(proposed.size() == 5 && proposed.front().local == local &&
                    proposed.back().local == output_pin,
                "placement probe did not receive its complete output trunk");
        pnr::BackwardTakeoffStateView trial;
        for (const auto &wire : proposed) {
          auto *route_tile = device.getTile(wire.from.x, wire.from.y);
          auto &state = trial.try_emplace(route_tile, route_tile->cb).first->second;
          if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
            state.local.local.setBit(wire.local);
          } else {
            if (wire.pos != 0) state.dst.jump.setBit(wire.local);
            if (wire.jump >= 0) state.src.jump.setBit(wire.jump);
          }
        }
        input = pnr::routeBackwardToTakeoff(
            *source, bit(input_pin), input_driver->coord, 8, 4, 0,
            input_probe, nullptr, {}, 32768, 0, 8, &trial);
        return input.success;
      });
  require(output.success && input.success &&
              input.source_src == (input_can_alternate ? alternate : east) &&
              output.source_src == (input_can_alternate ? east : alternate) &&
              full_path_checks >= 1 && full_path_checks <= 2,
          "incident routing failed: output=" + std::to_string(output.source_src) +
              " input=" + std::to_string(input.source_src) +
              " path_checks=" + std::to_string(full_path_checks));
  for (int x = 0; x < 4; ++x) {
    const auto &state = device.getTile(x, 0)->cb;
    require(state.src.jump == NodeMask{} && state.dst.jump == NodeMask{} &&
                state.local.local == NodeMask{},
            "incident proof changed live routing state");
  }
}

void moving_source_explores_free_continuations_before_preemption() {
  constexpr int pin = 20;
  constexpr int local = 31;
  const int east = encodedJump(1, 0);
  fpga::CBType cb = makeLinearDockingCrossbar();
  cb.local_src[local].jump |= bit(east);
  rememberConn(cb, fpga::CB_NODE_LOCAL, local, fpga::CB_NODE_SRC, east);
  cb.rebuildOutgoingSrcs();
  fpga::CBType branching = cb;
  branching.dst_src[1].jump.setBit(east);
  branching.rebuildOutgoingSrcs();

  // The first reverse layer has a busy alternative and a free continuation.
  // Its placement is illegal; the next legal placement is two or three hops
  // from the sink. Previously a boundary cut aborted before reaching it.
  for (int hops : {2, 3}) {
    for (int budget : {0, 1, 2}) {
      resetGrid(6, 1, cb);
      auto &device = fpga::Device::current();
      auto *target = device.getTile(5, 0);
      auto *near = device.getTile(4, 0);
      near->cb_type = &branching;
      near->cb.dst.jump.setBit(1);
      size_t probes = 0, cuts = 0;
      auto probe = [&](fpga::Tile &tile, int, pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        choice = {local, -1, -1};
        return tile.coord.x == 5 - hops;
      };
      auto cut = [&](const std::vector<pnr::DockingBridgeBlocker> &blockers) {
        // This callback really changes occupancy, so invoking it before
        // the free path is tried reproduces the production failure.
        require(blockers.size() == 1 && blockers[0].tile.x == 4 &&
                    blockers[0].dst == 1 && blockers[0].dst_busy,
                "source boundary did not identify the busy alternative");
        ++cuts;
        near->cb.dst.jump.clearBit(1);
        return true;
      };
      auto route = pnr::routeBackwardToTakeoff(
          *target, bit(pin), {0, 0}, 0, 6, -1, probe, nullptr, {},
          budget == 1 ? 1 : 0, 0, budget == 2 ? 2 : 0,
          nullptr, nullptr, {}, {}, cut);
      // Free success, expansion cancellation, and a bounded placement probe
      // must all leave the unrelated victim alone.
      require(cuts == 0 && !route.boundary_released,
              "source search preempted before exhausting free continuations");
      const bool expected_success = budget != 1 && !(budget == 2 && hops == 3);
      require(route.success == expected_success,
              "source search lost a reachable placement or ignored its budget");
      if (expected_success) {
        // The accepted suffix is source-to-sink, not just a successful probe.
        require(route.source_tile == device.getTile(5 - hops, 0) &&
                    probes == static_cast<size_t>(hops) &&
                    route.expanded == static_cast<size_t>(hops),
                "source search failed to probe the deeper free placement promptly");
        int x = 5 - hops;
        for (const auto &wire : route.fragments) {
          if (wire.jump < 0) continue;
          require(wire.from.x == x && wire.to.x == x + 1 && wire.dst == 0,
                  "source search returned a disconnected free suffix");
          ++x;
        }
        require(x == 5 && route.fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
                    route.fragments.back().local == pin,
                "source search omitted the final sink pin");
      } else {
        // Budget exhaustion is not exhaustion of the free component.
        require(route.remaining_frontier != 0 &&
                    (budget == 1 ? route.expansion_limit_reached : probes == 2),
                "bounded source search did not retain its unfinished status");
        // Resume past the rejected candidates, without resetting the offset
        // merely because no deeper placements had been discovered yet.
        const size_t offset = route.probe_offset_used + route.probe_candidates_scanned;
        probes = 0;
        auto resumed = pnr::routeBackwardToTakeoff(
            *target, bit(pin), {0, 0}, 0, 6, -1, probe, nullptr, {},
            0, offset, 2, nullptr, nullptr, {}, {}, cut);
        require(resumed.success && resumed.source_tile == device.getTile(5 - hops, 0) &&
                    probes == static_cast<size_t>(hops) - offset && cuts == 0,
                "resumed source search repeated its rejected window or preempted");
      }
      for (int x = 0; x < 6; ++x) {
        const auto &state = device.getTile(x, 0)->cb;
        // Search is speculative: retain the busy owner and acquire no leases.
        require(state.dst.jump == (x == 4 ? bit(1) : NodeMask{}) &&
                    state.src.jump == NodeMask{} && state.local.local == NodeMask{} &&
                    state.joint.jump == NodeMask{},
                "source free-continuation search changed live leases");
      }
    }
  }
}

void moving_source_preempts_after_exact_probe_budget_exhausts_free_search() {
  constexpr int pin = 20;
  const int east = encodedJump(1, 0);
  fpga::CBType cb = makeLinearDockingCrossbar();
  fpga::CBType branching = cb;
  branching.dst_src[1].jump.setBit(east);
  branching.rebuildOutgoingSrcs();
  fpga::CBType endpoint = cb;
  endpoint.dst_src[0].jump = {};
  endpoint.rebuildOutgoingSrcs();
  resetGrid(3, 1, cb);
  auto &device = fpga::Device::current();
  device.getTile(0, 0)->cb_type = &endpoint;
  auto *middle = device.getTile(1, 0);
  middle->cb_type = &branching;
  middle->cb.dst.jump.setBit(1);
  int probes = 0, cuts = 0;
  const auto route = pnr::routeBackwardToTakeoff(
      *device.getTile(2, 0), bit(pin), {0, 0}, 0, 3, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
        ++probes;
        return false;
      }, nullptr, {}, 0, 0, 2, nullptr, nullptr, {}, {},
      [&](const std::vector<pnr::DockingBridgeBlocker> &blockers) {
        // Both reachable placements were rejected, including the one reached
        // through the free alternative. Now the exact blocked edge may be cut.
        require(probes == 2 && blockers.size() == 1 && blockers[0].dst == 1 &&
                    blockers[0].tile.x == 1 && blockers[0].src == east,
                "source search lost its earlier blocker or cut before exhaustion");
        ++cuts;
        middle->cb.dst.jump.clearBit(1);
        return true;
      });
  // Consuming exactly the budget is not an interruption if no work remains.
  require(!route.success && route.boundary_released && cuts == 1 &&
              route.remaining_frontier == 0 && route.expanded == 2,
          "exact-budget free exhaustion incorrectly suppressed necessary preemption");
  for (int x = 0; x < 3; ++x) {
    const auto &state = device.getTile(x, 0)->cb;
    require(state.dst.jump == NodeMask{} && state.src.jump == NodeMask{} &&
                state.local.local == NodeMask{} && state.joint.jump == NodeMask{},
            "exhausted backward search leaked speculative leases");
  }
}

void moving_source_probes_before_exhausting_reverse_component() {
  constexpr int pin = 20;
  constexpr int local = 31;
  const int east = encodedJump(1, 0);
  fpga::CBType cb = makeLinearDockingCrossbar();
  cb.local_src[local].jump |= bit(east);
  rememberConn(cb, fpga::CB_NODE_LOCAL, local, fpga::CB_NODE_SRC, east);
  cb.rebuildOutgoingSrcs();
  resetGrid(72, 1, cb);
  auto &device = fpga::Device::current();
  auto *source = device.getTile(0, 0);
  auto *target = device.getTile(71, 0);
  auto *near_target = device.getTile(70, 0);
  int probes = 0;
  int boundary_calls = 0;
  auto route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        choice = {local, -1, -1};
        return true;
      }, nullptr, {}, 0, 0, 0, nullptr, nullptr, {}, {},
      [&](const auto &) { ++boundary_calls; return false; });
  // A legal reached placement must be tried before scanning the other 70
  // tiles. This reproduced the full-component delay in the 51K-cell run.
  require(route.success && route.source_tile == near_target &&
              route.expanded == 1 && probes == 1 && boundary_calls == 0,
          "Moving Sources exhausted reverse routing before trying placement");

  near_target->cb.dst.jump.setBit(0);
  probes = 0;
  route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        choice = {local, -1, -1};
        return true;
      }, nullptr, {}, 0, 0, 0, nullptr, nullptr, {}, {},
      [&](const auto &) { ++boundary_calls; return true; });
  // A free local takeoff wins even beside a busy transit DST: do not evict
  // another route when the reached placement itself works.
  require(route.success && !route.boundary_released && boundary_calls == 0 &&
              near_target->cb.dst.jump.testBit(0),
          "Moving Sources preempted before testing its free takeoff");

  fpga::CBType branching = cb;
  branching.dst_src[1].jump.setBit(east);
  branching.rebuildOutgoingSrcs();
  auto *original_type = near_target->cb_type;
  near_target->cb_type = &branching;
  probes = 0;
  route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
        ++probes;
        return false;
      }, nullptr, {}, 0, 0, 0, nullptr, nullptr, {}, {},
      [&](const std::vector<pnr::DockingBridgeBlocker> &blockers) {
        ++boundary_calls;
        // The caller gets the exact blocked edge after placement rejection,
        // not an arbitrary node near the sink. Emulate its owner releasing it.
        require(probes == 1 && blockers.size() == 1 &&
                    blockers[0].dst == 0 && blockers[0].src == east &&
                    blockers[0].tile.x == near_target->coord.x &&
                    blockers[0].tile.y == near_target->coord.y &&
                    blockers[0].dst_busy && !blockers[0].src_busy,
                "layer boundary did not describe the exact occupied transit");
        near_target->cb.dst.jump.clearBit(0);
        return true;
      });
  // Exhaust the free continuation before changing live ownership, then return
  // for a fresh retry instead of reusing the speculative state after a cut.
  require(!route.success && route.boundary_released && route.expanded == 2 &&
              boundary_calls == 1 && route.remaining_frontier == 0 &&
              !route.diagnostic_fragments.empty(),
          "reverse search continued after releasing a transit boundary");

  near_target->cb.dst.jump.setBit(0);
  boundary_calls = 0;
  route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; },
      nullptr, {}, 0, 0, 0, nullptr, nullptr, {}, {},
      [&](const auto &) { ++boundary_calls; return false; });
  // An ineligible victim must not stop exploration of the queued free DST.
  require(!route.success && !route.boundary_released && route.expanded == 2 &&
              boundary_calls == 1 && near_target->cb.dst.jump.testBit(0),
          "rejected preemption discarded an unexplored free branch");
  near_target->cb.dst.jump.clearBit(0);
  near_target->cb_type = original_type;
  probes = 0;
  route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [&](fpga::Tile &tile, int, pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        choice = {local, -1, -1};
        return tile.coord.x == 69;
      }, nullptr, {}, 0, 0, 0, nullptr, nullptr, {}, {},
      [](const auto &) { return false; });
  // Rejected nearer takeoffs must not terminate the free reverse search;
  // the next depth layer still gets a chance, without any live reservations.
  require(route.success && route.source_tile->coord.x == 69 &&
              route.expanded == 2 && probes == 2,
          "failed layer probes hid the next reachable placement");
  for (int x = 0; x < 72; ++x) {
    const auto &state = device.getTile(x, 0)->cb;
    require(state.src.jump == NodeMask{} && state.dst.jump == NodeMask{} &&
                state.local.local == NodeMask{},
            "layer probing leaked speculative leases");
  }

  near_target->cb.dst.jump.setBit(0);
  bool cancelled = false;
  boundary_calls = 0;
  route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
        cancelled = true;
        return false;
      }, nullptr, [&] { return cancelled; }, 0, 0, 0, nullptr, nullptr, {}, {},
      [&](const auto &) { ++boundary_calls; return true; });
  // Stage cancellation still forbids preemption, even when it arrives in a
  // placement probe between expansion and the layer boundary callback.
  require(!route.success && !route.boundary_released && boundary_calls == 0 &&
              near_target->cb.dst.jump.testBit(0),
          "cancelled source search released another route");
}

void moving_source_reverse_search_depth_is_ten_hops() {
  constexpr int pin = 20;
  constexpr int local = 31;
  const int east = encodedJump(1, 0);
  const int depth = pnr::movingSourceSearchDepth();
  require(depth == 10, "Moving Sources depth policy changed");
  fpga::CBType cb = makeLinearDockingCrossbar();
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, local, "SOURCE_LOCAL");
  cb.local_src[local].jump |= bit(east);
  rememberConn(cb, fpga::CB_NODE_LOCAL, local, fpga::CB_NODE_SRC, east);
  cb.rebuildOutgoingSrcs();
  resetGrid(12, 1, cb);
  auto *source = fpga::Device::current().getTile(0, 0);
  const std::vector<pnr::BackwardRouteAnchor> anchors{{source, 0, "ENTRY", 1}};
  auto source_probe = [&](fpga::Tile &tile, int src,
                          pnr::BackwardTakeoffChoice &choice) {
    choice.counts_toward_probe_limit = false;
    if (&tile != source || src != east) return false;
    choice = {local, -1, -1};
    return true;
  };
  // All three Moving Sources searches must accept exactly ten hops, reject
  // eleven, and exhaust the bounded frontier without needing cancellation.
  for (int distance : {10, 11}) {
    auto *target = fpga::Device::current().getTile(distance, 0);
    const bool expected = distance == 10;
    auto takeoff = pnr::routeBackwardToTakeoff(
        *target, bit(pin), source->coord, depth, 12,
        pnr::movingSourcePlacementRadius(), source_probe, nullptr, {}, 0, 0,
        pnr::movingSourceProbeLimit());
    auto prefix = pnr::routeBackwardToAnchors(
        *target, bit(pin), anchors, depth, 12, nullptr, {}, 0);
    auto input = pnr::routeBackwardToInput(
        *target, bit(pin), source->coord, 12, source_probe, nullptr, {},
        nullptr, nullptr, depth);
    auto anchored_input = pnr::routeBackwardToInput(
        *target, bit(pin), source->coord, 12, source_probe, nullptr, {},
        nullptr, &anchors, depth);
    for (const auto *result : {&takeoff, &prefix, &input, &anchored_input}) {
      require(result->success == expected,
              "reverse search violated the ten-hop boundary");
      if (!expected) {
        require(result->remaining_frontier == 0 &&
                    result->diagnostic_depth_limit_reached,
                "depth-limited search left work queued for retry");
      }
    }
    // Establish that rejection is the new depth policy, not broken topology.
    auto unrestricted = pnr::routeBackwardToInput(
        *target, bit(pin), source->coord, 12, source_probe, nullptr, {},
        nullptr, nullptr);
    require(unrestricted.success, "reference linear route is not connected");
    require(source->cb.src.jump == NodeMask{} &&
                target->cb.dst.jump == NodeMask{} &&
                target->cb.local.local == NodeMask{},
            "reverse depth test changed live leases");
  }
}

void backward_route_stops_at_first_legal_takeoff() {
  constexpr int pin = 20;
  constexpr int local = 31;
  int east = encodedJump(1, 0);
  fpga::CBType cb = makeLinearDockingCrossbar();
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, local, "SOURCE_LOCAL");
  cb.local_src[local].jump |= bit(east);
  rememberConn(cb, fpga::CB_NODE_LOCAL, local, fpga::CB_NODE_SRC, east);
  cb.rebuildOutgoingSrcs();
  resetGrid(4, 1, cb);

  fpga::Tile *source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *target = fpga::Device::current().getTile(3, 0);
  require(source && target, "backward takeoff test grid is incomplete");
  int probes = 0;
  bool probed_destination_side = false;
  pnr::BackwardTakeoffRoute route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, 0,
      [&](fpga::Tile &tile, int src,
          pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        if (&tile != source || src != east) {
          probed_destination_side = true;
          choice = {local, -1, -1};
          return true;
        }
        choice = {local, -1, -1};
        return true;
      });

  // Check: reverse traversal reaches the requested source tile and emits a
  // complete forward-ordered route without changing any live lease mask.
  require(route.success && route.source_tile == source &&
              route.source_src == east && route.takeoff.local == local,
          "backward routing did not stop at the accepted source takeoff");
  require(route.fragments.size() == 5 &&
              route.fragments.front().from.x == 0 &&
              route.fragments.front().to.x == 1 &&
              route.fragments.front().local == local &&
              route.fragments.front().jump == east &&
              route.fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
              route.fragments.back().local == pin,
          "backward routing did not materialize the complete ordered trunk");
  require(probes > 0 && !probed_destination_side &&
              route.probe_calls == static_cast<size_t>(probes) &&
              source->cb.src.jump == NodeMask{} &&
              target->cb.dst.jump == NodeMask{} &&
              target->pin_state.leased_nodes == NodeMask{},
          "backward placement probing changed live routing state");

  // Check: a stage deadline can stop a speculative reverse search before it
  // scans placement candidates or mutates any routing state.
  probes = 0;
  pnr::BackwardTakeoffRoute cancelled = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, 0,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
        ++probes;
        return false;
      },
      nullptr, []() { return true; });
  require(!cancelled.success && cancelled.expanded == 0 && probes == 0 &&
              cancelled.failure_tile == target,
          "cancelled backward routing continued into placement probing");

  // Check: one difficult source probe yields at its explicit expansion budget
  // so later Moving Sources tasks cannot be starved by one reverse search.
  pnr::BackwardTakeoffRoute bounded = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, 0,
      [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; },
      nullptr, {}, 1);
  require(!bounded.success && bounded.expanded == 1 &&
              bounded.failure_tile == target && bounded.failure_dst >= 0 &&
              !bounded.diagnostic_fragments.empty() &&
              bounded.diagnostic_fragments.back().type ==
                  fpga::Wire::WIRE_TILE_PIN &&
              bounded.diagnostic_fragments.back().from.x == target->coord.x &&
              source->cb.src.jump == NodeMask{} &&
              target->cb.dst.jump == NodeMask{},
          "backward source search ignored its expansion budget");

  // Check: a deepest frontier stopped by max_depth still resolves and reports
  // its incoming edge; a zero diagnostic must not masquerade as missing topology.
  pnr::BackwardTakeoffRoute depth_bounded = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 1, 4, 0,
      [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; });
  require(!depth_bounded.success && depth_bounded.diagnostic_depth == 1 &&
              depth_bounded.diagnostic_depth_limit_reached &&
              depth_bounded.diagnostic_incoming_sources != 0 &&
              depth_bounded.diagnostic_previous_dsts == 0,
          "depth-limited backward frontier falsely reported no incoming edge");

  // Check: a blocked destination that cannot seed reverse expansion still
  // reports its concrete DST-to-local attempt for terminal visualization.
  target->cb.local.local.setBit(pin);
  pnr::BackwardTakeoffRoute blocked_diagnostic =
      pnr::routeBackwardToTakeoff(
          *target, bit(pin), source->coord, 8, 4, 0,
          [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
            return false;
          });
  require(!blocked_diagnostic.success && blocked_diagnostic.expanded == 0 &&
              blocked_diagnostic.diagnostic_fragments.size() == 2 &&
              blocked_diagnostic.diagnostic_fragments.front().type ==
                  fpga::Wire::WIRE_CROSSBAR &&
              blocked_diagnostic.diagnostic_fragments.back().type ==
                  fpga::Wire::WIRE_TILE_PIN &&
              blocked_diagnostic.diagnostic_fragments.back().local == pin,
          "blocked backward endpoint lost its visualization path");
  target->cb.local.local.clearBit(pin);

  // Check: a bounded reverse walk must not use its fallback to bypass the
  // requested source neighborhood. This used to move sources arbitrarily far.
  fpga::Tile *frontier_source = fpga::Device::current().getTile(2, 0);
  require(frontier_source, "bounded backward frontier tile missing");
  probes = 0;
  pnr::BackwardTakeoffRoute frontier_route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, 0,
      [&](fpga::Tile &tile, int src,
          pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        if (&tile != frontier_source || src != east) {
          return false;
        }
        choice = {local, -1, -1};
        return true;
      },
      nullptr, {}, 1);
  require(!frontier_route.success && probes == 0,
          "bounded backward routing probed outside its source radius");

  // Check: the same proven frontier is accepted when the caller explicitly
  // grants a neighborhood that contains it.
  frontier_route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, 2,
      [&](fpga::Tile &tile, int src,
          pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        if (&tile != frontier_source || src != east) {
          return false;
        }
        choice = {local, -1, -1};
        return true;
      },
      nullptr, {}, 1);
  require(frontier_route.success &&
              frontier_route.source_tile == frontier_source &&
              frontier_route.fragments.size() == 3 && probes == 1,
          "bounded backward routing discarded an in-radius move site");
  require(frontier_source->cb.src.jump == NodeMask{} &&
              target->cb.dst.jump == NodeMask{},
          "frontier placement probing changed live routing leases");

  // Check: an equally near frontier rejected by packing cannot hide another
  // reached frontier at the same distance. The old record-minimum list kept
  // only the first tile and incorrectly reported that no placement existed.
  constexpr int branch_local = 32;
  int north_east = encodedJump(1, -1);
  fpga::CBType branched = makeLinearDockingCrossbar();
  branched.rememberNodeName(fpga::CB_NODE_LOCAL, branch_local,
                            "BRANCH_SOURCE_LOCAL");
  branched.rememberNodeName(fpga::CB_NODE_SRC, north_east,
                            "NORTH_EAST");
  branched.dst_src[0].jump |= bit(north_east);
  branched.local_src[branch_local].jump |= bit(east) | bit(north_east);
  rememberJumpTarget(branched, north_east, 0, fpga::Coord{1, -1});
  rememberConn(branched, fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC,
               north_east);
  rememberConn(branched, fpga::CB_NODE_LOCAL, branch_local,
               fpga::CB_NODE_SRC, east);
  rememberConn(branched, fpga::CB_NODE_LOCAL, branch_local,
               fpga::CB_NODE_SRC, north_east);
  branched.rebuildOutgoingSrcs();
  resetGrid(4, 2, branched);
  source = fpga::Device::current().getTile(0, 0);
  target = fpga::Device::current().getTile(3, 0);
  require(source && target, "branched backward frontier grid is incomplete");
  probes = 0;
  pnr::BackwardTakeoffRoute equal_frontier_route =
      pnr::routeBackwardToTakeoff(
          *target, bit(pin), source->coord, 8, 4, 2,
          [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &choice) {
            ++probes;
            if (probes == 1) {
              return false;
            }
            choice = {branch_local, -1, -1};
            return true;
          },
          nullptr, {}, 1);
  require(equal_frontier_route.success && probes == 2,
          "a rejected equal-distance frontier hid a legal sibling frontier");
  require(equal_frontier_route.source_tile->cb.src.jump == NodeMask{} &&
              target->cb.dst.jump == NodeMask{},
          "equal-distance frontier probing changed live routing leases");

  // Check: unrestricted route-guided probing accepts the first legal
  // direction-led frontier even when it lies outside the old source's local
  // neighborhood. Moving Sources uses this mode so one narrow radius cannot
  // consume the complete reverse-search budget.
  fpga::Tile *remote_frontier = fpga::Device::current().getTile(2, 0);
  probes = 0;
  pnr::BackwardTakeoffRoute unrestricted = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, -1,
      [&](fpga::Tile &tile, int src,
          pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        if (&tile != remote_frontier || src != east) {
          return false;
        }
        choice = {local, -1, -1};
        return true;
      });
  require(unrestricted.success && unrestricted.source_tile == remote_frontier &&
              probes > 0,
          "unrestricted backward probing ignored a legal remote frontier");

  // Check: unrestricted probing finishes its bounded reverse walk before
  // accepting placement. When every reached takeoff is legal, the route must
  // retain the source-side candidate instead of moving to the first candidate
  // encountered beside the destination.
  probes = 0;
  pnr::BackwardTakeoffRoute nearest = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        choice = {local, -1, -1};
        return true;
      });
  require(nearest.success && nearest.source_tile == source && probes == 1,
          "backward placement probing accepted a farther frontier before the "
          "nearest reached source-side takeoff");

  // Check: zero depth and expansion limits exhaust the reachable frontier.
  // More than 64 rejected probes must expose a resumable candidate window.
  resetGrid(72, 1, cb);
  source = fpga::Device::current().getTile(0, 0);
  target = fpga::Device::current().getTile(71, 0);
  require(source && target, "long backward takeoff grid is incomplete");
  probes = 0;
  pnr::BackwardTakeoffRoute after_rejected_prefix =
      pnr::routeBackwardToTakeoff(
          *target, bit(pin), source->coord, 0, 72, -1,
          [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &choice) {
            ++probes;
            if (probes <= 64) {
              return false;
            }
            choice = {local, -1, -1};
            return true;
          },
          nullptr, {}, 0, 0, 64);
  require(!after_rejected_prefix.success && probes == 64 &&
              after_rejected_prefix.takeoff_candidates > 64 &&
              !after_rejected_prefix.expansion_limit_reached &&
              after_rejected_prefix.remaining_frontier == 0,
          "bounded backward placement probing did not expose its remaining "
          "candidate window");
  after_rejected_prefix = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 0, 72, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &choice) {
        ++probes;
        choice = {local, -1, -1};
        return true;
      },
      nullptr, {}, 0, 64, 64);
  require(after_rejected_prefix.success && probes == 65 &&
              after_rejected_prefix.probe_offset_used == 64,
          "backward placement probing did not resume after its rejected "
          "64-candidate prefix");

  // Check: numeric reverse frontiers with no packable source candidate do not
  // consume the bounded placement-probe window. Sparse placement can leave the
  // only usable source behind many such routing-only frontier nodes.
  int frontier_callbacks = 0;
  int placement_probes = 0;
  pnr::BackwardTakeoffRoute sparse_candidates = pnr::routeBackwardToTakeoff(
      *target, bit(pin), target->coord, 0, 72, -1,
      [&](fpga::Tile &tile, int src, pnr::BackwardTakeoffChoice &choice) {
        ++frontier_callbacks;
        if (&tile != source || src != east) {
          choice.counts_toward_probe_limit = false;
          return false;
        }
        ++placement_probes;
        choice = {local, -1, -1};
        return true;
      },
      nullptr, {}, 0, 0, 2);
  require(sparse_candidates.success && sparse_candidates.source_tile == source &&
              frontier_callbacks > 2 && placement_probes == 1 &&
              sparse_candidates.probe_calls == 1 &&
              sparse_candidates.probe_candidates_scanned ==
                  static_cast<size_t>(frontier_callbacks),
          "routing-only reverse frontiers exhausted the placement-probe budget");

  // Check: a partial trunk's own takeoff lease blocks an ordinary reverse
  // search, but an ownership-aware copied state can replace that same trunk.
  resetGrid(4, 1, cb);
  source = fpga::Device::current().getTile(0, 0);
  target = fpga::Device::current().getTile(3, 0);
  require(source && target, "owned-prefix backward takeoff grid is incomplete");
  source->cb.src.jump |= bit(east);
  pnr::BackwardTakeoffRoute self_blocked = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, -1,
      [&](fpga::Tile &tile, int src, pnr::BackwardTakeoffChoice &choice) {
        if (&tile != source || src != east) {
          return false;
        }
        choice = {local, -1, -1};
        return true;
      });
  require(!self_blocked.success,
          "live partial-trunk lease did not block its own replacement route");

  // Check: an occupied edge inside the reverse component is reported as one
  // exact numeric boundary so Moving Sources can cut only its transit owner.
  resetGrid(4, 1, cb);
  source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *blocked_tile = fpga::Device::current().getTile(2, 0);
  target = fpga::Device::current().getTile(3, 0);
  require(source && blocked_tile && target,
          "blocked reverse-frontier grid is incomplete");
  blocked_tile->cb.dst.jump.setBit(0);
  pnr::BackwardTakeoffRoute blocked_frontier =
      pnr::routeBackwardToTakeoff(
          *target, bit(pin), source->coord, 8, 4, -1,
          [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
            return false;
          });
  require(!blocked_frontier.success &&
              blocked_frontier.blocked_reverse_edges == 1 &&
              blocked_frontier.blocked_reverse_frontier.size() == 1,
          "backward routing lost its occupied internal boundary");
  const pnr::DockingBridgeBlocker &blocked_edge =
      blocked_frontier.blocked_reverse_frontier.front();
  require(blocked_edge.valid &&
              blocked_edge.tile.x == blocked_tile->coord.x &&
              blocked_edge.tile.y == blocked_tile->coord.y &&
              blocked_edge.dst == 0 && blocked_edge.src == east &&
              blocked_edge.landing_tile.x == target->coord.x &&
              blocked_edge.landing_tile.y == target->coord.y &&
              blocked_edge.landing_dst == 0 && blocked_edge.dst_busy &&
              !blocked_edge.src_busy && blocked_tile->cb.dst.jump.testBit(0),
          "backward routing reported the wrong occupied numeric edge");

  // Check: a caller-confirmed current placement is accepted during reverse
  // traversal, before the ordinary relocation probe phase is entered.
  resetGrid(4, 1, cb);
  source = fpga::Device::current().getTile(0, 0);
  target = fpga::Device::current().getTile(3, 0);
  size_t ordinary_probes = 0;
  size_t preferred_probes = 0;
  pnr::BackwardTakeoffRoute preferred_takeoff = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, -1,
      [&](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) {
        ++ordinary_probes;
        return false;
      },
      nullptr, {}, 0, 0, 8, nullptr, nullptr,
      [&](fpga::Tile &tile, int src,
          pnr::BackwardTakeoffChoice &choice) {
        ++preferred_probes;
        if (&tile != source || src != east) {
          return false;
        }
        choice = {local, -1, -1};
        return true;
      });
  require(preferred_takeoff.success && preferred_takeoff.source_tile == source &&
              preferred_probes != 0 && ordinary_probes == 0,
          "backward routing exhausted relocation probes before accepting the "
          "current placement");

  // Clear the exclusively owned source bit only in the speculative copy. The
  // reverse route must succeed without releasing the committed live takeoff.
  resetGrid(4, 1, cb);
  source = fpga::Device::current().getTile(0, 0);
  target = fpga::Device::current().getTile(3, 0);
  require(source && target,
          "owned-prefix replacement grid is incomplete");
  source->cb.src.jump |= bit(east);
  pnr::BackwardTakeoffStateView replacement_view;
  replacement_view.emplace(source, source->cb);
  replacement_view.at(source).src.jump &= ~bit(east);
  pnr::BackwardTakeoffRoute replaceable = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 4, -1,
      [&](fpga::Tile &tile, int src, pnr::BackwardTakeoffChoice &choice) {
        if (&tile != source || src != east) {
          return false;
        }
        choice = {local, -1, -1};
        return true;
      },
      nullptr, {}, std::numeric_limits<size_t>::max(), 0,
      std::numeric_limits<size_t>::max(), &replacement_view);
  require(replaceable.success && replaceable.source_tile == source &&
              (source->cb.src.jump & bit(east)) != NodeMask{},
          "ownership-aware replacement changed live state or stayed blocked");

  // Check: reverse routing first docks to the exact live landing of a partial
  // trunk. It emits only a forked suffix and leaves the prefix lease intact.
  resetGrid(4, 1, cb);
  fpga::Tile *anchor_tile = fpga::Device::current().getTile(1, 0);
  target = fpga::Device::current().getTile(3, 0);
  require(anchor_tile && target, "partial-anchor test grid is incomplete");
  anchor_tile->cb.dst.jump |= bit(0);
  pnr::BackwardRouteAnchor anchor{anchor_tile, 0, "DST"};
  pnr::BackwardTakeoffRoute anchored = pnr::routeBackwardToAnchor(
      *target, bit(pin), anchor, 8, 4);
  require(anchored.success && anchored.completed_from_anchor &&
              anchored.source_tile == anchor_tile &&
              anchored.fragments.size() == 4 &&
              anchored.fragments.front().from.x == 1 &&
              anchored.fragments.front().to.x == 2 &&
              anchored.fragments.front().local == 0 &&
              anchored.fragments.front().pos == 2 &&
              !anchored.fragments.front().owns_dst &&
              anchored.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
          "reverse routing did not create the exact partial-trunk suffix");
  require(anchor_tile->cb.dst.jump.testBit(0) &&
              anchor_tile->cb.src.jump == NodeMask{} &&
              target->cb.dst.jump == NodeMask{},
          "speculative partial-trunk completion changed live leases");

  // Check: one reverse traversal skips an unreachable newest landing and
  // docks to the next retained prefix node without probing placement.
  fpga::Tile *dead_tail = fpga::Device::current().getTile(2, 0);
  require(dead_tail, "multi-anchor dead-tail tile is missing");
  std::vector<pnr::BackwardRouteAnchor> anchors{
      {dead_tail, 99, "DEAD_DST", 7}, {anchor_tile, 0, "DST", 5}};
  anchored = pnr::routeBackwardToAnchors(*target, bit(pin), anchors, 8, 4);
  require(anchored.success && anchored.completed_from_anchor &&
              anchored.anchor_id == 5 && anchored.source_tile == anchor_tile &&
              anchored.anchor_candidates == 1,
          "reverse routing did not fall back from a dead tail to its live "
          "prefix landing through the indexed numeric anchor");
}

void backward_route_seeds_only_physical_terminal_arrivals() {
  constexpr int pin = 20;
  constexpr int physical_dst = 0;
  constexpr int subtype_only_dst = 2;
  fpga::CBType cb = makeLinearDockingCrossbar();
  cb.rememberNodeName(fpga::CB_NODE_DST, subtype_only_dst,
                      "SUBTYPE_ONLY_DST");
  cb.dst_local[subtype_only_dst].local |= bit(pin);
  rememberConn(cb, fpga::CB_NODE_DST, subtype_only_dst,
               fpga::CB_NODE_LOCAL, pin);
  cb.rebuildOutgoingSrcs();
  resetGrid(2, 1, cb);

  fpga::Tile *source = fpga::Device::current().getTile(0, 0);
  fpga::Tile *target = fpga::Device::current().getTile(1, 0);
  require(source && target,
          "physical-terminal filtering grid is incomplete");
  target->incoming_dst_nodes = bit(physical_dst);
  target->cb.dst.jump.setBit(physical_dst);

  pnr::BackwardTakeoffRoute route = pnr::routeBackwardToTakeoff(
      *target, bit(pin), source->coord, 8, 2, -1,
      [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; });

  // The free subtype-only DST is not an arrival at this coordinate. Reverse
  // routing must not seed it after the sole physical terminal is occupied.
  require(!route.success && route.expanded == 0 && route.incoming_edges == 0,
          "backward routing seeded a nonphysical terminal destination");
}

void backward_anchor_search_does_not_starve_free_terminal_entries() {
  constexpr int pin = 20;
  const int long_src = encodedJump(1, 0, 1);
  fpga::CBType cb = makeLinearDockingCrossbar();
  cb.rememberNodeName(fpga::CB_NODE_DST, 1, "LONG_ENTRY");
  cb.rememberNodeName(fpga::CB_NODE_SRC, long_src, "LONG_EXIT");
  cb.dst_src[1].jump |= bit(long_src);
  cb.dst_local[1].local |= bit(pin);
  rememberJumpTarget(cb, long_src, 1, {1, 0});
  cb.rebuildOutgoingSrcs();
  resetGrid(12, 1, cb);
  auto *anchor = fpga::Device::current().getTile(10, 0);
  auto *target = fpga::Device::current().getTile(11, 0);
  anchor->cb.dst.jump.setBit(0);
  std::vector<pnr::BackwardRouteAnchor> anchors{{anchor, 0, "ENTRY", 7}};

  // Both target entries are free. Entry 1 leads down a long lane that cannot
  // join the anchor; entry 0 joins it in one hop. DFS used to exhaust lane 1
  // first, starving the usable entry even though it was already discovered.
  auto route = pnr::routeBackwardToAnchors(*target, bit(pin), anchors,
                                          0, 12, nullptr, {}, 2);
  require(route.success && route.anchor_id == 7 && route.expanded <= 2,
          "a long reverse branch starved a free one-hop terminal entry");
  // The accepted suffix is short, forward-ordered, and speculative: the
  // retained trunk lease is unchanged and no new SRC/DST has been leased.
  require(route.fragments.size() == 3 &&
              route.fragments.front().from == anchor->coord &&
              route.fragments.front().to == target->coord &&
              anchor->cb.dst.jump == bit(0) &&
              anchor->cb.src.jump == NodeMask{} &&
              target->cb.dst.jump == NodeMask{},
          "backward entry scheduling damaged the suffix or live leases");
}

void moving_input_proof_reuses_private_prefix_after_sink_move() {
  constexpr int pin = 20;
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(5, 1, cb);
  auto *anchor_tile = fpga::Device::current().getTile(2, 0);
  auto *target_tile = fpga::Device::current().getTile(4, 0);
  std::vector<fpga::Wire> old_route(4);
  old_route[0].type = fpga::Wire::WIRE_TILE_PIN;
  old_route[0].from = old_route[0].to = {1, 0};
  old_route[1].from = {1, 0};
  old_route[1].to = {2, 0};
  old_route[1].jump = encodedJump(1, 0);
  old_route[1].dst = 0;
  old_route[2].from = old_route[2].to = {2, 0};
  old_route[2].local = 0;
  old_route[2].dst = pin;
  old_route[3].type = fpga::Wire::WIRE_TILE_PIN;
  old_route[3].from = old_route[3].to = {2, 0};
  old_route[3].local = pin;

  // The old input has no sibling. Its driver takeoff stays occupied, so only
  // its own surviving fabric prefix can supply the new input suffix.
  anchor_tile->cb.dst.jump.setBit(0);
  std::vector<pnr::BackwardRouteAnchor> anchors;
  size_t prefix = fpga::retainedRoutePrefixSize(old_route, false, true);
  for (size_t i = 0; i < prefix; ++i) {
    const auto &wire = old_route[i];
    if (wire.type == fpga::Wire::WIRE_CROSSBAR && wire.dst >= 0) {
      anchors.push_back({fpga::Device::current().getTile(wire.to.x, wire.to.y),
                         wire.dst, {}, i + 1});
    }
  }
  require(anchors.size() == 1 && anchors[0].dst == 0,
          "input anchors lost the private prefix or included the old terminal");
  auto reject_second_takeoff = [](fpga::Tile &, int,
                                  pnr::BackwardTakeoffChoice &) { return false; };
  auto proof = pnr::routeBackwardToInput(*target_tile, bit(pin), {1, 0}, 5,
                                        reject_second_takeoff, nullptr, {},
                                        nullptr, &anchors);
  // Check: a complete suffix is found without allocating a second takeoff or
  // changing the retained landing and the candidate's live resource masks.
  require(proof.success && proof.completed_from_anchor && proof.anchor_id == 2 &&
              proof.fragments.front().from == anchor_tile->coord &&
              proof.fragments.back().local == pin &&
              anchor_tile->cb.dst.jump == bit(0) &&
              anchor_tile->cb.src.jump == NodeMask{} &&
              target_tile->cb.dst.jump == NodeMask{},
          "moving input proof failed to reuse the sole surviving prefix");
}

void moving_input_proof_does_not_mistake_expansion_limit_for_no_path() {
  constexpr int pin = 20;
  constexpr int width = 4102;
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(width, 1, cb);
  auto *anchor = fpga::Device::current().getTile(0, 0);
  auto *target = fpga::Device::current().getTile(width - 1, 0);
  anchor->cb.dst.jump.setBit(0);
  std::vector<pnr::BackwardRouteAnchor> anchors{{anchor, 0, "ENTRY", 1}};
  // Reproduce the previous cutoff: the only path is valid, but its retained
  // anchor is beyond expansion 4096. A bounded miss is not exhausted topology.
  auto bounded = pnr::routeBackwardToAnchors(*target, bit(pin), anchors,
                                             0, width, nullptr, {}, 4096);
  require(!bounded.success && bounded.expansion_limit_reached &&
              bounded.remaining_frontier > 0,
          "input budget regression did not reproduce the unfinished search");
  auto proof = pnr::routeBackwardToInput(*target, bit(pin), anchor->coord,
                                        width, {}, nullptr, {}, nullptr, &anchors);
  require(proof.success && proof.completed_from_anchor && proof.expanded > 4096 &&
              !proof.expansion_limit_reached,
          "input validation still rejects a reachable driver at a fixed cutoff");
  // Check: removing the local cutoff does not remove the stage's cancellation.
  size_t polls = 0;
  auto cancelled = pnr::routeBackwardToInput(
      *target, bit(pin), anchor->coord, width, {}, nullptr,
      [&]() { return ++polls > 3; }, nullptr, &anchors);
  require(!cancelled.success && cancelled.expanded <= 3 &&
              anchor->cb.dst.jump == bit(0) && target->cb.dst.jump == NodeMask{},
          "input proof ignored cancellation or modified live leases");
}

void moving_input_proof_keeps_its_incomplete_owned_prefix() {
  constexpr int pin = 20;
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(5, 1, cb);
  auto *driver = fpga::Device::current().getTile(1, 0);
  auto *anchor = fpga::Device::current().getTile(2, 0);
  auto *target = fpga::Device::current().getTile(4, 0);
  const int src = encodedJump(1, 0);
  std::vector<fpga::Wire> partial(2);
  partial[0].type = fpga::Wire::WIRE_TILE_PIN;
  partial[0].from = partial[0].to = driver->coord;
  partial[1].from = driver->coord;
  partial[1].to = anchor->coord;
  partial[1].jump = src;
  partial[1].dst = 0;
  driver->cb.src.jump.setBit(src);
  anchor->cb.dst.jump.setBit(0);

  // Reproduce source-moving input validation: this input's takeoff and
  // landing are leased, but the route never reached its old destination.
  std::vector<pnr::BackwardRouteAnchor> anchors;
  if (pnr::movingInputMayReuseRoute(true, false, false, true)) {
    const size_t keep = fpga::retainedRoutePrefixSize(partial, false, true);
    require(keep == partial.size(), "Moving truncated an unfinished input prefix");
    anchors.push_back({anchor, 0, "ENTRY", keep});
  }
  size_t takeoff_probes = 0;
  auto proof = pnr::routeBackwardToInput(*target, bit(pin), driver->coord, 5,
      [&](fpga::Tile &tile, int, pnr::BackwardTakeoffChoice &) {
        if (&tile == driver) {
          ++takeoff_probes;
        }
        return false;
      }, nullptr, {}, nullptr, &anchors);

  // The production eligibility rule must expose the owned partial landing;
  // otherwise backward search needlessly requests a second occupied takeoff.
  require(proof.success && proof.completed_from_anchor && proof.anchor_id == 2 &&
              takeoff_probes == 0 && proof.fragments.front().from == anchor->coord &&
              proof.fragments.back().local == pin,
          "Moving ignored its incomplete input prefix and requested a second takeoff");
  // The proof is speculative: both old leases survive and no suffix is leased.
  require(driver->cb.src.jump == bit(src) && anchor->cb.dst.jump == bit(0) &&
              anchor->cb.src.jump == NodeMask{} && target->cb.dst.jump == NodeMask{},
          "Moving input proof changed live ownership");
  // Do not extend this exception to incomplete siblings or moving drivers.
  require(!pnr::movingInputMayReuseRoute(false, false, false, false) &&
              !pnr::movingInputMayReuseRoute(true, false, true, true) &&
              !pnr::movingInputMayReuseRoute(false, true, false, true) &&
              pnr::movingInputMayReuseRoute(false, true, false, false),
          "Moving input anchors admitted an invalidated or unfinished sibling route");
}

void backward_failure_reports_last_congestion_with_connected_path() {
  auto cb = makeLinearDockingCrossbar();
  resetGrid(5, 1, cb);
  auto &device = fpga::Device::current();
  auto *blocked = device.getTile(1, 0);
  auto *target = device.getTile(4, 0);
  const int src = encodedJump(1, 0);
  blocked->cb.src.jump.setBit(src);
  auto result = pnr::routeBackwardToTakeoff(
      *target, bit(20), {0, 0}, 0, 5, 0,
      [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; });
  // The blocked predecessor is the congestion tile, not the free frontier
  // where reverse expansion stopped. Include that unleased attempted hop.
  require(!result.success && result.failure_tile == blocked &&
              result.failure_dst == 0 && !result.diagnostic_fragments.empty() &&
              result.diagnostic_fragments.front().from == blocked->coord &&
              result.diagnostic_fragments.front().jump == src &&
              result.diagnostic_fragments.back().from == target->coord,
          "backward failure lost its last congestion or connected suffix");
  // Failure diagnostics must not reserve speculative hops or clear blockers.
  for (auto &tile : device.tile_grid) {
    require(tile.cb.src.jump == (&tile == blocked ? bit(src) : NodeMask{}) &&
                tile.cb.dst.jump == NodeMask{},
            "backward failure visualization modified live leases");
  }
}

void moving_source_backward_docking_preserves_partial_prefix() {
  constexpr int dst = 0;
  constexpr int pin = 20;
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(5, 1, cb);

  fpga::Tile *anchor_tile = fpga::Device::current().getTile(1, 0);
  fpga::Tile *target_tile = fpga::Device::current().getTile(4, 0);
  require(anchor_tile && target_tile,
          "Moving Sources backward-docking grid is incomplete");

  // Emulate the useful landing owned by an incomplete Generic trunk. Moving
  // Sources must dock its destination-side search here without releasing it.
  anchor_tile->cb.dst.jump |= bit(dst);
  std::vector<pnr::BackwardRouteAnchor> anchors{
      {anchor_tile, dst, "ANCHOR_DST", 3}};
  pnr::BackwardTakeoffRoute suffix = pnr::routeBackwardToAnchors(
      *target_tile, bit(pin), anchors, 8, 5);

  // Check: the reverse route joins the retained prefix and emits only the
  // forward-ordered suffix from that anchor to the destination pin.
  require(suffix.success && suffix.completed_from_anchor &&
              suffix.anchor_id == 3 && suffix.source_tile == anchor_tile &&
              !suffix.fragments.empty() &&
              suffix.fragments.front().from == anchor_tile->coord &&
              suffix.fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
              suffix.fragments.back().local == pin,
          "Moving Sources failed to dock backward to its Generic prefix");

  // Check: speculative backward docking does not release or duplicate the
  // live anchor lease before the suffix is committed by RouteDesign.
  require(anchor_tile->cb.dst.jump.testBit(dst) &&
              anchor_tile->cb.src.jump == NodeMask{} &&
              target_tile->cb.dst.jump == NodeMask{},
          "Moving Sources backward docking changed live prefix leases");
}

void combinatorial_search_allows_two_tile_visits_and_rejects_third() {
  constexpr int pin = 20;
  fpga::CBType cb = makeRevisitingBackwardCrossbar();
  resetGrid(2, 1, cb);
  fpga::Tile *left = fpga::Device::current().getTile(0, 0);
  fpga::Tile *right = fpga::Device::current().getTile(1, 0);
  require(left && right, "combinatorial revisit grid is incomplete");

  // The reverse chain is right -> left -> right -> left -> right, with a
  // different destination node on every arrival. Two visits are legal; the
  // third right-tile visit must be rejected even though its node is unseen.
  pnr::BackwardTakeoffRoute route = pnr::routeBackwardToTakeoff(
      *right, bit(pin), left->coord, 0, 2, -1,
      [](fpga::Tile &, int, pnr::BackwardTakeoffChoice &) { return false; },
      nullptr, {}, 0, 0, 64);
  require(!route.success && route.expanded == 4 &&
              route.tile_visit_reject_count == 1 &&
              route.remaining_frontier == 0,
          "backward combinatorial search did not reject exactly the third "
          "visit to one crossbar");

  // A materialized candidate with two visits per tile remains legal.
  std::vector<fpga::Wire> valid(3);
  valid[0].from = left->coord;
  valid[0].to = right->coord;
  valid[1].from = right->coord;
  valid[1].to = left->coord;
  valid[2].from = left->coord;
  valid[2].to = right->coord;
  require(pnr::combinatorialRouteVisitsValid(valid),
          "combinatorial route rejected a legal second crossbar visit");

  // A fourth edge enters the left crossbar for the third time and is invalid.
  fpga::Wire third_left;
  third_left.from = right->coord;
  third_left.to = left->coord;
  valid.push_back(third_left);
  require(!pnr::combinatorialRouteVisitsValid(valid),
          "combinatorial route accepted a third crossbar visit");
}

void docking_counts_retained_prefix(bool iob, bool alternative, int blocked)
{
  auto cb = makeLinearDockingCrossbar();
  if (alternative) {
    // Bypass the exhausted Tile 1: 0 -> 4 -> 3.
    for (auto [src, dx] : {std::pair{encodedJump(4,0), 4},
                           std::pair{encodedJump(-1,0), -1}}) {
      cb.rememberNodeName(fpga::CB_NODE_SRC, src, "node_" + std::to_string(src));
      cb.dst_src[0].jump.setBit(src);
      rememberJumpTarget(cb, src, 0, {dx,0});
      rememberConn(cb, fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC, src);
    }
    cb.rebuildOutgoingSrcs();
  }
  auto tiles = resetGrid(5, 1, cb);
  auto &anchor = *tiles[0], &target = *tiles[3];
  anchor.cb.dst.jump.setBit(0);
  if (blocked == 1) tiles[1]->cb.src.jump.setBit(encodedJump(1,0));
  if (blocked == 2) target.cb.dst.jump.setBit(0);
  std::vector<fpga::CBState> before;
  for (auto *tile : tiles) before.push_back(tile->cb);
  const std::vector<fpga::Coord> history{{1,0}, {2,0}, {1,0}, {0,0}, {0,0}};
  auto run = [&](std::span<const fpga::Coord> prefix) {
    return iob ? pnr::dockIOB(anchor, 0, "", target, bit(20), false, {}, prefix)
               : pnr::dockGrounding(anchor, 0, "", target, bit(20), 5, 5, false, {}, nullptr, prefix);
  };
  const auto standalone = run({});
  require(standalone.success || standalone.blocked_terminal_reachable ||
              std::any_of(standalone.blocked_bridges.begin(), standalone.blocked_bridges.end(),
                  [](const auto &bridge) { return bridge.joins_frontiers; }),
          "fixture did not expose a standalone docking path/preemption opportunity");
  const auto result = run(history);
  require(result.success == (alternative && blocked != 2),
          "Docking ignored retained visits or missed the legal bypass");
  if (result.success) {
    std::vector<fpga::Wire> joined;
    for (size_t i = 1; i < history.size(); ++i) {
      fpga::Wire wire; wire.from = history[i-1]; wire.to = history[i]; joined.push_back(wire);
    }
    joined.insert(joined.end(), result.fragments.begin(), result.fragments.end());
    require(pnr::combinatorialRouteVisitsValid(joined), "Docking join exceeds combined Tile visits");
  } else if (!alternative) {
    require(result.tile_visit_reject_count > 0 && !result.blocked_terminal_reachable &&
                std::none_of(result.blocked_bridges.begin(), result.blocked_bridges.end(),
                    [](const auto &bridge) { return bridge.joins_frontiers; }),
            "an over-limit route was offered for terminal/bridge preemption");
  }
  for (size_t i = 0; i < tiles.size(); ++i)
    require(tiles[i]->cb.src.jump == before[i].src.jump &&
                tiles[i]->cb.dst.jump == before[i].dst.jump &&
                tiles[i]->cb.local.local == before[i].local.local &&
                tiles[i]->cb.joint.jump == before[i].joint.jump,
            "visit validation changed live leases");
}

void docking_join_counts_combined_history(int blocked)
{
  fpga::CBType cb;
  cb.name = "matrix_173"; cb.type_id = 0;
  auto edge = [&](int dst, int src, int arrival, int dx) {
    for (int id : {dst, arrival})
      cb.rememberNodeName(fpga::CB_NODE_DST, id, "node_" + std::to_string(id));
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "node_" + std::to_string(src));
    cb.dst_src[dst].jump.setBit(src);
    rememberJumpTarget(cb, src, arrival, {dx,0});
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
  };
  // Standalone Docking may visit Tile 1 twice, once on either side of the
  // meeting. Together with the retained prefix's visit, the join is invalid.
  edge(10, 17, 11, 1); // 0 -> 1
  edge(11, 23, 12, 1); // 1 -> 2
  edge(12, 29, 13, -1); // 2 -> 1 (different DST)
  edge(13, 31, 14, 2); // 1 -> 3
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 50, "pin_179");
  cb.dst_local[14].local.setBit(50);
  rememberConn(cb, fpga::CB_NODE_DST, 14, fpga::CB_NODE_LOCAL, 50);
  cb.rebuildOutgoingSrcs();
  auto tiles = resetGrid(4, 1, cb);
  tiles[0]->cb.dst.jump.setBit(10);
  if (blocked == 1) tiles[2]->cb.src.jump.setBit(29);
  if (blocked == 2) tiles[3]->cb.dst.jump.setBit(14);
  const std::vector<fpga::Coord> history{{1,0}, {0,0}};
  const auto baseline = pnr::dockGrounding(*tiles[0], 10, "", *tiles[3], bit(50));
  require(baseline.success || baseline.blocked_terminal_reachable ||
              std::any_of(baseline.blocked_bridges.begin(), baseline.blocked_bridges.end(),
                  [](const auto &b) { return b.joins_frontiers; }),
          "combined-history fixture has no standalone route/preemption proof");
  const auto result = pnr::dockGrounding(*tiles[0], 10, "", *tiles[3], bit(50),
                                        5, 5, false, {}, nullptr, history);
  require(!result.success && result.tile_visit_reject_count > 0 &&
              !result.blocked_terminal_reachable &&
              std::none_of(result.blocked_bridges.begin(), result.blocked_bridges.end(),
                  [](const auto &b) { return b.joins_frontiers; }),
          "Docking accepted individually valid frontiers whose combined path exceeds visits");
  require(tiles[2]->cb.src.jump.testBit(29) == (blocked == 1) &&
              tiles[3]->cb.dst.jump.testBit(14) == (blocked == 2),
          "invalid combined history changed a prospective victim's lease");
}

void backward_anchor_chooses_short_route(unsigned seed, int rotation,
                                         bool block_short_route) {
  auto &device = fpga::Device::current();
  device.tile_grid.clear();
  device.cb_types.clear();
  device.grid_spec.size = {5, 5};
  device.size_width = device.size_height = 5;
  device.tile_grid.resize(25);
  std::mt19937 rng(seed);
  constexpr int dst = 17, pin = 23;
  auto rotate = [&](fpga::Coord c) {
    for (int i = 0; i < rotation; ++i) c = {4 - c.y, c.x};
    return c;
  };
  for (int i = 0; i < 25; ++i) {
    auto &cb = device.cb_types.emplace_back();
    cb.name = "matrix_" + std::to_string(rng());
    cb.type_id = cb.base_type_id = i;
    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "node_" + std::to_string(rng()));
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "node_" + std::to_string(rng()));
  }
  for (int i = 0; i < 25; ++i) {
    auto &tile = device.tile_grid[i];
    tile.coord = {i % 5, i / 5};
    tile.cb_coord = tile.name = tile.coord;
    tile.cb_type = &device.cb_types[i];
    tile.cb.type = tile.cb_type;
  }
  auto tile_at = [&](fpga::Coord c) {
    c = rotate(c);
    return device.getTile(c.x, c.y);
  };
  int next_src = 100;
  auto edge = [&](fpga::Coord from, fpga::Coord to) {
    auto *a = tile_at(from), *b = tile_at(to);
    auto &cb = *a->cb_type;
    int src = next_src++;
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "node_" + std::to_string(rng()));
    cb.dst_src[dst].jump |= bit(src);
    rememberJumpTarget(cb, src, dst, b->coord - a->coord, b->cb_type->type_id);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
  };
  // Backward traversal sees the long corridor in the anchor's preferred
  // direction first. A different incoming edge reaches the same anchor in
  // three hops. Both corridors are genuinely connected and legally leasable.
  std::vector<fpga::Coord> long_path{
      {0,2}, {0,3}, {0,4}, {1,4}, {2,4}, {2,3}, {2,2}, {3,2}, {4,2}};
  for (size_t i = 1; i < long_path.size(); ++i) edge(long_path[i-1], long_path[i]);
  edge({0,2}, {3,1});
  edge({3,1}, {3,2});
  auto *anchor = tile_at({0,2}), *target = tile_at({4,2});
  target->cb_type->dst_local[dst].local |= bit(pin);
  rememberConn(*target->cb_type, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
  for (auto &cb : device.cb_types) cb.rebuildOutgoingSrcs();
  anchor->cb.dst.jump |= bit(dst);
  if (block_short_route) tile_at({3,1})->cb.dst.jump |= bit(dst);
  pnr::BackwardRouteAnchor root{anchor, dst, {}, 1};
  auto limited = pnr::routeBackwardToAnchor(*target, bit(pin), root, 0, 6,
                                           nullptr, {}, 1);
  require(!limited.success && limited.expansion_limit_reached &&
              limited.expanded == 1 && limited.remaining_frontier == 1,
          "breadth-first anchor search lost its expansion limit or frontier count");
  auto result = pnr::routeBackwardToAnchor(*target, bit(pin), root, 0, 6);
  require(result.success && result.completed_from_anchor,
          "reverse anchor search failed with a known free corridor");
  require(result.fragments.size() == (block_short_route ? 10u : 5u),
          "reverse anchor search committed the long detour before the short free corridor");
  for (const auto &tile : device.tile_grid) {
    require(tile.cb.src.jump == NodeMask{} && tile.cb.joint.jump == NodeMask{} &&
                tile.cb.local.local == NodeMask{} &&
                tile.pin_state.leased_nodes == NodeMask{},
            "speculative anchor search modified live resource leases");
    NodeMask expected = &tile == anchor || (block_short_route && &tile == tile_at({3,1}))
                            ? bit(dst) : NodeMask{};
    require(tile.cb.dst.jump == expected, "anchor search changed a live destination lease");
  }
}

void docking_preserves_intra_cb_continuity(int local_hops) {
  fpga::CBType cb;
  cb.name = "RELAY_MATRIX";
  cb.type_id = 0;
  const int enter_src = encodedJump(1, 0, 0);
  const int exit_src = encodedJump(1, 0, 1);
  const int terminal_dst = local_hops + 2;
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 20, "sink_q");
  for (int dst = 0; dst <= terminal_dst; ++dst)
    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "arrival_" + std::to_string(dst));
  cb.rememberNodeName(fpga::CB_NODE_SRC, enter_src, "launch_q");
  cb.rememberNodeName(fpga::CB_NODE_SRC, exit_src, "launch_r");
  cb.dst_src[0].jump = bit(enter_src);
  rememberJumpTarget(cb, enter_src, 1, {1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC, enter_src);
  for (int i = 0; i < local_hops; ++i) {
    int src = 16 + i;
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "relay_" + std::to_string(i));
    cb.rememberNodeName(fpga::CB_NODE_DST, i + 2, "relay_" + std::to_string(i));
    cb.dst_src[i + 1].jump = bit(src);
    rememberJumpTarget(cb, src, i + 2, {0, 0});
    rememberConn(cb, fpga::CB_NODE_DST, i + 1, fpga::CB_NODE_SRC, src);
  }
  cb.dst_src[local_hops + 1].jump = bit(exit_src);
  rememberJumpTarget(cb, exit_src, terminal_dst, {1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, local_hops + 1, fpga::CB_NODE_SRC, exit_src);
  cb.rememberNodeName(fpga::CB_NODE_JOINT, 11, "joint_q");
  cb.rememberNodeName(fpga::CB_NODE_JOINT, 12, "joint_r");
  cb.dst_joint[terminal_dst].joint = bit(11);
  cb.joint_joint[11].joint = bit(12);
  cb.joint_local[12].local = bit(20);
  rememberConn(cb, fpga::CB_NODE_DST, terminal_dst, fpga::CB_NODE_JOINT, 11);
  rememberConn(cb, fpga::CB_NODE_JOINT, 11, fpga::CB_NODE_JOINT, 12);
  rememberConn(cb, fpga::CB_NODE_JOINT, 12, fpga::CB_NODE_LOCAL, 20);
  cb.rebuildOutgoingSrcs();
  resetGrid(3, 1, cb);
  auto &device = fpga::Device::current();
  auto &start = *device.getTile(0, 0);
  auto &middle = *device.getTile(1, 0);
  auto &target = *device.getTile(2, 0);
  auto index = pnr::buildBackwardResolveIndex(device, target.coord, 5);
  auto incoming = pnr::resolveBackwardSources(index, {1, 0, 2});
  require(incoming && incoming->size() == 1 && incoming->front().tile == &middle &&
              incoming->front().src == 16,
          "reverse index lost the intra-CB continuation");
  auto forward = pnr::dockGrounding(start, 0, "arrival_0", target, bit(20), 12, 5);
  require(forward.success && forward.fragments.size() == size_t(local_hops + 4),
          "forward docking lost an intra-CB continuation");
  pnr::BackwardRouteAnchor anchor{&start, 0, "arrival_0", 1};
  auto reverse = pnr::routeBackwardToAnchor(target, bit(20), anchor, 12, 5);
  require(reverse.success && reverse.fragments.size() == forward.fragments.size(),
          "reverse docking discarded a zero-direction intra-CB continuation");
  require(reverse.fragments[reverse.fragments.size() - 2].joint == 12 &&
              reverse.fragments[reverse.fragments.size() - 2].joint2 == 11,
          "continuity route lost its two terminal joints");
  // Either occupied role must prevent a competing route through this wire.
  for (int role = 0; role < 2; ++role) {
    middle.cb = {};
    if (role == 0) middle.cb.src.jump = bit(16);
    else middle.cb.dst.jump = bit(2);
    auto blocked = pnr::routeBackwardToAnchor(target, bit(20), anchor, 12, 5);
    require(!blocked.success, "continuity search ignored an occupied wire role");
    require(middle.cb.src.jump == (role == 0 ? bit(16) : NodeMask{}) &&
                middle.cb.dst.jump == (role == 1 ? bit(2) : NodeMask{}),
            "speculative continuity search changed occupancy");
  }
}

// A local destination can be fed only through a long wire whose source is
// outside the docking window. Geometry must not erase that incoming edge.
// Exercise both a terminal-crossing wire and an outside predecessor one hop
// earlier, in all directions, with the production shared index and fallback.
void docking_follows_incoming_wires_outside_window(int rotation, bool relay,
                                                  bool shared, bool busy_local_terminal) {
  auto rotate = [rotation](fpga::Coord p) {
    for (int i = 0; i < rotation; ++i) p = {-p.y, p.x};
    return p;
  };
  auto coord = [&](int dx) {
    auto p = rotate({dx, 0});
    return fpga::Coord{12 + p.x, 12 + p.y};
  };
  fpga::CBType cb;
  cb.name = "BOUNDARY_MATRIX";
  cb.type_id = 0;
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 20, "terminal_q");
  for (int dst = 0; dst < 4; ++dst)
    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "arrival_" + std::to_string(dst));
  auto link = [&](int dst, int src, int next_dst, int dx) {
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "launch_" + std::to_string(src));
    cb.dst_src[dst].jump = bit(src);
    rememberJumpTarget(cb, src, next_dst, rotate({dx, 0}));
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
  };
  link(0, 30, 1, -6); // root (-1) -> outside (-7)
  link(1, 31, relay ? 2 : 3, relay ? 8 : 7);
  if (relay) link(2, 32, 3, -1); // relay (+1) -> terminal (0)
  cb.dst_local[3].local = bit(20);
  rememberConn(cb, fpga::CB_NODE_DST, 3, fpga::CB_NODE_LOCAL, 20);
  if (busy_local_terminal) {
    cb.rememberNodeName(fpga::CB_NODE_DST, 4, "occupied_local_entry");
    cb.rememberNodeName(fpga::CB_NODE_SRC, 33, "local_exit");
    cb.dst_src[0].jump |= bit(33);
    rememberJumpTarget(cb, 33, 4, rotate({1, 0}));
    rememberConn(cb, fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC, 33);
    cb.dst_local[4].local = bit(20);
    rememberConn(cb, fpga::CB_NODE_DST, 4, fpga::CB_NODE_LOCAL, 20);
  }
  cb.rebuildOutgoingSrcs();
  resetGrid(25, 25, cb);
  auto &device = fpga::Device::current();
  auto at = [&](int dx) -> fpga::Tile & {
    const auto p = coord(dx);
    return *device.getTile(p.x, p.y);
  };
  auto &root = at(-1);
  auto &outside = at(-7);
  auto &target = at(0);
  root.cb.dst.jump = bit(0); // reuse an existing routed arrival
  if (busy_local_terminal) target.cb.dst.jump = bit(4);
  pnr::BackwardResolveCache cache;
  pnr::BackwardResolveIndex index;
  if (shared) {
    pnr::buildBackwardResolveIndex(device, {12, 12}, 25, {}, &cache);
    index.center = target.coord;
    index.radius = 5;
    index.shared_cache = &cache;
    // A previous bounded lookup must not hide topology from docking later.
    auto p = relay ? at(1).coord : target.coord;
    require(!pnr::resolveBackwardSources(index, {p.x, p.y, relay ? 2 : 3}),
            "bounded control unexpectedly includes the outside source");
  }
  auto run = [&](int depth) {
    return pnr::dockGrounding(root, 0, "arrival_0", target, bit(20), depth,
                              5, true, {}, shared ? &index : nullptr);
  };
  auto result = run(5);
  if (!result.success) {
    std::fprintf(stderr, "boundary rotation=%d relay=%d shared=%d seeds=%d push=%d topology=%d busy=%d\n",
                 rotation, relay, shared, result.target_seed_count,
                 result.backward_push_count, result.backward_topology_reject_count,
                 result.backward_busy_reject_count);
  }
  require(result.success, "docking hid a free incoming wire outside its window");
  require(!shared || result.backward_mapping_scan_count == 0,
          "shared boundary lookup rescanned the device topology");
  require(result.fragments.size() == size_t(relay ? 5 : 4) &&
              result.fragments.front().from == root.coord &&
              result.fragments.front().to == outside.coord,
          "docking did not return the actual boundary-crossing path");
  require(!run(1).success, "boundary crossing bypassed the reverse hop limit");
  outside.cb.src.jump = bit(31);
  require(!run(5).success, "boundary crossing ignored an occupied source");
  require(root.cb.dst.jump == bit(0) && outside.cb.src.jump == bit(31) &&
              target.cb.local.local == NodeMask{} &&
              target.cb.dst.jump == (busy_local_terminal ? bit(4) : NodeMask{}),
          "boundary search mutated live routing leases");
}

// Adding outside predecessors must not erase a free local route by consuming
// its backward beam. Forward search is independently crowded, so only the
// local backward search can find the two-hop completion in this fixture.
void docking_preserves_local_path_when_outside_choices_grow(int outside_count,
                                                           bool local_first) {
  fpga::CBType cb;
  cb.name = "CROWDED_BOUNDARY_MATRIX";
  cb.type_id = 0;
  auto link = [&](int dst, int src, int next, fpga::Coord delta) {
    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "entry_" + std::to_string(dst));
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "exit_" + std::to_string(src));
    cb.dst_src[dst].jump |= bit(src);
    rememberJumpTarget(cb, src, next, delta);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
  };
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 20, "terminal");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 21, "unused_terminal");
  cb.rememberNodeName(fpga::CB_NODE_DST, 80, "terminal_entry");
  cb.dst_local[80].local = bit(20);
  rememberConn(cb, fpga::CB_NODE_DST, 80, fpga::CB_NODE_LOCAL, 20);
  for (int i = 0; i < 32; ++i) {
    cb.rememberNodeName(fpga::CB_NODE_DST, 100 + i, "dead_" + std::to_string(i));
    cb.dst_local[100 + i].local = bit(21);
    rememberConn(cb, fpga::CB_NODE_DST, 100 + i, fpga::CB_NODE_LOCAL, 21);
    link(0, 256 + i, 100 + i, {1, 0});
  }
  link(0, 500, 1, {0, 1});
  link(1, 501, 80, {4, -1});
  cb.rememberNodeName(fpga::CB_NODE_SRC, 502, "outside_exit");
  rememberJumpTarget(cb, 502, 80, {13, 0});
  for (int i = 0; i < outside_count; ++i) {
    cb.rememberNodeName(fpga::CB_NODE_DST, 40 + i, "outside_" + std::to_string(i));
    cb.dst_src[40 + i].jump = bit(502);
    rememberConn(cb, fpga::CB_NODE_DST, 40 + i, fpga::CB_NODE_SRC, 502);
  }
  cb.rebuildOutgoingSrcs();
  resetGrid(25, 25, cb);
  auto &device = fpga::Device::current();
  auto &root = *device.getTile(10, 10);
  auto &middle = *device.getTile(10, 11);
  auto &target = *device.getTile(14, 10);
  auto &outside = *device.getTile(1, 10);
  root.cb.dst.jump = bit(0);
  pnr::BackwardResolveCache cache;
  cache.incoming[{14, 10, 80}] = {{&outside, 502, 502}, {&middle, 501, 501}};
  if (local_first)
    std::reverse(cache.incoming[{14, 10, 80}].begin(), cache.incoming[{14, 10, 80}].end());
  cache.incoming[{10, 11, 1}] = {{&root, 500, 500}};
  pnr::BackwardResolveIndex index;
  index.center = target.coord;
  index.radius = 5;
  index.shared_cache = &cache;
  auto result = pnr::dockGrounding(root, 0, "entry_0", target, bit(20),
                                  5, 5, true, {}, &index);
  require(result.success,
          "outside predecessors consumed the local route's backward beam");
  require(result.fragments.size() == 4 && result.fragments[0].jump == 500 &&
              result.fragments[1].jump == 501,
          "expanded choices displaced the existing two-hop free route");
  require(result.backward_push_count == 1,
          "local success unnecessarily explored the outside dead-end population");
  root.cb.src.jump = bit(500);
  auto blocked = pnr::dockGrounding(root, 0, "entry_0", target, bit(20),
                                   5, 5, true, {}, &index);
  require(!blocked.success && !blocked.blocked_bridges.empty(),
          "widening ignored a lease or lost the local blocked-bridge proof");
  require(root.cb.dst.jump == bit(0) && root.cb.src.jump == bit(500) &&
              outside.cb.src.jump == NodeMask{} && target.cb.local.local == NodeMask{},
          "local/expanded comparison changed live leases");
}

// The only exit leaves the forward window. Reverse search must cover the
// entire corridor, so this checks the actual per-direction hop boundary.
void docking_hop_budget_tracks_radius(int radius, bool beyond_limit) {
  const int hops = 2 * radius + (beyond_limit ? 1 : 0);
  fpga::CBType cb;
  cb.name = "HOP_BUDGET_MATRIX";
  cb.type_id = 0;
  std::vector<fpga::Coord> positions{{radius + 5, 2}};
  for (int i = 1; i < hops; ++i) positions.push_back({1, i + 2});
  positions.push_back({radius + 6, 2});
  for (int i = 0; i <= hops; ++i)
    cb.rememberNodeName(fpga::CB_NODE_DST, i, "entry_" + std::to_string(i));
  for (int i = 0; i < hops; ++i) {
    const int src = 64 + i;
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "exit_" + std::to_string(i));
    cb.dst_src[i].jump = bit(src);
    rememberJumpTarget(cb, src, i + 1,
                       {positions[i + 1].x - positions[i].x,
                        positions[i + 1].y - positions[i].y});
    rememberConn(cb, fpga::CB_NODE_DST, i, fpga::CB_NODE_SRC, src);
  }
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 20, "terminal");
  cb.dst_local[hops].local = bit(20);
  rememberConn(cb, fpga::CB_NODE_DST, hops, fpga::CB_NODE_LOCAL, 20);
  cb.rebuildOutgoingSrcs();
  resetGrid(radius + 8, hops + 4, cb);
  auto &device = fpga::Device::current();
  auto &root = *device.getTile(positions.front().x, positions.front().y);
  auto &target = *device.getTile(positions.back().x, positions.back().y);
  root.cb.dst.jump = bit(0);
  require(!pnr::dockGrounding(root, 0, "entry_0", target, bit(20),
                              radius, radius).success,
          "control unexpectedly completed beyond the old radius-sized depth");
  const auto result = pnr::dockGrounding(root, 0, "entry_0", target, bit(20),
                                        -1, radius);
  require(result.success == !beyond_limit,
          "default docking budget did not enforce twice its radius");
  require(result.forward_push_count == 0,
          "hop-budget fixture did not isolate reverse search");
  if (result.success) {
    require(result.fragments.size() == size_t(hops + 2),
            "docking returned an incomplete corridor");
    auto &middle = *device.getTile(positions[1].x, positions[1].y);
    middle.cb.src.jump = bit(65);
    require(!pnr::dockGrounding(root, 0, "entry_0", target, bit(20),
                                -1, radius).success,
            "larger hop budget ignored an occupied corridor");
  }
  require(root.cb.dst.jump == bit(0) && target.cb.local.local == NodeMask{},
          "hop-budget probe changed terminal leases");
}

// No DST feeds the source's exit: only the requested output LOCAL can
// complete the reverse search. Include a same-numbered leased DST to catch
// accidental LOCAL/DST aliasing and a two-JOINT Takeoff variant.
// A short route through a dense matrix must meet from both ends rather than
// flood the reverse component. Count work, not machine-dependent elapsed time.
void grounding_dense_source_local(int rotation, int width) {
  auto rotate = [&](fpga::Coord delta) {
    for (int i = 0; i < rotation; ++i) delta = {-delta.y, delta.x};
    return delta;
  };
  fpga::CBType cb;
  cb.type_id = 0;
  cb.name = "DENSE_LOCAL_MATRIX";
  constexpr int output = 7, input = 20;
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, output, "launch");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, input, "receive");
  const std::array<fpga::Coord, 4> deltas{{{4, 0}, {0, 4}, {-4, 0}, {0, -4}}};
  for (int i = 0; i < width; ++i) {
    cb.rememberNodeName(fpga::CB_NODE_SRC, i, "exit_" + std::to_string(i));
    cb.rememberNodeName(fpga::CB_NODE_DST, i, "arrival_" + std::to_string(i));
    rememberJumpTarget(cb, i, i, rotate(deltas[i * 4 / width]));
    cb.dst_local[i].local = bit(input);
    rememberConn(cb, fpga::CB_NODE_DST, i, fpga::CB_NODE_LOCAL, input);
  }
  for (int dst = 0; dst < width; ++dst)
    for (int src = 0; src < width; ++src) {
      cb.dst_src[dst].jump.setBit(src);
      rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
    }
  cb.local_src[output].jump = bit(0);
  rememberConn(cb, fpga::CB_NODE_LOCAL, output, fpga::CB_NODE_SRC, 0);
  cb.rebuildOutgoingSrcs();
  auto tiles = resetGrid(13, 13, cb);
  auto &device = fpga::Device::current();
  auto &source = *device.getTile(6, 6);
  auto delta = rotate({4, 4});
  auto &target = *device.getTile(6 + delta.x, 6 + delta.y);
  source.cb.dst.jump = bit(output); // LOCAL and DST namespaces must not alias.
  source.cb.local.local = bit(output); // A sibling already uses this output.
  source.cb.src_deadend.jump = bit(0); // Grounding ignores directional deadends.
  auto result = pnr::dockGrounding(source, output, "launch", target, bit(input),
      -1, 5, false, {}, nullptr, {}, true);
  require(result.success && result.fragments.size() == 4 &&
              result.fragments.front().pos == 0 &&
              result.fragments.front().local == output &&
              result.fragments.front().jump == 0 &&
              result.fragments.back().local == input,
          "dense source-local Docking lost the two-hop route");
  require(result.backward_pop_count <= 8 && result.forward_pop_count <= 8 &&
              result.backward_push_count <= 2 * width,
          "dense source-local Docking flooded the backward component: pops=" +
              std::to_string(result.backward_pop_count) + " pushes=" +
              std::to_string(result.backward_push_count));
  for (auto *tile : tiles)
    require(tile->cb.src.jump == NodeMask{} && tile->cb.joint.jump == NodeMask{} &&
                tile->cb.dst.jump == (tile == &source ? bit(output) : NodeMask{}) &&
                tile->cb.local.local == (tile == &source ? bit(output) : NodeMask{}) &&
                !tile->isPinNodeLeased(input),
            "dense source-local search changed live leases");
}

void grounding_routes_backwards_to_source_local(int rotation, bool joints,
                                                bool reverse_only) {
  auto rotate = [&](fpga::Coord delta) {
    for (int i = 0; i < rotation; ++i) delta = {-delta.y, delta.x};
    return delta;
  };
  fpga::CBType cb;
  cb.type_id = 0;
  cb.name = "REVERSE_LOCAL_MATRIX";
  constexpr int output = 7, input = 20, first_src = 80, second_src = 81;
  constexpr int middle_dst = 10, final_dst = 11;
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, output, "output_7");
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, input, "input_20");
  cb.rememberNodeName(fpga::CB_NODE_SRC, first_src, "exit_80");
  cb.rememberNodeName(fpga::CB_NODE_SRC, second_src, "exit_81");
  cb.rememberNodeName(fpga::CB_NODE_DST, middle_dst, "arrival_10");
  cb.rememberNodeName(fpga::CB_NODE_DST, final_dst, "arrival_11");
  if (joints) {
    cb.local_joint[output].joint = bit(30);
    cb.joint_joint[30].joint = bit(31);
    cb.joint_src[31].jump = bit(first_src);
  } else {
    cb.local_src[output].jump = bit(first_src);
  }
  cb.dst_src[middle_dst].jump = bit(second_src);
  cb.dst_local[final_dst].local = bit(input);
  const int first_length = reverse_only ? 3 : 2;
  rememberJumpTarget(cb, first_src, middle_dst, rotate({first_length, 0}));
  rememberJumpTarget(cb, second_src, final_dst, rotate({3, 0}));
  cb.rebuildOutgoingSrcs();
  resetGrid(13, 13, cb);
  auto &device = fpga::Device::current();
  auto &source = *device.getTile(6, 6);
  auto middle_delta = rotate({first_length, 0});
  auto target_delta = rotate({first_length + 3, 0});
  auto &middle = *device.getTile(6 + middle_delta.x, 6 + middle_delta.y);
  auto &target = *device.getTile(6 + target_delta.x, 6 + target_delta.y);
  source.cb.dst.jump = bit(output);
  source.cb.local.local = bit(output); // Existing sibling's output.
  source.cb.src_deadend.jump = bit(first_src); // Grounding ignores deadends.
  require(!pnr::dockGrounding(source, output, "output_7", target, bit(input)).success,
          "DST-only control unexpectedly reached an output-only source");
  auto run = [&](int requested_output = 7) {
    return pnr::dockGrounding(source, requested_output, "output_7", target,
                              bit(input), -1, 5, true, {}, nullptr, {}, true);
  };
  auto result = run();
  require(result.success, "Grounding did not reach the source LOCAL");
  // Outside the forward window, only backward Grounding can finish this path.
  if (reverse_only)
    require(result.forward_push_count == 0 && result.backward_push_count > 0,
            "Grounding lost backward-only completion to the source LOCAL");
  require(result.fragments.size() == 4 && result.fragments[0].pos == 0 &&
              result.fragments[0].local == output && result.fragments[0].jump == first_src &&
              result.fragments[1].jump == second_src &&
              result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN &&
              result.fragments.back().local == input,
          "reverse Grounding lost Takeoff, transit or terminal fragments");
  if (joints)
    require(result.fragments[0].joint >= 0 && result.fragments[0].joint2 >= 0,
            "reverse source-local Takeoff lost its two JOINTs");
  require(!run(output + 1).success, "Grounding connected an unrelated output LOCAL");
  source.cb.src.jump = bit(first_src);
  require(!run().success, "Grounding ignored an occupied Takeoff SRC");
  source.cb.src.jump = {};
  middle.cb.src.jump = bit(second_src);
  require(!run().success, "Grounding ignored an occupied transit SRC");
  middle.cb.src.jump = {};
  if (joints) {
    source.cb.joint.jump = bit(31);
    require(!run().success, "Grounding ignored an occupied Takeoff JOINT");
    source.cb.joint.jump = {};
  }
  require(source.cb.dst.jump == bit(output) && source.cb.src.jump == NodeMask{} &&
              source.cb.local.local == bit(output) && target.cb.local.local == NodeMask{},
          "reverse Grounding probe changed live leases");
}

void backward_failure_report_explains_incoming_blockers() {
  fpga::CBType cb = makeLinearDockingCrossbar();
  resetGrid(2, 1, cb);
  auto &device = fpga::Device::current();
  auto &source = *device.getTile(0, 0);
  auto &target = *device.getTile(1, 0);
  auto index = pnr::buildBackwardResolveIndex(device, target.coord, 2);
  const int src = encodedJump(1, 0);
  for (int blocker = 0; blocker < 3; ++blocker) {
    source.cb = {};
    if (blocker == 1) source.cb.src.jump = bit(src);
    if (blocker == 2) source.cb.dst.jump = bit(0);
    std::ostringstream out;
    pnr::dumpBackwardTerminalReport(target, bit(20), index, 0, {}, out);
    const auto text = out.str();
    require(text.find("terminal_free=1") != std::string::npos,
            "incoming blocker incorrectly hid the free terminal");
    require(text.find("INCOMING tile=(0,0)") != std::string::npos &&
                text.find("in_docking_window=0") != std::string::npos,
            "failure report hid an incoming source outside the window");
    require(text.find(blocker ? "free_first_hops=0" : "free_first_hops=1") !=
                std::string::npos,
            "failure report misclassified the previous DST/SRC lease");
    require(!blocker || text.find("status=UNOWNED_LEASE") != std::string::npos,
            "failure report failed to verify a blocker with no live owner");
    require(source.cb.src.jump == (blocker == 1 ? bit(src) : NodeMask{}) &&
                source.cb.dst.jump == (blocker == 2 ? bit(0) : NodeMask{}) &&
                target.cb.local.local == NodeMask{},
            "failure report mutated routing leases");
  }
}

void backward_failure_report_prints_second_terminal_joint() {
  fpga::CBType cb = makeTwoJointTerminalCrossbar();
  resetGrid(1, 1, cb);
  auto &target = *fpga::Device::current().getTile(0, 0);
  target.cb.joint.jump = bit(11);
  pnr::BackwardResolveIndex index;
  std::ostringstream out;
  pnr::dumpBackwardTerminalReport(target, bit(20), index, 5, {}, out);
  const auto text = out.str();
  const std::string *name = cb.nodeName(fpga::CB_NODE_JOINT, 11);
  require(name && text.find("joint2=11[" + *name + "] busy=1") != std::string::npos &&
              text.find("terminal_free=0") != std::string::npos &&
              text.find("NO_INCOMING_SRC") != std::string::npos,
          "failure report omitted the second joint or absent incoming topology");
  require(target.cb.joint.jump == bit(11), "report changed the joint lease");
}

} // namespace

// A free DST behind a leased SRC must remain explorable in the second phase.
// Search itself never changes occupancy, even after finding a complete path.
void docking_provisionally_crosses_occupied_transitions() {
  for (bool source_local : {false, true}) {
    auto cb = makeLinearDockingCrossbar();
    const int east = encodedJump(1, 0);
    cb.local_src[21].jump |= bit(east);
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, 21, "OUTPUT_719");
    rememberConn(cb, fpga::CB_NODE_LOCAL, 21, fpga::CB_NODE_SRC, east);
    cb.rebuildOutgoingSrcs();
    auto tiles = resetGrid(5, 1, cb);
    int eligibility_calls = 0, terminal_checks = 0, complete_calls = 0;
    bool allow = true, accept = true;
    pnr::DockingPreemptionSearch policy;
    policy.eligible_edge = [&](const fpga::Wire &edge) {
      ++eligibility_calls;
      if (edge.jump < 0) ++terminal_checks;
      return allow;
    };
    policy.accept_complete = [&](const auto &) { ++complete_calls; return accept; };
    auto search = [&](const pnr::DockingPreemptionSearch *phase) {
      return pnr::dockGrounding(*tiles[0], source_local ? 21 : 0, {},
          *tiles[4], bit(20), 8, 5, true, {}, nullptr, {}, source_local, phase);
    };
    // Caller first uses the ordinary free search: no victim policy is needed.
    require(search(nullptr).success && eligibility_calls == 0,
            "free path consulted preemption policy");
    tiles[1]->cb.src.jump |= bit(east);
    tiles[2]->cb.dst.jump |= bit(0); // same transit's occupied landing
    tiles[3]->cb.src.jump |= bit(east);
    tiles[4]->cb.dst.jump |= bit(0); // second transit at the terminal
    std::vector<fpga::CBState> states;
    for (auto *tile : tiles) states.push_back(tile->cb);
    require(!search(nullptr).success, "free search crossed occupied transitions");
    auto unchanged = [&]() {
      for (size_t i = 0; i < tiles.size(); ++i)
        require(tiles[i]->cb.src.jump == states[i].src.jump &&
                tiles[i]->cb.dst.jump == states[i].dst.jump &&
                tiles[i]->cb.joint.jump == states[i].joint.jump &&
                tiles[i]->cb.local.local == states[i].local.local,
                "speculative search changed a crossbar mask");
      require(tiles[1]->cb.src.jump.testBit(east) && tiles[2]->cb.dst.jump.testBit(0) &&
              tiles[3]->cb.src.jump.testBit(east) && tiles[4]->cb.dst.jump.testBit(0) &&
              !tiles[4]->isPinNodeLeased(20), "speculative search changed leases");
    };
    require(search(&policy).success && eligibility_calls > 0 && complete_calls > 0,
            "provisional search stopped at occupied SRC before free predecessor DST");
    require(terminal_checks == 1 && complete_calls == 1,
            "search revalidated the terminal suffix while extending upstream");
    require(eligibility_calls <= 4,
            "linear provisional path performed more than one check per new occupied edge");
    unchanged();
    accept = false;
    require(!search(&policy).success, "search ignored full replacement rejection");
    unchanged();
    allow = false; accept = true;
    require(!search(&policy).success, "search traversed a protected obstruction");
    unchanged();
  }
}

// A full backward layer must not hide a higher-numbered predecessor that
// ALREADY joins the forward search. Reproduce a short branch with two occupied
// transit segments and a same-Tile SRC->DST continuation at the destination.
void docking_checks_late_join_after_predecessor_quota(bool occupied) {
  for (bool force_reverse : {false, true}) {
  for (int decoys : {31, 32, 33, 40}) {
    fpga::CBType cb{};
    cb.name = "QUOTA_JOIN";
    cb.type_id = 0;
    constexpr int reached_dst = 3840;
    constexpr int incoming_dst = 80;
    constexpr int terminal_dst = 81;
    constexpr int incoming_src = 88;
    constexpr int continuation_src = 89;
    constexpr int pin = 200;
    for (int dst = 0; dst < decoys; ++dst) {
      cb.rememberNodeName(fpga::CB_NODE_DST, dst, "DECOY_" + std::to_string(dst));
      cb.dst_src[dst].jump.setBit(incoming_src);
      rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, incoming_src);
    }
    cb.rememberNodeName(fpga::CB_NODE_DST, reached_dst, "REACHED_937");
    cb.rememberNodeName(fpga::CB_NODE_DST, incoming_dst, "ARRIVAL_563");
    cb.rememberNodeName(fpga::CB_NODE_DST, terminal_dst, "TERMINAL_821");
    cb.rememberNodeName(fpga::CB_NODE_SRC, incoming_src, "EXIT_419");
    cb.rememberNodeName(fpga::CB_NODE_SRC, continuation_src, "CONTINUATION_277");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "INPUT_613");
    cb.dst_src[reached_dst].jump.setBit(incoming_src);
    cb.dst_src[incoming_dst].jump.setBit(continuation_src);
    cb.dst_local[terminal_dst].local.setBit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, reached_dst, fpga::CB_NODE_SRC, incoming_src);
    rememberConn(cb, fpga::CB_NODE_DST, incoming_dst, fpga::CB_NODE_SRC, continuation_src);
    rememberConn(cb, fpga::CB_NODE_DST, terminal_dst, fpga::CB_NODE_LOCAL, pin);
    rememberJumpTarget(cb, incoming_src, incoming_dst, {1, 0});
    // Also force the occupied case to complete in reverse: provisional
    // forward expansion can now join early in the same-Tile case. The outside
    // variant keeps the late-predecessor quota check independently exercised,
    // rather than weakening its assertion when forward search improves.
    const bool outside = !occupied || force_reverse;
    rememberJumpTarget(cb, continuation_src, terminal_dst, {outside ? 9 : 0, 0});
    cb.rebuildOutgoingSrcs();
    auto tiles = resetGrid(outside ? 11 : 2, 1, cb);
    auto *source = tiles[0], *middle = tiles[1], *target = tiles.back();
    source->cb.dst.jump.setBit(reached_dst); // retained routed branch
    if (occupied) {
      source->cb.src.jump.setBit(incoming_src); // first transit obstruction
      middle->cb.dst.jump.setBit(incoming_dst);
      middle->cb.src.jump.setBit(continuation_src); // second transit obstruction
      target->cb.dst.jump.setBit(terminal_dst);
    }
    const auto source_before = source->cb, target_before = target->cb;
    int full_checks = 0, edge_checks = 0, terminal_checks = 0;
    bool allow_second = true, accept_complete = true;
    pnr::DockingPreemptionSearch policy;
    policy.eligible_edge = [&](const fpga::Wire &edge) {
      ++edge_checks;
      terminal_checks += edge.jump < 0;
      return allow_second || edge.jump != continuation_src;
    };
    policy.accept_complete = [&](const std::vector<fpga::Wire> &path) {
      ++full_checks;
      require(path.size() == 4 && path[0].local == reached_dst &&
                  path[0].jump == incoming_src && path[0].dst == incoming_dst &&
                  path[1].from.x == middle->coord.x && path[1].from.y == middle->coord.y &&
                  path[1].to.x == target->coord.x && path[1].to.y == target->coord.y &&
                  path[1].jump == continuation_src && path[1].dst == terminal_dst &&
                  path[2].local == terminal_dst && path[3].local == pin,
              "late join produced the wrong complete replacement");
      return accept_complete;
    };
    auto search = [&](const pnr::DockingPreemptionSearch *phase) {
      return pnr::dockGrounding(*source, reached_dst, "REACHED_937", *target,
          bit(pin), 4, 5, true, {}, nullptr, {}, false, phase);
    };
    auto unchanged = [&]() {
      require(source->cb.src.jump == source_before.src.jump &&
                  source->cb.dst.jump == source_before.dst.jump &&
                  target->cb.src.jump == target_before.src.jump &&
                  target->cb.dst.jump == target_before.dst.jump &&
                  target->cb.local.local == target_before.local.local &&
                  !target->isPinNodeLeased(pin),
              "late-join probing mutated live leases");
    };
    auto free_result = search(nullptr);
    require(free_result.success == !occupied && edge_checks == 0,
            "free search bypassed an obstruction or hid the late complete join with " +
                std::to_string(decoys) + " lower-numbered predecessors");
    auto result = search(&policy);
    require(result.success && full_checks == 1,
            "backward quota hid the late complete join with " +
                std::to_string(decoys) + " lower-numbered predecessors");
    if (outside)
      require(result.backward_push_count == 1 + std::min(decoys, 32),
              "late join removed the backward exploration quota");
    else
      require(result.backward_push_count == 1 + std::min(decoys, 32) &&
                  result.forward_push_count == 0,
              "new provisional forward search displaced a successful established join");
    // Each local/expanded attempt independently checks its terminal seed.
    // The 32 queued decoys, late join and continuation still get at most
    // 34 transit eligibility checks in total, not one per discarded decoy.
    require(terminal_checks <= 2 && edge_checks - terminal_checks <= 34,
            "full beam still performed expensive eligibility checks for decoys: checks=" +
                std::to_string(edge_checks) + " occupied=" + std::to_string(occupied) +
                " outside=" + std::to_string(outside) + " decoys=" + std::to_string(decoys));
    unchanged();
    accept_complete = false;
    require(!search(&policy).success, "late join ignored complete-path validation");
    unchanged();
    if (occupied) {
      accept_complete = true;
      allow_second = false;
      require(!search(&policy).success, "late join bypassed a protected obstruction");
      unchanged();
    }
  }
  }
}

void docking_preserves_interned_edge_names() {
  fpga::CBType cb{};
  cb.name = "SYNTHETIC_NAMED_CORRIDOR";
  cb.type_id = 0;
  const std::string incoming = "corridor_destination_long_annotation_127";
  const std::string outgoing = "corridor_source_long_annotation_256";
  const std::string terminal = "corridor_local_long_annotation_511";
  const int src = encodedJump(1, 0);
  cb.rememberNodeName(fpga::CB_NODE_DST, 0, incoming);
  cb.rememberNodeName(fpga::CB_NODE_SRC, src, outgoing);
  cb.rememberNodeName(fpga::CB_NODE_LOCAL, 20, terminal);
  cb.dst_src[0].jump.setBit(src);
  cb.dst_local[0].local.setBit(20);
  rememberJumpTarget(cb, src, 0, {1, 0});
  rememberConn(cb, fpga::CB_NODE_DST, 0, fpga::CB_NODE_SRC, src);
  rememberConn(cb, fpga::CB_NODE_DST, 0, fpga::CB_NODE_LOCAL, 20);
  cb.rebuildOutgoingSrcs();
  auto tiles = resetGrid(3, 1, cb);
  for (bool traced : {false, true}) {
    auto result = pnr::dockGrounding(*tiles.front(), 0, incoming,
                                    *tiles.back(), bit(20), 5, 5, traced);
    require(result.success, "named corridor docking failed");
    size_t hops = 0;
    for (const auto& edge : result.fragments) {
      if (edge.jump < 0) continue;
      ++hops;
      require(edge.from_wire_name == incoming && edge.src_wire_name == outgoing
                  && edge.dst_wire_name == incoming,
              "Docking lost or changed an interned edge annotation");
    }
    require(hops == 2 && result.fragments.back().local == 20,
            "named corridor did not retain both jumps and its terminal");
  }
}

int main() {
  try {
    docking_duplicate_checks_only_recent_entries();
    docking_preserves_interned_edge_names();
    for (int rotation = 0; rotation < 4; ++rotation)
      for (int width : {32, 64, 96})
        grounding_dense_source_local(rotation, width);
    for (bool occupied : {true, false})
      docking_checks_late_join_after_predecessor_quota(occupied);
    docking_provisionally_crosses_occupied_transitions();
    for (int rotation = 0; rotation < 4; ++rotation)
      for (bool joints : {false, true})
        for (bool reverse_only : {false, true})
          grounding_routes_backwards_to_source_local(rotation, joints, reverse_only);
    for (int radius : {2, 5})
      for (bool beyond_limit : {false, true})
        docking_hop_budget_tracks_radius(radius, beyond_limit);
    for (int count : {32, 31, 33})
      for (bool local_first : {false, true})
        docking_preserves_local_path_when_outside_choices_grow(count, local_first);
    for (int rotation = 0; rotation < 4; ++rotation)
      for (bool relay : {false, true})
        for (bool shared : {true, false})
          for (bool busy_local_terminal : {false, true})
            docking_follows_incoming_wires_outside_window(rotation, relay, shared,
                                                         busy_local_terminal);
    docking_preserves_intra_cb_continuity(1);
    docking_preserves_intra_cb_continuity(2);
    backward_failure_report_explains_incoming_blockers();
    backward_failure_report_prints_second_terminal_joint();
    for (unsigned seed : {17u, 93u, 511u})
      for (int rotation = 0; rotation < 4; ++rotation)
        for (bool blocked : {false, true})
          backward_anchor_chooses_short_route(seed, rotation, blocked);
    terminal_entry_cache_refreshes_after_topology_growth();
    for (unsigned seed = 1; seed <= 20; ++seed) {
      docking_finds_one_random_free_path(seed);
    }
    docking_extends_from_existing_anchor_dst();
    docking_uses_mask_angle_priority_before_numeric_node_order();
    docking_backward_meets_an_existing_anchor_dst();
    docking_late_terminal_entry_meets_anchor_after_width_limit();
    docking_reuses_beam_slots_from_dead_terminal_seeds();
    docking_ignores_src_deadends();
    docking_steps_out_from_non_enterable_target_dst();
    docking_iob_uses_wider_endpoint_window();
    docking_backward_uses_resolved_target_dst_namespace();
    docking_reports_only_the_reachable_blocked_terminal();
    for (unsigned seed = 1; seed <= 20; ++seed) {
      docking_builds_random_multi_hop_backward_routes(seed);
    }
    docking_memoizes_failed_backward_positions();
    docking_reports_exact_busy_bridge_between_separate_frontiers();
    docking_reports_busy_bridge_at_committed_forward_anchor();
    docking_reports_busy_reverse_boundary_before_frontiers_join();
    docking_preserves_bridges_from_blocked_terminal_search();
    docking_finds_blocked_bridge_after_full_backward_beam();
    docking_preserves_and_checks_both_terminal_joints();
    backward_input_avoids_proposed_output(true);
    backward_input_avoids_proposed_output(false);
    backward_route_stops_at_first_legal_takeoff();
    moving_source_reverse_search_depth_is_ten_hops();
    moving_source_explores_free_continuations_before_preemption();
    moving_source_preempts_after_exact_probe_budget_exhausts_free_search();
    moving_source_probes_before_exhausting_reverse_component();
    backward_route_seeds_only_physical_terminal_arrivals();
    backward_anchor_search_does_not_starve_free_terminal_entries();
    moving_input_proof_reuses_private_prefix_after_sink_move();
    moving_input_proof_keeps_its_incomplete_owned_prefix();
    backward_failure_reports_last_congestion_with_connected_path();
    moving_input_proof_does_not_mistake_expansion_limit_for_no_path();
    moving_source_backward_docking_preserves_partial_prefix();
    combinatorial_search_allows_two_tile_visits_and_rejects_third();
    for (int blocked : {0, 1, 2}) docking_join_counts_combined_history(blocked);
    for (bool iob : {false, true})
      for (bool alternative : {false, true})
        for (int blocked : {0, 1, 2})
          docking_counts_retained_prefix(iob, alternative, blocked);
  } catch (const TestFailure &failure) {
    std::fprintf(stderr, "docking_test failed: %s\n", failure.message.c_str());
    return EXIT_FAILURE;
  } catch (const std::exception &ex) {
    std::fprintf(stderr, "docking_test exception: %s\n", ex.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
