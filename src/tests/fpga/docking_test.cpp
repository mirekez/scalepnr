#include "Device.h"
#include "Docking.h"
#include "RoutePassState.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
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

  int one_window_scan_limit = 0;
  for (int y = target.y - radius; y <= target.y + radius; ++y) {
    for (int x = target.x - radius; x <= target.x + radius; ++x) {
      fpga::Tile *tile = fpga::Device::current().getTile(x, y);
      if (tile && tile->cb_type) {
        one_window_scan_limit +=
            static_cast<int>(tile->cb_type->dst_by_src.values.size());
      }
    }
  }
  require(result.backward_mapping_scan_count <= one_window_scan_limit,
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
  }

  // Packed endpoint reservations are topology constraints rather than live
  // leases. Docking must avoid either reserved joint just like direct entry.
  for (int reserved_joint : {first_joint, second_joint}) {
    cb = makeTwoJointTerminalCrossbar();
    resetGrid(1, 1, cb);
    tile = fpga::Device::current().getTile(0, 0);
    require(tile, "reserved two-joint docking tile missing");
    result = pnr::dockGrounding(*tile, dst, "ENTRY", *tile, bit(pin), 5, 5,
                                false, bit(reserved_joint));
    require(!result.success, "two-joint docking consumed a joint reserved by "
                             "another packed endpoint");
    require(tile->cb.joint.jump == NodeMask{},
            "failed reserved-joint docking changed the live joint leases");
  }
}

} // namespace

int main() {
  try {
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
  } catch (const TestFailure &failure) {
    std::fprintf(stderr, "docking_test failed: %s\n", failure.message.c_str());
    return EXIT_FAILURE;
  } catch (const std::exception &ex) {
    std::fprintf(stderr, "docking_test exception: %s\n", ex.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
