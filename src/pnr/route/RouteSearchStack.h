#pragma once

#include "Wire.h"

#include <span>
#include <vector>

namespace pnr {

// Suspended iterative DFS, one level per committed routing hop. Popping a
// child preserves its parent's alternatives; replacing a prefix drops only
// the levels below the changed edge. This is task-local, not a Tile deadend.
struct RouteSearchStack {
  struct Edge {
    fpga::Coord from{}, to{};
    int local = -1, src = -1, dst = -1, joint = -1, joint2 = -1;
    bool operator==(const Edge &other) const {
      return from.x == other.from.x && from.y == other.from.y &&
             to.x == other.to.x && to.y == other.to.y &&
             local == other.local && src == other.src && dst == other.dst &&
             joint == other.joint && joint2 == other.joint2;
    }
  };
  struct Level {
    Edge incoming;
    NodeMask tried_sources{};
  };
  std::vector<Level> levels;
  fpga::Coord target{-1, -1};
  int target_pos = -1;

  void clear() { levels.clear(); }

  void resume(std::span<const fpga::Wire> route, fpga::Coord destination,
              int position) {
    if (target.x != destination.x || target.y != destination.y ||
        target_pos != position) {
      clear();
      target = destination;
      target_pos = position;
    }
    size_t depth = 0;
    auto enter = [&](const Edge &edge) {
      if (depth < levels.size() && !(levels[depth].incoming == edge))
        levels.resize(depth);
      if (depth == levels.size()) levels.push_back({edge, {}});
      ++depth;
    };
    for (const fpga::Wire &wire : route) {
      if (wire.type != fpga::Wire::WIRE_CROSSBAR || wire.jump < 0) continue;
      if (depth == 0)
        enter({wire.from, wire.from, wire.local, -1, wire.local});
      // This child has already been selected. A later pop resumes after it.
      levels[depth - 1].tried_sources.setBit(wire.jump);
      enter({wire.from, wire.to, wire.local, wire.jump, wire.dst,
             wire.joint, wire.joint2});
    }
    levels.resize(depth);
  }
};

} // namespace pnr
