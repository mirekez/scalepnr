#pragma once

#include "Device.h"
#include "Element.h"
#include "RouteDiagnostics.h"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <ostream>
#include <vector>

namespace pnr {

struct FailurePlacementCandidate {
  fpga::Tile* tile;
  std::vector<int> packing_positions;
  std::vector<int> live_positions;
};

// Diagnostic only: exhaust focus positions using the same cluster orders and
// preview legality as Moving. Original placements and all leases stay intact.
inline std::vector<int> failurePackingPositions(
    fpga::Tile& tile, rtl::Inst& focus,
    const std::vector<std::vector<rtl::Inst*>>& orders, bool live) {
  if (!tile.tile_type || orders.empty() || orders.front().empty()) return {};
  struct Saved {
    rtl::Inst* inst;
    Referable<fpga::Tile>* tile;
    fpga::Coord coord;
    int pos;
  };
  struct Restore {
    std::vector<Saved> entries;
    ~Restore() {
      for (auto& old : entries) {
        old.inst->tile.clear();
        if (old.tile) old.inst->tile.set(old.tile);
        old.inst->coord = old.coord;
        old.inst->pos = old.pos;
      }
    }
  } restore;
  // Original locations are not relocations; do not count their occupied slots twice.
  for (auto* member : orders.front())
    if (!member || !member->cell_ref.peer || member->outline.fixed || member->tile.peer == &tile)
      return {};
  if (std::find(orders.front().begin(), orders.front().end(), &focus) == orders.front().end())
    return {};
  for (auto* member : orders.front()) {
    restore.entries.push_back({member, member->tile.peer, member->coord, member->pos});
    member->tile.clear();
    member->pos = -1;
  }
  fpga::ElementPackingPreview preview(tile);
  std::vector<int> result;
  for (int focus_pos : tile.candidatePositions(&focus)) {
    for (const auto& order : orders) {
      auto pack = [&](auto&& self, size_t index) -> bool {
        if (index == order.size()) return true;
        auto* member = order[index];
        for (int pos : tile.candidatePositions(member)) {
          if (member == &focus && pos != focus_pos) continue;
          const size_t checkpoint = preview.checkpoint();
          const int reserved = live ? preview.reserveAt(member, pos, true)
                                    : preview.reservePlacementAt(member, pos);
          const bool fits = reserved >= 0 && self(self, index + 1);
          preview.rollback(checkpoint);
          if (fits) return true;
        }
        return false;
      };
      if (pack(pack, 0)) { result.push_back(focus_pos); break; }
    }
  }
  return result;
}

// Stream every tile, including empty/unvisited tiles, before any preview detaches refs.
inline void dumpFailureTileContents(fpga::Device& device, std::ostream& out) {
  for (auto& tile : device.tile_grid) {
    out << "TILE coord=(" << tile.coord.x << ',' << tile.coord.y << ") name="
        << std::quoted(tile.full_name) << " type="
        << std::quoted(tile.tile_type ? tile.tile_type->name : "")
        << " cb=(" << tile.cb_coord.x << ',' << tile.cb_coord.y << ") sparse="
        << tile.sparse << " elements_initialized=" << tile.elements_initialized << '\n';
    for (size_t type = 0; type < tile.elements_pos.size(); ++type)
      out << "  ELEMENT type=" << type << " positions=" << tile.elements_pos[type]
          << " free=" << tile.elements_free[type] << '\n';
    for (auto* ref : tile.getPeers()) {
      auto* inst = reinterpret_cast<rtl::Inst*>(reinterpret_cast<char*>(
          Ref<fpga::Tile>::fromBase(ref)) - offsetof(rtl::Inst, tile));
      out << "  CELL name=" << std::quoted(inst->makeName(1000000))
          << " type=" << std::quoted(inst->cell_ref.peer ? inst->cell_ref->type : "")
          << " pos=" << inst->pos << " fixed=" << inst->outline.fixed << '\n';
      for (auto& conn : inst->conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) continue;
        auto* driver = conn.follow();
        out << "    INPUT port=" << std::quoted(conn.port_ref->makeName())
            << " signal=" << conn.port_ref->designator << " driver="
            << std::quoted(driver && driver->inst_ref.peer
                ? driver->inst_ref->makeName(1000000) : "<none>") << " port="
            << std::quoted(driver && driver->port_ref.peer ? driver->port_ref->makeName() : "") << '\n';
      }
    }
    if (tile.cb_type) printMovingCandidateState(out, tile, tile.cb);
    out << "END_TILE\n";
  }
}

// Packing is necessary, not sufficient: live positions include route-capacity
// and mandatory-joint checks, but neither column proves all inputs can route.
inline std::vector<FailurePlacementCandidate> dumpFailurePlacementMap(
    fpga::Device& device, rtl::Inst& focus,
    const std::vector<std::vector<rtl::Inst*>>& orders, std::ostream& out) {
  std::vector<FailurePlacementCandidate> result;
  out << "focus=" << std::quoted(focus.makeName(1000000))
      << " original=(" << focus.coord.x << ',' << focus.coord.y << ")\n"
      << "packing_positions=pure packing; live_positions=packing with live input capacity/joints\n"
      << "Neither list proves routing reachability. All grid tiles are scanned, not just visited tiles.\n";
  for (auto& tile : device.tile_grid) {
    auto positions = failurePackingPositions(tile, focus, orders, false);
    if (positions.empty()) continue;
    auto live = failurePackingPositions(tile, focus, orders, true);
    result.push_back({&tile, std::move(positions), std::move(live)});
  }
  // Reporting order only, never routing priority or candidate selection.
  auto distance = [&](fpga::Tile* tile) {
    return std::max(std::abs(tile->coord.x - focus.coord.x),
                    std::abs(tile->coord.y - focus.coord.y));
  };
  std::stable_sort(result.begin(), result.end(), [&](const auto& a, const auto& b) {
    return distance(a.tile) < distance(b.tile);
  });
  for (auto& candidate : result) {
    auto& tile = *candidate.tile;
    out << "CANDIDATE tile=(" << tile.coord.x << ',' << tile.coord.y << ") cb=("
        << tile.cb_coord.x << ',' << tile.cb_coord.y << ") distance=" << distance(&tile)
        << " packing_positions=[";
    for (int pos : candidate.packing_positions) out << pos << ',';
    out << "] live_positions=[";
    for (int pos : candidate.live_positions) out << pos << ',';
    out << "]\n";
  }
  out << "candidate_tiles=" << result.size() << '\n';
  return result;
}

} // namespace pnr
