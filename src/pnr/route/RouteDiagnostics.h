#pragma once

#include "Tile.h"
#include "Wire.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace pnr {

// Opt-in diagnostic scope shared by Generic, Fanout and Docking routines.
// No history is retained in normal routing. Repeated unchanged Tile snapshots
// within a search refer to their first event; every rejected edge is printed.
class RouteCongestionTrace {
 public:
  inline static thread_local RouteCongestionTrace* current = nullptr;

  static bool matches(std::string_view route) {
    static const std::string selected = [] {
      const char* value = std::getenv("SCALEPNR_CONGESTION_NET");
      return value ? std::string(value) : std::string{};
    }();
    return !selected.empty() && route == selected;
  }

  RouteCongestionTrace(std::string_view route, std::string_view stage,
                       std::vector<rtl::Net*> nets)
      : previous(current), design_nets(std::move(nets)), out(log()), id(++sequence) {
    current = this;
    out << "ATTEMPT_BEGIN id=" << id << " stage=" << stage
        << " route=" << std::quoted(std::string(route)) << '\n';
  }
  ~RouteCongestionTrace() {
    out << "ATTEMPT_END id=" << id << " events=" << events << '\n';
    out.flush();
    current = previous;
  }
  RouteCongestionTrace(const RouteCongestionTrace&) = delete;
  RouteCongestionTrace& operator=(const RouteCongestionTrace&) = delete;

  void search(fpga::Tile& from, fpga::Tile& to, int node, int depth) {
    snapshots.clear();
    out << "SEARCH attempt=" << id << " from=(" << from.coord.x << ',' << from.coord.y
        << ") node=" << node << " to=(" << to.coord.x << ',' << to.coord.y
        << ") depth=" << depth << '\n';
  }

  void blocked(fpga::Tile& tile, std::string_view reason,
               std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes,
               const fpga::CBState* search_state = nullptr) {
    const size_t event = ++events;
    out << "BLOCK attempt=" << id << " event=" << event << " reason=" << reason
        << " tile=(" << tile.coord.x << ',' << tile.coord.y << ")";
    for (auto [type, node] : nodes) {
      if (node < 0 || node >= CB_MAX_NODES) continue;
      bool live = fpga::congestionNodeMask(tile.cb, type).testBit(node) ||
                  (type == fpga::CB_NODE_LOCAL && tile.isPinNodeLeased(node));
      bool speculative = search_state && fpga::congestionNodeMask(*search_state, type).testBit(node);
      const char* label = type == fpga::CB_NODE_SRC ? "SRC" : type == fpga::CB_NODE_DST ? "DST" :
                          type == fpga::CB_NODE_JOINT ? "JOINT" : "LOCAL";
      out << " node=" << label << ':' << node
          << "/live=" << live << "/search=" << speculative;
    }
    out << '\n';
    if (search_state) {
      out << "SEARCH_ONLY";
      for (auto type : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST, fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL}) {
        NodeMask extra = fpga::congestionNodeMask(*search_state, type) & ~fpga::congestionNodeMask(tile.cb, type);
        extra.for_each_set_bit([&](int node) { out << ' ' << static_cast<int>(type) << ':' << node; return false; });
      }
      out << '\n';
    }
    auto key = std::pair{tile.coord.x, tile.coord.y};
    auto found = snapshots.find(key);
    bool unchanged = found != snapshots.end();
    if (unchanged) {
      for (auto type : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST, fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL}) {
        unchanged &= fpga::congestionNodeMask(found->second.cb, type) == fpga::congestionNodeMask(tile.cb, type);
      }
      unchanged &= found->second.pin == tile.pin_state.leased_nodes &&
                   found->second.cb.src_deadend.jump == tile.cb.src_deadend.jump;
    }
    if (unchanged) {
      out << "SNAPSHOT_REF attempt=" << id << " event=" << event
          << " snapshot_event=" << found->second.event << '\n';
    } else {
      out << "SNAPSHOT_BEGIN attempt=" << id << " event=" << event << '\n';
      fpga::auditTileCongestion(tile, out, design_nets);
      out << "SNAPSHOT_END\n";
      snapshots[key] = {tile.cb, tile.pin_state.leased_nodes, event};
    }
    out.flush();
  }

  // Call after mutations even if a released lease is immediately reused.
  void changed() { snapshots.clear(); }
  std::ostream& stream() { return out; }

 private:
  struct Snapshot { fpga::CBState cb; NodeMask pin; size_t event; };
  RouteCongestionTrace* previous;
  std::vector<rtl::Net*> design_nets;
  std::ostream& out;
  size_t id;
  size_t events = 0;
  std::map<std::pair<int, int>, Snapshot> snapshots;
  inline static size_t sequence = 0;
  static std::ostream& log() {
    static std::ofstream file([] {
      const char* path = std::getenv("SCALEPNR_CONGESTION_LOG");
      return path && *path ? path : "routing_congestion.log";
    }());
    if (!file) throw std::runtime_error("cannot write routing congestion diagnostic");
    return file;
  }
};
} // namespace pnr
