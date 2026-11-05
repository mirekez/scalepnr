#pragma once

#include "Tile.h"
#include "Wire.h"
#include "Device.h"

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <optional>
#include <concepts>
#include <chrono>

namespace pnr {

// Diagnostics cannot extend a stage or turn expired work into a failed-node
// proof. The half-open interval lets tests check the window without sleeping.
inline bool sourceTimeoutDiagnosticWindow(
    bool moving_sources, bool deadline_enabled,
    std::chrono::steady_clock::time_point deadline,
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds window) {
  return moving_sources && deadline_enabled && window.count() > 0 &&
         now < deadline && deadline - now <= window;
}

// Formatting belongs inside the enabled branch, including diagnostic replays.
class PackingTrace {
 public:
  PackingTrace(std::FILE* file, std::ofstream& replay) : file(file), replay(replay) {}
  bool enabled() const { return file || replay.is_open(); }
  void operator()(std::string_view message) const {
    if (file) {
      std::fwrite(message.data(), 1, message.size(), file);
      std::fputc('\n', file);
    } else if (replay.is_open()) replay << message << '\n';
  }
  template<std::invocable Formatter>
  void operator()(Formatter&& format) const {
    if (enabled()) (*this)(format());
  }
 private:
  std::FILE* file;
  std::ofstream& replay;
};

using MovingTerminalOwners = std::unordered_map<int, std::string>;

// Read-only snapshots taken DURING the candidate probe, before rollback. Base
// excludes the forward prefix; tile.cb includes it; trial also includes the
// trigger and previously reserved inputs. Never call a search lease an orphan.
inline void printMovingCandidateState(std::ostream& out, fpga::Tile& tile,
                                     const fpga::CBState& base) {
  out << "CANDIDATE_CB tile=(" << tile.coord.x << ',' << tile.coord.y << ")\n";
  auto mask = [&](const char* label, const NodeMask& bits) {
    out << label << "=[";
    bits.for_each_set_bit([&](int node) { out << node << ','; return false; });
    out << "]\n";
  };
  for (auto [type, name] : {std::pair{fpga::CB_NODE_SRC, "SRC"},
       {fpga::CB_NODE_DST, "DST"}, {fpga::CB_NODE_JOINT, "JOINT"},
       {fpga::CB_NODE_LOCAL, "LOCAL"}}) {
    out << name << ' ';
    mask("base", fpga::congestionNodeMask(base, type));
    out << name << ' ';
    mask("prefix_only", fpga::congestionNodeMask(tile.cb, type) &
                        ~fpga::congestionNodeMask(base, type));
  }
  mask("PIN", tile.pin_state.leased_nodes);
  mask("DEADEND", tile.cb.src_deadend.jump);
  mask("INCOMING", tile.incoming_dst_nodes);
}

inline void printMovingTerminalAlternatives(
    std::ostream& out, fpga::Tile& tile, const NodeMask& locals,
    const fpga::CBState& base, const fpga::CBState& trial,
    const NodeMask& trial_pins, const MovingTerminalOwners& local_owners,
    const MovingTerminalOwners& dst_owners, const MovingTerminalOwners& joint_owners) {
  auto node = [&](fpga::CBNodeNameType type, const char* label, int id,
                  const MovingTerminalOwners& owners) {
    if (id < 0) return;
    const auto* name = tile.cb_type->nodeName(type, id);
    auto* owner = fpga::findNetByNode(tile, type, id, false);
    auto temporary = owners.find(id);
    out << "  " << label << ':' << id << " name=" << std::quoted(name ? *name : "?")
        << " base=" << fpga::congestionNodeMask(base, type).testBit(id)
        << " prefix_only=" << (fpga::congestionNodeMask(tile.cb, type).testBit(id) &&
                                 !fpga::congestionNodeMask(base, type).testBit(id))
        << " trial=" << fpga::congestionNodeMask(trial, type).testBit(id)
        << " registered_route_owner=" << std::quoted(owner ? owner->makeName(1000000) : "<none>")
        << " temporary_owner=" << std::quoted(temporary == owners.end() ? "<none>" : temporary->second);
    if (type == fpga::CB_NODE_LOCAL)
      out << " base_pin=" << tile.isPinNodeLeased(id) << " trial_pin=" << trial_pins.testBit(id);
    out << '\n';
  };
  locals.for_each_set_bit([&](int local) {
    const auto& entries = tile.cb_type->terminalEntries(local);
    if (entries.empty()) out << "TERMINAL_PATH local=" << local << " reason=no-topology\n";
    for (const auto& entry : entries) {
      const bool incoming = tile.incoming_dst_nodes.testBit(entry.dst);
      const bool occupied = trial_pins.testBit(local) || trial.local.local.testBit(local) ||
          trial.dst.jump.testBit(entry.dst) ||
          (entry.joint >= 0 && trial.joint.jump.testBit(entry.joint)) ||
          (entry.joint2 >= 0 && trial.joint.jump.testBit(entry.joint2));
      out << "TERMINAL_PATH local=" << local << " dst=" << entry.dst
          << " joint=" << entry.joint << " joint2=" << entry.joint2
          << " reason=" << (!incoming ? "no-incoming-src" : occupied ? "occupied" : "free") << '\n';
      node(fpga::CB_NODE_LOCAL, "LOCAL", local, local_owners);
      node(fpga::CB_NODE_DST, "DST", entry.dst, dst_owners);
      node(fpga::CB_NODE_JOINT, "JOINT", entry.joint, joint_owners);
      node(fpga::CB_NODE_JOINT, "JOINT", entry.joint2, joint_owners);
    }
    return false;
  });
}

// Opt-in diagnostic scope shared by Generic, Fanout and Docking routines.
// No history is retained in normal routing. Repeated unchanged Tile snapshots
// Explicit blocked events refer to their first snapshot; numeric edge traces
// request snapshots only when state changes. Every rejected edge is printed.
class RouteCongestionTrace : public fpga::RouteSearchTrace {
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
                       std::vector<rtl::Net*> nets, std::ostream* output = nullptr,
                       std::vector<rtl::Inst*> insts = {}, bool defer_ownership = false)
      : previous(current), design_nets(std::move(nets)),
        design_insts(std::move(insts)), out(output ? *output : log()), id(++sequence),
        defer_ownership(defer_ownership) {
    current = this;
    out << "ATTEMPT_BEGIN id=" << id << " stage=" << stage
        << " route=" << std::quoted(std::string(route)) << '\n';
  }
  ~RouteCongestionTrace() {
    // Deadline windows capture decisions first; full-design owner reconstruction
    // runs only after search returns, without granting extra search time.
    if (defer_ownership && !deferred_tiles.empty()) {
      fpga::CongestionAuditIndex index(design_nets);
      for (auto* tile : deferred_tiles) {
        const auto found = snapshots.find({tile->coord.x, tile->coord.y});
        bool same = found != snapshots.end();
        if (same) {
          for (auto type : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST, fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL})
            same &= fpga::congestionNodeMask(found->second.cb, type) == fpga::congestionNodeMask(tile->cb, type);
          same &= found->second.pin == tile->pin_state.leased_nodes;
        }
        out << "POST_ATTEMPT_AUDIT tile=(" << tile->coord.x << ',' << tile->coord.y
            << ") matches_captured_masks=" << same
            << " matches_captured_revision=" << (found != snapshots.end() &&
                found->second.revision == fpga::routeMutationRevision()) << '\n';
        fpga::auditTileCongestion(*tile, out, index);
      }
    }
    out << "ATTEMPT_END id=" << id << " events=" << events << '\n';
    out.flush();
    // An outer task may activate lazily inside a shorter-lived replay scope.
    // Unlink this scope without restoring or hiding a different live trace.
    auto** link = &current;
    while (*link && *link != this) link = &(*link)->previous;
    if (*link == this) *link = previous;
  }
  RouteCongestionTrace(const RouteCongestionTrace&) = delete;
  RouteCongestionTrace& operator=(const RouteCongestionTrace&) = delete;

  void search(fpga::Tile& from, fpga::Tile& to, int node, int depth) {
    snapshots.clear();
    ownership_index.reset();
    out << "SEARCH attempt=" << id << " from=(" << from.coord.x << ',' << from.coord.y
        << ") node=" << node << " to=(" << to.coord.x << ',' << to.coord.y
        << ") depth=" << depth << '\n';
  }

  void blocked(fpga::Tile& tile, std::string_view reason,
               std::initializer_list<std::pair<fpga::CBNodeNameType, int>> nodes,
               const fpga::CBState* search_state = nullptr) {
    if (ownership_index && ownership_revision != fpga::routeMutationRevision()) changed();
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
    const bool unchanged = found != snapshots.end() && sameSnapshot(tile, found->second);
    if (unchanged) {
      out << "SNAPSHOT_REF attempt=" << id << " event=" << event
          << " snapshot_event=" << found->second.event << '\n';
    } else {
      out << "SNAPSHOT_BEGIN attempt=" << id << " event=" << event << '\n';
      if (defer_ownership) {
        deferred_tiles.insert(&tile);
        printMovingCandidateState(out, tile, tile.cb);
      } else {
      if (!ownership_index) {
        ownership_index.emplace(design_nets);
        ownership_revision = fpga::routeMutationRevision();
      }
      fpga::auditTileCongestion(tile, out, *ownership_index);
      }
      placement(tile);
      for (auto* resource : tile.attached_resource_tiles)
        if (resource && resource != &tile) placement(*resource);
      out << "SNAPSHOT_END\n";
      snapshots[key] = {tile.cb, tile.pin_state.leased_nodes, event, fpga::routeMutationRevision()};
    }
    out.flush();
  }

  // Call after mutations even if a released lease is immediately reused.
  void changed() { snapshots.clear(); ownership_index.reset(); }
  // Count registry/peer examinations so diagnostic work has a regression bound.
  size_t placementChecks() const { return placement_checks; }
  std::ostream& stream() override { return out; }
  void snapshot(fpga::Tile& tile) override {
    // The edge already prints both coordinates. Avoid flushing duplicate
    // state records for every converging edge (and twice for internal edges).
    const auto found = snapshots.find({tile.coord.x, tile.coord.y});
    if (found != snapshots.end() && sameSnapshot(tile, found->second)) return;
    blocked(tile, "numeric_repair", {});
  }

 private:
  struct Snapshot { fpga::CBState cb; NodeMask pin; size_t event; uint64_t revision; };
  // Equal masks can belong to a new owner after a cut and replacement route.
  // Include the publication revision even when ownership auditing is deferred.
  static bool sameSnapshot(const fpga::Tile& tile, const Snapshot& saved) {
    if (saved.revision != fpga::routeMutationRevision() ||
        saved.pin != tile.pin_state.leased_nodes ||
        saved.cb.src_deadend.jump != tile.cb.src_deadend.jump) return false;
    for (auto type : {fpga::CB_NODE_SRC, fpga::CB_NODE_DST, fpga::CB_NODE_JOINT, fpga::CB_NODE_LOCAL})
      if (fpga::congestionNodeMask(saved.cb, type) != fpga::congestionNodeMask(tile.cb, type)) return false;
    return true;
  }
  RouteCongestionTrace* previous;
  std::vector<rtl::Net*> design_nets;
  std::vector<rtl::Inst*> design_insts;
  std::ostream& out;
  size_t id;
  bool defer_ownership;
  std::unordered_set<fpga::Tile*> deferred_tiles;
  size_t events = 0;
  size_t placement_checks = 0;
  // Index reference identity, not tile membership: a candidate can move while
  // this trace is alive, without publishing any routing-mask revision.
  using PlacementRegistry = std::unordered_map<const RefBase<Referable<fpga::Tile>>*, rtl::Inst*>;
  std::optional<PlacementRegistry> placement_registry;
  std::map<std::pair<int, int>, Snapshot> snapshots;
  std::optional<fpga::CongestionAuditIndex> ownership_index;
  uint64_t ownership_revision = 0;
  inline static size_t sequence = 0;
  // List physical occupants and all element columns alongside the CB leases.
  void placement(fpga::Tile& tile) {
    out << "PLACEMENT_TILE coord=(" << tile.coord.x << ',' << tile.coord.y << ")\n";
    for (size_t i = 0; i < tile.elements_pos.size(); ++i) {
      if (!tile.elements_pos[i] && !tile.elements_free[i]) continue;
      out << "ELEMENT column=" << i << " pos=" << tile.elements_pos[i]
          << " free=" << tile.elements_free[i] << '\n';
      for (size_t bit = 0; bit < tile.elements_left[i].size(); ++bit)
        if (tile.elements_left[i][bit] || tile.elements_right[i][bit])
          out << "ELEMENT_LINK column=" << i << " bit=" << bit
              << " left=" << tile.elements_left[i][bit]
              << " right=" << tile.elements_right[i][bit] << '\n';
    }
    // Restrict occupants to the design registry, including unplaced cells
    // that may acquire a tile later in this diagnostic scope.
    std::unordered_set<rtl::Inst*> printed;
    auto emit = [&](rtl::Inst* inst) {
      if (!inst || inst->tile.peer != &tile || !printed.insert(inst).second) return;
      out << "OCCUPANT name=" << std::quoted(inst->cell_ref.peer ? inst->makeName(1000000) : "<untyped>")
          << " type=" << std::quoted(inst->cell_ref.peer ? inst->cell_ref->type : "")
          << " pos=" << inst->pos << " fixed=" << inst->outline.fixed << '\n';
      for (auto& conn : inst->conns) {
        if (!conn.port_ref.peer) continue;
        out << "OCCUPANT_PIN port=" << std::quoted(conn.port_ref->makeName())
            << " direction=" << conn.port_ref->type;
        if (conn.peer && conn.peer->inst_ref.peer && conn.peer->port_ref.peer)
          out << " connected_inst=" << std::quoted(conn.peer->inst_ref->cell_ref.peer
                  ? conn.peer->inst_ref->makeName(1000000) : "<untyped>")
              << " connected_port=" << std::quoted(conn.peer->port_ref->makeName());
        out << '\n';
      }
    };
    // Only actual grid elements are known to be Referable<Tile>. Standalone
    // diagnostic Tiles retain the registry scan, with no speculative downcast.
    auto& device = fpga::Device::current();
    Referable<fpga::Tile>* grid_tile = nullptr;
    const auto at = tile.coord;
    if (at.x >= 0 && at.y >= 0 && at.x < device.size_width && at.y < device.size_height &&
        device.grid_spec.size.x > 0) {
      const size_t index = size_t(at.y) * size_t(device.grid_spec.size.x) + size_t(at.x);
      if (index < device.tile_grid.size() && &device.tile_grid[index] == &tile)
        grid_tile = &device.tile_grid[index];
    }
    if (grid_tile && !design_insts.empty()) {
      if (!placement_registry) {
        placement_registry.emplace();
        placement_registry->reserve(design_insts.size());
        for (auto* inst : design_insts) {
          ++placement_checks;
          if (inst) placement_registry->emplace(&inst->tile, inst);
        }
      }
      // Ref::set/clear maintains these peers during trial moves. Ignore other
      // reference kinds rather than treating every Tile reference as an Inst.
      for (auto* peer : grid_tile->getPeers()) {
        ++placement_checks;
        auto found = placement_registry->find(peer);
        if (found != placement_registry->end()) emit(found->second);
      }
    } else {
      for (auto* inst : design_insts) { ++placement_checks; emit(inst); }
    }
    if (design_insts.empty())
      for (auto* net : design_nets) if (net)
        for (const auto& binding : net->routes) {
          emit(binding.from); emit(binding.to);
        }
  }
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
