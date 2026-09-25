#include "RouteDesign.h"
#include "Device.h"
#include "Docking.h"
#include "RouteClocks.h"
#include "RouteVCC.h"
#include "RouteDiagnostics.h"
#include "Tech.h"
#include "Visualizer.h"
#include "Wire.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace pnr;

void RouteDesign::RouteStats::clear() { *this = RouteStats{}; }

BackwardResolveIndex &
RouteDesign::backwardDockingIndex(fpga::Coord center, int radius) {
  static size_t cache_hits = 0;
  static size_t cache_misses = 0;
  auto report_cache = [&]() {
    if ((cache_hits + cache_misses) % 128 == 0) {
      PNR_LOG2("ROUT", "docking index cache: entries={}, hits={}, misses={}",
               docking_indexes.size(), cache_hits, cache_misses);
    }
  };
  for (DockingIndexCacheEntry &entry : docking_indexes) {
    if (entry.center.x == center.x && entry.center.y == center.y &&
        entry.radius == radius) {
      ++cache_hits;
      report_cache();
      return entry.index;
    }
  }

  ++cache_misses;
  if (!docking_resolved_arcs_complete) {
    // Build the numeric incoming-edge graph once. Per-task indexes below are
    // lightweight radius views and never rescan the device grid.
    fpga::Coord device_center{fpga->size_width / 2, fpga->size_height / 2};
    int device_radius = std::max(fpga->size_width, fpga->size_height);
    buildBackwardResolveIndex(*fpga, device_center, device_radius, {},
                              &docking_resolved_arcs);
    docking_resolved_arcs_complete = true;
  }
  // Destination views populate lazily from the shared device-wide topology.
  // A small working set bounds memory on designs with thousands of sinks.
  constexpr size_t cache_capacity = 1;
  BackwardResolveIndex index;
  index.center = center;
  index.radius = radius;
  index.shared_cache = &docking_resolved_arcs;
  DockingIndexCacheEntry entry{center, radius, std::move(index)};
  if (docking_indexes.size() < cache_capacity) {
    docking_indexes.push_back(std::move(entry));
    report_cache();
    return docking_indexes.back().index;
  }

  size_t replace = docking_index_replacement++ % cache_capacity;
  docking_indexes[replace] = std::move(entry);
  report_cache();
  return docking_indexes[replace].index;
}

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

bool envFlagEnabled(const char *name) {
  const char *value = std::getenv(name);
  return value && value[0] != '\0' && std::string_view(value) != "0";
}

// Keep failure artifacts together and overwrite the previous failed run.
std::filesystem::path failureArtifactDirectory() {
  const char *configured = std::getenv("SCALEPNR_FAILURE_ARTIFACT_DIR");
  std::filesystem::path directory =
      configured && configured[0] != '\0' ? configured : ".";
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    PNR_LOG1("ROUT",
             "cannot create failure artifact directory '{}': {}; using cwd",
             directory.string(), error.message());
    return ".";
  }
  return directory;
}

// A new run must not expose a prior run's failure as its own result.
void clearFailureArtifacts() {
  const std::filesystem::path directory = failureArtifactDirectory();
  std::error_code error;
  std::filesystem::remove(directory / "routing_failure.png", error);
  error.clear();
  std::filesystem::remove(directory / "routing_failure.txt", error);
  error.clear();
  std::filesystem::remove(directory / "routing_failure_backward.txt", error);
  std::filesystem::remove(directory / "routing_failure_packing.txt", error);
}

size_t statDepthBucket(int depth) {
  if (depth < 0) {
    return 0;
  }
  size_t bucket = static_cast<size_t>(depth);
  return std::min(bucket, RouteDesign::RouteStats::max_depth - 1);
}

std::string statArray(
    const std::array<size_t, RouteDesign::RouteStats::max_depth> &values) {
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      out << ',';
    }
    out << values[i];
  }
  out << ']';
  return out.str();
}

std::string maskString(NodeMask value) { return value.str(); }

struct RouteStageReport {
  bool started = false;
  bool timed_out = false;
  bool stagnated = false;
  size_t passes = 0;
  size_t start_tasks = 0;
  size_t remaining_tasks = 0;
  size_t attempted = 0;
  size_t completed = 0;
  size_t active = 0;
  size_t advanced = 0;
  size_t changed = 0;
  size_t searches = 0;
  size_t pops = 0;
  size_t edge_trials = 0;
  size_t edge_accepted = 0;
  size_t reject_busy = 0;
  size_t reject_target = 0;
  size_t reject_deadend = 0;
  size_t preempt_attempts = 0;
  size_t preempt_success = 0;
  size_t preempt_complete_victims = 0;
  size_t preempt_partial_victims = 0;
  size_t preempt_removed_fragments = 0;
  size_t preempt_takeoff_complete_victims = 0;
  size_t preempt_takeoff_partial_victims = 0;
  size_t preempt_bridge_complete_victims = 0;
  size_t preempt_bridge_partial_victims = 0;
  size_t preempt_grounding_complete_victims = 0;
  size_t preempt_grounding_partial_victims = 0;
  size_t deadend_marks = 0;
  double seconds = 0.0;
};

constexpr size_t CLOCK_STAGE_INDEX = 0;
constexpr size_t CONSTANT_STAGE_INDEX = 1;
constexpr size_t BASIC_STAGE_INDEX = 2;
constexpr size_t MOVING_SOURCES_STAGE_INDEX = 3;
constexpr size_t FANOUT_STAGE_INDEX = 4;
constexpr size_t MOVING_DESTINATIONS_STAGE_INDEX = 5;
constexpr size_t ROUTE_STAGE_COUNT = 6;

const char *routeStageName(size_t stage) {
  static constexpr const char *names[] = {
      "Clock routing", "Const routing", "Basic routing", "Moving sources",
      "Fanouts routing", "Moving destinations"};
  PNR_ASSERT(stage < ROUTE_STAGE_COUNT, "invalid routing stage {}", stage);
  return names[stage];
}

std::pair<const rtl::Inst *, std::string>
physicalSourceEndpoint(const rtl::Inst *inst, std::string port) {
  // Generated source passthroughs are aliases of the upstream driver pin. Walk
  // that tile-local chain so every logical name indexes one physical tree.
  rtl::Inst *current = const_cast<rtl::Inst *>(inst);
  for (size_t depth = 0; current && current->cell_ref.peer && depth < 64;
       ++depth) {
    auto kind = current->cell_ref->attributes.find("scalepnr_passthrough");
    if (kind == current->cell_ref->attributes.end() || kind->second != "source") {
      break;
    }
    rtl::Conn *input = nullptr;
    for (rtl::Conn &conn : current->conns) {
      if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_IN) {
        input = &conn;
        break;
      }
    }
    rtl::Conn *driver = input ? input->follow() : nullptr;
    if (!driver || !driver->inst_ref.peer || !driver->port_ref.peer) {
      break;
    }
    if (driver->inst_ref.peer == current) {
      break;
    }
    current = driver->inst_ref.peer;
    port = driver->port_ref->makeName();
  }
  return {current, std::move(port)};
}

// Build the scheduler key from the upstream physical source endpoint.
// Pointer identity is process-local and used only while this design is open.
std::string sourceRouteKey(const rtl::Inst *inst, const std::string &port) {
  if (!inst || port.empty()) {
    return {};
  }
  auto [physical_inst, physical_port] = physicalSourceEndpoint(inst, port);
  return std::format("{}:{}", reinterpret_cast<uintptr_t>(physical_inst),
                     physical_port);
}

// Compare canonical physical driver pins rather than logical net aliases.
// Missing endpoints never form a valid shared source tree.
bool samePhysicalSource(const rtl::Inst *left, const std::string &left_port,
                        const rtl::Inst *right,
                        const std::string &right_port) {
  const std::string left_key = sourceRouteKey(left, left_port);
  return !left_key.empty() && left_key == sourceRouteKey(right, right_port);
}

bool preemptionOwnerIsProtected(const RouteDesign *router,
                                const rtl::NetRouteBinding &binding) {
  return router && pnr::preemptionOwnerIsTransactionProtected(
                       router->preemption_protected_source_keys,
                       sourceRouteKey(binding.from, binding.from_port));
}

std::string stableInstName(rtl::Inst *inst) {
  return inst ? inst->makeName(std::numeric_limits<size_t>::max())
              : std::string{};
}

enum class MoveElementClass {
  other,
  lut,
  mux7,
  mux8,
  fd,
};

MoveElementClass moveElementClass(rtl::Inst *inst) {
  if (!inst || !inst->cell_ref.peer) {
    return MoveElementClass::other;
  }
  const std::string &type = inst->cell_ref->type;
  if (type.find("MUXF8") == 0) {
    return MoveElementClass::mux8;
  }
  if (type.find("MUX") == 0) {
    return MoveElementClass::mux7;
  }
  if (type.find("FD") == 0) {
    return MoveElementClass::fd;
  }
  if (type.find("LUT") == 0) {
    return MoveElementClass::lut;
  }
  return MoveElementClass::other;
}

int moveElementColumn(MoveElementClass type) {
  switch (type) {
  case MoveElementClass::lut:
    return 0;
  case MoveElementClass::mux7:
    return 1;
  case MoveElementClass::mux8:
    return 2;
  case MoveElementClass::fd:
    return 3;
  default:
    return 3;
  }
}

bool strictMoveChainInput(MoveElementClass driver, MoveElementClass sink,
                          const rtl::Port *sink_port) {
  if (!sink_port) {
    return false;
  }
  if ((driver == MoveElementClass::mux7 || driver == MoveElementClass::mux8) &&
      sink == MoveElementClass::fd) {
    return sink_port->name == "D";
  }
  if (sink_port->name != "I0" && sink_port->name != "I1") {
    return false;
  }
  return (driver == MoveElementClass::lut && sink == MoveElementClass::mux7) ||
         (driver == MoveElementClass::mux7 && sink == MoveElementClass::mux8);
}

void appendUniqueInst(std::vector<rtl::Inst *> &insts, rtl::Inst *inst) {
  if (!inst) {
    return;
  }
  if (std::find(insts.begin(), insts.end(), inst) == insts.end()) {
    insts.push_back(inst);
  }
}

bool sameInstOrder(const std::vector<rtl::Inst *> &a,
                   const std::vector<rtl::Inst *> &b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t index = 0; index < a.size(); ++index) {
    if (a[index] != b[index]) {
      return false;
    }
  }
  return true;
}

void appendUniqueOrder(std::vector<std::vector<rtl::Inst *>> &orders,
                       std::vector<rtl::Inst *> order) {
  if (order.empty()) {
    return;
  }
  for (const std::vector<rtl::Inst *> &old : orders) {
    if (sameInstOrder(old, order)) {
      return;
    }
  }
  orders.push_back(std::move(order));
}

bool isMovePassthroughInst(rtl::Inst *inst) {
  if (!inst || !inst->cell_ref.peer) {
    return false;
  }
  auto it = inst->cell_ref->attributes.find("scalepnr_passthrough");
  return it != inst->cell_ref->attributes.end() &&
         (it->second == "source" || it->second == "target");
}

bool clusterContainsInst(const std::vector<rtl::Inst *> &insts,
                         rtl::Inst *inst) {
  return inst && std::find(insts.begin(), insts.end(), inst) != insts.end();
}

bool passthroughMovesWithCluster(rtl::Inst &pass,
                                 const std::vector<rtl::Inst *> &cluster) {
  if (!isMovePassthroughInst(&pass)) {
    return false;
  }
  const std::string &kind =
      pass.cell_ref->attributes.at("scalepnr_passthrough");
  if (kind == "source") {
    for (rtl::Conn &conn : pass.conns) {
      if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
        continue;
      }
      rtl::Conn *driver = conn.follow();
      if (driver && clusterContainsInst(cluster, driver->inst_ref.peer)) {
        return true;
      }
    }
  }
  if (kind == "target") {
    for (rtl::Conn &conn : pass.conns) {
      if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT ||
          conn.peer) {
        continue;
      }
      for (auto *sink_ref : rtl::Conn::getSinks(conn)) {
        rtl::Conn *sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (sink && clusterContainsInst(cluster, sink->inst_ref.peer)) {
          return true;
        }
      }
    }
  }
  return false;
}

void appendAttachedPassthroughs(std::vector<rtl::Inst *> &cluster) {
  bool changed = true;
  while (changed && cluster.size() < 64) {
    changed = false;
    std::vector<rtl::Inst *> snapshot = cluster;
    for (rtl::Inst *inst : snapshot) {
      if (!inst) {
        continue;
      }
      for (rtl::Conn &conn : inst->conns) {
        if (!conn.port_ref.peer) {
          continue;
        }
        std::vector<rtl::Inst *> candidates;
        if (conn.port_ref->type == rtl::Port::PORT_IN) {
          rtl::Conn *driver = conn.follow();
          appendUniqueInst(candidates,
                           driver ? driver->inst_ref.peer : nullptr);
        } else if (conn.port_ref->type == rtl::Port::PORT_OUT && !conn.peer) {
          for (auto *sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn *sink =
                sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            appendUniqueInst(candidates, sink ? sink->inst_ref.peer : nullptr);
          }
        }
        for (rtl::Inst *candidate : candidates) {
          if (!isMovePassthroughInst(candidate) ||
              clusterContainsInst(cluster, candidate) ||
              !passthroughMovesWithCluster(*candidate, cluster)) {
            continue;
          }
          appendUniqueInst(cluster, candidate);
          changed = true;
        }
      }
    }
  }
}

std::vector<rtl::Inst *>
strictClusterDrivers(rtl::Inst *inst, const std::vector<rtl::Inst *> &cluster) {
  std::vector<rtl::Inst *> drivers;
  if (!inst) {
    return drivers;
  }
  MoveElementClass sink_type = moveElementClass(inst);
  for (rtl::Conn &input : inst->conns) {
    if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
      continue;
    }
    rtl::Conn *driver_conn = input.follow();
    rtl::Inst *driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
    if (!clusterContainsInst(cluster, driver) ||
        !strictMoveChainInput(moveElementClass(driver), sink_type,
                              input.port_ref.peer)) {
      continue;
    }
    appendUniqueInst(drivers, driver);
  }
  return drivers;
}

std::vector<rtl::Inst *>
strictClusterSinks(rtl::Inst *inst, const std::vector<rtl::Inst *> &cluster) {
  std::vector<rtl::Inst *> sinks;
  if (!inst) {
    return sinks;
  }
  MoveElementClass driver_type = moveElementClass(inst);
  for (rtl::Conn &output : inst->conns) {
    if (!output.port_ref.peer || output.port_ref->type != rtl::Port::PORT_OUT ||
        output.peer) {
      continue;
    }
    for (auto *sink_ref : rtl::Conn::getSinks(output)) {
      rtl::Conn *sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
      rtl::Inst *sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
      if (!clusterContainsInst(cluster, sink) ||
          !strictMoveChainInput(driver_type, moveElementClass(sink),
                                sink_conn ? sink_conn->port_ref.peer
                                          : nullptr)) {
        continue;
      }
      appendUniqueInst(sinks, sink);
    }
  }
  return sinks;
}

void appendClusterSinkFirst(rtl::Inst *inst,
                            const std::vector<rtl::Inst *> &cluster,
                            std::vector<rtl::Inst *> &order) {
  if (!clusterContainsInst(cluster, inst) || clusterContainsInst(order, inst)) {
    return;
  }
  appendUniqueInst(order, inst);
  for (rtl::Inst *driver : strictClusterDrivers(inst, cluster)) {
    appendClusterSinkFirst(driver, cluster, order);
  }
}

void appendClusterDriverFirst(rtl::Inst *inst,
                              const std::vector<rtl::Inst *> &cluster,
                              std::vector<rtl::Inst *> &order) {
  if (!clusterContainsInst(cluster, inst) || clusterContainsInst(order, inst)) {
    return;
  }
  for (rtl::Inst *driver : strictClusterDrivers(inst, cluster)) {
    appendClusterDriverFirst(driver, cluster, order);
  }
  appendUniqueInst(order, inst);
}

std::vector<std::vector<rtl::Inst *>>
clusterPlacementOrders(const std::vector<rtl::Inst *> &cluster) {
  std::vector<std::vector<rtl::Inst *>> orders;
  appendUniqueOrder(orders, cluster);
  if (cluster.size() > 1) {
    std::vector<rtl::Inst *> reversed = cluster;
    std::reverse(reversed.begin(), reversed.end());
    appendUniqueOrder(orders, std::move(reversed));
  }

  std::vector<rtl::Inst *> sink_first;
  for (rtl::Inst *inst : cluster) {
    if (strictClusterSinks(inst, cluster).empty()) {
      appendClusterSinkFirst(inst, cluster, sink_first);
    }
  }
  for (rtl::Inst *inst : cluster) {
    appendClusterSinkFirst(inst, cluster, sink_first);
  }
  appendUniqueOrder(orders, std::move(sink_first));

  std::vector<rtl::Inst *> driver_first;
  for (rtl::Inst *inst : cluster) {
    if (strictClusterDrivers(inst, cluster).empty()) {
      appendClusterDriverFirst(inst, cluster, driver_first);
    }
  }
  for (rtl::Inst *inst : cluster) {
    appendClusterDriverFirst(inst, cluster, driver_first);
  }
  appendUniqueOrder(orders, std::move(driver_first));
  return orders;
}

std::vector<rtl::Inst *> strictMoveCluster(rtl::Inst *seed) {
  std::vector<rtl::Inst *> cluster;
  std::vector<rtl::Inst *> stack;
  fpga::Tile *seed_tile = seed && seed->tile.peer ? &*seed->tile : nullptr;
  appendUniqueInst(cluster, seed);
  appendUniqueInst(stack, seed);
  while (!stack.empty() && cluster.size() < 32) {
    rtl::Inst *inst = stack.back();
    stack.pop_back();
    MoveElementClass inst_type = moveElementClass(inst);
    if (inst_type == MoveElementClass::other) {
      continue;
    }
    for (rtl::Conn &input : inst->conns) {
      if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
        continue;
      }
      rtl::Conn *driver_conn = input.follow();
      rtl::Inst *driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
      if (!driver || !driver->cell_ref.peer) {
        continue;
      }
      if (!seed_tile || !driver->tile.peer || &*driver->tile != seed_tile) {
        continue;
      }
      if (!strictMoveChainInput(moveElementClass(driver), inst_type,
                                input.port_ref.peer)) {
        continue;
      }
      size_t old_size = cluster.size();
      appendUniqueInst(cluster, driver);
      if (cluster.size() != old_size) {
        stack.push_back(driver);
      }
    }
    for (rtl::Conn &output : inst->conns) {
      if (!output.port_ref.peer ||
          output.port_ref->type != rtl::Port::PORT_OUT || output.peer) {
        continue;
      }
      for (auto *sink_ref : rtl::Conn::getSinks(output)) {
        rtl::Conn *sink_conn =
            sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        rtl::Inst *sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
        if (!sink || !sink->cell_ref.peer) {
          continue;
        }
        if (!seed_tile || !sink->tile.peer || &*sink->tile != seed_tile) {
          continue;
        }
        if (!strictMoveChainInput(inst_type, moveElementClass(sink),
                                  sink_conn ? sink_conn->port_ref.peer
                                            : nullptr)) {
          continue;
        }
        size_t old_size = cluster.size();
        appendUniqueInst(cluster, sink);
        if (cluster.size() != old_size) {
          stack.push_back(sink);
        }
      }
    }
  }
  appendAttachedPassthroughs(cluster);
  std::sort(cluster.begin(), cluster.end(), [](rtl::Inst *a, rtl::Inst *b) {
    int a_col = moveElementColumn(moveElementClass(a));
    int b_col = moveElementColumn(moveElementClass(b));
    if (a_col != b_col) {
      return a_col > b_col;
    }
    return stableInstName(a) < stableInstName(b);
  });
  return cluster;
}

// Resolve the current canonical owner because passthrough insertion can change
// strict packing-cluster membership while Moving routing is active.
uintptr_t movingClusterKey(rtl::Inst *inst) {
  if (!inst) {
    return 0;
  }
  std::vector<rtl::Inst *> cluster = strictMoveCluster(inst);
  rtl::Inst *owner = pnr::movingClusterOwner(
      cluster, inst, [](rtl::Inst *left, rtl::Inst *right) {
        return stableInstName(left) < stableInstName(right);
      });
  return reinterpret_cast<uintptr_t>(owner);
}

// Store every completed cluster member directly so later endpoint checks are
// cheap; route-tree invalidation explicitly clears these marks when needed.
bool movingInstIsFinished(const std::unordered_set<uintptr_t> &finished,
                          const rtl::Inst *inst) {
  return inst && finished.contains(reinterpret_cast<uintptr_t>(inst));
}

// Mark all members because later route tasks can reference any element in the
// strict packing cluster, not only its canonical placement-history owner.
void markMovingClusterFinished(std::unordered_set<uintptr_t> &finished,
                               rtl::Inst *inst) {
  std::vector<rtl::Inst *> cluster = strictMoveCluster(inst);
  if (cluster.empty() && inst) {
    cluster.push_back(inst);
  }
  for (rtl::Inst *member : cluster) {
    if (member) {
      finished.insert(reinterpret_cast<uintptr_t>(member));
    }
  }
}

// Invalidate every member mark when later routing work breaks a completed
// cluster.
void unmarkMovingClusterFinished(std::unordered_set<uintptr_t> &finished,
                                 rtl::Inst *inst) {
  // Reconstruct the same strict cluster that was marked after successful
  // Moving.
  std::vector<rtl::Inst *> cluster = strictMoveCluster(inst);
  // A standalone instance is its own cluster.
  if (cluster.empty() && inst) {
    // Include the focus so its stale finished mark is removed.
    cluster.push_back(inst);
  }
  // Clear every member because any member can appear in a later route task.
  for (rtl::Inst *member : cluster) {
    // Ignore absent optional members.
    if (member) {
      // Permit the invalidated cluster member to be selected by Moving again.
      finished.erase(reinterpret_cast<uintptr_t>(member));
    }
  }
}

size_t countSetBits(NodeMask value) {
  size_t count = 0;
  value.for_each_set_bit([&](int) {
    ++count;
    return false;
  });
  return count;
}

int debugDecodeSigned4(int value) {
  value &= 0xf;
  return (value & 0x8) ? value - 16 : value;
}

int debugJumpDeltaX(int jump) { return debugDecodeSigned4((jump >> 8) & 0xf); }

int debugJumpDeltaY(int jump) { return debugDecodeSigned4((jump >> 4) & 0xf); }

size_t
countDeadendBits(const std::unordered_map<uint64_t, NodeMask> &deadends) {
  size_t count = 0;
  for (const auto &entry : deadends) {
    count += countSetBits(entry.second);
  }
  return count;
}

struct RouteSearchReport {
  bool start_expanded = false;
  bool start_accepted = false;

  bool firstNodeBlocked() const { return start_expanded && !start_accepted; }
};

double elapsedSeconds(std::chrono::steady_clock::time_point start,
                      std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double>(end - start).count();
}

bool routeHeartbeatEnabled() {
  const char *value = std::getenv("SCALEPNR_ROUTE_HEARTBEAT");
  return !value || !*value || std::string(value) != "0";
}

int routeHeartbeatSeconds() {
  const char *value = std::getenv("SCALEPNR_ROUTE_HEARTBEAT_SEC");
  if (!value || !*value) {
    return 60;
  }
  char *end = nullptr;
  long seconds = std::strtol(value, &end, 10);
  if (end == value || seconds <= 0 || seconds > 3600) {
    return 60;
  }
  return static_cast<int>(seconds);
}

bool routeDebugMatches(const char *env_name, const std::string &text) {
  const char *value = std::getenv(env_name);
  if (!value || !*value) {
    return false;
  }
  std::string_view filters{value};
  if (filters == "*") {
    return true;
  }
  while (!filters.empty()) {
    size_t separator = filters.find_first_of(";\n");
    std::string_view filter = filters.substr(0, separator);
    if (!filter.empty() && text.find(filter) != std::string::npos) {
      return true;
    }
    if (separator == std::string_view::npos) {
      break;
    }
    filters.remove_prefix(separator + 1);
  }
  return false;
}

bool routeDebugEnabled(const char *env_name) {
  const char *value = std::getenv(env_name);
  return value && *value;
}

int routeDebugLineLimit() {
  const char *value = std::getenv("SCALEPNR_DEBUG_ROUTE_LINES");
  if (!value || !*value) {
    return 2000;
  }
  char *end = nullptr;
  long limit = std::strtol(value, &end, 10);
  if (end == value || limit <= 0 || limit > 100000) {
    return 2000;
  }
  return static_cast<int>(limit);
}

int movePathDebugLineLimit() {
  const char *value = std::getenv("SCALEPNR_DEBUG_MOVE_PATH_LINES");
  if (!value || !*value) {
    return 100000;
  }
  char *end = nullptr;
  long limit = std::strtol(value, &end, 10);
  if (end == value || limit <= 0 || limit > 1000000) {
    return 100000;
  }
  return static_cast<int>(limit);
}

bool sameCoord(const Coord &a, const Coord &b) {
  return a.x == b.x && a.y == b.y;
}

rtl::Inst *instFromTileRef(RefBase<Referable<Tile>> *ref) {
  auto *tile_ref = Ref<Tile>::fromBase(ref);
  return reinterpret_cast<rtl::Inst *>(reinterpret_cast<char *>(tile_ref) -
                                       offsetof(rtl::Inst, tile));
}

rtl::Inst *tileInstAtPos(Tile &tile, int pos) {
  auto &referable_tile = static_cast<Referable<Tile> &>(tile);
  for (auto *peer : referable_tile.getPeers()) {
    if (!peer) {
      continue;
    }

    rtl::Inst *candidate = instFromTileRef(peer);
    if (!candidate || candidate->pos != pos ||
        candidate->tile.peer != &referable_tile) {
      continue;
    }

    return candidate;
  }
  return nullptr;
}

bool hasLocalNodeUse(const fpga::CBType &cb_type, int local) {
  if (local < 0 || local >= CB_MAX_NODES) {
    return false;
  }
  auto not_empty = [](NodeMask value) { return value != NodeMask{}; };
  return cb_type.nodeName(fpga::CB_NODE_LOCAL, local) ||
         not_empty(cb_type.local_src[local].jump) ||
         not_empty(cb_type.local_joint[local].joint) ||
         not_empty(cb_type.local_local[local].local) ||
         not_empty(cb_type.dst_local[local].local);
}

bool anyDstCanReachLocal(const fpga::CBType &cb_type, int local) {
  if (local < 0 || local >= CB_MAX_NODES) {
    return false;
  }
  return cb_type.dsts_reaching_local[local].jump != NodeMask{};
}

bool isConcreteRouteTile(const Tile &tile) { return tile.cb_type != nullptr; }

bool supportsLocalNodes(const Tile &tile, NodeMask nodes) {
  if (!isConcreteRouteTile(tile)) {
    return false;
  }
  if (nodes == NodeMask{}) {
    return true;
  }
  return nodes.for_each_set_bit([&](int local) {
    tile.cb_type->ensureDerivedMasks();
    return anyDstCanReachLocal(*tile.cb_type, local);
  });
}

bool isRoutableOutputLocal(const Tile &tile, int local) {
  return tile.cb_type &&
         tile.cb_type->srcNodes(fpga::CB_NODE_LOCAL, local) != nullptr;
}

bool sameCrossbarType(const Tile &a, const Tile &b) {
  if (!a.cb_type || !b.cb_type) {
    return false;
  }
  return a.cb_type == b.cb_type || a.cb_type->name == b.cb_type->name;
}

bool supportsOutputLocalNodes(const Tile &tile, NodeMask nodes) {
  if (!isConcreteRouteTile(tile)) {
    return false;
  }
  if (nodes == NodeMask{}) {
    return true;
  }
  return nodes.for_each_set_bit(
      [&](int local) { return isRoutableOutputLocal(tile, local); });
}

bool isIoBuffer(rtl::Inst &inst);

NodeMask routeTileEndpointNodes(const Tile &route_tile, rtl::Inst &inst,
                                const std::string &port, bool output) {
  if (!inst.tile.peer || !inst.cell_ref.peer || !route_tile.cb_type) {
    return {};
  }
  Coord endpoint_cb_coord =
      inst.tile->cb_coord.x >= 0 && inst.tile->cb_coord.y >= 0
          ? inst.tile->cb_coord
          : inst.tile->coord;
  bool canonical_cb = endpoint_cb_coord.x == route_tile.coord.x &&
                      endpoint_cb_coord.y == route_tile.coord.y;
  if (!canonical_cb && !isIoBuffer(inst)) {
    return {};
  }
  fpga::TilePinNameType dir =
      output ? fpga::TILE_PIN_OUTPUT : fpga::TILE_PIN_INPUT;
  NodeMask endpoint_nodes{};
  if (canonical_cb && inst.tile->tile_type &&
      !inst.tile->tile_type->name.empty()) {
    endpoint_nodes = inst.tile->getPinNodesForRouteType(
        inst.cell_ref->type, port, inst.pos, dir, inst.tile->tile_type->name,
        Coord{});
  }
  bool supported = output ? supportsOutputLocalNodes(route_tile, endpoint_nodes)
                          : supportsLocalNodes(route_tile, endpoint_nodes);
  if (endpoint_nodes != NodeMask{} && supported) {
    return endpoint_nodes;
  }
  if (route_tile.tile_type && !route_tile.tile_type->name.empty()) {
    Coord route_delta = route_tile.coord - inst.tile->coord;
    endpoint_nodes = inst.tile->getPinNodesForRouteType(
        inst.cell_ref->type, port, inst.pos, dir, route_tile.tile_type->name,
        route_delta);
  }
  if (endpoint_nodes == NodeMask{}) {
    Coord route_delta = route_tile.cb_coord.x >= 0 && route_tile.cb_coord.y >= 0
                            ? route_tile.cb_coord - inst.tile->coord
                            : route_tile.coord - inst.tile->coord;
    endpoint_nodes = inst.tile->getPinNodesForRouteType(
        inst.cell_ref->type, port, inst.pos, dir, route_tile.cb_type->name,
        route_delta);
  }
  supported = output ? supportsOutputLocalNodes(route_tile, endpoint_nodes)
                     : supportsLocalNodes(route_tile, endpoint_nodes);
  if (endpoint_nodes != NodeMask{} && supported) {
    return endpoint_nodes;
  }

  // Exact route refs restrict unrelated candidates. The raw local belongs to
  // the one physical crossbar selected by the resource tile's cb_coord.
  if (!canonical_cb) {
    return {};
  }
  endpoint_nodes =
      output ? inst.tile->getOutputPinNodes(inst.cell_ref->type, port, inst.pos)
             : inst.tile->getPinNodes(inst.cell_ref->type, port, inst.pos);
  supported = output ? supportsOutputLocalNodes(route_tile, endpoint_nodes)
                     : supportsLocalNodes(route_tile, endpoint_nodes);
  if (endpoint_nodes != NodeMask{} && supported) {
    return endpoint_nodes;
  }

  return supported ? endpoint_nodes : NodeMask{};
}

NodeMask routeTileInputNodes(const Tile &route_tile, rtl::Inst &inst,
                             const std::string &port) {
  if (!inst.tile.peer || !inst.cell_ref.peer) {
    return {};
  }
  return routeTileEndpointNodes(route_tile, inst, port, false);
}

enum class TerminalEntryKind {
  none,
  dst,
  local,
};

struct TerminalEntryCandidate {
  TerminalEntryKind kind = TerminalEntryKind::none;
  int joint = -1;
  int joint2 = -1;
};

bool leaseConcreteTerminal(CBState &cb, int node, int pin,
                           const TerminalEntryCandidate &entry,
                           bool allow_existing_dst = false);
std::vector<TerminalEntryCandidate> targetEntryCandidates(const CBType &cb_type,
                                                          int node, int local);

bool targetDstCanEnterPin(const Tile &tile, int dst_node, NodeMask pin_nodes,
                          bool allow_existing_dst = false) {
  if (!tile.cb_type || dst_node < 0 || dst_node >= CB_MAX_NODES ||
      pin_nodes == NodeMask{}) {
    return false;
  }
  tile.cb_type->ensureDerivedMasks();
  return pin_nodes.for_each_set_bit([&](int pin) {
    if (tile.isPinNodeLeased(pin)) {
      return false;
    }
    for (const TerminalEntryCandidate &entry :
         targetEntryCandidates(*tile.cb_type, dst_node, pin)) {
      CBState test_cb = tile.cb;
      if (leaseConcreteTerminal(test_cb, dst_node, pin, entry,
                                allow_existing_dst)) {
        return true;
      }
    }
    return false;
  });
}

// Checks only crossbar topology; current leases are handled by terminal leasing
// and grounding preemption.
bool targetDstHasEntryPath(const Tile &tile, int dst_node, NodeMask pin_nodes) {
  if (!tile.cb_type || dst_node < 0 || dst_node >= CB_MAX_NODES ||
      pin_nodes == NodeMask{}) {
    return false;
  }
  return pin_nodes.for_each_set_bit([&](int pin) {
    return !targetEntryCandidates(*tile.cb_type, dst_node, pin).empty();
  });
}

// Reject a packed endpoint only when every physical terminal path consumes a
// joint already required by another input in the same route tile.
bool targetPinsHaveUnreservedEntryPath(Tile &tile, NodeMask pin_nodes,
                                       NodeMask reserved_joints) {
  if (!tile.cb_type || pin_nodes == NodeMask{}) {
    return false;
  }
  bool restrict_to_physical_dsts = tile.incoming_dst_nodes != NodeMask{};
  return pin_nodes.for_each_set_bit([&](int pin) {
    for (const CBType::TerminalEntry &entry :
         tile.cb_type->terminalEntries(pin)) {
      if (restrict_to_physical_dsts &&
          !tile.incoming_dst_nodes.testBit(entry.dst)) {
        continue;
      }
      if (pnr::terminalEntryAvoidsReservedJoints(
              entry.joint, entry.joint2, reserved_joints)) {
        return true;
      }
    }
    return false;
  });
}

bool isInputOnlyLocal(const Tile &tile, int local) {
  if (!tile.cb_type || local < 0 || local >= CB_MAX_NODES) {
    return false;
  }
  NodeMask bit = NodeMask{0, 1} << local;
  return (tile.cb_type->local_input_nodes & bit) != NodeMask{} &&
         (tile.cb_type->local_output_nodes & bit) == NodeMask{};
}

int routeDistance(const Coord &a, const Coord &b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool isIOBEndpoint(const rtl::Inst &inst) {
  return inst.cell_ref.peer && (inst.cell_ref.peer->type == "IBUF" ||
                                inst.cell_ref.peer->type == "OBUF");
}

int actualJumpId(int curr, int orig_curr) { return curr; }

bool routeIsComplete(const std::vector<Wire> &route) {
  return fpga::isRouteComplete(route);
}

size_t routeCrossbarFragments(const std::vector<Wire> *route) {
  if (!route) {
    return 0;
  }
  return std::count_if(route->begin(), route->end(), [](const Wire &wire) {
    return wire.type == Wire::WIRE_CROSSBAR;
  });
}

size_t routeUnsharedCrossbarFragments(const std::vector<Wire> *route) {
  if (!route) {
    return 0;
  }
  return std::count_if(route->begin(), route->end(), [](const Wire &wire) {
    return wire.type == Wire::WIRE_CROSSBAR && !wire.shared;
  });
}

std::vector<Wire> *findRoute(rtl::Inst &inst, const std::string &net_name) {
  for (auto &route : inst.wires) {
    if (!route.empty() && route.front().net_name == net_name) {
      return &route;
    }
  }
  return nullptr;
}

const std::vector<Wire> *findRoute(const rtl::Inst &inst,
                                   const std::string &net_name) {
  for (const auto &route : inst.wires) {
    if (!route.empty() && route.front().net_name == net_name) {
      return &route;
    }
  }
  return nullptr;
}

std::vector<Wire> *findBoundRoute(rtl::Net *net, rtl::Inst *from, rtl::Inst *to,
                                  const std::string &from_port,
                                  const std::string &to_port,
                                  const std::string &route_name) {
  if (!net) {
    return nullptr;
  }
  rtl::NetRouteLookup lookup =
      net->findRouteBinding(from, to, from_port, to_port, route_name);
  if (lookup.index == std::numeric_limits<size_t>::max()) {
    return nullptr;
  }
  if (lookup.count == 1) {
    rtl::NetRouteBinding &binding = net->routes[lookup.index];
    if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
      return nullptr;
    }
    return &binding.owner->wires[binding.route_index];
  }
  std::vector<Wire> *complete_match = nullptr;
  for (rtl::NetRouteBinding &binding : net->routes) {
    if (binding.route_name != route_name || binding.from != from ||
        binding.to != to || binding.from_port != from_port ||
        binding.to_port != to_port || !binding.owner ||
        binding.route_index >= binding.owner->wires.size()) {
      continue;
    }
    std::vector<Wire> *route = &binding.owner->wires[binding.route_index];
    // Retargeted passthrough bindings can temporarily share endpoint identity.
    // Finish every incomplete physical binding before accepting a complete
    // twin.
    if (!routeIsComplete(*route)) {
      return route;
    }
    if (!complete_match) {
      complete_match = route;
    }
  }
  return complete_match;
}

const std::vector<Wire> *
findBoundRoute(const rtl::Net *net, const rtl::Inst *from, const rtl::Inst *to,
               const std::string &from_port, const std::string &to_port,
               const std::string &route_name) {
  return findBoundRoute(
      const_cast<rtl::Net *>(net), const_cast<rtl::Inst *>(from),
      const_cast<rtl::Inst *>(to), from_port, to_port, route_name);
}

size_t findRouteIndex(const rtl::Inst &inst, const std::vector<Wire> *route) {
  for (size_t i = 0; i < inst.wires.size(); ++i) {
    if (&inst.wires[i] == route) {
      return i;
    }
  }
  return std::numeric_limits<size_t>::max();
}

size_t findNetRouteBindingIndex(const rtl::Net &net, const rtl::Inst &owner,
                                size_t owner_route_index) {
  for (size_t i = 0; i < net.routes.size(); ++i) {
    const rtl::NetRouteBinding &binding = net.routes[i];
    if (binding.owner == &owner && binding.route_index == owner_route_index) {
      return i;
    }
  }
  return std::numeric_limits<size_t>::max();
}

rtl::NetRouteBinding *findNetRouteBinding(rtl::Net &net, const rtl::Inst &from,
                                          const rtl::Inst &to,
                                          const std::string &from_port,
                                          const std::string &to_port,
                                          const std::string &route_name) {
  rtl::NetRouteLookup lookup = net.findRouteBinding(
      &from, &to, from_port, to_port, route_name);
  if (lookup.index != std::numeric_limits<size_t>::max()) {
    return &net.routes[lookup.index];
  }
  return nullptr;
}

struct ModuleNetDesignatorIndex {
  size_t net_count = std::numeric_limits<size_t>::max();
  std::unordered_map<int, rtl::Net *> nets;
};

std::unordered_map<rtl::Module *, ModuleNetDesignatorIndex>
    module_net_designators;

// Discard cached module-net pointers before a new routing transaction can
// reserve or append nets and invalidate their vector storage.
void resetModuleNetDesignatorIndex() { module_net_designators.clear(); }

// Resolve a numeric connection designator through a lazily rebuilt per-module
// index so focused routing never scans every net in a large design.
rtl::Net *findNetByDesignator(rtl::Inst &inst, int designator) {
  if (!inst.cell_ref.peer || !inst.cell_ref->module_ref.peer) {
    return nullptr;
  }
  rtl::Module *parent = inst.cell_ref->module_ref->parent_ref.peer;
  if (!parent) {
    return nullptr;
  }
  ModuleNetDesignatorIndex &index = module_net_designators[parent];
  if (index.net_count != parent->nets.size()) {
    index.nets.clear();
    for (auto &net : parent->nets) {
      for (int net_designator : net.designators) {
        index.nets.emplace(net_designator, &net);
      }
    }
    index.net_count = parent->nets.size();
  }
  auto found = index.nets.find(designator);
  return found == index.nets.end() ? nullptr : found->second;
}

bool netEndpointIsVoid(const rtl::Net &net, rtl::Inst *endpoint,
                       const std::string &port) {
  if (net.void_net) {
    return true;
  }
  if (!endpoint) {
    return false;
  }
  for (rtl::Conn &conn : endpoint->conns) {
    if (conn.port_ref.peer && conn.port_ref->makeName() == port) {
      return net.designatorIsVoid(conn.port_ref->designator);
    }
  }
  return false;
}

bool netBindingIsVoid(const rtl::Net &net,
                      const rtl::NetRouteBinding &binding) {
  return netEndpointIsVoid(net, binding.to, binding.to_port);
}

struct TransitVictim {
  rtl::Net *net = nullptr;
  size_t binding_index = 0;
  bool node_shared = false;
  Coord conflict_tile{-1, -1};
  fpga::CBNodeNameType conflict_type = fpga::CB_NODE_JUMP;
  int conflict_node = -1;
};

std::vector<Wire> *routeBindingRoute(rtl::NetRouteBinding &binding);
bool routeStartsWithSharedPrefix(const std::vector<Wire> *route);
bool routeHasSharedBranchSuffix(const std::vector<Wire> *route);
bool netHasCompleteRouteExcept(rtl::Net &net, size_t skip_index);
bool hasOtherCompleteSourceBinding(rtl::Inst &inst, const std::string &port,
                                   const rtl::NetRouteBinding *current);

bool bindingFromTile(const rtl::NetRouteBinding &binding, const Tile &tile) {
  return binding.from && binding.from->tile.peer &&
         pnr::endpointRouteTileMatches(binding.from->tile->coord,
                                       binding.from->tile->cb_coord,
                                       tile.coord);
}

bool bindingToTile(const rtl::NetRouteBinding &binding, const Tile &tile) {
  return binding.to && binding.to->tile.peer &&
         pnr::endpointRouteTileMatches(binding.to->tile->coord,
                                       binding.to->tile->cb_coord, tile.coord);
}

bool bindingGroundsInTile(rtl::NetRouteBinding &binding, const Tile &tile) {
  std::vector<Wire> *route = routeBindingRoute(binding);
  return pnr::groundingOwnerProtectsEndpoint(
      bindingToTile(binding, tile), route && routeIsComplete(*route));
}

TransitVictim findTransitDstVictim(Tile &tile, int dst_node, int joint_node,
                                   rtl::Net *current_net,
                                   rtl::Inst *current_source,
                                   const std::string &current_source_port,
                                   const std::string &current_route_name,
                                   const RouteDesign *router = nullptr) {
  for (auto &ref : tile.routedNets) {
    rtl::Net *net = ref.peer;
    if (!net || !net->routeCanBePreempted()) {
      continue;
    }
    for (size_t binding_index = 0; binding_index < net->routes.size();
         ++binding_index) {
      rtl::NetRouteBinding &binding = net->routes[binding_index];
      if ((!current_route_name.empty() &&
           binding.route_name == current_route_name) ||
          pnr::preemptionOwnerIsCurrentTree(
              net == current_net,
              sourceRouteKey(current_source, current_source_port),
              sourceRouteKey(binding.from, binding.from_port)) ||
          preemptionOwnerIsProtected(router, binding)) {
        continue;
      }
      if (bindingGroundsInTile(binding, tile)) {
        continue;
      }
      if (!binding.owner ||
          binding.route_index >= binding.owner->wires.size()) {
        continue;
      }
      const std::vector<Wire> &route =
          binding.owner->wires[binding.route_index];
      for (const Wire &fragment : route) {
        if (fragment.type != Wire::WIRE_CROSSBAR || fragment.pos == 0) {
          continue;
        }
        bool uses_dst = fragment.local == dst_node;
        bool uses_joint = joint_node >= 0 && (fragment.joint == joint_node ||
                                              fragment.joint2 == joint_node);
        if (!uses_dst && !uses_joint) {
          continue;
        }
        bool touches_tile = sameCoord(fragment.from, tile.coord) ||
                            sameCoord(fragment.to, tile.coord);
        bool transit_fragment = !sameCoord(fragment.from, fragment.to);
        if (touches_tile && transit_fragment) {
          fpga::CBNodeNameType conflict_type =
              uses_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_JOINT;
          int conflict_node = uses_dst ? dst_node : joint_node;
          return TransitVictim{net, binding_index, fragment.shared, tile.coord,
                               conflict_type, conflict_node};
        }
      }
    }
  }
  return TransitVictim{};
}

TransitVictim findGroundingVictim(Tile &tile, int dst_node, int joint_node,
                                  rtl::Net *current_net,
                                  rtl::Inst *current_source,
                                  const std::string &current_source_port,
                                  const std::string &current_route_name,
                                  const RouteDesign *router = nullptr) {
  // Grounding may evict only a route passing through this crossbar.
  // Endpoint routes own their selected entry and must remain intact.
  return findTransitDstVictim(tile, dst_node, joint_node, current_net,
                              current_source, current_source_port,
                              current_route_name, router);
}

TransitVictim findTransitSrcVictim(Tile &tile, int src_node,
                                   rtl::Net *current_net,
                                   rtl::Inst *current_source,
                                   const std::string &current_source_port,
                                   const std::string &current_route_name,
                                   const RouteDesign *router = nullptr) {
  for (auto &ref : tile.routedNets) {
    rtl::Net *net = ref.peer;
    if (!net || !net->routeCanBePreempted()) {
      continue;
    }
    for (size_t binding_index = 0; binding_index < net->routes.size();
         ++binding_index) {
      rtl::NetRouteBinding &binding = net->routes[binding_index];
      if ((!current_route_name.empty() &&
           binding.route_name == current_route_name) ||
          pnr::preemptionOwnerIsCurrentTree(
              net == current_net,
              sourceRouteKey(current_source, current_source_port),
              sourceRouteKey(binding.from, binding.from_port)) ||
          preemptionOwnerIsProtected(router, binding)) {
        continue;
      }
      if (bindingFromTile(binding, tile)) {
        continue;
      }
      if (!binding.owner ||
          binding.route_index >= binding.owner->wires.size()) {
        continue;
      }
      const std::vector<Wire> &route =
          binding.owner->wires[binding.route_index];
      for (const Wire &fragment : route) {
        if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump != src_node) {
          continue;
        }
        if (!sameCoord(fragment.from, tile.coord) ||
            sameCoord(fragment.from, fragment.to)) {
          continue;
        }
        if (fragment.pos != 0) {
          return TransitVictim{net, binding_index, fragment.shared, tile.coord,
                               fpga::CB_NODE_SRC, src_node};
        }
      }
    }
  }
  return TransitVictim{};
}

struct DockingBridgeVictim {
  rtl::Net *net = nullptr;
  size_t binding_index = 0;
};

// Compare the stable net/binding identity used while preflighting a bridge.
// This keeps shared-node owner enumeration from scheduling duplicate cuts.
bool sameDockingBridgeVictim(const DockingBridgeVictim &left,
                             const fpga::NetRouteRef &right) {
  return left.net == right.net && left.binding_index == right.binding_index;
}

// Validate every owner before changing state, then detach only suffixes that
// use the exact numeric resources separating the two docking frontiers.
bool preemptDockingBridge(
    RouteDesign *router, const pnr::DockingBridgeBlocker &bridge,
    rtl::Net *current_net, rtl::Inst *current_source,
    const std::string &current_source_port,
    const std::string &current_route_name, RouteDesign::RouteStats *stats,
    bool allow_complete_victim) {
  const bool debug_bridge =
      routeDebugMatches("SCALEPNR_DEBUG_ROUTE_NET", current_route_name);
  auto reject_bridge = [&](const char *reason) {
    if (debug_bridge) {
      PNR_LOG1("ROUT",
               "routeDesign docking bridge reject: current='{}', reason={}, "
               "tile=({},{}), dst={}, src={}, joint={}, joint2={}, "
               "landing=({},{}):{}, busy(dst={},src={},joint={},joint2={})",
               current_route_name, reason, bridge.tile.x, bridge.tile.y,
               bridge.dst, bridge.src, bridge.joint, bridge.joint2,
               bridge.landing_tile.x, bridge.landing_tile.y,
               bridge.landing_dst, bridge.dst_busy, bridge.src_busy,
               bridge.joint_busy, bridge.joint2_busy);
    }
    return false;
  };
  if (!router || !bridge.valid) {
    return reject_bridge("invalid_bridge");
  }
  Tile *tile = fpga::Device::current().getTile(bridge.tile.x, bridge.tile.y);
  if (!tile) {
    return reject_bridge("missing_tile");
  }

  std::vector<fpga::RouteCutNode> cuts;
  auto append_cut = [&](bool busy, fpga::CBNodeNameType type, int node) {
    if (busy && node >= 0) {
      cuts.push_back(fpga::RouteCutNode{bridge.tile, type, node});
    }
  };
  append_cut(bridge.dst_busy, fpga::CB_NODE_DST, bridge.dst);
  append_cut(bridge.src_busy, fpga::CB_NODE_SRC, bridge.src);
  append_cut(bridge.joint_busy, fpga::CB_NODE_JOINT, bridge.joint);
  append_cut(bridge.joint2_busy, fpga::CB_NODE_JOINT, bridge.joint2);
  if (cuts.empty()) {
    return reject_bridge("no_busy_cut");
  }

  std::vector<DockingBridgeVictim> victims;
  for (const fpga::RouteCutNode &cut : cuts) {
    std::vector<fpga::NetRouteRef> all =
        fpga::findNetRoutesByNode(*tile, cut.type, cut.node, false);
    std::vector<fpga::NetRouteRef> transit =
        fpga::findNetRoutesByNode(*tile, cut.type, cut.node, true);
    if (all.empty()) {
      return reject_bridge("busy_bit_without_owner");
    }
    for (const fpga::NetRouteRef &ref : all) {
      bool is_transit = std::any_of(
          transit.begin(), transit.end(), [&](const fpga::NetRouteRef &entry) {
            return entry.net == ref.net &&
                   entry.binding_index == ref.binding_index;
          });
      if (!is_transit || !ref.net ||
          ref.binding_index >= ref.net->routes.size() ||
          !ref.net->routeCanBePreempted()) {
        return reject_bridge(!is_transit ? "endpoint_owner"
                                         : "owner_not_preemptible");
      }
      rtl::NetRouteBinding &binding = ref.net->routes[ref.binding_index];
      if (!binding.from || !binding.to || binding.route_name.empty() ||
          bindingFromTile(binding, *tile) || bindingToTile(binding, *tile) ||
          (ref.net == current_net &&
           binding.route_name == current_route_name) ||
          pnr::preemptionOwnerIsCurrentTree(
              ref.net == current_net,
              sourceRouteKey(current_source, current_source_port),
              sourceRouteKey(binding.from, binding.from_port)) ||
          preemptionOwnerIsProtected(router, binding)) {
        return reject_bridge("same_source_or_incomplete_owner");
      }
      bool finished_endpoint =
          router->moving_stage &&
          (movingInstIsFinished(router->move_finished_insts, binding.from) ||
           movingInstIsFinished(router->move_finished_insts, binding.to));
      if (!pnr::canPreemptDuringFocusedMove(
              router->moving_stage, router->moving_focus_inst != nullptr,
              router->moving_sources_stage) ||
          !pnr::canPreemptMovingRoute(router->moving_stage,
                                      finished_endpoint) ||
          router->preempted_route_names_this_pass.contains(
              binding.route_name) ||
          pnr::preemptionWouldCycle(router->preempted_route_blockers,
                                    current_route_name,
                                    binding.route_name)) {
        return reject_bridge("preemption_policy");
      }
      bool duplicate = std::any_of(
          victims.begin(), victims.end(), [&](const DockingBridgeVictim &old) {
            return sameDockingBridgeVictim(old, ref);
          });
      if (!duplicate) {
        victims.push_back({ref.net, ref.binding_index});
      }
    }
  }
  if (victims.empty()) {
    return reject_bridge("no_victim");
  }
  if (!pnr::bridgePreemptionHasSingleOwner(victims.size())) {
    return reject_bridge("shared_bridge_owners");
  }

  size_t complete_victims = 0;
  for (const DockingBridgeVictim &victim : victims) {
    rtl::NetRouteBinding &binding = victim.net->routes[victim.binding_index];
    if (!fpga::canPreemptNetRouteFromNodes(*victim.net, victim.binding_index, cuts)) {
      return reject_bridge("protected_takeoff");
    }
    const std::vector<Wire> *route = routeBindingRoute(binding);
    complete_victims += route && routeIsComplete(*route) ? 1 : 0;
  }
  if (!pnr::bridgePreemptionConservesTasks(complete_victims)) {
    return reject_bridge("multiple_complete_victims");
  }
  if (!pnr::bridgePreemptionPhaseAccepts(router->fanout_stage,
                                         router->moving_stage,
                                         allow_complete_victim,
                                         complete_victims)) {
    return reject_bridge("complete_victim_deferred");
  }

  if (stats) {
    stats->preempt_attempts += victims.size();
  }
  for (const DockingBridgeVictim &victim : victims) {
    rtl::NetRouteBinding binding = victim.net->routes[victim.binding_index];
    const std::vector<Wire> *route_before = routeBindingRoute(binding);
    const size_t route_size_before = route_before ? route_before->size() : 0;
    const bool route_complete_before =
        route_before && routeIsComplete(*route_before);
    if (!fpga::unrouteNetRouteFromNodes(*victim.net, victim.binding_index,
                                       cuts)) {
      return reject_bridge("victim_cut_failed");
    }
    std::vector<Wire> *remaining =
        routeBindingRoute(victim.net->routes[victim.binding_index]);
    RouteDesign::RouteTask task{binding.from,
                                binding.to,
                                victim.net,
                                binding.from_port,
                                binding.to_port,
                                binding.route_name,
                                0,
                                0,
                                0,
                                {},
                                routeStartsWithSharedPrefix(remaining)};
    router->enqueueRouteTask(task, router->pending_route_todo);
    router->preempted_route_names_this_pass.insert(binding.route_name);
    if (!current_route_name.empty()) {
      pnr::rememberPreemptionBlocker(router->preempted_route_blockers,
                                     binding.route_name,
                                     current_route_name);
    }
    if (stats) {
      ++stats->preempt_success;
      const size_t route_size_after = remaining ? remaining->size() : 0;
      stats->preempt_removed_fragments +=
          route_size_before > route_size_after
              ? route_size_before - route_size_after
              : 0;
      if (route_complete_before) {
        ++stats->preempt_complete_victims;
        ++stats->preempt_bridge_complete_victims;
      } else {
        ++stats->preempt_partial_victims;
        ++stats->preempt_bridge_partial_victims;
      }
    }
    if (routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET",
                          binding.route_name)) {
      PNR_LOG1(
          "ROUT",
          "routeDesign invalidation trace: docking bridge cut watched "
          "route='{}', blocker='{}', tile=({},{}), cuts={}, size {}->{}, "
          "complete_before={}",
          binding.route_name, current_route_name, bridge.tile.x, bridge.tile.y,
          cuts.size(), route_size_before, remaining ? remaining->size() : 0,
          route_complete_before);
    }
  }
  for (const fpga::RouteCutNode &cut : cuts) {
    PNR_ASSERT(!fpga::congestionNodeMask(tile->cb, cut.type).testBit(cut.node),
               "bridge preemption retained contested node {}", cut.node);
  }
  return true;
}

bool unrouteVictimBinding(
    RouteDesign *router, TransitVictim victim, bool prefer_branch,
    bool require_shared_branch, bool force_fanout,
    const std::string &blocker_route_name, rtl::Inst *current_source = nullptr,
    const std::string &current_source_port = std::string{}) {
  if (!router || !victim.net ||
      victim.binding_index >= victim.net->routes.size()) {
    return false;
  }
  if (!victim.net->routeCanBePreempted()) {
    return false;
  }
  rtl::NetRouteBinding binding = victim.net->routes[victim.binding_index];
  if (!binding.from || !binding.to || binding.route_name.empty()) {
    return false;
  }
  if (preemptionOwnerIsProtected(router, binding)) {
    return false;
  }
  if (!pnr::canPreemptDuringFocusedMove(
          router->moving_stage, router->moving_focus_inst != nullptr,
          router->moving_sources_stage)) {
    return false;
  }
  bool finished_endpoint =
      router->moving_stage &&
      (movingInstIsFinished(router->move_finished_insts, binding.from) ||
       movingInstIsFinished(router->move_finished_insts, binding.to));
  if (!pnr::canPreemptMovingRoute(router->moving_stage, finished_endpoint)) {
    return false;
  }
  if (!binding.route_name.empty()) {
    if (router->preempted_route_names_this_pass.contains(binding.route_name)) {
      return false;
    }
    if (pnr::preemptionWouldCycle(router->preempted_route_blockers,
                                  blocker_route_name, binding.route_name)) {
      return false;
    }
  }
  std::vector<Wire> *route =
      routeBindingRoute(victim.net->routes[victim.binding_index]);
  bool shared_branch = routeHasSharedBranchSuffix(route);
  if (!pnr::canPreemptFanoutSuffix(require_shared_branch, shared_branch,
                                   victim.node_shared)) {
    return false;
  }
  // A transit conflict invalidates only the suffix beginning at the exact
  // occupied node; the committed prefix remains the victim's retry anchor.
  bool trimmed_to_conflict = false;
  size_t route_size_before_trim = route ? route->size() : 0;
  if (victim.conflict_node >= 0 && victim.conflict_tile.x >= 0 &&
      victim.conflict_tile.y >= 0) {
    if (!fpga::canPreemptNetRouteFromNodes(
            *victim.net, victim.binding_index,
            {{victim.conflict_tile, victim.conflict_type, victim.conflict_node}})) {
      return false;
    }
    trimmed_to_conflict = fpga::unrouteNetRouteFromNode(
        *victim.net, victim.binding_index, victim.conflict_tile,
        victim.conflict_type, victim.conflict_node);
  }
  if (trimmed_to_conflict) {
    if (routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET",
                          binding.route_name)) {
      const std::vector<Wire> *remaining =
          routeBindingRoute(victim.net->routes[victim.binding_index]);
      PNR_LOG1(
          "ROUT",
          "routeDesign invalidation trace: source-conflict trim watched "
          "route='{}', blocker='{}', conflict=({},{}), type={}, node={}, "
          "size {}->{}, current_source='{}'/'{}'",
          binding.route_name, blocker_route_name, victim.conflict_tile.x,
          victim.conflict_tile.y, static_cast<int>(victim.conflict_type),
          victim.conflict_node, route_size_before_trim,
          remaining ? remaining->size() : 0,
          current_source ? current_source->makeName(FULL_NAME_LIMIT)
                         : std::string{},
          current_source_port);
    }
    RouteDesign::RouteTask reroute_task{binding.from,
                                        binding.to,
                                        victim.net,
                                        binding.from_port,
                                        binding.to_port,
                                        binding.route_name,
                                        0,
                                        0,
                                        0,
                                        {},
                                        force_fanout};
    router->enqueueRouteTask(reroute_task, router->pending_route_todo);
    router->preempted_route_names_this_pass.insert(binding.route_name);
    if (!blocker_route_name.empty()) {
      pnr::rememberPreemptionBlocker(router->preempted_route_blockers,
                                     binding.route_name, blocker_route_name);
    }
    return true;
  }
  // A conflict in an unshared suffix does not require invalidating sibling
  // branches.
  if (prefer_branch && shared_branch && !victim.node_shared &&
      fpga::unrouteNetBranch(*victim.net, victim.binding_index)) {
    if (routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET",
                          binding.route_name)) {
      PNR_LOG1("ROUT",
               "routeDesign invalidation trace: unrouteVictimBinding removed "
               "watched branch='{}', blocker='{}', current_source='{}'/'{}', "
               "prefer_branch={}, shared_branch={}, node_shared={}",
               binding.route_name, blocker_route_name,
               current_source ? current_source->makeName(FULL_NAME_LIMIT)
                              : std::string{},
               current_source_port, prefer_branch, shared_branch,
               victim.node_shared);
    }
    RouteDesign::RouteTask reroute_task{binding.from,
                                        binding.to,
                                        victim.net,
                                        binding.from_port,
                                        binding.to_port,
                                        binding.route_name,
                                        0,
                                        0,
                                        0,
                                        {},
                                        true};
    router->enqueueRouteTask(reroute_task, router->pending_route_todo);
    router->preempted_route_names_this_pass.insert(binding.route_name);
    if (!blocker_route_name.empty()) {
      pnr::rememberPreemptionBlocker(router->preempted_route_blockers,
                                     binding.route_name, blocker_route_name);
    }
    return true;
  }

  // A physical source-port tree is atomic: removing one victim binding must
  // also remove every sibling that shares its leased trunk.
  if (router->sourceTreeTouchesFinishedInst(*victim.net, binding.from,
                                            binding.from_port)) {
    return false;
  }
  std::vector<RouteDesign::RouteTask> reroute_tasks;
  size_t removed = router->unrouteSourceTree(
      *victim.net, binding.from, binding.from_port, &reroute_tasks, false,
      false);
  if (removed == 0) {
    return false;
  }
  for (RouteDesign::RouteTask &reroute_task : reroute_tasks) {
    bool selected_victim = reroute_task.net == victim.net &&
                           reroute_task.net_name == binding.route_name &&
                           reroute_task.to == binding.to;
    reroute_task.fanout = !selected_victim || force_fanout;
    router->enqueueRouteTask(reroute_task, router->pending_route_todo);
  }
  if (!binding.route_name.empty()) {
    router->preempted_route_names_this_pass.insert(binding.route_name);
    if (!blocker_route_name.empty()) {
      pnr::rememberPreemptionBlocker(router->preempted_route_blockers,
                                     binding.route_name, blocker_route_name);
    }
  }
  return true;
}

bool hasRoutedNet(const rtl::Inst &inst, const std::string &net_name) {
  for (const auto &route : inst.wires) {
    if (!route.empty() && route.front().net_name == net_name &&
        routeIsComplete(route)) {
      return true;
    }
  }
  return false;
}

bool sameRouteTask(const RouteDesign::RouteTask &left,
                   const RouteDesign::RouteTask &right) {
  return left.net == right.net && left.from == right.from &&
         left.to == right.to && left.from_port == right.from_port &&
         left.to_port == right.to_port && left.net_name == right.net_name;
}

// Moving may retarget endpoint objects while preserving the physical route.
// Diagnostic selection therefore uses stable net/route identity as fallback.
bool sameVisualizationRouteTask(const RouteDesign::RouteTask &left,
                                const RouteDesign::RouteTask &right) {
  return sameRouteTask(left, right) ||
         (left.net == right.net && left.net_name == right.net_name);
}

size_t routeTaskHash(const RouteDesign::RouteTask &task) {
  size_t value = std::hash<const void *>{}(task.net);
  auto combine = [&](size_t part) {
    value ^= part + 0x9e3779b97f4a7c15ULL + (value << 6) + (value >> 2);
  };
  combine(std::hash<const void *>{}(task.from));
  combine(std::hash<const void *>{}(task.to));
  combine(std::hash<std::string>{}(task.from_port));
  combine(std::hash<std::string>{}(task.to_port));
  combine(std::hash<std::string>{}(task.net_name));
  return value;
}

void mergeRouteTaskState(RouteDesign::RouteTask &old,
                         const RouteDesign::RouteTask &task) {
  if (!task.fanout) {
    old.fanout = false;
  }
  old.attempt = std::max(old.attempt, task.attempt);
  old.fanout_branch_offset =
      std::max(old.fanout_branch_offset, task.fanout_branch_offset);
  old.fanout_branch_attempt =
      std::max(old.fanout_branch_attempt, task.fanout_branch_attempt);
  old.no_progress_passes =
      std::max(old.no_progress_passes, task.no_progress_passes);
  old.endpoints_prepared = old.endpoints_prepared && task.endpoints_prepared;
  old.source_tree_rebuilt =
      old.source_tree_rebuilt || task.source_tree_rebuilt;
  old.source_tree_rebuild_attempted =
      old.source_tree_rebuild_attempted || task.source_tree_rebuild_attempted;
  if (task.failure_coord.x >= 0 && task.failure_coord.y >= 0 &&
      task.failure_sequence >= old.failure_sequence) {
    old.failure_coord = task.failure_coord;
    old.failure_sequence = task.failure_sequence;
    old.failure_node_type = task.failure_node_type;
    old.failure_node = task.failure_node;
  }
  // A route invalidated later in the same pass must survive deferred queue
  // compaction and run again on the next pass.
  old.remove_after_pass = false;
}

// Rebuild large scheduler queues in insertion order with expected O(n)
// identity lookup; focus rotation otherwise performs quadratic duplicate scans.
void deduplicateRouteTasks(std::vector<RouteDesign::RouteTask> &tasks) {
  std::vector<RouteDesign::RouteTask> unique;
  unique.reserve(tasks.size());
  std::unordered_map<size_t, std::vector<size_t>> buckets;
  buckets.reserve(tasks.size());
  for (const RouteDesign::RouteTask &task : tasks) {
    std::vector<size_t> &candidates = buckets[routeTaskHash(task)];
    auto old =
        std::find_if(candidates.begin(), candidates.end(), [&](size_t index) {
          return sameRouteTask(unique[index], task);
        });
    if (old != candidates.end()) {
      mergeRouteTaskState(unique[*old], task);
      continue;
    }
    candidates.push_back(unique.size());
    unique.push_back(task);
  }
  tasks = std::move(unique);
}

bool appendUniqueRouteTask(std::vector<RouteDesign::RouteTask> &queue,
                           const RouteDesign::RouteTask &task) {
  for (RouteDesign::RouteTask &old : queue) {
    if (!sameRouteTask(old, task)) {
      continue;
    }
    mergeRouteTaskState(old, task);
    return false;
  }
  queue.push_back(task);
  return true;
}

std::string nodeNameForDump(const Tile &tile, fpga::CBNodeNameType type,
                            int value) {
  if (!tile.cb_type || value < 0) {
    return {};
  }
  const std::string *name = tile.cb_type->nodeName(type, value);
  return name ? *name : std::string{};
}

// Format one visualized numeric node with the exact CBType name used by the
// renderer, retaining a numeric fallback when no annotation is available.
std::string visualizationNodeName(fpga::Device &device, Coord coord,
                                  fpga::CBNodeNameType type, int node) {
  const char *kind = type == fpga::CB_NODE_LOCAL ? "LOCAL" :
                     type == fpga::CB_NODE_JOINT ? "JOINT" :
                     type == fpga::CB_NODE_SRC ? "SRC" : "DST";
  std::string name;
  if (Tile *tile = device.getTile(coord.x, coord.y)) {
    name = nodeNameForDump(*tile, type, node);
  }
  if (name.empty()) {
    name = std::format("{}[{}]", kind, node);
  }
  return std::format("{}({},{})/{}", kind, coord.x, coord.y, name);
}

// Reproduce drawPhysicalRoute's node order as text so a highlighted failure
// line can be located precisely in the generated 5x5 visualization.
std::string visualizationRouteTrace(fpga::Device &device,
                                    const std::vector<Wire> &route) {
  std::vector<std::string> nodes;
  auto append = [&](Coord coord, fpga::CBNodeNameType type, int node) {
    if (node < 0) {
      return;
    }
    std::string value = visualizationNodeName(device, coord, type, node);
    if (nodes.empty() || nodes.back() != value) {
      nodes.push_back(std::move(value));
    }
  };
  for (const Wire &fragment : route) {
    if (fragment.type == Wire::WIRE_TILE_PIN) {
      append(fragment.from, fpga::CB_NODE_LOCAL, fragment.local);
      continue;
    }
    if (fragment.type == Wire::WIRE_ROUTE_EDGE) {
      append(fragment.from,
             static_cast<fpga::CBNodeNameType>(fragment.from_node_type),
             fragment.from_node);
      append(fragment.to,
             static_cast<fpga::CBNodeNameType>(fragment.to_node_type),
             fragment.to_node);
      continue;
    }
    if (fragment.type != Wire::WIRE_CROSSBAR) {
      continue;
    }
    append(fragment.from,
           fragment.pos == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST,
           fragment.local);
    append(fragment.from, fpga::CB_NODE_JOINT, fragment.joint2);
    append(fragment.from, fpga::CB_NODE_JOINT, fragment.joint);
    append(fragment.from, fpga::CB_NODE_SRC, fragment.jump);
    if (fragment.jump >= 0) {
      append(fragment.to, fpga::CB_NODE_DST, fragment.dst);
    }
  }
  std::ostringstream trace;
  for (size_t index = 0; index < nodes.size(); ++index) {
    if (index != 0) {
      trace << " -> ";
    }
    trace << nodes[index];
  }
  return trace.str();
}

std::string wireTypeForDump(const Wire &wire) {
  switch (wire.type) {
  case Wire::WIRE_CROSSBAR:
    return "crossbar";
  case Wire::WIRE_TILE_PIN:
    return "tile_pin";
  case Wire::WIRE_ROUTE_EDGE:
    return "route_edge";
  }
  return "unknown";
}

std::string instNameForDump(const rtl::Inst *inst) {
  return inst ? const_cast<rtl::Inst *>(inst)->makeName(FULL_NAME_LIMIT)
              : std::string{};
}

std::string instTypeForDump(const rtl::Inst *inst) {
  return inst && inst->cell_ref.peer
             ? const_cast<Ref<rtl::Cell> &>(inst->cell_ref)->type
             : std::string{};
}

std::string tileNameForDump(const Tile *tile) {
  if (!tile) {
    return {};
  }
  if (!tile->full_name.empty()) {
    return tile->full_name;
  }
  return tile->makeName();
}

std::string coordForDump(const Coord &coord) {
  return std::format("({}, {})", coord.x, coord.y);
}

// Write one route task in a stable text form so timeout dumps are
// grep-friendly.
void dumpRouteTask(std::ostream &out, const char *queue_name, size_t index,
                   const RouteDesign::RouteTask &task) {
  const std::vector<Wire> *route =
      findBoundRoute(task.net, task.from, task.to, task.from_port, task.to_port,
                     task.net_name);
  out << queue_name << '[' << index << "]"
      << " net='" << task.net_name << "'"
      << " from='" << instNameForDump(task.from) << "'"
      << " from_type='" << instTypeForDump(task.from) << "'"
      << " from_port='" << task.from_port << "'"
      << " to='" << instNameForDump(task.to) << "'"
      << " to_type='" << instTypeForDump(task.to) << "'"
      << " to_port='" << task.to_port << "'"
      << " fanout=" << task.fanout << " attempt=" << task.attempt
      << " route_size=" << (route ? route->size() : 0)
      << " route_xbars=" << routeCrossbarFragments(route)
      << " complete=" << (route && routeIsComplete(*route)) << '\n';
}

std::vector<Tile *>
routeTileCandidates(rtl::Inst &inst, const std::string &port, bool output,
                    const Coord *preferred_target = nullptr);
bool partialRouteEndpoint(const std::vector<Wire> &route, Tile *&tile,
                          int &local, std::string &dst_wire);
uint64_t tileDeadendKey(const Coord &coord);
void applyRouteDeadends(
    const std::unordered_map<uint64_t, NodeMask> &src_deadends);
void logTargetTileEntryTable(const RouteDesign::RouteTask &task,
                             const char *context);
void logDockingBackwardState(const RouteDesign::RouteTask &task,
                             Tile &forward_tile, int forward_dst,
                             Tile &target_tile, NodeMask pin_nodes, int radius);
std::string maskBitsForDump(NodeMask value, size_t limit = 64);
std::string nodeOwnerForDump(Tile *tile, fpga::CBNodeNameType type, int node,
                             bool transit_only);
NodeMask incomingDstMaskForRouteTile(Tile &tile);
bool routeTileHasIncomingForLocalNodes(Tile &tile, NodeMask local_nodes);

// Show endpoint route-tile candidates so blocked tasks can be checked against
// placement-derived access points.
void dumpRouteTileCandidates(std::ostream &out, rtl::Inst *inst,
                             const std::string &port, bool output) {
  if (!inst) {
    out << "  route_candidates=none\n";
    return;
  }
  std::vector<Tile *> candidates = routeTileCandidates(*inst, port, output);
  out << "  route_candidates count=" << candidates.size() << " port='" << port
      << "' output=" << output << '\n';
  for (size_t i = 0; i < candidates.size(); ++i) {
    Tile *candidate = candidates[i];
    NodeMask nodes =
        candidate ? routeTileEndpointNodes(*candidate, *inst, port, output)
                  : NodeMask{};
    out << "    candidate[" << i << "] coord="
        << (candidate ? coordForDump(candidate->coord) : std::string{"none"})
        << " name='" << tileNameForDump(candidate) << "'"
        << " cb_type='"
        << (candidate && candidate->cb_type ? candidate->cb_type->name
                                            : std::string{})
        << "'"
        << " nodes=" << maskString(nodes) << '\n';
  }
}

// Print candidate route tiles and entry masks for the destination pin, separate
// from the current partial endpoint.
void dumpTargetCandidateSummaryLog(const RouteDesign::RouteTask &task,
                                   size_t task_index) {
  if (!task.to || !task.to->tile.peer || !task.to->cell_ref.peer) {
    PNR_LOG1("ROUT",
             "routeDesign unfinished[{}] target candidates skipped: missing "
             "destination placement",
             task_index);
    return;
  }
  std::vector<Tile *> candidates =
      routeTileCandidates(*task.to, task.to_port, false);
  PNR_LOG1("ROUT",
           "routeDesign unfinished[{}] target candidates: net='{}', "
           "target='{}' type='{}' port='{}', placed_tile=({},{})/{}, count={}",
           task_index, task.net_name, task.to->makeName(FULL_NAME_LIMIT),
           task.to->cell_ref.peer ? task.to->cell_ref->type : std::string{},
           task.to_port, task.to->tile->coord.x, task.to->tile->coord.y,
           task.to->pos, candidates.size());
  for (size_t i = 0; i < candidates.size(); ++i) {
    Tile *candidate = candidates[i];
    NodeMask pin_nodes =
        candidate ? routeTileInputNodes(*candidate, *task.to, task.to_port)
                  : NodeMask{};
    PNR_LOG1(
        "ROUT",
        "routeDesign unfinished[{}] target candidate[{}]: coord=({},{}) "
        "name='{}', tile_type='{}', cb_type='{}', pin_nodes={}, src={}, "
        "dst={}, joint={}, local={}, pin_leased={}, src_deadend={}",
        task_index, i, candidate ? candidate->coord.x : -1,
        candidate ? candidate->coord.y : -1, tileNameForDump(candidate),
        candidate && candidate->tile_type ? candidate->tile_type->name
                                          : std::string{},
        candidate && candidate->cb_type ? candidate->cb_type->name
                                        : std::string{},
        maskBitsForDump(pin_nodes),
        candidate ? maskBitsForDump(candidate->cb.src.jump) : std::string{},
        candidate ? maskBitsForDump(candidate->cb.dst.jump) : std::string{},
        candidate ? maskBitsForDump(candidate->cb.joint.jump) : std::string{},
        candidate ? maskBitsForDump(candidate->cb.local.local) : std::string{},
        candidate ? maskBitsForDump(candidate->pin_state.leased_nodes)
                  : std::string{},
        candidate ? maskBitsForDump(candidate->cb.src_deadend.jump)
                  : std::string{});
    if (!candidate || !candidate->cb_type) {
      continue;
    }
    pin_nodes.for_each_set_bit([&](int pin) {
      candidate->cb_type->ensureDerivedMasks();
      NodeMask dst_candidates =
          candidate->cb_type->dsts_reaching_local[pin].jump;
      PNR_LOG1("ROUT",
               "routeDesign unfinished[{}] target candidate[{}] pin: local={} "
               "'{}', leased={}, dsts_reaching_local={}",
               task_index, i, pin,
               nodeNameForDump(*candidate, fpga::CB_NODE_LOCAL, pin),
               candidate->isPinNodeLeased(pin),
               maskBitsForDump(dst_candidates));
      dst_candidates.for_each_set_bit([&](int dst) {
        std::vector<TerminalEntryCandidate> entries =
            targetEntryCandidates(*candidate->cb_type, dst, pin);
        for (const TerminalEntryCandidate &entry : entries) {
          CBState test_cb = candidate->cb;
          bool lease_ok = leaseConcreteTerminal(test_cb, dst, pin, entry);
          bool dst_leased =
              (candidate->cb.dst.jump & (NodeMask{0, 1} << dst)) != NodeMask{};
          bool joint_leased = entry.joint >= 0 &&
                              (candidate->cb.joint.jump &
                               (NodeMask{0, 1} << entry.joint)) != NodeMask{};
          bool local_leased = candidate->isPinNodeLeased(pin) ||
                              (candidate->cb.local.local &
                               (NodeMask{0, 1} << pin)) != NodeMask{};
          PNR_LOG1("ROUT",
                   "routeDesign unfinished[{}] target entry: cand={}, pin={} "
                   "'{}', dst={} '{}', kind={}, joint={} '{}', lease_ok={}, "
                   "leased(dst={},joint={},local={}), "
                   "owners(dst='{}',joint='{}',local='{}'), "
                   "transit(dst='{}',joint='{}',local='{}')",
                   task_index, i, pin,
                   nodeNameForDump(*candidate, fpga::CB_NODE_LOCAL, pin), dst,
                   nodeNameForDump(*candidate, fpga::CB_NODE_DST, dst),
                   static_cast<int>(entry.kind), entry.joint,
                   entry.joint >= 0
                       ? nodeNameForDump(*candidate, fpga::CB_NODE_JOINT,
                                         entry.joint)
                       : std::string{"direct"},
                   lease_ok, dst_leased, joint_leased, local_leased,
                   nodeOwnerForDump(candidate, fpga::CB_NODE_DST, dst, false),
                   entry.joint >= 0
                       ? nodeOwnerForDump(candidate, fpga::CB_NODE_JOINT,
                                          entry.joint, false)
                       : std::string{},
                   nodeOwnerForDump(candidate, fpga::CB_NODE_LOCAL, pin, false),
                   nodeOwnerForDump(candidate, fpga::CB_NODE_DST, dst, true),
                   entry.joint >= 0
                       ? nodeOwnerForDump(candidate, fpga::CB_NODE_JOINT,
                                          entry.joint, true)
                       : std::string{},
                   nodeOwnerForDump(candidate, fpga::CB_NODE_LOCAL, pin, true));
        }
        return false;
      });
      return false;
    });
  }
}

// Record the possible final-tile choices for unfinished routes plus current
// state masks.
void dumpTaskLastHopOptions(std::ostream &out, const char *queue_name,
                            size_t index, const RouteDesign::RouteTask &task) {
  const std::vector<Wire> *route =
      findBoundRoute(task.net, task.from, task.to, task.from_port, task.to_port,
                     task.net_name);
  if (!route || route->empty()) {
    out << queue_name << '[' << index << "] no_partial_route\n";
    return;
  }
  const Wire &last = route->back();
  Tile *tile = nullptr;
  int endpoint_dst = -1;
  std::string endpoint_dst_wire;
  bool has_endpoint =
      partialRouteEndpoint(*route, tile, endpoint_dst, endpoint_dst_wire);
  out << queue_name << '[' << index << "]"
      << " net='" << task.net_name << "'"
      << " failed_tile="
      << coordForDump(tile ? tile->coord : last.to) << " failed_tile_name='"
      << tileNameForDump(tile) << "'"
      << " last_from=" << coordForDump(last.from) << " last_src=" << last.jump
      << " last_dst=" << endpoint_dst << " last_joint=" << last.joint
      << " endpoint_resolved=" << has_endpoint << '\n';
  dumpRouteTileCandidates(out, task.to, task.to_port, false);
  if (!tile || !tile->cb_type || endpoint_dst < 0 ||
      endpoint_dst >= CB_MAX_NODES) {
    return;
  }

  out << "  dst_name='"
      << nodeNameForDump(*tile, fpga::CB_NODE_DST, endpoint_dst) << "'"
      << " dst_wire='" << endpoint_dst_wire << "'"
      << " direct_dst_src="
      << maskString(tile->cb_type->dst_src[endpoint_dst].jump)
      << " dst_joint="
      << maskString(tile->cb_type->dst_joint[endpoint_dst].joint)
      << " leased_src=" << maskString(tile->cb.src.jump)
      << " leased_dst=" << maskString(tile->cb.dst.jump)
      << " leased_joint=" << maskString(tile->cb.joint.jump)
      << " src_deadend=" << maskString(tile->cb.src_deadend.jump) << '\n';
  if (task.to && task.to->cell_ref.peer) {
    NodeMask pin_nodes = routeTileInputNodes(*tile, *task.to, task.to_port);
    out << "  failed_tile_target_pin_nodes=" << maskString(pin_nodes)
        << " target_inst='" << instNameForDump(task.to) << "'"
        << " target_type='" << instTypeForDump(task.to) << "'"
        << " target_port='" << task.to_port << "'" << '\n';
    pin_nodes.for_each_set_bit([&](int pin) {
      int joint = -1;
      bool can_in = tile->cb_type->canIn(endpoint_dst, pin, joint);
      CBState test_cb = tile->cb;
      bool lease_ok = can_in && test_cb.leaseIn(endpoint_dst, pin, joint);
      out << "  target_pin bit=" << pin << " name='"
          << nodeNameForDump(*tile, fpga::CB_NODE_LOCAL, pin) << "'"
          << " leased=" << tile->isPinNodeLeased(pin) << " can_in=" << can_in
          << " joint=" << joint << " lease_ok=" << lease_ok << '\n';
      return false;
    });
  }
  tile->cb_type->dst_src[endpoint_dst].jump.for_each_set_bit([&](int src) {
    out << "  possible_direct_src bit=" << src << " name='"
        << nodeNameForDump(*tile, fpga::CB_NODE_SRC, src) << "'"
        << " leased="
        << ((tile->cb.src.jump & (NodeMask{0, 1} << src)) != NodeMask{})
        << '\n';
    return false;
  });
  tile->cb_type->dst_joint[endpoint_dst].joint.for_each_set_bit([&](int joint) {
    out << "  possible_joint bit=" << joint << " name='"
        << nodeNameForDump(*tile, fpga::CB_NODE_JOINT, joint) << "'"
        << " leased="
        << ((tile->cb.joint.jump & (NodeMask{0, 1} << joint)) != NodeMask{})
        << " joint_src=" << maskString(tile->cb_type->joint_src[joint].jump)
        << '\n';
    tile->cb_type->joint_src[joint].jump.for_each_set_bit([&](int src) {
      out << "    possible_joint_src joint=" << joint << " src=" << src
          << " src_name='" << nodeNameForDump(*tile, fpga::CB_NODE_SRC, src)
          << "'"
          << " leased="
          << ((tile->cb.src.jump & (NodeMask{0, 1} << src)) != NodeMask{})
          << '\n';
      return false;
    });
    return false;
  });
}

std::string maskBitsForDump(NodeMask value, size_t limit) {
  std::ostringstream out;
  out << '[';
  size_t count = 0;
  bool truncated = false;
  value.for_each_set_bit([&](int bit) {
    if (count != 0) {
      out << ',';
    }
    if (count >= limit) {
      out << "...";
      truncated = true;
      return true;
    }
    out << bit;
    ++count;
    return false;
  });
  out << ']';
  if (truncated) {
    out << "(truncated)";
  }
  return out.str();
}

std::string nodeOwnerForDump(Tile *tile, fpga::CBNodeNameType type, int node,
                             bool transit_only) {
  if (!tile || node < 0) {
    return {};
  }
  rtl::Net *net = fpga::findNetByNode(*tile, type, node, transit_only);
  return net ? net->makeName(FULL_NAME_LIMIT) : std::string{};
}

std::string srcOwnerBindingForDump(Tile *tile, int src_node) {
  if (!tile || src_node < 0) {
    return {};
  }
  for (auto &ref : tile->routedNets) {
    rtl::Net *net = ref.peer;
    if (!net) {
      continue;
    }
    for (rtl::NetRouteBinding &binding : net->routes) {
      if (!binding.owner ||
          binding.route_index >= binding.owner->wires.size()) {
        continue;
      }
      const std::vector<Wire> &route =
          binding.owner->wires[binding.route_index];
      for (const Wire &fragment : route) {
        if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump != src_node ||
            !sameCoord(fragment.from, tile->coord) ||
            sameCoord(fragment.from, fragment.to)) {
          continue;
        }
        return std::format(
            "net='{}' from='{}'/'{}' to='{}'/'{}' route='{}' pos={}",
            net->makeName(FULL_NAME_LIMIT),
            binding.from ? binding.from->makeName(FULL_NAME_LIMIT)
                         : std::string{},
            binding.from_port,
            binding.to ? binding.to->makeName(FULL_NAME_LIMIT) : std::string{},
            binding.to_port, binding.route_name, fragment.pos);
      }
    }
  }
  return {};
}

std::string srcCandidateOwnersForDump(Tile *tile,
                                      const std::vector<uint16_t> *src_nodes,
                                      NodeMask raw_src_mask) {
  if (!tile || !src_nodes) {
    return {};
  }
  std::ostringstream out;
  out << '[';
  size_t count = 0;
  for (uint16_t src : *src_nodes) {
    if ((raw_src_mask & (NodeMask{0, 1} << src)) == NodeMask{}) {
      continue;
    }
    if (count != 0) {
      out << ';';
    }
    out << src << " '" << nodeNameForDump(*tile, fpga::CB_NODE_SRC, src) << "'"
        << " owner='" << nodeOwnerForDump(tile, fpga::CB_NODE_SRC, src, false)
        << "'"
        << " transit='" << nodeOwnerForDump(tile, fpga::CB_NODE_SRC, src, true)
        << "'";
    ++count;
    if (count >= 8) {
      out << ";...";
      break;
    }
  }
  out << ']';
  return out.str();
}

// Audit resources removed from one moved-sink suffix against the live bitmaps.
// Every remaining lease must resolve to another routed net, never an orphan
// bit.
void logMovedSuffixLeaseAudit(const std::string &route_name,
                              const std::vector<Wire> &removed) {
  std::unordered_set<std::string> seen;
  size_t checked = 0;
  size_t still_leased = 0;
  size_t orphaned = 0;
  auto audit_node = [&](const Coord &coord, fpga::CBNodeNameType type, int node,
                        const char *resource) {
    if (node < 0) {
      return;
    }
    std::string key = std::format("{}:{}:{}:{}", coord.x, coord.y,
                                  static_cast<int>(type), node);
    if (!seen.insert(key).second) {
      return;
    }
    Tile *tile = fpga::Device::current().getTile(coord.x, coord.y);
    if (!tile) {
      return;
    }
    NodeMask bit = NodeMask{0, 1} << node;
    bool leased =
        type == fpga::CB_NODE_SRC   ? (tile->cb.src.jump & bit) != NodeMask{}
        : type == fpga::CB_NODE_DST ? (tile->cb.dst.jump & bit) != NodeMask{}
        : type == fpga::CB_NODE_JOINT
            ? (tile->cb.joint.jump & bit) != NodeMask{}
            : tile->isPinNodeLeased(node) ||
                  (tile->cb.local.local & bit) != NodeMask{};
    ++checked;
    if (!leased) {
      return;
    }
    ++still_leased;
    std::string owner = nodeOwnerForDump(tile, type, node, false);
    if (owner.empty()) {
      ++orphaned;
    }
    PNR_LOG1(
        "ROUT",
        "routeDesign moving suffix lease retained: route='{}', resource={}, "
        "coord=({},{}), type={}, node={} '{}', owner='{}', orphan={}",
        route_name, resource, coord.x, coord.y, static_cast<int>(type), node,
        nodeNameForDump(*tile, type, node), owner, owner.empty());
  };

  for (size_t index = 0; index < removed.size(); ++index) {
    const Wire &fragment = removed[index];
    if (fragment.shared) {
      continue;
    }
    if (fragment.type == Wire::WIRE_TILE_PIN) {
      audit_node(fragment.from, fpga::CB_NODE_LOCAL, fragment.local,
                 "tile_pin");
      continue;
    }
    if (fragment.type != Wire::WIRE_CROSSBAR) {
      continue;
    }
    audit_node(fragment.from, fpga::CB_NODE_SRC, fragment.jump, "src");
    audit_node(fragment.from, fpga::CB_NODE_JOINT, fragment.joint, "joint");
    audit_node(fragment.from, fpga::CB_NODE_JOINT, fragment.joint2, "joint2");
    if (fragment.pos == 0) {
      audit_node(fragment.from, fpga::CB_NODE_LOCAL, fragment.local,
                 "source_local");
    } else if (fragment.owns_dst) {
      audit_node(fragment.from, fpga::CB_NODE_DST, fragment.local,
                 "incoming_dst");
    }
  }
  PNR_LOG1("ROUT",
           "routeDesign moving suffix lease audit: route='{}', "
           "removed_fragments={}, checked_resources={}, still_leased={}, "
           "orphaned={}",
           route_name, removed.size(), checked, still_leased, orphaned);
}

void dumpRouteTileStateLog(const char *prefix, Tile *tile) {
  if (!tile) {
    PNR_LOG1("ROUT", "{} tile_state: tile=none", prefix);
    return;
  }
  PNR_LOG1("ROUT",
           "{} tile_state: coord=({},{}), name='{}', cb_type='{}', src={}, "
           "dst={}, joint={}, local={}, src_deadend={}",
           prefix, tile->coord.x, tile->coord.y, tileNameForDump(tile),
           tile->cb_type ? tile->cb_type->name : std::string{},
           maskBitsForDump(tile->cb.src.jump),
           maskBitsForDump(tile->cb.dst.jump),
           maskBitsForDump(tile->cb.joint.jump),
           maskBitsForDump(tile->cb.local.local),
           maskBitsForDump(tile->cb.src_deadend.jump));
}

void dumpRouteFragmentDiagnostics(const RouteDesign::RouteTask &task,
                                  size_t task_index,
                                  const std::vector<Wire> &route) {
  for (size_t fragment_index = 0; fragment_index < route.size();
       ++fragment_index) {
    const Wire &fragment = route[fragment_index];
    Tile *from_tile =
        fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
    Tile *to_tile =
        fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
    PNR_LOG1(
        "ROUT",
        "routeDesign unfinished[{}] fragment[{}]: net='{}', type={}, "
        "from=({},{}) '{}', to=({},{}) '{}', local={} '{}', dst={} '{}', "
        "jump={} '{}', route_jump={}, joint={} '{}', joint2={} '{}', pos={}, "
        "shared={}, from_wire='{}', src_wire='{}', dst_wire='{}'",
        task_index, fragment_index, task.net_name, wireTypeForDump(fragment),
        fragment.from.x, fragment.from.y, tileNameForDump(from_tile),
        fragment.to.x, fragment.to.y, tileNameForDump(to_tile), fragment.local,
        from_tile ? nodeNameForDump(*from_tile,
                                    fragment.pos == 0 ? fpga::CB_NODE_LOCAL
                                                      : fpga::CB_NODE_DST,
                                    fragment.local)
                  : std::string{},
        fragment.dst,
        to_tile ? nodeNameForDump(*to_tile, fpga::CB_NODE_DST, fragment.dst)
                : std::string{},
        fragment.jump,
        from_tile
            ? nodeNameForDump(*from_tile, fpga::CB_NODE_SRC, fragment.jump)
            : std::string{},
        fragment.route_jump, fragment.joint,
        from_tile
            ? nodeNameForDump(*from_tile, fpga::CB_NODE_JOINT, fragment.joint)
            : std::string{},
        fragment.joint2,
        from_tile
            ? nodeNameForDump(*from_tile, fpga::CB_NODE_JOINT, fragment.joint2)
            : std::string{},
        fragment.pos, fragment.shared, fragment.from_wire_name,
        fragment.src_wire_name, fragment.dst_wire_name);
    if (fragment.type == Wire::WIRE_CROSSBAR) {
      PNR_LOG1(
          "ROUT",
          "routeDesign unfinished[{}] fragment[{}] owners: src_owner='{}', "
          "src_transit='{}', dst_owner='{}', dst_transit='{}', "
          "joint_owner='{}', joint_transit='{}'",
          task_index, fragment_index,
          nodeOwnerForDump(from_tile, fpga::CB_NODE_SRC, fragment.jump, false),
          nodeOwnerForDump(from_tile, fpga::CB_NODE_SRC, fragment.jump, true),
          nodeOwnerForDump(from_tile, fpga::CB_NODE_DST, fragment.local, false),
          nodeOwnerForDump(from_tile, fpga::CB_NODE_DST, fragment.local, true),
          nodeOwnerForDump(from_tile, fpga::CB_NODE_JOINT, fragment.joint,
                           false),
          nodeOwnerForDump(from_tile, fpga::CB_NODE_JOINT, fragment.joint,
                           true));
      dumpRouteTileStateLog(
          std::format("routeDesign unfinished[{}] fragment[{}] from",
                      task_index, fragment_index)
              .c_str(),
          from_tile);
      if (!sameCoord(fragment.from, fragment.to)) {
        dumpRouteTileStateLog(
            std::format("routeDesign unfinished[{}] fragment[{}] to",
                        task_index, fragment_index)
                .c_str(),
            to_tile);
      }
    }
  }
}

void dumpContinuationOptionsLog(const RouteDesign::RouteTask &task,
                                size_t task_index,
                                const std::vector<Wire> &route) {
  Tile *endpoint_tile = nullptr;
  int endpoint_dst = -1;
  std::string endpoint_dst_wire;
  if (!partialRouteEndpoint(route, endpoint_tile, endpoint_dst,
                            endpoint_dst_wire) ||
      !endpoint_tile || !endpoint_tile->cb_type) {
    PNR_LOG1("ROUT",
             "routeDesign unfinished[{}] continuation: no partial endpoint",
             task_index);
    return;
  }
  NodeMask endpoint_pin_nodes =
      task.to ? routeTileInputNodes(*endpoint_tile, *task.to, task.to_port)
              : NodeMask{};
  PNR_LOG1(
      "ROUT",
      "routeDesign unfinished[{}] continuation: endpoint=({},{}) '{}', dst={} "
      "'{}', dst_wire='{}', target=({},{}) '{}', endpoint_pin_nodes={}",
      task_index, endpoint_tile->coord.x, endpoint_tile->coord.y,
      tileNameForDump(endpoint_tile), endpoint_dst,
      nodeNameForDump(*endpoint_tile, fpga::CB_NODE_DST, endpoint_dst),
      endpoint_dst_wire,
      task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
      task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
      task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
      maskBitsForDump(endpoint_pin_nodes));
  dumpRouteTileStateLog(
      std::format("routeDesign unfinished[{}] endpoint", task_index).c_str(),
      endpoint_tile);
  const std::vector<uint16_t> *srcs =
      endpoint_tile->cb_type->srcNodes(fpga::CB_NODE_DST, endpoint_dst);
  if (!srcs || srcs->empty()) {
    PNR_LOG1(
        "ROUT",
        "routeDesign unfinished[{}] continuation: no src nodes from dst={}",
        task_index, endpoint_dst);
    return;
  }
  size_t printed = 0;
  for (uint16_t src : *srcs) {
    if (printed >= 48) {
      PNR_LOG1("ROUT",
               "routeDesign unfinished[{}] continuation: remaining src options "
               "truncated after {}",
               task_index, printed);
      break;
    }
    int joint = -1;
    bool can_jump =
        endpoint_tile->cb_type->canJump(endpoint_dst, src, src, joint);
    bool src_leased =
        (endpoint_tile->cb.src.jump & (NodeMask{0, 1} << src)) != NodeMask{};
    bool dst_leased = (endpoint_tile->cb.dst.jump &
                       (NodeMask{0, 1} << endpoint_dst)) != NodeMask{};
    bool joint_leased = joint >= 0 && (endpoint_tile->cb.joint.jump &
                                       (NodeMask{0, 1} << joint)) != NodeMask{};
    bool deadend = (endpoint_tile->cb.src_deadend.jump &
                    (NodeMask{0, 1} << src)) != NodeMask{};
    fpga::TileJumpTarget target =
        fpga::Device::current().resolveJump(*endpoint_tile, src);
    NodeMask src_mask = NodeMask{0, 1} << src;
    NodeMask dst_mask =
        endpoint_dst >= 0 ? (NodeMask{0, 1} << endpoint_dst) : NodeMask{};
    NodeMask joint_mask = joint >= 0 ? (NodeMask{0, 1} << joint) : NodeMask{};
    NodeMask target_dst_mask =
        target.dst_node >= 0 ? (NodeMask{0, 1} << target.dst_node) : NodeMask{};
    NodeMask target_pin_nodes =
        task.to && target.tile
            ? routeTileInputNodes(*target.tile, *task.to, task.to_port)
            : NodeMask{};
    bool target_pin_reject =
        task.to && task.to->tile.peer && target.tile &&
        sameCoord(target.tile->coord, task.to->tile->coord) &&
        !targetDstCanEnterPin(*target.tile, target.dst_node, target_pin_nodes);
    PNR_LOG1("ROUT",
             "routeDesign unfinished[{}] continuation src[{}]: src={} '{}', "
             "can_jump={}, joint={} '{}', "
             "masks(src={},dst={},joint={},target_dst={},target_pin_nodes={}), "
             "leased(src={},dst={},joint={}), deadend={}, owner='{}', "
             "transit='{}', binding=\"{}\", target=({},{}) '{}', target_dst={} "
             "'{}', target_pin_reject={}",
             task_index, printed, src,
             nodeNameForDump(*endpoint_tile, fpga::CB_NODE_SRC, src), can_jump,
             joint, nodeNameForDump(*endpoint_tile, fpga::CB_NODE_JOINT, joint),
             maskBitsForDump(src_mask), maskBitsForDump(dst_mask),
             maskBitsForDump(joint_mask), maskBitsForDump(target_dst_mask),
             maskBitsForDump(target_pin_nodes), src_leased, dst_leased,
             joint_leased, deadend,
             nodeOwnerForDump(endpoint_tile, fpga::CB_NODE_SRC, src, false),
             nodeOwnerForDump(endpoint_tile, fpga::CB_NODE_SRC, src, true),
             srcOwnerBindingForDump(endpoint_tile, src),
             target.tile ? target.tile->coord.x : -1,
             target.tile ? target.tile->coord.y : -1,
             target.tile ? tileNameForDump(target.tile) : std::string{},
             target.dst_node,
             target.tile ? nodeNameForDump(*target.tile, fpga::CB_NODE_DST,
                                           target.dst_node)
                         : std::string{},
             target_pin_reject);
    ++printed;
  }
}

void dumpBasicUnfinishedDiagnosticsLog(
    const std::vector<RouteDesign::RouteTask> &route_todo, int stage_pass,
    const char *reason, bool detailed = true) {
  constexpr size_t sample_limit = 10;
  struct SampledTask {
    uint64_t hash = std::numeric_limits<uint64_t>::max();
    size_t index = 0;
  };
  std::array<SampledTask, sample_limit> sample{};
  size_t sample_size = 0;
  auto task_hash = [](const RouteDesign::RouteTask &task) {
    uint64_t hash = 1469598103934665603ULL;
    auto append = [&](std::string_view value) {
      for (unsigned char byte : value) {
        hash ^= byte;
        hash *= 1099511628211ULL;
      }
      hash ^= 0xff;
      hash *= 1099511628211ULL;
    };
    append(task.net_name);
    append(task.from_port);
    append(task.to_port);
    if (task.from) {
      append(task.from->makeName(FULL_NAME_LIMIT));
    }
    if (task.to) {
      append(task.to->makeName(FULL_NAME_LIMIT));
    }
    return hash;
  };
  for (size_t index = 0; index < route_todo.size(); ++index) {
    SampledTask candidate{task_hash(route_todo[index]), index};
    if (sample_size < sample.size()) {
      sample[sample_size++] = candidate;
      continue;
    }
    auto worst = std::max_element(
        sample.begin(), sample.end(),
        [](const SampledTask &lhs, const SampledTask &rhs) {
          return lhs.hash < rhs.hash;
        });
    if (candidate.hash < worst->hash) {
      *worst = candidate;
    }
  }
  std::sort(sample.begin(), sample.begin() + sample_size,
            [](const SampledTask &lhs, const SampledTask &rhs) {
              return lhs.hash < rhs.hash;
            });
  PNR_LOG1("ROUT",
           "routeDesign unfinished diagnostics: pass={}, reason='{}', "
           "tasks={}, sampled={}",
           stage_pass, reason, route_todo.size(), sample_size);
  for (size_t sample_index = 0; sample_index < sample_size; ++sample_index) {
    size_t i = sample[sample_index].index;
    const RouteDesign::RouteTask &task = route_todo[i];
    const std::vector<Wire> *route =
        findBoundRoute(task.net, task.from, task.to, task.from_port,
                       task.to_port, task.net_name);
    PNR_LOG1("ROUT",
             "routeDesign unfinished[{}] task: net='{}', from='{}' type='{}' "
             "port='{}' tile=({},{})/{}, to='{}' type='{}' port='{}' "
             "tile=({},{})/{}, attempt={}, fanout={}, route_size={}, xbars={}, "
             "complete={}",
             i, task.net_name,
             task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
             task.from && task.from->cell_ref.peer ? task.from->cell_ref->type
                                                   : std::string{},
             task.from_port,
             task.from && task.from->tile.peer ? task.from->tile->coord.x : -1,
             task.from && task.from->tile.peer ? task.from->tile->coord.y : -1,
             task.from ? task.from->pos : -1,
             task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
             task.to && task.to->cell_ref.peer ? task.to->cell_ref->type
                                               : std::string{},
             task.to_port,
             task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
             task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
             task.to ? task.to->pos : -1, task.attempt, task.fanout,
             route ? route->size() : 0, routeCrossbarFragments(route),
             route && routeIsComplete(*route));
    if (!detailed) {
      continue;
    }
    if (!route || route->empty()) {
      dumpRouteTileCandidates(std::cout, task.from, task.from_port, true);
      dumpRouteTileCandidates(std::cout, task.to, task.to_port, false);
      dumpTargetCandidateSummaryLog(task, i);
      logTargetTileEntryTable(task, "Basic unfinished empty-route");
      continue;
    }
    dumpRouteFragmentDiagnostics(task, i, *route);
    dumpContinuationOptionsLog(task, i, *route);
    dumpTargetCandidateSummaryLog(task, i);
    logTargetTileEntryTable(task, "Basic unfinished");
  }
}

void collectInstsForDump(rtl::Inst &inst, std::vector<rtl::Inst *> &insts) {
  insts.push_back(&inst);
  for (auto &sub_inst : inst.insts) {
    collectInstsForDump(sub_inst, insts);
  }
}

std::vector<Wire> *bindingRouteForDump(rtl::NetRouteBinding &binding) {
  if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
    return nullptr;
  }
  return &binding.owner->wires[binding.route_index];
}

// Dump all routable state needed to trace a timeout without rerunning the
// placer.
void dumpFullRoutingState(
    const std::string &filename,
    const std::vector<RouteDesign::RouteTask> &route_todo,
    const std::vector<RouteDesign::RouteTask> &fanout_route_todo,
    const std::vector<RouteDesign::RouteTask> &pending_route_todo,
    const std::vector<RouteDesign::RouteTask> &moving_deferred_todo) {
  std::ofstream out(filename);
  if (!out) {
    PNR_LOG1("ROUT", "routeDesign failed to open routing state dump '{}'",
             filename);
    return;
  }

  rtl::Design &design = technology::Tech::current().design;
  std::vector<rtl::Inst *> insts;
  collectInstsForDump(design.top, insts);

  out << "==============================\n";
  out << "SECTION: CELLS\n";
  out << "==============================\n";
  for (rtl::Inst *inst : insts) {
    Tile *tile = inst && inst->tile.peer ? inst->tile.peer : nullptr;
    out << "cell name='" << instNameForDump(inst) << "'"
        << " type='" << instTypeForDump(inst) << "'"
        << " pos=" << (inst ? inst->pos : -1)
        << " tile=" << (tile ? coordForDump(tile->coord) : std::string{"none"})
        << " tile_name='" << tileNameForDump(tile) << "'"
        << " cb_coord="
        << (tile ? coordForDump(tile->cb_coord) : std::string{"none"})
        << " cb_name='" << (tile ? tile->cb_full_name : std::string{}) << "'"
        << " fixed=" << (inst ? inst->outline.fixed : false)
        << " wires=" << (inst ? inst->wires.size() : 0) << '\n';
  }

  out << "\n==============================\n";
  out << "SECTION: NETS\n";
  out << "==============================\n";
  for (auto &module : design.modules) {
    out << "module name='" << module.name << "' nets=" << module.nets.size()
        << '\n';
    for (auto &net : module.nets) {
      out << "net name='" << net.makeName(FULL_NAME_LIMIT) << "'"
          << " void=" << net.void_net << " routes=" << net.routes.size()
          << " src_port='"
          << (net.src_port.peer ? net.src_port->makeName() : std::string{})
          << "'"
          << " dst_port='"
          << (net.dst_port.peer ? net.dst_port->makeName() : std::string{})
          << "'" << '\n';
      for (size_t binding_index = 0; binding_index < net.routes.size();
           ++binding_index) {
        rtl::NetRouteBinding &binding = net.routes[binding_index];
        std::vector<Wire> *route = bindingRouteForDump(binding);
        out << "  binding index=" << binding_index << " route_name='"
            << binding.route_name << "'"
            << " owner='" << instNameForDump(binding.owner) << "'"
            << " route_index=" << binding.route_index << " from='"
            << instNameForDump(binding.from) << "'"
            << " from_port='" << binding.from_port << "'"
            << " to='" << instNameForDump(binding.to) << "'"
            << " to_port='" << binding.to_port << "'"
            << " route_size=" << (route ? route->size() : 0)
            << " route_xbars=" << routeCrossbarFragments(route)
            << " complete=" << (route && routeIsComplete(*route)) << '\n';
      }
    }
  }

  out << "\n==============================\n";
  out << "SECTION: WIRES\n";
  out << "==============================\n";
  for (rtl::Inst *inst : insts) {
    if (!inst) {
      continue;
    }
    for (size_t route_index = 0; route_index < inst->wires.size();
         ++route_index) {
      const std::vector<Wire> &route = inst->wires[route_index];
      for (size_t fragment_index = 0; fragment_index < route.size();
           ++fragment_index) {
        const Wire &wire = route[fragment_index];
        Tile *from_tile =
            fpga::Device::current().getTile(wire.from.x, wire.from.y);
        Tile *to_tile = fpga::Device::current().getTile(wire.to.x, wire.to.y);
        out << "wire owner='" << inst->makeName(FULL_NAME_LIMIT) << "'"
            << " route_index=" << route_index << " fragment=" << fragment_index
            << " net='" << wire.net_name << "'"
            << " type=" << wireTypeForDump(wire)
            << " from=" << coordForDump(wire.from) << " from_tile='"
            << tileNameForDump(from_tile) << "'"
            << " to=" << coordForDump(wire.to) << " to_tile='"
            << tileNameForDump(to_tile) << "'"
            << " local=" << wire.local << " dst=" << wire.dst
            << " jump=" << wire.jump
            << " joint=" << wire.joint << " pos=" << wire.pos
            << " shared=" << wire.shared << " from_name='"
            << wire.from_wire_name << "'"
            << " src_name='" << wire.src_wire_name << "'"
            << " dst_name='" << wire.dst_wire_name << "'"
            << " local_name='"
            << (from_tile
                    ? nodeNameForDump(
                          *from_tile,
                          wire.pos == 0 ? fpga::CB_NODE_LOCAL
                                        : fpga::CB_NODE_DST,
                          wire.local)
                    : std::string{})
            << "'"
            << " src_node_name='"
            << (from_tile
                    ? nodeNameForDump(*from_tile, fpga::CB_NODE_SRC, wire.jump)
                    : std::string{})
            << "'"
            << " dst_node_name='"
            << (to_tile
                    ? nodeNameForDump(*to_tile, fpga::CB_NODE_DST, wire.dst)
                    : std::string{})
            << "'"
            << " joint_name='"
            << (from_tile ? nodeNameForDump(*from_tile, fpga::CB_NODE_JOINT,
                                            wire.joint)
                          : std::string{})
            << "'"
            << " resource=" << coordForDump(wire.resource)
            << " resource_node=" << wire.resource_node
            << " pin_dir=" << wire.pin_dir << " cell_type='" << wire.cell_type
            << "'"
            << " port='" << wire.port << "'" << '\n';
      }
    }
  }

  auto dump_queue = [&](const char *name,
                        const std::vector<RouteDesign::RouteTask> &queue) {
    out << "\n==============================\n";
    out << "SECTION: " << name << "\n";
    out << "==============================\n";
    for (size_t i = 0; i < queue.size(); ++i) {
      dumpRouteTask(out, name, i, queue[i]);
    }
  };
  dump_queue("TODO_BASIC_OR_ACTIVE", route_todo);
  dump_queue("TODO_FANOUT_DEFERRED", fanout_route_todo);
  dump_queue("TODO_PENDING", pending_route_todo);
  dump_queue("TODO_MOVING_DEFERRED", moving_deferred_todo);

  out << "\n==============================\n";
  out << "SECTION: TODO_LAST_HOP_OPTIONS\n";
  out << "==============================\n";
  for (size_t i = 0; i < route_todo.size(); ++i) {
    dumpTaskLastHopOptions(out, "TODO_BASIC_OR_ACTIVE", i, route_todo[i]);
  }

  out << "\n==============================\n";
  out << "SECTION: TILES\n";
  out << "==============================\n";
  for (Tile &tile : fpga::Device::current().tile_grid) {
    out << "tile coord=" << coordForDump(tile.coord) << " name='"
        << tileNameForDump(&tile) << "'"
        << " type=" << tile.type << " tile_type='"
        << (tile.tile_type ? tile.tile_type->name : std::string{}) << "'"
        << " cb_type='" << (tile.cb_type ? tile.cb_type->name : std::string{})
        << "'"
        << " src=" << maskString(tile.cb.src.jump)
        << " dst=" << maskString(tile.cb.dst.jump)
        << " joint=" << maskString(tile.cb.joint.jump)
        << " local=" << maskString(tile.cb.local.local)
        << " src_deadend=" << maskString(tile.cb.src_deadend.jump)
        << " pin_leased=" << maskString(tile.pin_state.leased_nodes)
        << " routed_nets=" << tile.routedNets.size() << '\n';
    for (Ref<rtl::Net> &routed_net : tile.routedNets) {
      if (routed_net.peer) {
        out << "  routed_net='" << routed_net->makeName(FULL_NAME_LIMIT)
            << "'\n";
      }
    }
  }
}

const fpga::CBConnName *
selectConcreteConn(const fpga::CBType *type, fpga::CBNodeNameType from_type,
                   int from_value, fpga::CBNodeNameType to_type, int to_value,
                   const std::string &preferred_from = {},
                   const std::string &preferred_to = {}) {
  if (!type) {
    return nullptr;
  }
  const std::vector<fpga::CBConnName> *conns =
      type->connNames(from_type, from_value, to_type, to_value);
  if (!conns || conns->empty()) {
    return nullptr;
  }
  auto matches = [&](const fpga::CBConnName &conn, bool match_from,
                     bool match_to) {
    return (!match_from || conn.from == preferred_from) &&
           (!match_to || conn.to == preferred_to);
  };
  if (!preferred_from.empty() && !preferred_to.empty()) {
    for (const fpga::CBConnName &conn : *conns) {
      if (matches(conn, true, true)) {
        return &conn;
      }
    }
    return nullptr;
  }
  if (!preferred_from.empty()) {
    for (const fpga::CBConnName &conn : *conns) {
      if (matches(conn, true, false)) {
        return &conn;
      }
    }
    return nullptr;
  }
  if (!preferred_to.empty()) {
    for (const fpga::CBConnName &conn : *conns) {
      if (matches(conn, false, true)) {
        return &conn;
      }
    }
    return nullptr;
  }
  return &conns->front();
}

constexpr int ROUTE_POS_SOURCE = 0;
constexpr int ROUTE_POS_TRANSIT = 1;
constexpr int ROUTE_POS_FORK = 2;

bool leaseConcreteOut(CBState &cb, int local, int src, int joint = -1,
                      bool ignore_deadend = false, int joint2 = -1) {
  if ((!ignore_deadend && cb.src_deadend.jump.testBit(src)) ||
      cb.src.jump.testBit(src) ||
      (joint >= 0 && cb.joint.jump.testBit(joint)) ||
      (joint2 >= 0 && cb.joint.jump.testBit(joint2))) {
    return false;
  }
  (void)local;
  cb.src.jump.setBit(src);
  if (joint >= 0) {
    cb.joint.jump.setBit(joint);
  }
  if (joint2 >= 0) {
    cb.joint.jump.setBit(joint2);
  }
  return true;
}

bool leaseConcreteJump(CBState &cb, int dst, int src, int joint = -1,
                       bool ignore_deadend = false, int joint2 = -1) {
  if ((!ignore_deadend && cb.src_deadend.jump.testBit(src)) ||
      cb.dst.jump.testBit(dst) || cb.src.jump.testBit(src) ||
      (joint >= 0 && cb.joint.jump.testBit(joint)) ||
      (joint2 >= 0 && cb.joint.jump.testBit(joint2))) {
    return false;
  }
  cb.dst.jump.setBit(dst);
  cb.src.jump.setBit(src);
  if (joint >= 0) {
    cb.joint.jump.setBit(joint);
  }
  if (joint2 >= 0) {
    cb.joint.jump.setBit(joint2);
  }
  return true;
}

bool leaseConcreteFork(CBState &cb, int dst, int src, int joint = -1,
                       bool ignore_deadend = false, int joint2 = -1) {
  return pnr::leaseExistingDestinationFork(cb, dst, src, joint, ignore_deadend,
                                           joint2);
}

enum class ConcreteBusyReason {
  none,
  dst,
  dst_missing,
  src,
  local,
  joint,
  src_deadend,
};

ConcreteBusyReason concreteJumpBusyReason(const CBState &cb, int dst, int src,
                                          int joint = -1) {
  if (cb.src_deadend.jump.testBit(src)) {
    return ConcreteBusyReason::src_deadend;
  }
  if (cb.dst.jump.testBit(dst)) {
    return ConcreteBusyReason::dst;
  }
  if (cb.src.jump.testBit(src)) {
    return ConcreteBusyReason::src;
  }
  if (joint >= 0 && cb.joint.jump.testBit(joint)) {
    return ConcreteBusyReason::joint;
  }
  return ConcreteBusyReason::none;
}

ConcreteBusyReason concreteOutBusyReason(const CBState &cb, int src,
                                         int joint = -1) {
  if (cb.src_deadend.jump.testBit(src)) {
    return ConcreteBusyReason::src_deadend;
  }
  if (cb.src.jump.testBit(src)) {
    return ConcreteBusyReason::src;
  }
  if (joint >= 0 && cb.joint.jump.testBit(joint)) {
    return ConcreteBusyReason::joint;
  }
  return ConcreteBusyReason::none;
}

ConcreteBusyReason concreteForkBusyReason(const CBState &cb, int dst, int src,
                                          int joint = -1,
                                          int joint2 = -1) {
  if (!cb.dst.jump.testBit(dst)) {
    return ConcreteBusyReason::dst_missing;
  }
  if (cb.src_deadend.jump.testBit(src)) {
    return ConcreteBusyReason::src_deadend;
  }
  if (cb.src.jump.testBit(src)) {
    return ConcreteBusyReason::src;
  }
  if ((joint >= 0 && cb.joint.jump.testBit(joint)) ||
      (joint2 >= 0 && cb.joint.jump.testBit(joint2))) {
    return ConcreteBusyReason::joint;
  }
  return ConcreteBusyReason::none;
}

ConcreteBusyReason concreteInBusyReason(const CBState &cb, int dst, int local) {
  if (cb.dst.jump.testBit(dst)) {
    return ConcreteBusyReason::dst;
  }
  if (cb.local.local.testBit(local)) {
    return ConcreteBusyReason::local;
  }
  return ConcreteBusyReason::none;
}

std::string concreteSrcWireName(const Tile &tile,
                                fpga::CBNodeNameType from_type, int from_value,
                                int src_node, int joint,
                                const std::string &incoming_wire);

const char *busyReasonName(ConcreteBusyReason reason) {
  switch (reason) {
  case ConcreteBusyReason::none:
    return "none";
  case ConcreteBusyReason::dst:
    return "dst_busy";
  case ConcreteBusyReason::dst_missing:
    return "dst_missing";
  case ConcreteBusyReason::src:
    return "src_busy";
  case ConcreteBusyReason::local:
    return "local_busy";
  case ConcreteBusyReason::joint:
    return "joint_busy";
  case ConcreteBusyReason::src_deadend:
    return "src_deadend";
  }
  return "unknown";
}

std::string nodeDebugName(const Tile &tile, fpga::CBNodeNameType type,
                          int node) {
  if (!tile.cb_type || node < 0) {
    return {};
  }
  const std::string *name = tile.cb_type->nodeName(type, node);
  return name ? *name : std::string{};
}

std::string netDebugName(rtl::Net *net) {
  return net ? net->makeName(FULL_NAME_LIMIT) : std::string{};
}

void brutalLocalExitFailure(Tile &tile, const std::vector<uint16_t> *src_nodes,
                            int local, const Coord &target_coord,
                            rtl::Net *current_net,
                            RouteDesign::RouteStats *stats) {
  if (!std::getenv("SCALEPNR_ROUTE_BRUTAL_DEBUG")) {
    return;
  }
  PNR_LOG1("ROUT",
           "BRUTAL local-exit failure: net='{}', tile='{}', coord=({},{}), "
           "local={} '{}', target=({},{}), src_count={}",
           netDebugName(current_net), tile.makeName(), tile.coord.x,
           tile.coord.y, local, nodeDebugName(tile, fpga::CB_NODE_LOCAL, local),
           target_coord.x, target_coord.y, src_nodes ? src_nodes->size() : 0);
  PNR_LOG1("ROUT",
           "BRUTAL tile masks: src={}, dst={}, local={}, src_deadend={}",
           tile.cb.src.jump.str(), tile.cb.dst.jump.str(),
           tile.cb.local.local.str(), tile.cb.src_deadend.jump.str());
  if (stats) {
    PNR_LOG1("ROUT",
             "BRUTAL stats: edge_trials={}, edge_ok={}, "
             "reject(name={},busy={},busy_dst={},busy_src={},busy_local={},"
             "target={},deadend={},src_deadend={}), no_src={}, failed={}",
             stats->edge_trials, stats->edge_accepted,
             stats->edge_rejected_no_name, stats->edge_rejected_busy,
             stats->edge_rejected_busy_dst, stats->edge_rejected_busy_src,
             stats->edge_rejected_busy_local, stats->edge_rejected_no_target,
             stats->edge_rejected_deadend, stats->edge_rejected_src_deadend,
             stats->no_src_nodes, stats->failed);
  }
  if (src_nodes) {
    for (uint16_t src_node : *src_nodes) {
      ConcreteBusyReason reason = concreteOutBusyReason(tile.cb, src_node);
      rtl::Net *src_owner =
          fpga::findNetByNode(tile, fpga::CB_NODE_SRC, src_node, false);
      rtl::Net *transit_owner =
          fpga::findNetByNode(tile, fpga::CB_NODE_SRC, src_node, true);
      int joint = -1;
      std::string src_wire = concreteSrcWireName(tile, fpga::CB_NODE_LOCAL,
                                                 local, src_node, joint, {});
      fpga::TileJumpTarget target =
          fpga::Device::current().resolveJump(tile, src_node);
      PNR_LOG1(
          "ROUT",
          "BRUTAL src candidate: src={} '{}', joint={}, reason={}, owner='{}', "
          "transit_owner='{}', concrete='{}', target=({},{}):{} '{}'",
          static_cast<int>(src_node),
          nodeDebugName(tile, fpga::CB_NODE_SRC, src_node), joint,
          busyReasonName(reason), netDebugName(src_owner),
          netDebugName(transit_owner), src_wire,
          target.tile ? target.tile->coord.x : -1,
          target.tile ? target.tile->coord.y : -1, target.dst_node,
          target.dst_wire);
    }
  }
}

bool leaseConcreteIn(CBState &cb, int dst, int local, int joint = -1,
                     int joint2 = -1) {
  if (cb.dst.jump.testBit(dst) || cb.local.local.testBit(local) ||
      (joint >= 0 && cb.joint.jump.testBit(joint)) ||
      (joint2 >= 0 && cb.joint.jump.testBit(joint2))) {
    return false;
  }
  cb.dst.jump.setBit(dst);
  cb.local.local.setBit(local);
  if (joint >= 0) {
    cb.joint.jump.setBit(joint);
  }
  if (joint2 >= 0) {
    cb.joint.jump.setBit(joint2);
  }
  return true;
}

bool leaseConcreteTerminal(CBState &cb, int node, int pin,
                           const TerminalEntryCandidate &entry,
                           bool allow_existing_dst) {
  if (entry.kind == TerminalEntryKind::dst) {
    if (allow_existing_dst) {
      if (cb.dst.jump.testBit(node)) {
        cb.dst.jump.clearBit(node);
        bool ok = leaseConcreteIn(cb, node, pin, entry.joint, entry.joint2);
        cb.dst.jump.setBit(node);
        return ok;
      }
    }
    return leaseConcreteIn(cb, node, pin, entry.joint, entry.joint2);
  }
  if (entry.kind != TerminalEntryKind::local) {
    return false;
  }

  if (cb.local.local.testBit(node) || cb.local.local.testBit(pin) ||
      (entry.joint >= 0 && cb.joint.jump.testBit(entry.joint)) ||
      (entry.joint2 >= 0 && cb.joint.jump.testBit(entry.joint2))) {
    return false;
  }
  cb.local.local.setBit(node);
  cb.local.local.setBit(pin);
  if (entry.joint >= 0) {
    cb.joint.jump.setBit(entry.joint);
  }
  if (entry.joint2 >= 0) {
    cb.joint.jump.setBit(entry.joint2);
  }
  return true;
}

void fillTilePinEndpoint(Wire &wire, rtl::Inst &inst, const std::string &port,
                         fpga::TilePinNameType dir);
void refreshTilePinEndpoint(Wire &wire, rtl::Inst &inst,
                            const std::string &port, fpga::TilePinNameType dir);
void prependSourceEndpoint(std::vector<Wire> &route, rtl::Inst &from,
                           const std::string &from_port);
bool sourceLocalOwnedByDifferentEndpoint(Tile &route_tile, int local,
                                         rtl::Inst &from,
                                         const std::string &from_port,
                                         rtl::Net *net);

std::string concreteSrcWireName(const Tile &tile,
                                fpga::CBNodeNameType from_type, int from_value,
                                int src_node, int joint,
                                const std::string &from_wire_name = {}) {
  if (!tile.cb_type) {
    return {};
  }
  if (joint >= 0) {
    std::string joint_name;
    if (const fpga::CBConnName *first =
            selectConcreteConn(tile.cb_type, from_type, from_value,
                               fpga::CB_NODE_JOINT, joint, from_wire_name)) {
      if (!from_wire_name.empty() && first->from != from_wire_name) {
        return {};
      }
      joint_name = first->to;
    } else if (!from_wire_name.empty()) {
      return {};
    }
    if (const fpga::CBConnName *second =
            selectConcreteConn(tile.cb_type, fpga::CB_NODE_JOINT, joint,
                               fpga::CB_NODE_SRC, src_node, joint_name)) {
      return second->to;
    }
    if (const fpga::CBConnName *second =
            selectConcreteConn(tile.cb_type, fpga::CB_NODE_SRC, src_node,
                               fpga::CB_NODE_JOINT, joint, {}, joint_name)) {
      return second->from;
    }
  }
  if (const fpga::CBConnName *conn =
          selectConcreteConn(tile.cb_type, from_type, from_value,
                             fpga::CB_NODE_SRC, src_node, from_wire_name)) {
    if (!from_wire_name.empty() && conn->from != from_wire_name) {
      return {};
    }
    return conn->to;
  }
  if (!from_wire_name.empty()) {
    return {};
  }
  if (const std::string *src =
          tile.cb_type->nodeName(fpga::CB_NODE_SRC, src_node)) {
    return *src;
  }
  return {};
}

bool hasRoutableExitFromDst(Tile &tile, int dst_node) {
  if (!tile.cb_type || dst_node < 0) {
    return false;
  }
  tile.cb_type->ensureDerivedMasks();
  const std::vector<uint16_t> *src_nodes =
      tile.cb_type->srcNodes(fpga::CB_NODE_DST, dst_node);
  if (!src_nodes) {
    return false;
  }
  for (uint16_t src_node : *src_nodes) {
    if (tile.cb.src_deadend.jump.testBit(src_node)) {
      continue;
    }
    int joint = -1;
    if (!tile.cb_type->dst_src[dst_node].jump.testBit(src_node)) {
      NodeMask dst_to_joints = tile.cb_type->dst_joint[dst_node].joint;
      NodeMask joints_to_src = tile.cb_type->src_joint[src_node].joint;
      NodeMask intersect = dst_to_joints & joints_to_src;
      joint = intersect.firstSetBit();
      if (joint < 0) {
        dst_to_joints.for_each_set_bit([&](int index) {
          joint = (joints_to_src & tile.cb_type->joint_joint[index].joint)
                      .firstSetBit();
          return joint >= 0;
        });
      }
    }
    if (joint < 0 &&
        !tile.cb_type->dst_src[dst_node].jump.testBit(src_node)) {
      continue;
    }
    CBState test_cb = tile.cb;
    if (!leaseConcreteJump(test_cb, dst_node, src_node, joint)) {
      continue;
    }
    fpga::TileJumpTarget target =
        fpga::Device::current().resolveJump(tile, src_node);
    if (target.tile && target.tile->cb_type && target.dst_node >= 0) {
      return true;
    }
  }
  return false;
}

std::vector<TerminalEntryCandidate> targetEntryCandidates(const CBType &cb_type,
                                                          int node, int local) {
  const_cast<CBType &>(cb_type).ensureDerivedMasks();
  std::vector<TerminalEntryCandidate> entries;
  auto add_entry = [&](TerminalEntryKind kind, int joint, int joint2 = -1) {
    auto same = [&](const TerminalEntryCandidate &entry) {
      return entry.kind == kind && entry.joint == joint &&
             entry.joint2 == joint2;
    };
    if (std::find_if(entries.begin(), entries.end(), same) == entries.end()) {
      entries.push_back(TerminalEntryCandidate{kind, joint, joint2});
    }
  };

  if (cb_type.dst_local[node].local.testBit(local)) {
    add_entry(TerminalEntryKind::dst, -1);
  }
  NodeMask joints_to_local = cb_type.local_reachable_joints[local].joint;
  NodeMask direct_joints = cb_type.dst_joint[node].joint & joints_to_local;
  direct_joints.for_each_set_bit([&](int joint) {
    add_entry(TerminalEntryKind::dst, joint);
    return false;
  });
  NodeMask first_joints = cb_type.dst_joint[node].joint;
  first_joints.for_each_set_bit([&](int first_joint) {
    NodeMask second_joints =
        cb_type.joint_joint[first_joint].joint & joints_to_local;
    second_joints.for_each_set_bit([&](int second_joint) {
      add_entry(TerminalEntryKind::dst, second_joint, first_joint);
      return false;
    });
    return false;
  });
  return entries;
}

int directionIndexForDelta(int dx, int dy) {
  int sx = (dx > 0) - (dx < 0);
  int sy = (dy > 0) - (dy < 0);
  if (sx > 0 && sy == 0) {
    return 0;
  }
  if (sx > 0 && sy < 0) {
    return 1;
  }
  if (sx == 0 && sy < 0) {
    return 2;
  }
  if (sx < 0 && sy < 0) {
    return 3;
  }
  if (sx < 0 && sy == 0) {
    return 4;
  }
  if (sx < 0 && sy > 0) {
    return 5;
  }
  if (sx == 0 && sy > 0) {
    return 6;
  }
  if (sx > 0 && sy > 0) {
    return 7;
  }
  return -1;
}

bool offsetMatchesDirection(int dx, int dy, int direction) {
  if (dx == 0 && dy == 0) {
    return direction < 0;
  }
  return directionIndexForDelta(dx, dy) == direction;
}

std::vector<Tile *> routeTileCandidates(rtl::Inst &inst,
                                        const std::string &port, bool output,
                                        const Coord *preferred_target) {
  std::vector<Tile *> candidates;
  constexpr size_t max_route_candidates = 64;
  auto add_candidate = [&](Tile *tile) {
    if (!tile) {
      return;
    }
    for (Tile *candidate : candidates) {
      if (candidate == tile) {
        return;
      }
    }
    candidates.push_back(tile);
  };

  if (!inst.tile.peer) {
    return candidates;
  }
  bool debug_candidates = false;
  if (routeDebugEnabled("SCALEPNR_DEBUG_ROUTE_CANDIDATES")) {
    debug_candidates =
        routeDebugMatches("SCALEPNR_DEBUG_ROUTE_CANDIDATES",
                          inst.makeName(FULL_NAME_LIMIT)) ||
        routeDebugMatches("SCALEPNR_DEBUG_ROUTE_CANDIDATES",
                          inst.cell_ref.peer ? inst.cell_ref->type
                                             : std::string{}) ||
        routeDebugMatches("SCALEPNR_DEBUG_ROUTE_CANDIDATES", port);
  }

  constexpr int endpoint_search_radius = 6;
  fpga::Device &device = fpga::Device::current();
  auto try_offset = [&](int dx, int dy) {
    if (candidates.size() >= max_route_candidates) {
      return;
    }
    int distance = std::abs(dx) + std::abs(dy);
    if (distance > endpoint_search_radius) {
      return;
    }
    Coord coord = inst.tile->coord + Coord{dx, dy};
    Tile *tile_ptr = device.getTile(coord.x, coord.y);
    if (!tile_ptr) {
      return;
    }
    Tile *route_tile = device.routeTile(*tile_ptr);
    if (!route_tile) {
      return;
    }
    Tile &tile = *route_tile;
    // Endpoint route-type annotations decide which nearby route tile owns the
    // local node.
    NodeMask candidate_nodes = routeTileEndpointNodes(tile, inst, port, output);
    if (candidate_nodes == NodeMask{}) {
      if (debug_candidates) {
        fpga::TilePinNameType dir =
            output ? fpga::TILE_PIN_OUTPUT : fpga::TILE_PIN_INPUT;
        NodeMask tile_type_nodes =
            tile.tile_type
                ? inst.tile->getPinNodesForRouteType(
                      inst.cell_ref->type, port, inst.pos, dir,
                      tile.tile_type->name, tile.coord - inst.tile->coord)
                : NodeMask{};
        Coord cb_delta = tile.cb_coord.x >= 0 && tile.cb_coord.y >= 0
                             ? tile.cb_coord - inst.tile->coord
                             : tile.coord - inst.tile->coord;
        NodeMask cb_type_nodes = tile.cb_type
                                     ? inst.tile->getPinNodesForRouteType(
                                           inst.cell_ref->type, port, inst.pos,
                                           dir, tile.cb_type->name, cb_delta)
                                     : NodeMask{};
        PNR_LOG2("ROUT",
                 "routeTileCandidates reject empty: inst='{}' type='{}' "
                 "port='{}' output={} cand=({}, {}) name='{}' tile_type='{}' "
                 "cb_type='{}' distance={} tile_type_nodes={} cb_type_nodes={}",
                 inst.makeName(FULL_NAME_LIMIT),
                 inst.cell_ref.peer ? inst.cell_ref->type : std::string{}, port,
                 output, tile.coord.x, tile.coord.y, tileNameForDump(&tile),
                 tile.tile_type ? tile.tile_type->name : std::string{},
                 tile.cb_type ? tile.cb_type->name : std::string{}, distance,
                 maskString(tile_type_nodes), maskString(cb_type_nodes));
      }
      return;
    }
    if (!(output ? supportsOutputLocalNodes(tile, candidate_nodes)
                 : supportsLocalNodes(tile, candidate_nodes))) {
      if (debug_candidates) {
        PNR_LOG1("ROUT",
                 "routeTileCandidates reject unsupported: inst='{}' type='{}' "
                 "port='{}' output={} cand=({}, {}) name='{}' tile_type='{}' "
                 "cb_type='{}' nodes={}",
                 inst.makeName(FULL_NAME_LIMIT),
                 inst.cell_ref.peer ? inst.cell_ref->type : std::string{}, port,
                 output, tile.coord.x, tile.coord.y, tileNameForDump(&tile),
                 tile.tile_type ? tile.tile_type->name : std::string{},
                 tile.cb_type ? tile.cb_type->name : std::string{},
                 maskString(candidate_nodes));
      }
      return;
    }
    // Input endpoint candidates must have a real incoming jump destination that
    // can reach the pin local.
    if (!output && !routeTileHasIncomingForLocalNodes(tile, candidate_nodes)) {
      if (debug_candidates) {
        PNR_LOG1("ROUT",
                 "routeTileCandidates reject no-incoming: inst='{}' type='{}' "
                 "port='{}' output={} cand=({}, {}) name='{}' tile_type='{}' "
                 "cb_type='{}' nodes={} incoming_dsts={}",
                 inst.makeName(FULL_NAME_LIMIT),
                 inst.cell_ref.peer ? inst.cell_ref->type : std::string{}, port,
                 output, tile.coord.x, tile.coord.y, tileNameForDump(&tile),
                 tile.tile_type ? tile.tile_type->name : std::string{},
                 tile.cb_type ? tile.cb_type->name : std::string{},
                 maskString(candidate_nodes),
                 maskString(incomingDstMaskForRouteTile(tile)));
      }
      return;
    }
    if (debug_candidates) {
      PNR_LOG1("ROUT",
               "routeTileCandidates accept: inst='{}' type='{}' port='{}' "
               "output={} cand=({}, {}) name='{}' tile_type='{}' cb_type='{}' "
               "nodes={} score={}",
               inst.makeName(FULL_NAME_LIMIT),
               inst.cell_ref.peer ? inst.cell_ref->type : std::string{}, port,
               output, tile.coord.x, tile.coord.y, tileNameForDump(&tile),
               tile.tile_type ? tile.tile_type->name : std::string{},
               tile.cb_type ? tile.cb_type->name : std::string{},
               maskString(candidate_nodes), distance * 4);
    }
    add_candidate(&tile);
  };

  constexpr int direction_order_offsets[8] = {0, -1, 1, -2, 2, -3, 3, 4};
  int base_direction = -1;
  if (preferred_target) {
    Coord delta = *preferred_target - inst.tile->coord;
    base_direction = directionIndexForDelta(delta.x, delta.y);
  }
  // A resource tile may own its crossbar endpoint directly. Directional
  // ordering applies only to the remaining nearby candidates.
  try_offset(0, 0);
  if (base_direction >= 0) {
    for (int order_offset : direction_order_offsets) {
      int direction = (base_direction + order_offset + 8) & 7;
      for (int distance = 1; distance <= endpoint_search_radius; ++distance) {
        for (int dy = -distance; dy <= distance; ++dy) {
          for (int dx = -distance; dx <= distance; ++dx) {
            if (std::abs(dx) + std::abs(dy) != distance) {
              continue;
            }
            if (!offsetMatchesDirection(dx, dy, direction)) {
              continue;
            }
            try_offset(dx, dy);
          }
        }
      }
    }
  } else {
    for (int distance = 1; distance <= endpoint_search_radius; ++distance) {
      for (int dy = -distance; dy <= distance; ++dy) {
        for (int dx = -distance; dx <= distance; ++dx) {
          if (std::abs(dx) + std::abs(dy) != distance) {
            continue;
          }
          try_offset(dx, dy);
        }
      }
    }
  }
  if (candidates.empty() && inst.tile->full_name.empty() &&
      isConcreteRouteTile(*inst.tile)) {
    add_candidate(&*inst.tile);
  }
  return candidates;
}

std::string netOwnerName(rtl::Net *net) {
  return net ? net->makeName(FULL_NAME_LIMIT) : std::string{};
}

std::string netRouteBindingSummary(rtl::Net *net) {
  // Include the physical route endpoints in congestion diagnostics.
  if (!net || net->routes.empty()) {
    return {};
  }
  const rtl::NetRouteBinding &binding = net->routes.front();
  auto endpoint = [](rtl::Inst *inst, const std::string &port) {
    return std::format("{}:{} type={} pos={} tile=({},{}) cb=({},{})",
                       instNameForDump(inst), port,
                       inst && inst->cell_ref.peer ? inst->cell_ref->type
                                                   : std::string{},
                       inst ? inst->pos : -1,
                       inst && inst->tile.peer ? inst->tile->coord.x : -1,
                       inst && inst->tile.peer ? inst->tile->coord.y : -1,
                       inst && inst->tile.peer ? inst->tile->cb_coord.x : -1,
                       inst && inst->tile.peer ? inst->tile->cb_coord.y : -1);
  };
  return endpoint(binding.from, binding.from_port) + " -> " +
         endpoint(binding.to, binding.to_port);
}

std::string netNodeBindingSummary(rtl::Net *net, const Tile &tile,
                                  fpga::CBNodeNameType type, int node) {
  // Find the exact branch binding that owns one leased crossbar node.
  if (!net || node < 0) {
    return {};
  }
  for (rtl::NetRouteBinding &binding : net->routes) {
    std::vector<Wire> *route = bindingRouteForDump(binding);
    if (!route) {
      continue;
    }
    bool owns =
        std::any_of(route->begin(), route->end(), [&](const Wire &wire) {
          if (wire.type != Wire::WIRE_CROSSBAR ||
              !sameCoord(wire.from, tile.coord)) {
            return false;
          }
          if (type == fpga::CB_NODE_JOINT) {
            return wire.joint == node || wire.joint2 == node;
          }
          if (type == fpga::CB_NODE_SRC) {
            return wire.jump == node;
          }
          return type == fpga::CB_NODE_DST && wire.local == node &&
                 wire.pos != ROUTE_POS_SOURCE;
        });
    if (!owns) {
      continue;
    }
    rtl::Net single_binding_net;
    single_binding_net.routes.push_back(binding);
    return netRouteBindingSummary(&single_binding_net);
  }
  return netRouteBindingSummary(net);
}

NodeMask incomingDstMaskForRouteTile(Tile &tile) {
  fpga::Device &device = fpga::Device::current();
  Tile *route_tile = device.routeTile(tile);
  return route_tile ? route_tile->incoming_dst_nodes : NodeMask{};
}

bool routeTileHasIncomingForLocalNodes(Tile &tile, NodeMask local_nodes) {
  if (!tile.cb_type || local_nodes == NodeMask{}) {
    return false;
  }
  tile.cb_type->ensureDerivedMasks();
  NodeMask incoming_dsts = incomingDstMaskForRouteTile(tile);
  bool reachable = false;
  local_nodes.for_each_set_bit([&](int local) {
    if ((tile.cb_type->dsts_reaching_local[local].jump & incoming_dsts) !=
        NodeMask{}) {
      reachable = true;
      return true;
    }
    return false;
  });
  return reachable;
}

void logTargetTileEntryTable(const RouteDesign::RouteTask &task,
                             const char *context) {
  if (!task.to || !task.to->tile.peer || !task.to->cell_ref.peer) {
    PNR_LOG1("ROUT",
             "routeDesign {} target table skipped: missing destination "
             "placement for net='{}'",
             context, task.net_name);
    return;
  }

  std::vector<Tile *> to_tiles =
      routeTileCandidates(*task.to, task.to_port, false);
  for (Tile *tile : to_tiles) {
    if (!tile || !tile->cb_type) {
      continue;
    }
    NodeMask pin_nodes = routeTileInputNodes(*tile, *task.to, task.to_port);
    if (pin_nodes == NodeMask{}) {
      continue;
    }

    PNR_LOG1("ROUT",
             "routeDesign {} target tile: net='{}', target='{}'/'{}', "
             "tile=({},{}), tile_name='{}', tile_type='{}', cb_type='{}'",
             context, task.net_name, task.to->makeName(FULL_NAME_LIMIT),
             task.to_port, tile->coord.x, tile->coord.y, tileNameForDump(tile),
             tile->tile_type ? tile->tile_type->name : std::string{},
             tile->cb_type->name);
    PNR_LOG1("ROUT",
             "routeDesign {} target tile state masks: leased_src={}, "
             "leased_dst={}, leased_joint={}, leased_local={}, pin_leased={}, "
             "src_deadend={}, routed_nets={}",
             context, maskString(tile->cb.src.jump),
             maskString(tile->cb.dst.jump), maskString(tile->cb.joint.jump),
             maskString(tile->cb.local.local),
             maskString(tile->pin_state.leased_nodes),
             maskString(tile->cb.src_deadend.jump), tile->routedNets.size());
    size_t routed_index = 0;
    for (const Ref<rtl::Net> &routed_ref : tile->routedNets) {
      if (routed_ref.peer) {
        PNR_LOG1("ROUT", "routeDesign {} target routed_net[{}]='{}'", context,
                 routed_index, routed_ref.peer->makeName(FULL_NAME_LIMIT));
      }
      ++routed_index;
    }

    pin_nodes.for_each_set_bit([&](int pin) {
      PNR_LOG1("ROUT",
               "routeDesign {} entry table for local={} '{}' on tile=({},{})",
               context, pin, nodeNameForDump(*tile, fpga::CB_NODE_LOCAL, pin),
               tile->coord.x, tile->coord.y);
      tile->cb_type->ensureDerivedMasks();
      NodeMask dst_candidates = tile->cb_type->dsts_reaching_local[pin].jump;
      PNR_LOG1(
          "ROUT",
          "routeDesign {} entry resource masks: local={}, "
          "dsts_reaching_local={}, local_input_nodes={}, valid_dst_nodes={}",
          context, pin, maskString(dst_candidates),
          maskString(tile->cb_type->local_input_nodes),
          maskString(tile->cb_type->valid_dst_nodes));
      PNR_LOG1("ROUT",
               "routeDesign {} entry table columns: "
               "dst,dst_name,joint,joint_name,lease_ok,dst_leased,joint_leased,"
               "local_leased,dst_owner,dst_transit,joint_owner,joint_binding,"
               "joint_transit,local_owner,local_transit,joint2,joint2_name,"
               "joint2_leased,joint2_owner,joint2_binding,physical_dst",
               context);
      dst_candidates.for_each_set_bit([&](int dst) {
        // Keep both joints: dropping the second reports blocked paths as free.
        for (const TerminalEntryCandidate &entry :
             targetEntryCandidates(*tile->cb_type, dst, pin)) {
          if (entry.kind != TerminalEntryKind::dst) {
            continue;
          }
          const int joint = entry.joint;
          const int joint2 = entry.joint2;
          CBState test_cb = tile->cb;
          const bool physical_dst = tile->incoming_dst_nodes == NodeMask{} ||
                                    tile->incoming_dst_nodes.testBit(dst);
          bool lease_ok = physical_dst && !tile->isPinNodeLeased(pin) &&
                         leaseConcreteIn(test_cb, dst, pin, joint, joint2);
          bool dst_leased =
              (tile->cb.dst.jump & (NodeMask{0, 1} << dst)) != NodeMask{};
          bool joint_leased =
              joint >= 0 &&
              (tile->cb.joint.jump & (NodeMask{0, 1} << joint)) != NodeMask{};
          bool local_leased =
              tile->isPinNodeLeased(pin) ||
              (tile->cb.local.local & (NodeMask{0, 1} << pin)) != NodeMask{};
          rtl::Net *dst_owner =
              fpga::findNetByNode(*tile, fpga::CB_NODE_DST, dst, false);
          rtl::Net *dst_transit =
              fpga::findNetByNode(*tile, fpga::CB_NODE_DST, dst, true);
          rtl::Net *joint_owner =
              joint >= 0 ? fpga::findNetByNode(*tile, fpga::CB_NODE_JOINT,
                                               joint, false)
                         : nullptr;
          rtl::Net *joint_transit =
              joint >= 0
                  ? fpga::findNetByNode(*tile, fpga::CB_NODE_JOINT, joint, true)
                  : nullptr;
          rtl::Net *local_owner =
              fpga::findNetByNode(*tile, fpga::CB_NODE_LOCAL, pin, false);
          rtl::Net *local_transit =
              fpga::findNetByNode(*tile, fpga::CB_NODE_LOCAL, pin, true);
          rtl::Net *joint2_owner = joint2 >= 0
              ? fpga::findNetByNode(*tile, fpga::CB_NODE_JOINT, joint2, false)
              : nullptr;
          PNR_LOG1(
              "ROUT",
              "routeDesign {} entry row: "
              "{},'{}',{},'{}',{},{},{},{},'{}','{}','{}','{}','{}','{}','{}',"
              "{},'{}',{},'{}','{}',{}",
              context, dst, nodeNameForDump(*tile, fpga::CB_NODE_DST, dst),
              joint,
              joint >= 0 ? nodeNameForDump(*tile, fpga::CB_NODE_JOINT, joint)
                         : std::string{"direct"},
              lease_ok, dst_leased, joint_leased, local_leased,
              netOwnerName(dst_owner), netOwnerName(dst_transit),
              netOwnerName(joint_owner),
              netNodeBindingSummary(joint_owner, *tile, fpga::CB_NODE_JOINT,
                                    joint),
              netOwnerName(joint_transit), netOwnerName(local_owner),
              netOwnerName(local_transit), joint2,
              joint2 >= 0 ? nodeNameForDump(*tile, fpga::CB_NODE_JOINT, joint2)
                          : std::string{},
              joint2 >= 0 && tile->cb.joint.jump.testBit(joint2),
              netOwnerName(joint2_owner),
              netNodeBindingSummary(joint2_owner, *tile, fpga::CB_NODE_JOINT,
                                    joint2), physical_dst);
        }
        return false;
      });
      return false;
    });
  }
}

void logDockingBackwardState(const RouteDesign::RouteTask &task,
                             Tile &forward_tile, int forward_dst,
                             Tile &target_tile, NodeMask pin_nodes,
                             int radius) {
  if (!forward_tile.cb_type || !target_tile.cb_type ||
      pin_nodes == NodeMask{}) {
    return;
  }
  static std::unordered_set<std::string> reported_routes;
  std::string report_key =
      std::format("{}:{}:{}:{}:{}:{}", task.net_name, forward_tile.coord.x,
                  forward_tile.coord.y, forward_dst, target_tile.coord.x,
                  target_tile.coord.y);
  if (!reported_routes.insert(std::move(report_key)).second) {
    return;
  }

  struct IncomingOption {
    Tile *tile = nullptr;
    int dst = -1;
    int src = -1;
    int joint = -1;
    int joint2 = -1;
    bool lease_ok = false;
  };
  struct TargetEntry {
    int local = -1;
    int dst = -1;
    int joint = -1;
    int joint2 = -1;
    bool lease_ok = false;
    std::vector<IncomingOption> incoming;
  };

  target_tile.cb_type->ensureDerivedMasks();
  std::vector<TargetEntry> entries;
  NodeMask target_dsts;
  pin_nodes.for_each_set_bit([&](int local) {
    NodeMask dsts = target_tile.cb_type->dsts_reaching_local[local].jump;
    dsts.for_each_set_bit([&](int dst) {
      for (const TerminalEntryCandidate &candidate :
           targetEntryCandidates(*target_tile.cb_type, dst, local)) {
        if (candidate.kind != TerminalEntryKind::dst) {
          continue;
        }
        CBState test_cb = target_tile.cb;
        bool lease_ok = leaseConcreteIn(test_cb, dst, local, candidate.joint,
                                        candidate.joint2);
        entries.push_back(TargetEntry{
            local, dst, candidate.joint, candidate.joint2, lease_ok, {}});
        target_dsts |= NodeMask{0, 1} << dst;
      }
      return false;
    });
    return false;
  });

  fpga::Device &device = fpga::Device::current();
  for (int y = target_tile.coord.y - radius; y <= target_tile.coord.y + radius;
       ++y) {
    for (int x = target_tile.coord.x - radius;
         x <= target_tile.coord.x + radius; ++x) {
      Tile *previous = device.getTile(x, y);
      if (!previous || !previous->cb_type) {
        continue;
      }
      Coord previous_route_coord =
          previous->cb_coord.x >= 0 && previous->cb_coord.y >= 0
              ? previous->cb_coord
              : previous->coord;
      if (!sameCoord(previous_route_coord, previous->coord)) {
        continue;
      }
      previous->cb_type->ensureDerivedMasks();
      for (const auto &[src_key, mappings] :
           previous->cb_type->dst_by_src.values) {
        if (mappings.empty()) {
          continue;
        }
        int src = src_key;
        std::vector<fpga::TileJumpTarget> targets =
            device.resolveJumpTargets(*previous, src);
        for (const fpga::TileJumpTarget &target : targets) {
          if (target.tile != &target_tile || target.dst_node < 0 ||
              (target_dsts & (NodeMask{0, 1} << target.dst_node)) ==
                  NodeMask{}) {
            continue;
          }
          NodeMask previous_dsts =
              previous->cb_type->dsts_reaching_src[src].jump;
          previous_dsts.for_each_set_bit([&](int previous_dst) {
            int joint = -1;
            int joint2 = -1;
            if (!previous->cb_type->canJump(previous_dst, src, src, joint,
                                            &joint2)) {
              return false;
            }
            CBState test_cb = previous->cb;
            bool lease_ok = leaseConcreteJump(test_cb, previous_dst, src, joint,
                                              true, joint2);
            for (TargetEntry &entry : entries) {
              if (entry.dst != target.dst_node) {
                continue;
              }
              auto duplicate = [&](const IncomingOption &option) {
                return option.tile == previous && option.dst == previous_dst &&
                       option.src == src && option.joint == joint &&
                       option.joint2 == joint2;
              };
              if (std::find_if(entry.incoming.begin(), entry.incoming.end(),
                               duplicate) == entry.incoming.end()) {
                entry.incoming.push_back(IncomingOption{
                    previous, previous_dst, src, joint, joint2, lease_ok});
              }
            }
            return false;
          });
        }
      }
    }
  }

  bool has_free_reachable =
      std::any_of(entries.begin(), entries.end(), [](const TargetEntry &entry) {
        return entry.lease_ok &&
               std::any_of(entry.incoming.begin(), entry.incoming.end(),
                           [](const IncomingOption &option) {
                             return option.lease_ok;
                           });
      });

  PNR_LOG1(
      "ROUT",
      "docking backward state: net='{}', anchor=({},{}) dst={} '{}', "
      "target=({},{}) pin_nodes={}, radius={}, entries={}, free_reachable={}",
      task.net_name, forward_tile.coord.x, forward_tile.coord.y, forward_dst,
      nodeNameForDump(forward_tile, fpga::CB_NODE_DST, forward_dst),
      target_tile.coord.x, target_tile.coord.y,
      maskBitsForDump(pin_nodes, CB_MAX_NODES), radius, entries.size(),
      has_free_reachable);
  PNR_LOG1("ROUT",
           "docking backward anchor state: src={}, dst={}, joint={}, local={}, "
           "src_deadend={}",
           maskBitsForDump(forward_tile.cb.src.jump, CB_MAX_NODES),
           maskBitsForDump(forward_tile.cb.dst.jump, CB_MAX_NODES),
           maskBitsForDump(forward_tile.cb.joint.jump, CB_MAX_NODES),
           maskBitsForDump(forward_tile.cb.local.local, CB_MAX_NODES),
           maskBitsForDump(forward_tile.cb.src_deadend.jump, CB_MAX_NODES));
  PNR_LOG1("ROUT",
           "docking backward target state: src={}, dst={}, joint={}, local={}, "
           "pin_leased={}, src_deadend={}",
           maskBitsForDump(target_tile.cb.src.jump, CB_MAX_NODES),
           maskBitsForDump(target_tile.cb.dst.jump, CB_MAX_NODES),
           maskBitsForDump(target_tile.cb.joint.jump, CB_MAX_NODES),
           maskBitsForDump(target_tile.cb.local.local, CB_MAX_NODES),
           maskBitsForDump(target_tile.pin_state.leased_nodes, CB_MAX_NODES),
           maskBitsForDump(target_tile.cb.src_deadend.jump, CB_MAX_NODES));

  size_t free_unreachable = 0;
  size_t free_reachable = 0;
  size_t blocked_reachable = 0;
  size_t preemptible = 0;
  for (size_t entry_index = 0; entry_index < entries.size(); ++entry_index) {
    TargetEntry &entry = entries[entry_index];
    bool incoming_available = std::any_of(
        entry.incoming.begin(), entry.incoming.end(),
        [](const IncomingOption &option) { return option.lease_ok; });
    bool dst_leased =
        (target_tile.cb.dst.jump & (NodeMask{0, 1} << entry.dst)) != NodeMask{};
    bool joint_leased =
        entry.joint >= 0 && (target_tile.cb.joint.jump &
                             (NodeMask{0, 1} << entry.joint)) != NodeMask{};
    bool joint2_leased =
        entry.joint2 >= 0 && (target_tile.cb.joint.jump &
                              (NodeMask{0, 1} << entry.joint2)) != NodeMask{};
    bool local_leased = target_tile.isPinNodeLeased(entry.local) ||
                        (target_tile.cb.local.local &
                         (NodeMask{0, 1} << entry.local)) != NodeMask{};
    TransitVictim victim =
        findGroundingVictim(target_tile, entry.dst, entry.joint, task.net,
                            task.from, task.from_port, task.net_name);
    auto free_after_victim = [&](const IncomingOption &option) {
      auto blocked_by_other = [&](fpga::CBNodeNameType type, int node,
                                  bool leased) {
        if (!leased || node < 0) {
          return false;
        }
        return !victim.net || fpga::findNetByNode(*option.tile, type, node,
                                                  false) != victim.net;
      };
      bool previous_dst_leased = (option.tile->cb.dst.jump &
                                  (NodeMask{0, 1} << option.dst)) != NodeMask{};
      bool previous_src_leased = (option.tile->cb.src.jump &
                                  (NodeMask{0, 1} << option.src)) != NodeMask{};
      bool previous_joint_leased =
          option.joint >= 0 && (option.tile->cb.joint.jump &
                                (NodeMask{0, 1} << option.joint)) != NodeMask{};
      bool previous_joint2_leased =
          option.joint2 >= 0 &&
          (option.tile->cb.joint.jump & (NodeMask{0, 1} << option.joint2)) !=
              NodeMask{};
      return !blocked_by_other(fpga::CB_NODE_DST, option.dst,
                               previous_dst_leased) &&
             !blocked_by_other(fpga::CB_NODE_SRC, option.src,
                               previous_src_leased) &&
             !blocked_by_other(fpga::CB_NODE_JOINT, option.joint,
                               previous_joint_leased) &&
             !blocked_by_other(fpga::CB_NODE_JOINT, option.joint2,
                               previous_joint2_leased);
    };
    bool incoming_free_after_victim =
        victim.net && std::any_of(entry.incoming.begin(), entry.incoming.end(),
                                  free_after_victim);
    bool can_preempt = !has_free_reachable && incoming_free_after_victim &&
                       dst_leased && !local_leased && victim.net;

    const char *classification = "BLOCKED_UNREACHABLE";
    if (entry.lease_ok && incoming_available) {
      classification = "FREE_REACHABLE";
      ++free_reachable;
    } else if (entry.lease_ok) {
      classification = "FREE_UNREACHABLE";
      ++free_unreachable;
    } else if (can_preempt) {
      classification = "TRANSIT_PREEMPTIBLE";
      ++preemptible;
    } else if (!entry.incoming.empty()) {
      classification = "BLOCKED_REACHABLE";
      ++blocked_reachable;
    }

    PNR_LOG1(
        "ROUT",
        "docking backward entry[{}]: class={}, local={} '{}', dst={} '{}', "
        "joint={} '{}', joint2={} '{}', terminal_free={}, incoming={}, "
        "incoming_free={}, incoming_free_after_victim={}, "
        "leased(dst={},joint={},joint2={},local={}), "
        "owners(dst='{}',dst_transit='{}',joint='{}',joint_transit='{}',joint2="
        "'{}',joint2_transit='{}',local='{}'), victim='{}'",
        entry_index, classification, entry.local,
        nodeNameForDump(target_tile, fpga::CB_NODE_LOCAL, entry.local),
        entry.dst, nodeNameForDump(target_tile, fpga::CB_NODE_DST, entry.dst),
        entry.joint,
        entry.joint >= 0
            ? nodeNameForDump(target_tile, fpga::CB_NODE_JOINT, entry.joint)
            : std::string{"direct"},
        entry.joint2,
        entry.joint2 >= 0
            ? nodeNameForDump(target_tile, fpga::CB_NODE_JOINT, entry.joint2)
            : std::string{},
        entry.lease_ok, entry.incoming.size(), incoming_available,
        incoming_free_after_victim, dst_leased, joint_leased, joint2_leased,
        local_leased,
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_DST, entry.dst, false),
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_DST, entry.dst, true),
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_JOINT, entry.joint, false),
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_JOINT, entry.joint, true),
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_JOINT, entry.joint2,
                         false),
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_JOINT, entry.joint2, true),
        nodeOwnerForDump(&target_tile, fpga::CB_NODE_LOCAL, entry.local, false),
        victim.net ? victim.net->makeName(FULL_NAME_LIMIT) : std::string{});

    for (size_t incoming_index = 0; incoming_index < entry.incoming.size();
         ++incoming_index) {
      const IncomingOption &incoming = entry.incoming[incoming_index];
      Tile &previous = *incoming.tile;
      bool previous_dst_leased =
          (previous.cb.dst.jump & (NodeMask{0, 1} << incoming.dst)) !=
          NodeMask{};
      bool previous_src_leased =
          (previous.cb.src.jump & (NodeMask{0, 1} << incoming.src)) !=
          NodeMask{};
      bool previous_joint_leased =
          incoming.joint >= 0 &&
          (previous.cb.joint.jump & (NodeMask{0, 1} << incoming.joint)) !=
              NodeMask{};
      bool previous_joint2_leased =
          incoming.joint2 >= 0 &&
          (previous.cb.joint.jump & (NodeMask{0, 1} << incoming.joint2)) !=
              NodeMask{};
      PNR_LOG1(
          "ROUT",
          "docking backward entry[{}] incoming[{}]: prev=({},{}) tile='{}' "
          "cb='{}', dst={} '{}', src={} '{}', joint={} '{}', joint2={} '{}', "
          "path_free={}, leased(dst={},src={},joint={},joint2={}), deadend={}, "
          "owners(dst='{}',dst_transit='{}',src='{}',src_transit='{}',joint='{}"
          "',joint_transit='{}',joint2='{}',joint2_transit='{}')",
          entry_index, incoming_index, previous.coord.x, previous.coord.y,
          tileNameForDump(&previous),
          previous.cb_type ? previous.cb_type->name : std::string{},
          incoming.dst,
          nodeNameForDump(previous, fpga::CB_NODE_DST, incoming.dst),
          incoming.src,
          nodeNameForDump(previous, fpga::CB_NODE_SRC, incoming.src),
          incoming.joint,
          incoming.joint >= 0
              ? nodeNameForDump(previous, fpga::CB_NODE_JOINT, incoming.joint)
              : std::string{"direct"},
          incoming.joint2,
          incoming.joint2 >= 0
              ? nodeNameForDump(previous, fpga::CB_NODE_JOINT, incoming.joint2)
              : std::string{},
          incoming.lease_ok, previous_dst_leased, previous_src_leased,
          previous_joint_leased, previous_joint2_leased,
          (previous.cb.src_deadend.jump & (NodeMask{0, 1} << incoming.src)) !=
              NodeMask{},
          nodeOwnerForDump(&previous, fpga::CB_NODE_DST, incoming.dst, false),
          nodeOwnerForDump(&previous, fpga::CB_NODE_DST, incoming.dst, true),
          nodeOwnerForDump(&previous, fpga::CB_NODE_SRC, incoming.src, false),
          nodeOwnerForDump(&previous, fpga::CB_NODE_SRC, incoming.src, true),
          nodeOwnerForDump(&previous, fpga::CB_NODE_JOINT, incoming.joint,
                           false),
          nodeOwnerForDump(&previous, fpga::CB_NODE_JOINT, incoming.joint,
                           true),
          nodeOwnerForDump(&previous, fpga::CB_NODE_JOINT, incoming.joint2,
                           false),
          nodeOwnerForDump(&previous, fpga::CB_NODE_JOINT, incoming.joint2,
                           true));
    }
  }
  PNR_LOG1("ROUT",
           "docking backward summary: net='{}', free_reachable={}, "
           "free_unreachable={}, blocked_reachable={}, transit_preemptible={}, "
           "entries={}",
           task.net_name, free_reachable, free_unreachable, blocked_reachable,
           preemptible, entries.size());
}

// Format a mask-only docking attempt as the exact DST/JOINT/SRC node sequence.
std::string dockingAttemptPath(const DockingBackwardAttempt &attempt) {
  std::string path;
  std::string last_node;
  auto append = [&](Tile *tile, fpga::CBNodeNameType type, int node,
                    const Coord &coord) {
    const char *kind = type == fpga::CB_NODE_LOCAL ? "LOCAL"
                       : type == fpga::CB_NODE_DST ? "DST"
                       : type == fpga::CB_NODE_SRC ? "SRC"
                                                   : "JOINT";
    std::string name =
        tile ? nodeNameForDump(*tile, type, node) : std::string{};
    if (name.empty()) {
      name = "NODE";
    }
    std::string current =
        std::format("{}_{}({},{}).{}", name, kind, coord.x, coord.y, node);
    if (current == last_node) {
      return;
    }
    if (!path.empty()) {
      path += "->";
    }
    path += current;
    last_node = std::move(current);
  };
  for (const Wire &fragment : attempt.fragments) {
    Tile *source =
        fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
    Tile *target =
        fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
    if (fragment.type == Wire::WIRE_TILE_PIN) {
      append(source, fpga::CB_NODE_LOCAL, fragment.local, fragment.from);
      continue;
    }
    append(source, fpga::CB_NODE_DST, fragment.local, fragment.from);
    if (fragment.joint2 >= 0) {
      append(source, fpga::CB_NODE_JOINT, fragment.joint2, fragment.from);
    }
    if (fragment.joint >= 0) {
      append(source, fpga::CB_NODE_JOINT, fragment.joint, fragment.from);
    }
    if (fragment.jump >= 0) {
      append(source, fpga::CB_NODE_SRC, fragment.jump, fragment.from);
      append(target, fpga::CB_NODE_DST, fragment.dst, fragment.to);
    }
  }
  return path;
}

// Emit every backward branch retained by a focused docking probe.
void logDockingAttemptPaths(const std::string &net_name, Tile &target,
                            const DockingResult &result, const char *context) {
  for (size_t index = 0; index < result.backward_attempts.size(); ++index) {
    const DockingBackwardAttempt &attempt = result.backward_attempts[index];
    PNR_LOG1("ROUT",
             "routeDesign {} backward attempt[{}]: net='{}', target=({},{}), "
             "seed_dst={} '{}', result={}, fragments={}, path={}",
             context, index + 1, net_name, target.coord.x, target.coord.y,
             attempt.target_dst,
             nodeNameForDump(target, fpga::CB_NODE_DST, attempt.target_dst),
             attempt.result, attempt.fragments.size(),
             dockingAttemptPath(attempt));
  }
}

bool tryBestFirstRoute(
    Tile &from, Tile &to, int from_pos, rtl::Inst &dst_inst,
    const std::string &to_port, std::vector<Wire> &wire, int iteration_limit,
    bool start_from_dst = false,
    const std::string &start_dst_wire = std::string{}, bool *complete = nullptr,
    RouteDesign::RouteStats *stats = nullptr, RouteDesign *router = nullptr,
    rtl::Net *current_net = nullptr, bool branch_from_existing = false,
    rtl::Inst *current_source = nullptr,
    const std::string &current_source_port = std::string{},
    const std::string &current_route_name = std::string{},
    NodeMask dst_pin_nodes = {}, bool dst_pin_nodes_valid = false,
    bool debug_this_attempt = false, RouteSearchReport *report = nullptr,
    bool allow_transit_preempt = false,
    bool allow_docking_terminal_preempt = false) {
  if (stats) {
    stats->exhausted_coord = {-1, -1};
    stats->exhausted_node = -1;
  }
  if (RouteCongestionTrace::current)
    RouteCongestionTrace::current->search(from, to, from_pos, iteration_limit);
  auto profile_setup_start = std::chrono::steady_clock::now();
  const bool ignore_deadends =
      router &&
      pnr::routingIgnoresDeadends(router->route_deadends_enabled,
                                  router->fanout_stage, router->moving_stage);
  const bool transit_preemption_enabled = pnr::transitPreemptionEnabled(
      allow_transit_preempt && (!router || router->route_preemption_enabled),
      router && router->fanout_stage,
      !router || router->fanout_preemption_enabled);
  const bool explore_alternate_docking_candidates =
      router && router->moving_stage;
  std::string debug_net =
      !current_route_name.empty()
          ? current_route_name
          : (current_net ? current_net->makeName(FULL_NAME_LIMIT)
                         : std::string{});
  bool debug_route = debug_this_attempt;
  if (!debug_route && routeDebugEnabled("SCALEPNR_DEBUG_ROUTE_NET")) {
    debug_route =
        routeDebugMatches("SCALEPNR_DEBUG_ROUTE_NET", debug_net) ||
        (current_source &&
         routeDebugMatches("SCALEPNR_DEBUG_ROUTE_NET",
                           current_source->makeName(FULL_NAME_LIMIT))) ||
        routeDebugMatches("SCALEPNR_DEBUG_ROUTE_NET",
                          dst_inst.makeName(FULL_NAME_LIMIT));
  }
  bool profile_route = envFlagEnabled("SCALEPNR_ROUTE_PROFILE");
  int debug_route_lines = 0;
  int debug_route_line_limit = routeDebugLineLimit();
  auto debug_log_text = [&](const std::string &message) {
    ++debug_route_lines;
    PNR_LOG1("ROUT", "routeDesign route debug [{}]: {}", debug_route_lines,
             message);
  };
#define ROUTE_DEBUG_TEXT(message)                                              \
  do {                                                                         \
    if (debug_route && debug_route_lines < debug_route_line_limit) {           \
      debug_log_text(message);                                                 \
    }                                                                          \
  } while (false)
#define ROUTE_DEBUG_LOG(fmt, ...)                                              \
  do {                                                                         \
    if (debug_route && debug_route_lines < debug_route_line_limit) {           \
      debug_log_text(std::format(fmt, __VA_ARGS__));                           \
    }                                                                          \
  } while (false)

  struct Step {
    Coord coord;
    int local = -1;
    int prev = -1;
    int jump = -1;
    int route_jump = -1;
    int joint = -1;
    int joint2 = -1;
    int depth = 0;
    std::string src_wire;
    std::string dst_wire;
  };
  NodeMask pin_nodes =
      dst_pin_nodes_valid
          ? dst_pin_nodes
          : to.getPinNodes(dst_inst.cell_ref->type, to_port, dst_inst.pos);
  const char *forced_pin_text = std::getenv("SCALEPNR_DEBUG_DOCK_PIN");
  if (debug_route && forced_pin_text && forced_pin_text[0] != '\0') {
    int forced_pin = std::atoi(forced_pin_text);
    if (forced_pin >= 0 && forced_pin < CB_MAX_NODES) {
      pin_nodes = NodeMask{0, 1} << forced_pin;
    }
  }
  if (pin_nodes == NodeMask{}) {
    ROUTE_DEBUG_LOG("no destination pin nodes: net='{}', dst='{}'/'{}', "
                    "to_tile=({},{}), from_tile=({},{}), start_local={}",
                    debug_net, dst_inst.makeName(FULL_NAME_LIMIT), to_port,
                    to.coord.x, to.coord.y, from.coord.x, from.coord.y,
                    from_pos);
    return false;
  }
  const NodeMask packed_reserved_joints =
      fpga::packedInputJointReservations(to, &dst_inst, to_port);
  if (!targetPinsHaveUnreservedEntryPath(to, pin_nodes,
                                         packed_reserved_joints)) {
    ROUTE_DEBUG_LOG(
        "target has no unreserved physical terminal path: target=({},{}), "
        "pin_nodes={}, reserved_joints={}, incoming_dsts={}",
        to.coord.x, to.coord.y, maskString(pin_nodes),
        maskString(packed_reserved_joints), maskString(to.incoming_dst_nodes));
    return false;
  }
  if (stats) {
    ++stats->route_searches;
  }
  const bool reuses_start_dst = pnr::routeStartReusesDestination(
      from.cb, branch_from_existing, start_from_dst, from_pos);

  constexpr size_t max_steps = 96;
  std::vector<Step> steps;
  steps.reserve(max_steps + 1);
  steps.push_back(Step{from.coord,
                       from_pos,
                       -1,
                       -1,
                       -1,
                       -1,
                       -1,
                       start_from_dst ? 1 : 0,
                       {},
                       start_dst_wire});
  const int partial_route_depth =
      router ? std::max(1, router->route_suffix_depth_limit) : 5;

  ROUTE_DEBUG_LOG(
      "start: net='{}', from_tile='{}' coord=({},{}), start_local={} '{}', "
      "to_tile='{}' coord=({},{}), dst='{}'/'{}', pin_nodes={}, "
      "iteration_limit={}, max_depth={}, start_from_dst={}, "
      "start_dst_wire='{}', branch_from_existing={}",
      debug_net, from.makeName(), from.coord.x, from.coord.y, from_pos,
      nodeDebugName(from,
                    start_from_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL,
                    from_pos),
      to.makeName(), to.coord.x, to.coord.y, dst_inst.makeName(FULL_NAME_LIMIT),
      to_port, maskString(pin_nodes), iteration_limit, partial_route_depth,
      start_from_dst, start_dst_wire, branch_from_existing);

  bool trace_backtrack = envFlagEnabled("SCALEPNR_TRACE_BACKTRACK");
  bool backtrack_trace_printed = false;
  int final_step = -1;
  int exhausted_step = -1;
  // Keep the downstream failure when its parents subsequently exhaust exits.
  // This is diagnostic only; an alternate successful suffix discards it.
  auto remember_exhausted_step = [&](int index) {
    if (exhausted_step < 0 || steps[index].depth > steps[exhausted_step].depth) {
      exhausted_step = index;
    }
  };
  int final_pin = -1;
  int final_joint = -1;
  int final_joint2 = -1;
  bool final_allows_existing_dst = false;
  TerminalEntryKind final_entry_kind = TerminalEntryKind::none;
  DockingResult docked_route;
  bool completed_by_docking = false;
  std::vector<int> docking_candidate_steps;
  std::unordered_map<int, DockingResult> failed_docking_by_anchor;
  int partial_fallback_step = -1;
  auto remember_docking_candidate = [&](int step_index) {
    if (std::find(docking_candidate_steps.begin(),
                  docking_candidate_steps.end(),
                  step_index) == docking_candidate_steps.end()) {
      docking_candidate_steps.push_back(step_index);
    }
  };
  bool target_enter_failed = false;
  int start_docking_radius = isIOBEndpoint(dst_inst) ? 12 : 5;
  int max_depth = pnr::suffixDepthBeforeDocking(
      pnr::dockingWindowDistance(from.coord, to.coord), start_docking_radius,
      partial_route_depth);
  Coord target_coord = to.coord;
  std::unordered_map<uint64_t, NodeMask> search_src_deadends;
  auto search_src_deadend_mask = [&](const Coord &coord) -> NodeMask {
    auto it = search_src_deadends.find(tileDeadendKey(coord));
    return it == search_src_deadends.end() ? NodeMask{} : it->second;
  };
  auto persistent_src_deadend_mask = [&](const Coord &coord) -> NodeMask {
    if (!router || ignore_deadends) {
      return {};
    }
    NodeMask mask{};
    auto coord_it = router->route_src_deadends.find(tileDeadendKey(coord));
    if (coord_it != router->route_src_deadends.end()) {
      mask |= coord_it->second;
    }
    return mask;
  };
  auto record_failed_incoming_src =
      [&](int dead_index, const Step &dead_step, const char *reason,
          bool persistent_deadend = true) -> bool {
    (void)dead_index;
    pnr::FailedEdgePolicy policy =
        pnr::failedEdgePolicy(ignore_deadends, persistent_deadend,
                              dead_step.prev >= 0, dead_step.jump >= 0);
    if (!policy.retry_parent) {
      return false;
    }
    const Step &prev_step = steps[dead_step.prev];
    Tile *prev_tile =
        fpga::Device::current().getTile(prev_step.coord.x, prev_step.coord.y);
    if (!prev_tile) {
      return false;
    }
    search_src_deadends[tileDeadendKey(prev_tile->coord)].setBit(dead_step.jump);
    if (router && policy.persist) {
      router->route_src_deadends[tileDeadendKey(prev_tile->coord)].setBit(
          dead_step.jump);
      prev_tile->cb.src_deadend.jump.setBit(dead_step.jump);
    }
    if (stats) {
      ++stats->src_deadend_marks;
      ++stats->deadends_by_depth[statDepthBucket(prev_step.depth)];
      stats->has_last_deadend_mark = true;
      stats->last_src_deadend_coord = prev_tile->coord;
      stats->last_src_deadend_node = dead_step.jump;
      stats->last_deadend_net = debug_net;
    }
    Tile *dead_tile =
        fpga::Device::current().getTile(dead_step.coord.x, dead_step.coord.y);
    ROUTE_DEBUG_LOG(
        "mark src deadend: reason={}, prev=({},{}), prev_node={} '{}', src={} "
        "'{}', dead=({},{}), dead_node={} '{}'",
        reason, prev_step.coord.x, prev_step.coord.y, prev_step.local,
        nodeDebugName(*prev_tile,
                      prev_step.depth == 0 ? fpga::CB_NODE_LOCAL
                                           : fpga::CB_NODE_DST,
                      prev_step.local),
        dead_step.jump,
        nodeDebugName(*prev_tile, fpga::CB_NODE_SRC, dead_step.jump),
        dead_step.coord.x, dead_step.coord.y, dead_step.local,
        dead_tile
            ? nodeDebugName(*dead_tile, fpga::CB_NODE_DST, dead_step.local)
            : std::string{});
    return true;
  };
  auto temp_state_for_step = [&](int step_index, Tile &query_tile) {
    // Reconstruct at most five temporary suffix leases from parent links.
    // Keeping full 4096-bit CBState copies on every candidate was quadratic
    // in suffix depth and dominated routing of large designs.
    CBState state = query_tile.cb;
    for (int child_index = step_index;
         child_index >= 0 && steps[child_index].prev >= 0;
         child_index = steps[child_index].prev) {
      const Step &child = steps[child_index];
      const Step &parent = steps[child.prev];
      if (!sameCoord(parent.coord, query_tile.coord)) {
        continue;
      }
      if (child.jump >= 0) {
        state.src.jump.setBit(child.jump);
      }
      if (parent.depth > 0 && parent.local >= 0) {
        state.dst.jump.setBit(parent.local);
      }
      if (child.joint >= 0) {
        state.joint.jump.setBit(child.joint);
      }
      if (child.joint2 >= 0) {
        state.joint.jump.setBit(child.joint2);
      }
    }
    return state;
  };
  auto selected_path_debug_string = [&](int step_index) {
    std::vector<int> path;
    for (int idx = step_index; idx >= 0; idx = steps[idx].prev) {
      path.push_back(idx);
    }
    std::reverse(path.begin(), path.end());
    std::string result;
    for (size_t i = 0; i < path.size(); ++i) {
      const Step &path_step = steps[path[i]];
      Tile *path_tile =
          fpga::Device::current().getTile(path_step.coord.x, path_step.coord.y);
      if (!result.empty()) {
        result += " -> ";
      }
      result +=
          std::format("({},{})/{} '{}'", path_step.coord.x, path_step.coord.y,
                      path_step.local,
                      path_tile ? nodeDebugName(*path_tile,
                                                path_step.depth == 0 && i == 0
                                                    ? fpga::CB_NODE_LOCAL
                                                    : fpga::CB_NODE_DST,
                                                path_step.local)
                                : std::string{});
      if (i > 0) {
        const Step &prev_step = steps[path[i - 1]];
        Tile *prev_tile = fpga::Device::current().getTile(prev_step.coord.x,
                                                          prev_step.coord.y);
        result +=
            std::format(" via src={} '{}'", path_step.jump,
                        prev_tile ? nodeDebugName(*prev_tile, fpga::CB_NODE_SRC,
                                                  path_step.jump)
                                  : std::string{});
      }
    }
    return result;
  };
  const bool trace_move_paths =
      router && router->moving_stage &&
      routeDebugEnabled("SCALEPNR_DEBUG_MOVE_PATH_NET") &&
      (routeDebugMatches("SCALEPNR_DEBUG_MOVE_PATH_NET", debug_net) ||
       (current_source &&
        routeDebugMatches("SCALEPNR_DEBUG_MOVE_PATH_NET",
                          current_source->makeName(FULL_NAME_LIMIT))) ||
       routeDebugMatches("SCALEPNR_DEBUG_MOVE_PATH_NET",
                         dst_inst.makeName(FULL_NAME_LIMIT)));
  const bool trace_docking_paths = trace_move_paths || debug_route;
  int move_path_trace_lines = 0;
  const int move_path_trace_limit = movePathDebugLineLimit();
  std::unordered_map<int, int> move_target_dst_attempts;
  auto path_node_text = [](Tile *tile, fpga::CBNodeNameType type, int node,
                           const Coord &coord) {
    std::string name = tile ? nodeDebugName(*tile, type, node) : std::string{};
    if (name.empty()) {
      name = "NODE";
    }
    const char *kind = type == fpga::CB_NODE_LOCAL ? "LOCAL"
                       : type == fpga::CB_NODE_DST ? "DST"
                       : type == fpga::CB_NODE_SRC ? "SRC"
                                                   : "JOINT";
    return std::format("{}_{}({},{}).{}", name, kind, coord.x, coord.y, node);
  };
  auto move_path_prefix = [&](int step_index) {
    std::vector<int> path;
    for (int walk = step_index; walk >= 0; walk = steps[walk].prev) {
      path.push_back(walk);
    }
    std::reverse(path.begin(), path.end());
    std::string result;
    auto append = [&](const std::string &node) {
      if (!result.empty()) {
        result += "->";
      }
      result += node;
    };
    if (path.empty()) {
      return result;
    }
    const Step &root = steps[path.front()];
    Tile *root_tile =
        fpga::Device::current().getTile(root.coord.x, root.coord.y);
    append(path_node_text(
        root_tile, start_from_dst ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL,
        root.local, root.coord));
    for (size_t path_pos = 1; path_pos < path.size(); ++path_pos) {
      const Step &child = steps[path[path_pos]];
      const Step &parent = steps[path[path_pos - 1]];
      Tile *parent_tile =
          fpga::Device::current().getTile(parent.coord.x, parent.coord.y);
      Tile *child_tile =
          fpga::Device::current().getTile(child.coord.x, child.coord.y);
      if (child.joint2 >= 0) {
        append(path_node_text(parent_tile, fpga::CB_NODE_JOINT, child.joint2,
                              parent.coord));
      }
      if (child.joint >= 0) {
        append(path_node_text(parent_tile, fpga::CB_NODE_JOINT, child.joint,
                              parent.coord));
      }
      append(path_node_text(parent_tile, fpga::CB_NODE_SRC, child.jump,
                            parent.coord));
      append(path_node_text(child_tile, fpga::CB_NODE_DST, child.local,
                            child.coord));
    }
    return result;
  };
  auto trace_move_path = [&](const char *result, int step_index,
                             Tile &source_tile, int src_node, int joint,
                             int joint2, const fpga::TileJumpTarget *target,
                             int terminal_pin = -1,
                             const TerminalEntryCandidate *terminal = nullptr) {
    if (!trace_move_paths || move_path_trace_lines >= move_path_trace_limit) {
      return;
    }
    std::string path = move_path_prefix(step_index);
    auto append = [&](const std::string &node) {
      if (!path.empty()) {
        path += "->";
      }
      path += node;
    };
    if (joint2 >= 0) {
      append(path_node_text(&source_tile, fpga::CB_NODE_JOINT, joint2,
                            source_tile.coord));
    }
    if (joint >= 0) {
      append(path_node_text(&source_tile, fpga::CB_NODE_JOINT, joint,
                            source_tile.coord));
    }
    append(path_node_text(&source_tile, fpga::CB_NODE_SRC, src_node,
                          source_tile.coord));
    bool target_dst_free = false;
    const Step *source_step =
        step_index >= 0 && static_cast<size_t>(step_index) < steps.size()
            ? &steps[step_index]
            : nullptr;
    const fpga::CBNodeNameType source_node_type =
        source_step && source_step->depth == 0 && !start_from_dst
            ? fpga::CB_NODE_LOCAL
            : fpga::CB_NODE_DST;
    const int source_node = source_step ? source_step->local : -1;
    const bool source_node_leased =
        source_node >= 0 &&
        (source_node_type == fpga::CB_NODE_LOCAL
             ? source_tile.isPinNodeLeased(source_node)
             : (source_tile.cb.dst.jump & (NodeMask{0, 1} << source_node)) !=
                   NodeMask{});
    const bool src_leased =
        src_node >= 0 &&
        (source_tile.cb.src.jump & (NodeMask{0, 1} << src_node)) != NodeMask{};
    const bool joint_leased =
        joint >= 0 &&
        (source_tile.cb.joint.jump & (NodeMask{0, 1} << joint)) != NodeMask{};
    const bool joint2_leased =
        joint2 >= 0 &&
        (source_tile.cb.joint.jump & (NodeMask{0, 1} << joint2)) != NodeMask{};
    std::string target_dst_owner;
    std::string terminal_joint_owner;
    std::string terminal_joint2_owner;
    std::string terminal_pin_owner;
    if (target && target->tile && target->dst_node >= 0) {
      if (sameCoord(target->tile->coord, target_coord)) {
        ++move_target_dst_attempts[target->dst_node];
      }
      append(path_node_text(target->tile, fpga::CB_NODE_DST, target->dst_node,
                            target->tile->coord));
      target_dst_free = (target->tile->cb.dst.jump &
                         (NodeMask{0, 1} << target->dst_node)) == NodeMask{};
      target_dst_owner = nodeOwnerForDump(target->tile, fpga::CB_NODE_DST,
                                          target->dst_node, false);
      if (terminal) {
        terminal_joint_owner = nodeOwnerForDump(
            target->tile, fpga::CB_NODE_JOINT, terminal->joint, false);
        terminal_joint2_owner = nodeOwnerForDump(
            target->tile, fpga::CB_NODE_JOINT, terminal->joint2, false);
        terminal_pin_owner = nodeOwnerForDump(target->tile, fpga::CB_NODE_LOCAL,
                                              terminal_pin, false);
        if (terminal->joint2 >= 0) {
          append(path_node_text(target->tile, fpga::CB_NODE_JOINT,
                                terminal->joint2, target->tile->coord));
        }
        if (terminal->joint >= 0) {
          append(path_node_text(target->tile, fpga::CB_NODE_JOINT,
                                terminal->joint, target->tile->coord));
        }
        if (terminal_pin >= 0) {
          append(path_node_text(target->tile, fpga::CB_NODE_LOCAL, terminal_pin,
                                target->tile->coord));
        }
      }
    }
    ++move_path_trace_lines;
    PNR_LOG1(
        "ROUT",
        "routeDesign move path attempt[{}]: net='{}', result={}, "
        "target=({},{}), target_dst={}, target_dst_free={}, terminal_pin={}, "
        "terminal_joint={}, terminal_joint2={}, "
        "source_state(dst={},src={},joint={},local={}), "
        "owners(from={}:'{}',src={}:'{}',joint={}:'{}',joint2={}:'{}',target_"
        "dst='{}',terminal_joint='{}',terminal_joint2='{}',terminal_pin='{}'), "
        "path={}",
        move_path_trace_lines, debug_net, result,
        target && target->tile ? target->tile->coord.x : -1,
        target && target->tile ? target->tile->coord.y : -1,
        target ? target->dst_node : -1, target_dst_free, terminal_pin,
        terminal ? terminal->joint : -1, terminal ? terminal->joint2 : -1,
        maskBitsForDump(source_tile.cb.dst.jump, CB_MAX_NODES),
        maskBitsForDump(source_tile.cb.src.jump, CB_MAX_NODES),
        maskBitsForDump(source_tile.cb.joint.jump, CB_MAX_NODES),
        maskBitsForDump(source_tile.cb.local.local, CB_MAX_NODES),
        source_node_leased,
        nodeOwnerForDump(&source_tile, source_node_type, source_node, false),
        src_leased,
        nodeOwnerForDump(&source_tile, fpga::CB_NODE_SRC, src_node, false),
        joint_leased,
        nodeOwnerForDump(&source_tile, fpga::CB_NODE_JOINT, joint, false),
        joint2_leased,
        nodeOwnerForDump(&source_tile, fpga::CB_NODE_JOINT, joint2, false),
        target_dst_owner, terminal_joint_owner, terminal_joint2_owner,
        terminal_pin_owner, path);
  };
  auto trace_move_target_entries = [&](int step_index, Tile &source_tile,
                                       int src_node, int joint, int joint2,
                                       const fpga::TileJumpTarget &target) {
    if (!trace_move_paths || !target.tile || target.dst_node < 0 ||
        !sameCoord(target.tile->coord, target_coord)) {
      return;
    }
    bool found_entry = false;
    pin_nodes.for_each_set_bit([&](int pin) {
      for (const TerminalEntryCandidate &entry :
           targetEntryCandidates(*target.tile->cb_type, target.dst_node, pin)) {
        found_entry = true;
        CBState terminal_test = target.tile->cb;
        bool lease_ok =
            !target.tile->isPinNodeLeased(pin) &&
            leaseConcreteTerminal(terminal_test, target.dst_node, pin, entry);
        trace_move_path(lease_ok ? "target_entry_free" : "target_entry_blocked",
                        step_index, source_tile, src_node, joint, joint2,
                        &target, pin, &entry);
      }
      return false;
    });
    if (!found_entry) {
      trace_move_path("target_dst_has_no_pin_path", step_index, source_tile,
                      src_node, joint, joint2, &target);
    }
  };
  if (trace_move_paths) {
    PNR_LOG1("ROUT",
             "routeDesign move path search: net='{}', from=({},{})/{}, "
             "target=({},{}) pin_nodes={}, start_from_dst={}",
             debug_net, from.coord.x, from.coord.y, from_pos, target_coord.x,
             target_coord.y, maskBitsForDump(pin_nodes, CB_MAX_NODES),
             start_from_dst);
  }
  auto trace_move_target_summary = [&]() {
    if (!trace_move_paths || !to.cb_type) {
      return;
    }
    std::unordered_set<int> reported;
    pin_nodes.for_each_set_bit([&](int pin) {
      to.cb_type->ensureDerivedMasks();
      NodeMask target_dsts = to.cb_type->dsts_reaching_local[pin].jump;
      target_dsts.for_each_set_bit([&](int dst) {
        if (!reported.insert(dst).second) {
          return false;
        }
        NodeMask dst_bit = NodeMask{0, 1} << dst;
        bool dst_free = (to.cb.dst.jump & dst_bit) == NodeMask{};
        if (!dst_free) {
          return false;
        }
        bool terminal_free = false;
        std::string terminal_paths;
        for (const TerminalEntryCandidate &entry :
             targetEntryCandidates(*to.cb_type, dst, pin)) {
          CBState terminal_test = to.cb;
          bool entry_free =
              !to.isPinNodeLeased(pin) &&
              leaseConcreteTerminal(terminal_test, dst, pin, entry);
          terminal_free = terminal_free || entry_free;
          if (!terminal_paths.empty()) {
            terminal_paths += ';';
          }
          terminal_paths += std::format("joint={},joint2={},free={}",
                                        entry.joint, entry.joint2, entry_free);
        }
        PNR_LOG1(
            "ROUT",
            "routeDesign move path target summary: net='{}', target=({},{}), "
            "dst={} '{}', dst_free=true, terminal_free={}, attempts={}, pin={} "
            "'{}', terminal_paths=[{}]",
            debug_net, to.coord.x, to.coord.y, dst,
            nodeDebugName(to, fpga::CB_NODE_DST, dst), terminal_free,
            move_target_dst_attempts[dst], pin,
            nodeDebugName(to, fpga::CB_NODE_LOCAL, pin), terminal_paths);
        return false;
      });
      return false;
    });
  };
  auto trace_move_docking_path = [&](int anchor_step,
                                     const DockingResult &result) {
    if (!trace_move_paths || !result.success) {
      return;
    }
    std::string path = move_path_prefix(anchor_step);
    auto append = [&](const std::string &node) {
      if (!path.empty()) {
        path += "->";
      }
      path += node;
    };
    for (const Wire &fragment : result.fragments) {
      Tile *source_tile =
          fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
      Tile *target_tile =
          fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
      if (fragment.type == Wire::WIRE_TILE_PIN) {
        append(path_node_text(source_tile, fpga::CB_NODE_LOCAL, fragment.local,
                              fragment.from));
        continue;
      }
      if (fragment.jump < 0) {
        if (fragment.joint2 >= 0) {
          append(path_node_text(source_tile, fpga::CB_NODE_JOINT,
                                fragment.joint2, fragment.from));
        }
        if (fragment.joint >= 0) {
          append(path_node_text(source_tile, fpga::CB_NODE_JOINT,
                                fragment.joint, fragment.from));
        }
        continue;
      }
      if (fragment.joint2 >= 0) {
        append(path_node_text(source_tile, fpga::CB_NODE_JOINT, fragment.joint2,
                              fragment.from));
      }
      if (fragment.joint >= 0) {
        append(path_node_text(source_tile, fpga::CB_NODE_JOINT, fragment.joint,
                              fragment.from));
      }
      append(path_node_text(source_tile, fpga::CB_NODE_SRC, fragment.jump,
                            fragment.from));
      append(path_node_text(target_tile, fpga::CB_NODE_DST, fragment.dst,
                            fragment.to));
    }
    PNR_LOG1("ROUT",
             "routeDesign move path docking success: net='{}', "
             "anchor=({},{})/{}, target=({},{}), fragments={}, path={}",
             debug_net, steps[anchor_step].coord.x, steps[anchor_step].coord.y,
             steps[anchor_step].local, target_coord.x, target_coord.y,
             result.fragments.size(), path);
  };
  // Print every retained backward docking alternative as concrete node steps.
  auto trace_docking_backward_attempts = [&](const DockingResult &result) {
    if (!trace_docking_paths) {
      return;
    }
    for (const DockingBackwardAttempt &attempt : result.backward_attempts) {
      if (move_path_trace_lines >= move_path_trace_limit) {
        return;
      }
      std::string path;
      auto append = [&](const std::string &node) {
        if (!path.empty()) {
          path += "->";
        }
        path += node;
      };
      for (const Wire &fragment : attempt.fragments) {
        Tile *source_tile =
            fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
        Tile *target_tile =
            fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
        if (fragment.type == Wire::WIRE_TILE_PIN) {
          append(path_node_text(source_tile, fpga::CB_NODE_LOCAL,
                                fragment.local, fragment.from));
          continue;
        }
        append(path_node_text(source_tile, fpga::CB_NODE_DST, fragment.local,
                              fragment.from));
        if (fragment.joint2 >= 0) {
          append(path_node_text(source_tile, fpga::CB_NODE_JOINT,
                                fragment.joint2, fragment.from));
        }
        if (fragment.joint >= 0) {
          append(path_node_text(source_tile, fpga::CB_NODE_JOINT,
                                fragment.joint, fragment.from));
        }
        if (fragment.jump >= 0) {
          append(path_node_text(source_tile, fpga::CB_NODE_SRC, fragment.jump,
                                fragment.from));
          append(path_node_text(target_tile, fpga::CB_NODE_DST, fragment.dst,
                                fragment.to));
        }
      }
      ++move_path_trace_lines;
      PNR_LOG1(
          "ROUT",
          "routeDesign docking backward attempt[{}]: net='{}', target=({},{}), "
          "seed_dst={} '{}', result={}, fragments={}, path={}",
          move_path_trace_lines, debug_net, to.coord.x, to.coord.y,
          attempt.target_dst,
          nodeDebugName(to, fpga::CB_NODE_DST, attempt.target_dst),
          attempt.result, attempt.fragments.size(), path);
    }
  };
  auto partial_endpoint_can_continue = [&](int endpoint_step) {
    if (endpoint_step <= 0 || endpoint_step >= static_cast<int>(steps.size())) {
      return false;
    }
    const Step &endpoint = steps[endpoint_step];
    Tile *endpoint_tile =
        fpga::Device::current().getTile(endpoint.coord.x, endpoint.coord.y);
    if (!endpoint_tile || !endpoint_tile->cb_type) {
      return false;
    }
    const std::vector<uint16_t> *src_nodes =
        endpoint_tile->cb_type->srcNodes(fpga::CB_NODE_DST, endpoint.local);
    if (!src_nodes) {
      return false;
    }
    for (uint16_t src_node : *src_nodes) {
      NodeMask combined_endpoint_deadend =
          persistent_src_deadend_mask(endpoint.coord) |
          search_src_deadend_mask(endpoint.coord);
      if (combined_endpoint_deadend.testBit(src_node)) {
        fpga::TileJumpTarget deadend_target =
            fpga::Device::current().resolveJump(*endpoint_tile, src_node);
        if (!deadend_target.tile ||
            !sameCoord(deadend_target.tile->coord, target_coord)) {
          continue;
        }
      }
      if (endpoint_tile->cb.src.jump.testBit(src_node) ||
          endpoint_tile->cb_type->dst_by_src[src_node].empty()) {
        continue;
      }
      int joint = -1;
      int joint2 = -1;
      if (!endpoint_tile->cb_type->canJump(endpoint.local, src_node, src_node,
                                           joint, &joint2)) {
        continue;
      }
      CBState test_cb = temp_state_for_step(endpoint_step, *endpoint_tile);
      if (!leaseConcreteJump(test_cb, endpoint.local, src_node, joint,
                             ignore_deadends, joint2)) {
        continue;
      }
      fpga::TileJumpTarget target = fpga::Device::current().resolveJumpToward(
          *endpoint_tile, src_node, target_coord);
      if (!target.tile || target.dst_node < 0) {
        continue;
      }
      if (sameCoord(target.tile->coord, target_coord) &&
          targetDstCanEnterPin(*target.tile, target.dst_node, pin_nodes)) {
        return true;
      }
      return true;
    }
    return false;
  };
  auto search_loop_start = std::chrono::steady_clock::now();
  if (stats && profile_route) {
    // edge_order_ns is retained as the existing ABI field, but now measures
    // all setup before the bounded routing loop; candidate ordering itself is preloaded.
    stats->edge_order_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            search_loop_start - profile_setup_start)
            .count());
  }
  std::vector<int> frontier;
  frontier.reserve(max_steps + 1);
  frontier.push_back(0);
  std::vector<uint16_t> active_src_nodes;
  std::vector<uint16_t> target_hop_src_nodes;
  while (!frontier.empty() && steps.size() <= max_steps) {
    int idx = frontier.back();
    frontier.pop_back();
    const Step &step = steps[idx];
    Tile *tile = fpga::Device::current().getTile(step.coord.x, step.coord.y);
    if (!tile || !isConcreteRouteTile(*tile) || step.depth > max_depth) {
      ROUTE_DEBUG_LOG("skip pop: coord=({},{}), depth={}, local={}, "
                      "tile_ok={}, concrete={}, max_depth={}",
                      step.coord.x, step.coord.y, step.depth, step.local,
                      tile != nullptr,
                      tile ? isConcreteRouteTile(*tile) : false, max_depth);
      continue;
    }
    bool report_start_step = report && idx == 0;
    if (report_start_step) {
      report->start_expanded = true;
    }
    if (stats) {
      ++stats->search_pops;
      ++stats->pops_by_depth[statDepthBucket(step.depth)];
      if (tile->cb.src_deadend.jump != NodeMask{}) {
        ++stats->pops_on_deadend_tile;
      }
      if (tile->cb.src_deadend.jump != NodeMask{}) {
        ++stats->pops_on_src_deadend_tile;
      }
    }
    bool continuing_from_existing_dst = start_from_dst && idx == 0;
    int docking_radius = isIOBEndpoint(dst_inst) ? 12 : 5;
    if (pnr::rememberDockingCandidate(
            step.depth,
            pnr::dockingWindowDistance(step.coord, target_coord),
            docking_radius)) {
      // Retain the rail before expansion because a later accepted hop in this
      // bounded suffix can lead back out of the docking window.
      remember_docking_candidate(idx);
    }
    bool target_non_enterable =
        sameCoord(step.coord, target_coord) && step.depth > 0 &&
        !targetDstHasEntryPath(*tile, step.local, pin_nodes);

    bool source_local_at_target = idx == 0 && !start_from_dst;
    if (sameCoord(step.coord, target_coord) && !source_local_at_target) {
      ROUTE_DEBUG_LOG("at target tile: coord=({},{}), depth={}, dst_node={} "
                      "'{}', target_non_enterable={}, pin_nodes={}",
                      step.coord.x, step.coord.y, step.depth, step.local,
                      nodeDebugName(*tile, fpga::CB_NODE_DST, step.local),
                      target_non_enterable, maskString(pin_nodes));
      if (target_non_enterable) {
        target_enter_failed = true;
        if (stats) {
          ++stats->edge_rejected_no_target;
        }
        if (pnr::preserveBlockedEndpointForDocking(
                false, step.depth,
                pnr::dockingWindowDistance(step.coord, target_coord),
                docking_radius)) {
          remember_docking_candidate(idx);
          ROUTE_DEBUG_LOG(
              "defer non-enterable target to docking and retry parent: "
              "coord=({},{}), depth={}, dst={} '{}', target=({},{}), "
              "radius={}",
              step.coord.x, step.coord.y, step.depth, step.local,
              nodeDebugName(*tile, fpga::CB_NODE_DST, step.local),
              target_coord.x, target_coord.y, docking_radius);
          if (record_failed_incoming_src(
                  idx, step, "target_docking_candidate", false)) {
            frontier.push_back(step.prev);
          }
          continue;
        }
        if (record_failed_incoming_src(idx, step,
                                       "target_dst_cannot_enter_pin")) {
          frontier.push_back(step.prev);
        }
        continue;
      }
      bool ok = pin_nodes.for_each_set_bit([&](int pin) {
        bool pin_leased = tile->isPinNodeLeased(pin);
        std::vector<TerminalEntryCandidate> entry_candidates =
            pin_leased ? std::vector<TerminalEntryCandidate>{}
                       : targetEntryCandidates(*tile->cb_type, step.local, pin);
        bool can_in = !pin_leased && !entry_candidates.empty();
        bool lease_ok = false;
        TerminalEntryCandidate selected_entry;
        for (const TerminalEntryCandidate &candidate_entry : entry_candidates) {
          if (!pnr::terminalEntryAvoidsReservedJoints(candidate_entry.joint,
                                                      candidate_entry.joint2,
                                                      packed_reserved_joints)) {
            continue;
          }
          CBState test_cb = temp_state_for_step(idx, *tile);
          if (leaseConcreteTerminal(test_cb, step.local, pin, candidate_entry,
                                    continuing_from_existing_dst)) {
            selected_entry = candidate_entry;
            lease_ok = true;
            break;
          }
        }
        ROUTE_DEBUG_LOG(
            "target pin check: dst_node={} '{}', pin={} '{}', pin_leased={}, "
            "can_in={}, entry_candidates={}, kind={}, joint={}, lease_ok={}, "
            "dst_mask={}, local_mask={}",
            step.local, nodeDebugName(*tile, fpga::CB_NODE_DST, step.local),
            pin, nodeDebugName(*tile, fpga::CB_NODE_LOCAL, pin), pin_leased,
            can_in, entry_candidates.size(),
            static_cast<int>(selected_entry.kind), selected_entry.joint,
            lease_ok, maskString(tile->cb.dst.jump),
            maskString(tile->cb.local.local));
        if (pin_leased || !can_in) {
          return false;
        }
        if (!lease_ok) {
          return false;
        }
        final_step = idx;
        final_pin = pin;
        final_joint = selected_entry.joint;
        final_joint2 = selected_entry.joint2;
        final_allows_existing_dst = continuing_from_existing_dst;
        final_entry_kind = selected_entry.kind;
        return true;
      });
      if (ok) {
        break;
      }
      target_enter_failed = true;
      remember_exhausted_step(idx);
      if (record_failed_incoming_src(idx, step, "target_entry_failed")) {
        frontier.push_back(step.prev);
      }
      continue;
    }
    if (step.depth >= max_depth) {
      bool close_enough_for_docking =
          pnr::dockingWindowDistance(step.coord, to.coord) <= docking_radius;
      bool endpoint_can_continue = partial_endpoint_can_continue(idx);
      if (close_enough_for_docking && explore_alternate_docking_candidates) {
        remember_docking_candidate(idx);
        if (endpoint_can_continue && partial_fallback_step < 0) {
          partial_fallback_step = idx;
        }
        ROUTE_DEBUG_LOG(
            "defer depth-limit endpoint to docking and retry parent: "
            "coord=({},{}), depth={}, node={} '{}', can_continue={}",
            step.coord.x, step.coord.y, step.depth, step.local,
            nodeDebugName(*tile, fpga::CB_NODE_DST, step.local),
            endpoint_can_continue);
        if (record_failed_incoming_src(
                idx, step, "depth_limit_docking_candidate", false)) {
          frontier.push_back(step.prev);
        }
        continue;
      }
      if (close_enough_for_docking || endpoint_can_continue) {
        final_step = idx;
        ROUTE_DEBUG_LOG(
            "depth limit accepted partial endpoint: coord=({},{}), depth={}, "
            "node={} '{}', close_for_docking={}, can_continue={}",
            step.coord.x, step.coord.y, step.depth, step.local,
            nodeDebugName(*tile, fpga::CB_NODE_DST, step.local),
            close_enough_for_docking, endpoint_can_continue);
        break;
      }
      ROUTE_DEBUG_LOG("depth limit deadend: coord=({},{}), depth={}, node={} "
                      "'{}', close_for_docking={}, can_continue={}",
                      step.coord.x, step.coord.y, step.depth, step.local,
                      nodeDebugName(*tile, fpga::CB_NODE_DST, step.local),
                      close_enough_for_docking, endpoint_can_continue);
      remember_exhausted_step(idx);
      if (record_failed_incoming_src(idx, step, "depth_limit_no_future")) {
        frontier.push_back(step.prev);
      }
      continue;
    }

    fpga::CBNodeNameType from_type =
        step.depth == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
    const std::vector<uint16_t> *src_nodes =
        tile->cb_type->srcNodes(from_type, step.local);
    if (!src_nodes) {
      remember_exhausted_step(idx);
      NodeMask joint_mask = from_type == fpga::CB_NODE_LOCAL
                                ? tile->cb_type->local_joint[step.local].joint
                                : tile->cb_type->dst_joint[step.local].joint;
      ROUTE_DEBUG_LOG("no src nodes: coord=({},{}), depth={}, from_type={}, "
                      "node={} '{}', joint_mask={}",
                      step.coord.x, step.coord.y, step.depth,
                      from_type == fpga::CB_NODE_LOCAL ? "local" : "dst",
                      step.local, nodeDebugName(*tile, from_type, step.local),
                      maskString(joint_mask));
      if (stats) {
        ++stats->no_src_nodes;
        if (step.depth == 0) {
          ++stats->no_src_nodes_depth0;
        }
        NodeMask joint_mask = from_type == fpga::CB_NODE_LOCAL
                                  ? tile->cb_type->local_joint[step.local].joint
                                  : tile->cb_type->dst_joint[step.local].joint;
        if (joint_mask != NodeMask{}) {
          ++stats->no_src_nodes_with_joint_path;
        }
        stats->has_last_no_src = true;
        stats->last_no_src_coord = tile->coord;
        stats->last_no_src_depth = step.depth;
        stats->last_no_src_local = step.local;
        stats->last_no_src_type = from_type;
        stats->last_no_src_joint_mask = joint_mask;
      }
      if (step.depth == 0) {
        brutalLocalExitFailure(*tile, nullptr, step.local, to.coord,
                               current_net, stats);
      }
      if (record_failed_incoming_src(idx, step, "no_src_nodes")) {
        frontier.push_back(step.prev);
      }
      continue;
    }
    active_src_nodes.clear();
    active_src_nodes.reserve(src_nodes->size());
    target_hop_src_nodes.clear();
    NodeMask target_hop_deadend_srcs;
    size_t deadend_filtered_count = 0;
    size_t unresolved_filtered_count = 0;
    NodeMask persistent_src_deadend = persistent_src_deadend_mask(tile->coord);
    NodeMask current_search_src_deadend = search_src_deadend_mask(tile->coord);
    // Tile state carries sticky committed-suffix deadends that may not be
    // target-keyed.
    NodeMask combined_src_deadend = pnr::effectiveSearchDeadends(
        current_search_src_deadend,
        tile->cb.src_deadend.jump | persistent_src_deadend, ignore_deadends);
    NodeMask raw_src_mask = from_type == fpga::CB_NODE_LOCAL
                                ? tile->cb_type->local_src[step.local].jump
                                : tile->cb_type->dst_src[step.local].jump;
    // Ordinary transit preemption is takeoff-only. Grounding and docking use
    // their own owner checks and must not turn nearby transit into candidates.
    bool first_committed_step =
        idx == 0 && step.depth == 0 && !start_from_dst;
    bool step_preempt_allowed = pnr::transitPreemptionStepAllowed(
        transit_preemption_enabled,
        router && router->protected_route_preemption_enabled,
        first_committed_step);
    bool inspect_leased_sources = step_preempt_allowed;
    // The live state still rejects busy sources during leasing; this ordering
    // view also enumerates them so takeoff preemption can inspect owners.
    NodeMask iteration_leased =
        inspect_leased_sources ? NodeMask{} : tile->cb.src.jump;
    NodeMask ordered_candidates =
        raw_src_mask & ~iteration_leased & ~combined_src_deadend;
    if (RouteCongestionTrace::current) {
      NodeMask excluded = raw_src_mask & (iteration_leased | combined_src_deadend);
      excluded.for_each_set_bit([&](int src) {
        RouteCongestionTrace::current->blocked(*tile,
            iteration_leased.testBit(src) ? "source_filtered_leased" : "source_filtered_deadend",
            {{from_type, step.local}, {fpga::CB_NODE_SRC, src}});
        return false;
      });
    }
    // Walk the loaded angle/length priority exactly once. Calling iterate()
    // once per candidate restarted its 4096-node scan and made wide nodes
    // quadratic in their number of exits. The node-specific cache also avoids
    // scanning unrelated SRC bits on every routing-state pop.
    for (uint16_t src_node :
         tile->cb_type->orderedSrcNodes(from_type, step.local,
                                        to.coord - step.coord)) {
      if (!ordered_candidates.testBit(src_node)) {
        continue;
      }
      if (src_node >= CB_MAX_NODES || !tile->cb_type ||
          tile->cb_type->dst_by_src[src_node].empty()) {
        ++unresolved_filtered_count;
        ROUTE_DEBUG_LOG(
            "skip unresolved src: coord=({},{}), depth={}, from_node={} '{}', "
            "src={} '{}', dst_by_src_bits={}",
            step.coord.x, step.coord.y, step.depth, step.local,
            nodeDebugName(*tile, from_type, step.local),
            static_cast<int>(src_node),
            nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node),
            tile->cb_type && src_node < CB_MAX_NODES
                ? countSetBits(tile->cb_type->dstMaskForSrc(src_node))
                : 0);
        trace_move_path("unresolved_mapping", idx, *tile, src_node, -1, -1,
                        nullptr);
        continue;
      }
      active_src_nodes.push_back(src_node);
    }
    for (uint16_t src_node : *src_nodes) {
      if (!combined_src_deadend.testBit(src_node) ||
          tile->cb.src.jump.testBit(src_node)) {
        continue;
      }
      if (!tile->cb_type || tile->cb_type->dst_by_src[src_node].empty()) {
        continue;
      }
      fpga::TileJumpTarget deadend_target =
          fpga::Device::current().resolveJumpToward(*tile, src_node,
                                                    target_coord);
      bool reaches_target = deadend_target.tile &&
                            sameCoord(deadend_target.tile->coord, target_coord);
      if (!pnr::targetHopMayBypassDeadend(
              reaches_target, current_search_src_deadend.testBit(src_node))) {
        ++deadend_filtered_count;
        continue;
      }
      target_hop_deadend_srcs.setBit(src_node);
      target_hop_src_nodes.push_back(src_node);
      ROUTE_DEBUG_LOG(
          "use src deadend for target hop: coord=({},{}), depth={}, "
          "from_node={} '{}', src={} '{}', target=({},{}), dst_node={} '{}'",
          step.coord.x, step.coord.y, step.depth, step.local,
          nodeDebugName(*tile, from_type, step.local),
          static_cast<int>(src_node),
          nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node),
          deadend_target.tile->coord.x, deadend_target.tile->coord.y,
          deadend_target.dst_node,
          nodeDebugName(*deadend_target.tile, fpga::CB_NODE_DST,
                        deadend_target.dst_node));
    }
    active_src_nodes.insert(active_src_nodes.begin(),
                            target_hop_src_nodes.begin(),
                            target_hop_src_nodes.end());
    NodeMask available_src_mask = ordered_candidates | target_hop_deadend_srcs;
    ROUTE_DEBUG_LOG(
        "expand: coord=({},{}), depth={}, from_type={}, node={} '{}', "
        "src_candidates={}, active={}, deadend_filtered={}, "
        "unresolved_filtered={}, raw_src_mask={}, leased_src_mask={}, "
        "src_deadend_mask={}, available_src_mask={}, src_owners={}",
        step.coord.x, step.coord.y, step.depth,
        from_type == fpga::CB_NODE_LOCAL ? "local" : "dst", step.local,
        nodeDebugName(*tile, from_type, step.local), src_nodes->size(),
        active_src_nodes.size(), deadend_filtered_count,
        unresolved_filtered_count, maskString(raw_src_mask),
        maskString(tile->cb.src.jump), maskString(combined_src_deadend),
        maskString(available_src_mask),
        srcCandidateOwnersForDump(tile, src_nodes, raw_src_mask));
    if (debug_route && envFlagEnabled("SCALEPNR_DEBUG_ROUTE_CANDIDATES")) {
      size_t order_index = 0;
      for (uint16_t candidate_src : active_src_nodes) {
        int candidate_joint = -1;
        bool has_mask_path =
            step.depth == 0
                ? tile->cb_type->canOut(step.local, candidate_src,
                                        candidate_src, candidate_joint)
                : tile->cb_type->canJump(step.local, candidate_src,
                                         candidate_src, candidate_joint);
        fpga::TileJumpTarget candidate_target =
            fpga::Device::current().resolveJumpToward(*tile, candidate_src,
                                                      target_coord);
        bool leased = (tile->cb.src.jump & (NodeMask{0, 1} << candidate_src)) !=
                      NodeMask{};
        bool deadend = (combined_src_deadend &
                        (NodeMask{0, 1} << candidate_src)) != NodeMask{};
        Coord resolved_delta =
            candidate_target.tile
                ? Coord{candidate_target.tile->coord.x - step.coord.x,
                        candidate_target.tile->coord.y - step.coord.y}
                : Coord{-99, -99};
        ROUTE_DEBUG_LOG(
            "candidate order: coord=({},{}), depth={}, order={}, from_node={} "
            "'{}', src={} '{}', resolved_delta=({},{}), encoded_delta=({},{}), "
            "target=({},{}), dst={} '{}', dist={} -> {}, mask_path={}, "
            "joint={}, leased={}, deadend={}",
            step.coord.x, step.coord.y, step.depth, order_index++, step.local,
            nodeDebugName(*tile, from_type, step.local),
            static_cast<int>(candidate_src),
            nodeDebugName(*tile, fpga::CB_NODE_SRC, candidate_src),
            resolved_delta.x, resolved_delta.y, debugJumpDeltaX(candidate_src),
            debugJumpDeltaY(candidate_src),
            candidate_target.tile ? candidate_target.tile->coord.x : -1,
            candidate_target.tile ? candidate_target.tile->coord.y : -1,
            candidate_target.dst_node,
            candidate_target.tile
                ? nodeDebugName(*candidate_target.tile, fpga::CB_NODE_DST,
                                candidate_target.dst_node)
                : std::string{},
            routeDistance(step.coord, target_coord),
            candidate_target.tile
                ? routeDistance(candidate_target.tile->coord, target_coord)
                : -1,
            has_mask_path, candidate_joint, leased, deadend);
      }
    }

    auto dump_backtrack_candidate_table = [&](uint16_t accepted_src,
                                              const fpga::TileJumpTarget
                                                  &accepted_target) {
      if (!trace_backtrack || backtrack_trace_printed || idx <= 0) {
        return;
      }
      const Step &prev_step = steps[step.prev];
      if (!accepted_target.tile ||
          !sameCoord(accepted_target.tile->coord, prev_step.coord)) {
        return;
      }
      backtrack_trace_printed = true;
      std::fprintf(
          stderr,
          "\nARENA_BACKTRACK net='%s' A=(%d,%d) B=(%d,%d) back_src=%u '%s' "
          "back_dst=%d '%s' target=(%d,%d)\n",
          debug_net.c_str(), prev_step.coord.x, prev_step.coord.y, step.coord.x,
          step.coord.y, static_cast<unsigned>(accepted_src),
          nodeDebugName(*tile, fpga::CB_NODE_SRC, accepted_src).c_str(),
          accepted_target.dst_node,
          nodeDebugName(*accepted_target.tile, fpga::CB_NODE_DST,
                        accepted_target.dst_node)
              .c_str(),
          target_coord.x, target_coord.y);
      std::fprintf(stderr,
                   "B_STATE src=%s dst=%s joint=%s local=%s src_deadend=%s "
                   "raw_src=%s available_src=%s\n",
                   maskString(tile->cb.src.jump).c_str(),
                   maskString(tile->cb.dst.jump).c_str(),
                   maskString(tile->cb.joint.jump).c_str(),
                   maskString(tile->cb.local.local).c_str(),
                   maskString(tile->cb.src_deadend.jump).c_str(),
                   maskString(raw_src_mask).c_str(),
                   maskString(available_src_mask).c_str());
      fpga::Tile *prev_tile =
          fpga::Device::current().getTile(prev_step.coord.x, prev_step.coord.y);
      if (prev_tile) {
        std::fprintf(stderr,
                     "A_STATE src=%s dst=%s joint=%s local=%s src_deadend=%s\n",
                     maskString(prev_tile->cb.src.jump).c_str(),
                     maskString(prev_tile->cb.dst.jump).c_str(),
                     maskString(prev_tile->cb.joint.jump).c_str(),
                     maskString(prev_tile->cb.local.local).c_str(),
                     maskString(prev_tile->cb.src_deadend.jump).c_str());
      }
      std::fprintf(stderr, "B_CANDIDATES\n");
      for (uint16_t candidate_src : active_src_nodes) {
        int candidate_joint = -1;
        bool has_mask_path =
            step.depth == 0
                ? tile->cb_type->canOut(step.local, candidate_src,
                                        candidate_src, candidate_joint)
                : tile->cb_type->canJump(step.local, candidate_src,
                                         candidate_src, candidate_joint);
        CBState test_cb = tile->cb;
        bool lease_ok = false;
        ConcreteBusyReason busy_reason = ConcreteBusyReason::none;
        if (has_mask_path) {
          lease_ok = step.depth == 0
                         ? leaseConcreteOut(test_cb, step.local, candidate_src,
                                            candidate_joint)
                         : leaseConcreteJump(test_cb, step.local, candidate_src,
                                             candidate_joint);
          if (!lease_ok) {
            busy_reason =
                step.depth == 0
                    ? concreteOutBusyReason(test_cb, candidate_src,
                                            candidate_joint)
                    : concreteJumpBusyReason(test_cb, step.local, candidate_src,
                                             candidate_joint);
          }
        }
        fpga::TileJumpTarget candidate_target =
            fpga::Device::current().resolveJumpToward(*tile, candidate_src,
                                                      target_coord);
        bool is_back = candidate_target.tile &&
                       sameCoord(candidate_target.tile->coord, prev_step.coord);
        int current_dx = std::abs(step.coord.x - target_coord.x);
        int current_dy = std::abs(step.coord.y - target_coord.y);
        int next_dx =
            candidate_target.tile
                ? std::abs(candidate_target.tile->coord.x - target_coord.x)
                : -1;
        int next_dy =
            candidate_target.tile
                ? std::abs(candidate_target.tile->coord.y - target_coord.y)
                : -1;
        int current_distance = routeDistance(step.coord, target_coord);
        int next_distance =
            candidate_target.tile
                ? routeDistance(candidate_target.tile->coord, target_coord)
                : -1;
        bool target_pin_reject =
            candidate_target.tile &&
            sameCoord(candidate_target.tile->coord, target_coord) &&
            !targetDstCanEnterPin(*candidate_target.tile,
                                  candidate_target.dst_node, pin_nodes);
        bool path_conflict_without_edge_guard = false;
        for (int path_index = idx; path_index >= 0;
             path_index = steps[path_index].prev) {
          const Step &route_step = steps[path_index];
          if (path_index != idx && route_step.depth != 0 &&
              route_step.coord.x == step.coord.x &&
              route_step.coord.y == step.coord.y &&
              route_step.local == step.local) {
            path_conflict_without_edge_guard = true;
            break;
          }
          if (route_step.prev >= 0) {
            const Step &route_prev = steps[route_step.prev];
            if (route_prev.coord.x == step.coord.x &&
                route_prev.coord.y == step.coord.y) {
              if (route_step.jump == candidate_src ||
                  (candidate_joint >= 0 &&
                   route_step.joint == candidate_joint)) {
                path_conflict_without_edge_guard = true;
                break;
              }
            }
          }
        }
        std::fprintf(
            stderr,
            "  src=%u '%s' target=(%d,%d) dst=%d '%s' back=%d mask=%d lease=%d "
            "busy=%s target_pin_reject=%d path_conflict_no_edge=%d dist=%d->%d "
            "dx=%d->%d dy=%d->%d\n",
            static_cast<unsigned>(candidate_src),
            nodeDebugName(*tile, fpga::CB_NODE_SRC, candidate_src).c_str(),
            candidate_target.tile ? candidate_target.tile->coord.x : -1,
            candidate_target.tile ? candidate_target.tile->coord.y : -1,
            candidate_target.dst_node,
            candidate_target.tile
                ? nodeDebugName(*candidate_target.tile, fpga::CB_NODE_DST,
                                candidate_target.dst_node)
                      .c_str()
                : "",
            is_back ? 1 : 0, has_mask_path ? 1 : 0, lease_ok ? 1 : 0,
            lease_ok ? "none" : busyReasonName(busy_reason),
            target_pin_reject ? 1 : 0, path_conflict_without_edge_guard ? 1 : 0,
            current_distance, next_distance, current_dx, next_dx, current_dy,
            next_dy);
      }
      std::fprintf(stderr, "END_ARENA_BACKTRACK\n\n");
    };

    bool accepted_from_step = false;
    rtl::Net *preempt_victim = nullptr;
    size_t preempt_binding_index = std::numeric_limits<size_t>::max();
    size_t preempt_route_size = std::numeric_limits<size_t>::max();
    bool preempt_route_complete = true;
    bool preempt_node_shared = true;
    int preempt_src = -1;
    bool retry_current_step = false;
    int child_step = -1;
    bool accepted_final_target = false;
    int direction_passes = inspect_leased_sources ? 2 : 1;
    for (int direction_pass = 0; direction_pass < direction_passes;
         ++direction_pass) {
      if (accepted_from_step) {
        break;
      }
      for (uint16_t src_node : active_src_nodes) {
        bool live_source_leased = tile->cb.src.jump.testBit(src_node);
        if (!pnr::sourceCandidateInPhase(live_source_leased, direction_pass)) {
          continue;
        }
        if (!available_src_mask.testBit(src_node)) {
          if (stats) {
            ++stats->edge_rejected_src_deadend;
          }
          ROUTE_DEBUG_LOG(
              "reject unavailable src: coord=({},{}), depth={}, from_node={} "
              "'{}', src={} '{}', leased={}, deadend={}",
              step.coord.x, step.coord.y, step.depth, step.local,
              nodeDebugName(*tile, from_type, step.local),
              static_cast<int>(src_node),
              nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node),
              tile->cb.src.jump.testBit(src_node),
              combined_src_deadend.testBit(src_node));
          if (trace_move_paths) {
            int trace_joint = -1;
            int trace_joint2 = -1;
            if (step.depth == 0) {
              tile->cb_type->canOut(step.local, src_node, src_node, trace_joint,
                                    &trace_joint2);
            } else {
              tile->cb_type->canJump(step.local, src_node, src_node,
                                     trace_joint, &trace_joint2);
            }
            fpga::TileJumpTarget trace_target =
                fpga::Device::current().resolveJumpToward(*tile, src_node,
                                                          target_coord);
            trace_move_path(combined_src_deadend.testBit(src_node)
                                ? "unavailable_deadend"
                                : "unavailable_leased",
                            idx, *tile, src_node, trace_joint, trace_joint2,
                            &trace_target);
          }
          continue;
        }
        if (stats) {
          ++stats->edge_trials;
          ++stats->trials_by_depth[statDepthBucket(step.depth)];
        }
        int joint = -1;
        int joint2 = -1;
        auto name_start = profile_route
                              ? std::chrono::steady_clock::now()
                              : std::chrono::steady_clock::time_point{};
        bool has_mask_path =
            step.depth == 0 ? tile->cb_type->canOut(step.local, src_node,
                                                    src_node, joint, &joint2)
                            : tile->cb_type->canJump(step.local, src_node,
                                                     src_node, joint, &joint2);
        if (stats && profile_route) {
          stats->edge_name_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - name_start)
                  .count());
        }
        if (!has_mask_path) {
          if (RouteCongestionTrace::current)
            RouteCongestionTrace::current->blocked(*tile, "crossbar_topology_missing",
                {{from_type, step.local}, {fpga::CB_NODE_SRC, src_node}});
          ROUTE_DEBUG_LOG(
              "reject no crossbar mask path: coord=({},{}), depth={}, "
              "from_node={} '{}', src={} '{}', joint={}, incoming_dst='{}'",
              step.coord.x, step.coord.y, step.depth, step.local,
              nodeDebugName(*tile, from_type, step.local),
              static_cast<int>(src_node),
              nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), joint,
              step.dst_wire);
          if (stats) {
            ++stats->edge_rejected_no_name;
          }
          fpga::TileJumpTarget trace_target;
          if (trace_move_paths) {
            trace_target = fpga::Device::current().resolveJumpToward(
                *tile, src_node, target_coord);
          }
          trace_move_path("no_crossbar_path", idx, *tile, src_node, joint,
                          joint2, trace_move_paths ? &trace_target : nullptr);
          continue;
        }
        std::string src_wire;
        auto cb_copy_start = profile_route
                                 ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
        CBState test_cb = temp_state_for_step(idx, *tile);
        if (stats && profile_route) {
          stats->edge_cb_copy_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - cb_copy_start)
                  .count());
        }
        bool lease_ok = false;
        bool ignore_src_deadend =
            ignore_deadends || target_hop_deadend_srcs.testBit(src_node);
        auto lease_start = profile_route
                               ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
        if (reuses_start_dst && idx == 0 && step.depth != 0) {
          lease_ok = leaseConcreteFork(test_cb, step.local, src_node, joint,
                                       ignore_src_deadend, joint2);
        } else {
          lease_ok = step.depth == 0
                         ? leaseConcreteOut(test_cb, step.local, src_node,
                                            joint, ignore_src_deadend, joint2)
                         : leaseConcreteJump(test_cb, step.local, src_node,
                                             joint, ignore_src_deadend, joint2);
        }
        if (stats && profile_route) {
          stats->edge_lease_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - lease_start)
                  .count());
        }
        if (!lease_ok) {
          if (RouteCongestionTrace::current) {
            RouteCongestionTrace::current->blocked(*tile, "crossbar_lease",
                {{from_type, step.local}, {fpga::CB_NODE_SRC, src_node},
                 {fpga::CB_NODE_JOINT, joint}, {fpga::CB_NODE_JOINT, joint2}}, &test_cb);
          }
          ConcreteBusyReason reason =
              reuses_start_dst && idx == 0 && step.depth != 0
                  ? concreteForkBusyReason(test_cb, step.local, src_node,
                                           joint, joint2)
              : step.depth == 0
                  ? concreteOutBusyReason(test_cb, src_node, joint)
                  : concreteJumpBusyReason(test_cb, step.local, src_node,
                                           joint);
          ConcreteBusyReason live_reason =
              reuses_start_dst && idx == 0 && step.depth != 0
                  ? concreteForkBusyReason(tile->cb, step.local, src_node,
                                           joint, joint2)
              : step.depth == 0
                  ? concreteOutBusyReason(tile->cb, src_node, joint)
                  : concreteJumpBusyReason(tile->cb, step.local, src_node,
                                           joint);
          if (debug_route && debug_route_lines < debug_route_line_limit) {
            rtl::Net *busy_src_net =
                live_reason == ConcreteBusyReason::src
                    ? fpga::findNetByNode(*tile, fpga::CB_NODE_SRC, src_node,
                                          false)
                    : nullptr;
            rtl::Net *busy_src_transit_net =
                live_reason == ConcreteBusyReason::src
                    ? fpga::findNetByNode(*tile, fpga::CB_NODE_SRC, src_node,
                                          true)
                    : nullptr;
            rtl::Net *busy_dst_net =
                live_reason == ConcreteBusyReason::dst
                    ? fpga::findNetByNode(*tile, fpga::CB_NODE_DST, step.local,
                                          false)
                    : nullptr;
            rtl::Net *busy_dst_transit_net =
                live_reason == ConcreteBusyReason::dst
                    ? fpga::findNetByNode(*tile, fpga::CB_NODE_DST, step.local,
                                          true)
                    : nullptr;
            std::string busy_src_owner =
                live_reason == ConcreteBusyReason::src
                    ? srcOwnerBindingForDump(tile, src_node)
                    : std::string{};
            if (live_reason == ConcreteBusyReason::src &&
                busy_src_owner.empty()) {
              busy_src_owner =
                  nodeOwnerForDump(tile, fpga::CB_NODE_SRC, src_node, false);
            }
            std::string busy_dst_owner =
                live_reason == ConcreteBusyReason::dst
                    ? nodeOwnerForDump(tile, fpga::CB_NODE_DST, step.local,
                                       false)
                    : std::string{};
            ROUTE_DEBUG_LOG(
                "reject busy: coord=({},{}), depth={}, from_node={} '{}', "
                "src={} '{}', joint={}, src_wire='{}', reason={}, "
                "live_reason={}, busy_src_net='{}', busy_src_transit_net='{}', "
                "busy_dst_net='{}', busy_dst_transit_net='{}', "
                "busy_src_owner=\"{}\", busy_dst_owner=\"{}\", "
                "temp_src_mask={}, temp_dst_mask={}, temp_deadend={}, "
                "live_src_mask={}, live_dst_mask={}, live_local_mask={}",
                step.coord.x, step.coord.y, step.depth, step.local,
                nodeDebugName(*tile, from_type, step.local),
                static_cast<int>(src_node),
                nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), joint,
                src_wire, busyReasonName(reason), busyReasonName(live_reason),
                busy_src_net ? busy_src_net->makeName(FULL_NAME_LIMIT)
                             : std::string{},
                busy_src_transit_net
                    ? busy_src_transit_net->makeName(FULL_NAME_LIMIT)
                    : std::string{},
                busy_dst_net ? busy_dst_net->makeName(FULL_NAME_LIMIT)
                             : std::string{},
                busy_dst_transit_net
                    ? busy_dst_transit_net->makeName(FULL_NAME_LIMIT)
                    : std::string{},
                busy_src_owner, busy_dst_owner, maskString(test_cb.src.jump),
                maskString(test_cb.dst.jump),
                maskString(test_cb.src_deadend.jump),
                maskString(tile->cb.src.jump), maskString(tile->cb.dst.jump),
                maskString(tile->cb.local.local));
          }
          if (step_preempt_allowed) {
            TransitVictim transit_victim;
            if (reason == live_reason && reason == ConcreteBusyReason::src) {
              transit_victim = findTransitSrcVictim(
                  *tile, src_node, current_net, current_source,
                  current_source_port, current_route_name, router);
            }
            if (transit_victim.net &&
                transit_victim.binding_index <
                    transit_victim.net->routes.size() &&
                (current_route_name.empty() ||
                 transit_victim.net->routes[transit_victim.binding_index]
                         .route_name != current_route_name)) {
              rtl::NetRouteBinding &victim_binding =
                  transit_victim.net->routes[transit_victim.binding_index];
              const std::vector<Wire> *victim_route =
                  routeBindingRoute(victim_binding);
              const size_t route_size = victim_route ? victim_route->size() : 0;
              const bool route_complete =
                  victim_route && routeIsComplete(*victim_route);
              if (pnr::selectPreemptionCandidate(preempt_victim != nullptr,
                                                 preempt_route_complete,
                                                 route_complete)) {
                preempt_victim = transit_victim.net;
                preempt_binding_index = transit_victim.binding_index;
                preempt_route_size = route_size;
                preempt_route_complete = route_complete;
                preempt_node_shared = transit_victim.node_shared;
                preempt_src = src_node;
              }
            }
          }
        }
        if (!lease_ok) {
          if (stats) {
            ++stats->edge_rejected_busy;
            ConcreteBusyReason reason =
                reuses_start_dst && idx == 0 && step.depth != 0
                    ? concreteForkBusyReason(test_cb, step.local, src_node,
                                             joint, joint2)
                : step.depth == 0
                    ? concreteOutBusyReason(test_cb, src_node, joint)
                    : concreteJumpBusyReason(test_cb, step.local, src_node,
                                             joint);
            if (reason == ConcreteBusyReason::dst) {
              ++stats->edge_rejected_busy_dst;
            } else if (reason == ConcreteBusyReason::src) {
              ++stats->edge_rejected_busy_src;
            } else if (reason == ConcreteBusyReason::local) {
              ++stats->edge_rejected_busy_local;
            } else if (reason == ConcreteBusyReason::src_deadend) {
              ++stats->edge_rejected_deadend;
              ++stats->edge_rejected_src_deadend;
            }
            stats->has_last_busy = true;
            stats->last_busy_coord = tile->coord;
            stats->last_busy_depth = step.depth;
            stats->last_busy_local = step.local;
            stats->last_busy_src = src_node;
            stats->last_busy_src_mask = test_cb.src.jump;
            stats->last_busy_dst_mask = test_cb.dst.jump;
            stats->last_busy_local_mask = tile->cb.local.local;
          }
          fpga::TileJumpTarget trace_target;
          if (trace_move_paths) {
            trace_target = fpga::Device::current().resolveJumpToward(
                *tile, src_node, target_coord);
          }
          trace_move_path("crossbar_lease_blocked", idx, *tile, src_node, joint,
                          joint2, trace_move_paths ? &trace_target : nullptr);
          continue;
        }
        auto resolve_start = profile_route
                                 ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
        fpga::TileJumpTarget target = fpga::Device::current().resolveJumpToward(
            *tile, src_node, target_coord);
        if (stats && profile_route) {
          stats->edge_resolve_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - resolve_start)
                  .count());
        }
        Coord next;
        int next_local = -1;
        if (target.tile && target.tile->cb_type && target.dst_node >= 0) {
          next = target.tile->coord;
          next_local = target.dst_node;
          dump_backtrack_candidate_table(src_node, target);
          if (target.tile->cb.dst.jump.testBit(next_local)) {
            if (RouteCongestionTrace::current)
              RouteCongestionTrace::current->blocked(*target.tile, "landing_leased",
                  {{fpga::CB_NODE_DST, next_local}});
            ROUTE_DEBUG_LOG(
                "reject busy resolved destination: from=({},{}), depth={}, "
                "src={} '{}', target=({},{}), dst_node={} '{}', owner='{}', "
                "transit='{}'",
                step.coord.x, step.coord.y, step.depth,
                static_cast<int>(src_node),
                nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), next.x,
                next.y, next_local,
                nodeDebugName(*target.tile, fpga::CB_NODE_DST, next_local),
                nodeOwnerForDump(target.tile, fpga::CB_NODE_DST, next_local,
                                 false),
                nodeOwnerForDump(target.tile, fpga::CB_NODE_DST, next_local,
                                 true));
            if (stats) {
              ++stats->edge_rejected_busy;
              ++stats->edge_rejected_busy_dst;
            }
            trace_move_path("target_dst_leased", idx, *tile, src_node, joint,
                            joint2, &target);
            continue;
          }
          bool target_pin_reject = false;
          if (sameCoord(next, target_coord)) {
            auto target_pin_start =
                profile_route ? std::chrono::steady_clock::now()
                              : std::chrono::steady_clock::time_point{};
            target_pin_reject =
                !targetDstCanEnterPin(*target.tile, next_local, pin_nodes);
            if (stats && profile_route) {
              stats->edge_target_pin_ns += static_cast<uint64_t>(
                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now() - target_pin_start)
                      .count());
            }
          }
          if (target_pin_reject) {
            if (RouteCongestionTrace::current) {
              pin_nodes.for_each_set_bit([&](int pin) {
                RouteCongestionTrace::current->blocked(*target.tile, "target_pin_unreachable",
                    {{fpga::CB_NODE_DST, next_local}, {fpga::CB_NODE_LOCAL, pin}});
                return false;
              });
            }
            ROUTE_DEBUG_LOG(
                "reject target cannot enter pin: from=({},{}), src={} '{}', "
                "src_wire='{}', target=({},{}), dst_node={} '{}', "
                "dst_wire='{}', pin_nodes={}",
                step.coord.x, step.coord.y, static_cast<int>(src_node),
                nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), src_wire,
                next.x, next.y, next_local,
                nodeDebugName(*target.tile, fpga::CB_NODE_DST, next_local),
                target.dst_wire, maskString(pin_nodes));
            if (routeDebugMatches("SCALEPNR_DEBUG_TARGET_NET", debug_net)) {
              PNR_LOG1(
                  "ROUT",
                  "routeDesign target reject: net='{}', from='{}'/'{}', "
                  "target_tile='{}' cb='{}' cb_ptr={} coord=({},{}), "
                  "dst_node={} '{}', pin_nodes={}, dst_wire='{}'",
                  debug_net,
                  current_source ? current_source->makeName(FULL_NAME_LIMIT)
                                 : std::string{},
                  current_source_port, target.tile->makeName(),
                  target.tile->cb_type ? target.tile->cb_type->name
                                       : std::string{},
                  static_cast<const void *>(target.tile->cb_type),
                  target.tile->coord.x, target.tile->coord.y, next_local,
                  nodeDebugName(*target.tile, fpga::CB_NODE_DST, next_local),
                  maskString(pin_nodes), target.dst_wire);
              pin_nodes.for_each_set_bit([&](int pin) {
                int debug_joint = -1;
                bool debug_can_in =
                    target.tile->cb_type &&
                    target.tile->cb_type->canIn(next_local, pin, debug_joint);
                bool debug_direct =
                    target.tile->cb_type &&
                    (target.tile->cb_type->dst_local[next_local].local &
                     (NodeMask{0, 1} << pin)) != NodeMask{};
                PNR_LOG1(
                    "ROUT",
                    "routeDesign target reject pin: pin={} '{}', leased={}, "
                    "can_in={}, direct={}, joint={}, dst_local={}, "
                    "dst_joint={}",
                    pin, nodeDebugName(*target.tile, fpga::CB_NODE_LOCAL, pin),
                    target.tile->isPinNodeLeased(pin), debug_can_in,
                    debug_direct, debug_joint,
                    target.tile->cb_type
                        ? maskString(
                              target.tile->cb_type->dst_local[next_local].local)
                        : std::string{},
                    target.tile->cb_type
                        ? maskString(
                              target.tile->cb_type->dst_joint[next_local].joint)
                        : std::string{});
                return false;
              });
            }
            if (stats) {
              ++stats->edge_rejected_no_target;
            }
            trace_move_target_entries(idx, *tile, src_node, joint, joint2,
                                      target);
            continue;
          }
        } else {
          ROUTE_DEBUG_LOG(
              "reject unresolved jump: coord=({},{}), depth={}, from_node={} "
              "'{}', src={} '{}', src_wire='{}', src_mask={}, dst_mask={}, "
              "joint_mask={}, local_mask={}, src_deadend={}",
              step.coord.x, step.coord.y, step.depth, step.local,
              nodeDebugName(*tile, from_type, step.local),
              static_cast<int>(src_node),
              nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), src_wire,
              maskString(tile->cb.src.jump), maskString(tile->cb.dst.jump),
              maskString(tile->cb.joint.jump), maskString(tile->cb.local.local),
              maskString(tile->cb.src_deadend.jump));
          if (stats) {
            ++stats->edge_rejected_no_target;
          }
          trace_move_path("jump_unresolved", idx, *tile, src_node, joint,
                          joint2, &target);
          continue;
        }
        // Concrete names annotate an already viable numeric edge. Delaying
        // this lookup avoids string work for candidates rejected by masks.
        src_wire = concreteSrcWireName(*tile, from_type, step.local, src_node,
                                       joint, step.dst_wire);
        if (step.depth > 0 && !step.dst_wire.empty() && src_wire.empty()) {
          ROUTE_DEBUG_LOG(
              "reject no concrete transit wire: coord=({},{}), depth={}, "
              "from_node={} '{}', incoming_dst='{}', src={} '{}', joint={}",
              step.coord.x, step.coord.y, step.depth, step.local,
              nodeDebugName(*tile, from_type, step.local), step.dst_wire,
              static_cast<int>(src_node),
              nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), joint);
          if (stats) {
            ++stats->edge_rejected_no_name;
          }
          trace_move_path("no_concrete_transit", idx, *tile, src_node, joint,
                          joint2, &target);
          continue;
        }
        int next_depth = step.depth + 1;
        auto state_start = profile_route
                               ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
        steps.push_back(Step{next, next_local, idx, src_node, target.jump_node,
                             joint, joint2, next_depth, src_wire,
                             target.dst_wire});
        if (stats && profile_route) {
          stats->edge_state_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - state_start)
                  .count());
        }
        ROUTE_DEBUG_LOG(
            "accept step: from=({},{}), depth={}, from_node={} '{}', src={} "
            "'{}', src_wire='{}', next=({},{}), dst_node={} '{}', "
            "dst_wire='{}', joint={}, distance={}, accepted_src_bit={}, "
            "accepted_dst_bit={}, accepted_joint_bit={}, tested_src_mask={}, "
            "tested_dst_mask={}, tested_joint_mask={}, tested_local_mask={}, "
            "live_src_mask={}, live_dst_mask={}, live_joint_mask={}, "
            "live_local_mask={}, src_deadend={}, target_dst_mask={}, "
            "target_joint_mask={}, target_local_mask={}",
            step.coord.x, step.coord.y, step.depth, step.local,
            nodeDebugName(*tile, from_type, step.local),
            static_cast<int>(src_node),
            nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), src_wire, next.x,
            next.y, next_local,
            target.tile
                ? nodeDebugName(*target.tile, fpga::CB_NODE_DST, next_local)
                : std::string{},
            target.dst_wire, joint, routeDistance(next, to.coord),
            maskString(NodeMask{0, 1} << src_node),
            maskString(step.depth == 0 ? NodeMask{}
                                       : (NodeMask{0, 1} << step.local)),
            maskString(joint >= 0 ? (NodeMask{0, 1} << joint) : NodeMask{}),
            maskString(test_cb.src.jump),
            maskString(test_cb.dst.jump), maskString(test_cb.joint.jump),
            maskString(test_cb.local.local), maskString(tile->cb.src.jump),
            maskString(tile->cb.dst.jump), maskString(tile->cb.joint.jump),
            maskString(tile->cb.local.local),
            maskString(tile->cb.src_deadend.jump),
            target.tile ? maskString(target.tile->cb.dst.jump) : std::string{},
            target.tile ? maskString(target.tile->cb.joint.jump)
                        : std::string{},
            target.tile ? maskString(target.tile->cb.local.local)
                        : std::string{});
        if (sameCoord(next, target_coord)) {
          trace_move_target_entries(idx, *tile, src_node, joint, joint2,
                                    target);
        } else {
          trace_move_path("accepted_step", idx, *tile, src_node, joint, joint2,
                          &target);
        }
        if (stats) {
          ++stats->edge_accepted;
          ++stats->accepted_by_depth[statDepthBucket(step.depth)];
        }
        accepted_from_step = true;
        if (report_start_step) {
          report->start_accepted = true;
        }
        int next_idx = static_cast<int>(steps.size() - 1);
        child_step = next_idx;
        if (sameCoord(next, target_coord)) {
          pin_nodes.for_each_set_bit([&](int pin) {
            if (target.tile->isPinNodeLeased(pin)) {
              return false;
            }
            for (const TerminalEntryCandidate &candidate_entry :
                 targetEntryCandidates(*target.tile->cb_type, next_local,
                                       pin)) {
              if (!pnr::terminalEntryAvoidsReservedJoints(
                      candidate_entry.joint, candidate_entry.joint2,
                      packed_reserved_joints)) {
                continue;
              }
              CBState terminal_test = target.tile->cb;
              if (!leaseConcreteTerminal(terminal_test, next_local, pin,
                                         candidate_entry)) {
                continue;
              }
              final_step = next_idx;
              final_pin = pin;
              final_joint = candidate_entry.joint;
              final_joint2 = candidate_entry.joint2;
              final_allows_existing_dst = false;
              final_entry_kind = candidate_entry.kind;
              accepted_final_target = true;
              ROUTE_DEBUG_LOG(
                  "accept final target: from=({},{}), src={} '{}', "
                  "target=({},{}), dst_node={} '{}', pin={} '{}', "
                  "entry_kind={}, joint={}",
                  step.coord.x, step.coord.y, static_cast<int>(src_node),
                  nodeDebugName(*tile, fpga::CB_NODE_SRC, src_node), next.x,
                  next.y, next_local,
                  nodeDebugName(*target.tile, fpga::CB_NODE_DST, next_local),
                  pin, nodeDebugName(*target.tile, fpga::CB_NODE_LOCAL, pin),
                  static_cast<int>(candidate_entry.kind),
                  candidate_entry.joint);
              break;
            }
            return accepted_final_target;
          });
        }
        break;
      }
    }
    if (accepted_final_target) {
      break;
    }
    if (accepted_from_step && child_step >= 0) {
      frontier.push_back(child_step);
      continue;
    }
    if (preempt_victim && router &&
        pnr::shouldPreemptTakeoff(accepted_from_step,
                                  step_preempt_allowed)) {
      if (stats) {
        ++stats->preempt_attempts;
      }
      rtl::NetRouteBinding *victim_binding = nullptr;
      size_t victim_binding_index = std::numeric_limits<size_t>::max();
      if (preempt_binding_index < preempt_victim->routes.size()) {
        rtl::NetRouteBinding &binding =
            preempt_victim->routes[preempt_binding_index];
        if (binding.from && !binding.from_port.empty()) {
          victim_binding = &binding;
          victim_binding_index = preempt_binding_index;
        }
      }
      if (!victim_binding) {
        for (size_t binding_index = 0;
             binding_index < preempt_victim->routes.size(); ++binding_index) {
          rtl::NetRouteBinding &binding = preempt_victim->routes[binding_index];
          if (binding.from && !binding.from_port.empty()) {
            victim_binding = &binding;
            victim_binding_index = binding_index;
            break;
          }
        }
      }
      std::string victim_route_name =
          victim_binding ? victim_binding->route_name : std::string{};
      rtl::Inst *victim_from = victim_binding ? victim_binding->from : nullptr;
      rtl::Inst *victim_to = victim_binding ? victim_binding->to : nullptr;
      std::string victim_from_port =
          victim_binding ? victim_binding->from_port : std::string{};
      std::string victim_to_port =
          victim_binding ? victim_binding->to_port : std::string{};
      if (victim_binding &&
          victim_binding_index != std::numeric_limits<size_t>::max() &&
          [&]() {
            const std::vector<Wire> *route =
                routeBindingRoute(*victim_binding);
            preempt_route_size = route ? route->size() : 0;
            preempt_route_complete = route && routeIsComplete(*route);
            return true;
          }() &&
          unrouteVictimBinding(
              router,
              TransitVictim{preempt_victim, victim_binding_index,
                            preempt_node_shared, tile->coord, fpga::CB_NODE_SRC,
                            preempt_src},
              true, router && router->fanout_stage, false, current_route_name,
              current_source, current_source_port)) {
        if (stats) {
          ++stats->preempt_success;
          const std::vector<Wire> *remaining_route =
              victim_binding_index < preempt_victim->routes.size()
                  ? routeBindingRoute(
                        preempt_victim->routes[victim_binding_index])
                  : nullptr;
          const size_t remaining_size =
              remaining_route ? remaining_route->size() : 0;
          stats->preempt_removed_fragments +=
              preempt_route_size > remaining_size
                  ? preempt_route_size - remaining_size
                  : 0;
          if (preempt_route_complete) {
            ++stats->preempt_complete_victims;
            ++stats->preempt_takeoff_complete_victims;
          } else {
            ++stats->preempt_partial_victims;
            ++stats->preempt_takeoff_partial_victims;
          }
        }
        PNR_LOG2("ROUT",
                 "routeDesign preempt: current='{}', tile=({},{}), local={}, "
                 "src={}, victim='{}', victim_from='{}'/'{}', "
                 "victim_to='{}'/'{}', victim_route='{}'",
                 debug_net, tile->coord.x, tile->coord.y, step.local,
                 preempt_src, preempt_victim->makeName(FULL_NAME_LIMIT),
                 victim_from ? victim_from->makeName(FULL_NAME_LIMIT)
                             : std::string{},
                 victim_from_port,
                 victim_to ? victim_to->makeName(FULL_NAME_LIMIT)
                           : std::string{},
                 victim_to_port, victim_route_name);
        retry_current_step = true;
      }
    } else if (!accepted_from_step && idx == 0 && !preempt_victim) {
      brutalLocalExitFailure(*tile, src_nodes, step.local, to.coord,
                             current_net, stats);
    }
    if (retry_current_step) {
      frontier.push_back(idx);
      continue;
    }
    if (!accepted_from_step) {
      remember_exhausted_step(idx);
    }
    if (pnr::preserveBlockedEndpointForDocking(
            accepted_from_step, step.depth,
            pnr::dockingWindowDistance(step.coord, target_coord),
            docking_radius)) {
      remember_docking_candidate(idx);
      ROUTE_DEBUG_LOG(
          "defer blocked endpoint to docking and retry parent: coord=({},{}), "
          "depth={}, dst={} '{}', target=({},{}), radius={}",
          step.coord.x, step.coord.y, step.depth, step.local,
          nodeDebugName(*tile, fpga::CB_NODE_DST, step.local), target_coord.x,
          target_coord.y, docking_radius);
      if (record_failed_incoming_src(idx, step, "blocked_docking_candidate",
                                     false)) {
        frontier.push_back(step.prev);
      }
      continue;
    }
    if (record_failed_incoming_src(idx, step, "no_accepted_edges")) {
      frontier.push_back(step.prev);
    }
    continue;
  }
  trace_move_target_summary();
  if (stats && profile_route) {
    stats->best_first_loop_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - search_loop_start)
            .count());
  }

  bool completed = final_pin >= 0;
  bool docking_terminal_preempted = false;
  bool docking_bridge_preempted = false;
  bool docking_attempted = false;
  ROUTE_DEBUG_LOG(
      "search end: completed={}, final_step={}, steps={}, max_steps={}",
      completed, final_step, steps.size(), max_steps);
  if (!completed && !docking_candidate_steps.empty()) {
    std::vector<int> tried_anchors;
    for (int candidate_step : docking_candidate_steps) {
      if (candidate_step < 0 ||
          candidate_step >= static_cast<int>(steps.size())) {
        continue;
      }
      std::vector<int> selected_path;
      for (int idx = candidate_step; idx >= 0; idx = steps[idx].prev) {
        selected_path.push_back(idx);
      }
      std::reverse(selected_path.begin(), selected_path.end());

      int docking_anchor_step = -1;
      int candidate_radius = isIOBEndpoint(dst_inst) ? 12 : 5;
      std::vector<std::pair<int, int>> depth_distance;
      depth_distance.reserve(selected_path.size());
      for (int idx : selected_path) {
        depth_distance.push_back(
            {steps[idx].depth,
             pnr::dockingWindowDistance(steps[idx].coord, to.coord)});
      }
      int anchor_path_index =
          pnr::latestGroundingAnchor(depth_distance, candidate_radius);
      if (anchor_path_index >= 0) {
        docking_anchor_step = selected_path[anchor_path_index];
      }
      if (docking_anchor_step < 0 ||
          std::find(tried_anchors.begin(), tried_anchors.end(),
                    docking_anchor_step) != tried_anchors.end()) {
        continue;
      }
      tried_anchors.push_back(docking_anchor_step);

      Step &anchor = steps[docking_anchor_step];
      Tile *anchor_tile =
          fpga::Device::current().getTile(anchor.coord.x, anchor.coord.y);
      if (!anchor_tile || !anchor_tile->cb_type || anchor.depth <= 0) {
        continue;
      }
      auto dock_start = profile_route ? std::chrono::steady_clock::now()
                                      : std::chrono::steady_clock::time_point{};
      docking_attempted = true;
      DockingResult candidate_route =
          isIOBEndpoint(dst_inst)
              ? dockIOB(*anchor_tile, anchor.local, anchor.dst_wire, to,
                        pin_nodes, trace_docking_paths, packed_reserved_joints)
              : dockGrounding(
                    *anchor_tile, anchor.local, anchor.dst_wire, to, pin_nodes,
                    5, 5, trace_docking_paths, packed_reserved_joints,
                    router ? &router->backwardDockingIndex(to.coord, 5)
                           : nullptr);
      if (stats && profile_route) {
        stats->best_first_dock_ns += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - dock_start)
                .count());
      }
      ROUTE_DEBUG_LOG(
          "candidate docking: endpoint_step={}, anchor=({},{}), dst_node={}, "
          "success={}, seeds={}, entries={}, busy={}, fpush={}, bpush={}",
          candidate_step, anchor.coord.x, anchor.coord.y, anchor.local,
          candidate_route.success, candidate_route.target_seed_count,
          candidate_route.target_entry_count, candidate_route.target_busy_count,
          candidate_route.forward_push_count,
          candidate_route.backward_push_count);
      trace_docking_backward_attempts(candidate_route);
      if (!candidate_route.success) {
        failed_docking_by_anchor.emplace(docking_anchor_step,
                                         std::move(candidate_route));
        continue;
      }
      docked_route = std::move(candidate_route);
      completed = true;
      completed_by_docking = true;
      final_step = docking_anchor_step;
      trace_move_docking_path(docking_anchor_step, docked_route);
      break;
    }
    if (!completed && final_step < 0) {
      final_step = partial_fallback_step >= 0 ? partial_fallback_step
                                              : docking_candidate_steps.front();
    }
  }
  if (!completed) {
    if (target_enter_failed) {
      ROUTE_DEBUG_TEXT("search target enter failed; keeping first viable "
                       "priority partial path if one was found");
    }
    ROUTE_DEBUG_LOG("partial select: final_step={}", final_step);
    if (final_step >= 0) {
      ROUTE_DEBUG_LOG("partial selected path: {}",
                      selected_path_debug_string(final_step));
    }
    int iob_docking_anchor_step = -1;
    if (isIOBEndpoint(dst_inst) && final_step >= 0) {
      for (int idx = final_step; idx >= 0; idx = steps[idx].prev) {
        if (steps[idx].depth > 0 &&
            pnr::dockingWindowDistance(steps[idx].coord, to.coord) <= 12) {
          iob_docking_anchor_step = idx;
          break;
        }
      }
      ROUTE_DEBUG_LOG(
          "iob docking anchor scan: final_step={}, final_coord=({},{}), "
          "final_distance={}, selected_step={}, selected_coord=({},{}), "
          "selected_distance={}, target=({},{}), pin_nodes={}",
          final_step, steps[final_step].coord.x, steps[final_step].coord.y,
          routeDistance(steps[final_step].coord, to.coord),
          iob_docking_anchor_step,
          iob_docking_anchor_step >= 0 ? steps[iob_docking_anchor_step].coord.x
                                       : -1,
          iob_docking_anchor_step >= 0 ? steps[iob_docking_anchor_step].coord.y
                                       : -1,
          iob_docking_anchor_step >= 0
              ? pnr::dockingWindowDistance(steps[iob_docking_anchor_step].coord,
                                           to.coord)
              : -1,
          to.coord.x, to.coord.y, maskString(pin_nodes));
    }
    bool use_iob_docking = iob_docking_anchor_step >= 0;
    int grounding_docking_anchor_step = -1;
    if (!use_iob_docking && final_step >= 0) {
      std::vector<int> selected_path;
      for (int idx = final_step; idx >= 0; idx = steps[idx].prev) {
        selected_path.push_back(idx);
      }
      std::reverse(selected_path.begin(), selected_path.end());
      std::vector<std::pair<int, int>> depth_distance;
      depth_distance.reserve(selected_path.size());
      for (int idx : selected_path) {
        depth_distance.push_back(
            {steps[idx].depth,
             pnr::dockingWindowDistance(steps[idx].coord, to.coord)});
      }
      int anchor_path_index = pnr::latestGroundingAnchor(depth_distance, 5);
      if (anchor_path_index >= 0) {
        grounding_docking_anchor_step = selected_path[anchor_path_index];
      }
    }
    // Docking is a normal final-entry attempt once the routed frontier is close
    // enough. Do not wait for an exact target-tile entry failure; nearby rails
    // may be the dockable ones.
    bool use_grounding_docking = grounding_docking_anchor_step >= 0;
    if (!completed && (use_iob_docking || use_grounding_docking)) {
      int docking_anchor_step = use_iob_docking ? iob_docking_anchor_step
                                                : grounding_docking_anchor_step;
      Step &anchor = steps[docking_anchor_step];
      Tile *anchor_tile =
          fpga::Device::current().getTile(anchor.coord.x, anchor.coord.y);
      if (anchor_tile && anchor_tile->cb_type && anchor.depth > 0) {
        docking_attempted = true;
        auto previous = failed_docking_by_anchor.find(docking_anchor_step);
        auto dock_start =
            previous == failed_docking_by_anchor.end() && profile_route
                ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
        if (previous != failed_docking_by_anchor.end()) {
          docked_route = std::move(previous->second);
        } else {
          docked_route =
              use_iob_docking
                  ? dockIOB(*anchor_tile, anchor.local, anchor.dst_wire, to,
                            pin_nodes, trace_docking_paths,
                            packed_reserved_joints)
                  : dockGrounding(
                        *anchor_tile, anchor.local, anchor.dst_wire, to,
                        pin_nodes, 5, 5, trace_docking_paths,
                        packed_reserved_joints,
                        router ? &router->backwardDockingIndex(to.coord, 5)
                               : nullptr);
        }
        trace_docking_backward_attempts(docked_route);
        if (stats && previous == failed_docking_by_anchor.end() &&
            profile_route) {
          stats->best_first_dock_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - dock_start)
                  .count());
        }
        if (docked_route.success) {
          completed = true;
          completed_by_docking = true;
          final_step = docking_anchor_step;
          trace_move_docking_path(docking_anchor_step, docked_route);
          ROUTE_DEBUG_LOG(
              "{} docking succeeded: anchor=({},{}), dst_node={}, "
              "fragments={}, seeds={}, entries={}, busy={}, fpush={}, "
              "bpush={}, fpop={}, bpop={}, bscan={}, breach={}, bno_prev={}, "
              "btopology={}, bbusy={}, bseen={}, bdead={}, bdead_reject={}",
              use_iob_docking ? "iob" : "grounding", anchor.coord.x,
              anchor.coord.y, anchor.local, docked_route.fragments.size(),
              docked_route.target_seed_count, docked_route.target_entry_count,
              docked_route.target_busy_count, docked_route.forward_push_count,
              docked_route.backward_push_count, docked_route.forward_pop_count,
              docked_route.backward_pop_count,
              docked_route.backward_mapping_scan_count,
              docked_route.backward_mapping_reaches_count,
              docked_route.backward_missing_prev_dst_count,
              docked_route.backward_topology_reject_count,
              docked_route.backward_busy_reject_count,
              docked_route.backward_seen_reject_count,
              docked_route.backward_deadend_count,
              docked_route.backward_deadend_reject_count);
        } else {
          ROUTE_DEBUG_LOG(
              "{} docking failed: anchor=({},{}), dst_node={}, seeds={}, "
              "entries={}, busy={}, bridges={}, fpush={}, bpush={}, fpop={}, "
              "bpop={}, bscan={}, breach={}, bno_prev={}, btopology={}, "
              "bbusy={}, bseen={}, bdead={}, bdead_reject={}",
              use_iob_docking ? "iob" : "grounding", anchor.coord.x,
              anchor.coord.y, anchor.local, docked_route.target_seed_count,
              docked_route.target_entry_count, docked_route.target_busy_count,
              docked_route.blocked_bridges.size(),
              docked_route.forward_push_count, docked_route.backward_push_count,
              docked_route.forward_pop_count, docked_route.backward_pop_count,
              docked_route.backward_mapping_scan_count,
              docked_route.backward_mapping_reaches_count,
              docked_route.backward_missing_prev_dst_count,
              docked_route.backward_topology_reject_count,
              docked_route.backward_busy_reject_count,
              docked_route.backward_seen_reject_count,
              docked_route.backward_deadend_count,
              docked_route.backward_deadend_reject_count);
          if (trace_move_paths) {
            PNR_LOG1(
                "ROUT",
                "routeDesign move docking blocked: net='{}', anchor=({},{}):{}, "
                "target=({},{}), seeds={}, entries={}, busy={}, bridges={}",
                debug_net, anchor.coord.x, anchor.coord.y, anchor.local,
                to.coord.x, to.coord.y, docked_route.target_seed_count,
                docked_route.target_entry_count,
                docked_route.target_busy_count,
                docked_route.blocked_bridges.size());
            for (size_t bridge_index = 0;
                 bridge_index < docked_route.blocked_bridges.size();
                 ++bridge_index) {
              const pnr::DockingBridgeBlocker &bridge =
                  docked_route.blocked_bridges[bridge_index];
              Tile *bridge_tile = fpga::Device::current().getTile(
                  bridge.tile.x, bridge.tile.y);
              Tile *landing_tile = fpga::Device::current().getTile(
                  bridge.landing_tile.x, bridge.landing_tile.y);
              PNR_LOG1(
                  "ROUT",
                  "routeDesign move docking bridge[{}]: net='{}', "
                  "joined={}, edge=({},{}) dst={} '{}' -> src={} '{}' -> "
                  "({},{}) dst={} '{}', busy(dst={},src={},joint={},joint2={}), "
                  "owners(dst='{}',src='{}',joint='{}',joint2='{}',landing='{}')",
                  bridge_index, debug_net, bridge.joins_frontiers,
                  bridge.tile.x, bridge.tile.y, bridge.dst,
                  bridge_tile ? nodeDebugName(*bridge_tile, fpga::CB_NODE_DST,
                                              bridge.dst)
                              : std::string{},
                  bridge.src,
                  bridge_tile ? nodeDebugName(*bridge_tile, fpga::CB_NODE_SRC,
                                              bridge.src)
                              : std::string{},
                  bridge.landing_tile.x, bridge.landing_tile.y,
                  bridge.landing_dst,
                  landing_tile
                      ? nodeDebugName(*landing_tile, fpga::CB_NODE_DST,
                                      bridge.landing_dst)
                      : std::string{},
                  bridge.dst_busy, bridge.src_busy, bridge.joint_busy,
                  bridge.joint2_busy,
                  nodeOwnerForDump(bridge_tile, fpga::CB_NODE_DST, bridge.dst,
                                   false),
                  nodeOwnerForDump(bridge_tile, fpga::CB_NODE_SRC, bridge.src,
                                   false),
                  nodeOwnerForDump(bridge_tile, fpga::CB_NODE_JOINT,
                                   bridge.joint, false),
                  nodeOwnerForDump(bridge_tile, fpga::CB_NODE_JOINT,
                                   bridge.joint2, false),
                  nodeOwnerForDump(landing_tile, fpga::CB_NODE_DST,
                                   bridge.landing_dst, false));
            }
          }
          if (debug_route && debug_route_lines < debug_route_line_limit) {
            auto frontier_text = [](const auto &frontier) {
              std::string text;
              for (const auto &node : frontier) {
                if (!text.empty()) {
                  text += ",";
                }
                text += std::format("({},{}):{}/{}", node.coord.x,
                                    node.coord.y, node.dst, node.depth);
              }
              return text;
            };
            ROUTE_DEBUG_LOG("docking forward frontier: {}",
                            frontier_text(docked_route.forward_frontier));
            ROUTE_DEBUG_LOG("docking backward frontier: {}",
                            frontier_text(docked_route.backward_frontier));
          }
          if (debug_route && current_source && current_net) {
            RouteDesign::RouteTask debug_task{current_source,
                                              &dst_inst,
                                              current_net,
                                              current_source_port,
                                              to_port,
                                              current_route_name,
                                              0,
                                              0,
                                              0,
                                              {},
                                              branch_from_existing};
            logTargetTileEntryTable(debug_task, "docking failed");
            logDockingBackwardState(debug_task, *anchor_tile, anchor.local, to,
                                    pin_nodes, use_iob_docking ? 12 : 5);
          }
          // The two docking searches may be separated by one occupied transit
          // edge. Prefer a proven complete bridge; otherwise remove one exact
          // reverse-frontier blocker and retry docking on the next pass.
          if (allow_docking_terminal_preempt && router && current_net) {
            // Try partial victims before one completed foreign transit route.
            // preemptDockingBridge still protects our own tree and endpoints.
            for (bool allow_complete_victim : {false, true}) {
              for (bool require_joined : {true, false}) {
                for (const pnr::DockingBridgeBlocker &bridge :
                     docked_route.blocked_bridges) {
                  if (bridge.joins_frontiers != require_joined ||
                      !pnr::dockingBoundaryMayPreempt(bridge.joins_frontiers,
                                                      final_step >= 0) ||
                      !preemptDockingBridge(
                          router, bridge, current_net, current_source,
                          current_source_port, current_route_name, stats,
                          allow_complete_victim)) {
                    continue;
                  }
                  docking_bridge_preempted = true;
                  PNR_LOG2(
                      "ROUT",
                      "routeDesign docking bridge preempt: current='{}', "
                      "joined={}, tile=({},{}), dst={}, src={}, joint={}, "
                      "joint2={}, landing=({},{}):{}",
                      debug_net, bridge.joins_frontiers, bridge.tile.x,
                      bridge.tile.y, bridge.dst, bridge.src, bridge.joint,
                      bridge.joint2, bridge.landing_tile.x,
                      bridge.landing_tile.y, bridge.landing_dst);
                  // A complete bridge can be committed from the same snapshot.
                  // A boundary blocker only schedules a precise cut and retry.
                  if (pnr::materializeDockingBridge(bridge, docked_route)) {
                    completed = true;
                    completed_by_docking = true;
                    final_step = docking_anchor_step;
                    ROUTE_DEBUG_LOG(
                        "{} docking joined separated frontiers after exact "
                        "bridge preemption: anchor=({},{}), dst_node={}, "
                        "fragments={}",
                        use_iob_docking ? "iob" : "grounding", anchor.coord.x,
                        anchor.coord.y, anchor.local,
                        docked_route.fragments.size());
                  }
                  break;
                }
                if (docking_bridge_preempted) {
                  break;
                }
              }
              if (docking_bridge_preempted) {
                break;
              }
            }
          }
          int preempted_dst = -1;
          int preempted_joint = -1;
          int preempted_joint2 = -1;
          int preempted_pin = -1;

          if (!completed && !docking_bridge_preempted &&
              allow_docking_terminal_preempt && router && current_net) {
            bool preempt_done = pin_nodes.for_each_set_bit([&](int pin) {
              if (to.isPinNodeLeased(pin)) {
                return false;
              }
              if (!docked_route.blocked_terminal_reachable) {
                return false;
              }
              const pnr::GroundingTerminalPath required{
                  docked_route.blocked_dst, docked_route.blocked_joint,
                  docked_route.blocked_joint2};
              const RouteDesign::RouteTask grounding_task{
                  current_source, &dst_inst, current_net, current_source_port,
                  to_port, current_route_name, 0, 0, 0, {}, false};
              const auto selected_path = router->preemptGroundingTerminal(
                  to, pin, incomingDstMaskForRouteTile(to),
                  packed_reserved_joints, grounding_task, &required);
              if (selected_path.dst < 0) {
                return false;
              }
              const int dst = selected_path.dst;
              const int blocking_joint = selected_path.joint;
              const int blocking_joint2 = selected_path.joint2;
              preempted_dst = dst;
              preempted_joint = blocking_joint;
              preempted_joint2 = blocking_joint2;
              preempted_pin = pin;
              docking_terminal_preempted = true;
              return true;
            });
            docking_terminal_preempted =
                docking_terminal_preempted || preempt_done;
          }
          bool claimed_preempted_terminal = pnr::retryGroundingAfterPreemption(
              docking_terminal_preempted, [&]() {
                docked_route =
                    use_iob_docking
                        ? dockIOB(*anchor_tile, anchor.local, anchor.dst_wire,
                                  to, pin_nodes, trace_docking_paths,
                                  packed_reserved_joints)
                        : dockGrounding(
                              *anchor_tile, anchor.local, anchor.dst_wire, to,
                              pin_nodes, 5, 5, trace_docking_paths,
                              packed_reserved_joints,
                              router
                                  ? &router->backwardDockingIndex(to.coord, 5)
                                  : nullptr);
                trace_docking_backward_attempts(docked_route);
                return docked_route.success;
              });
          if (claimed_preempted_terminal) {
            completed = true;
            completed_by_docking = true;
            final_step = docking_anchor_step;
            ROUTE_DEBUG_LOG("{} docking claimed preempted terminal: "
                            "anchor=({},{}), dst_node={}, fragments={}",
                            use_iob_docking ? "iob" : "grounding",
                            anchor.coord.x, anchor.coord.y, anchor.local,
                            docked_route.fragments.size());
          } else if (docking_terminal_preempted) {
            PNR_LOG1(
                "ROUT",
                "routeDesign grounding claim failed after preempt: "
                "current='{}', tile=({},{}), dst={} leased={} owner='{}' "
                "transit='{}', joint={} leased={} owner='{}', joint2={} "
                "leased={} owner='{}', pin={} leased={} "
                "local_owner='{}', "
                "retry(entries={},busy={},fpush={},bpush={},fpop={},bpop={})",
                debug_net, to.coord.x, to.coord.y, preempted_dst,
                preempted_dst >= 0 &&
                    (to.cb.dst.jump & (NodeMask{0, 1} << preempted_dst)) !=
                        NodeMask{},
                nodeOwnerForDump(&to, fpga::CB_NODE_DST, preempted_dst, false),
                nodeOwnerForDump(&to, fpga::CB_NODE_DST, preempted_dst, true),
                preempted_joint,
                preempted_joint >= 0 &&
                    (to.cb.joint.jump & (NodeMask{0, 1} << preempted_joint)) !=
                        NodeMask{},
                nodeOwnerForDump(&to, fpga::CB_NODE_JOINT, preempted_joint,
                                 false),
                preempted_joint2,
                preempted_joint2 >= 0 &&
                    (to.cb.joint.jump &
                     (NodeMask{0, 1} << preempted_joint2)) != NodeMask{},
                nodeOwnerForDump(&to, fpga::CB_NODE_JOINT,
                                 preempted_joint2, false),
                preempted_pin,
                preempted_pin >= 0 &&
                    (to.cb.local.local & (NodeMask{0, 1} << preempted_pin)) !=
                        NodeMask{},
                nodeOwnerForDump(&to, fpga::CB_NODE_LOCAL, preempted_pin,
                                 false),
                docked_route.target_entry_count, docked_route.target_busy_count,
                docked_route.forward_push_count,
                docked_route.backward_push_count,
                docked_route.forward_pop_count,
                docked_route.backward_pop_count);
          }
        }
      }
    }
  }
  if ((docking_terminal_preempted || docking_bridge_preempted) && !completed) {
    return false;
  }
  if (start_from_dst && !completed && final_step == 0 && docking_attempted) {
    return false;
  }
  if (final_step < 0) {
    if (stats && exhausted_step >= 0) {
      const Step &blocked = steps[exhausted_step];
      stats->exhausted_coord = blocked.coord;
      stats->exhausted_type = blocked.depth == 0 ? fpga::CB_NODE_LOCAL
                                               : fpga::CB_NODE_DST;
      stats->exhausted_node = blocked.local;
      ROUTE_DEBUG_LOG(
          "failed search endpoint: coord=({},{}), depth={}, node={}, "
          "uncommitted=true", blocked.coord.x, blocked.coord.y,
          blocked.depth, blocked.local);
    }
    return false;
  }
  (void)partial_endpoint_can_continue;
  if (complete) {
    *complete = completed;
  }
  if (stats) {
    size_t depth = statDepthBucket(steps[final_step].depth);
    if (completed) {
      ++stats->completed_by_depth[depth];
    } else {
      ++stats->partial_by_depth[depth];
    }
  }

  auto commit_start = profile_route ? std::chrono::steady_clock::now()
                                    : std::chrono::steady_clock::time_point{};
  struct Commit {
    Tile *tile = nullptr;
    CBState cb;
    TilePinState pin_state;
  };
  std::vector<Commit> commits;
  auto snapshot_tile = [&](Tile *tile) {
    for (Commit &commit : commits) {
      if (commit.tile == tile) {
        return;
      }
    }
    commits.push_back(Commit{tile, tile->cb, tile->pin_state});
  };
  size_t final_depth = statDepthBucket(steps[final_step].depth);
  auto rollback = [&]() {
    if (stats) {
      ++stats->commit_rollbacks;
      ++stats->rollbacks_by_depth[final_depth];
    }
    for (Commit &commit : commits) {
      commit.tile->cb = commit.cb;
      commit.tile->pin_state = commit.pin_state;
    }
  };

  std::vector<int> path;
  for (int idx = final_step; idx >= 0; idx = steps[idx].prev) {
    path.push_back(idx);
  }
  std::reverse(path.begin(), path.end());

  const bool validate_route_owners =
      envFlagEnabled("SCALEPNR_VALIDATE_ROUTE_OWNERS");
  auto stale_owner_labels = [](Tile &tile, fpga::CBNodeNameType type,
                               int node) {
    std::ostringstream labels;
    bool first = true;
    for (const fpga::NetRouteRef &owner :
         fpga::findNetOwnersByNode(tile, type, node, false)) {
      if (!owner.net || owner.binding_index >= owner.net->routes.size()) {
        continue;
      }
      const rtl::NetRouteBinding &binding =
          owner.net->routes[owner.binding_index];
      labels << (first ? "" : ", ")
             << (binding.from ? binding.from->makeName(FULL_NAME_LIMIT)
                              : std::string{"<none>"})
             << "/" << binding.from_port << " -> "
             << (binding.to ? binding.to->makeName(FULL_NAME_LIMIT)
                            : std::string{"<none>"})
             << "/" << binding.to_port;
      first = false;
    }
    return labels.str();
  };
  auto assert_free_node_has_no_owner = [&](Tile &tile,
                                           fpga::CBNodeNameType type, int node,
                                           const NodeMask &leased) {
    // Full route-tree ownership scans are an opt-in invariant check, not a
    // routing hot path.
    if (!validate_route_owners) {
      return;
    }
    if (node < 0 || (leased & (NodeMask{0, 1} << node)) != NodeMask{}) {
      return;
    }
    std::string owners = stale_owner_labels(tile, type, node);
    PNR_ASSERT(owners.empty(),
               "free route node still has an owning route: current='{}', "
               "tile=({},{}), type={}, node={}, owners=[{}]",
               debug_net, tile.coord.x, tile.coord.y, static_cast<int>(type),
               node, owners);
  };

  for (size_t i = 1; i < path.size(); ++i) {
    const Step &prev = steps[path[i - 1]];
    const Step &curr = steps[path[i]];
    Tile *prev_tile =
        fpga::Device::current().getTile(prev.coord.x, prev.coord.y);
    if (!prev_tile) {
      ROUTE_DEBUG_LOG(
          "commit rollback: missing prev tile at ({},{}) path_index={}",
          prev.coord.x, prev.coord.y, i);
      rollback();
      return false;
    }
    snapshot_tile(prev_tile);
    assert_free_node_has_no_owner(*prev_tile, fpga::CB_NODE_SRC, curr.jump,
                                  prev_tile->cb.src.jump);
    assert_free_node_has_no_owner(*prev_tile, fpga::CB_NODE_JOINT, curr.joint,
                                  prev_tile->cb.joint.jump);
    assert_free_node_has_no_owner(*prev_tile, fpga::CB_NODE_JOINT, curr.joint2,
                                  prev_tile->cb.joint.jump);
    if (prev.depth != 0 && !(reuses_start_dst && i == 1)) {
      assert_free_node_has_no_owner(*prev_tile, fpga::CB_NODE_DST, prev.local,
                                    prev_tile->cb.dst.jump);
    }
    bool lease_ok = false;
    if (reuses_start_dst && i == 1 && prev.depth != 0) {
      lease_ok = leaseConcreteFork(prev_tile->cb, prev.local, curr.jump,
                                   curr.joint, ignore_deadends, curr.joint2);
    } else {
      lease_ok =
          prev.depth == 0
              ? leaseConcreteOut(prev_tile->cb, prev.local, curr.jump,
                                 curr.joint, ignore_deadends, curr.joint2)
              : leaseConcreteJump(prev_tile->cb, prev.local, curr.jump,
                                  curr.joint, ignore_deadends, curr.joint2);
    }
    if (!lease_ok) {
      ConcreteBusyReason reason =
          prev.depth == 0
              ? concreteOutBusyReason(prev_tile->cb, curr.jump, curr.joint)
              : concreteJumpBusyReason(prev_tile->cb, prev.local, curr.jump,
                                       curr.joint);
      ROUTE_DEBUG_LOG(
          "commit rollback: path lease failed path_index={}, from=({},{}), "
          "depth={}, from_node={} '{}', src={} '{}', reason={}, "
          "branch_existing={}, "
          "cb(src={},dst={},joint={},local={},src_deadend={})",
          i, prev.coord.x, prev.coord.y, prev.depth, prev.local,
          nodeDebugName(*prev_tile,
                        prev.depth == 0 ? fpga::CB_NODE_LOCAL
                                        : fpga::CB_NODE_DST,
                        prev.local),
          curr.jump, nodeDebugName(*prev_tile, fpga::CB_NODE_SRC, curr.jump),
          busyReasonName(reason), reuses_start_dst && i == 1 && prev.depth != 0,
          maskString(prev_tile->cb.src.jump),
          maskString(prev_tile->cb.dst.jump),
          maskString(prev_tile->cb.joint.jump),
          maskString(prev_tile->cb.local.local),
          maskString(prev_tile->cb.src_deadend.jump));
      rollback();
      return false;
    }
  }

  auto lease_docking_fragments = [&]() {
    for (size_t i = 0; i < docked_route.fragments.size(); ++i) {
      Wire &fragment = docked_route.fragments[i];
      Tile *tile =
          fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
      if (!tile) {
        return false;
      }
      snapshot_tile(tile);
      if (fragment.type == Wire::WIRE_TILE_PIN) {
        continue;
      }
      if (fragment.jump >= 0) {
        // Docking is the final local grounding attempt; sticky deadend marks
        // must not block it, only real leased source/destination bits should.
        bool uses_existing_dst = i == 0 && reuses_start_dst && final_step == 0;
        fragment.owns_dst = !uses_existing_dst;
        bool lease_ok =
            uses_existing_dst
                ? leaseConcreteFork(tile->cb, fragment.local, fragment.jump,
                                    fragment.joint, true, fragment.joint2)
                : leaseConcreteJump(tile->cb, fragment.local, fragment.jump,
                                    fragment.joint, true, fragment.joint2);
        if (!lease_ok) {
          ConcreteBusyReason reason = concreteJumpBusyReason(
              tile->cb, fragment.local, fragment.jump, fragment.joint);
          ROUTE_DEBUG_LOG(
              "commit rollback: docking lease failed fragment={}, "
              "from=({},{}), dst={}, src={}, reason={}, start_from_dst={}, "
              "cb(src={},dst={},joint={},local={},src_deadend={})",
              i, fragment.from.x, fragment.from.y, fragment.local,
              fragment.jump, busyReasonName(reason), start_from_dst,
              maskString(tile->cb.src.jump), maskString(tile->cb.dst.jump),
              maskString(tile->cb.joint.jump), maskString(tile->cb.local.local),
              maskString(tile->cb.src_deadend.jump));
          return false;
        }
        continue;
      }
      if (i + 1 >= docked_route.fragments.size() ||
          docked_route.fragments[i + 1].type != Wire::WIRE_TILE_PIN) {
        return false;
      }
      int pin = docked_route.fragments[i + 1].local;
      if (!leaseConcreteIn(tile->cb, fragment.local, pin, fragment.joint,
                           fragment.joint2) ||
          !tile->leasePinNode(pin)) {
        ROUTE_DEBUG_LOG(
            "commit rollback: docking terminal lease failed fragment={}, "
            "tile=({},{}), dst={}, pin={}, joint={}, pin_leased={}, "
            "cb(src={},dst={},joint={},local={})",
            i, fragment.from.x, fragment.from.y, fragment.local, pin,
            fragment.joint, tile->isPinNodeLeased(pin),
            maskString(tile->cb.src.jump), maskString(tile->cb.dst.jump),
            maskString(tile->cb.joint.jump), maskString(tile->cb.local.local));
        return false;
      }
    }
    if (!completed && !docked_route.fragments.empty()) {
      const Wire &last = docked_route.fragments.back();
      if (last.jump < 0 || last.dst < 0) {
        return false;
      }
      Tile *landing =
          fpga::Device::current().getTile(last.to.x, last.to.y);
      if (!landing) {
        return false;
      }
      snapshot_tile(landing);
      NodeMask landing_bit = NodeMask{0, 1} << last.dst;
      if ((landing->cb.dst.jump & landing_bit) != NodeMask{}) {
        return false;
      }
      landing->cb.dst.jump |= landing_bit;
    }
    return true;
  };

  if (completed && !completed_by_docking) {
    Tile *final_tile =
        fpga::Device::current().getTile(target_coord.x, target_coord.y);
    if (!final_tile) {
      rollback();
      return false;
    }
    snapshot_tile(final_tile);
    if (!leaseConcreteTerminal(
            final_tile->cb, steps[final_step].local, final_pin,
            TerminalEntryCandidate{final_entry_kind, final_joint, final_joint2},
            final_allows_existing_dst) ||
        !final_tile->leasePinNode(final_pin)) {
      ROUTE_DEBUG_LOG(
          "commit rollback: final terminal lease failed tile=({},{}), dst={}, "
          "pin={}, joint={}, kind={}, allow_existing_dst={}, pin_leased={}, "
          "cb(src={},dst={},joint={},local={})",
          final_tile->coord.x, final_tile->coord.y, steps[final_step].local,
          final_pin, final_joint, static_cast<int>(final_entry_kind),
          final_allows_existing_dst, final_tile->isPinNodeLeased(final_pin),
          maskString(final_tile->cb.src.jump),
          maskString(final_tile->cb.dst.jump),
          maskString(final_tile->cb.joint.jump),
          maskString(final_tile->cb.local.local));
      rollback();
      return false;
    }
  }
  if (completed_by_docking && !lease_docking_fragments()) {
    rollback();
    return false;
  }
  if (!completed && !completed_by_docking && final_step > 0) {
    Tile *landing_tile = fpga::Device::current().getTile(
        steps[final_step].coord.x, steps[final_step].coord.y);
    if (!landing_tile || steps[final_step].local < 0) {
      rollback();
      return false;
    }
    snapshot_tile(landing_tile);
    NodeMask landing_bit = NodeMask{0, 1} << steps[final_step].local;
    // A partial suffix owns its landing before the next short attempt starts.
    // Routing is single-threaded, so any existing lease makes this commit
    // stale.
    if ((landing_tile->cb.dst.jump & landing_bit) != NodeMask{}) {
      rollback();
      return false;
    }
    landing_tile->cb.dst.jump |= landing_bit;
  }
  if (stats && profile_route) {
    stats->best_first_commit_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - commit_start)
            .count());
  }

  auto materialize_start = profile_route
                               ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
  wire.clear();
  for (size_t i = 1; i < path.size(); ++i) {
    const Step &prev = steps[path[i - 1]];
    const Step &curr = steps[path[i]];
    Wire fragment;
    fragment.from = prev.coord;
    fragment.to = curr.coord;
    fragment.local = prev.local;
    fragment.jump = curr.jump;
    fragment.route_jump = curr.route_jump;
    fragment.dst = curr.local;
    fragment.joint = curr.joint;
    fragment.joint2 = curr.joint2;
    fragment.pos =
        (reuses_start_dst && i == 1 && prev.depth != 0)
            ? ROUTE_POS_FORK
            : (prev.depth == 0 ? ROUTE_POS_SOURCE : ROUTE_POS_TRANSIT);
    fragment.owns_dst = !(reuses_start_dst && i == 1 && prev.depth != 0);
    const Tile *prev_tile =
        fpga::Device::current().getTile(prev.coord.x, prev.coord.y);
    const Tile *curr_tile =
        fpga::Device::current().getTile(curr.coord.x, curr.coord.y);
    if (prev_tile) {
      fpga::CBNodeNameType from_type =
          prev.depth == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
      if (prev.depth > 0) {
        fragment.from_wire_name = prev.dst_wire;
      }
      fragment.src_wire_name = curr.src_wire;
      if (fragment.src_wire_name.empty()) {
        fragment.src_wire_name =
            concreteSrcWireName(*prev_tile, from_type, prev.local, curr.jump,
                                curr.joint, prev.dst_wire);
      }
      if (prev.depth > 0 && fragment.from_wire_name.empty() &&
          !fragment.src_wire_name.empty() && prev_tile->cb_type) {
        fpga::CBNodeNameType to_type =
            curr.joint >= 0 ? fpga::CB_NODE_JOINT : fpga::CB_NODE_SRC;
        int to_node = curr.joint >= 0 ? curr.joint : curr.jump;
        if (const fpga::CBConnName *conn = selectConcreteConn(
                prev_tile->cb_type, fpga::CB_NODE_DST, prev.local, to_type,
                to_node, {},
                curr.joint >= 0 ? std::string{} : fragment.src_wire_name.str())) {
          fragment.from_wire_name = conn->from;
        }
      }
      if (prev.depth > 0 && !prev.dst_wire.empty() &&
          fragment.src_wire_name.empty()) {
        ROUTE_DEBUG_LOG(
            "commit rollback: missing concrete source wire from=({},{}), "
            "from_node={} '{}', incoming_dst='{}', src={} '{}', joint={}",
            prev.coord.x, prev.coord.y, prev.local,
            nodeDebugName(*prev_tile, from_type, prev.local), prev.dst_wire,
            curr.jump, nodeDebugName(*prev_tile, fpga::CB_NODE_SRC, curr.jump),
            curr.joint);
        rollback();
        return false;
      }
    }
    if (!curr.dst_wire.empty()) {
      fragment.dst_wire_name = curr.dst_wire;
    } else if (curr_tile && curr_tile->cb_type) {
      if (const std::string *dst =
              curr_tile->cb_type->nodeName(fpga::CB_NODE_DST, curr.local)) {
        fragment.dst_wire_name = *dst;
      }
    }
    wire.push_back(fragment);
  }
  if (!completed && !completed_by_docking && !wire.empty()) {
    wire.back().owns_landing = true;
  }

  if (completed_by_docking) {
    wire.insert(wire.end(), docked_route.fragments.begin(),
                docked_route.fragments.end());
    if (!completed && !wire.empty()) {
      wire.back().owns_landing = true;
    }
    for (Wire &fragment : wire) {
      if (fragment.type == Wire::WIRE_TILE_PIN &&
          (sameCoord(fragment.to, to.coord) ||
           sameCoord(fragment.to, target_coord))) {
        fillTilePinEndpoint(fragment, dst_inst, to_port, fpga::TILE_PIN_INPUT);
        refreshTilePinEndpoint(fragment, dst_inst, to_port,
                               fpga::TILE_PIN_INPUT);
      }
    }
  } else if (completed) {
    Wire enter;
    enter.from = target_coord;
    enter.to = target_coord;
    enter.local = steps[final_step].local;
    enter.joint = final_joint;
    enter.joint2 = final_joint2;
    enter.pos = 1;
    enter.dst_wire_name = steps[final_step].dst_wire;
    enter.owns_dst = !final_allows_existing_dst;
    wire.push_back(enter);

    Wire pin;
    fillTilePinEndpoint(pin, dst_inst, to_port, fpga::TILE_PIN_INPUT);
    pin.from = target_coord;
    pin.to = target_coord;
    pin.local = final_pin;
    refreshTilePinEndpoint(pin, dst_inst, to_port, fpga::TILE_PIN_INPUT);
    wire.push_back(pin);
  }
  if (stats && profile_route) {
    stats->best_first_materialize_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - materialize_start)
            .count());
  }
  return true;
}
#undef ROUTE_DEBUG_LOG
#undef ROUTE_DEBUG_TEXT

bool isIoBuffer(rtl::Inst &inst) {
  return technology::Tech::current().buffers_ports.find(inst.cell_ref->type) !=
         technology::Tech::current().buffers_ports.end();
}

// Initializes a tile-pin wire with the resource-side identity known from the
// placed instance. The selected local may be refined later, so callers can
// refresh resource_node after choosing it.
void fillTilePinEndpoint(Wire &wire, rtl::Inst &inst, const std::string &port,
                         fpga::TilePinNameType dir) {
  wire.type = Wire::WIRE_TILE_PIN;
  wire.resource = inst.tile.peer ? inst.tile->coord : Coord{};
  wire.pos = inst.pos;
  wire.resource_node =
      (inst.tile.peer && inst.cell_ref.peer)
          ? inst.tile->getResourceNodeNum(inst.cell_ref->type, port, inst.pos,
                                          dir, wire.local)
          : -1;
  wire.pin_dir = dir;
  wire.cell_type = inst.cell_ref.peer ? inst.cell_ref->type : std::string{};
  wire.port = port;
}

// Recomputes the resource endpoint after routing has selected the concrete
// local node. This keeps exported annotations tied to the actual
// local-to-resource connection used.
void refreshTilePinEndpoint(Wire &wire, rtl::Inst &inst,
                            const std::string &port,
                            fpga::TilePinNameType dir) {
  if (!inst.tile.peer || !inst.cell_ref.peer) {
    return;
  }
  wire.resource_node = inst.tile->getResourceNodeNum(inst.cell_ref->type, port,
                                                     inst.pos, dir, wire.local);
}

// Adds the source resource-to-crossbar hop before the first crossbar fragment.
// Routing stores crossbar fragments first, so export needs this explicit
// endpoint fragment.
void prependSourceEndpoint(std::vector<Wire> &route, rtl::Inst &from,
                           const std::string &from_port) {
  if (route.empty() || route.front().type == Wire::WIRE_TILE_PIN ||
      !from.tile.peer) {
    return;
  }

  Wire source;
  fillTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
  source.from = route.front().from;
  source.to = route.front().from;
  source.local = route.front().local;
  refreshTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
  source.net_name = route.front().net_name;
  route.insert(route.begin(), std::move(source));
}

void unplaceInst(rtl::Inst &inst, const char *reason = "unknown") {
  if (!inst.tile.peer) {
    return;
  }
  Tile &tile = *inst.tile;
  const std::string &type = inst.cell_ref->type;
  if (type.find("FD") == 0) {
    tile.regs_cnt = std::max(0, tile.regs_cnt - 1);
  } else if (type.find("LUT") == 0) {
    if (inst.cnt_inputs == 1) {
      tile.luts1cnt = std::max(0, tile.luts1cnt - 1);
    } else if (inst.cnt_inputs == 6) {
      tile.luts6cnt = std::max(0, tile.luts6cnt - 1);
    } else {
      tile.luts5cnt = std::max(0, tile.luts5cnt - 1);
    }
  } else if (type.find("CARRY") == 0) {
    tile.carry = 0;
  } else if (type.find("MUX") == 0) {
    tile.mux = 0;
    tile.luts6cnt = std::max(0, tile.luts6cnt - 2);
  }
  if (std::getenv("SCALEPNR_UNPLACE_TRACE")) {
    PNR_LOG1("ROUT", "unplaceInst reason={} inst='{}' type='{}' old=({},{})/{}",
             reason, inst.makeName(FULL_NAME_LIMIT),
             inst.cell_ref.peer ? inst.cell_ref->type : std::string{},
             tile.coord.x, tile.coord.y, inst.pos);
  }
  inst.tile.clear();
  tile.invalidatePlacementCaches();
}

void restoreInstPlacement(rtl::Inst &inst, Tile &tile, int pos) {
  const std::string &type = inst.cell_ref->type;
  if (type.find("FD") == 0) {
    ++tile.regs_cnt;
  } else if (type.find("LUT") == 0) {
    if (inst.cnt_inputs == 1) {
      ++tile.luts1cnt;
    } else if (inst.cnt_inputs == 6) {
      ++tile.luts6cnt;
    } else {
      ++tile.luts5cnt;
    }
  } else if (type.find("CARRY") == 0) {
    tile.carry = 4;
  } else if (type.find("MUX") == 0) {
    tile.mux = 1;
    tile.luts6cnt += 2;
  }
  tile.assign(&inst);
  inst.coord = tile.coord;
  inst.pos = pos;
  tile.elements_initialized = false;
}

bool isGeneratedPassthroughInst(rtl::Inst *inst) {
  return isMovePassthroughInst(inst);
}

// Build the complete tile-local endpoint ownership set once when Moving
// selects a focus; large deferred queues can then test membership directly.
std::unordered_set<rtl::Inst *> movingFocusEndpointClosure(rtl::Inst *focus) {
  std::unordered_set<rtl::Inst *> closure;
  if (!focus) {
    return closure;
  }
  std::vector<rtl::Inst *> pending = strictMoveCluster(focus);
  if (pending.empty()) {
    pending.push_back(focus);
  }
  fpga::Tile *focus_tile = focus->tile.peer ? &*focus->tile : nullptr;
  while (!pending.empty()) {
    rtl::Inst *current = pending.back();
    pending.pop_back();
    if (!current || !closure.insert(current).second) {
      continue;
    }
    for (rtl::Conn &conn : current->conns) {
      if (!conn.port_ref.peer) {
        continue;
      }
      rtl::Net *net = findNetByDesignator(*current, conn.port_ref->designator);
      if (!net || !net->designatorIsVoid(conn.port_ref->designator)) {
        continue;
      }
      auto append_neighbor = [&](rtl::Inst *neighbor) {
        if (!neighbor || !focus_tile || !neighbor->tile.peer ||
            &*neighbor->tile != focus_tile) {
          return;
        }
        if (!isGeneratedPassthroughInst(current) &&
            !isGeneratedPassthroughInst(neighbor)) {
          return;
        }
        if (!closure.contains(neighbor)) {
          pending.push_back(neighbor);
        }
      };
      if (conn.port_ref->type == rtl::Port::PORT_OUT && !conn.peer) {
        for (auto *sink_ref : rtl::Conn::getSinks(conn)) {
          rtl::Conn *sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
          append_neighbor(sink ? sink->inst_ref.peer : nullptr);
        }
      } else if (conn.port_ref->type == rtl::Port::PORT_IN) {
        rtl::Conn *driver = conn.follow();
        append_neighbor(driver ? driver->inst_ref.peer : nullptr);
      }
    }
  }
  return closure;
}

std::vector<rtl::Net *> incidentRouteNets(
    const std::unordered_set<rtl::Inst *> &endpoints,
    rtl::Net *extra_net = nullptr) {
  // Collect only nets named by the endpoints' numeric connection designators.
  // This avoids a module-wide binding scan for every Moving focus.
  std::vector<rtl::Net *> nets;
  std::unordered_set<rtl::Net *> seen;
  auto append = [&](rtl::Net *net) {
    if (net && seen.insert(net).second) {
      nets.push_back(net);
    }
  };
  append(extra_net);
  for (rtl::Inst *endpoint : endpoints) {
    if (!endpoint) {
      continue;
    }
    for (rtl::Conn &conn : endpoint->conns) {
      if (conn.port_ref.peer) {
        append(findNetByDesignator(*endpoint, conn.port_ref->designator));
      }
    }
  }
  return nets;
}

void appendUniquePassthroughNeighbor(std::vector<rtl::Inst *> &neighbors,
                                     rtl::Inst *inst) {
  if (!isGeneratedPassthroughInst(inst)) {
    return;
  }
  if (std::find(neighbors.begin(), neighbors.end(), inst) == neighbors.end()) {
    neighbors.push_back(inst);
  }
}

std::vector<rtl::Inst *> generatedPassthroughNeighbors(rtl::Inst &inst) {
  // Moving a real element invalidates generated tile-local passthroughs
  // attached to it.
  std::vector<rtl::Inst *> neighbors;
  for (rtl::Conn &conn : inst.conns) {
    if (!conn.port_ref.peer) {
      continue;
    }
    rtl::Net *net = findNetByDesignator(inst, conn.port_ref->designator);
    if (!net || !net->designatorIsVoid(conn.port_ref->designator)) {
      continue;
    }
    if (conn.port_ref->type == rtl::Port::PORT_OUT && !conn.peer) {
      for (auto *sink_ref : rtl::Conn::getSinks(conn)) {
        rtl::Conn *sink_conn =
            sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        appendUniquePassthroughNeighbor(
            neighbors, sink_conn ? sink_conn->inst_ref.peer : nullptr);
      }
    } else if (conn.port_ref->type == rtl::Port::PORT_IN) {
      rtl::Conn *driver = conn.follow();
      appendUniquePassthroughNeighbor(neighbors,
                                      driver ? driver->inst_ref.peer : nullptr);
    }
  }
  return neighbors;
}

// Resolve an unplaced generated endpoint to the real packed element whose
// placement Moving must change before the endpoint can be rehomed.
rtl::Inst *movingPlacementTarget(rtl::Inst *inst) {
  if (!inst || inst->tile.peer || !isGeneratedPassthroughInst(inst)) {
    return inst;
  }
  for (rtl::Conn &conn : inst->conns) {
    if (!conn.port_ref.peer) {
      continue;
    }
    if (conn.port_ref->type == rtl::Port::PORT_OUT && !conn.peer) {
      for (auto *sink_ref : rtl::Conn::getSinks(conn)) {
        rtl::Conn *sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (sink && sink->inst_ref.peer && sink->inst_ref->tile.peer) {
          return sink->inst_ref.peer;
        }
      }
    } else {
      rtl::Conn *driver = conn.follow();
      if (driver && driver->inst_ref.peer && driver->inst_ref->tile.peer) {
        return driver->inst_ref.peer;
      }
    }
  }
  return inst;
}

rtl::Module *parentModule(rtl::Inst &inst) {
  if (!inst.cell_ref.peer || !inst.cell_ref->module_ref.peer) {
    return nullptr;
  }
  return inst.cell_ref->module_ref->parent_ref.peer;
}

bool allIncidentRoutesComplete(rtl::Inst &inst) {
  if (!parentModule(inst)) {
    return true;
  }
  // Resolve generated tile-local endpoints once; repeating that traversal for
  // every binding makes completion audits quadratic on large modules.
  std::unordered_set<rtl::Inst *> focus_endpoints =
      movingFocusEndpointClosure(&inst);
  for (rtl::Net *net : incidentRouteNets(focus_endpoints)) {
    if (!net || net->void_net ||
        (!net->routeCanBePreempted() && !net->distributed_source)) {
      continue;
    }
    for (const rtl::NetRouteBinding &binding : net->routes) {
      if (netBindingIsVoid(*net, binding)) {
        continue;
      }
      if (!pnr::routeBindingTouchesKnownEndpoint(
              binding, [&](rtl::Inst *endpoint) {
                return focus_endpoints.contains(endpoint);
              })) {
        continue;
      }
      if (!binding.owner ||
          binding.route_index >= binding.owner->wires.size()) {
        return false;
      }
      if (!routeIsComplete(binding.owner->wires[binding.route_index])) {
        return false;
      }
    }
  }
  return true;
}

size_t collectIncompleteIncidentRouteTasks(
    rtl::Inst &inst, std::vector<RouteDesign::RouteTask> &tasks) {
  if (!parentModule(inst)) {
    return 0;
  }
  size_t added = 0;
  // Reuse one complete endpoint closure while collecting all incident work;
  // no binding may trigger another module-wide passthrough lookup.
  std::unordered_set<rtl::Inst *> focus_endpoints =
      movingFocusEndpointClosure(&inst);
  for (rtl::Net *net_ptr : incidentRouteNets(focus_endpoints)) {
    if (!net_ptr) {
      continue;
    }
    rtl::Net &net = *net_ptr;
    if (net.void_net || net.distributed_source ||
        !net.routeCanBePreempted()) {
      continue;
    }
    for (size_t binding_index = 0; binding_index < net.routes.size();) {
      const rtl::NetRouteBinding &binding = net.routes[binding_index];
      if (netBindingIsVoid(net, binding)) {
        ++binding_index;
        continue;
      }
      if (!pnr::routeBindingTouchesKnownEndpoint(
              binding, [&](rtl::Inst *endpoint) {
                return focus_endpoints.contains(endpoint);
              })) {
        ++binding_index;
        continue;
      }
      bool has_route =
          binding.owner && binding.route_index < binding.owner->wires.size();
      bool complete =
          has_route &&
          routeIsComplete(binding.owner->wires[binding.route_index]);
      bool has_complete_twin = false;
      if (!has_route) {
        for (size_t twin_index = 0; twin_index < net.routes.size();
             ++twin_index) {
          if (twin_index == binding_index) {
            continue;
          }
          const rtl::NetRouteBinding &twin = net.routes[twin_index];
          if (twin.route_name != binding.route_name ||
              twin.from != binding.from || twin.to != binding.to ||
              twin.from_port != binding.from_port ||
              twin.to_port != binding.to_port || !twin.owner ||
              twin.route_index >= twin.owner->wires.size() ||
              !routeIsComplete(twin.owner->wires[twin.route_index])) {
            continue;
          }
          has_complete_twin = true;
          break;
        }
      }
      if (pnr::discardOwnerlessDuplicateBinding(has_route, has_complete_twin)) {
        PNR_LOG1("ROUT",
                 "routeDesign moving: discarded ownerless duplicate binding "
                 "net='{}' route='{}' from='{}'/'{}' to='{}'/'{}'",
                 net.makeName(FULL_NAME_LIMIT), binding.route_name,
                 binding.from ? binding.from->makeName(FULL_NAME_LIMIT)
                              : std::string{},
                 binding.from_port,
                 binding.to ? binding.to->makeName(FULL_NAME_LIMIT)
                            : std::string{},
                 binding.to_port);
        net.eraseRouteBinding(binding_index);
        continue;
      }
      if (!pnr::incidentBindingNeedsRouting(false, has_route, complete)) {
        ++binding_index;
        continue;
      }
      bool fanout = !net.distributed_source && binding.from &&
                    pnr::incidentBindingIsFanout(hasOtherCompleteSourceBinding(
                        *binding.from, binding.from_port, &binding));
      RouteDesign::RouteTask task{binding.from,
                                  binding.to,
                                  &net,
                                  binding.from_port,
                                  binding.to_port,
                                  binding.route_name,
                                  0,
                                  0,
                                  0,
                                  {},
                                  fanout};
      if (appendUniqueRouteTask(tasks, task)) {
        ++added;
      }
      ++binding_index;
    }
  }
  return added;
}

// Count incomplete physical bindings without changing ownership or queues.
// This separates scheduler task counts from the route state they represent.
size_t countIncompleteRouteBindings(const rtl::Design &design) {
  size_t count = 0;
  for (const auto &module_ref : design.modules) {
    for (const auto &net_ref : module_ref.nets) {
      if (net_ref.void_net || net_ref.distributed_source ||
          !net_ref.routeCanBePreempted()) {
        continue;
      }
      for (const rtl::NetRouteBinding &binding : net_ref.routes) {
        if (netBindingIsVoid(net_ref, binding)) {
          continue;
        }
        if (!binding.owner ||
            binding.route_index >= binding.owner->wires.size() ||
            !routeIsComplete(binding.owner->wires[binding.route_index])) {
          ++count;
        }
      }
    }
  }
  return count;
}

// Audit every physical binding when all stage queues become empty. Moving can
// invalidate a non-focused sibling after its task was removed from the queue.
size_t collectIncompleteRouteTasks(rtl::Design &design,
                                   std::vector<RouteDesign::RouteTask> &tasks) {
  size_t added = 0;
  for (auto &module_ref : design.modules) {
    for (auto &net_ref : module_ref.nets) {
      rtl::Net &net = net_ref;
      if (net.void_net || net.distributed_source ||
          !net.routeCanBePreempted()) {
        continue;
      }
      for (size_t binding_index = 0; binding_index < net.routes.size();) {
        const rtl::NetRouteBinding &binding = net.routes[binding_index];
        if (netBindingIsVoid(net, binding)) {
          ++binding_index;
          continue;
        }
        bool has_route =
            binding.owner && binding.route_index < binding.owner->wires.size();
        bool complete =
            has_route &&
            routeIsComplete(binding.owner->wires[binding.route_index]);
        bool has_complete_twin = false;
        if (!has_route) {
          for (size_t twin_index = 0; twin_index < net.routes.size();
               ++twin_index) {
            if (twin_index == binding_index) {
              continue;
            }
            const rtl::NetRouteBinding &twin = net.routes[twin_index];
            if (twin.route_name != binding.route_name ||
                twin.from != binding.from || twin.to != binding.to ||
                twin.from_port != binding.from_port ||
                twin.to_port != binding.to_port || !twin.owner ||
                twin.route_index >= twin.owner->wires.size() ||
                !routeIsComplete(twin.owner->wires[twin.route_index])) {
              continue;
            }
            has_complete_twin = true;
            break;
          }
        }
        if (pnr::discardOwnerlessDuplicateBinding(has_route,
                                                  has_complete_twin)) {
          net.eraseRouteBinding(binding_index);
          continue;
        }
        if (!pnr::incidentBindingNeedsRouting(false, has_route, complete)) {
          ++binding_index;
          continue;
        }
        bool fanout =
            !net.distributed_source && binding.from &&
            pnr::incidentBindingIsFanout(hasOtherCompleteSourceBinding(
                *binding.from, binding.from_port, &binding));
        RouteDesign::RouteTask task{binding.from,
                                    binding.to,
                                    &net,
                                    binding.from_port,
                                    binding.to_port,
                                    binding.route_name,
                                    0,
                                    0,
                                    0,
                                    {},
                                    fanout};
        if (appendUniqueRouteTask(tasks, task)) {
          ++added;
        }
        ++binding_index;
      }
    }
  }
  return added;
}

const rtl::NetRouteBinding *firstSourceBinding(const rtl::Inst &inst,
                                               rtl::Net **out_net,
                                               bool *out_complete) {
  if (out_net) {
    *out_net = nullptr;
  }
  if (out_complete) {
    *out_complete = false;
  }
  rtl::Module *module = parentModule(const_cast<rtl::Inst &>(inst));
  if (!module) {
    return nullptr;
  }
  // Moving relocates sinks only; a source binding means this instance owns
  // driver takeoff.
  for (auto &net_ref : module->nets) {
    rtl::Net &net = net_ref;
    for (const rtl::NetRouteBinding &binding : net.routes) {
      if (binding.from != &inst) {
        continue;
      }
      if (out_net) {
        *out_net = &net;
      }
      if (out_complete && binding.owner &&
          binding.route_index < binding.owner->wires.size()) {
        *out_complete =
            routeIsComplete(binding.owner->wires[binding.route_index]);
      }
      return &binding;
    }
  }
  return nullptr;
}

const rtl::NetRouteBinding *firstCompleteSourceBinding(const rtl::Inst &inst,
                                                       const std::string &port,
                                                       rtl::Net **out_net) {
  if (out_net) {
    *out_net = nullptr;
  }
  rtl::Module *module = parentModule(const_cast<rtl::Inst &>(inst));
  if (!module) {
    return nullptr;
  }
  // A fanout branch is legal only after Generic created one complete route from
  // this physical source pin.
  for (auto &net_ref : module->nets) {
    rtl::Net &net = net_ref;
    for (const rtl::NetRouteBinding &binding : net.routes) {
      if (!samePhysicalSource(binding.from, binding.from_port, &inst, port) ||
          !binding.owner ||
          binding.route_index >= binding.owner->wires.size()) {
        continue;
      }
      if (!routeIsComplete(binding.owner->wires[binding.route_index])) {
        continue;
      }
      if (out_net) {
        *out_net = &net;
      }
      return &binding;
    }
  }
  return nullptr;
}

bool bindingHasRoutedSourceExit(const rtl::NetRouteBinding &binding) {
  if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
    return false;
  }
  const std::vector<Wire> &route = binding.owner->wires[binding.route_index];
  return routeIsComplete(route) && routeCrossbarFragments(&route) != 0;
}

bool hasCompleteSourceExitBinding(rtl::Inst &inst, const std::string &port) {
  rtl::Module *module = parentModule(inst);
  if (!module) {
    return false;
  }
  // Fanout mode must branch from an already routed takeoff, not from the source
  // tile.
  for (auto &net_ref : module->nets) {
    rtl::Net &net = net_ref;
    for (const rtl::NetRouteBinding &binding : net.routes) {
      if (samePhysicalSource(binding.from, binding.from_port, &inst, port) &&
          bindingHasRoutedSourceExit(binding)) {
        return true;
      }
    }
  }
  return false;
}

struct SourceRouteBinding {
  rtl::Net *net = nullptr;
  rtl::NetRouteBinding *binding = nullptr;
};

std::vector<SourceRouteBinding>
completeSourceBindings(rtl::Inst &inst, const std::string &port) {
  std::vector<SourceRouteBinding> result;
  rtl::Module *module = parentModule(inst);
  if (!module) {
    return result;
  }
  // Fanout routes branch from the physical source pin tree, even when RTL split
  // it into several Net objects.
  for (auto &net_ref : module->nets) {
    rtl::Net &net = net_ref;
    for (rtl::NetRouteBinding &binding : net.routes) {
      if (!samePhysicalSource(binding.from, binding.from_port, &inst, port) ||
          !bindingHasRoutedSourceExit(binding)) {
        continue;
      }
      result.push_back(SourceRouteBinding{&net, &binding});
    }
  }
  return result;
}

bool passthroughInputSourceEndpoint(rtl::Inst &inst, rtl::Inst *&source_inst,
                                    std::string &source_port) {
  // A generated source passthrough is fed by the real source pin through a void
  // net.
  if (!inst.cell_ref.peer) {
    return false;
  }
  auto kind = inst.cell_ref->attributes.find("scalepnr_passthrough");
  if (kind == inst.cell_ref->attributes.end() || kind->second != "source") {
    return false;
  }
  for (rtl::Conn &conn : inst.conns) {
    if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
      continue;
    }
    rtl::Conn *driver = conn.follow();
    if (!driver || !driver->inst_ref.peer || !driver->port_ref.peer) {
      continue;
    }
    source_inst = driver->inst_ref.peer;
    source_port = driver->port_ref->makeName();
    return true;
  }
  return false;
}

std::vector<SourceRouteBinding>
completeFanoutSeedBindings(rtl::Inst &inst, const std::string &port,
                           rtl::Inst **seed_inst = nullptr,
                           std::string *seed_port = nullptr) {
  // Fanout mode may branch from the routed trunk feeding a generated source
  // passthrough.
  std::vector<SourceRouteBinding> bindings = completeSourceBindings(inst, port);
  if (!bindings.empty()) {
    if (seed_inst) {
      *seed_inst = bindings.front().binding->from;
    }
    if (seed_port) {
      *seed_port = bindings.front().binding->from_port;
    }
    return bindings;
  }
  rtl::Inst *upstream_inst = nullptr;
  std::string upstream_port;
  if (!passthroughInputSourceEndpoint(inst, upstream_inst, upstream_port) ||
      !upstream_inst) {
    return bindings;
  }
  bindings = completeSourceBindings(*upstream_inst, upstream_port);
  if (!bindings.empty()) {
    if (seed_inst) {
      *seed_inst = upstream_inst;
    }
    if (seed_port) {
      *seed_port = upstream_port;
    }
    return bindings;
  }

  rtl::Module *module = parentModule(inst);
  if (!module) {
    return bindings;
  }
  for (auto &net_ref : module->nets) {
    rtl::Net &net = net_ref;
    for (rtl::NetRouteBinding &binding : net.routes) {
      if (!binding.from || binding.from == &inst || binding.from_port != port ||
          !bindingHasRoutedSourceExit(binding)) {
        continue;
      }
      rtl::Inst *sibling_upstream = nullptr;
      std::string sibling_port;
      if (!passthroughInputSourceEndpoint(*binding.from, sibling_upstream,
                                          sibling_port) ||
          sibling_upstream != upstream_inst || sibling_port != upstream_port) {
        continue;
      }
      bindings.push_back(SourceRouteBinding{&net, &binding});
    }
  }
  if (!bindings.empty()) {
    if (seed_inst) {
      *seed_inst =
          bindings.front().binding ? bindings.front().binding->from : nullptr;
    }
    if (seed_port) {
      *seed_port = bindings.front().binding
                       ? bindings.front().binding->from_port
                       : std::string{};
    }
  }
  return bindings;
}

bool hasOtherCompleteSourceBinding(rtl::Inst &inst, const std::string &port,
                                   const rtl::NetRouteBinding *current) {
  // A route cannot serve as its own fanout seed while Moving detaches its sink.
  for (const SourceRouteBinding &source :
       completeFanoutSeedBindings(inst, port)) {
    if (source.binding && source.binding != current) {
      return true;
    }
  }
  return false;
}

bool hasCompleteFanoutSeedBinding(rtl::Inst &inst, const std::string &port) {
  return !completeFanoutSeedBindings(inst, port).empty();
}

std::vector<SourceRouteBinding> completeIndexedFanoutSeedBindings(
    RouteDesign &router, rtl::Inst &inst, const std::string &port,
    rtl::Inst **seed_inst = nullptr, std::string *seed_port = nullptr,
    size_t max_results = 0) {
  std::vector<SourceRouteBinding> result;
  auto indexed = router.source_route_nets.find(sourceRouteKey(&inst, port));
  if (indexed != router.source_route_nets.end()) {
    for (rtl::Net *net : indexed->second) {
      if (!net) {
        continue;
      }
      for (rtl::NetRouteBinding &binding : net->routes) {
        if (samePhysicalSource(binding.from, binding.from_port, &inst, port) &&
            bindingHasRoutedSourceExit(binding)) {
          result.push_back(SourceRouteBinding{net, &binding});
          if (max_results != 0 && result.size() >= max_results) {
            break;
          }
        }
      }
      if (max_results != 0 && result.size() >= max_results) {
        break;
      }
    }
  }
  if (!result.empty()) {
    if (seed_inst) {
      *seed_inst = result.front().binding->from;
    }
    if (seed_port) {
      *seed_port = result.front().binding->from_port;
    }
    return result;
  }
  if (indexed != router.source_route_nets.end()) {
    return result;
  }
  // Legacy bindings without an index entry still receive the exhaustive lookup.
  return completeFanoutSeedBindings(inst, port, seed_inst, seed_port);
}

bool hasOtherIncompleteSourceBinding(rtl::Inst &inst, const std::string &port,
                                     const rtl::NetRouteBinding *current) {
  rtl::Module *module = parentModule(inst);
  if (!module) {
    return false;
  }
  for (auto &net_ref : module->nets) {
    for (rtl::NetRouteBinding &binding : net_ref.routes) {
      if (!samePhysicalSource(binding.from, binding.from_port, &inst, port)) {
        continue;
      }
      std::vector<Wire> *route = routeBindingRoute(binding);
      if (pnr::incompleteBindingBlocksMovingSeed(
              &binding == current, route && !route->empty(),
              route && routeIsComplete(*route))) {
        return true;
      }
    }
  }
  return false;
}

bool attachSharedDestinationLocalFanout(
    RouteDesign::RouteTask &task,
    const RouteDesign::FanoutBranchIndex &branch_index,
    const std::vector<Tile *> &to_route_tiles) {
  // Complete a fanout at an endpoint already reached by the same source tree.
  // Reuse either its exact local or its terminal dst/joints and lease only a
  // new local.
  if (!task.net || !task.to || !task.to->tile.peer) {
    return false;
  }
  for (Tile *to_route_tile : to_route_tiles) {
    if (!to_route_tile) {
      continue;
    }
    NodeMask pin_nodes =
        routeTileInputNodes(*to_route_tile, *task.to, task.to_port);
    if (pin_nodes == NodeMask{}) {
      continue;
    }
    auto endpoints = branch_index.endpoints_by_tile.find(to_route_tile);
    if (endpoints == branch_index.endpoints_by_tile.end()) {
      continue;
    }
    for (const RouteDesign::FanoutEndpointIndexEntry &endpoint :
         endpoints->second) {
      if (!endpoint.owner ||
          endpoint.route_index >= endpoint.owner->wires.size()) {
        continue;
      }
      const std::vector<Wire> &base = endpoint.owner->wires[endpoint.route_index];
      if (endpoint.fragment_index >= base.size()) {
        continue;
      }
      const size_t fragment_index = endpoint.fragment_index;
      const Wire &fragment = base[fragment_index];
      if (fragment.type != Wire::WIRE_TILE_PIN ||
          !sameCoord(fragment.from, to_route_tile->coord) ||
          !sameCoord(fragment.to, to_route_tile->coord)) {
        continue;
      }
      int selected_pin = -1;
      bool maps_to_target =
          (pin_nodes & (NodeMask{0, 1} << fragment.local)) != NodeMask{};
      bool has_foreign_owner = false;
      if (maps_to_target) {
        std::vector<NetRouteRef> owners = findNetOwnersByNode(
            *to_route_tile, fpga::CB_NODE_LOCAL, fragment.local, false);
        has_foreign_owner = std::any_of(
            owners.begin(), owners.end(), [&](const NetRouteRef &owner) {
              if (!owner.net || owner.binding_index >= owner.net->routes.size()) {
                return false;
              }
              const rtl::NetRouteBinding &binding =
                  owner.net->routes[owner.binding_index];
              return !samePhysicalSource(binding.from, binding.from_port,
                                         task.from, task.from_port);
            });
      }
      bool reuse_exact_local =
          pnr::fanoutMayReuseExactLocal(maps_to_target, has_foreign_owner);
      if (reuse_exact_local) {
        selected_pin = fragment.local;
      } else if (fragment_index > 0 && to_route_tile->cb_type) {
        const Wire &terminal = base[fragment_index - 1];
        bool terminal_on_target =
            terminal.type == Wire::WIRE_CROSSBAR && terminal.jump < 0 &&
            terminal.local >= 0 &&
            sameCoord(terminal.from, to_route_tile->coord) &&
            sameCoord(terminal.to, to_route_tile->coord);
        if (terminal_on_target) {
          pin_nodes.for_each_set_bit([&](int pin) {
            NodeMask pin_bit = NodeMask{0, 1} << pin;
            if (to_route_tile->isPinNodeLeased(pin) ||
                (to_route_tile->cb.local.local & pin_bit) != NodeMask{}) {
              return false;
            }
            std::vector<TerminalEntryCandidate> entries =
                targetEntryCandidates(*to_route_tile->cb_type, terminal.local,
                                      pin);
            bool shares_terminal =
                std::any_of(entries.begin(), entries.end(),
                            [&](const TerminalEntryCandidate &entry) {
                              return entry.kind == TerminalEntryKind::dst &&
                                     entry.joint == terminal.joint &&
                                     entry.joint2 == terminal.joint2;
                            });
            if (shares_terminal) {
              selected_pin = pin;
              return true;
            }
            return false;
          });
        }
      }
      if (selected_pin < 0) {
        continue;
      }
      std::vector<Wire> shared_route(
          base.begin(),
          base.begin() + static_cast<std::ptrdiff_t>(fragment_index));
      if (routeCrossbarFragments(&shared_route) == 0) {
        continue;
      }
      for (Wire &shared_fragment : shared_route) {
        shared_fragment.shared = true;
        shared_fragment.net_name = task.net_name;
      }
      if (!reuse_exact_local) {
        if (!to_route_tile->leasePinNode(selected_pin)) {
          continue;
        }
        to_route_tile->cb.local.local |= NodeMask{0, 1} << selected_pin;
        // This branch shares the terminal dst/joints and must not release
        // them.
        shared_route.back().owns_dst = false;
      }
      Wire pin;
      fillTilePinEndpoint(pin, *task.to, task.to_port, fpga::TILE_PIN_INPUT);
      pin.from = to_route_tile->coord;
      pin.to = to_route_tile->coord;
      pin.local = selected_pin;
      pin.net_name = task.net_name;
      pin.shared = reuse_exact_local;
      refreshTilePinEndpoint(pin, *task.to, task.to_port,
                             fpga::TILE_PIN_INPUT);
      shared_route.push_back(std::move(pin));
      task.to->wires.emplace_back(std::move(shared_route));
      size_t binding_index = fpga::attachNetRoute(
          *task.net, *task.to, task.to->wires.size() - 1, task.from, task.to,
          task.from_port, task.to_port, task.net_name);
      fpga::registerNetRouteTiles(*task.net, task.to->wires.back(),
                                  binding_index);
      PNR_LOG3("ROUT",
               "routeFanoutTask shared terminal reuse: net='{}', "
               "tile=({},{}), local={}, exact_local={}",
               task.net_name, to_route_tile->coord.x, to_route_tile->coord.y,
               selected_pin, reuse_exact_local);
      return true;
    }
  }
  return false;
}

int countAvailableForkExits(Tile &tile, int dst_node,
                            const std::string &incoming_wire) {
  if (!tile.cb_type || dst_node < 0) {
    return 0;
  }
  const std::vector<uint16_t> *src_nodes =
      tile.cb_type->srcNodes(fpga::CB_NODE_DST, dst_node);
  if (!src_nodes) {
    return 0;
  }
  int count = 0;
  for (uint16_t src_node : *src_nodes) {
    int joint = -1;
    int joint2 = -1;
    if (!tile.cb_type->canJump(dst_node, src_node, src_node, joint, &joint2)) {
      continue;
    }
    CBState test_state = tile.cb;
    // Branch selection runs only in Fanout mode, where Basic deadends are
    // stale.
    if (leaseConcreteFork(test_state, dst_node, src_node, joint, true,
                          joint2)) {
      ++count;
    }
  }
  return count;
}

struct FanoutBranchStart {
  Tile *tile = nullptr;
  int dst_node = -1;
  std::string dst_wire;
};

std::vector<FanoutBranchStart> fanoutBranchStarts(rtl::Inst &inst,
                                                  const std::string &port) {
  // Reuse concrete destination nodes from an existing source tree for fanout
  // trial routing.
  std::vector<FanoutBranchStart> branches;
  std::vector<FanoutBranchStart> fallbacks;
  for (const SourceRouteBinding &source_binding :
       completeFanoutSeedBindings(inst, port)) {
    if (!source_binding.binding || !source_binding.binding->owner ||
        source_binding.binding->route_index >=
            source_binding.binding->owner->wires.size()) {
      continue;
    }
    const std::vector<Wire> &route =
        source_binding.binding->owner
            ->wires[source_binding.binding->route_index];
    for (const Wire &fragment : route) {
      if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0) {
        continue;
      }
      Tile *from_tile =
          fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
      if (!from_tile || !from_tile->cb_type) {
        continue;
      }
      fpga::TileJumpTarget target;
      if (fragment.dst >= 0) {
        target.tile =
            fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
        target.dst_node = fragment.dst;
        target.dst_wire = fragment.dst_wire_name;
      } else {
        int route_jump =
            fragment.route_jump >= 0 ? fragment.route_jump : fragment.jump;
        target = fpga::Device::current().resolveJump(*from_tile, route_jump);
      }
      if (!target.tile || !target.tile->cb_type || target.dst_node < 0) {
        continue;
      }
      FanoutBranchStart candidate{target.tile, target.dst_node,
                                  target.dst_wire};
      int free_exits = countAvailableForkExits(*target.tile, target.dst_node,
                                               target.dst_wire);
      if (!pnr::fanoutBranchIsUsableFallback(free_exits)) {
        continue;
      }
      bool duplicate =
          std::any_of(branches.begin(), branches.end(),
                      [&](const FanoutBranchStart &branch) {
                        return branch.tile == target.tile &&
                               branch.dst_node == target.dst_node &&
                               branch.dst_wire == target.dst_wire;
                      });
      if (pnr::fanoutBranchIsPreferred(free_exits) && !duplicate) {
        branches.push_back(candidate);
      }
      bool duplicate_fallback =
          std::any_of(fallbacks.begin(), fallbacks.end(),
                      [&](const FanoutBranchStart &branch) {
                        return branch.tile == target.tile &&
                               branch.dst_node == target.dst_node &&
                               branch.dst_wire == target.dst_wire;
                      });
      if (!duplicate_fallback) {
        fallbacks.push_back(std::move(candidate));
      }
    }
  }
  if (branches.empty()) {
    branches = std::move(fallbacks);
  }
  return branches;
}

bool netHasCompleteRouteExcept(rtl::Net &net, size_t skip_index) {
  for (size_t index = 0; index < net.routes.size(); ++index) {
    if (index == skip_index) {
      continue;
    }
    rtl::NetRouteBinding &binding = net.routes[index];
    if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
      continue;
    }
    if (routeIsComplete(binding.owner->wires[binding.route_index])) {
      return true;
    }
  }
  return false;
}

uint64_t placementKey(const Coord &coord, int pos) {
  return pnr::movingPlacementKey(coord.x, coord.y, pos);
}

bool placementWasTried(const std::vector<uint64_t> &tried, const Coord &coord,
                       int pos) {
  uint64_t key = placementKey(coord, pos);
  return std::find(tried.begin(), tried.end(), key) != tried.end();
}

int iterationLimitFromCells(int cells) { return std::max(16, cells / 10); }

int routeDepthLimitForAttempt(size_t attempt, int max_limit) {
  return std::min(max_limit, 5 + static_cast<int>(attempt));
}

int moveAttemptLimitFromCells(int cells) { return std::max(16, cells / 10); }

int countReachableCells(rtl::Inst &inst, RegBunch *bunch, uint64_t mark) {
  if (inst.mark == mark) {
    return 0;
  }
  inst.mark = mark;

  int cells = inst.cell_ref.peer && inst.cell_ref->module_ref.peer &&
                      inst.cell_ref->module_ref->is_blackbox
                  ? 1
                  : 0;

  for (auto &conn : inst.conns) {
    if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
      continue;
    }
    rtl::Conn *driver_conn = conn.follow();
    if (!driver_conn || !driver_conn->inst_ref.peer) {
      continue;
    }
    cells += countReachableCells(*driver_conn->inst_ref, nullptr, mark);
  }

  if (bunch) {
    for (auto &subbunch : bunch->sub_bunches) {
      if (subbunch.reg) {
        cells += countReachableCells(*subbunch.reg, &subbunch, mark);
      }
    }
  }

  return cells;
}

int countDesignCells(std::list<Referable<RegBunch>> &bunch_list) {
  uint64_t mark = rtl::Inst::genMark();
  int cells = 0;
  for (auto &bunch : bunch_list) {
    if (bunch.reg) {
      cells += countReachableCells(*bunch.reg, &bunch, mark);
    }
  }
  return cells;
}

void resetRoutingState() {
  for (auto &tile : fpga::Device::current().tile_grid) {
    tile.cb = {};
    tile.cb.type = tile.cb_type;
    tile.pin_state = {};
    tile.clearRoutedNets();
  }
}

Wire makeEndpointWire(rtl::Inst &from, const std::string &from_port,
                      rtl::Inst &to, const std::string &to_port) {
  Wire wire;
  wire.port = to_port.empty() ? from_port : to_port;

  if (isIoBuffer(from) && from.tile.peer) {
    fillTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
    wire.from = from.tile->coord;
    wire.to = from.tile->coord;
    wire.local =
        from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos)
            .firstSetBit();
    if (wire.local < 0) {
      wire.local = from.pos;
    }
    refreshTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
    wire.pos = from.pos;
    wire.port = from_port;
    return wire;
  }

  if (isIoBuffer(to) && to.tile.peer) {
    fillTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
    wire.from = to.tile->coord;
    wire.to = to.tile->coord;
    wire.local =
        to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos).firstSetBit();
    if (wire.local < 0) {
      wire.local = to.pos;
    }
    refreshTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
    wire.pos = to.pos;
    wire.port = to_port;
    return wire;
  }

  if (to.tile.peer) {
    fillTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
    wire.from = to.tile->coord;
    wire.to = to.tile->coord;
    wire.local =
        to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos).firstSetBit();
    refreshTilePinEndpoint(wire, to, to_port, fpga::TILE_PIN_INPUT);
    wire.pos = to.pos;
    return wire;
  }

  if (from.tile.peer) {
    fillTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
    wire.from = from.tile->coord;
    wire.to = from.tile->coord;
    wire.local =
        from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos)
            .firstSetBit();
    refreshTilePinEndpoint(wire, from, from_port, fpga::TILE_PIN_OUTPUT);
    wire.pos = from.pos;
    return wire;
  }

  wire.from = Coord{-1, -1};
  wire.to = Coord{-1, -1};
  return wire;
}

bool anyRoutableOutputCandidate(const std::vector<Tile *> &route_tiles,
                                rtl::Inst &inst, const std::string &port,
                                NodeMask output_nodes) {
  for (Tile *tile : route_tiles) {
    if (!tile) {
      continue;
    }
    NodeMask route_nodes = routeTileEndpointNodes(*tile, inst, port, true);
    bool same_coord = inst.tile.peer && inst.tile->coord.x == tile->coord.x &&
                      inst.tile->coord.y == tile->coord.y;
    route_nodes = pnr::mappedOutputCandidateNodes(
        route_nodes, output_nodes, same_coord,
        supportsOutputLocalNodes(*tile, output_nodes));
    bool has_routable = route_nodes.for_each_set_bit(
        [&](int local) { return isRoutableOutputLocal(*tile, local); });
    if (has_routable) {
      return true;
    }
  }
  return false;
}

bool anyFreeRoutableOutputCandidate(const std::vector<Tile *> &route_tiles,
                                    rtl::Inst &inst,
                                    const std::string &port,
                                    NodeMask output_nodes, rtl::Net *net) {
  for (Tile *tile : route_tiles) {
    if (!tile) {
      continue;
    }
    NodeMask route_nodes = routeTileEndpointNodes(*tile, inst, port, true);
    bool same_coord = inst.tile.peer && inst.tile->coord.x == tile->coord.x &&
                      inst.tile->coord.y == tile->coord.y;
    route_nodes = pnr::mappedOutputCandidateNodes(
        route_nodes, output_nodes, same_coord,
        supportsOutputLocalNodes(*tile, output_nodes));
    bool has_free_takeoff = route_nodes.for_each_set_bit([&](int local) {
      return !sourceLocalOwnedByDifferentEndpoint(*tile, local, inst, port,
                                                  net) &&
             tile->cb.hasFreeOut(local);
    });
    if (has_free_takeoff) {
      return true;
    }
  }
  return false;
}

bool tryDirectResourceRoute(rtl::Inst &from, const std::string &from_port,
                            rtl::Inst &to, const std::string &to_port,
                            std::vector<Wire> &wire, rtl::Net *net) {
  if (!from.tile.peer || !to.tile.peer || from.tile.peer != to.tile.peer) {
    return false;
  }

  // Only an explicitly tile-internal endpoint may omit crossbar routing.
  // Ordinary same-tile nets must lease a real path through the routing graph.
  if (!net || !netEndpointIsVoid(*net, &to, to_port)) {
    return false;
  }

  NodeMask output_nodes =
      from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
  NodeMask input_nodes =
      to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos);
  if (output_nodes == NodeMask{}) {
    return false;
  }

  int output_node = output_nodes.firstSetBit();
  if (sourceLocalOwnedByDifferentEndpoint(*from.tile, output_node, from,
                                          from_port, net)) {
    return false;
  }
  int input_node = -1;
  if (input_nodes == NodeMask{} && from.tile.peer == to.tile.peer) {
    // Same-tile packed shapes may expose only the driven output node in the
    // tile map.
    input_nodes = NodeMask{0, 1} << output_node;
  }
  if (input_nodes == NodeMask{}) {
    return false;
  }
  bool input_ok = input_nodes.for_each_set_bit([&](int local) {
    if (to.tile->isPinNodeLeased(local)) {
      return false;
    }
    input_node = local;
    return true;
  });
  if (!input_ok || input_node < 0 || !to.tile->leasePinNode(input_node)) {
    return false;
  }

  wire.clear();
  Wire source;
  fillTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
  source.from = from.tile->coord;
  source.to = from.tile->coord;
  source.local = output_node;
  refreshTilePinEndpoint(source, from, from_port, fpga::TILE_PIN_OUTPUT);
  source.net_name = net ? net->makeName(FULL_NAME_LIMIT) : std::string{};
  wire.push_back(source);

  Wire sink;
  fillTilePinEndpoint(sink, to, to_port, fpga::TILE_PIN_INPUT);
  sink.from = to.tile->coord;
  sink.to = to.tile->coord;
  sink.local = input_node;
  refreshTilePinEndpoint(sink, to, to_port, fpga::TILE_PIN_INPUT);
  sink.net_name = source.net_name;
  wire.push_back(sink);
  return true;
}

bool partialRouteEndpoint(const std::vector<Wire> &route, Tile *&tile,
                          int &local, std::string &dst_wire) {
  tile = nullptr;
  local = -1;
  dst_wire.clear();
  for (auto it = route.rbegin(); it != route.rend(); ++it) {
    const Wire &fragment = *it;
    if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0) {
      continue;
    }
    const Tile *from_tile =
        fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
    if (!from_tile) {
      return false;
    }
    if (fragment.dst >= 0) {
      Tile *target_tile =
          fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
      if (!target_tile) {
        return false;
      }
      tile = target_tile;
      local = fragment.dst;
      dst_wire = fragment.dst_wire_name;
      return true;
    }
    int route_jump =
        fragment.route_jump >= 0 ? fragment.route_jump : fragment.jump;
    fpga::TileJumpTarget target =
        fpga::Device::current().resolveJump(*from_tile, route_jump);
    if (!target.tile || target.dst_node < 0) {
      return false;
    }
    tile = target.tile;
    local = target.dst_node;
    dst_wire = target.dst_wire;
    return true;
  }
  return false;
}

void releaseRouteFragments(std::vector<Wire> &route) {
  std::vector<Wire> removed = route;
  route.clear();
  fpga::releaseRouteLeases(removed);
}

} // namespace

// Atomically lease an already-materialized route for backward source recovery
// or forward destination relocation, reporting the failed fragment if requested.
bool pnr::commitPreparedRoute(std::vector<Wire> &route,
                              size_t *failed_fragment) {
  struct Snapshot {
    Tile *tile = nullptr;
    CBState cb;
    TilePinState pin;
  };
  std::vector<Snapshot> snapshots;
  auto snapshot = [&](Tile *tile) {
    if (!tile || std::any_of(snapshots.begin(), snapshots.end(),
                             [&](const Snapshot &old) {
                               return old.tile == tile;
                             })) {
      return;
    }
    snapshots.push_back({tile, tile->cb, tile->pin_state});
  };
  auto rollback = [&]() {
    for (Snapshot &old : snapshots) {
      old.tile->cb = old.cb;
      old.tile->pin_state = old.pin;
    }
  };

  for (size_t index = 0; index < route.size(); ++index) {
    if (failed_fragment) {
      *failed_fragment = index;
    }
    Wire &fragment = route[index];
    Tile *tile = fpga::Device::current().getTile(fragment.from.x,
                                                  fragment.from.y);
    if (!tile) {
      rollback();
      return false;
    }
    snapshot(tile);
    if (fragment.type == Wire::WIRE_TILE_PIN) {
      continue;
    }
    if (fragment.jump >= 0) {
      bool leased =
          fragment.pos == ROUTE_POS_SOURCE
              ? leaseConcreteOut(tile->cb, fragment.local, fragment.jump,
                                 fragment.joint, true, fragment.joint2)
          : fragment.pos == ROUTE_POS_FORK
              ? leaseConcreteFork(tile->cb, fragment.local, fragment.jump,
                                  fragment.joint, true, fragment.joint2)
              : leaseConcreteJump(tile->cb, fragment.local, fragment.jump,
                                  fragment.joint, true, fragment.joint2);
      if (!leased) {
        rollback();
        return false;
      }
      continue;
    }
    if (index + 1 >= route.size() ||
        route[index + 1].type != Wire::WIRE_TILE_PIN) {
      rollback();
      return false;
    }
    int pin = route[index + 1].local;
    if (!leaseConcreteTerminal(tile->cb, fragment.local, pin,
            TerminalEntryCandidate{TerminalEntryKind::dst, fragment.joint,
                                   fragment.joint2},
            fragment.pos == ROUTE_POS_FORK && !fragment.owns_dst) ||
        !tile->leasePinNode(pin)) {
      rollback();
      return false;
    }
  }
  return true;
}

namespace {

bool discardRouteBranchSuffix(std::vector<Wire> &route) {
  if (route.empty()) {
    return false;
  }
  size_t branch_start = 0;
  while (branch_start < route.size() && route[branch_start].shared) {
    ++branch_start;
  }
  if (branch_start == route.size()) {
    return false;
  }
  std::vector<Wire> removed = route;
  route.resize(branch_start);
  removed.erase(removed.begin(),
                removed.begin() + static_cast<std::ptrdiff_t>(branch_start));
  fpga::releaseRouteLeases(removed);
  return true;
}

bool ripLastRouteStep(std::vector<Wire> &route,
                      RouteDesign::RouteStats *stats = nullptr) {
  if (stats) {
    ++stats->backstep_attempts;
  }
  std::vector<Wire> old_route = route;
  size_t removed_count = 0;
  while (!route.empty() && route.back().type == Wire::WIRE_TILE_PIN) {
    route.pop_back();
    ++removed_count;
  }
  if (route.empty()) {
    fpga::releaseRouteLeases(old_route);
    if (stats) {
      stats->backstep_fragments += removed_count;
    }
    return false;
  }
  if (route.back().shared) {
    old_route.erase(old_route.begin(),
                    old_route.begin() +
                        static_cast<std::ptrdiff_t>(route.size()));
    fpga::releaseRouteLeases(old_route);
    if (stats) {
      stats->backstep_fragments += removed_count;
    }
    return false;
  }
  size_t depth = 0;
  for (const Wire &fragment : route) {
    if (fragment.type == Wire::WIRE_CROSSBAR) {
      ++depth;
    }
  }
  route.pop_back();
  ++removed_count;
  old_route.erase(old_route.begin(),
                  old_route.begin() +
                      static_cast<std::ptrdiff_t>(route.size()));
  fpga::releaseRouteLeases(old_route);
  if (stats) {
    ++stats->backstep_success;
    stats->backstep_fragments += removed_count;
    ++stats->backsteps_by_depth[statDepthBucket(static_cast<int>(depth))];
  }
  return true;
}

uint64_t routeEndpointKey(const Coord &coord, int dst) {
  uint64_t x = static_cast<uint16_t>(coord.x);
  uint64_t y = static_cast<uint16_t>(coord.y);
  uint64_t node = static_cast<uint16_t>(dst);
  return x | (y << 16) | (node << 42);
}

bool continuationEndsAtExistingEndpoint(const std::vector<Wire> &route,
                                        const std::vector<Wire> &continuation) {
  if (continuation.empty()) {
    return false;
  }
  const Wire *last = nullptr;
  for (auto it = continuation.rbegin(); it != continuation.rend(); ++it) {
    if (it->type == Wire::WIRE_CROSSBAR && it->dst >= 0) {
      last = &*it;
      break;
    }
  }
  if (!last) {
    return false;
  }
  uint64_t endpoint = routeEndpointKey(last->to, last->dst);
  for (const Wire &fragment : route) {
    if (fragment.type == Wire::WIRE_CROSSBAR && fragment.dst >= 0 &&
        routeEndpointKey(fragment.to, fragment.dst) == endpoint) {
      return true;
    }
  }
  return false;
}

bool continuePartialRoute(
    std::vector<Wire> &route, rtl::Inst &to, const std::string &to_port,
    int iteration_limit, bool &complete,
    RouteDesign::RouteStats *stats = nullptr, RouteDesign *router = nullptr,
    rtl::Net *current_net = nullptr, rtl::Inst *current_source = nullptr,
    const std::string &current_source_port = std::string{},
    const std::string &current_route_name = std::string{},
    bool *root_blocked = nullptr,
    bool allow_transit_preempt = false,
    bool allow_docking_terminal_preempt = false) {
  complete = false;
  if (root_blocked) {
    *root_blocked = false;
  }
  Tile *from_tile = nullptr;
  int from_pos = -1;
  std::string from_dst_wire;
  if (!partialRouteEndpoint(route, from_tile, from_pos, from_dst_wire) ||
      !from_tile) {
    return false;
  }
  bool debug_continuation =
      routeDebugMatches("SCALEPNR_DEBUG_FANOUT_NET", current_route_name) ||
      routeDebugMatches("SCALEPNR_DEBUG_ROUTE_NET", current_route_name);
  if (debug_continuation) {
    PNR_LOG1("ROUT",
             "continuePartialRoute debug start: net='{}', tail_tile='{}' "
             "coord=({},{}) tail_node={} '{}' tail_wire='{}', route_size={}, "
             "to='{}'/'{}'",
             current_route_name, from_tile->makeName(), from_tile->coord.x,
             from_tile->coord.y, from_pos,
             nodeDebugName(*from_tile, fpga::CB_NODE_DST, from_pos),
             from_dst_wire, route.size(), to.makeName(FULL_NAME_LIMIT),
             to_port);
  }
  Coord preferred_target{};
  const Coord *preferred_target_ptr = nullptr;
  if (current_source && current_source->tile.peer) {
    preferred_target = current_source->tile->coord;
    preferred_target_ptr = &preferred_target;
  }
  std::vector<Tile *> to_route_tiles =
      routeTileCandidates(to, to_port, false, preferred_target_ptr);
  bool any_root_blocked = false;
  auto try_continuation_to_target = [&](size_t target_index,
                                        std::vector<Wire> &continuation,
                                        bool &attempt_complete,
                                        RouteSearchReport &report) {
    Tile *to_route_tile = to_route_tiles[target_index];
    NodeMask pin_nodes = to_route_tile
                             ? routeTileInputNodes(*to_route_tile, to, to_port)
                             : NodeMask{};
    bool result =
        to_route_tile &&
        tryBestFirstRoute(
            *from_tile, *to_route_tile, from_pos, to, to_port, continuation,
            iteration_limit, true, from_dst_wire, &attempt_complete, stats,
            router, current_net, true, current_source, current_source_port,
            current_route_name, pin_nodes, true, false, &report,
            allow_transit_preempt, allow_docking_terminal_preempt);
    if (debug_continuation) {
      PNR_LOG1("ROUT",
               "continuePartialRoute debug result: net='{}', target_index={}, "
               "to_tile='{}' coord=({},{}) pin_nodes={}, result={}, "
               "complete={}, continuation_size={}, root_blocked={}",
               current_route_name, target_index,
               to_route_tile ? to_route_tile->makeName() : std::string{},
               to_route_tile ? to_route_tile->coord.x : -1,
               to_route_tile ? to_route_tile->coord.y : -1, pin_nodes.str(),
               result, attempt_complete, continuation.size(),
               report.firstNodeBlocked());
    }
    return result;
  };

  for (size_t target_index = 0; target_index < to_route_tiles.size();
       ++target_index) {
    std::vector<Wire> continuation;
    bool attempt_complete = false;
    RouteSearchReport report;
    if (try_continuation_to_target(target_index, continuation, attempt_complete,
                                   report)) {
      if (attempt_complete) {
        route.insert(route.end(), continuation.begin(), continuation.end());
        complete = true;
        if (root_blocked) {
          *root_blocked = any_root_blocked || report.firstNodeBlocked();
        }
        return true;
      }
      route.insert(route.end(), continuation.begin(), continuation.end());
      complete = false;
      if (root_blocked) {
        *root_blocked = any_root_blocked || report.firstNodeBlocked();
      }
      return true;
    }
    if (report.firstNodeBlocked()) {
      any_root_blocked = true;
    }
  }

  if (root_blocked) {
    *root_blocked = any_root_blocked;
  }
  return false;
}

void collectInstsWithRoutes(rtl::Inst &inst, std::vector<rtl::Inst *> &insts) {
  insts.push_back(&inst);
  for (auto &sub_inst : inst.insts) {
    collectInstsWithRoutes(sub_inst, insts);
  }
}

bool sameSourceEndpoint(const Wire &existing, rtl::Inst &from,
                        const std::string &from_port, int local) {
  if (!from.tile.peer || !from.cell_ref.peer) {
    return false;
  }

  int resource_node = from.tile->getResourceNodeNum(
      from.cell_ref->type, from_port, from.pos, fpga::TILE_PIN_OUTPUT, local);
  bool resource_node_matches = existing.resource_node < 0 ||
                               resource_node < 0 ||
                               existing.resource_node == resource_node;
  return existing.pin_dir == fpga::TILE_PIN_OUTPUT && existing.local == local &&
         existing.resource.x == from.tile->coord.x &&
         existing.resource.y == from.tile->coord.y &&
         existing.pos == from.pos &&
         existing.cell_type == from.cell_ref->type &&
         existing.port == from_port && resource_node_matches;
}

bool sourceLocalOwnedByDifferentEndpoint(Tile &route_tile, int local,
                                         rtl::Inst &from,
                                         const std::string &from_port,
                                         rtl::Net *net) {
  if (local < 0 || !from.tile.peer || !from.cell_ref.peer) {
    return false;
  }

  // A clear numeric local lease proves that no routed endpoint owns this node.
  // Avoid walking every route binding for the common first-use case.
  if ((route_tile.cb.local.local & (NodeMask{0, 1} << local)) == NodeMask{}) {
    return false;
  }

  if (std::getenv("SCALEPNR_ENDPOINT_DEBUG")) {
    NodeMask source_nodes =
        from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
    PNR_LOG1("ROUT",
             "source conflict check net='{}' route_tile='{}' cb='{}' local={} "
             "source='{}' type='{}' port='{}' pos={} endpoint_nodes={}",
             net ? net->makeName(FULL_NAME_LIMIT) : std::string{},
             route_tile.makeName(),
             route_tile.cb_type ? route_tile.cb_type->name : std::string{},
             local, from.makeName(), from.cell_ref->type, from_port, from.pos,
             source_nodes.str());
  }

  // Source endpoint conflicts are tile-local, so inspect only nets registered
  // on this tile.
  for (const Ref<rtl::Net> &routed_net_ref : route_tile.routedNets) {
    rtl::Net *routed_net = routed_net_ref.peer;
    if (!routed_net) {
      continue;
    }
    for (rtl::NetRouteBinding &binding : routed_net->routes) {
      if (!binding.owner ||
          binding.route_index >= binding.owner->wires.size()) {
        continue;
      }
      const std::vector<Wire> &route =
          binding.owner->wires[binding.route_index];
      if (route.empty()) {
        continue;
      }
      const Wire &existing = route.front();
      if (existing.type != Wire::WIRE_TILE_PIN ||
          existing.pin_dir != fpga::TILE_PIN_OUTPUT ||
          existing.local != local || existing.from.x != route_tile.coord.x ||
          existing.from.y != route_tile.coord.y) {
        continue;
      }
      if (sameSourceEndpoint(existing, from, from_port, local)) {
        continue;
      }

      static int source_conflict_logs = 0;
      if (std::getenv("SCALEPNR_ENDPOINT_DEBUG") && source_conflict_logs < 64) {
        ++source_conflict_logs;
        PNR_LOG1(
            "ROUT",
            "source local conflict: net='{}', tile='{}' ({},{}), local={}, "
            "new={}/{} pos={} resource=({},{}), existing_net='{}', "
            "existing={}/{} pos={} resource=({},{}), owner='{}'",
            net ? net->makeName(FULL_NAME_LIMIT) : std::string{},
            route_tile.makeName(), route_tile.coord.x, route_tile.coord.y,
            local, from.cell_ref->type, from_port, from.pos, from.tile->coord.x,
            from.tile->coord.y, existing.net_name, existing.cell_type,
            existing.port, existing.pos, existing.resource.x,
            existing.resource.y, binding.owner->makeName());
      }
      return true;
    }
  }
  return false;
}

std::string routeTreeDebugFilter() {
  const char *filter = std::getenv("SCALEPNR_ROUTE_TREE_DEBUG_NET");
  return filter ? std::string(filter) : std::string{};
}

bool routeTreeDebugMatches(const rtl::Net &net, const std::string &filter) {
  if (filter.empty()) {
    return true;
  }
  if (net.name.find(filter) != std::string::npos) {
    return true;
  }
  for (const rtl::NetRouteBinding &binding : net.routes) {
    if (binding.route_name.find(filter) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string routeNodeId(const Coord &coord, const std::string &wire_name,
                        const char *fallback_type, int fallback_node) {
  if (!wire_name.empty()) {
    return std::format("({},{})/{}", coord.x, coord.y, wire_name);
  }
  return std::format("({},{})/{}{}", coord.x, coord.y, fallback_type,
                     fallback_node);
}

void appendRouteFragmentNodes(std::vector<std::string> &nodes,
                              const Wire &fragment) {
  auto push_unique = [&](std::string node) {
    if (node.empty()) {
      return;
    }
    if (nodes.empty() || nodes.back() != node) {
      nodes.push_back(std::move(node));
    }
  };

  if (fragment.type == Wire::WIRE_TILE_PIN) {
    push_unique(routeNodeId(fragment.from, fragment.src_wire_name, "L",
                            fragment.local));
    return;
  }
  if (fragment.type != Wire::WIRE_CROSSBAR) {
    return;
  }
  push_unique(routeNodeId(fragment.from, fragment.from_wire_name,
                          fragment.pos == ROUTE_POS_SOURCE ? "L" : "D",
                          fragment.local));
  push_unique(
      routeNodeId(fragment.from, fragment.src_wire_name, "S", fragment.jump));
  push_unique(routeNodeId(fragment.to, fragment.dst_wire_name, "D",
                          fragment.dst >= 0 ? fragment.dst : fragment.local));
}

std::vector<std::string> routeTreeNodes(const std::vector<Wire> &route) {
  std::vector<std::string> nodes;

  for (const Wire &fragment : route) {
    appendRouteFragmentNodes(nodes, fragment);
  }
  return nodes;
}

std::vector<Wire> *routeBindingRoute(rtl::NetRouteBinding &binding) {
  if (!binding.owner || binding.route_index >= binding.owner->wires.size()) {
    return nullptr;
  }
  return &binding.owner->wires[binding.route_index];
}

bool routeHasSharedPrefix(rtl::NetRouteBinding &binding) {
  std::vector<Wire> *route = routeBindingRoute(binding);
  return route && !route->empty() && route->front().shared;
}

bool routeStartsWithSharedPrefix(const std::vector<Wire> *route) {
  return route && !route->empty() && route->front().shared;
}

bool routeHasSharedBranchSuffix(const std::vector<Wire> *route) {
  if (!routeStartsWithSharedPrefix(route)) {
    return false;
  }
  for (const Wire &fragment : *route) {
    if (!fragment.shared) {
      return true;
    }
  }
  return false;
}

std::vector<std::string>
routeSharedPrefixNodes(const std::vector<Wire> &route) {
  std::vector<std::string> nodes;
  for (const Wire &fragment : route) {
    if (!fragment.shared) {
      break;
    }
    appendRouteFragmentNodes(nodes, fragment);
  }
  return nodes;
}

std::vector<std::string> routeOwnedLeaseNodes(const std::vector<Wire> &route) {
  std::vector<std::string> nodes;
  auto append = [&](const Coord &coord, const char *type, int node) {
    if (node >= 0) {
      nodes.push_back(routeNodeId(coord, {}, type, node));
    }
  };
  for (size_t index = 0; index < route.size(); ++index) {
    const Wire &fragment = route[index];
    if (fragment.shared) {
      continue;
    }
    if (fragment.type == Wire::WIRE_TILE_PIN) {
      if (index + 1 == route.size()) {
        append(fragment.from, "L", fragment.local);
      }
      continue;
    }
    if (fragment.type != Wire::WIRE_CROSSBAR) {
      continue;
    }
    append(fragment.from, "S", fragment.jump);
    append(fragment.from, "J", fragment.joint);
    append(fragment.from, "J", fragment.joint2);
    if (fragment.pos == ROUTE_POS_SOURCE) {
      append(fragment.from, "L", fragment.local);
    } else if (fragment.owns_dst) {
      append(fragment.from, "D", fragment.local);
    }
  }
  return nodes;
}

std::vector<std::string> routeSharedLeaseNodes(const std::vector<Wire> &route) {
  std::vector<std::string> nodes;
  auto append = [&](const Coord &coord, const char *type, int node) {
    if (node >= 0) {
      nodes.push_back(routeNodeId(coord, {}, type, node));
    }
  };
  for (const Wire &fragment : route) {
    if (!fragment.shared) {
      continue;
    }
    if (fragment.type != Wire::WIRE_CROSSBAR) {
      continue;
    }
    append(fragment.from, "S", fragment.jump);
    append(fragment.from, "J", fragment.joint);
    append(fragment.from, "J", fragment.joint2);
    if (fragment.pos == ROUTE_POS_SOURCE) {
      append(fragment.from, "L", fragment.local);
    } else {
      // A shared replica does not own this destination, but another route in
      // the same physical source tree must own it.
      append(fragment.from, "D", fragment.local);
    }
  }
  return nodes;
}

bool routeHasSharedFragments(const std::vector<Wire> *route) {
  return route && std::any_of(route->begin(), route->end(),
                              [](const Wire &wire) { return wire.shared; });
}

bool rotateFanoutBranch(RouteDesign::RouteTask &task,
                        std::vector<Wire> *existing_route) {
  if (!task.fanout || !existing_route || existing_route->empty()) {
    return false;
  }

  // Remove both the private suffix and its shared-prefix replica. The shared
  // fragments do not own leases, but retaining them causes duplicate rotation.
  bool removed = false;
  if (task.net && task.to) {
    size_t owner_route_index = findRouteIndex(*task.to, existing_route);
    size_t binding_index =
        owner_route_index != std::numeric_limits<size_t>::max()
            ? findNetRouteBindingIndex(*task.net, *task.to, owner_route_index)
            : std::numeric_limits<size_t>::max();
    removed = binding_index != std::numeric_limits<size_t>::max() &&
              fpga::discardNetBranch(*task.net, binding_index);
  }
  if (!removed) {
    discardRouteBranchSuffix(*existing_route);
    existing_route->clear();
    removed = true;
  }
  if (removed) {
    pnr::consumeFanoutBranch(task.attempt, task.fanout_branch_offset,
                             task.fanout_branch_attempt);
  }
  return removed;
}

constexpr unsigned fanout_branch_retry_limit = 4;

size_t repairStaleSharedRoutePrefixesImpl(
    rtl::Design &design, std::vector<RouteDesign::RouteTask> &fanout_tasks) {
  std::unordered_map<std::string, std::unordered_set<std::string>>
      owned_nodes_by_source;
  std::unordered_map<std::string, std::unordered_set<std::string>>
      sources_by_owned_node;
  std::unordered_map<std::string, std::string> source_labels;
  std::unordered_set<std::string> protected_sources;
  std::unordered_set<std::string> stale_sources;
  for (auto &module : design.modules) {
    for (auto &net : module.nets) {
      for (rtl::NetRouteBinding &binding : net.routes) {
        std::vector<Wire> *route = routeBindingRoute(binding);
        // An incomplete route can still own a valid source prefix while
        // Moving reroutes its suffix; include that owner for sibling checks.
        if (!route || route->empty() || !binding.from) {
          continue;
        }
        std::string source_key =
            sourceRouteKey(binding.from, binding.from_port);
        if (!net.routeCanBePreempted()) {
          protected_sources.insert(source_key);
        }
        // Route names remain stable even when tests intentionally rebuild
        // endpoint storage.
        source_labels[source_key] =
            binding.route_name + " (" + binding.from_port + ")";
        auto &owned = owned_nodes_by_source[source_key];
        for (const std::string &node : routeOwnedLeaseNodes(*route)) {
          owned.insert(node);
          sources_by_owned_node[node].insert(source_key);
        }
      }
    }
  }

  // Recover ownership metadata damaged inside one physical source tree. A
  // shared prefix may become its owner's only survivor after branch edits; it
  // can take ownership only when no different source owns any prefix lease.
  for (auto &module : design.modules) {
    for (auto &net : module.nets) {
      if (!net.routeCanBePreempted()) {
        continue;
      }
      for (rtl::NetRouteBinding &binding : net.routes) {
        std::vector<Wire> *route = routeBindingRoute(binding);
        if (!routeHasSharedFragments(route) || !binding.from) {
          continue;
        }
        const std::string source_key =
            sourceRouteKey(binding.from, binding.from_port);
        std::vector<std::string> prefix_nodes =
            routeSharedLeaseNodes(*route);
        auto owned_it = owned_nodes_by_source.find(source_key);
        bool missing_owner = false;
        bool foreign_owner = false;
        for (const std::string &node : prefix_nodes) {
          if (owned_it == owned_nodes_by_source.end() ||
              !owned_it->second.contains(node)) {
            missing_owner = true;
          }
          auto source_it = sources_by_owned_node.find(node);
          if (source_it != sources_by_owned_node.end()) {
            for (const std::string &owner_source : source_it->second) {
              foreign_owner |= owner_source != source_key;
            }
          }
        }
        if (!missing_owner || foreign_owner) {
          continue;
        }
        for (Wire &fragment : *route) {
          if (!fragment.shared) {
            continue;
          }
          fragment.shared = false;
          if (fragment.type == Wire::WIRE_CROSSBAR &&
              fragment.pos != ROUTE_POS_SOURCE) {
            fragment.owns_dst = true;
          }
        }
        auto &owned = owned_nodes_by_source[source_key];
        for (const std::string &node : routeOwnedLeaseNodes(*route)) {
          owned.insert(node);
          sources_by_owned_node[node].insert(source_key);
        }
      }
    }
  }

  // A physical route node cannot be privately owned by independent source
  // trees. Repair every conflicting tree so normal lease checks choose again.
  size_t ownership_conflicts = 0;
  for (const auto &[node, sources] : sources_by_owned_node) {
    if (sources.size() < 2) {
      continue;
    }
    for (const std::string &source : sources) {
      if (!protected_sources.contains(source)) {
        stale_sources.insert(source);
      }
    }
    if (++ownership_conflicts <= 32) {
      std::ostringstream source_list;
      bool first = true;
      for (const std::string &source : sources) {
        auto label = source_labels.find(source);
        source_list << (first ? "" : ", ")
                    << (label == source_labels.end() ? source : label->second);
        first = false;
      }
      PNR_LOG1(
          "ROUT",
          "routeDesign cross-source ownership repair: node='{}', sources=[{}]",
          node, source_list.str());
    }
  }

  for (auto &module : design.modules) {
    for (auto &net : module.nets) {
      if (!net.routeCanBePreempted()) {
        continue;
      }
      for (size_t binding_index = 0; binding_index < net.routes.size();
           ++binding_index) {
        rtl::NetRouteBinding &binding = net.routes[binding_index];
        std::vector<Wire> *route = routeBindingRoute(binding);
        if (!route || route->empty() || !routeHasSharedFragments(route) ||
            !routeIsComplete(*route) || !binding.from || !binding.to) {
          continue;
        }

        std::vector<std::string> prefix_nodes =
            routeSharedLeaseNodes(*route);
        if (prefix_nodes.empty()) {
          continue;
        }
        std::string source_key =
            sourceRouteKey(binding.from, binding.from_port);
        auto owned_it = owned_nodes_by_source.find(source_key);
        std::string missing_node;
        for (const std::string &node : prefix_nodes) {
          if (owned_it == owned_nodes_by_source.end() ||
              !owned_it->second.contains(node)) {
            missing_node = node;
            break;
          }
        }
        if (missing_node.empty()) {
          continue;
        }
        stale_sources.insert(source_key);
        if (stale_sources.size() <= 32) {
          PNR_LOG1("ROUT",
                   "routeDesign shared-prefix repair: net='{}', "
                   "route_name='{}', binding={}, missing='{}', source='{}'",
                   net.makeName(FULL_NAME_LIMIT), binding.route_name,
                   binding_index, missing_node, source_key);
        }
      }
    }
  }

  size_t repaired = 0;
  for (const std::string &source_key : stale_sources) {
    bool has_generic = false;
    for (auto &module : design.modules) {
      for (auto &net : module.nets) {
        if (!net.routeCanBePreempted()) {
          continue;
        }
        std::vector<size_t> route_indices;
        std::vector<RouteDesign::RouteTask> source_tasks;
        for (size_t binding_index = 0; binding_index < net.routes.size();
             ++binding_index) {
          rtl::NetRouteBinding &binding = net.routes[binding_index];
          if (!binding.from || !binding.to || binding.route_name.empty() ||
              sourceRouteKey(binding.from, binding.from_port) != source_key) {
            continue;
          }
          std::vector<Wire> *route = routeBindingRoute(binding);
          if (!route || route->empty()) {
            continue;
          }
          route_indices.push_back(binding_index);
          source_tasks.push_back(RouteDesign::RouteTask{binding.from,
                                                        binding.to,
                                                        &net,
                                                        binding.from_port,
                                                        binding.to_port,
                                                        binding.route_name,
                                                        0,
                                                        0,
                                                        0,
                                                        {},
                                                        has_generic});
          has_generic = true;
        }
        if (route_indices.empty() ||
            !fpga::unrouteNetRouteTree(net, route_indices)) {
          continue;
        }
        // Repaired endpoint bindings must not keep aliases to the same
        // cleared route slot; each reroute allocates independent storage.
        for (size_t route_index : route_indices) {
          rtl::NetRouteBinding &binding = net.routes[route_index];
          binding.owner = nullptr;
          binding.route_index = std::numeric_limits<size_t>::max();
        }
        repaired += route_indices.size();
        for (RouteDesign::RouteTask &task : source_tasks) {
          appendUniqueRouteTask(fanout_tasks, task);
        }
      }
    }
  }
  if (repaired != 0) {
    PNR_LOG1(
        "ROUT",
        "routeDesign shared-prefix repair: sources={}, repaired={}, tasks={}",
        stale_sources.size(), repaired, fanout_tasks.size());
  }
  return repaired;
}

std::string routeTreeOwnerList(const std::vector<size_t> &owners) {
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < owners.size(); ++i) {
    if (i) {
      out << ',';
    }
    out << owners[i];
  }
  out << ']';
  return out.str();
}

void logMalformedRouteTrees(rtl::Design &design) {
  const std::string filter = routeTreeDebugFilter();
  size_t malformed = 0;
  for (auto &module : design.modules) {
    for (auto &net : module.nets) {
      // Distributed constants intentionally have independent physical roots
      // per crossbar, so the ordinary single-root tree invariant is inapplicable.
      if (net.distributed_source || net.routes.empty() ||
          !routeTreeDebugMatches(net, filter)) {
        continue;
      }

      std::unordered_map<std::string, std::vector<size_t>> node_routes;
      std::unordered_map<std::string, std::vector<std::string>> children;
      std::vector<std::vector<std::string>> route_nodes;
      std::vector<size_t> binding_indices;
      bool self_repeat = false;
      bool duplicate_non_root = false;
      bool duplicate_source_takeoff = false;

      for (size_t binding_index = 0; binding_index < net.routes.size();
           ++binding_index) {
        rtl::NetRouteBinding &binding = net.routes[binding_index];
        std::vector<Wire> *route = routeBindingRoute(binding);
        if (!route || route->empty() || !routeIsComplete(*route)) {
          continue;
        }
        std::vector<std::string> nodes = routeTreeNodes(*route);
        if (nodes.empty()) {
          continue;
        }
        std::unordered_set<std::string> seen_in_route;
        for (const std::string &node : nodes) {
          if (!seen_in_route.insert(node).second) {
            self_repeat = true;
          }
          node_routes[node].push_back(binding_index);
        }
        for (size_t i = 0; i + 1 < nodes.size(); ++i) {
          auto &outs = children[nodes[i]];
          if (std::find(outs.begin(), outs.end(), nodes[i + 1]) == outs.end()) {
            outs.push_back(nodes[i + 1]);
          }
        }
        binding_indices.push_back(binding_index);
        route_nodes.push_back(std::move(nodes));
      }

      if (route_nodes.size() < 2 && !self_repeat) {
        continue;
      }

      std::string root = route_nodes.empty() || route_nodes[0].empty()
                             ? std::string{}
                             : route_nodes[0][0];
      for (const auto &[node, owners] : node_routes) {
        if (node != root && owners.size() > 1) {
          duplicate_non_root = true;
          break;
        }
      }
      if (!root.empty()) {
        auto child_it = children.find(root);
        duplicate_source_takeoff =
            child_it != children.end() && child_it->second.size() > 1;
      }

      if (!self_repeat && !duplicate_non_root && !duplicate_source_takeoff) {
        continue;
      }

      ++malformed;
      PNR_LOG1(
          "ROUT",
          "routeTree malformed: net='{}', routes={}, root='{}', "
          "self_repeat={}, duplicate_non_root={}, duplicate_source_takeoff={}",
          net.makeName(FULL_NAME_LIMIT), route_nodes.size(), root, self_repeat,
          duplicate_non_root, duplicate_source_takeoff);
      for (size_t i = 0; i < route_nodes.size(); ++i) {
        rtl::NetRouteBinding &binding = net.routes[binding_indices[i]];
        PNR_LOG1("ROUT",
                 "routeTree route: net='{}', binding={}, route_name='{}', "
                 "from='{}' port='{}', to='{}' port='{}', nodes={}, "
                 "owner='{}', route_index={}",
                 net.makeName(FULL_NAME_LIMIT), binding_indices[i],
                 binding.route_name,
                 binding.from ? binding.from->makeName(FULL_NAME_LIMIT)
                              : std::string{},
                 binding.from_port,
                 binding.to ? binding.to->makeName(FULL_NAME_LIMIT)
                            : std::string{},
                 binding.to_port, route_nodes[i].size(),
                 binding.owner ? binding.owner->makeName(FULL_NAME_LIMIT)
                               : std::string{},
                 binding.route_index);
        if (route_nodes[i].size() >= 2) {
          PNR_LOG1("ROUT",
                   "routeTree route ends: binding={}, first='{}', second='{}', "
                   "last='{}'",
                   binding_indices[i], route_nodes[i][0], route_nodes[i][1],
                   route_nodes[i].back());
        }
      }
      for (const auto &[node, owners] : node_routes) {
        if (node == root || owners.size() <= 1) {
          continue;
        }
        PNR_LOG1("ROUT",
                 "routeTree duplicate node: net='{}', node='{}', bindings={}",
                 net.makeName(FULL_NAME_LIMIT), node,
                 routeTreeOwnerList(owners));
      }
      if (!root.empty()) {
        auto child_it = children.find(root);
        if (child_it != children.end() && child_it->second.size() > 1) {
          PNR_LOG1("ROUT",
                   "routeTree source takeoff: net='{}', root='{}', children={}",
                   net.makeName(FULL_NAME_LIMIT), root,
                   child_it->second.size());
          for (const std::string &child : child_it->second) {
            PNR_LOG1("ROUT",
                     "routeTree source child: net='{}', root='{}', child='{}'",
                     net.makeName(FULL_NAME_LIMIT), root, child);
          }
        }
      }
    }
  }
  if (malformed || !filter.empty()) {
    PNR_LOG1("ROUT", "routeTree diagnostics: malformed={}, filter='{}'",
             malformed, filter);
  }
}

uint64_t tileDeadendKey(const Coord &coord) {
  return (static_cast<uint64_t>(static_cast<uint16_t>(coord.x)) << 48) |
         (static_cast<uint64_t>(static_cast<uint16_t>(coord.y)) << 32);
}

void applyRouteDeadends(
    const std::unordered_map<uint64_t, NodeMask> &src_deadends) {
  for (auto &tile_ref : fpga::Device::current().tile_grid) {
    Tile &tile = tile_ref;
    tile.cb.src_deadend = {};
    auto src_it = src_deadends.find(tileDeadendKey(tile.coord));
    if (src_it != src_deadends.end()) {
      tile.cb.src_deadend.jump |= src_it->second;
    }
  }
}

struct LocalRouteNode {
  fpga::CBNodeNameType type = fpga::CB_NODE_LOCAL;
  int value = -1;
};

uint32_t localRouteNodeKey(const LocalRouteNode &node) {
  return (static_cast<uint32_t>(node.type) << 16) |
         static_cast<uint16_t>(node.value);
}

LocalRouteNode localRouteNodeFromKey(uint32_t key) {
  return LocalRouteNode{
      static_cast<fpga::CBNodeNameType>((key >> 16) & 0xff),
      static_cast<int>(key & 0xffff)};
}

NodeMask &leasedLocalRouteNodes(Tile &tile, fpga::CBNodeNameType type) {
  switch (type) {
  case fpga::CB_NODE_LOCAL:
    return tile.cb.local.local;
  case fpga::CB_NODE_JOINT:
    return tile.cb.joint.jump;
  case fpga::CB_NODE_SRC:
    return tile.cb.src.jump;
  case fpga::CB_NODE_DST:
    return tile.cb.dst.jump;
  case fpga::CB_NODE_JUMP:
    break;
  }
  PNR_ASSERT(false, "unsupported local route node type {}",
             static_cast<int>(type));
  return tile.cb.local.local;
}

std::vector<LocalRouteNode>
localRouteSuccessors(const fpga::CBType &type, const LocalRouteNode &node) {
  std::vector<LocalRouteNode> result;
  auto append = [&](fpga::CBNodeNameType kind, const NodeMask &mask) {
    mask.for_each_set_bit([&](int value) {
      result.push_back(LocalRouteNode{kind, value});
      return false;
    });
  };
  if (node.type == fpga::CB_NODE_LOCAL) {
    append(fpga::CB_NODE_LOCAL, type.local_local[node.value].local);
    append(fpga::CB_NODE_JOINT, type.local_joint[node.value].joint);
  } else if (node.type == fpga::CB_NODE_JOINT) {
    append(fpga::CB_NODE_LOCAL, type.joint_local[node.value].local);
    append(fpga::CB_NODE_JOINT, type.joint_joint[node.value].joint);
  }
  return result;
}

bool localRouteNodeOwnedByNet(Tile &tile, const LocalRouteNode &node,
                              const rtl::Net &net) {
  std::vector<fpga::NetRouteRef> owners = fpga::findNetOwnersByNode(
      tile, node.type, node.value, false);
  return !owners.empty() &&
         std::all_of(owners.begin(), owners.end(),
                     [&](const fpga::NetRouteRef &owner) {
                       return owner.net == &net;
                     });
}

// A mandatory local path may cross an occupied node only when every foreign
// owner can be returned to the ordinary generic scheduler.
bool distributedLocalNodeAvailable(Tile &tile, const LocalRouteNode &node,
                                   const rtl::Net &net,
                                   bool allow_transit_preemptible) {
  NodeMask bit = NodeMask{0, 1} << node.value;
  if ((leasedLocalRouteNodes(tile, node.type) & bit) == NodeMask{}) {
    return true;
  }
  std::vector<fpga::NetRouteRef> owners = fpga::findNetOwnersByNode(
      tile, node.type, node.value, false);
  if (owners.empty()) {
    return false;
  }
  return std::all_of(
      owners.begin(), owners.end(), [&](const fpga::NetRouteRef &owner) {
        if (owner.net == &net) {
          return true;
        }
        if (!allow_transit_preemptible || !owner.net ||
            !owner.net->routeCanBePreempted() ||
            owner.binding_index >= owner.net->routes.size()) {
          return false;
        }
        // A mandatory local source may displace transit congestion, but never
        // an ordinary route that terminates on a resource in this tile.
        return !bindingToTile(owner.net->routes[owner.binding_index], tile);
      });
}

// Find one path inside a route tile from any declared distributed source to a
// requested local endpoint. Connectivity and occupancy are numeric masks only.
std::vector<LocalRouteNode>
findDistributedLocalPath(Tile &tile, int target_local, rtl::Net &net,
                         const NodeMask &root_nodes,
                         bool allow_transit_preemptible = false,
                         size_t *root_count = nullptr,
                         size_t *visited_count = nullptr) {
  if (!tile.cb_type || target_local < 0 || target_local >= CB_MAX_NODES) {
    return {};
  }

  std::deque<LocalRouteNode> queue;
  std::unordered_map<uint32_t, uint32_t> parent;
  std::unordered_set<uint32_t> roots;
  root_nodes.for_each_set_bit([&](int local) {
    if (root_count) {
      ++*root_count;
    }
    LocalRouteNode root{fpga::CB_NODE_LOCAL, local};
    uint32_t key = localRouteNodeKey(root);
    roots.insert(key);
    parent.emplace(key, key);
    queue.push_back(root);
    return false;
  });

  uint32_t reached = std::numeric_limits<uint32_t>::max();
  while (!queue.empty()) {
    LocalRouteNode current = queue.front();
    queue.pop_front();
    if (visited_count) {
      ++*visited_count;
    }
    if (current.type == fpga::CB_NODE_LOCAL &&
        current.value == target_local) {
      reached = localRouteNodeKey(current);
      break;
    }
    for (const LocalRouteNode &next :
         localRouteSuccessors(*tile.cb_type, current)) {
      uint32_t key = localRouteNodeKey(next);
      if (parent.contains(key)) {
        continue;
      }
      if (!distributedLocalNodeAvailable(tile, next, net,
                                         allow_transit_preemptible)) {
        continue;
      }
      parent.emplace(key, localRouteNodeKey(current));
      queue.push_back(next);
    }
  }
  if (reached == std::numeric_limits<uint32_t>::max()) {
    return {};
  }

  std::vector<LocalRouteNode> path;
  for (uint32_t cursor = reached;; cursor = parent.at(cursor)) {
    path.push_back(localRouteNodeFromKey(cursor));
    if (roots.contains(cursor)) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  return path;
}

struct DistributedLocalVictim {
  fpga::NetRouteRef route;
  LocalRouteNode node;
};

// Collect every foreign owner that must be released before claiming one
// mandatory local path. Empty ownership on a leased bit is treated as stale
// state and is never preempted speculatively.
bool distributedLocalPathVictims(
    Tile &tile, const std::vector<LocalRouteNode> &path, int target_local,
    rtl::Net &net, std::vector<DistributedLocalVictim> &victims) {
  victims.clear();
  auto collect = [&](const LocalRouteNode &node, bool occupied) {
    if (!occupied) {
      return true;
    }
    std::vector<fpga::NetRouteRef> owners = fpga::findNetOwnersByNode(
        tile, node.type, node.value, false);
    if (owners.empty()) {
      return false;
    }
    for (const fpga::NetRouteRef &owner : owners) {
      if (!owner.net || owner.net == &net) {
        continue;
      }
      if (!owner.net->routeCanBePreempted() ||
          owner.binding_index >= owner.net->routes.size()) {
        return false;
      }
      if (bindingToTile(owner.net->routes[owner.binding_index], tile)) {
        return false;
      }
      bool duplicate = std::any_of(
          victims.begin(), victims.end(),
          [&](const DistributedLocalVictim &old) {
            return old.route.net == owner.net &&
                   old.route.binding_index == owner.binding_index;
          });
      if (!duplicate) {
        victims.push_back(DistributedLocalVictim{owner, node});
      }
    }
    return true;
  };

  for (size_t index = 1; index < path.size(); ++index) {
    const LocalRouteNode &node = path[index];
    NodeMask bit = NodeMask{0, 1} << node.value;
    if (!collect(node, (leasedLocalRouteNodes(tile, node.type) & bit) !=
                           NodeMask{})) {
      return false;
    }
  }
  LocalRouteNode target{fpga::CB_NODE_LOCAL, target_local};
  return collect(target, tile.isPinNodeLeased(target_local));
}

} // namespace

// Validate every terminal owner before cutting any route, including takeoff protection.
// A rejected path leaves all leases intact and does not hide later terminal choices.
GroundingTerminalPath RouteDesign::preemptGroundingTerminal(
    Tile &tile, int local, NodeMask incoming_dsts,
    NodeMask unavailable_joints, const RouteTask &task,
    const GroundingTerminalPath *required_path) {
  if (!route_preemption_enabled || !tile.cb_type || local < 0 ||
      local >= CB_MAX_NODES || tile.isPinNodeLeased(local) ||
      tile.cb.local.local.testBit(local)) {
    return {};
  }
  return groundingPreemptionPath(
      *tile.cb_type, tile.cb, local, incoming_dsts, unavailable_joints,
      [&](const GroundingTerminalPath &path) {
        if ((required_path && !sameGroundingTerminalPath(path, *required_path)) ||
            (path.joint >= 0 && unavailable_joints.testBit(path.joint)) ||
            (path.joint2 >= 0 && unavailable_joints.testBit(path.joint2))) {
          return false;
        }
        std::vector<fpga::RouteCutNode> cuts;
        for (auto [type, node] : {
                 std::pair{fpga::CB_NODE_DST, path.dst},
                 std::pair{fpga::CB_NODE_JOINT, path.joint},
                 std::pair{fpga::CB_NODE_JOINT, path.joint2}}) {
          if (node >= 0 && fpga::congestionNodeMask(tile.cb, type).testBit(node)) {
            cuts.push_back({tile.coord, type, node});
          }
        }
        std::vector<fpga::NetRouteRef> victims;
        for (const fpga::RouteCutNode &cut : cuts) {
          auto owners = fpga::findNetRoutesByNode(tile, cut.type, cut.node, false);
          if (owners.empty()) {
            return false;
          }
          for (const fpga::NetRouteRef &owner : owners) {
            if (!owner.net || !owner.net->routeCanBePreempted() ||
                owner.binding_index >= owner.net->routes.size()) {
              return false;
            }
            const auto &binding = owner.net->routes[owner.binding_index];
            if (!binding.from || !binding.to || binding.route_name.empty() ||
                preemptionOwnerIsCurrentTree(
                    owner.net == task.net, sourceRouteKey(task.from, task.from_port),
                    sourceRouteKey(binding.from, binding.from_port)) ||
                preemptionOwnerIsProtected(this, binding) ||
                bindingGroundsInTile(owner.net->routes[owner.binding_index], tile) ||
                !canPreemptDuringFocusedMove(moving_stage, moving_focus_inst != nullptr,
                                            moving_sources_stage) ||
                !canPreemptMovingRoute(moving_stage,
                    movingInstIsFinished(move_finished_insts, binding.from) ||
                    movingInstIsFinished(move_finished_insts, binding.to)) ||
                preempted_route_names_this_pass.contains(binding.route_name) ||
                preemptionWouldCycle(preempted_route_blockers, task.net_name,
                                     binding.route_name) ||
                !fpga::canPreemptNetRouteFromNodes(*owner.net, owner.binding_index, cuts)) {
              return false;
            }
            if (std::none_of(victims.begin(), victims.end(), [&](const auto &old) {
                  return old.net == owner.net && old.binding_index == owner.binding_index;
                })) {
              victims.push_back(owner);
            }
          }
        }
        if (victims.empty()) {
          return false;
        }
        // All checks precede mutation; shared owners are cut as one transaction.
        for (const fpga::NetRouteRef &victim : victims) {
          const auto binding = victim.net->routes[victim.binding_index];
          const auto *before = routeBindingRoute(victim.net->routes[victim.binding_index]);
          const size_t old_size = before->size();
          const bool complete = routeIsComplete(*before);
          PNR_ASSERT(fpga::unrouteNetRouteFromNodes(*victim.net, victim.binding_index, cuts),
                     "preflighted grounding cut failed for {}", binding.route_name);
          auto *after = routeBindingRoute(victim.net->routes[victim.binding_index]);
          enqueueRouteTask(RouteTask{binding.from, binding.to, victim.net,
              binding.from_port, binding.to_port, binding.route_name,
              0, 0, 0, {}, routeStartsWithSharedPrefix(after)}, pending_route_todo);
          preempted_route_names_this_pass.insert(binding.route_name);
          rememberPreemptionBlocker(preempted_route_blockers, binding.route_name, task.net_name);
          route_stats.preempt_removed_fragments += old_size - after->size();
          if (complete) {
            ++route_stats.preempt_complete_victims;
            ++route_stats.preempt_grounding_complete_victims;
          } else {
            ++route_stats.preempt_partial_victims;
            ++route_stats.preempt_grounding_partial_victims;
          }
        }
        for (const fpga::RouteCutNode &cut : cuts) {
          PNR_ASSERT(!fpga::congestionNodeMask(tile.cb, cut.type).testBit(cut.node),
                     "grounding retained contested node ({},{}) type={} node={}",
                     tile.coord.x, tile.coord.y, static_cast<int>(cut.type), cut.node);
        }
        ++route_stats.preempt_attempts;
        ++route_stats.preempt_success;
        route_changed = true;
        PNR_LOG1("ROUT", "routeDesign grounding released: net='{}', tile=({},{}), "
                 "dst={}, joint={}, joint2={}, local={}, victims={}",
                 task.net_name, tile.coord.x, tile.coord.y, path.dst, path.joint,
                 path.joint2, local, victims.size());
        return true;
      });
}

// Retain a bounded unleased physical attempt for terminal visualization.
// A newer attempt for the same route replaces its predecessor.
void RouteDesign::rememberFailureVisualizationPath(
    RouteTask task, std::vector<Wire> route) {
  if (route.empty()) {
    return;
  }
  constexpr size_t diagnostic_path_limit = 128;
  std::erase_if(failure_visualization_paths,
                [&](const FailureVisualizationPath &path) {
                  return sameVisualizationRouteTask(path.task, task);
                });
  if (failure_visualization_paths.size() >= diagnostic_path_limit) {
    failure_visualization_paths.erase(failure_visualization_paths.begin());
  }
  for (Wire &fragment : route) {
    fragment.net_name = task.net_name;
  }
  failure_visualization_paths.push_back(
      FailureVisualizationPath{std::move(task), std::move(route)});
}

size_t RouteDesign::repairStaleSharedRoutePrefixes(
    rtl::Design &design, std::vector<RouteTask> &fanout_tasks) {
  return repairStaleSharedRoutePrefixesImpl(design, fanout_tasks);
}

// Split repaired source trees into one Generic seed per physical source port.
// Dependent routes stay deferred until that seed provides a complete branchable
// tree.
size_t
RouteDesign::scheduleSharedPrefixRepairs(std::vector<RouteTask> &repairs,
                                         std::vector<RouteTask> &generic_tasks,
                                         std::vector<RouteTask> &fanout_tasks) {
  std::unordered_set<std::string> scheduled_sources;
  size_t generic_count = 0;
  for (RouteTask &task : repairs) {
    std::string source_key = sourceRouteKey(task.from, task.from_port);
    if (scheduled_sources.insert(source_key).second) {
      task.fanout = false;
      appendUniqueRouteTask(generic_tasks, task);
      ++generic_count;
    } else {
      task.fanout = true;
      appendUniqueRouteTask(fanout_tasks, task);
    }
  }
  repairs.clear();
  return generic_count;
}

void RouteDesign::scheduleOneSeedPerSource(
    std::vector<RouteTask> &tasks, std::vector<RouteTask> &generic_tasks,
    std::vector<RouteTask> &fanout_tasks) {
  // Preserve one physical source-port trunk and defer every sibling to the
  // branch-building stage, including distributed constant-source routes.
  scheduleOneSeedPerSourcePort(
      tasks, generic_tasks, fanout_tasks,
      [](const RouteTask &task) {
        return sourceRouteKey(task.from, task.from_port);
      },
      [](std::vector<RouteTask> &queue, const RouteTask &task) {
        appendUniqueRouteTask(queue, task);
      });
}

bool RouteDesign::routeNet(rtl::Inst &from, const std::string &from_port,
                           rtl::Inst &to, const std::string &to_port,
                           std::vector<Wire> &wire, bool &complete,
                           size_t attempt, rtl::Net *net,
                           const std::string &route_name) {
  //    PNR_ASSERT(!from.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not
  //    assigned", from.makeName()) PNR_ASSERT(!to.tile.peer,
  //    "RouteDesign::tryOut, inst '%s' tile is not assigned", to.makeName())
  complete = true;
  // Caller sets the bounded suffix depth before entering this route attempt.
  std::string trace_net =
      !route_name.empty()
          ? route_name
          : (net ? net->makeName(FULL_NAME_LIMIT) : std::string{});
  size_t searches_before = route_stats.route_searches;
  if (!from.tile.peer || !to.tile.peer) {
    complete = false;
    PNR_LOG1("ROUT",
             "routeNet missing placement: net='{}', from='{}' type='{}' "
             "port='{}' placed={}, to='{}' type='{}' port='{}' placed={}",
             trace_net, from.makeName(FULL_NAME_LIMIT),
             from.cell_ref.peer ? from.cell_ref->type : std::string{},
             from_port, from.tile.peer != nullptr, to.makeName(FULL_NAME_LIMIT),
             to.cell_ref.peer ? to.cell_ref->type : std::string{}, to_port,
             to.tile.peer != nullptr);
    return false;
  }
  bool trace_route_net =
      routeDebugMatches("SCALEPNR_ROUTE_NET_TRACE", trace_net);
  auto candidates_start = std::chrono::steady_clock::now();
  Coord to_coord = to.tile->coord;
  Coord from_coord = from.tile->coord;
  std::vector<Tile *> from_route_tiles =
      routeTileCandidates(from, from_port, true, &to_coord);
  std::vector<Tile *> to_route_tiles =
      routeTileCandidates(to, to_port, false, &from_coord);
  route_stats.task_candidate_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - candidates_start)
          .count());
  if (trace_route_net) {
    PNR_LOG1("ROUT",
             "routeNet trace start: net='{}', from='{}'/'{}' to='{}'/'{}', "
             "from_tiles={}, to_tiles={}, attempt={}",
             trace_net, from.makeName(FULL_NAME_LIMIT), from_port,
             to.makeName(FULL_NAME_LIMIT), to_port, from_route_tiles.size(),
             to_route_tiles.size(), attempt);
  }
  NodeMask output_nodes =
      from.tile->getOutputPinNodes(from.cell_ref->type, from_port, from.pos);
  if (from.tile.peer == to.tile.peer) {
    // Packed same-tile resources should be connected locally before trying
    // global routing.
    auto direct_start = std::chrono::steady_clock::now();
    if (tryDirectResourceRoute(from, from_port, to, to_port, wire, net)) {
      route_stats.task_direct_ns += static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - direct_start)
              .count());
      complete = true;
      return true;
    }
    route_stats.task_direct_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - direct_start)
            .count());
  }

  auto check_output_local = [&](Tile &route_tile, int local,
                                bool assert_on_invalid) {
    if (isRoutableOutputLocal(route_tile, local)) {
      return true;
    }
    if (assert_on_invalid) {
      const std::string *name =
          route_tile.cb_type
              ? route_tile.cb_type->nodeName(fpga::CB_NODE_LOCAL, local)
              : nullptr;
      PNR_ASSERT(
          false,
          "routeNet tried to start net '{}' from non-endpoint or non-routable "
          "output local node {}{}{} in tile '{}' at ({},{}); route starts must "
          "use the mapped local output node for that source port",
          net ? net->makeName() : std::string{}, local, name ? " '" : "",
          name ? *name + "'" : std::string{}, route_tile.makeName(),
          route_tile.coord.x, route_tile.coord.y);
    }
    return false;
  };

  auto try_output_nodes = [&](NodeMask nodes, bool assert_on_invalid) {
    for (Tile *from_route_tile : from_route_tiles) {
      if (!from_route_tile) {
        continue;
      }
      auto endpoint_start = std::chrono::steady_clock::now();
      NodeMask route_nodes =
          routeTileEndpointNodes(*from_route_tile, from, from_port, true);
      bool same_coord = from.tile->coord.x == from_route_tile->coord.x &&
                        from.tile->coord.y == from_route_tile->coord.y;
      if (route_nodes == NodeMask{} && same_coord &&
          supportsOutputLocalNodes(*from_route_tile, nodes)) {
        route_nodes = nodes;
      }
      route_stats.task_endpoint_ns += static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - endpoint_start)
              .count());
      bool routed = route_nodes.for_each_set_bit([&](int local) {
        for (Tile *to_route_tile : to_route_tiles) {
          wire.clear();
          bool attempt_complete = false;
          if (!to_route_tile ||
              !check_output_local(*from_route_tile, local, assert_on_invalid)) {
            continue;
          }
          if (sourceLocalOwnedByDifferentEndpoint(*from_route_tile, local, from,
                                                  from_port, net)) {
            continue;
          }
          auto input_endpoint_start = std::chrono::steady_clock::now();
          NodeMask pin_nodes = routeTileInputNodes(*to_route_tile, to, to_port);
          route_stats.task_endpoint_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - input_endpoint_start)
                  .count());
          if (trace_route_net) {
            PNR_LOG1("ROUT",
                     "routeNet trace try: net='{}', from_tile=({},{}) "
                     "local={}, to_tile=({},{}) pin_nodes={}, depth_limit={}",
                     trace_net, from_route_tile->coord.x,
                     from_route_tile->coord.y, local, to_route_tile->coord.x,
                     to_route_tile->coord.y, maskString(pin_nodes),
                     iteration_limit);
          }
          auto best_first_start = std::chrono::steady_clock::now();
          bool allow_preempt =
              route_preemption_enabled &&
              pnr::movingRouteMayPreempt(moving_stage,
                                         moving_focus_inst != nullptr,
                                         moving_sources_stage);
          const pnr::RouteAttemptPreemption preemption =
              pnr::genericRouteAttemptPreemption(allow_preempt);
          bool candidate_routed = tryBestFirstRoute(
              *from_route_tile, *to_route_tile, local, to, to_port, wire,
              iteration_limit, false, {}, &attempt_complete, &route_stats, this,
              net, false, &from, from_port, trace_net, pin_nodes, true, false,
              nullptr, preemption.transit, preemption.docking);
          route_stats.task_best_first_ns += static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - best_first_start)
                  .count());
          if (trace_route_net) {
            PNR_LOG1("ROUT",
                     "routeNet trace result: net='{}', routed={}, complete={}, "
                     "wire_size={}, depth_limit={}",
                     trace_net, candidate_routed, attempt_complete, wire.size(),
                     iteration_limit);
          }
          if (candidate_routed) {
            prependSourceEndpoint(wire, from, from_port);
            complete = attempt_complete;
            return true;
          }
        }
        return false;
      });
      if (routed) {
        return true;
      }
    }
    return false;
  };

  bool routed = try_output_nodes(output_nodes, false);
  if (!routed && !anyRoutableOutputCandidate(from_route_tiles, from, from_port,
                                             output_nodes)) {
    routed = tryDirectResourceRoute(from, from_port, to, to_port, wire, net);
    if (routed) {
      complete = true;
    }
  }
  if (!routed && route_stats.route_searches == searches_before) {
    NodeMask input_nodes =
        to.tile.peer ? to.tile->getPinNodes(to.cell_ref->type, to_port, to.pos)
                     : NodeMask{};
    if (trace_route_net ||
        envFlagEnabled("SCALEPNR_ROUTE_NO_SEARCH_DETAIL")) {
      PNR_LOG1("ROUT",
               "routeNet no-search failure: net='{}', from='{}' type='{}' "
             "port='{}' tile=({},{})/{} output_nodes={}, from_route_tiles={}, "
             "to='{}' type='{}' port='{}' tile=({},{})/{} input_nodes={}, "
             "to_route_tiles={}, attempt={}",
             net ? net->makeName(FULL_NAME_LIMIT) : std::string{},
             from.makeName(FULL_NAME_LIMIT), from.cell_ref->type, from_port,
             from.tile.peer ? from.tile->coord.x : -1,
             from.tile.peer ? from.tile->coord.y : -1, from.pos,
             maskString(output_nodes), from_route_tiles.size(),
             to.makeName(FULL_NAME_LIMIT), to.cell_ref->type, to_port,
             to.tile.peer ? to.tile->coord.x : -1,
             to.tile.peer ? to.tile->coord.y : -1, to.pos,
               maskString(input_nodes), to_route_tiles.size(), attempt);
    }
  }
  return routed;
}

bool RouteDesign::routeNet(rtl::Inst &from, rtl::Inst &to,
                           const std::string &to_port,
                           std::vector<Wire> &wire) {
  bool complete = true;
  return routeNet(from, std::string(), to, to_port, wire, complete) && complete;
}

bool RouteDesign::routeNet(rtl::Inst &from, rtl::Inst &to,
                           std::vector<Wire> &wire) {
  bool complete = true;
  return routeNet(from, std::string(), to, std::string(), wire, complete) &&
         complete;
}

namespace {
class TaskCongestionDiagnostic {
 public:
  TaskCongestionDiagnostic(RouteDesign& router, const RouteDesign::RouteTask& task, const char* operation)
      : task(task), history(task.net, operation, task.net_name) {
    if (!RouteCongestionTrace::matches(task.net_name)) return;
    if (envFlagEnabled("SCALEPNR_CONGESTION_FORWARD_ONLY") &&
        std::string_view(operation) != "Moving forward placement") return;
    std::vector<rtl::Net*> nets;
    if (router.tech) {
      for (rtl::Module& module : router.tech->design.modules)
        for (rtl::Net& net : module.nets) nets.push_back(&net);
    }
    for (const auto& [source, source_nets] : router.source_route_nets)
      nets.insert(nets.end(), source_nets.begin(), source_nets.end());
    if (task.net) nets.push_back(task.net);
    trace = std::make_unique<RouteCongestionTrace>(task.net_name,
        router.moving_stage ? "Moving" : router.fanout_stage ? "Fanout" : "Basic",
        std::move(nets));
    trace->stream() << "TASK retry=" << task.attempt << " branch_retry=" << task.fanout_branch_attempt
        << " branch_offset=" << task.fanout_branch_offset << " driver="
        << std::quoted(task.from ? task.from->makeName(FULL_NAME_LIMIT) : "")
        << " port=" << std::quoted(task.from_port) << " sink="
        << std::quoted(task.to ? task.to->makeName(FULL_NAME_LIMIT) : "")
        << " port=" << std::quoted(task.to_port) << '\n';
    state("BEFORE");
  }
  ~TaskCongestionDiagnostic() { if (trace) state("AFTER"); }
 private:
  const RouteDesign::RouteTask& task;
  fpga::RouteHistoryScope history;
  std::unique_ptr<RouteCongestionTrace> trace;
  void state(const char* phase) {
    const auto* route = findBoundRoute(task.net, task.from, task.to, task.from_port,
                                      task.to_port, task.net_name);
    auto& out = trace->stream();
    out << phase << " complete=" << (route && routeIsComplete(*route))
        << " fragments=" << (route ? route->size() : 0) << " retry=" << task.attempt
        << " branch_retry=" << task.fanout_branch_attempt << " branch_offset=" << task.fanout_branch_offset << '\n';
    if (route) for (size_t i = 0; i < route->size(); ++i) {
      const auto& w = (*route)[i];
      out << "PATH fragment=" << i << " type=" << w.type << " from=(" << w.from.x << ',' << w.from.y
          << ") to=(" << w.to.x << ',' << w.to.y << ") local=" << w.local << " src=" << w.jump
          << " dst=" << w.dst << " joint=" << w.joint << " joint2=" << w.joint2
          << " shared=" << w.shared << " owns_dst=" << w.owns_dst << " owns_landing=" << w.owns_landing << '\n';
    }
  }
};
}

bool RouteDesign::routeFanoutTask(RouteTask &task, int depth) {
  TaskCongestionDiagnostic diagnostic(*this, task, __func__);
  PNR_ASSERT(task.from && task.to,
             "routeFanoutTask got null endpoint for net '{}'", task.net_name);
  if (!task.net) {
    return routeNetTask(task, depth);
  }

  ++route_stats.task_attempts;
  static int fanout_debug_task_lines = 0;
  if (std::getenv("SCALEPNR_DEBUG_FANOUT_TASKS") &&
      fanout_debug_task_lines < 200) {
    ++fanout_debug_task_lines;
    PNR_LOG1("ROUT",
             "routeFanoutTask debug task[{}]: net='{}', from='{}' type='{}' "
             "port='{}' tile=({},{})/{}, to='{}' type='{}' port='{}' "
             "tile=({},{})/{}, attempt={}, routes={}",
             fanout_debug_task_lines, task.net_name,
             task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
             task.from && task.from->cell_ref.peer ? task.from->cell_ref->type
                                                   : std::string{},
             task.from_port,
             task.from && task.from->tile.peer ? task.from->tile->coord.x : -1,
             task.from && task.from->tile.peer ? task.from->tile->coord.y : -1,
             task.from ? task.from->pos : -1,
             task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
             task.to && task.to->cell_ref.peer ? task.to->cell_ref->type
                                               : std::string{},
             task.to_port,
             task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
             task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
             task.to ? task.to->pos : -1, task.attempt,
             task.net ? task.net->routes.size() : 0);
  }
  PNR_LOG3_(
      "ROUT", depth,
      "routeFanoutTask, net: '{}', from: '{}' port '{}', to: '{}' port '{}'",
      task.net_name, task.from->makeName(), task.from_port, task.to->makeName(),
      task.to_port);
  bool debug_fanout_net =
      routeDebugMatches("SCALEPNR_DEBUG_FANOUT_NET", task.net_name) ||
      (std::getenv("SCALEPNR_DEBUG_FANOUT_TASKS") &&
       fanout_debug_task_lines <= 8);

  rtl::NetRouteBinding *existing_binding =
      findNetRouteBinding(*task.net, *task.from, *task.to, task.from_port,
                          task.to_port, task.net_name);
  std::vector<Wire> *existing_route =
      existing_binding ? routeBindingRoute(*existing_binding) : nullptr;
  if (existing_route && routeIsComplete(*existing_route)) {
    if (task.net) {
      size_t route_index = findRouteIndex(*task.to, existing_route);
      if (route_index != std::numeric_limits<size_t>::max()) {
        size_t binding_index = fpga::attachNetRoute(
            *task.net, *task.to, route_index, task.from, task.to,
            task.from_port, task.to_port, task.net_name);
        fpga::registerNetRouteTiles(*task.net, *existing_route,
                                    binding_index);
      }
    }
    ++route_stats.already_complete;
    return true;
  }
  if (route_recursion_budget <= 0) {
    return false;
  }

  bool route_complete = false;
  // A shared-only route is a stale replica left after its private branch was
  // consumed. Clear it without consuming the next branch rotation position.
  if (task.fanout && existing_route && !existing_route->empty() &&
      !routeIsComplete(*existing_route) &&
      routeStartsWithSharedPrefix(existing_route) &&
      routeUnsharedCrossbarFragments(existing_route) == 0) {
    size_t owner_route_index = findRouteIndex(*task.to, existing_route);
    size_t binding_index =
        task.net && owner_route_index != std::numeric_limits<size_t>::max()
            ? findNetRouteBindingIndex(*task.net, *task.to, owner_route_index)
            : std::numeric_limits<size_t>::max();
    bool discarded = task.net &&
                     binding_index != std::numeric_limits<size_t>::max() &&
                     fpga::discardNetBranch(*task.net, binding_index);
    if (!discarded) {
      existing_route->clear();
      discarded = true;
    }
    if (discarded) {
      route_changed = true;
      pnr::cleanFanoutSharedPrefix(task.fanout_branch_attempt);
      if (debug_fanout_net) {
        PNR_LOG1("ROUT",
                 "routeFanoutTask debug cleaned shared-only fanout prefix "
                 "without rotating again: net='{}', next_offset={}",
                 task.net_name, task.fanout_branch_offset);
      }
    }
    --route_recursion_budget;
    return false;
  }
  if (existing_route && !existing_route->empty()) {
    ++route_stats.continuation_attempts;
    if (task.fanout_branch_attempt >= fanout_branch_retry_limit) {
      bool discarded = rotateFanoutBranch(task, existing_route);
      task.fanout_branch_attempt = 0;
      route_changed = true;
      ++route_stats.failed;
      ++route_stats.cont_failed_empty;
      if (debug_fanout_net) {
        PNR_LOG1("ROUT",
                 "routeFanoutTask debug discarded retry-limited branch: "
                 "net='{}', discarded={}, limit={}, next_offset={}",
                 task.net_name, discarded, fanout_branch_retry_limit,
                 task.fanout_branch_offset);
      }
      --route_recursion_budget;
      return false;
    }
    size_t before_size = existing_route->size();
    route_iteration_budget = iteration_limit;
    // Fanout continuations retain the same grounding-preemption policy as
    // their initial branch attempt; only unfocused Moving reroutes defer it.
    bool allow_preempt =
        route_preemption_enabled && fanout_preemption_enabled &&
        pnr::movingRouteMayPreempt(moving_stage,
                                   moving_focus_inst != nullptr,
                                   moving_sources_stage);
    bool root_blocked = false;
    if (continuePartialRoute(*existing_route, *task.to, task.to_port,
                             iteration_limit, route_complete, &route_stats,
                             this, task.net, task.from, task.from_port,
                             task.net_name, &root_blocked, allow_preempt,
                             allow_preempt && task.fanout)) {
      for (Wire &fragment : *existing_route) {
        fragment.net_name = task.net_name;
      }
      size_t route_index = findRouteIndex(*task.to, existing_route);
      if (route_index != std::numeric_limits<size_t>::max()) {
        size_t binding_index = fpga::attachNetRoute(
            *task.net, *task.to, route_index, task.from, task.to,
            task.from_port, task.to_port, task.net_name);
        fpga::registerNetRouteTiles(*task.net, *existing_route,
                                    binding_index);
      }
      route_changed = true;
      route_progress = route_progress || route_complete ||
                       existing_route->size() > before_size;
      if (route_complete) {
        ++route_stats.completed;
        ++route_stats.cont_completed;
        task.fanout_branch_attempt = 0;
      } else if (existing_route->size() > before_size) {
        ++route_stats.partial_advanced;
        ++route_stats.cont_advanced;
        task.fanout_branch_attempt = 0;
      } else {
        ++route_stats.cont_no_advance;
        ++task.fanout_branch_attempt;
        if (task.fanout_branch_attempt >= fanout_branch_retry_limit) {
          bool discarded = rotateFanoutBranch(task, existing_route);
          route_changed = true;
          if (debug_fanout_net) {
            PNR_LOG1("ROUT",
                     "routeFanoutTask debug rotated no-advance retry-limited "
                     "branch: net='{}', discarded={}, limit={}, next_offset={}",
                     task.net_name, discarded, fanout_branch_retry_limit,
                     task.fanout_branch_offset);
          }
        }
      }
      --route_recursion_budget;
      return route_complete;
    }
    ++route_stats.failed;
    ++route_stats.cont_failed_empty;
    if (task.fanout) {
      pnr::BlockedFanoutAction blocked_action = pnr::blockedFanoutAction(
          root_blocked, routeUnsharedCrossbarFragments(existing_route));
      if (blocked_action == pnr::BlockedFanoutAction::backstep && task.net &&
          task.to) {
        ++route_stats.backstep_attempts;
        size_t route_index = findRouteIndex(*task.to, existing_route);
        size_t binding_index =
            route_index != std::numeric_limits<size_t>::max()
                ? findNetRouteBindingIndex(*task.net, *task.to, route_index)
                : std::numeric_limits<size_t>::max();
        if (binding_index != std::numeric_limits<size_t>::max() &&
            fpga::unrouteLastRouteStep(*task.net, binding_index)) {
          ++route_stats.backstep_success;
          ++route_stats.backstep_fragments;
          task.fanout_branch_attempt = 0;
          route_changed = true;
          if (debug_fanout_net) {
            PNR_LOG1("ROUT",
                     "routeFanoutTask debug removed blocked private hop: "
                     "net='{}', remaining={}, source_attempt={}, offset={}",
                     task.net_name, existing_route->size(), task.attempt,
                     task.fanout_branch_offset);
          }
          --route_recursion_budget;
          return false;
        }
      }
      if (blocked_action == pnr::BlockedFanoutAction::rotate) {
        bool discarded = rotateFanoutBranch(task, existing_route);
        route_changed = route_changed || discarded;
        if (debug_fanout_net) {
          PNR_LOG1("ROUT",
                   "routeFanoutTask debug rotated blocked one-hop branch: "
                   "net='{}', discarded={}, source_attempt={}, offset={}",
                   task.net_name, discarded, task.attempt,
                   task.fanout_branch_offset);
        }
        --route_recursion_budget;
        return false;
      }
      ++task.fanout_branch_attempt;
      if (task.fanout_branch_attempt >= fanout_branch_retry_limit) {
        bool discarded = rotateFanoutBranch(task, existing_route);
        route_changed = true;
        if (debug_fanout_net) {
          PNR_LOG1("ROUT",
                   "routeFanoutTask debug rotated failed retry-limited branch: "
                   "net='{}', discarded={}, limit={}, next_offset={}",
                   task.net_name, discarded, fanout_branch_retry_limit,
                   task.fanout_branch_offset);
        }
      }
    }
    --route_recursion_budget;
    return false;
  }

  struct BranchPoint {
    Tile *tile = nullptr;
    int local = -1;
    std::string dst_wire;
    bool start_from_dst = true;
    int score = 0;
    rtl::Inst *prefix_owner = nullptr;
    size_t prefix_route_index = std::numeric_limits<size_t>::max();
    size_t prefix_size = 0;
  };
  route_iteration_budget = iteration_limit;

  std::vector<BranchPoint> branches;
  std::vector<BranchPoint> fallback_branches;
  rtl::Inst *seed_inst = nullptr;
  std::string seed_port;
  // Index every completed branch of this source tree. Limiting this lookup to
  // the trunk starves fanouts once its finite set of fork points is exhausted.
  std::vector<SourceRouteBinding> source_bindings =
      completeIndexedFanoutSeedBindings(*this, *task.from, task.from_port,
                                        &seed_inst, &seed_port);
  size_t skipped_self_bindings = 0;
  if (source_bindings.empty()) {
    PNR_LOG1("ROUT",
             "routeFanoutTask missing seed: promote seed candidate net='{}' "
             "from='{}'/'{}'",
             task.net_name, task.from->makeName(FULL_NAME_LIMIT),
             task.from_port);
    task.fanout = false;
    ++route_stats.failed;
    --route_recursion_budget;
    return false;
  }
  if (seed_inst != task.from || seed_port != task.from_port) {
    PNR_LOG2("ROUT",
             "routeFanoutTask seed redirected: net='{}', "
             "task_source='{}'/'{}', seed_source='{}'/'{}'",
             task.net_name, task.from->makeName(FULL_NAME_LIMIT),
             task.from_port,
             seed_inst ? seed_inst->makeName(FULL_NAME_LIMIT) : std::string{},
             seed_port);
  }
  const std::string branch_index_key =
      sourceRouteKey(seed_inst ? seed_inst : task.from,
                     seed_inst ? seed_port : task.from_port);
  FanoutBranchIndex &branch_index = fanout_branch_indexes[branch_index_key];
  auto index_branches_from_binding = [&](SourceRouteBinding source_binding,
                                         bool count_skip) {
    if (!source_binding.binding) {
      return;
    }
    rtl::NetRouteBinding &binding = *source_binding.binding;
    bool same_sink_binding = existing_binding && &binding == existing_binding;
    if (same_sink_binding || !binding.owner ||
        binding.route_index >= binding.owner->wires.size()) {
      if (count_skip) {
        ++skipped_self_bindings;
      }
      return;
    }
    std::vector<Wire> &base = binding.owner->wires[binding.route_index];
    auto &indexed_owner_routes = branch_index.indexed_routes[binding.owner];
    if (!indexed_owner_routes.insert(binding.route_index).second) {
      return;
    }
    PNR_ASSERT(routeCrossbarFragments(&base) != 0,
               "Fanout routing source binding '{}' for task '{}' has no routed "
               "crossbar exit",
               binding.route_name, task.net_name);
    for (size_t fragment_index = 0; fragment_index < base.size();
         ++fragment_index) {
      const Wire &fragment = base[fragment_index];
      if (fragment.type == Wire::WIRE_TILE_PIN &&
          sameCoord(fragment.from, fragment.to)) {
        Tile *endpoint_tile = fpga::Device::current().getTile(
            fragment.from.x, fragment.from.y);
        if (endpoint_tile) {
          branch_index.endpoints_by_tile[endpoint_tile].push_back(
              FanoutEndpointIndexEntry{binding.owner, binding.route_index,
                                       fragment_index});
        }
      }
      if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0) {
        continue;
      }
      Tile *from_tile =
          fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
      if (!from_tile || !from_tile->cb_type) {
        continue;
      }
      fpga::TileJumpTarget target;
      if (fragment.dst >= 0) {
        target.tile =
            fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
        target.dst_node = fragment.dst;
        target.jump_node =
            fragment.route_jump >= 0 ? fragment.route_jump : fragment.jump;
        target.dst_wire = fragment.dst_wire_name;
      } else {
        int route_jump =
            fragment.route_jump >= 0 ? fragment.route_jump : fragment.jump;
        target = fpga::Device::current().resolveJump(*from_tile, route_jump);
      }
      if (!target.tile || !target.tile->cb_type || target.dst_node < 0) {
        continue;
      }
      // Shared prefixes are replicated in every completed sink binding. Index
      // each physical branch once across this routing pass.
      if (!branch_index.indexed_nodes[target.tile]
               .insert(target.dst_node)
               .second) {
        continue;
      }
      size_t indexed_branch = branch_index.branches.size();
      branch_index.branches.push_back(FanoutBranchIndexEntry{
          binding.owner, binding.route_index, fragment_index + 1, target.tile,
          target.dst_node, target.dst_wire, binding.route_name});
      branch_index.branches_by_tile[target.tile].push_back(indexed_branch);
    }
  };
  for (size_t binding_index = 0; binding_index < source_bindings.size();
       ++binding_index) {
    index_branches_from_binding(source_bindings[binding_index],
                                binding_index == 0);
  }
  constexpr size_t max_fanout_branch_attempts_per_task = 64;
  size_t retry_skip =
      std::min(task.fanout_branch_offset, branch_index.branches.size());
  size_t desired_candidates =
      std::min(branch_index.branches.size(),
               max_fanout_branch_attempts_per_task + retry_skip);
  auto collect_indexed_branch = [&](size_t indexed_branch_number) {
    if (indexed_branch_number >= branch_index.branches.size()) {
      return;
    }
    const FanoutBranchIndexEntry &indexed_branch =
        branch_index.branches[indexed_branch_number];
    if (!indexed_branch.owner ||
        indexed_branch.route_index >= indexed_branch.owner->wires.size()) {
      return;
    }
    std::vector<Wire> &base =
        indexed_branch.owner->wires[indexed_branch.route_index];
    if (indexed_branch.prefix_size == 0 ||
        indexed_branch.prefix_size > base.size() || !indexed_branch.tile ||
        !indexed_branch.tile->cb_type || indexed_branch.dst < 0) {
      return;
    }
    Tile *target_tile = indexed_branch.tile;
    int target_dst = indexed_branch.dst;
    const std::string &target_wire = indexed_branch.dst_wire;
    int free_exits =
        countAvailableForkExits(*target_tile, target_dst, target_wire);
    if (debug_fanout_net) {
      PNR_LOG1(
          "ROUT",
          "routeFanoutTask debug branch candidate: binding='{}', "
          "tile=({},{}) dst={} '{}', free_exits={}, preferred={}, usable={}",
          indexed_branch.route_name, target_tile->coord.x,
          target_tile->coord.y, target_dst, target_wire, free_exits,
          pnr::fanoutBranchIsPreferred(free_exits),
          pnr::fanoutBranchIsUsableFallback(free_exits));
    }
    if (!pnr::fanoutBranchIsUsableFallback(free_exits)) {
      if (RouteCongestionTrace::current)
        RouteCongestionTrace::current->blocked(*target_tile, "fanout_fork_saturated",
            {{fpga::CB_NODE_DST, target_dst}});
      return;
    }
    BranchPoint candidate{
        target_tile,
        target_dst,
        target_wire,
        true,
        routeDistance(target_tile->coord, task.to->tile->coord),
        indexed_branch.owner,
        indexed_branch.route_index,
        indexed_branch.prefix_size};
    if (pnr::fanoutBranchIsPreferred(free_exits)) {
      branches.push_back(std::move(candidate));
    } else {
      candidate.score += 10000;
      fallback_branches.push_back(std::move(candidate));
    }
  };
  const Coord branch_center = task.to->tile->coord;
  std::vector<std::vector<size_t>> branches_by_distance;
  for (size_t indexed_branch_number = 0;
       indexed_branch_number < branch_index.branches.size();
       ++indexed_branch_number) {
    Tile *tile = branch_index.branches[indexed_branch_number].tile;
    if (!tile) {
      continue;
    }
    size_t distance = static_cast<size_t>(
        routeDistance(branch_center, tile->coord));
    if (branches_by_distance.size() <= distance) {
      branches_by_distance.resize(distance + 1);
    }
    branches_by_distance[distance].push_back(indexed_branch_number);
  }
  // Fanout trees are sparse in large devices. Walk only their indexed branch
  // nodes in nearest-distance buckets instead of probing every grid coordinate
  // in expanding square rings. This retains nearest-first ordering without a
  // per-task sort or any work in the recursive numeric route search.
  for (const std::vector<size_t> &distance_bucket : branches_by_distance) {
    for (size_t indexed_branch_number : distance_bucket) {
      collect_indexed_branch(indexed_branch_number);
      if (branches.size() >= desired_candidates) {
        break;
      }
    }
    if (branches.size() >= desired_candidates) {
      break;
    }
  }
  // Exit count is a preference, not a reachability test. Retain lower-capacity
  // forks after the preferred ones so failure/rotation can actually try them.
  // The existing per-task attempt budget still bounds all candidate searches.
  branches.insert(branches.end(),
                  std::make_move_iterator(fallback_branches.begin()),
                  std::make_move_iterator(fallback_branches.end()));
  if (!branches.empty()) {
    size_t rotate_index = task.fanout_branch_offset % branches.size();
    std::rotate(branches.begin(),
                branches.begin() + static_cast<std::ptrdiff_t>(rotate_index),
                branches.end());
  }
  if (debug_fanout_net) {
    PNR_LOG1("ROUT",
             "routeFanoutTask debug: net='{}', from='{}' port='{}', to='{}' "
             "port='{}', complete_source_bindings={}, skipped_self={}, "
             "branches={}, task_net_routes={}",
             task.net_name,
             task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
             task.from_port,
             task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
             task.to_port, source_bindings.size(), skipped_self_bindings,
             branches.size(), task.net ? task.net->routes.size() : 0);
    size_t branch_index = 0;
    for (const BranchPoint &branch : branches) {
      if (branch_index >= 16) {
        break;
      }
      PNR_LOG1("ROUT",
               "routeFanoutTask debug branch[{}]: tile='{}' coord=({},{}) "
               "local={} '{}' dst_wire='{}' score={} shared_prefix={}",
               branch_index,
               branch.tile ? branch.tile->makeName() : std::string{},
               branch.tile ? branch.tile->coord.x : -1,
               branch.tile ? branch.tile->coord.y : -1, branch.local,
               branch.tile
                   ? nodeDebugName(*branch.tile,
                                   branch.start_from_dst ? fpga::CB_NODE_DST
                                                         : fpga::CB_NODE_LOCAL,
                                   branch.local)
                   : std::string{},
               branch.dst_wire, branch.score, branch.prefix_size);
      ++branch_index;
    }
    size_t binding_index = 0;
    for (const SourceRouteBinding &source_binding : source_bindings) {
      if (binding_index >= 4 || !source_binding.binding ||
          !source_binding.binding->owner ||
          source_binding.binding->route_index >=
              source_binding.binding->owner->wires.size()) {
        break;
      }
      const std::vector<Wire> &base =
          source_binding.binding->owner
              ->wires[source_binding.binding->route_index];
      PNR_LOG1("ROUT",
               "routeFanoutTask debug source_binding[{}]: net='{}' owner='{}' "
               "route_name='{}' fragments={} complete={} xbars={}",
               binding_index,
               source_binding.net
                   ? source_binding.net->makeName(FULL_NAME_LIMIT)
                   : std::string{},
               source_binding.binding->owner->makeName(FULL_NAME_LIMIT),
               source_binding.binding->route_name, base.size(),
               routeIsComplete(base), routeCrossbarFragments(&base));
      for (size_t fragment_index = 0;
           fragment_index < std::min<size_t>(base.size(), 12);
           ++fragment_index) {
        const Wire &fragment = base[fragment_index];
        PNR_LOG1("ROUT",
                 "routeFanoutTask debug source_binding[{}].fragment[{}]: "
                 "type={} from=({},{}) to=({},{}) local={} jump={} "
                 "route_jump={} dst={} joint={} pos={} from_wire='{}' src='{}' "
                 "dst_wire='{}' resource=({},{}) cell='{}' port='{}'",
                 binding_index, fragment_index,
                 fragment.type == Wire::WIRE_TILE_PIN ? "tile_pin" : "crossbar",
                 fragment.from.x, fragment.from.y, fragment.to.x, fragment.to.y,
                 fragment.local, fragment.jump, fragment.route_jump,
                 fragment.dst, fragment.joint, fragment.pos,
                 fragment.from_wire_name, fragment.src_wire_name,
                 fragment.dst_wire_name, fragment.resource.x,
                 fragment.resource.y, fragment.cell_type, fragment.port);
      }
      ++binding_index;
    }
  }
  if (std::getenv("SCALEPNR_DEBUG_FANOUT") && branches.empty()) {
    PNR_LOG1("ROUT",
             "routeFanoutTask debug: no branch points for net='{}', from='{}' "
             "port='{}', to='{}' port='{}', complete_source_bindings={}, "
             "skipped_self={}, task_net_routes={}",
             task.net_name,
             task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
             task.from_port,
             task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
             task.to_port, source_bindings.size(), skipped_self_bindings,
             task.net ? task.net->routes.size() : 0);
  }

  std::vector<Tile *> to_route_tiles =
      routeTileCandidates(*task.to, task.to_port, false);
  if (!to_route_tiles.empty()) {
    size_t branch_rotation =
        branches.empty()
            ? task.fanout_branch_offset
            : task.fanout_branch_offset / std::max<size_t>(1, branches.size());
    std::rotate(to_route_tiles.begin(),
                to_route_tiles.begin() +
                    static_cast<std::ptrdiff_t>(branch_rotation %
                                                to_route_tiles.size()),
                to_route_tiles.end());
  }
  if (debug_fanout_net) {
    PNR_LOG1("ROUT", "routeFanoutTask debug: destination route candidates={}",
             to_route_tiles.size());
    size_t candidate_index = 0;
    for (Tile *to_route_tile : to_route_tiles) {
      if (candidate_index >= 16) {
        break;
      }
      NodeMask pin_nodes =
          to_route_tile
              ? routeTileInputNodes(*to_route_tile, *task.to, task.to_port)
              : NodeMask{};
      PNR_LOG1(
          "ROUT",
          "routeFanoutTask debug to[{}]: tile='{}' coord=({},{}) pin_nodes={}",
          candidate_index,
          to_route_tile ? to_route_tile->makeName() : std::string{},
          to_route_tile ? to_route_tile->coord.x : -1,
          to_route_tile ? to_route_tile->coord.y : -1, pin_nodes.str());
      ++candidate_index;
    }
  }
  if (attachSharedDestinationLocalFanout(task, branch_index, to_route_tiles)) {
    route_changed = true;
    route_progress = true;
    route_complete = true;
    ++route_stats.completed;
    ++route_stats.new_completed;
    --route_recursion_budget;
    return true;
  }
  ++route_stats.new_attempts;

  size_t branch_target_pairs = branches.size() * to_route_tiles.size();
  int fanout_attempt_budget = static_cast<int>(std::min<size_t>(
      branch_target_pairs, max_fanout_branch_attempts_per_task));
  int fanout_attempts_used = 0;

  auto branch_prefix = [](const BranchPoint &branch)
      -> const std::vector<Wire> * {
    if (!branch.prefix_owner ||
        branch.prefix_route_index >= branch.prefix_owner->wires.size()) {
      return nullptr;
    }
    const std::vector<Wire> &prefix_route =
        branch.prefix_owner->wires[branch.prefix_route_index];
    if (branch.prefix_size == 0 || branch.prefix_size > prefix_route.size() ||
        !routeIsComplete(prefix_route)) {
      return nullptr;
    }
    return &prefix_route;
  };

  auto commit_fanout_route = [&](const BranchPoint &branch,
                                 std::vector<Wire> &wire,
                                 bool attempt_complete) {
    const std::vector<Wire> *prefix_route = branch_prefix(branch);
    if (!prefix_route) {
      return false;
    }
    if (branch.prefix_size != 0) {
      std::vector<Wire> full_route;
      full_route.reserve(branch.prefix_size + wire.size());
      full_route.insert(
          full_route.end(), prefix_route->begin(),
          prefix_route->begin() + static_cast<std::ptrdiff_t>(branch.prefix_size));
      for (Wire &prefix_fragment : full_route) {
        prefix_fragment.shared = true;
      }
      full_route.insert(full_route.end(), std::make_move_iterator(wire.begin()),
                        std::make_move_iterator(wire.end()));
      wire = std::move(full_route);
    }
    for (Wire &fragment : wire) {
      fragment.net_name = task.net_name;
    }
    rtl::NetRouteBinding *route_binding =
        findNetRouteBinding(*task.net, *task.from, *task.to, task.from_port,
                            task.to_port, task.net_name);
    std::vector<Wire> *route_slot =
        route_binding ? routeBindingRoute(*route_binding) : nullptr;
    if (route_slot) {
      if (routeStartsWithSharedPrefix(route_slot)) {
        discardRouteBranchSuffix(*route_slot);
      } else {
        releaseRouteFragments(*route_slot);
      }
      *route_slot = std::move(wire);
    } else {
      task.to->wires.emplace_back(std::move(wire));
      route_slot = &task.to->wires.back();
    }
    size_t route_index = findRouteIndex(*task.to, route_slot);
    PNR_ASSERT(
        route_index != std::numeric_limits<size_t>::max(),
        "Fanout route '{}' was committed outside destination route storage",
        task.net_name);
    size_t binding_index = fpga::attachNetRoute(
        *task.net, *task.to, route_index, task.from, task.to, task.from_port,
        task.to_port, task.net_name);
    fpga::registerNetRouteTiles(*task.net, *route_slot, binding_index);
    route_changed = true;
    route_progress = true;
    route_complete = attempt_complete;
    if (route_complete) {
      task.fanout_branch_attempt = 0;
    } else {
      task.fanout_branch_attempt = 0;
    }
    if (debug_fanout_net) {
      PNR_LOG1("ROUT",
               "routeFanoutTask debug accepted route: net='{}', to='{}'/'{}', "
               "complete={} wire_size={} first_from=({},{}) last_to=({},{})",
               task.net_name,
               task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
               task.to_port, route_complete, route_slot->size(),
               route_slot->empty() ? -1 : route_slot->front().from.x,
               route_slot->empty() ? -1 : route_slot->front().from.y,
               route_slot->empty() ? -1 : route_slot->back().to.x,
               route_slot->empty() ? -1 : route_slot->back().to.y);
    }
    if (route_complete) {
      ++route_stats.completed;
      ++route_stats.new_completed;
    } else {
      ++route_stats.partial_started;
      ++route_stats.new_partial;
    }
    return true;
  };

  for (size_t branch_index = 0; branch_index < branches.size();
       ++branch_index) {
    const BranchPoint &branch = branches[branch_index];
    for (size_t target_index = 0; target_index < to_route_tiles.size();
         ++target_index) {
      Tile *to_route_tile = to_route_tiles[target_index];
      if (fanout_attempt_budget <= 0) {
        break;
      }
      --fanout_attempt_budget;
      if (!branch.tile || !to_route_tile) {
        continue;
      }
      if (!branch_prefix(branch)) {
        continue;
      }
      ++fanout_attempts_used;
      std::vector<Wire> wire;
      bool attempt_complete = false;
      NodeMask pin_nodes =
          to_route_tile
              ? routeTileInputNodes(*to_route_tile, *task.to, task.to_port)
              : NodeMask{};
      if (debug_fanout_net) {
        PNR_LOG1(
            "ROUT",
            "routeFanoutTask debug attempt: branch_tile=({},{}) "
            "branch_local={} '{}' to_tile=({},{}) pin_nodes={} iter_budget={}",
            branch.tile ? branch.tile->coord.x : -1,
            branch.tile ? branch.tile->coord.y : -1, branch.local,
            branch.tile
                ? nodeDebugName(*branch.tile,
                                branch.start_from_dst ? fpga::CB_NODE_DST
                                                      : fpga::CB_NODE_LOCAL,
                                branch.local)
                : std::string{},
            to_route_tile ? to_route_tile->coord.x : -1,
            to_route_tile ? to_route_tile->coord.y : -1, pin_nodes.str(),
            fanout_attempt_budget);
      }
      int fanout_distance =
          routeDistance(branch.tile->coord, to_route_tile->coord);
      int fanout_depth_limit = std::max(iteration_limit, fanout_distance + 4);
      fanout_depth_limit =
          std::max(fanout_depth_limit, fanout_distance * 2 + 8);
      fanout_depth_limit = std::min(fanout_depth_limit, 64);
      // Fanout grounding may replace a blocking transit tree when the
      // destination has no free physically incoming node; the victim tree is
      // requeued atomically.
      bool allow_preempt = route_preemption_enabled &&
                           fanout_preemption_enabled &&
                           pnr::movingRouteMayPreempt(
                               moving_stage, moving_focus_inst != nullptr,
                               moving_sources_stage);
      if (!tryBestFirstRoute(*branch.tile, *to_route_tile, branch.local,
                             *task.to, task.to_port, wire, fanout_depth_limit,
                             branch.start_from_dst, branch.dst_wire,
                             &attempt_complete, &route_stats, this, task.net,
                             true, task.from, task.from_port, task.net_name,
                             pin_nodes, true, debug_fanout_net, nullptr,
                             allow_preempt, allow_preempt && task.fanout)) {
        if (debug_fanout_net) {
          PNR_LOG1("ROUT",
                   "routeFanoutTask debug attempt failed: branch_tile=({},{}) "
                   "branch_local={} to_tile=({},{})",
                   branch.tile ? branch.tile->coord.x : -1,
                   branch.tile ? branch.tile->coord.y : -1, branch.local,
                   to_route_tile ? to_route_tile->coord.x : -1,
                   to_route_tile ? to_route_tile->coord.y : -1);
        }
        continue;
      }
      if (wire.empty()) {
        if (debug_fanout_net) {
          PNR_LOG1("ROUT", "routeFanoutTask debug attempt produced empty wire");
        }
        continue;
      }
      if (!commit_fanout_route(branch, wire, attempt_complete)) {
        releaseRouteFragments(wire);
        continue;
      }
      --route_recursion_budget;
      return attempt_complete;
    }
    if (fanout_attempt_budget <= 0) {
      break;
    }
  }

  if (branches.empty()) {
    ++route_stats.new_empty;
  }

  ++route_stats.failed;
  ++route_stats.new_failed;
  const size_t attempt_before_failure = task.attempt;
  ++task.attempt;
  task.fanout_branch_offset += std::max(1, fanout_attempts_used);
  if (pnr::movingFanoutNeedsSourceTreeRebuild(
          moving_stage, moving_focus_inst != nullptr,
          task.source_tree_rebuild_attempted, task.fanout,
          sourceTreeHasCompleteExit(*task.from, task.from_port), false, false,
          attempt_before_failure, task.attempt)) {
    // The moved sink has tested every currently exposed branch point. Rebuild
    // the physical source tree toward this sink instead of moving it again.
    std::vector<RouteTask> recovered;
    size_t invalidated = unrouteSourceTree(
        *task.net, task.from, task.from_port, &recovered, true, true);
    if (invalidated != 0) {
      size_t queued = pnr::scheduleMovingSourceTreeRebuild(
          task, recovered,
          [](const RouteTask &left, const RouteTask &right) {
            return sameRouteTask(left, right);
          },
          [&](const RouteTask &sibling) {
            return enqueueRouteTask(sibling, pending_route_todo);
          });
      task.source_tree_rebuilt = true;
      route_changed = true;
      PNR_LOG1("ROUT",
               "routeDesign moving: exhausted fanout net='{}' rebuilt "
               "source='{}'/'{}' as Generic seed, invalidated={}, siblings={}",
               task.net_name, task.from->makeName(FULL_NAME_LIMIT),
               task.from_port, invalidated, queued);
    }
  }
  --route_recursion_budget;
  return false;
}

// Prepare physical passthrough endpoints and move every affected source-tree
// binding and queued task together, preserving one Generic seed for rerouting.
bool RouteDesign::canonicalizeRouteTaskSource(RouteTask &task) {
  std::pair<rtl::Inst *, std::string> endpoint{task.from, task.from_port};
  bool changed = pnr::resolveSourceRetarget(
      endpoint, source_endpoint_retargets,
      [](const auto &value) { return sourceRouteKey(value.first, value.second); });
  if (changed) {
    task.from = endpoint.first;
    task.from_port = endpoint.second;
  }
  return changed;
}

bool RouteDesign::prepareRouteTaskEndpoints(RouteTask &task,
                                             bool allow_new_source_passthrough) {
  canonicalizeRouteTaskSource(task);
  const bool preserve_generic_seed =
      allow_new_source_passthrough && !task.fanout;
  rtl::Inst *original_from = task.from;
  rtl::Inst *original_to = task.to;
  rtl::Net *original_net = task.net;
  std::string original_from_port = task.from_port;
  std::string original_to_port = task.to_port;
  bool changed = fpga::preparePassthroughRouteEndpoints(
      task.from, task.from_port, task.to, task.to_port, task.net,
      allow_new_source_passthrough);
  bool source_changed =
      original_from != task.from || original_from_port != task.from_port;
  if (source_changed && original_from && original_net && task.from) {
    std::vector<RouteTask> recovered;
    unrouteSourceTree(*original_net, original_from, original_from_port,
                      &recovered, false);

    const std::string original_source_key =
        sourceRouteKey(original_from, original_from_port);
    const std::string replacement_source_key =
        sourceRouteKey(task.from, task.from_port);
    std::vector<rtl::Net *> affected_nets;
    auto indexed = source_route_nets.find(original_source_key);
    if (indexed != source_route_nets.end()) {
      affected_nets = std::move(indexed->second);
      source_route_nets.erase(indexed);
    }
    if (std::find(affected_nets.begin(), affected_nets.end(), original_net) ==
        affected_nets.end()) {
      affected_nets.push_back(original_net);
    }
    std::vector<rtl::Net *> &replacement_nets =
        source_route_nets[replacement_source_key];
    for (rtl::Net *affected_net : affected_nets) {
      if (!affected_net) {
        continue;
      }
      fpga::retargetNetRouteSourceBindings(
          *affected_net, original_from, original_from_port, task.from,
          task.from_port);
      if (std::find(replacement_nets.begin(), replacement_nets.end(),
                    affected_net) == replacement_nets.end()) {
        replacement_nets.push_back(affected_net);
      }
    }
    source_endpoint_retargets[original_source_key] =
        std::pair<rtl::Inst *, std::string>{task.from, task.from_port};

    auto retarget_task_source = [&](RouteTask &queued) {
      if (queued.from == original_from &&
          queued.from_port == original_from_port) {
        queued.from = task.from;
        queued.from_port = task.from_port;
        queued.endpoints_prepared = false;
      }
    };
    bool current_recovered = false;
    bool recovered_has_generic = false;
    for (RouteTask &recovered_task : recovered) {
      retarget_task_source(recovered_task);
      recovered_has_generic = recovered_has_generic || !recovered_task.fanout;
      if (recovered_task.net_name == task.net_name &&
          recovered_task.to == original_to &&
          recovered_task.to_port == original_to_port) {
        task.fanout = pnr::retargetedCurrentTaskIsFanout(
            preserve_generic_seed, true, recovered_task.fanout,
            recovered_has_generic);
        current_recovered = true;
        continue;
      }
      recovered_task.fanout = pnr::retargetedSiblingTaskIsFanout(
          preserve_generic_seed, recovered_task.fanout);
      enqueueRouteTask(recovered_task, pending_route_todo);
    }
    if (!current_recovered) {
      task.fanout = pnr::retargetedCurrentTaskIsFanout(
          preserve_generic_seed, false, false, recovered_has_generic);
    }
  }
  if (task.to) {
    for (rtl::Conn &conn : task.to->conns) {
      if (!conn.port_ref.peer || conn.port_ref->makeName() != task.to_port) {
        continue;
      }
      rtl::Net *current_net =
          findNetByDesignator(*task.to, conn.port_ref->designator);
      if (current_net) {
        task.net = current_net;
      }
      break;
    }
  }
  bool identity_changed =
      original_from != task.from || original_to != task.to ||
      original_from_port != task.from_port ||
      original_to_port != task.to_port || original_net != task.net;
  if (identity_changed && original_net && task.net) {
    fpga::retargetNetRouteBindings(
        *original_net, *task.net, source_changed ? task.from : original_from,
        original_to, source_changed ? task.from_port : original_from_port,
        original_to_port, task.from, task.to, task.from_port, task.to_port,
        task.net_name);
  }
  return changed || identity_changed;
}

// Route one task whose source is a database-declared local capability. Const
// routing owns retries and bindings; ordinary routing never receives it.
bool RouteDesign::routeDistributedLocalTask(RouteTask &task) {
  PNR_ASSERT(task.from && task.to && task.net && task.net->distributed_source,
             "distributed local route has invalid task identity");
  task.distributed_one = task.net->distributed_one;
  ++route_stats.task_attempts;
  bool debug_task = routeTaskDebugMatches(task);
  auto distributed_roots = [&](const Tile &tile) -> const NodeMask & {
    return task.distributed_one ? tile.cb_type->constant_one_nodes
                                : tile.cb_type->constant_zero_nodes;
  };

  auto lookup_start = std::chrono::steady_clock::now();
  rtl::NetRouteBinding *existing_binding = findNetRouteBinding(
      *task.net, *task.from, *task.to, task.from_port, task.to_port,
      task.net_name);
  std::vector<Wire> *existing =
      existing_binding ? routeBindingRoute(*existing_binding) : nullptr;
  if (existing && routeIsComplete(*existing)) {
    route_stats.distributed_lookup_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - lookup_start)
            .count());
    ++route_stats.already_complete;
    return true;
  }
  // A distributed source has no generic routed prefix to continue. Moving may
  // leave a shared-only remnant, so discard that binding and rebuild locally.
  if (existing && !existing->empty()) {
    size_t binding_index = static_cast<size_t>(
        existing_binding - task.net->routes.data());
    PNR_ASSERT(fpga::unrouteNetRoute(*task.net, binding_index),
               "failed to reset incomplete distributed route '{}'",
               task.net_name);
    route_changed = true;
  }
  route_stats.distributed_lookup_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - lookup_start)
          .count());
  if (!task.to->tile.peer || !task.to->cell_ref.peer) {
    if (debug_task) {
      PNR_LOG1("ROUT",
               "distributed route debug: net='{}' has no placed sink or cell "
               "type, sink='{}'/'{}'",
               task.net_name, task.to->makeName(FULL_NAME_LIMIT),
               task.to_port);
    }
    ++route_stats.failed;
    return false;
  }

  Tile &resource_tile = *task.to->tile;
  Tile *direct_route_tile = fpga::Device::current().routeTile(resource_tile);
  Tile *route_tile = nullptr;
  NodeMask targets;
  int target = -1;
  bool pin_shared = false;
  std::vector<LocalRouteNode> path;
  size_t route_candidate_count = 0;
  auto endpoint_start = std::chrono::steady_clock::now();
  auto find_local_path = [&](Tile &candidate_tile, int candidate,
                             bool allow_preemptible = false) {
    size_t roots = 0;
    size_t visited = 0;
    auto path_start = std::chrono::steady_clock::now();
    std::vector<LocalRouteNode> result = findDistributedLocalPath(
        candidate_tile, candidate, *task.net,
        distributed_roots(candidate_tile), allow_preemptible, &roots,
        &visited);
    route_stats.distributed_path_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - path_start)
            .count());
    ++route_stats.distributed_path_calls;
    route_stats.distributed_path_roots += roots;
    route_stats.distributed_path_nodes += visited;
    return result;
  };
  auto try_route_tile = [&](Tile *candidate_tile) {
    if (!candidate_tile || !candidate_tile->cb_type ||
        distributed_roots(*candidate_tile) == NodeMask{}) {
      return false;
    }
    ++route_candidate_count;
    NodeMask candidate_targets =
        routeTileInputNodes(*candidate_tile, *task.to, task.to_port);
    if (candidate_targets == NodeMask{} && candidate_tile == direct_route_tile) {
      candidate_targets = resource_tile.getPinNodes(
          task.to->cell_ref->type, task.to_port, task.to->pos);
    }
    if (candidate_targets == NodeMask{}) {
      return false;
    }
    if (!route_tile) {
      route_tile = candidate_tile;
      targets = candidate_targets;
    }
    int candidate_target = -1;
    bool candidate_shared = false;
    std::vector<LocalRouteNode> candidate_path;
    candidate_targets.for_each_set_bit([&](int candidate) {
      bool pin_busy = candidate_tile->isPinNodeLeased(candidate);
      LocalRouteNode local{fpga::CB_NODE_LOCAL, candidate};
      if (pin_busy &&
          !localRouteNodeOwnedByNet(*candidate_tile, local, *task.net)) {
        return false;
      }
      std::vector<LocalRouteNode> local_path =
          find_local_path(*candidate_tile, candidate);
      if (local_path.empty()) {
        return false;
      }
      candidate_target = candidate;
      candidate_shared = pin_busy;
      candidate_path = std::move(local_path);
      return true;
    });
    if (candidate_path.empty()) {
      return false;
    }
    route_tile = candidate_tile;
    targets = candidate_targets;
    target = candidate_target;
    pin_shared = candidate_shared;
    path = std::move(candidate_path);
    return true;
  };

  // Distributed roots normally terminate inside the sink's canonical route
  // tile. Construct nearby endpoint candidates only when this direct numeric
  // endpoint is absent or blocked.
  if (!try_route_tile(direct_route_tile)) {
    for (Tile *candidate_tile :
         routeTileCandidates(*task.to, task.to_port, false)) {
      if (candidate_tile == direct_route_tile) {
        continue;
      }
      if (try_route_tile(candidate_tile)) {
        break;
      }
    }
  }
  if (!route_tile) {
    route_stats.distributed_endpoint_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - endpoint_start)
            .count());
    if (debug_task) {
      PNR_LOG1(
          "ROUT",
          "distributed route debug: net='{}' sink='{}'/'{}' resource=({},{}) "
          "has no route candidate with distributed source and input nodes; "
          "candidate_count={}",
          task.net_name, task.to->makeName(FULL_NAME_LIMIT), task.to_port,
          resource_tile.coord.x, resource_tile.coord.y,
          route_candidate_count);
    }
    ++route_stats.failed;
    return false;
  }

  if (debug_task) {
    PNR_LOG1(
        "ROUT",
        "distributed route debug: net='{}' sink='{}' type='{}' port='{}' "
        "resource=({},{})/{} route=({},{}) cb='{}' roots={} targets={}",
        task.net_name, task.to->makeName(FULL_NAME_LIMIT),
        task.to->cell_ref->type, task.to_port, resource_tile.coord.x,
        resource_tile.coord.y, task.to->pos, route_tile->coord.x,
        route_tile->coord.y, route_tile->cb_type->name,
        distributed_roots(*route_tile).str(), targets.str());
  }
  if (path.empty() && moving_stage) {
    // Distributed roots are mandatory local capabilities. If ordinary routing
    // still consumes every legal local path after placement recovery begins,
    // select one whose complete owner set can be atomically returned to
    // Generic/Fanout scheduling. Basic never destabilizes a completed trunk.
    std::vector<DistributedLocalVictim> victims;
    auto select_preemptible_path = [&]() {
      targets.for_each_set_bit([&](int candidate) {
        bool pin_busy = route_tile->isPinNodeLeased(candidate);
        LocalRouteNode local{fpga::CB_NODE_LOCAL, candidate};
        if (pin_busy &&
            !distributedLocalNodeAvailable(*route_tile, local, *task.net,
                                           true)) {
          return false;
        }
        std::vector<LocalRouteNode> candidate_path =
            find_local_path(*route_tile, candidate, true);
        std::vector<DistributedLocalVictim> candidate_victims;
        if (candidate_path.empty() ||
            !distributedLocalPathVictims(*route_tile, candidate_path,
                                         candidate, *task.net,
                                         candidate_victims)) {
          return false;
        }
        target = candidate;
        path = std::move(candidate_path);
        victims = std::move(candidate_victims);
        return true;
      });
      return !path.empty();
    };
    // Endpoint routes retain their terminal resources. If no transit-only
    // path exists, Moving relocates this distributed task's sink instead.
    select_preemptible_path();

    if (!path.empty() && !victims.empty()) {
      struct SourceTree {
        rtl::Net *net = nullptr;
        rtl::Inst *from = nullptr;
        std::string from_port;
      };
      std::vector<SourceTree> source_trees;
      for (const DistributedLocalVictim &victim : victims) {
        const rtl::NetRouteBinding &binding =
            victim.route.net->routes[victim.route.binding_index];
        if (!binding.from || binding.from_port.empty()) {
          path.clear();
          break;
        }
        bool duplicate = std::any_of(
            source_trees.begin(), source_trees.end(),
            [&](const SourceTree &tree) {
              return tree.from == binding.from &&
                     tree.from_port == binding.from_port;
            });
        if (!duplicate) {
          source_trees.push_back(
              SourceTree{victim.route.net, binding.from, binding.from_port});
        }
      }

      // Detach the suffix beginning at the exact blocking local/joint first.
      // This preserves a live trunk and sibling branches whenever ownership
      // permits a branch-local release.
      bool suffix_release_failed = false;
      if (!path.empty()) {
        route_stats.preempt_attempts += victims.size();
        for (const DistributedLocalVictim &victim : victims) {
          rtl::NetRouteBinding binding =
              victim.route.net->routes[victim.route.binding_index];
          PNR_LOG1(
              "ROUT",
              "routeDesign distributed local preempt: current='{}', "
              "tile=({},{}), node_type={}, node={}, victim='{}', endpoint={}",
              task.net_name, route_tile->coord.x, route_tile->coord.y,
              static_cast<int>(victim.node.type), victim.node.value,
              binding.route_name, bindingToTile(binding, *route_tile));
          if (!fpga::unrouteNetRouteFromNode(
                  *victim.route.net, victim.route.binding_index,
                  route_tile->coord, victim.node.type, victim.node.value)) {
            suffix_release_failed = true;
            continue;
          }
          ++route_stats.preempt_success;
          bool fanout = binding.from &&
                        sourceTreeHasCompleteExit(*binding.from,
                                                  binding.from_port);
          enqueueRouteTask(RouteTask{binding.from,
                                     binding.to,
                                     victim.route.net,
                                     binding.from_port,
                                     binding.to_port,
                                     binding.route_name,
                                     0,
                                     0,
                                     0,
                                     {},
                                     fanout},
                           pending_route_todo);
        }
      }

      path = find_local_path(*route_tile, target);
      if (path.empty() || suffix_release_failed) {
        // Shared ownership can make an exact suffix indivisible. Fall back to
        // the established atomic source-tree operation only for those rare
        // cases, preserving route lease consistency.
        std::vector<RouteTask> reroute_tasks;
        route_stats.preempt_attempts += source_trees.size();
        for (const SourceTree &tree : source_trees) {
          size_t removed = unrouteSourceTree(*tree.net, tree.from,
                                             tree.from_port, &reroute_tasks,
                                             false);
          if (removed != 0) {
            ++route_stats.preempt_success;
          }
        }
        for (RouteTask &reroute_task : reroute_tasks) {
          enqueueRouteTask(reroute_task, pending_route_todo);
        }
        path = find_local_path(*route_tile, target);
      }

      // Re-read the numeric path after releasing its owners; this also catches
      // any shared owner that kept a physical lease alive.
      if (!path.empty()) {
        pin_shared = route_tile->isPinNodeLeased(target) &&
                     localRouteNodeOwnedByNet(
                         *route_tile,
                         LocalRouteNode{fpga::CB_NODE_LOCAL, target},
                         *task.net);
      }
    }
  }
  route_stats.distributed_endpoint_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - endpoint_start)
          .count());
  if (path.empty()) {
    if (debug_task) {
      PNR_LOG1("ROUT",
               "distributed route debug: net='{}' found no free numeric "
               "local path from roots={} to targets={} in route tile "
               "({},{}) cb='{}'",
               task.net_name,
               distributed_roots(*route_tile).str(), targets.str(),
               route_tile->coord.x, route_tile->coord.y,
               route_tile->cb_type->name);
    }
    ++route_stats.failed;
    return false;
  }

  auto commit_start = std::chrono::steady_clock::now();
  std::vector<LocalRouteNode> newly_leased;
  std::vector<bool> shared(path.size(), false);
  for (size_t index = 1; index < path.size(); ++index) {
    const LocalRouteNode &node = path[index];
    NodeMask bit = NodeMask{0, 1} << node.value;
    NodeMask &state = leasedLocalRouteNodes(*route_tile, node.type);
    if ((state & bit) != NodeMask{}) {
      if (!localRouteNodeOwnedByNet(*route_tile, node, *task.net)) {
        for (const LocalRouteNode &leased : newly_leased) {
          leasedLocalRouteNodes(*route_tile, leased.type) &=
              ~(NodeMask{0, 1} << leased.value);
        }
        ++route_stats.failed;
        return false;
      }
      shared[index] = true;
      continue;
    }
    state |= bit;
    newly_leased.push_back(node);
  }
  if (!pin_shared && !route_tile->leasePinNode(target)) {
    for (const LocalRouteNode &leased : newly_leased) {
      leasedLocalRouteNodes(*route_tile, leased.type) &=
          ~(NodeMask{0, 1} << leased.value);
    }
    ++route_stats.failed;
    return false;
  }

  std::vector<Wire> route;
  route.reserve(path.size());
  for (size_t index = 1; index < path.size(); ++index) {
    Wire edge;
    edge.type = Wire::WIRE_ROUTE_EDGE;
    edge.from = route_tile->coord;
    edge.to = route_tile->coord;
    edge.from_node_type = path[index - 1].type;
    edge.from_node = path[index - 1].value;
    edge.to_node_type = path[index].type;
    edge.to_node = path[index].value;
    edge.shared = shared[index];
    edge.net_name = task.net_name;
    route.push_back(std::move(edge));
  }

  Wire pin;
  pin.type = Wire::WIRE_TILE_PIN;
  pin.from = route_tile->coord;
  pin.to = route_tile->coord;
  pin.resource = resource_tile.coord;
  pin.local = target;
  pin.resource_node = resource_tile.getResourceNodeNum(
      task.to->cell_ref->type, task.to_port, task.to->pos,
      fpga::TILE_PIN_INPUT, target);
  pin.pin_dir = fpga::TILE_PIN_INPUT;
  pin.pos = task.to->pos;
  pin.cell_type = task.to->cell_ref->type;
  pin.port = task.to_port;
  pin.net_name = task.net_name;
  route.push_back(std::move(pin));

  task.to->wires.push_back(std::move(route));
  size_t route_index = task.to->wires.size() - 1;
  auto attach_start = std::chrono::steady_clock::now();
  size_t binding_index = fpga::attachNetRoute(
      *task.net, *task.to, route_index, task.from, task.to, task.from_port,
      task.to_port, task.net_name);
  fpga::registerNetRouteTiles(*task.net, task.to->wires.back(), binding_index);
  route_stats.task_attach_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - attach_start)
          .count());
  route_stats.distributed_commit_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - commit_start)
          .count());
  route_changed = true;
  route_progress = true;
  ++route_stats.completed;
  ++route_stats.new_completed;
  return true;
}

// Rebuild constant branches invalidated by a placement change before the move
// is exposed to ordinary routing. Completed constants are removed in place.
bool RouteDesign::routeConstantTasksImmediately(
    std::vector<RouteTask> &tasks, std::string *fail_reason) {
  bool saved_moving_stage = moving_stage;
  bool saved_preemption = route_preemption_enabled;
  moving_stage = false;
  route_preemption_enabled = false;
  bool success = true;
  for (RouteTask &task : tasks) {
    if (!task.net || !task.net->distributed_source) {
      continue;
    }
    task.fanout = false;
    task.distributed_one = task.net->distributed_one;
    if (!routeDistributedLocalTask(task)) {
      success = false;
      if (fail_reason) {
        *fail_reason = "constant reroute failed for '" + task.net_name + "'";
      }
      break;
    }
  }
  route_preemption_enabled = saved_preemption;
  moving_stage = saved_moving_stage;
  if (success) {
    std::erase_if(tasks, [](const RouteTask &task) {
      return task.net && task.net->distributed_source;
    });
  }
  return success;
}

// Preserve chronological failure evidence without affecting search or leases.
// Queue merges carry these fields to the eventual terminal diagnostic.
void RouteDesign::recordRouteTaskFailure(RouteTask &task, Coord coord,
                                         fpga::CBNodeNameType type, int node) {
  task.failure_coord = coord;
  task.failure_node_type = type;
  task.failure_node = node;
  task.failure_sequence = ++failure_sequence;
}

// Record the numeric tile where a failed task most recently exhausted routing.
// Search diagnostics take precedence over the committed partial-route tail.
void RouteDesign::rememberRouteTaskFailure(RouteTask &task,
                                           const std::vector<Wire> *route) {
  auto valid = [&](fpga::Coord coord) {
    return fpga && coord.x >= 0 && coord.y >= 0 &&
           coord.x < fpga->size_width && coord.y < fpga->size_height;
  };
  if (valid(route_stats.exhausted_coord)) {
    recordRouteTaskFailure(task, route_stats.exhausted_coord,
                          route_stats.exhausted_type, route_stats.exhausted_node);
    return;
  }
  if (!route_stats.has_last_busy && route_stats.has_last_no_src &&
      valid(route_stats.last_no_src_coord)) {
    recordRouteTaskFailure(task, route_stats.last_no_src_coord,
                          route_stats.last_no_src_type, route_stats.last_no_src_local);
    return;
  }
  if (route_stats.has_last_busy && valid(route_stats.last_busy_coord)) {
    recordRouteTaskFailure(task, route_stats.last_busy_coord,
                          fpga::CB_NODE_SRC, route_stats.last_busy_src);
    return;
  }
  if (route) {
    for (auto fragment = route->rbegin(); fragment != route->rend();
         ++fragment) {
      if (valid(fragment->to)) {
        recordRouteTaskFailure(task, fragment->to, fpga::CB_NODE_DST,
                              fragment->dst);
        return;
      }
      if (valid(fragment->from)) {
        recordRouteTaskFailure(task, fragment->from, fpga::CB_NODE_LOCAL,
                              fragment->local);
        return;
      }
    }
  }
  if (task.to && task.to->tile.peer && valid(task.to->tile->coord)) {
    recordRouteTaskFailure(task, task.to->tile->coord, fpga::CB_NODE_LOCAL, -1);
  } else if (task.from && task.from->tile.peer &&
             valid(task.from->tile->coord)) {
    recordRouteTaskFailure(task, task.from->tile->coord, fpga::CB_NODE_LOCAL, -1);
  }
}

bool RouteDesign::routeNetTask(RouteTask &task, int depth) {
  canonicalizeRouteTaskSource(task);
  TaskCongestionDiagnostic diagnostic(*this, task, __func__);
  PNR_ASSERT(task.from && task.to,
             "routeNetTask got null endpoint for net '{}'", task.net_name);
  route_stats.has_last_busy = false;
  route_stats.has_last_no_src = false;
  route_stats.exhausted_coord = {-1, -1};
  route_stats.exhausted_node = -1;
  rememberRouteTaskFailure(task);
  indexSourceRoute(task.net, task.from, task.from_port);
  if (task.net && task.net->distributed_source) {
    bool complete = routeDistributedLocalTask(task);
    if (!complete) {
      rememberRouteTaskFailure(task);
    }
    return complete;
  }
  auto passthrough_start = std::chrono::steady_clock::now();
  if (!task.endpoints_prepared) {
    if (prepareRouteTaskEndpoints(task, !task.fanout)) {
      logRouteTaskDecision(
          "task.passthrough", task,
          "inserted or reused tile-local passthrough before routing");
    }
    // Placement and endpoint identity remain stable across bounded retries.
    // Moving creates fresh tasks after relocating or rehoming an endpoint.
    task.endpoints_prepared = true;
  }
  // Passthrough insertion may retarget the physical source endpoint.
  indexSourceRoute(task.net, task.from, task.from_port);
  route_stats.task_passthrough_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - passthrough_start)
          .count());
  if (task.net && netEndpointIsVoid(*task.net, task.to, task.to_port)) {
    ++route_stats.already_complete;
    return true;
  }
  if (task.fanout) {
    bool complete = routeFanoutTask(task, depth);
    if (!complete) {
      std::vector<Wire> *route = findBoundRoute(
          task.net, task.from, task.to, task.from_port, task.to_port,
          task.net_name);
      rememberRouteTaskFailure(task, route);
    }
    return complete;
  }
  ++route_stats.task_attempts;
  PNR_LOG3_("ROUT", depth,
            "routeNetTask, net: '{}', from: '{}' port '{}', to: '{}' port '{}'",
            task.net_name, task.from->makeName(), task.from_port,
            task.to->makeName(), task.to_port);

  auto find_start = std::chrono::steady_clock::now();
  std::vector<Wire> *existing_route =
      findBoundRoute(task.net, task.from, task.to, task.from_port, task.to_port,
                     task.net_name);
  bool existing_complete = existing_route && routeIsComplete(*existing_route);
  route_stats.task_find_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - find_start)
          .count());
  if (existing_complete) {
    if (task.net) {
      auto attach_start = std::chrono::steady_clock::now();
      size_t route_index = findRouteIndex(*task.to, existing_route);
      if (route_index != std::numeric_limits<size_t>::max()) {
        size_t binding_index = fpga::attachNetRoute(
            *task.net, *task.to, route_index, task.from, task.to,
            task.from_port, task.to_port, task.net_name);
        fpga::registerNetRouteTiles(*task.net, *existing_route,
                                    binding_index);
      }
      route_stats.task_attach_ns += static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - attach_start)
              .count());
    }
    ++route_stats.already_complete;
    return true;
  }

  if (route_recursion_budget <= 0) {
    rememberRouteTaskFailure(task, existing_route);
    return false;
  }

  bool route_complete = false;
  if (existing_route && !existing_route->empty()) {
    ++route_stats.continuation_attempts;
    size_t before_size = existing_route->size();
    bool root_blocked = false;
    int route_depth_limit =
        routeDepthLimitForAttempt(task.attempt, iteration_limit);
    auto route_start = std::chrono::steady_clock::now();
    // Generic routes may displace transit immediately; displaced trees are
    // requeued atomically.
    bool allow_preempt =
        route_preemption_enabled &&
        pnr::movingRouteMayPreempt(moving_stage,
                                   moving_focus_inst != nullptr,
                                   moving_sources_stage);
    bool continued = continuePartialRoute(
        *existing_route, *task.to, task.to_port, route_depth_limit,
        route_complete, &route_stats, this, task.net, task.from, task.from_port,
        task.net_name, &root_blocked, allow_preempt, allow_preempt);
    route_stats.task_route_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - route_start)
            .count());
    if (continued) {
      for (Wire &fragment : *existing_route) {
        fragment.net_name = task.net_name;
      }
      if (task.net) {
        auto attach_start = std::chrono::steady_clock::now();
        size_t route_index = findRouteIndex(*task.to, existing_route);
        if (route_index != std::numeric_limits<size_t>::max()) {
          size_t binding_index = fpga::attachNetRoute(
              *task.net, *task.to, route_index, task.from, task.to,
              task.from_port, task.to_port, task.net_name);
          fpga::registerNetRouteTilesFrom(*task.net, *existing_route,
                                          before_size, binding_index);
        }
        route_stats.task_attach_ns += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - attach_start)
                .count());
      }
      route_changed = true;
      route_progress = route_progress || route_complete ||
                       existing_route->size() > before_size;
      if (!fanout_stage && !moving_stage && existing_route->size() > before_size) {
        task.attempt = 0;
      }
      if (route_complete) {
        ++route_stats.completed;
        ++route_stats.cont_completed;
      } else if (existing_route->size() > before_size) {
        ++route_stats.partial_advanced;
        ++route_stats.cont_advanced;
      } else {
        ++route_stats.cont_no_advance;
      }
      // Commit at most one Generic suffix per scheduler turn. Letting a long
      // route claim several suffixes before its peers increases displacement
      // churn under congestion; failed attempts still retain their retry budget.
      if (!route_complete && !fanout_stage && !moving_stage) {
        route_recursion_budget = 1;
      }
      --route_recursion_budget;
      if (route_complete) {
        return true;
      }
      rememberRouteTaskFailure(task, existing_route);
      return false;
    }

    if (!existing_route->empty()) {
      ++route_stats.cont_no_advance;
      if (task.net) {
        size_t unrouted = 0;
        bool retry_after_backstep = false;
        bool has_parent_hop = routeCrossbarFragments(existing_route) > 1;
        if (pnr::failedBasicRootNeedsBackstep(fanout_stage, moving_stage,
                                              root_blocked)) {
          // A blocked Basic frontier is a deadend regardless of whether every
          // exit is absent, occupied, or already learned dead. Mark and remove
          // exactly its incoming hop so the parent tries another source bit.
          const Wire *tail = nullptr;
          for (auto fragment = existing_route->rbegin();
               fragment != existing_route->rend(); ++fragment) {
            if (fragment->type == Wire::WIRE_CROSSBAR && fragment->jump >= 0) {
              tail = &*fragment;
              break;
            }
          }
          if (tail) {
            Tile *tail_tile = fpga::Device::current().getTile(
                tail->from.x, tail->from.y);
            if (tail_tile) {
              route_src_deadends[tileDeadendKey(tail_tile->coord)]
                  .setBit(tail->jump);
              tail_tile->cb.src_deadend.jump.setBit(tail->jump);
              ++route_stats.src_deadend_marks;
            }
          }
          size_t route_index = findRouteIndex(*task.to, existing_route);
          size_t binding_index =
              route_index != std::numeric_limits<size_t>::max()
                  ? findNetRouteBindingIndex(*task.net, *task.to, route_index)
                  : std::numeric_limits<size_t>::max();
          if (binding_index != std::numeric_limits<size_t>::max() &&
              has_parent_hop) {
            unrouted = fpga::unrouteLastRouteStep(*task.net, binding_index)
                           ? 1
                           : 0;
          } else {
            std::vector<RouteTask> reroute_tasks;
            unrouted = unrouteSourceTree(*task.net, task.from, task.from_port,
                                         &reroute_tasks, false, false);
            for (RouteTask &reroute_task : reroute_tasks) {
              if (sameRouteTask(reroute_task, task)) {
                task.fanout = false;
                continue;
              }
              reroute_task.fanout = true;
              enqueueRouteTask(reroute_task, pending_route_todo);
            }
          }
          retry_after_backstep = unrouted != 0;
        } else if (pnr::failedGenericContinuationKeepsPrefix(
                       fanout_stage, moving_stage)) {
          // The bounded search leases only a successful suffix. A failed
          // speculative suffix therefore leaves every committed Generic hop
          // intact; failed child edges were already recorded as deadends by
          // tryBestFirstRoute().
          PNR_LOG3("ROUT",
                   "routeDesign Generic continuation failed: preserved "
                   "committed prefix '{}' after bounded failure, "
                   "root_blocked={}, remaining={}",
                   task.net_name, root_blocked, existing_route->size());
        } else if (pnr::failedMovingGenericKeepsPrefix(moving_stage,
                                                       task.fanout)) {
          // Keep every committed hop.  If bounded continuations stop
          // advancing, the Moving scheduler relocates the focused sink.
          if (routeTaskDebugMatches(task)) {
            PNR_LOG1("ROUT",
                     "routeDesign Moving continuation failed: preserved "
                     "Generic prefix '{}' after bounded failure, remaining={}",
                     task.net_name, existing_route->size());
          } else {
            PNR_LOG3("ROUT",
                     "routeDesign Moving continuation failed: preserved "
                     "Generic prefix '{}' after bounded failure, remaining={}",
                     task.net_name, existing_route->size());
          }
        } else if (pnr::failedContinuationOwnsOnlyBranch(
                       moving_stage, fanout_stage, task.fanout)) {
          // Fanout and Moving failures invalidate only their unique branch;
          // the already-routed external source tree remains authoritative.
          size_t route_index = findRouteIndex(*task.to, existing_route);
          size_t binding_index =
              route_index != std::numeric_limits<size_t>::max()
                  ? findNetRouteBindingIndex(*task.net, *task.to, route_index)
                  : std::numeric_limits<size_t>::max();
          if (binding_index != std::numeric_limits<size_t>::max()) {
            bool removed = fpga::unrouteNetBranch(*task.net, binding_index);
            if (!removed) {
              removed = fpga::discardNetBranch(*task.net, binding_index);
            }
            unrouted = removed ? 1 : 0;
          }
          if (routeTaskDebugMatches(task)) {
            PNR_LOG1(
                "ROUT",
                "routeDesign {} continuation failed: unrouted branch '{}' for "
                "retry, removed={}",
                moving_stage ? "Moving" : "Fanout", task.net_name, unrouted);
          }
        } else {
          // A failed trunk continuation invalidates the complete physical
          // source tree because sibling nets may share its leased prefix.
          std::vector<RouteTask> reroute_tasks;
          unrouted = unrouteSourceTree(*task.net, task.from, task.from_port,
                                       &reroute_tasks, false, false);
          for (RouteTask &reroute_task : reroute_tasks) {
            bool current_task = sameRouteTask(reroute_task, task);
            if (current_task) {
              task.fanout = false;
              continue;
            }
            reroute_task.fanout = true;
            enqueueRouteTask(reroute_task, pending_route_todo);
          }
          if (routeTaskDebugMatches(task)) {
            PNR_LOG1("ROUT",
                     "routeDesign continuation failed: unrouted source tree "
                     "'{}'/'{}' for retry, routes={}",
                     task.from ? task.from->makeName(FULL_NAME_LIMIT)
                               : std::string{},
                     task.from_port, unrouted);
          }
        }
        route_changed = route_changed || unrouted != 0;
        // Continue this task only after one real Basic backstep. Every other
        // failed continuation gets one bounded search in this scheduler pass.
        route_progress = route_progress || retry_after_backstep;
      } else {
        PNR_LOG3("ROUT",
                 "routeDesign continuation failed: kept partial route '{}'",
                 task.net_name);
      }
    } else {
      ++route_stats.failed;
      ++route_stats.cont_failed_empty;
    }
    ++task.attempt;
    --route_recursion_budget;
    rememberRouteTaskFailure(task, existing_route);
    return false;
  }

  ++route_stats.new_attempts;
  std::vector<Wire> wire;
  int saved_iteration_limit = iteration_limit;
  iteration_limit =
      routeDepthLimitForAttempt(task.attempt, saved_iteration_limit);
  route_iteration_budget = iteration_limit;
  auto route_start = std::chrono::steady_clock::now();
  bool routed =
      routeNet(*task.from, task.from_port, *task.to, task.to_port, wire,
               route_complete, task.attempt, task.net, task.net_name);
  iteration_limit = saved_iteration_limit;
  route_stats.task_route_ns += static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - route_start)
          .count());
  if (routed) {
    for (Wire &fragment : wire) {
      fragment.net_name = task.net_name;
    }
    if (!wire.empty()) {
      task.to->wires.emplace_back(std::move(wire));
      if (task.net) {
        auto attach_start = std::chrono::steady_clock::now();
        size_t binding_index = fpga::attachNetRoute(
            *task.net, *task.to, task.to->wires.size() - 1, task.from,
            task.to, task.from_port, task.to_port, task.net_name);
        fpga::registerNetRouteTiles(*task.net, task.to->wires.back(),
                                    binding_index);
        route_stats.task_attach_ns += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - attach_start)
                .count());
      }
      route_changed = true;
      route_progress = true;
      if (route_complete) {
        ++route_stats.completed;
        ++route_stats.new_completed;
      } else {
        ++route_stats.partial_started;
        ++route_stats.new_partial;
      }
    } else {
      ++route_stats.new_empty;
    }
    // Starting a Generic prefix is one forward commit; do not spend the
    // deadend-cleanup budget extending it again in the same pass.
    if (!route_complete && !fanout_stage && !moving_stage) {
      route_recursion_budget = 1;
    }
    --route_recursion_budget;
    if (route_complete) {
      return true;
    }
    rememberRouteTaskFailure(task, findBoundRoute(
                                       task.net, task.from, task.to,
                                       task.from_port, task.to_port,
                                       task.net_name));
    return false;
  }

  if ((task.attempt & 63U) == 0U) {
    PNR_LOG3("ROUT",
             "warning: failed limited route attempt for net '{}' from '{}' "
             "port '{}' to '{}' port '{}'",
             task.net_name, task.from->makeName(), task.from_port,
             task.to->makeName(), task.to_port);
  }
  ++route_stats.failed;
  ++route_stats.new_failed;
  ++task.attempt;
  --route_recursion_budget;
  rememberRouteTaskFailure(task);
  return false;
}

bool RouteDesign::enqueueRouteTask(const RouteTask &task,
                                   std::vector<RouteTask> &queue) {
  if (RouteCongestionTrace::current) RouteCongestionTrace::current->changed();
  indexSourceRoute(task.net, task.from, task.from_port);
  if (&queue == &pending_route_todo) {
    // Preemption can invalidate thousands of source trees in one pass.  Keep
    // this operation O(1); pass finalization merges duplicate task state once.
    queue.push_back(task);
    return true;
  }
  auto same_task = [&](const RouteTask &old) {
    return sameRouteTask(old, task);
  };
  auto merge_task = [&](std::vector<RouteTask> &tasks) {
    for (RouteTask &old : tasks) {
      if (!same_task(old)) {
        continue;
      }
      if (!task.fanout) {
        old.fanout = false;
      }
      old.attempt = std::max(old.attempt, task.attempt);
      old.fanout_branch_offset =
          std::max(old.fanout_branch_offset, task.fanout_branch_offset);
      old.fanout_branch_attempt =
          std::max(old.fanout_branch_attempt, task.fanout_branch_attempt);
      old.no_progress_passes =
          std::max(old.no_progress_passes, task.no_progress_passes);
      old.endpoints_prepared =
          old.endpoints_prepared && task.endpoints_prepared;
      if (task.failure_coord.x >= 0 && task.failure_coord.y >= 0 &&
          task.failure_sequence >= old.failure_sequence) {
        old.failure_coord = task.failure_coord;
        old.failure_sequence = task.failure_sequence;
        old.failure_node_type = task.failure_node_type;
        old.failure_node = task.failure_node;
      }
      old.remove_after_pass = false;
      return true;
    }
    return false;
  };
  if (!task.fanout && &queue != &fanout_route_todo) {
    std::erase_if(fanout_route_todo, [&](const RouteTask &deferred) {
      return pnr::promoteGenericOutOfFanoutQueue(task.fanout,
                                                 same_task(deferred));
    });
  }
  if (merge_task(route_todo) || merge_task(pending_route_todo) ||
      (task.fanout && merge_task(fanout_route_todo)) || merge_task(queue)) {
    return false;
  }
  queue.push_back(task);
  return true;
}

void RouteDesign::indexSourceRoute(rtl::Net *net, rtl::Inst *from,
                                   const std::string &from_port) {
  if (!net || !from || from_port.empty()) {
    return;
  }
  std::vector<rtl::Net *> &nets =
      source_route_nets[sourceRouteKey(from, from_port)];
  if (std::find(nets.begin(), nets.end(), net) == nets.end()) {
    nets.push_back(net);
  }
}

bool RouteDesign::sourceTreeHasCompleteExit(
    rtl::Inst &from, const std::string &from_port) const {
  auto indexed = source_route_nets.find(sourceRouteKey(&from, from_port));
  if (indexed != source_route_nets.end()) {
    for (const rtl::Net *net : indexed->second) {
      if (!net) {
        continue;
      }
      for (const rtl::NetRouteBinding &binding : net->routes) {
        if (samePhysicalSource(binding.from, binding.from_port, &from,
                               from_port) &&
            bindingHasRoutedSourceExit(binding)) {
          return true;
        }
      }
    }
    return false;
  }
  return hasCompleteFanoutSeedBinding(from, from_port);
}

bool RouteDesign::movingSourceNeedsRelocation(const RouteTask &task) const {
  if (!task.from || !task.from->tile.peer || !task.from->cell_ref.peer) {
    return false;
  }
  const std::vector<Wire> *route = findBoundRoute(
      task.net, task.from, task.to, task.from_port, task.to_port, task.net_name);
  bool has_committed_takeoff = routeCrossbarFragments(route) != 0;
  // Generated passthroughs are route-local adapters, not movable drivers.
  if (isGeneratedPassthroughInst(task.from)) {
    return false;
  }
  // A moved or unstarted suffix need not contain the driver's takeoff. It
  // belongs to the physical source tree, possibly under another logical net.
  // Partial sibling prefixes count too: Moving detaches sinks while retaining
  // committed source routing. Requiring a complete sibling would lose those.
  auto net_has_takeoff = [&](const rtl::Net *net) {
    if (!net) {
      return false;
    }
    for (const rtl::NetRouteBinding &binding : net->routes) {
      if (samePhysicalSource(binding.from, binding.from_port, task.from,
                             task.from_port) &&
          binding.owner && binding.route_index < binding.owner->wires.size() &&
          routeCrossbarFragments(&binding.owner->wires[binding.route_index]) != 0) {
        return true;
      }
    }
    return false;
  };
  if (!has_committed_takeoff) {
    has_committed_takeoff = net_has_takeoff(task.net);
  }
  if (!has_committed_takeoff) {
    auto indexed = source_route_nets.find(
        sourceRouteKey(task.from, task.from_port));
    if (indexed != source_route_nets.end()) {
      for (const rtl::Net *net : indexed->second) {
        if (net != task.net && net_has_takeoff(net)) {
          has_committed_takeoff = true;
          break;
        }
      }
    } else if (rtl::Module *module = parentModule(*task.from)) {
      // Legacy callers may attach bindings without populating the index.
      for (const auto &net : module->nets) {
        if (&net != task.net && net_has_takeoff(&net)) {
          has_committed_takeoff = true;
          break;
        }
      }
    }
  }
  if (has_committed_takeoff) {
    // Its occupied takeoff is reusable; do not demand another unused exit.
    return false;
  }
  NodeMask output_nodes = task.from->tile->getOutputPinNodes(
      task.from->cell_ref->type, task.from_port, task.from->pos);
  if (output_nodes == NodeMask{}) {
    return false;
  }
  std::vector<Tile *> route_tiles =
      routeTileCandidates(*task.from, task.from_port, true);
  bool has_free_takeoff = anyFreeRoutableOutputCandidate(
      route_tiles, *task.from, task.from_port, output_nodes, task.net);
  return pnr::movingSourceNeedsRelocation(has_committed_takeoff,
                                         has_free_takeoff);
}

// Duplicate preemption attempts are scoped to one routing pass, while blocker
// ancestry remains persistent so reciprocal preemption cycles stay rejected.
void RouteDesign::resetPassPreemptionState() {
  pnr::resetPassPreemptionContainers(preempted_route_names_this_pass,
                                     preempted_route_blockers);
  fanout_branch_indexes.clear();
}

// Stop speculative work at either the hard stage deadline or after repeated
// one-minute windows retire less than the configured percentage of tasks.
bool RouteDesign::routeStageSearchCancelled() {
  if (route_stage_stagnated) {
    return true;
  }
  auto now = std::chrono::steady_clock::now();
  if (route_stage_deadline_enabled && now >= route_stage_deadline) {
    route_stage_deadline_expired = true;
    return true;
  }
  RouteProgressSample sample =
      route_progress_watchdog.observe(route_progress_remaining_tasks, now);
  if (sample.sampled) {
    PNR_LOG1("ROUT",
             "routeDesign progress window: stage={}, tasks={}->{}, "
             "retired={}, required={}, stagnant_windows={}",
             route_progress_stage_name, sample.start_tasks,
             sample.remaining_tasks, sample.completed_tasks,
             sample.required_tasks, sample.stagnant_windows);
  }
  if (sample.stagnated) {
    route_stage_stagnated = true;
    return true;
  }
  return false;
}

// A stage transition starts a fresh watchdog; re-entering the same stage
// keeps its existing deficient-window history.
void RouteDesign::resetRouteProgressWatchdog(size_t stage,
                                             std::string stage_name,
                                             size_t remaining_tasks,
                                             int window_seconds,
                                             unsigned stagnant_windows) {
  route_progress_stage = stage;
  route_progress_stage_name = std::move(stage_name);
  route_progress_remaining_tasks = remaining_tasks;
  route_stage_stagnated = false;
  route_progress_watchdog.reset(
      remaining_tasks, std::chrono::steady_clock::now(),
      std::chrono::seconds(std::max(1, window_seconds)), 1,
      std::max(1U, stagnant_windows));
}

// Queue compaction supplies the exact outstanding count after each pass.
void RouteDesign::updateRouteProgressRemaining(size_t remaining_tasks) {
  route_progress_remaining_tasks = remaining_tasks;
  routeStageSearchCancelled();
}

// A completed task is committed progress even before pass compaction erases it.
void RouteDesign::noteRouteProgressCompletion() {
  if (route_progress_remaining_tasks != 0) {
    --route_progress_remaining_tasks;
  }
  routeStageSearchCancelled();
}

void RouteDesign::requeueNet(rtl::Net &net, bool fanout) {
  if (net.void_net) {
    return;
  }
  for (size_t route_index = 0; route_index < net.routes.size(); ++route_index) {
    const rtl::NetRouteBinding &binding = net.routes[route_index];
    if (netBindingIsVoid(net, binding)) {
      continue;
    }
    if (!binding.from || !binding.to || binding.route_name.empty()) {
      continue;
    }
    enqueueRouteTask(
        RouteTask{binding.from,
                  binding.to,
                  &net,
                  binding.from_port,
                  binding.to_port,
                  binding.route_name,
                  0,
                  0,
                  0,
                  {},
                  fanout || netHasCompleteRouteExcept(net, route_index)},
        pending_route_todo);
  }
}

// Remove every routed branch that starts from the same physical source pin.
// Logical aliases are indexed together, then rebuilt as one seed and fanouts.
size_t RouteDesign::sourceTreeRouteCount(rtl::Net &seed_net, rtl::Inst *from,
                                         const std::string &from_port) const {
  if (!from || from_port.empty()) {
    return 0;
  }
  size_t count = 0;
  auto indexed = source_route_nets.find(sourceRouteKey(from, from_port));
  if (indexed == source_route_nets.end()) {
    for (const rtl::NetRouteBinding &binding : seed_net.routes) {
      count += samePhysicalSource(binding.from, binding.from_port, from,
                                  from_port);
    }
    return count;
  }
  for (const rtl::Net *net : indexed->second) {
    if (!net) {
      continue;
    }
    for (const rtl::NetRouteBinding &binding : net->routes) {
      count += samePhysicalSource(binding.from, binding.from_port, from,
                                  from_port);
    }
  }
  return count;
}

bool RouteDesign::sourceTreeTouchesFinishedInst(
    rtl::Net &seed_net, rtl::Inst *from, const std::string &from_port) const {
  if (!moving_stage || !from) {
    return false;
  }
  auto indexed = source_route_nets.find(sourceRouteKey(from, from_port));
  if (indexed == source_route_nets.end()) {
    for (const rtl::NetRouteBinding &binding : seed_net.routes) {
      if (samePhysicalSource(binding.from, binding.from_port, from, from_port) &&
          (movingInstIsFinished(move_finished_insts, binding.from) ||
           movingInstIsFinished(move_finished_insts, binding.to))) {
        return true;
      }
    }
    return false;
  }
  for (const rtl::Net *net : indexed->second) {
    if (!net) {
      continue;
    }
    for (const rtl::NetRouteBinding &binding : net->routes) {
      if (!samePhysicalSource(binding.from, binding.from_port, from,
                              from_port)) {
        continue;
      }
      if (movingInstIsFinished(move_finished_insts, binding.from) ||
          movingInstIsFinished(move_finished_insts, binding.to)) {
        return true;
      }
    }
  }
  return false;
}

// Moving source-tree subsequence:
// 1. Collect every physical binding driven by one source port across logical
// nets.
// 2. Atomically release all surviving leases belonging to that source tree.
// 3. Requeue every binding, including routes already emptied by earlier
// cleanup.
// 4. Assign the first task as Generic and all remaining tasks as Fanouts.
// 5. Generic rebuilds the takeoff/trunk, then Fanouts rebuild the dependent
// branches. This keeps route leases and scheduler work atomic during relocation
// or preemption.
size_t RouteDesign::unrouteSourceTree(rtl::Net &seed_net, rtl::Inst *from,
                                      const std::string &from_port,
                                      std::vector<RouteTask> *tasks,
                                      bool fanout,
                                      bool include_empty_bindings) {
  if (!from || from_port.empty()) {
    return 0;
  }

  indexSourceRoute(&seed_net, from, from_port);
  auto indexed = source_route_nets.find(sourceRouteKey(from, from_port));
  PNR_ASSERT(indexed != source_route_nets.end(),
             "missing source-tree index for '{}'/'{}'", from->makeName(),
             from_port);
  std::vector<rtl::Net *> nets = indexed->second;
  if (nets.empty()) {
    nets.push_back(&seed_net);
  }

  size_t unrouted = 0;
  bool generic_task_added = fanout;
  std::vector<fpga::NetRouteRef> source_routes;
  std::vector<RouteTask> source_tasks;
  for (rtl::Net *net : nets) {
    if (!net) {
      continue;
    }
    std::vector<size_t> route_indices;
    std::vector<RouteTask> net_tasks;
    for (size_t route_index = 0; route_index < net->routes.size();
         ++route_index) {
      rtl::NetRouteBinding &binding = net->routes[route_index];
      bool source_matches = samePhysicalSource(
          binding.from, binding.from_port, from, from_port);
      if (!source_matches) {
        continue;
      }
      if (!binding.from || !binding.to || binding.route_name.empty()) {
        continue;
      }
      std::vector<Wire> *route = routeBindingRoute(binding);
      bool owns_route = route && !route->empty();
      if (owns_route) {
        route_indices.push_back(route_index);
        source_routes.push_back(fpga::NetRouteRef{net, route_index});
      }
      // Basic already holds every empty sibling in its deferred Fanout queue;
      // only bindings that owned physical state need another scheduler task.
      if (!include_empty_bindings && !owns_route) {
        continue;
      }
      net_tasks.push_back(RouteTask{binding.from,
                                    binding.to,
                                    net,
                                    binding.from_port,
                                    binding.to_port,
                                    binding.route_name,
                                    0,
                                    0,
                                    0,
                                    {},
                                    true});
    }
    if (route_indices.empty() && net_tasks.empty()) {
      continue;
    }
    bool removes_watched_route = false;
    std::string watched_path_before;
    for (size_t route_index : route_indices) {
      if (route_index >= net->routes.size()) {
        continue;
      }
      rtl::NetRouteBinding &binding = net->routes[route_index];
      if (!routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET",
                             binding.route_name)) {
        continue;
      }
      removes_watched_route = true;
      if (std::vector<Wire> *route = routeBindingRoute(binding)) {
        for (const std::string &node : routeTreeNodes(*route)) {
          if (!watched_path_before.empty()) {
            watched_path_before += "->";
          }
          watched_path_before += node;
        }
      }
    }
    if (removes_watched_route) {
      PNR_LOG1(
          "ROUT",
          "routeDesign invalidation trace: unrouteSourceTree removing watched "
          "source='{}'/'{}', active_culprit='{}', culprit_from='{}'/'{}', "
          "culprit_to='{}'/'{}', fanout={}, routes={}, watched_path={}",
          from->makeName(FULL_NAME_LIMIT), from_port,
          debug_active_route_task_valid ? debug_active_route_task.net_name
                                        : std::string("<scheduler>"),
          debug_active_route_task_valid && debug_active_route_task.from
              ? debug_active_route_task.from->makeName(FULL_NAME_LIMIT)
              : std::string{},
          debug_active_route_task_valid ? debug_active_route_task.from_port
                                        : std::string{},
          debug_active_route_task_valid && debug_active_route_task.to
              ? debug_active_route_task.to->makeName(FULL_NAME_LIMIT)
              : std::string{},
          debug_active_route_task_valid ? debug_active_route_task.to_port
                                        : std::string{},
          fanout, route_indices.size(), watched_path_before);
    }
    source_tasks.insert(source_tasks.end(),
                        std::make_move_iterator(net_tasks.begin()),
                        std::make_move_iterator(net_tasks.end()));
  }
  // Release the complete indexed source tree in one operation. No branch from
  // this physical source remains to own a shared prefix.
  bool route_state_changed = fpga::unrouteSourceRouteTree(source_routes);
  // Requeue bindings even when an earlier cleanup already emptied their route.
  unrouted += pnr::enqueueInvalidatedSourceTreeTasks(
      source_tasks, generic_task_added, route_state_changed,
      [&](RouteTask &next_task) {
        if (tasks) {
          return appendUniqueRouteTask(*tasks, next_task);
        }
        return enqueueRouteTask(next_task, pending_route_todo);
      });
  return unrouted;
}

bool RouteDesign::moveUnfinishedCell(const RouteTask &task,
                                     std::vector<RouteTask> *moved_tasks,
                                     const RouteTask *trigger_task,
                                     std::string *fail_reason,
                                     bool replay_failed_candidates) {
  const size_t moving_candidate_retry_limit =
      pnr::movingCandidateRetryLimit(move_attempt_limit);
  const RouteTask &route_task = trigger_task ? *trigger_task : task;
  TaskCongestionDiagnostic diagnostic(*this, route_task, "Moving forward placement");
  failure_packing_trace.reset();
  failure_packing_net.clear();
  failure_candidate_directory.clear();
  bool replaying = false;
  size_t replay_states = 0, replay_state_limit = 0;
  size_t replay_candidate = 0, replay_frame = 0, replay_images = 0;
  std::ofstream replay_report, replay_manifest, replay_decisions;
  std::string replay_stem;
  long replay_trace_begin = 0;
  std::vector<rtl::Net *> replay_nets;
  std::function<void(const std::string &)> replay_picture = [](const auto &) {};
  std::function<void()> reset_trial_diagnostic = [] {};
  auto move_search_cancelled = [&]() {
    // Diagnostics must survive the deadline expiring during PNG output. This
    // is bounded by the already exhausted search's number of reached states.
    return replaying ? replay_states > replay_state_limit : routeStageSearchCancelled();
  };
  const std::shared_ptr<std::FILE> packing_trace(std::tmpfile(), [](std::FILE *file) {
    if (file) std::fclose(file);
  });
  auto packing_note = [&](const std::string &message) {
    if (packing_trace) std::fprintf(packing_trace.get(), "%s\n", message.c_str());
    if (replaying && !packing_trace && replay_report.is_open()) replay_report << message << '\n';
  };
  packing_note(std::format("net={}\nfrom={}/{}\nto={}/{}", route_task.net_name,
      instNameForDump(route_task.from), route_task.from_port,
      instNameForDump(route_task.to), route_task.to_port));
  auto set_fail = [&](const std::string &reason) {
    packing_note("MOVING_FAILED " + reason);
    failure_packing_trace = packing_trace;
    failure_packing_net = route_task.net_name;
    if (fail_reason) {
      *fail_reason = reason;
    }
  };
  rtl::Inst *inst = task.to;
  if (!inst || !inst->tile.peer || isIoBuffer(*inst) || inst->outline.fixed) {
    set_fail("endpoint is not movable");
    return false;
  }
  std::vector<rtl::Inst *> move_cluster = strictMoveCluster(inst);
  // Generated route endpoints are rehomed after the real packed elements;
  // treating an unplaced endpoint as a strict member rejects a legal move.
  std::erase_if(move_cluster, [](rtl::Inst *member) {
    return isGeneratedPassthroughInst(member);
  });
  if (move_cluster.empty()) {
    move_cluster.push_back(inst);
  }
  uintptr_t inst_key = movingClusterKey(inst);
  // Check whether this focus completed an earlier Moving subsequence.
  bool marked_finished = movingInstIsFinished(move_finished_insts, inst);
  // A later source-tree invalidation can make that completion mark stale.
  if (marked_finished) {
    // Validate the mark against current physical route completion.
    bool incident_complete = allIncidentRoutesComplete(*inst);
    // Keep stable placement only while every incident route remains complete.
    if (pnr::movingFinishedMarkIsValid(marked_finished, incident_complete)) {
      // Explain why this endpoint is intentionally not moved again.
      set_fail("endpoint is marked finished");
      // No relocation is required for a still-complete focus.
      return false;
    }
    // Reopen the whole cluster so Generic and Fanout rerouting may move it
    // again.
    unmarkMovingClusterFinished(move_finished_insts, inst);
  }
  Tile *old_tile = &*inst->tile;
  int old_pos = inst->pos;
  Coord old_coord = old_tile->coord;
  auto moved_member = [&](rtl::Inst *candidate) {
    return std::find(move_cluster.begin(), move_cluster.end(), candidate) !=
           move_cluster.end();
  };
  // Generated endpoints share Moving ownership but are rehomed after the real
  // packed cluster, avoiding combinatorial placement permutations.
  std::vector<rtl::Inst *> moving_endpoint_closure = move_cluster;
  pnr::appendMovingEndpointChain(
      moving_endpoint_closure, [](rtl::Inst *member) {
        return member ? generatedPassthroughNeighbors(*member)
                      : std::vector<rtl::Inst *>{};
      });
  auto moved_endpoint = [&](rtl::Inst *candidate) {
    return std::find(moving_endpoint_closure.begin(),
                     moving_endpoint_closure.end(),
                     candidate) != moving_endpoint_closure.end();
  };
  std::unordered_set<rtl::Inst *> moving_endpoint_set(
      moving_endpoint_closure.begin(), moving_endpoint_closure.end());
  std::vector<rtl::Net *> incident_route_nets =
      incidentRouteNets(moving_endpoint_set, route_task.net);
  struct DetachedPassthrough {
    rtl::Inst *inst = nullptr;
    Tile *tile = nullptr;
    int pos = -1;
  };
  std::vector<DetachedPassthrough> detached_passthroughs;
  for (rtl::Inst *endpoint : moving_endpoint_closure) {
    if (!endpoint || moved_member(endpoint) ||
        !isGeneratedPassthroughInst(endpoint)) {
      continue;
    }
    detached_passthroughs.push_back(
        {endpoint, endpoint->tile.peer ? &*endpoint->tile : nullptr,
         endpoint->pos});
  }
  if (move_cluster.size() > 1) {
    for (rtl::Inst *member : move_cluster) {
      if (!member || !member->tile.peer || isIoBuffer(*member) ||
          member->outline.fixed) {
        set_fail("strict move cluster contains non-movable member");
        return false;
      }
    }
  }
  bool moving_debug = routeTaskDebugMatches(route_task);
  int forced_move_x = -1;
  int forced_move_y = -1;
  int forced_move_pos = -1;
  const char *forced_move_text = std::getenv("SCALEPNR_DEBUG_MOVE_TARGET");
  int forced_move_fields =
      forced_move_text
          ? std::sscanf(forced_move_text, "%d,%d,%d", &forced_move_x,
                        &forced_move_y, &forced_move_pos)
          : 0;
  bool force_move_target = moving_debug && forced_move_fields >= 2;
  if (moving_debug) {
    std::string cluster_members;
    for (rtl::Inst *member : move_cluster) {
      if (!cluster_members.empty()) {
        cluster_members += " | ";
      }
      cluster_members += member
          ? std::format("{}:{}@{}", member->makeName(FULL_NAME_LIMIT),
                        member->cell_ref.peer ? member->cell_ref->type
                                              : std::string{},
                        reinterpret_cast<uintptr_t>(member))
          : std::string{"<null>"};
    }
    PNR_LOG1("ROUT",
             "routeDesign moving debug: begin inst='{}' type='{}' route='{}' "
             "from='{}'/'{}' to='{}'/'{}' old=({},{})/{} cluster=[{}]",
             inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type,
             route_task.net_name,
             route_task.from ? route_task.from->makeName(FULL_NAME_LIMIT)
                             : std::string{},
             route_task.from_port,
             route_task.to ? route_task.to->makeName(FULL_NAME_LIMIT)
                           : std::string{},
             route_task.to_port, old_coord.x, old_coord.y, old_pos,
             cluster_members);
  }
  std::vector<uint64_t> &tried = move_tried_placements[inst_key];
  uint64_t old_key = placementKey(old_coord, old_pos);
  if (std::find(tried.begin(), tried.end(), old_key) == tried.end()) {
    tried.push_back(old_key);
  }
  if (tried.size() >= moving_candidate_retry_limit) {
    set_fail("placement focus retry cap tried=" + std::to_string(tried.size()) +
             " limit=" + std::to_string(moving_candidate_retry_limit));
    return false;
  }

  auto has_incident_route_binding = [&]() {
    for (rtl::Net *net : incident_route_nets) {
      if (!net) {
        continue;
      }
      for (const rtl::NetRouteBinding &binding : net->routes) {
        if (moved_endpoint(binding.from) || moved_endpoint(binding.to)) {
          return true;
        }
      }
    }
    return false;
  };
  bool trigger_touches_inst =
      moved_endpoint(route_task.from) || moved_endpoint(route_task.to);
  if (!has_incident_route_binding() && !trigger_touches_inst) {
    set_fail("move produced no affected routes");
    return false;
  }

  // Prove every route affected by the move before accepting a candidate
  // placement.
  std::vector<RouteTask> candidate_route_tasks;
  auto add_candidate_route_task = [&](const RouteTask &candidate_task) {
    appendUniqueRouteTask(candidate_route_tasks, candidate_task);
  };
  for (rtl::Net *net : incident_route_nets) {
    if (net) {
      // Distributed protected sources use this same Moving transaction;
      // other infrastructure nets retain their independent owner.
      if (!net->routeCanBePreempted() && !net->distributed_source) {
        continue;
      }
      for (const rtl::NetRouteBinding &binding : net->routes) {
        if ((!moved_endpoint(binding.from) && !moved_endpoint(binding.to)) ||
            !binding.from || !binding.to || binding.route_name.empty()) {
          continue;
        }
        add_candidate_route_task(
            RouteTask{binding.from,
                      binding.to,
                      net,
                      binding.from_port,
                      binding.to_port,
                      binding.route_name,
                      0,
                      0,
                      0,
                      {},
                      !net->distributed_source &&
                          !moved_endpoint(binding.from) &&
                          sourceTreeHasCompleteExit(*binding.from,
                                                    binding.from_port)});
      }
    }
  }
  add_candidate_route_task(route_task);

  struct MovedPlacement {
    rtl::Inst *inst = nullptr;
    Tile *tile = nullptr;
    int pos = -1;
  };
  std::vector<MovedPlacement> old_placements;
  std::string placement_reject_reason;
  old_placements.reserve(move_cluster.size());
  for (rtl::Inst *member : move_cluster) {
    old_placements.push_back(MovedPlacement{
        member, member && member->tile.peer ? &*member->tile : nullptr,
        member ? member->pos : -1});
    const auto &old = old_placements.back();
    // Record the original cluster before temporary candidate placements begin.
    packing_note(std::format("ORIGINAL_MEMBER name={} tile=({},{}) pos={}",
        instNameForDump(member), old.tile ? old.tile->coord.x : -1,
        old.tile ? old.tile->coord.y : -1, old.pos));
  }
  auto restore_move_cluster = [&]() {
    for (MovedPlacement &old : old_placements) {
      if (!old.inst || !old.tile) {
        continue;
      }
      if (old.inst->tile.peer) {
        unplaceInst(*old.inst, "restore-move-cluster-current");
      }
      restoreInstPlacement(*old.inst, *old.tile, old.pos);
    }
  };
  auto unplace_move_cluster = [&](const char *reason) {
    for (rtl::Inst *member : move_cluster) {
      if (member && member->tile.peer) {
        unplaceInst(*member, reason);
      }
    }
  };
  auto unplace_detached_passthroughs = [&](const char *reason) {
    for (DetachedPassthrough &detached : detached_passthroughs) {
      if (detached.inst && detached.inst->tile.peer) {
        unplaceInst(*detached.inst, reason);
      }
    }
  };
  auto restore_detached_passthroughs_exact = [&]() {
    unplace_detached_passthroughs("restore-generated-endpoint-current");
    for (DetachedPassthrough &detached : detached_passthroughs) {
      if (detached.inst && detached.tile) {
        restoreInstPlacement(*detached.inst, *detached.tile, detached.pos);
      }
    }
  };
  auto rehome_detached_passthroughs = [&]() {
    std::string blocked_endpoint;
    std::string blocked_reason;
    bool rehomed = pnr::resolveMovingEndpointDependencies(
        detached_passthroughs, [&](DetachedPassthrough &detached) {
          std::string reason;
          if (detached.inst &&
              fpga::rehomeGeneratedPassthrough(*detached.inst, &reason)) {
            return true;
          }
          blocked_endpoint = detached.inst
                                 ? detached.inst->makeName(FULL_NAME_LIMIT)
                                 : std::string("<null>");
          blocked_reason = reason;
          return false;
        });
    if (!rehomed) {
      placement_reject_reason = "cannot rehome generated endpoint '" +
                                blocked_endpoint + "': " + blocked_reason;
      unplace_detached_passthroughs("reject-generated-endpoint-rehome");
      return false;
    }
    return true;
  };
  auto restore_detached_passthroughs = [&]() {
    // Restore from live void-link anchors first, repairing endpoint placements
    // made stale by an earlier move; raw coordinates are the final fallback.
    unplace_detached_passthroughs("restore-generated-endpoint-from-anchor");
    std::string saved_reject_reason = placement_reject_reason;
    if (rehome_detached_passthroughs()) {
      placement_reject_reason = std::move(saved_reject_reason);
      return;
    }
    placement_reject_reason = std::move(saved_reject_reason);
    restore_detached_passthroughs_exact();
  };
  std::function<bool()> accept_move_placement;
  auto place_move_cluster = [&](Tile &tile, std::vector<rtl::Inst *> *placed,
                                bool require_endpoint_rehome) {
    if (placed) {
      placed->clear();
    }
    std::vector<std::vector<rtl::Inst *>> orders =
        clusterPlacementOrders(move_cluster);
    auto clear_placed = [&]() {
      if (!placed) {
        return;
      }
      for (auto it = placed->rbegin(); it != placed->rend(); ++it) {
        if (*it && (*it)->tile.peer) {
          unplaceInst(**it, "reject-cluster-placement");
        }
      }
      placed->clear();
    };
    auto try_order = [&](const std::vector<rtl::Inst *> &order) {
      packing_note("PACKING_ORDER");
      for (rtl::Inst *member : order)
        packing_note("  member=" + instNameForDump(member));
      auto place_member = [&](auto &&self, size_t index) -> bool {
        if (move_search_cancelled()) return false;
        if (index >= order.size()) {
          // Endpoint lanes are part of candidate packing legality. A blocked
          // lane backtracks through other cluster positions in this tile.
          if (require_endpoint_rehome && !rehome_detached_passthroughs()) {
            packing_note("ENDPOINT_REHOME_REJECT " + placement_reject_reason);
            replay_picture("endpoint rehome rejected: " + placement_reject_reason);
            return false;
          }
          const bool accepted = !accept_move_placement || accept_move_placement();
          if (replaying) {
            replay_picture(accepted ? "WOULD_ACCEPT (diagnostic only)" : placement_reject_reason);
            // Explore remaining packing positions without committing a route
            // or treating an unexpected diagnostic success as routing success.
          } else if (accepted) return true;
          packing_note("PACKED_BUT_GROUNDING_OR_SUPPORT_REJECT " + placement_reject_reason);
          unplace_detached_passthroughs("reject-unreachable-grounding");
          return false;
        }
        rtl::Inst *member = order[index];
        if (!member) {
          placement_reject_reason = "null member in strict move cluster";
          return false;
        }
        std::vector<int> positions = tile.candidatePositions(member);
        size_t attempted_positions = 0;
        if (member->pos >= 0) {
          auto old_pos_it =
              std::find(positions.begin(), positions.end(), member->pos);
          if (old_pos_it != positions.end()) {
            std::rotate(positions.begin(), old_pos_it, old_pos_it + 1);
          }
        }
        for (int pos : positions) {
          if (move_search_cancelled()) return false;
          ++attempted_positions;
          reset_trial_diagnostic();
          packing_note(std::format("PACK_POSITION tile=({},{}) member={} type={} pos={} depth={}",
              tile.coord.x, tile.coord.y, instNameForDump(member),
              member->cell_ref.peer ? member->cell_ref->type : "<missing>", pos, index));
          if (tile.tryAddAt(member, pos, true, packing_trace.get()) < 0) {
            packing_note("PACK_POSITION_REJECT");
            replay_picture(std::format("packing rejected member={} pos={}", instNameForDump(member), pos));
            continue;
          }
          if (placed) {
            placed->push_back(member);
          }
          if (self(self, index + 1)) {
            return true;
          }
          packing_note(std::format("PACK_BACKTRACK member={} pos={} (slot fitted; remaining cluster or grounding failed)",
                                   instNameForDump(member), pos));
          if (placed && !placed->empty()) {
            placed->pop_back();
          }
          if (member->tile.peer) {
            unplaceInst(*member, "reject-cluster-position");
          }
        }
        if (placement_reject_reason.empty()) {
          placement_reject_reason = std::format(
              "cannot place strict member '{}' type='{}' on tile=({},{}) candidates={}",
              member->makeName(FULL_NAME_LIMIT),
              member->cell_ref.peer ? member->cell_ref->type : std::string{},
              tile.coord.x, tile.coord.y, attempted_positions);
        }
        packing_note("PACK_MEMBER_EXHAUSTED " + placement_reject_reason);
        return false;
      };
      bool ok = place_member(place_member, 0);
      if (!ok) {
        clear_placed();
      }
      return ok;
    };
    for (const std::vector<rtl::Inst *> &order : orders) {
      if (try_order(order)) {
        return true;
      }
    }
    return false;
  };

  unplace_detached_passthroughs("begin-moving-endpoint-rehome");
  unplace_move_cluster("begin-moving-candidate-scan");
  if (force_move_target && forced_move_fields >= 3) {
    inst->pos = forced_move_pos;
  }
  std::vector<int> scan_start_positions;
  for (rtl::Inst *member : move_cluster) scan_start_positions.push_back(member->pos);

  std::unordered_map<Tile *, CBState> candidate_terminal_states;
  std::unordered_map<Tile *, CBState> forward_base_states;
  auto candidate_base = [&](Tile *tile) -> const CBState & {
    auto found = forward_base_states.find(tile);
    return found == forward_base_states.end() ? tile->cb : found->second;
  };
  std::unordered_map<Tile *, NodeMask> candidate_terminal_pins;
  std::unordered_map<Tile *, std::unordered_map<int, std::string>>
      candidate_terminal_local_owners;
  std::unordered_map<Tile *, std::unordered_map<int, std::string>>
      candidate_terminal_dst_owners;
  std::unordered_map<Tile *, std::unordered_map<int, std::string>>
      candidate_terminal_joint_owners;
  reset_trial_diagnostic = [&]() {
    candidate_terminal_states.clear();
    candidate_terminal_pins.clear();
    candidate_terminal_local_owners.clear();
    candidate_terminal_dst_owners.clear();
    candidate_terminal_joint_owners.clear();
  };
  auto placement_supports_route_task = [&](const RouteTask &candidate_task) {
    placement_reject_reason.clear();
    if (!inst->tile.peer || !inst->cell_ref.peer) {
      placement_reject_reason = "missing tile/cell reference";
      return false;
    }
    if (moved_endpoint(candidate_task.from)) {
      NodeMask output_nodes = candidate_task.from->tile->getOutputPinNodes(
          candidate_task.from->cell_ref->type, candidate_task.from_port,
          candidate_task.from->pos);
      if (output_nodes == NodeMask{}) {
        placement_reject_reason = "no output nodes for route '" +
                                  candidate_task.net_name + "' port '" +
                                  candidate_task.from_port + "'";
        return false;
      }
      std::vector<Tile *> candidates = routeTileCandidates(
          *candidate_task.from, candidate_task.from_port, true);
      if (!anyFreeRoutableOutputCandidate(
              candidates, *candidate_task.from, candidate_task.from_port,
              output_nodes, candidate_task.net)) {
        placement_reject_reason = "no free routable output candidate for route '" +
                                  candidate_task.net_name + "' port '" +
                                  candidate_task.from_port +
                                  "' nodes=" + maskString(output_nodes);
        return false;
      }
    }
    if (moved_endpoint(candidate_task.to)) {
      NodeMask input_nodes = candidate_task.to->tile->getPinNodes(
          candidate_task.to->cell_ref->type, candidate_task.to_port,
          candidate_task.to->pos);
      if (input_nodes == NodeMask{}) {
        placement_reject_reason = "no input nodes for route '" +
                                  candidate_task.net_name + "' port '" +
                                  candidate_task.to_port + "'";
        return false;
      }
      std::vector<Tile *> candidates = routeTileCandidates(
          *candidate_task.to, candidate_task.to_port, false);
      if (candidates.empty()) {
        placement_reject_reason = "no route tile candidates for route '" +
                                  candidate_task.net_name + "' input port '" +
                                  candidate_task.to_port +
                                  "' nodes=" + maskString(input_nodes);
        return false;
      }
      bool terminal_reserved = false;
      std::string terminal_reject_detail;
      for (Tile *candidate : candidates) {
        if (!candidate || !candidate->cb_type) {
          continue;
        }
        NodeMask route_nodes = routeTileInputNodes(
            *candidate, *candidate_task.to, candidate_task.to_port);
        if (route_nodes == NodeMask{}) {
          continue;
        }
        auto [state_it, state_inserted] =
            candidate_terminal_states.try_emplace(candidate, candidate->cb);
        auto [pins_it, pins_inserted] = candidate_terminal_pins.try_emplace(
            candidate, candidate->pin_state.leased_nodes);
        (void)state_inserted;
        (void)pins_inserted;
        std::vector<pnr::MovingTerminalPath> paths;
        route_nodes.for_each_set_bit([&](int local) {
          for (const CBType::TerminalEntry &entry :
               candidate->cb_type->terminalEntries(local)) {
            if (!candidate->incoming_dst_nodes.testBit(entry.dst)) continue;
            paths.push_back(pnr::MovingTerminalPath{
                local, entry.dst, entry.joint, entry.joint2});
          }
          return false;
        });
        CBState &state = state_it->second;
        pnr::MovingTerminalPath selected_path;
        terminal_reserved = pnr::reserveMovingTerminalPath(
            paths, pins_it->second, state.local.local, state.dst.jump,
            state.joint.jump, &selected_path);
        if (terminal_reserved) {
          candidate_terminal_local_owners[candidate][selected_path.local] =
              candidate_task.net_name;
          candidate_terminal_dst_owners[candidate][selected_path.dst] =
              candidate_task.net_name;
          if (selected_path.joint >= 0) {
            candidate_terminal_joint_owners[candidate][selected_path.joint] =
                candidate_task.net_name;
          }
          if (selected_path.joint2 >= 0) {
            candidate_terminal_joint_owners[candidate][selected_path.joint2] =
                candidate_task.net_name;
          }
        }
        if (terminal_reserved) {
          packing_note(std::format("OTHER_INPUT_RESERVED net={} port={} cb=({},{}) local={} dst={} joint={} joint2={}",
              candidate_task.net_name, candidate_task.to_port, candidate->coord.x,
              candidate->coord.y, selected_path.local, selected_path.dst,
              selected_path.joint, selected_path.joint2));
          break;
        }
        if (packing_trace) {
          std::ostringstream details;
          details << "OTHER_INPUT_REJECT net=" << std::quoted(candidate_task.net_name)
                  << " port=" << std::quoted(candidate_task.to_port)
                  << " resource=(" << candidate_task.to->tile->coord.x << ','
                  << candidate_task.to->tile->coord.y << ") pos=" << candidate_task.to->pos << '\n';
          printMovingCandidateState(details, *candidate, candidate_base(candidate));
          printMovingTerminalAlternatives(details, *candidate, route_nodes,
              candidate_base(candidate), state, pins_it->second,
              candidate_terminal_local_owners[candidate], candidate_terminal_dst_owners[candidate],
              candidate_terminal_joint_owners[candidate]);
          packing_note(details.str());
        }
        if (moving_debug) {
          std::string path_text;
          std::string existing_owners;
          std::unordered_set<int> reported_locals;
          std::unordered_set<int> reported_dsts;
          std::unordered_set<int> reported_joints;
          for (const pnr::MovingTerminalPath &path : paths) {
            if (!path_text.empty()) {
              path_text += ",";
            }
            path_text += std::format("{}/{}/{}/{}", path.local, path.dst,
                                     path.joint, path.joint2);
            if (reported_locals.insert(path.local).second) {
              rtl::Net *local_owner =
                  fpga::findNetByNode(*candidate, fpga::CB_NODE_LOCAL,
                                      path.local, false);
              if (!existing_owners.empty()) {
                existing_owners += ",";
              }
              existing_owners += std::format("local{}='{}'", path.local,
                                              netDebugName(local_owner));
            }
            if (reported_dsts.insert(path.dst).second) {
              rtl::Net *dst_owner =
                  fpga::findNetByNode(*candidate, fpga::CB_NODE_DST, path.dst,
                                      false);
              existing_owners += std::format("{}dst{}='{}'",
                                             existing_owners.empty() ? "" : ",",
                                             path.dst,
                                             netDebugName(dst_owner));
            }
            for (int joint : {path.joint, path.joint2}) {
              if (joint < 0 || !reported_joints.insert(joint).second) {
                continue;
              }
              rtl::Net *joint_owner =
                  fpga::findNetByNode(*candidate, fpga::CB_NODE_JOINT, joint,
                                      false);
              existing_owners += std::format(
                  "{}joint{}='{}'", existing_owners.empty() ? "" : ",",
                  joint, netDebugName(joint_owner));
            }
          }
          terminal_reject_detail = std::format(
              " resource_tile='{}'@({},{})/{} route_tile='{}'@({},{}) "
              "input={} route={} leased(pin={},local={},dst={},joint={}) "
              "paths=[{}] existing_owners=[{}] local_owners=[{}] "
              "dst_owners=[{}] joint_owners=[{}]",
              candidate_task.to->tile->makeName(),
              candidate_task.to->tile->coord.x,
              candidate_task.to->tile->coord.y, candidate_task.to->pos,
              candidate->makeName(), candidate->coord.x, candidate->coord.y,
              maskString(input_nodes), maskString(route_nodes),
              maskString(pins_it->second), maskString(state.local.local),
              maskString(state.dst.jump), maskString(state.joint.jump),
              path_text, existing_owners,
              [&]() {
                std::string owners;
                for (const auto &[local, owner] :
                     candidate_terminal_local_owners[candidate]) {
                  if (!owners.empty()) {
                    owners += ",";
                  }
                  owners += std::format("{}='{}'", local, owner);
                }
                return owners;
              }(),
              [&]() {
                std::string owners;
                for (const auto &[dst, owner] :
                     candidate_terminal_dst_owners[candidate]) {
                  if (!owners.empty()) {
                    owners += ",";
                  }
                  owners += std::format("{}='{}'", dst, owner);
                }
                return owners;
              }(),
              [&]() {
                std::string owners;
                for (const auto &[joint, owner] :
                     candidate_terminal_joint_owners[candidate]) {
                  if (!owners.empty()) {
                    owners += ",";
                  }
                  owners += std::format("{}='{}'", joint, owner);
                }
                return owners;
              }());
        }
      }
      if (!terminal_reserved) {
        placement_reject_reason =
            "no distinct free terminal path for route '" +
            candidate_task.net_name + "' input port '" +
            candidate_task.to_port + "'" + terminal_reject_detail;
        return false;
      }
    }
    return true;
  };

  // Candidate positions are discovered by the route, never by a geometric
  // placement scan. Retain the exact source-to-branch proof for committing it.
  std::vector<ForwardPlacementAnchor> anchors;
  std::vector<bool> anchor_retains_trigger;
  // Shared trunk copies describe the same owned branch node. Seed it once;
  // this is not a visited-Tile set for the subsequent path-local search.
  std::unordered_set<BackwardResolveKey, BackwardResolveKeyHash> branch_nodes;
  auto add_anchor = [&](const std::vector<Wire> &route, size_t count, bool trigger) {
    if (routeStageSearchCancelled()) return;
    const Wire &last = route[count - 1];
    if (last.type != Wire::WIRE_CROSSBAR || last.jump < 0 || last.dst < 0)
      return;
    Tile *tile = fpga->getTile(last.to.x, last.to.y);
    if (!tile || !tile->cb_type || !tile->cb.dst.jump.testBit(last.dst)) return;
    if (!branch_nodes.insert({tile->coord.x, tile->coord.y, last.dst}).second)
      return;
    ForwardPlacementAnchor anchor;
    anchor.tile = tile;
    anchor.node = last.dst;
    anchor.prefix.assign(route.begin(), route.begin() + count);
    anchors.push_back(std::move(anchor));
    anchor_retains_trigger.push_back(trigger);
  };
  if (route_task.net && route_task.from && !moved_endpoint(route_task.from)) {
    for (size_t i = 0; i < route_task.net->routes.size(); ++i) {
      auto &binding = route_task.net->routes[i];
      if (binding.from != route_task.from || binding.to != route_task.to ||
          binding.from_port != route_task.from_port ||
          binding.to_port != route_task.to_port ||
          binding.route_name != route_task.net_name) continue;
      const auto *route = routeBindingRoute(binding);
      if (route && !routeIsComplete(*route)) {
        // Prefer continuing the tail, but a dead tail must not hide earlier
        // source-connected branches. Keep all old leases until a replacement
        // has actually grounded; commit below truncates only the chosen tail.
        for (size_t count = route->size(); count > 0; --count)
          add_anchor(*route, count, true);
      }
    }
    for (const SourceRouteBinding &source : completeIndexedFanoutSeedBindings(
             *this, *route_task.from, route_task.from_port)) {
      if (!source.net || !source.binding || moved_endpoint(source.binding->to))
        continue; // Its prefix must survive this relocation.
      const auto *route = routeBindingRoute(*source.binding);
      if (!route || !routeIsComplete(*route)) continue;
      for (size_t count = 1; count <= route->size(); ++count)
        add_anchor(*route, count, false);
    }
    // A Generic seed without a retained trunk starts at its real output,
    // never at a free destination unconnected to the source.
    if (anchors.empty() && !route_task.fanout) {
      for (Tile *tile : routeTileCandidates(*route_task.from, route_task.from_port, true)) {
        if (!tile || !tile->cb_type) continue;
        routeTileEndpointNodes(*tile, *route_task.from, route_task.from_port, true)
            .for_each_set_bit([&](int local) {
              anchors.push_back({tile, local, false, {}});
              anchor_retains_trigger.push_back(false);
              return false;
            });
      }
    }
  }
  Tile *new_tile = nullptr;
  int new_pos = -1;
  int move_candidates = 0;
  int move_reject_no_tile = 0, move_reject_old_tile = 0;
  int move_reject_place = 0, move_reject_tried = 0;
  int move_reject_support = 0, move_reject_route = 0;
  int failed_scans = move_failed_scans[inst_key];
  bool move_scan_timed_out = false;
  uint64_t move_scan_place_ns = 0, move_scan_requirements_ns = 0;
  uint64_t move_scan_support_ns = 0;
  size_t move_scan_placed_candidates = 0;
  ForwardPlacementResult forward;
  Tile *reached_tile = nullptr;
  int reached_dst = -1;
  bool reached_owned = false;
  std::vector<Wire> selected_terminal;

  replay_picture = [&](const std::string &reason) {
    if (!replaying || !reached_tile) return;
    const std::string file = std::format("{}_{:03}.png", replay_stem, ++replay_frame);
    packing_note("PICTURE reason=" + reason + " file=" + file);
    for (rtl::Inst *member : move_cluster)
      packing_note(std::format("PICTURE_MEMBER name={} placed={} tile=({},{}) pos={}",
          instNameForDump(member), member->tile.peer != nullptr,
          member->tile.peer ? member->tile->coord.x : -1,
          member->tile.peer ? member->tile->coord.y : -1, member->pos));
    try {
      // The picture must show the same temporary input reservations as the
      // rejection report, not the restored placement or another failed task.
      struct TrialView {
        struct Saved { Tile *tile; CBState cb; NodeMask pins; };
        std::vector<Saved> saved;
        ~TrialView() {
          for (auto &s : saved) { s.tile->cb = s.cb; s.tile->pin_state.leased_nodes = s.pins; }
        }
      } view;
      for (const auto &[tile, state] : candidate_terminal_states) {
        view.saved.push_back({tile, tile->cb, tile->pin_state.leased_nodes});
        tile->cb = state;
        if (auto pins = candidate_terminal_pins.find(tile); pins != candidate_terminal_pins.end())
          tile->pin_state.leased_nodes = pins->second;
      }
      fpga::Visualizer visual(*fpga);
      visual.drawCB(reached_tile->coord.x, reached_tile->coord.y);
      visual.drawRoutes(route_task.net_name);
      visual.highlightNode(reached_tile->coord, fpga::CB_NODE_DST, reached_dst,
                           "reached DST " + std::to_string(reached_dst));
      for (const auto &[tile, owners] : candidate_terminal_local_owners)
        for (const auto &[pin, owner] : owners)
          visual.highlightNode(tile->coord, fpga::CB_NODE_LOCAL, pin, "reserved: " + owner);
      visual.drawFailureLabel(std::format("candidate {} frame {}: {} | {}",
          replay_candidate, replay_frame, reason, route_task.net_name));
      visual.writePNG(file);
      ++replay_images;
      replay_manifest << "IMAGE " << std::quoted(file) << " reason=" << std::quoted(reason) << '\n';
    } catch (const std::exception &error) {
      packing_note("PICTURE_ERROR " + std::string(error.what()));
      replay_manifest << "IMAGE_ERROR " << std::quoted(file) << ' ' << std::quoted(error.what()) << '\n';
    }
  };

  accept_move_placement = [&]() {
    ++move_scan_placed_candidates;
    if (move_search_cancelled()) {
      placement_reject_reason = "stage search cancelled";
      move_scan_timed_out = true; return false;
    }
    if (force_move_target && forced_move_fields >= 3 && inst->pos != forced_move_pos) {
      placement_reject_reason = "position excluded by forced target";
      return false;
    }
    if (placementWasTried(tried, inst->tile->coord, inst->pos)) {
      placement_reject_reason = "position already tried";
      ++move_reject_tried;
      return false;
    }
    const auto targets = routeTileCandidates(*route_task.to, route_task.to_port, false);
    if (std::find(targets.begin(), targets.end(), reached_tile) == targets.end()) {
      placement_reject_reason = "reached CrossBox cannot serve this packed input";
      return false;
    }
    const NodeMask pins = routeTileInputNodes(*reached_tile, *route_task.to, route_task.to_port);
    const NodeMask reserved = fpga::packedInputJointReservations(
        *reached_tile, route_task.to, route_task.to_port);
    // Keep the existing constrained-input-first ordering for the other
    // affected routes. The trigger itself must use this reached DST.
    const auto requirements_start = std::chrono::steady_clock::now();
    std::vector<std::vector<pnr::MovingTerminalPath>>
        terminal_requirements(candidate_route_tasks.size());
    for (size_t i = 0; i < candidate_route_tasks.size(); ++i) {
      const auto &other = candidate_route_tasks[i];
      if (sameRouteTask(other, route_task) || !moved_endpoint(other.to) ||
          !other.to->tile.peer) continue;
      for (Tile *tile : routeTileCandidates(*other.to, other.to_port, false)) {
        if (!tile || !tile->cb_type) continue;
        routeTileInputNodes(*tile, *other.to, other.to_port).for_each_set_bit(
            [&](int pin) {
              for (const auto &entry : tile->cb_type->terminalEntries(pin)) {
                if (!tile->incoming_dst_nodes.testBit(entry.dst)) continue;
                terminal_requirements[i].push_back(
                    {pin, entry.dst, entry.joint, entry.joint2});
              }
              return false;
            });
      }
    }
    const auto terminal_order = pnr::movingTerminalReservationOrder(terminal_requirements);
    move_scan_requirements_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - requirements_start).count();
    bool accepted = pins.for_each_set_bit([&](int pin) {
      if (reached_tile->isPinNodeLeased(pin)) {
        packing_note(std::format("GROUNDING_REJECT pin={} reason=leased-pin", pin));
        return false;
      }
      size_t matching_entries = 0;
      for (const auto &entry : reached_tile->cb_type->terminalEntries(pin)) {
        if (entry.dst != reached_dst) continue;
        ++matching_entries;
        if ((entry.joint >= 0 && reserved.testBit(entry.joint)) ||
            (entry.joint2 >= 0 && reserved.testBit(entry.joint2))) {
          packing_note(std::format("GROUNDING_REJECT pin={} dst={} joint={} joint2={} reason=packed-input-reservation",
              pin, reached_dst, entry.joint, entry.joint2));
          continue;
        }
        CBState trial = reached_tile->cb;
        if (!leaseConcreteTerminal(trial, reached_dst, pin,
              {TerminalEntryKind::dst, entry.joint, entry.joint2}, reached_owned)) {
          packing_note(std::format("GROUNDING_REJECT pin={} dst={} joint={} joint2={} reason=terminal-occupied",
              pin, reached_dst, entry.joint, entry.joint2));
          continue;
        }
        candidate_terminal_states.clear();
        candidate_terminal_pins.clear();
        candidate_terminal_local_owners.clear();
        candidate_terminal_dst_owners.clear();
        candidate_terminal_joint_owners.clear();
        candidate_terminal_states.emplace(reached_tile, trial);
        candidate_terminal_pins.emplace(reached_tile, reached_tile->pin_state.leased_nodes);
        candidate_terminal_pins[reached_tile].setBit(pin);
        candidate_terminal_local_owners[reached_tile][pin] = route_task.net_name;
        candidate_terminal_dst_owners[reached_tile][reached_dst] = route_task.net_name;
        if (entry.joint >= 0)
          candidate_terminal_joint_owners[reached_tile][entry.joint] = route_task.net_name;
        if (entry.joint2 >= 0)
          candidate_terminal_joint_owners[reached_tile][entry.joint2] = route_task.net_name;
        packing_note(std::format("TRIGGER_RESERVED net={} cb=({},{}) local={} dst={} joint={} joint2={}",
            route_task.net_name, reached_tile->coord.x, reached_tile->coord.y,
            pin, reached_dst, entry.joint, entry.joint2));
        bool supported = true;
        const auto support_start = std::chrono::steady_clock::now();
        for (size_t i : terminal_order) {
          const auto &other = candidate_route_tasks[i];
          if (sameRouteTask(other, route_task)) continue;
          if (!placement_supports_route_task(other)) { supported = false; break; }
        }
        move_scan_support_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - support_start).count();
        if (!supported) {
          packing_note("OTHER_ROUTE_SUPPORT_REJECT " + placement_reject_reason);
          ++move_reject_support; continue;
        }
        Wire terminal;
        terminal.from = terminal.to = reached_tile->coord;
        terminal.local = reached_dst;
        terminal.jump = -1;
        terminal.joint = entry.joint;
        terminal.joint2 = entry.joint2;
        terminal.pos = reached_owned ? ROUTE_POS_FORK : ROUTE_POS_TRANSIT;
        terminal.owns_dst = !reached_owned;
        Wire endpoint;
        endpoint.type = Wire::WIRE_TILE_PIN;
        endpoint.from = endpoint.to = reached_tile->coord;
        endpoint.local = pin;
        fillTilePinEndpoint(endpoint, *route_task.to, route_task.to_port,
                            fpga::TILE_PIN_INPUT);
        refreshTilePinEndpoint(endpoint, *route_task.to, route_task.to_port,
                               fpga::TILE_PIN_INPUT);
        selected_terminal = {terminal, endpoint};
        return true;
      }
      if (!matching_entries)
        packing_note(std::format("GROUNDING_REJECT pin={} dst={} reason=no-terminal-entry-from-reached-dst", pin, reached_dst));
      return false;
    });
    if (!accepted) {
      ++move_reject_route;
      if (placement_reject_reason.empty())
        placement_reject_reason = "no usable grounding from reached DST; pins=" + pins.str();
    }
    return accepted;
  };
  auto scan_move_candidate = [&]() {
    forward = routeForwardToPlacement(anchors,
        [&](Tile &tile, int dst, bool owned, std::vector<Wire> &terminal) {
          reached_tile = &tile;
          reached_dst = dst;
          reached_owned = owned;
          if (packing_trace) {
            std::ostringstream details;
            printMovingCandidateState(details, tile, candidate_base(&tile));
            packing_note(details.str());
          }
          std::vector<Tile *> resources{&tile};
          for (Tile *resource : tile.attached_resource_tiles)
            if (resource && resource != &tile) resources.push_back(resource);
          for (Tile *resource : resources) {
            if (move_search_cancelled()) return false;
            if (force_move_target && (resource->coord.x != forced_move_x ||
                                      resource->coord.y != forced_move_y)) continue;
            ++move_candidates;
            placement_reject_reason.clear();
            reset_trial_diagnostic();
            if (replaying) {
              replay_frame = 0;
              replay_stem = (std::filesystem::path(failure_candidate_directory) /
                  std::format("{:03}_cb_{}_{}_dst_{}_resource_{}_{}", ++replay_candidate,
                      tile.coord.x, tile.coord.y, dst, resource->coord.x, resource->coord.y)).string();
              replay_report.open(replay_stem + ".txt", std::ios::trunc);
              replay_report << "net=" << route_task.net_name << "\nphase=diagnostic-replay\n";
              if (packing_trace) { std::fflush(packing_trace.get()); replay_trace_begin = std::ftell(packing_trace.get()); }
              replay_manifest << "CANDIDATE " << replay_candidate << " report=" << std::quoted(replay_stem + ".txt") << '\n';
            }
            struct ReportScope {
              std::function<void()> finish;
              ~ReportScope() { finish(); }
            } report_scope{[&]() {
              if (!replaying) return;
              if (packing_trace) {
                std::fflush(packing_trace.get());
                const long end = std::ftell(packing_trace.get());
                if (replay_trace_begin >= 0 && end >= replay_trace_begin &&
                    std::fseek(packing_trace.get(), replay_trace_begin, SEEK_SET) == 0) {
                  char buffer[8192];
                  long left = end - replay_trace_begin;
                  while (left > 0) {
                    size_t count = std::fread(buffer, 1, std::min<long>(left, sizeof(buffer)), packing_trace.get());
                    if (!count) break;
                    replay_report.write(buffer, count);
                    left -= count;
                  }
                  std::fseek(packing_trace.get(), end, SEEK_SET);
                }
              }
              replay_report.flush();
              const bool written = replay_report.good();
              replay_report.close();
              replay_report.clear();
              replay_manifest << "REPORT_WRITTEN " << written << " frames=" << replay_frame << '\n';
              replay_manifest.flush();
              PNR_LOG("ROUT", "Moving candidate replay: candidate={} frames={} report='{}' written={}",
                      replay_candidate, replay_frame, replay_stem + ".txt", written);
            }};
            packing_note(std::format("CANDIDATE reached=({},{}) dst={} owned={} resource=({},{}) type={}",
                tile.coord.x, tile.coord.y, dst, owned, resource->coord.x, resource->coord.y,
                resource->tile_type ? resource->tile_type->name : "<missing>"));
            if (replaying) {
              std::ostringstream details;
              printMovingCandidateState(details, tile, candidate_base(&tile));
              // Audit only committed occupancy. Prefix-only leases are printed
              // separately above and must not appear as ownerless live routes.
              {
                const CBState search_state = tile.cb;
                struct RestoreCB { Tile &tile; CBState state; ~RestoreCB() { tile.cb = state; } } restore{tile, search_state};
                tile.cb = candidate_base(&tile);
                fpga::auditTileCongestion(tile, details, replay_nets);
              }
              packing_note(details.str());
            }
            auto trace_placement = [&](const std::string &reason) {
              packing_note("CANDIDATE_RESULT " + reason);
              if (!RouteCongestionTrace::current) return;
              RouteCongestionTrace::current->stream()
                  << "FORWARD_PLACEMENT reached=(" << tile.coord.x << ',' << tile.coord.y
                  << ") dst=" << dst << " resource=(" << resource->coord.x << ','
                  << resource->coord.y << ") reason=" << std::quoted(reason) << '\n';
            };
            if (resource == old_tile) {
              trace_placement("original placement excluded");
              replay_picture("original placement excluded");
              ++move_reject_old_tile; continue;
            }
            if (!resource->tile_type || resource->tile_type->elements.empty()) {
              trace_placement("no packing resources");
              replay_picture("no packing resources");
              ++move_reject_no_tile;
              continue;
            }
            std::vector<rtl::Inst *> placed;
            const auto begin = std::chrono::steady_clock::now();
            bool packed = place_move_cluster(*resource, &placed, true);
            move_scan_place_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - begin).count();
            if (!packed) {
              trace_placement(placement_reject_reason);
              if (replaying && replay_frame == 0) replay_picture(placement_reject_reason);
              ++move_reject_place; continue;
            }
            trace_placement("accepted");
            new_tile = resource;
            new_pos = inst->pos;
            terminal = selected_terminal;
            return true;
          }
          return false;
        }, move_search_cancelled,
        replaying ? &replay_decisions : RouteCongestionTrace::current ? &RouteCongestionTrace::current->stream() : nullptr,
        [&](const auto &saved) {
          if (replaying) ++replay_states;
          forward_base_states.clear();
          for (const auto &[tile, cb] : saved) forward_base_states.emplace(tile, cb);
        });
    move_scan_timed_out = move_scan_timed_out || forward.cancelled;
    PNR_LOG1("ROUT", "routeDesign Moving forward growth: net='{}', anchors={}, "
             "reached={}, placement_probes={}, visit_rejects={}, grounded={}, cancelled={}, diagnostic_replay={}",
             route_task.net_name, anchors.size(), forward.expanded, move_candidates,
             forward.tile_visit_rejects, forward.success, forward.cancelled, replaying);
    return forward.success;
  };

  if (tried.size() < moving_candidate_retry_limit && scan_move_candidate()) {
    bool candidate_remains_placed =
        inst->tile.peer == new_tile && inst->pos == new_pos;
    bool final_placement_was_tried =
        candidate_remains_placed &&
        placementWasTried(tried, new_tile->coord, new_pos);
    if (!pnr::acceptMovingPlacedCandidate(candidate_remains_placed,
                                          final_placement_was_tried)) {
      if (candidate_remains_placed) {
        unplace_detached_passthroughs("reject-invalid-generated-endpoints");
        unplace_move_cluster("reject-invalid-moving-candidate");
      }
      new_tile = nullptr;
      new_pos = -1;
    } else {
      tried.push_back(placementKey(new_tile->coord, new_pos));
      move_failed_scans.erase(inst_key);
      if (moving_debug) {
        PNR_LOG1(
            "ROUT",
            "routeDesign moving scan selected: inst='{}', selected=({},{})/{} "
            "scanned={}, placed={}, reject(place={},tried={},support={}), "
            "time_ms(place={:.3f},requirements={:.3f},support={:.3f})",
            inst->makeName(FULL_NAME_LIMIT), new_tile->coord.x,
            new_tile->coord.y, new_pos, move_candidates,
            move_scan_placed_candidates, move_reject_place,
            move_reject_tried, move_reject_support,
            static_cast<double>(move_scan_place_ns) / 1000000.0,
            static_cast<double>(move_scan_requirements_ns) / 1000000.0,
            static_cast<double>(move_scan_support_ns) / 1000000.0);
      }
      if (moving_debug) {
        PNR_LOG1("ROUT",
                 "routeDesign moving debug: selected ({},{})/{} hops={} "
                 "cluster={} for route '{}'",
                 new_tile->coord.x, new_tile->coord.y, new_pos, forward.fragments.size(),
                 move_cluster.size(), route_task.net_name);
      }
    }
  }

  if (!new_tile) {
    if (move_scan_timed_out) {
      PNR_LOG1(
          "ROUT",
          "routeDesign moving scan timeout: inst='{}', scanned={}, placed={}, "
          "reject(place={},tried={},support={}), time_ms(place={:.3f},"
          "requirements={:.3f},support={:.3f})",
          inst->makeName(FULL_NAME_LIMIT), move_candidates,
          move_scan_placed_candidates, move_reject_place, move_reject_tried,
          move_reject_support,
          static_cast<double>(move_scan_place_ns) / 1000000.0,
          static_cast<double>(move_scan_requirements_ns) / 1000000.0,
          static_cast<double>(move_scan_support_ns) / 1000000.0);
      set_fail("Moving stage deadline reached during candidate scan");
      restore_move_cluster();
      restore_detached_passthroughs();
      return false;
    }
    move_failed_scans[inst_key] = failed_scans + 1;
    // Failed placement scans are common while congestion is being moved;
    // only print the full grid-scan breakdown for a selected debug route.
    if (moving_debug) {
      PNR_LOG1(
          "ROUT",
          "routeDesign moving debug: no candidate for inst='{}' type='{}' "
          "route='{}' old=({},{})/{}, scanned={}, no_tile={}, old_tile={}, "
          "cannot_place={}, already_tried={}, unsupported={}, unroutable={}",
          inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type,
          route_task.net_name, old_coord.x, old_coord.y, old_pos,
          move_candidates, move_reject_no_tile, move_reject_old_tile,
          move_reject_place, move_reject_tried, move_reject_support,
          move_reject_route);
    }
    set_fail("no candidate old=(" + std::to_string(old_coord.x) + "," +
             std::to_string(old_coord.y) + ")/" + std::to_string(old_pos) +
             " scanned=" + std::to_string(move_candidates) +
             " no_tile=" + std::to_string(move_reject_no_tile) +
             " old_tile=" + std::to_string(move_reject_old_tile) +
             " cannot_place=" + std::to_string(move_reject_place) +
             " already_tried=" + std::to_string(move_reject_tried) +
             " unsupported=" + std::to_string(move_reject_support) +
             " unroutable=" + std::to_string(move_reject_route) +
             (placement_reject_reason.empty()
                  ? std::string{}
                  : " last_reason='" + placement_reject_reason + "'"));
    if (replay_failed_candidates && forward.expanded != 0) {
      const auto normal_forward = forward;
      const auto normal_reason = placement_reject_reason;
      const int normal_candidates = move_candidates;
      try {
        // Never overwrite or delete artifacts from an earlier task/run.
        const auto parent = std::filesystem::absolute(failureArtifactDirectory());
        for (size_t number = 1; ; ++number) {
          const auto directory = parent / std::format("routing_failure_candidates_{:03}", number);
          if (std::filesystem::create_directory(directory)) {
            failure_candidate_directory = directory.string();
            break;
          }
        }
        replay_manifest.open(std::filesystem::path(failure_candidate_directory) / "index.txt");
        replay_decisions.open(std::filesystem::path(failure_candidate_directory) / "forward_search.txt");
        if (!replay_manifest || !replay_decisions) throw std::runtime_error("cannot open candidate replay reports");
        replay_manifest << "net=" << route_task.net_name << "\nnormal_reached=" << forward.expanded
                        << "\nnormal_candidates=" << normal_candidates << "\nmode=diagnostic-only\n";
        std::unordered_set<rtl::Net *> registry;
        if (tech) {
          for (rtl::Module &module : tech->design.modules)
            for (rtl::Net &net : module.nets) registry.insert(&net);
        } else {
          for (const auto &[source, nets] : source_route_nets) registry.insert(nets.begin(), nets.end());
          for (int y = 0; y < fpga->size_height; ++y)
            for (int x = 0; x < fpga->size_width; ++x)
              if (Tile *tile = fpga->getTile(x, y))
                for (const auto &ref : tile->routedNets) if (ref.peer) registry.insert(ref.peer);
        }
        replay_nets.assign(registry.begin(), registry.end());
        unplace_detached_passthroughs("candidate-replay-start");
        unplace_move_cluster("candidate-replay-start");
        for (size_t i = 0; i < move_cluster.size(); ++i) move_cluster[i]->pos = scan_start_positions[i];
        replay_state_limit = normal_forward.expanded;
        replaying = true;
        move_candidates = 0;
        packing_note("CANDIDATE_REPLAY_BEGIN directory=" + failure_candidate_directory);
        scan_move_candidate();
        replay_manifest << "replayed_states=" << forward.expanded << "\nreplayed_candidates=" << replay_candidate
                        << "\nimages=" << replay_images << "\ncompleted=" << !forward.cancelled << '\n';
        packing_note(std::format("CANDIDATE_REPLAY_END candidates={} images={} completed={}",
                                 replay_candidate, replay_images, !forward.cancelled));
      } catch (const std::exception &error) {
        packing_note("CANDIDATE_REPLAY_ERROR " + std::string(error.what()));
        if (replay_manifest) replay_manifest << "error=" << error.what() << '\n';
      }
      replaying = false;
      replay_decisions.close();
      replay_manifest.close();
      forward = normal_forward;
      move_candidates = normal_candidates;
      placement_reject_reason = normal_reason;
    }
    restore_move_cluster();
    restore_detached_passthroughs();
    return false;
  }

  // The source-connected route is proven BEFORE accepting the placement.
  // Commit it before invalidating other incident bindings, and exclude this
  // now-complete trigger from the subsequent destination detachment set.
  PNR_ASSERT(forward.success, "Moving accepted a position without forward grounding");
  const auto &chosen = anchors[forward.anchor];
  const bool retains_trigger = anchor_retains_trigger[forward.anchor];
  if (!commitPreparedRoute(forward.fragments)) {
    restore_move_cluster();
    restore_detached_passthroughs();
    set_fail("forward grounding changed before commit");
    return false;
  }
  auto *binding = findNetRouteBinding(*route_task.net, *route_task.from,
      *route_task.to, route_task.from_port, route_task.to_port, route_task.net_name);
  if (retains_trigger) {
    auto *existing = binding ? routeBindingRoute(*binding) : nullptr;
    const size_t keep = chosen.prefix.size();
    if (!existing || keep == 0 || keep > existing->size() ||
        (keep < existing->size() &&
         !fpga::truncateNetRoute(*route_task.net,
                                binding - route_task.net->routes.data(), keep))) {
      fpga::releaseRouteLeases(forward.fragments);
      restore_move_cluster();
      restore_detached_passthroughs();
      set_fail("forward grounding could not retain its selected private prefix");
      return false;
    }
  }
  if (binding && !retains_trigger) {
    size_t index = binding - route_task.net->routes.data();
    fpga::discardNetBranch(*route_task.net, index);
  }
  // Truncation transfers ownership of the selected landing to the retained
  // prefix. Use that updated route, not the anchor's pre-truncation copy.
  std::vector<Wire> grounded = retains_trigger ? *routeBindingRoute(*binding)
                                              : chosen.prefix;
  if (!retains_trigger)
    for (auto &wire : grounded) wire.shared = true;
  grounded.insert(grounded.end(), forward.fragments.begin(), forward.fragments.end());
  if (!chosen.from_dst) prependSourceEndpoint(grounded, *route_task.from, route_task.from_port);
  for (auto &wire : grounded) wire.net_name = route_task.net_name;
  binding = findNetRouteBinding(*route_task.net, *route_task.from, *route_task.to,
      route_task.from_port, route_task.to_port, route_task.net_name);
  auto *slot = binding ? routeBindingRoute(*binding) : nullptr;
  if (slot) *slot = std::move(grounded);
  else {
    route_task.to->wires.push_back(std::move(grounded));
    slot = &route_task.to->wires.back();
  }
  size_t storage = findRouteIndex(*route_task.to, slot);
  size_t committed = fpga::attachNetRoute(*route_task.net, *route_task.to, storage,
      route_task.from, route_task.to, route_task.from_port, route_task.to_port,
      route_task.net_name);
  fpga::registerNetRouteTiles(*route_task.net, *slot, committed);
  auto is_grounded_trigger = [&](const rtl::NetRouteBinding &candidate) {
    return candidate.from == route_task.from && candidate.to == route_task.to &&
           candidate.from_port == route_task.from_port &&
           candidate.to_port == route_task.to_port && candidate.route_name == route_task.net_name;
  };
  route_changed = route_progress = true;
  for (rtl::Inst *member : move_cluster) {
    if (!member || !member->tile.peer) {
      continue;
    }
    member->coord = member->tile->coord;
    member->outline.x =
        (member->tile->coord.x + 0.25f * static_cast<float>(member->pos % 4)) /
        aspect_x;
    member->outline.y =
        (member->tile->coord.y + 0.25f * static_cast<float>(member->pos / 4)) /
        aspect_y;
  }

  size_t unrouted = 1; // The triggering connection is already repaired.
  std::vector<RouteTask> moved_constant_tasks;
  auto appendMovedRouteTask = [&](const RouteTask &route_task) {
    if (route_task.net && route_task.net->distributed_source) {
      appendUniqueRouteTask(moved_constant_tasks, route_task);
      return;
    }
    if (!moved_tasks) {
      enqueueRouteTask(route_task, pending_route_todo);
      return;
    }
    for (RouteTask &old : *moved_tasks) {
      if (!sameRouteTask(old, route_task)) {
        continue;
      }
      old.fanout = old.fanout || route_task.fanout;
      return;
    }
    moved_tasks->push_back(route_task);
  };
  RouteTask saved_debug_active_task = debug_active_route_task;
  bool saved_debug_active_task_valid = debug_active_route_task_valid;
  debug_active_route_task = route_task;
  debug_active_route_task_valid = true;
  if (!incident_route_nets.empty()) {
    std::vector<fpga::NetRouteRef> moved_sink_routes;
    for (rtl::Net *net : incident_route_nets) {
      if (!net) {
        continue;
      }
      // A moved clock sink is repaired by its dedicated tree router after
      // ordinary routing. Keep the old protected leases reserved until then.
      if (pnr::movementDefersProtectedTree(net->route_protected,
                                           net->distributed_source)) {
        clock_routes_dirty = std::any_of(
                                 net->routes.begin(), net->routes.end(),
                                 [&](const rtl::NetRouteBinding &binding) {
                                   return moved_endpoint(binding.from) ||
                                          moved_endpoint(binding.to);
                                 }) ||
                             clock_routes_dirty;
        continue;
      }
      for (size_t route_index = 0; route_index < net->routes.size();
           ++route_index) {
        const rtl::NetRouteBinding &binding = net->routes[route_index];
        // Co-moved sinks form one atomic invalidation set; routes starting at
        // moved sources are handled by source-tree invalidation below.
        if (!moved_endpoint(binding.from) && moved_endpoint(binding.to) &&
            !is_grounded_trigger(binding)) {
          moved_sink_routes.push_back({net, route_index});
        }
      }
    }
    fpga::invalidateMovedSinkRoutes(moved_sink_routes);
    for (const fpga::NetRouteRef &route_ref : moved_sink_routes) {
      if (!route_ref.net || route_ref.net->routeCanBePreempted() ||
          route_ref.net->distributed_source) {
        continue;
      }
      // Protected infrastructure will choose a fresh branch from its live
      // tree; an old shared-prefix replica must not bias that later search.
      fpga::discardNetBranch(*route_ref.net, route_ref.binding_index);
    }

    std::vector<std::pair<rtl::Inst *, std::string>> unrouted_sources;
    auto already_unrouted_source = [&](rtl::Inst *source,
                                       const std::string &port) {
      return std::any_of(unrouted_sources.begin(), unrouted_sources.end(),
                         [&](const auto &old) {
                           return old.first == source && old.second == port;
                         });
    };
    for (rtl::Net *net_ptr : incident_route_nets) {
      if (!net_ptr) {
        continue;
      }
      rtl::Net &net = *net_ptr;
      // Distributed protected sources are ordinary Moving work; only other
      // infrastructure nets retain a separate repair owner.
      if (!net.routeCanBePreempted() && !net.distributed_source) {
        continue;
      }
      for (size_t route_index = 0; route_index < net.routes.size();
           ++route_index) {
        rtl::NetRouteBinding binding = net.routes[route_index];
        if (is_grounded_trigger(binding)) continue;
        if (!moved_endpoint(binding.from) && !moved_endpoint(binding.to)) {
          continue;
        }
        if (!binding.from || binding.from_port.empty()) {
          continue;
        }
        // A moved cell invalidates every route touching it, even when
        // the opposite endpoint was already completed earlier.
        if (moved_endpoint(binding.from)) {
          if (already_unrouted_source(binding.from, binding.from_port)) {
            continue;
          }
          unrouted_sources.push_back({binding.from, binding.from_port});
          if (moving_debug) {
            PNR_LOG1("ROUT",
                     "routeDesign moving debug: unroute moved source='{}' "
                     "port='{}' because route='{}' starts at moved inst",
                     binding.from ? binding.from->makeName(FULL_NAME_LIMIT)
                                  : std::string{},
                     binding.from_port, binding.route_name);
          }
          unrouted += unrouteSourceTree(net, binding.from, binding.from_port,
                                        moved_tasks, false);
          continue;
        }

        std::vector<Wire> *sink_route = routeBindingRoute(binding);
        rtl::NetRouteBinding *current_binding = &net.routes[route_index];
        bool has_external_seed =
            binding.from &&
            hasOtherCompleteSourceBinding(*binding.from, binding.from_port,
                                          current_binding);
        size_t sink_route_size_before = sink_route ? sink_route->size() : 0;
        bool binding_debug =
            moving_debug ||
            routeDebugMatches("SCALEPNR_DEBUG_TASK_NET", binding.route_name);
        size_t shared_prefix_before = 0;
        if (sink_route) {
          while (shared_prefix_before < sink_route->size() &&
                 (*sink_route)[shared_prefix_before].shared) {
            ++shared_prefix_before;
          }
        }
        std::vector<Wire> route_before_invalidation =
            binding_debug && sink_route ? *sink_route : std::vector<Wire>{};
        if (binding_debug) {
          PNR_LOG1("ROUT",
                   "routeDesign moving debug: unroute branch net='{}' "
                   "from='{}'/'{}' to='{}'/'{}' because moved inst is sink; "
                   "fragments={}, shared_prefix={}, private_or_nonprefix={}",
                   binding.route_name,
                   binding.from ? binding.from->makeName(FULL_NAME_LIMIT)
                                : std::string{},
                   binding.from_port,
                   binding.to ? binding.to->makeName(FULL_NAME_LIMIT)
                              : std::string{},
                   binding.to_port, sink_route_size_before,
                   shared_prefix_before,
                   sink_route_size_before - shared_prefix_before);
        }
        // Every moved sink invalidates its binding, including bindings
        // whose route is already empty. Retain only a reusable source or
        // shared prefix and always schedule the new destination.
        // Moving changes only the sink endpoint, so retain the committed
        // source takeoff and reroute the invalid destination suffix.
        // The complete moved-sink set was detached atomically before this
        // scheduling pass, including bindings whose route was already empty.
        bool branch_removed = true;
        bool needs_reroute = branch_removed;
        if (binding_debug) {
          PNR_LOG1("ROUT",
                   "routeDesign moving debug: branch removal net='{}' "
                   "binding_removed={} sink_route_found={} external_seed={} "
                   "needs_reroute={} size_before={} size_after={}",
                   binding.route_name, branch_removed, sink_route != nullptr,
                   has_external_seed, needs_reroute, sink_route_size_before,
                   sink_route ? sink_route->size() : 0);
          size_t retained = sink_route ? sink_route->size() : 0;
          if (retained < route_before_invalidation.size()) {
            std::vector<Wire> removed(route_before_invalidation.begin() +
                                          static_cast<std::ptrdiff_t>(retained),
                                      route_before_invalidation.end());
            logMovedSuffixLeaseAudit(binding.route_name, removed);
          }
        }
        if (needs_reroute) {
          RouteTask branch_task{binding.from,
                                binding.to,
                                &net,
                                binding.from_port,
                                binding.to_port,
                                binding.route_name,
                                0,
                                0,
                                0,
                                {},
                                !net.distributed_source && has_external_seed};
          if (binding_debug) {
            PNR_LOG1("ROUT",
                     "routeDesign moving debug: schedule branch task fanout={} "
                     "net='{}'",
                     branch_task.fanout, branch_task.net_name);
          }
          ++unrouted;
          appendMovedRouteTask(branch_task);
        }
      }
    }
  }
  debug_active_route_task = std::move(saved_debug_active_task);
  debug_active_route_task_valid = saved_debug_active_task_valid;

  if (moved_tasks) {
    // Keep the triggering route alive when it has no existing NetRouteBinding
    // yet.
    RouteTask live_task = route_task;
    // A moved endpoint needs one ordinary route at the new tile before fanout
    // branching can resume.
    if (moved_endpoint(live_task.from) || moved_endpoint(live_task.to)) {
      const std::vector<Wire> *live_route = findBoundRoute(
          live_task.net, live_task.from, live_task.to, live_task.from_port,
          live_task.to_port, live_task.net_name);
      live_task.fanout =
          !(live_task.net && live_task.net->distributed_source) &&
          (live_task.fanout ||
          (moved_endpoint(live_task.to) &&
           routeStartsWithSharedPrefix(live_route) && live_task.from &&
           sourceTreeHasCompleteExit(*live_task.from, live_task.from_port)));
    }
    bool live_task_already_scheduled =
        std::any_of(moved_tasks->begin(), moved_tasks->end(),
                    [&](const RouteTask &scheduled) {
                      return sameRouteTask(scheduled, live_task);
                    });
    if (!live_task_already_scheduled) {
      appendUniqueRouteTask(*moved_tasks, live_task);
      ++unrouted;
    } else if (unrouted == 0) {
      unrouted = 1;
    }

    // A relocation owns the complete incident task set. Bindings that were
    // already empty still need to be routed before this focus can finish.
    pnr::enqueueIncompleteAffectedTasks(
        candidate_route_tasks,
        [&](const RouteTask &candidate_task) {
          const std::vector<Wire> *candidate_route =
              findBoundRoute(candidate_task.net, candidate_task.from,
                             candidate_task.to, candidate_task.from_port,
                             candidate_task.to_port, candidate_task.net_name);
          return candidate_route && routeIsComplete(*candidate_route);
        },
        [&](const RouteTask &candidate_task) {
          size_t before = moved_tasks->size();
          appendMovedRouteTask(candidate_task);
          return moved_tasks->size() != before;
        });
    pnr::prioritizeMovingTrigger(
        *moved_tasks, live_task,
        [&](const RouteTask &queued, const RouteTask &trigger) {
          return sameRouteTask(queued, trigger);
        });
    if (moving_debug) {
      PNR_LOG1("ROUT",
               "routeDesign moving debug: moved task list has {} tasks after "
               "moving inst='{}'",
               moved_tasks->size(), inst->makeName(FULL_NAME_LIMIT));
      for (const RouteTask &moved_task : *moved_tasks) {
        PNR_LOG1("ROUT",
                 "routeDesign moving debug: task fanout={} net='{}' "
                 "from='{}'/'{}' to='{}'/'{}'",
                 moved_task.fanout, moved_task.net_name,
                 moved_task.from ? moved_task.from->makeName(FULL_NAME_LIMIT)
                                 : std::string{},
                 moved_task.from_port,
                 moved_task.to ? moved_task.to->makeName(FULL_NAME_LIMIT)
                               : std::string{},
                 moved_task.to_port);
      }
    }
  }
  std::string constant_reroute_failure;
  if (!routeConstantTasksImmediately(moved_constant_tasks,
                                     &constant_reroute_failure)) {
    PNR_ASSERT(false, "{}", constant_reroute_failure);
  }
  if (unrouted == 0) {
    restore_move_cluster();
    restore_detached_passthroughs();
    set_fail("move produced no affected routes");
    return false;
  }

  if (moving_debug || envFlagEnabled("SCALEPNR_ROUTE_MOVE_DETAIL")) {
    PNR_LOG1("ROUT",
             "routeDesign moving: inst='{}' type='{}' ({},{})/{} -> "
             "({},{})/{}, tried={}, unrouted_routes={}",
             inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type, old_coord.x,
             old_coord.y, old_pos, new_tile->coord.x, new_tile->coord.y,
             new_pos, tried.size(), unrouted);
  }
  // Capture destination occupancy immediately after relocation and before
  // rerouting changes it.
  if (moved_tasks && envFlagEnabled("SCALEPNR_ROUTE_MOVE_TARGET_DUMP")) {
    for (size_t task_index = 0; task_index < moved_tasks->size();
         ++task_index) {
      const RouteTask &moved_task = (*moved_tasks)[task_index];
      if (routeDebugMatches("SCALEPNR_DEBUG_TASK_NET", moved_task.net_name)) {
        dumpTargetCandidateSummaryLog(moved_task, task_index);
      }
    }
  }
  return true;
}

// Check a hypothetical source placement against all live output ports. The
// required port must drive the exact reverse-route SRC selected by docking.
bool RouteDesign::routeOutTry(rtl::Inst &inst, Tile &resource_tile, int pos,
                              const std::string &required_port,
                              Tile &required_route_tile, int required_src,
                              BackwardTakeoffChoice &takeoff,
                              std::string *fail_reason,
                              const BackwardTakeoffStateView *state_view) {
  auto fail = [&](const std::string &reason) {
    if (fail_reason) {
      *fail_reason = reason;
    }
    return false;
  };
  const bool debug_takeoff =
      routeDebugMatches("SCALEPNR_DEBUG_TASK_NET",
                        inst.makeName(FULL_NAME_LIMIT));
  auto debug_node_name = [](const CBType &type, fpga::CBNodeNameType node_type,
                            int node) {
    const std::string *name = type.nodeName(node_type, node);
    return name ? *name : std::string{};
  };
  if (!inst.tile.peer || !inst.cell_ref.peer || pos < 0 ||
      required_src < 0) {
    return fail("invalid placement or required source node");
  }

  Tile *saved_tile = &*inst.tile;
  Coord saved_coord = inst.coord;
  int saved_pos = inst.pos;
  inst.tile.clear();
  inst.tile.set(static_cast<Referable<Tile> *>(&resource_tile));
  inst.coord = resource_tile.coord;
  inst.pos = pos;
  auto restore = [&]() {
    inst.tile.clear();
    inst.tile.set(static_cast<Referable<Tile> *>(saved_tile));
    inst.coord = saved_coord;
    inst.pos = saved_pos;
  };

  struct OutputPort {
    std::string name;
    rtl::Net *net = nullptr;
    Coord target;
  };
  std::vector<OutputPort> outputs;
  auto append_output = [&](const std::string &name, rtl::Net *net,
                           Coord target) {
    if (name.empty() ||
        std::any_of(outputs.begin(), outputs.end(), [&](const OutputPort &old) {
          return old.name == name;
        })) {
      return;
    }
    outputs.push_back({name, net, target});
  };
  for (rtl::Conn &conn : inst.conns) {
    if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT ||
        conn.port_ref->is_global || conn.peer) {
      continue;
    }
    Coord target = resource_tile.coord;
    for (auto *sink_ref : rtl::Conn::getSinks(conn)) {
      rtl::Conn *sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
      if (sink && sink->inst_ref.peer && sink->inst_ref->tile.peer) {
        target = sink->inst_ref->tile->coord;
        break;
      }
    }
    append_output(conn.port_ref->makeName(),
                  findNetByDesignator(inst, conn.port_ref->designator), target);
  }
  append_output(required_port, nullptr, required_route_tile.coord);
  std::stable_partition(outputs.begin(), outputs.end(),
                        [&](const OutputPort &port) {
                          return port.name == required_port;
                        });

  std::unordered_map<Tile *, CBState> states;
  std::unordered_map<Tile *, NodeMask> reserved_locals;
  auto initial_state = [&](Tile *tile) {
    if (state_view) {
      auto found = state_view->find(tile);
      if (found != state_view->end()) {
        return found->second;
      }
    }
    return tile->cb;
  };
  bool required_selected = false;
  for (const OutputPort &output : outputs) {
    std::vector<Tile *> route_tiles =
        routeTileCandidates(inst, output.name, true, &output.target);
    if (output.name == required_port) {
      route_tiles.erase(
          std::remove_if(route_tiles.begin(), route_tiles.end(),
                         [&](Tile *tile) { return tile != &required_route_tile; }),
          route_tiles.end());
    }
    bool selected = false;
    bool required_topology_candidate = false;
    if (debug_takeoff) {
      PNR_LOG1("ROUT",
               "routeOutTry output: inst='{}', resource=({},{})/{}, "
               "port='{}', required={}, route_candidates={}",
               inst.makeName(FULL_NAME_LIMIT), resource_tile.coord.x,
               resource_tile.coord.y, pos, output.name,
               output.name == required_port, route_tiles.size());
    }
    for (Tile *route_tile : route_tiles) {
      if (!route_tile || !route_tile->cb_type) {
        continue;
      }
      NodeMask locals =
          routeTileEndpointNodes(*route_tile, inst, output.name, true);
      if (debug_takeoff) {
        PNR_LOG1("ROUT",
                 "routeOutTry route tile: coord=({},{}) type='{}', "
                 "port='{}', locals={}, required_src={}",
                 route_tile->coord.x, route_tile->coord.y,
                 route_tile->cb_type->name, output.name, locals.str(),
                 output.name == required_port ? required_src : -1);
      }
      selected = locals.for_each_set_bit([&](int local) {
        const bool reserved = reserved_locals[route_tile].testBit(local);
        const bool foreign_owner = sourceLocalOwnedByDifferentEndpoint(
            *route_tile, local, inst, output.name, output.net);
        if (reserved || foreign_owner) {
          if (debug_takeoff) {
            PNR_LOG1("ROUT",
                     "routeOutTry local reject: coord=({},{}) local={} "
                     "name='{}', reserved={}, foreign_owner={}",
                     route_tile->coord.x, route_tile->coord.y, local,
                     debug_node_name(*route_tile->cb_type,
                                     fpga::CB_NODE_LOCAL, local),
                     reserved, foreign_owner);
          }
          return false;
        }
        const std::vector<uint16_t> *available =
            route_tile->cb_type->srcNodes(fpga::CB_NODE_LOCAL, local);
        if (!available) {
          if (debug_takeoff) {
            PNR_LOG1("ROUT",
                     "routeOutTry local reject: coord=({},{}) local={} "
                     "name='{}', reason=no_src_nodes",
                     route_tile->coord.x, route_tile->coord.y, local,
                     debug_node_name(*route_tile->cb_type,
                                     fpga::CB_NODE_LOCAL, local));
          }
          return false;
        }
        std::vector<uint16_t> exact;
        const std::vector<uint16_t> *ordered = available;
        if (output.name == required_port) {
          exact.push_back(static_cast<uint16_t>(required_src));
          ordered = &exact;
        } else {
          ordered = &route_tile->cb_type->orderedSrcNodes(
              fpga::CB_NODE_LOCAL, local, output.target - route_tile->coord);
        }
        NodeMask available_mask;
        for (uint16_t src : *available) {
          available_mask.setBit(src);
        }
        for (uint16_t src : *ordered) {
          if (!available_mask.testBit(src)) {
            if (debug_takeoff) {
              PNR_LOG1("ROUT",
                       "routeOutTry src reject: coord=({},{}) local={} "
                       "src={} name='{}', reason=not_connected",
                       route_tile->coord.x, route_tile->coord.y, local, src,
                       debug_node_name(*route_tile->cb_type, fpga::CB_NODE_SRC,
                                       src));
            }
            continue;
          }
          if (route_tile->cb_type->dst_by_src[src].empty()) {
            if (debug_takeoff) {
              PNR_LOG1("ROUT",
                       "routeOutTry src reject: coord=({},{}) local={} "
                       "src={} name='{}', reason=no_dst_by_src",
                       route_tile->coord.x, route_tile->coord.y, local, src,
                       debug_node_name(*route_tile->cb_type, fpga::CB_NODE_SRC,
                                       src));
            }
            continue;
          }
          int joint = -1;
          int joint2 = -1;
          if (!route_tile->cb_type->canOut(local, src, src, joint, &joint2)) {
            if (debug_takeoff) {
              PNR_LOG1("ROUT",
                       "routeOutTry src reject: coord=({},{}) local={} "
                       "src={} name='{}', reason=canOut",
                       route_tile->coord.x, route_tile->coord.y, local, src,
                       debug_node_name(*route_tile->cb_type, fpga::CB_NODE_SRC,
                                       src));
            }
            continue;
          }
          if (output.name == required_port) {
            required_topology_candidate = true;
          }
          CBState trial = states.contains(route_tile) ? states[route_tile]
                                                      : initial_state(route_tile);
          if (!leaseConcreteOut(trial, local, src, joint, true, joint2)) {
            if (debug_takeoff) {
              PNR_LOG1("ROUT",
                       "routeOutTry src reject: coord=({},{}) local={} "
                       "src={} name='{}', joint={}/{} leased(local={},src={},"
                       "joint={},joint2={})",
                       route_tile->coord.x, route_tile->coord.y, local, src,
                       debug_node_name(*route_tile->cb_type, fpga::CB_NODE_SRC,
                                       src),
                       joint, joint2, trial.local.local.testBit(local),
                       trial.src.jump.testBit(src),
                       joint >= 0 && trial.joint.jump.testBit(joint),
                       joint2 >= 0 && trial.joint.jump.testBit(joint2));
            }
            continue;
          }
          states[route_tile] = trial;
          reserved_locals[route_tile].setBit(local);
          if (output.name == required_port) {
            takeoff = BackwardTakeoffChoice{local, joint, joint2};
            required_selected = true;
          }
          return true;
        }
        return false;
      });
      if (selected) {
        break;
      }
    }
    if (!selected) {
      if (output.name == required_port && !required_topology_candidate) {
        // The reverse frontier reached a SRC that this placement cannot drive.
        // It is not a placement attempt and must not consume the bounded probe
        // window before a structurally compatible SRC is examined.
        takeoff.counts_toward_probe_limit = false;
      }
      restore();
      return fail("output port '" + output.name +
                  "' has no simultaneously free takeoff");
    }
  }

  // Reserve one concrete terminal path for every ordinary input in the same
  // temporary CB states. A placement whose local input cannot be reached is
  // rejected before any live cell, route, or lease is changed.
  for (rtl::Conn &conn : inst.conns) {
    if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN ||
        conn.port_ref->is_global ||
        tech->check_clocked(inst.cell_ref->type, conn.port_ref->name)) {
      continue;
    }
    rtl::Conn *driver = conn.follow();
    if (!driver || !driver->inst_ref.peer || !driver->port_ref.peer) {
      continue;
    }
    const bool constant_input =
        driver == tech->design.GND || driver == tech->design.VCC;
    const bool input_one = driver == tech->design.VCC;
    rtl::Net *input_net =
        findNetByDesignator(inst, conn.port_ref->designator);
    const bool void_net =
        input_net && input_net->designatorIsVoid(conn.port_ref->designator);
    if (!pnr::movingSourceInputNeedsFabricTerminal(true, void_net)) {
      continue;
    }
    std::string input_name = conn.port_ref->makeName();
    bool selected = false;
    for (Tile *route_tile : routeTileCandidates(inst, input_name, false)) {
      if (!route_tile || !route_tile->cb_type) {
        continue;
      }
      NodeMask pins = routeTileInputNodes(*route_tile, inst, input_name);
      if (debug_takeoff) {
        PNR_LOG1("ROUT",
                 "routeOutTry input: inst='{}', resource=({},{})/{}, "
                 "route_tile=({},{}) type='{}', port='{}', pins={}",
                 inst.makeName(FULL_NAME_LIMIT), resource_tile.coord.x,
                 resource_tile.coord.y, pos, route_tile->coord.x,
                 route_tile->coord.y, route_tile->cb_type->name, input_name,
                 pins.str());
      }
      selected = pins.for_each_set_bit([&](int pin) {
        const bool pin_leased = route_tile->isPinNodeLeased(pin);
        const bool candidate_reserved =
            reserved_locals[route_tile].testBit(pin);
        bool same_driver =
            pin_leased &&
            fpga::inputLocalReservedByDriver(*route_tile, pin, driver);
        bool live_same_driver_owner = false;
        if (pin_leased && (same_driver || constant_input)) {
          for (const fpga::NetRouteRef &owner : fpga::findNetRoutesByNode(
                   *route_tile, fpga::CB_NODE_LOCAL, pin, false)) {
            if (!owner.net || owner.binding_index >= owner.net->routes.size()) {
              continue;
            }
            const rtl::NetRouteBinding &binding =
                owner.net->routes[owner.binding_index];
            // Distributed sources have generated route endpoints, not the
            // logical constant Conn. Match their value only on a live owner.
            if (pnr::movingSourceInputOwnerMatches(
                    constant_input, input_one, owner.net->distributed_source,
                    owner.net->distributed_one,
                    !constant_input &&
                        samePhysicalSource(binding.from, binding.from_port,
                                           driver->inst_ref.peer,
                                           driver->port_ref->makeName()))) {
              same_driver = true;
              live_same_driver_owner = true;
              break;
            }
          }
        }
        if (!pnr::movingSourceInputTerminalAvailable(
                pin_leased, candidate_reserved, same_driver,
                live_same_driver_owner)) {
          if (debug_takeoff) {
            PNR_LOG1(
                "ROUT",
                "routeOutTry input local reject: coord=({},{}) local={} "
                "name='{}', pin_leased={}, reserved={}, same_driver={}, "
                "live_same_driver_owner={}, constant={}, one={}, owner='{}'",
                route_tile->coord.x, route_tile->coord.y, pin,
                debug_node_name(*route_tile->cb_type, fpga::CB_NODE_LOCAL,
                                pin),
                pin_leased, candidate_reserved, same_driver,
                live_same_driver_owner, constant_input, input_one,
                nodeOwnerForDump(route_tile, fpga::CB_NODE_LOCAL, pin, false));
          }
          return false;
        }
        // The existing route already owns this same-driver local and terminal;
        // immediate Fanout routing attaches the moved sink to that route tree.
        if (same_driver) {
          if (debug_takeoff) {
            PNR_LOG1("ROUT",
                     "routeOutTry input local share: coord=({},{}) local={} "
                     "name='{}', port='{}', constant={}, one={}",
                     route_tile->coord.x, route_tile->coord.y, pin,
                     debug_node_name(*route_tile->cb_type,
                                     fpga::CB_NODE_LOCAL, pin),
                     input_name, constant_input, input_one);
          }
          return true;
        }
        for (const CBType::TerminalEntry &entry :
             route_tile->cb_type->terminalEntries(pin)) {
          CBState trial = states.contains(route_tile) ? states[route_tile]
                                                      : initial_state(route_tile);
          if (!leaseConcreteIn(trial, entry.dst, pin, entry.joint,
                               entry.joint2)) {
            if (debug_takeoff) {
              PNR_LOG1(
                  "ROUT",
                  "routeOutTry input path reject: coord=({},{}) local={} "
                  "dst={} joint={}/{} leased(local={},dst={},joint={},"
                  "joint2={}) owners(local='{}',dst='{}',joint='{}',"
                  "joint2='{}')",
                  route_tile->coord.x, route_tile->coord.y, pin, entry.dst,
                  entry.joint, entry.joint2, trial.local.local.testBit(pin),
                  trial.dst.jump.testBit(entry.dst),
                  entry.joint >= 0 && trial.joint.jump.testBit(entry.joint),
                  entry.joint2 >= 0 && trial.joint.jump.testBit(entry.joint2),
                  nodeOwnerForDump(route_tile, fpga::CB_NODE_LOCAL, pin, false),
                  nodeOwnerForDump(route_tile, fpga::CB_NODE_DST, entry.dst,
                                   false),
                  entry.joint >= 0
                      ? nodeOwnerForDump(route_tile, fpga::CB_NODE_JOINT,
                                         entry.joint, false)
                      : std::string{},
                  entry.joint2 >= 0
                      ? nodeOwnerForDump(route_tile, fpga::CB_NODE_JOINT,
                                         entry.joint2, false)
                      : std::string{});
            }
            continue;
          }
          states[route_tile] = trial;
          reserved_locals[route_tile].setBit(pin);
          return true;
        }
        return false;
      });
      if (selected) {
        break;
      }
    }
    if (!selected) {
      restore();
      return fail("input port '" + input_name +
                  "' has no free local terminal path");
    }
  }
  restore();
  return required_selected || fail("required output did not select takeoff");
}

// Complete one retained Generic prefix by routing backward from its sink to
// the newest reachable numeric landing; no placement is changed here.
RouteDesign::MovingSourceAnchorResult
RouteDesign::tryCompleteMovingSourcePrefix(RouteTask &task, rtl::Inst &source,
                                           size_t max_expansions,
                                           fpga::NetRouteRef *missed_prefix,
                                           size_t *expanded) {
  if (missed_prefix) {
    *missed_prefix = {};
  }
  if (expanded) {
    *expanded = 0;
  }
  if (!task.net || !task.to || !task.to->tile.peer || !source.tile.peer) {
    return MovingSourceAnchorResult::NoPrefix;
  }
  std::vector<Wire> *existing_route = findBoundRoute(
      task.net, task.from, task.to, task.from_port, task.to_port, task.net_name);
  const bool had_incomplete_prefix = pnr::movingSourceUsesBackwardAnchor(
      existing_route && !existing_route->empty(),
      existing_route && routeIsComplete(*existing_route));
  if (!had_incomplete_prefix) {
    return MovingSourceAnchorResult::NoPrefix;
  }

  size_t existing_binding_index = std::numeric_limits<size_t>::max();
  for (size_t index = task.net->routes.size(); index > 0; --index) {
    if (routeBindingRoute(task.net->routes[index - 1]) == existing_route) {
      existing_binding_index = index - 1;
      break;
    }
  }
  if (existing_binding_index == std::numeric_limits<size_t>::max()) {
    return MovingSourceAnchorResult::NoPrefix;
  }
  if (missed_prefix) {
    *missed_prefix = {task.net, existing_binding_index};
  }

  // Reverse route order makes the latest retained landing the first anchor.
  std::vector<BackwardRouteAnchor> prefix_anchors;
  for (size_t index = existing_route->size(); index > 0; --index) {
    const Wire &fragment = (*existing_route)[index - 1];
    if (fragment.type != Wire::WIRE_CROSSBAR || fragment.jump < 0 ||
        fragment.dst < 0) {
      continue;
    }
    Tile *landing = fpga->getTile(fragment.to.x, fragment.to.y);
    if (!landing || !landing->cb_type) {
      continue;
    }
    bool duplicate = std::any_of(
        prefix_anchors.begin(), prefix_anchors.end(),
        [&](const BackwardRouteAnchor &old) {
          return old.tile == landing && old.dst == fragment.dst;
        });
    if (!duplicate) {
      prefix_anchors.push_back(
          {landing, fragment.dst, fragment.dst_wire_name, index});
    }
  }
  if (prefix_anchors.empty()) {
    return MovingSourceAnchorResult::Missed;
  }

  int anchor_distance =
      routeDistance(prefix_anchors.front().tile->coord, task.to->tile->coord);
  int source_distance = routeDistance(source.tile->coord, task.to->tile->coord);
  int anchor_radius =
      std::max(12, std::max(anchor_distance, source_distance) + 12);
  for (Tile *target_tile : routeTileCandidates(
           *task.to, task.to_port, false, &prefix_anchors.front().tile->coord)) {
    if (!target_tile) {
      continue;
    }
    NodeMask pin_nodes =
        routeTileInputNodes(*target_tile, *task.to, task.to_port);
    BackwardTakeoffRoute suffix = routeBackwardToAnchors(
        *target_tile, pin_nodes, prefix_anchors, 0, anchor_radius,
        &backwardDockingIndex(task.to->tile->coord, anchor_radius),
        [&]() { return routeStageSearchCancelled(); },
        max_expansions);
    if (expanded) {
      *expanded += suffix.expanded;
    }
    if (!suffix.success && suffix.failure_tile) {
      recordRouteTaskFailure(task, suffix.failure_tile->coord,
                            fpga::CB_NODE_DST, suffix.failure_dst);
      rememberFailureVisualizationPath(task, suffix.diagnostic_fragments);
    }
    if (!suffix.success || !suffix.completed_from_anchor) {
      continue;
    }
    for (Wire &fragment : suffix.fragments) {
      fragment.net_name = task.net_name;
    }
    if (!suffix.fragments.empty() &&
        suffix.fragments.back().type == Wire::WIRE_TILE_PIN) {
      fillTilePinEndpoint(suffix.fragments.back(), *task.to, task.to_port,
                          fpga::TILE_PIN_INPUT);
      refreshTilePinEndpoint(suffix.fragments.back(), *task.to, task.to_port,
                             fpga::TILE_PIN_INPUT);
    }
    if (!commitPreparedRoute(suffix.fragments)) {
      continue;
    }
    if (suffix.anchor_id == 0 || suffix.anchor_id > existing_route->size() ||
        (suffix.anchor_id < existing_route->size() &&
         !fpga::truncateNetRoute(*task.net, existing_binding_index,
                                 suffix.anchor_id))) {
      fpga::releaseRouteLeases(suffix.fragments);
      continue;
    }
    existing_route->insert(existing_route->end(), suffix.fragments.begin(),
                           suffix.fragments.end());
    fpga::registerNetRouteTiles(*task.net, *existing_route,
                                existing_binding_index);
    moving_source_probe_offsets.erase(movingClusterKey(&source));
    route_changed = true;
    route_progress = true;
    PNR_LOG1("ROUT",
             "routeDesign source partial trunk completed in place: net='{}', "
             "anchor=({},{})/{}, suffix_fragments={}, reverse_expanded={}, "
             "anchor_candidates={}",
             task.net_name, suffix.source_tile->coord.x,
             suffix.source_tile->coord.y, suffix.fragments.front().local,
             suffix.fragments.size(), suffix.expanded, suffix.anchor_candidates);
    return MovingSourceAnchorResult::Completed;
  }
  return MovingSourceAnchorResult::Missed;
}

// Route a trunk backward first, select a source placement at its first legal
// takeoff, then move and reconnect the source without scheduler-visible gaps.
bool RouteDesign::moveUnfinishedSource(RouteTask &task,
                                       std::vector<RouteTask> *moved_tasks,
                                       std::string *fail_reason,
                                       size_t *released_prefixes,
                                       bool *source_moved,
                                       bool *candidate_retryable,
                                       bool *boundary_released) {
  if (released_prefixes) {
    *released_prefixes = 0;
  }
  if (source_moved) {
    *source_moved = false;
  }
  if (candidate_retryable) {
    *candidate_retryable = false;
  }
  if (boundary_released) {
    *boundary_released = false;
  }
  if (task.to && task.to->tile.peer) {
    recordRouteTaskFailure(task, task.to->tile->coord, fpga::CB_NODE_LOCAL, -1);
  } else if (task.from && task.from->tile.peer) {
    recordRouteTaskFailure(task, task.from->tile->coord, fpga::CB_NODE_LOCAL, -1);
  }
  // Record the intended numeric destination entry before eligibility checks;
  // some source clusters fail before reverse expansion can produce a suffix.
  bool endpoint_recorded = false;
  if (task.to && task.to->tile.peer) {
    for (Tile *target_tile :
         routeTileCandidates(*task.to, task.to_port, false)) {
      if (!target_tile || !target_tile->cb_type) {
        continue;
      }
      NodeMask pin_nodes =
          routeTileInputNodes(*target_tile, *task.to, task.to_port);
      pin_nodes.for_each_set_bit([&](int pin) {
        int selected_dst = -1;
        TerminalEntryCandidate selected_entry;
        target_tile->cb_type->ensureDerivedMasks();
        target_tile->cb_type->dsts_reaching_local[pin].jump.for_each_set_bit(
            [&](int dst) {
              std::vector<TerminalEntryCandidate> entries =
                  targetEntryCandidates(*target_tile->cb_type, dst, pin);
              if (entries.empty()) {
                return false;
              }
              selected_dst = dst;
              selected_entry = entries.front();
              return true;
            });
        if (selected_dst < 0) {
          return false;
        }
        Wire enter;
        enter.from = target_tile->coord;
        enter.to = target_tile->coord;
        enter.local = selected_dst;
        enter.joint = selected_entry.joint;
        enter.joint2 = selected_entry.joint2;
        enter.pos = ROUTE_POS_TRANSIT;
        if (const std::string *name = target_tile->cb_type->nodeName(
                fpga::CB_NODE_DST, selected_dst)) {
          enter.dst_wire_name = *name;
        }
        Wire tile_pin;
        tile_pin.type = Wire::WIRE_TILE_PIN;
        tile_pin.from = target_tile->coord;
        tile_pin.to = target_tile->coord;
        tile_pin.local = pin;
        tile_pin.pos = ROUTE_POS_TRANSIT;
        recordRouteTaskFailure(task, target_tile->coord, fpga::CB_NODE_DST,
                              selected_dst);
        rememberFailureVisualizationPath(task, {enter, tile_pin});
        endpoint_recorded = true;
        return true;
      });
      if (endpoint_recorded) {
        break;
      }
    }
  }
  rtl::Inst *source = pnr::movingSourcePlacementTarget(
      task.from, [](rtl::Inst *endpoint) -> rtl::Inst * {
        rtl::Inst *owner = nullptr;
        std::string owner_port;
        return endpoint && passthroughInputSourceEndpoint(*endpoint, owner,
                                                          owner_port)
                   ? owner
                   : nullptr;
      });
  auto fail = [&](const std::string &reason) {
    if (fail_reason) {
      *fail_reason = reason;
    }
    return false;
  };
  if (!source || !task.from || !source->tile.peer || !task.from->tile.peer ||
      !task.to || !task.to->tile.peer || !task.net || isIoBuffer(*source) ||
      source->outline.fixed) {
    return fail("source is not eligible for route-first relocation");
  }
  std::vector<rtl::Inst *> cluster = strictMoveCluster(source);
  if (cluster.empty()) {
    cluster.push_back(source);
  }
  if (std::find(cluster.begin(), cluster.end(), task.from) == cluster.end()) {
    cluster.push_back(task.from);
  }
  for (rtl::Inst *member : cluster) {
    if (!member || !member->tile.peer || isIoBuffer(*member) ||
        member->outline.fixed) {
      return fail("source cluster contains a non-movable member");
    }
  }

  // Collect every ordinary input before probing placements. Candidate
  // validation reuses these endpoints without changing live routes or leases.
  std::vector<RouteTask> candidate_input_tasks;
  for (rtl::Inst *member : cluster) {
    for (rtl::Conn &sink_conn : member->conns) {
      if (!sink_conn.port_ref.peer ||
          sink_conn.port_ref->type != rtl::Port::PORT_IN ||
          tech->check_clocked(member->cell_ref->type,
                              sink_conn.port_ref->name)) {
        continue;
      }
      rtl::Conn *driver_conn = sink_conn.follow();
      if (!driver_conn || !driver_conn->inst_ref.peer ||
          !driver_conn->port_ref.peer || driver_conn->port_ref->is_global ||
          !driver_conn->inst_ref->cell_ref.peer ||
          !driver_conn->inst_ref->cell_ref->module_ref.peer ||
          !driver_conn->inst_ref->cell_ref->module_ref->is_blackbox) {
        continue;
      }
      rtl::Net *input_net =
          findNetByDesignator(*member, sink_conn.port_ref->designator);
      if (!input_net ||
          input_net->designatorIsVoid(sink_conn.port_ref->designator)) {
        continue;
      }
      std::string driver_port = driver_conn->port_ref->makeName();
      appendUniqueRouteTask(
          candidate_input_tasks,
          RouteTask{driver_conn->inst_ref.peer,
                    member,
                    input_net,
                    driver_port,
                    sink_conn.port_ref->makeName(),
                    sink_conn.makeNetName(nullptr, FULL_NAME_LIMIT),
                    0,
                    0,
                    0,
                    {},
                    !input_net->distributed_source &&
                        sourceTreeHasCompleteExit(*driver_conn->inst_ref.peer,
                                                  driver_port)});
    }
  }

  Tile *old_tile = &*source->tile;
  Coord old_coord = old_tile->coord;
  int old_pos = source->pos;
  uintptr_t source_cluster_key = movingClusterKey(source);
  const bool debug_source = routeTaskDebugMatches(task);
  const auto recovery_started = std::chrono::steady_clock::now();
  if (debug_source) {
    logRouteTaskDecision("source.reverse.begin", task);
  }
  fpga::NetRouteRef missed_prefix;
  size_t anchor_expanded = 0;
  MovingSourceAnchorResult anchor_result = tryCompleteMovingSourcePrefix(
      task, *source, 0, &missed_prefix, &anchor_expanded);
  if (debug_source) {
    PNR_LOG1("ROUT", "source anchor proof: net='{}' result={} expanded={} seconds={:.3f}",
             task.net_name, static_cast<int>(anchor_result), anchor_expanded,
             elapsedSeconds(recovery_started, std::chrono::steady_clock::now()));
    logTargetTileEntryTable(task, "source reverse endpoint");
  }
  if (anchor_result == MovingSourceAnchorResult::Completed) {
    return true;
  }
  // Cancellation is not proof that all anchors failed. Keep the owned prefix
  // and return to the stage exit before releasing any route resources.
  if (routeStageSearchCancelled()) {
    if (candidate_retryable) {
      *candidate_retryable = true;
    }
    return fail("source recovery cancelled before replacement search");
  }
  std::vector<Wire> *existing_route = findBoundRoute(
      task.net, task.from, task.to, task.from_port, task.to_port, task.net_name);
  // A retained prefix was useful only as a backward-docking anchor. Once all
  // anchors fail, release this binding so it cannot congest replacement
  // search or unrelated sources; no other task is disturbed.
  if (pnr::movingSourceReleasesPrefixAfterDockMiss(
          anchor_result == MovingSourceAnchorResult::Missed, false) &&
      existing_route && missed_prefix.net) {
    PNR_ASSERT(fpga::unrouteNetRoute(*missed_prefix.net,
                                     missed_prefix.binding_index),
               "failed to release unreachable Moving Sources prefix for '{}'",
               task.net_name);
    existing_route = nullptr;
    route_changed = true;
    if (released_prefixes) {
      ++*released_prefixes;
    }
  }
  BackwardTakeoffStateView route_first_states;
  auto state_for = [&](Tile *tile) -> CBState & {
    return route_first_states.try_emplace(tile, tile->cb).first->second;
  };
  auto owned_only_by_existing = [&](Tile &tile, fpga::CBNodeNameType type,
                                    int node) {
    if (!existing_route || node < 0) {
      return false;
    }
    std::vector<fpga::NetRouteRef> owners =
        fpga::findNetOwnersByNode(tile, type, node, false);
    return owners.empty() ||
           std::all_of(owners.begin(), owners.end(), [&](const auto &owner) {
             return owner.net && owner.binding_index < owner.net->routes.size() &&
                    routeBindingRoute(
                        owner.net->routes[owner.binding_index]) == existing_route;
           });
  };
  auto clear_owned_node = [&](Tile *tile, fpga::CBNodeNameType type, int node) {
    if (!tile || !owned_only_by_existing(*tile, type, node)) {
      return;
    }
    NodeMask keep = ~(NodeMask{0, 1} << node);
    CBState &view = state_for(tile);
    switch (type) {
    case fpga::CB_NODE_LOCAL:
      view.local.local &= keep;
      break;
    case fpga::CB_NODE_JOINT:
      view.joint.jump &= keep;
      break;
    case fpga::CB_NODE_SRC:
      view.src.jump &= keep;
      break;
    case fpga::CB_NODE_DST:
      view.dst.jump &= keep;
      break;
    case fpga::CB_NODE_JUMP:
      // Generic name-only jump identities do not identify a lease namespace.
      break;
    }
  };
  if (existing_route) {
    for (const Wire &fragment : *existing_route) {
      if (fragment.type == Wire::WIRE_CROSSBAR) {
        Tile *from_tile = fpga->getTile(fragment.from.x, fragment.from.y);
        Tile *landing_tile = fpga->getTile(fragment.to.x, fragment.to.y);
        if (!fragment.shared) {
          clear_owned_node(from_tile, fpga::CB_NODE_SRC, fragment.jump);
          clear_owned_node(from_tile, fpga::CB_NODE_JOINT, fragment.joint);
          clear_owned_node(from_tile, fpga::CB_NODE_JOINT, fragment.joint2);
          clear_owned_node(from_tile,
                           fragment.pos == 0 ? fpga::CB_NODE_LOCAL
                                             : fpga::CB_NODE_DST,
                           fragment.local);
        }
        if (fragment.owns_landing) {
          clear_owned_node(landing_tile, fpga::CB_NODE_DST, fragment.dst);
        }
      } else if (fragment.type == Wire::WIRE_ROUTE_EDGE && !fragment.shared) {
        Tile *to_tile = fpga->getTile(fragment.to.x, fragment.to.y);
        clear_owned_node(to_tile,
                         static_cast<fpga::CBNodeNameType>(fragment.to_node_type),
                         fragment.to_node);
      }
    }
  }
  Tile *selected_resource = nullptr;
  int selected_pos = -1;
  std::vector<fpga::ElementPackingChoice> selected_placements;
  BackwardTakeoffChoice selected_takeoff;
  std::string last_probe_failure;
  int search_distance = routeDistance(old_coord, task.to->tile->coord);
  int reverse_radius = std::max(12, search_distance + 12);
  BackwardTakeoffRoute backward_route;
  size_t reverse_expanded = 0;
  size_t reverse_incoming_edges = 0;
  size_t reverse_near_source_edges = 0;
  size_t reverse_probe_calls = 0;
  size_t reverse_tile_visit_rejects = 0;
  size_t candidate_tiles_probed = 0;
  size_t candidate_positions_found = 0;
  bool reverse_probe_window_remaining = false;
  struct CandidatePosition {
    Tile *tile = nullptr;
    int source_pos = -1;
    int endpoint_pos = -1;
    std::vector<fpga::ElementPackingChoice> placements;
  };
  struct CandidateInputProbeKey {
    Tile *tile = nullptr;
    int source_pos = -1;
    int endpoint_pos = -1;
    uint64_t placement_signature = 0;
    Tile *output_tile = nullptr;
    int output_src = -1;

    bool operator==(const CandidateInputProbeKey &) const = default;
  };
  struct CandidateInputProbeKeyHash {
    size_t operator()(const CandidateInputProbeKey &key) const {
      size_t hash = std::hash<Tile *>{}(key.tile);
      hash ^= std::hash<int>{}(key.source_pos) + 0x9e3779b9 + (hash << 6) +
              (hash >> 2);
      hash ^= std::hash<int>{}(key.endpoint_pos) + 0x9e3779b9 + (hash << 6) +
              (hash >> 2);
      hash ^= std::hash<uint64_t>{}(key.placement_signature) + 0x9e3779b9 +
              (hash << 6) + (hash >> 2);
      hash ^= std::hash<Tile *>{}(key.output_tile) + 0x9e3779b9 +
              (hash << 6) + (hash >> 2);
      hash ^= std::hash<int>{}(key.output_src) + 0x9e3779b9 +
              (hash << 6) + (hash >> 2);
      return hash;
    }
  };
  struct CandidateInputProbeResult {
    bool routable = false;
    std::string reason;
  };
  struct CandidateInputAnchorPrefix {
    const std::vector<Wire> *route = nullptr;
    size_t size = 0;
  };
  struct CandidateInputAnchorSet {
    std::vector<BackwardRouteAnchor> anchors;
    std::unordered_map<size_t, CandidateInputAnchorPrefix> shared_prefixes;
  };
  struct PreparedCandidateInput {
    RouteTask task;
    BackwardTakeoffRoute proof;
    std::vector<Wire> shared_prefix;
    bool extends_own_prefix = false;
  };
  std::unordered_map<Tile *, std::vector<CandidatePosition>>
      candidates_by_route_tile;
  std::unordered_map<CandidateInputProbeKey, CandidateInputProbeResult,
                     CandidateInputProbeKeyHash>
      candidate_input_probe_cache;
  std::unordered_map<const RouteTask *, CandidateInputAnchorSet>
      candidate_input_anchor_cache;
  std::vector<PreparedCandidateInput> selected_input_proofs;

  // Apply one proven route to a private state view. Candidate validation uses
  // this to prove coexistence without reserving any live crossbar resource.
  auto apply_route_to_view = [&](BackwardTakeoffStateView &states,
                                 const std::vector<Wire> &route) {
    auto state_for_route_tile = [&](Tile *tile) -> CBState & {
      auto found = states.find(tile);
      if (found != states.end()) {
        return found->second;
      }
      return states.emplace(tile, tile->cb).first->second;
    };
    for (size_t index = 0; index < route.size(); ++index) {
      const Wire &fragment = route[index];
      if (fragment.type == Wire::WIRE_TILE_PIN) {
        continue;
      }
      Tile *tile = fpga->getTile(fragment.from.x, fragment.from.y);
      if (!tile) {
        return false;
      }
      CBState &state = state_for_route_tile(tile);
      if (fragment.jump >= 0) {
        bool leased =
            fragment.pos == ROUTE_POS_SOURCE
                ? leaseConcreteOut(state, fragment.local, fragment.jump,
                                   fragment.joint, true, fragment.joint2)
                : fragment.pos == ROUTE_POS_FORK
                      ? leaseConcreteFork(state, fragment.local, fragment.jump,
                                          fragment.joint, true,
                                          fragment.joint2)
                      : leaseConcreteJump(state, fragment.local, fragment.jump,
                                          fragment.joint, true,
                                          fragment.joint2);
        if (!leased) {
          return false;
        }
        continue;
      }
      if (index + 1 >= route.size() ||
          route[index + 1].type != Wire::WIRE_TILE_PIN ||
          !leaseConcreteIn(state, fragment.local, route[index + 1].local,
                           fragment.joint, fragment.joint2)) {
        return false;
      }
    }
    return true;
  };

  // Prove all moved-cluster inputs against a hypothetical placement. Each
  // reverse route is discarded, so sequential candidate probing leases no
  // live node and needs no persistent reservation.
  auto candidate_inputs_routable =
      [&](const CandidatePosition &candidate,
          const std::vector<Wire> &output_route,
          std::string *reason,
          std::vector<PreparedCandidateInput> *prepared) -> bool {
    bool placement_changes = std::any_of(
        candidate.placements.begin(), candidate.placements.end(),
        [&](const fpga::ElementPackingChoice &placement) {
          return !placement.inst || !placement.inst->tile.peer ||
                 &*placement.inst->tile != candidate.tile ||
                 placement.inst->pos != placement.pos;
        });
    if (!placement_changes) {
      if (prepared) {
        prepared->clear();
      }
      if (reason) {
        reason->clear();
      }
      return true;
    }
    uint64_t placement_signature = 1469598103934665603ULL;
    for (const fpga::ElementPackingChoice &placement : candidate.placements) {
      placement_signature ^=
          static_cast<uint64_t>(reinterpret_cast<uintptr_t>(placement.inst));
      placement_signature *= 1099511628211ULL;
      placement_signature ^= static_cast<uint64_t>(placement.pos + 1);
      placement_signature *= 1099511628211ULL;
    }
    // Reverse routing records one suffix per (Tile,SRC) in this search. An
    // input failure is reusable only for that output path, not the placement
    // alone: another takeoff can leave the required input resources free.
    CandidateInputProbeKey cache_key{candidate.tile, candidate.source_pos,
                                     candidate.endpoint_pos,
                                     placement_signature,
                                     fpga->getTile(output_route.front().from.x,
                                                   output_route.front().from.y),
                                     output_route.front().jump};
    if (auto cached = candidate_input_probe_cache.find(cache_key);
        cached != candidate_input_probe_cache.end()) {
      if (reason) {
        *reason = cached->second.reason;
      }
      return cached->second.routable;
    }

    // Input routes must avoid the entire proposed output trunk, not just live
    // leases. Build this once, before any expensive input search. Failed paths
    // reject this takeoff, not every possible route from the placement.
    BackwardTakeoffStateView proof_states = route_first_states;
    if (!apply_route_to_view(proof_states, output_route)) {
      if (reason) {
        *reason = "output trunk has conflicting numeric resources";
      }
      return false;
    }

    struct SavedPlacement {
      rtl::Inst *inst = nullptr;
      Referable<Tile> *tile = nullptr;
      Coord coord;
      int pos = -1;
    };
    std::vector<SavedPlacement> saved;
    saved.reserve(candidate.placements.size());
    for (const fpga::ElementPackingChoice &placement : candidate.placements) {
      if (!placement.inst || !candidate.tile || placement.pos < 0) {
        if (reason) {
          *reason = "candidate cluster placement is incomplete";
        }
        return false;
      }
      saved.push_back({placement.inst, placement.inst->tile.peer,
                       placement.inst->coord, placement.inst->pos});
      placement.inst->tile.clear();
      placement.inst->tile.set(
          static_cast<Referable<Tile> *>(candidate.tile));
      placement.inst->coord = candidate.tile->coord;
      placement.inst->pos = placement.pos;
    }
    auto restore = [&]() {
      for (const SavedPlacement &old : saved) {
        old.inst->tile.clear();
        if (old.tile) {
          old.inst->tile.set(old.tile);
        }
        old.inst->coord = old.coord;
        old.inst->pos = old.pos;
      }
    };
    auto reject = [&](const RouteTask &input,
                      const std::string &detail, bool cache_failure = true) {
      restore();
      std::string failure =
          "input '" + input.to_port + "' from '" +
          (input.from ? input.from->makeName(FULL_NAME_LIMIT)
                      : std::string{"<none>"}) +
          "/" + input.from_port + "' " + detail;
      if (cache_failure) {
        candidate_input_probe_cache.emplace(
            cache_key, CandidateInputProbeResult{false, failure});
      }
      if (reason) {
        *reason = failure;
      }
      return false;
    };

    // Extend the output's private reservation with each successfully proven
    // input, keeping all incident routes mutually exclusive.
    std::vector<PreparedCandidateInput> candidate_proofs;
    candidate_proofs.reserve(candidate_input_tasks.size());

    for (const RouteTask &input : candidate_input_tasks) {
      if (routeStageSearchCancelled()) {
        return reject(input, "was cancelled by the stage budget/watchdog", false);
      }
      if (!input.from || !input.from->tile.peer || input.from->pos < 0 ||
          !input.to || !input.to->tile.peer) {
        return reject(input, "has an invalid endpoint");
      }

      // Extend this input's retained prefix, complete or partial, or branch
      // from a complete sibling without demanding a second source takeoff.
      const std::string source_key = sourceRouteKey(input.from, input.from_port);
      const std::vector<Wire> *own_input_route = findBoundRoute(
          input.net, input.from, input.to, input.from_port, input.to_port,
          input.net_name);
      auto [anchor_entry, anchors_inserted] =
          candidate_input_anchor_cache.try_emplace(&input);
      CandidateInputAnchorSet &anchor_set = anchor_entry->second;
      std::vector<BackwardRouteAnchor> &anchors = anchor_set.anchors;
      if (anchors_inserted) {
        std::unordered_set<rtl::Net *> anchor_nets;
        if (input.net) {
          anchor_nets.insert(input.net);
        }
        auto indexed = source_route_nets.find(source_key);
        if (indexed != source_route_nets.end()) {
          anchor_nets.insert(indexed->second.begin(), indexed->second.end());
        }
        for (rtl::Net *anchor_net : anchor_nets) {
          if (!anchor_net) {
            continue;
          }
          for (rtl::NetRouteBinding &binding : anchor_net->routes) {
            if (!samePhysicalSource(binding.from, binding.from_port, input.from,
                                    input.from_port)) {
              continue;
            }
            const std::vector<Wire> *route = routeBindingRoute(binding);
            if (debug_source) {
              PNR_LOG1("ROUT", "source input anchor binding: source='{}' input='{}' "
                       "driver='{}'/{} own={} fragments={} complete={} retained={}",
                       task.from->makeName(FULL_NAME_LIMIT), input.to_port,
                       input.from->makeName(FULL_NAME_LIMIT), input.from_port,
                       route == own_input_route, route ? route->size() : 0,
                       route && routeIsComplete(*route),
                       route ? fpga::retainedRoutePrefixSize(*route, false, true) : 0);
              if (route && route == own_input_route) {
                dumpRouteFragmentDiagnostics(input, 0, *route);
              }
            }
            if (!route || route->empty()) {
              continue;
            }
            bool sink_moves = std::find(cluster.begin(), cluster.end(), binding.to) !=
                              cluster.end();
            bool source_moves = std::find(cluster.begin(), cluster.end(), binding.from) !=
                                cluster.end();
            // Another moved input will replace its private tail too. Use this
            // input's own retained path or a sibling unaffected by the move.
            if (!pnr::movingInputMayReuseRoute(route == own_input_route,
                                              routeIsComplete(*route),
                                              source_moves, sink_moves)) {
              continue;
            }
            // Relocation keeps a moved sink's fabric prefix, but never the
            // old terminal or any route whose source itself is being moved.
            size_t prefix_size = fpga::retainedRoutePrefixSize(
                *route,
                source_moves, sink_moves);
            for (size_t fragment_index = 0; fragment_index < prefix_size;
                 ++fragment_index) {
              const Wire &fragment = (*route)[fragment_index];
              if (fragment.type != Wire::WIRE_CROSSBAR || fragment.dst < 0) {
                continue;
              }
              Tile *landing = fpga->getTile(fragment.to.x, fragment.to.y);
              if (!landing || !landing->cb_type) {
                continue;
              }
              bool duplicate = std::any_of(
                  anchors.begin(), anchors.end(), [&](const auto &old) {
                    return old.tile == landing && old.dst == fragment.dst;
                  });
              if (!duplicate) {
                size_t anchor_id = anchors.size() + 1;
                anchors.push_back({landing, fragment.dst,
                                   fragment.dst_wire_name, anchor_id});
                anchor_set.shared_prefixes.emplace(
                    anchor_id,
                    CandidateInputAnchorPrefix{route, fragment_index + 1});
              }
            }
          }
        }
      }

      bool input_complete = false;
      for (Tile *target_tile :
           routeTileCandidates(*input.to, input.to_port, false,
                               &input.from->tile->coord)) {
        if (!target_tile || !target_tile->cb_type) {
          continue;
        }
        NodeMask pin_nodes =
            routeTileInputNodes(*target_tile, *input.to, input.to_port);
        if (pin_nodes == NodeMask{}) {
          continue;
        }
        int radius = std::max(
            12, routeDistance(target_tile->coord, input.from->tile->coord) +
                    12);
        BackwardTakeoffProbe source_probe =
            [&](Tile &route_tile, int src,
                BackwardTakeoffChoice &choice) -> bool {
          Tile *driver_resource = &*input.from->tile;
          bool attached = &route_tile == driver_resource ||
              std::find(route_tile.attached_resource_tiles.begin(),
                        route_tile.attached_resource_tiles.end(),
                        driver_resource) !=
                  route_tile.attached_resource_tiles.end();
          if (!attached) {
            choice.counts_toward_probe_limit = false;
            return false;
          }
          std::string ignored;
          return routeOutTry(*input.from, *driver_resource, input.from->pos,
                             input.from_port, route_tile, src, choice,
                             &ignored, &proof_states);
        };
        BackwardTakeoffRoute proof = routeBackwardToInput(
            *target_tile, pin_nodes, input.from->tile->coord, radius,
            source_probe,
            &backwardDockingIndex(target_tile->coord, radius),
            [&]() { return routeStageSearchCancelled(); },
            &proof_states, anchors.empty() ? nullptr : &anchors);
        // Focused diagnostics distinguish exhausted topology from a bounded
        // input proof that still has reachable reverse frontiers to examine.
        if (routeDebugMatches("SCALEPNR_DEBUG_TASK_NET",
                              task.from->makeName(FULL_NAME_LIMIT))) {
          PNR_LOG1("ROUT",
                   "source input proof: source='{}' resource=({},{})/{} "
                   "input='{}' driver='{}'/{} target=({},{}) pins={} "
                   "anchors={} success={} expanded={} limited={} pending={} "
                   "probes={} frontier=({},{})/{}",
                   task.from->makeName(FULL_NAME_LIMIT), candidate.tile->coord.x,
                   candidate.tile->coord.y, candidate.source_pos, input.to_port,
                   input.from->makeName(FULL_NAME_LIMIT), input.from_port,
                   target_tile->coord.x, target_tile->coord.y,
                   maskBitsForDump(pin_nodes), anchors.size(), proof.success,
                   proof.expanded, proof.expansion_limit_reached,
                   proof.remaining_frontier, proof.probe_calls,
                   proof.failure_tile ? proof.failure_tile->coord.x : -1,
                   proof.failure_tile ? proof.failure_tile->coord.y : -1,
                   proof.failure_dst);
        }
        // A stage cancellation is incomplete work, not evidence of an
        // impossible placement. Never put it in the negative-result cache.
        if (routeStageSearchCancelled()) {
          return reject(input, "was cancelled by the stage budget/watchdog", false);
        }
        const CandidateInputAnchorPrefix *shared_prefix = nullptr;
        if (proof.success && proof.completed_from_anchor) {
          auto prefix = anchor_set.shared_prefixes.find(proof.anchor_id);
          if (prefix == anchor_set.shared_prefixes.end()) {
            continue;
          }
          shared_prefix = &prefix->second;
        }
        if (proof.success &&
            apply_route_to_view(proof_states, proof.fragments)) {
          PreparedCandidateInput prepared_input{input, std::move(proof), {}};
          if (shared_prefix) {
            if (!shared_prefix->route ||
                shared_prefix->size > shared_prefix->route->size()) {
              continue;
            }
            prepared_input.shared_prefix.assign(
                shared_prefix->route->begin(),
                shared_prefix->route->begin() +
                    static_cast<std::ptrdiff_t>(shared_prefix->size));
            prepared_input.extends_own_prefix = shared_prefix->route == own_input_route;
          }
          candidate_proofs.push_back(std::move(prepared_input));
          input_complete = true;
          break;
        }
      }
      if (!input_complete) {
        return reject(input, "has no complete numeric route");
      }
    }
    restore();
    if (prepared) {
      *prepared = std::move(candidate_proofs);
    }
    if (reason) {
      reason->clear();
    }
    return true;
  };

  Coord source_hint = old_coord;
  auto preempt_blocked_reverse_seed = [&](Tile &target_tile,
                                           NodeMask pin_nodes) {
    if (!route_preemption_enabled || !target_tile.cb_type) {
      return false;
    }
    return pin_nodes.for_each_set_bit([&](int pin) {
      return preemptGroundingTerminal(
          target_tile, pin, incomingDstMaskForRouteTile(target_tile),
          fpga::packedInputJointReservations(target_tile, task.to, task.to_port),
          task).dst >= 0;
    });
  };
  // Probe each reached reverse layer before expanding another one. Candidates
  // within that layer prefer the old source, without excluding distant tiles.
  const int source_search_radii[] = {pnr::movingSourcePlacementRadius()};
  for (int source_radius : source_search_radii) {
    for (Tile *target_tile :
         routeTileCandidates(*task.to, task.to_port, false, &source_hint)) {
      if (!target_tile) {
        continue;
      }
      NodeMask pin_nodes = routeTileInputNodes(*target_tile, *task.to,
                                               task.to_port);
      // Backward expansion needs one free terminal seed. An unfinished route
      // ending at this tile is transit ownership until it actually reaches a
      // local pin, so free that exact blocker and requeue its binding.
      preempt_blocked_reverse_seed(*target_tile, pin_nodes);
      selected_resource = nullptr;
      selected_pos = -1;
      // The existing packed position is globally preferable to relocation.
      // Probe it as soon as its attached route tile reaches the reverse walk.
      auto current_placement_probe =
          [&](Tile &route_tile, int src, BackwardTakeoffChoice &choice) {
            choice.counts_toward_probe_limit = false;
            bool owns_old_resource = &route_tile == old_tile ||
                std::find(route_tile.attached_resource_tiles.begin(),
                          route_tile.attached_resource_tiles.end(), old_tile) !=
                    route_tile.attached_resource_tiles.end();
            if (!owns_old_resource || !task.from->tile.peer ||
                &*task.from->tile != old_tile || task.from->pos < 0 ||
                source->pos < 0) {
              return false;
            }
            std::vector<fpga::ElementPackingChoice> placements;
            placements.reserve(cluster.size());
            for (rtl::Inst *member : cluster) {
              if (!member || !member->tile.peer ||
                  &*member->tile != old_tile || member->pos < 0) {
                return false;
              }
              placements.push_back({member, member->pos});
            }
            BackwardTakeoffChoice current_takeoff;
            if (!routeOutTry(*task.from, *old_tile, task.from->pos,
                             task.from_port, route_tile, src, current_takeoff,
                             &last_probe_failure, &route_first_states)) {
              choice = current_takeoff;
              return false;
            }
            selected_resource = old_tile;
            selected_pos = source->pos;
            selected_placements = std::move(placements);
            selected_takeoff = current_takeoff;
            choice = current_takeoff;
            return true;
          };
      BackwardTakeoffPathProbe placement_probe =
          [&](Tile &route_tile, int src, BackwardTakeoffChoice &choice,
              const BackwardTakeoffPath &output_path) {
            // Numeric reverse frontiers without a packable source location are
            // cheap skips, not failed placement/takeoff probes.
            choice.counts_toward_probe_limit = false;
            if (routeStageSearchCancelled()) {
              return false;
            }
            auto [known, inserted] =
                candidates_by_route_tile.try_emplace(&route_tile);
            if (inserted) {
              std::vector<Tile *> resource_candidates;
              auto append_resource = [&](Tile *candidate) {
                if (candidate &&
                    std::find(resource_candidates.begin(),
                              resource_candidates.end(), candidate) ==
                        resource_candidates.end()) {
                  resource_candidates.push_back(candidate);
                }
              };
              append_resource(&route_tile);
              for (Tile *attached : route_tile.attached_resource_tiles) {
                append_resource(attached);
              }
              for (Tile *candidate : resource_candidates) {
                    if (routeStageSearchCancelled()) {
                      break;
                    }
                    ++candidate_tiles_probed;
                    if (!candidate || !candidate->tile_type ||
                        candidate->tile_type->elements.empty()) {
                      continue;
                    }
                    if (candidate == old_tile) {
                      std::vector<fpga::ElementPackingChoice> placements;
                      bool complete = true;
                      for (rtl::Inst *member : cluster) {
                        if (!member || !member->tile.peer ||
                            &*member->tile != candidate || member->pos < 0) {
                          complete = false;
                          break;
                        }
                        placements.push_back({member, member->pos});
                      }
                      if (complete && task.from->tile.peer &&
                          &*task.from->tile == candidate && task.from->pos >= 0 &&
                          source->pos >= 0) {
                        ++candidate_positions_found;
                        known->second.push_back(
                            {candidate, source->pos, task.from->pos,
                             std::move(placements)});
                      }
                      continue;
                    }
                    struct SavedPlacement {
                      rtl::Inst *inst = nullptr;
                      Referable<Tile> *tile = nullptr;
                      Coord coord;
                      int pos = -1;
                    };
                    std::vector<SavedPlacement> saved;
                    saved.reserve(cluster.size());
                    for (rtl::Inst *member : cluster) {
                      saved.push_back({member, member->tile.peer, member->coord,
                                       member->pos});
                      member->tile.clear();
                      member->pos = -1;
                    }
                    fpga::ElementPackingPreview preview(*candidate);
                    std::vector<rtl::Inst *> remaining;
                    remaining.reserve(cluster.size());
                    for (rtl::Inst *member : cluster) {
                      if (member != task.from) {
                        remaining.push_back(member);
                      }
                    }
                    // Reuse one dependency order for every endpoint-position trial.
                    PNR_ASSERT(fpga::ElementPackingPreview::orderPack(remaining),
                               "invalid local dependency order in source cluster");
                    for (int endpoint_pos :
                         candidate->candidatePositions(task.from)) {
                      size_t checkpoint = preview.checkpoint();
                      if (preview.reserveAt(task.from, endpoint_pos, false) < 0) {
                        preview.rollback(checkpoint);
                        continue;
                      }
                      std::vector<fpga::ElementPackingChoice> placements{
                          {task.from, endpoint_pos}};
                      if (!preview.reservePack(remaining, placements, false)) {
                        preview.rollback(checkpoint);
                        continue;
                      }
                      auto source_choice = std::find_if(
                          placements.begin(), placements.end(),
                          [&](const fpga::ElementPackingChoice &choice) {
                            return choice.inst == source;
                          });
                      bool complete_cluster =
                          pnr::routeFirstClusterPlacementComplete(
                              cluster, placements,
                              [](const fpga::ElementPackingChoice &choice) {
                                return choice.inst;
                              });
                      if (complete_cluster &&
                          source_choice != placements.end() &&
                          !placementWasTried(
                              move_tried_placements[source_cluster_key],
                              candidate->coord, source_choice->pos)) {
                        ++candidate_positions_found;
                        known->second.push_back(
                            {candidate, source_choice->pos, endpoint_pos,
                             placements});
                      }
                      preview.rollback(checkpoint);
                    }
                    for (const SavedPlacement &old : saved) {
                      old.inst->tile.clear();
                      if (old.tile) {
                        old.inst->tile.set(old.tile);
                      }
                      old.inst->coord = old.coord;
                      old.inst->pos = old.pos;
                    }
              }
            }
            for (const CandidatePosition &candidate_pos : known->second) {
              if (!candidate_pos.tile || candidate_pos.source_pos < 0 ||
                  candidate_pos.endpoint_pos < 0) {
                continue;
              }
              BackwardTakeoffChoice candidate_takeoff;
              if (!routeOutTry(*task.from, *candidate_pos.tile,
                               candidate_pos.endpoint_pos, task.from_port,
                               route_tile, src, candidate_takeoff,
                               &last_probe_failure, &route_first_states)) {
                if (routeDebugMatches("SCALEPNR_DEBUG_TASK_NET",
                                      task.from->makeName(FULL_NAME_LIMIT))) {
                  PNR_LOG1(
                      "ROUT",
                      "routeDesign source takeoff candidate reject: "
                      "source='{}', resource=({},{}) source_pos={} "
                      "endpoint_pos={} route_tile=({},{}) src={} reason='{}' "
                      "counts_toward_limit={}",
                      task.from->makeName(FULL_NAME_LIMIT),
                      candidate_pos.tile->coord.x,
                      candidate_pos.tile->coord.y, candidate_pos.source_pos,
                      candidate_pos.endpoint_pos, route_tile.coord.x,
                      route_tile.coord.y, src, last_probe_failure,
                      candidate_takeoff.counts_toward_probe_limit);
                }
                choice = candidate_takeoff;
                continue;
              }
              std::string input_probe_failure;
              std::vector<PreparedCandidateInput> input_proofs;
              if (!candidate_inputs_routable(candidate_pos,
                                             output_path(candidate_takeoff),
                                             &input_probe_failure,
                                             &input_proofs)) {
                last_probe_failure = input_probe_failure;
                if (routeDebugMatches("SCALEPNR_DEBUG_TASK_NET",
                                      task.from->makeName(FULL_NAME_LIMIT))) {
                  PNR_LOG1(
                      "ROUT",
                      "routeDesign source candidate input reject: source='{}' "
                      "resource=({},{}) source_pos={} endpoint_pos={} "
                      "reason='{}'",
                      task.from->makeName(FULL_NAME_LIMIT),
                      candidate_pos.tile->coord.x,
                      candidate_pos.tile->coord.y, candidate_pos.source_pos,
                      candidate_pos.endpoint_pos, input_probe_failure);
                }
                choice = candidate_takeoff;
                continue;
              }
              selected_resource = candidate_pos.tile;
              selected_pos = candidate_pos.source_pos;
              selected_placements = candidate_pos.placements;
              selected_takeoff = candidate_takeoff;
              selected_input_proofs = std::move(input_proofs);
              choice = candidate_takeoff;
              return true;
            }
            return false;
          };
      // A different target entry builds a different reverse tree; its suffix
      // at the same (Tile,SRC) need not match the preceding search's suffix.
      candidate_input_probe_cache.clear();
      const auto reverse_started = std::chrono::steady_clock::now();
      backward_route = routeBackwardToTakeoff(
          *target_tile, pin_nodes, old_coord, 0, reverse_radius,
          source_radius, {},
          &backwardDockingIndex(task.to->tile->coord, reverse_radius),
          [&]() { return routeStageSearchCancelled(); },
          0,
          moving_source_probe_offsets[source_cluster_key],
          pnr::movingSourceProbeLimit(),
          &route_first_states, nullptr, current_placement_probe,
          placement_probe,
          // All free continuations and placement probes failed. Release at
          // most one eligible transit suffix, then discard this state view.
          [&](const std::vector<pnr::DockingBridgeBlocker> &blockers) {
            for (const auto &blocker : blockers) {
              if (routeStageSearchCancelled()) {
                return false;
              }
              if (preemptDockingBridge(
                      this, blocker, task.net, task.from, task.from_port,
                      task.net_name, &route_stats,
                      pnr::movingSourceBoundaryMayExchangeComplete(
                          moving_sources_stage))) {
                return true;
              }
            }
            return false;
          });
      // The search stopped immediately after an ownership change. Let the
      // ordinary scheduler rebuild both this task and the exact victim.
      if (backward_route.boundary_released) {
        if (backward_route.failure_tile) {
          recordRouteTaskFailure(task, backward_route.failure_tile->coord,
                                fpga::CB_NODE_DST, backward_route.failure_dst);
        }
        if (!backward_route.diagnostic_fragments.empty()) {
          rememberFailureVisualizationPath(
              task, std::move(backward_route.diagnostic_fragments));
        }
        route_changed = true;
        if (candidate_retryable) {
          *candidate_retryable = true;
        }
        if (boundary_released) {
          *boundary_released = true;
        }
        PNR_LOG2("ROUT", "source reverse boundary released: net='{}', "
                 "expanded={}, probes={}, remaining_frontier={}",
                 task.net_name, backward_route.expanded,
                 backward_route.probe_calls, backward_route.remaining_frontier);
        return fail("backward trunk released one occupied transit boundary; "
                    "retry required");
      }
      // Keep aggregate search evidence even if cancellation prevents the
      // normal relocation result from being logged below.
      if (debug_source) {
        PNR_LOG1("ROUT",
                 "source reverse proof: net='{}' source='{}' target=({},{}) "
                 "expanded={} edges={} candidates={} scanned={} probes={} "
                 "placement_tiles={} placement_positions={} success={} "
                 "seconds={:.3f} cancelled={} last_reject='{}'",
                 task.net_name, task.from->makeName(FULL_NAME_LIMIT), target_tile->coord.x,
                 target_tile->coord.y, backward_route.expanded,
                 backward_route.incoming_edges, backward_route.takeoff_candidates,
                 backward_route.probe_candidates_scanned, backward_route.probe_calls,
                 candidate_tiles_probed, candidate_positions_found,
                 backward_route.success,
                 elapsedSeconds(reverse_started, std::chrono::steady_clock::now()),
                 routeStageSearchCancelled(), last_probe_failure);
        dumpRouteFragmentDiagnostics(task, 0,
            backward_route.success ? backward_route.fragments
                                   : backward_route.diagnostic_fragments);
      }
      if (!backward_route.success && backward_route.failure_tile) {
        recordRouteTaskFailure(task, backward_route.failure_tile->coord,
                              fpga::CB_NODE_DST, backward_route.failure_dst);
      }
      // Preserve an unleased reverse-search path for a terminal failure image.
      if (!backward_route.success &&
          !backward_route.diagnostic_fragments.empty()) {
        rememberFailureVisualizationPath(
            task, std::move(backward_route.diagnostic_fragments));
      }
      reverse_expanded += backward_route.expanded;
      reverse_incoming_edges += backward_route.incoming_edges;
      reverse_near_source_edges +=
          backward_route.source_neighborhood_edges;
      reverse_probe_calls += backward_route.probe_calls;
      reverse_tile_visit_rejects += backward_route.tile_visit_reject_count;
      if (backward_route.success) {
        moving_source_probe_offsets.erase(source_cluster_key);
      } else if (backward_route.takeoff_candidates != 0) {
        size_t next = backward_route.probe_offset_used +
                      backward_route.probe_candidates_scanned;
        const bool has_remaining =
            pnr::movingSourceProbeWindowHasRemaining(
                backward_route.takeoff_candidates,
                backward_route.probe_offset_used,
                backward_route.probe_candidates_scanned,
                backward_route.remaining_frontier);
        reverse_probe_window_remaining |= has_remaining;
        moving_source_probe_offsets[source_cluster_key] =
            has_remaining ? next : 0;
      }
      if (backward_route.success) {
        break;
      }
    }
    if (backward_route.success) {
      break;
    }
  }
  if (!backward_route.success || !selected_resource || selected_pos < 0 ||
      selected_placements.empty()) {
    // An interrupted search has not exhausted its component. In particular,
    // do not preempt another net after the stage watchdog has stopped us.
    if (routeStageSearchCancelled()) {
      if (candidate_retryable) {
        *candidate_retryable = true;
      }
      return fail("source recovery cancelled before boundary preemption");
    }
    // The engine alone requests boundary preemption after free exhaustion.
    // A bounded probe must resume its window, not cut a victim here or move
    // the destination while its free continuation remains unexamined.
    if (reverse_probe_window_remaining) {
      if (candidate_retryable) {
        *candidate_retryable = true;
      }
      return fail("backward trunk has unexamined free continuations or placements; "
                  "retry required");
    }
    std::vector<pnr::MovingTerminalPath> terminal_paths;
    NodeMask terminal_pins;
    NodeMask terminal_locals;
    NodeMask terminal_dsts;
    NodeMask terminal_joints;
    for (Tile *target_tile :
         routeTileCandidates(*task.to, task.to_port, false, &source_hint)) {
      if (!target_tile || !target_tile->cb_type) {
        continue;
      }
      NodeMask pin_nodes =
          routeTileInputNodes(*target_tile, *task.to, task.to_port);
      const NodeMask incoming_dsts = incomingDstMaskForRouteTile(*target_tile);
      const bool restrict_to_physical_dsts = incoming_dsts != NodeMask{};
      pin_nodes.for_each_set_bit([&](int local) {
        for (const CBType::TerminalEntry &entry :
             target_tile->cb_type->terminalEntries(local)) {
          // Ignore subtype paths whose destination wire never physically
          // arrives at this tile; they cannot prevent terminal legalization.
          if (restrict_to_physical_dsts &&
              !incoming_dsts.testBit(entry.dst)) {
            continue;
          }
          terminal_paths.push_back(
              {local, entry.dst, entry.joint, entry.joint2});
        }
        return false;
      });
      if (terminal_paths.empty()) {
        continue;
      }
      terminal_pins = target_tile->pin_state.leased_nodes;
      terminal_locals = target_tile->cb.local.local;
      terminal_dsts = target_tile->cb.dst.jump;
      terminal_joints = target_tile->cb.joint.jump;
      break;
    }
    const bool terminal_is_blocked =
        reverse_expanded == 0 && reverse_incoming_edges == 0 &&
        pnr::movingSourceNeedsTerminalLegalization(
            terminal_paths, terminal_pins, terminal_locals, terminal_dsts,
            terminal_joints);
    const bool terminal_ingress_is_blocked =
        pnr::movingSourceNeedsIngressLegalization(
            !terminal_paths.empty(), backward_route.diagnostic_depth,
            backward_route.diagnostic_previous_dsts,
            backward_route.diagnostic_free_previous_dsts);
    // Relocate only a terminal bottleneck: either no physical seed is free or
    // a free seed cannot cross its immediate incoming routing boundary.
    if (terminal_is_blocked || terminal_ingress_is_blocked) {
      std::vector<RouteTask> legalized_tasks;
      std::string legalization_failure;
      if (moveUnfinishedDestination(task, &legalized_tasks,
                                    &legalization_failure)) {
        if (moved_tasks) {
          for (RouteTask &legalized : legalized_tasks) {
            appendUniqueRouteTask(*moved_tasks, legalized);
          }
        } else {
          for (RouteTask &legalized : legalized_tasks) {
            enqueueRouteTask(legalized, pending_route_todo);
          }
        }
        route_changed = true;
        route_progress = true;
        PNR_LOG1("ROUT",
                 "routeDesign Moving Sources terminal legalization: net='{}', "
                 "target='{}'/'{}', reason={}, replacement_tasks={}",
                 task.net_name, task.to->makeName(FULL_NAME_LIMIT), task.to_port,
                 terminal_is_blocked ? "terminal_busy" : "ingress_busy",
                 legalized_tasks.size());
        return true;
      }
      return fail("target terminal legalization failed: " +
                  legalization_failure);
    }
    return fail("backward trunk found no route-guided placement; expanded=" +
                std::to_string(reverse_expanded) +
                " incoming=" +
                std::to_string(reverse_incoming_edges) +
                " near_source=" +
                std::to_string(reverse_near_source_edges) +
                " probes=" + std::to_string(reverse_probe_calls) +
                " tile_visit_rejects=" +
                std::to_string(reverse_tile_visit_rejects) +
                " limit=" +
                std::to_string(backward_route.expansion_limit_reached) +
                " frontier=" +
                std::to_string(backward_route.remaining_frontier) +
                " diagnostic_incoming=" +
                std::to_string(backward_route.diagnostic_incoming_sources) +
                " diagnostic_cached=" +
                std::to_string(backward_route.diagnostic_cached_sources) +
                " diagnostic_window=" +
                std::to_string(backward_route.diagnostic_window_sources) +
                " diagnostic_resolved=" +
                std::to_string(backward_route.diagnostic_key_was_resolved) +
                " diagnostic_depth=" +
                std::to_string(backward_route.diagnostic_depth) +
                " diagnostic_depth_limit=" +
                std::to_string(
                    backward_route.diagnostic_depth_limit_reached) +
                " diagnostic_predecessors=" +
                std::to_string(backward_route.diagnostic_previous_dsts) +
                " diagnostic_free=" +
                std::to_string(backward_route.diagnostic_free_previous_dsts) +
                " diagnostic_children=" +
                std::to_string(backward_route.diagnostic_children) +
                " blocked_reverse=" +
                std::to_string(backward_route.blocked_reverse_edges) +
                " retained_blockers=" +
                std::to_string(
                    backward_route.blocked_reverse_frontier.size()) +
                " candidate_tiles=" +
                std::to_string(candidate_tiles_probed) +
                " placeable=" +
                std::to_string(candidate_positions_found) +
                " takeoff_candidates=" +
                std::to_string(backward_route.takeoff_candidates) +
                " probe_offset=" +
                std::to_string(backward_route.probe_offset_used) +
                " probe_scanned=" +
                std::to_string(backward_route.probe_candidates_scanned) +
                " probe_remaining=" +
                std::to_string(reverse_probe_window_remaining) +
                (last_probe_failure.empty()
                     ? std::string{}
                     : " last_probe='" + last_probe_failure + "'"));
  }

  recordRouteTaskFailure(task, selected_resource->coord, fpga::CB_NODE_LOCAL, -1);

  bool placement_unchanged = std::all_of(
      selected_placements.begin(), selected_placements.end(),
      [&](const fpga::ElementPackingChoice &choice) {
        return choice.inst && choice.inst->tile.peer &&
               &*choice.inst->tile == selected_resource &&
               choice.inst->pos == choice.pos;
      });

  // Changed placements already proved all inputs against the reserved output
  // inside the takeoff probe. Do not replay/copy their complete routes here.
  if (placement_unchanged && (!existing_route || existing_route->empty())) {
    for (Wire &fragment : backward_route.fragments) {
      fragment.net_name = task.net_name;
    }
    if (!backward_route.fragments.empty() &&
        backward_route.fragments.back().type == Wire::WIRE_TILE_PIN) {
      fillTilePinEndpoint(backward_route.fragments.back(), *task.to,
                          task.to_port, fpga::TILE_PIN_INPUT);
      refreshTilePinEndpoint(backward_route.fragments.back(), *task.to,
                             task.to_port, fpga::TILE_PIN_INPUT);
    }
    if (!commitPreparedRoute(backward_route.fragments)) {
      return fail("proven backward trunk could not be committed in place");
    }
    prependSourceEndpoint(backward_route.fragments, *task.from, task.from_port);
    task.to->wires.emplace_back(std::move(backward_route.fragments));
    size_t route_index = task.to->wires.size() - 1;
    size_t binding_index = fpga::attachNetRoute(
        *task.net, *task.to, route_index, task.from, task.to, task.from_port,
        task.to_port, task.net_name);
    fpga::registerNetRouteTiles(*task.net, task.to->wires.back(), binding_index);
    route_changed = true;
    route_progress = true;
    PNR_LOG1("ROUT",
             "routeDesign source route completed in place: net='{}', "
             "source='{}' ({},{})/{}, reverse_expanded={}, trunk_fragments={}",
             task.net_name, source->makeName(FULL_NAME_LIMIT), old_coord.x,
             old_coord.y, old_pos, backward_route.expanded,
             task.to->wires.back().size());
    return true;
  }

  std::vector<uint64_t> &tried =
      move_tried_placements[source_cluster_key];
  if (!placementWasTried(tried, old_coord, old_pos)) {
    tried.push_back(placementKey(old_coord, old_pos));
  }
  tried.push_back(placementKey(selected_resource->coord, selected_pos));

  struct SourcePlacementSnapshot {
    rtl::Inst *inst = nullptr;
    Tile *tile = nullptr;
    int pos = -1;
  };
  std::vector<SourcePlacementSnapshot> old_placements;
  old_placements.reserve(cluster.size());
  for (rtl::Inst *member : cluster) {
    old_placements.push_back(
        {member, member && member->tile.peer ? &*member->tile : nullptr,
         member ? member->pos : -1});
  }
  auto restore_source_cluster = [&]() {
    for (SourcePlacementSnapshot &old : old_placements) {
      if (!old.inst || !old.tile) {
        continue;
      }
      if (old.inst->tile.peer) {
        unplaceInst(*old.inst, "restore-route-guided-source-cluster");
      }
      restoreInstPlacement(*old.inst, *old.tile, old.pos);
    }
  };

  std::unordered_set<rtl::Inst *> endpoints = movingFocusEndpointClosure(source);
  std::vector<rtl::Net *> incident = incidentRouteNets(endpoints, task.net);
  std::vector<fpga::NetRouteRef> input_routes;
  std::vector<RouteTask> input_tasks;
  std::vector<std::pair<rtl::Net *, std::pair<rtl::Inst *, std::string>>>
      source_ports;
  for (rtl::Net *net : incident) {
    // Source relocation can also move a sequential element's clock sink.
    // Its protected tree stays leased and is rebuilt by Clock routing later.
    if (net && pnr::movementDefersProtectedTree(net->route_protected,
                                                net->distributed_source)) {
      clock_routes_dirty = std::any_of(
                               net->routes.begin(), net->routes.end(),
                               [&](const rtl::NetRouteBinding &binding) {
                                 return endpoints.contains(binding.from) ||
                                        endpoints.contains(binding.to);
                               }) ||
                           clock_routes_dirty;
      continue;
    }
    if (!net || (!net->routeCanBePreempted() && !net->distributed_source)) {
      continue;
    }
    for (size_t index = 0; index < net->routes.size(); ++index) {
      const rtl::NetRouteBinding &binding = net->routes[index];
      if (endpoints.contains(binding.to) && binding.from && binding.to) {
        input_routes.push_back({net, index});
        input_tasks.push_back(RouteTask{
            binding.from, binding.to, net, binding.from_port, binding.to_port,
            binding.route_name, 0, 0, 0, {},
            !net->distributed_source &&
                hasOtherCompleteSourceBinding(*binding.from,
                                              binding.from_port, &binding)});
      }
      if (endpoints.contains(binding.from) && binding.from &&
          !binding.from_port.empty()) {
        auto key = std::pair<rtl::Inst *, std::string>{binding.from,
                                                       binding.from_port};
        bool duplicate = std::any_of(
            source_ports.begin(), source_ports.end(), [&](const auto &old) {
              return old.first == net && old.second == key;
            });
        if (!duplicate) {
          source_ports.push_back({net, std::move(key)});
        }
      }
    }
  }
  // Bindings do not exist for every unfinished input yet. Reconstruct inputs
  // for every packed member so the complete moved source cluster is proven at
  // the selected placement before the scheduler can expose it to other work.
  for (rtl::Inst *member : cluster) {
    for (rtl::Conn &sink_conn : member->conns) {
      if (!sink_conn.port_ref.peer ||
          sink_conn.port_ref->type != rtl::Port::PORT_IN ||
          tech->check_clocked(member->cell_ref->type,
                              sink_conn.port_ref->name)) {
        continue;
      }
      rtl::Conn *driver_conn = sink_conn.follow();
      if (!driver_conn || !driver_conn->inst_ref.peer ||
          !driver_conn->port_ref.peer || driver_conn->port_ref->is_global ||
          !driver_conn->inst_ref->cell_ref.peer ||
          !driver_conn->inst_ref->cell_ref->module_ref.peer ||
          !driver_conn->inst_ref->cell_ref->module_ref->is_blackbox) {
        continue;
      }
      rtl::Net *input_net =
          findNetByDesignator(*member, sink_conn.port_ref->designator);
      if (!input_net ||
          input_net->designatorIsVoid(sink_conn.port_ref->designator)) {
        continue;
      }
      std::string driver_port = driver_conn->port_ref->makeName();
      RouteTask input_task{
          driver_conn->inst_ref.peer,
          member,
          input_net,
          driver_port,
          sink_conn.port_ref->makeName(),
          sink_conn.makeNetName(nullptr, FULL_NAME_LIMIT),
          0,
          0,
          0,
          {},
          !input_net->distributed_source &&
              sourceTreeHasCompleteExit(*driver_conn->inst_ref.peer,
                                        driver_port)};
      appendUniqueRouteTask(input_tasks, input_task);
    }
  }

  // Snapshot only route trees that the relocation transaction may mutate.
  // Candidate rejection restores these bindings, routes, masks, and queues.
  std::unordered_set<std::string> affected_source_keys;
  affected_source_keys.insert(sourceRouteKey(task.from, task.from_port));
  for (const RouteTask &input : input_tasks) {
    if (input.from && !input.from_port.empty()) {
      affected_source_keys.insert(sourceRouteKey(input.from, input.from_port));
    }
  }
  for (const auto &[net, endpoint] : source_ports) {
    (void)net;
    if (endpoint.first && !endpoint.second.empty()) {
      affected_source_keys.insert(
          sourceRouteKey(endpoint.first, endpoint.second));
    }
  }
  auto binding_is_affected = [&](const rtl::NetRouteBinding &binding) {
    return endpoints.contains(binding.from) || endpoints.contains(binding.to) ||
           (binding.from && !binding.from_port.empty() &&
            affected_source_keys.contains(
                sourceRouteKey(binding.from, binding.from_port)));
  };
  struct BindingSnapshot {
    rtl::Net *net = nullptr;
    size_t binding_index = 0;
    rtl::NetRouteBinding binding;
    bool has_storage = false;
    std::vector<Wire> route;
  };
  struct TileRouteSnapshot {
    Tile *tile = nullptr;
    CBState cb;
    TilePinState pins;
  };
  std::vector<BindingSnapshot> binding_snapshots;
  std::vector<TileRouteSnapshot> tile_route_snapshots;
  std::unordered_map<rtl::Net *, size_t> net_route_counts;
  std::unordered_set<rtl::Net *> affected_nets;
  auto snapshot_route_tile = [&](Tile *tile) {
    if (!tile || std::any_of(tile_route_snapshots.begin(),
                             tile_route_snapshots.end(),
                             [&](const TileRouteSnapshot &old) {
                               return old.tile == tile;
                             })) {
      return;
    }
    tile_route_snapshots.push_back({tile, tile->cb, tile->pin_state});
  };
  auto snapshot_route = [&](const std::vector<Wire> &route) {
    for (const Wire &fragment : route) {
      snapshot_route_tile(fpga->getTile(fragment.from.x, fragment.from.y));
      snapshot_route_tile(fpga->getTile(fragment.to.x, fragment.to.y));
    }
  };
  if (rtl::Module *module = parentModule(*source)) {
    for (auto &net_ref : module->nets) {
      rtl::Net &net = net_ref;
      for (size_t index = 0; index < net.routes.size(); ++index) {
        rtl::NetRouteBinding &binding = net.routes[index];
        if (!binding_is_affected(binding)) {
          continue;
        }
        affected_nets.insert(&net);
        net_route_counts.try_emplace(&net, net.routes.size());
        BindingSnapshot snapshot{&net, index, binding, false, {}};
        if (std::vector<Wire> *route = routeBindingRoute(binding)) {
          snapshot.has_storage = true;
          snapshot.route = *route;
          snapshot_route(snapshot.route);
        }
        binding_snapshots.push_back(std::move(snapshot));
      }
    }
  }
  for (const RouteTask &input : input_tasks) {
    if (input.net) {
      affected_nets.insert(input.net);
      net_route_counts.try_emplace(input.net, input.net->routes.size());
    }
  }
  for (const auto &[net, endpoint] : source_ports) {
    (void)endpoint;
    if (net) {
      affected_nets.insert(net);
      net_route_counts.try_emplace(net, net->routes.size());
    }
  }
  affected_nets.insert(task.net);
  net_route_counts.try_emplace(task.net, task.net->routes.size());
  const size_t pending_queue_size = pending_route_todo.size();
  const size_t fanout_queue_size = fanout_route_todo.size();
  auto rollback_route_first_move = [&]() {
    std::unordered_set<Tile *> touched_tiles;
    for (rtl::Net *net : affected_nets) {
      if (!net) {
        continue;
      }
      for (size_t index = 0; index < net->routes.size(); ++index) {
        rtl::NetRouteBinding &binding = net->routes[index];
        if (!binding_is_affected(binding)) {
          continue;
        }
        if (std::vector<Wire> *route = routeBindingRoute(binding)) {
          for (const Wire &fragment : *route) {
            if (Tile *tile = fpga->getTile(fragment.from.x, fragment.from.y)) {
              touched_tiles.insert(tile);
            }
            if (Tile *tile = fpga->getTile(fragment.to.x, fragment.to.y)) {
              touched_tiles.insert(tile);
            }
          }
        }
        fpga::unrouteNetRoute(*net, index);
      }
    }
    restore_source_cluster();
    for (const auto &[net, old_count] : net_route_counts) {
      while (net && net->routes.size() > old_count) {
        net->eraseRouteBinding(net->routes.size() - 1);
      }
    }
    for (const BindingSnapshot &snapshot : binding_snapshots) {
      if (!snapshot.net || snapshot.binding_index >= snapshot.net->routes.size()) {
        continue;
      }
      snapshot.net->routes[snapshot.binding_index] = snapshot.binding;
      snapshot.net->invalidateRouteLookup();
      if (snapshot.has_storage && snapshot.binding.owner &&
          snapshot.binding.route_index < snapshot.binding.owner->wires.size()) {
        snapshot.binding.owner->wires[snapshot.binding.route_index] =
            snapshot.route;
      }
    }
    for (const TileRouteSnapshot &snapshot : tile_route_snapshots) {
      snapshot.tile->cb = snapshot.cb;
      snapshot.tile->pin_state = snapshot.pins;
      touched_tiles.insert(snapshot.tile);
    }
    for (Tile *tile : touched_tiles) {
      for (rtl::Net *net : affected_nets) {
        tile->removeRoutedBindings(net);
      }
    }
    for (const BindingSnapshot &snapshot : binding_snapshots) {
      if (snapshot.net && !snapshot.route.empty()) {
        fpga::registerNetRouteTiles(*snapshot.net, snapshot.route,
                                    snapshot.binding_index);
      }
    }
    // Candidate-local work belongs to the rejected move. Transit trees
    // preempted while testing its inputs are foreign work and must survive.
    pnr::preserveExternalPreemptionTasks(
        pending_route_todo, pending_queue_size, [&](const RouteTask &queued) {
          if (endpoints.contains(queued.from) || endpoints.contains(queued.to)) {
            return true;
          }
          return queued.from && !queued.from_port.empty() &&
                 affected_source_keys.contains(
                     sourceRouteKey(queued.from, queued.from_port));
        });
    fanout_route_todo.resize(fanout_queue_size);
  };

  std::vector<RouteTask> displaced_outputs;
  for (auto &[net, endpoint] : source_ports) {
    unrouteSourceTree(*net, endpoint.first, endpoint.second,
                      &displaced_outputs, false, true);
  }
  // An input proof can extend an incomplete owned prefix. Preserve exactly
  // its selected anchor before sink invalidation releases the old tail.
  std::vector<fpga::ProvenRoutePrefix> proven_input_prefixes;
  for (const PreparedCandidateInput &prepared : selected_input_proofs) {
    if (!prepared.extends_own_prefix) {
      continue;
    }
    const RouteTask &input = prepared.task;
    auto binding = input.net->findRouteBinding(
        input.from, input.to, input.from_port, input.to_port, input.net_name);
    proven_input_prefixes.push_back(
        {{input.net, binding.index}, prepared.shared_prefix.size()});
  }
  bool inputs_invalidated =
      fpga::invalidateMovedSinkRoutes(input_routes, proven_input_prefixes);
  if (!proven_input_prefixes.empty() && !inputs_invalidated) {
    rollback_route_first_move();
    return fail("proven input prefix was lost before sink invalidation");
  }
  for (rtl::Inst *member : cluster) {
    unplaceInst(*member, "route-guided-source-cluster-move");
  }
  for (const fpga::ElementPackingChoice &choice : selected_placements) {
    if (!choice.inst || choice.pos < 0 ||
        selected_resource->tryAddAt(choice.inst, choice.pos, false) < 0) {
      rollback_route_first_move();
      return fail("selected route-guided cluster placement became unavailable");
    }
    choice.inst->coord = selected_resource->coord;
  }

  for (Wire &fragment : backward_route.fragments) {
    fragment.net_name = task.net_name;
  }
  if (!backward_route.fragments.empty() &&
      backward_route.fragments.back().type == Wire::WIRE_TILE_PIN) {
    fillTilePinEndpoint(backward_route.fragments.back(), *task.to,
                        task.to_port, fpga::TILE_PIN_INPUT);
    refreshTilePinEndpoint(backward_route.fragments.back(), *task.to,
                           task.to_port, fpga::TILE_PIN_INPUT);
  }
  if (!commitPreparedRoute(backward_route.fragments)) {
    rollback_route_first_move();
    return fail("proven backward trunk could not be committed after move");
  }
  prependSourceEndpoint(backward_route.fragments, *task.from, task.from_port);
  task.to->wires.emplace_back(std::move(backward_route.fragments));
  size_t route_index = task.to->wires.size() - 1;
  size_t binding_index = fpga::attachNetRoute(
      *task.net, *task.to, route_index, task.from, task.to, task.from_port,
      task.to_port, task.net_name);
  fpga::registerNetRouteTiles(*task.net, task.to->wires.back(), binding_index);

  // Inputs are rerouted synchronously at this placement. Focused preemption may
  // clear transit congestion, while endpoint and finished-tree guards remain.
  size_t immediate_input_count = input_tasks.size();
  std::vector<RouteTask> remaining_inputs = std::move(input_tasks);
  std::string constant_reroute_failure;
  if (!routeConstantTasksImmediately(remaining_inputs,
                                     &constant_reroute_failure)) {
    rollback_route_first_move();
    return fail(constant_reroute_failure);
  }

  // Commit the exact input paths proven before relocation. The proof paths
  // were held only in memory; live leases are acquired now, after placement.
  for (PreparedCandidateInput &prepared : selected_input_proofs) {
    auto pending = std::find_if(
        remaining_inputs.begin(), remaining_inputs.end(),
        [&](const RouteTask &input) { return sameRouteTask(input, prepared.task); });
    if (pending == remaining_inputs.end()) {
      continue;
    }
    std::vector<Wire> suffix = std::move(prepared.proof.fragments);
    for (Wire &fragment : suffix) {
      fragment.net_name = pending->net_name;
    }
    if (!suffix.empty() && suffix.back().type == Wire::WIRE_TILE_PIN) {
      fillTilePinEndpoint(suffix.back(), *pending->to, pending->to_port,
                          fpga::TILE_PIN_INPUT);
      refreshTilePinEndpoint(suffix.back(), *pending->to, pending->to_port,
                             fpga::TILE_PIN_INPUT);
    }
    if (prepared.extends_own_prefix) {
      // The old binding still owns the retained input trunk. Extend it in
      // place, releasing only the tail beyond the selected docking anchor.
      auto binding = pending->net->findRouteBinding(
          pending->from, pending->to, pending->from_port, pending->to_port,
          pending->net_name);
      std::vector<Wire> *route = binding.index < pending->net->routes.size()
                                    ? routeBindingRoute(pending->net->routes[binding.index])
                                    : nullptr;
      size_t keep = prepared.shared_prefix.size();
      size_t failed_fragment = suffix.size();
      if (!route || keep == 0 || keep > route->size() ||
          !commitPreparedRoute(suffix, &failed_fragment)) {
        PNR_LOG1("ROUT", "source input commit failure: trunk='{}' moving='{}' "
                 "old=({},{})/{} candidate=({},{})/{} input='{}' driver='{}'/{} "
                 "sink='{}'/{} binding={} binding_count={} keep={} live_size={} "
                 "failed_fragment={} suffix_size={}",
                 task.net_name, source->makeName(FULL_NAME_LIMIT), old_coord.x,
                 old_coord.y, old_pos, selected_resource->coord.x,
                 selected_resource->coord.y, selected_pos, pending->net_name,
                 pending->from->makeName(FULL_NAME_LIMIT), pending->from_port,
                 pending->to->makeName(FULL_NAME_LIMIT), pending->to_port,
                 binding.index, pending->net->routes.size(), keep,
                 route ? route->size() : 0, failed_fragment, suffix.size());
        for (const BindingSnapshot &snapshot : binding_snapshots) {
          if (snapshot.net == pending->net && snapshot.binding_index == binding.index) {
            PNR_LOG1("ROUT", "source input before invalidation: fragments={} complete={}",
                     snapshot.route.size(), routeIsComplete(snapshot.route));
            dumpRouteFragmentDiagnostics(*pending, 0, snapshot.route);
          }
        }
        PNR_LOG1("ROUT", "source input selected prefix:");
        dumpRouteFragmentDiagnostics(*pending, 1, prepared.shared_prefix);
        PNR_LOG1("ROUT", "source input proposed suffix:");
        dumpRouteFragmentDiagnostics(*pending, 2, suffix);
        rollback_route_first_move();
        return fail("retained input prefix could not be extended after move");
      }
      if (keep < route->size() &&
          !fpga::truncateNetRoute(*pending->net, binding.index, keep)) {
        fpga::releaseRouteLeases(suffix);
        rollback_route_first_move();
        return fail("retained input tail could not be released after move");
      }
      route->insert(route->end(), std::make_move_iterator(suffix.begin()),
                    std::make_move_iterator(suffix.end()));
      fpga::registerNetRouteTiles(*pending->net, *route, binding.index);
      remaining_inputs.erase(pending);
      continue;
    }
    if (!commitPreparedRoute(suffix)) {
      rollback_route_first_move();
      if (candidate_retryable) {
        *candidate_retryable = true;
      }
      return fail("pre-move input proof conflicted with the committed trunk");
    }

    std::vector<Wire> committed_route;
    if (!prepared.shared_prefix.empty()) {
      committed_route = std::move(prepared.shared_prefix);
      for (Wire &fragment : committed_route) {
        fragment.shared = true;
        fragment.net_name = pending->net_name;
      }
      committed_route.insert(committed_route.end(),
                             std::make_move_iterator(suffix.begin()),
                             std::make_move_iterator(suffix.end()));
    } else {
      committed_route = std::move(suffix);
      prependSourceEndpoint(committed_route, *pending->from,
                            pending->from_port);
    }
    pending->to->wires.emplace_back(std::move(committed_route));
    size_t input_route_index = pending->to->wires.size() - 1;
    size_t input_binding_index = fpga::attachNetRoute(
        *pending->net, *pending->to, input_route_index, pending->from,
        pending->to, pending->from_port, pending->to_port, pending->net_name);
    fpga::registerNetRouteTiles(*pending->net,
                                pending->to->wires[input_route_index],
                                input_binding_index);
    remaining_inputs.erase(pending);
  }

  bool saved_route_preemption = route_preemption_enabled;
  std::unordered_set<std::string> saved_preemption_protected_sources =
      std::move(preemption_protected_source_keys);
  preemption_protected_source_keys = affected_source_keys;
  route_preemption_enabled = pnr::immediateMovingInputMayPreempt(
      saved_route_preemption, moving_sources_stage,
      moving_focus_inst != nullptr);
  int input_pass_limit = std::max(
      16, (fpga::Device::current().size_width +
           fpga::Device::current().size_height + route_suffix_depth_limit - 1) /
              route_suffix_depth_limit);
  for (int pass = 0; pass < input_pass_limit && !remaining_inputs.empty();
       ++pass) {
    RouteBatchResult batch = routeTaskBatch(RouteTaskMode::Moving,
                                            remaining_inputs,
                                            remaining_inputs.size(), 5);
    std::erase_if(remaining_inputs,
                  [&](const RouteTask &input) {
                    const std::vector<Wire> *route = findBoundRoute(
                        input.net, input.from, input.to, input.from_port,
                        input.to_port, input.net_name);
                    return route && routeIsComplete(*route);
                  });
    if (batch.completed == 0 && batch.advanced == 0 && batch.changed == 0) {
      break;
    }
  }
  route_preemption_enabled = saved_route_preemption;
  preemption_protected_source_keys =
      std::move(saved_preemption_protected_sources);

  auto append_moved = [&](RouteTask replacement) {
    if (moved_tasks) {
      appendUniqueRouteTask(*moved_tasks, replacement);
    } else {
      enqueueRouteTask(replacement, pending_route_todo);
    }
  };
  if (!remaining_inputs.empty()) {
    PNR_LOG1("ROUT",
             "routeDesign source move immediate input failure: source='{}' "
             "selected=({},{})/{}, unfinished={}/{}",
             source->makeName(FULL_NAME_LIMIT), selected_resource->coord.x,
             selected_resource->coord.y, selected_pos, remaining_inputs.size(),
             immediate_input_count);
    for (size_t input_index = 0; input_index < remaining_inputs.size();
         ++input_index) {
      const RouteTask &input = remaining_inputs[input_index];
      const std::vector<Wire> *input_route = findBoundRoute(
          input.net, input.from, input.to, input.from_port, input.to_port,
          input.net_name);
      size_t input_crossbars =
          input_route
              ? std::count_if(input_route->begin(), input_route->end(),
                              [](const Wire &fragment) {
                                return fragment.type == Wire::WIRE_CROSSBAR;
                              })
              : 0;
      PNR_LOG1(
          "ROUT",
          "routeDesign source move unfinished input[{}]: net='{}', "
          "from='{}'/'{}' ({},{})/{}, to='{}'/'{}' ({},{})/{}, "
          "fanout={}, route_size={}, xbars={}, complete={}",
          input_index, input.net_name,
          input.from ? input.from->makeName(FULL_NAME_LIMIT) : std::string{},
          input.from_port,
          input.from && input.from->tile.peer ? input.from->tile->coord.x : -1,
          input.from && input.from->tile.peer ? input.from->tile->coord.y : -1,
          input.from ? input.from->pos : -1,
          input.to ? input.to->makeName(FULL_NAME_LIMIT) : std::string{},
          input.to_port,
          input.to && input.to->tile.peer ? input.to->tile->coord.x : -1,
          input.to && input.to->tile.peer ? input.to->tile->coord.y : -1,
          input.to ? input.to->pos : -1, input.fanout,
          input_route ? input_route->size() : 0,
          input_crossbars,
          input_route && routeIsComplete(*input_route));
      dumpTargetCandidateSummaryLog(input, input_index);
    }
    // Reject atomically: the old placement, exact route vectors, route masks,
    // ownership registrations, and scheduler queues all return unchanged. The
    // failed placement remains in history so the fair queue tries a different
    // route-guided candidate on its next source-recovery cycle.
    size_t unfinished_inputs = remaining_inputs.size();
    rollback_route_first_move();
    if (candidate_retryable) {
      *candidate_retryable = true;
    }
    return fail("immediate input reroute left " +
                std::to_string(unfinished_inputs) + " unfinished");
  }

  size_t displaced_fanouts = 0;
  size_t displaced_trunks = 0;
  for (RouteTask &replacement : displaced_outputs) {
    if (sameRouteTask(replacement, task)) {
      continue;
    }
    canonicalizeRouteTaskSource(replacement);
    bool same_source_port = replacement.from == task.from &&
                            replacement.from_port == task.from_port;
    bool source_has_complete_exit =
        replacement.from && sourceTreeHasCompleteExit(
                                *replacement.from, replacement.from_port);
    if (pnr::movedSourceRouteBecomesFanout(same_source_port,
                                           source_has_complete_exit)) {
      replacement.fanout = true;
      appendUniqueRouteTask(fanout_route_todo, replacement);
      ++displaced_fanouts;
    } else {
      replacement.fanout = false;
      append_moved(replacement);
      ++displaced_trunks;
    }
  }

  route_changed = true;
  route_progress = true;
  if (source_moved) {
    *source_moved = true;
  }
  PNR_LOG1("ROUT",
           "routeDesign source move routed-first: net='{}', source='{}' "
           "({},{})/{} -> ({},{})/{}, reverse_expanded={}, trunk_fragments={}, "
           "inputs={}, displaced_fanouts={}, displaced_trunks={}",
           task.net_name, source->makeName(FULL_NAME_LIMIT), old_coord.x,
           old_coord.y, old_pos, selected_resource->coord.x,
           selected_resource->coord.y, selected_pos, backward_route.expanded,
           task.to->wires.back().size(), immediate_input_count,
           displaced_fanouts, displaced_trunks);
  return true;
}

// Relocate the physical load of an unfinished suffix while retaining the
// existing destination-focused Moving behavior.
bool RouteDesign::moveUnfinishedDestination(
    const RouteTask &task, std::vector<RouteTask> *moved_tasks,
    std::string *fail_reason) {
  return moveUnfinishedCell(task, moved_tasks, &task, fail_reason);
}

// Moving sources is complete when every source touched by the moved cluster
// owns one completed trunk; secondary bindings remain deferred fanouts.
bool RouteDesign::movingSourceTrunksComplete(rtl::Inst &inst) {
  std::vector<RouteTask> incomplete;
  collectIncompleteIncidentRouteTasks(inst, incomplete);
  std::unordered_set<rtl::Inst *> focus_endpoints =
      movingFocusEndpointClosure(&inst);
  for (RouteTask &task : incomplete) {
    canonicalizeRouteTaskSource(task);
    if (!task.from || !focus_endpoints.contains(task.from)) {
      continue;
    }
    if (!task.net || task.net->distributed_source ||
        !sourceTreeHasCompleteExit(*task.from, task.from_port)) {
      return false;
    }
  }
  return true;
}

// Rebuild one Generic trunk task per missing physical source and park every
// already-seeded secondary binding for the later Fanouts stage.
size_t RouteDesign::collectMovingSourceTasks(
    rtl::Inst &inst, std::vector<RouteTask> &trunk_tasks,
    std::vector<RouteTask> &fanout_tasks) {
  std::vector<RouteTask> incomplete;
  collectIncompleteIncidentRouteTasks(inst, incomplete);
  for (RouteTask &task : incomplete) {
    canonicalizeRouteTaskSource(task);
  }
  pnr::normalizeMovingSourceRoles(
      incomplete,
      [&](const RouteTask &task) {
        return task.from ? sourceRouteKey(task.from, task.from_port)
                         : std::string{};
      },
      [&](const RouteTask &task) {
        return task.from &&
               sourceTreeHasCompleteExit(*task.from, task.from_port);
      },
      [](const RouteTask &task) {
        return task.net && task.net->distributed_source;
      });
  size_t trunks = 0;
  for (RouteTask &task : incomplete) {
    if (task.fanout) {
      appendUniqueRouteTask(fanout_tasks, task);
    } else if (appendUniqueRouteTask(trunk_tasks, task)) {
      ++trunks;
    }
  }
  return trunks;
}

bool RouteDesign::routeTaskDebugMatches(const RouteTask &task) const {
  if (!routeDebugEnabled("SCALEPNR_DEBUG_TASK_NET")) {
    return false;
  }
  return routeDebugMatches("SCALEPNR_DEBUG_TASK_NET", task.net_name) ||
         routeDebugMatches("SCALEPNR_DEBUG_TASK_NET", task.from_port) ||
         routeDebugMatches("SCALEPNR_DEBUG_TASK_NET", task.to_port) ||
         (task.from &&
          routeDebugMatches("SCALEPNR_DEBUG_TASK_NET",
                            task.from->makeName(FULL_NAME_LIMIT))) ||
         (task.to && routeDebugMatches("SCALEPNR_DEBUG_TASK_NET",
                                       task.to->makeName(FULL_NAME_LIMIT)));
}

void RouteDesign::logRouteTaskDecision(const char *phase, const RouteTask &task,
                                       const std::string &detail) const {
  if (!routeTaskDebugMatches(task)) {
    return;
  }
  const bool has_seed =
      task.from && hasCompleteSourceExitBinding(*task.from, task.from_port);
  const std::vector<Wire> *existing_route =
      findBoundRoute(task.net, task.from, task.to, task.from_port, task.to_port,
                     task.net_name);
  PNR_LOG1(
      "ROUT",
      "routeDesign task debug: phase={}, net='{}', from='{}' type='{}' "
      "port='{}' tile=({},{})/{}, to='{}' type='{}' port='{}' tile=({},{})/{}, "
      "fanout={}, attempt={}, has_source_exit={}, "
      "route(size={},xbars={},complete={}), "
      "queues(basic={},fanout={},pending={},moving={}){}{}",
      phase ? phase : "", task.net_name,
      task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
      task.from && task.from->cell_ref.peer ? task.from->cell_ref->type
                                            : std::string{},
      task.from_port,
      task.from && task.from->tile.peer ? task.from->tile->coord.x : -1,
      task.from && task.from->tile.peer ? task.from->tile->coord.y : -1,
      task.from ? task.from->pos : -1,
      task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
      task.to && task.to->cell_ref.peer ? task.to->cell_ref->type
                                        : std::string{},
      task.to_port, task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
      task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
      task.to ? task.to->pos : -1, task.fanout, task.attempt, has_seed,
      existing_route ? existing_route->size() : 0,
      routeCrossbarFragments(existing_route),
      existing_route && routeIsComplete(*existing_route), route_todo.size(),
      fanout_route_todo.size(), pending_route_todo.size(),
      moving_deferred_todo.size(), detail.empty() ? "" : ", detail=", detail);
}

void RouteDesign::collectRouteTasks(rtl::Inst &inst, RegBunch *bunch) {
  if (inst.mark == travers_mark) {
    return;
  }

  inst.mark = travers_mark;

  for (auto &conn : std::ranges::views::reverse(inst.conns)) {
    rtl::Conn *curr = &conn;
    if (curr->port_ref->type != rtl::Port::PORT_IN) {
      continue;
    }
    if (tech->check_clocked(curr->inst_ref->cell_ref->type,
                            curr->port_ref->name)) {
      continue;
    }
    curr = curr->follow();
    if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox ||
        curr->port_ref->is_global) {
      continue;
    }
    rtl::Inst *source_inst = curr->inst_ref.peer;
    std::string source_port_name = curr->port_ref->makeName();
    std::string source_key = sourceRouteKey(source_inst, source_port_name);
    rtl::Net *net = findNetByDesignator(inst, conn.port_ref->designator);
    std::string route_name = conn.makeNetName(nullptr, FULL_NAME_LIMIT);
    rtl::NetRouteBinding *completed_binding =
        net && source_inst
            ? findNetRouteBinding(*net, *source_inst, inst, source_port_name,
                                  conn.port_ref->makeName(), route_name)
            : nullptr;
    if (completed_binding) {
      const std::vector<Wire> *completed_route =
          routeBindingRoute(*completed_binding);
      if (completed_route && routeIsComplete(*completed_route)) {
        collectRouteTasks(*curr->inst_ref.peer, nullptr);
        continue;
      }
    }
    if (net && net->designatorIsVoid(conn.port_ref->designator)) {
      if (source_inst) {
        source_route_marks.insert(source_key);
      }
      collectRouteTasks(*curr->inst_ref.peer, nullptr);
      continue;
    }
    RouteTask task{source_inst,
                   &inst,
                   net,
                   source_port_name,
                   conn.port_ref->makeName(),
                   route_name,
                   0,
                   0,
                   0,
                   {},
                   false};
    indexSourceRoute(net, source_inst, source_port_name);
    if (source_inst && source_route_marks.contains(source_key)) {
      task.fanout = true;
      logRouteTaskDecision(
          "collect.defer_fanout", task,
          "source already marked before this task was collected");
      fanout_route_todo.push_back(std::move(task));
    } else {
      if (source_inst) {
        source_route_marks.insert(source_key);
      }
      logRouteTaskDecision("collect.seed_basic", task,
                           "first task collected for this source");
      route_todo.push_back(std::move(task));
    }
    collectRouteTasks(*curr->inst_ref.peer, nullptr);
  }

  if (bunch) {
    for (auto &subbunch : bunch->sub_bunches) {
      collectRouteTasks(*subbunch.reg, &subbunch);
    }
  }
}

RouteDesign::RouteBatchResult
RouteDesign::routeTaskBatch(RouteTaskMode mode, std::vector<RouteTask> &tasks,
                            size_t task_limit, int recursion_limit) {
  // A batch is one routing pass; preemption suppression must not leak into the
  // next pass.
  resetPassPreemptionState();
  RouteBatchResult result;
  result.before = tasks.size();
  size_t attempted_this_batch = 0;

  // Index pending Generic sources once per pass; scanning the full task queue
  // for every fanout is quadratic on large recovered Moving queues.
  std::unordered_set<std::string> moving_pending_seed_sources;
  if (mode == RouteTaskMode::Moving) {
    for (RouteTask &queued : tasks) {
      canonicalizeRouteTaskSource(queued);
      if (!queued.remove_after_pass && !queued.fanout && queued.from) {
        moving_pending_seed_sources.insert(
            sourceRouteKey(queued.from, queued.from_port));
      }
    }
  }
  auto moving_source_seed_pending = [&](const RouteTask &fanout_task) {
    return mode == RouteTaskMode::Moving && fanout_task.from &&
           moving_pending_seed_sources.contains(
               sourceRouteKey(fanout_task.from, fanout_task.from_port));
  };

  auto watched_route = [&]() -> std::vector<Wire> * {
    if (!debug_route_watch.initialized) {
      return nullptr;
    }
    RouteTask &watched = debug_route_watch.task;
    return findBoundRoute(watched.net, watched.from, watched.to,
                          watched.from_port, watched.to_port, watched.net_name);
  };
  auto watched_path = [&](const std::vector<Wire> *route) {
    std::string path;
    if (!route) {
      return path;
    }
    for (const std::string &node : routeTreeNodes(*route)) {
      if (!path.empty()) {
        path += "->";
      }
      path += node;
    }
    return path;
  };
  auto mode_name = [&]() {
    switch (mode) {
    case RouteTaskMode::Constant: return "Constant";
    case RouteTaskMode::Generic: return "Generic";
    case RouteTaskMode::Fanout: return "Fanout";
    case RouteTaskMode::Moving: return "Moving";
    }
    return "Unknown";
  };

  for (auto it = tasks.begin(); it != tasks.end();) {
    if (routeStageSearchCancelled()) {
      std::rotate(tasks.begin(), it, tasks.end());
      break;
    }
    if (attempted_this_batch >= task_limit) {
      std::rotate(tasks.begin(), it, tasks.end());
      break;
    }
    if (it->net && it->net->distributed_source) {
      PNR_ASSERT(pnr::constantTaskModeIsValid(
                     mode == RouteTaskMode::Constant, true),
                 "distributed route '{}' escaped the Const routing stage",
                 it->net_name);
      // Every constant destination owns an independent local physical root.
      it->fanout = false;
      // Reconstructed Moving/repair tasks recover source polarity from the
      // stable net rather than an aggregate-constructor default.
      it->distributed_one = it->net->distributed_one;
    } else {
      PNR_ASSERT(pnr::constantTaskModeIsValid(
                     mode == RouteTaskMode::Constant, false),
                 "ordinary route '{}' entered the Const routing stage",
                 it->net_name);
    }
    if (pnr::movingFanoutWaitsForSourceSeed(mode == RouteTaskMode::Moving,
                                            it->fanout,
                                            moving_source_seed_pending(*it))) {
      pending_route_todo.push_back(*it);
      ++result.deferred_fanout;
      it->remove_after_pass = true;
      ++it;
      continue;
    }
    if (!it->from || !it->to) {
      it->remove_after_pass = true;
      ++it;
      continue;
    }
    if (!it->net_name.empty() &&
        preempted_route_names_this_pass.contains(it->net_name)) {
      ++it;
      continue;
    }
    if (mode == RouteTaskMode::Fanout) {
      if (!it->fanout) {
        // The outer scheduler will return this seed to Generic mode at the
        // end of the pass without letting Fanout mutate its partial route.
        ++it;
        continue;
      } else {
        if (prepareRouteTaskEndpoints(*it, false)) {
          logRouteTaskDecision("batch.fanout.passthrough", *it,
                               "reused and retargeted tile-local passthrough "
                               "before Fanout assertion");
        }
        it->endpoints_prepared = true;
      }
      bool has_complete_seed =
          sourceTreeHasCompleteExit(*it->from, it->from_port);
      const std::vector<Wire> *queued_route = findBoundRoute(
          it->net, it->from, it->to, it->from_port, it->to_port, it->net_name);
      bool queued_route_complete =
          queued_route && routeIsComplete(*queued_route);
      if (pnr::fanoutWaitsForGenericSeed(
              true, it->fanout, queued_route_complete, has_complete_seed)) {
        logRouteTaskDecision(
            "batch.fanout.defer_missing_seed", *it,
            "fanout task waits until Generic creates a routed source exit");
        fanout_route_todo.push_back(std::move(*it));
        it->remove_after_pass = true;
        ++it;
        ++result.deferred_fanout;
        continue;
      }
    }
    ++attempted_this_batch;
    ++result.attempted;
    const bool initializes_invalidation_watch =
        routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET", it->net_name) ||
        routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET",
                          it->from->makeName(FULL_NAME_LIMIT)) ||
        routeDebugMatches("SCALEPNR_DEBUG_INVALIDATION_NET",
                          it->to->makeName(FULL_NAME_LIMIT));
    if (!debug_route_watch.initialized && initializes_invalidation_watch) {
      debug_route_watch.task = *it;
      debug_route_watch.initialized = true;
      std::vector<Wire> *route = watched_route();
      PNR_LOG1("ROUT",
               "routeDesign invalidation trace: watch initialized net='{}', "
               "from='{}'/'{}', to='{}'/'{}', complete={}, size={}, path={}",
               it->net_name, it->from->makeName(FULL_NAME_LIMIT), it->from_port,
               it->to->makeName(FULL_NAME_LIMIT), it->to_port,
               route && routeIsComplete(*route), route ? route->size() : 0,
               watched_path(route));
    }
    std::vector<Wire> *watched_before_route = watched_route();
    bool watched_complete_before =
        watched_before_route && routeIsComplete(*watched_before_route);
    size_t watched_size_before =
        watched_before_route ? watched_before_route->size() : 0;
    std::string watched_path_before = watched_complete_before
                                          ? watched_path(watched_before_route)
                                          : std::string{};
    if (mode == RouteTaskMode::Fanout && result.attempted_names.size() < 16) {
      result.attempted_names.push_back(it->net_name);
    }
    if (std::getenv("SCALEPNR_TRACE_ROUTE_TASKS")) {
      PNR_LOG1("ROUT",
               "routeTaskBatch attempt: mode={}, index={}, remaining={}, "
               "net='{}', from='{}'/'{}' tile=({},{})/{}, to='{}'/'{}' "
               "tile=({},{})/{}, fanout={}, attempt={}",
               mode_name(),
               attempted_this_batch, tasks.size(), it->net_name,
               it->from->makeName(FULL_NAME_LIMIT), it->from_port,
               it->from->tile.peer ? it->from->tile->coord.x : -1,
               it->from->tile.peer ? it->from->tile->coord.y : -1,
               it->from->pos, it->to->makeName(FULL_NAME_LIMIT), it->to_port,
               it->to->tile.peer ? it->to->tile->coord.x : -1,
               it->to->tile.peer ? it->to->tile->coord.y : -1, it->to->pos,
               it->fanout, it->attempt);
    }

    const int task_attempt_budget = pnr::routeTaskAttemptBudget(
        mode == RouteTaskMode::Generic || mode == RouteTaskMode::Constant,
        route_suffix_depth_limit == 1,
        recursion_limit);
    route_recursion_budget = task_attempt_budget;
    bool task_complete = false;
    bool task_progress = false;
    bool task_changed = false;
    bool original_fanout = it->fanout;
    rtl::NetRouteBinding *current_binding = nullptr;
    bool complete_seed = false;
    bool incomplete_source = false;
    if (pnr::routeBatchNeedsSourceTreeResetState(
            mode == RouteTaskMode::Moving)) {
      current_binding =
          it->net && it->from && it->to
              ? findNetRouteBinding(*it->net, *it->from, *it->to,
                                    it->from_port, it->to_port, it->net_name)
              : nullptr;
      complete_seed =
          it->from && sourceTreeHasCompleteExit(*it->from, it->from_port);
      incomplete_source =
          it->from && hasOtherIncompleteSourceBinding(
                          *it->from, it->from_port, current_binding);
    }
    if (pnr::resetIncompleteSourceTree(mode == RouteTaskMode::Moving,
                                       it->fanout, complete_seed,
                                       incomplete_source)) {
      // Stale partial siblings own the takeoff but cannot serve as a Fanout
      // trunk. Release the tree once, keep this task Generic, and defer every
      // sibling.
      std::vector<RouteTask> recovered;
      bool watched_complete_before_reset =
          watched_route() && routeIsComplete(*watched_route());
      size_t watched_size_before_reset =
          watched_route() ? watched_route()->size() : 0;
      unrouteSourceTree(*it->net, it->from, it->from_port, &recovered, false);
      bool watched_complete_after_reset =
          watched_route() && routeIsComplete(*watched_route());
      size_t watched_size_after_reset =
          watched_route() ? watched_route()->size() : 0;
      if (watched_complete_before_reset && !watched_complete_after_reset) {
        PNR_LOG1("ROUT",
                 "routeDesign invalidation trace: resetIncompleteSourceTree "
                 "invalidated watched net='{}' while processing culprit "
                 "net='{}', from='{}'/'{}', to='{}'/'{}', fanout={}, complete "
                 "{}->{}, size {}->{}, recovered={}, before_path={}",
                 debug_route_watch.task.net_name, it->net_name,
                 it->from->makeName(FULL_NAME_LIMIT), it->from_port,
                 it->to->makeName(FULL_NAME_LIMIT), it->to_port, it->fanout,
                 watched_complete_before_reset, watched_complete_after_reset,
                 watched_size_before_reset, watched_size_after_reset,
                 recovered.size(), watched_path_before);
      }
      for (RouteTask &sibling : recovered) {
        if (sameRouteTask(sibling, *it)) {
          continue;
        }
        sibling.fanout = true;
        enqueueRouteTask(sibling, pending_route_todo);
      }
      it->fanout = false;
      moving_pending_seed_sources.insert(
          sourceRouteKey(it->from, it->from_port));
      task_changed = true;
      logRouteTaskDecision(
          "batch.moving.reset_incomplete_source", *it,
          "released stale partial source tree before Generic seed retry");
    }
    if (mode == RouteTaskMode::Generic) {
      it->fanout = false;
      moving_pending_seed_sources.insert(
          sourceRouteKey(it->from, it->from_port));
    } else if (mode == RouteTaskMode::Fanout && it->fanout) {
      // Fanout tasks must branch from a Generic-routed source exit; starting
      // from the source tile is illegal here.
      logRouteTaskDecision(
          "batch.fanout.assert_source_exit", *it,
          "Fanout mode has an already routed Generic source exit");
      it->fanout = true;
    } else if (mode == RouteTaskMode::Moving && it->fanout &&
               !sourceTreeHasCompleteExit(*it->from, it->from_port)) {
      logRouteTaskDecision(
          "batch.moving.promote_missing_seed", *it,
          "fanout task lost its routed source exit before this Moving attempt");
      it->fanout = false;
    }

    std::vector<Wire> *route_before = findBoundRoute(
        it->net, it->from, it->to, it->from_port, it->to_port, it->net_name);
    size_t route_size_before = route_before ? route_before->size() : 0;
    size_t route_xbars_before = routeCrossbarFragments(route_before);
    RouteStats stats_before = route_stats;
    debug_active_route_task = *it;
    debug_active_route_task_valid = true;
    while (route_recursion_budget > 0) {
      route_changed = false;
      route_progress = false;
      if (routeNetTask(*it)) {
        task_complete = true;
        task_progress = true;
        task_changed = true;
        break;
      }
      task_changed = task_changed || route_changed;
      if (!route_progress) {
        break;
      }
      task_progress = true;
    }
    if (route_stats.preempt_success != stats_before.preempt_success) {
      // A victim may have supplied a cached branch through a shared prefix.
      // Rebuild lazily for the next task from the surviving route bindings.
      fanout_branch_indexes.clear();
    }
    std::vector<Wire> *route_after_for_progress = findBoundRoute(
        it->net, it->from, it->to, it->from_port, it->to_port, it->net_name);
    size_t route_size_after_for_progress =
        route_after_for_progress ? route_after_for_progress->size() : 0;
    if (!task_complete) {
      task_progress = route_size_after_for_progress > route_size_before;
      it->no_progress_passes = mode == RouteTaskMode::Moving
                                   ? pnr::updateMovingTaskNoProgressPasses(
                                         it->no_progress_passes, task_progress)
                                   : 0;
    }
    debug_active_route_task_valid = false;
    std::vector<Wire> *watched_after_route = watched_route();
    bool watched_complete_after =
        watched_after_route && routeIsComplete(*watched_after_route);
    size_t watched_size_after =
        watched_after_route ? watched_after_route->size() : 0;
    if (watched_complete_before != watched_complete_after ||
        watched_size_before != watched_size_after) {
      PNR_LOG1("ROUT",
               "routeDesign invalidation trace: mode={}, culprit net='{}', "
               "from='{}'/'{}', to='{}'/'{}', fanout={}, watched net='{}', "
               "complete {}->{}, size {}->{}, before_path={}, after_path={}",
               mode_name(), it->net_name, it->from->makeName(FULL_NAME_LIMIT),
               it->from_port, it->to->makeName(FULL_NAME_LIMIT), it->to_port,
               it->fanout, debug_route_watch.task.net_name,
               watched_complete_before, watched_complete_after,
               watched_size_before, watched_size_after, watched_path_before,
               watched_path(watched_after_route));
    }
    if (it->source_tree_rebuilt) {
      // routeFanoutTask promoted this moved sink to the replacement Generic
      // seed; do not restore its old Fanout role after returning to the batch.
      it->fanout = false;
      it->source_tree_rebuilt = false;
      if (it->from) {
        moving_pending_seed_sources.insert(
            sourceRouteKey(it->from, it->from_port));
      }
    } else if (original_fanout && mode == RouteTaskMode::Fanout && it->from &&
        !sourceTreeHasCompleteExit(*it->from, it->from_port)) {
      PNR_ASSERT(false,
                 "Fanout route '{}' invalidated Generic seed from '{}'/'{}' "
                 "while routing to '{}'/'{}'",
                 it->net_name, it->from->makeName(FULL_NAME_LIMIT),
                 it->from_port, it->to->makeName(FULL_NAME_LIMIT), it->to_port);
    } else {
      it->fanout = original_fanout;
    }
    logRouteTaskDecision(
        "batch.result", *it,
        "complete=" + std::string(task_complete ? "true" : "false") +
            ", progress=" + (task_progress ? "true" : "false") +
            ", changed=" + (task_changed ? "true" : "false") + ", mode=" +
            mode_name());
    if (mode == RouteTaskMode::Generic && !task_complete && !task_progress &&
        !task_changed &&
        route_stats.route_searches == stats_before.route_searches) {
      PNR_LOG1("ROUT",
               "routeDesign generic no-search task: net='{}', from='{}' "
               "type='{}' port='{}' tile=({},{})/{}, to='{}' type='{}' "
               "port='{}' tile=({},{})/{}, attempt={}",
               it->net_name,
               it->from ? it->from->makeName(FULL_NAME_LIMIT) : std::string{},
               it->from && it->from->cell_ref.peer ? it->from->cell_ref->type
                                                   : std::string{},
               it->from_port,
               it->from && it->from->tile.peer ? it->from->tile->coord.x : -1,
               it->from && it->from->tile.peer ? it->from->tile->coord.y : -1,
               it->from ? it->from->pos : -1,
               it->to ? it->to->makeName(FULL_NAME_LIMIT) : std::string{},
               it->to && it->to->cell_ref.peer ? it->to->cell_ref->type
                                               : std::string{},
               it->to_port,
               it->to && it->to->tile.peer ? it->to->tile->coord.x : -1,
               it->to && it->to->tile.peer ? it->to->tile->coord.y : -1,
               it->to ? it->to->pos : -1, it->attempt);
    }
    if (mode == RouteTaskMode::Moving && routeTaskDebugMatches(*it)) {
      std::vector<Wire> *route_after = findBoundRoute(
          it->net, it->from, it->to, it->from_port, it->to_port, it->net_name);
      size_t route_size_after = route_after ? route_after->size() : 0;
      size_t route_xbars_after = routeCrossbarFragments(route_after);
      PNR_LOG1(
          "ROUT",
          "routeDesign moving debug: route task result complete={} progress={} "
          "changed={} recursions={} net='{}' from='{}'/'{}' to='{}'/'{}' "
          "fanout={} route(size {}->{}, xbars {}->{}) change(failed={}, "
          "busy={}, busy_src={}, busy_dst={}, no_src={}, searches={}, pops={})",
          task_complete, task_progress, task_changed,
          task_attempt_budget - route_recursion_budget, it->net_name,
          it->from ? it->from->makeName(FULL_NAME_LIMIT) : std::string{},
          it->from_port,
          it->to ? it->to->makeName(FULL_NAME_LIMIT) : std::string{},
          it->to_port, it->fanout, route_size_before, route_size_after,
          route_xbars_before, route_xbars_after,
          route_stats.failed - stats_before.failed,
          route_stats.edge_rejected_busy - stats_before.edge_rejected_busy,
          route_stats.edge_rejected_busy_src -
              stats_before.edge_rejected_busy_src,
          route_stats.edge_rejected_busy_dst -
              stats_before.edge_rejected_busy_dst,
          route_stats.no_src_nodes - stats_before.no_src_nodes,
          route_stats.route_searches - stats_before.route_searches,
          route_stats.search_pops - stats_before.search_pops);
    }

    if (pnr::shouldRotateFailedGenericSeed(
            mode == RouteTaskMode::Generic, task_complete, task_progress,
            !route_after_for_progress || route_after_for_progress->empty()) &&
        rotateFailedGenericSeed(*it)) {
      task_changed = true;
      logRouteTaskDecision("batch.generic.rotate_seed", *it,
                           "replaced a repeatedly failed sink with another "
                           "sink from the same source port");
    }

    if (task_complete) {
      noteRouteProgressCompletion();
      ++result.completed;
      if (pnr::movingCompletionRenewsPlacement(
              true, it->net && it->net->distributed_source)) {
        ++result.placement_completed;
      }
      ++result.active;
      if (mode == RouteTaskMode::Moving && !it->fanout && it->from) {
        moving_pending_seed_sources.erase(
            sourceRouteKey(it->from, it->from_port));
      }
      // The outer pass compacts all completed tasks once. Erasing each item
      // here shifts the remaining vector and is quadratic on large designs.
      it->remove_after_pass = true;
      ++it;
      continue;
    }
    if (task_progress) {
      ++result.advanced;
    }
    if (task_changed) {
      ++result.changed;
    }
    if (task_changed || task_progress) {
      ++result.active;
    }
    ++it;
  }
  result.after = tasks.size();
  return result;
}

bool RouteDesign::rotateFailedGenericSeed(RouteTask &task) {
  return pnr::rotateFailedGenericSeedTaskNearest(
      task, fanout_route_todo, pending_route_todo,
      [](const RouteTask &left, const RouteTask &right) {
        return sameRouteTask(left, right);
      },
      [&](RouteTask failed) {
        appendUniqueRouteTask(fanout_route_todo, failed);
      },
      [](const RouteTask &candidate) {
        if (!candidate.from || !candidate.to || !candidate.from->tile.peer ||
            !candidate.to->tile.peer) {
          return std::numeric_limits<int>::max();
        }
        return routeDistance(candidate.from->tile->coord,
                             candidate.to->tile->coord);
      });
}

bool RouteDesign::routeInstTask(rtl::Inst &inst, int depth) {
  PNR_LOG2_("ROUT", depth, "routeInst, inst: '{}' ({}), x: {}, y: {}",
            inst.makeName(), inst.cell_ref->type, inst.coord.x, inst.coord.y);

  bool task_complete = true;
  for (auto &conn : std::ranges::views::reverse(inst.conns)) {
    rtl::Conn *curr = &conn;
    if (curr->port_ref->type == rtl::Port::PORT_IN) {
      if (tech->check_clocked(curr->inst_ref->cell_ref->type,
                              curr->port_ref->name)) { // clock ports
        // route clocks
        continue;
      }

      curr = curr->follow();
      if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox ||
          curr->port_ref->is_global) { // after BUFs (can be something?)
        continue;
      }

      rtl::Inst *peer = curr->inst_ref.peer;
      std::string net_name = conn.makeNetName(nullptr, FULL_NAME_LIMIT);

      std::vector<Wire> *existing_route = findRoute(inst, net_name);
      if (!hasRoutedNet(inst, net_name)) {
        task_complete = false;
        if (route_recursion_budget <= 0) {
          return false;
        }

        bool route_complete = routeIsComplete(
            existing_route ? *existing_route : std::vector<Wire>{});
        while (!route_complete && route_recursion_budget > 0) {
          if (existing_route && !routeIsComplete(*existing_route)) {
            size_t before_size = existing_route->size();
            if (continuePartialRoute(
                    *existing_route, inst, conn.port_ref->makeName(),
                    iteration_limit, route_complete, &route_stats, this,
                    nullptr, peer, curr->port_ref->makeName(), {})) {
              for (Wire &fragment : *existing_route) {
                fragment.net_name = net_name;
              }
              route_progress = route_progress || route_complete ||
                               existing_route->size() > before_size;
              --route_recursion_budget;
              continue;
            }
            --route_recursion_budget;
            return false;
          }

          std::vector<Wire> wire;
          route_iteration_budget = iteration_limit;
          if (routeNet(*peer, curr->port_ref->makeName(), inst,
                       conn.port_ref->makeName(), wire, route_complete)) {
            if (wire.empty()) {
              route_complete = false;
            }
            for (Wire &fragment : wire) {
              fragment.net_name = net_name;
            }
            if (existing_route && !routeIsComplete(*existing_route) &&
                !wire.empty()) {
              *existing_route = std::move(wire);
              route_progress = true;
            } else if (!wire.empty()) {
              inst.wires.emplace_back(std::move(wire));
              existing_route = &inst.wires.back();
              route_progress = true;
            }
            --route_recursion_budget;
          } else {
            PNR_LOG1("ROUT",
                     "warning: failed limited route attempt for net '{}' from "
                     "'{}' port '{}' to '{}' port '{}'",
                     net_name, peer->makeName(), curr->port_ref->makeName(),
                     inst.makeName(), conn.port_ref->makeName());
            --route_recursion_budget;
          }
        }
        if (!route_complete) {
          return false;
        }
        task_complete = true;
      }
    }
  }

  return task_complete;
}

// All terminal routing failures render the live mesh before exiting nonzero.
// Keep this outside search and speculation so the picture reflects real leases.
[[noreturn]] void RouteDesign::failRouting(
    const RouteTask *task, std::string_view reason,
    fpga::CBNodeNameType failure_node_type, int failure_node) {
  RouteTask fallback;
  if (!task) {
    fallback.net_name = "unknown_unfinished_route";
    task = &fallback;
  }
  if (failure_node < 0) {
    failure_node_type = task->failure_node_type;
    failure_node = task->failure_node;
  }
  PNR_LOG("ROUT", "routing failed: reason='{}', net='{}', from='{}'/'{}', to='{}'/'{}'",
          reason, task->net_name, instNameForDump(task->from), task->from_port,
          instNameForDump(task->to), task->to_port);
  if (fpga && fpga->size_width > 0 && fpga->size_height > 0) {
    fpga::Coord center = task->failure_coord;
    auto coord_valid = [&](fpga::Coord coord) {
      return coord.x >= 0 && coord.y >= 0 && coord.x < fpga->size_width &&
             coord.y < fpga->size_height;
    };
    if (!coord_valid(center)) {
      const std::vector<Wire> *route = findBoundRoute(
          task->net, task->from, task->to, task->from_port, task->to_port,
          task->net_name);
      if (route) {
        for (auto fragment = route->rbegin(); fragment != route->rend();
             ++fragment) {
          if (coord_valid(fragment->to)) {
            center = fragment->to;
            break;
          }
          if (coord_valid(fragment->from)) {
            center = fragment->from;
            break;
          }
        }
      }
    }
    if (!coord_valid(center) && task->to && task->to->tile.peer) {
      center = task->to->tile->coord;
    }
    if (!coord_valid(center) && task->from && task->from->tile.peer) {
      center = task->from->tile->coord;
    }
    if (!coord_valid(center)) {
      center = {0, 0};
    }
    // Print the live congestion state at exactly the PNG center, not only
    // the destination tile (which may be far from the failed search).
    if (Tile *tile = fpga->getTile(center.x, center.y)) {
      PNR_LOG("ROUT", "routing failure tile: net='{}' coord=({},{}) node_type={} "
               "node={} name='{}' sequence={} src={} dst={} joint={} local={} "
               "pin_leased={} src_deadend={}", task->net_name, center.x, center.y,
               static_cast<int>(failure_node_type), failure_node,
               failure_node >= 0 ? nodeNameForDump(*tile, failure_node_type, failure_node)
                                 : std::string{},
               task->failure_sequence, tile->cb.src.jump.str(), tile->cb.dst.jump.str(),
               tile->cb.joint.jump.str(), tile->cb.local.local.str(),
               tile->pin_state.leased_nodes.str(), tile->cb.src_deadend.jump.str());
      std::unordered_set<rtl::Net *> nets;
      if (tech) {
        for (rtl::Module &module : tech->design.modules)
          for (rtl::Net &net : module.nets) nets.insert(&net);
      } else {
        // Isolated routers lack a design registry; production audits must
        // preserve that registry's authority to expose stale index entries.
        for (const auto &[source, indexed] : source_route_nets)
          nets.insert(indexed.begin(), indexed.end());
        for (const auto &ref : tile->routedNets)
          if (ref.peer) nets.insert(ref.peer);
      }
      std::ostringstream audit;
      fpga::auditTileCongestion(*tile, audit,
                               std::vector<rtl::Net *>(nets.begin(), nets.end()));
      PNR_LOG("ROUT", "routing failure tile ownership:\n{}", audit.str());
    }
    const std::filesystem::path artifact_directory =
        failureArtifactDirectory();
    const std::filesystem::path image_path =
        artifact_directory / "routing_failure.png";
    const std::filesystem::path report_path =
        artifact_directory / "routing_failure.txt";
    const std::filesystem::path backward_path =
        artifact_directory / "routing_failure_backward.txt";
    const std::filesystem::path packing_path =
        artifact_directory / "routing_failure_packing.txt";
    bool packing_written = false;
    if (failure_packing_trace && failure_packing_net == task->net_name) {
      std::fflush(failure_packing_trace.get());
      std::rewind(failure_packing_trace.get());
      std::ofstream packing(packing_path, std::ios::binary | std::ios::trunc);
      PNR_LOG("ROUT", "routing failure candidate packing trace: {}", packing_path.string());
      char buffer[8192];
      size_t count;
      while ((count = std::fread(buffer, 1, sizeof(buffer), failure_packing_trace.get())) != 0) {
        packing.write(buffer, count);
        std::fwrite(buffer, 1, count, stdout);
      }
      packing.flush();
      packing_written = packing.good() && !std::ferror(failure_packing_trace.get());
    }
    bool backward_written = false;
    try {
      std::ofstream backward(backward_path, std::ios::trunc);
      if (!backward) throw std::runtime_error("cannot open backward report");
      backward << "net=" << task->net_name << '\n'
               << "from=" << instNameForDump(task->from) << '/' << task->from_port << '\n'
               << "to=" << instNameForDump(task->to) << '/' << task->to_port << '\n'
               << "stored_failure_coord=" << center.x << ',' << center.y << '\n';
      std::unordered_set<rtl::Net *> registry;
      if (tech) {
        for (rtl::Module &module : tech->design.modules)
          for (rtl::Net &net : module.nets) registry.insert(&net);
      } else {
        for (const auto &[source, indexed] : source_route_nets)
          registry.insert(indexed.begin(), indexed.end());
        for (int y = 0; y < fpga->size_height; ++y)
          for (int x = 0; x < fpga->size_width; ++x)
            if (Tile *tile = fpga->getTile(x, y))
              for (const auto &ref : tile->routedNets)
                if (ref.peer) registry.insert(ref.peer);
      }
      const std::vector<rtl::Net *> nets(registry.begin(), registry.end());
      if (task->to && task->to->tile.peer) {
        backward << "sink_resource_coord=" << task->to->tile->coord.x << ','
                 << task->to->tile->coord.y << '\n';
        for (Tile *target : routeTileCandidates(*task->to, task->to_port, false)) {
          if (!target || !target->cb_type) continue;
          auto &index = backwardDockingIndex(
              target->coord, std::max(fpga->size_width, fpga->size_height));
          dumpBackwardTerminalReport(
              *target, routeTileInputNodes(*target, *task->to, task->to_port),
              index, 5, nets, backward,
              fpga::packedInputJointReservations(*target, task->to, task->to_port));
        }
      } else {
        backward << "NO_PLACED_SINK\n";
      }
      backward.flush();
      backward_written = bool(backward);
      PNR_LOG("ROUT", "routing backward terminal report: written={} file='{}'",
              backward_written, backward_path.string());
    } catch (const std::exception &error) {
      PNR_LOG("ROUT", "routing backward terminal report failed: {}", error.what());
    }
    bool image_written = false;
    std::string image_error;
    fpga::Visualizer::Stats report_stats;
    try {
      dumpTargetCandidateSummaryLog(*task, 0);
      const std::vector<Wire> *diagnostic_route = nullptr;
      for (auto path = failure_visualization_paths.rbegin();
           path != failure_visualization_paths.rend(); ++path) {
        if (sameVisualizationRouteTask(*task, path->task) &&
            path->task.failure_sequence == task->failure_sequence &&
            !path->route.empty()) {
          diagnostic_route = &path->route;
          break;
        }
      }
      const bool has_retained_diagnostic = diagnostic_route != nullptr;
      const std::vector<Wire> *focused_route = diagnostic_route;
      if (!focused_route) {
        focused_route = findBoundRoute(
              task->net, task->from, task->to, task->from_port, task->to_port,
              task->net_name);
      }
      if (focused_route && focused_route->empty()) {
        focused_route = nullptr;
      }
      // Failed Fanout endpoint identity may be stale while its named physical
      // binding is still indexed and highlighted by drawRoutes().
      if (!focused_route && task->net) {
        for (const rtl::NetRouteBinding &binding : task->net->routes) {
          if (binding.route_name == task->net_name && binding.owner &&
              binding.route_index < binding.owner->wires.size()) {
            const std::vector<Wire> &bound_route =
                binding.owner->wires[binding.route_index];
            if (!bound_route.empty()) {
              focused_route = &bound_route;
              break;
            }
          }
        }
      }
      std::vector<Wire> terminal_diagnostic;
      if (!focused_route && task->to && task->to->tile.peer) {
        bool built = false;
        for (fpga::Tile *candidate :
             routeTileCandidates(*task->to, task->to_port, false)) {
          if (!candidate || !candidate->cb_type) {
            continue;
          }
          NodeMask pin_nodes =
              routeTileInputNodes(*candidate, *task->to, task->to_port);
          pin_nodes.for_each_set_bit([&](int pin) {
            int selected_dst = -1;
            TerminalEntryCandidate selected_entry;
            candidate->cb_type->ensureDerivedMasks();
            candidate->cb_type->dsts_reaching_local[pin].jump.for_each_set_bit(
                [&](int dst) {
                  std::vector<TerminalEntryCandidate> entries =
                      targetEntryCandidates(*candidate->cb_type, dst, pin);
                  if (entries.empty()) {
                    return false;
                  }
                  selected_dst = dst;
                  selected_entry = entries.front();
                  return true;
                });
            if (selected_dst < 0) {
              return false;
            }
            Wire enter;
            enter.from = candidate->coord;
            enter.to = candidate->coord;
            enter.local = selected_dst;
            enter.joint = selected_entry.joint;
            enter.joint2 = selected_entry.joint2;
            enter.pos = ROUTE_POS_TRANSIT;
            enter.net_name = task->net_name;
            if (const std::string *name = candidate->cb_type->nodeName(
                    fpga::CB_NODE_DST, selected_dst)) {
              enter.dst_wire_name = *name;
            }
            Wire tile_pin;
            tile_pin.type = Wire::WIRE_TILE_PIN;
            tile_pin.from = candidate->coord;
            tile_pin.to = candidate->coord;
            tile_pin.local = pin;
            tile_pin.pos = ROUTE_POS_TRANSIT;
            tile_pin.net_name = task->net_name;
            terminal_diagnostic = {std::move(enter), std::move(tile_pin)};
            built = true;
            return true;
          });
          if (built) {
            break;
          }
        }
        if (!terminal_diagnostic.empty()) {
          focused_route = &terminal_diagnostic;
        }
      }
      const bool has_terminal_diagnostic = !terminal_diagnostic.empty();
      bool draw_unindexed_route = focused_route != nullptr;
      PNR_LOG1("ROUT",
               "routeDesign VISUALIZATION route selection: retained={}, "
               "terminal={}, fragments={}",
               has_retained_diagnostic, has_terminal_diagnostic,
               focused_route ? focused_route->size() : 0);
      fpga::Visualizer visualization(*fpga);
      visualization.drawCB(center.x, center.y);
      visualization.drawRoutes(task->net_name);
      if (focused_route && draw_unindexed_route) {
        visualization.drawRoute(*focused_route, true, task->net_name);
      }
      if (focused_route) {
        PNR_LOG1("ROUT", "routeDesign VISUALIZATION highlighted path: {}",
                 visualizationRouteTrace(*fpga, *focused_route));
      }
      if (visualization.stats().highlighted_labels == 0 &&
          task->to && task->to->tile.peer) {
        // A task without a trunk has no honest route line to draw, so mark its
        // first numeric destination pin instead of highlighting another net.
        for (fpga::Tile *candidate :
             routeTileCandidates(*task->to, task->to_port, false)) {
          if (!candidate) {
            continue;
          }
          NodeMask pin_nodes =
              routeTileInputNodes(*candidate, *task->to, task->to_port);
          bool marked = pin_nodes.for_each_set_bit([&](int pin) {
            std::string pin_name =
                nodeNameForDump(*candidate, fpga::CB_NODE_LOCAL, pin);
            if (!visualization.highlightNode(
                    candidate->coord, fpga::CB_NODE_LOCAL, pin,
                    task->net_name + " -> " + pin_name)) {
              return false;
            }
            PNR_LOG1("ROUT",
                     "routeDesign VISUALIZATION unresolved target: "
                     "tile=({},{}) local={} '{}'",
                     candidate->coord.x, candidate->coord.y, pin, pin_name);
            return true;
          });
          if (marked) {
            break;
          }
        }
      }
      if (failure_node >= 0) {
        visualization.highlightNode(center, failure_node_type, failure_node,
                                    task->net_name);
      }
      visualization.drawFailureLabel(task->net_name);
      visualization.writePNG(image_path.string());
      const fpga::Visualizer::Stats &visual_stats = visualization.stats();
      report_stats = visual_stats;
      image_written = true;
      PNR_LOG("ROUT",
               "routeDesign VISUALIZATION: reason='{}', net='{}', "
               "center=({},{}), tiles={}, nodes={} "
               "(local={},joint={},src={},dst={}), occupied={}, routes={}, "
               "segments={}, highlighted_labels={}, highlighted_nodes={}, "
               "file='{}'",
               reason, task->net_name, center.x, center.y,
               visual_stats.tiles, visual_stats.nodes,
               visual_stats.local_nodes, visual_stats.joint_nodes,
               visual_stats.src_nodes, visual_stats.dst_nodes,
               visual_stats.occupied_nodes, visual_stats.routes,
               visual_stats.route_segments, visual_stats.highlighted_labels,
               visual_stats.highlighted_nodes, image_path.string());

    } catch (const std::exception &error) {
      image_error = error.what();
      PNR_LOG("ROUT",
               "routeDesign VISUALIZATION failed: reason='{}', net='{}', "
               "center=({},{}), error='{}'",
               reason, task->net_name, center.x, center.y, error.what());
    }
    std::ofstream report(report_path, std::ios::trunc);
    if (report) {
      report << "routing_status=failed\n"
             << "reason=" << reason << '\n'
             << "stage=" << route_progress_stage_name << '\n'
             << "net=" << task->net_name << '\n'
             << "from_inst=" << instNameForDump(task->from) << '\n'
             << "from_port=" << task->from_port << '\n'
             << "to_inst=" << instNameForDump(task->to) << '\n'
             << "to_port=" << task->to_port << '\n'
             << "failure_coord=" << center.x << ',' << center.y << '\n'
             << "failure_node_type=" << static_cast<int>(failure_node_type)
             << '\n'
             << "failure_node=" << failure_node << '\n'
             << "backward_report=" << backward_path.string() << '\n'
             << "backward_report_written=" << (backward_written ? "true" : "false") << '\n'
             << "packing_report=" << packing_path.string() << '\n'
             << "packing_report_written=" << (packing_written ? "true" : "false") << '\n'
             << "candidate_replay_directory="
             << (failure_packing_net == task->net_name ? failure_candidate_directory : std::string{}) << '\n'
             << "image=" << image_path.string() << '\n'
             << "image_written=" << (image_written ? "true" : "false")
             << '\n';
      if (!image_error.empty()) {
        report << "image_error=" << image_error << '\n';
      }
      report << "tiles=" << report_stats.tiles << '\n'
             << "nodes=" << report_stats.nodes << '\n'
             << "occupied_nodes=" << report_stats.occupied_nodes << '\n'
             << "routes=" << report_stats.routes << '\n'
             << "route_segments=" << report_stats.route_segments << '\n'
             << "highlighted_labels=" << report_stats.highlighted_labels
             << '\n'
             << "highlighted_nodes=" << report_stats.highlighted_nodes
             << '\n';
      report.flush();
      PNR_LOG("ROUT", "routeDesign failure report: file='{}'",
               report_path.string());
    } else {
      PNR_LOG("ROUT", "routeDesign failure report write failed: file='{}'",
               report_path.string());
    }
  }
  std::fflush(stdout);
  std::fflush(stderr);
  std::exit(EXIT_FAILURE);
}

void RouteDesign::routeDesign(std::list<Referable<RegBunch>> &bunch_list) {
  clearFailureArtifacts();
  failure_sequence = 0;
  failure_visualization_paths.clear();
  int total_bunches = 0;
  int total_regs = 0;
  int total_comb = 0;

  for (auto &bunch : bunch_list) {
    total_bunches += bunch.size;
    total_regs += bunch.size_regs;
    total_comb += bunch.size_comb; // need size of CARRY, MUX, SRL?   // then
                                   // think about BRAM, LRAM, DSP
  }
  int design_cells = countDesignCells(bunch_list);
  if (design_cells <= 0) {
    design_cells = std::max(total_bunches, total_regs + total_comb);
  }
  iteration_limit = iterationLimitFromCells(design_cells);
  move_attempt_limit = moveAttemptLimitFromCells(design_cells);
  PNR_LOG1(
      "ROUT",
      "routeDesign, cells: {}, iteration_limit: {}, move_attempt_limit: {}",
      design_cells, iteration_limit, move_attempt_limit);
  //    combs_per_box = /*total_comb*/(float)fpga.cnt_luts /
  //    (mesh_width*mesh_height);

  fpga_width = fpga->size_width;
  fpga_height = fpga->size_height;

  // Stabilize Net addresses before any stage stores route bindings or tile
  // ownership references; Clock and Const now run before ordinary routing.
  if (tech && tech->design.top.cell_ref.peer &&
      tech->design.top.cell_ref->module_ref.peer) {
    auto &nets = tech->design.top.cell_ref->module_ref->nets;
    nets.reserve(nets.size() +
                 static_cast<size_t>(std::max(1024, design_cells * 8)));
  }

  resetRoutingState();
  resetModuleNetDesignatorIndex();
  route_deadends_enabled = true;
  clock_routes_dirty = false;
  route_src_deadends.clear();
  docking_indexes.clear();
  docking_resolved_arcs = {};
  docking_index_replacement = 0;

  // Dedicated clocks reserve their numeric resources before constants and
  // ordinary routing; later stages treat these protected leases as fixed.
  RouteClocks::Stats clock_stage_stats;
  double clock_stage_seconds = 0.0;
  if (tech) {
    PNR_LOG1("ROUT", "routeDesign stage: Clock routing");
    auto clock_start = std::chrono::steady_clock::now();
    RouteClocks clock_router(*tech, *fpga);
    bool clocks_routed = clock_router.routeDesign(tech->clocks);
    clock_stage_seconds = elapsedSeconds(clock_start,
                                         std::chrono::steady_clock::now());
    clock_stage_stats = clock_router.stats();
    if (!clocks_routed) {
      const RouteClocks::Failure &failure = clock_router.failure();
      RouteTask failed;
      failed.net_name = failure.net_name;
      failed.failure_coord = {failure.x, failure.y};
      failRouting(&failed, "Clock routing",
                  static_cast<fpga::CBNodeNameType>(failure.node_type),
                  failure.node);
    }
    PNR_LOG1("ROUT",
             "routeDesign stage complete: Clock routing, sinks={}, "
             "routed={}, time={:.3f}s",
             clock_stage_stats.sinks, clock_stage_stats.routed,
             clock_stage_seconds);
  }

  aspect_x = (float)fpga_width / mesh_width;
  aspect_y = (float)fpga_height / mesh_height;

  int route_recursion_limit = 5;
  route_suffix_depth_limit = 5;
  int max_route_passes = std::max(1, design_cells * 20);
  route_todo.clear();
  pending_route_todo.clear();
  fanout_route_todo.clear();
  moving_destination_todo.clear();
  moving_deferred_todo.clear();
  resetPassPreemptionState();
  fanout_stage = false;
  fanout_preemption_enabled = true;
  moving_sources_stage = false;
  moving_stage = false;
  moving_focus_inst = nullptr;
  move_tried_placements.clear();
  moving_source_probe_offsets.clear();
  move_failed_scans.clear();
  move_finished_insts.clear();
  source_route_marks.clear();
  source_route_nets.clear();
  source_endpoint_retargets.clear();
  std::vector<RouteTask> distributed_route_tasks;
  if (tech) {
    RouteVCC vcc(*tech, *fpga);
    auto append_distributed = [&](RouteVCC::PreparedRoutes prepared, bool one) {
      if (!prepared) {
        return;
      }
      for (const RouteVCC::Sink &sink : prepared.sinks) {
        RouteTask task;
        task.from = prepared.source;
        task.to = sink.inst;
        task.net = prepared.net;
        task.from_port = "O";
        task.to_port = sink.port;
        task.net_name = sink.route_name;
        task.fanout = false;
        task.distributed_one = one;
        distributed_route_tasks.push_back(std::move(task));
      }
    };
    append_distributed(vcc.prepareDesign(), true);
    append_distributed(vcc.prepareGroundDesign(), false);
  }
  travers_mark = rtl::Inst::genMark();
  for (auto &bunch : bunch_list) {
    PNR_ASSERT(bunch.reg, "zero reg in bunch with address {}",
               (uint64_t)&bunch);
    collectRouteTasks(*bunch.reg, &bunch);
  }
  size_t distributed_tasks = distributed_route_tasks.size();
  // Constants remain in their dedicated queue. They have one physical root
  // per target crossbar and therefore are neither trunks nor fanout suffixes.
  size_t nearest_seed_replacements = pnr::selectNearestGenericSeeds(
      route_todo, fanout_route_todo,
      [](const RouteTask &task) {
        return sourceRouteKey(task.from, task.from_port);
      },
      [](const RouteTask &task) {
        if (!task.from || !task.to || !task.from->tile.peer ||
            !task.to->tile.peer) {
          return std::numeric_limits<int>::max();
        }
        return std::abs(task.from->tile->coord.x - task.to->tile->coord.x) +
               std::abs(task.from->tile->coord.y - task.to->tile->coord.y);
      });
  PNR_LOG1("ROUT",
           "routeDesign tasks: basic={}, constants={}, "
           "deferred_fanout={}, nearest_seed_replacements={}",
           route_todo.size(), distributed_tasks, fanout_route_todo.size(),
           nearest_seed_replacements);

  auto route_start_time = std::chrono::steady_clock::now();
  double default_route_stage_timeout_seconds = 600.0;
  if (const char *timeout = std::getenv("SCALEPNR_ROUTE_STAGE_TIMEOUT")) {
    char *end = nullptr;
    double requested = std::strtod(timeout, &end);
    if (end != timeout && requested > 0.0) {
      default_route_stage_timeout_seconds = requested;
    }
  }
  std::array<double, ROUTE_STAGE_COUNT> route_stage_timeout_seconds{
      default_route_stage_timeout_seconds, default_route_stage_timeout_seconds,
      default_route_stage_timeout_seconds, default_route_stage_timeout_seconds,
      default_route_stage_timeout_seconds, default_route_stage_timeout_seconds};
  auto read_stage_timeout = [&](size_t stage, const char *name) {
    if (const char *timeout = std::getenv(name)) {
      char *end = nullptr;
      double requested = std::strtod(timeout, &end);
      if (end != timeout && requested > 0.0) {
        route_stage_timeout_seconds[stage] = requested;
      }
    }
  };
  read_stage_timeout(CLOCK_STAGE_INDEX, "SCALEPNR_ROUTE_CLOCK_TIMEOUT");
  read_stage_timeout(CONSTANT_STAGE_INDEX, "SCALEPNR_ROUTE_CONST_TIMEOUT");
  read_stage_timeout(BASIC_STAGE_INDEX, "SCALEPNR_ROUTE_BASIC_TIMEOUT");
  read_stage_timeout(MOVING_SOURCES_STAGE_INDEX,
                     "SCALEPNR_ROUTE_MOVING_SOURCE_TIMEOUT");
  read_stage_timeout(FANOUT_STAGE_INDEX, "SCALEPNR_ROUTE_FANOUT_TIMEOUT");
  read_stage_timeout(MOVING_DESTINATIONS_STAGE_INDEX,
                     "SCALEPNR_ROUTE_MOVING_TIMEOUT");
  read_stage_timeout(MOVING_DESTINATIONS_STAGE_INDEX,
                     "SCALEPNR_ROUTE_MOVING_DESTINATION_TIMEOUT");
  int route_progress_window_seconds = 60;
  unsigned route_stagnant_window_limit =
      RouteProgressWatchdog::default_stagnant_windows;
  if (const char *window = std::getenv("SCALEPNR_ROUTE_PROGRESS_WINDOW")) {
    char *end = nullptr;
    long requested = std::strtol(window, &end, 10);
    if (end != window && requested > 0) {
      route_progress_window_seconds = static_cast<int>(requested);
    }
  }
  if (const char *windows =
          std::getenv("SCALEPNR_ROUTE_STAGNANT_WINDOWS")) {
    char *end = nullptr;
    unsigned long requested = std::strtoul(windows, &end, 10);
    if (end != windows && requested > 0) {
      route_stagnant_window_limit = static_cast<unsigned>(requested);
    }
  }
  std::array<RouteStageReport, ROUTE_STAGE_COUNT> stage_reports{};
  std::array<std::chrono::steady_clock::time_point, ROUTE_STAGE_COUNT>
      stage_wall_starts{};
  RouteStageReport &clock_report = stage_reports[CLOCK_STAGE_INDEX];
  clock_report.started = true;
  clock_report.start_tasks = clock_stage_stats.sinks;
  clock_report.remaining_tasks = clock_stage_stats.failed;
  clock_report.attempted = clock_stage_stats.sinks;
  clock_report.completed = clock_stage_stats.routed;
  clock_report.active = clock_stage_stats.routed;
  clock_report.searches = clock_stage_stats.nets;
  clock_report.pops = clock_stage_stats.graph_nodes;
  clock_report.seconds = clock_stage_seconds;
  route_stage_deadline_enabled = false;
  route_stage_deadline_expired = false;
  bool heartbeat_enabled = routeHeartbeatEnabled();
  int heartbeat_seconds = routeHeartbeatSeconds();
  auto last_heartbeat_time = route_start_time;
  int stagnant_passes = 0;
  int moving_passes = 0;
  int moving_placement_passes = 0;
  bool moving_placement_has_completion = false;
  int moving_relocation_epoch = 0;
  std::unordered_map<uintptr_t, int> moving_blocked_until_epoch;
  int stage_pass = 0;
  size_t fanout_stagnant_attempts = 0;
  int basic_no_net_progress_passes = 0;
  size_t basic_best_remaining = route_todo.size();
  size_t basic_growth_passes = 0;
  int fanout_no_completion_passes = 0;
  bool fanout_seed_repair_active = false;
  bool moving_sources_completed = false;
  std::vector<RouteTask> moving_source_retry_todo;
  bool moving_relocate_next = false;
  bool moving_force_incident_relocation = false;
  size_t moving_incident_recoveries = 0;
  size_t moving_source_prefix_releases_since_generic = 0;
  size_t moving_source_moves_since_generic = 0;
  size_t moving_source_changed_routes_since_generic = 0;
  bool moving_source_generic_recovery_pending = false;
  int moving_no_completion_passes = 0;
  bool basic_rest_diagnostics_dumped = false;
  const int moving_no_candidate_block_epochs = 1;
  rtl::Inst *cached_moving_focus = nullptr;
  std::unordered_set<rtl::Inst *> cached_moving_endpoints;
  auto endpoint_matches_moving_focus = [&](rtl::Inst *endpoint) {
    if (!moving_focus_inst) {
      return false;
    }
    if (cached_moving_focus != moving_focus_inst) {
      cached_moving_focus = moving_focus_inst;
      cached_moving_endpoints = movingFocusEndpointClosure(moving_focus_inst);
    }
    return endpoint && cached_moving_endpoints.contains(endpoint);
  };
  auto task_matches_moving_focus = [&](const RouteTask &task) {
    if (moving_sources_stage) {
      return endpoint_matches_moving_focus(task.from);
    }
    return endpoint_matches_moving_focus(task.from) ||
           endpoint_matches_moving_focus(task.to);
  };
  const size_t moving_focus_retry_limit =
      pnr::movingCandidateRetryLimit(move_attempt_limit);
  // Keep each moved cluster atomic during one bounded placement slice. A yield
  // conserves the complete incident queue and its candidate history together.
  constexpr size_t moving_focus_slice_limit = pnr::MOVING_FOCUS_SLICE_LIMIT;
  constexpr size_t moving_history_tail_threshold = 32;
  size_t moving_focus_slice_start = 0;
  int moving_last_cooldown_clear_epoch = -1;
  int moving_cooldown_clears_without_move = 0;
  auto first_unfinished_task = [&]() -> const RouteTask * {
    return pnr::latestUnfinishedRouteTask<RouteTask>(
        {&route_todo, &moving_deferred_todo, &moving_source_retry_todo,
         &moving_destination_todo, &fanout_route_todo, &pending_route_todo},
        [&](const RouteTask &task) {
          const std::vector<Wire> *route = findBoundRoute(
              task.net, task.from, task.to, task.from_port, task.to_port,
              task.net_name);
          return route && routeIsComplete(*route);
        });
  };
  auto stage_remaining_tasks = [&](size_t stage_index) {
    auto live_tasks = [](const std::vector<RouteTask> &tasks) {
      return static_cast<size_t>(std::count_if(
          tasks.begin(), tasks.end(),
          [](const RouteTask &task) { return !task.remove_after_pass; }));
    };
    size_t remaining = live_tasks(route_todo) + live_tasks(pending_route_todo);
    if (stage_index == FANOUT_STAGE_INDEX) {
      remaining += live_tasks(fanout_route_todo);
    } else if (stage_index == MOVING_SOURCES_STAGE_INDEX) {
      remaining += live_tasks(moving_deferred_todo) +
                   live_tasks(moving_source_retry_todo);
    } else if (stage_index == MOVING_DESTINATIONS_STAGE_INDEX) {
      remaining += live_tasks(fanout_route_todo) +
                   live_tasks(moving_destination_todo) +
                   live_tasks(moving_deferred_todo);
    }
    return remaining;
  };
  auto print_stage_report = [&](const char *reason) {
    PNR_LOG1("ROUT",
             "routeDesign stage report: reason={}, "
             "budgets=({:.1f}s,{:.1f}s,{:.1f}s,{:.1f}s,{:.1f}s,{:.1f}s), "
             "progress_window={}s, min_progress=1%, stagnant_windows={}",
             reason, route_stage_timeout_seconds[CLOCK_STAGE_INDEX],
             route_stage_timeout_seconds[CONSTANT_STAGE_INDEX],
             route_stage_timeout_seconds[BASIC_STAGE_INDEX],
             route_stage_timeout_seconds[MOVING_SOURCES_STAGE_INDEX],
             route_stage_timeout_seconds[FANOUT_STAGE_INDEX],
             route_stage_timeout_seconds[MOVING_DESTINATIONS_STAGE_INDEX],
             route_progress_window_seconds, route_stagnant_window_limit);
    for (size_t index = 0; index < stage_reports.size(); ++index) {
      const RouteStageReport &report = stage_reports[index];
      double task_rate =
          report.seconds > 0.0
              ? static_cast<double>(report.attempted) / report.seconds
              : 0.0;
      double edge_rate =
          report.seconds > 0.0
              ? static_cast<double>(report.edge_trials) / report.seconds
              : 0.0;
      PNR_LOG1(
          "ROUT",
          "routeDesign stage report: stage={}, started={}, timeout={}, "
          "stagnated={}, "
          "time={:.3f}s, passes={}, tasks={}->{}, attempted={}, completed={}, "
          "active={}, advanced={}, changed={}, task_rate={:.1f}/s, "
          "searches={}, pops={}, edge_trials={}, edge_ok={}, "
          "edge_rate={:.1f}/s, reject(busy={},target={},deadend={}), "
          "preempt={}/{}, preempt_victims(complete={},partial={},fragments={}), "
          "preempt_by_kind(takeoff={}/{},bridge={}/{},grounding={}/{}), "
          "deadend_marks={}",
          routeStageName(index), report.started, report.timed_out,
          report.stagnated,
          report.seconds, report.passes, report.start_tasks,
          report.remaining_tasks, report.attempted, report.completed,
          report.active, report.advanced, report.changed, task_rate,
          report.searches, report.pops, report.edge_trials,
          report.edge_accepted, edge_rate, report.reject_busy,
          report.reject_target, report.reject_deadend, report.preempt_success,
          report.preempt_attempts, report.preempt_complete_victims,
          report.preempt_partial_victims, report.preempt_removed_fragments,
          report.preempt_takeoff_complete_victims,
          report.preempt_takeoff_partial_victims,
          report.preempt_bridge_complete_victims,
          report.preempt_bridge_partial_victims,
          report.preempt_grounding_complete_victims,
          report.preempt_grounding_partial_victims,
          report.deadend_marks);
    }
  };
  auto fail_stage = [&](size_t stage_index, const char *failure_kind,
                        bool timed_out, bool stagnated) {
    route_stage_deadline_enabled = false;
    route_stage_deadline_expired = timed_out;
    route_stage_stagnated = stagnated;
    RouteStageReport &report = stage_reports[stage_index];
    report.timed_out = timed_out;
    report.stagnated = stagnated;
    report.remaining_tasks = stage_remaining_tasks(stage_index);
    std::string timeout_name =
        stage_index == CLOCK_STAGE_INDEX
            ? "clock"
            : (stage_index == CONSTANT_STAGE_INDEX
                   ? "constant"
                   : (stage_index == BASIC_STAGE_INDEX
                          ? "basic"
                          : (stage_index == MOVING_SOURCES_STAGE_INDEX
                                 ? "moving_sources"
                                 : (stage_index == FANOUT_STAGE_INDEX
                                        ? "fanout"
                                        : "moving_destinations"))));
    std::string timeout_dump = std::format(
        "/tmp/scalepnr_{}_{}_state.txt", timeout_name, failure_kind);
    // Full state dumps can exceed the design itself by gigabytes; emit one
    // only for an explicitly requested offline diagnostic run.
    if (envFlagEnabled("SCALEPNR_WRITE_TIMEOUT_DUMP") &&
        !envFlagEnabled("SCALEPNR_SKIP_TIMEOUT_DUMP")) {
      dumpFullRoutingState(timeout_dump, route_todo, fanout_route_todo,
                           pending_route_todo, moving_deferred_todo);
    } else {
      timeout_dump = "disabled unless SCALEPNR_WRITE_TIMEOUT_DUMP=1";
    }
    auto dump_timeout_queue = [&](const std::vector<RouteTask> &tasks,
                                  const char *queue_name) {
      if (tasks.empty()) {
        return;
      }
      std::string reason =
          std::format("{} {}: {}", routeStageName(stage_index), failure_kind,
                      queue_name);
      dumpBasicUnfinishedDiagnosticsLog(tasks, stage_pass, reason.c_str(),
                                        false);
    };
    dump_timeout_queue(route_todo, "active");
    dump_timeout_queue(pending_route_todo, "pending");
    dump_timeout_queue(fanout_route_todo, "fanout");
    dump_timeout_queue(moving_destination_todo, "moving-destination");
    dump_timeout_queue(moving_deferred_todo, "moving-deferred");
    dump_timeout_queue(moving_source_retry_todo, "moving-source-retry");
    print_stage_report(failure_kind);
    const RouteTask *first_task = first_unfinished_task();
    // Retained search paths may illustrate this live task, but must never
    // replace its identity with an old failure that has since been routed.
    failRouting(first_task,
                std::format("{} {}", routeStageName(stage_index),
                            failure_kind));
    PNR_ASSERT(false,
               "routeDesign {} {} after {:.1f}s with {} unfinished route "
               "tasks; state dumped to '{}'; first unfinished net='{}' "
               "from='{}'/'{}' to='{}'/'{}'",
               routeStageName(stage_index), failure_kind, report.seconds,
               report.remaining_tasks, timeout_dump,
               first_task ? first_task->net_name : std::string{},
               first_task ? instNameForDump(first_task->from) : std::string{},
               first_task ? first_task->from_port : std::string{},
               first_task ? instNameForDump(first_task->to) : std::string{},
               first_task ? first_task->to_port : std::string{});
  };
  auto fail_stage_timeout = [&](size_t stage_index) {
    fail_stage(stage_index, "timeout", true, false);
  };
  auto fail_stage_stagnation = [&](size_t stage_index) {
    fail_stage(stage_index, "stagnation", false, true);
  };
  auto moving_cooldown_epochs = [&](rtl::Inst *inst) {
    if (!inst) {
      return moving_no_candidate_block_epochs;
    }
    auto tried_it = move_tried_placements.find(movingClusterKey(inst));
    size_t tried_count =
        tried_it == move_tried_placements.end() ? 0 : tried_it->second.size();
    return std::max(moving_no_candidate_block_epochs,
                    2 + static_cast<int>(tried_count / 4));
  };

  auto drop_complete_tasks = [&](std::vector<RouteTask> &tasks,
                                 const char *queue_name) {
    if (!tech || tasks.empty()) {
      return size_t{0};
    }
    size_t dropped = pnr::removeCompletedMovingTasks(
        tasks, [&](const RouteTask &task) {
      const std::vector<Wire> *route =
          findBoundRoute(task.net, task.from, task.to, task.from_port,
                         task.to_port, task.net_name);
          return route && routeIsComplete(*route);
        });
    deduplicateRouteTasks(tasks);
    if (dropped != 0) {
      PNR_LOG1("ROUT",
               "routeDesign moving: dropped {} already-complete tasks from {}",
               dropped, queue_name);
    }
    return dropped;
  };

  auto restore_moving_deferred_tasks = [&]() {
    auto log_restore_task = [&](const char *queue, size_t index,
                                const RouteTask &task) {
      PNR_LOG1(
          "ROUT",
          "routeDesign moving restore {}[{}]: net='{}', from='{}' type='{}' "
          "port='{}' tile=({},{})/{}, to='{}' type='{}' port='{}' "
          "tile=({},{})/{}, fanout={}, attempt={}",
          queue, index, task.net_name,
          task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
          task.from && task.from->cell_ref.peer ? task.from->cell_ref->type
                                                : std::string{},
          task.from_port,
          task.from && task.from->tile.peer ? task.from->tile->coord.x : -1,
          task.from && task.from->tile.peer ? task.from->tile->coord.y : -1,
          task.from ? task.from->pos : -1,
          task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
          task.to && task.to->cell_ref.peer ? task.to->cell_ref->type
                                            : std::string{},
          task.to_port,
          task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
          task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
          task.to ? task.to->pos : -1, task.fanout, task.attempt);
    };
    if (route_todo.size() <= 4 && moving_deferred_todo.size() <= 4) {
      for (size_t i = 0; i < route_todo.size(); ++i) {
        log_restore_task("active", i, route_todo[i]);
      }
      for (size_t i = 0; i < moving_deferred_todo.size(); ++i) {
        log_restore_task("deferred", i, moving_deferred_todo[i]);
      }
    }
    if (moving_sources_stage) {
      pnr::deferMovingSourceRetry(moving_source_retry_todo, route_todo);
      drop_complete_tasks(moving_source_retry_todo,
                          "Moving source retry queue");
    } else {
      moving_deferred_todo.insert(
          moving_deferred_todo.end(),
          std::make_move_iterator(route_todo.begin()),
          std::make_move_iterator(route_todo.end()));
      route_todo.clear();
      drop_complete_tasks(moving_deferred_todo, "Moving deferred queue");
    }
    // Source retries wait for the next fair cycle. Destination work retains
    // the established persistent deferred-pool behavior.
  };

  auto moving_focus_complete = [&](rtl::Inst &inst) {
    return moving_sources_stage ? movingSourceTrunksComplete(inst)
                                : allIncidentRoutesComplete(inst);
  };

  auto collect_moving_focus_tasks = [&](rtl::Inst &inst,
                                        std::vector<RouteTask> &tasks) {
    if (!moving_sources_stage) {
      return collectIncompleteIncidentRouteTasks(inst, tasks);
    }
    return collectMovingSourceTasks(inst, tasks, fanout_route_todo);
  };

  auto defer_exhausted_moving_focus = [&](const char *context) {
    if (!moving_focus_inst) {
      return false;
    }
    uintptr_t focus_key = movingClusterKey(moving_focus_inst);
    std::vector<uint64_t> &focus_placements = move_tried_placements[focus_key];
    size_t focus_tried = focus_placements.size();
    bool slice_exhausted = pnr::movingFocusSliceExhausted(
        focus_tried, moving_focus_slice_start, moving_focus_slice_limit);
    bool cycle_exhausted = pnr::movingFocusPlacementsExhausted(
        focus_tried, moving_focus_retry_limit);
    if (!slice_exhausted && !cycle_exhausted) {
      return false;
    }
    if (cycle_exhausted) {
      PNR_ASSERT(pnr::restartMovingPlacementCycle(focus_placements,
                                                  moving_focus_retry_limit),
                 "failed to restart exhausted Moving placement cycle for '{}'",
                 moving_focus_inst->makeName(FULL_NAME_LIMIT));
      moving_focus_slice_start = 0;
      moving_relocate_next = true;
      PNR_LOG1("ROUT",
               "routeDesign moving: restarting atomic focus inst='{}' after "
               "full placement cycle={}/{} at {}, active={}, deferred={}",
               moving_focus_inst->makeName(FULL_NAME_LIMIT), focus_tried,
               moving_focus_retry_limit, context, route_todo.size(),
               moving_deferred_todo.size());
      return true;
    }
    int cooldown = moving_cooldown_epochs(moving_focus_inst);
    size_t unfinished_tasks = route_todo.size() + moving_deferred_todo.size() +
                              moving_source_retry_todo.size();
    bool retain_history = pnr::retainMovingPlacementHistory(
        unfinished_tasks, moving_history_tail_threshold, cycle_exhausted);
    if (!retain_history) {
      focus_placements.clear();
    }
    PNR_LOG1("ROUT",
             "routeDesign moving: yielding focus inst='{}' after slice={}/{} "
             "total={}/{} at {}, active={}, deferred={}, cooldown={}, "
             "retain_history={}, cycle_restart={}",
             moving_focus_inst->makeName(FULL_NAME_LIMIT),
             focus_tried - moving_focus_slice_start, moving_focus_slice_limit,
             focus_tried, moving_focus_retry_limit, context, route_todo.size(),
             moving_deferred_todo.size(), cooldown, retain_history,
             cycle_exhausted);
    moving_blocked_until_epoch[focus_key] = moving_relocation_epoch + cooldown;
    restore_moving_deferred_tasks();
    moving_focus_inst = nullptr;
    moving_focus_slice_start = 0;
    stagnant_passes = 0;
    moving_passes = 0;
    moving_no_completion_passes = 0;
    // The yielded focus already ran its isolated recovery slice. Select the
    // next deferred endpoint without globally rescanning every restored task.
    moving_relocate_next =
        pnr::movingRelocatesImmediatelyAfterFocus(moving_sources_stage);
    return true;
  };

  auto perform_moving_relocation = [&]() -> bool {
    ++moving_passes;
    ++moving_relocation_epoch;
    moving_no_completion_passes = 0;
    if (moving_focus_inst && envFlagEnabled("SCALEPNR_ROUTE_MOVE_DETAIL")) {
      for (size_t index = 0; index < route_todo.size(); ++index) {
        const RouteTask &task = route_todo[index];
        const std::vector<Wire> *route =
            findBoundRoute(task.net, task.from, task.to, task.from_port,
                           task.to_port, task.net_name);
        PNR_LOG1(
            "ROUT",
            "routeDesign moving blocker[{}]: focus='{}', net='{}', "
            "from='{}'/'{}', to='{}'/'{}', fanout={}, attempt={}, "
            "route_size={}, complete={}",
            index, moving_focus_inst->makeName(FULL_NAME_LIMIT), task.net_name,
            task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
            task.from_port,
            task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
            task.to_port, task.fanout, task.attempt, route ? route->size() : 0,
            route && routeIsComplete(*route));
      }
    }
    if (move_attempt_limit > 0 && moving_passes % move_attempt_limit == 0) {
      PNR_LOG1("ROUT",
               "routeDesign Moving progress: relocation_epochs={}, "
               "unfinished_tasks={}",
               moving_passes, route_todo.size());
    }
    bool moved = false;
    bool force_incident_relocation = moving_force_incident_relocation;
    moving_force_incident_relocation = false;
    if (force_incident_relocation) {
      PNR_LOG1("ROUT", "routeDesign moving: relocating repeated incomplete "
                       "incident set without identity-based task filtering");
    }
    if (moving_sources_stage && route_todo.empty() &&
        pnr::activateMovingSourceRetryCycle(moving_deferred_todo,
                                            moving_source_retry_todo)) {
      drop_complete_tasks(moving_deferred_todo,
                          "Moving source next-cycle queue");
      PNR_LOG1("ROUT",
               "routeDesign Moving sources: starting next fair retry cycle "
               "with {} trunks",
               moving_deferred_todo.size());
    }
    if (moving_sources_stage &&
        pnr::activateMovingSourceWorkset(route_todo,
                                         moving_deferred_todo)) {
      PNR_LOG1("ROUT",
               "routeDesign Moving sources: activated complete retry workset "
               "with {} trunks",
               route_todo.size());
    }
    if (route_todo.empty() && moving_deferred_todo.empty()) {
      return true;
    }
    if (moving_focus_inst) {
      size_t focus_tried =
          move_tried_placements[movingClusterKey(moving_focus_inst)].size();
      if (pnr::movingFocusSliceExhausted(focus_tried, moving_focus_slice_start,
                                         moving_focus_slice_limit) ||
          pnr::movingFocusPlacementsExhausted(focus_tried,
                                              moving_focus_retry_limit)) {
        defer_exhausted_moving_focus("relocation entry");
        return false;
      }
    }
    auto is_movable_inst = [&](rtl::Inst *inst) {
      if (!inst || !inst->tile.peer || isIoBuffer(*inst) ||
          inst->outline.fixed) {
        return false;
      }
      // Check whether this candidate completed an earlier Moving subsequence.
      bool marked_finished = movingInstIsFinished(move_finished_insts, inst);
      // Route-tree invalidation can make a previously finished candidate
      // movable.
      if (marked_finished) {
        // Revalidate completion against all routes incident to the candidate.
        bool incident_complete = moving_focus_complete(*inst);
        // Reject only candidates whose completed routing is still intact.
        if (pnr::movingFinishedMarkIsValid(marked_finished,
                                           incident_complete)) {
          // Keep the successfully routed placement stable.
          return false;
        }
        // Remove stale cluster marks before returning the candidate as movable.
        unmarkMovingClusterFinished(move_finished_insts, inst);
      }
      return true;
    };
    auto source_needs_move = [&](const RouteTask &task) {
      return movingSourceNeedsRelocation(task);
    };
    auto relocation_target = [&](const RouteTask &task) {
      if (moving_sources_stage) {
        return pnr::movingSourcePlacementTarget(
            task.from, [](rtl::Inst *endpoint) -> rtl::Inst * {
              rtl::Inst *owner = nullptr;
              std::string owner_port;
              return endpoint && passthroughInputSourceEndpoint(
                                     *endpoint, owner, owner_port)
                         ? owner
                         : nullptr;
            });
      }
      rtl::Inst *sink = movingPlacementTarget(task.to);
      return pnr::movingDestinationPlacementTarget(
          moving_focus_inst, sink, endpoint_matches_moving_focus(task.from),
          endpoint_matches_moving_focus(task.to));
    };
    auto move_blocked = [&](rtl::Inst *inst) {
      if (!inst) {
        return false;
      }
      auto it = moving_blocked_until_epoch.find(movingClusterKey(inst));
      return it != moving_blocked_until_epoch.end() &&
             it->second > moving_relocation_epoch;
    };
    auto can_move_task = [&](const RouteTask &task) {
      if (moving_focus_inst) {
        if (!task_matches_moving_focus(task)) return false;
        rtl::Inst *target = moving_sources_stage ? moving_focus_inst
                                                : relocation_target(task);
        if (pnr::movingQueuedTaskInvalidatesFinishedMark(
                movingInstIsFinished(move_finished_insts, target),
                true)) {
          unmarkMovingClusterFinished(move_finished_insts, target);
        }
        return is_movable_inst(target) &&
               (target == moving_focus_inst || !move_blocked(target));
      }
      if (!moving_sources_stage && source_needs_move(task)) {
        NodeMask output_nodes =
            task.from && task.from->tile.peer && task.from->cell_ref.peer
                ? task.from->tile->getOutputPinNodes(
                      task.from->cell_ref->type, task.from_port, task.from->pos)
                : NodeMask{};
        failRouting(&task, "invalid driver relocation");
        PNR_ASSERT(
            false,
            "routeDesign moving tried to move driver inst='{}' type='{}' "
            "port='{}' at ({},{})/{} for net='{}' to sink='{}'/'{}'; driver "
            "takeoff must be solved in Generic routing, output_nodes={}",
            task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
            task.from && task.from->cell_ref.peer ? task.from->cell_ref->type
                                                  : std::string{},
            task.from_port,
            task.from && task.from->tile.peer ? task.from->tile->coord.x : -1,
            task.from && task.from->tile.peer ? task.from->tile->coord.y : -1,
            task.from ? task.from->pos : -1, task.net_name,
            task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
            task.to_port, maskString(output_nodes));
      }
      rtl::Inst *placement_target = relocation_target(task);
      if (pnr::movingQueuedTaskInvalidatesFinishedMark(
              movingInstIsFinished(move_finished_insts, placement_target),
              true)) {
        unmarkMovingClusterFinished(move_finished_insts, placement_target);
      }
      return is_movable_inst(placement_target) &&
             !move_blocked(placement_target);
    };
    if (!moving_focus_inst && route_todo.empty()) {
      // Focused diagnostics select directly from the persistent deferred pool;
      // otherwise the requested task may be hidden behind thousands of entries.
      if (routeDebugEnabled("SCALEPNR_DEBUG_MOVE_FOCUS")) {
        auto focused = std::find_if(
            moving_deferred_todo.begin(), moving_deferred_todo.end(),
            [&](const RouteTask &candidate) {
              rtl::Inst *target = relocation_target(candidate);
              return target &&
                     routeDebugMatches("SCALEPNR_DEBUG_MOVE_FOCUS",
                                       target->makeName(FULL_NAME_LIMIT));
            });
        if (focused != moving_deferred_todo.end()) {
          std::iter_swap(focused, std::prev(moving_deferred_todo.end()));
        }
      }
      std::vector<RouteTask> skipped;
      size_t skipped_cooldown = 0;
      size_t skipped_immovable = 0;
      while (!moving_deferred_todo.empty()) {
        RouteTask candidate = std::move(moving_deferred_todo.back());
        moving_deferred_todo.pop_back();
        const std::vector<Wire> *candidate_route = findBoundRoute(
            candidate.net, candidate.from, candidate.to, candidate.from_port,
            candidate.to_port, candidate.net_name);
        if (!pnr::movingTaskNeedsRelocation(
                candidate_route && routeIsComplete(*candidate_route))) {
          continue;
        }
        if (!can_move_task(candidate)) {
          rtl::Inst *candidate_target = relocation_target(candidate);
          if (move_blocked(candidate_target)) {
            ++skipped_cooldown;
          } else {
            ++skipped_immovable;
          }
          skipped.push_back(std::move(candidate));
          continue;
        }
        route_todo.push_back(std::move(candidate));
        break;
      }
      moving_deferred_todo.insert(
          moving_deferred_todo.end(),
          std::make_move_iterator(skipped.begin()),
          std::make_move_iterator(skipped.end()));
      if (route_todo.empty()) {
        if (moving_deferred_todo.empty()) {
          return true;
        }
        if (pnr::movingDeferredScanNeedsRetry(moving_deferred_todo.size(),
                                              skipped_cooldown)) {
          moving_relocate_next = true;
          PNR_LOG1("ROUT",
                   "routeDesign moving: deferred scan found {} cooling-down "
                   "and {} immovable tasks; advancing relocation epoch",
                   skipped_cooldown, skipped_immovable);
          return false;
        }
        failRouting(&moving_deferred_todo.front(),
                                    "no movable endpoint");
        PNR_ASSERT(false,
                   "routeDesign moving deferred scan found {} unfinished "
                   "tasks but no movable endpoint (immovable={})",
                   moving_deferred_todo.size(), skipped_immovable);
        return true;
      }
    }
    size_t move_attempted = 0;
    size_t move_failed = 0;
    size_t move_failure_printed = 0;
    if (moving_sources_stage && !moving_focus_inst && !route_todo.empty()) {
      // Keep prefixes committed by earlier Moving Sources passes. Each
      // selected relocation probes through a lease-free state view and tears
      // down only its own source tree after proving the replacement route.
      const size_t batch_limit = pnr::movingSourceRelocationBatchLimit(
          move_attempt_limit, route_todo.size());
      const size_t batch_attempt_limit =
          pnr::movingSourceRelocationAttemptLimit(batch_limit,
                                                  route_todo.size());
      std::vector<RouteTask> pending_sources = std::move(route_todo);
      std::vector<RouteTask> rebuilt_tasks;
      std::vector<RouteTask> rejected_sources;
      rebuilt_tasks.reserve(pending_sources.size());
      rejected_sources.reserve(batch_attempt_limit);

      std::unordered_set<uintptr_t> considered_sources;
      std::unordered_set<uintptr_t> relocated_sources;
      size_t relocated = 0;
      size_t released_prefixes = 0;
      size_t physical_source_moves = 0;
      size_t changed_routes = 0;
      bool batch_deadline_reached = false;

      // route_todo is temporarily held in this batch. Include unprocessed and
      // rebuilt work so cancellation callbacks observe actual queue progress.
      auto update_batch_progress = [&](size_t next_task) {
        updateRouteProgressRemaining(
            pending_sources.size() - next_task + rebuilt_tasks.size() +
            rejected_sources.size() +
            stage_remaining_tasks(MOVING_SOURCES_STAGE_INDEX));
      };
      for (size_t task_index = 0; task_index < pending_sources.size();
           ++task_index) {
        RouteTask &task = pending_sources[task_index];
        update_batch_progress(task_index);
        if (!batch_deadline_reached && routeStageSearchCancelled()) {
          batch_deadline_reached = true;
        }
        if (batch_deadline_reached) {
          rebuilt_tasks.push_back(std::move(task));
          continue;
        }
        const std::vector<Wire> *candidate_route = findBoundRoute(
            task.net, task.from, task.to, task.from_port, task.to_port,
            task.net_name);
        if (!pnr::movingSourceRouteGuidedTaskNeedsRelocation(
                candidate_route && routeIsComplete(*candidate_route))) {
          continue;
        }
        rtl::Inst *source = relocation_target(task);
        uintptr_t source_key = movingClusterKey(source);
        if (relocated_sources.contains(source_key)) {
          continue;
        }
        if (relocated >= batch_limit || move_attempted >= batch_attempt_limit) {
          rebuilt_tasks.push_back(std::move(task));
          continue;
        }
        if (!source || !considered_sources.insert(source_key).second ||
            !can_move_task(task)) {
          rejected_sources.push_back(std::move(task));
          continue;
        }

        ++move_attempted;
        ++stage_reports[MOVING_SOURCES_STAGE_INDEX].attempted;
        std::vector<RouteTask> moved_tasks;
        std::string move_fail_reason;
        size_t task_released_prefixes = 0;
        bool task_source_moved = false;
        bool candidate_retryable = false;
        bool boundary_released = false;
        bool move_succeeded = false;
        size_t boundary_retries = 0;
        do {
          candidate_retryable = false;
          boundary_released = false;
          move_succeeded = moveUnfinishedSource(
              task, &moved_tasks, &move_fail_reason,
              &task_released_prefixes, &task_source_moved,
              &candidate_retryable, &boundary_released);
          if (move_succeeded ||
              !pnr::movingSourceRetriesReleasedBoundary(
                  boundary_released, boundary_retries)) {
            break;
          }
          ++boundary_retries;
        } while (!routeStageSearchCancelled());
        if (!move_succeeded) {
          released_prefixes += task_released_prefixes;
          ++move_failed;
          if (pnr::movingFailureRequiresExit(false, candidate_retryable)) {
            failRouting(
                &task,
                "Moving sources failed: " +
                    (move_fail_reason.empty() ? std::string{"unknown"}
                                              : move_fail_reason));
          }
          if (!candidate_retryable) {
            moving_blocked_until_epoch[source_key] =
                moving_relocation_epoch +
                std::max(4, moving_no_candidate_block_epochs);
          }
          if (move_failure_printed < 12) {
            ++move_failure_printed;
            PNR_LOG1("ROUT",
                     "routeDesign Moving sources batch relocation failed: "
                     "source='{}', net='{}', reason={}",
                     source->makeName(FULL_NAME_LIMIT), task.net_name,
                     move_fail_reason.empty() ? std::string{"unknown"}
                                              : move_fail_reason);
          }
          rejected_sources.push_back(std::move(task));
          continue;
        }

        released_prefixes += task_released_prefixes;
        physical_source_moves += task_source_moved ? 1 : 0;
        changed_routes += moved_tasks.size();
        relocated_sources.insert(source_key);
        ++relocated;
        const auto *completed_route = findBoundRoute(
            task.net, task.from, task.to, task.from_port, task.to_port,
            task.net_name);
        if (completed_route && routeIsComplete(*completed_route)) {
          ++stage_reports[MOVING_SOURCES_STAGE_INDEX].completed;
        }
        rebuilt_tasks.insert(rebuilt_tasks.end(),
                             std::make_move_iterator(moved_tasks.begin()),
                             std::make_move_iterator(moved_tasks.end()));
      }
      update_batch_progress(pending_sources.size());

      pnr::rotateRejectedMovingSources(rebuilt_tasks, rejected_sources);
      route_todo = std::move(rebuilt_tasks);
      deduplicateRouteTasks(route_todo);
      moving_source_prefix_releases_since_generic += released_prefixes;
      moving_source_moves_since_generic += physical_source_moves;
      moving_source_changed_routes_since_generic += changed_routes;
      bool recovery_due = pnr::movingSourceGenericRecoveryDue(
          moving_source_prefix_releases_since_generic,
          moving_source_moves_since_generic,
          moving_source_changed_routes_since_generic,
          pending_route_todo.size());
      if (recovery_due) {
        moving_source_prefix_releases_since_generic = 0;
        moving_source_moves_since_generic = 0;
        moving_source_changed_routes_since_generic = 0;
        moving_source_generic_recovery_pending = true;
      }
      if (batch_deadline_reached) {
        return relocated != 0;
      }
      if (relocated != 0) {
        stagnant_passes = 0;
        moving_no_completion_passes = 0;
        moving_placement_passes = 0;
        moving_placement_has_completion = false;
        PNR_LOG1("ROUT",
                 "routeDesign Moving sources batch: relocated={}, "
                 "attempted={}, failed={}, released_prefixes={}, source_moves={}, "
                 "changed_routes={}, reroute_tasks={}, batch_limit={}, "
                 "attempt_limit={}, generic_recovery={}",
                 relocated, move_attempted, move_failed, released_prefixes,
                 physical_source_moves, changed_routes, route_todo.size(),
                 batch_limit, batch_attempt_limit,
                 moving_source_generic_recovery_pending);
        moving_relocate_next = !moving_source_generic_recovery_pending;
        return true;
      }
      if (move_attempted != 0) {
        // Rejected probes rotate to the next source without an O(N) retry.
        // Only accumulated prefix releases or moves trigger Generic recovery.
        PNR_LOG1("ROUT",
                 "routeDesign Moving sources batch: relocated=0, attempted={}, "
                 "failed={}, released_prefixes={}, reroute_tasks={}, "
                 "batch_limit={}, attempt_limit={}, generic_recovery={}",
                 move_attempted, move_failed, released_prefixes,
                 route_todo.size(), batch_limit, batch_attempt_limit,
                 moving_source_generic_recovery_pending);
        moving_relocate_next = !moving_source_generic_recovery_pending;
        return false;
      }
    }
    // Focused diagnostics may select a deferred sink before unrelated active
    // work, so the requested relocation still has time to reroute its cluster.
    if (!moving_focus_inst &&
        routeDebugEnabled("SCALEPNR_DEBUG_MOVE_FOCUS")) {
      auto focused = std::find_if(
          moving_deferred_todo.begin(), moving_deferred_todo.end(),
          [&](const RouteTask &candidate) {
            rtl::Inst *target = relocation_target(candidate);
            return target &&
                   routeDebugMatches("SCALEPNR_DEBUG_MOVE_FOCUS",
                                     target->makeName(FULL_NAME_LIMIT));
          });
      if (focused != moving_deferred_todo.end()) {
        route_todo.insert(route_todo.begin(), std::move(*focused));
        moving_deferred_todo.erase(focused);
      }
    }
    // A focused diagnostic may select one blocked sink before the normal
    // scheduler order so its first relocation can be reproduced directly.
    if (!moving_focus_inst && !route_todo.empty() &&
        routeDebugEnabled("SCALEPNR_DEBUG_MOVE_FOCUS")) {
      auto focused = std::find_if(
          route_todo.begin(), route_todo.end(),
          [&](const RouteTask &candidate) {
            rtl::Inst *target = relocation_target(candidate);
            return target &&
                   routeDebugMatches("SCALEPNR_DEBUG_MOVE_FOCUS",
                                     target->makeName(FULL_NAME_LIMIT));
          });
      if (focused != route_todo.end() && focused != route_todo.begin()) {
        std::iter_swap(route_todo.begin(), focused);
      }
    }
    for (RouteTask &task : route_todo) {
      const std::vector<Wire> *candidate_route =
          findBoundRoute(task.net, task.from, task.to, task.from_port,
                         task.to_port, task.net_name);
      // Skip completed deferred work lazily so each focus examines candidates
      // only until it finds the next genuinely unfinished endpoint.
      if (!pnr::movingTaskNeedsRelocation(candidate_route &&
                                          routeIsComplete(*candidate_route))) {
        continue;
      }
      if (!can_move_task(task)) {
        continue;
      }
      ++move_attempted;
      std::vector<RouteTask> moved_tasks;
      RouteTask move_task = task;
      if (moving_focus_inst) {
        if (!task_matches_moving_focus(task)) {
          continue;
        }
        move_task.to = moving_sources_stage ? moving_focus_inst
                                           : relocation_target(task);
      } else if (!moving_sources_stage && source_needs_move(task) &&
                 is_movable_inst(task.from)) {
        failRouting(&task, "invalid driver relocation");
        PNR_ASSERT(false,
                   "routeDesign moving tried to move driver inst='{}' for net "
                   "'{}'; driver takeoff must be solved in Generic routing",
                   task.from->makeName(FULL_NAME_LIMIT), task.net_name);
      }
      if (!moving_focus_inst) {
        move_task.to = relocation_target(move_task);
      }
      rtl::Inst *moved_inst = move_task.to;
      std::string move_fail_reason;
      bool move_succeeded = false;
      bool candidate_retryable = false;
      if (!moving_focus_inst) {
        move_succeeded =
            moving_sources_stage
                ? moveUnfinishedSource(task, &moved_tasks, &move_fail_reason,
                                       nullptr, nullptr,
                                       &candidate_retryable, nullptr)
                : moveUnfinishedCell(move_task, &moved_tasks, &task,
                                     &move_fail_reason, true);
      } else {
        move_succeeded = moveUnfinishedCell(move_task, &moved_tasks, &task,
                                            &move_fail_reason, true);
      }
      if (!move_succeeded) {
        ++move_failed;
        if (pnr::movingFailureRequiresExit(move_succeeded, candidate_retryable)) {
          const size_t failed_stage = moving_sources_stage
              ? MOVING_SOURCES_STAGE_INDEX : MOVING_DESTINATIONS_STAGE_INDEX;
          stage_reports[failed_stage].remaining_tasks = stage_remaining_tasks(failed_stage);
          print_stage_report("Moving task failed");
          failRouting(
              &task,
              std::string(moving_sources_stage ? "Moving sources failed: "
                                              : "Moving destinations failed: ") +
                  (move_fail_reason.empty() ? std::string{"unknown"}
                                            : move_fail_reason));
        }
        if (moved_inst && !candidate_retryable) {
          moving_blocked_until_epoch[movingClusterKey(moved_inst)] =
              moving_relocation_epoch +
              std::max(4, moving_no_candidate_block_epochs);
        }
        if (move_failure_printed < 12) {
          ++move_failure_printed;
          PNR_LOG1("ROUT",
                   "routeDesign moving relocation failed: net='{}', "
                   "from='{}'/'{}', to='{}'/'{}', fanout={}, reason={}",
                   task.net_name,
                   task.from ? task.from->makeName(FULL_NAME_LIMIT)
                             : std::string{},
                   task.from_port,
                   task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
                   task.to_port, task.fanout,
                   move_fail_reason.empty() ? std::string{"unknown"}
                                            : move_fail_reason);
        }
        continue;
      }
      // Rebuild endpoint membership after relocation or a source-to-sink
      // focus switch before partitioning the active and deferred work.
      pnr::invalidateMovingEndpointCache(cached_moving_focus,
                                         cached_moving_endpoints);
      if (moving_focus_inst != moved_inst) {
        if (moving_focus_inst) {
          PNR_LOG1("ROUT", "routeDesign moving: switching focus '{}' -> '{}' "
                   "for destination of net='{}'; preserving source placement and prefix",
                   instNameForDump(moving_focus_inst), instNameForDump(moved_inst),
                   task.net_name);
        }
        moving_focus_inst = moved_inst;
        moving_no_completion_passes = 0;
        size_t focus_tried =
            move_tried_placements[movingClusterKey(moving_focus_inst)].size();
        moving_focus_slice_start = focus_tried == 0 ? 0 : focus_tried - 1;
        // Replace only this focus's incident tasks. Unrelated work deferred by
        // earlier stage deadlines must survive every focused relocation.
        size_t removed_stale_incident = pnr::appendNonFocusMovingTasks(
            moving_deferred_todo, route_todo,
            [&](const RouteTask &deferred) {
              return task_matches_moving_focus(deferred);
            });
        if (removed_stale_incident != 0 &&
            envFlagEnabled("SCALEPNR_ROUTE_MOVE_DETAIL")) {
          PNR_LOG1("ROUT",
                   "routeDesign moving: removed {} stale incident tasks from "
                   "the deferred pool before focused rerouting",
                   removed_stale_incident);
        }
        PNR_LOG1("ROUT",
                 "routeDesign moving: focusing inst='{}', deferred_tasks={}, "
                 "focus_tasks={}",
                 moving_focus_inst->makeName(FULL_NAME_LIMIT),
                 moving_deferred_todo.size(), moved_tasks.size());
        std::vector<rtl::Inst *> focus_cluster =
            strictMoveCluster(moving_focus_inst);
        std::vector<rtl::Inst *> endpoint_closure = focus_cluster;
        pnr::appendMovingEndpointChain(endpoint_closure, [](rtl::Inst *member) {
          return member ? generatedPassthroughNeighbors(*member)
                        : std::vector<rtl::Inst *>{};
        });
        PNR_LOG1("ROUT",
                 "routeDesign moving: focus shape cluster={}, "
                 "endpoint_closure={}, incomplete_bindings={}",
                 focus_cluster.size(), endpoint_closure.size(),
                 tech ? countIncompleteRouteBindings(tech->design) : 0);
      }
      if (moving_focus_inst) {
        // Keep incident suffixes that currently exist only in the active
        // queue; binding-based move reconstruction cannot rediscover them.
        size_t preserved_incident = pnr::preserveActiveIncidentTasks(
            moved_tasks, route_todo,
            [&](const RouteTask &active) {
              return task_matches_moving_focus(active);
            },
            [&](const RouteTask &active, const RouteTask &replacement) {
              return sameRouteTask(active, replacement);
            },
            [&](RouteTask &replacement, const RouteTask &active) {
              // Relocation rebuilds bindings with zeroed retry fields. Keep
              // the active Fanout cursor so another placement tries the next
              // branch or routed sibling instead of repeating offset zero,
              // but reset the placement-local no-progress counter.
              pnr::mergeRelocatedMovingTaskState(
                  replacement, active,
                  [](RouteTask &rebuilt, const RouteTask &live) {
                    mergeRouteTaskState(rebuilt, live);
                  });
            });
        if (preserved_incident != 0 &&
            envFlagEnabled("SCALEPNR_ROUTE_MOVE_DETAIL")) {
          PNR_LOG1("ROUT",
                   "routeDesign moving: preserved {} binding-less incident "
                   "tasks across relocation",
                   preserved_incident);
        }
        // Source-tree invalidation can also return sibling routes that do not
        // touch this moved endpoint. Defer those siblings before running the
        // focused Generic/Fanout sequence so they cannot abort the focus.
        size_t deferred_replacement = pnr::partitionMovingReplacementTasks(
            moved_tasks,
            [&](const RouteTask &replacement) {
              return task_matches_moving_focus(replacement);
            },
            [&](const RouteTask &replacement) {
              appendUniqueRouteTask(moving_deferred_todo, replacement);
            });
        if (deferred_replacement != 0 &&
            envFlagEnabled("SCALEPNR_ROUTE_MOVE_DETAIL")) {
          PNR_LOG1("ROUT",
                   "routeDesign moving: deferred {} non-incident replacement "
                   "tasks before focused rerouting",
                   deferred_replacement);
        }
        // Non-incident active tasks were appended to the persistent deferred
        // pool above; incident tasks missing bindings were preserved here.
        // Re-enqueuing every displaced task would duplicate both groups and
        // require rebuilding the complete large deferred pool per relocation.
      }
      route_todo.clear();
      if (moving_focus_inst) {
        for (const RouteTask &moved_task : moved_tasks) {
          appendUniqueRouteTask(route_todo, moved_task);
        }
        pnr::MovingSeedNormalization normalized =
            pnr::normalizeMovingSourceRoles(
                route_todo,
                [&](const RouteTask &focus_task) {
                  return focus_task.from ? sourceRouteKey(focus_task.from,
                                                          focus_task.from_port)
                                         : std::string{};
                },
                [&](const RouteTask &focus_task) {
                  return focus_task.from &&
                         sourceTreeHasCompleteExit(*focus_task.from,
                                                   focus_task.from_port);
                },
                [](const RouteTask &focus_task) {
                  return focus_task.net &&
                         focus_task.net->distributed_source;
                });
        if (moving_sources_stage) {
          std::vector<RouteTask> source_trunks;
          source_trunks.reserve(route_todo.size());
          for (RouteTask &focus_task : route_todo) {
            if (focus_task.fanout) {
              appendUniqueRouteTask(fanout_route_todo, focus_task);
            } else if (task_matches_moving_focus(focus_task)) {
              source_trunks.push_back(std::move(focus_task));
            } else {
              appendUniqueRouteTask(moving_deferred_todo, focus_task);
            }
          }
          route_todo = std::move(source_trunks);
        }
        if (normalized.promoted != 0 || normalized.demoted != 0) {
          PNR_LOG1("ROUT",
                   "routeDesign moving: normalized focus source roles "
                   "promoted={}, demoted={}, tasks={}",
                   normalized.promoted, normalized.demoted, route_todo.size());
        }
        if (envFlagEnabled("SCALEPNR_ROUTE_MOVE_DETAIL")) {
          PNR_LOG1(
              "ROUT",
              "routeDesign moving: active focus_tasks={}, deferred_tasks={}",
              route_todo.size(), moving_deferred_todo.size());
        }
      } else {
        route_todo = std::move(moved_tasks);
      }
      moved = true;
      moving_incident_recoveries = 0;
      stagnant_passes = 0;
      moving_placement_passes = 0;
      moving_placement_has_completion = false;
      break;
    }
    if (!pending_route_todo.empty()) {
      if (moving_focus_inst) {
        for (const RouteTask &task : pending_route_todo) {
          appendUniqueRouteTask(route_todo, task);
        }
      } else {
        for (const RouteTask &task : pending_route_todo) {
          appendUniqueRouteTask(route_todo, task);
        }
      }
      pending_route_todo.clear();
    }
    if (!moved && moving_focus_inst) {
      PNR_LOG1("ROUT",
               "routeDesign moving: focus inst='{}' has no movable active "
               "tasks, restoring {} deferred tasks",
               moving_focus_inst->makeName(FULL_NAME_LIMIT),
               moving_deferred_todo.size());
      moving_blocked_until_epoch[movingClusterKey(moving_focus_inst)] =
          moving_relocation_epoch + moving_cooldown_epochs(moving_focus_inst);
      restore_moving_deferred_tasks();
      moving_focus_inst = nullptr;
      stagnant_passes = 0;
      moving_no_completion_passes = 0;
      // This focus has no movable incident work; select another deferred
      // endpoint without globally rescanning the restored queue.
      moving_relocate_next =
          pnr::movingRelocatesImmediatelyAfterFocus(moving_sources_stage);
      return false;
    }
    if (!moved) {
      PNR_LOG1("ROUT",
               "routeDesign moving found no successful relocation among {} "
               "tasks, focus='{}', deferred={}, attempted={}, failed={}",
               route_todo.size(),
               moving_focus_inst ? moving_focus_inst->makeName(FULL_NAME_LIMIT)
                                 : std::string{},
               moving_deferred_todo.size(), move_attempted, move_failed);
      // Report rejected candidates before cooldown handling returns. This is
      // the only useful diagnostic when every remaining endpoint is fixed or
      // retains a finished-placement mark.
      size_t printed = 0;
      for (const RouteTask &task : route_todo) {
        if (printed++ >= 12) {
          break;
        }
        PNR_LOG1(
            "ROUT",
            "routeDesign moving no-movable task: net='{}', from='{}' type='{}' "
            "port='{}' tile=({},{})/{}, to='{}' type='{}' port='{}' "
            "tile=({},{})/{}, fanout={}, attempt={}, from_movable={}, "
            "to_movable={}, source_needs_move={}",
            task.net_name,
            task.from ? task.from->makeName(FULL_NAME_LIMIT) : std::string{},
            task.from && task.from->cell_ref.peer ? task.from->cell_ref->type
                                                  : std::string{},
            task.from_port,
            task.from && task.from->tile.peer ? task.from->tile->coord.x : -1,
            task.from && task.from->tile.peer ? task.from->tile->coord.y : -1,
            task.from ? task.from->pos : -1,
            task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
            task.to && task.to->cell_ref.peer ? task.to->cell_ref->type
                                              : std::string{},
            task.to_port,
            task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
            task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
            task.to ? task.to->pos : -1, task.fanout, task.attempt,
            is_movable_inst(task.from), is_movable_inst(task.to),
            source_needs_move(task));
      }
      // With every remaining sink on cooldown, no relocation can advance
      // the epoch that would expire those cooldowns. Release them once.
      if (pnr::movingCooldownMustBeReleased(route_todo.size(), move_attempted,
                                            !moving_blocked_until_epoch.empty(),
                                            moving_relocation_epoch,
                                            moving_last_cooldown_clear_epoch)) {
        moving_blocked_until_epoch.clear();
        moving_last_cooldown_clear_epoch = moving_relocation_epoch;
        ++moving_cooldown_clears_without_move;
        moving_relocate_next = true;
        PNR_LOG1("ROUT",
                 "routeDesign moving: released all-blocked cooldowns at "
                 "relocation epoch {}, releases_without_move={}",
                 moving_relocation_epoch, moving_cooldown_clears_without_move);
        return false;
      }
      if ((move_attempted == 0 || move_failed != 0) && !route_todo.empty()) {
        moving_relocate_next = false;
        return false;
      }
    }
    if (!moved) {
      failRouting(first_unfinished_task(),
                                  "no movable unfinished endpoint");
    }
    PNR_ASSERT(moved,
               "routeDesign moving found no movable unfinished endpoint among "
               "{} tasks",
               route_todo.size());
    if (moving_focus_inst && route_todo.empty()) {
      if (moving_focus_complete(*moving_focus_inst)) {
        markMovingClusterFinished(move_finished_insts, moving_focus_inst);
      } else {
        size_t requeued =
            collect_moving_focus_tasks(*moving_focus_inst, route_todo);
        PNR_LOG1("ROUT",
                 "routeDesign moving: inst='{}' focus queue empty but incident "
                 "routes remain, requeued {} incident tasks",
                 moving_focus_inst->makeName(FULL_NAME_LIMIT), requeued);
        if (requeued != 0) {
          size_t focus_tried =
              move_tried_placements[movingClusterKey(moving_focus_inst)].size();
          if (pnr::movingFocusSliceExhausted(focus_tried,
                                             moving_focus_slice_start,
                                             moving_focus_slice_limit) ||
              pnr::movingFocusPlacementsExhausted(focus_tried,
                                                  moving_focus_retry_limit)) {
            defer_exhausted_moving_focus("relocation incident recovery");
            return false;
          }
          moving_relocate_next = pnr::relocateAfterIncidentRequeue(
              requeued, moving_incident_recoveries);
          moving_force_incident_relocation = moving_relocate_next;
          ++moving_incident_recoveries;
          stagnant_passes = 0;
          moving_no_completion_passes = 0;
          return false;
        }
      }
      PNR_LOG1("ROUT",
               "routeDesign moving: inst='{}' had no incident routes left, "
               "restoring {} deferred tasks",
               moving_focus_inst->makeName(FULL_NAME_LIMIT),
               moving_deferred_todo.size());
      restore_moving_deferred_tasks();
      moving_focus_inst = nullptr;
      stagnant_passes = 0;
      moving_passes = 0;
      moving_no_completion_passes = 0;
      // Incident recovery is complete; move directly to the next deferred
      // endpoint instead of globally routing the restored queue.
      moving_relocate_next =
          pnr::movingRelocatesImmediatelyAfterFocus(moving_sources_stage);
      return false;
    }
    return true;
  };
  auto normalize_basic_route_queue = [&]() {
    for (RouteTask &task : route_todo) {
      canonicalizeRouteTaskSource(task);
    }
    std::vector<RouteTask> normalized;
    normalized.reserve(route_todo.size());
    std::vector<RouteTask> deferred_tasks;
    std::unordered_set<std::string> seed_sources;
    size_t promoted = 0;
    size_t deferred = 0;
    for (const RouteTask &task : route_todo) {
      PNR_ASSERT(!task.net || !task.net->distributed_source,
                 "constant route '{}' reached Basic queue normalization",
                 task.net_name);
      std::string key = sourceRouteKey(task.from, task.from_port);
      if (!task.fanout) {
        bool has_seed = task.from &&
                        sourceTreeHasCompleteExit(*task.from, task.from_port);
        if (!has_seed && !seed_sources.contains(key)) {
          seed_sources.insert(key);
          logRouteTaskDecision("normalize.keep_basic", task,
                               "task is the Generic seed candidate for this "
                               "source in this pass");
          normalized.push_back(task);
        } else {
          RouteTask fanout_task = task;
          fanout_task.fanout = true;
          logRouteTaskDecision("normalize.defer_duplicate_seed", fanout_task,
                               has_seed
                                   ? "source already has routed source exit"
                                   : "another Generic seed for this source is "
                                     "already in this pass");
          deferred_tasks.push_back(std::move(fanout_task));
          ++deferred;
        }
        continue;
      }
      bool has_seed =
          task.from && sourceTreeHasCompleteExit(*task.from, task.from_port);
      if (!has_seed && !seed_sources.contains(key)) {
        RouteTask seed_task = task;
        seed_task.fanout = false;
        seed_sources.insert(key);
        logRouteTaskDecision("normalize.promote_missing_seed", seed_task,
                             "fanout task has no routed source exit and "
                             "becomes the Generic seed candidate");
        normalized.push_back(std::move(seed_task));
        ++promoted;
      } else {
        logRouteTaskDecision(
            "normalize.defer_fanout", task,
            has_seed ? "source already has routed source exit"
                     : "seed for this source is already in current Basic pass");
        deferred_tasks.push_back(task);
        ++deferred;
      }
    }
    if (promoted != 0 || deferred != 0) {
      PNR_LOG1("ROUT",
               "routeDesign Basic queue normalized: promoted_seeds={}, "
               "deferred_fanouts={}, basic_tasks={}, fanout_deferred={}",
               promoted, deferred, normalized.size(), fanout_route_todo.size());
    }
    deduplicateRouteTasks(normalized);
    if (!deferred_tasks.empty()) {
      fanout_route_todo.insert(
          fanout_route_todo.end(),
          std::make_move_iterator(deferred_tasks.begin()),
          std::make_move_iterator(deferred_tasks.end()));
      deduplicateRouteTasks(fanout_route_todo);
    }
    route_todo = std::move(normalized);
  };
  auto normalize_moving_route_queue = [&]() {
    for (RouteTask &task : route_todo) {
      canonicalizeRouteTaskSource(task);
    }
    pnr::MovingSeedNormalization normalized = pnr::normalizeMovingSourceRoles(
        route_todo,
        [&](const RouteTask &task) {
          return task.from ? sourceRouteKey(task.from, task.from_port)
                           : std::string{};
        },
        [&](const RouteTask &task) {
          return task.from &&
                 sourceTreeHasCompleteExit(*task.from, task.from_port);
        },
        [](const RouteTask &task) {
          return task.net && task.net->distributed_source;
        });
    if (normalized.promoted != 0 || normalized.demoted != 0) {
      PNR_LOG1("ROUT",
               "routeDesign moving queue normalized: promoted_seeds={}, "
               "demoted_duplicates={}, tasks={}",
               normalized.promoted, normalized.demoted, route_todo.size());
    }
    if (moving_sources_stage) {
      std::vector<RouteTask> trunks;
      trunks.reserve(route_todo.size());
      for (RouteTask &task : route_todo) {
        if (task.fanout) {
          appendUniqueRouteTask(fanout_route_todo, task);
        } else {
          trunks.push_back(std::move(task));
        }
      }
      route_todo = std::move(trunks);
    }
  };

  // Constants are routed to completion from per-tile distributed roots before
  // Basic starts. They never enter trunk, fanout, or placement-recovery queues.
  {
    RouteStageReport &constant_report = stage_reports[CONSTANT_STAGE_INDEX];
    constant_report.started = true;
    constant_report.start_tasks = distributed_route_tasks.size();
    std::vector<RouteTask> constant_work = distributed_route_tasks;
    auto constant_start = std::chrono::steady_clock::now();
    stage_wall_starts[CONSTANT_STAGE_INDEX] = constant_start;
    route_stage_deadline =
        constant_start +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                route_stage_timeout_seconds[CONSTANT_STAGE_INDEX]));
    route_stage_deadline_enabled = true;
    route_stage_deadline_expired = false;
    resetRouteProgressWatchdog(
        CONSTANT_STAGE_INDEX, routeStageName(CONSTANT_STAGE_INDEX),
        constant_work.size(), route_progress_window_seconds,
        route_stagnant_window_limit);
    PNR_LOG1("ROUT", "routeDesign stage: Const routing, tasks={}",
             constant_work.size());

    while (!constant_work.empty()) {
      for (RouteTask &task : constant_work) {
        task.remove_after_pass = false;
        PNR_ASSERT(task.net && task.net->distributed_source,
                   "Const routing received non-constant task '{}'",
                   task.net_name);
      }
      route_stats.clear();
      RouteBatchResult batch = routeTaskBatch(
          RouteTaskMode::Constant, constant_work, constant_work.size(),
          route_recursion_limit);
      std::erase_if(constant_work, [](const RouteTask &task) {
        return task.remove_after_pass;
      });
      updateRouteProgressRemaining(constant_work.size());
      ++constant_report.passes;
      constant_report.attempted += batch.attempted;
      constant_report.completed += batch.completed;
      constant_report.active += batch.active;
      constant_report.advanced += batch.advanced;
      constant_report.changed += batch.changed;
      constant_report.searches += route_stats.distributed_path_calls;
      constant_report.pops += route_stats.distributed_path_nodes;
      constant_report.remaining_tasks = constant_work.size();
      PNR_LOG1("ROUT",
               "routeDesign pass: {}, stage=Const routing, todo: {} -> {}, "
               "completed={}, active={}, changed={}",
               constant_report.passes, batch.before, constant_work.size(),
               batch.completed, batch.active, batch.changed);

      bool timed_out = routeStageSearchCancelled() &&
                       route_stage_deadline_expired;
      if (route_stage_stagnated) {
        fail_stage_stagnation(CONSTANT_STAGE_INDEX);
      }
      if (timed_out) {
        constant_report.timed_out = true;
        failRouting(
            constant_work.empty() ? nullptr : &constant_work.front(),
            "Const routing timeout");
      }
      PNR_ASSERT(!timed_out,
                 "Const routing timeout with {} unresolved constant routes",
                 constant_work.size());
      if (!constant_work.empty() && batch.completed == 0 &&
          batch.active == 0 && batch.changed == 0) {
        failRouting(&constant_work.front(),
                                    "Const routing made no progress");
        PNR_ASSERT(false,
                   "Const routing made no progress with {} unresolved "
                   "constant routes; first='{}'",
                   constant_work.size(), constant_work.front().net_name);
      }
    }
    route_stage_deadline_enabled = false;
    constant_report.seconds = elapsedSeconds(
        constant_start, std::chrono::steady_clock::now());
    PNR_LOG1("ROUT",
             "routeDesign stage complete: Const routing, tasks={} -> 0, "
             "passes={}, time={:.3f}s",
             constant_report.start_tasks, constant_report.passes,
             constant_report.seconds);
  }

  for (int pass = 0;
       pass < max_route_passes &&
       pnr::routeSchedulerHasWork(!route_todo.empty(), moving_stage,
                                  !moving_deferred_todo.empty() ||
                                      !moving_source_retry_todo.empty());
       ++pass) {
    ++stage_pass;
    route_suffix_depth_limit = pnr::routeSuffixDepthForPass(
        !fanout_stage && !moving_stage && !fanout_seed_repair_active,
        stage_pass, 5);
    auto epoch_start_time = std::chrono::steady_clock::now();
    size_t pass_stage_index =
        moving_sources_stage
            ? MOVING_SOURCES_STAGE_INDEX
            : (moving_stage ? MOVING_DESTINATIONS_STAGE_INDEX
                            : ((fanout_stage || fanout_seed_repair_active)
                                   ? FANOUT_STAGE_INDEX
                                   : BASIC_STAGE_INDEX));
    RouteStageReport &pass_stage_report = stage_reports[pass_stage_index];
    if (stage_wall_starts[pass_stage_index] ==
        std::chrono::steady_clock::time_point{}) {
      stage_wall_starts[pass_stage_index] = epoch_start_time;
    }
    double stage_wall_seconds = std::chrono::duration<double>(
                                    epoch_start_time -
                                    stage_wall_starts[pass_stage_index])
                                    .count();
    if (!pass_stage_report.started) {
      pass_stage_report.started = true;
      pass_stage_report.start_tasks = route_todo.size();
    }
    const double pass_stage_timeout_seconds =
        route_stage_timeout_seconds[pass_stage_index];
    bool pass_entry_timeout = pnr::routeStageTimeoutIsFatal(
        stage_wall_seconds, pass_stage_timeout_seconds,
        !route_todo.empty() || !moving_deferred_todo.empty() ||
            !moving_source_retry_todo.empty());
    if (pnr::routeStageEntryTimeoutRequiresFailure(pass_entry_timeout,
                                                   moving_stage)) {
      fail_stage_timeout(pass_stage_index);
    }
    pnr::RouteStageTimeCharge stage_time_charge(pass_stage_report.seconds);
    double stage_seconds_left = pnr::routeStageSecondsRemaining(
        stage_wall_seconds, pass_stage_timeout_seconds);
    route_stage_deadline =
        stage_wall_starts[pass_stage_index] +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(pass_stage_timeout_seconds));
    route_stage_deadline_enabled = true;
    route_stage_deadline_expired = stage_seconds_left <= 0.0;
    size_t progress_remaining = stage_remaining_tasks(pass_stage_index);
    if (route_progress_stage != pass_stage_index) {
      resetRouteProgressWatchdog(
          pass_stage_index, routeStageName(pass_stage_index),
          progress_remaining, route_progress_window_seconds,
          route_stagnant_window_limit);
    } else {
      updateRouteProgressRemaining(progress_remaining);
    }
    if (pnr::routeStageStagnationRequiresFailure(
            route_stage_stagnated, pass_stage_index == BASIC_STAGE_INDEX)) {
      stage_time_charge.finish();
      fail_stage_stagnation(pass_stage_index);
    }
    // Only Generic routing consumes persistent deadends. Fanout and Moving
    // clear tile masks once on stage entry and must not sweep the whole grid
    // on every short routing pass.
    if (route_deadends_enabled) {
      applyRouteDeadends(route_src_deadends);
    }
    // A completed/yielded focus can leave only persistent deferred work. Start
    // the next relocation now instead of burning the stage budget on empty passes.
    if (pnr::movingDeferredWorkNeedsRelocation(
            moving_stage, moving_focus_inst != nullptr, !route_todo.empty(),
            !moving_deferred_todo.empty() ||
                !moving_source_retry_todo.empty())) {
      moving_relocate_next = true;
    }
    if (moving_stage && moving_relocate_next) {
      moving_relocate_next = false;
      perform_moving_relocation();
      if (routeStageSearchCancelled()) {
        stage_time_charge.finish();
        if (route_stage_stagnated) {
          fail_stage_stagnation(pass_stage_index);
        }
        fail_stage_timeout(pass_stage_index);
      }
      if (pnr::movingRelocationCanContinue(
              moving_relocate_next, !route_todo.empty(),
              !moving_deferred_todo.empty() || !moving_source_retry_todo.empty(),
              !pending_route_todo.empty())) {
        continue;
      }
      moving_relocate_next = false;
    }
    if (moving_stage) {
      normalize_moving_route_queue();
    }
    if (!fanout_stage && !moving_stage) {
      bool route_pass_detail = envFlagEnabled("SCALEPNR_ROUTE_PASS_DETAIL");
      auto normalize_start = std::chrono::steady_clock::time_point{};
      size_t before_normalize = 0;
      size_t before_fanout_deferred = 0;
      if (route_pass_detail) {
        normalize_start = std::chrono::steady_clock::now();
        before_normalize = route_todo.size();
        before_fanout_deferred = fanout_route_todo.size();
      }
      normalize_basic_route_queue();
      if (stage_pass > 1) {
        std::array<size_t, 3> route_classes =
            pnr::prioritizeGenericRouteTasks(
            route_todo, [&](const RouteTask &task) {
              const std::vector<Wire> *route =
                  findBoundRoute(task.net, task.from, task.to, task.from_port,
                                 task.to_port, task.net_name);
              size_t crossbars = routeCrossbarFragments(route);
              return crossbars == 0 ? 0U : (crossbars == 1 ? 1U : 2U);
            }, true);
        if (route_pass_detail) {
          PNR_LOG1("ROUT",
                   "routeDesign normalize: stage=Basic, prefix_first={}, "
                   "takeoff_second={}, empty_last={}",
                   route_classes[2], route_classes[1], route_classes[0]);
        }
      }
      if (route_pass_detail) {
        auto normalize_end = std::chrono::steady_clock::now();
        PNR_LOG1("ROUT",
                 "routeDesign normalize: stage=Basic, in={}, out={}, "
                 "fanout_deferred {}->{}, time={:.3f}s",
                 before_normalize, route_todo.size(), before_fanout_deferred,
                 fanout_route_todo.size(),
                 elapsedSeconds(normalize_start, normalize_end));
      }
    }
    route_stats.clear();
    resetPassPreemptionState();
    for (RouteTask &task : route_todo) {
      task.remove_after_pass = false;
    }
    size_t before = route_todo.size();
    int completed_this_pass = 0;
    int placement_completed_this_pass = 0;
    int active_this_pass = 0;
    int advanced_this_pass = 0;
    int changed_this_pass = 0;
    size_t task_index = 0;
    size_t attempted_this_pass = 0;
    size_t task_limit_this_pass = route_todo.size();
    if (moving_sources_stage && moving_source_generic_recovery_pending) {
      task_limit_this_pass =
          pnr::movingSourceGenericRecoveryTaskLimit(route_todo.size());
    }
    if (fanout_stage) {
      task_limit_this_pass =
          std::min(route_todo.size(),
                   static_cast<size_t>(std::max(64, design_cells / 4)));
    }
    if (!heartbeat_enabled) {
      RouteTaskMode mode =
          moving_stage
              ? RouteTaskMode::Moving
              : (fanout_stage ? RouteTaskMode::Fanout : RouteTaskMode::Generic);
      RouteBatchResult batch = routeTaskBatch(
          mode, route_todo, task_limit_this_pass, route_recursion_limit);
      if (pnr::routeStageStagnationRequiresFailure(
              route_stage_stagnated, pass_stage_index == BASIC_STAGE_INDEX)) {
        stage_time_charge.finish();
        fail_stage_stagnation(pass_stage_index);
      }
      attempted_this_pass = batch.attempted;
      completed_this_pass = static_cast<int>(batch.completed);
      placement_completed_this_pass =
          static_cast<int>(batch.placement_completed);
      active_this_pass = static_cast<int>(batch.active);
      advanced_this_pass = static_cast<int>(batch.advanced);
      changed_this_pass = static_cast<int>(batch.changed);
      if (batch.deferred_fanout != 0) {
        PNR_LOG1("ROUT",
                 "routeDesign moving: deferred {} fanout tasks until base "
                 "routes complete",
                 batch.deferred_fanout);
      }
      if (fanout_stage && completed_this_pass == 0 &&
          !batch.attempted_names.empty()) {
        std::string attempted_names;
        for (const std::string &name : batch.attempted_names) {
          if (!attempted_names.empty()) {
            attempted_names += " | ";
          }
          attempted_names += name;
        }
        PNR_LOG1("ROUT", "routeDesign fanout zero-complete attempted={}",
                 attempted_names);
      }
    } else {
      std::vector<std::string> heartbeat_attempted_names;
      std::vector<std::string> heartbeat_active_names;
      std::vector<std::string> heartbeat_advanced_names;
      std::vector<std::string> heartbeat_changed_names;
      // Index pending Generic sources once per pass; Moving fanouts then wait
      // for their own source in constant expected time.
      std::unordered_set<std::string> moving_pending_seed_sources;
      if (moving_stage) {
        for (RouteTask &queued : route_todo) {
          canonicalizeRouteTaskSource(queued);
          if (!queued.remove_after_pass && !queued.fanout && queued.from) {
            moving_pending_seed_sources.insert(
                sourceRouteKey(queued.from, queued.from_port));
          }
        }
      }
      auto moving_source_seed_pending = [&](const RouteTask &fanout_task) {
        return moving_stage && fanout_task.from &&
               moving_pending_seed_sources.contains(
                   sourceRouteKey(fanout_task.from, fanout_task.from_port));
      };
      for (auto it = route_todo.begin(); it != route_todo.end();) {
        if (routeStageSearchCancelled()) {
          std::rotate(route_todo.begin(), it, route_todo.end());
          break;
        }
        if (attempted_this_pass >= task_limit_this_pass) {
          std::rotate(route_todo.begin(), it, route_todo.end());
          break;
        }
        if (it->net && it->net->distributed_source) {
          PNR_ASSERT(false,
                     "constant route '{}' escaped into ordinary routing",
                     it->net_name);
        }
        if (pnr::movingFanoutWaitsForSourceSeed(
                moving_stage, it->fanout, moving_source_seed_pending(*it))) {
          pending_route_todo.push_back(*it);
          it->remove_after_pass = true;
          ++it;
          continue;
        }
        if (!it->from || !it->to) {
          it->remove_after_pass = true;
          ++it;
          continue;
        }
        if (!it->net_name.empty() &&
            preempted_route_names_this_pass.contains(it->net_name)) {
          ++it;
          continue;
        }
        if (fanout_stage && !it->fanout) {
          // Leave Generic repair seeds untouched; pass finalization moves
          // them back to Generic mode before any dependent Fanout runs.
          ++it;
          continue;
        }
        const std::vector<Wire> *queued_route =
            findBoundRoute(it->net, it->from, it->to, it->from_port,
                           it->to_port, it->net_name);
        bool queued_route_complete =
            queued_route && routeIsComplete(*queued_route);
        if (fanout_stage &&
            pnr::fanoutWaitsForGenericSeed(
                true, it->fanout, queued_route_complete,
                sourceTreeHasCompleteExit(*it->from, it->from_port))) {
          logRouteTaskDecision(
              "loop.fanout.defer_missing_seed", *it,
              "fanout task waits until Generic creates a routed source exit");
          fanout_route_todo.push_back(*it);
          it->remove_after_pass = true;
          ++it;
          continue;
        }
        ++attempted_this_pass;
        if (fanout_stage && heartbeat_attempted_names.size() < 16) {
          heartbeat_attempted_names.push_back(it->net_name);
        }
        size_t current_task_index = task_index++;
        if (std::getenv("SCALEPNR_ROUTE_TASK_TRACE")) {
          PNR_LOG1(
              "ROUT",
              "routeDesign task start: stage={}, pass={}, task={}/{}, "
              "net='{}', from='{}'/'{}', to='{}'/'{}', fanout={}, attempt={}",
              moving_stage
                  ? (moving_sources_stage ? "Moving sources"
                                          : "Moving destinations")
                  : ((fanout_stage || fanout_seed_repair_active)
                         ? "Fanouts routing"
                         : "Basic routing"),
              pass + 1, current_task_index, task_limit_this_pass, it->net_name,
              it->from ? it->from->makeName(FULL_NAME_LIMIT) : std::string{},
              it->from_port,
              it->to ? it->to->makeName(FULL_NAME_LIMIT) : std::string{},
              it->to_port, it->fanout, it->attempt);
        }
        if (moving_stage && it->fanout &&
            !sourceTreeHasCompleteExit(*it->from, it->from_port)) {
          logRouteTaskDecision("loop.moving.promote_missing_seed", *it,
                               "fanout task lost its routed source exit before "
                               "this Moving attempt");
          it->fanout = false;
          moving_pending_seed_sources.insert(
              sourceRouteKey(it->from, it->from_port));
        }
        bool heartbeat_sample = false;
        auto heartbeat_now = std::chrono::steady_clock::now();
        if (heartbeat_enabled &&
            elapsedSeconds(last_heartbeat_time, heartbeat_now) >=
                heartbeat_seconds) {
          heartbeat_sample = true;
        }
        std::vector<Wire> *existing_route_before =
            findBoundRoute(it->net, it->from, it->to, it->from_port,
                           it->to_port, it->net_name);
        size_t route_size_before =
            existing_route_before ? existing_route_before->size() : 0;
        size_t route_xbars_before =
            heartbeat_sample ? routeCrossbarFragments(existing_route_before)
                             : 0;
        bool route_complete_before = heartbeat_sample &&
                                     existing_route_before &&
                                     routeIsComplete(*existing_route_before);
        RouteStats stats_before = route_stats;
        const int task_attempt_budget = pnr::routeTaskAttemptBudget(
            !fanout_stage && !moving_stage, route_suffix_depth_limit == 1,
            route_recursion_limit);
        route_recursion_budget = task_attempt_budget;
        bool task_complete = false;
        bool task_progress = false;
        bool task_changed = false;
        if (envFlagEnabled("SCALEPNR_ROUTE_TASK_TRACE")) {
          const char *stage_name_now =
              moving_stage
                  ? (moving_sources_stage ? "Moving sources"
                                          : "Moving destinations")
                  : ((fanout_stage || fanout_seed_repair_active)
                         ? "Fanouts routing"
                         : "Basic routing");
          PNR_LOG1("ROUT",
                   "routeDesign task start: stage={}, pass={}, task={}/{}, "
                   "todo={}, net='{}', from='{}' port='{}', to='{}' port='{}', "
                   "fanout={}, attempt={}",
                   stage_name_now, pass + 1, current_task_index,
                   task_limit_this_pass, route_todo.size(), it->net_name,
                   it->from ? it->from->makeName(FULL_NAME_LIMIT)
                            : std::string{},
                   it->from_port,
                   it->to ? it->to->makeName(FULL_NAME_LIMIT) : std::string{},
                   it->to_port, it->fanout, it->attempt);
        }
        auto invoke_start = std::chrono::steady_clock::now();
        while (route_recursion_budget > 0) {
          route_changed = false;
          route_progress = false;
          if (routeNetTask(*it)) {
            task_complete = true;
            task_progress = true;
            task_changed = true;
            break;
          }
          task_changed = task_changed || route_changed;
          if (!route_progress) {
            break;
          }
          task_progress = true;
        }
        route_stats.task_invoke_ns += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - invoke_start)
                .count());
        std::vector<Wire> *existing_route_after =
            findBoundRoute(it->net, it->from, it->to, it->from_port,
                           it->to_port, it->net_name);
        size_t route_size_after =
            existing_route_after ? existing_route_after->size() : 0;
        size_t route_xbars_after =
            heartbeat_sample ? routeCrossbarFragments(existing_route_after) : 0;
        bool route_complete_after = heartbeat_sample && existing_route_after &&
                                    routeIsComplete(*existing_route_after);
        if (!task_complete) {
          task_progress = route_size_after > route_size_before;
          it->no_progress_passes =
              moving_stage ? pnr::updateMovingTaskNoProgressPasses(
                                 it->no_progress_passes, task_progress)
                           : 0;
        }
        logRouteTaskDecision(
            "loop.result", *it,
            "complete=" + std::string(task_complete ? "true" : "false") +
                ", progress=" + (task_progress ? "true" : "false") +
                ", changed=" + (task_changed ? "true" : "false") +
                ", mode=" +
                (moving_stage
                     ? (moving_sources_stage ? "Moving sources"
                                             : "Moving destinations")
                     : ((fanout_stage || fanout_seed_repair_active) ? "Fanout"
                                                                    : "Generic")));
        bool generic_mode =
            !fanout_stage && !moving_stage && !moving_focus_inst;
        if (it->source_tree_rebuilt) {
          // Preserve the replacement Generic seed selected after exhausting
          // the old source tree's branch points in Moving mode.
          it->fanout = false;
          it->source_tree_rebuilt = false;
          if (it->from) {
            moving_pending_seed_sources.insert(
                sourceRouteKey(it->from, it->from_port));
          }
        }
        if (pnr::shouldRotateFailedGenericSeed(
                generic_mode, task_complete, task_progress,
                !existing_route_after || existing_route_after->empty()) &&
            rotateFailedGenericSeed(*it)) {
          task_changed = true;
          logRouteTaskDecision("loop.generic.rotate_seed", *it,
                               "replaced a repeatedly failed sink with another "
                               "sink from the same source port");
        }
        if (heartbeat_sample) {
          const char *stage_name_now =
              moving_stage
                  ? (moving_sources_stage ? "Moving sources"
                                          : "Moving destinations")
                  : ((fanout_stage || fanout_seed_repair_active)
                         ? "Fanouts routing"
                         : "Basic routing");
          PNR_LOG1(
              "ROUT",
              "routeDesign heartbeat: stage={}, pass={}, task={}/{}, todo={}, "
              "net='{}', from='{}' port='{}', to='{}' port='{}', fanout={}, "
              "attempt={}, before(size={},xbars={},complete={}), "
              "after(size={},xbars={},complete={}), "
              "result(complete={},progress={},changed={},recursions={}), "
              "pass_progress(done={},advanced={},changed={}), "
              "stats(tasks={},edge_trials={},edge_ok={},reject_busy={},reject_"
              "busy_src={},reject_busy_dst={},src_deadend_bits={},time_ms("
              "setup={:.3f},name={:.3f},conflict={:.3f},copy={:.3f},lease={:."
              "3f},resolve={:.3f},target={:.3f},state={:.3f},bf_loop={:.3f},bf_"
              "dock={:.3f},bf_commit={:.3f},bf_mat={:.3f},passthrough={:.3f},"
              "find={:.3f},candidates={:.3f},endpoint={:.3f},direct={:.3f},"
              "best_first={:.3f},route={:.3f},attach={:.3f},invoke={:.3f},"
              "distributed(lookup={:.3f},endpoint={:.3f},path={:.3f},"
              "commit={:.3f},calls={},roots={},nodes={})))",
              stage_name_now, pass + 1, current_task_index,
              task_limit_this_pass, route_todo.size(), it->net_name,
              it->from ? it->from->makeName(FULL_NAME_LIMIT) : std::string{},
              it->from_port,
              it->to ? it->to->makeName(FULL_NAME_LIMIT) : std::string{},
              it->to_port, it->fanout, it->attempt, route_size_before,
              route_xbars_before, route_complete_before, route_size_after,
              route_xbars_after, route_complete_after, task_complete,
              task_progress, task_changed,
              task_attempt_budget - route_recursion_budget,
              completed_this_pass, advanced_this_pass, changed_this_pass,
              route_stats.task_attempts, route_stats.edge_trials,
              route_stats.edge_accepted, route_stats.edge_rejected_busy,
              route_stats.edge_rejected_busy_src,
              route_stats.edge_rejected_busy_dst,
              countDeadendBits(route_src_deadends),
              static_cast<double>(route_stats.edge_order_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_name_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_conflict_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_cb_copy_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_lease_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_resolve_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_target_pin_ns) / 1000000.0,
              static_cast<double>(route_stats.edge_state_ns) / 1000000.0,
              static_cast<double>(route_stats.best_first_loop_ns) / 1000000.0,
              static_cast<double>(route_stats.best_first_dock_ns) / 1000000.0,
              static_cast<double>(route_stats.best_first_commit_ns) / 1000000.0,
              static_cast<double>(route_stats.best_first_materialize_ns) /
                  1000000.0,
              static_cast<double>(route_stats.task_passthrough_ns) / 1000000.0,
              static_cast<double>(route_stats.task_find_ns) / 1000000.0,
              static_cast<double>(route_stats.task_candidate_ns) / 1000000.0,
              static_cast<double>(route_stats.task_endpoint_ns) / 1000000.0,
              static_cast<double>(route_stats.task_direct_ns) / 1000000.0,
              static_cast<double>(route_stats.task_best_first_ns) / 1000000.0,
              static_cast<double>(route_stats.task_route_ns) / 1000000.0,
              static_cast<double>(route_stats.task_attach_ns) / 1000000.0,
              static_cast<double>(route_stats.task_invoke_ns) / 1000000.0,
              static_cast<double>(route_stats.distributed_lookup_ns) /
                  1000000.0,
              static_cast<double>(route_stats.distributed_endpoint_ns) /
                  1000000.0,
              static_cast<double>(route_stats.distributed_path_ns) /
                  1000000.0,
              static_cast<double>(route_stats.distributed_commit_ns) /
                  1000000.0,
              route_stats.distributed_path_calls,
              route_stats.distributed_path_roots,
              route_stats.distributed_path_nodes);
          last_heartbeat_time = std::chrono::steady_clock::now();
        }
        if (task_complete) {
          noteRouteProgressCompletion();
          ++completed_this_pass;
          if (pnr::movingCompletionRenewsPlacement(
                  true, it->net && it->net->distributed_source)) {
            ++placement_completed_this_pass;
          }
          ++active_this_pass;
          if (moving_stage && !it->fanout && it->from) {
            moving_pending_seed_sources.erase(
                sourceRouteKey(it->from, it->from_port));
          }
          // Compact once after the pass. Erasing every completed task here is
          // quadratic for large designs because each erase moves the tail.
          it->remove_after_pass = true;
          ++it;
          continue;
        }
        if (task_progress) {
          ++advanced_this_pass;
          if (fanout_stage && heartbeat_advanced_names.size() < 8) {
            heartbeat_advanced_names.push_back(it->net_name);
          }
        }
        if (task_changed) {
          ++changed_this_pass;
          if (fanout_stage && heartbeat_changed_names.size() < 8) {
            heartbeat_changed_names.push_back(it->net_name);
          }
        }
        if (task_changed || task_progress) {
          ++active_this_pass;
          if (fanout_stage && heartbeat_active_names.size() < 8) {
            heartbeat_active_names.push_back(it->net_name);
          }
        }
        ++it;
      }
      if (fanout_stage && completed_this_pass == 0 &&
          !heartbeat_attempted_names.empty()) {
        auto join_names = [](const std::vector<std::string> &names) {
          std::string result;
          for (const std::string &name : names) {
            if (!result.empty()) {
              result += " | ";
            }
            result += name;
          }
          return result;
        };
        PNR_LOG1("ROUT",
                 "routeDesign fanout zero-complete attempted={} active={} "
                 "advanced={} changed={}",
                 join_names(heartbeat_attempted_names),
                 join_names(heartbeat_active_names),
                 join_names(heartbeat_advanced_names),
                 join_names(heartbeat_changed_names));
      }
    }
    std::erase_if(route_todo,
                  [](const RouteTask &task) { return task.remove_after_pass; });
    updateRouteProgressRemaining(stage_remaining_tasks(pass_stage_index));
    if (pnr::routeStageStagnationRequiresFailure(
            route_stage_stagnated, pass_stage_index == BASIC_STAGE_INDEX)) {
      stage_time_charge.finish();
      fail_stage_stagnation(pass_stage_index);
    }
    // Keep a useful moved placement after grounding one incident route. The
    // remaining siblings receive a fresh finite window at this same location.
    pnr::renewMovingTaskWindowsAfterCompletion(
        route_todo,
        moving_focus_inst && placement_completed_this_pass != 0);
    if (moving_focus_inst && placement_completed_this_pass != 0) {
      moving_placement_has_completion = true;
    }
    if (!pending_route_todo.empty()) {
      bool appended_route = false;
      bool appended_fanout = false;
      bool appended_moving = false;
      for (RouteTask &task : pending_route_todo) {
        if (moving_sources_stage && task.fanout) {
          fanout_route_todo.push_back(std::move(task));
          appended_fanout = true;
          continue;
        }
        if (moving_focus_inst && !task_matches_moving_focus(task)) {
          // Repeated source-tree repairs can invalidate the same external
          // sibling; merge its retry state instead of growing the pool.
          pnr::mergeMovingDeferredTask(
              moving_deferred_todo, task,
              [](const RouteTask &left, const RouteTask &right) {
                return sameRouteTask(left, right);
              },
              [](RouteTask &old, const RouteTask &replacement) {
                mergeRouteTaskState(old, replacement);
              });
          appended_moving = true;
          continue;
        }
        // Fanout is a complete logical stage. Demoted Generic victims are
        // conserved for Moving instead of interrupting all active branches.
        if (pnr::deferFanoutRepairToMoving(fanout_stage, task.fanout)) {
          moving_deferred_todo.push_back(std::move(task));
          appended_moving = true;
          continue;
        }
        if (pnr::deferToFanoutStage(task.fanout, fanout_stage, moving_stage,
                                    moving_focus_inst != nullptr)) {
          fanout_route_todo.push_back(std::move(task));
          appended_fanout = true;
          continue;
        }
        route_todo.push_back(std::move(task));
        appended_route = true;
      }
      if (appended_route) {
        deduplicateRouteTasks(route_todo);
      }
      if (appended_fanout) {
        deduplicateRouteTasks(fanout_route_todo);
      }
      if (appended_moving && !moving_focus_inst) {
        deduplicateRouteTasks(moving_deferred_todo);
      }
      pending_route_todo.clear();
    }
    // One amortized Generic pass consumes capacity released by a substantial
    // source-move batch, then control returns immediately to backward routing.
    if (moving_sources_stage && moving_source_generic_recovery_pending) {
      moving_source_generic_recovery_pending = false;
      moving_relocate_next = !route_todo.empty();
    }
    bool restored_moving_focus = false;
    if (moving_focus_inst && route_todo.empty()) {
      if (moving_focus_complete(*moving_focus_inst)) {
        markMovingClusterFinished(move_finished_insts, moving_focus_inst);
      } else {
        size_t requeued =
            collect_moving_focus_tasks(*moving_focus_inst, route_todo);
        PNR_LOG1("ROUT",
                 "routeDesign moving: inst='{}' focus queue empty but incident "
                 "routes remain, requeued {} incident tasks",
                 moving_focus_inst->makeName(FULL_NAME_LIMIT), requeued);
        if (requeued != 0) {
          size_t focus_tried =
              move_tried_placements[movingClusterKey(moving_focus_inst)].size();
          if (pnr::movingFocusSliceExhausted(focus_tried,
                                             moving_focus_slice_start,
                                             moving_focus_slice_limit) ||
              pnr::movingFocusPlacementsExhausted(focus_tried,
                                                  moving_focus_retry_limit)) {
            defer_exhausted_moving_focus("pass incident recovery");
            continue;
          }
          stagnant_passes = 0;
          moving_no_completion_passes = 0;
          moving_relocate_next = pnr::relocateAfterIncidentRequeue(
              requeued, moving_incident_recoveries);
          moving_force_incident_relocation = moving_relocate_next;
          ++moving_incident_recoveries;
          continue;
        }
      }
      PNR_LOG1(
          "ROUT",
          "routeDesign moving: inst='{}' rerouted, restoring {} deferred tasks",
          moving_focus_inst->makeName(FULL_NAME_LIMIT),
          moving_deferred_todo.size());
      restore_moving_deferred_tasks();
      moving_focus_inst = nullptr;
      stagnant_passes = 0;
      moving_passes = 0;
      moving_no_completion_passes = 0;
      moving_stage = true;
      // This focus already completed its Generic/Fanout incident recovery.
      // Select the next deferred endpoint without a global queue sweep.
      moving_relocate_next =
          pnr::movingRelocatesImmediatelyAfterFocus(moving_sources_stage);
      restored_moving_focus = true;
    }
    bool basic_mode = !fanout_stage && !fanout_seed_repair_active &&
                      !moving_stage && !moving_focus_inst;
    basic_growth_passes = pnr::updateBasicGrowthPasses(
        basic_mode, before, route_todo.size(), basic_growth_passes);
    bool basic_congestion_handoff = pnr::basicGrowthRequiresHandoff(
        basic_mode, before, route_todo.size(), completed_this_pass,
        advanced_this_pass, basic_growth_passes);
    bool route_stagnant_this_pass =
        completed_this_pass == 0 && advanced_this_pass == 0 &&
        changed_this_pass == 0 && route_todo.size() >= before;
    if ((fanout_stage || moving_stage || moving_focus_inst) &&
        changed_this_pass > 0) {
      route_stagnant_this_pass = false;
    }
    if (basic_mode && completed_this_pass == 0 && advanced_this_pass == 0 &&
        changed_this_pass == 0 && route_todo.size() >= before &&
        attempted_this_pass >= task_limit_this_pass) {
      route_stagnant_this_pass = true;
    }
    if (fanout_stage) {
      if (completed_this_pass == 0 && advanced_this_pass == 0 &&
          changed_this_pass == 0) {
        fanout_stagnant_attempts += attempted_this_pass;
      } else {
        fanout_stagnant_attempts = 0;
      }
      if (!route_todo.empty() &&
          !pnr::fanoutPassMadeProgress(completed_this_pass, advanced_this_pass,
                                       changed_this_pass) &&
          route_todo.size() >= before) {
        ++fanout_no_completion_passes;
      } else {
        fanout_no_completion_passes = 0;
      }
    } else {
      fanout_stagnant_attempts = 0;
      fanout_no_completion_passes = 0;
    }
    basic_no_net_progress_passes = pnr::updateBasicNoNetProgressPasses(
        basic_mode, route_todo.size(), basic_best_remaining,
        basic_no_net_progress_passes);
    const bool basic_net_progress_handoff = pnr::basicNetProgressRequiresHandoff(
        basic_mode, route_todo.size(), basic_no_net_progress_passes,
        route_recursion_limit);
    if (route_stagnant_this_pass) {
      ++stagnant_passes;
    } else {
      stagnant_passes = 0;
    }
    if (moving_focus_inst && attempted_this_pass != 0 &&
        active_this_pass == 0) {
      size_t dropped_complete =
          drop_complete_tasks(route_todo, "inactive focus queue");
      // Focused preemption can invalidate siblings from another source tree.
      // Preserve them for later without letting them abandon this focus.
      size_t deferred_nonincident = pnr::partitionMovingReplacementTasks(
          route_todo,
          [&](const RouteTask &task) {
            return task_matches_moving_focus(task);
          },
          [&](const RouteTask &task) {
            appendUniqueRouteTask(moving_deferred_todo, task);
          });
      if (deferred_nonincident != 0) {
        PNR_LOG1("ROUT",
                 "routeDesign moving: deferred {} non-incident tasks "
                 "generated during focused routing for inst='{}'",
                 deferred_nonincident,
                 moving_focus_inst->makeName(FULL_NAME_LIMIT));
      }
      if (route_todo.empty()) {
        if (moving_focus_complete(*moving_focus_inst)) {
          markMovingClusterFinished(move_finished_insts, moving_focus_inst);
        } else {
          size_t requeued =
              collect_moving_focus_tasks(*moving_focus_inst, route_todo);
          PNR_LOG1("ROUT",
                   "routeDesign moving: inst='{}' inactive queue empty but "
                   "incident routes remain, requeued {} incident tasks",
                   moving_focus_inst->makeName(FULL_NAME_LIMIT), requeued);
          size_t focus_tried =
              move_tried_placements[movingClusterKey(moving_focus_inst)].size();
          if (requeued != 0 && (pnr::movingFocusSliceExhausted(
                                    focus_tried, moving_focus_slice_start,
                                    moving_focus_slice_limit) ||
                                pnr::movingFocusPlacementsExhausted(
                                    focus_tried, moving_focus_retry_limit))) {
            defer_exhausted_moving_focus("inactive incident recovery");
            continue;
          }
          if (requeued != 0) {
            moving_relocate_next = pnr::relocateAfterIncidentRequeue(
                requeued, moving_incident_recoveries);
            moving_force_incident_relocation = moving_relocate_next;
            ++moving_incident_recoveries;
            stagnant_passes = 0;
            moving_no_completion_passes = 0;
            continue;
          }
          moving_blocked_until_epoch[movingClusterKey(moving_focus_inst)] =
              moving_relocation_epoch +
              moving_cooldown_epochs(moving_focus_inst);
        }
        PNR_LOG1("ROUT",
                 "routeDesign moving: focus inst='{}' had {} complete inactive "
                 "tasks, restoring {} deferred tasks",
                 moving_focus_inst->makeName(FULL_NAME_LIMIT), dropped_complete,
                 moving_deferred_todo.size());
        restore_moving_deferred_tasks();
        moving_focus_inst = nullptr;
        stagnant_passes = 0;
        moving_passes = 0;
        moving_no_completion_passes = 0;
        // This focus has no incomplete incident work; select the next
        // deferred endpoint without a global queue sweep.
        moving_relocate_next =
            pnr::movingRelocatesImmediatelyAfterFocus(moving_sources_stage);
        continue;
      }
      bool has_route_into_focus = std::any_of(
          route_todo.begin(), route_todo.end(), [&](const RouteTask &task) {
            return endpoint_matches_moving_focus(task.to);
          });
      bool has_route_out_of_focus = std::any_of(
          route_todo.begin(), route_todo.end(), [&](const RouteTask &task) {
            return endpoint_matches_moving_focus(task.from) &&
                   !endpoint_matches_moving_focus(task.to);
          });
      if (pnr::movingFocusHandsOffToLoads(moving_sources_stage,
                                          has_route_into_focus,
                                          has_route_out_of_focus)) {
        PNR_LOG1("ROUT",
                 "routeDesign moving: focus inst='{}' has only downstream load "
                 "work; preserving its placement and handing off {} routes",
                 moving_focus_inst->makeName(FULL_NAME_LIMIT),
                 route_todo.size());
        markMovingClusterFinished(move_finished_insts, moving_focus_inst);
        restore_moving_deferred_tasks();
        moving_focus_inst = nullptr;
        moving_passes = 0;
        stagnant_passes = 0;
        moving_no_completion_passes = 0;
        // Downstream loads are independent future focuses. Select the next
        // deferred endpoint instead of globally routing the restored queue.
        moving_relocate_next = true;
        continue;
      }
      PNR_LOG1("ROUT",
               "routeDesign moving: focus inst='{}' made no active route "
               "work, dropped_complete={}, active_left={}; retaining the "
               "placement for its bounded retry window with {} deferred tasks",
               moving_focus_inst->makeName(FULL_NAME_LIMIT), dropped_complete,
               route_todo.size(),
               moving_deferred_todo.size());
    }
    if (!route_todo.empty()) {
      moving_no_completion_passes = pnr::updateMovingNoCompletionPasses(
          moving_no_completion_passes, moving_stage, completed_this_pass);
    } else {
      moving_no_completion_passes = 0;
    }
    bool route_blocked_this_pass = active_this_pass == 0;
    bool start_fanout_after_pass = false;
    bool start_moving_sources_after_pass = false;
    bool start_moving_after_pass = false;
    bool fanout_full_stagnant_cycle =
        fanout_stage && !route_todo.empty() &&
        fanout_stagnant_attempts >= route_todo.size() * 16;
    bool fanout_no_completion_exhausted =
        fanout_stage && !route_todo.empty() &&
        fanout_no_completion_passes >= std::max(20, route_recursion_limit * 4);
    bool fanout_low_progress_exhausted =
        fanout_stage && !route_todo.empty() &&
        pnr::fanoutShouldHandOff(stage_pass, completed_this_pass,
                                 route_recursion_limit);
    bool fanout_blocked_with_unfinished =
        fanout_full_stagnant_cycle &&
        (route_blocked_this_pass || stagnant_passes >= 3);
    fanout_blocked_with_unfinished =
        fanout_blocked_with_unfinished || fanout_no_completion_exhausted;
    fanout_blocked_with_unfinished =
        fanout_blocked_with_unfinished || fanout_low_progress_exhausted;
    bool basic_blocked_with_unfinished =
        !fanout_stage && !fanout_seed_repair_active && !moving_stage &&
        !moving_focus_inst &&
        !route_todo.empty() &&
        (route_blocked_this_pass || stagnant_passes >= 3 ||
         basic_net_progress_handoff || route_stage_stagnated ||
         pnr::fanoutSeedRepairPassesExhausted(
             fanout_seed_repair_active, stage_pass,
             route_recursion_limit));
    if (!fanout_stage && !fanout_seed_repair_active && !moving_stage &&
        !moving_focus_inst &&
        (route_blocked_this_pass || stagnant_passes >= 3)) {
      if (route_todo.empty() && !fanout_route_todo.empty()) {
        start_fanout_after_pass = true;
      }
    }
    int moving_focus_pass_limit =
        moving_focus_inst && moving_placement_has_completion
            ? pnr::movingUsefulPlacementRetryLimit(route_recursion_limit,
                                                   route_todo.size())
            : route_recursion_limit;
    if (moving_focus_inst) {
      // Retain useful route growth, but charge every pass to this placement's
      // bounded slice so one wandering suffix cannot monopolize Moving.
      moving_placement_passes = pnr::updateMovingPlacementNoProgressPasses(
          moving_placement_passes, placement_completed_this_pass != 0);
    }
    bool focus_pass_exhausted =
        moving_focus_inst &&
        pnr::movingPlacementPassesExhausted(moving_placement_passes,
                                            moving_focus_pass_limit);
    bool unfocused_moving_pass_exhausted =
        moving_stage && !moving_focus_inst &&
        pnr::movingPlacementPassesExhausted(stage_pass, route_recursion_limit);
    bool unfocused_moving_no_completion_exhausted =
        moving_stage && !moving_focus_inst &&
        moving_no_completion_passes >= route_recursion_limit;
    bool focus_moving_no_completion_exhausted =
        moving_focus_inst &&
        moving_no_completion_passes >=
            pnr::movingFocusNoCompletionLimit(moving_focus_pass_limit);
    bool focus_task_no_progress_exhausted =
        moving_focus_inst &&
        std::any_of(route_todo.begin(), route_todo.end(),
                    [&](const RouteTask &task) {
                      return pnr::movingTaskNoProgressExhausted(
                          task.no_progress_passes, moving_focus_pass_limit);
                    });
    bool focus_retry_window_exhausted =
        focus_moving_no_completion_exhausted ||
        focus_task_no_progress_exhausted || focus_pass_exhausted;
    bool focus_blocked_this_pass = pnr::focusedMovingPassIsBlocked(
        route_blocked_this_pass, focus_retry_window_exhausted);
    bool should_move_unfocused =
        moving_stage && !moving_focus_inst &&
        (route_blocked_this_pass || stagnant_passes >= 3 ||
         unfocused_moving_pass_exhausted ||
         unfocused_moving_no_completion_exhausted);
    bool should_move_focus =
        moving_focus_inst &&
        (pnr::focusedMovingShouldRelocate(
            focus_blocked_this_pass, stagnant_passes, moving_focus_pass_limit,
            focus_moving_no_completion_exhausted ||
                focus_task_no_progress_exhausted,
            focus_pass_exhausted));
    if (pnr::movingStageShouldRelocate(
            restored_moving_focus, moving_focus_inst != nullptr,
            route_todo.size(), before, should_move_unfocused,
            should_move_focus)) {
      start_moving_after_pass = true;
    }
    if (route_todo.empty() && fanout_stage && !moving_stage &&
        !fanout_route_todo.empty()) {
      fanout_stage = false;
      start_fanout_after_pass = true;
    }
    if (moving_sources_stage &&
        pnr::movingSourcesReachedZero(route_todo.size(),
                                      moving_deferred_todo.size() +
                                          moving_source_retry_todo.size(),
                                      moving_focus_inst != nullptr)) {
      moving_sources_completed = true;
      moving_sources_stage = false;
      moving_stage = false;
      start_fanout_after_pass = !fanout_route_todo.empty();
      start_moving_after_pass =
          fanout_route_todo.empty() && !moving_destination_todo.empty();
      PNR_LOG1("ROUT",
               "routeDesign Moving sources completed with zero trunks; "
               "releasing {} suffixes to Fanouts",
               fanout_route_todo.size());
    }
    if (route_todo.empty() && !fanout_stage && !moving_stage &&
        !moving_sources_completed) {
      start_moving_sources_after_pass = true;
    } else if (route_todo.empty() && !fanout_stage && !moving_stage &&
               !fanout_route_todo.empty()) {
      start_fanout_after_pass = true;
    }
    if (moving_sources_completed && route_todo.empty() &&
        fanout_route_todo.empty() && !moving_stage &&
        !moving_destination_todo.empty()) {
      start_moving_after_pass = true;
    }

    auto route_end_time = std::chrono::steady_clock::now();
    double route_seconds = elapsedSeconds(epoch_start_time, route_end_time);
    double epoch_seconds = stage_time_charge.finish();
    double total_seconds = elapsedSeconds(route_start_time, route_end_time);

    route_stage_deadline_enabled = false;
    ++pass_stage_report.passes;
    pass_stage_report.remaining_tasks = route_todo.size();
    pass_stage_report.attempted += attempted_this_pass;
    pass_stage_report.completed += completed_this_pass;
    pass_stage_report.active += active_this_pass;
    pass_stage_report.advanced += advanced_this_pass;
    pass_stage_report.changed += changed_this_pass;
    pass_stage_report.searches += route_stats.route_searches;
    pass_stage_report.pops += route_stats.search_pops;
    pass_stage_report.edge_trials += route_stats.edge_trials;
    pass_stage_report.edge_accepted += route_stats.edge_accepted;
    pass_stage_report.reject_busy += route_stats.edge_rejected_busy;
    pass_stage_report.reject_target += route_stats.edge_rejected_no_target;
    pass_stage_report.reject_deadend += route_stats.edge_rejected_deadend +
                                        route_stats.edge_rejected_src_deadend;
    pass_stage_report.preempt_attempts += route_stats.preempt_attempts;
    pass_stage_report.preempt_success += route_stats.preempt_success;
    pass_stage_report.preempt_complete_victims +=
        route_stats.preempt_complete_victims;
    pass_stage_report.preempt_partial_victims +=
        route_stats.preempt_partial_victims;
    pass_stage_report.preempt_removed_fragments +=
        route_stats.preempt_removed_fragments;
    pass_stage_report.preempt_takeoff_complete_victims +=
        route_stats.preempt_takeoff_complete_victims;
    pass_stage_report.preempt_takeoff_partial_victims +=
        route_stats.preempt_takeoff_partial_victims;
    pass_stage_report.preempt_bridge_complete_victims +=
        route_stats.preempt_bridge_complete_victims;
    pass_stage_report.preempt_bridge_partial_victims +=
        route_stats.preempt_bridge_partial_victims;
    pass_stage_report.preempt_grounding_complete_victims +=
        route_stats.preempt_grounding_complete_victims;
    pass_stage_report.preempt_grounding_partial_victims +=
        route_stats.preempt_grounding_partial_victims;
    pass_stage_report.deadend_marks += route_stats.src_deadend_marks;

    const char *stage_name =
        moving_stage
            ? (moving_sources_stage ? "Moving sources"
                                    : "Moving destinations")
            : ((fanout_stage || fanout_seed_repair_active)
                   ? "Fanouts routing"
                   : "Basic routing");
    PNR_LOG1(
        "ROUT",
        "routeDesign pass: {}, stage_pass={}, stage={}, todo: {} -> {}, "
        "completed={}, active={}, advanced={}, changed={}, attempted={}/{}",
        pass + 1, stage_pass, stage_name, before, route_todo.size(),
        completed_this_pass, active_this_pass, advanced_this_pass,
        changed_this_pass, attempted_this_pass, task_limit_this_pass);
    // Explicit offline audit: inspect a consistent pass boundary, never repair
    // leases or change routing decisions. Stop deliberately after the report.
    if (const char* audit_pass = std::getenv("SCALEPNR_AUDIT_FANOUT_PASS");
        audit_pass && fanout_stage && !moving_stage &&
        stage_pass == std::atoi(audit_pass)) {
      const char* audit_path = std::getenv("SCALEPNR_CONGESTION_LOG");
      std::ofstream audit_out(audit_path ? audit_path : "routing_congestion.log");
      if (!audit_out) throw std::runtime_error("cannot write full congestion audit");
      std::unordered_set<rtl::Net*> unique_nets;
      for (rtl::Module& module : tech->design.modules)
        for (rtl::Net& net : module.nets) unique_nets.insert(&net);
      for (const auto& [source, nets] : source_route_nets)
        unique_nets.insert(nets.begin(), nets.end());
      std::vector<rtl::Net*> nets(unique_nets.begin(), unique_nets.end());
      fpga::CongestionAudit total;
      size_t tiles = 0;
      PNR_LOG1("ROUT", "Full congestion audit starting: Fanouts pass={}, nets={}",
               stage_pass, nets.size());
      for (int y = 0; y < fpga->size_height; ++y) {
        for (int x = 0; x < fpga->size_width; ++x) {
          auto* tile = fpga->getTile(x, y);
          if (!tile) continue;
          auto result = fpga::auditTileCongestion(*tile, audit_out, nets);
          ++tiles;
          total.leased_nodes += result.leased_nodes;
          total.orphan_leases += result.orphan_leases;
          total.missing_leases += result.missing_leases;
          total.missing_registrations += result.missing_registrations;
          total.stale_registrations += result.stale_registrations;
        }
        audit_out.flush();
        if (y % 10 == 0)
          PNR_LOG1("ROUT", "Full congestion audit row={}/{}, tiles={}, unowned={}, missing={}",
                   y, fpga->size_height, tiles, total.orphan_leases, total.missing_leases);
      }
      audit_out << "FULL_AUDIT stage_pass=" << stage_pass << " tiles=" << tiles
                << " leased=" << total.leased_nodes << " unowned=" << total.orphan_leases
                << " missing_lease=" << total.missing_leases
                << " missing_registration=" << total.missing_registrations
                << " stale_registration=" << total.stale_registrations << '\n';
      audit_out.close();
      PNR_LOG1("ROUT", "Full congestion audit finished: tiles={}, leased={}, unowned={}, missing={}, missing_registration={}, stale_registration={}",
               tiles, total.leased_nodes, total.orphan_leases, total.missing_leases,
               total.missing_registrations, total.stale_registrations);
      throw std::runtime_error("diagnostic stop after full congestion audit (not a routing pass)");
    }
    if (envFlagEnabled("SCALEPNR_ROUTE_PASS_DETAIL")) {
      size_t empty_routes = 0;
      size_t takeoff_routes = 0;
      size_t prefix_routes = 0;
      for (const RouteTask &task : route_todo) {
        const std::vector<Wire> *route =
            findBoundRoute(task.net, task.from, task.to, task.from_port,
                           task.to_port, task.net_name);
        size_t crossbars = routeCrossbarFragments(route);
        if (!route || route->empty() || crossbars == 0) {
          ++empty_routes;
        } else if (crossbars == 1) {
          ++takeoff_routes;
        } else {
          ++prefix_routes;
        }
      }
      PNR_LOG1(
          "ROUT",
          "routeDesign pass state: stage={}, empty={}, takeoff={}, prefix={}, "
          "preempt={}/{}, victims(complete={},partial={},fragments={}), "
          "by_kind(takeoff={}/{},bridge={}/{},grounding={}/{}), "
          "deadend_bits={}",
          stage_name, empty_routes, takeoff_routes, prefix_routes,
          route_stats.preempt_success, route_stats.preempt_attempts,
          route_stats.preempt_complete_victims,
          route_stats.preempt_partial_victims,
          route_stats.preempt_removed_fragments,
          route_stats.preempt_takeoff_complete_victims,
          route_stats.preempt_takeoff_partial_victims,
          route_stats.preempt_bridge_complete_victims,
          route_stats.preempt_bridge_partial_victims,
          route_stats.preempt_grounding_complete_victims,
          route_stats.preempt_grounding_partial_victims,
          countDeadendBits(route_src_deadends));
      PNR_LOG1(
          "ROUT",
          "routeDesign scheduler: pass={}, stage_pass={}, stage={}, "
          "stagnant={}, stagnant_now={}, blocked={}, basic_no_net_progress={}, "
          "fanout_no_completion={}, fanout_no_completion_exhausted={}, "
          "focus_exhausted={}, unfocused_exhausted={}, "
          "no_completion_exhausted={}, focus_no_completion_exhausted={}, "
          "no_completion_passes={}, focus_limit={}, moving_passes={}, "
          "placement_passes={}, focus='{}', pending={}, fanout_deferred={}, "
          "moving_deferred={}, fanout_stage={}, moving_stage={}, "
          "next_fanout={}, next_moving={}",
          pass + 1, stage_pass, stage_name, stagnant_passes,
          route_stagnant_this_pass, route_blocked_this_pass,
          basic_no_net_progress_passes, fanout_no_completion_passes,
          fanout_no_completion_exhausted, focus_pass_exhausted,
          unfocused_moving_pass_exhausted,
          unfocused_moving_no_completion_exhausted,
          focus_moving_no_completion_exhausted, moving_no_completion_passes,
          moving_focus_pass_limit, moving_passes, moving_placement_passes,
          moving_focus_inst ? moving_focus_inst->makeName(FULL_NAME_LIMIT)
                            : std::string{},
          pending_route_todo.size(), fanout_route_todo.size(),
          moving_deferred_todo.size(), fanout_stage, moving_stage,
          start_fanout_after_pass, start_moving_after_pass);
      PNR_LOG1(
          "ROUT",
          "routeDesign time: pass={}, epoch={:.3f}s, route={:.3f}s, "
          "total={:.3f}s, tasks_per_sec={:.1f}, trials_per_sec={:.1f}",
          pass + 1, epoch_seconds, route_seconds, total_seconds,
          route_seconds > 0.0
              ? static_cast<double>(route_stats.task_attempts) / route_seconds
              : 0.0,
          route_seconds > 0.0
              ? static_cast<double>(route_stats.edge_trials) / route_seconds
              : 0.0);
      PNR_LOG1("ROUT",
               "routeDesign stats: tasks={}, new={}, cont={}, done={}, "
               "partial_start={}, partial_adv={}, rip={}, backtry={}, "
               "backok={}, backfrag={}, rollback={}, preempt={}/{}, "
               "no_src={}/depth0:{} joint_path:{}, src_deadend={}, "
               "src_deadend_tiles={}, src_deadend_bits={}, failed={}, "
               "searches={}, pops={}, deadend_tile_pops={}, "
               "src_deadend_tile_pops={}, edge_trials={}, edge_ok={}, "
               "reject(name={},busy={},busy_dst={},busy_src={},busy_local={},"
               "target={},deadend={},src_deadend={})",
               route_stats.task_attempts, route_stats.new_attempts,
               route_stats.continuation_attempts, route_stats.completed,
               route_stats.partial_started, route_stats.partial_advanced,
               route_stats.rip_backs, route_stats.backstep_attempts,
               route_stats.backstep_success, route_stats.backstep_fragments,
               route_stats.commit_rollbacks, route_stats.preempt_success,
               route_stats.preempt_attempts, route_stats.no_src_nodes,
               route_stats.no_src_nodes_depth0,
               route_stats.no_src_nodes_with_joint_path,
               route_stats.src_deadend_marks, route_src_deadends.size(),
               countDeadendBits(route_src_deadends), route_stats.failed,
               route_stats.route_searches, route_stats.search_pops,
               route_stats.pops_on_deadend_tile,
               route_stats.pops_on_src_deadend_tile, route_stats.edge_trials,
               route_stats.edge_accepted, route_stats.edge_rejected_no_name,
               route_stats.edge_rejected_busy,
               route_stats.edge_rejected_busy_dst,
               route_stats.edge_rejected_busy_src,
               route_stats.edge_rejected_busy_local,
               route_stats.edge_rejected_no_target,
               route_stats.edge_rejected_deadend,
               route_stats.edge_rejected_src_deadend);
      PNR_LOG1("ROUT",
               "routeDesign outcomes: already_done={}, "
               "new(done={},partial={},empty={},failed={}), "
               "cont(done={},advanced={},noadv={},rip={},empty={})",
               route_stats.already_complete, route_stats.new_completed,
               route_stats.new_partial, route_stats.new_empty,
               route_stats.new_failed, route_stats.cont_completed,
               route_stats.cont_advanced, route_stats.cont_no_advance,
               route_stats.cont_failed_rip, route_stats.cont_failed_empty);
      PNR_LOG1("ROUT",
               "routeDesign depth: pops={}, trials={}, ok={}, back={}, "
               "rollback={}, deadend={}, partial={}, done={}",
               statArray(route_stats.pops_by_depth),
               statArray(route_stats.trials_by_depth),
               statArray(route_stats.accepted_by_depth),
               statArray(route_stats.backsteps_by_depth),
               statArray(route_stats.rollbacks_by_depth),
               statArray(route_stats.deadends_by_depth),
               statArray(route_stats.partial_by_depth),
               statArray(route_stats.completed_by_depth));
      if (route_stats.has_last_busy) {
        PNR_LOG1("ROUT",
                 "routeDesign last_busy: coord=({},{}), depth={}, local={}, "
                 "src={}, src_jump={}, dst_jump={}, local_mask={}",
                 route_stats.last_busy_coord.x, route_stats.last_busy_coord.y,
                 route_stats.last_busy_depth, route_stats.last_busy_local,
                 route_stats.last_busy_src,
                 maskString(route_stats.last_busy_src_mask),
                 maskString(route_stats.last_busy_dst_mask),
                 maskString(route_stats.last_busy_local_mask));
      }
      if (route_stats.has_last_no_src) {
        PNR_LOG1("ROUT",
                 "routeDesign last_no_src: coord=({},{}), depth={}, local={}, "
                 "joint_mask={}",
                 route_stats.last_no_src_coord.x,
                 route_stats.last_no_src_coord.y, route_stats.last_no_src_depth,
                 route_stats.last_no_src_local,
                 maskString(route_stats.last_no_src_joint_mask));
      }
      if (route_stats.has_last_deadend_mark) {
        PNR_LOG1("ROUT", "routeDesign last_deadend: net='{}', src=({},{}):{}",
                 route_stats.last_deadend_net,
                 route_stats.last_src_deadend_coord.x,
                 route_stats.last_src_deadend_coord.y,
                 route_stats.last_src_deadend_node);
      }
      if (envFlagEnabled("SCALEPNR_ROUTE_PROFILE")) {
        PNR_LOG1(
            "ROUT",
            "routeDesign hotpath: setup_ms={:.3f}, name_ms={:.3f}, "
            "conflict_ms={:.3f}, copy_ms={:.3f}, lease_ms={:.3f}, "
            "resolve_ms={:.3f}, target_ms={:.3f}, state_ms={:.3f}, "
            "bf_loop_ms={:.3f}, bf_dock_ms={:.3f}, bf_commit_ms={:.3f}, "
            "bf_mat_ms={:.3f}, passthrough_ms={:.3f}, find_ms={:.3f}, "
            "candidates_ms={:.3f}, endpoint_ms={:.3f}, direct_ms={:.3f}, "
            "best_first_ms={:.3f}, route_ms={:.3f}, attach_ms={:.3f}",
            static_cast<double>(route_stats.edge_order_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_name_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_conflict_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_cb_copy_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_lease_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_resolve_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_target_pin_ns) / 1000000.0,
            static_cast<double>(route_stats.edge_state_ns) / 1000000.0,
            static_cast<double>(route_stats.best_first_loop_ns) / 1000000.0,
            static_cast<double>(route_stats.best_first_dock_ns) / 1000000.0,
            static_cast<double>(route_stats.best_first_commit_ns) / 1000000.0,
            static_cast<double>(route_stats.best_first_materialize_ns) /
                1000000.0,
            static_cast<double>(route_stats.task_passthrough_ns) / 1000000.0,
            static_cast<double>(route_stats.task_find_ns) / 1000000.0,
            static_cast<double>(route_stats.task_candidate_ns) / 1000000.0,
            static_cast<double>(route_stats.task_endpoint_ns) / 1000000.0,
            static_cast<double>(route_stats.task_direct_ns) / 1000000.0,
            static_cast<double>(route_stats.task_best_first_ns) / 1000000.0,
            static_cast<double>(route_stats.task_route_ns) / 1000000.0,
            static_cast<double>(route_stats.task_attach_ns) / 1000000.0);
      }
    }
    int basic_rest_dump_count = -1;
    if (const char *dump_count_env =
            std::getenv("SCALEPNR_DEBUG_BASIC_REST_COUNT")) {
      char *end = nullptr;
      long parsed = std::strtol(dump_count_env, &end, 10);
      if (end != dump_count_env && parsed > 0 && parsed < 100000) {
        basic_rest_dump_count = static_cast<int>(parsed);
      }
    }
    if (basic_mode && !basic_rest_diagnostics_dumped && !route_todo.empty() &&
        static_cast<int>(route_todo.size()) == basic_rest_dump_count) {
      basic_rest_diagnostics_dumped = true;
      dumpBasicUnfinishedDiagnosticsLog(route_todo, stage_pass,
                                        "basic rest-count diagnostics");
    }
    if (!fanout_stage && !fanout_seed_repair_active && !moving_stage) {
      if (const char *debug_stop_pass =
              std::getenv("SCALEPNR_DEBUG_STOP_BASIC_PASS")) {
        int debug_pass = std::atoi(debug_stop_pass);
        if (debug_pass > 0 && stage_pass >= debug_pass) {
          std::string debug_dump = "/tmp/scalepnr_basic_debug_state.txt";
          bool compact_debug =
              envFlagEnabled("SCALEPNR_DEBUG_BASIC_COMPACT");
          if (!compact_debug) {
            dumpFullRoutingState(debug_dump, route_todo, fanout_route_todo,
                                 pending_route_todo, moving_deferred_todo);
            dumpBasicUnfinishedDiagnosticsLog(route_todo, stage_pass,
                                              "debug stop pass");
          }
          size_t debug_task_limit = route_todo.size();
          if (compact_debug) {
            debug_task_limit = std::min<size_t>(debug_task_limit, 16);
          }
          for (size_t task_index = 0; task_index < debug_task_limit;
               ++task_index) {
            const RouteTask &task = route_todo[task_index];
            PNR_LOG1(
                "ROUT",
                "routeDesign Basic debug task[{}]: net='{}', from='{}' type='{}' "
                "port='{}' tile=({},{})/{}, to='{}' type='{}' port='{}' "
                "tile=({},{})/{}, attempt={}, fanout={}",
                task_index, task.net_name,
                task.from ? task.from->makeName(FULL_NAME_LIMIT)
                          : std::string{},
                task.from && task.from->cell_ref.peer
                    ? task.from->cell_ref->type
                    : std::string{},
                task.from_port,
                task.from && task.from->tile.peer ? task.from->tile->coord.x
                                                  : -1,
                task.from && task.from->tile.peer ? task.from->tile->coord.y
                                                  : -1,
                task.from ? task.from->pos : -1,
                task.to ? task.to->makeName(FULL_NAME_LIMIT) : std::string{},
                task.to && task.to->cell_ref.peer ? task.to->cell_ref->type
                                                  : std::string{},
                task.to_port,
                task.to && task.to->tile.peer ? task.to->tile->coord.x : -1,
                task.to && task.to->tile.peer ? task.to->tile->coord.y : -1,
                task.to ? task.to->pos : -1, task.attempt, task.fanout);
          }
          failRouting(route_todo.empty()
                                          ? nullptr
                                          : &route_todo.front(),
                                      "Basic debug stop");
          PNR_ASSERT(false,
                     "routeDesign stopped after Basic pass {} with {} "
                     "unfinished route tasks{}",
                     stage_pass, route_todo.size(),
                     compact_debug ? "" :
                                     std::format("; state dumped to '{}'",
                                                 debug_dump));
        }
      }
    }
    if (basic_blocked_with_unfinished) {
      for (const RouteTask &task : route_todo) {
        PNR_ASSERT(!task.net || !task.net->distributed_source,
                   "constant route '{}' escaped into Basic recovery",
                   task.net_name);
      }
    }
    if (basic_blocked_with_unfinished) {
      std::string debug_dump = "/tmp/scalepnr_basic_blocked_state.txt";
      if (pnr::routeStateDumpEnabled(
              envFlagEnabled("SCALEPNR_SKIP_TIMEOUT_DUMP"),
              envFlagEnabled("SCALEPNR_SKIP_STATE_DUMP"))) {
        dumpFullRoutingState(debug_dump, route_todo, fanout_route_todo,
                             pending_route_todo, moving_deferred_todo);
      } else {
        debug_dump = "disabled by routing state-dump policy";
      }
      dumpBasicUnfinishedDiagnosticsLog(route_todo, stage_pass,
                                        "basic blocked");
      const RouteTask &first_task = route_todo.front();
      PNR_LOG1(
          "ROUT",
          "routeDesign Basic routing blocked with {} unfinished trunk tasks; "
          "state dump='{}'; conserving work for later stages; first net='{}' "
          "from='{}'/'{}' to='{}'/'{}'",
          route_todo.size(), debug_dump, first_task.net_name,
          first_task.from ? first_task.from->makeName(FULL_NAME_LIMIT)
                          : std::string{},
          first_task.from_port,
          first_task.to ? first_task.to->makeName(FULL_NAME_LIMIT)
                        : std::string{},
          first_task.to_port);
    }
    if (fanout_blocked_with_unfinished && !route_stage_stagnated) {
      const RouteTask &first_task = route_todo.front();
      std::string first_net_name = first_task.net_name;
      std::string first_from_name =
          first_task.from ? first_task.from->makeName(FULL_NAME_LIMIT)
                          : std::string{};
      std::string first_from_port = first_task.from_port;
      std::string first_to_name = first_task.to
                                      ? first_task.to->makeName(FULL_NAME_LIMIT)
                                      : std::string{};
      std::string first_to_port = first_task.to_port;
      size_t blocked_task_count = route_todo.size();
      size_t blocked_stagnant_attempts = fanout_stagnant_attempts;
      std::string debug_dump = "/tmp/scalepnr_fanout_blocked_state.txt";
      if (pnr::routeStateDumpEnabled(
              envFlagEnabled("SCALEPNR_SKIP_TIMEOUT_DUMP"),
              envFlagEnabled("SCALEPNR_SKIP_STATE_DUMP"))) {
        dumpFullRoutingState(debug_dump, route_todo, fanout_route_todo,
                             pending_route_todo, moving_deferred_todo);
      } else {
        debug_dump = "disabled by routing state-dump policy";
      }
      logTargetTileEntryTable(first_task, "Fanouts blocked");
      std::vector<RouteTask> seed_tasks;
      std::vector<RouteTask> deferred_fanout_tasks;
      for (RouteTask &task : route_todo) {
        if (task.fanout) {
          deferred_fanout_tasks.push_back(std::move(task));
        } else {
          seed_tasks.push_back(std::move(task));
        }
      }
      if (!seed_tasks.empty()) {
        route_todo = std::move(seed_tasks);
        fanout_route_todo.insert(
            fanout_route_todo.end(),
            std::make_move_iterator(deferred_fanout_tasks.begin()),
            std::make_move_iterator(deferred_fanout_tasks.end()));
        fanout_stage = false;
        moving_sources_stage = false;
        moving_stage = false;
        fanout_seed_repair_active = true;
        route_deadends_enabled = false;
        applyRouteDeadends({});
        resetPassPreemptionState();
        stagnant_passes = 0;
        basic_no_net_progress_passes = 0;
        fanout_no_completion_passes = 0;
        fanout_stagnant_attempts = 0;
        stage_pass = 0;
        PNR_LOG1("ROUT",
                 "routeDesign Fanouts routing blocked with {} unfinished tasks "
                 "after {} stagnant attempts; state dumped to '{}'; repairing "
                 "{} demoted seed tasks inside Fanouts before resuming {} "
                 "suffixes; first "
                 "net='{}' from='{}'/'{}' to='{}'/'{}'",
                 blocked_task_count, blocked_stagnant_attempts, debug_dump,
                 route_todo.size(), deferred_fanout_tasks.size(),
                 first_net_name, first_from_name, first_from_port,
                 first_to_name, first_to_port);
      } else {
        route_todo = std::move(deferred_fanout_tasks);
        start_moving_after_pass = true;
        PNR_LOG1("ROUT",
                 "routeDesign Fanouts routing blocked with {} unfinished tasks "
                 "after {} stagnant attempts; state dumped to '{}'; first "
                 "net='{}' from='{}'/'{}' to='{}'/'{}'; switching to Moving",
                 route_todo.size(), blocked_stagnant_attempts, debug_dump,
                 first_net_name, first_from_name, first_from_port,
                 first_to_name, first_to_port);
      }
    }
    bool stage_timeout_reached =
        route_stage_deadline_expired ||
        pnr::routeStageTimeoutIsFatal(
                                      std::chrono::duration<double>(
                                          std::chrono::steady_clock::now() -
                                          stage_wall_starts[pass_stage_index])
                                          .count(),
                                      pass_stage_timeout_seconds,
                                      !route_todo.empty());
    bool basic_timeout_handoff = stage_timeout_reached &&
                                 pass_stage_index == BASIC_STAGE_INDEX &&
                                 !route_todo.empty();
    bool basic_stage_handoff = pnr::basicStageRequiresHandoff(
        basic_timeout_handoff, basic_congestion_handoff,
        basic_blocked_with_unfinished);
    bool fanout_timeout_handoff = stage_timeout_reached &&
                                  pass_stage_index == FANOUT_STAGE_INDEX &&
                                  !route_todo.empty();
    const bool fanout_budget_handoff = pnr::fanoutStageBudgetRequiresHandoff(
        pass_stage_index == FANOUT_STAGE_INDEX, fanout_timeout_handoff,
        route_stage_stagnated);
    if (basic_stage_handoff) {
      const size_t unfinished_basic = route_todo.size();
      if (basic_congestion_handoff) {
        dumpBasicUnfinishedDiagnosticsLog(
            route_todo, stage_pass, "Basic congestion-growth handoff", false);
      }
      // Deferred siblings may still name the logical source that existed
      // before a Generic seed inserted its physical passthrough endpoint.
      for (RouteTask &fanout_task : fanout_route_todo) {
        canonicalizeRouteTaskSource(fanout_task);
      }
      pnr::prepareMovingSourceTasks(route_todo);
      deduplicateRouteTasks(route_todo);
      deduplicateRouteTasks(fanout_route_todo);
      pass_stage_report.timed_out = basic_timeout_handoff;
      pass_stage_report.stagnated = route_stage_stagnated;
      pass_stage_report.remaining_tasks = unfinished_basic;
      route_stage_deadline_expired = false;
      start_moving_sources_after_pass = true;
      PNR_LOG1(
          "ROUT",
          "routeDesign Basic routing {} with {} trunks; "
          "parked {} suffixes until Moving sources reaches zero",
          basic_timeout_handoff
              ? "budget exhausted"
              : (route_stage_stagnated ? "watchdog stagnated"
                 : basic_net_progress_handoff ? "no net progress"
                 : basic_congestion_handoff ? "congestion growth detected"
                                            : "blocked"),
          unfinished_basic, fanout_route_todo.size());
    } else if (fanout_budget_handoff) {
      const size_t deferred_fanout_tasks = pnr::deferFanoutTimeoutTasks(
          fanout_route_todo, moving_destination_todo,
          [](std::vector<RouteTask> &tasks, const RouteTask &task) {
            appendUniqueRouteTask(tasks, task);
          });
      start_moving_after_pass = true;
      pass_stage_report.timed_out = fanout_timeout_handoff;
      pass_stage_report.stagnated = route_stage_stagnated;
      pass_stage_report.remaining_tasks = route_todo.size();
      route_stage_deadline_expired = false;
      PNR_LOG1("ROUT",
               "routeDesign Fanouts routing {} with {} active "
               "and {} deferred tasks; handing all work to Moving "
               "destinations",
               route_stage_stagnated ? "stagnated" : "budget exhausted",
               route_todo.size(), deferred_fanout_tasks);
    } else if (pnr::routeStageTimeoutRequiresFailure(
                   stage_timeout_reached,
                   basic_stage_handoff || fanout_timeout_handoff)) {
      fail_stage_timeout(pass_stage_index);
    }
    if (start_moving_sources_after_pass) {
      PNR_ASSERT(!moving_sources_completed,
                 "Moving sources restarted after reaching zero trunks");
      moving_sources_stage = true;
      moving_stage = true;
      fanout_stage = false;
      fanout_preemption_enabled = true;
      route_deadends_enabled = false;
      applyRouteDeadends({});
      moving_source_retry_todo.clear();
      moving_source_prefix_releases_since_generic = 0;
      moving_source_moves_since_generic = 0;
      moving_source_changed_routes_since_generic = 0;
      moving_source_generic_recovery_pending = false;
      // Source recovery starts with destination-to-source route probing. A
      // generic retry is useful only after a relocation frees new capacity.
      moving_relocate_next = pnr::movingStageStartsWithRelocation(
          true, !route_todo.empty());
      stagnant_passes = 0;
      moving_no_completion_passes = 0;
      stage_pass = 0;
      RouteStageReport &source_report =
          stage_reports[MOVING_SOURCES_STAGE_INDEX];
      if (!source_report.started) {
        source_report.started = true;
        source_report.start_tasks = route_todo.size();
      }
      PNR_LOG1("ROUT",
               "routeDesign stage: Moving sources, trunks={}, "
               "parked_suffixes={}",
               route_todo.size(), fanout_route_todo.size());
      if (route_todo.empty()) {
        source_report.remaining_tasks = 0;
        moving_sources_completed = true;
        moving_sources_stage = false;
        moving_stage = false;
        start_fanout_after_pass = !fanout_route_todo.empty();
        start_moving_after_pass =
            fanout_route_todo.empty() && !moving_destination_todo.empty();
        PNR_LOG1("ROUT",
                 "routeDesign Moving sources completed with zero trunks; "
                 "releasing {} suffixes to Fanouts",
                 fanout_route_todo.size());
      }
    }
    if (start_fanout_after_pass) {
      std::vector<RouteTask> ready_fanouts;
      std::vector<RouteTask> missing_seed_fanouts;
      std::unordered_set<std::string> promoted_sources;
      size_t promoted_missing_seeds = 0;
      std::vector<RouteTask> deferred_fanouts = std::move(fanout_route_todo);
      fanout_route_todo.clear();
      for (RouteTask &task : deferred_fanouts) {
        canonicalizeRouteTaskSource(task);
        bool has_seed = task.from &&
                        sourceTreeHasCompleteExit(*task.from, task.from_port);
        const std::vector<Wire> *queued_route =
            findBoundRoute(task.net, task.from, task.to, task.from_port,
                           task.to_port, task.net_name);
        bool queued_route_complete =
            queued_route && routeIsComplete(*queued_route);
        if (!pnr::fanoutWaitsForGenericSeed(true, task.fanout,
                                            queued_route_complete, has_seed)) {
          ready_fanouts.push_back(std::move(task));
          continue;
        }
        missing_seed_fanouts.push_back(std::move(task));
      }

      if (moving_sources_completed && !missing_seed_fanouts.empty()) {
        failRouting(&missing_seed_fanouts.front(),
                                    "Fanout missing Generic trunk");
      }
      PNR_ASSERT(
          !moving_sources_completed ||
              pnr::fanoutMayStartAfterMovingSources(
                  moving_sources_completed, missing_seed_fanouts.size()),
          "Fanouts started after Moving sources but {} suffixes still lack a "
          "completed trunk",
          missing_seed_fanouts.size());

      if (!missing_seed_fanouts.empty()) {
        // Build every missing physical source trunk before consuming any
        // Fanout work; otherwise completed branches congest later trunks.
        for (RouteTask &task : ready_fanouts) {
          appendUniqueRouteTask(fanout_route_todo, task);
        }
        for (RouteTask &task : missing_seed_fanouts) {
          std::string key = sourceRouteKey(task.from, task.from_port);
          if (promoted_sources.contains(key)) {
            appendUniqueRouteTask(fanout_route_todo, task);
            continue;
          }
          RouteTask seed_task = task;
          seed_task.fanout = false;
          seed_task.attempt = 0;
          promoted_sources.insert(key);
          appendUniqueRouteTask(route_todo, seed_task);
          ++promoted_missing_seeds;
          logRouteTaskDecision("stage.promote_missing_seed", seed_task,
                               "deferred Fanout source has no Generic trunk; "
                               "route one seed before Fanout stage");
        }
        fanout_stage = false;
        moving_sources_stage = false;
        moving_stage = false;
        fanout_seed_repair_active = true;
        moving_focus_inst = nullptr;
        route_deadends_enabled = false;
        applyRouteDeadends({});
        resetPassPreemptionState();
        stagnant_passes = 0;
        basic_no_net_progress_passes = 0;
        fanout_no_completion_passes = 0;
        moving_no_completion_passes = 0;
        stage_pass = 0;
        PNR_LOG1("ROUT",
                 "routeDesign Basic promoted deferred fanout seeds: "
                 "promoted={}, remaining_fanouts={}, basic_tasks={}",
                 promoted_missing_seeds, fanout_route_todo.size(),
                 route_todo.size());
      } else if (pnr::fanoutStageCanStart(ready_fanouts.size(),
                                          missing_seed_fanouts.size())) {
        route_todo.insert(route_todo.end(),
                          std::make_move_iterator(ready_fanouts.begin()),
                          std::make_move_iterator(ready_fanouts.end()));
        fanout_stage = true;
        fanout_preemption_enabled = true;
        moving_sources_stage = false;
        moving_stage = false;
        fanout_seed_repair_active = false;
        moving_focus_inst = nullptr;
        route_deadends_enabled = false;
        applyRouteDeadends({});
        resetPassPreemptionState();
        stagnant_passes = 0;
        basic_no_net_progress_passes = 0;
        fanout_no_completion_passes = 0;
        moving_no_completion_passes = 0;
        stage_pass = 0;
        PNR_LOG1("ROUT",
                 "routeDesign stage: Fanouts routing, tasks={}, "
                 "deferred_missing_seed={}",
                 route_todo.size(), fanout_route_todo.size());
      }
    }
    if (start_moving_after_pass && moving_sources_stage) {
      moving_relocate_next = true;
      stagnant_passes = 0;
      moving_no_completion_passes = 0;
      stage_pass = 0;
      start_moving_after_pass = false;
    }
    if (start_moving_after_pass) {
      bool entering_moving_stage = !moving_stage || moving_sources_stage;
      if (!moving_sources_stage && !moving_destination_todo.empty()) {
        for (RouteTask &task : moving_destination_todo) {
          appendUniqueRouteTask(route_todo, task);
        }
        moving_destination_todo.clear();
      }
      if (!moving_stage) {
        // Large designs can hand tens of thousands of tasks to Moving. Keep
        // stdout bounded while retaining a deterministic diagnostic sample.
        dumpBasicUnfinishedDiagnosticsLog(
            route_todo, stage_pass, "Fanout handoff to Moving", false);
        const char *probe_text = std::getenv("SCALEPNR_DEBUG_DOCK_PROBE");
        int probe_x = -1;
        int probe_y = -1;
        int probe_pin = -1;
        if (probe_text && std::sscanf(probe_text, "%d,%d,%d", &probe_x,
                                      &probe_y, &probe_pin) == 3) {
          for (const RouteTask &task : route_todo) {
            if (!routeTaskDebugMatches(task)) {
              continue;
            }
            const std::vector<Wire> *route =
                findBoundRoute(task.net, task.from, task.to, task.from_port,
                               task.to_port, task.net_name);
            if (!route) {
              continue;
            }
            std::vector<Wire> retained_prefix;
            for (const Wire &fragment : *route) {
              if (!fragment.shared) {
                break;
              }
              retained_prefix.push_back(fragment);
            }
            const std::vector<Wire> &probe_route =
                retained_prefix.empty() ? *route : retained_prefix;
            Tile *anchor = nullptr;
            int anchor_dst = -1;
            std::string anchor_wire;
            Tile *target = fpga::Device::current().getTile(probe_x, probe_y);
            if (!target ||
                !partialRouteEndpoint(probe_route, anchor, anchor_dst,
                                      anchor_wire) ||
                !anchor || probe_pin < 0 || probe_pin >= CB_MAX_NODES) {
              continue;
            }
            DockingResult probe =
                dockGrounding(*anchor, anchor_dst, anchor_wire, *target,
                              NodeMask{0, 1} << probe_pin, 5, 5, true);
            PNR_LOG1(
                "ROUT",
                "routeDesign focused docking probe: net='{}', retained={}, "
                "anchor=({},{})/{} '{}', target=({},{})/local={}, success={}, "
                "seeds={}, entries={}, busy={}, fpush={}, bpush={}",
                task.net_name, probe_route.size(), anchor->coord.x,
                anchor->coord.y, anchor_dst, anchor_wire, target->coord.x,
                target->coord.y, probe_pin, probe.success,
                probe.target_seed_count, probe.target_entry_count,
                probe.target_busy_count, probe.forward_push_count,
                probe.backward_push_count);
            logDockingAttemptPaths(task.net_name, *target, probe,
                                   "focused docking probe");
            break;
          }
        }

        // Earlier-stage preemption leaves stale completed and duplicate work
        // in both handoff queues. Compact it once before focused relocation.
        for (RouteTask &task : route_todo) {
          canonicalizeRouteTaskSource(task);
        }
        for (RouteTask &task : moving_deferred_todo) {
          canonicalizeRouteTaskSource(task);
        }
        pnr::MovingStageQueueCompaction compacted =
            pnr::compactMovingStageQueues(
                route_todo, moving_deferred_todo,
                [&](const RouteTask &task) {
                  const std::vector<Wire> *route = findBoundRoute(
                      task.net, task.from, task.to, task.from_port,
                      task.to_port, task.net_name);
                  return route && routeIsComplete(*route);
                },
                [](const RouteTask &task) { return routeTaskHash(task); },
                [](const RouteTask &left, const RouteTask &right) {
                  return sameRouteTask(left, right);
                },
                [](RouteTask &existing, const RouteTask &duplicate) {
                  mergeRouteTaskState(existing, duplicate);
                });
        PNR_LOG1("ROUT",
                 "routeDesign Moving handoff compacted completed={}, "
                 "duplicates={}, remaining={}",
                 compacted.completed, compacted.duplicates,
                 route_todo.size());
      }
      moving_sources_stage = false;
      moving_stage = true;
      fanout_stage = false;
      fanout_preemption_enabled = true;
      route_deadends_enabled = false;
      applyRouteDeadends({});
      moving_relocate_next = true;
      stagnant_passes = 0;
      fanout_no_completion_passes = 0;
      moving_no_completion_passes = 0;
      stage_pass = 0;
      if (entering_moving_stage) {
        PNR_LOG1("ROUT",
                 "routeDesign stage: Moving destinations, tasks={}, "
                 "incomplete_bindings={}",
                 route_todo.size(),
                 tech ? countIncompleteRouteBindings(tech->design) : 0);
      } else {
        PNR_LOG3("ROUT", "routeDesign moving relocation cycle: tasks={}",
                 route_todo.size());
      }
    }
    if (route_todo.empty() && tech) {
      std::vector<RouteTask> shared_prefix_repairs;
      size_t repaired =
          repairStaleSharedRoutePrefixes(tech->design, shared_prefix_repairs);
      if (repaired != 0) {
        size_t generic_repairs = scheduleSharedPrefixRepairs(
            shared_prefix_repairs, route_todo, fanout_route_todo);
        route_todo.insert(route_todo.end(),
                          std::make_move_iterator(fanout_route_todo.begin()),
                          std::make_move_iterator(fanout_route_todo.end()));
        fanout_route_todo.clear();
        deduplicateRouteTasks(route_todo);
        fanout_stage = false;
        moving_sources_stage = false;
        moving_stage = true;
        fanout_seed_repair_active = false;
        moving_focus_inst = nullptr;
        moving_relocate_next = false;
        route_deadends_enabled = false;
        applyRouteDeadends({});
        resetPassPreemptionState();
        stagnant_passes = 0;
        fanout_stagnant_attempts = 0;
        fanout_no_completion_passes = 0;
        moving_no_completion_passes = 0;
        stage_pass = 0;
        PNR_LOG1("ROUT",
                 "routeDesign Moving destinations shared-prefix repair: "
                 "seeds={}, tasks={}",
                 generic_repairs, route_todo.size());
      }
    }
    bool all_route_queues_empty =
        route_todo.empty() && pending_route_todo.empty() &&
        fanout_route_todo.empty() && moving_destination_todo.empty() &&
        moving_deferred_todo.empty() && moving_source_retry_todo.empty() &&
        moving_focus_inst == nullptr;
    if (all_route_queues_empty && tech) {
      size_t audited = collectIncompleteRouteTasks(tech->design, route_todo);
      if (audited != 0) {
        moving_sources_stage = false;
        moving_stage = true;
        fanout_stage = false;
        fanout_preemption_enabled = true;
        route_deadends_enabled = false;
        applyRouteDeadends({});
        moving_relocate_next = false;
        stagnant_passes = 0;
        moving_no_completion_passes = 0;
        stage_pass = 0;
        PNR_LOG1("ROUT",
                 "routeDesign final binding audit requeued {} incomplete "
                 "ordinary physical routes",
                 audited);
      }
    }
    bool scheduler_can_continue =
        route_todo.empty() || active_this_pass > 0 ||
        route_todo.size() < before || changed_this_pass > 0 ||
        (fanout_stage &&
         fanout_stagnant_attempts < route_todo.size() * 16) ||
        pnr::focusedMovingMayRetryInactivePass(
            moving_focus_inst != nullptr, focus_retry_window_exhausted) ||
        pnr::movingRelocationSatisfiesProgress(moving_stage,
                                               moving_relocate_next) ||
        start_fanout_after_pass || start_moving_after_pass;
    if (!scheduler_can_continue) {
      failRouting(first_unfinished_task(),
                                  "scheduler made no progress");
    }
    PNR_ASSERT(scheduler_can_continue,
               "routeDesign made no progress in pass {} with {} cells",
               pass + 1, design_cells);
  }
  if (pnr::routeSchedulerHasWork(!route_todo.empty(), moving_stage,
                                 !moving_deferred_todo.empty() ||
                                     !moving_source_retry_todo.empty())) {
    print_stage_report("pass limit");
    failRouting(first_unfinished_task(), "pass limit");
    PNR_ASSERT(false,
               "routeDesign did not finish after {} limited passes with {} "
               "active, {} deferred and {} source-retry unfinished route tasks",
               max_route_passes, route_todo.size(), moving_deferred_todo.size(),
               moving_source_retry_todo.size());
  }

  if (!pending_route_todo.empty() || !fanout_route_todo.empty() ||
      !moving_destination_todo.empty()) {
    failRouting(first_unfinished_task(),
                "scheduler exited with deferred routing work");
  }

  // A later relocation may change a protected clock endpoint. Its old tree
  // stayed leased, so replace that tree once without exposing clock work to
  // any ordinary routing queue.
  if (clock_routes_dirty && tech) {
    PNR_LOG1("ROUT",
             "routeDesign Clock routing repair: rebuilding moved endpoints");
    auto &nets = tech->design.top.cell_ref->module_ref->nets;
    std::vector<fpga::NetRouteRef> protected_routes;
    for (rtl::Net &net : nets) {
      if (!net.route_protected || net.distributed_source) {
        continue;
      }
      for (size_t index = 0; index < net.routes.size(); ++index) {
        protected_routes.push_back({&net, index});
      }
    }
    // Clock branches may be represented by more than one logical net object.
    // Clear their physical trees together before dropping route bindings.
    fpga::unrouteSourceRouteTree(protected_routes);
    for (rtl::Net &net : nets) {
      if (!net.route_protected || net.distributed_source) {
        continue;
      }
      net.clearRouteBindings();
    }

    auto repair_start = std::chrono::steady_clock::now();
    RouteClocks clock_repair(*tech, *fpga);
    bool repaired = clock_repair.routeDesign(tech->clocks);
    double repair_seconds = elapsedSeconds(repair_start,
                                           std::chrono::steady_clock::now());
    const RouteClocks::Stats &repair_stats = clock_repair.stats();
    if (!repaired) {
      const RouteClocks::Failure &failure = clock_repair.failure();
      RouteTask failed;
      failed.net_name = failure.net_name;
      failed.failure_coord = {failure.x, failure.y};
      failRouting(&failed, "Clock routing repair",
                  static_cast<fpga::CBNodeNameType>(failure.node_type),
                  failure.node);
    }
    clock_report.attempted += repair_stats.sinks;
    clock_report.completed += repair_stats.routed;
    clock_report.searches += repair_stats.nets;
    clock_report.pops += repair_stats.graph_nodes;
    clock_report.seconds += repair_seconds;
    clock_report.remaining_tasks = repair_stats.failed;
    PNR_LOG1("ROUT",
             "routeDesign Clock routing repair complete: sinks={}, routed={}, "
             "time={:.3f}s",
             repair_stats.sinks, repair_stats.routed, repair_seconds);
    clock_routes_dirty = false;
  }

  // Clock and constant stages are barriers. Ordinary queues may observe their
  // leases but cannot own, reschedule, or leave either route class incomplete.
  for (const RouteTask &task : distributed_route_tasks) {
    const std::vector<Wire> *route = findBoundRoute(
        task.net, task.from, task.to, task.from_port, task.to_port,
        task.net_name);
    if (!route || !routeIsComplete(*route)) {
      failRouting(&task,
                                  "Const route invalidated after its stage");
    }
    PNR_ASSERT(route && routeIsComplete(*route),
               "constant route '{}' was invalidated after Const routing",
               task.net_name);
  }

  print_stage_report("complete");

  logMalformedRouteTrees(technology::Tech::current().design);

  travers_mark = rtl::Inst::genMark();
  image.init(mesh_width * aspect_x * image_zoom,
             mesh_height * aspect_y * image_zoom);
  image.clear();
  for (auto &bunch : bunch_list) {
    recurseDrawDesign(*bunch.reg, &bunch, false);
  }
  travers_mark = rtl::Inst::genMark();
  for (auto &bunch : bunch_list) {
    recurseDrawDesign(*bunch.reg, &bunch, true);
  }
  image.write(std::string("route_output.png"));
}

void RouteDesign::recurseDrawDesign(rtl::Inst &inst, RegBunch *bunch,
                                    bool place, int depth) {
  if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
    return;
  }
  inst.mark = travers_mark;

  if (place) {
    if (inst.cell_ref->type.find("BUF") != std::string::npos) {
      image.set_pixel(inst.outline.x * aspect_x * image_zoom,
                      inst.outline.y * aspect_y * image_zoom, 0, 255, 255, 255);
    } else if (inst.cell_ref->type.find("LUT") != std::string::npos) {
      image.set_pixel(inst.outline.x * aspect_x * image_zoom,
                      inst.outline.y * aspect_y * image_zoom, 0, 255, 0, 255);
    } else {
      image.set_pixel(inst.outline.x * aspect_x * image_zoom,
                      inst.outline.y * aspect_y * image_zoom, 0, 0, 255, 255);
    }
  } else {
    for (auto &wireing : inst.wires) {
      for (auto &wire : wireing) {
        //    std::print("\naaaaaaaaaaaaaaaa");
        int r = wire.to.x > wire.from.x ? wire.to.x - wire.from.x
                                        : wire.from.x - wire.to.x;
        int g = wire.to.y > wire.from.y ? wire.to.y - wire.from.y
                                        : wire.from.y - wire.to.y;
        image.draw_line(wire.from.x * image_zoom + r,
                        wire.from.y * image_zoom + g,
                        wire.to.x * image_zoom + r,
                        wire.from.y * image_zoom + g, r * 100, g * 100, 0, 255);
        image.draw_line(wire.to.x * image_zoom + r,
                        wire.from.y * image_zoom + g,
                        wire.to.x * image_zoom + r, wire.to.y * image_zoom + g,
                        r * 100, g * 100, 0, 255);
      }
    }
  }

  for (auto &conn : std::ranges::views::reverse(inst.conns)) {
    rtl::Conn *curr = &conn;
    if (curr->port_ref->type == rtl::Port::PORT_IN) {
      if (tech->check_clocked(curr->inst_ref->cell_ref->type,
                              curr->port_ref->name)) { // excluding clock ports
        continue;
      }

      curr = curr->follow();
      if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox ||
          curr->port_ref->is_global) { // after BUFs (can be something?)
        continue;
      }

      rtl::Inst *peer = curr->inst_ref.peer;

      /*            if (peer->coord.fixed || curr->inst_ref->coord.fixed) {
                      image.draw_line(inst.coord.x*aspect_x*image_zoom,
      inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom,
      peer->coord.y*aspect_y*image_zoom, 200, 200, 200, 100);
                  }
                  else
                  if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
      if (mode == 1) {
                      image.draw_line(inst.coord.x*aspect_x*image_zoom,
      inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom,
      peer->coord.y*aspect_y*image_zoom, 255, 0, 0, 100);
      }
                  }
                  else {
      if (mode == 1) {
                      image.draw_line(inst.coord.x*aspect_x*image_zoom,
      inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom,
      peer->coord.y*aspect_y*image_zoom, 0, 200, 200, 100);
      }
                  }
      */
      if (peer->mark != travers_mark) {
        //                peer->mark = travers_mark;
        recurseDrawDesign(*peer, nullptr, place, depth + 1);
      }
    }
  }

  if (bunch) {
    for (auto &subbunch : bunch->sub_bunches) {
      recurseDrawDesign(*subbunch.reg, &subbunch, place, depth + 1);
    }
  }
}
