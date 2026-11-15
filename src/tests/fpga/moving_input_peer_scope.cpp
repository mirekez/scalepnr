#include "SourceInputFixture.h"

#include "route/RouteDesign.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace {

// Two required inputs cannot simultaneously own this single bridge. Without
// transaction protection, each successful repair cuts the other completed input.
void peerInputBridge(bool peer_required, bool reverse_order = false) {
  using routing_tests::SourceInputFixture;
  using routing_tests::require;

  SourceInputFixture f(false, 0, 8);
  for (fpga::Coord coord : {fpga::Coord{4, 0}, fpga::Coord{5, 0}})
    f.tile(coord).tile_type->elements = f.tile({6, 0}).tile_type->elements;
  auto *a = f.cell("bridge_driver_a", "LUT6", {{"O", true}}, {4, 0});
  auto *b = f.cell("bridge_driver_b", "LUT6", {{"O", true}}, {5, 0});
  auto *receiver = f.cell("bridge_receiver", "FDRE",
                          {{"CE", false}, {"D", false}}, {1, 0});
  auto *a_sink = peer_required
      ? receiver
      : f.cell("foreign_receiver", "FDRE", {{"CE", false}}, {2, 0});
  a->outline.fixed = b->outline.fixed = receiver->outline.fixed = true;
  a_sink->outline.fixed = true;

  auto &arrival = f.tile({1, 0});
  arrival.tile_type->pin_map.nodes[{"FDRE", "D", 0}].setBit(64);
  arrival.cb_type->local_input_nodes.setBit(64);
  SourceInputFixture::name(arrival, fpga::CB_NODE_LOCAL, 64);
  rtl::Net *net_a = f.connect(*a, "O", *a_sink, "CE");
  rtl::Net *net_b = f.connect(*b, "O", *receiver, "D");

  std::vector<fpga::Wire> route_a{
      SourceInputFixture::pin({4, 0}, 169, true),
      f.edge({4, 0}, {3, 0}, 169, 100, 80, true),
      f.edge({3, 0}, {1, 0}, 80, 102, 90)};
  if (!peer_required)
    route_a.push_back(f.edge({1, 0}, {2, 0}, 90, 105, 92));
  route_a.push_back(f.terminal(peer_required ? fpga::Coord{1, 0}
                                           : fpga::Coord{2, 0},
                               peer_required ? 90 : 92, 63));
  route_a.push_back(SourceInputFixture::pin(
      peer_required ? fpga::Coord{1, 0} : fpga::Coord{2, 0}, 63, false));
  f.edge({5, 0}, {3, 0}, 169, 101, 81, true);
  f.edge({3, 0}, {1, 0}, 81, 102, 90);
  arrival.cb_type->dst_local[90].local.setBit(64);
  for (fpga::Coord coord : {fpga::Coord{1, 0}, fpga::Coord{2, 0}, fpga::Coord{3, 0},
                            fpga::Coord{4, 0}, fpga::Coord{5, 0}})
    f.tile(coord).cb_type->rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();

  require(pnr::commitPreparedRoute(route_a), "cannot install complete peer route");
  a_sink->wires.push_back(std::move(route_a));
  const size_t a_storage = a_sink->wires.size() - 1;
  receiver->wires.push_back({});
  const size_t b_storage = receiver->wires.size() - 1;
  const size_t a_index = fpga::attachNetRoute(
      *net_a, *a_sink, a_storage, a, a_sink, "O", "CE", "required_input_a");
  fpga::registerNetRouteTiles(*net_a, a_sink->wires[a_storage], a_index);
  fpga::attachNetRoute(*net_b, *receiver, b_storage, b, receiver, "O", "D",
                       "required_input_b");
  f.auditAll();
  const auto a_before = a_sink->wires[a_storage];

  auto &router = f.tech.route;
  router.fpga = &f.device;
  router.moving_stage = true;
  router.moving_focus_inst = receiver;
  router.route_preemption_enabled = true;
  router.iteration_limit = router.route_suffix_depth_limit = 200;
  router.route_progress_stage_name = "Moving destinations";
  router.route_progress_remaining_tasks = peer_required ? 2 : 1;
  router.route_stage_deadline_enabled = true;
  router.route_stage_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  router.indexSourceRoute(net_a, a, "O");
  router.indexSourceRoute(net_b, b, "O");
  pnr::RouteDesign::RouteTask task_a{a, a_sink, net_a, "O", "CE",
                                    "required_input_a"};
  pnr::RouteDesign::RouteTask task_b{b, receiver, net_b, "O", "D",
                                    "required_input_b"};
  task_a.endpoints_prepared = task_b.endpoints_prepared = true;
  std::vector<pnr::RouteDesign::RouteTask> inputs;
  if (peer_required && reverse_order) inputs.push_back(task_b);
  if (peer_required) inputs.push_back(task_a);
  if (!peer_required || !reverse_order) inputs.push_back(task_b);
  const std::string retained_key =
      std::to_string(reinterpret_cast<uintptr_t>(b)) + ":O";
  if (!peer_required)
    router.preemption_protected_source_keys.insert(retained_key);
  std::string reason;
  const bool complete = router.moveInputSources(inputs, &reason);
  const auto &a_route = net_a->routes.at(0).owner->wires.at(
      net_a->routes.at(0).route_index);
  const auto &b_route = net_b->routes.at(0).owner->wires.at(
      net_b->routes.at(0).route_index);

  if (peer_required) {
    // Reject the impossible placement instead of exchanging the bridge until
    // the test-only deadline; every retained peer fragment must remain intact.
    require(!complete, "exclusive bridge completed two unrelated required inputs");
    require(!router.route_stage_deadline_expired,
            "required inputs repeatedly preempted each other until deadline");
    require(router.route_stats.preempt_success == 0,
            "required peer input was cut during recursive input repair");
    require(fpga::isRouteComplete(a_route) && !fpga::isRouteComplete(b_route),
            "failed input repair damaged the completed peer route");
    require(a_route.size() == a_before.size() &&
                std::equal(a_route.begin(), a_route.end(), a_before.begin(),
                    [](const fpga::Wire &live, const fpga::Wire &old) {
                      return live.type == old.type && live.from == old.from &&
                          live.to == old.to && live.local == old.local &&
                          live.jump == old.jump && live.dst == old.dst &&
                          live.joint == old.joint && live.joint2 == old.joint2 &&
                          live.pos == old.pos && live.shared == old.shared &&
                          live.owns_dst == old.owns_dst &&
                          live.owns_landing == old.owns_landing;
                    }),
            "failed input repair changed a completed peer fragment");
    require(router.preemption_protected_source_keys.empty(),
            "input protection leaked after failed repair");
  } else {
    // A route outside the required input set remains an eligible transit victim.
    // Cutting it must leave an explicit repair task and preserve an outer scope.
    require(complete, "unrelated transit owner could not be preempted: " + reason);
    require(router.route_stats.preempt_success > 0,
            "unrelated transit owner was not cut");
    require(!fpga::isRouteComplete(a_route) && fpga::isRouteComplete(b_route),
            "foreign transit cut did not transfer the bridge to the requested input");
    require(std::any_of(router.pending_route_todo.begin(),
                        router.pending_route_todo.end(),
                        [&](const auto &queued) {
                          return queued.net == net_a && queued.from == a &&
                                 queued.to == a_sink;
                        }),
            "foreign transit owner was cut without a repair task");
    require(router.preemption_protected_source_keys.size() == 1 &&
                router.preemption_protected_source_keys.contains(retained_key),
            "input protection removed a pre-existing source key");
  }
  f.auditAll();
}

} // namespace

void moving_input_peer_scope_regression() {
  peerInputBridge(true, false);
  peerInputBridge(true, true);
  peerInputBridge(false);
}
