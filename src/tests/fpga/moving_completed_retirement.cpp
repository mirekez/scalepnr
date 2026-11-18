#include "RoutePreemption.h"
#include "SourceInputFixture.h"

#include "route/RouteDesign.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

void retargetedCompletionRetiresOldTask() {
  using routing_tests::SourceInputFixture;
  using routing_tests::require;

  SourceInputFixture f(false, 0, 8);
  auto *old_source = f.cell("old_source", "FDRE", {{"D", false}, {"Q", true}}, {1, 0});
  auto *middle_source = f.cell("middle_source", "FDRE", {{"Q", true}}, {2, 0});
  auto *new_source = f.source;
  auto *sink = f.sink;
  auto *sibling_net = f.connect(*new_source, "Q", *old_source, "D");

  std::vector<fpga::Wire> route{
      SourceInputFixture::pin({7, 0}, 141, true),
      f.edge({7, 0}, {0, 0}, 141, 220, 620, true),
      f.terminal({0, 0}, 620, 87),
      SourceInputFixture::pin({0, 0}, 87, false)};
  f.tile({7, 0}).cb_type->rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(route), "cannot install completed retargeted route");
  sink->wires.push_back(std::move(route));
  const size_t complete_storage = sink->wires.size() - 1;
  const size_t complete_binding = fpga::attachNetRoute(
      *f.signal, *sink, complete_storage, new_source, sink, "Q", "I0",
      "retargeted_completion");
  fpga::registerNetRouteTiles(*f.signal, sink->wires[complete_storage], complete_binding);
  old_source->wires.push_back({});
  fpga::attachNetRoute(*sibling_net, *old_source, 0, new_source, old_source,
                       "Q", "D", "unfinished_sibling");
  f.auditAll();

  auto &router = f.tech.route;
  router.fpga = &f.device;
  const auto key = [](rtl::Inst *inst) {
    return std::to_string(reinterpret_cast<uintptr_t>(inst)) + ":Q";
  };
  router.source_endpoint_retargets[key(old_source)] = {middle_source, "Q"};
  router.source_endpoint_retargets[key(middle_source)] = {new_source, "Q"};
  pnr::RouteDesign::RouteTask completed{old_source, sink, f.signal, "Q", "I0",
                                        "retargeted_completion"};
  pnr::RouteDesign::RouteTask unfinished{new_source, old_source, sibling_net,
                                         "Q", "D", "unfinished_sibling"};
  auto canonical = completed;
  require(router.canonicalizeRouteTaskSource(canonical) &&
              canonical.from == new_source && router.routeTaskIsComplete(canonical),
          "two-step source retarget did not resolve the completed live binding");
  require(f.signal->routes.at(complete_binding).from == new_source &&
              completed.from == old_source &&
              !router.routeTaskIsComplete(unfinished),
          "retirement fixture did not retain distinct queued identities");
  const auto before_route = sink->wires[complete_storage];
  const auto before_src = f.tile({7, 0}).cb.src.jump;
  const auto before_dst = f.tile({0, 0}).cb.dst.jump;
  const auto before_local = f.tile({0, 0}).cb.local.local;
  const auto before_pin = f.tile({0, 0}).pin_state.leased_nodes;
  std::vector<pnr::RouteDesign::RouteTask> queue{completed, unfinished};
  const size_t retired = router.retireCompletedMovingTasks(queue);
  std::cout << "retarget retirement: retired=" << retired
            << " remaining=" << queue.size() << '\n';
  require(retired == 1 && queue.size() == 1 &&
              queue.front().net == sibling_net &&
              !router.routeTaskIsComplete(queue.front()),
          "Moving repeated a completed retargeted task or retired its unfinished sibling");
  // Read-only completion and direct relocation must agree even for a stale
  // task copy, without any tracing, placement probes or preemption.
  require(router.routeTaskIsComplete(completed),
          "read-only completion did not follow physical source replacements");
  const auto attempts = router.route_stats.task_attempts;
  const auto skips = router.route_stats.already_complete;
  bool wire_only = false;
  std::string reason;
  require(router.moveUnfinishedCell(completed, nullptr, &completed, &reason,
                                   false, &wire_only) && wire_only,
          "completed replacement entered relocation: " + reason);
  require(router.route_stats.task_attempts == attempts &&
              router.route_stats.already_complete == skips + 1 &&
              router.route_stats.moving_observer_calls == 0 &&
              router.route_stats.preempt_success == 0,
          "completed replacement performed routing or placement work");
  require(fpga::isRouteComplete(sink->wires[complete_storage]) &&
              sink->wires[complete_storage].size() == before_route.size() &&
              f.tile({7, 0}).cb.src.jump == before_src &&
              f.tile({0, 0}).cb.dst.jump == before_dst &&
              f.tile({0, 0}).cb.local.local == before_local &&
              f.tile({0, 0}).pin_state.leased_nodes == before_pin,
          "retirement changed the completed physical route");
  f.auditAll();
}

void voidCompletionRetiresWithoutRoute() {
  using routing_tests::SourceInputFixture;
  using routing_tests::require;

  SourceInputFixture f(false, 0, 8);
  auto &local_elements = f.tile({2, 0}).tile_type->elements;
  const auto &logic_elements = f.tile({6, 0}).tile_type->elements;
  local_elements.insert(local_elements.end(), logic_elements.begin(),
                        logic_elements.end());
  auto *local_source = f.cell("local_source", "LUT6", {{"O", true}}, {2, 0});
  auto *local_sink = f.cell("local_sink", "FDRE", {{"D", false}}, {2, 0});
  auto *void_net = f.connect(*local_source, "O", *local_sink, "D");
  void_net->void_net = true;
  local_sink->wires.push_back({});
  fpga::attachNetRoute(*void_net, *local_sink, 0, local_source, local_sink,
                       "O", "D", "local_void_input");
  f.auditAll();

  auto &router = f.tech.route;
  router.fpga = &f.device;
  router.moving_stage = true;
  pnr::RouteDesign::RouteTask completed{local_source, local_sink, void_net,
                                        "O", "D", "local_void_input"};
  pnr::RouteDesign::RouteTask unfinished{f.source, f.sink, f.signal,
                                         "Q", "I0", "unfinished_input"};
  const auto before_src = f.tile({2, 0}).cb.src.jump;
  const auto before_dst = f.tile({2, 0}).cb.dst.jump;
  const auto before_local = f.tile({2, 0}).cb.local.local;
  const auto before_pin = f.tile({2, 0}).pin_state.leased_nodes;
  bool wire_only = false;
  std::string reason;
  require(router.moveUnfinishedCell(completed, nullptr, &completed, &reason,
                                   false, &wire_only) && wire_only &&
              router.routeTaskIsComplete(completed),
          "void input was not completed in place: " + reason);
  std::vector<pnr::RouteDesign::RouteTask> queue{completed, unfinished};
  const size_t retired = router.retireCompletedMovingTasks(queue);
  std::cout << "void retirement: retired=" << retired
            << " remaining=" << queue.size() << '\n';
  require(retired == 1 && queue.size() == 1 &&
              queue.front().net == f.signal &&
              !router.routeTaskIsComplete(queue.front()),
          "Moving repeated a void input or retired an unfinished physical sibling");
  // Void completion requires neither physical fragments nor a placement probe.
  require(router.route_stats.already_complete == 1 &&
              router.route_stats.task_attempts == 0 &&
              router.route_stats.moving_observer_calls == 0 &&
              router.route_stats.preempt_success == 0,
          "void completion performed routing or placement work");
  require(local_sink->wires.front().empty() &&
              f.tile({2, 0}).cb.src.jump == before_src &&
              f.tile({2, 0}).cb.dst.jump == before_dst &&
              f.tile({2, 0}).cb.local.local == before_local &&
              f.tile({2, 0}).pin_state.leased_nodes == before_pin,
          "void retirement invented a physical route");
  f.auditAll();
}

} // namespace

void moving_retarget_retirement_regression() {
  retargetedCompletionRetiresOldTask();
}

void moving_void_retirement_regression() {
  voidCompletionRetiresWithoutRoute();
}
