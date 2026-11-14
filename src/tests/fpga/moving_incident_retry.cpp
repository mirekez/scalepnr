#include "SourceInputFixture.h"

#include <filesystem>

using routing_tests::SourceInputFixture;
using routing_tests::require;

void moving_incident_retry_regression() {
  // The native failure moved a register for one input, rerouted its output,
  // then hard-exited on another input before trying a second placement.
  // Reproduce that post-move state with a disconnected but packable input
  // at (2,0), a real preserved control prefix, and a reachable slot at (1,0).
  const auto artifacts = std::filesystem::temp_directory_path() /
      ("scalepnr-incident-retry-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(artifacts);
  setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", artifacts.c_str(), 1);
  for (int mode = 0; mode < 5; ++mode) {
    SourceInputFixture f(false, 0, 16);
    auto& router = f.tech.route;
    router.moving_stage = true;
    router.moving_focus_inst = f.source;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    router.iteration_limit = router.route_suffix_depth_limit = 200;
    router.complete_routes_only = router.preserve_routes = true;
    const auto control_id = f.enable->routes.front().route_id;
    fpga::invalidateMovedSinkRoutes({{f.enable, 0}});
    require(f.tile({7,0}).unassign(f.source), "cannot release old register slot");
    require(f.tile({2,0}).tryAddAt(f.source, 0) == 0, "first move is not packable");
    pnr::RouteDesign::RouteTask output{f.source, f.sink, f.signal, "Q", "I0", "pattern_output_71"};
    output.endpoints_prepared = true;
    require(router.routeTaskOrFail(output, "Moving destinations", 5, true, false),
        "post-move output must be routed before testing the remaining input");
    pnr::RouteDesign::RouteTask input{f.control, f.source, f.enable, "O", "CE", "control_tree"};
    input.endpoints_prepared = true;
    // A reachable input alone must not select the old register slot: its Q
    // local has no takeoff. The alternate slot has a real local-to-SRC path.
    require(f.tile({7,0}).cb_type->srcNodes(fpga::CB_NODE_LOCAL,141)==nullptr &&
        f.tile({1,0}).cb_type->srcNodes(fpga::CB_NODE_LOCAL,141)!=nullptr,
        "fixture lost the input-reachable but output-disconnected placement");
    // A larger numeric route budget cannot connect disconnected graph components.
    require(!router.completeTaskFromSource(input), "current input unexpectedly routes without moving");
    require(f.source->tile.peer == &f.tile({2,0}), "wire-only control moved the register");
    f.auditAll();
    if (mode == 3) f.source->outline.fixed = f.source->pnr_completed = true;
    if (mode == 4) {
      router.route_stage_deadline_enabled = true;
      router.route_stage_deadline = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    }
    bool done = false;
    std::string failure;
    try {
      if (mode == 1) {
        std::vector<pnr::RouteDesign::RouteTask> tasks{input};
        done = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving,
                                     tasks, 1, 5, true).completed == 1;
      } else {
        done = router.routeTaskOrFail(input, "Moving destinations", 5, true, mode != 2);
      }
    } catch (const std::exception& error) { failure = error.what(); }
    std::cout << "incident retry mode=" << mode << " done=" << done
              << " placement=(" << f.source->coord.x << ',' << f.source->coord.y
              << ") pending=" << router.pending_route_todo.size()
              << " failure=" << failure << '\n';
    if (mode < 2 || mode == 3) {
      // Both top-level dispatch paths must repair the failed input at the new location.
      require(done && f.source->tile.peer == &f.tile({1,0}),
          "remaining input hard-failed before alternative destination search: " + failure);
      // Earlier-run placement protection must survive, but its completion
      // flag cannot survive a move with incident output repairs still pending.
      if (mode == 3)
        require(f.source->outline.fixed && !f.source->pnr_completed,
            "fixed-cell routing lost protection or kept stale completion");
      require(f.enable->routes.front().route_id == control_id,
          "destination retry replaced the logical input binding");
      require(f.tile({2,0}).hasFreeElement(fpga::ELEMENT_FD) &&
          !f.tile({1,0}).hasFreeElement(fpga::ELEMENT_FD), "move leaked a register reservation");
      // Rerouting the moved output is mandatory work, not a lost completed task.
      auto pending = std::find_if(router.pending_route_todo.begin(), router.pending_route_todo.end(),
          [&](const auto& task) { return task.from == f.source && task.to == f.sink; });
      require(pending != router.pending_route_todo.end(), "retry lost the invalidated output task");
      auto reroute = *pending;
      require(router.routeTaskOrFail(reroute, "Moving destinations", 5, true, false),
          "output could not be rerouted after successful input relocation");
      for (auto* net : {f.enable, f.signal})
        for (const auto& binding : net->routes)
          require(binding.owner && binding.route_index < binding.owner->wires.size() &&
              fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
              "destination retry reported success with an incomplete incident route");
    } else {
      // Nested wire-only work and deadline cancellation still fail hard.
      require(!done && !failure.empty() && f.source->tile.peer == &f.tile({2,0}),
          "retry ignored its routing scope or deadline");
    }
    require(f.control->tile.peer == &f.tile({6,0}) && f.sink->tile.peer == &f.tile({0,0}),
        "destination routing moved fixed neighboring cells");
    f.auditAll(); // Every remaining lease must still have its real owning route.
  }
  std::filesystem::remove_all(artifacts);
}
