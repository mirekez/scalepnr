#include "SourceInputFixture.h"
#include "InputRepairBudget.h"
#include "route/MovingSourceReachability.h"

// Small budgets exercise the exact production cutoff without requiring a
// million-node graph. Names in this topology are neutral node_N and signal_N.
void source_input_budget_regression() {
  using namespace routing_tests;
  require(pnr::MAX_INPUT_REPAIRING_EXPANDED_STATES_PER_ONE_MOVING == 1000000,
          "production input repair allowance changed");
  for (bool defer : {true, false}) {
    SourceInputFixture f(false);
    auto &router = f.tech.route;
    router.moving_stage = router.moving_sources_stage = true;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal, "Q", "I0", "FILL_OUTPUT"};
    task.endpoints_prepared = true;
    pnr::InputRepairBudget budget;
    if (defer) budget.limit = 10;
    std::vector<pnr::RouteDesign::RouteTask> todo;
    std::string reason;
    bool moved = false;
    require(router.moveUnfinishedSource(task, &todo, &reason, nullptr, &moved,
                nullptr, nullptr, true, nullptr, nullptr, &budget),
            "input budget discarded a legal output placement: " + reason);
    // Exceeding the nested input budget must not cancel the enclosing stage.
    require(moved && !router.route_stage_deadline_expired && !router.routeStageSearchCancelled(),
            "input budget cancelled the outer source search");
    auto output = f.signal->findRouteBinding(f.source, f.sink, "Q", "I0", task.net_name);
    require(output.index < f.signal->routes.size(), "output binding missing after move");
    auto &binding = f.signal->routes[output.index];
    // Output routing stays committed even when the inputs are explicitly deferred.
    require(fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
            "input deferral discarded the proven output trunk");
    if (defer) {
      // The first candidate is packable but its input cannot be repaired cheaply.
      require(budget.defer_inputs && budget.expanded == budget.limit,
              "input searches overshot or reset the aggregate expansion limit");
      require(f.source->coord == fpga::Coord{2,0}, "deferred candidate was rejected");
      // Both caller task handoff and the old lease must describe an unfinished input.
      require(std::any_of(todo.begin(), todo.end(), [&](const auto &input) {
                return input.net == f.enable && input.from == f.control &&
                    input.to == f.source && input.from_port == "O" && input.to_port == "CE";
              }), "deferred input disappeared from source-moving work");
      require(!f.tile({7,0}).cb.local.local.testBit(63), "old input terminal lease leaked");
      for (const auto &input : f.enable->routes)
        if (input.owner && input.route_index < input.owner->wires.size())
          require(!fpga::isRouteComplete(input.owner->wires[input.route_index]),
                  "deferred input still claims a complete old route");
    } else {
      // A fresh move receives a fresh allowance and retains immediate repair.
      require(!budget.defer_inputs && todo.empty(), "cheap input repair was deferred");
      require(f.source->coord == fpga::Coord{1,0}, "normal input proof chose the wrong placement");
      bool complete = false;
      for (const auto &input : f.enable->routes)
        if (input.owner && input.route_index < input.owner->wires.size())
          complete |= fpga::isRouteComplete(input.owner->wires[input.route_index]);
      require(complete, "normal source move lost immediate input repair");
    }
    // Check real owners for all remaining leases, including the retained input prefix.
    f.auditAll();
    std::cout << "FILL_INPUT_BUDGET deferred=" << budget.defer_inputs
              << " expanded=" << budget.expanded << " limit=" << budget.limit
              << " queued=" << todo.size() << '\n';
  }

  SourceInputFixture f(false);
  pnr::InputRepairBudget aggregate{7};
  auto search = [&]() {
    pnr::MovingSourceReachability walk;
    walk.source(&f.tile({6,0}), 169);
    // This impossible target forces the walk to consume its remaining allowance.
    walk.target(&f.tile({2,0}), 63);
    pnr::InputRepairBudget::Scope scope(aggregate);
    walk.mayConnect([] { return pnr::InputRepairBudget::stopped(); });
    require(!walk.closed(), "budget cutoff was cached as a closed source component");
    return walk.expanded();
  };
  // A second nested search must not receive a new budget after the first used it.
  require(search() == 7 && search() == 0 && aggregate.expanded == 7,
          "input budget reset between searches");
  require(!pnr::InputRepairBudget::stopped(), "input search scope leaked into other routing");
}
