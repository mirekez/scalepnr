#include "SourceInputFixture.h"

#include <cstdlib>
#include <iostream>
#include <optional>

void source_recovery_probe_cap_regression() {
  using routing_tests::require;
  using routing_tests::SourceInputFixture;
  // The large-design recovery escalated its caller budget while every
  // two-ended search silently retained the fast-probe cap. Reproduce that
  // mismatch with a neutral 135-node witness and a 64-label fast probe.
  const char *previous = std::getenv("SCALEPNR_FREE_SEARCH_STATES");
  const std::optional<std::string> saved =
      previous ? std::optional<std::string>(previous) : std::nullopt;
  struct Restore {
    const std::optional<std::string> &saved;
    ~Restore() {
      if (saved)
        setenv("SCALEPNR_FREE_SEARCH_STATES", saved->c_str(), 1);
      else
        unsetenv("SCALEPNR_FREE_SEARCH_STATES");
    }
  } restore{saved};
  setenv("SCALEPNR_FREE_SEARCH_STATES", "64", 1);
  for (int scenario = 0; scenario < 3; ++scenario) {
    SourceInputFixture f(false, 135);
    auto &router = f.tech.route;
    router.complete_routes_only = true;
    router.moving_stage = router.moving_sources_stage = true;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    router.route_recursion_budget = 5;
    pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal,
                                     "Q",      "I0",   "probe_cap_signal"};
    task.endpoints_prepared = true;
    if (scenario == 1) {
      // Removing the final free arc must not turn escalation into false
      // success.
      f.tile({0, 0}).cb_type->local_local[77].local.clearBit(115);
    } else if (scenario == 2) {
      // A cancelled stage cannot start the larger search or mutate any leases.
      router.route_stage_deadline_enabled = true;
      router.route_stage_deadline =
          std::chrono::steady_clock::now() - std::chrono::seconds(1);
    }
    std::ostringstream before;
    fpga::dumpNetRouteHistory(*f.enable, before);
    // A genuinely small caller budget must fail atomically, not fabricate a
    // path.
    require(!router.completeTaskFromSource(task, false, 64, true, true),
            "small two-ended probe unexpectedly completed the long witness");
    require(f.sink->wires.empty(), "failed probe published a partial route");
    f.auditAll();
    // Source recovery's explicit larger allowance must not repeat the capped
    // search.
    const bool done = router.recoverMovingSourceWires(task, true);
    std::cout << "scenario=" << scenario << " fast_cap=64 recovery_budget="
              << fpga::UNBOUND_ROUTE_STATE_BUDGETS.back()
              << " complete=" << done << '\n';
    if (scenario == 0) {
      require(done, "source recovery ignored its larger budget after the fast "
                    "probe failed");
      require(f.sink->wires.size() == 1 &&
                  fpga::isRouteComplete(f.sink->wires[0]),
              "source recovery installed an incomplete route");
    } else {
      require(!done && f.sink->wires.empty(),
              "disconnected or cancelled recovery published a route");
    }
    // Free-only recovery must preserve placements and every unrelated owner.
    require(f.source->tile.peer == &f.tile({7, 0}) &&
                f.sink->tile.peer == &f.tile({0, 0}),
            "free-only recovery moved a cell");
    std::ostringstream after;
    fpga::dumpNetRouteHistory(*f.enable, after);
    require(before.str() == after.str() &&
                router.route_stats.preempt_success == 0,
            "free-only recovery changed the control route");
    f.auditAll();
  }
}
