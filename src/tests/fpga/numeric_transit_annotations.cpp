#include "SourceInputFixture.h"

// Names are optional export/debug annotations, not a second connectivity graph.
// Keep the first transit outside grounding range so docking cannot hide a veto.
void numeric_transit_annotations_regression() {
  using namespace routing_tests;
  for (bool connected : {true, false}) {
    SourceInputFixture f(false, 0, 4, 20);
    f.edge({7,0}, {15,2}, 141, 100, 200, true);
    f.edge({15,2}, {12,2}, 200, 101, 201);
    f.edge({12,2}, {0,0}, 201, 102, 202);
    f.terminal({0,0}, 202, 87);
    // Keeping node names and a resolved jump must not invent a missing CB edge.
    if (!connected) f.tile({15,2}).cb_type->dst_src[200].jump.clearBit(101);
    for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    auto &router = f.tech.route;
    router.moving_stage = router.complete_routes_only = router.preserve_routes = true;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    router.iteration_limit = router.route_recursion_budget = 200;
    pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal,
        "Q", "I0", SourceInputFixture::port(*f.sink, "I0").makeNetName()};
    task.endpoints_prepared = true;
    // Every resource is named, but no textual connection records were installed.
    require(router.routeNetTask(task) == connected,
        "numeric transit disagrees with masks when optional connection names are absent");
    require(router.routeTaskIsComplete(task) == connected,
        "numeric transit reported incorrect endpoint completion");
    size_t jumps = 0;
    for (const auto &binding : f.signal->routes)
      for (const auto &wire : binding.owner->wires.at(binding.route_index))
        jumps += wire.type == fpga::Wire::WIRE_CROSSBAR && wire.jump >= 0;
    // Completion traverses all physical links, not an annotation shortcut.
    require(jumps == (connected ? 3 : 0), "numeric transit lost or invented a physical hop");
    f.auditAll();
  }
}
