#include "RoutePreemption.h"
#include "SourceInputFixture.h"

#include <iostream>

namespace {
using routing_tests::SourceInputFixture;
using routing_tests::require;

void addLogicSlot(fpga::Tile &tile) {
  fpga::Element element;
  element.name = "test_logic";
  element.type = fpga::ELEMENT_LUT5;
  tile.tile_type->elements.push_back(element);
  element.name = "test_logic_extension";
  element.type = fpga::ELEMENT_LUT1;
  tile.tile_type->elements.push_back(element);
}

void runSourceExitCase(bool busy_src, bool protected_owner) {
  SourceInputFixture f(false, 0, 8, 8);
  addLogicSlot(f.tile({3, 0}));
  addLogicSlot(f.tile({5, 0}));
  auto *owner_source = f.cell("transit_source", "LUT6", {{"O", true}}, {3, 0});
  auto *owner_sink = f.cell("transit_sink", "FDRE", {{"CE", false}}, {1, 0});
  auto *owner = f.connect(*owner_source, "O", *owner_sink, "CE");
  owner_source->outline.fixed = owner_sink->outline.fixed = true;
  owner->route_protected = protected_owner;

  const int owner_src = busy_src ? 200 : 300;
  const int owner_dst = busy_src ? 501 : 500;
  std::vector<fpga::Wire> route{
      SourceInputFixture::pin({3, 0}, 169, true),
      f.edge({3, 0}, {7, 0}, 169, 310, 600, true),
      f.edge({7, 0}, {5, 0}, 600, owner_src, owner_dst),
      f.edge({5, 0}, {1, 0}, owner_dst, 400, 700),
      f.terminal({1, 0}, 700, 63),
      SourceInputFixture::pin({1, 0}, 63, false)};
  f.edge({7, 0}, {5, 0}, 141, 200, owner_dst, true);
  f.terminal({5, 0}, 500, 87);
  if (busy_src) {
    // This SRC has two physical landings; only the owner's landing is busy.
    f.tile({7, 0}).cb_type->dst_by_src[200].front().dsts.jump.setBit(500);
  }
  for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(route), "cannot install transit owner");
  owner_sink->wires.push_back(std::move(route));
  const size_t owner_binding = fpga::attachNetRoute(*owner, *owner_sink, 0,
      owner_source, owner_sink, "O", "CE", "transit_owner");
  fpga::registerNetRouteTiles(*owner, owner_sink->wires[0], owner_binding);

  f.sink->outline.fixed = false;
  f.sink->wires.emplace_back();
  const std::string route_name = SourceInputFixture::port(*f.sink, "I0").makeNetName();
  const size_t request_binding = fpga::attachNetRoute(*f.signal, *f.sink, 0,
      f.source, f.sink, "Q", "I0", route_name);
  const fpga::RouteCutNode cut{busy_src ? fpga::Coord{7, 0} : fpga::Coord{5, 0},
      busy_src ? fpga::CB_NODE_SRC : fpga::CB_NODE_DST,
      busy_src ? 200 : 500};
  require(f.tile({7, 0}).cb.src.jump.testBit(200) == busy_src &&
          f.tile({5, 0}).cb.dst.jump.testBit(500) == !busy_src,
      "source SRC and landing DST are not independently blocked");
  require(fpga::findNetRouteCutVictims({cut}).size() == 1 &&
          fpga::canPreemptNetRouteFromNodes(*owner, owner_binding, {cut}) ==
              !protected_owner,
      "source exit owner eligibility does not match its protection");
  const auto positions = f.tile({5, 0}).candidatePositions(f.sink);
  require(std::find(positions.begin(), positions.end(), 3) != positions.end(),
      "source exit has no packable destination");
  f.auditAll();

  auto &router = f.tech.route;
  router.moving_stage = router.complete_routes_only = true;
  router.route_preemption_enabled = router.fanout_preemption_enabled = true;

  router.iteration_limit = router.route_suffix_depth_limit = 128;
  router.route_recursion_budget = 64;
  pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal, "Q", "I0", route_name};
  task.endpoints_prepared = true;
  std::vector<pnr::RouteDesign::RouteTask> affected;
  std::string reason;
  bool wire_only = false;
  const bool done = router.moveUnfinishedCell(task, &affected, &task, &reason,
      false, &wire_only);
  std::cout << "source exit busy_src=" << busy_src << " protected="
            << protected_owner << " done=" << done << " cuts="
            << router.route_stats.preempt_success << " reason=" << reason << '\n';
  if (protected_owner) {
    require(!done && !wire_only && router.route_stats.preempt_success == 0 &&
            fpga::isRouteComplete(owner_sink->wires[0]) &&
            f.sink->tile.peer == &f.tile({0, 0}) &&
            router.pending_route_todo.empty(),
        "protected source exit was cut or accepted");
  } else {
    const auto &binding = f.signal->routes.at(request_binding);
    require(done && !wire_only && router.route_stats.preempt_success == 1 &&
            f.sink->tile.peer == &f.tile({5, 0}) && binding.owner &&
            fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
        "Moving missed the source-exit transit cut and reachable placement: " + reason);
    const auto &victim = owner->routes.at(owner_binding);
    const auto &retained = victim.owner->wires.at(victim.route_index);
    require(!fpga::isRouteComplete(retained) && retained.size() >= 2 &&
            retained[0].type == fpga::Wire::WIRE_TILE_PIN &&
            retained[1].type == fpga::Wire::WIRE_CROSSBAR &&
            retained[1].from == fpga::Coord{3, 0} &&
            retained[1].to == fpga::Coord{7, 0} &&
            retained[1].jump == 310,
        "source-exit cut lost the foreign victim's protected first hop");
    require(std::any_of(router.pending_route_todo.begin(),
                        router.pending_route_todo.end(),
                        [&](const auto &pending) {
                          return pending.net == owner &&
                                 pending.from == owner_source &&
                                 pending.to == owner_sink &&
                                 pending.net_name == "transit_owner";
                        }),
        "source-exit cut did not queue the displaced foreign binding");
  }
  f.auditAll();
}
} // namespace

void moving_source_exit_preemption_regression(bool busy_src, bool protected_owner) {
  runSourceExitCase(busy_src, protected_owner);
}
