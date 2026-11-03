#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"

#include <iostream>

void moving_numeric_input_anchor_regression() {
  using fpga::Wire;
  using routing_tests::require;
  using routing_tests::SourceInputFixture;
  for (bool connected : {false, true})
    for (bool numeric : {false, true}) {
      SourceInputFixture f(false, 0, 32);
      auto [driver, sibling] = f.installCEFrontier(connected);
      if (numeric) {
        // Preserve every physical arc, but encode the retained tree as numeric
        // edges, as the complete-route repair engine legitimately does.
        std::vector<Wire> converted;
        for (const Wire &wire : sibling->wires[0]) {
          // Keep the ordinary takeoff; later numeric landings contain the only
          // usable branch to the candidate, beyond its leased first landing.
          if (wire.type != Wire::WIRE_CROSSBAR || wire.jump < 0 ||
              wire.pos == 0) {
            converted.push_back(wire);
            continue;
          }
          require(wire.joint < 0 && wire.joint2 < 0,
                  "unexpected fixture joint");
          Wire edge;
          edge.type = Wire::WIRE_ROUTE_EDGE;
          edge.from = edge.to = wire.from;
          edge.from_node_type =
              wire.pos == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
          edge.from_node = wire.local;
          edge.to_node_type = fpga::CB_NODE_SRC;
          edge.to_node = wire.jump;
          converted.push_back(edge);
          edge.from_node_type = fpga::CB_NODE_SRC;
          edge.from_node = wire.jump;
          edge.to = wire.to;
          edge.to_node_type = fpga::CB_NODE_DST;
          edge.to_node = wire.dst;
          converted.push_back(edge);
        }
        require(fpga::unrouteNetRoute(*f.enable, 0),
                "cannot replace control representation");
        f.enable->clearRouteBindings();
        require(pnr::commitPreparedRoute(converted),
                "numeric control tree cannot lease its arcs");
        sibling->wires[0] = std::move(converted);
        const auto index = fpga::attachNetRoute(
            *f.enable, *sibling, 0, driver, sibling, "O", "CE",
            SourceInputFixture::port(*sibling, "CE").makeNetName());
        fpga::registerNetRouteTiles(*f.enable, sibling->wires[0], index);
        f.tech.route.indexSourceRoute(f.enable, driver, "O");
      }
      // Every occupied node must still be justified by a real installed route.
      f.auditAll();
      require(fpga::isRouteComplete(sibling->wires[0]),
              "control tree became incomplete");
      std::ostringstream before;
      fpga::dumpNetRouteHistory(*f.enable, before);
      auto &router = f.tech.route;
      router.moving_stage = router.moving_sources_stage = true;
      router.route_preemption_enabled = router.route_deadends_enabled = false;
      const auto old_tile = f.source->tile.peer;
      const auto old_coord = f.source->coord;
      const auto old_pos = f.source->pos;
      f.source->tile.clear();
      f.source->tile.set(static_cast<Referable<fpga::Tile> *>(&f.tile({1, 0})));
      f.source->coord = {1, 0};
      f.source->pos = 0;
      pnr::RouteDesign::RouteTask input{
          driver, f.source, f.enable, "O", "CE", "numeric_control_input"};
      pnr::MovingPlacementBlacklist blacklist({{f.source}});
      const bool possible =
          router.movingInputMayRoute(input, {f.source}, {}, blacklist);
      f.source->tile.clear();
      f.source->tile.set(old_tile);
      f.source->coord = old_coord;
      f.source->pos = old_pos;
      std::cout << "connected=" << connected << " numeric=" << numeric
                << " input_possible=" << possible << '\n';
      // The same retained landing must remain a usable branch in both
      // encodings.
      require(
          possible == connected,
          "retained input anchor reachability depends on fragment encoding");
      std::ostringstream after;
      fpga::dumpNetRouteHistory(*f.enable, after);
      require(before.str() == after.str(),
              "input preflight changed the retained tree");
      f.auditAll();
      // Neither representation may manufacture a connection to the closed
      // branch.
      if (!connected)
        continue;
      pnr::RouteDesign::RouteTask output{f.source, f.sink, f.signal,
                                         "Q",      "I0",   "numeric_output"};
      output.endpoints_prepared = true;
      router.iteration_limit = router.route_suffix_depth_limit = 200;
      bool moved = false;
      std::string reason;
      // Exercise the second anchor collector too: its full input proof must
      // branch from the numeric landing, not merely accept the preflight.
      require(router.moveUnfinishedSource(output, nullptr, &reason, nullptr,
                                          &moved),
              reason);
      require(
          moved && f.source->tile.peer == &f.tile({1, 0}),
          "numeric input anchor did not permit the known feasible relocation");
      require(fpga::isRouteComplete(sibling->wires[0]),
              "relocation damaged its retained sibling");
      bool input_complete = false;
      for (const auto &binding : f.enable->routes)
        if (binding.to == f.source && binding.to_port == "CE" &&
            binding.owner && binding.route_index < binding.owner->wires.size())
          input_complete |=
              fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
      // Accepting a new position requires the incident input to be routed now.
      require(input_complete,
              "source relocation left its input deferred or incomplete");
      f.auditAll();
    }
}
