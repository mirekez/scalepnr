#include "SourceInputFixture.h"

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
      const auto sibling_before=sibling->wires[0];
      auto &router = f.tech.route;
      router.moving_stage = router.moving_sources_stage = true;
      router.route_preemption_enabled = router.route_deadends_enabled = false;
      // This is the post-placement input phase, not a hypothetical proof.
      // Both route encodings must expose the same live sibling branch point.
      require(f.tile({7,0}).unassign(f.source) &&
                  f.tile({1,0}).tryAddAt(f.source,0)==0,
              "cannot commit the input receiver's selected placement");
      pnr::RouteDesign::RouteTask input{
          driver, f.source, f.enable, "O", "CE", "numeric_control_input"};
      input.endpoints_prepared=true;
      router.iteration_limit=router.route_suffix_depth_limit=200;
      std::vector<pnr::RouteDesign::RouteTask> inputs{input};
      std::string reason;
      const bool complete=router.moveInputSources(inputs,&reason);
      std::cout << "connected=" << connected << " numeric=" << numeric
                << " input_complete=" << complete << " reason=" << reason << '\n';
      // The same retained landing must remain a usable branch in both
      // encodings.
      require(
          complete == connected,
          "recursive input routing depends on fragment encoding: "+reason);
      require(f.source->tile.peer==&f.tile({1,0}),
          "input routing displaced its committed receiver");
      require(fpga::isRouteComplete(sibling->wires[0]),
              "relocation damaged its retained sibling");
      bool input_complete = false;
      for (const auto &binding : f.enable->routes)
        if (binding.to == f.source && binding.to_port == "CE" &&
            binding.owner && binding.route_index < binding.owner->wires.size())
          input_complete |=
              fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
      // Accepting a new position requires the incident input to be routed now.
      require(input_complete==connected,
              "input completion disagrees with the actual branch connectivity");
      // Compare the original sibling itself: adding a new binding to its net
      // legitimately changes whole-net history, but not its retained route.
      require(driver->coord==fpga::Coord{6,0} && sibling->coord==fpga::Coord{3,0},
              "recursive input routing moved a fixed sibling endpoint");
      require(sibling_before.size()==sibling->wires[0].size(),
              "recursive input routing resized the sibling trunk");
      for(size_t i=0;i<sibling_before.size();++i) {
        const auto& a=sibling_before[i];const auto& b=sibling->wires[0][i];
        require(a.type==b.type && a.from==b.from && a.to==b.to &&
            a.local==b.local && a.jump==b.jump && a.dst==b.dst &&
            a.joint==b.joint && a.joint2==b.joint2 &&
            a.from_node_type==b.from_node_type && a.from_node==b.from_node &&
            a.to_node_type==b.to_node_type && a.to_node==b.to_node &&
            a.shared==b.shared && a.owns_dst==b.owns_dst && a.owns_landing==b.owns_landing,
            "recursive input routing changed a sibling node or lease");
      }
      f.auditAll();
    }
}
