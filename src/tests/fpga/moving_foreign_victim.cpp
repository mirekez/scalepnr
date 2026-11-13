#include "SourceInputFixture.h"

#include <filesystem>

using routing_tests::SourceInputFixture;
using routing_tests::require;

void moving_foreign_victim_regression() {
  // Example 1: a committed move preempted a different net. Its retained
  // prefix and suffix could not be joined at the original placement. Both
  // relocation fallbacks compared the victim endpoints to moving_focus_inst,
  // so neither ran. A valid destination slot was never searched.
  // Use a real transit cut and a different registered owner for the blocked
  // hop; no fabricated busy bits or device-specific wire labels.
  const auto artifacts = std::filesystem::temp_directory_path() /
      ("scalepnr-foreign-victim-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(artifacts);
  setenv("SCALEPNR_FAILURE_ARTIFACT_DIR", artifacts.c_str(), 1);
  for (bool recover : {false, true}) {
    SourceInputFixture f(false, 0, 16);
    for (fpga::Coord at : {fpga::Coord{5,2}, fpga::Coord{5,1}}) {
      fpga::Element e; e.name="pattern_logic_41"; e.type=fpga::ELEMENT_LUT5;
      f.tile(at).tile_type->elements.push_back(e);
      e.name="pattern_extension_43"; e.type=fpga::ELEMENT_LUT1;
      f.tile(at).tile_type->elements.push_back(e);
    }
    auto* driver=f.cell("pattern_driver_47","LUT6",{{"O",true}},{5,2});
    auto* sink=f.cell("pattern_sink_53","LUT6",{{"I0",false}},{5,1});
    auto* competitor=f.connect(*driver,"O",*sink,"I0");
    std::vector<fpga::RouteCutNode> cuts{{{6,1},fpga::CB_NODE_SRC,911}};
    const auto victims=fpga::findNetRouteCutVictims(cuts);
    require(fpga::unrouteNetRoutesFromNodes(victims,cuts,true),"cannot cut registered victim transit");
    require(fpga::hasRouteGaps(f.source->wires[0]),"cut did not retain a real gap");
    std::vector<fpga::Wire> route{
      SourceInputFixture::pin({5,2},169,true),
      f.edge({5,2},{6,1},169,1201,1202,true),
      f.edge({6,1},{5,1},1202,911,711),
      f.terminal({5,1},711,87),SourceInputFixture::pin({5,1},87,false)};
    for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(route),"cannot install competing route");
    sink->wires.push_back(std::move(route));
    const auto id=fpga::attachNetRoute(*competitor,*sink,0,driver,sink,"O","I0",competitor->name);
    fpga::registerNetRouteTiles(*competitor,sink->wires[0],id);
    f.auditAll();
    const auto victim_id=f.enable->routes[0].route_id;
    f.source->outline.fixed=f.source->pnr_completed=true;
    f.control->pnr_completed=true;
    auto& router=f.tech.route;
    router.moving_stage=true;
    router.moving_focus_inst=f.sink; // Neither endpoint of the damaged control net.
    router.route_preemption_enabled=router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    router.complete_routes_only=router.preserve_routes=true;
    pnr::RouteDesign::RouteTask task{f.control,f.source,f.enable,"O","CE","control_tree"};
    task.endpoints_prepared=true;
    require(!router.completeTaskFromSource(task),"blocked victim unexpectedly has a wire-only bypass");
    bool done=false; std::string failure;
    try { done=router.routeTaskOrFail(task,"Moving destinations",5,true,recover); }
    catch(const std::exception& e) { failure=e.what(); }
    if (recover) {
      require(done,"unrelated moving focus excluded victim recovery: "+failure);
      require(f.source->tile.peer==&f.tile({1,0}),"recovery missed the reachable register bank");
      require(f.enable->routes[0].route_id==victim_id && fpga::isRouteComplete(f.source->wires[0]),
          "recovery lost the victim binding or left its gap unfinished");
      require(router.moving_focus_inst==f.sink,"recovery leaked its temporary focus");
    } else {
      require(!done && !failure.empty() && f.source->tile.peer==&f.tile({7,0}),
          "nested wire-only repair relocated an endpoint");
    }
    require(fpga::isRouteComplete(sink->wires[0]),"recovery damaged the competing route");
    require(f.control->tile.peer==&f.tile({6,0}) && f.sink->tile.peer==&f.tile({0,0}),
        "recovery moved an unrelated cell");
    f.auditAll();
  }
  std::filesystem::remove_all(artifacts);
}
