#include "SourceInputFixture.h"
#include "RepairStatistics.h"
#include <iostream>

using routing_tests::SourceInputFixture;
using routing_tests::require;

// A real transit owner blocks the old CE connection, while a nearby register
// bank is reachable. Moving must reach that placement by angle-led tracing,
// without a broad input proof or a pre-relocation search budget.
// Node and instance labels are synthetic patterns, not device wire names.
void destination_direct_move_regression() {
  const char* old_profile=std::getenv("SCALEPNR_PROFILE_REPAIR");
  struct RestoreProfile {
    bool present; std::string value;
    ~RestoreProfile() { if(present)setenv("SCALEPNR_PROFILE_REPAIR",value.c_str(),1);else unsetenv("SCALEPNR_PROFILE_REPAIR"); }
  } restore{old_profile!=nullptr,old_profile?old_profile:""};
  setenv("SCALEPNR_PROFILE_REPAIR","1",1);
  for (bool fixed : {true, false}) {
    SourceInputFixture f(false, 0, 16, 24);
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
    f.source->outline.fixed=f.source->pnr_completed=fixed;
    auto& router=f.tech.route;
    router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
    router.route_preemption_enabled=router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    pnr::RouteDesign::RouteTask task{f.control,f.source,f.enable,"O","CE","control_tree"};
    task.endpoints_prepared=true;
    fpga::resetRepairStatistics();
    bool wire_only=false;
    std::string reason;
    std::vector<pnr::RouteDesign::RouteTask> affected;
    require(router.moveUnfinishedCell(task,&affected,&task,&reason,false,&wire_only),
        "angle-led destination relocation failed: "+reason);
    require(!wire_only && f.source->coord==fpga::Coord{1,0} &&
        fpga::isRouteComplete(f.source->wires[0]),
        "destination move did not complete the same connection at the reachable bank");
    require(fpga::isRouteComplete(sink->wires[0]),"relocation damaged competing route");
    require(!router.routeStageSearchCancelled(),"relocation unexpectedly cancelled");
    const auto& work=fpga::repairStatistics();
    require(work[fpga::RepairCounter::source_calls]==0 &&
        work[fpga::RepairCounter::gap_calls]==0 &&
        work[fpga::RepairCounter::two_ended_calls]==0,
        "destination relocation performed a broad route proof");
    f.auditAll();
  }
}
