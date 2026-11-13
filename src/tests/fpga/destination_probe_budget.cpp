#include "SourceInputFixture.h"
#include "InputRepairBudget.h"
#include "RepairStatistics.h"
#include <iostream>

using routing_tests::SourceInputFixture;
using routing_tests::require;

// A real transit owner blocks the old CE connection, while a nearby register
// bank is reachable. A broad dead-end component makes preserving the original
// placement expensive. All free/gap/source searches must share one allowance;
// reaching it must hand this same task to relocation, not cancel or defer it.
// Node and instance labels are synthetic patterns, not device wire names.
void destination_probe_budget_regression() {
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
    // Keep a reachable free tree beyond the production allowance. A shallow
    // tree avoids confusing the expansion budget with the route hop limit.
    constexpr int count=65535, first_leaf=(count-1)/2;
    auto at=[](int i) { return fpga::Coord{8+i/CB_MAX_NODES,0}; };
    auto node_id=[](int i) { return i%CB_MAX_NODES; };
    f.resolvedLocal({6,0},169,at(0),fpga::CB_NODE_LOCAL,0);
    for(int i=0;i<first_leaf;++i)
      for(int child : {2*i+1,2*i+2})
        f.resolvedLocal(at(i),node_id(i),at(child),
            child>=first_leaf?fpga::CB_NODE_SRC:fpga::CB_NODE_LOCAL,node_id(child));
    for(auto& cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    f.source->outline.fixed=f.source->pnr_completed=fixed;
    auto& router=f.tech.route;
    router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
    router.route_preemption_enabled=router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    pnr::RouteDesign::RouteTask task{f.control,f.source,f.enable,"O","CE","control_tree"};
    task.endpoints_prepared=true;
    const auto revision=fpga::routeMutationRevision();
    pnr::InputRepairBudget budget{pnr::MAX_PRE_RELOCATION_REPAIR_EXPANDED_STATES};
    fpga::resetRepairStatistics();
    require(!router.recoverBeforeMoving(task,&budget),"blocked input unexpectedly repaired in place");
    const auto& work=fpga::repairStatistics();
    std::cout<<"destination probe fixed="<<fixed<<" expanded="<<budget.expanded
        <<" limit="<<budget.limit<<" exhausted="<<budget.defer_inputs
        <<" gap_expanded="<<work[fpga::RepairCounter::gap_expanded]
        <<" source_expanded="<<work[fpga::RepairCounter::source_expanded]<<'\n';
    require(budget.expanded>budget.limit/3 && budget.expanded<=budget.limit &&
        work[fpga::RepairCounter::source_limit]>0,
        "pre-relocation searches did not exercise a bounded broad frontier");
    require(work[fpga::RepairCounter::gap_expanded]+work[fpga::RepairCounter::source_expanded]==budget.expanded,
        "reported allowance does not account for all expanded states");
    if(fixed) require(budget.expanded==budget.limit && budget.defer_inputs,
        "fixed-cell retry received a fresh expansion allowance");
    require(fpga::routeMutationRevision()==revision && router.pending_route_todo.empty(),
        "failed probe changed leases or deferred current work");
    require(!router.routeStageSearchCancelled() && !pnr::InputRepairBudget::stopped(),
        "probe cutoff cancelled relocation or leaked its scope");
    require(f.source->coord==fpga::Coord{7,0} && fpga::hasRouteGaps(f.source->wires[0]),
        "probe changed placement or claimed unfinished work complete");
    f.auditAll();
    bool wire_only=false;
    std::string reason;
    std::vector<pnr::RouteDesign::RouteTask> affected;
    require(router.moveUnfinishedCell(task,&affected,&task,&reason,false,&wire_only),
        "budget cutoff prevented destination relocation: "+reason);
    require(!wire_only && f.source->coord==fpga::Coord{1,0} &&
        fpga::isRouteComplete(f.source->wires[0]),
        "destination move did not complete the same connection at the reachable bank");
    require(fpga::isRouteComplete(sink->wires[0]),"relocation damaged competing route");
    require(!router.routeStageSearchCancelled(),"relocation inherited probe cancellation");
    f.auditAll();
  }
}
