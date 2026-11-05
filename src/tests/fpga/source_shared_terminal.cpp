#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"

// A relocation can share an input terminal already driven by the same tree.
// Its preflight must agree with the private proof and the committed binding;
// demanding a second free ingress rejects an otherwise compatible placement.
void source_shared_terminal_regression() {
  using namespace routing_tests;
  SourceInputFixture f(false);
  // Use the production connection name for the pre-existing input binding.
  auto& old_binding=f.enable->routes.front();
  old_binding.route_name=SourceInputFixture::port(*f.source,"CE").makeNetName();
  f.enable->invalidateRouteLookup();
  auto& target=f.tile({1,0});
  fpga::Element e;e.name="resident_slot";e.type=fpga::ELEMENT_LUT5;
  target.tile_type->elements.push_back(e);
  e.name="resident_extension";e.type=fpga::ELEMENT_LUT1;e.bitmap_pos=0;
  target.tile_type->elements.push_back(e);
  target.tile_type->pin_map.nodes[{"LUT6","I0",3}]={};
  target.tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(63);
  auto* resident=f.cell("resident_receiver","LUT6",{{"I0",false}},{1,0});
  resident->outline.fixed=true;
  SourceInputFixture::port(*resident,"I0").port_ref->designator=f.enable->designators.front();
  SourceInputFixture::port(*resident,"I0").set(
      &rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));

  // Install a genuine branch of the existing input tree, not an orphan busy bit.
  std::vector<fpga::Wire> route(f.source->wires[0].begin(),f.source->wires[0].begin()+2);
  for(auto& w:route){w.shared=true;w.owns_landing=false;w.owns_dst=false;}
  auto fork=f.edge({6,1},{1,0},710,950,700);fork.pos=2;fork.owns_dst=false;
  route.push_back(fork);route.push_back(f.terminal({1,0},700,63));
  route.push_back(f.pin({1,0},63,false));
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  std::vector<fpga::Wire> suffix(route.begin()+2,route.end());
  require(pnr::commitPreparedRoute(suffix),"cannot lease resident input branch");
  resident->wires.push_back(route);
  const auto binding=fpga::attachNetRoute(*f.enable,*resident,0,f.control,resident,"O","I0","resident_signal");
  fpga::registerNetRouteTiles(*f.enable,resident->wires[0],binding);
  f.tech.route.indexSourceRoute(f.enable,f.control,"O");
  f.auditAll();
  // The candidate has a legal output, and the shared input has a real owner.
  require(target.isPinNodeLeased(63),"resident terminal was not occupied");
  auto& router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=true;router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","output_signal"};
  task.endpoints_prepared=true;
  pnr::MovingPlacementBlacklist work({{f.source}});
  std::string reason;
  const bool done=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,nullptr,
      nullptr,nullptr,false,&work);
  std::cout<<"shared input move="<<done<<" reason="<<reason<<'\n';
  // A same-signal occupied terminal must not make this unique useful bank fail.
  require(done && f.source->tile.peer==&target,"shared-terminal placement rejected: "+reason);
  require(fpga::isRouteComplete(resident->wires[0]),"resident input was disrupted");
  // Every completed branch must retain a real owner for all shared leases.
  f.auditAll();
  bool complete=false;
  size_t moved_binding=f.enable->routes.size();
  for(size_t n=0;n<f.enable->routes.size();++n) {
    const auto& b=f.enable->routes[n];
    if(b.to==f.source && b.owner && b.route_index<b.owner->wires.size()) {
      complete|=fpga::isRouteComplete(b.owner->wires[b.route_index]);moved_binding=n;
    }
  }
  require(complete,"moved input was not attached to the existing terminal");
  // Removing the new replica must leave the original terminal and tree live.
  require(fpga::unrouteNetRoute(*f.enable,moved_binding),"cannot remove moved input replica");
  require(target.isPinNodeLeased(63) && fpga::isRouteComplete(resident->wires[0]),
      "removing the moved input released the resident's terminal");
  f.auditAll();
}
