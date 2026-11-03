#include "SourceInputFixture.h"
#include <cmath>

using namespace routing_tests;

// Older runs freeze placement, but later routing can preempt their transit.
// A fixed register must still be eligible for backward route-first relocation.
// The original position cannot route Q; (1,0) can route both Q and CE.
void moving_fixed_source_regression() {
  SourceInputFixture f(false);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  f.source->outline.fixed=f.source->pnr_completed=true;
  for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","fixed_register_output"};
  task.endpoints_prepared=true;
  bool moved=false;std::string reason;
  require(router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved),
      "fixed register was excluded from backward relocation: "+reason);
  require(moved && f.source->tile.peer==&f.tile({1,0}),
      "backward search missed the only feasible fixed-register placement");
  require(f.source->outline.fixed && !f.source->pnr_completed,
      "relocated register lost placement protection or kept stale run completion");
  require(std::isfinite(f.source->outline.x) && std::isfinite(f.source->outline.y),
      "standalone source move produced an invalid placement target");
  require(f.sink->tile.peer==&f.tile({0,0}) && f.control->tile.peer==&f.tile({6,0}),
      "source recovery displaced an unrelated fixed endpoint");
  bool signal=false,enable=false;
  for(auto *net:{f.signal,f.enable})for(auto &binding:net->routes) {
    require(binding.owner && binding.route_index<binding.owner->wires.size() &&
        fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
        "fixed source move left an incomplete output or CE input");
    signal|=net==f.signal;enable|=net==f.enable;
  }
  require(signal && enable,"fixed-source test missed output/control routing");
  require(router.route_stats.preempt_success==0,"free fixed-source move preempted another signal");
  f.audit();
}

// Reproduce the policy gate from the failed old-run LUT: the current position
// has no incoming path, while a reached nearby Tile has a free complete LUT6
// pair. Run the public destination-recovery entry, not just a packing helper.
void moving_fixed_destination_regression() {
  SourceInputFixture f(false,0,16);
  auto logic=[&](fpga::Coord at) {
    for(auto type:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}) {
      fpga::Element e;e.name="logic_pair_"+std::to_string(type);e.type=type;
      f.tile(at).tile_type->elements.push_back(e);
    }
    f.tile(at).tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(87);
    f.tile(at).cb_type->local_input_nodes.setBit(87);
  };
  logic({0,1});logic({4,1});
  auto *load=f.cell("completed_logic_83","LUT6",
      {{"I0",false},{"I1",false},{"I2",false},{"I3",false},
       {"I4",false},{"I5",false},{"O",true}},{0,1});
  auto &input=SourceInputFixture::port(*load,"I0");
  input.port_ref->designator=SourceInputFixture::port(*f.source,"Q").port_ref->designator;
  input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.source,"Q")));
  load->outline.fixed=load->pnr_completed=true;
  f.source->outline.fixed=f.source->pnr_completed=true;
  f.edge({7,0},{4,1},141,201,202,true);f.terminal({4,1},202,87);
  for(auto &cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto &router=f.tech.route;
  router.moving_stage=router.complete_routes_only=true;
  router.route_recursion_budget=16;
  router.route_preemption_enabled=router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  pnr::RouteDesign::RouteTask task{f.source,load,f.signal,"Q","I0","fixed_logic_input"};
  task.endpoints_prepared=true;
  std::vector<pnr::RouteDesign::RouteTask> affected;std::string reason;
  require(router.moveUnfinishedDestination(task,&affected,&reason,false),
      "fixed LUT was excluded from forward relocation: "+reason);
  require(load->tile.peer==&f.tile({4,1}) && load->pos==3,
      "forward search missed the only reached full-LUT placement");
  require(load->outline.fixed && !load->pnr_completed,
      "relocated LUT lost placement protection or kept stale run completion");
  require(std::isfinite(load->outline.x) && std::isfinite(load->outline.y),
      "standalone destination move produced an invalid placement target");
  require(f.source->tile.peer==&f.tile({7,0}) && f.source->pnr_completed,
      "destination recovery moved or reopened its untouched fixed driver");
  const auto binding=f.signal->findRouteBinding(f.source,load,"Q","I0",task.net_name);
  require(binding.index<f.signal->routes.size(),"fixed destination move omitted its route binding");
  const auto &b=f.signal->routes[binding.index];
  require(b.owner && fpga::isRouteComplete(b.owner->wires[b.route_index]),
      "fixed destination move published an incomplete connection");
  require(router.route_stats.preempt_success==0,"free fixed-destination move preempted another signal");
  f.audit();
}
