#include "SourceInputFixture.h"

using namespace routing_tests;

// Source routing can find a free output from a new register bank while its
// CE needs ordinary endpoint preemption. The CE obstruction below is a real
// transit, with a retained Takeoff and a free bypass. A free-only CE proof is
// not proof that this bank cannot be used. All node names are synthetic.
void source_input_preemption_regression(int mode) {
  const bool control_only=mode==1;
  SourceInputFixture f(false,0,8);
  for (auto [at,type]:std::vector<std::pair<fpga::Coord,fpga::ElementType>>{
      {{4,2},fpga::ELEMENT_FD},{{5,2},fpga::ELEMENT_LUT5},{{5,2},fpga::ELEMENT_LUT1}}) {
    fpga::Element e;e.name="transit_owner_slot_"+std::to_string(type);e.type=type;
    f.tile(at).tile_type->elements.push_back(e);
  }
  f.tile({4,2}).tile_type->pin_map.output_nodes[f.tile({4,2}).getNodeNum("FDRE","Q",0)].setBit(170);
  f.tile({4,2}).cb_type->local_output_nodes.setBit(170);
  auto* driver=f.cell("transit_driver_47","FDRE",{{"Q",true}},{4,2});
  auto* receiver=f.cell("transit_receiver_53","LUT6",{{"I0",false}},{5,2});
  auto* victim=f.connect(*driver,"Q",*receiver,"I0");
  std::vector<fpga::Wire> route{SourceInputFixture::pin({4,2},170,true),
      f.edge({4,2},{6,0},170,1500,1503,true),
      f.edge({6,0},{1,0},1503,900,700),
      f.edge({1,0},{5,2},700,1501,1502),
      f.terminal({5,2},1502,87),SourceInputFixture::pin({5,2},87,false)};
  f.edge({6,0},{5,2},1503,1504,1505);f.terminal({5,2},1505,87);
  if(mode==2)f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);
  for(auto&cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(route),"cannot install CE transit obstruction");
  receiver->wires.push_back(std::move(route));
  auto id=fpga::attachNetRoute(*victim,*receiver,0,driver,receiver,"Q","I0",victim->name);
  fpga::registerNetRouteTiles(*victim,receiver->wires[0],id);f.auditAll();
  auto& router=f.tech.route;router.moving_stage=router.moving_sources_stage=!control_only;
  router.complete_routes_only=router.preserve_routes=true;
  router.route_deadends_enabled=false;router.route_preemption_enabled=true;
  router.iteration_limit=router.route_suffix_depth_limit=200;router.route_recursion_budget=200;
  if(control_only) {
    auto* probe=f.cell("control_input_probe_59","FDRE",{{"CE",false}},{1,0});
    auto& input=SourceInputFixture::port(*probe,"CE");
    input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
    input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
    pnr::RouteDesign::RouteTask task{f.control,probe,f.enable,"O","CE","control_grounding_reference"};
    require(router.routeTaskOrFail(task,"control grounding reference",200),
        "ordinary routing cannot preempt the CE transit in the reference");
  } else {
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","register_output_61"};
    pnr::MovingPlacementBlacklist placements({{f.source}});
    std::string reason;size_t released=0,retries=0;bool moved=false,retryable=false;
    bool done=router.moveUnfinishedSource(task, nullptr, &reason, &released, &moved, &retryable, nullptr, false, &placements, &retries);
    f.auditAll();
    if(mode==2) {
      require(!done,"disconnected control input unexpectedly routed");
      // Input failure follows a real parent move. Backward Grounding may
      // release the input's transit obstruction before discovering that its
      // source is disconnected; no speculative end-to-end proof precedes it.
      require(moved && !reason.empty(),"disconnected input did not fail after its parent attempt");
      require(std::any_of(router.pending_route_todo.begin(),router.pending_route_todo.end(),
          [&](const auto& pending){return pending.net==f.enable && pending.to==f.source &&
              pending.to_port=="CE" && !router.routeTaskIsComplete(pending);}),
          "failed recursive CE input disappeared from pending work");
      const auto& binding=victim->routes[id];
      require(binding.owner && binding.route_index<binding.owner->wires.size(),
          "failed recursive input lost its transit victim binding");
      const auto& retained=binding.owner->wires[binding.route_index];
      require(fpga::isRouteComplete(retained) ||
          std::any_of(router.pending_route_todo.begin(),router.pending_route_todo.end(),
              [&](const auto& pending){return pending.net==victim && pending.from==driver &&
                  pending.to==receiver && pending.from_port=="Q" && pending.to_port=="I0";}),
          "failed recursive input lost the displaced transit task");
      // Grounding may cut only the transit suffix, never its driver's takeoff.
      require(retained.size()>=2 && retained.front().from==fpga::Coord{4,2} &&
          f.tile({4,2}).cb.src.jump.testBit(1500),
          "failed recursive input released its victim's takeoff");
      f.auditAll();return;
    }
    require(done,"Source Moving rejected the only legal bank: CE needs eligible transit preemption: "+reason);
    require(f.source->tile.peer==&f.tile({1,0}),"Source Moving missed the reference register bank");
    auto complete=[&](rtl::Net& net,rtl::Inst* from,rtl::Inst* to,const char* input) {
      return std::any_of(net.routes.begin(),net.routes.end(),[&](const auto& binding) {
        return binding.from==from && binding.to==to && binding.to_port==input &&
            binding.owner && binding.route_index<binding.owner->wires.size() &&
            fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
      });
    };
    require(complete(*f.signal,f.source,f.sink,"I0"),"moved register output stayed unfinished");
    require(complete(*f.enable,f.control,f.source,"CE"),"moved register CE stayed unfinished");
  }
  require(router.route_stats.preempt_success>0,"reference did not actually preempt CE transit");
  pnr::RouteDesign::RouteTask repair{driver,receiver,victim,"Q","I0",victim->name};
  require(router.routeTaskOrFail(repair,"repair CE transit victim",200),"transit victim cannot take its free bypass");
  require(fpga::isRouteComplete(receiver->wires[victim->routes[0].route_index]),"transit victim stayed disconnected");
  f.auditAll();
}
