#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"

using routing_tests::SourceInputFixture;
using routing_tests::require;

namespace {
size_t count(const std::string &text, const std::string &needle) {
  size_t result=0, at=0;
  while ((at=text.find(needle,at))!=std::string::npos) { ++result; at+=needle.size(); }
  return result;
}
void secondRegister(SourceInputFixture &f, fpga::Coord at) {
  fpga::Element slot; slot.name="register_lane_2"; slot.type=fpga::ELEMENT_FD;
  slot.bitmap_pos=1;
  f.tile(at).tile_type->elements.push_back(slot);
  f.tile(at).invalidatePlacementCaches();
  // Separate Q outputs, but one shared physical CE input for both registers.
  f.tile(at).tile_type->pin_map.nodes[{"FDRE","CE",4}].setBit(63);
  f.tile(at).tile_type->pin_map.output_nodes[
      f.tile(at).getNodeNum("FDRE","Q",4)].setBit(142);
}
void finalize(SourceInputFixture &f) {
  for(auto &type:f.device.cb_types) type.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
}
void auditPreserved(SourceInputFixture &f) {
  f.auditAll();
  for(auto &net:f.parent->nets) for(auto &binding:net.routes) {
    const bool background=binding.from && binding.from->cell_ref->name.starts_with("background_driver_");
    const bool sibling=binding.to && binding.to->cell_ref->name=="control_receiver_47";
    if (background || sibling)
      require(binding.owner && binding.route_index<binding.owner->wires.size() &&
          fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
          "blacklist search damaged a retained CE sibling or background transit");
  }
}
void compareInputWalks(SourceInputFixture &f,rtl::Inst &moving,fpga::Coord bad) {
  // Compare the identical real graph with fresh versus shared blacklist state.
  // The actual Moving searches below separately verify integration/order.
  struct Restore {
    rtl::Inst &inst; Referable<fpga::Tile> *tile; fpga::Coord coord; int pos;
    ~Restore() { inst.tile.clear(); inst.tile.set(tile); inst.coord=coord; inst.pos=pos; }
  } restore{moving,moving.tile.peer,moving.coord,moving.pos};
  auto *driver=SourceInputFixture::port(moving,"CE").follow();
  require(driver && driver->inst_ref.peer,"missing real CE driver");
  pnr::RouteDesign::RouteTask input{driver->inst_ref.peer,&moving,f.enable,
      driver->port_ref->makeName(),"CE","blacklist_comparison"};
  pnr::MovingPlacementBlacklist shared({});
  size_t uncached_walks=0;
  for(bool reuse:{false,true}) for(int attempt=0;attempt<64;++attempt) {
    moving.tile.clear(); moving.tile.set(static_cast<Referable<fpga::Tile>*>(&f.tile(bad)));
    moving.coord=bad; moving.pos=(attempt%2)*4;
    pnr::MovingPlacementBlacklist fresh({});
    auto &blacklist=reuse?shared:fresh;
    const auto before=blacklist.input_proofs;
    require(!f.tech.route.movingInputMayRoute(input,{&moving},{},blacklist),
        "disconnected CE was reachable in comparison control");
    if(!reuse) uncached_walks+=blacklist.input_proofs-before;
  }
  std::cout << "CE proof comparison: uncached=" << uncached_walks
            << " shared=" << shared.input_proofs << " avoided=" << shared.input_hits << '\n';
  require(uncached_walks==64 && shared.input_proofs==1 && shared.input_hits==63,
      "blacklist failed to eliminate repeated real CE walks");
}
rtl::Net *blockBranch(SourceInputFixture &f,fpga::Coord landing,int src) {
  // Occupy the known good CE branch with a complete unrelated transit, then
  // remove only that route between searches. Topology remains immutable.
  secondRegister(f,{3,1});
  auto *driver=f.cell("branch_blocker_91","FDRE",{{"Q",true}},{3,1},4);
  driver->outline.fixed=true;
  rtl::Inst *sink=nullptr;
  for(auto &inst:f.tech.design.top.insts)
    if(inst.cell_ref->name=="background_receiver_2") sink=&inst;
  require(sink!=nullptr,"missing blocker sink");
  auto *net=f.connect(*driver,"Q",*sink,"I2");
  std::vector<fpga::Wire> route{SourceInputFixture::pin({3,1},142,true),
      f.edge({3,1},landing,142,src,src+1,true),
      f.edge(landing,{7,2},src+1,src+2,src+3),
      f.terminal({7,2},src+3,214),SourceInputFixture::pin({7,2},214,false)};
  finalize(f);
  require(pnr::commitPreparedRoute(route),"cannot install branch blocker");
  sink->wires.push_back(std::move(route));
  const auto id=fpga::attachNetRoute(*net,*sink,sink->wires.size()-1,driver,sink,"Q","I2",
      SourceInputFixture::port(*sink,"I2").makeNetName());
  fpga::registerNetRouteTiles(*net,sink->wires.back(),id);
  auditPreserved(f);
  return net;
}
}

void moving_source_input_blacklist_regression() {
  SourceInputFixture f(false,0,32);
  f.installCEFrontier(false,{2,0});
  f.edge({3,1},{1,0},713,1780,1781); f.terminal({1,0},1781,63);
  auto *blocker=blockBranch(f,{1,0},1780);
  secondRegister(f,{2,0});
  for(int n=0;n<32;++n) f.tile({2,0}).cb_type->local_src[142].jump.setBit(100+n);
  finalize(f);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  compareInputWalks(f,*f.source,{2,0});
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","source_blacklist_signal"};
  task.endpoints_prepared=true;
  pnr::MovingPlacementBlacklist blacklist({{f.source}});
  require(blacklist.rejects(f.tile({0,0})),"packing control unexpectedly accepts a register");
  std::ostringstream closed_trace; bool closed_moved=false; std::string closed_reason;
  {
    pnr::RouteCongestionTrace capture(task.net_name,"source_blacklist_closed",{},&closed_trace);
    require(!router.moveUnfinishedSource(task,nullptr,&closed_reason,nullptr,&closed_moved,
        nullptr,nullptr,false,&blacklist),"closed CE graph unexpectedly relocates source");
  }
  require(!closed_moved && f.source->tile.peer==&f.tile({7,0}),"closed proof changed source placement");
  require(count(closed_trace.str(),"PLACEMENT_INPUT_PROOF target=(2,0)")==1,
      "closed source search repeated the physical CE proof");
  auditPreserved(f);
  // Change reachability, then retry with the SAME packing blacklist. Routing
  // failures must be invalidated, while pure packing rejections remain valid.
  fpga::unrouteNet(*blocker);
  std::ostringstream trace; bool moved=false; std::string reason;
  {
    pnr::RouteCongestionTrace capture(task.net_name,"source_blacklist",{f.signal,f.enable},&trace);
    require(router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved,
        nullptr,nullptr,false,&blacklist),reason);
  }
  // Thirty-two output exits and two legal register slots share an unreachable
  // CE pin at Tile(2,0). One independent CE proof must reject ALL
  // those placements. Search must still reach the feasible source at (1,0).
  const size_t walks=count(trace.str(),"BACKWARD_BEGIN target=(2,0)");
  const auto skips=count(trace.str(),"PLACEMENT_INPUT_BLACKLIST");
  std::cout << "source blacklist: exits=32 slots=2 CE_walks=" << walks
            << " input_proof_skips=" << skips << '\n';
  require(moved && f.source->tile.peer==&f.tile({1,0}),"source blacklist hid a feasible placement");
  require(walks<=2 && walks>0,"source repeated CE searches for equivalent physical pins: "+std::to_string(walks));
  require(count(trace.str(),"PLACEMENT_INPUT_PROOF target=(2,0)")==1 && skips>=1,
      "source did not share its failed physical CE proof");
  const auto packing_checks=blacklist.checks;
  require(blacklist.rejects(f.tile({0,0})) && blacklist.checks==packing_checks,
      "source routing retry discarded its unchanged packing blacklist");
  auditPreserved(f);
}

void moving_destination_input_blacklist_regression() {
  for(bool shared_ce:{false,true}) {
  SourceInputFixture f(false,0,32);
  auto [control,sibling]=f.installCEFrontier(false);
  auto &router=f.tech.route;
  router.moving_stage=true;
  router.complete_routes_only=true;
  router.route_recursion_budget=16;
  router.route_preemption_enabled=router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  f.source->outline.fixed=true;
  // The trigger is D; CE is a different incident input. The bad Tile offers
  // thirty-two genuinely reachable D arrivals and two register slots. Fourteen
  // CE ingresses have real owners; two are free but cannot reach the driver.
  // The next Tile can accept both D and CE via its retained branch.
  for(fpga::Coord at:std::array<fpga::Coord,1>{{{5,2}}}) {
    fpga::Element slot; slot.name="destination_register"; slot.type=fpga::ELEMENT_FD;
    f.tile(at).tile_type->elements.push_back(slot);
    f.tile(at).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(87);
    f.tile(at).cb_type->local_input_nodes.setBit(87);
  }
  secondRegister(f,{1,0});
  f.tile({1,0}).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(87);
  f.tile({1,0}).tile_type->pin_map.nodes[{"FDRE","D",4}].setBit(87);
  f.tile({2,0}).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(87);
  auto *load=f.cell("movable_receiver_71","FDRE",{{"D",false},{"CE",false}},{2,0});
  // Reuse the real CE net with its complete retained sibling, and attach an
  // empty binding so production Moving must account for this additional input.
  auto &ce=SourceInputFixture::port(*load,"CE");
  ce.port_ref->designator=SourceInputFixture::port(*f.source,"CE").port_ref->designator;
  ce.set(&rtl::Conn::fromBase(SourceInputFixture::port(*control,"O")));
  load->wires.emplace_back();
  fpga::attachNetRoute(*f.enable,*load,0,control,load,"O","CE",ce.makeNetName());
  // Add another sink to Q's existing net, preserving the fixed original
  // sink's designator rather than changing the driver's signal identity.
  auto *data=f.signal;
  auto &d=SourceInputFixture::port(*load,"D");
  d.port_ref->designator=SourceInputFixture::port(*f.source,"Q").port_ref->designator;
  d.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.source,"Q")));
  for(int n=0;n<32;++n) {
    f.edge({7,0},{1,0},141,1000+n,1500+n,true);
    f.terminal({1,0},1500+n,87);
  }
  f.edge({7,0},{5,2},141,2000,2001,true); f.terminal({5,2},2001,87);
  f.edge({3,1},{5,2},713,2100,2101); f.terminal({5,2},2101,63);
  auto *blocker=blockBranch(f,{5,2},2100);
  finalize(f);
  pnr::RouteDesign::RouteTask task{f.source,load,data,"Q","D","destination_blacklist_signal"};
  task.endpoints_prepared=true;
  std::ostringstream closed_trace; std::string closed_reason;
  compareInputWalks(f,*load,{1,0});
  {
    pnr::RouteCongestionTrace capture(task.net_name,"destination_blacklist_closed",{},&closed_trace);
    require(!router.moveUnfinishedCell(task,nullptr,&task,&closed_reason),
        "destination accepted a locally free CE that has no driver connection");
  }
  require(load->tile.peer==&f.tile({2,0}),"failed destination search changed placement");
  require(count(closed_trace.str(),"PLACEMENT_INPUT_PROOF target=(1,0) port=CE")==1,
      "closed destination search repeated the physical CE proof");
  auditPreserved(f);
  // A new routing view must reconsider the previously rejected good Tile.
  fpga::unrouteNet(*blocker);
  if(shared_ce) {
    // A correctly driven CE shared with a fixed register is usable, even
    // though its physical pin is leased and no fresh ingress can be leased.
    secondRegister(f,{5,2});
    auto *neighbor=f.cell("shared_control_receiver_95","FDRE",{{"CE",false}},{5,2},4);
    neighbor->outline.fixed=neighbor->pnr_completed=true;
    auto &input=SourceInputFixture::port(*neighbor,"CE");
    input.port_ref->designator=ce.port_ref->designator;
    input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*control,"O")));
    std::vector<fpga::Wire> route(sibling->wires.front().begin(),sibling->wires.front().begin()+5);
    for(auto &wire:route) wire.shared=true;
    auto fork=f.edge({3,1},{5,2},713,2100,2101); fork.pos=2; fork.owns_dst=false;
    std::vector<fpga::Wire> suffix{fork,f.terminal({5,2},2101,63),
        SourceInputFixture::pin({5,2},63,false)};
    require(pnr::commitPreparedRoute(suffix),"cannot install shared CE control");
    route.insert(route.end(),suffix.begin(),suffix.end());
    neighbor->wires.push_back(std::move(route));
    const auto id=fpga::attachNetRoute(*f.enable,*neighbor,0,control,neighbor,"O","CE",input.makeNetName());
    fpga::registerNetRouteTiles(*f.enable,neighbor->wires.front(),id);
    pnr::MovingPlacementBlacklist packing({{load}});
    require(!packing.rejects(f.tile({5,2})) && f.tile({5,2}).isPinNodeLeased(63),
        "shared CE control is not physically packable or routed");
    auditPreserved(f);
  }
  std::ostringstream trace; std::string reason; std::vector<pnr::RouteDesign::RouteTask> todo;
  {
    std::vector<rtl::Net*> nets;
    for(auto &net:f.parent->nets) nets.push_back(&net);
    pnr::RouteCongestionTrace capture(task.net_name,"destination_blacklist",nets,&trace);
    require(router.moveUnfinishedCell(task,&todo,&task,&reason),reason);
  }
  const auto repeated=count(trace.str(),"reason=\"no distinct free terminal path");
  const auto skips=count(trace.str(),"PLACEMENT_INPUT_BLACKLIST");
  std::cout << "destination blacklist: arrivals=32 slots=2 repeated_support_rejections="
            << repeated << " input_proof_skips=" << skips << " shared_ce=" << shared_ce << '\n';
  require(load->tile.peer==&f.tile({5,2}),"destination accepted a disconnected CE or hid the feasible Tile");
  require(repeated<=1,"destination repeated impossible CE support per arrival: "+std::to_string(repeated));
  require(skips>0,"destination did not use its input blacklist");
  require(count(trace.str(),"PLACEMENT_INPUT_PROOF target=(1,0) port=CE")==1,
      "destination repeated the disconnected CE proof");
  if(shared_ce) require(count(trace.str(),"PLACEMENT_INPUT_SHARED target=(5,2) port=CE")==1,
      "destination did not recognize its correctly driven shared CE pin");
  require(!todo.empty(),"destination silently dropped its CE repair task");
  for(auto &repair:todo)
    require(router.routeNetTask(repair),"selected destination cannot reconnect an incident input");
  auditPreserved(f);
  }
}
