#include "SourceInputFixture.h"

#include <cstdio>
#include <sstream>
#include <tuple>

using namespace routing_tests;

// The saved source-relocation failure had a three-hop CE suffix from a
// retained tree landing. Only the first jump was foreign transit; the final
// DST -> JOINT -> input was free. A terminal-only preview cannot discover it.
// Preserve that nine-node structure with synthetic coordinates and names.
void source_input_branch_preemption_regression(int mode) {
  const bool control_only=mode==1, far=mode==2, blocked=mode==3;
  SourceInputFixture f(false,0,12,mode==7?40:far?16:8);
  const fpga::Coord target_coord{far?14:1,0};
  auto& target=f.tile(target_coord);
  // The real failed register had no CE binding of its own: its available
  // branches belonged to completed, independently placed sibling registers.
  // Replace the fixture's initial own-input route with such a sibling route.
  fpga::Element sibling_slot;sibling_slot.name="retained_control_slot";
  sibling_slot.type=fpga::ELEMENT_FD;
  f.tile({3,2}).tile_type->elements.push_back(sibling_slot);
  auto* sibling=f.cell("retained_control_receiver_89","FDRE",{{"CE",false}},{3,2});
  sibling->outline.fixed=sibling->pnr_completed=true;
  auto& sibling_input=SourceInputFixture::port(*sibling,"CE");
  sibling_input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
  sibling_input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
  auto retained=f.source->wires[0];
  fpga::unrouteNetRoute(*f.enable,0);f.enable->eraseRouteBinding(0);f.source->wires.clear();
  retained[retained.size()-3]=f.edge({0,1},{3,2},716,1510,718);
  retained[retained.size()-2]=f.terminal({3,2},718,63);
  retained.back()=SourceInputFixture::pin({3,2},63,false);
  if(mode==7) {
    // A retained branch can wander outside the bounded input search. Its
    // released downstream resources are inside that window, so the cut
    // touches the negative boundary but still cannot reach this anchor.
    retained[2]=f.edge({6,1},{30,1},710,1910,1880);
    retained.insert(retained.begin()+3,f.edge({30,1},{5,1},1880,1911,711));
  }
  require(pnr::commitPreparedRoute(retained),"cannot install the retained sibling CE route");
  sibling->wires.push_back(std::move(retained));
  auto sibling_binding=fpga::attachNetRoute(*f.enable,*sibling,0,f.control,sibling,
      "O","CE",sibling_input.makeNetName());
  fpga::registerNetRouteTiles(*f.enable,sibling->wires[0],sibling_binding);
  if(far) {
    fpga::Element e;e.name="distant_register_slot";e.type=fpga::ELEMENT_FD;
    target.tile_type->elements.push_back(e);
    f.tile({1,0}).tile_type->elements.clear();
    f.tile({1,0}).cb_type->local_src[141].jump.clearBit(300);
    f.edge(target_coord,{0,0},141,300,800,true);
  }
  target.tile_type->pin_map.input_nodes[target.getNodeNum("FDRE","CE",0)]={};
  target.tile_type->pin_map.input_nodes[target.getNodeNum("FDRE","CE",0)].setBit(40);
  target.cb_type->local_input_nodes.setBit(40);
  f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);
  f.edge({5,1},{3,0},711,3537,977);
  f.edge({3,0},{2,0},977,545,545);
  f.edge({2,0},target_coord,545,1026,1025);
  if(blocked)f.tile({2,0}).cb_type->dst_src[545].jump.clearBit(1026);
  SourceInputFixture::name(target,fpga::CB_NODE_JOINT,63);
  SourceInputFixture::name(target,fpga::CB_NODE_LOCAL,40);
  target.cb_type->dst_joint[1025].joint.setBit(63);
  target.cb_type->joint_local[63].local.setBit(40);
  if(mode==5) {
    // The next device failure admitted CE in the independent view, but its
    // candidate Q reservation isolated CE. Thousands of previews released
    // unrelated foreign transit and repeated the same exhausted input walk.
    // All 32 Q alternatives here reserve the same destination JOINT. CE has
    // a real free route through that JOINT before Q is reserved; releasing
    // the branch victim below cannot free a privately reserved Q resource.
    auto& cb=*f.tile({0,0}).cb_type;
    SourceInputFixture::name(f.tile({0,0}),fpga::CB_NODE_JOINT,65);
    for(int n=0;n<32;++n) {
      cb.dst_local[500+n].local.clearBit(87);
      cb.dst_joint[500+n].joint.setBit(65);
    }
    cb.joint_local[65].local.setBit(87);
    // Preserve a distinct potential grounding, so this JOINT is not a
    // mandatory packing reservation for the already placed data sink.
    f.terminal({0,0},1999,87);
    f.edge({5,1},{0,0},711,1980,1981);
    cb.dst_joint[1981].joint.setBit(65);
    cb.joint_local[65].local.setBit(186);
    f.resolvedLocal({0,0},186,{2,0},fpga::CB_NODE_LOCAL,63);
  }
  if(mode==6 || mode==7) {
    // The native repeater has a NEGATIVE exact free-input preflight, unlike
    // mode 5's positive preflight followed by a private output conflict.
    // This free corridor reaches a retained-tree TILE but not its actual
    // landing node 711. The coarse region must remain uncertain, while the
    // exact input proof must fail. Releasing the legal transit cut elsewhere
    // cannot connect this corridor to that retained tree.
    f.edge({5,1},{4,0},1990,1991,1992);
    f.tile({4,0}).cb_type->dst_src[1992].jump.setBit(1000);
    // Enlarge the disconnected reverse region with neutral-named joints.
    // Each real preview traverses these nodes; there is no test-side loop
    // calling the proof repeatedly to manufacture the observed work.
    auto& cb=*f.tile({4,0}).cb_type;
    for(int n=0;n<128;++n) {
      SourceInputFixture::name(f.tile({4,0}),fpga::CB_NODE_JOINT,2000+n);
      cb.joint_src[2000+n].jump.setBit(1000+n%24);
      if(n) cb.joint_joint[2000+n-1].joint.setBit(2000+n);
    }
  }
  for(auto [at,type]:std::vector<std::pair<fpga::Coord,fpga::ElementType>>{
      {{4,2},fpga::ELEMENT_FD},{{5,2},fpga::ELEMENT_LUT5},{{5,2},fpga::ELEMENT_LUT1}}) {
    fpga::Element e;e.name="branch_owner_slot_"+std::to_string(type);e.type=type;
    f.tile(at).tile_type->elements.push_back(e);
  }
  f.tile({4,2}).tile_type->pin_map.output_nodes[f.tile({4,2}).getNodeNum("FDRE","Q",0)].setBit(170);
  f.tile({4,2}).cb_type->local_output_nodes.setBit(170);
  auto* driver=f.cell("branch_transit_driver_71","FDRE",{{"Q",true}},{4,2});
  auto* receiver=f.cell("branch_transit_receiver_73","LUT6",{{"I0",false}},{5,2});
  auto* victim=f.connect(*driver,"Q",*receiver,"I0");
  std::vector<fpga::Wire> route{SourceInputFixture::pin({4,2},170,true),
      f.edge({4,2},{5,1},170,1500,1503,true),
      f.edge({5,1},{3,0},1503,3537,977),
      f.edge({3,0},{5,2},977,1501,1502),f.terminal({5,2},1502,87),
      SourceInputFixture::pin({5,2},87,false)};
  f.edge({5,1},{5,2},1503,1504,1505);f.terminal({5,2},1505,87);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(route),"cannot install the CE branch transit owner");
  receiver->wires.push_back(std::move(route));
  auto id=fpga::attachNetRoute(*victim,*receiver,0,driver,receiver,"Q","I0",victim->name);
  fpga::registerNetRouteTiles(*victim,receiver->wires[0],id);
  rtl::Inst* remote_receiver=nullptr;
  rtl::Net* remote_victim=nullptr;
  if(mode==7) {
    for(auto [at,type]:std::vector<std::pair<fpga::Coord,fpga::ElementType>>{
        {{7,2},fpga::ELEMENT_FD},{{6,2},fpga::ELEMENT_LUT5},{{6,2},fpga::ELEMENT_LUT1}}) {
      fpga::Element e;e.name="remote_owner_slot_"+std::to_string(type);e.type=type;
      f.tile(at).tile_type->elements.push_back(e);
    }
    auto* remote_driver=f.cell("remote_transit_driver","FDRE",{{"Q",true}},{7,2});
    remote_receiver=f.cell("remote_transit_receiver","LUT6",{{"I0",false}},{6,2});
    remote_victim=f.connect(*remote_driver,"Q",*remote_receiver,"I0");
    std::vector<fpga::Wire> remote{SourceInputFixture::pin({7,2},141,true),
        f.edge({7,2},{30,1},141,1800,1800,true),
        f.edge({30,1},{4,0},1800,1801,1801),
        f.edge({4,0},{6,2},1801,1802,1802),f.terminal({6,2},1802,87),
        SourceInputFixture::pin({6,2},87,false)};
    f.tile({30,1}).cb_type->dst_src[1880].jump.setBit(1801);
    // This occupied landing is on the bad input's exact boundary. Its owner
    // is real transit, but releasing it does not bring the remote anchor
    // inside the configured radius. Every Q exit otherwise repeats the walk.
    f.tile({4,0}).cb_type->dst_src[1801].jump.setBit(1000);
    require(pnr::commitPreparedRoute(remote),"cannot install remote transit owner");
    remote_receiver->wires.push_back(std::move(remote));
    auto binding=fpga::attachNetRoute(*remote_victim,*remote_receiver,0,remote_driver,
        remote_receiver,"Q","I0",remote_victim->name);
    fpga::registerNetRouteTiles(*remote_victim,remote_receiver->wires[0],binding);
  }
  if(mode==8) {
    // Input and output can coexist at the early bank, except when Q takes
    // source 100. A negative PRIVATE proof for that Q must not poison the
    // independent cut proof used by the next output choice.
    f.tile({2,0}).cb_type->dst_src[545].jump.setBit(100);
    f.edge({0,0},{2,0},500,1950,1960);
    f.terminal({2,0},1960,63);
  }
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  f.auditAll();
  const auto retained_before=sibling->wires[0];
  auto& router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=!control_only;
  router.complete_routes_only=router.preserve_routes=true;
  router.route_deadends_enabled=false;router.route_preemption_enabled=true;
  router.iteration_limit=router.route_suffix_depth_limit=router.route_recursion_budget=200;
  if(control_only) {
    auto* probe=f.cell("branch_control_probe_79","FDRE",{{"CE",false}},{1,0});
    auto& input=SourceInputFixture::port(*probe,"CE");
    input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
    input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
    pnr::RouteDesign::RouteTask task{f.control,probe,f.enable,"O","CE","branch_control_reference"};
    task.fanout=true;
    require(router.routeTaskOrFail(task,"CE branch reference",200),
        "ordinary CE branching cannot use the transit-preemptible reference");
  } else {
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","branch_register_output_83"};
    pnr::MovingPlacementBlacklist placements({{f.source}});
    std::string reason;size_t released=0,retries=0;bool moved=false,retryable=false;
    const bool done=router.moveUnfinishedSource(task, nullptr, &reason, &released, &moved, &retryable, nullptr, false, &placements, &retries);
    if(blocked) {
      require(!done,"disconnected CE branch unexpectedly completed relocation");
      // The parent was committed before its recursive input failed. Both the
      // input and any preempted transit must remain accounted scheduler work.
      const auto queued=[&](rtl::Net* net,rtl::Inst* target) {
        return std::any_of(router.pending_route_todo.begin(),router.pending_route_todo.end(),
            [&](const auto& next){return next.net==net && next.to==target;});
      };
      require(!reason.empty() && queued(f.enable,f.source),"failed recursive input was not queued");
      require(fpga::isRouteComplete(receiver->wires[0]) || queued(victim,receiver),
          "failed branch attempt lost its transit victim");
      require(fpga::isRouteComplete(sibling->wires[0]),"failed branch preview damaged its retained sibling");
      require(placements.input_branch_previews==0,
          "branch release outside the negative input region repeated an exact proof");
      f.auditAll();return;
    }
    require(done,
        "Source Moving rejected CE with a free terminal and a preemptible branch Takeoff: "+reason);
    // The angle-led search may choose either legal nearby bank after its cut.
    require(f.source->tile.peer && (mode==7
        ? (f.source->tile.peer==&target || f.source->tile.peer==&f.tile({2,0}))
        : f.source->tile.peer==&target),
        "Source Moving missed the legal nearby banks");
    auto complete=[&](rtl::Net& net,rtl::Inst* from,rtl::Inst* to,const char* input) {
      return std::any_of(net.routes.begin(),net.routes.end(),[&](const auto& b) {
        return b.from==from && b.to==to && b.to_port==input && b.owner &&
            b.route_index<b.owner->wires.size() && fpga::isRouteComplete(b.owner->wires[b.route_index]);
      });
    };
    require(complete(*f.signal,f.source,f.sink,"I0"),"branch relocation left Q unfinished");
    require(complete(*f.enable,f.control,f.source,"CE"),"branch relocation left CE unfinished");
    // Preserve the difficult physical graphs, not the removed pre-placement
    // proof/cache machinery. Completion above must come from actual routing.
    require(placements.input_branch_previews==0 && placements.input_branch_expanded==0 &&
        placements.input_cut_proofs==0,"recursive Moving reintroduced speculative input proofs");
    if(mode==7) {
      auto live=[&](rtl::Net& net,rtl::Inst* from,rtl::Inst* to,const char* port)
          -> const std::vector<fpga::Wire>* {
        for(const auto& binding:net.routes)
          if(binding.from==from && binding.to==to && binding.to_port==port &&
              binding.owner && binding.route_index<binding.owner->wires.size())
            return &binding.owner->wires[binding.route_index];
        return nullptr;
      };
      const auto* q=live(*f.signal,f.source,f.sink,"I0");
      const auto* ce=live(*f.enable,f.control,f.source,"CE");
      require(q && ce && fpga::isRouteComplete(*q) && fpga::isRouteComplete(*ce) &&
          q->front().from==f.source->tile->cb_coord &&
          q->back().to==f.sink->tile->cb_coord &&
          ce->back().to==f.source->tile->cb_coord,
          "relocated Q or CE binding does not reach its live endpoint");
      const auto signature=[](const fpga::Wire& w) {
        return std::tuple{w.type,w.from.x,w.from.y,w.to.x,w.to.y,w.local,w.pos,
          w.jump,w.dst,w.joint,w.joint2,w.from_node_type,w.from_node,
          w.to_node_type,w.to_node,w.pin_dir,w.shared,w.owns_dst,w.owns_landing};
      };
      require(sibling->tile.peer==&f.tile({3,2}) &&
          sibling->wires[0].size()==retained_before.size() &&
          std::equal(sibling->wires[0].begin(),sibling->wires[0].end(),retained_before.begin(),
            [&](const auto& a,const auto& b){return signature(a)==signature(b);}) &&
          fpga::isRouteComplete(sibling->wires[0]),
          "relocation changed the retained control sibling");
      require(remote_victim && remote_victim->routes.size()==1 &&
          remote_victim->routes[0].owner==remote_receiver &&
          remote_victim->routes[0].route_index<remote_receiver->wires.size(),
          "remote transit owner lost its live binding");
      const auto& remote=remote_receiver->wires[remote_victim->routes[0].route_index];
      const auto queued=std::count_if(router.pending_route_todo.begin(),router.pending_route_todo.end(),
          [&](const auto& next){return next.net==remote_victim && next.to==remote_receiver &&
              next.net_name==remote_victim->routes[0].route_name;});
      require(fpga::isRouteComplete(remote) || queued==1,
          "recursive input routing lost a remote transit owner");
      f.auditAll();
    }
  }
  require(router.route_stats.preempt_success>0,"reference never preempted the branch transit");
  if(mode==0) {
    std::ostringstream history;
    router.dumpFailureDecisions(history);
    const std::string victim_id="victim=\""+victim->name+"\" route_id="+
        std::to_string(victim->routeId(id));
    std::istringstream events(history.str());
    std::string event;
    bool recorded_cut=false;
    while(std::getline(events,event)) {
      if(event.find("CUT kind=")==std::string::npos || event.find(victim_id)==std::string::npos)
        continue;
      const auto fragments=event.find("fragments=");
      size_t before=0,after=0;
      if(fragments!=std::string::npos &&
          std::sscanf(event.c_str()+fragments,"fragments=%zu->%zu",&before,&after)==2 &&
          before>after) recorded_cut=true;
    }
    require(recorded_cut,"actual Moving cut omitted victim binding ID or fragment change from failure history");
  }
  pnr::RouteDesign::RouteTask repair{driver,receiver,victim,"Q","I0",victim->name};
  require(router.routeTaskOrFail(repair,"repair CE branch transit",200),"branch victim cannot use its free bypass");
  require(fpga::isRouteComplete(receiver->wires[victim->routes[0].route_index]),"branch transit victim stayed incomplete");
  require(sibling->tile.peer==&f.tile({3,2}) && fpga::isRouteComplete(sibling->wires[0]),
      "branch routing damaged its retained control sibling");
  f.auditAll();
}
