#include "SourceInputFixture.h"

namespace {
using namespace routing_tests;
struct Node { int x,y,kind,id; };

// Chunk 5 snapshot: a live-compatible register slot has disjoint Q/CE/D
// paths (8/7/9 jumps). Q traverses DST->LOCAL->SRC twice. The old reverse
// walker only considered DST->[JOINTs]->SRC, so it never reached this slot.
// One real transit owner occupies both its Takeoff SRC and landing DST.
// Import that owner's complete path too; never manufacture busy bits.
// Resource coordinates are mapped to their CB coordinates; names are neutral.
struct LocalOutputFixture : SourceInputFixture {
  explicit LocalOutputFixture(bool occupied=true,bool connected=true,int protected_count=0)
      : SourceInputFixture(false,0,16,100,40) {
    auto element=[&](fpga::Coord co,fpga::ElementType type) {
      fpga::Element e;e.name="slot_"+std::to_string(int(type));e.type=type;
      tile(co).tile_type->elements.push_back(e);
    };
    element({70,6},fpga::ELEMENT_FD);element({58,4},fpga::ELEMENT_FD);
    for(fpga::Coord co:std::vector<fpga::Coord>{{75,12},{41,3},{22,7},{53,7},{53,6}}) {
      element(co,fpga::ELEMENT_LUT5);element(co,fpga::ELEMENT_LUT1);
    }
    auto pinmap=[&](fpga::Coord co,const char* type,const char* port,int pos,int pin,bool output=false) {
      auto& t=tile(co);
      if(output) {t.tile_type->pin_map.output_nodes[t.getNodeNum(type,port,pos)]={};
        t.tile_type->pin_map.output_nodes[t.getNodeNum(type,port,pos)].setBit(pin);
        t.cb_type->local_output_nodes.setBit(pin);
      }else {t.tile_type->pin_map.nodes[{type,port,pos}]={};
        t.tile_type->pin_map.nodes[{type,port,pos}].setBit(pin);t.cb_type->local_input_nodes.setBit(pin);}
    };
    pinmap({70,6},"FDRE","Q",0,141,true);pinmap({70,6},"FDRE","CE",0,63);
    pinmap({70,6},"FDRE","D",0,22);
    pinmap({58,4},"FDRE","Q",0,145,true);pinmap({58,4},"FDRE","CE",0,41);
    pinmap({58,4},"FDRE","D",0,1);
    pinmap({75,12},"LUT6","O",3,161,true);pinmap({41,3},"LUT6","O",3,150,true);
    pinmap({22,7},"LUT6","I0",3,80);
    pinmap({53,7},"LUT6","O",3,152,true);pinmap({53,6},"LUT6","I0",3,86);
    source=cell("register_5901","FDRE",{{"CE",false},{"D",false},{"Q",true}},{70,6});
    sink=cell("receiver_5903","LUT6",{{"I0",false}},{22,7});
    control=cell("control_5907","LUT6",{{"O",true}},{75,12});
    data_driver=cell("data_5911","LUT6",{{"O",true}},{41,3});
    signal=connect(*source,"Q",*sink,"I0");enable=connect(*control,"O",*source,"CE");
    data_net=connect(*data_driver,"O",*source,"D");
    victim_source=cell("transit_driver_5917","LUT6",{{"O",true}},{53,7});
    victim_sink=cell("transit_sink_5919","LUT6",{{"I0",false}},{53,6});
    victim=connect(*victim_source,"O",*victim_sink,"I0");
    for(auto* fixed:{sink,control,data_driver,victim_source,victim_sink})
      fixed->outline.fixed=fixed->pnr_completed=true;
    q_path={
      {58,4,0,145},
      {58,4,3,3072},
      {50,4,4,3077},
      {50,4,3,3072},
      {41,4,4,3077},
      {41,4,3,3536},
      {37,1,4,3379},
      {37,1,3,3079},
      {32,1,4,3075},
      {32,1,3,3379},
      {26,5,4,3379},
      {26,5,0,140},
      {26,5,3,3536},
      {22,1,4,3536},
      {22,1,0,183},
      {22,1,3,163},
      {22,6,4,96},
      {22,6,3,16},
      {22,7,4,19},
      {22,7,0,80},
    };
    ce_path={
      {75,12,0,161},
      {75,12,3,259},
      {76,12,4,257},
      {76,12,3,737},
      {81,11,4,737},
      {81,11,3,3537},
      {75,7,4,3537},
      {75,7,3,3537},
      {70,3,4,3537},
      {70,3,3,3074},
      {66,3,4,3072},
      {66,3,3,3074},
      {62,3,4,3072},
      {62,3,3,246},
      {58,4,4,3843},
      {58,4,1,60},
      {58,4,1,45},
      {58,4,1,64},
      {58,4,0,41},
    };
    data_path={
      {41,3,0,150},
      {41,3,3,737},
      {44,2,4,737},
      {44,2,3,737},
      {45,1,4,737},
      {45,1,3,977},
      {50,4,4,818},
      {50,4,3,1029},
      {58,4,4,1030},
      {58,4,3,1029},
      {67,4,4,1030},
      {67,4,3,162},
      {67,3,4,97},
      {67,3,3,3076},
      {63,3,4,3073},
      {63,3,3,3844},
      {62,3,4,3840},
      {62,3,3,3616},
      {58,4,4,3616},
      {58,4,1,23},
      {58,4,0,1},
    };
    victim_path={
      {53,7,0,152},
      {53,7,3,195},
      {53,5,4,195},
      {53,5,3,1030},
      {57,5,4,1027},
      {57,5,3,262},
      {58,4,4,260},
      {58,4,0,165},
      {58,4,3,3072},
      {50,4,4,3077},
      {50,4,0,183},
      {50,4,3,819},
      {54,8,4,819},
      {54,8,3,3846},
      {53,8,4,3842},
      {53,8,3,195},
      {53,6,4,195},
      {53,6,0,86},
    };
    for(const auto* path:{&q_path,&ce_path,&data_path,&victim_path})
      for(size_t i=1;i<path->size();++i) {
        auto a=(*path)[i-1],b=(*path)[i];
        if(!connected && path==&q_path && a.x==26 && a.y==5 && a.kind==4 && b.kind==0)continue;
        arc(a,b);
      }
    // The snapshot also has a free short CE entry through DST 738. It is
    // disconnected from this CE driver; only the longer three-JOINT entry
    // above completes CE. Keep it so ordinary endpoint checks see precisely
    // the same distinction between a free terminal and a reachable terminal.
    arc({58,3,3,738},{58,4,4,738});
    arc({58,4,4,738},{58,4,1,64});
    std::vector<std::pair<rtl::Inst*,std::vector<Node>>> protected_paths;
    rtl::Inst* blocker_driver=nullptr;
    if(protected_count) {
      element({5,25},fpga::ELEMENT_LUT5);element({5,25},fpga::ELEMENT_LUT1);
      pinmap({5,25},"LUT6","O",3,169,true);
      blocker_driver=cell("protected_driver_5923","LUT6",{{"O",true}},{5,25});
      blocker_driver->outline.fixed=blocker_driver->pnr_completed=true;
      for(int n=0;n<protected_count;++n) {
        const fpga::Coord co{n%90,30+n/90};
        element(co,fpga::ELEMENT_LUT5);element(co,fpga::ELEMENT_LUT1);
        pinmap(co,"LUT6","I0",3,87);
        auto* receiver=cell("protected_receiver_"+std::to_string(n),"LUT6",{{"I0",false}},co);
        receiver->outline.fixed=receiver->pnr_completed=true;
        if(!protected_net) {
          protected_net=connect(*blocker_driver,"O",*receiver,"I0");
          protected_net->route_protected=true;
        } else {
          auto& in=port(*receiver,"I0");auto& out=port(*blocker_driver,"O");
          in.port_ref->designator=out.port_ref->designator;
          in.set(&rtl::Conn::fromBase(out));
        }
        std::vector<Node> path{{5,25,0,169},{5,25,3,1000+n},
          {22,6,4,2048+n},{22,6,3,3000+n},{co.x,co.y,4,1000+n},{co.x,co.y,0,87}};
        for(size_t i=1;i<path.size();++i)arc(path[i-1],path[i]);
        // Every occupied arrival also offers a blocked approach to Q's
        // penultimate hop. All 257 precede the eligible, deeper transit cut.
        arc({22,6,4,2048+n},{22,6,3,16});
        protected_paths.emplace_back(receiver,std::move(path));
      }
    }
    for(auto& cb:device.cb_types)cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    if(occupied)install(victim_path,*victim,*victim_source,*victim_sink,"I0");
    for(const auto& [receiver,path]:protected_paths)
      install(path,*protected_net,*blocker_driver,*receiver,"I0");
    auditAll();
  }
  rtl::Inst *data_driver=nullptr,*victim_source=nullptr,*victim_sink=nullptr;
  rtl::Net *data_net=nullptr,*victim=nullptr,*protected_net=nullptr;
  std::vector<Node> q_path,ce_path,data_path,victim_path;
  fpga::Tile& tile(fpga::Coord co) {
    auto& t=SourceInputFixture::tile(co);if(t.cb_type)return t;
    const int id=device.cb_types.size();auto& cb=device.cb_types.emplace_back();
    cb.name="matrix_relay_"+std::to_string(id);cb.type_id=cb.base_type_id=id;
    auto& type=device.tile_types.emplace_back();type.name="relay_resource_"+std::to_string(id);
    type.type=fpga::Tile::TILE_LUTS;t.type=fpga::Tile::TILE_LUTS;
    t.full_name="relay_tile_"+std::to_string(co.x)+"_"+std::to_string(co.y);
    t.tile_type=&type;t.cb_type=t.cb.type=&cb;return t;
  }

  void arc(Node a,Node b) {
    auto& t=tile({a.x,a.y});auto& to=tile({b.x,b.y});auto& cb=*t.cb_type;
    name(t,static_cast<fpga::CBNodeNameType>(a.kind),a.id);
    name(to,static_cast<fpga::CBNodeNameType>(b.kind),b.id);
    if(a.kind==fpga::CB_NODE_SRC && b.kind==fpga::CB_NODE_DST) {
      fpga::CBType::ResolvedJump e{};e.delta={b.x-a.x,b.y-a.y};e.target_tile_coord=true;
      e.target_cb_type_id=to.cb_type->type_id;e.dsts.jump.setBit(b.id);cb.dst_by_src[a.id].push_back(e);return;
    }
    if(a.kind==fpga::CB_NODE_LOCAL && (a.x!=b.x || a.y!=b.y)) {
      resolvedLocal({a.x,a.y},a.id,{b.x,b.y},static_cast<fpga::CBNodeNameType>(b.kind),b.id);return;
    }
    require(a.x==b.x && a.y==b.y,"non-LOCAL internal edge crossed a Tile");
    if(a.kind==0 && b.kind==0)cb.local_local[a.id].local.setBit(b.id);
    else if(a.kind==0 && b.kind==1)cb.local_joint[a.id].joint.setBit(b.id);
    else if(a.kind==0 && b.kind==3)cb.local_src[a.id].jump.setBit(b.id);
    else if(a.kind==1 && b.kind==0)cb.joint_local[a.id].local.setBit(b.id);
    else if(a.kind==1 && b.kind==1)cb.joint_joint[a.id].joint.setBit(b.id);
    else if(a.kind==1 && b.kind==3)cb.joint_src[a.id].jump.setBit(b.id);
    else if(a.kind==4 && b.kind==0)cb.dst_local[a.id].local.setBit(b.id);
    else if(a.kind==4 && b.kind==1)cb.dst_joint[a.id].joint.setBit(b.id);
    else if(a.kind==4 && b.kind==3)cb.dst_src[a.id].jump.setBit(b.id);
    else require(false,"unrecognized captured numeric arc");
  }
  void install(const std::vector<Node>& path,rtl::Net& net,rtl::Inst& driver,
               rtl::Inst& sink,const std::string& in) {
    auto start=path.front(),last=path.back();
    std::vector<fpga::Wire> route{pin({start.x,start.y},start.id,true)};
    for(size_t i=1;i<path.size();++i) {
      auto a=path[i-1],b=path[i];fpga::Wire w;w.type=fpga::Wire::WIRE_ROUTE_EDGE;
      w.from={a.x,a.y};w.to={b.x,b.y};w.from_node_type=static_cast<fpga::CBNodeNameType>(a.kind);
      w.from_node=a.id;w.to_node_type=static_cast<fpga::CBNodeNameType>(b.kind);w.to_node=b.id;
      route.push_back(w);
    }
    route.push_back(pin({last.x,last.y},last.id,false));
    if(&net==victim) {
      // Preserve the captured ordinary Takeoff and graph-edge transit island:
      // cut eligibility depends on roles, not merely on the occupied bits.
      auto graph=route;
      route={pin({53,7},152,true),edge({53,7},{53,5},152,195,195,true),
        edge({53,5},{57,5},195,1030,1027),edge({57,5},{58,4},1027,262,260)};
      auto begin=std::find_if(graph.begin(),graph.end(),[](const fpga::Wire& w){
        return w.type==fpga::Wire::WIRE_ROUTE_EDGE && w.from==fpga::Coord{58,4} &&
            w.from_node_type==fpga::CB_NODE_DST && w.from_node==260;
      });
      for(auto it=begin;it!=graph.end();++it) {
        if(it->type==fpga::Wire::WIRE_ROUTE_EDGE && it->from==fpga::Coord{54,8})break;
        route.push_back(*it);
      }
      route.push_back(edge({54,8},{53,8},819,3846,3842));
      route.push_back(edge({53,8},{53,6},3842,195,195));
      route.push_back(terminal({53,6},195,86));route.push_back(pin({53,6},86,false));
    }
    size_t failed=route.size();require(pnr::commitPreparedRoute(route,&failed),
        "captured "+in+" route fails at fragment "+std::to_string(failed));
    sink.wires.push_back(std::move(route));const auto idx=sink.wires.size()-1;
    auto b=fpga::attachNetRoute(net,sink,idx,&driver,&sink,&driver==source?"Q":"O",in,"captured_"+in);
    fpga::registerNetRouteTiles(net,sink.wires[idx],b);
    require(fpga::isRouteComplete(sink.wires[idx]),"captured route is incomplete");
  }
};
}

void moving_source_local_output_regression() {
  // Prove the entire reference placement, including the real transit cut,
  // before asking SourceMoving to discover it on a fresh design.
  {
    LocalOutputFixture f;
    const std::vector<fpga::RouteCutNode> cuts{{{58,4},fpga::CB_NODE_SRC,3072},
                                            {{50,4},fpga::CB_NODE_DST,3077}};
    require(fpga::canPreemptNetRouteFromNodes(*f.victim,0,cuts),"reference transit cut is ineligible");
    require(fpga::unrouteNetRouteFromNodes(*f.victim,0,cuts),"reference transit cut failed");
    require(f.tile({70,6}).unassign(f.source) && f.tile({58,4}).tryAddAt(f.source,0,false)==0,
        "reference register slot cannot pack");
    f.install(f.q_path,*f.signal,*f.source,*f.sink,"I0");
    f.install(f.ce_path,*f.enable,*f.control,*f.source,"CE");
    f.install(f.data_path,*f.data_net,*f.data_driver,*f.source,"D");
    f.auditAll();
  }
  for(bool occupied:{false,true}) {
    LocalOutputFixture f(occupied);auto& router=f.tech.route;
    router.moving_stage=router.moving_sources_stage=true;router.complete_routes_only=true;
    router.route_preemption_enabled=occupied;router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;router.route_recursion_budget=100;
    if (occupied) {
      auto index=router.backwardDockingIndex(f.sink->coord,100);
      NodeMask pin;pin.setBit(80);
      const auto blocked=pnr::traceBackwardToTakeoff(f.tile({22,7}),pin,
          f.source->coord,index,
          [](fpga::Tile&,int,pnr::BackwardTakeoffChoice&) { return false; });
      const auto& cut=blocked.blocked_reverse_cut;
      require(!blocked.success && cut.size()==2 &&
          cut[0].tile==fpga::Coord{58,4} && cut[0].type==fpga::CB_NODE_SRC &&
          cut[0].node==3072 && cut[1].tile==fpga::Coord{50,4} &&
          cut[1].type==fpga::CB_NODE_DST && cut[1].node==3077,
          "reverse trace lost the exact occupied LOCAL-bridge transit edge");
      const auto victims=fpga::findNetRouteCutVictims(cut);
      require(victims.size()==1 && victims.front().net==f.victim &&
          fpga::canPreemptNetRouteFromNodes(*f.victim,0,cut),
          "reverse blocker is not the eligible captured transit owner");
      f.auditAll();
    }
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",SourceInputFixture::port(*f.sink,"I0").makeNetName()};
    task.endpoints_prepared=true;bool moved=false;std::string reason;
    pnr::MovingPlacementBlacklist placements({{f.source}});
    size_t released=0,retries=0;bool retryable=false;
    const bool done=router.moveUnfinishedSource(task, nullptr, &reason, &released, &moved, &retryable, nullptr, false, &placements, &retries);
    std::cout<<"LOCAL_OUTPUT_MOVING occupied="<<occupied<<" complete="<<done<<" moved="<<moved<<" reason="<<reason<<'\n';
    require(done && moved && f.source->tile.peer==&f.tile({58,4}) && f.source->pos==0,
        "reverse routing missed the captured LOCAL-bridge output: "+reason);
    int complete=0;
    for(auto* net:{f.signal,f.enable,f.data_net})for(auto& b:net->routes)
      if(b.owner && fpga::isRouteComplete(b.owner->wires[b.route_index]))++complete;
    require(complete==3,"source move did not finish Q, CE and D");
    require(router.route_stats.preempt_success==(occupied?1:0),"unexpected transit preemption count");
    require(f.sink->tile.peer==&f.tile({22,7}) && f.control->tile.peer==&f.tile({75,12}) &&
        f.data_driver->tile.peer==&f.tile({41,3}),"moving changed a fixed endpoint");
    f.auditAll();
  }
  for(bool broken:{false,true}) {
    LocalOutputFixture f(true,!broken);auto& router=f.tech.route;
    f.victim->route_protected=!broken;
    router.moving_stage=router.moving_sources_stage=true;router.complete_routes_only=true;
    router.route_preemption_enabled=true;router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;router.route_recursion_budget=100;
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
        SourceInputFixture::port(*f.sink,"I0").makeNetName()};
    task.endpoints_prepared=true;bool moved=false,retryable=false;std::string reason;
    pnr::MovingPlacementBlacklist placements({{f.source}});size_t released=0,retries=0;
    require(!router.moveUnfinishedSource(task, nullptr, &reason, &released, &moved, &retryable, nullptr, false, &placements, &retries),
        "protected owner or disconnected LOCAL bridge was bypassed");
    require(!moved && f.source->tile.peer==&f.tile({70,6}) && !retries &&
        router.route_stats.preempt_success==0,"failed output proof changed placement or transit owner");
    require(fpga::isRouteComplete(f.victim_sink->wires.front()),
        "failed output proof damaged the transit route");
    f.auditAll();
  }
}

void moving_source_local_output_frontier_regression() {
  // The real run reached (50,4)/SRC3072 but never tried the valid source slot
  // at (58,4): an eligible transit cut lay outside its 256-blocker sample.
  // Preserve the same Q/CE/D witness and its owner, with 257 real protected
  // approaches encountered before that deeper cut. No synthetic busy bits.
  LocalOutputFixture f(true,true,257);auto& router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;router.complete_routes_only=true;
  router.route_preemption_enabled=true;router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;router.route_recursion_budget=100;
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
      SourceInputFixture::port(*f.sink,"I0").makeNetName()};
  task.endpoints_prepared=true;bool moved=false,retryable=false;std::string reason;
  pnr::MovingPlacementBlacklist placements({{f.source}});size_t released=0,retries=0;
  const bool done=router.moveUnfinishedSource(task, nullptr, &reason, &released, &moved, &retryable, nullptr, false, &placements, &retries);
  std::cout<<"LOCAL_OUTPUT_FRONTIER protected="<<f.protected_net->routes.size()
      <<" complete="<<done<<" moved="<<moved<<" cuts="<<router.route_stats.preempt_success
      <<" reason="<<reason<<'\n';
  require(done && moved && f.source->tile.peer==&f.tile({58,4}),
      "protected frontier sample hid the eligible transit cut: "+reason);
  require(router.route_stats.preempt_success==1,"protected frontier caused extra cuts");
  int completed=0;
  for(auto* net:{f.signal,f.enable,f.data_net})for(auto& b:net->routes)
    completed+=b.owner && fpga::isRouteComplete(b.owner->wires[b.route_index]);
  require(completed==3,"frontier routing did not complete Q, CE and D");
  require(f.protected_net->routes.size()==257,"protected owners were not all installed");
  for(auto& b:f.protected_net->routes)
    require(fpga::isRouteComplete(b.owner->wires[b.route_index]),"protected approach was damaged");
  f.auditAll();
}
