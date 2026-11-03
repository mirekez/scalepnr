#include "SourceInputFixture.h"
#include "route/MovingInputReachability.h"

namespace {
using namespace routing_tests;
struct Node { int x,y,kind,id; };

// Run 5's source at (46,38) could not reach its fixed sink at (23,8).
// Slot 4 at resource (24,5), CB (25,5), passed live packing. Independent
// searches of the saved model found disjoint Q/CE/D paths of 5/86/7 hops.
// CE crosses resolved LOCAL links, including an 84-node LOCAL-heavy tail.
// Import all three numeric paths; names and resource types are synthetic.
// Unrelated congestion is omitted, so the golden solution is independently
// verifiable instead of assuming that moving another owner will succeed.
struct LocalInputFixture : SourceInputFixture {
  explicit LocalInputFixture(bool connected=true) : SourceInputFixture(false,0,16,100,80) {
    auto element=[&](fpga::Coord c,fpga::ElementType type,int bit=0) {
      fpga::Element e; e.name="slot_"+std::to_string(int(type))+"_"+std::to_string(bit);
      e.type=type;e.bitmap_pos=bit;tile(c).tile_type->elements.push_back(e);
    };
    auto logic=[&](fpga::Coord c) {element(c,fpga::ELEMENT_LUT5);element(c,fpga::ELEMENT_LUT1);};
    element({45,38},fpga::ELEMENT_FD);element({25,5},fpga::ELEMENT_FD,1);
    logic({66,70});logic({31,1});logic({22,8});
    auto pinmap=[&](fpga::Coord c,const char* type,const char* port,int pos,int pin,bool output=false) {
      auto& t=tile(c);if(output) {
        t.tile_type->pin_map.output_nodes[t.getNodeNum(type,port,pos)]={};
        t.tile_type->pin_map.output_nodes[t.getNodeNum(type,port,pos)].setBit(pin);
        t.cb_type->local_output_nodes.setBit(pin);
      } else {t.tile_type->pin_map.nodes[{type,port,pos}]={};
        t.tile_type->pin_map.nodes[{type,port,pos}].setBit(pin);t.cb_type->local_input_nodes.setBit(pin);}
    };
    pinmap({66,70},"LUT6","O",3,161,true);pinmap({31,1},"LUT6","O",3,161,true);
    pinmap({25,5},"FDRE","Q",4,146,true);pinmap({25,5},"FDRE","CE",4,64);
    pinmap({25,5},"FDRE","D",4,26);pinmap({22,8},"LUT6","I0",3,112);
    pinmap({45,38},"FDRE","D",0,0);pinmap({45,38},"FDRE","CE",0,40);
    pinmap({45,38},"FDRE","Q",0,141,true);
    source=cell("register_4701","FDRE",{{"CE",false},{"D",false},{"Q",true}},{45,38});
    sink=cell("receiver_4703","LUT6",{{"I0",false}},{22,8});
    control=cell("control_4707","LUT6",{{"O",true}},{66,70});
    data_driver=cell("data_4711","LUT6",{{"O",true}},{31,1});
    signal=connect(*source,"Q",*sink,"I0");enable=connect(*control,"O",*source,"CE");
    data_net=connect(*data_driver,"O",*source,"D");
    control->outline.fixed=control->pnr_completed=true;
    data_driver->outline.fixed=data_driver->pnr_completed=true;sink->outline.fixed=sink->pnr_completed=true;
    q_path={
      {25,5,0,146},
      {25,5,3,737},
      {26,4,4,737},
      {26,4,3,3537},
      {22,1,4,3378},
      {22,1,3,3811},
      {21,1,4,3616},
      {21,1,3,96},
      {21,7,4,96},
      {21,7,3,544},
      {22,8,4,544},
      {22,8,0,112},
    };
    ce_path={
      {66,70,0,161},
      {66,70,3,162},
      {66,64,4,162},
      {66,64,3,162},
      {66,58,4,162},
      {66,58,3,162},
      {66,51,4,162},
      {66,51,3,162},
      {66,45,4,162},
      {66,45,3,1028},
      {70,45,4,1026},
      {70,45,3,978},
      {75,41,4,978},
      {75,41,3,162},
      {75,35,4,162},
      {75,35,3,162},
      {75,29,4,162},
      {75,29,3,738},
      {76,28,4,738},
      {76,28,3,194},
      {76,25,4,194},
      {76,25,1,62},
      {76,25,1,47},
      {76,25,0,30},
      {77,25,0,80},
      {78,26,0,361},
      {78,26,0,298},
      {78,26,0,338},
      {78,26,0,274},
      {78,26,0,114},
      {77,26,0,2},
      {76,26,0,2},
      {75,26,0,38},
      {74,26,0,2},
      {73,26,0,2},
      {72,26,0,2},
      {71,26,0,2},
      {70,26,0,38},
      {69,26,0,2},
      {68,26,0,2},
      {67,26,0,2},
      {66,26,0,38},
      {65,26,0,2},
      {64,26,0,2},
      {63,26,0,2},
      {62,26,0,38},
      {61,26,0,2},
      {60,26,0,2},
      {59,26,0,2},
      {58,26,0,2},
      {57,26,0,38},
      {56,26,0,2},
      {55,26,0,2},
      {54,26,0,2},
      {53,26,0,38},
      {52,26,0,2},
      {51,26,0,2},
      {50,26,0,2},
      {49,26,0,38},
      {48,26,0,2},
      {47,26,0,2},
      {46,26,0,2},
      {45,26,0,2},
      {44,26,0,38},
      {43,26,0,2},
      {42,26,0,2},
      {41,26,0,2},
      {40,26,0,38},
      {39,26,0,2},
      {38,26,0,2},
      {37,26,0,2},
      {36,26,0,38},
      {35,26,0,2},
      {34,26,0,2},
      {33,26,0,2},
      {32,26,0,2},
      {31,26,0,38},
      {30,26,0,2},
      {29,26,0,2},
      {28,26,0,2},
      {27,26,0,2},
      {26,26,0,2},
      {26,26,0,10},
      {25,26,0,18},
      {25,26,0,31},
      {25,25,0,72},
      {25,24,0,72},
      {25,23,0,72},
      {25,22,0,72},
      {25,21,0,72},
      {25,20,0,72},
      {25,19,0,72},
      {25,18,0,72},
      {25,17,0,72},
      {25,16,0,72},
      {25,15,0,72},
      {25,14,0,72},
      {25,13,0,72},
      {25,12,0,72},
      {25,11,0,72},
      {25,10,0,72},
      {25,9,0,72},
      {25,8,0,72},
      {25,7,0,72},
      {25,6,0,72},
      {25,5,0,72},
      {25,5,0,78},
      {25,5,1,56},
      {25,5,0,64},
    };
    data_path={
      {31,1,0,161},
      {31,1,3,3845},
      {26,1,4,3841},
      {26,1,3,3076},
      {22,1,4,3073},
      {22,1,3,3076},
      {16,1,4,3073},
      {16,1,3,97},
      {16,7,4,97},
      {16,7,3,1026},
      {22,7,4,1025},
      {22,7,3,737},
      {25,6,4,737},
      {25,6,3,242},
      {25,5,4,246},
      {25,5,1,18},
      {25,5,0,26},
    };
    for(const auto* path:{&q_path,&ce_path,&data_path})
      for(size_t i=1;i<path->size();++i) {
        const auto &a=(*path)[i-1], &b=(*path)[i];
        // A disconnected control provides a negative proof, not a timeout.
        if (!connected && path==&ce_path && a.x==25 && a.y==15 && b.y==14) continue;
        arc(a,b);
      }
    for(auto& cb:device.cb_types)cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  rtl::Inst* data_driver=nullptr;rtl::Net* data_net=nullptr;
  std::vector<Node> q_path,ce_path,data_path;
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
};
}

void moving_source_local_input_regression() {
  // Construct and commit the golden solution separately. A bare free suffix
  // is insufficient evidence; all three paths must coexist in a legal slot.
  {
    LocalInputFixture f;
    require(f.tile({45,38}).unassign(f.source) && f.tile({25,5}).tryAddAt(f.source,4,false)==4,
        "captured reference register slot does not pack");
    auto install=[&](const std::vector<Node>& path,rtl::Net& net,rtl::Inst& driver,
                     rtl::Inst& sink,const std::string& in) {
      auto start=path.front(),last=path.back();
      std::vector<fpga::Wire> route{SourceInputFixture::pin({start.x,start.y},start.id,true)};
      for(size_t i=1;i<path.size();++i) {
        auto a=path[i-1],b=path[i];fpga::Wire w;w.type=fpga::Wire::WIRE_ROUTE_EDGE;
        w.from={a.x,a.y};w.to={b.x,b.y};
        w.from_node_type=static_cast<fpga::CBNodeNameType>(a.kind);w.from_node=a.id;
        w.to_node_type=static_cast<fpga::CBNodeNameType>(b.kind);w.to_node=b.id;route.push_back(w);
      }
      route.push_back(SourceInputFixture::pin({last.x,last.y},last.id,false));
      std::vector<fpga::Wire> undeclared(route.begin()+1,route.end());
      require(!pnr::commitPreparedRoute(undeclared),"unleased graph root needs its explicit source pin");
      size_t failed=route.size();
      require(pnr::commitPreparedRoute(route,&failed),"golden "+in+" fails at fragment "+std::to_string(failed));
      sink.wires.push_back(std::move(route));const auto index=sink.wires.size()-1;
      auto binding=fpga::attachNetRoute(net,sink,index,&driver,&sink,
          &driver==f.source?"Q":"O",in,"golden_"+in);
      fpga::registerNetRouteTiles(net,sink.wires[index],binding);
      require(fpga::isRouteComplete(sink.wires[index]),"golden route is incomplete");
    };
    install(f.q_path,*f.signal,*f.source,*f.sink,"I0");
    install(f.ce_path,*f.enable,*f.control,*f.source,"CE");
    install(f.data_path,*f.data_net,*f.data_driver,*f.source,"D");
    f.auditAll();
  }
  LocalInputFixture f;auto& router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;router.route_preemption_enabled=false;
  router.route_deadends_enabled=false;router.iteration_limit=router.route_suffix_depth_limit=200;
  auto index=router.backwardDockingIndex({25,5},100);NodeMask pins;pins.setBit(64);
  pnr::MovingInputReachability region(f.tile({25,5}));
  const bool possible=region.mayReach(pins,{&f.tile({66,70})},index);
  std::cout << "LOCAL_INPUT_REGION possible=" << possible << " expanded=" << region.expanded() << '\n';
  require(region.expanded()>=80 && region.expanded()<=160,
      "necessary input proof omitted the long relay or expanded unrelated states");
  require(possible,"fast input region falsely rejects the captured LOCAL-link CE path");
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
      SourceInputFixture::port(*f.sink,"I0").makeNetName()};
  task.endpoints_prepared=true;bool moved=false;std::string reason;
  pnr::BackwardTakeoffChoice choice;std::string tail_reason;
  const bool tail=router.routeOutTry(*f.source,f.tile({25,5}),4,"Q",f.tile({25,5}),737,choice,&tail_reason);
  require(tail,"local relay terminal rejected before its complete proof: "+tail_reason);
  require(f.source->tile.peer==&f.tile({45,38}) && f.source->pos==0,
      "takeoff preview changed the register placement");
  fpga::packedInputJointReservations(f.tile({25,5}));
  fpga::packedInputJointReservations(f.tile({45,38}));
  require(!f.tile({25,5}).input_local_reservations.contains(64) &&
          !f.tile({25,5}).input_local_reservations.contains(26) &&
          f.tile({45,38}).input_local_reservations.contains(40),
      "hypothetical placement left stale input reservations");
  const bool done=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved);
  std::cout << "LOCAL_INPUT_MOVING complete=" << done << " moved=" << moved << " reason=" << reason << '\n';
  require(done && moved && f.source->tile.peer==&f.tile({25,5}) && f.source->pos==4,
      "source recovery discarded a feasible disjoint Q/CE/D solution: "+reason);
  int complete=0;
  for(auto* net:{f.signal,f.enable,f.data_net})for(auto& b:net->routes)
    if(b.owner && fpga::isRouteComplete(b.owner->wires[b.route_index]))++complete;
  require(complete==3,"source recovery did not commit Q, CE and D");
  require(f.control->tile.peer==&f.tile({66,70}) && f.data_driver->tile.peer==&f.tile({31,1}) &&
          f.sink->tile.peer==&f.tile({22,8}),"source recovery moved a fixed endpoint");
  require(router.route_stats.preempt_success==0,"free captured input solution preempted an unrelated route");
  f.auditAll();

  LocalInputFixture disconnected(false);
  auto& rejected=disconnected.tech.route;
  rejected.moving_stage=rejected.moving_sources_stage=true;
  rejected.route_preemption_enabled=rejected.route_deadends_enabled=false;
  rejected.iteration_limit=rejected.route_suffix_depth_limit=200;
  auto blocked_index=rejected.backwardDockingIndex({25,5},100);
  pnr::MovingInputReachability blocked(disconnected.tile({25,5}));
  require(!blocked.mayReach(pins,{&disconnected.tile({66,70})},blocked_index),
      "broken control relay was admitted by the necessary input proof");
  require(blocked.expanded()>5 && blocked.expanded()<40,
      "broken control proof did not exhaust at its actual disconnected segment");
  pnr::RouteDesign::RouteTask failed{disconnected.source,disconnected.sink,disconnected.signal,
      "Q","I0",SourceInputFixture::port(*disconnected.sink,"I0").makeNetName()};
  failed.endpoints_prepared=true;bool wrongly_moved=false;std::string failure;
  require(!rejected.moveUnfinishedSource(failed,nullptr,&failure,nullptr,&wrongly_moved),
      "source was moved without its control input");
  require(!wrongly_moved && disconnected.source->tile.peer==&disconnected.tile({45,38}) &&
      disconnected.source->pos==0,"failed proof changed the source placement");
  require(rejected.route_stats.preempt_success==0,"disconnected input proof cut an unrelated route");
  disconnected.auditAll();
}
