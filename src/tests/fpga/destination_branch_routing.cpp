#include "SourceInputFixture.h"
#include <tuple>
using namespace routing_tests;
namespace {
using C=fpga::Coord;
using W=fpga::Wire;
using Route=std::vector<W>;
using Node=std::tuple<int,int,int,int>;
void init(SourceInputFixture& f,C c) {
  auto& t=f.tile(c);if(t.cb_type)return;
  auto& cb=f.device.cb_types.emplace_back();cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
  cb.name="output_matrix_"+std::to_string(cb.type_id);
  auto& type=f.device.tile_types.emplace_back();type.name="output_resource_"+std::to_string(cb.type_id);
  type.type=t.type=fpga::Tile::TILE_LUTS;t.tile_type=&type;t.cb_type=t.cb.type=&cb;
}
void slots(SourceInputFixture& f,C c) {
  init(f,c);for(int bit=0;bit<8;++bit)for(auto type:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}) {
    fpga::Element e;e.type=type;e.bitmap_pos=bit;e.name="slot_"+std::to_string(bit)+"_"+std::to_string(type);
    f.tile(c).tile_type->elements.push_back(e);
  }
}
void arc(SourceInputFixture& f,Node a,Node b) {
  auto[x,y,k,n]=a;auto[u,v,l,m]=b;init(f,{x,y});init(f,{u,v});auto& cb=*f.tile({x,y}).cb_type;
  SourceInputFixture::name(f.tile({x,y}),fpga::CBNodeNameType(k),n);
  SourceInputFixture::name(f.tile({u,v}),fpga::CBNodeNameType(l),m);
  if(k==fpga::CB_NODE_SRC && l==fpga::CB_NODE_DST) {
    auto& targets=cb.dst_by_src[n];if(!targets.empty())return;
    fpga::CBType::ResolvedJump j;j.delta={u-x,v-y};j.target_tile_coord=true;
    j.target_cb_type_id=f.tile({u,v}).cb_type->type_id;j.dsts.jump.setBit(m);targets.push_back(j);return;
  }
  require(x==u && y==v,"unexpected non-jump coordinate change");
  if(k==0 && l==3)cb.local_src[n].jump.setBit(m);
  else if(k==0 && l==0)cb.local_local[n].local.setBit(m);
  else if(k==4 && l==3)cb.dst_src[n].jump.setBit(m);
  else if(k==4 && l==0)cb.dst_local[n].local.setBit(m);
  else if(k==4 && l==1)cb.dst_joint[n].joint.setBit(m);
  else if(k==1 && l==1)cb.joint_joint[n].joint.setBit(m);
  else if(k==1 && l==0)cb.joint_local[n].local.setBit(m);
  else require(false,"unsupported recorded edge");
}
Node node(C c,int kind,int id){return {c.x,c.y,kind,id};}
void install(SourceInputFixture& f,rtl::Net& net,rtl::Inst* source,rtl::Inst* sink,
             const char* port,Route route) {
  // Reconstruct the captured committed prefix / disconnected downstream
  // islands. Every bit is backed by a binding; no anonymous busy masks.
  auto lease=[&](C c,int kind,int id) {
    if(id<0)return;
    init(f,c);auto& cb=f.tile(c).cb;
    if(kind==0)cb.local.local.setBit(id);
    if(kind==1)cb.joint.jump.setBit(id);
    if(kind==3)cb.src.jump.setBit(id);
    if(kind==4)cb.dst.jump.setBit(id);
  };
  for(auto& w:route) {
    init(f,w.from);init(f,w.to);
    if(w.type==W::WIRE_TILE_PIN) {
      lease(w.from,0,w.local);f.tile(w.from).pin_state.leased_nodes.setBit(w.local);
      SourceInputFixture::name(f.tile(w.from),fpga::CB_NODE_LOCAL,w.local);
    } else if(w.type==W::WIRE_CROSSBAR) {
      if(w.jump>=0) {
        arc(f,node(w.from,w.pos==0?0:4,w.local),node(w.from,3,w.jump));
        arc(f,node(w.from,3,w.jump),node(w.to,4,w.dst));
        lease(w.from,3,w.jump);
      } else if(w.joint>=0) {
        Node previous=node(w.from,4,w.local);
        for(int joint:{w.joint2,w.joint})if(joint>=0) {
          arc(f,previous,node(w.from,1,joint));previous=node(w.from,1,joint);
        }
        const size_t i=static_cast<size_t>(&w-route.data());
        require(i+1<route.size(),"terminal lacks endpoint");
        arc(f,previous,node(w.from,0,route[i+1].local));
      } else if(w.joint<0) {
        // The recorded terminal is direct; its following PIN defines LOCAL.
        const size_t i=static_cast<size_t>(&w-route.data());require(i+1<route.size(),"terminal lacks endpoint");
        arc(f,node(w.from,4,w.local),node(w.from,0,route[i+1].local));
      }
      if(w.pos!=0 && w.owns_dst)lease(w.from,4,w.local);
      if(w.owns_landing)lease(w.to,4,w.dst);
      lease(w.from,1,w.joint);lease(w.from,1,w.joint2);
    } else if(w.type==W::WIRE_ROUTE_EDGE) {
      arc(f,node(w.from,w.from_node_type,w.from_node),node(w.to,w.to_node_type,w.to_node));
      lease(w.to,w.to_node_type,w.to_node);
    }
  }
  sink->wires.push_back(std::move(route));auto& stored=sink->wires.back();
  auto b=fpga::attachNetRoute(net,*sink,sink->wires.size()-1,source,sink,"O",port,
      SourceInputFixture::port(*sink,port).makeNetName());fpga::registerNetRouteTiles(net,stored,b);
}

}


// Captured Moving failure: a completed sibling owns the shared Takeoff,
// while the requested input retains a GAP and its two-JOINT downstream tail.
// Free current-position routing fails before a branch transit cut. The cut
// opens an 18-node / seven-jump path back to the ORIGINAL input. Moving used
// to skip that Tile and spend the remaining deadline relocating a LUT whose
// other input cannot follow. Keep the witness geometry and numeric node IDs;
// all names are generated. Unrelated closed exits are omitted. The closed
// control removes the required middle hop and must never claim completion.
void destination_branch_current_position_regression(bool closed) {
  SourceInputFixture f(false,0,16,68,15);
  for(C c:std::vector<C>{{56,10},{51,8},{59,10},{60,9},{65,12},{43,2}})slots(f,c);
  for(auto [resource,cb]:std::vector<std::pair<C,C>>{
      {{56,10},{57,10}},{{51,8},{50,8}},{{59,10},{58,10}},
      {{60,9},{60,9}},{{65,12},{65,12}},{{43,2},{44,2}}}) {
    init(f,cb);f.tile(resource).cb_coord=cb;
    f.tile(cb).attached_resource_tiles.push_back(&f.tile(resource));
  }
  auto output=[&](C c,int pos,int local) {
    auto& t=f.tile(c);t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",pos)].setBit(local);
    f.tile(t.cb_coord).cb_type->local_output_nodes.setBit(local);
  };
  auto input=[&](C c,int pos,const char* port,int local) {
    auto& t=f.tile(c);t.tile_type->pin_map.nodes[{"LUT6",port,pos}].setBit(local);
    f.tile(t.cb_coord).cb_type->local_input_nodes.setBit(local);
  };
  output({56,10},143,152);output({60,9},3,160);output({43,2},131,149);
  input({59,10},139,"I1",100);input({51,8},135,"I0",94);
  input({51,8},135,"I4",105);input({65,12},3,"I0",90);
  auto* driver=f.cell("branch_driver_503","LUT6",{{"O",true}},{56,10},143);
  auto* sibling=f.cell("branch_sink_509","LUT6",{{"I1",false}},{59,10},139);
  auto* sink=f.cell("gap_sink_521","LUT6",{{"I0",false},{"I4",false}},{51,8},135);
  auto* transit=f.cell("transit_driver_523","LUT6",{{"O",true}},{60,9},3);
  auto* remote=f.cell("transit_sink_541","LUT6",{{"I0",false}},{65,12},3);
  auto* control=f.cell("other_input_547","LUT6",{{"O",true}},{43,2},131);
  auto* net=f.connect(*driver,"O",*sibling,"I1");
  auto& in=SourceInputFixture::port(*sink,"I0");
  in.set(&rtl::Conn::fromBase(SourceInputFixture::port(*driver,"O")));
  in.port_ref->designator=SourceInputFixture::port(*driver,"O").port_ref->designator;
  auto* victim=f.connect(*transit,"O",*remote,"I0");
  auto* support=f.connect(*control,"O",*sink,"I4");
  driver->outline.fixed=driver->pnr_completed=sink->outline.fixed=sink->pnr_completed=true;
  const std::vector<Node> path={{57,10,0,152},{57,10,3,739},{58,9,4,739},
      {58,9,3,547},{62,10,4,547},{62,10,3,979},{66,6,4,979},
      {66,6,3,3075},{57,6,4,3080},{57,6,3,66},{57,8,4,66},
      {57,8,3,3845},{54,8,4,3841},{54,8,3,3076},{50,8,4,3073},
      {50,8,1,24},{50,8,1,10},{50,8,0,94}};
  for(size_t i=1;i<path.size();++i)arc(f,path[i-1],path[i]);
  if(closed)f.tile({62,10}).cb_type->dst_src[547].jump.clearBit(979);
  init(f,{58,10});init(f,{44,2});
  Route first{f.pin({57,10},152,true),f.edge({57,10},{58,9},152,739,739,true),
      f.edge({58,9},{58,10},739,19,22),f.terminal({58,10},22,100),f.pin({58,10},100,false)};
  install(f,*net,driver,sibling,"I1",first);
  Route broken{first[0],first[1]};for(auto& w:broken)w.shared=true;
  W gap;gap.type=W::WIRE_GAP;gap.from={58,9};gap.to={54,8};
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;gap.from_node=739;gap.to_node=3841;
  broken.push_back(gap);broken.push_back(f.edge({54,8},{50,8},3841,3076,3073));
  W terminal;terminal.from=terminal.to={50,8};terminal.local=3073;terminal.pos=1;
  terminal.joint2=24;terminal.joint=10;broken.push_back(terminal);broken.push_back(f.pin({50,8},94,false));
  install(f,*net,driver,sink,"I0",broken);
  Route obstruction{f.pin({60,9},160,true),f.edge({60,9},{58,9},160,900,900,true),
      f.edge({58,9},{62,10},900,547,547),f.edge({62,10},{65,12},547,900,900),
      f.terminal({65,12},900,90),f.pin({65,12},90,false)};
  install(f,*victim,transit,remote,"I0",obstruction);
  Route other{f.pin({44,2},149,true),f.edge({44,2},{50,8},149,901,3842,true),
      f.terminal({50,8},3842,105),f.pin({50,8},105,false)};
  install(f,*support,control,sink,"I4",other);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  f.auditAll();auto& r=f.tech.route;
  r.moving_stage=r.complete_routes_only=r.preserve_routes=true;
  r.route_deadends_enabled=false;r.route_preemption_enabled=true;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;
  r.indexSourceRoute(net,driver,"O");r.indexSourceRoute(victim,transit,"O");r.indexSourceRoute(support,control,"O");
  pnr::RouteDesign::RouteTask task{driver,sink,net,"O","I0",in.makeNetName()};task.fanout=true;task.endpoints_prepared=true;
  require(!r.routeAtCurrentPlacement(task,32768,true),"fixture already has a free current-position route before cutting transit");
  std::vector<pnr::RouteDesign::RouteTask> moved;std::string reason;
  bool wire_only=false;
  bool done=r.moveUnfinishedCell(task,&moved,&task,&reason,false,&wire_only);
  std::fprintf(stderr,"BRANCH_REPAIR done=%d cuts=%zu reason=%s\n",done,size_t(r.route_stats.preempt_success),reason.c_str());
  require(r.route_stats.preempt_success>0,"fixture never reached branch transit preemption");
  const auto found=net->findRouteBinding(driver,sink,"O","I0",task.net_name);
  require(found.index<net->routes.size(),"branch attempt lost its live binding");
  const auto& route=net->routes[found.index].owner->wires[net->routes[found.index].route_index];
  // A failed trace may retire its detached tail. Its live prefix must remain
  // incomplete, but need not retain a GAP marker for that retired tail.
  if(closed) {require(!done && !wire_only && !r.routeTaskIsComplete(task),"closed branch claimed a complete route");f.auditAll();return;}
  if(!done)require(r.routeAtCurrentPlacement(task,32768,true),
      "captured bypass cannot complete even after the real branch cut");
  require(done && wire_only && !fpga::hasRouteGaps(route) && route.back().type==W::WIRE_TILE_PIN,
      "branch cut opened original input, but Moving kept searching incompatible placements: "+reason);
  require(sink->coord.x==51 && sink->coord.y==8 && sink->pos==135 && sink->pnr_completed,
      "wire-only repair moved or readmitted the fixed sink");
  // Publishing the new trigger may append storage after the unrelated input.
  // Check that input's binding, not the most recently appended wire vector.
  const auto support_binding=support->findRouteBinding(control,sink,"O","I4",
      SourceInputFixture::port(*sink,"I4").makeNetName());
  require(support_binding.index<support->routes.size(),"lost independent input binding");
  const auto& support_route=support->routes[support_binding.index];
  require(sibling->wires.front().size()==first.size() &&
      support_route.owner->wires[support_route.route_index].size()==other.size(),
      "wire-only routing damaged the sibling or independent input");
  require(moved.empty() && !r.pending_route_todo.empty(),"lost displaced transit repair or detached original inputs");
  f.auditAll();
}


// Example 1, chunk 6: a register moved with its local logic to a Tile where
// another register already has the same CE. Its old 17-fragment route is still
// a source-connected prefix, not a GAP. Focused Moving continued that prefix
// before checking the completed sibling's terminal and failed. Reproduce the
// recorded prefix geometry and IDs with abstract node names. The completed
// sibling's unrelated detour is shortened; its shared prefix and private
// DST -> JOINT -> CE terminal remain. No anonymous occupancy is introduced.
void destination_retained_shared_ce_regression(bool foreign) {
  SourceInputFixture f(false,0,16,45,27);
  slots(f,{42,17}); init(f,{35,11}); init(f,{36,11});init(f,{41,17});
  auto& bank=f.tile({35,11});
  for(int bit:{0,2}) {fpga::Element e;e.name="register_lane_"+std::to_string(bit);
    e.type=fpga::ELEMENT_FD;e.bitmap_pos=bit;bank.tile_type->elements.push_back(e);}
  for(auto [resource,cb]:std::vector<std::pair<C,C>>{{{42,17},{41,17}},{{35,11},{36,11}}}) {
    f.tile(resource).cb_coord=cb; f.tile(cb).attached_resource_tiles.push_back(&f.tile(resource));
  }
  f.tile({42,17}).tile_type->pin_map.output_nodes[
      f.tile({42,17}).getNodeNum("LUT6","O",131)].setBit(157);
  f.tile({41,17}).cb_type->local_output_nodes.setBit(157);
  for(int pos:{0,8}) bank.tile_type->pin_map.nodes[{"FDRE","CE",pos}].setBit(64);
  f.tile({36,11}).cb_type->local_input_nodes.setBit(64);
  auto* driver=f.cell("enable_driver_601","LUT6",{{"O",true}},{42,17},131);
  auto* peer=f.cell("completed_register_607","FDRE",{{"CE",false}},{35,11},0);
  auto* sink=f.cell("moved_register_613","FDRE",{{"CE",false}},{35,11},8);
  auto* net=f.connect(*driver,"O",*peer,"CE");
  auto& in=SourceInputFixture::port(*sink,"CE");
  in.set(&rtl::Conn::fromBase(SourceInputFixture::port(*driver,"O")));
  in.port_ref->designator=SourceInputFixture::port(*driver,"O").port_ref->designator;
  // The real retained prefix includes a same-Tile crossbar jump and a private
  // detour returning to a different DST on an already visited Tile.
  const std::vector<std::tuple<C,C,int,int,int>> hops={
    {{41,17},{37,17},157,3077,3074},{{37,17},{37,18},3074,22,26},
    {{37,18},{37,19},26,19,22},{{37,19},{37,19},22,23,23},
    {{37,19},{37,20},23,16,19},{{37,20},{37,22},19,64,64},
    {{37,22},{37,23},64,20,24},{{37,23},{37,24},24,17,20},
    {{37,24},{40,24},20,258,262},{{40,24},{41,25},262,546,546},
    {{41,25},{41,19},546,162,162},{{41,19},{37,19},162,3076,3073},
    {{37,19},{37,21},3073,65,65},{{37,21},{37,23},65,65,65},
    {{37,23},{37,22},65,242,246},{{37,22},{37,20},246,193,193}};
  Route retained{f.pin({41,17},157,true)};
  for(auto [a,b,input,src,dst]:hops) {init(f,a);init(f,b);
    retained.push_back(f.edge(a,b,input,src,dst,retained.size()==1));}
  Route complete(retained.begin(),retained.begin()+6);
  complete.push_back(f.edge({37,20},{36,11},19,901,1026));
  W terminal;terminal.from=terminal.to={36,11};terminal.local=1026;
  terminal.pos=1;terminal.joint=56;
  complete.push_back(terminal);complete.push_back(f.pin({36,11},64,false));
  install(f,*net,driver,peer,"CE",complete);
  for(size_t n=0;n<6;++n)retained[n].shared=true;
  install(f,*net,driver,sink,"CE",retained);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  f.auditAll();auto& r=f.tech.route;
  r.moving_stage=r.complete_routes_only=r.preserve_routes=true;
  r.moving_focus_inst=sink;r.route_deadends_enabled=false;r.route_preemption_enabled=false;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;
  r.indexSourceRoute(net,driver,"O");
  pnr::RouteDesign::RouteTask task{driver,sink,net,"O","CE",in.makeNetName()};
  task.fanout=true;task.endpoints_prepared=true;
  if(foreign)task.from=f.control;
  bool done=r.routeNetTask(task);
  std::fprintf(stderr,"RETAINED_SHARED_CE done=%d foreign=%d continuations=%zu\n",
      done,foreign,size_t(r.route_stats.continuation_attempts));
  if(foreign) {require(!done,"borrowed a foreign CE terminal");f.auditAll();return;}
  auto found=net->findRouteBinding(driver,sink,"O","CE",task.net_name);
  require(done && found.index<net->routes.size(),"focused Moving missed the already completed shared CE input");
  auto& binding=net->routes[found.index];auto& result=binding.owner->wires[binding.route_index];
  require(fpga::isRouteComplete(result) && result.back().local==64 && result.back().shared,
      "shared CE replacement is not complete and shared");
  require(result.size()==complete.size() && peer->wires.front().size()==complete.size(),
      "shared CE replacement changed the sibling or consumed an extra route");
  require(r.route_stats.continuation_attempts==0 && r.route_stats.preempt_success==0,
      "shared CE replacement performed unnecessary prefix search or preemption");
  require(sink->coord.x==35 && sink->coord.y==11 && sink->pos==8,
      "shared CE replacement moved the register again");
  require(!f.tile({37,22}).cb.src.jump.testBit(20) &&
      !f.tile({37,20}).cb.dst.jump.testBit(193),"obsolete private prefix leases survived");
  f.auditAll();
}

// Example 1, run 6: Moving cuts the branch at (54,3)/SRC1030,
// opening a ten-jump source-to-original-input path. It is NOT completely free:
// its final jump is transit owned by another retained GAP route. The old
// post-cut retry disabled Grounding preemption, then resumed relocation until
// the stage deadline. Reproduce the witness coordinates, GAP boundaries,
// numeric IDs and two-JOINT terminal, with generated names and real owners.
// Unrelated closed exits and five input routes are omitted; one independent
// input suffices to prevent the sink from moving along this output-only path.
// A protected-transit control must not complete or destroy that owner.
void destination_branch_grounding_regression(bool protected_transit) {
  SourceInputFixture f(false,0,16,78,9);
  const std::vector<std::pair<C,C>> resources={
      {{46,2},{45,2}},{{69,3},{70,3}},{{55,2},{54,2}},
      {{52,4},{52,4}},{{60,5},{60,5}},{{72,6},{72,6}},
      {{73,5},{73,5}},{{74,3},{75,3}}};
  for(auto [resource,cb]:resources) {
    slots(f,resource);init(f,cb);f.tile(resource).cb_coord=cb;
    f.tile(cb).attached_resource_tiles.push_back(&f.tile(resource));
  }
  auto output=[&](C c,int pos,int local) {
    auto& t=f.tile(c);t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",pos)].setBit(local);
    f.tile(t.cb_coord).cb_type->local_output_nodes.setBit(local);
  };
  auto input=[&](C c,int pos,const char* port,int local) {
    auto& t=f.tile(c);t.tile_type->pin_map.nodes[{"LUT6",port,pos}].setBit(local);
    f.tile(t.cb_coord).cb_type->local_input_nodes.setBit(local);
  };
  output({46,2},131,149);output({52,4},3,150);output({72,6},3,151);output({74,3},3,161);
  input({69,3},3,"I3",91);input({69,3},3,"I2",81);input({55,2},3,"I0",100);
  input({60,5},3,"I0",90);input({73,5},3,"I0",92);
  auto* driver=f.cell("signal_driver_701","LUT6",{{"O",true}},{46,2},131);
  auto* sink=f.cell("retained_sink_709","LUT6",{{"I3",false},{"I2",false}},{69,3},3);
  auto* sibling=f.cell("branch_sink_719","LUT6",{{"I0",false}},{55,2},3);
  auto* branch_source=f.cell("branch_transit_source_727","LUT6",{{"O",true}},{52,4},3);
  auto* branch_sink=f.cell("branch_transit_sink_733","LUT6",{{"I0",false}},{60,5},3);
  auto* terminal_source=f.cell("terminal_transit_source_739","LUT6",{{"O",true}},{72,6},3);
  auto* terminal_sink=f.cell("terminal_transit_sink_743","LUT6",{{"I0",false}},{73,5},3);
  auto* control=f.cell("independent_input_751","LUT6",{{"O",true}},{74,3},3);
  auto* net=f.connect(*driver,"O",*sibling,"I0");
  auto& in=SourceInputFixture::port(*sink,"I3");
  in.set(&rtl::Conn::fromBase(SourceInputFixture::port(*driver,"O")));
  in.port_ref->designator=SourceInputFixture::port(*driver,"O").port_ref->designator;
  auto* branch=f.connect(*branch_source,"O",*branch_sink,"I0");
  auto* terminal_net=f.connect(*terminal_source,"O",*terminal_sink,"I0");
  auto* support=f.connect(*control,"O",*sink,"I2");
  driver->outline.fixed=driver->pnr_completed=sink->outline.fixed=sink->pnr_completed=true;
  const std::vector<Node> path={{45,2,0,149},{45,2,3,192},{45,1,4,67},
      {45,1,3,262},{49,1,4,259},{49,1,3,739},{50,1,4,544},
      {50,1,3,544},{53,2,4,544},{53,2,3,240},{54,3,4,259},
      {54,3,3,1030},{58,3,4,1027},{58,3,3,979},{63,2,4,816},
      {63,2,3,976},{67,3,4,819},{67,3,3,547},{70,4,4,547},
      {70,4,3,244},{70,3,4,248},{70,3,1,52},{70,3,1,37},{70,3,0,91}};
  for(size_t i=1;i<path.size();++i)arc(f,path[i-1],path[i]);
  auto edge=[&](C a,C b,int local,int src,int dst,bool takeoff=false) {
    init(f,a);init(f,b);return f.edge(a,b,local,src,dst,takeoff);
  };
  Route first{f.pin({45,2},149,true),edge({45,2},{45,1},149,192,67,true),
      edge({45,1},{49,1},67,262,259),edge({49,1},{50,1},259,739,544),
      edge({50,1},{53,2},544,544,544),edge({53,2},{54,3},544,240,259),
      edge({54,3},{54,2},259,244,248),f.terminal({54,2},248,100),f.pin({54,2},100,false)};
  install(f,*net,driver,sibling,"I0",first);
  Route broken(first.begin(),first.begin()+6);for(auto& w:broken)w.shared=true;
  W gap;gap.type=W::WIRE_GAP;gap.from={54,3};gap.to={54,1};
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;gap.from_node=259;gap.to_node=244;
  broken.push_back(gap);
  broken.push_back(edge({54,1},{57,1},244,738,545));
  broken.push_back(edge({57,1},{58,1},545,737,546));
  broken.push_back(edge({58,1},{62,1},546,260,263));
  broken.push_back(edge({62,1},{63,1},263,261,258));
  broken.push_back(edge({63,1},{66,1},258,259,257));
  broken.push_back(edge({66,1},{67,2},257,545,545));
  broken.push_back(edge({67,2},{70,3},545,545,545));
  broken.push_back(f.terminal({70,3},545,91));broken.push_back(f.pin({70,3},91,false));
  install(f,*net,driver,sink,"I3",broken);
  Route obstruction{f.pin({52,4},150,true),edge({52,4},{54,3},150,900,900,true),
      edge({54,3},{58,3},900,1030,1027),edge({58,3},{60,5},1027,900,900),
      f.terminal({60,5},900,90),f.pin({60,5},90,false)};
  install(f,*branch,branch_source,branch_sink,"I0",obstruction);
  W victim_gap=gap;victim_gap.from={72,5};victim_gap.to={70,4};
  victim_gap.from_node=victim_gap.to_node=901;
  Route obstruction2{f.pin({72,6},151,true),edge({72,6},{72,5},151,901,901,true),victim_gap,
      edge({70,4},{70,3},901,244,248),edge({70,3},{73,5},248,901,901),
      f.terminal({73,5},901,92),f.pin({73,5},92,false)};
  install(f,*terminal_net,terminal_source,terminal_sink,"I0",obstruction2);
  terminal_net->route_protected=protected_transit;
  Route other{f.pin({75,3},161,true),edge({75,3},{70,3},161,3077,3074,true)};
  W terminal;terminal.from=terminal.to={70,3};terminal.local=3074;terminal.pos=1;
  terminal.joint2=54;terminal.joint=39;other.push_back(terminal);other.push_back(f.pin({70,3},81,false));
  install(f,*support,control,sink,"I2",other);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();f.auditAll();auto& r=f.tech.route;
  r.moving_stage=r.complete_routes_only=r.preserve_routes=true;
  r.route_deadends_enabled=false;r.route_preemption_enabled=true;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;
  for(auto [n,source]:std::vector<std::pair<rtl::Net*,rtl::Inst*>>{
      {net,driver},{branch,branch_source},{terminal_net,terminal_source},{support,control}})
    r.indexSourceRoute(n,source,"O");
  pnr::RouteDesign::RouteTask task{driver,sink,net,"O","I3",in.makeNetName()};
  task.fanout=true;task.endpoints_prepared=true;
  require(!r.routeAtCurrentPlacement(task,32768,true),"fixture has an unexpected free route before the branch cut");
  std::vector<pnr::RouteDesign::RouteTask> moved;std::string reason;bool wire_only=false;
  bool done=r.moveUnfinishedCell(task,&moved,&task,&reason,false,&wire_only);
  const auto cuts=r.route_stats.preempt_success;
  std::fprintf(stderr,"BRANCH_GROUNDING done=%d wire_only=%d cuts=%zu protected=%d reason=%s\n",
      done,wire_only,size_t(cuts),protected_transit,reason.c_str());
  require(cuts>0,"fixture did not exercise the real branch cut");
  auto found=net->findRouteBinding(driver,sink,"O","I3",task.net_name);
  require(found.index<net->routes.size(),"grounding attempt lost its live binding");
  auto& result=net->routes[found.index].owner->wires[net->routes[found.index].route_index];
  if(protected_transit) {
    // Prefix truncation is allowed; completing through a protected owner is not.
    require(!done && !wire_only && !r.routeTaskIsComplete(task),"cut protected transit to claim completion");
    require(terminal_sink->wires.front().size()==obstruction2.size() &&
        f.tile({70,4}).cb.src.jump.testBit(244),"protected transit was changed");
    f.auditAll();return;
  }
  if(!done)require(r.routeAtCurrentPlacement(task,32768,true,true),
      "recorded focused Grounding witness cannot repair after the branch cut");
  require(done && wire_only,"Moving ignored legal Grounding after its branch cut and resumed relocation: "+reason);
  require(cuts==2 && fpga::isRouteComplete(result),"repair did not use exactly the branch and terminal transit cuts");
  require(std::any_of(result.begin(),result.end(),[](const W& w) {
    return w.from.x==70 && w.from.y==3 && w.joint==37 && w.joint2==52;
  }),"repair missed the captured two-JOINT terminal");
  require(sink->coord.x==69 && sink->coord.y==3 && sink->pos==3 && sink->pnr_completed && moved.empty(),
      "wire-only completion relocated or detached the sink");
  // The unrelated input must survive even when trigger routing replaces storage.
  const auto support_binding=support->findRouteBinding(control,sink,"O","I2",
      SourceInputFixture::port(*sink,"I2").makeNetName());
  require(support_binding.index<support->routes.size(),"lost Grounding-independent input binding");
  const auto& support_route=support->routes[support_binding.index];
  require(sibling->wires.front().size()==first.size() &&
      support_route.owner->wires[support_route.route_index].size()==other.size(),
      "Grounding damaged the sibling or independent input");
  require(!r.pending_route_todo.empty(),"lost displaced transit repairs");f.auditAll();
}

// Example 1 chunk 6: A and B exchange SRC258 while each first cuts C's
// SRC65. C can repair through SRC65 for free. The post-cut in-place probe
// uses the parallel SRC258, never the newly opened branch. This repeats
// without attempting A's reachable relocation at the end of its retained
// tail. Preserve the contested geometry and numeric IDs; shorten C's remote
// source prefix and omit closed exits/unrelated inputs. All occupancy has a
// real binding and all names are abstract. This isolates search ordering,
// not a claim that the reduced input/output constraints replay all packing.
void destination_branch_parallel_cycle_regression() {
  SourceInputFixture f(false,0,16,78,22);
  for(C c:std::vector<C>{{64,10},{61,10},{69,9},{65,13},{55,12},{74,10}})slots(f,c);
  for(auto [resource,cb]:std::vector<std::pair<C,C>>{
      {{64,10},{63,10}},{{61,10},{62,10}},{{69,9},{70,9}},
      {{65,13},{66,13}},{{55,12},{54,12}},{{74,10},{75,10}}}) {
    init(f,cb);f.tile(resource).cb_coord=cb;
    f.tile(cb).attached_resource_tiles.push_back(&f.tile(resource));
  }
  auto output=[&](C c,int pos,int local) {
    auto& t=f.tile(c);t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",pos)].setBit(local);
    f.tile(t.cb_coord).cb_type->local_output_nodes.setBit(local);
  };
  auto input=[&](C c,int pos,const char* port,int local) {
    auto& t=f.tile(c);t.tile_type->pin_map.nodes[{"LUT6",port,pos}].setBit(local);
    t.tile_type->pin_map.nodes[{"LUT2",port,pos}].setBit(local);
    f.tile(t.cb_coord).cb_type->local_input_nodes.setBit(local);
  };
  output({64,10},3,153);output({61,10},15,156);output({65,13},131,149);
  input({69,9},3,"I4",88);input({69,9},15,"I0",120);
  input({55,12},135,"I1",99);input({74,10},131,"I4",89);
  auto* a=f.cell("cycle_source_701","LUT6",{{"O",true}},{64,10},3);
  auto* b=f.cell("cycle_source_709","LUT6",{{"O",true}},{61,10},15);
  auto* c=f.cell("cycle_source_719","LUT6",{{"O",true}},{65,13},131);
  auto* sa=f.cell("cycle_sink_727","LUT6",{{"I4",false}},{69,9},3);
  auto* sb=f.cell("cycle_sink_733","LUT2",{{"I0",false}},{69,9},15);
  auto* sc=f.cell("cycle_sink_739","LUT2",{{"I1",false}},{55,12},135);
  for(auto* cell:{a,b,c,sa,sb,sc})cell->outline.fixed=cell->pnr_completed=true;
  auto* na=f.connect(*a,"O",*sa,"I4");auto* nb=f.connect(*b,"O",*sb,"I0");auto* nc=f.connect(*c,"O",*sc,"I1");
  auto edge=[&](C x,C y,int local,int src,int dst,bool takeoff=false) {
    init(f,x);init(f,y);return f.edge(x,y,local,src,dst,takeoff);
  };
  W gap;gap.type=W::WIRE_GAP;gap.from={66,10};gap.to={67,12};
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;gap.from_node=261;gap.to_node=66;
  Route ra{f.pin({63,10},153,true),edge({63,10},{66,10},153,257,261,true),gap,
      edge({67,12},{66,13},66,3618,3618),edge({66,13},{75,13},3618,1029,1030),
      edge({75,13},{75,11},1030,194,194),edge({75,11},{75,10},194,243,247),
      edge({75,10},{71,10},247,3843,3848),edge({71,10},{70,9},3848,3840,3845),
      f.terminal({70,9},3845,88),f.pin({70,9},88,false)};
  install(f,*na,a,sa,"I4",ra);
  Route rb{f.pin({62,10},156,true),edge({62,10},{63,10},156,261,258,true),
      edge({63,10},{66,10},258,259,257),edge({66,10},{67,10},257,258,262),
      edge({67,10},{70,9},262,738,738)};
  W terminal;terminal.from=terminal.to={70,9};terminal.local=738;terminal.pos=1;
  terminal.joint2=56;terminal.joint=41;rb.push_back(terminal);rb.push_back(f.pin({70,9},120,false));
  install(f,*nb,b,sb,"I0",rb);
  Route rc{f.pin({66,13},149,true),edge({66,13},{66,10},149,3078,3079,true),
      edge({66,10},{66,12},3079,65,65),edge({66,12},{57,12},65,3078,3079),
      edge({57,12},{54,12},3079,3843,3848)};
  terminal.from=terminal.to={54,12};terminal.local=3848;terminal.joint2=60;terminal.joint=45;
  rc.push_back(terminal);rc.push_back(f.pin({54,12},99,false));install(f,*nc,c,sc,"I1",rc);
  arc(f,{66,10,4,261},{66,10,3,65});arc(f,{66,10,4,261},{66,10,3,258});
  arc(f,{66,10,4,257},{66,10,3,65});
  arc(f,{67,10,4,262},{67,10,3,66});arc(f,{67,10,3,66},{67,12,4,66});
  arc(f,{75,11,4,194},{75,11,3,248});
  arc(f,{75,11,3,248},{75,10,4,242});
  arc(f,{75,10,4,242},{75,10,0,89});
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();f.auditAll();
  auto& r=f.tech.route;r.moving_stage=r.complete_routes_only=r.preserve_routes=true;
  r.route_deadends_enabled=false;r.route_preemption_enabled=true;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;
  std::vector<pnr::RouteDesign::RouteTask> tasks;
  for(auto [n,driver,sink,port]:std::vector<std::tuple<rtl::Net*,rtl::Inst*,rtl::Inst*,const char*>>{
      {na,a,sa,"I4"},{nc,c,sc,"I1"},{nb,b,sb,"I0"},{nc,c,sc,"I1"}}) {
    r.indexSourceRoute(n,driver,"O");
    pnr::RouteDesign::RouteTask task{driver,sink,n,"O",port,SourceInputFixture::port(*sink,port).makeNetName()};
    task.fanout=true;task.endpoints_prepared=true;tasks.push_back(task);
  }
  size_t completions=0;
  for(int round=0;round<3 && sa->coord.x==69;++round)for(auto& task:tasks) {
    if(r.routeAtCurrentPlacement(task,32768,true))continue;
    std::vector<pnr::RouteDesign::RouteTask> moved;std::string reason;bool wire_only=false;
    bool done=r.moveUnfinishedCell(task,&moved,&task,&reason,false,&wire_only);
    std::fprintf(stderr,"PARALLEL_CYCLE round=%d task=%s done=%d wire_only=%d cuts=%zu location=(%d,%d) reason=%s\n",
        round,task.to->cell_ref->name.c_str(),done,wire_only,size_t(r.route_stats.preempt_success),task.to->coord.x,task.to->coord.y,reason.c_str());
    require(done,"captured cycle task failed before completing its repeated repair: "+reason);
    ++completions;f.auditAll();
    if(sa->coord.x!=69)break;
  }
  require(sa->coord.x==74 && sa->coord.y==10 && sa->pos==131,
      "post-cut Grounding kept exchanging the parallel exit instead of reaching the available relocation");
  require(completions<=2 && r.route_stats.preempt_success<=3,"unbounded repairs before relocation");
  auto binding=na->findRouteBinding(a,sa,"O","I4",tasks.front().net_name);
  require(binding.index<na->routes.size(),"lost relocated trigger binding");
  const auto& route=na->routes[binding.index];
  require(fpga::isRouteComplete(route.owner->wires[route.route_index]),"relocation left the trigger unfinished");
  require(!r.pending_route_todo.empty(),"branch cuts lost their victim repairs");
  f.auditAll();
}
