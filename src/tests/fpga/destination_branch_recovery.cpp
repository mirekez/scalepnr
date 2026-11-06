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
// Free current-position recovery fails before a branch transit cut. The cut
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
  require(!r.recoverCurrentPosition(task,32768,true),"fixture already has a free current-position route before cutting transit");
  std::vector<pnr::RouteDesign::RouteTask> moved;std::string reason;
  bool wire_only=false;
  bool done=r.moveUnfinishedCell(task,&moved,&task,&reason,false,&wire_only);
  std::fprintf(stderr,"BRANCH_REPAIR done=%d cuts=%zu reason=%s\n",done,size_t(r.route_stats.preempt_success),reason.c_str());
  require(r.route_stats.preempt_success>0,"fixture never reached branch transit preemption");
  const auto found=net->findRouteBinding(driver,sink,"O","I0",task.net_name);
  const auto& route=net->routes[found.index].owner->wires[net->routes[found.index].route_index];
  if(closed) {require(!done && !wire_only && fpga::hasRouteGaps(route),"closed branch claimed a complete route");f.auditAll();return;}
  if(!done)require(r.recoverCurrentPosition(task,32768,true),
      "captured bypass cannot complete even after the real branch cut");
  require(done && wire_only && !fpga::hasRouteGaps(route) && route.back().type==W::WIRE_TILE_PIN,
      "branch cut opened original input, but Moving kept searching incompatible placements: "+reason);
  require(sink->coord.x==51 && sink->coord.y==8 && sink->pos==135 && sink->pnr_completed,
      "wire-only repair moved or readmitted the fixed sink");
  require(sibling->wires.front().size()==first.size() && sink->wires.back().size()==other.size(),
      "wire-only recovery damaged the sibling or independent input");
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
