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
std::string state(SourceInputFixture& f,rtl::Net& a,rtl::Net& b) {
  std::ostringstream out;fpga::dumpNetRouteHistory(a,out);fpga::dumpNetRouteHistory(b,out);
  for(auto& t:f.device.tile_grid)if(t.cb_type)out<<t.cb.local.local.str()<<t.cb.dst.jump.str()
    <<t.cb.src.jump.str()<<t.cb.joint.jump.str()<<t.pin_state.leased_nodes.str();
  return out.str();
}
}

// Example 048 failed after relocating a six-input LUT. Its I3 driver has two
// valid output LOCALs in the same CB. A three-fragment prefix starts at 151;
// that output has no completion. The captured 18-jump, 45-node path below
// starts at 159, crossing internal LOCAL bridges and two final
// JOINTs. Its first jump needs two transit cuts affecting two 12-fragment
// partial siblings with an existing GAP. Keep those exact routes and geometry;
// omit unrelated closed exits and owner siblings, normalizing the source PIN
// ownership of the imported siblings. All textual names are generated here.
// Mode 1 removes a required bridge: eligible foreign transit may be cut and
// queued, while the requester's prefix and fixed placements must survive.
// Mode 2 disables preemption; mode 3 unmaps the alternate output LOCAL. Both
// require unchanged routing. The positive case uses Moving's angle-led trace.
void destination_prefix_alternate_output_regression(int mode) {
  SourceInputFixture f(false,0,16,56,36);
  for(C c:std::vector<C>{{51,7},{55,28},{55,9},{43,2}})slots(f,c);
  for(auto [resource,cb]:std::vector<std::pair<C,C>>{{{51,7},{50,7}},{{55,28},{54,28}},{{55,9},{54,9}},{{43,2},{44,2}}}) {
    init(f,cb);f.tile(resource).cb_coord=cb;f.tile(cb).attached_resource_tiles.push_back(&f.tile(resource));
  }
  auto output=[&](C resource,int pos,std::initializer_list<int> pins) {
    auto& t=f.tile(resource);for(int pin:pins) {
      t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",pos)].setBit(pin);
      f.tile(t.cb_coord).cb_type->local_output_nodes.setBit(pin);
    }
  };
  auto input=[&](C resource,int pos,const char* port,int pin) {
    auto& t=f.tile(resource);t.tile_type->pin_map.nodes[{"LUT6",port,pos}].setBit(pin);
    f.tile(t.cb_coord).cb_type->local_input_nodes.setBit(pin);
  };
  output({51,7},139,{151,159});output({55,9},15,{156});
  if(mode==3) {
    auto& t=f.tile({51,7});
    t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",139)].clearBit(159);
  }
  input({55,28},139,"I3",101);input({43,2},15,"I0",118);input({43,2},11,"I0",111);
  auto* driver=f.cell("driver_401","LUT6",{{"O",true}},{51,7},139);
  auto* sink=f.cell("sink_409","LUT6",{{"I3",false}},{55,28},139);
  auto* owner=f.cell("transit_driver_419","LUT6",{{"O",true}},{55,9},15);
  auto* a=f.cell("transit_sink_421","LUT6",{{"I0",false}},{43,2},15);
  auto* b=f.cell("transit_sink_431","LUT6",{{"I0",false}},{43,2},11);
  driver->outline.fixed=driver->pnr_completed=sink->outline.fixed=true;
  auto* signal=f.connect(*driver,"O",*sink,"I3");auto* victim=f.connect(*owner,"O",*a,"I0");
  auto& extra=SourceInputFixture::port(*b,"I0");extra.set(&rtl::Conn::fromBase(SourceInputFixture::port(*owner,"O")));
  extra.port_ref->designator=SourceInputFixture::port(*owner,"O").port_ref->designator;
  const std::vector<Node> path={
    {50,7,0,159},
    {50,7,3,3536},
    {45,3,4,3536},
    {45,3,3,3536},
    {41,2,4,3379},
    {41,2,3,3619},
    {40,3,4,3619},
    {40,3,3,3619},
    {37,4,4,3619},
    {37,4,3,3379},
    {32,8,4,3379},
    {32,8,0,140},
    {32,8,3,816},
    {37,12,4,816},
    {37,12,3,3376},
    {32,16,4,3376},
    {32,16,0,183},
    {32,16,3,3379},
    {26,20,4,3379},
    {26,20,3,3379},
    {22,24,4,3379},
    {22,24,3,819},
    {26,29,4,819},
    {26,29,3,547},
    {31,30,4,547},
    {31,30,3,1031},
    {40,30,4,1031},
    {40,30,0,128},
    {40,30,0,165},
    {40,30,3,818},
    {44,34,4,818},
    {44,34,3,1028},
    {49,34,4,1026},
    {49,34,3,260},
    {50,34,4,263},
    {50,34,0,140},
    {50,34,3,976},
    {54,30,4,976},
    {54,30,3,241},
    {54,29,4,245},
    {54,29,3,241},
    {54,28,4,245},
    {54,28,1,23},
    {54,28,1,9},
    {54,28,0,101},
  };
  for(size_t i=1;i<path.size();++i)arc(f,path[i-1],path[i]);
  if(mode==1)f.tile({40,30}).cb_type->local_local[128].local.clearBit(165);
  for(int copy=0;copy<2;++copy) {
    Route route;
    { W w;w.type=W::Type(1);w.local=156;w.pos=15;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=1;w.shared=true;w.owns_dst=true;w.owns_landing=false;w.from={54,9};w.to={54,9};w.resource={55,9};route.push_back(w); }
    { W w;w.type=W::Type(0);w.local=156;w.pos=0;w.jump=3811;w.dst=3811;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=true;w.from={54,9};w.to={53,8};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(0);w.local=3811;w.pos=1;w.jump=3811;w.dst=3811;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=true;w.from={53,8};w.to={50,7};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(2);w.local=-1;w.pos=-1;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=4;w.from_node=3811;w.to_node_type=0;w.to_node=140;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=false;w.from={50,7};w.to={50,7};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(2);w.local=-1;w.pos=-1;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=0;w.from_node=140;w.to_node_type=3;w.to_node=3536;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=false;w.from={50,7};w.to={50,7};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(2);w.local=-1;w.pos=-1;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=3;w.from_node=3536;w.to_node_type=4;w.to_node=3536;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=false;w.from={50,7};w.to={45,3};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(3);w.local=-1;w.pos=-1;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=4;w.from_node=3536;w.to_node_type=4;w.to_node=817;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=false;w.from={45,3};w.to={45,3};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(0);w.local=817;w.pos=1;w.jump=737;w.dst=737;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=true;w.from={45,3};w.to={49,2};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(0);w.local=737;w.pos=1;w.jump=3809;w.dst=3809;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=true;w.from={49,2};w.to={45,1};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(0);w.local=3809;w.pos=1;w.jump=246;w.dst=3843;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=true;w.from={45,1};w.to={44,2};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(0);w.local=3843;w.pos=1;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=-1;w.shared=false;w.owns_dst=true;w.owns_landing=false;w.from={44,2};w.to={44,2};w.resource={-1,-1};route.push_back(w); }
    { W w;w.type=W::Type(1);w.local=118;w.pos=15;w.jump=-1;w.dst=-1;w.joint=-1;w.joint2=-1;w.from_node_type=-1;w.from_node=-1;w.to_node_type=-1;w.to_node=-1;w.pin_dir=0;w.shared=false;w.owns_dst=true;w.owns_landing=false;w.from={44,2};w.to={44,2};w.resource={43,2};route.push_back(w); }
    route.front().shared=false;
    if(copy){route.back().local=111;route.back().pos=11;}
    install(f,*victim,owner,copy?b:a,"I0",std::move(route));
  }
  init(f,{50,8});
  Route prefix{f.pin({50,7},151,true),f.edge({50,7},{50,8},151,22,26,true),
      f.edge({50,8},{50,8},26,23,23)};
  install(f,*signal,driver,sink,"I3",std::move(prefix));
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();f.auditAll();
  auto& r=f.tech.route;r.moving_stage=r.complete_routes_only=r.preserve_routes=true;
  r.route_deadends_enabled=false;r.route_preemption_enabled=mode!=2;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;
  r.indexSourceRoute(signal,driver,"O");r.indexSourceRoute(victim,owner,"O");
  pnr::RouteDesign::RouteTask task{driver,sink,signal,"O","I3",SourceInputFixture::port(*sink,"I3").makeNetName()};
  task.endpoints_prepared=true;
  const auto before=state(f,*signal,*victim);
  const Route requester_before=signal->routes[0].owner->wires[signal->routes[0].route_index];
  const auto requester_signature=[](const W& w) {
    return std::tuple{w.type,w.from.x,w.from.y,w.to.x,w.to.y,w.local,w.pos,
      w.jump,w.dst,w.joint,w.joint2,w.from_node_type,w.from_node,
      w.to_node_type,w.to_node,w.pin_dir,w.shared,w.owns_dst,w.owns_landing};
  };
  // Moving must exercise its angle-led entry, not the disabled broad source
  // proof. The retained prefix and alternate mapped output remain unchanged.
  const bool done=r.routeBeforeMoving(task);
  if(mode==1) {
    const auto& binding=signal->routes[0];
    const auto& requester=binding.owner->wires[binding.route_index];
    require(!done && binding.from==driver && binding.to==sink &&
        requester.size()==requester_before.size() &&
        std::equal(requester.begin(),requester.end(),requester_before.begin(),
          [&](const W& a,const W& b){return requester_signature(a)==requester_signature(b);}) &&
        driver->coord==C{51,7} && sink->coord==C{55,28} &&
        driver->outline.fixed && sink->outline.fixed,
        "closed bridge changed the requester prefix or fixed endpoints");
    const auto queued=[&](rtl::Inst* receiver,size_t index) {
      return std::count_if(r.pending_route_todo.begin(),r.pending_route_todo.end(),
        [&](const auto& next){return next.net==victim && next.from==owner &&
          next.to==receiver && next.net_name==victim->routes[index].route_name && next.fanout;});
    };
    require(r.pending_route_todo.size()==2 && queued(a,0)==1 && queued(b,1)==1 &&
        victim->routes.size()==2 && a->wires[0].size()==9 && b->wires[0].size()==9 &&
        r.route_stats.preempt_success==2 && r.route_stats.preempt_partial_victims==2 &&
        r.route_stats.preempt_complete_victims==0 &&
        !f.tile({50,7}).cb.src.jump.testBit(3536) &&
        !f.tile({45,3}).cb.dst.jump.testBit(3536),
        "closed bridge lost a transit victim or retained its cut leases");
    f.auditAll();return;
  }
  if(mode) {
    require(!done && state(f,*signal,*victim)==before && r.pending_route_todo.empty(),
        "failed alternate-output repair changed retained routing");f.auditAll();return;
  }
  require(done,"retained prefix rejected the other valid source LOCAL despite an 18-jump replacement");
  const auto binding=signal->findRouteBinding(driver,sink,"O","I3",task.net_name);
  const auto& complete=signal->routes[binding.index].owner->wires[signal->routes[binding.index].route_index];
  if(complete.front().local!=159 || complete.back().local!=101) {
    std::ostringstream dump;fpga::dumpNetRouteHistory(*signal,dump);std::fprintf(stderr,"%s",dump.str().c_str());
  }
  require(fpga::isRouteComplete(complete) && complete.front().local==159 && complete.back().local==101,
      "alternate-output routing did not replace both endpoints correctly");
  require(r.route_stats.preempt_success>0 && r.pending_route_todo.size()==2,
      "alternate-output routing did not preserve and schedule both transit victims");
  require(driver->coord==C{51,7} && sink->coord==C{55,28} && driver->outline.fixed && sink->outline.fixed,
      "wire repair moved a fixed endpoint");
  require(!f.tile({50,7}).cb.src.jump.testBit(22) && !f.tile({50,8}).cb.src.jump.testBit(23),
      "replaced dead prefix leaked its old transit leases");
  require(!f.tile({50,7}).pin_state.leased_nodes.testBit(151) &&
      !f.tile({50,7}).cb.local.local.testBit(151) &&
      !f.tile({50,7}).pin_state.leased_nodes.testBit(159),
      "replacement leaked the old reservation or charged an ordinary source reference");
  require(complete[1].from==complete.front().from && complete[1].pos==0 &&
      complete[1].local==complete.front().local,
      "source PIN is disconnected from the replacement Takeoff");
  require(r.moving_focus_inst==nullptr,"repair leaked its temporary focus");f.auditAll();
  require(fpga::unrouteNet(*signal),"cannot remove the replacement route");
  f.auditAll();
}
