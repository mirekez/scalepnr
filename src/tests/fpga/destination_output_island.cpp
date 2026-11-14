#include "SourceInputFixture.h"
#include <filesystem>
#include <fstream>
#include <tuple>

using namespace routing_tests;
namespace {
using C=fpga::Coord;
using W=fpga::Wire;
using N=std::tuple<int,int,int,int>;
void init(SourceInputFixture& f,C at) {
  auto& t=f.tile(at);if(t.cb_type)return;
  auto& cb=f.device.cb_types.emplace_back();cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
  cb.name="FILL_ISLAND_MATRIX_"+std::to_string(cb.type_id);
  auto& tt=f.device.tile_types.emplace_back();tt.name="FILL_ISLAND_RESOURCE_"+std::to_string(cb.type_id);
  tt.type=t.type=fpga::Tile::TILE_LUTS;t.tile_type=&tt;t.cb_type=t.cb.type=&cb;
}
void slots(SourceInputFixture& f,C at,C cb_at,int bit) {
  init(f,at);init(f,cb_at);auto& t=f.tile(at);t.cb_coord=cb_at;
  if(at!=cb_at)f.tile(cb_at).attached_resource_tiles.push_back(&t);
  for(auto type:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1,fpga::ELEMENT_MUXF7}) {
    fpga::Element e;e.name="FILL_SLOT_"+std::to_string(type);e.type=type;e.bitmap_pos=bit;
    e.elements_to_left=int(type);
    t.tile_type->elements.push_back(e);
  }
  for(int site=0;site<2;++site)t.tile_type->sites.push_back(
      fpga::SiteModel{.name="FILL_SITE_"+std::to_string(site),.type="LOGIC",.pos=site});
}
void arc(SourceInputFixture& f,N a,N b) {
  auto[x,y,k,n]=a;auto[u,v,l,m]=b;init(f,{x,y});init(f,{u,v});
  SourceInputFixture::name(f.tile({x,y}),fpga::CBNodeNameType(k),n);
  SourceInputFixture::name(f.tile({u,v}),fpga::CBNodeNameType(l),m);
  auto& cb=*f.tile({x,y}).cb_type;
  if(k==3 && l==4) {
    fpga::CBType::ResolvedJump j;j.delta={u-x,v-y};j.target_tile_coord=true;
    j.target_cb_type_id=f.tile({u,v}).cb_type->type_id;j.dsts.jump.setBit(m);
    auto& links=cb.dst_by_src[n];
    if(links.empty())links.push_back(j);
    else require(links.front().delta==j.delta && links.front().dsts.jump==j.dsts.jump,"conflicting captured jump");
    return;
  }
  require(x==u && y==v,"unexpected non-jump captured arc");
  if(k==0 && l==3)cb.local_src[n].jump.setBit(m);
  else if(k==0 && l==1)cb.local_joint[n].joint.setBit(m);
  else if(k==0 && l==0)cb.local_local[n].local.setBit(m);
  else if(k==4 && l==3)cb.dst_src[n].jump.setBit(m);
  else if(k==4 && l==1)cb.dst_joint[n].joint.setBit(m);
  else if(k==4 && l==0)cb.dst_local[n].local.setBit(m);
  else if(k==1 && l==1)cb.joint_joint[n].joint.setBit(m);
  else if(k==1 && l==0)cb.joint_local[n].local.setBit(m);
  else require(false,"unsupported captured arc");
}
void install(SourceInputFixture& f,rtl::Net& net,rtl::Inst* from,const char* output,
             rtl::Inst* to,const char* input,std::vector<W> route) {
  require(pnr::commitPreparedRoute(route),"cannot install reference route");
  to->wires.push_back(std::move(route));auto& ws=to->wires.back();
  auto i=fpga::attachNetRoute(net,*to,to->wires.size()-1,from,to,output,input,
      SourceInputFixture::port(*to,input).makeNetName());fpga::registerNetRouteTiles(net,ws,i);
}

// Drain accepted angle-led suffixes as the scheduler does. A failed attempt
// without progress ends the task; no removed whole-route proof is invoked.
bool traceAffected(pnr::RouteDesign& router,pnr::RouteDesign::RouteTask& task) {
  for(size_t pass=0;pass<4*fpga::Device::current().tile_grid.size();++pass) {
    router.route_progress=false;
    router.route_recursion_budget=router.route_suffix_depth_limit;
    if(router.routeNetTask(task))return true;
    if(!router.route_progress) {
      std::cout<<"AFFECTED_TRACE_FAILED net="<<task.net_name<<" fanout="<<task.fanout
               <<" pass="<<pass<<" attempt="<<task.attempt
               <<" changed="<<router.route_changed<<" moving="<<router.moving_stage
               <<" prepared="<<task.endpoints_prepared<<'\n';
      if(task.net)fpga::dumpNetRouteHistory(*task.net,std::cout);
      return false;
    }
  }
  require(false,"angle-led affected output did not terminate");
  return false;
}
}

// Example 059, run 6: Destination Moving accepted a LUT5 at (38,1)/131.
// Its rehomed output endpoint had free Takeoffs, but all 527 reachable free
// nodes formed an island short of the unchanged load at (64,17). The next
// affected output failed immediately. Preserve the captured free component,
// resource/CB split, trigger arrival and output pin. Unrelated occupied exits
// are omitted, rather than represented by ownerless busy bits. This isolates
// a legal local Takeoff with a disconnected downstream route. Placement must
// not perform a broad output proof; the later routing task reports the island.
// The open control adds one bridge out of that same island. Textual names are
// synthetic throughout. Both cases exercise production Destination Moving.
void destination_output_island_regression(int mode) {
  const bool open=mode!=0;
  SourceInputFixture f(false,0,16,70,25);
  const C old{52,17},old_cb{53,17},candidate{38,1},cb{37,1},load{64,17},load_cb{63,17};
  slots(f,old,old_cb,6);slots(f,candidate,cb,4);slots(f,load,load_cb,0);
  slots(f,{42,1},{41,1},0);init(f,{58,12});
  auto input=[&](C at,const char* type,const char* port,int pos,int pin) {
    f.tile(at).tile_type->pin_map.nodes[{type,port,pos}].setBit(pin);
    f.tile(f.tile(at).cb_coord).cb_type->local_input_nodes.setBit(pin);
  };
  auto output=[&](C at,const char* type,int pos,int pin) {
    auto& t=f.tile(at);const int site=t.tile_type->sitePosForPlacedPos(pos);
    t.tile_type->pin_map.output_nodes[t.getNodeNum(type,"O",pos)+(site>=0?site*256:0)].setBit(pin);
    f.tile(t.cb_coord).cb_type->local_output_nodes.setBit(pin);
  };
  input(old,"LUT5","I4",139,89);input(candidate,"LUT5","I4",131,89);
  input(load,"LUT5","I0",3,87);input({42,1},"LUT5","I0",3,3900);
  output(old,"MUXF7",137,157);output(candidate,"MUXF7",129,157);
  auto* moving=f.cell("FILL_MOVING_31402","LUT5",{{"I4",false},{"O",true}},old,139);
  auto* sink=f.cell("FILL_LOAD_31403","LUT5",{{"I0",false}},load,3);
  auto* sibling=f.cell("FILL_BRANCH_LOAD","LUT5",{{"I0",false}},{42,1},3);
  auto* endpoint=f.cell("FILL_OUTPUT_ENDPOINT_2643","MUXF7",{{"I1",false},{"O",true}},old,137);
  endpoint->cell_ref->attributes["scalepnr_passthrough"]="source";
  auto* local=f.connect(*moving,"O",*endpoint,"I1");local->void_net=true;
  // Reproduce the generated endpoint too: its output LOCAL is reachable only
  // through the linked element, and it must move with its LUT while the load
  // stays put. Initialize reciprocal links after connecting the live cells.
  for (auto [at,bit] : {std::pair{old,6},std::pair{candidate,4}}) {
    auto& tile=f.tile(at);
    for(auto& element:tile.tile_type->elements) {
      if(element.type==fpga::ELEMENT_LUT5)element.right_blockers[bit]|=1u<<bit;
      if(element.type==fpga::ELEMENT_MUXF7)element.left_blockers[bit]|=1u<<bit;
    }
    tile.elements_right[fpga::ELEMENT_LUT5][bit]|=1u<<bit;
    tile.elements_left[fpga::ELEMENT_MUXF7][bit]|=1u<<bit;
  }
  auto* trigger=f.connect(*f.control,"O",*moving,"I4");
  auto* signal=f.connect(*endpoint,"O",*sink,"I0");
  auto& peer=SourceInputFixture::port(*sibling,"I0");
  peer.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
  peer.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"destination_output_island_frontier.txt");
  require(bool(file),"missing captured output frontier");size_t edges=0;
  for(std::string line;std::getline(file,line);) {
    if(line.empty() || line[0]=='#')continue;
    std::istringstream row(line);
    int x,y,k,n,u,v,l,m;require(bool(row>>x>>y>>k>>n>>u>>v>>l>>m),"invalid captured arc");
    arc(f,{x,y,k,n},{u,v,l,m});++edges;
  }
  require(edges==805,"captured output component lost edges");
  auto control_cb=f.control->tile->cb_coord;const int control_pin=f.control->tile->getOutputPinNodes("LUT6","O",f.control->pos).firstSetBit();
  // Keep a completed trigger trunk and its real ownership before relocation.
  auto prefix=f.edge(control_cb,{41,1},control_pin,3901,3378,true);
  install(f,*trigger,f.control,"O",sibling,"I0",{f.pin(control_cb,control_pin,true),prefix,
      f.terminal({41,1},3378,3900),f.pin({41,1},3900,false)});
  f.edge({41,1},cb,3378,3077,3074);
  arc(f,{37,1,4,3074},{37,1,1,62});arc(f,{37,1,1,62},{37,1,1,47});arc(f,{37,1,1,47},{37,1,0,89});
  install(f,*signal,endpoint,"O",sink,"I0",{f.pin(old_cb,157,true),
      f.edge(old_cb,load_cb,157,3902,3902,true),f.terminal(load_cb,3902,87),f.pin(load_cb,87,false)});
  if(open) {f.edge({58,7},load_cb,818,3903,3903);f.terminal(load_cb,3903,87);}
  rtl::Net* victim=nullptr;
  if(mode==2) {
    // The same output bridge requires Grounding preemption. A free-only
    // negative must not reject this placement or cut this owner speculatively.
    const C va{62,16},vb{65,18};slots(f,va,va,0);slots(f,vb,vb,0);
    output(va,"LUT5",3,201);input(vb,"LUT5","I0",3,202);
    auto* a=f.cell("FILL_TRANSIT_DRIVER","LUT5",{{"O",true}},va,3);
    auto* b=f.cell("FILL_TRANSIT_LOAD","LUT5",{{"I0",false}},vb,3);
    victim=f.connect(*a,"O",*b,"I0");
    auto transit=f.edge(load_cb,vb,3910,3911,3911);transit.joint=3904;
    auto& fabric=*f.tile(load_cb).cb_type;
    fabric.dst_src[3910].jump.clearBit(3911);
    fabric.dst_local[3903].local.clearBit(87);
    arc(f,{63,17,4,3910},{63,17,1,3904});
    arc(f,{63,17,4,3903},{63,17,1,3904});
    arc(f,{63,17,1,3904},{63,17,0,87});
    fabric.joint_src[3904].jump.setBit(3911);
    install(f,*victim,a,"O",b,"I0",{f.pin(va,201,true),
        f.edge(va,load_cb,201,3910,3910,true),transit,
        f.terminal(vb,3911,202),f.pin(vb,202,false)});
  }
  for(auto& type:f.device.cb_types)type.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto& router=f.tech.route;router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
  router.moving_focus_inst=moving;
  router.route_preemption_enabled=mode==2;router.route_deadends_enabled=false;
  router.iteration_limit=router.route_recursion_budget=router.route_suffix_depth_limit=200;
  router.indexSourceRoute(trigger,f.control,"O");router.indexSourceRoute(signal,endpoint,"O");
  f.auditAll();
  pnr::RouteDesign::RouteTask task{f.control,moving,trigger,"O","I4",SourceInputFixture::port(*moving,"I4").makeNetName()};
  task.fanout=task.endpoints_prepared=true;
  std::vector<pnr::RouteDesign::RouteTask> affected;std::string reason;
  auto victim_state=[&] {
    std::ostringstream out;if(victim)fpga::dumpNetRouteHistory(*victim,out);return out.str();
  };
  const auto victim_before=victim_state();
  const bool moved=router.moveUnfinishedCell(task,&affected,&task,&reason);
  require(victim_state()==victim_before,"output placement preview cut a transit owner");
  require(moved,"local placement and recursive input routing failed: "+reason);
  require(std::any_of(affected.begin(),affected.end(),[&](const auto& next) {
    return next.net==signal && next.from==endpoint && next.to==sink;
  }),"relocation lost the unfinished output task");
  bool outputs=true;
  for(auto& next:affected)outputs&=traceAffected(router,next);
  std::cout<<"FILL_OUTPUT_ISLAND mode="<<mode<<" moved="<<moved<<" outputs="<<outputs
      <<" at=("<<moving->coord.x<<','<<moving->coord.y<<") reason="<<reason<<'\n';
  require(outputs==open,"output routing confused a disconnected island with its open bridge");
  if(mode==2)require(victim_state()!=victim_before,"preemption control did not use its transit bridge");
  require(moving->tile.peer==&f.tile(candidate),"unexpected selected placement");
  require(endpoint->tile.peer==moving->tile.peer && endpoint->pos==129,
      "generated endpoint was not rehomed/restored with its driver");
  require(sink->tile.peer==&f.tile(load),"output proof moved the unchanged load");
  f.auditAll();
}

// Example 060: a moved LUT6 at (35,3)/7 has a 20-hop free route from
// LOCAL162 to the unchanged load (80,6)/15. Its other output LOCAL154 and
// the linked MUX output LOCAL161 instead enter closed components. The MUX
// can reach a nearby first load, hiding the problem until the second fanout.
// Preserve the captured free arcs, all six inputs, resource/CB separation,
// and the output triangle. Moving validates local Takeoff, not downstream
// reachability. Route the far load as the Generic seed so ordinary tracing
// selects LOCAL162; endpoint preparation must preserve it for the near fanout.
void destination_output_retarget_regression() {
  SourceInputFixture f(false,0,32,105,25);
  const C old{30,4},old_cb{31,4},candidate{35,3},cb{36,3};
  const C near{33,5},near_cb{32,5},far{80,6},far_cb{81,6},trunk{43,6};
  slots(f,old,old_cb,4);slots(f,candidate,cb,1);
  // The output MUX occupies lane 0; the LUT occupies lane 1.
  for(auto& e:f.tile(candidate).tile_type->elements)
    if(e.type==fpga::ELEMENT_MUXF7)e.bitmap_pos=0;
  slots(f,near,near_cb,3);slots(f,far,far_cb,3);slots(f,trunk,trunk,0);
  for(C at:{old,candidate,near,far}) {
    auto& resource=f.tile(at);auto& route=f.tile(resource.cb_coord);
    resource.cb_type=resource.cb.type=route.cb_type;
  }
  auto input=[&](C at,const char* type,const std::string& port,int pos,int pin) {
    f.tile(at).tile_type->pin_map.nodes[{type,port,pos}].setBit(pin);
    f.tile(f.tile(at).cb_coord).cb_type->local_input_nodes.setBit(pin);
  };
  auto output=[&](C at,const char* type,int pos,int pin) {
    auto& t=f.tile(at);int site=t.tile_type->sitePosForPlacedPos(pos);
    t.tile_type->pin_map.output_nodes[t.getNodeNum(type,"O",pos)+(site>=0?site*256:0)].setBit(pin);
    f.tile(t.cb_coord).cb_type->local_output_nodes.setBit(pin);
  };
  const std::array<int,6> inputs{95,98,97,107,104,92};
  for(int i=0;i<6;++i) {
    input(old,"LUT6","I"+std::to_string(i),131,inputs[i]);
    input(candidate,"LUT6","I"+std::to_string(i),7,inputs[i]);
  }
  input(near,"LUT6","I5",15,123);input(far,"LUT2","I0",15,120);
  input(far,"LUT2","I1",15,121);input(trunk,"LUT5","I0",3,3900);
  output(old,"LUT6",131,154);
  output(candidate,"LUT6",7,154);output(candidate,"LUT6",7,162);
  output(candidate,"MUXF7",1,161);output(near,"LUT6",15,156);
  auto* moving=f.cell("FILL_RELOCATED_LOGIC_83","LUT6",
      {{"I0",false},{"I1",false},{"I2",false},{"I3",false},{"I4",false},{"I5",false},{"O",true}},old,131);
  auto* first=f.cell("FILL_NEAR_LOAD_92","LUT6",{{"I5",false},{"O",true}},near,15);
  auto* last=f.cell("FILL_FAR_LOAD_82","LUT2",{{"I0",false},{"I1",false}},far,15);
  auto* branch=f.cell("FILL_TRIGGER_BRANCH","LUT5",{{"I0",false}},trunk,3);
  auto* signal=f.connect(*moving,"O",*first,"I5");
  auto& last_input=SourceInputFixture::port(*last,"I0");
  last_input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*moving,"O")));
  last_input.port_ref->designator=SourceInputFixture::port(*moving,"O").port_ref->designator;
  for(auto [sink,port]:{std::pair{first,"I5"},std::pair{last,"I0"}}) {
    sink->wires.emplace_back();
    fpga::attachNetRoute(*signal,*sink,sink->wires.size()-1,moving,sink,"O",port,
        SourceInputFixture::port(*sink,port).makeNetName());
  }
  auto* triangle=f.connect(*first,"O",*last,"I1");
  install(f,*triangle,first,"O",last,"I1",{f.pin(near_cb,156,true),
      f.edge(near_cb,far_cb,156,3901,3901,true),f.terminal(far_cb,3901,121),f.pin(far_cb,121,false)});
  for(int i=0;i<5;++i) {
    C at{10+i,0};slots(f,at,at,0);output(at,"LUT5",3,169);
    auto* driver=f.cell("FILL_INPUT_DRIVER_"+std::to_string(i),"LUT5",{{"O",true}},at,3);
    const std::string port="I"+std::to_string(i);
    auto* net=f.connect(*driver,"O",*moving,port);
    install(f,*net,driver,"O",moving,port.c_str(),{f.pin(at,169,true),
        f.edge(at,old_cb,169,3910+i,3910+i,true),f.terminal(old_cb,3910+i,inputs[i]),f.pin(old_cb,inputs[i],false)});
    f.edge(old_cb,cb,3910+i,3920+i,3920+i);f.terminal(cb,3920+i,inputs[i]);
    f.tech.route.indexSourceRoute(net,driver,"O");
  }
  auto* trigger=f.connect(*f.control,"O",*moving,"I5");
  auto& branch_input=SourceInputFixture::port(*branch,"I0");
  branch_input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
  branch_input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
  const C start=f.control->tile->cb_coord;
  install(f,*trigger,f.control,"O",branch,"I0",{f.pin(start,169,true),
      f.edge(start,trunk,169,3930,3930,true),f.terminal(trunk,3930,3900),f.pin(trunk,3900,false)});
  f.edge(trunk,cb,3930,3931,21);f.terminal(cb,21,92);
  auto& t=f.tile(candidate);
  for(auto& e:t.tile_type->elements) {
    if(e.type==fpga::ELEMENT_LUT5)e.right_blockers[0]|=1u<<1;
    if(e.type==fpga::ELEMENT_MUXF7)e.left_blockers[1]|=1u<<0;
  }
  // Install after fixture placements have initialized the copied element map.
  t.elements_right[fpga::ELEMENT_LUT5][0]|=1u<<1;
  t.elements_left[fpga::ELEMENT_MUXF7][1]|=1u<<0;
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"destination_output_retarget_frontier.txt");
  require(bool(file),"missing captured output retarget frontier");size_t edges=0;
  for(std::string line;std::getline(file,line);) {
    if(line.empty()||line[0]=='#')continue;
    int x,y,k,n,u,v,l,m;std::istringstream row(line);
    require(bool(row>>x>>y>>k>>n>>u>>v>>l>>m),"invalid output retarget arc");
    if(k==0 && (x!=u||y!=v)) {
      init(f,{x,y});init(f,{u,v});f.resolvedLocal({x,y},n,{u,v},fpga::CBNodeNameType(l),m);
    } else arc(f,{x,y,k,n},{u,v,l,m});
    ++edges;
  }
  require(edges==3527,"captured output retarget frontier lost edges");
  // SourceInputFixture initializes a small live circuit before these resource
  // attachments exist. Refresh the grid-keyed attachment index without
  // moving any referenced Tile (the unused final slot has no owners).
  f.device.tile_grid.pop_back();f.device.routeTile(t);
  auto& unused=f.device.tile_grid.emplace_back();unused.coord=unused.cb_coord={104,24};
  for(C at:{old,candidate,near,far})
    require(f.device.routeTile(f.tile(at))==&f.tile(f.tile(at).cb_coord),
        "resource/CB attachment differs from captured device");
  for(auto& type:f.device.cb_types)type.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto& router=f.tech.route;
  router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
  router.moving_focus_inst=moving;router.route_preemption_enabled=false;router.route_deadends_enabled=false;
  router.iteration_limit=router.route_recursion_budget=router.route_suffix_depth_limit=200;
  router.indexSourceRoute(trigger,f.control,"O");router.indexSourceRoute(signal,moving,"O");
  router.indexSourceRoute(triangle,first,"O");f.auditAll();
  pnr::RouteDesign::RouteTask task{f.control,moving,trigger,"O","I5",SourceInputFixture::port(*moving,"I5").makeNetName()};
  task.fanout=task.endpoints_prepared=true;
  std::vector<pnr::RouteDesign::RouteTask> affected;std::string reason;
  require(router.moveUnfinishedCell(task,&affected,&task,&reason),"cannot relocate captured LUT: "+reason);
  require(moving->tile.peer==&t && moving->pos==7,"relocation did not choose captured position");
  rtl::NetRouteBinding probe;probe.from=moving;probe.to=last;probe.from_port="O";probe.to_port="I0";
  bool limited=false;
  require(fpga::previewUnboundNetRoute(*signal,probe,f.pin(cb,162,true),f.pin(far_cb,120,false),{},4096,200,{},&limited),
      "captured direct output witness is not routable");
  require(!fpga::previewUnboundNetRoute(*signal,probe,f.pin(cb,161,true),f.pin(far_cb,120,false),{},4096,200,{},&limited)&&!limited,
      "replacement MUX is not the captured closed component");
  // Choose the constrained load as the seed in this endpoint-identity test.
  // The old assertion relied on a removed broad output proof during placement.
  auto seed=std::find_if(affected.begin(),affected.end(),[&](const auto& next) {
    return next.net==signal && next.to==last && next.to_port=="I0";
  });
  require(seed!=affected.end(),"relocation lost the far output task");
  std::rotate(affected.begin(),seed,seed+1);
  bool complete=true;
  for(auto& next:affected) {
    const bool ok=traceAffected(router,next);
    complete &=ok;
    std::cout<<"FILL_OUTPUT_RETARGET source="<<next.from->makeName()<<" port="<<next.to_port<<" to="<<next.to->makeName()<<" pos="<<next.to->pos<<" complete="<<ok<<'\n';
    if(!ok)break;
  }
  require(complete,"output seed or fanout failed after endpoint-preserving relocation");
  for(auto* net:{signal,triangle})for(auto& b:net->routes)
    require(b.owner && fpga::isRouteComplete(b.owner->wires[b.route_index]),"output triangle is incomplete");
  for(auto& b:signal->routes)require(b.from==moving,"validated direct source was replaced");
  require(first->coord==near && last->coord==far,"post-move repair displaced an unchanged load");
  f.auditAll();
}
