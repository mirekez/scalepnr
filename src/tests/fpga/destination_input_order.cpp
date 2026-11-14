#include "SourceInputFixture.h"
#include "RouteSource.h"
#include <iostream>
using namespace routing_tests;
namespace {
using C=fpga::Coord;
void init(SourceInputFixture& f,C at,int slots=0) {
 auto& t=f.tile(at);
 if(!t.cb_type){auto& cb=f.device.cb_types.emplace_back();cb.name="probe_matrix_"+std::to_string(at.x)+"_"+std::to_string(at.y);cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;auto&tt=f.device.tile_types.emplace_back();tt.name="probe_resource_"+std::to_string(at.x)+"_"+std::to_string(at.y);tt.type=fpga::Tile::TILE_LUTS;t.coord=t.cb_coord=at;t.type=fpga::Tile::TILE_LUTS;t.tile_type=&tt;t.cb_type=t.cb.type=&cb;}
 for(int b=0;b<slots;++b)for(auto k:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}){fpga::Element e;e.name="probe_slot_"+std::to_string(b)+"_"+std::to_string(k);e.type=k;e.bitmap_pos=b;t.tile_type->elements.push_back(e);}
}
void mapinput(SourceInputFixture& f,C at,const char*port,int pos,int pin) {
 auto&t=f.tile(at);t.tile_type->pin_map.nodes[{"LUT6",port,pos}].setBit(pin);t.cb_type->local_input_nodes.setBit(pin);SourceInputFixture::name(t,fpga::CB_NODE_LOCAL,pin);
}
void install(SourceInputFixture&f,rtl::Net&n,rtl::Inst&from,const char*out,rtl::Inst&to,const char*in,std::vector<fpga::Wire>r){require(pnr::commitPreparedRoute(r),"cannot install probe owner");to.wires.push_back(std::move(r));auto id=fpga::attachNetRoute(n,to,to.wires.size()-1,&from,&to,out,in,SourceInputFixture::port(to,in).makeNetName());fpga::registerNetRouteTiles(n,to.wires.back(),id);}
// A column of packable destinations has disconnected independent inputs; a
// later branch reaches a usable position. Retain three inputs, one output,
// the original input's return path, and real registered transit ownership.
// Both input orders must complete actual recursive routing without proofs.
// Closed controls must report unfinished work and preserve consistent leases.
void run(int mode) {
 // Vary lane density, input order and 48-layer retained-path connectivity.
 // These are physical graphs, not search-budget or input-preview tests.
 const bool bounded=mode>=7,wide_success=mode==10;
 const bool remote=mode>=4,local_first=mode==1 || mode==5 || mode==8 || mode==10,
            open=mode!=2 && mode!=6 && mode!=9;
 const int decoys=bounded?2:18;
 SourceInputFixture f(false,0,16,40,32);
 C original{28,0},wide_at{29,0},good{26,0},root{24,0},reg_at{27,0}, output_at{27,1};
 C tight_at=remote?C{31,0}:original;
 if(remote)init(f,tight_at,1);
 init(f,original,2);init(f,wide_at,1);init(f,good,1);init(f,root,1);init(f,reg_at);init(f,output_at,1);
 fpga::Element e;e.name="probe_register";e.type=fpga::ELEMENT_FD;f.tile(reg_at).tile_type->elements.push_back(e);
 f.tile(reg_at).tile_type->pin_map.output_nodes[f.tile(reg_at).getNodeNum("FDRE","Q",0)].setBit(141);f.tile(reg_at).cb_type->local_output_nodes.setBit(141);
 for(auto at:{original,wide_at,tight_at}){auto&t=f.tile(at);t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",3)].setBit(152);t.cb_type->local_output_nodes.setBit(152);}
 for(auto [p,n]:std::vector<std::pair<const char*,int>>{{"I0",110},{"I3",111},{"I5",112}})mapinput(f,original,p,7,n);
 for(auto [p,n]:std::vector<std::pair<const char*,int>>{{"I0",130},{"I3",131},{"I5",132}})mapinput(f,good,p,3,n);
 mapinput(f,root,"I0",3,133);
 auto*reg=f.cell("probe_launch_51","FDRE",{{"Q",true}},reg_at);
 auto*wide=f.cell("probe_wide_53","LUT6",{{"O",true}},wide_at);
 auto*tight=f.cell("probe_local_59","LUT6",{{"O",true}},tight_at,3);
 auto*receiver=local_first
   ? f.cell("probe_receiver_61","LUT6",{{"I5",false},{"I3",false},{"I0",false},{"O",true}},original,7)
   : f.cell("probe_receiver_61","LUT6",{{"I0",false},{"I3",false},{"I5",false},{"O",true}},original,7);
 mapinput(f,output_at,"I0",3,140);
 auto*output_sink=f.cell("probe_output_71","LUT6",{{"I0",false}},output_at);
 auto*sibling=f.cell("probe_sibling_67","LUT6",{{"I0",false}},root,3);
 receiver->outline.fixed=receiver->pnr_completed=true;
 tight->outline.fixed=tight->pnr_completed=true;
 auto*wi=f.connect(*wide,"O",*receiver,"I0");auto*ti=f.connect(*tight,"O",*receiver,"I5");auto*tr=f.connect(*reg,"Q",*sibling,"I0");
 SourceInputFixture::port(*receiver,"I3").set(&rtl::Conn::fromBase(SourceInputFixture::port(*reg,"Q")));SourceInputFixture::port(*receiver,"I3").port_ref->designator=SourceInputFixture::port(*reg,"Q").port_ref->designator;
 auto wr=std::vector<fpga::Wire>{f.pin(wide_at,152,true),f.edge(wide_at,original,152,1800,1801,true),f.terminal(original,1801,110),f.pin(original,110,false)};
 auto*output_net=f.connect(*receiver,"O",*output_sink,"I0");
 f.tile(original).tile_type->pin_map.output_nodes[f.tile(original).getNodeNum("LUT6","O",7)].setBit(154);f.tile(original).cb_type->local_output_nodes.setBit(154);
 f.tile(good).tile_type->pin_map.output_nodes[f.tile(good).getNodeNum("LUT6","O",3)].setBit(154);f.tile(good).cb_type->local_output_nodes.setBit(154);
 auto outr=std::vector<fpga::Wire>{f.pin(original,154,true),f.edge(original,output_at,154,1810,1811,true),f.terminal(output_at,1811,140),f.pin(output_at,140,false)};
 f.edge(good,output_at,154,1812,1813,true);f.terminal(output_at,1813,140);
 C relay{30,1};init(f,relay);
 auto ir=std::vector<fpga::Wire>{f.pin(tight_at,152,true),f.edge(tight_at,relay,152,1802,1803,true),f.edge(relay,original,1803,1804,1805),f.terminal(original,1805,112),f.pin(original,112,false)};
 auto rr=std::vector<fpga::Wire>{f.pin(reg_at,141,true),f.edge(reg_at,root,141,1900,1901,true),f.terminal(root,1901,133),f.pin(root,133,false)};
 // The early exit follows a column of packable but unreachable candidates.
 for(int y=1;y<=decoys;++y){C at{24,y},up{35,y};init(f,at,8);init(f,up);
  f.edge(y==1?root:C{24,y-1},at,y==1?1901:2001,16,2001);
  for(int bank=0;bank<8;++bank){const int pos=(bank/4)*128+(bank%4)*4+3;
   mapinput(f,at,"I0",pos,130+bank*8);mapinput(f,at,"I3",pos,131+bank*8);mapinput(f,at,"I5",pos,132+bank*8);
   if(mode!=3) f.terminal(at,2001,131+bank*8);
   else {
    f.edge(up,at,3700+bank,3200+bank,3200+bank);
    f.terminal(at,3200+bank,131+bank*8);
   }
   f.tile(at).tile_type->pin_map.output_nodes[f.tile(at).getNodeNum("LUT6","O",pos)].setBit(500+bank);f.tile(at).cb_type->local_output_nodes.setBit(500+bank);
   f.edge(at,up,500+bank,400+bank,3900+bank,true);
   for(int i=0;i<2;++i){const int lane=bank*2+i;f.edge(up,at,3500+lane,3000+lane,3000+lane);f.terminal(at,3000+lane,(i?132:130)+bank*8);}
  }
 }
 f.edge(root,good,1901,64,2002);f.terminal(good,2002,131);
 if(!wide_success) {
  f.edge(wide_at,good,152,1902,1903,true);f.terminal(good,1903,130);
  f.edge(original,good,1801,1906,1907);f.terminal(good,1907,130);
 }
 if(open) {
  f.edge(tight_at,good,152,1904,1905,true);f.terminal(good,1905,132);
  f.edge(original,good,1805,1908,1909);f.terminal(good,1909,132);
 } else {
  init(f,{35,0});f.edge({35,0},good,3500,1904,1905);f.terminal(good,1905,132);
 }
 // A wide free component of the earlier I0 driver is unchanged for every
 // rejected candidate. The local I5 driver can reject those candidates cheaply.
 std::vector<C> layers;
 for(int y=4;y<8;++y)for(int x=wide_success?12:8;x<(wide_success?24:20);++x){layers.push_back({x,y});init(f,{x,y});}
 const int lanes=bounded?256:32;
 for(int lane=0;lane<lanes;++lane)f.edge(wide_at,layers.front(),152,2048+lane,2048+lane,true);
 for(size_t k=0;k+1<layers.size();++k)for(int lane=0;lane<lanes;++lane)f.edge(layers[k],layers[k+1],2048+lane,2048+lane,2048+lane);
 // I0 reaches every decoy, but the non-local I5 driver cannot. Completion
 // must depend on all actual input routes, not just the successful first one.
 if(remote && !bounded) for(int y=1;y<=decoys;++y) {
  C up{35,y},at{24,y};
  f.edge(layers.back(),up,2048,3600+y,3400+y);
  for(int bank=0;bank<8;++bank)
   f.edge(up,at,3400+y,3000+bank*2,3000+bank*2);
 }
 if(wide_success) {
  f.edge(original,layers.front(),1801,3800,2048);
  f.edge(layers.back(),good,2048,3650,3651);f.terminal(good,3651,130);
 }
 for(auto&cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
 f.device.rebuildIncomingDstMasks();
 install(f,*wi,*wide,"O",*receiver,"I0",std::move(wr));install(f,*ti,*tight,"O",*receiver,"I5",std::move(ir));install(f,*tr,*reg,"Q",*sibling,"I0",std::move(rr));install(f,*output_net,*receiver,"O",*output_sink,"I0",std::move(outr));f.auditAll();
 auto&r=f.tech.route;r.moving_stage=r.complete_routes_only=r.preserve_routes=true;r.moving_sources_stage=false;r.route_preemption_enabled=r.route_deadends_enabled=false;r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;r.indexSourceRoute(wi,wide,"O");r.indexSourceRoute(ti,tight,"O");r.indexSourceRoute(tr,reg,"Q");
 pnr::RouteDesign::RouteTask task{reg,receiver,tr,"Q","I3",SourceInputFixture::port(*receiver,"I3").makeNetName()};task.fanout=task.endpoints_prepared=true;std::vector<pnr::RouteDesign::RouteTask>affected;std::string reason;std::ostringstream log;
 // Select and commit first, then route every input recursively. There is no
 // input-proof allowance and no budget-driven deferral to a later operation.
 bool done;{pnr::RouteCongestionTrace trace(task.net_name,"recursive_order",{wi,ti,tr,output_net},&log);done=r.moveUnfinishedCell(task,&affected,&task,&reason);}
 std::string text=log.str();size_t work=0,at=0;while((at=text.find("SOURCE_REACHABILITY_POP ",at))!=std::string::npos){++work;++at;}
 at=0;while((at=text.find("INPUT_PROOF_END ",at))!=std::string::npos){auto n=text.find("expanded=",at);require(n!=std::string::npos,"missing input proof counter");work+=std::stoull(text.substr(n+9));++at;}
 require(work==0,"recursive destination moving performed a removed input proof");
 std::cout<<"RESULT done="<<done<<" target="<<receiver->coord.x<<','<<receiver->coord.y<<" input_work="<<work<<" affected="<<affected.size()<<" reason="<<reason<<'\n';
 if(!open) {
  require(!done && !reason.empty(),"closed local input was reported complete");
  // Child moves may remain committed. Every unfinished input must still be
  // represented in the parent's returned work or the live pending queue.
  for(const auto& input:r.movingInputTasks(*receiver)) if(!r.routeTaskIsComplete(input)) {
   const auto queued=[&](const auto& tasks){return std::any_of(tasks.begin(),tasks.end(),
       [&](const auto& t){return t.net==input.net && t.from==input.from &&
           t.to==input.to && t.from_port==input.from_port && t.to_port==input.to_port;});};
   require(queued(affected)||queued(r.pending_route_todo),"failed recursive input disappeared");
  }
  f.auditAll();
  return;
 }
 require(done && receiver->tile.peer==&f.tile(good),"missed viable candidate: "+reason);
 if(bounded) {
  // Match the real destination focus: inputs without a completed sibling
  // become Generic and can use its exact numeric retained-prefix fallback.
  r.moving_focus_inst=receiver;
  pnr::normalizeMovingSourceRoles(affected,[](const auto& t) {
    const auto source=rtl::routeSourceEndpoint(t.from,t.from_port);
    return std::to_string(reinterpret_cast<uintptr_t>(source.first))+source.second;
  },[&](const auto& t) {return r.sourceTreeHasCompleteExit(*t.from,t.from_port);},
    [](const auto& t) {return t.net && t.net->distributed_source;});
 }
 for(auto& t:affected) {
  std::ostringstream repair_trace;
  bool repaired;
  {pnr::RouteCongestionTrace trc(t.net_name,"probe_repair",{wi,ti,tr,output_net},&repair_trace);repaired=r.routeNetTask(t);}
  std::cout<<"REPAIR "<<t.net_name<<" port="<<t.to_port<<" done="<<repaired<<" fanout="<<t.fanout<<'\n';
  if(!repaired)std::cerr<<repair_trace.str();
  require(repaired,"cannot repair incident probe input");
 }
 if(bounded)r.moving_focus_inst=nullptr;
 f.auditAll();
 require(affected.size()==4,"Moving omitted an incident input or output");
 require(wide->tile.peer==&f.tile(wide_at) && tight->tile.peer==&f.tile(tight_at)
         && reg->tile.peer==&f.tile(reg_at) && sibling->tile.peer==&f.tile(root)
         && output_sink->tile.peer==&f.tile(output_at),"Moving changed a stationary endpoint");
 for(auto*net:{wi,ti,tr,output_net})for(const auto&binding:net->routes) {
   require(binding.owner && binding.route_index<binding.owner->wires.size(),"incident route lost storage");
   const auto&route=binding.owner->wires[binding.route_index];
   require(fpga::isRouteComplete(route) && route.front().from==binding.from->tile->cb_coord
           && route.back().from==binding.to->tile->cb_coord,"incident route retained stale placement endpoints");
 }
 require(r.route_stats.preempt_success==0,"proof-order regression preempted another owner");
 require(work==0,"incident completion used a removed input-proof search");
}
}
void destination_input_order_regression(int mode) {run(mode);}
