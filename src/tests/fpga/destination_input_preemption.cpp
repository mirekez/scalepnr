#include "SourceInputFixture.h"
#include "RouteSource.h"
#include <iostream>

using namespace routing_tests;

namespace {
using C=fpga::Coord;
using Route=std::vector<fpga::Wire>;
void slots(SourceInputFixture& f,C at,int count,bool reg=false) {
  for(int bit=0;bit<count;++bit)
    for(auto kind:reg?std::vector<fpga::ElementType>{fpga::ELEMENT_FD}
                     :std::vector<fpga::ElementType>{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}) {
      fpga::Element e;e.name="slot_"+std::to_string(bit)+"_"+std::to_string(kind);
      e.type=kind;e.bitmap_pos=bit;f.tile(at).tile_type->elements.push_back(e);
    }
}
void input(SourceInputFixture& f,C at,const char* port,int pos,int pin) {
  f.tile(at).tile_type->pin_map.nodes[{"LUT6",port,pos}].setBit(pin);
  f.tile(at).cb_type->local_input_nodes.setBit(pin);
  SourceInputFixture::name(f.tile(at),fpga::CB_NODE_LOCAL,pin);
}
void output(SourceInputFixture& f,C at,const char* kind,const char* port,int pos,int pin) {
  auto& t=f.tile(at);t.tile_type->pin_map.output_nodes[t.getNodeNum(kind,port,pos)].setBit(pin);
  t.cb_type->local_output_nodes.setBit(pin);SourceInputFixture::name(t,fpga::CB_NODE_LOCAL,pin);
}
void install(SourceInputFixture& f,rtl::Net& net,rtl::Inst* from,const char* out,
             rtl::Inst* to,const char* in,Route wires) {
  require(pnr::commitPreparedRoute(wires),"cannot install independent-input reference route");
  to->wires.push_back(std::move(wires));
  auto id=fpga::attachNetRoute(net,*to,to->wires.size()-1,from,to,out,in,
      SourceInputFixture::port(*to,in).makeNetName());
  fpga::registerNetRouteTiles(net,to->wires.back(),id);
}
bool complete(const rtl::Net& net,const rtl::Inst* sink) {
  for(const auto& b:net.routes)if(b.to==sink && b.owner &&
      b.route_index<b.owner->wires.size() && fpga::isRouteComplete(b.owner->wires[b.route_index])) return true;
  return false;
}
}

// Example 1's moved six-input LUT has an independent input whose free source
// component fits only in occupied or chain-incompatible Tiles. A free-only
// input proof is therefore negative everywhere else, even though a new
// Takeoff can preempt real foreign transit. Reproduce that decision with two
// LUT inputs, a full original Tile, one free destination, and a transit with a
// genuine bypass. Every busy node is owned by a committed registered route;
// all names use our numeric patterns. No cut is allowed during placement proof.
// mode 1 removes the input continuation after the contested hop: an eligible
// obstruction by itself must never admit a destination or damage its owner.
// mode 2 routes the transit over its bypass initially, preserving free-first
// behavior and requiring zero preemptions.
void destination_input_preemption_regression(int mode) {
  const bool repeated=mode>=6, broad=mode>=3 && !repeated, saturated=mode==5;
  SourceInputFixture f(false,0,16,repeated?16:8,repeated?20:broad?12:0);
  const bool disconnected=mode==1 || mode==3 || mode==6,free_control=mode==2;

  const C old{6,1},relay{4,0},good{3,1},launch{0,2},branch{2,1},load{3,2},va{7,2},vb{7,1};
  for(auto [at,n]:std::vector<std::pair<C,int>>{{old,2},{good,1},{branch,1},{load,1},{vb,1}})slots(f,at,n);
  slots(f,launch,1,true);slots(f,va,1,true);
  output(f,old,"LUT6","O",3,152);output(f,old,"LUT6","O",7,154);
  output(f,good,"LUT6","O",3,154);output(f,launch,"FDRE","Q",0,141);output(f,va,"FDRE","Q",0,141);
  input(f,old,"I0",7,112);input(f,old,"I1",7,111);
  input(f,good,"I0",3,132);input(f,good,"I1",3,131);
  input(f,branch,"I0",3,133);input(f,load,"I0",3,140);input(f,vb,"I0",3,87);
  auto* tight=f.cell("independent_driver_41","LUT6",{{"O",true}},old,3);
  auto* sink=f.cell("moved_receiver_43","LUT6",{{"I0",false},{"I1",false},{"O",true}},old,7);
  auto* reg=f.cell("trigger_driver_47","FDRE",{{"Q",true}},launch);
  auto* sibling=f.cell("trigger_sibling_53","LUT6",{{"I0",false}},branch);
  auto* out=f.cell("output_receiver_59","LUT6",{{"I0",false}},load);
  auto* vd=f.cell("crossing_driver_61","FDRE",{{"Q",true}},va);
  auto* vs=f.cell("crossing_receiver_67","LUT6",{{"I0",false}},vb);
  sink->outline.fixed=sink->pnr_completed=tight->outline.fixed=tight->pnr_completed=true;
  auto* other=f.connect(*tight,"O",*sink,"I0");
  auto* trigger=f.connect(*reg,"Q",*sibling,"I0");
  auto& ti=SourceInputFixture::port(*sink,"I1");
  ti.set(&rtl::Conn::fromBase(SourceInputFixture::port(*reg,"Q")));
  ti.port_ref->designator=SourceInputFixture::port(*reg,"Q").port_ref->designator;
  auto* onward=f.connect(*sink,"O",*out,"I0");auto* victim=f.connect(*vd,"Q",*vs,"I0");
  Route old_input{f.pin(old,152,true),f.edge(old,relay,152,1802,1803,true),
      f.edge(relay,old,1803,1804,1805),f.terminal(old,1805,112),f.pin(old,112,false)};
  Route trunk{f.pin(launch,141,true),f.edge(launch,branch,141,1900,1901,true),
      f.terminal(branch,1901,133),f.pin(branch,133,false)};
  f.edge(branch,good,1901,1902,1903);f.terminal(good,1903,131);
  f.edge(old,good,152,1904,1905,true);f.terminal(good,1905,132);
  // Packing has a free terminal alternative, but its ingress is disconnected
  // from this driver. Only the contested Takeoff can reach the actual input.
  f.edge(vb,good,2500,2501,2502);f.terminal(good,2502,132);
  Route old_output{f.pin(old,154,true),f.edge(old,load,154,1810,1811,true),
      f.terminal(load,1811,140),f.pin(load,140,false)};
  f.edge(good,load,154,1812,1813,true);f.terminal(load,1813,140);
  Route crossing{f.pin(va,141,true),f.edge(va,old,141,3000,3001,true)};
  auto contested=f.edge(old,good,3001,1904,1905);
  auto continuation=f.edge(good,vb,1905,3002,3003);f.terminal(vb,3003,87);
  auto bypass=f.edge(old,vb,3001,3004,3005);f.terminal(vb,3005,87);
  if(free_control)crossing.insert(crossing.end(),{bypass,f.terminal(vb,3005,87)});
  else crossing.insert(crossing.end(),{contested,continuation,f.terminal(vb,3003,87)});
  crossing.push_back(f.pin(vb,87,false));
  if(disconnected || repeated) {
    f.tile(good).cb_type->dst_local[1905].local.clearBit(132);
    f.edge(vb,good,2400,2401,2402);f.terminal(good,2402,132);
  }
  // Example 047: native samples caught repeated prepareInputRoute calls in
  // the per-exit preemption preview. The proposed cut opens only a tiny
  // forward region, but the candidate input has a broad unrelated reverse
  // fan-in. Cutting an eligible transit alone is not proof of connectivity.
  // Keep every occupied bit owned; the added 24x128 lanes are genuinely free.
  // The open control must still complete all incident routes and the victim.
  if(broad) {
    std::vector<C> layers;
    for(int y=4;y<(saturated?5:8);++y)for(int x=0;x<(saturated?2:6);++x) {
      C at{x,y};layers.push_back(at);auto& t=f.tile(at);
      auto& cb=f.device.cb_types.emplace_back();cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
      cb.name="preview_matrix_"+std::to_string(cb.type_id);
      auto& tt=f.device.tile_types.emplace_back();tt.name="preview_resource_"+std::to_string(cb.type_id);
      tt.type=fpga::Tile::TILE_LUTS;t.type=fpga::Tile::TILE_LUTS;
      t.tile_type=&tt;t.cb_type=t.cb.type=&cb;
    }
    const int lanes=saturated?2048:128,base=saturated?0:2048;
    if(saturated)f.tile(good).cb_type->dst_local[1905].local.clearBit(132);
    for(int lane=0;lane<lanes;++lane) {
      const int n=base+lane;
      // Preserve existing control/output jumps and do not recreate the
      // removed direct terminal from the contested landing itself.
      if(saturated && (n==1905 || !f.tile(good).cb_type->dst_by_src[n].empty()))continue;
      if(saturated)f.edge(good,layers.front(),1905,n,n);
      for(size_t k=0;k+1<layers.size();++k)f.edge(layers[k],layers[k+1],n,n,n);
      f.edge(layers.back(),good,n,n,n);f.terminal(good,n,132);
    }
  }
  // Example 050: the unchanged independent input has a small free region;
  // eligible Takeoff cuts open another closed region. Moving reaches thousands
  // of distinct candidate pins outside both, repeating the SAME cut/component
  // proof at every position. Reproduce the topology with a 61-Tile closed
  // region and a column of 13 Tiles, each with eight real packing positions.
  // Every obstruction has its registered transit owner. The open control's
  // final Tile lies inside the cut's component and must never be blacklisted.
  if(repeated) {
    auto init=[&](C at) {
      auto& t=f.tile(at);if(t.cb_type)return;
      auto& cb=f.device.cb_types.emplace_back();cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
      cb.name="component_matrix_"+std::to_string(cb.type_id);
      auto& tt=f.device.tile_types.emplace_back();tt.name="component_resource_"+std::to_string(cb.type_id);
      tt.type=t.type=fpga::Tile::TILE_LUTS;t.tile_type=&tt;t.cb_type=t.cb.type=&cb;
    };
    std::vector<C> region;
    for(int i=0;i<61;++i){C at{8+i%8,4+i/8};init(at);region.push_back(at);}
    f.edge(good,region.front(),1905,2700,2000);
    for(size_t i=0;i<region.size();++i) {
      auto at=region[i];auto& cb=*f.tile(at).cb_type;
      cb.dst_joint[2000].joint.setBit(100);SourceInputFixture::name(f.tile(at),fpga::CB_NODE_JOINT,100);
      for(int j=100;j<111;++j){cb.joint_joint[j].joint.setBit(j+1);SourceInputFixture::name(f.tile(at),fpga::CB_NODE_JOINT,j+1);}
      for(int local=200;local<210;++local){cb.joint_local[111].local.setBit(local);SourceInputFixture::name(f.tile(at),fpga::CB_NODE_LOCAL,local);}
      if(i+1<region.size())f.edge(at,region[i+1],2000,2001,2000);
    }
    for(int y=3;y<=15;++y) {
      C at{3,y},outside{4,y};init(at);init(outside);slots(f,at,8);
      f.edge(y==3?good:C{3,y-1},at,y==3?1903:2001,2100,2001);
      for(int bank=0;bank<8;++bank) {
        const int pos=bank/4*128+bank%4*4+3;
        input(f,at,"I1",pos,131+bank*8);input(f,at,"I0",pos,132+bank*8);
        f.terminal(at,2001,131+bank*8);
        f.edge(outside,at,2200+bank,2200+bank,2200+bank);f.terminal(at,2200+bank,132+bank*8);
        output(f,at,"LUT6","O",pos,500+bank);
        f.edge(at,load,500+bank,2300+bank,2300+bank,true);f.terminal(load,2300+bank,140);
      }
    }
    if(mode==7){f.edge(region.back(),{3,15},2000,2400,2400);f.terminal({3,15},2400,132);}
  }
  for(auto& cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  install(f,*other,tight,"O",sink,"I0",std::move(old_input));
  install(f,*trigger,reg,"Q",sibling,"I0",std::move(trunk));
  install(f,*onward,sink,"O",out,"I0",std::move(old_output));
  install(f,*victim,vd,"Q",vs,"I0",std::move(crossing));f.auditAll();
  auto& r=f.tech.route;r.moving_stage=r.complete_routes_only=r.preserve_routes=true;
  r.route_preemption_enabled=true;r.route_deadends_enabled=false;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;
  r.indexSourceRoute(other,tight,"O");r.indexSourceRoute(trigger,reg,"Q");
  pnr::RouteDesign::RouteTask task{reg,sink,trigger,"Q","I1",ti.makeNetName()};
  task.fanout=task.endpoints_prepared=true;std::vector<pnr::RouteDesign::RouteTask> affected;std::string reason;
  std::ostringstream decisions;bool moved;
  {pnr::RouteCongestionTrace trace(task.net_name,"preemption_preview",{other,trigger,onward,victim},&decisions);
    moved=r.moveUnfinishedCell(task,&affected,&task,&reason);}
  size_t expanded=0;std::istringstream lines(decisions.str());
  for(std::string line;std::getline(lines,line);) {
    if(line.starts_with("SOURCE_REACHABILITY_POP "))++expanded;
    if(line.starts_with("INPUT_PROOF_END ")) {
      const auto n=line.find("expanded=");require(n!=std::string::npos,"missing proof counter");
      expanded+=std::stoull(line.substr(n+9));
    }
  }
  std::cout<<"PREVIEW_WORK mode="<<mode<<" expanded="<<expanded<<" moved="<<moved<<'\n';
  if(saturated)require(decisions.str().find("SOURCE_REACHABILITY_LIMIT states=4096")!=std::string::npos,
      "reachable preview did not exercise the unknown-on-limit fallback");
  if(repeated) require(expanded>500 && expanded<10000,
      "Moving repeated a closed Takeoff component for every candidate pin");
  if(broad) require(expanded>0 && expanded<(saturated?30000:1000),
      "preemption preview flooded unrelated reverse fan-in instead of rejecting its dead landing");
  require(r.route_stats.preempt_success==0 && complete(*victim,vs),"placement proof cut a live transit");
  if(disconnected) {
    require(!moved && sink->tile.peer==&f.tile(old) && affected.empty(),"incomplete replacement admitted a destination");
    require(complete(*other,sink) && complete(*onward,out) && r.pending_route_todo.empty(),"failed preview damaged or deferred incident work");
    f.auditAll();return;
  }
  require(moved && sink->tile.peer==&f.tile(repeated?C{3,15}:good),"destination proof rejected a routable independent input: "+reason);
  require(affected.size()==3,"Moving omitted an incident connection");
  // Use the same source-role normalization as the real post-move focus.
  // The independent input has no completed sibling: it must become Generic,
  // rather than be helped by a test-only recovery call after Fanouts fails.
  r.moving_focus_inst=sink;
  pnr::normalizeMovingSourceRoles(affected,[](const auto& t) {
      const auto source=rtl::routeSourceEndpoint(t.from,t.from_port);
      return std::to_string(reinterpret_cast<uintptr_t>(source.first))+source.second;
    },[&](const auto& t) {return r.sourceTreeHasCompleteExit(*t.from,t.from_port);},
      [](const auto& t) {return t.net && t.net->distributed_source;});
  for(auto& t:affected)
    require(r.routeNetTask(t),"cannot finish incident connection after destination relocation");
  r.moving_focus_inst=nullptr;
  require(complete(*other,sink) && complete(*trigger,sink) && complete(*onward,out),"moved endpoint retained unfinished routes");
  require((r.route_stats.preempt_success==0)==free_control,"preemption did not preserve the free-path control");
  pnr::RouteDesign::RouteTask repair{vd,vs,victim,"Q","I0",SourceInputFixture::port(*vs,"I0").makeNetName()};
  require(r.routeNetTask(repair) && complete(*victim,vs),"displaced transit did not finish on its bypass");
  require(tight->tile.peer==&f.tile(old) && reg->tile.peer==&f.tile(launch) &&
      sibling->tile.peer==&f.tile(branch) && out->tile.peer==&f.tile(load),"Moving changed a stationary endpoint");
  require(sink->outline.fixed && !sink->pnr_completed && tight->outline.fixed && tight->pnr_completed,"Moving did not invalidate only the relocated cell completion");
  f.auditAll();
}
