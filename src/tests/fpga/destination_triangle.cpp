#include "SourceInputFixture.h"

#include <iomanip>

using namespace routing_tests;

namespace {
using Coord = fpga::Coord;
using Route = std::vector<fpga::Wire>;

void initialize(SourceInputFixture& f, Coord at) {
  auto& tile=f.tile(at);
  if(tile.cb_type)return;
  auto& cb=f.device.cb_types.emplace_back();
  cb.name="triangle_matrix_"+std::to_string(at.x)+"_"+std::to_string(at.y);
  cb.type_id=cb.base_type_id=int(f.device.cb_types.size()-1);
  auto& type=f.device.tile_types.emplace_back();
  type.name="triangle_resource_"+std::to_string(at.x)+"_"+std::to_string(at.y);
  type.type=fpga::Tile::TILE_LUTS;
  tile.coord=tile.cb_coord=at;tile.name=at;tile.full_name=type.name;
  tile.type=fpga::Tile::TILE_LUTS;tile.tile_type=&type;tile.cb_type=tile.cb.type=&cb;
}
void logic(SourceInputFixture& f,Coord at,int banks) {
  initialize(f,at);
  for(int bank=0;bank<banks;++bank)for(auto kind:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}) {
    fpga::Element e;e.name="triangle_logic_"+std::to_string(bank)+"_"+std::to_string(kind);
    e.type=kind;e.bitmap_pos=bank;f.tile(at).tile_type->elements.push_back(e);
  }
}
void install(SourceInputFixture& f,rtl::Net& net,rtl::Inst& from,const char* out,
             rtl::Inst& to,const char* in,Route route) {
  require(pnr::commitPreparedRoute(route),"cannot install triangle reference route");
  to.wires.push_back(std::move(route));
  auto id=fpga::attachNetRoute(net,to,to.wires.size()-1,&from,&to,out,in,
      SourceInputFixture::port(to,in).makeNetName());
  fpga::registerNetRouteTiles(net,to.wires.back(),id);
}
bool complete(rtl::Net& net,rtl::Inst& sink,const char* input) {
  for(const auto& b:net.routes)if(b.to==&sink && b.to_port==input && b.owner &&
      b.route_index<b.owner->wires.size() && fpga::isRouteComplete(b.owner->wires[b.route_index]))return true;
  return false;
}
int outputResource(fpga::Tile& tile,const std::string& kind,const std::string& port,int pos) {
  const int base=tile.getNodeNum(kind,port,pos);
  require(base>=0,"triangle output has no modeled resource pin: "+kind+"/"+port);
  const int site=tile.tile_type->sitePosForPlacedPos(pos);
  return base+(site>=0?site*256:0);
}
size_t expansions(const std::string& trace) {
  size_t total=0,at=0;
  while((at=trace.find("INPUT_PROOF_END ",at))!=std::string::npos) {
    auto n=trace.find("expanded=",at);require(n!=std::string::npos,"missing input work counter");
    total+=std::stoull(trace.substr(n+9));++at;
  }
  at=0;
  while((at=trace.find("SOURCE_REACHABILITY_POP ",at))!=std::string::npos){++total;++at;}
  at=0;
  while((at=trace.find("INPUT_REACHABILITY_POP ",at))!=std::string::npos){++total;++at;}
  return total;
}
}

// Example 1, 500-cell run 7: a register drives I1 of an older LUT2. That
// LUT2 and its I0-driving LUT2 share a Tile. Destination Moving reaches free
// Tiles near the register, but repeatedly floods a large disconnected I0
// fan-in for different LUT slots. The recorded attempt made 282 input proofs
// in 95 placement probes; its final I0 proof was cancelled at 177015 states.
// Keep the three-cell geometry and the retained two-jump I0 route, scaling
// the irrelevant fan-in down to a deterministic counter-based regression.
// This calls Moving itself. Inputs are routed recursively after committing a
// candidate; a disconnected input must fail without reviving preflight search.
// Every resource/node name is a synthetic pattern, and every busy node has
// a committed, registered route owner. No real-device node names are used.
void run_moving_triangle(int family,bool open,bool source_moving) {
  SourceInputFixture f(false,0,32,55,55);
  const Coord original{46,4},relay{45,5},reg_at{38,37},trunk_at{37,39},candidate{40,49};
  const std::array<std::string,5> kinds{"LUT2","MUXF5","MUXF7","CARRY4","FDRE"};
  const auto& kind=kinds.at(family);
  const std::string other_port=family==0?"I0":family<=2?"S":family==3?"CI":"D";
  const std::string trigger_port=family==0?"I1":family<=2?"I0":family==3?"S0":"CE";
  const std::string output_port=family==3?"CO[3]":family==4?"Q":"O";
  const int first_pos=family==0?3:family<=2?1:family==3?2:0;
  const int receiver_pos=family==0?7:family<=2?9:family==3?130:4;
  const int positions=family==3?2:family==1||family==2?4:8;
  auto position=[&](int bank) {
    if(family==3)return bank*128+2;
    if(family==4)return (bank/4)*128+(bank%4)*4;
    if(family<=2 && family>0)return (bank/2)*128+(bank%2)*8+1;
    return (bank/4)*128+(bank%4)*4+(family==0?3:1);
  };
  if(family==0){logic(f,original,2);logic(f,candidate,8);}
  else {
    const auto element=family<=2?fpga::ELEMENT_MUXF7:family==3?fpga::ELEMENT_CARRY:fpga::ELEMENT_FD;
    for(auto [at,count]:std::array<std::pair<Coord,int>,2>{{{original,2},{candidate,positions}}}) {
      initialize(f,at);
      for(int bank=0;bank<count;++bank){fpga::Element e;
        e.name="triangle_slot_"+std::to_string(bank);e.type=element;
        e.bitmap_pos=family==3?bank*4:family<=2?(bank/2)*4+(bank%2)*2:(bank/4)*8+bank%4;
        f.tile(at).tile_type->elements.push_back(e);}
    }
  }
  logic(f,trunk_at,1);
  for(auto at:{original,candidate}) {
    auto& sites=f.tile(at).tile_type->sites;
    sites.resize(2);
    for(int site=0;site<2;++site) {
      sites[site].name="triangle_site_"+std::to_string(site);
      sites[site].pos=site;
    }
  }
  initialize(f,relay);initialize(f,reg_at);
  fpga::Element reg_slot;reg_slot.name="triangle_register_slot";reg_slot.type=fpga::ELEMENT_FD;
  f.tile(reg_at).tile_type->elements.push_back(reg_slot);
  auto& old=*f.tile(original).tile_type;
  old.pin_map.output_nodes[outputResource(f.tile(original),kind,output_port,first_pos)].setBit(152);
  old.pin_map.nodes[{kind,other_port,receiver_pos}].setBit(113);
  old.pin_map.nodes[{kind,trigger_port,receiver_pos}].setBit(114);
  f.tile(original).cb_type->local_output_nodes.setBit(152);
  for(int n:{113,114})f.tile(original).cb_type->local_input_nodes.setBit(n);
  f.tile(reg_at).tile_type->pin_map.output_nodes[
      f.tile(reg_at).getNodeNum("FDRE","Q",0)].setBit(141);
  f.tile(reg_at).cb_type->local_output_nodes.setBit(141);
  f.tile(trunk_at).tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(87);
  f.tile(trunk_at).cb_type->local_input_nodes.setBit(87);
  auto* first=f.cell("triangle_first_41",kind,{{output_port.c_str(),true}},original,first_pos);
  auto* receiver=f.cell("triangle_receiver_43",kind,
      {{other_port.c_str(),false},{trigger_port.c_str(),false},{output_port.c_str(),true}},original,receiver_pos);
  f.tile(reg_at).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(95);
  f.tile(reg_at).cb_type->local_input_nodes.setBit(95);
  auto* reg=f.cell("triangle_register_47","FDRE",{{"D",false},{"Q",true}},reg_at,0);
  auto* sibling=f.cell("triangle_trunk_receiver_53","LUT6",{{"I0",false}},trunk_at,3);
  first->outline.fixed=first->pnr_completed=true;
  receiver->outline.fixed=receiver->pnr_completed=true;
  sibling->outline.fixed=sibling->pnr_completed=true;
  auto* other=f.connect(*first,output_port,*receiver,other_port);
  auto* signal=f.connect(*reg,"Q",*receiver,trigger_port);
  rtl::Net* output=nullptr;
  if(source_moving) {
    old.pin_map.output_nodes[outputResource(f.tile(original),kind,output_port,receiver_pos)].setBit(154);
    f.tile(original).cb_type->local_output_nodes.setBit(154);
    output=f.connect(*receiver,output_port,*reg,"D");
  }
  auto& input=SourceInputFixture::port(*sibling,"I0");
  input.port_ref->designator=SourceInputFixture::port(*reg,"Q").port_ref->designator;
  input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*reg,"Q")));
  Route local{SourceInputFixture::pin(original,152,true),
      f.edge(original,relay,152,1600,1601,true),f.edge(relay,original,1601,1602,1603),
      f.terminal(original,1603,113),SourceInputFixture::pin(original,113,false)};
  if(source_moving) {
    // Unlike the captured Destination case's two-hop detour, this suite
    // starts with a genuinely internal connection and no leased Takeoff.
    f.tile(original).cb_type->local_local[152].local.setBit(113);
    fpga::Wire internal;internal.type=fpga::Wire::WIRE_ROUTE_EDGE;
    internal.from=internal.to=original;
    internal.from_node_type=internal.to_node_type=fpga::CB_NODE_LOCAL;
    internal.from_node=152;internal.to_node=113;
    local={SourceInputFixture::pin(original,152,true),internal,
        SourceInputFixture::pin(original,113,false)};
  }
  Route trunk{SourceInputFixture::pin(reg_at,141,true),
      f.edge(reg_at,trunk_at,141,1700,1701,true),
      f.terminal(trunk_at,1701,87),SourceInputFixture::pin(trunk_at,87,false)};
  auto& dest=f.tile(candidate);
  for(int bank=0;bank<positions;++bank) {
    const int pos=position(bank);
    dest.tile_type->pin_map.nodes[{kind,other_port,pos}].setBit(180+bank);
    dest.tile_type->pin_map.nodes[{kind,trigger_port,pos}].setBit(220+bank);
    dest.cb_type->local_input_nodes.setBit(180+bank);
    dest.cb_type->local_input_nodes.setBit(220+bank);
    if(source_moving) {
      dest.tile_type->pin_map.output_nodes[outputResource(dest,kind,output_port,pos)].setBit(300+bank);
      dest.cb_type->local_output_nodes.setBit(300+bank);
      f.edge(candidate,reg_at,300+bank,2400+bank,2500+bank,true);
      f.terminal(reg_at,2500+bank,95);
      require(dest.getOutputPinNodes(kind,output_port,pos)==(NodeMask{0,1}<<(300+bank)),
          "source triangle output mapping does not select its declared fabric rail");
    }
  }
  for(int arrival=0;arrival<32;++arrival) {
    f.edge(trunk_at,candidate,1701,1750+arrival,1800+arrival);
    for(int bank=0;bank<positions;++bank)f.terminal(candidate,1800+arrival,220+bank);
  }
  // A real directed 2D fan-in, with converging alternatives at each hop.
  // It has no connection to the fixed first LUT in the closed case.
  std::vector<Coord> path;
  for(int y=45;y<=49;++y)for(int x=36;x<=40;++x)
    path.push_back({y%2?76-x:x,y});
  std::erase_if(path,[&](Coord at){return at.x==candidate.x && at.y==candidate.y;});
  for(auto at:path)initialize(f,at);
  for(size_t layer=0;layer+1<path.size();++layer)for(int lane=0;lane<32;++lane) {
    for(int previous=0;previous<32;++previous)
      f.edge(path[layer],path[layer+1],2100+previous,2000+lane,2100+lane);
  }
  for(int lane=0;lane<32;++lane) {
    f.edge(path.back(),candidate,2100+lane,2000+lane,2100+lane);
    for(int bank=0;bank<positions;++bank)f.terminal(candidate,2100+lane,180+bank);
  }
  if(open) {
    f.edge(relay,candidate,1601,1900,1901);
    for(int bank=0;bank<positions;++bank)f.terminal(candidate,1901,180+bank);
  }
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  install(f,*other,*first,output_port.c_str(),*receiver,other_port.c_str(),std::move(local));
  install(f,*signal,*reg,"Q",*sibling,"I0",std::move(trunk));
  require(f.tile(original).getOutputPinNodes(kind,output_port,first_pos)==(NodeMask{0,1}<<152),
      "triangle first driver's output aliases the other site's resource");
  if(source_moving)require(f.tile(original).getOutputPinNodes(kind,output_port,receiver_pos)==(NodeMask{0,1}<<154),
      "triangle moving source's output aliases the first driver's resource");
  require(first->tile.peer==receiver->tile.peer && first->pos!=receiver->pos,
      "triangle LUTs did not start in the same Tile");
  f.auditAll();
  auto& router=f.tech.route;
  router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
  router.moving_sources_stage=source_moving;router.route_recursion_budget=200;
  router.route_preemption_enabled=router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  router.indexSourceRoute(signal,reg,"Q");
  router.indexSourceRoute(other,first,output_port);
  pnr::RouteDesign::RouteTask task=source_moving
      ? pnr::RouteDesign::RouteTask{receiver,reg,output,output_port,"D",
          SourceInputFixture::port(*reg,"D").makeNetName()}
      : pnr::RouteDesign::RouteTask{reg,receiver,signal,"Q",trigger_port,
          SourceInputFixture::port(*receiver,trigger_port).makeNetName()};
  task.fanout=!source_moving;task.endpoints_prepared=true;
  std::vector<pnr::RouteDesign::RouteTask> affected;std::string reason;
  std::ostringstream trace;
  bool done=false,moved=false;
  {
    std::vector<rtl::Net*> nets;for(auto& net:f.parent->nets)nets.push_back(&net);
    pnr::RouteCongestionTrace capture(task.net_name,source_moving?"source_triangle":"destination_triangle",nets,&trace);
    done=source_moving ? router.moveUnfinishedSource(task,&affected,&reason,nullptr,&moved)
        : router.moveUnfinishedCell(task,&affected,&task,&reason);
  }
  const size_t work=expansions(trace.str());
  std::cout<<(source_moving?"source":"destination")<<" triangle kind="<<kind<<" open="<<open<<" complete="<<done
      <<" input_expansions="<<work<<" reason="<<reason<<'\n';
  require(done==open,"triangle gave the wrong reachability result: "+reason);
  if(source_moving)require(moved,"source triangle did not attempt the reachable candidate");
  require(reg->tile.peer==&f.tile(reg_at) && first->tile.peer==&f.tile(original),
      "triangle Moving changed a stationary endpoint");
  require(complete(*signal,*sibling,"I0"),"Destination Moving damaged the completed register trunk");
  if(open) {
    require(receiver->tile.peer==&dest && complete(*signal,*receiver,trigger_port.c_str()),
        "Destination Moving did not finish the trigger at the reference Tile");
    if(source_moving)require(complete(*other,*receiver,other_port.c_str()) && complete(*output,*reg,"D"),
        "Source Moving deferred an unfinished input/output instead of completing its routing");
    for(auto& repair:affected)require(router.routeNetTask(repair),"cannot reconnect triangle I0");
    require(complete(*other,*receiver,other_port.c_str()),"triangle receiver lost its other input");
    bool fabric=false,endpoint=false;
    for(const auto& b:other->routes)if(b.to==receiver && b.to_port==other_port && b.owner &&
        b.route_index<b.owner->wires.size()) {
      const auto& route=b.owner->wires[b.route_index];
      if(route.empty())continue;
      endpoint|=route.back().from==candidate;
      for(const auto& wire:route)fabric|=wire.from!=wire.to &&
          (wire.type==fpga::Wire::WIRE_ROUTE_EDGE || wire.type==fpga::Wire::WIRE_CROSSBAR);
    }
    require(fabric && endpoint,"former internal input did not become fabric connected at the moved cell");
    if(source_moving)require(complete(*output,*reg,"D"),"moved source did not complete its output");
  } else {
    // A real input failure happens after placement, not in a speculative
    // proof. Keep committed child work and explicitly account for unfinished inputs.
    require(!reason.empty() && !complete(*other,*receiver,other_port.c_str()),
        "disconnected triangle input was silently reported complete");
    size_t unfinished=0;
    for(const auto& input_task:router.movingInputTasks(*receiver)) {
      if(router.routeTaskIsComplete(input_task))continue;
      ++unfinished;
      const auto queued=[&](const auto& tasks) {
        return std::any_of(tasks.begin(),tasks.end(),[&](const auto& t) {
          return t.net==input_task.net && t.from==input_task.from && t.to==input_task.to &&
              t.from_port==input_task.from_port && t.to_port==input_task.to_port;
        });
      };
      require(queued(affected)||queued(router.pending_route_todo),
          "failed triangle input disappeared from pending work");
    }
    require(unfinished>0,"closed triangle has no recorded unfinished input");
  }
  for(auto& net:f.parent->nets)for(const auto& b:net.routes)
    if(b.owner && b.route_index<b.owner->wires.size() && !b.owner->wires[b.route_index].empty() &&
        !fpga::isRouteComplete(b.owner->wires[b.route_index])) {
      // A failed recursive child can retain its leased prefix for the pending
      // connection. Successful moves must leave no such unfinished binding.
      require(!open,"successful triangle left an incomplete routing binding");
      const auto queued=[&](const auto& tasks) {
        return std::any_of(tasks.begin(),tasks.end(),[&](const auto& t) {
          return t.net==&net && t.from==b.from && t.to==b.to &&
              t.from_port==b.from_port && t.to_port==b.to_port;
        });
      };
      require(queued(affected)||queued(router.pending_route_todo),
          "incomplete triangle prefix has no pending routing task");
    }
  f.auditAll();
  // Neither successful nor failed recursive moves may invoke the removed
  // input-proof search while selecting placement candidates.
  require(work==0,"Moving performed a removed input-reachability proof");
}

void destination_triangle_family_regression(int family,bool open) {run_moving_triangle(family,open,false);}
void destination_triangle_regression(bool open) {destination_triangle_family_regression(0,open);}
