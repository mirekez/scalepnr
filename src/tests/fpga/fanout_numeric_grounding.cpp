#include "Tech.h"
#include "Wire.h"

#include <list>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
using Route=std::vector<Wire>;
void check(bool ok,const std::string &why) {if(!ok) throw std::runtime_error(why);}
struct Node {int x,y,kind,id;};

// Recorded source-completion prefix for run-5 signal 1869. Preserve all
// coordinates, internal LOCAL bridges, and the zero-distance SRC/DST link.
// The final occupied jump is a real transit, not an invented busy bitmap.
// Other occupied sink approaches and unrelated branches cannot make this
// particular complete witness invalid, and are omitted to isolate the gap
// between numeric source search and Grounding preemption.
const Node prefix[]={
 {63,11,0,156},{63,11,3,261},{66,11,4,258},{66,11,3,1028},
 {70,11,4,1026},{70,11,3,66},{70,13,4,66},{70,13,3,3075},
 {62,13,4,3080},{62,13,3,3840},{58,12,4,3845},{58,12,0,183},
 {58,12,3,3379},{54,16,4,3379},{54,16,0,140},{54,16,3,3376},
 {50,20,4,3376},{50,20,0,165},{50,20,3,160},{50,14,4,160},
 {50,14,3,3536},{45,10,4,3536},{45,10,3,3072},{37,10,4,3077},
 {37,10,3,160},{37,4,4,160},{37,4,3,192},{37,2,4,192},
 {37,2,3,3072},{26,2,4,3077},{26,2,3,192},{26,1,4,67},
 {26,1,3,23},{26,1,4,23},{26,1,0,165},{26,1,3,3376},
 {22,5,4,3376},{22,5,3,96},{22,11,4,96},{22,11,3,257},
 {25,11,4,261}
};
struct Fixture {
 technology::Tech tech;
 fpga::Device &d=fpga::Device::current();
 std::list<Referable<rtl::Cell>> cells;
 Referable<rtl::Module> *parent,*primitive;
 rtl::Inst *source,*first,*sink,*other_source,*other_sink;
 rtl::Net *net,*victim;
 explicit Fixture(int mode) {
  d.tile_grid.clear(); d.tile_types.clear(); d.cb_types.clear();
  d.route_wire_graph.clear(); d.local_route_wire_mappings.clear();
  d.size_width=75; d.size_height=30; d.grid_spec.size={75,30};
  d.tile_grid.resize(75*30); d.tile_types.reserve(64); d.cb_types.reserve(64);
  for(int y=0;y<30;++y) for(int x=0;x<75;++x) d.getTile(x,y)->coord={x,y};
  tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr; tech.route.preserve_routes=true;
  parent=&tech.design.modules.emplace_back(); parent->name="top"; parent->is_blackbox=false;
  parent->nets.reserve(4);
  primitive=&tech.design.modules.emplace_back(); primitive->name="logic";
  primitive->is_blackbox=true; primitive->parent_ref.set(parent);
  tech.design.top_cell.name="top"; tech.design.top_cell.module_ref.set(parent);
  tech.design.top.cell_ref.set(&tech.design.top_cell);
  source=cell("driver_1869",{63,11}); first=cell("receiver_6309",{66,11});
  sink=cell("receiver_6299",{25,12});
  other_source=cell("driver_1016",mode==1?Coord{25,11}:Coord{22,11});
  other_sink=cell("receiver_6753",{22,12});
  net=connect("signal_1869",1,source,{first,sink});
  victim=connect("signal_1016",2,other_source,{other_sink});
  for(size_t i=1;i<std::size(prefix);++i) {
   if(mode==2 && i==12) continue; // Broken remote LOCAL exit: never cut a victim.
   arc(prefix[i-1],prefix[i]);
  }
  Route trunk{pin({63,11},156,true),hop({63,11},{66,11},156,261,258,true)};
  terminal(trunk,{66,11},258,56,64); install(*net,source,first,std::move(trunk));
  Route occupied{pin(other_source->coord,156,true)};
  if(mode!=1) occupied.push_back(hop({22,11},{25,11},156,259,257,true));
  occupied.push_back(hop({25,11},{25,12},mode==1?156:257,17,20,mode==1));
  occupied.push_back(hop({25,12},{22,12},20,3844,3840));
  terminal(occupied,{22,12},3840,56,64);
  install(*victim,other_source,other_sink,std::move(occupied));
  arc({25,11,4,261},{25,11,3,17});
  Route unused; terminal(unused,{25,12},20,55,63);
  // Repair alternative for the preempted transit; not reachable by requester.
  hop({25,11},{25,12},257,18,19);
  hop({25,12},{22,12},19,3844,3840);
  for(auto &cb:d.cb_types) cb.rebuildOutgoingSrcs();
  d.rebuildIncomingDstMasks();
  audit();
 }
 fpga::Tile &tile(Coord p) {
  auto &t=*d.getTile(p.x,p.y); if(t.cb_type) return t;
  auto &cb=d.cb_types.emplace_back(); cb.type_id=cb.base_type_id=d.cb_types.size()-1;
  cb.name="matrix_"+std::to_string(cb.type_id);
  auto &type=d.tile_types.emplace_back(); type.name=cb.name; type.type=fpga::Tile::TILE_LUTS;
  type.pin_map.output_nodes[16].setBit(156);
  type.pin_map.nodes[{"LUT6","I0",0}].setBit(p==Coord{25,12}?63:64);
  t.coord=t.cb_coord=p; t.name=p; t.full_name="tile_"+std::to_string(p.x)+"_"+std::to_string(p.y);
  t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
  cb.local_output_nodes.setBit(156); cb.local_input_nodes.setBit(p==Coord{25,12}?63:64);
  return t;
 }
 void arc(Node a,Node b) {
  auto &t=tile({a.x,a.y}); auto &to=tile({b.x,b.y}); auto &cb=*t.cb_type;
  auto name=[](fpga::Tile &t,Node n) {
   t.cb_type->rememberNodeName(static_cast<fpga::CBNodeNameType>(n.kind),n.id,
     "node_"+std::to_string(unsigned(n.id)*2654435761U+unsigned(n.kind)));
  }; name(t,a); name(to,b);
  if(a.kind==fpga::CB_NODE_SRC && b.kind==fpga::CB_NODE_DST) {
   fpga::CBType::ResolvedJump j{}; j.delta=Coord{b.x-a.x,b.y-a.y}; j.target_tile_coord=true;
   j.target_cb_type_id=to.cb_type->type_id; j.dsts.jump.setBit(b.id); cb.dst_by_src[a.id].push_back(j);
  } else {
   check(a.x==b.x && a.y==b.y,"unexpected cross-Tile intra-CB arc");
   if(a.kind==0 && b.kind==3) cb.local_src[a.id].jump.setBit(b.id);
   else if(a.kind==4 && b.kind==3) cb.dst_src[a.id].jump.setBit(b.id);
   else if(a.kind==4 && b.kind==0) cb.dst_local[a.id].local.setBit(b.id);
   else if(a.kind==4 && b.kind==1) cb.dst_joint[a.id].joint.setBit(b.id);
   else if(a.kind==1 && b.kind==0) cb.joint_local[a.id].local.setBit(b.id);
   else check(false,"unsupported fixture arc");
   cb.rememberConnName(static_cast<fpga::CBNodeNameType>(a.kind),a.id,
     static_cast<fpga::CBNodeNameType>(b.kind),b.id,*cb.nodeName(static_cast<fpga::CBNodeNameType>(a.kind),a.id),
     *cb.nodeName(static_cast<fpga::CBNodeNameType>(b.kind),b.id));
  }
 }
 Wire pin(Coord p,int local,bool out) {
  Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=p; w.local=local;
  w.pin_dir=out?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT; w.cell_type="LUT6"; w.port=out?"O":"I0"; return w;
 }
 Wire hop(Coord a,Coord b,int in,int src,int dst,bool local=false) {
  arc({a.x,a.y,local?0:4,in},{a.x,a.y,3,src}); arc({a.x,a.y,3,src},{b.x,b.y,4,dst});
  Wire w; w.from=a; w.to=b; w.local=in; w.jump=w.route_jump=src; w.dst=dst;
  w.pos=local?0:1; w.owns_landing=true; return w;
 }
 void terminal(Route &r,Coord p,int dst,int joint,int local) {
  arc({p.x,p.y,4,dst},{p.x,p.y,1,joint}); arc({p.x,p.y,1,joint},{p.x,p.y,0,local});
  Wire w; w.from=w.to=p; w.pos=1; w.local=dst; w.joint=joint;
  r.push_back(w); r.push_back(pin(p,local,false));
 }
 rtl::Inst *cell(const std::string &name,Coord p) {
  auto &c=cells.emplace_back(); c.name=name; c.type="LUT6"; c.module_ref.set(primitive); c.ports.reserve(2);
  for(auto dir:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
   auto &port=c.ports.emplace_back(); port.type=dir; port.name=dir==rtl::Port::PORT_IN?"I0":"O"; port.designator=-1;
  }
  auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&c); i.parent_ref.set(&tech.design.top);
  i.depth=1; i.pos=0; i.coord=p; i.outline.fixed=true; i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
  for(auto &port:c.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&port);conn.inst_ref.set(&i);}
  tile(p).assign(&i); return &i;
 }
 rtl::Net *connect(const std::string &name,int id,rtl::Inst *from,std::initializer_list<rtl::Inst *> tos) {
  auto &n=parent->nets.emplace_back(); n.name=name; n.designators.push_back(id);
  from->conns[1].port_ref->designator=id;
  for(auto *to:tos) {to->conns[0].port_ref->designator=id;to->conns[0].set(&from->conns[1]);} return &n;
 }
 void install(rtl::Net &n,rtl::Inst *from,rtl::Inst *to,Route r) {
  auto name=to->conns[0].makeNetName(nullptr,1000000);
  for(auto &w:r) {
   w.net_name=name; auto &t=tile(w.from);
   if(w.type==Wire::WIRE_CROSSBAR) {
    if(w.pos!=0) t.cb.dst.jump.setBit(w.local);
    if(w.jump>=0) {t.cb.src.jump.setBit(w.jump);tile(w.to).cb.dst.jump.setBit(w.dst);}
    if(w.joint>=0) t.cb.joint.jump.setBit(w.joint);
   } else if(w.pin_dir==fpga::TILE_PIN_INPUT) {t.cb.local.local.setBit(w.local);t.pin_state.leased_nodes.setBit(w.local);}
  }
  to->wires.push_back(std::move(r));
  auto b=fpga::attachNetRoute(n,*to,0,from,to,"O","I0",name); fpga::registerNetRouteTiles(n,to->wires[0],b);
 }
 void audit() {
  for(auto &t:d.tile_grid) if(t.cb_type) {
   std::ostringstream s; auto a=fpga::auditTileCongestion(t,s,{net,victim});
   check(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&!a.stale_registrations&&!a.conflicting_driver_nodes,s.str());
  }
 }
};
}

void fanout_numeric_grounding_regression(int mode) {
 debug_level=1; Fixture f(mode);
 rtl::NetRouteBinding endpoints; endpoints.from=f.source; endpoints.to=f.sink;
 endpoints.from_port="O"; endpoints.to_port="I0"; endpoints.route_name=f.sink->conns[0].makeNetName(nullptr,1000000);
 size_t cancelled_previews=0;
 check(!fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin({63,11},156,true),f.pin({25,12},63,false),
   []{return true;},32768,200,[&](const auto &,bool,auto &){++cancelled_previews;return false;}),
   "cancelled completion succeeded");
 check(cancelled_previews==0,"cancelled completion entered preemption");
 check(!fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin({63,11},156,true),f.pin({25,12},63,false)),
   "free-only search unexpectedly crossed the occupied terminal");
 check(f.sink->wires.empty() && f.net->routes.size()==1 && fpga::isRouteComplete(f.other_sink->wires[0]),
   "failed free search mutated a binding or victim");
 f.audit();
 pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,.from_port="O",.to_port="I0",
   .net_name=endpoints.route_name,.fanout=true,.endpoints_prepared=true};
 f.tech.route.fanout_stage=true; std::string failure;
 try {f.tech.route.routeTaskOrFail(task,"Fanouts routing",200);} catch(const std::runtime_error &e) {failure=e.what();}
 f.audit();
 check(fpga::isRouteComplete(f.first->wires[0]),"damaged existing source trunk");
 if(mode!=0) {
  check(!failure.empty() && f.sink->wires.empty(),"protected/disconnected case routed");
  check(fpga::isRouteComplete(f.other_sink->wires[0]),"cut victim without an eligible complete replacement"); return;
 }
 check(failure.empty(),"BUG: reached numeric prefix cannot complete through eligible Grounding transit: "+failure);
 check(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),"new suffix incomplete");
 size_t hops=0; bool internal_local=false,same_tile_jump=false;
 for(const auto &w:f.sink->wires[0]) {
  const bool moved=w.from.x!=w.to.x || w.from.y!=w.to.y;
  hops+=moved && ((w.type==Wire::WIRE_CROSSBAR && w.jump>=0) || w.type==Wire::WIRE_ROUTE_EDGE);
  internal_local|=w.type==Wire::WIRE_ROUTE_EDGE && w.from.x==58 && w.from.y==12 &&
      w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==183;
  same_tile_jump|=w.type==Wire::WIRE_CROSSBAR && w.from.x==26 && w.from.y==1 && !moved && w.jump==23 && w.dst==23;
 }
 check(hops==18 && internal_local && same_tile_jump,"completion did not use the recorded 18-hop witness");
 check(fpga::hasRouteGaps(f.other_sink->wires[0]),"victim was not preserved with a repairable gap");
 auto owners=fpga::findNetOwnersByNode(f.tile({25,11}),fpga::CB_NODE_SRC,17);
 check(!owners.empty() && owners.front().net==f.net,"final jump ownership was not transferred");
 pnr::RouteDesign::RouteTask repair{.from=f.other_source,.to=f.other_sink,.net=f.victim,.from_port="O",.to_port="I0",
   .net_name=f.other_sink->conns[0].makeNetName(nullptr,1000000),.fanout=true,.endpoints_prepared=true};
 f.tech.route.routeTaskOrFail(repair,"Fanouts routing",200); f.audit();
 check(fpga::isRouteComplete(f.other_sink->wires[0]) && fpga::isRouteComplete(f.sink->wires[0]),"victim repair failed");
 check(fpga::unrouteNetRoute(*f.net,1),"suffix teardown failed"); f.audit();
 check(fpga::isRouteComplete(f.first->wires[0]) && fpga::isRouteComplete(f.other_sink->wires[0]),"teardown damaged other routes");
}
