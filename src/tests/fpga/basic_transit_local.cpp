#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <list>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Example 1 run 6: Basic failed although the read-only source audit found
// this entirely free ten-hop path. Keep all witness coordinates/numeric arcs,
// including both internal LOCAL bridges outside the sink's Docking zone.
// Names are synthetic, unrelated congestion is omitted, and endpoint resource
// Tiles are colocated with their CBs. No completed seed/fanout exists: this
// exercises Basic, unlike the earlier fanout_transit_local regression.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent, *primitive;
  rtl::Inst *source, *sink;
  rtl::Net *net;

  explicit Fixture(int mode) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=52; device.size_height=12; device.grid_spec.size={52,12};
    device.tile_grid.resize(52*12);
    device.tile_types.reserve(32); device.cb_types.reserve(32);
    for(int y=0;y<12;++y) for(int x=0;x<52;++x) device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="basic_bridge_top";
    parent->is_blackbox=false; parent->nets.reserve(8);
    primitive=&tech.design.modules.emplace_back(); primitive->name="abstract_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_101",{50,4},true); sink=cell("sink_102",{16,6},false);
    net=&parent->nets.emplace_back(); net->name="signal_1171"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1; sink->conns[0].port_ref->designator=1;
    sink->conns[0].set(&source->conns[1]);
    edge({50,4},{45,4},145,3074,3072,true);
    edge({45,4},{37,4},3072,3073,3078);
    edge({37,4},{36,3},3078,3809,3809);
    edge({36,3},{31,7},3809,3376,3376);
    bridge({31,7},{25,3},3376,196,3539,3539,mode==2?0:mode);
    bridge({25,3},{25,4},3539,177,162,97,mode==2?2:mode);
    edge({25,4},{22,5},97,3617,3617);
    edge({22,5},{22,11},3617,97,97);
    edge({22,11},{16,7},97,3538,3538);
    edge({16,7},{16,6},3538,248,242);
    tile({16,6}).cb_type->dst_local[242].local.setBit(114);
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  fpga::Tile &tile(Coord c) {
    auto &t=*device.getTile(c.x,c.y); if(t.cb_type) return t;
    auto &cb=device.cb_types.emplace_back(); cb.type_id=cb.base_type_id=device.cb_types.size()-1;
    cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=device.tile_types.emplace_back(); type.name="resource_"+std::to_string(cb.type_id);
    type.type=fpga::Tile::TILE_LUTS; type.pin_map.output_nodes[1].setBit(145);
    type.pin_map.nodes[{"LUT6","I5",0}].setBit(114);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(145); cb.local_input_nodes.setBit(114);
    name(t,fpga::CB_NODE_LOCAL,145); name(t,fpga::CB_NODE_LOCAL,114); return t;
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+std::to_string(unsigned(id)*2654435761U+unsigned(kind)));
  }
  void edge(Coord a,Coord b,int input,int src,int dst,bool local=false) {
    auto &t=tile(a); auto &to=tile(b); auto &cb=*t.cb_type;
    name(t,local?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,input);
    name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(local) cb.local_src[input].jump.setBit(src); else cb.dst_src[input].jump.setBit(src);
    const auto kind=local?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    cb.rememberConnName(kind,input,fpga::CB_NODE_SRC,src,*cb.nodeName(kind,input),
        *cb.nodeName(fpga::CB_NODE_SRC,src));
    fpga::CBType::ResolvedJump j{}; j.delta=b-a; j.target_tile_coord=true;
    j.target_cb_type_id=to.cb_type->type_id; j.dsts.jump.setBit(dst); cb.dst_by_src[src].push_back(j);
  }
  void bridge(Coord a,Coord b,int dst,int local,int src,int landing,int mode) {
    if(mode==1) {edge(a,b,dst,src,landing);return;}
    auto &t=tile(a); name(t,fpga::CB_NODE_LOCAL,local);
    if(mode!=2) {
      t.cb_type->dst_local[dst].local.setBit(local);
      t.cb_type->rememberConnName(fpga::CB_NODE_DST,dst,fpga::CB_NODE_LOCAL,local,
          *t.cb_type->nodeName(fpga::CB_NODE_DST,dst),*t.cb_type->nodeName(fpga::CB_NODE_LOCAL,local));
    }
    edge(a,b,local,src,landing,true);
  }
  rtl::Inst *cell(const std::string &name,Coord c,bool output) {
    auto &type=cells.emplace_back(); type.name=name; type.type=output?"FDRE":"LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction; p.designator=-1;
      p.name=direction==rtl::Port::PORT_IN?(output?"D":"I5"):(output?"Q":"O");
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c; i.outline.fixed=true;
    i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    tile(c).assign(&i); return &i;
  }
  Wire pin(Coord c,int local,bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=c; w.local=local; w.pos=0;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"FDRE":"LUT6"; w.port=output?"Q":"I5"; return w;
  }
  void audit() {
    for(auto &t:device.tile_grid) if(t.cb_type) {
      std::ostringstream out; auto a=fpga::auditTileCongestion(t,out,{net});
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
          !a.stale_registrations&&!a.conflicting_driver_nodes,out.str());
    }
  }
};
}

void basic_transit_local_regression(int mode) {
  debug_level=1; Fixture f(mode);
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I5";
  std::ostringstream reach;
  require(fpga::auditUnroutedReachability(*f.net,endpoints,f.pin({50,4},145,true),
      f.pin({16,6},114,false),reach)==(mode!=2),reach.str());
  f.audit();
  pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,
      .from_port="Q",.to_port="I5",.net_name=f.sink->conns[0].makeNetName(nullptr,1000000),
      .fanout=false,.endpoints_prepared=true};
  if(mode==2) {
    // A small opt-in diagnostic bound must be reported as limited, not as
    // proof that no path exists. It must not install a diagnostic binding.
    setenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY","signal_1171",1);
    setenv("SCALEPNR_SOURCE_REACHABILITY_STATES","7",1);
    setenv("SCALEPNR_SOURCE_REACHABILITY_LOG","basic_local_audit.log",1);
    std::ofstream clear("basic_local_audit.log",std::ios::trunc);
  }
  std::string failure;
  try {f.tech.route.routeTaskOrFail(task,"Basic routing",200);}
  catch(const std::runtime_error &e) {failure=e.what();}
  f.audit();
  require(!f.tech.route.route_stage_deadline_expired&&!f.tech.route.route_stage_stagnated,
      "fixture hit a deadline rather than exercising Basic topology");
  if(mode==2) {
    require(!failure.empty() && f.net->routes.empty() && f.sink->wires.empty(),
        "disconnected Basic route succeeded or retained speculative routing");
    std::ifstream in("basic_local_audit.log"); std::ostringstream audit; audit<<in.rdbuf();
    require(audit.str().find("max_states=7")!=std::string::npos &&
        audit.str().find("SOURCE_REACH end states=7 limited=1")!=std::string::npos,
        "failure audit ignored its diagnostic-only bound");
    return;
  }
  require(failure.empty(),"BUG: free ten-hop Basic witness rejected: "+failure);
  require(f.net->routes.size()==1 && f.sink->wires.size()==1 &&
      fpga::isRouteComplete(f.sink->wires[0]),"Basic did not install exactly one complete route");
  if(mode==0) {
    size_t hops=0, local_entries=0, local_exits=0;
    for(const auto &w:f.sink->wires[0]) {
      const bool moved=w.from.x!=w.to.x || w.from.y!=w.to.y;
      hops+=(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) ||
          (w.type==Wire::WIRE_ROUTE_EDGE && moved);
      if(w.type!=Wire::WIRE_ROUTE_EDGE) continue;
      local_entries+=w.from_node_type==fpga::CB_NODE_DST && w.to_node_type==fpga::CB_NODE_LOCAL &&
          (w.to_node==196 || w.to_node==177);
      local_exits+=w.from_node_type==fpga::CB_NODE_LOCAL && w.to_node_type==fpga::CB_NODE_SRC &&
          (w.from_node==196 || w.from_node==177);
    }
    require(hops==10 && local_entries==2 && local_exits==2,"completion did not use both witness bridges");
    require(f.tile({31,7}).cb.local.local.testBit(196) && f.tile({25,3}).cb.local.local.testBit(177),
        "internal bridge LOCAL was not leased");
  }
  require(f.tech.route.route_stats.preempt_success==0,"free witness caused preemption");
  require(f.source->coord==Coord{50,4} && f.sink->coord==Coord{16,6},"fixed endpoints moved");
  require(fpga::unrouteNetRoute(*f.net,0),"Basic witness teardown failed"); f.audit();
  require(!f.tile({31,7}).cb.local.local.testBit(196) && !f.tile({25,3}).cb.local.local.testBit(177),
      "teardown leaked internal bridge leases");
  std::puts("Basic ten-hop witness complete; no preemption; both internal bridges audited");
}
