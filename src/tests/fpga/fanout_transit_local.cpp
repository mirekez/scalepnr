#include "Tech.h"
#include "Wire.h"

#include <array>
#include <cstdio>
#include <list>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
using Route = std::vector<Wire>;
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Import the complete free witness from Example 1 run 5, not just its free
// terminal approach. Preserve coordinates, numeric arcs, the completed trunk,
// and the internal DST -> LOCAL -> SRC bridge, far outside the Docking zone.
// Only names and unrelated congestion are replaced/omitted. The direct-arc
// control isolates bridge support from direction ordering and search budgets.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device = fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent, *primitive;
  rtl::Inst *source, *first, *sink;
  rtl::Net *net;

  explicit Fixture(int mode) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=70; device.size_height=35; device.grid_spec.size={70,35};
    device.tile_grid.resize(70*35);
    device.tile_types.reserve(32); device.cb_types.reserve(32);
    for (int y=0;y<35;++y) for (int x=0;x<70;++x)
      device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="bridge_top";
    parent->is_blackbox=false; parent->nets.reserve(8);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_206",{63,6}); first=cell("sink_701",{66,5});
    sink=cell("sink_702",{36,29});
    net=&parent->nets.emplace_back(); net->name="signal_206";
    net->designators.push_back(1); source->conns[1].port_ref->designator=1;
    for (auto *s:{first,sink}) {
      s->conns[0].port_ref->designator=1;
      s->conns[0].set(&source->conns[1]);
    }
    Route trunk{pin({63,6},153,true),edge({63,6},{66,7},153,240,259,true),
      edge({66,7},{66,6},259,244,248),edge({66,6},{66,5},248,249,244)};
    terminal(trunk,{66,5},244);
    install(trunk);
    edge({63,6},{58,2},153,3536,3536,true);
    if(mode==3) edge({66,5},{58,2},244,3536,3536);
    edge({58,2},{50,2},3536,3072,3077);
    edge({50,2},{41,2},3077,3072,3077);
    edge({41,2},{32,2},3077,3072,3077);
    auto &bridge=tile({32,2});
    name(bridge,fpga::CB_NODE_LOCAL,165);
    if (mode!=2) {
      if (mode==1) edge({32,2},{26,6},3077,3376,3376);
      else {
        bridge.cb_type->dst_local[3077].local.setBit(165);
        bridge.cb_type->rememberConnName(fpga::CB_NODE_DST,3077,fpga::CB_NODE_LOCAL,165,
            *bridge.cb_type->nodeName(fpga::CB_NODE_DST,3077),
            *bridge.cb_type->nodeName(fpga::CB_NODE_LOCAL,165));
        edge({32,2},{26,6},165,3376,3376,true);
      }
    }
    edge({26,6},{26,12},3376,96,96);
    edge({26,12},{26,18},96,96,96);
    edge({26,18},{26,24},96,96,96);
    edge({26,24},{32,29},96,816,816);
    edge({32,29},{36,29},816,257,261);
    Route unused; terminal(unused,{36,29},261);
    for (auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    audit();
  }
  fpga::Tile &tile(Coord c) {
    auto &t=*device.getTile(c.x,c.y);
    if (t.cb_type) return t;
    auto &cb=device.cb_types.emplace_back();
    cb.type_id=cb.base_type_id=device.cb_types.size()-1; cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=device.tile_types.emplace_back();
    type.name="resource_"+std::to_string(cb.type_id); type.type=fpga::Tile::TILE_LUTS;
    type.pin_map.output_nodes[16].setBit(153);
    type.pin_map.nodes[{"LUT6","I0",0}].setBit(64);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(153); cb.local_input_nodes.setBit(64);
    name(t,fpga::CB_NODE_LOCAL,153); name(t,fpga::CB_NODE_LOCAL,64);
    return t;
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+std::to_string(unsigned(id)*2654435761U+unsigned(kind)));
  }
  Wire pin(Coord c,int local,bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=c; w.local=local;
    w.pos=0; w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type="LUT6"; w.port=output?"O":"I0"; return w;
  }
  Wire edge(Coord a,Coord b,int in,int src,int dst,bool local=false) {
    auto &t=tile(a); auto &to=tile(b); auto &cb=*t.cb_type;
    const auto kind=local?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    name(t,kind,in); name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(local) cb.local_src[in].jump.setBit(src); else cb.dst_src[in].jump.setBit(src);
    cb.rememberConnName(kind,in,fpga::CB_NODE_SRC,src,*cb.nodeName(kind,in),
        *cb.nodeName(fpga::CB_NODE_SRC,src));
    fpga::CBType::ResolvedJump j{}; j.delta=b-a; j.target_tile_coord=true;
    j.target_cb_type_id=to.cb_type->type_id; j.dsts.jump.setBit(dst); cb.dst_by_src[src].push_back(j);
    Wire w; w.from=a; w.to=b; w.local=in; w.jump=w.route_jump=src; w.dst=dst;
    w.pos=local?0:1; w.owns_landing=true; return w;
  }
  void terminal(Route &r,Coord c,int dst) {
    auto &t=tile(c); name(t,fpga::CB_NODE_DST,dst); name(t,fpga::CB_NODE_JOINT,56);
    t.cb_type->dst_joint[dst].joint.setBit(56);
    t.cb_type->joint_local[56].local.setBit(64);
    Wire w; w.from=w.to=c; w.pos=1; w.local=dst; w.joint=56;
    r.push_back(w); r.push_back(pin(c,64,false));
  }
  rtl::Inst *cell(const std::string &name,Coord c) {
    auto &type=cells.emplace_back(); type.name=name; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    tile(c).assign(&i); return &i;
  }
  void install(Route r) {
    const auto route_name=first->conns[0].makeNetName(nullptr,1000000);
    for(auto &w:r) {
      w.net_name=route_name; auto &cb=tile(w.from).cb;
      if(w.type==Wire::WIRE_CROSSBAR) {
        if(w.pos!=0) cb.dst.jump.setBit(w.local);
        if(w.jump>=0) {cb.src.jump.setBit(w.jump);tile(w.to).cb.dst.jump.setBit(w.dst);}
        if(w.joint>=0) cb.joint.jump.setBit(w.joint);
      } else if(w.pin_dir==fpga::TILE_PIN_INPUT) {
        cb.local.local.setBit(w.local); tile(w.from).pin_state.leased_nodes.setBit(w.local);
      }
    }
    first->wires.push_back(std::move(r));
    auto b=fpga::attachNetRoute(*net,*first,0,source,first,"O","I0",route_name);
    fpga::registerNetRouteTiles(*net,first->wires[0],b);
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

void fanout_transit_local_regression(int mode) {
  debug_level=1; Fixture f(mode);
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="O"; endpoints.to_port="I0";
  std::ostringstream reach;
  const bool reachable=fpga::auditUnroutedReachability(*f.net,endpoints,
      f.pin({63,6},153,true),f.pin({36,29},64,false),reach);
  require(reachable==(mode!=2),reach.str());
  endpoints.route_name=f.sink->conns[0].makeNetName(nullptr,1000000);
  require(!fpga::routeUnboundNetFromSource(*f.net,endpoints,
      f.pin({63,6},153,true),f.pin({36,29},64,false),[]{return true;}),
      "cancelled numeric completion succeeded");
  require(f.net->routes.size()==1 && f.sink->wires.empty(),
      "cancelled completion installed a placeholder binding");
  f.audit();
  const auto trunk=f.first->wires[0].size();
  pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,
      .from_port="O",.to_port="I0",.net_name=f.sink->conns[0].makeNetName(nullptr,1000000),
      .fanout=true,.endpoints_prepared=true};
  f.tech.route.fanout_stage=true;
  std::string failure;
  try {f.tech.route.routeTaskOrFail(task,"Fanouts routing",200);}
  catch(const std::runtime_error &e) {failure=e.what();}
  f.audit();
  require(fpga::isRouteComplete(f.first->wires[0]) && f.first->wires[0].size()==trunk,
      "search damaged the completed source trunk");
  require(!f.tech.route.route_stage_deadline_expired && !f.tech.route.route_stage_stagnated,
      "test hit a deadline instead of finishing search");
  if(mode==2) {require(!failure.empty(),"disconnected control unexpectedly routed");return;}
  require(failure.empty(),"BUG: free complete witness with internal LOCAL is not routable: "+failure);
  require(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),"fanout incomplete");
  if(mode==0 || mode==3) {
    require(f.tile({32,2}).cb.local.local.testBit(165),"bridge LOCAL was not leased");
    bool entry=false,exit=false;
    size_t hops=0;
    for(const auto &w:f.sink->wires[0]) {
      const bool moved=w.from.x!=w.to.x || w.from.y!=w.to.y;
      hops+=(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) ||
          (w.type==Wire::WIRE_ROUTE_EDGE && moved);
      if(w.type!=Wire::WIRE_ROUTE_EDGE || w.from.x!=32 || w.from.y!=2 || moved) continue;
      entry|=w.from_node_type==fpga::CB_NODE_DST && w.from_node==3077 &&
             w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==165;
      exit|=w.from_node_type==fpga::CB_NODE_LOCAL && w.from_node==165 &&
            w.to_node_type==fpga::CB_NODE_SRC && w.to_node==3376;
    }
    require(entry && exit && hops==(mode==3?13U:10U),
        "numeric completion did not preserve the bridge and preferred source prefix");
  }
  if(mode==3) require(!f.tile({63,6}).cb.src.jump.testBit(3536),
      "BUG: numeric fallback opened another Takeoff despite a routable existing branch");
  require(f.source->coord==Coord{63,6} && f.sink->coord==Coord{36,29},"fixed endpoints moved");
  require(fpga::unrouteNetRoute(*f.net,1),"new route teardown failed");
  f.audit();
  require(!f.tile({32,2}).cb.local.local.testBit(165) && fpga::isRouteComplete(f.first->wires[0]),
      "numeric route teardown leaked bridge or damaged trunk");
  std::puts("free witness routed; original trunk preserved; ownership clean");
}
