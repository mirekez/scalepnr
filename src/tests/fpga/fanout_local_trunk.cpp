#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <list>
#include <sstream>
#include <stdexcept>

namespace {
void require(bool ok,const std::string &message) {
  if(!ok) throw std::runtime_error(message);
}

// Example 1: the only complete seed is three fragments, LOCAL144 ->
// LOCAL127 inside one CB, jump=-1. The next sink is in another Tile.
// Fanouts incorrectly required a jump on its source fragment and generated
// zero attempts. Translate (63,69)->(58,61) by (-55,-60); keep the local
// node IDs/three-fragment representation, but use abstract names/topology.
// Unrelated congestion and RTL aliases are omitted: the bug occurs before
// ANY occupancy or path search. Every lease here has a real registered owner.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*local_sink,*remote_sink;
  rtl::Net *signal;

  explicit Fixture(bool blocked) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=10; device.size_height=11; device.grid_spec.size={10,11};
    device.tile_grid.resize(110); device.tile_types.reserve(110); device.cb_types.reserve(110);
    for(int y=0;y<11;++y) for(int x=0;x<10;++x) {
      auto &t=*device.getTile(x,y); auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*10+x; cb.name="matrix_"+std::to_string(y*10+x);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(y*10+x); type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(144);
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(107);
      type.pin_map.nodes[{"LUT6","I0",1}].setBit(127);
      t.coord=t.cb_coord={x,y}; t.name=t.coord; t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      cb.local_output_nodes.setBit(144); cb.local_input_nodes.setBit(107); cb.local_input_nodes.setBit(127);
      for(int n:{107,127,144}) name(t,fpga::CB_NODE_LOCAL,n);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="local_seed_top";
    parent->is_blackbox=false; parent->nets.reserve(16);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("driver_701",{8,9},0);
    local_sink=cell("near_702",{8,9},1);
    remote_sink=cell("far_703",{3,1},0);
    auto &net=parent->nets.emplace_back(); net.name="signal_704"; net.designators.push_back(1); signal=&net;
    source->conns[1].port_ref->designator=1;
    for(auto *sink:{local_sink,remote_sink}) {
      sink->conns[0].port_ref->designator=1; sink->conns[0].set(&source->conns[1]);
    }
    device.getTile(8,9)->cb_type->local_local[144].local.setBit(127);
    // Like a normal CB input, the local sink also has an incoming DST arc;
    // it is not the path used by this local-only trunk.
    name(*device.getTile(8,9),fpga::CB_NODE_DST,90);
    device.getTile(8,9)->cb_type->dst_local[90].local.setBit(127);
    edge({7,9},{8,9},91,190,90,false);
    edge({8,9},{8,5},144,110,210,true);
    device.getTile(8,5)->cb_type->dst_local[210].local.setBit(107);
    if(!blocked) edge({8,5},{3,1},210,120,220,false);
    device.getTile(3,1)->cb_type->dst_local[220].local.setBit(107);
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    // Start from the captured completed trunk, as in the incremental run.
    // This tests Fanouts, not construction of tile-internal primitive paths.
    fpga::Wire a,b,c;
    a.type=c.type=fpga::Wire::WIRE_TILE_PIN;
    a.from=a.to=a.resource=b.from=b.to=c.from=c.to=c.resource={8,9};
    a.local=b.local=144; a.pos=b.pos=0; a.pin_dir=fpga::TILE_PIN_OUTPUT;
    a.cell_type="LUT6"; a.port="O";
    c.local=127; c.pos=1; c.pin_dir=fpga::TILE_PIN_INPUT; c.cell_type="LUT6"; c.port="I0";
    const auto route_name=local_sink->conns[0].makeNetName(nullptr,1000000);
    a.net_name=b.net_name=c.net_name=route_name;
    local_sink->wires.push_back({a,b,c});
    auto &tile=*device.getTile(8,9);
    for(int local:{144,127}) {tile.cb.local.local.setBit(local);tile.pin_state.leased_nodes.setBit(local);}
    const auto binding=fpga::attachNetRoute(*signal,*local_sink,0,source,local_sink,"O","I0",route_name);
    fpga::registerNetRouteTiles(*signal,local_sink->wires.front(),binding);
    audit();
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+std::to_string(unsigned(id)*2654435761U+unsigned(kind)));
  }
  void edge(fpga::Coord a,fpga::Coord b,int in,int src,int dst,bool takeoff) {
    auto &t=*device.getTile(a.x,a.y); auto &to=*device.getTile(b.x,b.y); auto &cb=*t.cb_type;
    name(t,takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,in);
    name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(takeoff) cb.local_src[in].jump.setBit(src); else cb.dst_src[in].jump.setBit(src);
    fpga::CBType::ResolvedJump j{}; j.delta=b-a; j.target_tile_coord=true;
    j.target_cb_type_id=to.cb_type->type_id; j.dsts.jump.setBit(dst); cb.dst_by_src[src].push_back(j);
  }
  rtl::Inst *cell(const std::string &name,fpga::Coord c,int pos) {
    auto &type=cells.emplace_back(); type.name=name; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type); i.parent_ref.set(&tech.design.top);
    i.depth=1; i.pos=pos; i.coord=c; i.outline.fixed=true; i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    device.getTile(c.x,c.y)->assign(&i); return &i;
  }
  std::vector<fpga::Wire> *route(rtl::Inst *sink) {
    for(auto &b:signal->routes) if(b.to==sink && b.owner && b.route_index<b.owner->wires.size())
      return &b.owner->wires[b.route_index];
    return nullptr;
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream out; auto a=fpga::auditTileCongestion(t,out,{signal});
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&!a.stale_registrations,out.str());
    }
  }
};
}

void fanout_local_trunk_regression(bool blocked) {
  debug_level=1; Fixture f(blocked); std::string failure;
  pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.remote_sink,.net=f.signal,
      .from_port="O",.to_port="I0",
      .net_name=f.remote_sink->conns[0].makeNetName(nullptr,1000000),
      .fanout=true,.endpoints_prepared=true};
  f.tech.route.fanout_stage=true;
  try {f.tech.route.routeTaskOrFail(task,"Fanouts routing",200);}
  catch(const std::runtime_error &e) {failure=e.what();}
  auto *local=f.route(f.local_sink), *remote=f.route(f.remote_sink);
  require(local && fpga::isRouteComplete(*local) && local->size()==3,
      "fixture did not produce the captured three-fragment local trunk");
  require((*local)[1].type==fpga::Wire::WIRE_CROSSBAR && (*local)[1].pos==0 &&
      (*local)[1].local==144 && (*local)[1].jump==-1 && local->back().local==127,
      "local seed shape differs from the real failure");
  f.audit();
  if(blocked) {
    require(!failure.empty() && (!remote || remote->empty()),"removed continuation unexpectedly routed");
    require(!f.device.getTile(8,5)->cb.src.jump.testBit(120),"failed search leaked a route lease");
    return;
  }
  if(!failure.empty()) {
    require(failure=="routing failed: Fanouts routing: current connection did not finish; no task/stage handoff",
        "unexpected regression failure: "+failure);
    require(!remote || remote->empty(),"failed fanout left partial routing");
    throw std::runtime_error("BUG: complete local-only trunk provides no Fanouts source branch");
  }
  require(remote && fpga::isRouteComplete(*remote),"remote fanout incomplete");
  require(f.device.getTile(8,9)->cb.src.jump.testBit(110) && f.device.getTile(8,5)->cb.src.jump.testBit(120),
      "fanout did not use the two-hop source exit");
  require(f.source->tile.peer==f.device.getTile(8,9) && f.local_sink->pos==1 &&
      f.remote_sink->tile.peer==f.device.getTile(3,1),"fixed endpoint moved");
  std::puts("local trunk preserved, remote fanout complete, all ownership audits clean");
}
