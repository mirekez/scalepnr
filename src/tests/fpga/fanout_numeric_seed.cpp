#include "Tech.h"
#include "Wire.h"

#include <algorithm>
#include <cstdio>
#include <list>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
void require(bool ok,const std::string &message) {
  if(!ok) throw std::runtime_error(message);
}

// A numeric repair produces PIN, ROUTE_EDGE..., PIN, with no CROSSBAR
// fragments. At the next incremental run this is still a complete trunk.
// The old Generic/Fanout barrier counts only CROSSBARs and loses that seed.
// Exercise the actual numeric committer, then the full top-level scheduler.
// All node names are abstract; every occupied node has a registered owner.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  std::list<Referable<pnr::RegBunch>> bunches;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*first,*second;
  rtl::Net *net;
  rtl::Net *other;
  bool local_seed;
  bool edge_seed;
  int output_local;
  explicit Fixture(bool local=false,bool edge_case=false):local_seed(local),edge_seed(edge_case),
      output_local(local||edge_case?163:300) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=5; device.size_height=3; device.grid_spec.size={5,3};
    device.tile_grid.resize(15); device.tile_types.reserve(15); device.cb_types.reserve(15);
    for(int y=0;y<3;++y) for(int x=0;x<5;++x) {
      auto &t=*device.getTile(x,y); auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*5+x; cb.name="matrix_"+std::to_string(cb.type_id);
      auto &type=device.tile_types.emplace_back(); type.name="resource_"+std::to_string(cb.type_id);
      type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(output_local);
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(80);
      type.pin_map.nodes[{"LUT6","I0",1}].setBit(1);
      t.coord=t.cb_coord={x,y}; t.name=t.coord; t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      cb.local_output_nodes.setBit(output_local); cb.local_input_nodes.setBit(80); cb.local_input_nodes.setBit(1);
      name(t,fpga::CB_NODE_LOCAL,output_local); name(t,fpga::CB_NODE_LOCAL,80); name(t,fpga::CB_NODE_LOCAL,1);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="numeric_seed_top";
    parent->is_blackbox=false; parent->nets.reserve(32);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("driver_1088",{0,1});
    first=cell("sink_5881",local?Coord{0,1}:(edge_seed?Coord{1,1}:Coord{3,1}),local?1:0);
    second=cell("sink_5882",{4,1});
    net=&parent->nets.emplace_back(); net->name="signal_1088"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1;
    for(auto *sink:{first,second}) {
      sink->conns[0].port_ref->designator=1; sink->conns[0].set(&source->conns[1]);
      auto &b=bunches.emplace_back(); b.reg=sink; b.size=b.size_comb=1;
    }
    // As in run 5, unrelated Basic work precedes the Fanouts handoff. An
    // otherwise empty Basic queue exercises a different scheduler case.
    auto *other_source=cell("driver_2000",{0,0});
    auto *other_sink=cell("sink_2001",{1,0});
    other=&parent->nets.emplace_back(); other->name="signal_2000"; other->designators.push_back(2);
    other_source->conns[1].port_ref->designator=other_sink->conns[0].port_ref->designator=2;
    other_sink->conns[0].set(&other_source->conns[1]);
    auto &work=bunches.emplace_front(); work.reg=other_sink; work.size=work.size_comb=1;
    edge({0,0},{1,0},output_local,104,204,true);
    device.getTile(1,0)->cb_type->dst_local[204].local.setBit(80);
    edge({0,1},{1,1},output_local,100,200,true);
    edge({1,1},{2,1},200,101,201,false);
    edge({2,1},{3,1},201,102,202,false);
    edge({2,1},{4,1},201,103,203,false);
    if(local_seed || edge_seed) {
      // Exact intra-CB shape of the repaired real trunk, translated from
      // (54,29) to (0,1). Only names and unrelated surrounding nets differ.
      auto &t=*device.getTile(0,1); auto &cb=*t.cb_type;
      for(int j:{22,8,23}) name(t,fpga::CB_NODE_JOINT,j);
      cb.local_joint[163].joint.setBit(22);
      cb.joint_joint[22].joint.setBit(8);
      cb.joint_joint[8].joint.setBit(23);
      if(edge_seed) {
        cb.local_src[163].jump.clearBit(100);
        cb.joint_src[23].jump.setBit(100);
        device.getTile(1,1)->cb_type->dst_local[200].local.setBit(80);
        edge({1,1},{4,1},200,105,203,false);
      } else cb.joint_local[23].local.setBit(1);
    }
    for(auto [c,dst]:{std::pair{Coord{3,1},202},std::pair{Coord{4,1},203}}) {
      auto &t=*device.getTile(c.x,c.y); t.cb_type->dst_local[dst].local.setBit(80);
      t.cb_type->rememberConnName(fpga::CB_NODE_DST,dst,fpga::CB_NODE_LOCAL,80,
          *t.cb_type->nodeName(fpga::CB_NODE_DST,dst),*t.cb_type->nodeName(fpga::CB_NODE_LOCAL,80));
    }
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+std::to_string(unsigned(id)*2654435761U+unsigned(kind)));
  }
  void edge(Coord a,Coord b,int local,int src,int dst,bool output) {
    auto &t=*device.getTile(a.x,a.y); auto &to=*device.getTile(b.x,b.y); auto &cb=*t.cb_type;
    const auto kind=output?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    name(t,kind,local); name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(output) cb.local_src[local].jump.setBit(src); else cb.dst_src[local].jump.setBit(src);
    cb.rememberConnName(kind,local,fpga::CB_NODE_SRC,src,*cb.nodeName(kind,local),*cb.nodeName(fpga::CB_NODE_SRC,src));
    fpga::CBType::ResolvedJump j{}; j.delta=b-a; j.target_tile_coord=true;
    j.target_cb_type_id=to.cb_type->type_id; j.dsts.jump.setBit(dst); cb.dst_by_src[src].push_back(j);
  }
  rtl::Inst *cell(const std::string &name,Coord c,int pos=0) {
    auto &type=cells.emplace_back(); type.name=name; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction; p.designator=-1;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O";
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=pos; i.coord=c; i.outline.fixed=true;
    i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    device.getTile(c.x,c.y)->assign(&i); return &i;
  }
  Wire pin(rtl::Inst &inst,bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=inst.coord;
    w.local=output?output_local:(inst.pos==1?1:80); w.pos=inst.pos; w.cell_type="LUT6"; w.port=output?"O":"I0";
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT; return w;
  }
  void seed(bool legacy) {
    const auto route_name=first->conns[0].makeNetName(nullptr,1000000);
    if(legacy) {
      pnr::RouteDesign::RouteTask task{.from=source,.to=first,.net=net,.from_port="O",.to_port="I0",
          .net_name=route_name,.endpoints_prepared=true};
      tech.route.routeTaskOrFail(task,"Basic routing",200);
    } else {
      rtl::NetRouteBinding endpoints; endpoints.from=source; endpoints.to=first;
      endpoints.from_port="O"; endpoints.to_port="I0"; endpoints.route_name=route_name;
      require(fpga::routeUnboundNetFromSource(*net,endpoints,pin(*source,true),pin(*first,false)),
          "could not produce the numeric seed through the actual committer");
    }
    require(first->wires.size()==1 && fpga::isRouteComplete(first->wires.front()),"seed not complete");
    const auto &r=first->wires.front();
    if(edge_seed) {
      require(std::any_of(r.begin(),r.end(),[](const Wire &w){return w.type==Wire::WIRE_ROUTE_EDGE &&
          w.from_node_type==fpga::CB_NODE_SRC && w.from_node==100 &&
          w.to_node_type==fpga::CB_NODE_DST && w.to_node==200;}),
          "three-JOINT takeoff did not produce a numeric arrival");
    } else require(std::any_of(r.begin(),r.end(),[](const Wire &w){return w.type==Wire::WIRE_CROSSBAR;})==legacy,
        "seed representation does not distinguish numeric repair from legacy routing");
    tech.route.indexSourceRoute(net,source,"O");
    audit();
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream out; const auto a=fpga::auditTileCongestion(t,out,{net,other});
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
          !a.stale_registrations&&!a.conflicting_driver_nodes,out.str());
    }
  }
};
}

void fanout_numeric_seed_regression(int mode) {
  debug_level=1; Fixture f(mode!=1 && mode!=4,mode==4); f.seed(mode==1);
  if(mode!=1 && mode!=4) require(f.first->wires.front().size()==6,"recorded four-edge local trunk was not recreated");
  if(mode!=3) require(f.tech.route.sourceTreeHasCompleteExit(*f.source,"O"),
      "BUG: a complete numeric trunk is rejected because it has no CROSSBAR fragments");
  if(mode==2) {
    f.device.getTile(2,1)->cb_type->dst_src[201].jump.clearBit(103);
    f.device.getTile(2,1)->cb_type->dst_by_src[103].clear();
    f.device.rebuildIncomingDstMasks();
  }
  std::string failure;
  try {f.tech.route.routeDesign(f.bunches);}
  catch(const std::runtime_error &e) {failure=e.what();}
  f.audit();
  require(fpga::isRouteComplete(f.first->wires.front()),"existing numeric trunk was destroyed");
  if(mode==2) {
    require(!failure.empty() && failure.find("missing Generic trunk")==std::string::npos,
        "disconnected continuation was not rejected by actual route search");
    require(f.second->wires.empty() || f.second->wires.front().empty(),"failed suffix retained speculative routing");
    return;
  }
  require(failure.empty(),"numeric trunk cannot seed the next run: "+failure);
  require(f.second->wires.size()==1 && fpga::isRouteComplete(f.second->wires.front()),"new fanout incomplete");
  if(mode==4) {
    bool indexed=false;
    for(const auto &[key,index]:f.tech.route.fanout_branch_indexes)
      for(const auto &branch:index.branches)
        indexed|=branch.tile==f.device.getTile(1,1) && branch.dst==200;
    require(indexed,"numeric arrival was skipped; whole-source fallback concealed a broken branch index");
  }
  const auto &r=f.second->wires.front();
  require(std::any_of(r.begin(),r.end(),[&](const Wire &w){return w.type==Wire::WIRE_CROSSBAR &&
      w.from.x==(mode==4?1:2) && w.from.y==1 && (mode==4?w.jump>=0:w.jump==103);}),
      "new fanout did not branch through the ordinary downstream index");
  require(f.source->coord==Coord{0,1} && f.first->coord==(mode==4?Coord{1,1}:(mode!=1?Coord{0,1}:Coord{3,1})) &&
      f.second->coord==Coord{4,1},
      "fixed endpoints moved");
  std::puts("numeric seed recognized; downstream fanout complete; all node owners valid");
}
