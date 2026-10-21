#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <list>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
void require(bool ok,const std::string &message) {
  if(!ok) throw std::runtime_error(message);
}

// Example 1 run 6: template-only LOCALs consumed the 32768-state frontier.
// Import its free arcs and independent 20-hop witness. The companion file
// identifies actual LOCALs with no intra-CB arcs and only unresolvable cross-
// Tile templates. Normalize their unmatched target types to abstract base 1;
// every traversable CB has base 0. No occupied bits or artificial load.
// Source/sink resource Tiles are colocated with CBs in this routing fixture.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*sink;
  rtl::Net *net;
  explicit Fixture(bool templates=true) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=68; device.size_height=29; device.grid_spec.size={68,29};
    device.tile_grid.resize(68*29);
    device.tile_types.reserve(68*29); device.cb_types.reserve(68*29);
    for(int y=0;y<29;++y) for(int x=0;x<68;++x) device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="budget_top";
    parent->is_blackbox=false; parent->nets.reserve(8);
    primitive=&tech.design.modules.emplace_back(); primitive->name="abstract_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_1701",{53,20},true); sink=cell("sink_17309",{15,7},false);
    net=&parent->nets.emplace_back(); net->name="signal_1701"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1; sink->conns[0].port_ref->designator=1;
    sink->conns[0].set(&source->conns[1]);
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename("basic_local_templates_frontier.txt"));
    require(bool(in),"missing recorded free-edge snapshot");
    int x,y,kind,id,nx,ny,nkind,nid; size_t arcs=0;
    while(in>>x>>y>>kind>>id>>nx>>ny>>nkind>>nid) {
      auto &from=tile({x,y}); auto &to=tile({nx,ny}); auto &cb=*from.cb_type;
      name(from,kind,id); name(to,nkind,nid);
      if(kind==fpga::CB_NODE_SRC && nkind==fpga::CB_NODE_DST) {
        fpga::CBType::ResolvedJump jump{}; jump.delta=Coord{nx,ny}-Coord{x,y};
        jump.target_tile_coord=true; jump.target_cb_type_id=0;
        jump.dsts.jump.setBit(nid); cb.dst_by_src[id].push_back(jump);
      } else if(kind==fpga::CB_NODE_LOCAL && (x!=nx || y!=ny)) {
        fpga::CBType::ResolvedLocal local{}; local.delta=Coord{nx,ny}-Coord{x,y};
        local.target_cb_type_id=0;
        local.target_node_type=static_cast<fpga::CBNodeNameType>(nkind);
        local.target_nodes.setBit(nid); cb.local_by_local[id].push_back(local);
      } else {
        require(x==nx && y==ny,"unexpected non-jump cross-Tile arc");
        if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_SRC) cb.local_src[id].jump.setBit(nid);
        else if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_LOCAL) cb.local_local[id].local.setBit(nid);
        else if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_JOINT) cb.local_joint[id].joint.setBit(nid);
        else if(kind==fpga::CB_NODE_DST && nkind==fpga::CB_NODE_SRC) cb.dst_src[id].jump.setBit(nid);
        else if(kind==fpga::CB_NODE_DST && nkind==fpga::CB_NODE_LOCAL) cb.dst_local[id].local.setBit(nid);
        else if(kind==fpga::CB_NODE_DST && nkind==fpga::CB_NODE_JOINT) cb.dst_joint[id].joint.setBit(nid);
        else if(kind==fpga::CB_NODE_SRC && nkind==fpga::CB_NODE_JOINT) cb.src_joint[id].joint.setBit(nid);
        else if(kind==fpga::CB_NODE_JOINT && nkind==fpga::CB_NODE_SRC) cb.joint_src[id].jump.setBit(nid);
        else if(kind==fpga::CB_NODE_JOINT && nkind==fpga::CB_NODE_LOCAL) cb.joint_local[id].local.setBit(nid);
        else if(kind==fpga::CB_NODE_JOINT && nkind==fpga::CB_NODE_JOINT) cb.joint_joint[id].joint.setBit(nid);
        else require(false,"unexpected snapshot arc kind");
        cb.rememberConnName(static_cast<fpga::CBNodeNameType>(kind),id,
            static_cast<fpga::CBNodeNameType>(nkind),nid,
            *cb.nodeName(static_cast<fpga::CBNodeNameType>(kind),id),
            *cb.nodeName(static_cast<fpga::CBNodeNameType>(nkind),nid));
      }
      ++arcs;
    }
    require(in.eof() && arcs==77720,"incomplete free-edge snapshot");
    if(templates) {
      std::ifstream leaves(std::filesystem::path(__FILE__).replace_filename("basic_local_templates_unresolved.txt"));
      require(bool(leaves),"missing unresolved-template snapshot");
      int lx,ly,id,count; size_t n=0;
      while(leaves>>lx>>ly>>id>>count) {
        auto &cb=*tile({lx,ly}).cb_type;
        require(cb.local_src[id].jump.firstSetBit()<0 &&
            cb.local_joint[id].joint.firstSetBit()<0 && cb.local_local[id].local.firstSetBit()<0,
            "recorded unresolved LOCAL acquired an intra-CB continuation");
        for(int i=0;i<count;++i) {
          fpga::CBType::ResolvedLocal link{};
          link.delta={-1,0}; link.target_cb_type_id=1;
          link.target_node_type=fpga::CB_NODE_LOCAL; link.target_nodes.setBit(id);
          cb.local_by_local[id].push_back(link);
        }
        require(device.resolveLocalTargets(tile({lx,ly}),id).empty(),"template resolved unexpectedly");
        ++n;
      }
      require(leaves.eof() && n==10790,"incomplete unresolved-template snapshot");
    }
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  fpga::Tile &tile(Coord c) {
    auto &t=*device.getTile(c.x,c.y); if(t.cb_type) return t;
    auto &cb=device.cb_types.emplace_back(); cb.type_id=device.cb_types.size()-1; cb.base_type_id=0;
    cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=device.tile_types.emplace_back(); type.name="resource_"+std::to_string(cb.type_id);
    type.type=fpga::Tile::TILE_LUTS; type.pin_map.output_nodes[1].setBit(145);
    type.pin_map.nodes[{"LUT6","I2",0}].setBit(97);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(145); cb.local_input_nodes.setBit(97);
    name(t,fpga::CB_NODE_LOCAL,145); name(t,fpga::CB_NODE_LOCAL,97); return t;
  }
  static void name(fpga::Tile &t,int kind,int id) {
    t.cb_type->rememberNodeName(static_cast<fpga::CBNodeNameType>(kind),id,
        "node_"+std::to_string(unsigned(id)*2654435761U+unsigned(kind)));
  }
  rtl::Inst *cell(const std::string &name,Coord c,bool output) {
    auto &type=cells.emplace_back(); type.name=name; type.type=output?"FDRE":"LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction; p.designator=-1;
      p.name=direction==rtl::Port::PORT_IN?(output?"D":"I2"):(output?"Q":"O");
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c; i.outline.fixed=true;
    i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    tile(c).assign(&i); return &i;
  }
  Wire pin(bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=output?Coord{53,20}:Coord{15,7};
    w.local=output?145:97; w.pos=0;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"FDRE":"LUT6"; w.port=output?"Q":"I2"; return w;
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

void basic_local_templates_regression(int mode) {
  debug_level=1; Fixture f(mode!=3);
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I2";
  endpoints.route_name="connection_1701";
  if(mode==2) f.tile({15,7}).cb_type->dst_local[1024].local.clearBit(97);
  std::ostringstream reach;
  require(fpga::auditUnroutedReachability(*f.net,endpoints,f.pin(true),f.pin(false),reach)==(mode!=2),
      reach.str());
  require(!fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),[]{return true;}),
      "cancelled search installed a route");
  require(f.net->routes.empty() && f.sink->wires.empty(),"diagnostic or cancelled search mutated bindings");
  f.audit();
  setenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY","signal_1701",1);
  setenv("SCALEPNR_SOURCE_REACHABILITY_LOG","source_budget.log",1);
  { std::ofstream clear("source_budget.log",std::ios::trunc); }
  const bool done=fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),{},
      mode==1?65536:32768,200);
  f.audit();
  if(mode==2) {
    require(!done && f.net->routes.empty() && f.sink->wires.empty(),"disconnected snapshot routed or leaked state");
    return;
  }
  require(done,"BUG: free 20-hop witness lost to numeric state-budget exhaustion");
  require(f.net->routes.size()==1 && f.sink->wires.size()==1 &&
      fpga::isRouteComplete(f.sink->wires.front()),"numeric search did not install exactly one complete route");
  int hops=0; bool first_bridge=false, second_bridge=false;
  for(const auto &w:f.sink->wires.front()) {
    hops+=(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) ||
        (w.type==Wire::WIRE_ROUTE_EDGE && (w.from.x!=w.to.x || w.from.y!=w.to.y));
    first_bridge|=w.type==Wire::WIRE_ROUTE_EDGE && w.from.x==53 && w.from.y==8 &&
        w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==196;
    second_bridge|=w.type==Wire::WIRE_ROUTE_EDGE && w.from.x==40 && w.from.y==2 &&
        w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==165;
  }
  // The independent BFS witness is not the only solution. Numeric routing
  // finds a shorter 19-hop alternative in this reduced graph.
  require(hops>0 && hops<=20,"completion exceeded the free witness hop count");
  require(first_bridge,"completion lost the necessary internal LOCAL bridge");
  require(f.source->coord==Coord{53,20} && f.sink->coord==Coord{15,7},"fixed endpoints moved");
  require(fpga::unrouteNetRoute(*f.net,0),"snapshot teardown failed"); f.audit();
  std::printf("recorded frontier: complete %d-hop free path; LOCAL bridges=%d,%d\n",
      hops,first_bridge,second_bridge);
}
