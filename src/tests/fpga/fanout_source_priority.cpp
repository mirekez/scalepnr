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

// Run-5 repair snapshot: the free westward jump of SRC3073 at (136,84)
// was queued as state 55 but starved until the state budget was exhausted.
// Import eligible free edges, the complete free witness, and the retained
// binding. Unrelated owner routes are omitted; names are abstract. Removing
// their topology lets leaf pruning shrink the search: this reduced snapshot
// reproduces starvation at 16K, with a 32K positive control. The real failure
// exhausted 32K. Neither test changes the production search limit.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*sink;
  rtl::Net *net;
  Fixture() {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=144; device.size_height=103; device.grid_spec.size={144,103};
    device.tile_grid.resize(144*103);
    device.tile_types.reserve(144*103); device.cb_types.reserve(144*103);
    for(int y=0;y<103;++y) for(int x=0;x<144;++x) device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="budget_top";
    parent->is_blackbox=false; parent->nets.reserve(8);
    primitive=&tech.design.modules.emplace_back(); primitive->name="abstract_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_163756",{136,96},true); sink=cell("sink_20208",{11,19},false);
    net=&parent->nets.emplace_back(); net->name="signal_163756"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1; sink->conns[0].port_ref->designator=1;
    sink->conns[0].set(&source->conns[1]);
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename("fanout_source_priority_frontier.txt"));
    require(bool(in),"missing recorded free-edge snapshot");
    int x,y,kind,id,nx,ny,nkind,nid; size_t arcs=0;
    while(in>>x>>y>>kind>>id>>nx>>ny>>nkind>>nid) {
      auto &from=tile({x,y}); auto &to=tile({nx,ny}); auto &cb=*from.cb_type;
      name(from,kind,id); name(to,nkind,nid);
      if(kind==fpga::CB_NODE_SRC && nkind==fpga::CB_NODE_DST) {
        fpga::CBType::ResolvedJump jump{}; jump.delta=Coord{nx,ny}-Coord{x,y};
        jump.target_tile_coord=true; jump.target_cb_type_id=to.cb_type->type_id;
        jump.dsts.jump.setBit(nid); cb.dst_by_src[id].push_back(jump);
      } else if(kind==fpga::CB_NODE_LOCAL && (x!=nx || y!=ny)) {
        fpga::CBType::ResolvedLocal local{}; local.delta=Coord{nx,ny}-Coord{x,y};
        local.target_cb_type_id=to.cb_type->type_id;
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
    require(in.eof() && arcs==62258,"incomplete free-edge snapshot");
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  fpga::Tile &tile(Coord c) {
    auto &t=*device.getTile(c.x,c.y); if(t.cb_type) return t;
    auto &cb=device.cb_types.emplace_back(); cb.type_id=cb.base_type_id=device.cb_types.size()-1;
    cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=device.tile_types.emplace_back(); type.name="resource_"+std::to_string(cb.type_id);
    type.type=fpga::Tile::TILE_LUTS; type.pin_map.output_nodes[1].setBit(150);
    type.pin_map.nodes[{"LUT6","I4",0}].setBit(119);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(150); cb.local_input_nodes.setBit(119);
    name(t,fpga::CB_NODE_LOCAL,150); name(t,fpga::CB_NODE_LOCAL,119); return t;
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
      p.name=direction==rtl::Port::PORT_IN?(output?"D":"I4"):(output?"Q":"O");
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c; i.outline.fixed=true;
    i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    tile(c).assign(&i); return &i;
  }
  Wire pin(bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=output?Coord{136,96}:Coord{11,19};
    w.local=output?150:119; w.pos=0;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"FDRE":"LUT6"; w.port=output?"Q":"I4"; return w;
  }
  void install() {
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename("fanout_source_priority_route.txt"));
    require(bool(in),"missing retained route snapshot");
    std::vector<Wire> route;
    int type,x,y,tx,ty,local,src,dst,joint,joint2,pos,fkind,fid,tkind,tid;
    while(in>>type>>x>>y>>tx>>ty>>local>>src>>dst>>joint>>joint2>>pos>>fkind>>fid>>tkind>>tid) {
      Wire w; w.type=static_cast<decltype(w.type)>(type); w.from={x,y}; w.to={tx,ty};
      w.local=local; w.jump=src; w.dst=dst; w.joint=joint; w.joint2=joint2;
      w.pos=pos==2?1:pos; w.owns_dst=true; w.shared=false;
      w.from_node_type=static_cast<fpga::CBNodeNameType>(fkind); w.from_node=fid;
      w.to_node_type=static_cast<fpga::CBNodeNameType>(tkind); w.to_node=tid;
      auto &at=tile(w.from); auto &cb=*at.cb_type;
      if(w.type==Wire::WIRE_TILE_PIN) {
        w=pin(route.empty());
        if(w.pin_dir==fpga::TILE_PIN_INPUT) {
          at.cb.local.local.setBit(w.local);
          at.pin_state.leased_nodes.setBit(w.local);
        }
      } else if(w.type==Wire::WIRE_CROSSBAR) {
        name(at,w.pos==0?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,local);
        if(w.pos!=0) at.cb.dst.jump.setBit(local);
        if(src>=0) {
          require(joint<0 && joint2<0,"unexpected joined transit in saved binding");
          name(at,fpga::CB_NODE_SRC,src); name(tile(w.to),fpga::CB_NODE_DST,dst);
          if(w.pos==0) cb.local_src[local].jump.setBit(src);
          else cb.dst_src[local].jump.setBit(src);
          bool exists=false;
          for(const auto &j:cb.dst_by_src[src])
            exists|=j.delta.x==tx-x && j.delta.y==ty-y && j.dsts.jump.testBit(dst);
          if(!exists) {
            fpga::CBType::ResolvedJump j{}; j.delta={tx-x,ty-y}; j.target_tile_coord=true;
            j.target_cb_type_id=tile(w.to).cb_type->type_id; j.dsts.jump.setBit(dst);
            cb.dst_by_src[src].push_back(j);
          }
          at.cb.src.jump.setBit(src); tile(w.to).cb.dst.jump.setBit(dst);
          w.owns_landing=true;
        } else {
          require(joint==5 && joint2==19,"unexpected saved terminal chain");
          name(at,fpga::CB_NODE_JOINT,joint); name(at,fpga::CB_NODE_JOINT,joint2);
          cb.dst_joint[local].joint.setBit(joint2); cb.joint_joint[joint2].joint.setBit(joint);
          cb.joint_local[joint].local.setBit(119);
          at.cb.joint.jump.setBit(joint); at.cb.joint.jump.setBit(joint2);
        }
      } else require(w.type==Wire::WIRE_GAP,"unexpected saved fragment type");
      route.push_back(w);
    }
    require(in.eof() && route.size()==36,"incomplete retained binding");
    sink->wires.push_back(std::move(route));
    auto binding=fpga::attachNetRoute(*net,*sink,0,source,sink,"Q","I4","connection_163756");
    fpga::registerNetRouteTiles(*net,sink->wires.front(),binding);
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
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

void fanout_source_priority_regression(int mode) {
  debug_level=1; Fixture f; f.install(); f.audit();
  if(mode==2) f.tile({11,19}).cb_type->joint_local[5].local.clearBit(119);
  std::ostringstream reach;
  require(fpga::auditNetRouteReachability(*f.net,0,reach)==(mode!=2),reach.str());
  std::ostringstream before; fpga::dumpNetRouteHistory(*f.net,before);
  require(!fpga::repairNetRouteGaps(*f.net,0,[]{return true;}),"cancelled repair succeeded");
  std::ostringstream cancelled; fpga::dumpNetRouteHistory(*f.net,cancelled);
  require(cancelled.str()==before.str(),"cancelled repair changed binding");
  setenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY","signal_163756",1);
  setenv("SCALEPNR_SOURCE_REACHABILITY_LOG","source_priority.log",1);
  {std::ofstream clear("source_priority.log",std::ios::trunc);}
  const bool done=fpga::repairNetRouteGaps(*f.net,0,{},mode==1?32768:16384,200);
  f.audit();
  if(mode==2) {
    std::ostringstream after; fpga::dumpNetRouteHistory(*f.net,after);
    require(!done && after.str()==before.str(),"broken terminal repair changed retained route");
    return;
  }
  require(done,"BUG: queued long jump starved before reaching the retained suffix");
  require(f.net->routes.size()==1 && fpga::isRouteComplete(f.sink->wires.front()),
      "repair did not finish the recorded binding");
  require(f.source->coord==Coord{136,96} && f.sink->coord==Coord{11,19},"fixed endpoints moved");
  int hops=0;
  for(const auto &w:f.sink->wires.front())
    hops+=(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) ||
        (w.type==Wire::WIRE_ROUTE_EDGE && (w.from.x!=w.to.x || w.from.y!=w.to.y));
  require(hops>0 && hops<=200,"repair bypassed the hop bound");
  require(fpga::unrouteNetRoute(*f.net,0),"repaired binding teardown failed"); f.audit();
  std::printf("recorded source-priority frontier: repaired %d-hop path; ownership verified\n",hops);
}
