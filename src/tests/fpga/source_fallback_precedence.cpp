#include "Tech.h"
#include "Wire.h"

#include <list>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
using Route=std::vector<Wire>;
void check(bool ok,const std::string &message) {if(!ok)throw std::runtime_error(message);}
struct Node {Coord tile;int kind,id;};

// Two feasible terminal approaches reproduce the changed-choice regression.
// Original priority reaches a one-victim approach first; weaker geometry
// reaches a cheaper but two-victim approach first. An alternate fallback must
// not replace the already successful original choice. All labels are abstract.
struct Fixture {
  technology::Tech tech;
  fpga::Device &d=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*sink,*trunk_sink,*a_source,*a_sink,*b_source,*b_sink,*c_source,*c_sink;
  rtl::Net *main,*a,*b,*c;
  Fixture() {
    d.tile_grid.clear();d.tile_types.clear();d.cb_types.clear();
    d.route_wire_graph.clear();d.local_route_wire_mappings.clear();
    d.size_width=14;d.size_height=6;d.grid_spec.size={14,6};d.tile_grid.resize(14*6);
    d.tile_types.reserve(64);d.cb_types.reserve(64);
    for(int y=0;y<6;++y)for(int x=0;x<14;++x)d.getTile(x,y)->coord={x,y};
    tech.init();tech.design.GND=nullptr;tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back();parent->name="top";parent->is_blackbox=false;
    parent->nets.reserve(4);primitive=&tech.design.modules.emplace_back();
    primitive->name="logic";primitive->is_blackbox=true;primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top";tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("driver",{0,0},100,true);sink=cell("receiver",{12,0},200,false);
    trunk_sink=cell("existing_receiver",{0,1},101,false);
    a_source=cell("near_driver",{10,0},300,true);a_sink=cell("near_receiver",{13,0},301,false);
    b_source=cell("far_driver",{4,4},300,true);b_sink=cell("far_receiver",{13,1},301,false);
    c_source=cell("bridge_driver",{12,1},300,true);c_sink=cell("bridge_receiver",{13,2},301,false);
    main=net("preferred_signal",1,source,{trunk_sink,sink});
    a=net("near_transit",2,a_source,{a_sink});b=net("far_transit",3,b_source,{b_sink});
    c=net("bridge_transit",4,c_source,{c_sink});
    Route trunk{pin({0,0},100,true),hop({0,0},100,true,{1,0},0,0),hop({1,0},0,false,{0,1},99,99)};
    terminal(trunk,{0,1},99,101);install(*main,source,trunk_sink,std::move(trunk));
    Route ar{pin({10,0},300,true),hop({10,0},300,true,{11,0},90,90),
             hop({11,0},90,false,{12,0},10,10),hop({12,0},10,false,{13,0},11,11)};
    terminal(ar,{13,0},11,301);install(*a,a_source,a_sink,std::move(ar));
    Route br{pin({4,4},300,true),hop({4,4},300,true,{5,4},90,90),
             hop({5,4},90,false,{12,0},20,20),hop({12,0},20,false,{13,1},21,21)};
    terminal(br,{13,1},21,301);install(*b,b_source,b_sink,std::move(br));
    Route cr{pin({12,1},300,true),hop({12,1},300,true,{12,0},90,90),
             hop({12,0},90,false,{13,2},30,30,7)};
    terminal(cr,{13,2},30,301);install(*c,c_source,c_sink,std::move(cr));
    hop({1,0},0,false,{11,1},1,1);hop({1,0},0,false,{5,5},2,2);
    chain({11,1},1,1,8,3,{11,0},3);chain({5,5},2,10,2,4,{5,4},4);
    arc({{11,0},4,3},{{11,0},3,10});arc({{12,0},4,10},{{12,0},0,200});
    arc({{5,4},4,4},{{5,4},3,20});arc({{12,0},4,20},{{12,0},1,7});
    arc({{12,0},1,7},{{12,0},0,200});
    // Private repair outlet for the preferred victim's retained boundaries.
    hop({11,0},90,false,{13,0},40,11);
    for(auto &cb:d.cb_types)cb.rebuildOutgoingSrcs();
    d.rebuildIncomingDstMasks();audit();
  }
  fpga::Tile &tile(Coord p) {
    auto &t=*d.getTile(p.x,p.y);if(t.cb_type)return t;
    auto &cb=d.cb_types.emplace_back();cb.type_id=cb.base_type_id=d.cb_types.size()-1;
    cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=d.tile_types.emplace_back();type.name=cb.name;type.type=fpga::Tile::TILE_LUTS;
    t.coord=t.cb_coord=p;t.name=p;t.full_name="tile_"+std::to_string(p.x)+"_"+std::to_string(p.y);
    t.type=fpga::Tile::TILE_LUTS;t.tile_type=&type;t.cb_type=t.cb.type=&cb;return t;
  }
  void arc(Node from,Node to) {
    auto &at=tile(from.tile),&target=tile(to.tile);auto &cb=*at.cb_type;
    for(const auto &n:{from,to})tile(n.tile).cb_type->rememberNodeName(
        static_cast<fpga::CBNodeNameType>(n.kind),n.id,"node_"+std::to_string(n.kind)+"_"+std::to_string(n.id));
    if(from.kind==3 && to.kind==4) {
      fpga::CBType::ResolvedJump j{};j.delta=to.tile-from.tile;j.target_tile_coord=true;
      j.target_cb_type_id=target.cb_type->type_id;j.dsts.jump.setBit(to.id);cb.dst_by_src[from.id].push_back(j);
    } else {
      check(from.tile.x==to.tile.x && from.tile.y==to.tile.y,"invalid intra-Tile arc");
      if(from.kind==0 && to.kind==3)cb.local_src[from.id].jump.setBit(to.id);
      else if(from.kind==4 && to.kind==3)cb.dst_src[from.id].jump.setBit(to.id);
      else if(from.kind==4 && to.kind==1)cb.dst_joint[from.id].joint.setBit(to.id);
      else if(from.kind==4 && to.kind==0)cb.dst_local[from.id].local.setBit(to.id);
      else if(from.kind==1 && to.kind==1)cb.joint_joint[from.id].joint.setBit(to.id);
      else if(from.kind==1 && to.kind==3)cb.joint_src[from.id].jump.setBit(to.id);
      else if(from.kind==1 && to.kind==0)cb.joint_local[from.id].local.setBit(to.id);
      else check(false,"unsupported fixture arc");
    }
  }
  void chain(Coord p,int dst,int first,int count,int src,Coord landing,int end) {
    Node previous{p,4,dst};
    for(int i=0;i<count;++i){Node next{p,1,first+i};arc(previous,next);previous=next;}
    arc(previous,{p,3,src});arc({p,3,src},{landing,4,end});
  }
  Wire pin(Coord p,int local,bool output) {
    Wire w;w.type=Wire::WIRE_TILE_PIN;w.from=w.to=w.resource=p;w.local=local;w.pos=0;
    w.cell_type="LUT6";w.port=output?"O":"I0";w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;return w;
  }
  Wire hop(Coord p,int input,bool local,Coord to,int src,int dst,int joint=-1) {
    Node start{p,local?0:4,input};
    if(joint>=0){arc(start,{p,1,joint});start={p,1,joint};}
    arc(start,{p,3,src});arc({p,3,src},{to,4,dst});
    Wire w;w.from=p;w.to=to;w.local=input;w.pos=local?0:1;w.jump=w.route_jump=src;
    w.dst=dst;w.joint=joint;w.owns_landing=true;return w;
  }
  void terminal(Route &route,Coord p,int dst,int local) {
    arc({p,4,dst},{p,0,local});Wire w;w.from=w.to=p;w.pos=1;w.local=dst;
    route.push_back(w);route.push_back(pin(p,local,false));
  }
  rtl::Inst *cell(const std::string &name,Coord p,int node,bool output) {
    auto &type=cells.emplace_back();type.name=name;type.type="LUT6";type.module_ref.set(primitive);type.ports.reserve(2);
    for(auto dir:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &port=type.ports.emplace_back();port.type=dir;port.name=dir==rtl::Port::PORT_IN?"I0":"O";port.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back();i.cell_ref.set(&type);i.parent_ref.set(&tech.design.top);
    i.depth=1;i.coord=p;i.pos=0;i.outline.fixed=true;i.cnt_inputs=i.cnt_outputs=1;i.conns.reserve(2);
    for(auto &port:type.ports){auto &conn=i.conns.emplace_back();conn.port_ref.set(&port);conn.inst_ref.set(&i);}
    auto &at=tile(p);at.tile_type->pin_map.nodes[{"LUT6",output?"O":"I0",0}].setBit(node);
    (output?at.cb_type->local_output_nodes:at.cb_type->local_input_nodes).setBit(node);at.assign(&i);return &i;
  }
  rtl::Net *net(const std::string &name,int id,rtl::Inst *from,std::initializer_list<rtl::Inst *> receivers) {
    auto &n=parent->nets.emplace_back();n.name=name;n.designators.push_back(id);from->conns[1].port_ref->designator=id;
    for(auto *to:receivers){to->conns[0].port_ref->designator=id;to->conns[0].set(&from->conns[1]);}return &n;
  }
  void install(rtl::Net &n,rtl::Inst *from,rtl::Inst *to,Route route) {
    const auto name=n.name+"_"+to->cell_ref->name;
    for(auto &w:route) {
      w.net_name=name;auto &at=tile(w.from);
      if(w.type==Wire::WIRE_CROSSBAR) {
        if(w.pos!=0)at.cb.dst.jump.setBit(w.local);
        if(w.jump>=0){at.cb.src.jump.setBit(w.jump);tile(w.to).cb.dst.jump.setBit(w.dst);}
        if(w.joint>=0)at.cb.joint.jump.setBit(w.joint);
      } else if(w.pin_dir==fpga::TILE_PIN_INPUT){at.cb.local.local.setBit(w.local);at.pin_state.leased_nodes.setBit(w.local);}
    }
    to->wires.push_back(std::move(route));auto index=to->wires.size()-1;
    auto binding=fpga::attachNetRoute(n,*to,index,from,to,"O","I0",name);fpga::registerNetRouteTiles(n,to->wires[index],binding);
  }
  void audit() {
    for(auto &at:d.tile_grid)if(at.cb_type) {
      std::ostringstream s;auto v=fpga::auditTileCongestion(at,s,{main,a,b,c});
      check(!v.orphan_leases&&!v.missing_leases&&!v.missing_registrations&&!v.stale_registrations&&!v.conflicting_driver_nodes,s.str());
    }
  }
};
}

void source_fallback_precedence_regression() {
  debug_level=1;Fixture f;std::set<rtl::Net *> cut_owners;
  rtl::NetRouteBinding endpoints;endpoints.from=f.source;endpoints.to=f.sink;
  endpoints.from_port="O";endpoints.to_port="I0";endpoints.route_name="new_receiver";
  auto preempt=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,std::vector<fpga::RouteCutNode> &released) {
    auto owners=fpga::findNetRouteCutVictims(cuts);if(owners.empty())return false;
    for(const auto &owner:owners) {
      if(owner.net==f.main || !fpga::canPreemptNetRouteFromNodes(*owner.net,owner.binding_index,cuts))return false;
      for(const auto &cut:cuts) {
        const auto owners_here=fpga::findNetOwnersByNode(f.tile(cut.tile),cut.type,cut.node);
        if(std::none_of(owners_here.begin(),owners_here.end(),[&](const auto &r){
          return r.net==owner.net && r.binding_index==owner.binding_index;
        }))continue;
        auto refs=fpga::findNetRoutesByNode(f.tile(cut.tile),cut.type,cut.node,true);
        if(std::none_of(refs.begin(),refs.end(),[&](const auto &r){return r.net==owner.net&&r.binding_index==owner.binding_index;}))return false;
      }
    }
    released=fpga::previewNetRouteCuts(owners,cuts,true);if(released.empty())return false;
    if(!commit)return true;
    for(const auto &owner:owners)cut_owners.insert(owner.net);
    return fpga::unrouteNetRoutesFromNodes(owners,cuts,true);
  };
  check(fpga::routeUnboundNetFromSource(*f.main,endpoints,f.pin({0,0},100,true),f.pin({12,0},200,false),{},32768,200,preempt),
      "failed both feasible terminal approaches");f.audit();
  std::string chosen;
  for(auto *owner:cut_owners)chosen+=owner->name+";";
  check(cut_owners.size()==1 && cut_owners.contains(f.a),
      "BUG: replaced a previously successful one-victim approach with a two-victim choice: "+chosen);
  check(fpga::isRouteComplete(f.trunk_sink->wires[0])&&fpga::isRouteComplete(f.b_sink->wires[0])&&
      fpga::isRouteComplete(f.c_sink->wires[0]),"damaged unrelated completed routes");
  check(fpga::repairNetRouteGaps(*f.a,0),"preferred transit could not be repaired");f.audit();
  check(fpga::unrouteNetRoute(*f.main,1),"requester teardown failed");f.audit();
  check(fpga::isRouteComplete(f.trunk_sink->wires[0]),"teardown damaged original source tree");
}
