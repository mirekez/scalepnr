#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <list>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace {
using fpga::Coord;
using fpga::Wire;
void require(bool ok,const std::string &message) {
  if(!ok) throw std::runtime_error(message);
}

// Run 6: the normal numeric search never reaches Grounding, although a
// whole-source read-only probe validates a 17-hop path with one private
// transit cut. Preserve all admitted arcs, raw physical SRC priorities and
// the witness. Abstract node names replace all device-specific labels.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*sink;
  rtl::Net *net, *victim;
  rtl::Inst *victim_source, *victim_sink;
  explicit Fixture(bool expanded=false) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=148; device.size_height=209; device.grid_spec.size={148,209};
    device.tile_grid.resize(148*209);
    device.tile_types.reserve(4096); device.cb_types.reserve(4096);
    for(int y=0;y<209;++y) for(int x=0;x<148;++x) device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="detour_top";
    parent->is_blackbox=false; parent->nets.reserve(4096);
    primitive=&tech.design.modules.emplace_back(); primitive->name="abstract_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_166890",{41,8},true); sink=cell("sink_4178",{37,36},false);
    net=&parent->nets.emplace_back(); net->name="signal_166890"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1; sink->conns[0].port_ref->designator=1;
    sink->conns[0].set(&source->conns[1]);
    int x,y,kind,id,nx,ny,nkind,nid; size_t arcs=0;
    std::set<std::tuple<int,int,int,int>> productive;
    std::set<std::tuple<int,int,int,int>> available;
    for(const char *file:{"basic_grounding_detour_frontier.txt",
                         expanded?"basic_grounding_detour_extra_frontier.txt":nullptr}) {
    if(!file)continue;
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename(file));
    require(bool(in),"missing recorded free-edge snapshot");
    while(in>>x>>y>>kind>>id>>nx>>ny>>nkind>>nid) {
      auto &from=tile({x,y}); auto &to=tile({nx,ny}); auto &cb=*from.cb_type;
      name(from,kind,id); name(to,nkind,nid);
      available.insert({x,y,kind,id});available.insert({nx,ny,nkind,nid});
      if(kind==fpga::CB_NODE_LOCAL || kind==fpga::CB_NODE_JOINT) productive.insert({x,y,kind,id});
      if(nkind==fpga::CB_NODE_LOCAL || nkind==fpga::CB_NODE_JOINT) productive.insert({nx,ny,nkind,nid});
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
    require(in.eof(),"invalid free-edge snapshot");
    }
    require(arcs==(expanded?122286:104554),"incomplete free-edge snapshot");
    install_victim();
    // Even a busy/unexpanded landing participates in SRC priority. Preserve
    // the real physical landing coordinates; unavailable/unknown landings
    // use an owned opaque DST instead of becoming invented free bypasses.
    int src,joint,tx,ty,dst;size_t source_rows=0;
    for(const char *file:{"basic_grounding_detour_sources.txt",
                         expanded?"basic_grounding_detour_extra_sources.txt":nullptr}) {
    if(!file)continue;
    std::ifstream sources(std::filesystem::path(__FILE__).replace_filename(file));
    require(bool(sources),"missing physical SRC priority metadata");
    while(sources>>x>>y>>src>>joint>>tx>>ty>>dst) {
      auto &at=tile({x,y});auto &cb=*at.cb_type;
      if(joint && cb.src_joint[src].joint.firstSetBit()<0)cb.src_joint[src].joint.setBit(4092);
      if(tx>=0 && cb.dst_by_src[src].empty()) {
        auto &landing=tile({tx,ty});
        fpga::CBType::ResolvedJump jump{};jump.delta=Coord{tx,ty}-Coord{x,y};
        jump.target_tile_coord=true;jump.target_cb_type_id=landing.cb_type->type_id;
        jump.dsts.jump.setBit(available.count({tx,ty,fpga::CB_NODE_DST,dst})?dst:4091);
        cb.dst_by_src[src].push_back(jump);
      }
      ++source_rows;
    }
    require(sources.eof(),"invalid physical SRC metadata");
    }
    require(source_rows>=12271,"incomplete physical SRC metadata");
    for(auto &at:device.tile_grid)if(at.cb_type)blocked_outlet(at);
    for(const auto &[px,py,pk,pid]:productive) {
      auto &at=tile({px,py});
      if(pk==fpga::CB_NODE_LOCAL) at.cb_type->local_src[pid].jump.setBit(4095);
      else at.cb_type->joint_src[pid].jump.setBit(4095);
    }
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  fpga::Tile &tile(Coord c) {
    auto *at=device.getTile(c.x,c.y);
    require(at,"snapshot coordinate outside fixture: "+std::to_string(c.x)+","+std::to_string(c.y));
    auto &t=*at; if(t.cb_type) return t;
    require(device.cb_types.size()<device.cb_types.capacity(),"fixture exceeded stable CB storage");
    auto &cb=device.cb_types.emplace_back(); cb.type_id=cb.base_type_id=device.cb_types.size()-1;
    cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=device.tile_types.emplace_back(); type.name="resource_"+std::to_string(cb.type_id);
    type.type=fpga::Tile::TILE_LUTS; type.pin_map.output_nodes[1].setBit(146);
    type.pin_map.nodes[{"LUT6","I1",0}].setBit(83);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(146); cb.local_input_nodes.setBit(83);
    name(t,fpga::CB_NODE_LOCAL,146); name(t,fpga::CB_NODE_LOCAL,83); return t;
  }
  static void name(fpga::Tile &t,int kind,int id) {
    t.cb_type->rememberNodeName(static_cast<fpga::CBNodeNameType>(kind),id,
        "node_"+std::to_string(unsigned(id)*2654435761U+unsigned(kind)));
  }
  rtl::Inst *cell(const std::string &name,Coord c,bool output,int position=0) {
    auto &type=cells.emplace_back(); type.name=name; type.type=output?"FDRE":"LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction; p.designator=-1;
      p.name=direction==rtl::Port::PORT_IN?(output?"D":"I1"):(output?"Q":"O");
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=position; i.coord=c; i.outline.fixed=true;
    i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    tile(c).assign(&i); return &i;
  }
  void blocked_outlet(fpga::Tile &at) {
    const auto c=at.coord;
    const auto suffix=std::to_string(c.x)+"_"+std::to_string(c.y);
    auto *driver=cell("obstruction_driver_"+suffix,c,true,1);
    auto *consumer=cell("obstruction_sink_"+suffix,c,false,2);
    auto &foreign=parent->nets.emplace_back(); foreign.name="obstruction_"+suffix;
    const int designator=parent->nets.size(); foreign.designators.push_back(designator);
    driver->conns[1].port_ref->designator=designator;
    consumer->conns[0].port_ref->designator=designator;
    consumer->conns[0].set(&driver->conns[1]);
    auto &cb=*at.cb_type;
    for(auto [kind,id]:{std::pair{fpga::CB_NODE_LOCAL,4094},
                       {fpga::CB_NODE_LOCAL,4093},{fpga::CB_NODE_SRC,4095},
                       {fpga::CB_NODE_JOINT,4092},{fpga::CB_NODE_DST,4091}}) name(at,kind,id);
    at.tile_type->pin_map.nodes[{"FDRE","Q",1}].setBit(4094);
    at.tile_type->pin_map.nodes[{"LUT6","I1",2}].setBit(4093);
    cb.local_output_nodes.setBit(4094); cb.local_input_nodes.setBit(4093);
    cb.local_src[4094].jump.setBit(4095);
    fpga::CBType::ResolvedJump jump{};jump.delta={0,0};jump.target_tile_coord=true;
    jump.target_cb_type_id=cb.type_id;jump.dsts.jump.setBit(4091);
    cb.dst_by_src[4095].push_back(jump);
    cb.dst_joint[4091].joint.setBit(4092);
    cb.joint_local[4092].local.setBit(4093);
    cb.derived_masks_valid=false;
    Wire from;from.type=Wire::WIRE_TILE_PIN;from.from=from.to=from.resource=c;
    from.local=4094;from.pos=1;from.cell_type="FDRE";from.port="Q";from.pin_dir=fpga::TILE_PIN_OUTPUT;
    Wire to=from;to.local=4093;to.pos=2;to.cell_type="LUT6";to.port="I1";to.pin_dir=fpga::TILE_PIN_INPUT;
    rtl::NetRouteBinding endpoints;endpoints.from=driver;endpoints.to=consumer;
    endpoints.from_port="Q";endpoints.to_port="I1";endpoints.route_name=foreign.name;
    if(!fpga::routeUnboundNetFromSource(foreign,endpoints,from,to)) {
      std::ostringstream out;
      fpga::auditUnroutedReachability(foreign,endpoints,from,to,out,1000);
      require(false,"failed to route live obstruction at "+suffix+"\n"+out.str());
    }
    require(at.cb.src.jump.testBit(4095),"obstruction did not own its outlet");
    require(at.cb.dst.jump.testBit(4091),"obstruction did not own its landing");
  }

  void add_arc(Coord a,int kind,int id,Coord b,int nkind,int nid) {
    auto &from=tile(a), &to=tile(b); auto &cb=*from.cb_type;
    name(from,kind,id); name(to,nkind,nid);
    if(kind==fpga::CB_NODE_SRC && nkind==fpga::CB_NODE_DST) {
      // Do not resolve while the fixture's topology is still under construction.
      for(const auto &target:std::as_const(cb).dst_by_src[id])
        if(target.delta.x==b.x-a.x && target.delta.y==b.y-a.y &&
            target.target_cb_type_id==to.cb_type->type_id &&
            target.dsts.jump.testBit(nid)) return;
      fpga::CBType::ResolvedJump j{};j.delta=b-a;j.target_tile_coord=true;
      j.target_cb_type_id=to.cb_type->type_id;j.dsts.jump.setBit(nid);
      cb.dst_by_src[id].push_back(j);
    } else {
      require(a==b,"non-jump victim arc crosses Tiles");
      if(kind==0 && nkind==3)cb.local_src[id].jump.setBit(nid);
      else if(kind==0 && nkind==0)cb.local_local[id].local.setBit(nid);
      else if(kind==0 && nkind==1)cb.local_joint[id].joint.setBit(nid);
      else if(kind==4 && nkind==3)cb.dst_src[id].jump.setBit(nid);
      else if(kind==4 && nkind==0)cb.dst_local[id].local.setBit(nid);
      else if(kind==4 && nkind==1)cb.dst_joint[id].joint.setBit(nid);
      else if(kind==1 && nkind==3)cb.joint_src[id].jump.setBit(nid);
      else if(kind==1 && nkind==0)cb.joint_local[id].local.setBit(nid);
      else if(kind==1 && nkind==1)cb.joint_joint[id].joint.setBit(nid);
      else require(false,"unsupported victim arc");
      cb.rememberConnName(static_cast<fpga::CBNodeNameType>(kind),id,
        static_cast<fpga::CBNodeNameType>(nkind),nid,
        *cb.nodeName(static_cast<fpga::CBNodeNameType>(kind),id),
        *cb.nodeName(static_cast<fpga::CBNodeNameType>(nkind),nid));
    }
  }
  void lease(Coord c,int kind,int id) {
    auto &cb=tile(c).cb;
    switch(kind) {
      case fpga::CB_NODE_LOCAL:cb.local.local.setBit(id);break;
      case fpga::CB_NODE_DST:cb.dst.jump.setBit(id);break;
      case fpga::CB_NODE_SRC:cb.src.jump.setBit(id);break;
      case fpga::CB_NODE_JOINT:cb.joint.jump.setBit(id);break;
    }
  }
  void install_victim() {
    victim_source=cell("transit_driver",{54,31},true,3);
    victim_sink=cell("transit_receiver",{37,37},false,3);
    victim=&parent->nets.emplace_back();victim->name="transit_signal";victim->designators.push_back(2);
    victim->pnr_preserved=true; // The real obstruction was retained from an older run.
    victim_source->conns[1].port_ref->designator=2;
    victim_sink->conns[0].port_ref->designator=2;
    victim_sink->conns[0].set(&victim_source->conns[1]);
    tile({54,31}).tile_type->pin_map.nodes[{"FDRE","Q",3}].setBit(150);
    tile({54,31}).cb_type->local_output_nodes.setBit(150);
    tile({37,37}).tile_type->pin_map.nodes[{"LUT6","I1",3}].setBit(99);
    tile({37,37}).cb_type->local_input_nodes.setBit(99);
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename("basic_grounding_detour_victim.txt"));
    std::vector<Wire> route;int type;Wire w;
    while(in>>type>>w.from.x>>w.from.y>>w.to.x>>w.to.y>>w.local>>w.jump>>w.dst
          >>w.joint>>w.joint2>>w.pos>>w.owns_dst>>w.owns_landing
          >>w.from_node_type>>w.from_node>>w.to_node_type>>w.to_node>>w.pin_dir) {
      w.type=static_cast<Wire::Type>(type);w.route_jump=w.jump;
      w.shared=false; // The contested segment is private; unrelated siblings are omitted.
      w.net_name=victim->name;w.resource=w.from;
      if(w.type==Wire::WIRE_TILE_PIN) {
        w.pos=3;w.cell_type=w.pin_dir==fpga::TILE_PIN_OUTPUT?"FDRE":"LUT6";
        w.port=w.pin_dir==fpga::TILE_PIN_OUTPUT?"Q":"I1";
      }
      route.push_back(w);
    }
    require(in.eof() && route.size()==17,"incomplete captured transit route");
    for(size_t i=0;i<route.size();++i) {
      const auto &w=route[i];
      if(w.type==Wire::WIRE_ROUTE_EDGE) {
        add_arc(w.from,w.from_node_type,w.from_node,w.to,w.to_node_type,w.to_node);
        lease(w.from,w.from_node_type,w.from_node);lease(w.to,w.to_node_type,w.to_node);
      } else if(w.type==Wire::WIRE_CROSSBAR) {
        int kind=w.pos==0?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,id=w.local;
        if(w.pos!=0)lease(w.from,kind,id);
        auto append=[&](int k,int n) {add_arc(w.from,kind,id,w.from,k,n);kind=k;id=n;};
        if(w.joint2>=0){append(fpga::CB_NODE_JOINT,w.joint2);lease(w.from,kind,id);}
        if(w.joint>=0){append(fpga::CB_NODE_JOINT,w.joint);lease(w.from,kind,id);}
        if(w.jump>=0) {
          append(fpga::CB_NODE_SRC,w.jump);lease(w.from,kind,id);
          add_arc(w.from,kind,id,w.to,fpga::CB_NODE_DST,w.dst);lease(w.to,fpga::CB_NODE_DST,w.dst);
        } else {
          require(i+1<route.size() && route[i+1].type==Wire::WIRE_TILE_PIN,"bad victim terminal");
          append(fpga::CB_NODE_LOCAL,route[i+1].local);lease(w.from,kind,id);
        }
      } else if(w.pin_dir==fpga::TILE_PIN_INPUT) {
        lease(w.from,fpga::CB_NODE_LOCAL,w.local);tile(w.from).pin_state.leased_nodes.setBit(w.local);
      }
    }
    victim_sink->wires.push_back(std::move(route));
    auto binding=fpga::attachNetRoute(*victim,*victim_sink,0,victim_source,victim_sink,"Q","I1",victim->name);
    fpga::registerNetRouteTiles(*victim,victim_sink->wires[0],binding);
    // A private repair outlet, unreachable from the requester's free prefix.
    // It reconnects the kept victim boundary without touching the new route.
    add_arc({36,37},fpga::CB_NODE_DST,819,{36,37},fpga::CB_NODE_SRC,4090);
    add_arc({36,37},fpga::CB_NODE_SRC,4090,{37,37},fpga::CB_NODE_DST,19);
  }

  Wire pin(bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=output?Coord{41,8}:Coord{37,36};
    w.local=output?146:83; w.pos=0;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"FDRE":"LUT6"; w.port=output?"Q":"I1"; return w;
  }
  void audit() {
    std::vector<rtl::Net*> all_nets;
    for(auto &owner:parent->nets) all_nets.push_back(&owner);
    for(auto &t:device.tile_grid) if(t.cb_type) {
      std::ostringstream out; auto a=fpga::auditTileCongestion(t,out,all_nets);
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
          !a.stale_registrations&&!a.conflicting_driver_nodes,out.str());
    }
  }
};
}

void basic_grounding_detour_regression(int mode) {
  debug_level=1;Fixture f(mode==4);
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source;endpoints.to=f.sink;endpoints.from_port="Q";endpoints.to_port="I1";
  endpoints.route_name="connection_166890";
  if(mode==2)f.victim->route_protected=true;
  if(mode==3) {
    auto &cb=*f.tile({37,36}).cb_type;
    cb.joint_local[9].local.clearBit(83);cb.derived_masks_valid=false;
  }
  f.audit();
  std::ostringstream reach;
  require(!fpga::auditUnroutedReachability(*f.net,endpoints,f.pin(true),f.pin(false),reach),
      "fixture unexpectedly has a completely free input path");
  unsigned previews=0,commits=0,owners_checked=0;
  auto preemption=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,
                      std::vector<fpga::RouteCutNode> &released) {
    if(cuts.empty())return false;
    auto owners=fpga::findNetRouteCutVictims(cuts);
    if(owners.empty())return false;
    for(const auto &owner:owners) {
      if(owner.net==f.victim)++owners_checked;
      if(owner.net!=f.victim || !owner.net->routeCanBePreempted() ||
         !fpga::canPreemptNetRouteFromNodes(*owner.net,owner.binding_index,cuts))return false;
      for(const auto &cut:cuts) {
        auto *tile=f.device.getTile(cut.tile.x,cut.tile.y);
        auto transit=fpga::findNetRoutesByNode(*tile,cut.type,cut.node,true);
        if(std::none_of(transit.begin(),transit.end(),[&](const auto &ref) {
          return ref.net==owner.net && ref.binding_index==owner.binding_index;
        }))return false;
      }
    }
    released=fpga::previewNetRouteCuts(owners,cuts,true);
    if(released.empty())return false;
    if(!commit){++previews;return true;}
    ++commits;
    return fpga::unrouteNetRoutesFromNodes(owners,cuts,true);
  };
  require(!fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),
      []{return true;},32768,200,preemption),"cancelled search routed");
  require(!previews && !commits,"cancelled search inspected/cut victims");
  if(mode==0) {
    // A positive preview requiring a real transit cut must remain read-only,
    // including the new unfiltered failure-snapshot entry point.
    std::ostringstream diagnostic;
    require(fpga::auditRouteCompletion(*f.net,endpoints,f.pin(true),f.pin(false),
        diagnostic,preemption,1,65536,200),"failed to audit the known terminal cut");
    require(previews>0 && !commits && f.net->routes.empty() && f.sink->wires.empty() &&
        fpga::isRouteComplete(f.victim_sink->wires[0]) &&
        diagnostic.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=")!=std::string::npos,
        "preview-only terminal witness cut or attached a route");
    f.audit();previews=owners_checked=0;
  }
  setenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY","signal_166890",1);
  setenv("SCALEPNR_SOURCE_REACHABILITY_LOG","grounding_detour.log",1);
  {std::ofstream clear("grounding_detour.log",std::ios::trunc);}
  const auto start=std::chrono::steady_clock::now();
  bool done=false;
  std::string failure;
  if(mode==4) {
    // The first capture omitted arcs reached only by the alternate priority.
    // Replay their union through Basic itself: its original 32K attempts all
    // miss the legal terminal cut. A raw 65K control is already tested above.
    // This test must exercise the caller's recovery, not silently enlarge a
    // low-level per-attempt budget or rescue it with a diagnostic probe.
    f.tech.route.preserve_routes=true;
    pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,
        .from_port="Q",.to_port="I1",.net_name=endpoints.route_name,
        .fanout=false,.endpoints_prepared=true};
    try {f.tech.route.routeTaskOrFail(task,"Basic routing",200);done=true;}
    catch(const std::runtime_error &e) {failure=e.what();}
    commits=f.tech.route.route_stats.preempt_success;
  } else done=fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),
      {},mode==0?32768:65536,200,preemption);
  std::printf("grounding detour mode=%d complete=%d previews=%u commits=%u ms=%.3f\n",
      mode,done,previews,commits,
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  f.audit();
  std::ifstream log("grounding_detour.log");
  const std::string report((std::istreambuf_iterator<char>(log)),{});
  if(mode==4)require(report.find("max_states=65536")!=std::string::npos,
      "recovery regression did not exercise the larger caller-owned frontier");
  if(mode==0)require(report.find("cost_scale=2")!=std::string::npos,"missing alternate bounded attempt");
  if(mode==1)require(report.find("cost_scale=2")==std::string::npos,"retried after an original successful completion");
  if(mode==2 || mode==3) {
    if(mode==2)require(owners_checked>0,"protected control never inspected its transit owner");
    require(!done && !commits && f.net->routes.empty() && f.sink->wires.empty() &&
        fpga::isRouteComplete(f.victim_sink->wires[0]),"failed/protected completion damaged state");
    return;
  }
  require(done,"BUG: normal numeric frontier misses a validated 17-hop Grounding approach: "+failure);
  require(commits==1 && f.net->routes.size()==1 && f.sink->wires.size()==1 &&
      fpga::isRouteComplete(f.sink->wires[0]),"did not atomically install one complete replacement");
  int hops=0;
  for(const auto &w:f.sink->wires[0])
    if((w.type==Wire::WIRE_CROSSBAR && w.jump>=0) || w.type==Wire::WIRE_ROUTE_EDGE)
      hops+=w.from.x!=w.to.x || w.from.y!=w.to.y;
  require(hops>0 && hops<=17,"exceeded the validated 17-hop path");
  require(f.source->coord==Coord{41,8} && f.sink->coord==Coord{37,36},"fixed endpoints moved");
  require(fpga::hasRouteGaps(f.victim_sink->wires[0]),"transit victim was not retained around its gap");
  require(fpga::repairNetRouteGaps(*f.victim,0),"private transit repair failed");
  f.audit();
  require(fpga::isRouteComplete(f.victim_sink->wires[0]) &&
      fpga::isRouteComplete(f.sink->wires[0]),"repair damaged requester");
  require(fpga::unrouteNetRoute(*f.net,0),"requester teardown failed");f.audit();
}
