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

// Example 1 run 6, connection 165390: both 65K numeric priorities miss
// an entirely free 52-node witness. Preserve both admitted frontiers and
// the missing witness arcs, with raw SRC priorities and live obstructions.
// Textual names are abstract; coordinates and node IDs match the capture.
// This is a reduced frontier, not the complete device graph.
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
    device.size_width=128; device.size_height=64; device.grid_spec.size={128,64};
    device.tile_grid.resize(128*64);
    device.tile_types.reserve(128*64); device.cb_types.reserve(128*64);
    for(int y=0;y<64;++y) for(int x=0;x<128;++x) device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="detour_top";
    parent->is_blackbox=false; parent->nets.reserve(10000);
    primitive=&tech.design.modules.emplace_back(); primitive->name="abstract_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_165390",{75,1},true); sink=cell("sink_17465",{12,13},false);
    net=&parent->nets.emplace_back(); net->name="signal_165390"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1; sink->conns[0].port_ref->designator=1;
    sink->conns[0].set(&source->conns[1]);
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename("basic_free_detour_frontier.txt"));
    require(bool(in),"missing recorded free-edge snapshot");
    int x,y,kind,id,nx,ny,nkind,nid; size_t arcs=0;
    std::set<std::tuple<int,int,int,int>> productive;
    std::set<std::tuple<int,int,int,int>> available;
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
    require(in.eof() && arcs==342117,"incomplete free-edge snapshot");
    // Even a busy/unexpanded landing participates in SRC priority. Preserve
    // the real physical landing coordinates; unavailable/unknown landings
    // use an owned opaque DST instead of becoming invented free bypasses.
    std::ifstream sources(std::filesystem::path(__FILE__).replace_filename("basic_free_detour_sources.txt"));
    require(bool(sources),"missing physical SRC priority metadata");
    int src,joint,tx,ty,dst;size_t source_rows=0;
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
    require(sources.eof() && source_rows==29918,"incomplete physical SRC metadata");
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
    auto &cb=device.cb_types.emplace_back(); cb.type_id=cb.base_type_id=device.cb_types.size()-1;
    cb.name="matrix_"+std::to_string(cb.type_id);
    auto &type=device.tile_types.emplace_back(); type.name="resource_"+std::to_string(cb.type_id);
    type.type=fpga::Tile::TILE_LUTS; type.pin_map.output_nodes[1].setBit(142);
    type.pin_map.nodes[{"LUT6","I5",0}].setBit(84);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(142); cb.local_input_nodes.setBit(84);
    name(t,fpga::CB_NODE_LOCAL,142); name(t,fpga::CB_NODE_LOCAL,84); return t;
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
      p.name=direction==rtl::Port::PORT_IN?(output?"D":"I5"):(output?"Q":"O");
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
    at.tile_type->pin_map.nodes[{"LUT6","I5",2}].setBit(4093);
    cb.local_output_nodes.setBit(4094); cb.local_input_nodes.setBit(4093);
    cb.local_src[4094].jump.setBit(4095);
    fpga::CBType::ResolvedJump jump{};jump.delta={0,0};jump.target_tile_coord=true;
    jump.target_cb_type_id=cb.type_id;jump.dsts.jump.setBit(4091);
    cb.dst_by_src[4095].push_back(jump);
    cb.dst_joint[4091].joint.setBit(4092);
    cb.joint_local[4092].local.setBit(4093);
    Wire from;from.type=Wire::WIRE_TILE_PIN;from.from=from.to=from.resource=c;
    from.local=4094;from.pos=1;from.cell_type="FDRE";from.port="Q";from.pin_dir=fpga::TILE_PIN_OUTPUT;
    Wire to=from;to.local=4093;to.pos=2;to.cell_type="LUT6";to.port="I5";to.pin_dir=fpga::TILE_PIN_INPUT;
    rtl::NetRouteBinding endpoints;endpoints.from=driver;endpoints.to=consumer;
    endpoints.from_port="Q";endpoints.to_port="I5";endpoints.route_name=foreign.name;
    require(fpga::routeUnboundNetFromSource(foreign,endpoints,from,to),"failed to route live obstruction");
    require(at.cb.src.jump.testBit(4095),"obstruction did not own its outlet");
    require(at.cb.dst.jump.testBit(4091),"obstruction did not own its landing");
  }
  Wire pin(bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=output?Coord{75,1}:Coord{12,13};
    w.local=output?142:84; w.pos=0;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"FDRE":"LUT6"; w.port=output?"Q":"I5"; return w;
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

void basic_free_detour_regression(int mode) {
  debug_level=1; Fixture f;
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink;
  endpoints.from_port="Q"; endpoints.to_port="I5";
  endpoints.route_name="connection_165390";
  if(mode==2) f.tile({12,13}).cb_type->dst_local[738].local.clearBit(84);
  std::ostringstream reach;
  require(fpga::auditUnroutedReachability(*f.net,endpoints,f.pin(true),f.pin(false),reach)==(mode!=2),
      reach.str());
  require(!fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),
      []{return true;},65536,200),"cancelled search installed a route");
  require(f.net->routes.empty() && f.sink->wires.empty(),"read-only probe mutated bindings");
  f.audit();
  // Full frontier text is captured from Example 1, not regenerated on every
  // regression run. This test checks routing and real ownership directly.
  unsigned commits=0;
  const auto start=std::chrono::steady_clock::now();
  // Reject every proposed cut: the recorded complete witness is fully free.
  // Supplying the callback also exercises the existing alternate priority.
  bool done=false;
  std::string failure;
  if(mode==0) {
    f.tech.route.preserve_routes=true;
    pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,
        .from_port="Q",.to_port="I5",.net_name=endpoints.route_name,
        .fanout=false,.endpoints_prepared=true};
    try {f.tech.route.routeTaskOrFail(task,"Basic routing",200);done=true;}
    catch(const std::runtime_error &e) {failure=e.what();}
    commits=f.tech.route.route_stats.preempt_success;
  } else {
    done=fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),
        {},mode==3?1:131072,200,
        [&](const auto &,bool commit,auto &) {commits+=commit;return false;});
  }
  std::printf("free detour mode=%d complete=%d ms=%.3f\n",mode,done,
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  f.audit();
  if(mode==2 || mode==3) {
    require(!done && !commits && f.net->routes.empty() && f.sink->wires.empty(),
        "disconnected/limited frontier routed or damaged owners");
    return;
  }
  require(done,"BUG: normal numeric frontier misses the captured free detour: "+failure);
  require(!commits && f.net->routes.size()==1 && f.sink->wires.size()==1 &&
      fpga::isRouteComplete(f.sink->wires[0]),"did not install exactly one complete free route");
  unsigned hops=0;bool bridge=false;
  for(const auto &w:f.sink->wires[0]) {
    hops+=(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) ||
        (w.type==Wire::WIRE_ROUTE_EDGE && (w.from.x!=w.to.x || w.from.y!=w.to.y));
    bridge|=w.type==Wire::WIRE_ROUTE_EDGE && w.from.x==22 && w.from.y==12 &&
        w.from_node_type==fpga::CB_NODE_DST && w.from_node==3845 &&
        w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==183;
  }
  require(hops>0 && hops<=21 && bridge,"lost required internal LOCAL detour or exceeded witness");
  require(f.source->coord==Coord{75,1} && f.sink->coord==Coord{12,13},"fixed endpoints moved");
  require(fpga::unrouteNetRoute(*f.net,0),"requester teardown failed");f.audit();
}
