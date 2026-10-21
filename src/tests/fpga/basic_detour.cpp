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

// Example 1 run 6, signal 165576: free numeric search exhausted its normal
// budget before taking a required detour away from the input. Keep its full
// admitted frontier plus the independent 19-hop free witness. All names are
// abstract. Live blocked outlets preserve the structural productivity of
// admitted LOCAL/JOINT nodes when their congested branches are omitted.
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
    device.size_width=87; device.size_height=44; device.grid_spec.size={87,44};
    device.tile_grid.resize(87*44);
    device.tile_types.reserve(87*44); device.cb_types.reserve(87*44);
    for(int y=0;y<44;++y) for(int x=0;x<87;++x) device.getTile(x,y)->coord={x,y};
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="detour_top";
    parent->is_blackbox=false; parent->nets.reserve(4096);
    primitive=&tech.design.modules.emplace_back(); primitive->name="abstract_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_165576",{66,21},true); sink=cell("sink_17492",{15,12},false);
    net=&parent->nets.emplace_back(); net->name="signal_165576"; net->designators.push_back(1);
    source->conns[1].port_ref->designator=1; sink->conns[0].port_ref->designator=1;
    sink->conns[0].set(&source->conns[1]);
    std::ifstream in(std::filesystem::path(__FILE__).replace_filename("basic_detour_frontier.txt"));
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
    require(in.eof() && arcs==62410,"incomplete free-edge snapshot");
    // Even a busy/unexpanded landing participates in SRC priority. Preserve
    // the real physical landing coordinates; unavailable/unknown landings
    // use an owned opaque DST instead of becoming invented free bypasses.
    std::ifstream sources(std::filesystem::path(__FILE__).replace_filename("basic_detour_sources.txt"));
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
    require(sources.eof() && source_rows==8983,"incomplete physical SRC metadata");
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
    type.pin_map.nodes[{"LUT6","I3",0}].setBit(106);
    t.cb_coord=c; t.name=c; t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
    cb.local_output_nodes.setBit(142); cb.local_input_nodes.setBit(106);
    name(t,fpga::CB_NODE_LOCAL,142); name(t,fpga::CB_NODE_LOCAL,106); return t;
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
      p.name=direction==rtl::Port::PORT_IN?(output?"D":"I3"):(output?"Q":"O");
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
    at.tile_type->pin_map.nodes[{"LUT6","I3",2}].setBit(4093);
    cb.local_output_nodes.setBit(4094); cb.local_input_nodes.setBit(4093);
    cb.local_src[4094].jump.setBit(4095);
    fpga::CBType::ResolvedJump jump{};jump.delta={0,0};jump.target_tile_coord=true;
    jump.target_cb_type_id=cb.type_id;jump.dsts.jump.setBit(4091);
    cb.dst_by_src[4095].push_back(jump);
    cb.dst_joint[4091].joint.setBit(4092);
    cb.joint_local[4092].local.setBit(4093);
    Wire from;from.type=Wire::WIRE_TILE_PIN;from.from=from.to=from.resource=c;
    from.local=4094;from.pos=1;from.cell_type="FDRE";from.port="Q";from.pin_dir=fpga::TILE_PIN_OUTPUT;
    Wire to=from;to.local=4093;to.pos=2;to.cell_type="LUT6";to.port="I3";to.pin_dir=fpga::TILE_PIN_INPUT;
    rtl::NetRouteBinding endpoints;endpoints.from=driver;endpoints.to=consumer;
    endpoints.from_port="Q";endpoints.to_port="I3";endpoints.route_name=foreign.name;
    require(fpga::routeUnboundNetFromSource(foreign,endpoints,from,to),"failed to route live obstruction");
    require(at.cb.src.jump.testBit(4095),"obstruction did not own its outlet");
    require(at.cb.dst.jump.testBit(4091),"obstruction did not own its landing");
  }
  Wire pin(bool output) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=output?Coord{66,21}:Coord{15,12};
    w.local=output?142:106; w.pos=0;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"FDRE":"LUT6"; w.port=output?"Q":"I3"; return w;
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

void basic_detour_regression(int mode) {
  debug_level=1; Fixture f;
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I3";
  endpoints.route_name="connection_165576";
  if(mode==2) f.tile({15,12}).cb_type->dst_local[246].local.clearBit(106);
  std::ostringstream reach;
  require(fpga::auditUnroutedReachability(*f.net,endpoints,f.pin(true),f.pin(false),reach)==(mode!=2),
      reach.str());
  require(!fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),[]{return true;}),
      "cancelled search installed a route");
  require(f.net->routes.empty() && f.sink->wires.empty(),"diagnostic or cancelled search mutated bindings");
  f.audit();
  if(mode==4) {
    // A time-limited placement can change the next failing net. Capture the
    // ACTUAL failed task, without a preselected name, and keep it failed even
    // if a read-only replay discovers this fixture's complete free detour.
    unsetenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY");
    setenv("SCALEPNR_AUDIT_FAILED_SOURCE","1",1);
    setenv("SCALEPNR_SOURCE_REACHABILITY_STATES","65536",1);
    setenv("SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES","65536",1);
    setenv("SCALEPNR_SOURCE_REACHABILITY_LOG","failed_source.log",1);
    f.tech.route.preserve_routes=true; // Catch the original failure, never exit before assertions.
    {std::ofstream clear("failed_source.log",std::ios::trunc);}
    pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,
        .from_port="Q",.to_port="I3",.net_name=endpoints.route_name,
        .endpoints_prepared=true};
    task.failure_coord={66,21};
    std::string failure;
    try {f.tech.route.failRouting(&task,"deliberate failed-task capture");}
    catch(const std::runtime_error &e) {failure=e.what();}
    unsetenv("SCALEPNR_AUDIT_FAILED_SOURCE");
    unsetenv("SCALEPNR_SOURCE_REACHABILITY_STATES");
    unsetenv("SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES");
    require(failure.find("deliberate failed-task capture")!=std::string::npos &&
        f.net->routes.empty() && f.sink->wires.empty(),
        "failed-task diagnostic rescued the task or changed its failure");
    f.audit();
    std::ifstream log("failed_source.log");
    const std::string report((std::istreambuf_iterator<char>(log)),{});
    require(report.find("FAILED_SOURCE_PAIR")!=std::string::npos &&
        report.find("FAILED_SOURCE_OWNERSHIP")!=std::string::npos &&
        report.find("source_cb=66,21")!=std::string::npos &&
        report.find("sink_cb=15,12")!=std::string::npos &&
        report.find("SOURCE_SEARCH_EDGE")!=std::string::npos &&
        report.find("SOURCE_SEARCH_NODE")!=std::string::npos &&
        report.find("SOURCE_DIAGNOSTIC_COMPLETE nodes=")!=std::string::npos,
        "failed-task capture requires a preselected net or omits its witnessed frontier");
    return;
  }
  setenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY","signal_165576",1);
  setenv("SCALEPNR_SOURCE_REACHABILITY_LOG","source_budget.log",1);
  { std::ofstream clear("source_budget.log",std::ios::trunc); }
  if(mode==3) {
    // A failed one-state task must stay failed even when the opt-in larger
    // diagnostic discovers this fixture's complete free detour. The probe
    // must neither install that path nor ask the router to commit a cut.
    setenv("SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES","65536",1);
    bool committed=false;
    const bool done=fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),{},1,200,
        [&](const auto &,bool commit,auto &) { committed|=commit; return false; });
    unsetenv("SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES");
    require(!done && !committed && f.net->routes.empty() && f.sink->wires.empty(),
        "read-only failed-task probe committed a route or cut");
    std::ifstream log("source_budget.log");
    const std::string report((std::istreambuf_iterator<char>(log)),{});
    require(report.find("SOURCE_DIAGNOSTIC_COMPLETE nodes=")!=std::string::npos,
        "larger read-only probe did not report the witnessed free detour");
    f.audit();
    return;
  }
  const auto started=std::chrono::steady_clock::now();
  const bool done=fpga::routeUnboundNetFromSource(*f.net,endpoints,f.pin(true),f.pin(false),{},
      mode==1?65536:32768,200);
  std::printf("detour search: mode=%d complete=%d ms=%.3f\n",mode,done,
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
  f.audit();
  if(mode==2) {
    require(!done && f.net->routes.empty() && f.sink->wires.empty(),"disconnected snapshot routed or leaked state");
    return;
  }
  require(done,"BUG: reachable detour lost to numeric state-budget exhaustion");
  require(f.net->routes.size()==1 && f.sink->wires.size()==1 &&
      fpga::isRouteComplete(f.sink->wires.front()),"numeric search did not install exactly one complete route");
  int hops=0; bool bridge=false;
  for(const auto &w:f.sink->wires.front()) {
    hops+=(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) ||
        (w.type==Wire::WIRE_ROUTE_EDGE && (w.from.x!=w.to.x || w.from.y!=w.to.y));
    bridge|=w.type==Wire::WIRE_ROUTE_EDGE && w.from.x==12 && w.from.y==19 &&
        w.from_node_type==fpga::CB_NODE_DST && w.from_node==3539 &&
        w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==184;
  }
  require(hops>0 && hops<=19 && bridge,"completion exceeded the witnessed path or lost its LOCAL input");
  require(f.source->coord==Coord{66,21} && f.sink->coord==Coord{15,12},"fixed endpoints moved");
  require(fpga::unrouteNetRoute(*f.net,0),"snapshot teardown failed"); f.audit();
  std::printf("recorded frontier: complete %d-hop free path; internal LOCAL bridge and ownership verified\n",hops);
}
