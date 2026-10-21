#include "Tech.h"
#include "Wire.h"

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

// Reduced reproduction of the latest shared-input owner repair failure.
// Two interrupted bindings retain the SAME terminal DST -> JOINT -> LOCAL.
// Repairing their exact DST-to-DST gap is impossible, but another source exit
// reaches a free terminal DST feeding that same JOINT. The old terminal input
// must change for both interrupted bindings together, not for just one copy.
//
// Original boundary coordinates (49,41) -> (57,39), terminal (53,40), are
// translated by (-49,-38). Intermediate routing is deliberately reduced;
// this is a mechanism reproduction, not a claim to copy the entire chip.
// Every lease comes from commitPreparedRoute; the gap comes from a real cut.
// A foreign completed Grounding occupies the cut approach. There are no
// manually forged busy bits, missing owners, or vendor-specific node names.
struct Fixture {
  static constexpr int width=22, height=14, input=63;
  int output=300;
  Coord source_at{9,12}, start{0,3}, cut_at{4,1}, finish{8,1}, sink_at{4,2};
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  Referable<rtl::Module> *parent, *primitive;
  std::list<Referable<rtl::Cell>> cells;
  std::vector<rtl::Net *> nets;
  rtl::Inst *source, *owner, *replica=nullptr;
  rtl::Net *signal, *blocker;

  explicit Fixture(bool shared, bool recorded=false) {
    if(recorded) {
      output=156; source_at={17,12}; start={8,3}; cut_at={8,1};
      finish={16,1}; sink_at={12,2};
    }
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=width; device.size_height=height; device.grid_spec.size={width,height};
    device.tile_grid.resize(width*height);
    std::vector<Coord> active{source_at,start,cut_at,finish,sink_at,{0,4}};
    if(recorded) for(Coord c:{Coord{21,13},{21,12},{17,11},{16,11},{16,9},
        {16,3},{8,2},{9,1},{9,12},{0,12},{4,11},{4,5},{9,13}}) active.push_back(c);
    device.tile_types.reserve(active.size()); device.cb_types.reserve(active.size());
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
      auto &t=*device.getTile(x,y); t.coord=t.cb_coord={x,y};
    }
    // Only the recorded Tiles participate; do not allocate Crossbar models for
    // the empty geometric space between the retained boundary coordinates.
    for(Coord c:active) {
      auto &t=tile(c); auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=device.cb_types.size()-1;
      cb.name="matrix_"+std::to_string(cb.type_id);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(cb.type_id); type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(output);
      for(int slot:{0,1}) type.pin_map.nodes[{"LUT6","I0",slot}].setBit(input);
      t.name=t.coord; t.full_name=type.name;
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      cb.local_output_nodes.setBit(output); cb.local_input_nodes.setBit(input);
      name(t.coord,fpga::CB_NODE_LOCAL,output); name(t.coord,fpga::CB_NODE_LOCAL,input);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="shared_terminal_top";
    parent->is_blackbox=false; parent->nets.reserve(8);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_701",source_at); owner=cell("owner_702",sink_at);
    signal=connect(*source,*owner);
    Route route;
    if(recorded) {
      // All coordinates and numeric arcs of the last complete owner route,
      // translated by (-41,-38). Normalize unrelated shared-prefix flags:
      // only the two terminal-sharing bindings are instantiated here.
      route={pin(source_at,true),edge(source_at,{21,13},156,547,547,true),
        edge({21,13},{21,12},547,244,248),edge({21,12},{17,11},248,3840,3845),
        edge({17,11},{16,11},3845,3841,3846),edge({16,11},{16,9},3846,193,193),
        edge({16,9},{16,3},193,161,161),edge({16,3},start,161,3073,3078),
        edge(start,cut_at,3078,193,193),edge(cut_at,finish,193,1027,1029),
        edge(finish,{16,3},1029,65,65),edge({16,3},start,65,3078,3079),
        edge(start,{8,2},3079,248,242),edge({8,2},{9,1},242,737,737),
        edge({9,1},sink_at,737,545,545),terminal(sink_at,545,55),pin(sink_at,false)};
    } else {
      route={pin(source_at,true),edge(source_at,start,output,701,3078,true),
        edge(start,cut_at,3078,702,801),edge(cut_at,finish,801,703,1029),
        edge(finish,sink_at,1029,704,545),terminal(sink_at,545,55),pin(sink_at,false)};
    }
    publish(*signal,*source,*owner,route);
    if(shared) {
      replica=cell("replica_703",sink_at,1); connect(*source,*replica);
      for(auto &w:route) w.shared=true;
      // Sharing attaches a second binding to already committed resources;
      // commitPreparedRoute is deliberately not a sharing API.
      publish(*signal,*source,*replica,route,true);
    }
    audit();
    // Cut both copies atomically at the intermediate landing. The source
    // prefix and downstream shared input remain registered and leased.
    const int cut_dst=recorded?193:801, cut_src=recorded?193:702;
    std::vector<fpga::RouteCutNode> cuts{{cut_at,fpga::CB_NODE_DST,cut_dst}};
    auto victims=fpga::findNetRouteCutVictims(cuts);
    require(victims.size()==(shared?2U:1U),"cut did not discover every terminal replica");
    require(fpga::unrouteNetRoutesFromNodes(victims,cuts,true),"cannot form retained gap");
    for(auto *sink:{owner,replica}) if(sink) {
      const auto &r=sink->wires.front();
      unsigned gaps=0;
      for(auto &w:r) if(w.type==Wire::WIRE_GAP) {
        ++gaps;
        require(w.from.x==start.x && w.from.y==start.y && w.from_node==3078 &&
            w.to.x==finish.x && w.to.y==finish.y && w.to_node==1029 &&
            w.from_node_type==fpga::CB_NODE_DST && w.to_node_type==fpga::CB_NODE_DST,
            "cut produced a different gap boundary");
      }
      require(gaps==1 && !fpga::isRouteComplete(r),"expected one retained transit gap");
    }
    auto *a=cell("blocker_source_704",{0,4}), *b=cell("blocker_sink_705",cut_at);
    blocker=connect(*a,*b);
    publish(*blocker,*a,*b,{pin({0,4},true),edge({0,4},start,output,705,802,true),
        edge(start,cut_at,802,cut_src,cut_dst),terminal(cut_at,cut_dst),pin(cut_at,false)});
    audit();
  }
  void recordedBypass() {
    // Exact free/same-source witness from the real failure audit. Only its
    // final DST977 -> SRC545 differs from a retained input arc. It reconnects
    // a disconnected island, not another signal, and does not require a cut.
    // A complete sibling owns the first hop, as in the captured witness.
    auto *stable=cell("stable_706",{9,13}); connect(*source,*stable);
    publish(*signal,*source,*stable,{pin(source_at,true),
        edge(source_at,{9,12},156,3075,3080,true),
        edge({9,12},{9,13},3080,708,803),terminal({9,13},803),pin({9,13},false)});
    edge({9,12},{0,12},3080,3075,3080);
    auto &cb=*tile({0,12}).cb_type;
    name({0,12},fpga::CB_NODE_LOCAL,184);
    cb.dst_local[3080].local.setBit(184);
    edge({0,12},{4,11},184,818,977,true);
    edge({4,11},{4,5},977,161,161);
    edge({4,5},{9,1},161,977,977);
    edge({9,1},sink_at,977,545,545);
  }
  void protectOtherDestination() {
    // This incomplete sibling uses the same detached SRC but continues to a
    // different input. It is outside the terminal-replacement group, so a
    // repair must not silently discard or repoint its retained tail.
    auto *other=cell("other_destination_707",{0,4},1); connect(*source,*other);
    Route suffix{edge(sink_at,{0,4},545,710,804),terminal({0,4},804),pin({0,4},false)};
    suffix.front().pos=2; suffix.front().owns_dst=false;
    prepare(); require(pnr::commitPreparedRoute(suffix),"cannot commit protected sibling suffix");
    Route r(owner->wires.front().begin(),owner->wires.front().end()-2);
    for(auto &w:r) w.shared=true;
    r.insert(r.end(),suffix.begin(),suffix.end());
    publish(*signal,*source,*other,std::move(r),true);
  }
  fpga::Tile &tile(Coord c) { return *device.getTile(c.x,c.y); }
  void name(Coord c,fpga::CBNodeNameType kind,int id) {
    tile(c).cb_type->rememberNodeName(kind,id,"node_"+
        std::to_string((unsigned(id)*2654435761U)^(unsigned(kind)*101U)));
  }
  Wire edge(Coord a,Coord b,int local,int src,int dst,bool takeoff=false) {
    auto &cb=*tile(a).cb_type; auto role=takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    name(a,role,local); name(a,fpga::CB_NODE_SRC,src); name(b,fpga::CB_NODE_DST,dst);
    if(takeoff) cb.local_src[local].jump.setBit(src); else cb.dst_src[local].jump.setBit(src);
    cb.rememberConnName(role,local,fpga::CB_NODE_SRC,src,
        *cb.nodeName(role,local),*cb.nodeName(fpga::CB_NODE_SRC,src));
    if(cb.dst_by_src[src].empty()) {
      fpga::CBType::ResolvedJump jump{}; jump.delta=b-a; jump.target_tile_coord=true;
      jump.target_cb_type_id=tile(b).cb_type->type_id; jump.dsts.jump.setBit(dst);
      cb.dst_by_src[src].push_back(jump);
    }
    Wire w; w.from=a; w.to=b; w.local=local; w.jump=w.route_jump=src; w.dst=dst;
    w.pos=takeoff?0:1; return w;
  }
  Wire terminal(Coord c,int dst,int joint=-1) {
    auto &cb=*tile(c).cb_type; name(c,fpga::CB_NODE_DST,dst);
    if(joint<0) cb.dst_local[dst].local.setBit(input);
    else {
      name(c,fpga::CB_NODE_JOINT,joint);
      cb.dst_joint[dst].joint.setBit(joint); cb.joint_local[joint].local.setBit(input);
    }
    Wire w; w.from=w.to=c; w.pos=1; w.local=dst; w.joint=joint; return w;
  }
  Wire pin(Coord c,bool out) {
    Wire w; w.type=Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=c;
    w.local=out?output:input; w.pin_dir=out?1:0; w.pos=out?0:1;
    w.cell_type="LUT6"; w.port=out?"O":"I0"; return w;
  }
  rtl::Inst *cell(const std::string &label,Coord c,int pos=0) {
    auto &type=cells.emplace_back(); type.name=label; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=pos; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) { auto &conn=i.conns.emplace_back(); conn.port_ref.set(&p); conn.inst_ref.set(&i); }
    tile(c).assign(&i); return &i;
  }
  rtl::Net *connect(rtl::Inst &a,rtl::Inst &b) {
    auto &out=a.conns[1]; auto &in=b.conns[0]; int id=out.port_ref->designator;
    if(id<0) {
      id=parent->nets.size()+1; auto &n=parent->nets.emplace_back();
      n.name="signal_"+std::to_string(id); n.designators.push_back(id);
      out.port_ref->designator=id; nets.push_back(&n);
    }
    in.port_ref->designator=id; in.set(&out); return &parent->nets[id-1];
  }
  void publish(rtl::Net &n,rtl::Inst &a,rtl::Inst &b,Route route,bool borrowed=false) {
    prepare();
    if(!borrowed) require(pnr::commitPreparedRoute(route),"fixture route commit failed for "+b.makeName());
    b.wires.push_back(std::move(route));
    auto index=fpga::attachNetRoute(n,b,b.wires.size()-1,&a,&b,"O","I0",b.makeName()+".signal");
    fpga::registerNetRouteTiles(n,b.wires.back(),index);
  }
  void prepare() {
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream out; auto a=fpga::auditTileCongestion(t,out,nets);
      require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
          !a.stale_registrations,out.str());
    }
  }
};
} // namespace

// mode 0: shared-terminal bypass; 1: private-terminal
// control; 2: exact-gap repair control; 3: genuinely blocked shared terminal;
// 4: imported real retained-SRC bypass; 5: imported route exact-gap control;
// 6: retained-SRC used by an interrupted binding to another destination.
void fanout_shared_terminal_gap_regression(int mode) {
  Fixture f(mode!=1,mode>=4);
  if(mode==4 || mode==6) {
    f.recordedBypass();
    if(mode==6) f.protectOtherDestination();
  }
  else if(mode==2 || mode==5) f.edge(f.start,f.finish,3078,707,1029);
  else if(mode!=3) {
    f.edge(f.source_at,f.sink_at,f.output,706,244,true);
    f.terminal(f.sink_at,244,55);
  }
  f.prepare(); f.audit();
  std::ostringstream before, blockers;
  fpga::dumpNetRouteHistory(*f.signal,before); fpga::dumpNetRouteHistory(*f.blocker,blockers);
  std::ostringstream strict, physical;
  const bool preserving=fpga::auditNetRouteReachability(*f.signal,0,strict,true);
  const bool reachable=fpga::auditNetRouteReachability(*f.signal,0,physical,false);
  require(strict.str().find("limited=0")!=std::string::npos &&
      physical.str().find("limited=0")!=std::string::npos,
      "reduced fixture unexpectedly exhausted the reachability budget");
  require(reachable==(mode!=3),"fixture has unexpected physical reachability");
  const bool repaired=fpga::repairNetRouteGaps(*f.signal,0,{},32768,200);
  f.audit();
  std::ostringstream blocker_after; fpga::dumpNetRouteHistory(*f.blocker,blocker_after);
  require(blockers.str()==blocker_after.str(),"repair changed protected foreign Grounding");
  std::printf("shared_terminal_gap: mode=%d physical_reachable=%d preserving_inputs=%d repaired=%d\n",
      mode,reachable,preserving,repaired);
  if(!repaired) {
    std::ostringstream after; fpga::dumpNetRouteHistory(*f.signal,after);
    require(before.str()==after.str(),"failed repair changed retained routes");
    std::fprintf(stderr,"%s%s",strict.str().c_str(),physical.str().c_str());
  }
  if(mode==3 || mode==6) { require(!repaired,"protected control unexpectedly repaired"); return; }
  require(repaired,mode==4 ? "retained-SRC gap repair rejected the imported source-to-sink bypass" :
      "shared-terminal gap repair rejected a reachable replacement input");
  require(fpga::isRouteComplete(f.owner->wires.front()),"owner still incomplete");
  if(f.replica) {
    // One physical shared input cannot keep its old disconnected incoming
    // arc in another binding after the owner changes to the new input.
    require(fpga::isRouteComplete(f.replica->wires.front()),"terminal replica was not repaired together");
  }
  for(const auto &binding:f.signal->routes)
    require(binding.owner && fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
        "repair left a same-source binding incomplete");
}
