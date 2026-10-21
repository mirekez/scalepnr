#include "Tech.h"
#include "Wire.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Numeric reduction of the run-4 Docking approach: the requester has a
// completed trunk and reaches a point four columns from its free input.
// Its candidate suffix turns north, south, west, then south again. Several
// independent routes own the required transit resources; a retained route
// owns the final arrival. Node labels deliberately carry no device vocabulary.
// This fixture tests production Fanouts/Docking and physical ownership, not
// an imitation of their acceptance predicates.
// The focused replay confirmed the contested owner's seven-fragment shape:
// source pin, takeoff, GAP, incoming hop, same-CB hop, GAP, sink pin.
// In the original coordinates the same-CB hop is (57,36) DST26->SRC23->DST23;
// here it is translated by (-50,-33). Every ownership flag is produced by
// real commits and two partial cuts, not copied into a fabricated busy mask.
struct Fixture {
  static constexpr int output=300, input=95, width=10, height=10;
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent, *primitive;
  std::vector<rtl::Net *> nets;
  rtl::Inst *source, *trunk, *sink;
  rtl::Net *signal;
  rtl::Net *continuation_owner=nullptr;

  Fixture(bool long_approach=false) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=width; device.size_height=height; device.grid_spec.size={width,height};
    device.tile_grid.resize(width*height);
    device.tile_types.reserve(width*height); device.cb_types.reserve(width*height);
    for (int y=0;y<height;++y) for (int x=0;x<width;++x) {
      auto &t=*device.getTile(x,y); auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*width+x; cb.name="matrix_"+std::to_string(y*width+x);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(y*width+x); type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(output);
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(input);
      t.coord=t.cb_coord={x,y}; t.name=t.coord;
      t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      cb.local_output_nodes.setBit(output); cb.local_input_nodes.setBit(input);
      name(t,fpga::CB_NODE_LOCAL,output); name(t,fpga::CB_NODE_LOCAL,input);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="docking_owner_top";
    parent->is_blackbox=false; parent->nets.reserve(32);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("driver",{8,8}); trunk=cell("trunk",{8,7}); sink=cell("sink",{3,6});
    signal=connect(*source,*trunk); connect(*source,*sink);
    publish(*signal,*source,*trunk,{pin({8,8},true),
        edge({8,8},{7,7},output,400,3079,true),
        edge({7,7},{8,7},3079,401,80),terminal({8,7},80),pin({8,7},false)});
    if (long_approach) {
      // Both frontiers must participate: 13 hops exceed either half's
      // default ten-hop bound. All positions are inside the Docking window.
      // A foreign transit occupies the first useful exit. Backward-only
      // provisional preemption cannot reach it; forward provisional search
      // must cross it speculatively, without cutting before a complete join.
      const std::vector<fpga::Coord> path{{7,7},{7,6},{7,5},{7,4},{7,3},
          {6,3},{5,3},{4,3},{3,3},{3,4},{3,5},{4,5},{4,6},{3,6}};
      for (size_t i=0;i+1<path.size();++i)
        edge(path[i],path[i+1],i==0?3079:599+i,500+i,600+i);
      terminal(path.back(),612);
      // Keep one genuinely free but useless branch exit. Saturated-branch
      // selection must not hide the Docking regression being exercised.
      edge({7,7},{7,9},3079,97,97); terminal({7,9},97);
      return;
    }
    edge({7,7},{7,1},3079,162,162);
    // The observed forward position also had a free, unsuccessful departure.
    // Preserve it so branch selection does not reject a saturated start before
    // Docking runs; this test is about Docking's owner checks, not that filter.
    edge({7,7},{7,9},3079,97,97);
    terminal({7,9},97);
    edge({7,1},{7,2},162,21,25);
    edge({7,2},{7,3},25,22,26);
    edge({7,3},{7,4},26,19,22);
    edge({7,4},{3,4},22,3079,3075);
    edge({3,4},{3,6},3075,67,67);
    terminal({3,6},67);
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+
        std::to_string((unsigned(id)*2654435761U)^(unsigned(kind)*101U)));
  }
  fpga::Wire edge(fpga::Coord a,fpga::Coord b,int local,int src,int dst,bool takeoff=false) {
    auto &t=*device.getTile(a.x,a.y); auto &to=*device.getTile(b.x,b.y); auto &cb=*t.cb_type;
    auto role=takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    name(t,role,local); name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(takeoff) cb.local_src[local].jump.setBit(src); else cb.dst_src[local].jump.setBit(src);
    cb.rememberConnName(role,local,fpga::CB_NODE_SRC,src,
        *cb.nodeName(role,local),*cb.nodeName(fpga::CB_NODE_SRC,src));
    fpga::CBType::ResolvedJump jump{}; jump.delta=b-a; jump.target_tile_coord=true;
    jump.target_cb_type_id=to.cb_type->type_id; jump.dsts.jump.setBit(dst);
    // Reusing a physical SRC must not duplicate its destination mapping.
    if(cb.dst_by_src[src].empty()) cb.dst_by_src[src].push_back(jump);
    fpga::Wire w; w.from=a; w.to=b; w.local=local; w.jump=w.route_jump=src; w.dst=dst;
    w.pos=takeoff?0:1;
    w.from_wire_name=*cb.nodeName(role,local); w.src_wire_name=*cb.nodeName(fpga::CB_NODE_SRC,src);
    w.dst_wire_name=*to.cb_type->nodeName(fpga::CB_NODE_DST,dst);
    return w;
  }
  fpga::Wire terminal(fpga::Coord c,int dst) {
    auto &t=*device.getTile(c.x,c.y); auto &cb=*t.cb_type; name(t,fpga::CB_NODE_DST,dst);
    cb.dst_local[dst].local.setBit(input);
    cb.rememberConnName(fpga::CB_NODE_DST,dst,fpga::CB_NODE_LOCAL,input,
        *cb.nodeName(fpga::CB_NODE_DST,dst),*cb.nodeName(fpga::CB_NODE_LOCAL,input));
    fpga::Wire w; w.from=w.to=c; w.local=dst; w.pos=1;
    w.dst_wire_name=*cb.nodeName(fpga::CB_NODE_DST,dst); return w;
  }
  fpga::Wire pin(fpga::Coord c,bool out) {
    fpga::Wire w; w.type=fpga::Wire::WIRE_TILE_PIN; w.from=w.to=w.resource=c;
    w.local=out?output:input; w.pin_dir=out?1:0; w.pos=out?0:1;
    w.cell_type="LUT6"; w.port=out?"O":"I0"; return w;
  }
  rtl::Inst *cell(const std::string &label,fpga::Coord c) {
    auto &type=cells.emplace_back(); type.name=label; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) { auto &c=i.conns.emplace_back(); c.port_ref.set(&p); c.inst_ref.set(&i); }
    device.getTile(c.x,c.y)->assign(&i); return &i;
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
  void publish(rtl::Net &n,rtl::Inst &a,rtl::Inst &b,std::vector<fpga::Wire> route) {
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    require(pnr::commitPreparedRoute(route),"fixture route cannot be committed");
    b.wires.push_back(std::move(route));
    size_t index=fpga::attachNetRoute(n,b,b.wires.size()-1,&a,&b,"O","I0",b.makeName()+".signal");
    fpga::registerNetRouteTiles(n,b.wires.back(),index);
  }
  rtl::Net *owner(const std::string &label,fpga::Coord from,fpga::Coord to,
             std::vector<fpga::Wire> middle,bool partial=false) {
    auto *a=cell(label+"_driver",from), *b=cell(label+"_sink",to); auto *n=connect(*a,*b);
    std::vector<fpga::Wire> route{pin(from,true)};
    route.insert(route.end(),middle.begin(),middle.end());
    // Create a real completed owner, then truncate it through the production
    // cut routine. This reserves its retained landing without fabricating a
    // busy mask or an owns_landing flag in the fixture.
    if(partial) route.push_back(edge(middle.back().to,to,middle.back().dst,900,901));
    route.push_back(terminal(to,route.back().dst)); route.push_back(pin(to,false));
    publish(*n,*a,*b,std::move(route));
    if(partial) {
      std::vector<fpga::RouteCutNode> cuts{{middle.back().to,fpga::CB_NODE_SRC,900}};
      require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cuts),cuts,true),
          "cannot form retained owner prefix and endpoint island");
    }
    return n;
  }
  void occupy(bool same_tile_continuation,bool grounding_control) {
    owner("owner_a",{0,0},{7,0},{edge({0,0},{7,7},output,101,3073,true),
        edge({7,7},{7,1},3073,162,162),edge({7,1},{7,0},162,162,162)});
    owner("owner_b",{0,1},{9,2},{edge({0,1},{7,1},output,102,20,true),
        edge({7,1},{7,2},20,21,25),edge({7,2},{9,2},25,546,546)});
    std::vector<fpga::Wire> retained{edge({0,6},{4,6},output,103,1024,true),
        edge({4,6},{6,2},1024,106,194),edge({6,2},{7,2},194,107,195),
        edge({7,2},{7,3},195,22,26)};
    if(same_tile_continuation) {
      // Same-CB SRC->DST continuity is a fabric hop, not Grounding. Its
      // endpoints coincide geometrically, but neither endpoint is LOCAL.
      retained.push_back(edge({7,3},{7,3},26,23,23));
    }
    continuation_owner=owner("owner_c",{0,6},grounding_control?fpga::Coord{7,3}:fpga::Coord{3,5},
        std::move(retained),!grounding_control);
    if(!grounding_control) {
      // A second cut retains the source prefix AND this downstream island,
      // just as the captured owner has a gap before its (7,2)->(7,3) hop.
      std::vector<fpga::RouteCutNode> cuts{{{4,6},fpga::CB_NODE_SRC,106}};
      require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cuts),cuts,true),
          "cannot retain the intermediate continuation island");
    }
    owner("owner_d",{0,3},{0,4},{edge({0,3},{7,4},output,104,67,true),
        edge({7,4},{3,4},67,3079,3075),edge({3,4},{0,4},3075,3846,3842)});
    owner("owner_e",{0,5},{9,6},{edge({0,5},{3,4},output,105,67,true),
        edge({3,4},{3,6},67,67,67)},true);
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream out; const auto a=fpga::auditTileCongestion(t,out,nets);
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&!a.stale_registrations,out.str());
    }
  }
  auto leases() const {
    std::vector<std::array<NodeMask,5>> result;
    for(const auto &t:device.tile_grid)
      result.push_back({t.cb.src.jump,t.cb.dst.jump,t.cb.joint.jump,t.cb.local.local,t.pin_state.leased_nodes});
    return result;
  }
  bool run() {
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    auto &r=tech.route; r.fanout_stage=true; r.complete_routes_only=true; r.route_recursion_budget=5;
    pnr::RouteDesign::RouteTask task{.from=source,.to=sink,.net=signal,
        .from_port="O",.to_port="I0",.net_name="request_owner",.fanout=true};
    return r.routeFanoutTask(task);
  }
};
}

void fanout_docking_owner_regression(int mode) {
  if (mode==11) {
    Fixture f;
    auto add_branch=[&](rtl::Inst *sink, std::vector<fpga::Wire> route, size_t shared) {
      for (size_t i=0;i<shared;++i) route[i].shared=true;
      for (auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
      std::vector<fpga::Wire> suffix(route.begin()+shared,route.end());
      require(pnr::commitPreparedRoute(suffix),"cannot commit nested fork suffix");
      sink->wires.push_back(std::move(route));
      auto b=fpga::attachNetRoute(*f.signal,*sink,0,f.source,sink,"O","I0",sink->makeName());
      fpga::registerNetRouteTiles(*f.signal,sink->wires.front(),b);
      return b;
    };
    auto *a=f.cell("replaced_branch",{4,7}); f.connect(*f.source,*a);
    auto *b=f.cell("surviving_branch",{4,8}); f.connect(*f.source,*b);
    const auto &trunk=f.signal->routes.front();
    const auto &old=trunk.owner->wires[trunk.route_index];
    std::vector<fpga::Wire> first(old.begin(),old.begin()+2);
    auto fork=f.edge({7,7},{6,7},3079,450,451); fork.pos=2; fork.owns_dst=false;
    first.push_back(fork);
    first.push_back(f.edge({6,7},{5,7},451,452,453));
    first.push_back(f.edge({5,7},{4,7},453,454,455));
    first.push_back(f.terminal({4,7},455)); first.push_back(f.pin({4,7},false));
    const auto replaced=add_branch(a,std::move(first),2);
    std::vector<fpga::Wire> second(a->wires.front().begin(),a->wires.front().begin()+3);
    fork=f.edge({6,7},{6,8},451,456,457); fork.pos=2; fork.owns_dst=false;
    second.push_back(fork);
    second.push_back(f.edge({6,8},{4,8},457,458,459));
    second.push_back(f.terminal({4,8},459)); second.push_back(f.pin({4,8},false));
    const auto survivor=add_branch(b,std::move(second),3);
    f.audit();
    // The landing at (6,7) belongs to the old route's NEXT fragment, not
    // its shared prefix. The survivor forks there without owning the DST.
    require(!a->wires.front()[2].owns_landing && a->wires.front()[3].owns_dst &&
        !b->wires.front()[3].owns_dst,"nested fork ownership shape changed");
    const std::vector<fpga::RouteCutNode> cuts{{{5,7},fpga::CB_NODE_SRC,454}};
    require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cuts),cuts,true),
        "cannot cut old branch beyond the surviving fork");
    f.device.getTile(5,7)->cb_type->dst_src[453].jump.clearBit(454);
    f.edge({8,8},{0,8},Fixture::output,460,461,true);
    f.edge({0,8},{4,7},461,462,463); f.terminal({4,7},463);
    for (auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    require(fpga::repairNetRouteGaps(*f.signal,replaced),"source replacement failed");
    require(fpga::isRouteComplete(a->wires.front()) && fpga::isRouteComplete(b->wires.front()),
        "source replacement disconnected a sibling");
    require(f.device.getTile(6,7)->cb.dst.jump.testBit(451),
        "source replacement cleared the surviving fork's branching DST");
    f.audit();
    require(fpga::unrouteNetRoute(*f.signal,survivor),"cannot tear down surviving fork");
    require(!f.device.getTile(6,7)->cb.dst.jump.testBit(451),"surviving fork teardown leaked DST");
    f.audit();
    std::puts("PASSED: source replacement transfers the next-fragment branching DST");
    return;
  }
  debug_level=1;
  if (mode==9) {
    Fixture f;
    auto *sink=f.cell("retained_fork_sink",{4,7});
    f.connect(*f.source,*sink);
    auto &trunk=f.signal->routes.front();
    const auto &original=trunk.owner->wires[trunk.route_index];
    std::vector<fpga::Wire> route(original.begin(),original.begin()+2);
    for (auto &w:route) w.shared=true;
    auto fork=f.edge({7,7},{6,7},3079,450,451);
    fork.pos=2; fork.owns_dst=false;
    route.push_back(fork);
    route.push_back(f.edge({6,7},{5,7},451,452,453));
    route.push_back(f.edge({5,7},{4,7},453,454,455));
    route.push_back(f.terminal({4,7},455)); route.push_back(f.pin({4,7},false));
    // The common prefix is already leased by the trunk. Commit only the
    // private suffix, then register the full shared-prefix binding, just as
    // production Fanouts does (commitPreparedRoute is not a sharing API).
    for (auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    std::vector<fpga::Wire> suffix(route.begin()+2,route.end());
    require(pnr::commitPreparedRoute(suffix),"cannot commit private fork suffix");
    sink->wires.push_back(std::move(route));
    const auto binding=fpga::attachNetRoute(*f.signal,*sink,0,f.source,sink,
        "O","I0","retained_fork");
    fpga::registerNetRouteTiles(*f.signal,sink->wires.front(),binding);
    f.audit();
    const std::vector<fpga::RouteCutNode> cuts{{{6,7},fpga::CB_NODE_SRC,452}};
    require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cuts),cuts,true),
        "cannot produce retained fork landing through a real partial cut");
    auto &retained=sink->wires.front();
    require(retained[2].pos==2 && !retained[2].owns_dst && retained[2].owns_landing &&
        retained[3].type==fpga::Wire::WIRE_GAP,"captured fork ownership shape not reproduced");
    require(f.device.getTile(6,7)->cb.dst.jump.testBit(451),"cut lost retained fork landing lease");
    f.audit();
    require(fpga::unrouteNetRoute(*f.signal,0),"cannot remove sibling trunk");
    f.audit();
    require(f.device.getTile(6,7)->cb.dst.jump.testBit(451),"sibling removal lost retained fork landing");
    require(fpga::repairNetRouteGaps(*f.signal,1),"cannot repair retained fork through its free old path");
    f.audit();
    require(fpga::unrouteNetRoute(*f.signal,1),"cannot remove repaired fork");
    f.audit();
    require(!f.device.getTile(6,7)->cb.dst.jump.testBit(451),"repaired route teardown leaked landing");
    std::puts("PASSED: retained fork landing survives cut, sibling removal and repair");
    return;
  }
  if (mode>=5) {
    Fixture f(true);
    if (mode!=6) {
      std::vector<fpga::Wire> occupied{
          f.edge({0,0},{7,7},Fixture::output,101,800,true),
          f.edge({7,7},{7,6},800,500,600)};
      if (mode!=7) occupied.push_back(f.edge({7,6},{0,1},600,850,850));
      f.owner("forward_obstruction",{0,0},mode==7?fpga::Coord{7,6}:fpga::Coord{0,1},occupied);
      if (mode!=7)
        f.owner("middle_obstruction",{0,2},{0,3},{
            f.edge({0,2},{7,3},Fixture::output,102,801,true),
            f.edge({7,3},{6,3},801,504,604),
            f.edge({6,3},{0,3},604,851,851)});
    }
    if (mode==8) f.device.getTile(4,3)->cb_type->dst_src[606].jump.clearBit(507);
    if (mode==10) {
      // A later source-local alternative already solves the whole task.
      // New forward growth must not preempt the long first branch before
      // that later alternative has had its original chance to succeed.
      f.edge({8,8},{0,9},Fixture::output,910,910,true);
      f.edge({0,9},{3,6},910,911,911); f.terminal({3,6},911);
    }
    f.audit();
    const auto before=f.leases();
    const bool routed=f.run();
    f.audit();
    if (mode==7 || mode==8) {
      require(!routed,"forward provisional search ignored protected Grounding or missing topology");
      require(before==f.leases(),"unsuccessful provisional search changed leases");
    } else {
      require(routed,"occupied forward exit stranded the two-frontier Docking search");
      bool complete=false;
      for (auto &b:f.signal->routes) if (b.to==f.sink)
        complete=fpga::isRouteComplete(b.owner->wires[b.route_index]);
      require(complete,"provisional forward join did not commit a complete route");
      if (mode==10)
        for (size_t i=1;i<f.nets.size();++i) {
          const auto &b=f.nets[i]->routes.front();
          require(fpga::isRouteComplete(b.owner->wires[b.route_index]),
              "new early forward possibility displaced a successful later task alternative");
        }
    }
    std::puts("PASSED: two-frontier Docking, real owners, atomic leases");
    return;
  }
  Fixture f;
  if(mode!=1) f.occupy(mode==0||mode==3,mode==4);
  if(mode==3) require(fpga::unrouteNetRoute(*f.continuation_owner,0),
      "could not remove only the contested continuation owner");
  f.audit();
  if(mode==0) {
    auto &t=*f.device.getTile(7,3);
    auto owners=fpga::findNetOwnersByNode(t,fpga::CB_NODE_DST,26);
    require(owners.size()==1 && owners.front().net==f.continuation_owner,
        "the contested arrival must have exactly its real retained owner");
    auto &binding=f.continuation_owner->routes.front();
    const auto &route=binding.owner->wires[binding.route_index];
    require(route.size()==7 && route[0].type==fpga::Wire::WIRE_TILE_PIN &&
        route[1].owns_landing && route[2].type==fpga::Wire::WIRE_GAP &&
        route[3].from.x==7 && route[3].from.y==2 && route[3].local==195 &&
        route[3].jump==22 && route[3].dst==26 &&
        route[5].type==fpga::Wire::WIRE_GAP && route[6].type==fpga::Wire::WIRE_TILE_PIN,
        "retained owner does not match the captured seven-fragment, two-gap shape");
    auto fragment=std::find_if(route.begin(),route.end(),[](const auto &w) {
      return w.type==fpga::Wire::WIRE_CROSSBAR && w.local==26 && w.jump==23;
    });
    require(fragment!=route.end(),"retained continuation fragment missing");
    const auto &continuation=*fragment;
    require(continuation.from.x==t.coord.x && continuation.from.y==t.coord.y &&
        continuation.to.x==t.coord.x && continuation.to.y==t.coord.y &&
        continuation.local==26 && continuation.jump==23 && continuation.dst==23 &&
        continuation.pos!=0 && !continuation.shared && continuation.owns_dst && continuation.owns_landing,
        "contested DST must belong to an owned same-Tile fabric hop, not an endpoint");
    require(binding.from->tile.peer!=&t && binding.to->tile.peer!=&t,
        "neither endpoint belongs to the contested Tile");
    require(fpga::canPreemptNetRouteFromNodes(*f.continuation_owner,0,{{t.coord,fpga::CB_NODE_DST,26}}),
        "the contested hop must be past the protected Takeoff");
    for(auto [kind,node]:{std::pair{fpga::CB_NODE_DST,26},
                         std::pair{fpga::CB_NODE_SRC,23},
                         std::pair{fpga::CB_NODE_DST,23}}) {
      const auto transit=fpga::findNetOwnersByNode(t,kind,node,true);
      require(transit.size()==1 && transit.front().net==f.continuation_owner,
          "same-Tile fabric continuation lost its transit owner");
    }
    const std::vector<fpga::RouteCutNode> cuts{{t.coord,fpga::CB_NODE_DST,26}};
    const auto released=fpga::previewNetRouteCuts(fpga::findNetRouteCutVictims(cuts),cuts,true);
    require(std::any_of(released.begin(),released.end(),[&](const auto &cut) {
      return cut.tile.x==t.coord.x && cut.tile.y==t.coord.y &&
          cut.type==fpga::CB_NODE_DST && cut.node==26;
    }),"partial-cut preview cannot release the contested same-Tile transit");
    std::printf("Owner proof: physical=%zu transit=%zu, same-Tile SRC/DST continuation, not LOCAL\n",
        owners.size(),fpga::findNetRoutesByNode(t,fpga::CB_NODE_DST,26,true).size());
  }
  const auto before=f.leases();
  const bool routed=f.run();
  f.audit();
  require(f.tech.route.route_stats.edge_rejected_no_name==0,
      "synthetic connection names are incomplete");
  for(auto &i:f.tech.design.top.insts)
    require(i.tile.peer && i.tile->coord==i.coord,"routing moved a fixed endpoint");
  if(!routed) require(before==f.leases(),"a failed Docking attempt changed physical leases");
  if(mode==4) {
    require(!routed,"Grounding control allowed preemption of a real endpoint");
    std::puts("PASSED: true Grounding remained protected and all leases unchanged");
    return;
  }
  require(routed,"production Fanouts rejected the transit-owned Docking path");
  bool complete=false;
  for(auto &b:f.signal->routes) if(b.to==f.sink)
    complete=fpga::isRouteComplete(b.owner->wires[b.route_index]);
  require(complete,"Fanouts reported success without a complete route");
  if(mode==0) {
    const auto &binding=f.continuation_owner->routes.front();
    const auto &retained=binding.owner->wires[binding.route_index];
    require(!retained.empty() && retained.front().type==fpga::Wire::WIRE_TILE_PIN &&
        retained.back().type==fpga::Wire::WIRE_TILE_PIN &&
        std::any_of(retained.begin(),retained.end(),[](const auto &w) {
          return w.type==fpga::Wire::WIRE_GAP;
        }),"preemption discarded the victim endpoints instead of keeping a repairable gap");
  }
  std::puts("PASSED: abstract Docking approach completed with consistent ownership");
}
