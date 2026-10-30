#include "Tech.h"
#include "Wire.h"

#include <algorithm>
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

// Reduced Example 1 failure, not a copy of a device database:
// - A completed short trunk has no usable branch exit at its destination.
// - The next fanout must leave the source through a different SRC.
// - The first free exit explores >200 states outside the Docking window.
// - A later free exit has a two-hop, unoccupied path to the destination.
// The production DFS consumes its GLOBAL state budget below the first exit
// and fails without trying the second. No random search, fabricated occupancy,
// timeout, preemption, or test-side routing algorithm is involved.
// All node names are abstract. Geometry only isolates the exhausted-budget
// mechanism; it does not claim to reproduce every obstruction of Example 1.
struct Fixture {
  static constexpr int trap_x=fpga::GROUNDING_RADIUS+1;
  static constexpr int width=trap_x+21, height=15;
  static constexpr int output=300, input=80, arrival=200;
  static constexpr int trunk_exit=100, trap_exit=110, good_exit=120;
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  std::list<Referable<pnr::RegBunch>> bunches;
  Referable<rtl::Module> *parent, *primitive;
  rtl::Inst *source, *first, *last;
  rtl::Net *signal;

  Fixture(int trap_length, bool omit_trap) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=width; device.size_height=height;
    device.grid_spec.size={width,height}; device.tile_grid.resize(width*height);
    device.tile_types.reserve(width*height); device.cb_types.reserve(width*height);
    for (int y=0;y<height;++y) for (int x=0;x<width;++x) {
      auto &t=*device.getTile(x,y);
      auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*width+x;
      cb.name="matrix_"+std::to_string(y*width+x);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(y*width+x);
      type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(output);
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(input);
      t.coord=t.cb_coord={x,y}; t.name=t.coord;
      t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      cb.local_output_nodes.setBit(output); cb.local_input_nodes.setBit(input);
      name(t,fpga::CB_NODE_LOCAL,output); name(t,fpga::CB_NODE_LOCAL,input);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true; // failRouting reports and throws normally.
    parent=&tech.design.modules.emplace_back(); parent->name="budget_top";
    parent->is_blackbox=false; parent->nets.reserve(16);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("driver",{0,1}); first=cell("trunk_sink",{1,1});
    last=cell("fanout_sink",{0,14});
    auto &net=parent->nets.emplace_back(); net.name="budget_signal";
    net.designators.push_back(1); signal=&net;
    source->conns[1].port_ref->designator=1;
    for (auto *sink:{first,last}) {
      sink->conns[0].port_ref->designator=1;
      sink->conns[0].set(&source->conns[1]);
      auto &b=bunches.emplace_back(); b.reg=sink; b.size=b.size_comb=1;
    }
    edge({0,1},{1,1},output,trunk_exit,true);
    terminal({1,1}); // no outgoing edge: this trunk cannot branch here.
    edge({0,1},{0,0},output,good_exit,true);
    edge({0,0},{0,14},arrival,130);
    terminal({0,14});
    // Keep the serpentine tree outside the configured grounding window.
    // Even entries are its spine; each odd entry is a one-hop dead-end leaf.
    // This forces real backtracking and spends >200 states at depth only ~100,
    // distinguishing a global state quota from a per-path 200-hop limit.
    // Every arrival is a different tile, so repetition prevention is irrelevant.
    std::vector<fpga::Coord> corridor;
    for (int y=1;y<=10;++y) for (int i=0;i<21;++i)
      corridor.push_back({y%2?trap_x+i:trap_x+20-i,y});
    require(trap_length>0 && trap_length%2==0 && trap_length<=int(corridor.size()),
        "invalid tree size");
    if (!omit_trap) edge({0,1},corridor[0],output,trap_exit,true);
    for (int i=0;i<trap_length;i+=2) {
      edge(corridor[i],corridor[i+1],arrival,140);
      // A physical landing must reach some resource to be a valid DST. This
      // unused local input is not our sink and offers no outgoing SRC.
      terminal(corridor[i+1]);
      if (i+2<trap_length) edge(corridor[i],corridor[i+2],arrival,141);
    }
    for (auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    for (int i=0;i<trap_length;i+=2) {
      auto &tile=*device.getTile(corridor[i].x,corridor[i].y);
      require(device.resolveJump(tile,140).tile==
          device.getTile(corridor[i+1].x,corridor[i+1].y),"leaf landing is not resolvable");
    }
    // Assert the search-order precondition using production ordering, rather
    // than silently depending on numeric SRC naming or an incidental cache.
    auto &cb=*device.getTile(0,1)->cb_type;
    const auto &order=cb.orderedSrcNodes(fpga::CB_NODE_LOCAL,output,{0,13});
    if (!omit_trap) require(std::find(order.begin(),order.end(),trap_exit)<
        std::find(order.begin(),order.end(),good_exit),"trap must precede alternate exit");
  }

  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+std::to_string(int(kind))+"_"+std::to_string(id));
  }
  void edge(fpga::Coord a,fpga::Coord b,int local,int src,bool takeoff=false) {
    auto &t=*device.getTile(a.x,a.y); auto &to=*device.getTile(b.x,b.y);
    auto &cb=*t.cb_type;
    name(t,takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,local);
    name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,arrival);
    if (takeoff) cb.local_src[local].jump.setBit(src);
    else cb.dst_src[local].jump.setBit(src);
    const auto role=takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    cb.rememberConnName(role,local,fpga::CB_NODE_SRC,src,
        *cb.nodeName(role,local),*cb.nodeName(fpga::CB_NODE_SRC,src));
    fpga::CBType::ResolvedJump jump{};
    jump.delta=b-a; jump.target_tile_coord=true;
    jump.target_cb_type_id=to.cb_type->type_id; jump.dsts.jump.setBit(arrival);
    cb.dst_by_src[src].push_back(jump);
  }
  void terminal(fpga::Coord c) {
    auto &cb=*device.getTile(c.x,c.y)->cb_type;
    cb.dst_local[arrival].local.setBit(input);
    cb.rememberConnName(fpga::CB_NODE_DST,arrival,fpga::CB_NODE_LOCAL,input,
        *cb.nodeName(fpga::CB_NODE_DST,arrival),*cb.nodeName(fpga::CB_NODE_LOCAL,input));
  }
  rtl::Inst *cell(const std::string &name,fpga::Coord c) {
    auto &type=cells.emplace_back(); type.name=name; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for (auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=i.cnt_outputs=1; i.conns.reserve(2);
    for (auto &p:type.ports) {
      auto &conn=i.conns.emplace_back(); conn.port_ref.set(&p); conn.inst_ref.set(&i);
    }
    device.getTile(c.x,c.y)->assign(&i); return &i;
  }
  bool complete(rtl::Inst *sink) {
    for (auto &b:signal->routes)
      if (b.to==sink && b.owner && b.route_index<b.owner->wires.size())
        return fpga::isRouteComplete(b.owner->wires[b.route_index]);
    return false;
  }
  void audit() {
    for (auto &tile:device.tile_grid) {
      std::ostringstream report;
      const auto a=fpga::auditTileCongestion(tile,report,{signal});
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
          !a.stale_registrations,report.str());
    }
  }
};
}

void fanout_search_budget_regression(int mode) {
  debug_level=1;
  // Controls: 1 removes only the misleading source exit; 2 shortens its dead
  // end. Both still run the same complete production Basic/Fanouts sequence.
  Fixture f(mode==2?20:210,mode==1);
  if (mode==3) {
    // No-solution control: exhaust the misleading exit without looping, and
    // do not turn a search quota into a persistent physical deadend mark.
    auto &cb=*f.device.getTile(0,1)->cb_type;
    cb.local_src[Fixture::output].jump.clearBit(Fixture::good_exit);
    cb.rebuildOutgoingSrcs();
  }
  // The failure audit must also work before a task has any route binding.
  // A read-only source-to-sink witness is not permission to install phantom
  // fragments/leases. The disconnected control must remain unreachable.
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.from_port="O";
  endpoints.to=f.last; endpoints.to_port="I0";
  fpga::Wire source,sink;
  source.type=sink.type=fpga::Wire::WIRE_TILE_PIN;
  source.from=f.source->coord; source.local=Fixture::output;
  sink.from=f.last->coord; sink.local=Fixture::input;
  std::ostringstream reach;
  require(fpga::auditUnroutedReachability(*f.signal,endpoints,source,sink,reach)==(mode!=3),
      "unrouted-task reachability disagrees with known topology: "+reach.str());
  require(f.signal->routes.empty(),"read-only reachability registered a fake route");
  f.audit();
  std::string failure;
  try { f.tech.route.routeDesign(f.bunches); }
  catch (const std::runtime_error &e) { failure=e.what(); }
  f.audit();
  require(f.complete(f.first),"Basic did not finish the reference trunk");
  require(f.source->tile.peer==f.device.getTile(0,1) &&
      f.first->tile.peer==f.device.getTile(1,1) &&
      f.last->tile.peer==f.device.getTile(0,14),"fixed endpoints moved");
  if (!failure.empty()) {
    require(mode==0 || mode==3,"control unexpectedly failed: "+failure);
    require(failure==(mode==3
        ? "routing failed: Moving destinations: current connection did not finish; no task/stage handoff"
        : "routing failed: Fanouts routing: current connection did not finish; no task/stage handoff"),
        "wrong failure: "+failure);
    require(!f.tech.route.route_stage_deadline_expired && !f.tech.route.route_stage_stagnated,
        "test hit time/stagnation limit instead of search limit");
    if (mode==3) {
      // A failed Fanout stays pending until recovery. Moving must attempt
      // wire-only routing despite fixed endpoints, then fail on the genuinely
      // disconnected input without changing the completed trunk or leases.
      require(f.tech.route.moving_stage && !f.tech.route.moving_sources_stage,
          "blocked Fanout did not reach Moving destinations");
      require(f.tech.route.route_stats.edge_accepted>0,"blocked search was never attempted");
    } else {
      require(f.tech.route.route_stats.edge_accepted==200,"expected 200-state exhausted search");
    }
    require(f.tech.route.route_stats.edge_rejected_no_name==0,
        "fixture has incomplete concrete connection names");
    require(!f.complete(f.last),"reported failure despite a complete fanout");
    require(!f.device.getTile(0,1)->cb.src.jump.testBit(Fixture::trap_exit),
        "failed wandering suffix was committed");
    if (mode==3) {
      require(!f.device.getTile(0,1)->cb.src_deadend.jump.testBit(Fixture::trap_exit),
          "budget exhaustion incorrectly marked a physical source deadend");
      std::puts("BLOCKED CONTROL PASSED: finite failure, no speculative leases or false source deadend");
      return;
    }
    std::puts("REPRODUCED: first exit consumed 200 states; valid alternate fanout unfinished; owners clean");
    // Deliberately RED until production routing can find the proven alternate.
    // Do not mark WILL_FAIL: that would turn the routing defect into success.
    throw std::runtime_error("fanout search exhausted before trying a routable alternate source exit");
  }
  require(mode!=3,"no-solution control unexpectedly routed the sink");
  require(f.complete(f.last),"fanout did not complete");
  require(f.device.getTile(0,1)->cb.src.jump.testBit(Fixture::good_exit),
      "fanout did not use the reference alternate exit");
  std::puts("PASSED: trunk and alternate fanout complete; fixed positions and node audit intact");
}
