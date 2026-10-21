#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <limits>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Reduced reproduction of the Example 1 run-4 failure:
//   1. Basic completes source -> first_sink; second_sink waits in Fanouts.
//   2. A later Basic Grounding preempts the trunk's transit, preserving a gap.
//   3. Its repair is classified as Fanout work, so Basic becomes empty.
//   4. The handoff rejects second_sink before attempting the queued repair.
//
// All names/topology are synthetic. The real contested transit was a SRC;
// here a JOINT shared with Grounding causes the same retained-gap callback and
// queue transition. This is a scheduler reproduction, not a chip-model import.
// Every busy resource has a real registered route owner. A two-hop bypass
// proves that the broken trunk can be repaired without moving endpoints or
// stealing the requester's Grounding. No scheduler logic is copied into tests.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device = fpga::Device::current();
  Referable<rtl::Module> *parent, *primitive;
  std::list<Referable<rtl::Cell>> cells;
  std::list<Referable<pnr::RegBunch>> bunches;
  rtl::Inst *source, *first, *second, *request_source, *request_sink;
  rtl::Inst *request_seed = nullptr;
  rtl::Net *signal, *request;
  bool takeoff;

  explicit Fixture(bool takeoff_case = false) : takeoff(takeoff_case) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    const int width=takeoff?12:5;
    device.size_width = width; device.size_height = 3; device.grid_spec.size = {width,3};
    device.tile_grid.resize(width*3);
    device.tile_types.reserve(width*3); device.cb_types.reserve(width*3);
    for (int y=0; y<3; ++y) for (int x=0; x<width; ++x) {
      auto &t = *device.getTile(x,y);
      auto &cb = device.cb_types.emplace_back();
      cb.type_id = cb.base_type_id = y*width+x;
      cb.name = "matrix_" + std::to_string(701+y*width+x);
      auto &type = device.tile_types.emplace_back();
      type.name = "resource_" + std::to_string(901+y*width+x);
      type.type = fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(300);
      type.pin_map.nodes[{"LUT6", "I0", 0}].setBit(80);
      t.coord=t.cb_coord={x,y}; t.name=t.coord;
      t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      name(t,fpga::CB_NODE_LOCAL,300); name(t,fpga::CB_NODE_LOCAL,80);
      cb.local_output_nodes.setBit(300); cb.local_input_nodes.setBit(80);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    // Match incremental runs, including failRouting's exception/report path.
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="handoff_top";
    parent->is_blackbox=false; parent->nets.reserve(2048);
    primitive=&tech.design.modules.emplace_back(); primitive->name="handoff_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_101",{0,1}); first=cell("first_sink_102",{3,1});
    second=cell("second_sink_103",{4,1});
    request_source=cell("request_source_201",takeoff?fpga::Coord{1,1}:fpga::Coord{0,0});
    request_sink=cell("request_sink_202",takeoff?fpga::Coord{10,1}:fpga::Coord{1,1});
    signal=connect(*source,*first); connect(*source,*second);
    if(takeoff) {request_seed=cell("request_seed_203",{0,0});connect(*request_source,*request_seed);}
    request=connect(*request_source,*request_sink);
    // Order is intentional: create the victim, then preempt it, with a new
    // sibling still deferred. The nearer first sink remains the Generic seed.
    const std::vector<rtl::Inst *> order=takeoff
        ?std::vector<rtl::Inst *>{first,request_seed,request_sink,second}
        :std::vector<rtl::Inst *>{first,request_sink,second};
    for (auto *sink : order) {
      auto &b=bunches.emplace_back(); b.reg=sink; b.size=1; b.size_comb=1;
    }
    edge({0,1},{1,1},300,100,200,true);
    edge({1,1},{2,1},200,246,20);
    auto &cross=*device.getTile(1,1)->cb_type;
    if(!takeoff) {
      cross.dst_src[200].jump.clearBit(246);
      cross.dst_joint[200].joint.setBit(33);
      cross.joint_src[33].jump.setBit(246);
      name(*device.getTile(1,1),fpga::CB_NODE_JOINT,33);
    }
    edge({2,1},{3,1},20,140,220);
    terminal({3,1},220);
    // The eventual new sibling branches beyond the repaired gap.
    edge({2,1},{4,1},20,141,221); terminal({4,1},221);
    // Longer free bypass: Basic initially takes the direct trunk, while
    // repair can avoid the contested JOINT after the requester owns it.
    edge({1,1},{1,2},200,180,260);
    if(takeoff) {
      // The request owns the old landing DST20 as well as SRC246. Rejoin
      // beyond it; a bypass into that same foreign input is not a solution.
      edge({1,2},{3,1},260,181,220);
      edge({3,1},{4,1},220,141,221);
    } else edge({1,2},{2,1},260,181,20);
    if(takeoff) {
      // The requester has a completed seed with no useful downstream fork.
      // Its distant suffix must take off through the victim's transit SRC.
      // Keep the sink outside immediate Grounding so this exercises the
      // ordinary Takeoff callback, not preemptDockingBridge's correct path.
      edge({1,1},{0,0},300,110,201,true); terminal({0,0},201);
      edge({1,1},{2,1},300,246,20,true);
      edge({2,1},{10,1},20,142,222); terminal({10,1},222);
    } else {
      edge({0,0},{1,1},300,110,201,true);
      cross.dst_joint[201].joint.setBit(33);
      cross.joint_local[33].local.setBit(80);
    }
    for (auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }

  static void name(fpga::Tile &t, fpga::CBNodeNameType kind, int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+
        std::to_string((unsigned(id)*2654435761U)^(unsigned(kind)*101U)));
  }
  void edge(fpga::Coord a,fpga::Coord b,int input,int src,int dst,bool takeoff=false) {
    auto &t=*device.getTile(a.x,a.y); auto &to=*device.getTile(b.x,b.y);
    auto &cb=*t.cb_type;
    name(t,takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,input);
    name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(takeoff) cb.local_src[input].jump.setBit(src);
    else cb.dst_src[input].jump.setBit(src);
    fpga::CBType::ResolvedJump jump{};
    jump.delta=b-a; jump.target_tile_coord=true;
    jump.target_cb_type_id=to.cb_type->type_id; jump.dsts.jump.setBit(dst);
    cb.dst_by_src[src].push_back(jump);
  }
  void terminal(fpga::Coord c,int dst) {
    device.getTile(c.x,c.y)->cb_type->dst_local[dst].local.setBit(80);
  }
  rtl::Inst *cell(const std::string &name,fpga::Coord c) {
    auto &type=cells.emplace_back(); type.name=name; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=0; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=1; i.cnt_outputs=1;
    i.conns.reserve(2);
    for(auto &p:type.ports) {
      auto &conn=i.conns.emplace_back(); conn.port_ref.set(&p); conn.inst_ref.set(&i);
    }
    device.getTile(c.x,c.y)->assign(&i);
    return &i;
  }
  rtl::Net *connect(rtl::Inst &a,rtl::Inst &b) {
    auto &out=a.conns[1]; auto &in=b.conns[0];
    int id=out.port_ref->designator;
    if(id<0) {
      id=parent->nets.size()+1;
      auto &net=parent->nets.emplace_back(); net.name="signal_"+std::to_string(id);
      net.designators.push_back(id); out.port_ref->designator=id;
    }
    in.port_ref->designator=id; in.set(&out); return &parent->nets[id-1];
  }
  std::vector<fpga::Wire> *route(rtl::Net &net,rtl::Inst &sink) {
    for(auto &b:net.routes) if(b.to==&sink && b.owner && b.route_index<b.owner->wires.size())
      return &b.owner->wires[b.route_index];
    return nullptr;
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream text;
      auto a=fpga::auditTileCongestion(t,text,{signal,request});
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
              !a.stale_registrations,text.str());
    }
  }
  void checkPreemption() {
    require(signal->routes.size()>=1,"fixture never created the trunk");
    const auto &name=signal->routes.front().route_name;
    const auto &blockers=tech.route.preempted_route_blockers;
    auto it=blockers.find(name);
    require(it!=blockers.end() && !it->second.empty(),
            "fixture did not exercise trunk preemption");
  }
  void checkComplete() {
    for(auto pair:{std::pair{signal,first},std::pair{signal,second},
                   std::pair{request,request_sink}}) {
      auto *w=route(*pair.first,*pair.second);
      require(w && fpga::isRouteComplete(*w),"scheduler left an incomplete connection");
    }
    if(request_seed) {
      auto *seed=route(*request,*request_seed);
      require(seed && fpga::isRouteComplete(*seed),"requester's seed was damaged");
    }
    require(device.getTile(1,1)->cb.src.jump.testBit(180) &&
            device.getTile(1,2)->cb.src.jump.testBit(181),"repair did not use the bypass");
    require(source->tile.peer==device.getTile(0,1) && first->tile.peer==device.getTile(3,1) &&
            second->tile.peer==device.getTile(4,1) && request_source->tile.peer==device.getTile(takeoff?1:0,takeoff?1:0) &&
            request_sink->tile.peer==device.getTile(takeoff?10:1,1),"routing moved a fixed endpoint");
    checkPreemption();
    audit();
  }
};
} // namespace

static void run_handoff_regression(bool repair_control,bool takeoff) {
  debug_level=1;
  Fixture f(takeoff);
  std::string failure;
  try { f.tech.route.routeDesign(f.bunches); }
  catch(const std::runtime_error &e) { failure=e.what(); }
  if(!failure.empty()) {
    require(failure=="routing failed: Fanout missing Generic trunk",
            "unexpected fixture failure: "+failure);
    auto *trunk=f.route(*f.signal,*f.first);
    auto *replacement=f.route(*f.request,*f.request_sink);
    require(trunk && fpga::hasRouteGaps(*trunk),"failure did not retain the preempted trunk gap");
    require(replacement && fpga::isRouteComplete(*replacement),"preempting Grounding did not complete");
    require(!f.route(*f.signal,*f.second),"deferred sibling was already attempted");
    require(!f.tech.route.sourceTreeHasCompleteExit(*f.source,"O"),"source still has a complete trunk");
    f.checkPreemption();
    f.audit();
    std::puts("REPRODUCED: completed replacement + gapped trunk + untouched sibling; all Tile owners valid");
    if(repair_control) {
      // Control only: manually run the repair the scheduler failed to attempt.
      // The ordinary regression never changes task order or repairs by hand.
      auto &b=f.signal->routes.front();
      pnr::RouteDesign::RouteTask task{.from=b.from,.to=b.to,.net=f.signal,
          .from_port=b.from_port,.to_port=b.to_port,.net_name=b.route_name,
          .fanout=true,.endpoints_prepared=true};
      pnr::RouteDesign::RouteTask sibling{.from=f.source,.to=f.second,.net=f.signal,
          .from_port="O",.to_port="I0",
          .net_name=f.second->conns[0].makeNetName(nullptr,std::numeric_limits<size_t>::max()),
          .fanout=true};
      std::vector<pnr::RouteDesign::RouteTask> work{task,sibling};
      f.tech.route.fanout_stage=true;
      auto result=f.tech.route.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Fanout,
                                             work,work.size(),200);
      require(result.completed==2,"control: repair then sibling did not complete");
      f.checkComplete();
      std::puts("CONTROL PASSED: gap repaired, sibling routed, replacement and ownership intact");
      return;
    }
    throw std::runtime_error("trunk repair was deferred past its sibling's Generic-trunk barrier: "+failure);
  }
  // Full-flow contract: repair the existing trunk, then route its sibling.
  f.checkComplete();
}

void fanout_trunk_handoff_regression(bool repair_control) {
  run_handoff_regression(repair_control,false);
}

void fanout_takeoff_handoff_regression(bool repair_control) {
  run_handoff_regression(repair_control,true);
}
