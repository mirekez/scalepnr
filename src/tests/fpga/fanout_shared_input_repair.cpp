#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <initializer_list>
#include <limits>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Reduced incremental-run reproduction of a Fanouts shared-input dependency:
//
//   source -> (1,1)/DST200 -> JOINT33 -> SRC246 -> (2,1) -> owner
//                  |                                      (3,1)/LOCAL80
//                  +-> (1,2) --------> (2,1)                 ^
//                       free repair bypass                  |
//   source -------------------------------------- new sibling (same input)
//   source -------------------------------------- stable sibling (4,2)
//
// Run 1 completes the owner and stable sibling. Run 2 Basic creates a second
// source's trunk, then Fanouts routes its fanout through JOINT33 to LOCAL80
// in (1,1). Its Grounding preempts the owner's transit and retains the owner's
// downstream input lease. The next new sibling needs exactly that input.
// Another complete source branch survives, so the missing-trunk handoff guard
// cannot help. The owner repair must precede shared-terminal attachment.
//
// This isolates the recorded scheduler dependency, not the full chip geometry:
// the real transit cut was a SRC, here it is a JOINT. Both use real preemption,
// retained gaps and queued repairs. No busy masks or route gaps are fabricated.
// All node names are abstract; the bypass proves feasibility after the cut.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device = fpga::Device::current();
  Referable<rtl::Module> *parent, *primitive;
  std::list<Referable<rtl::Cell>> cells;
  std::list<Referable<pnr::RegBunch>> bunches;
  rtl::Inst *source, *owner, *stable, *sibling, *request_source, *request_trunk, *request_sink;
  rtl::Net *signal, *request=nullptr;

  explicit Fixture(bool with_bypass=true) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=5; device.size_height=3; device.grid_spec.size={5,3};
    device.tile_grid.resize(15); device.tile_types.reserve(15); device.cb_types.reserve(15);
    for (int y=0; y<3; ++y) for (int x=0; x<5; ++x) {
      auto &t=*device.getTile(x,y);
      auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*5+x;
      cb.name="matrix_"+std::to_string(701+y*5+x);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(901+y*5+x);
      type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(300);
      // Two physical cells deliberately share a routing input, as in the
      // captured failure. They are not assigned to the same physical slot.
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(80);
      type.pin_map.nodes[{"LUT6","I0",1}].setBit(80);
      t.coord=t.cb_coord={x,y}; t.name=t.coord;
      t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      name(t,fpga::CB_NODE_LOCAL,300); name(t,fpga::CB_NODE_LOCAL,80);
      cb.local_output_nodes.setBit(300); cb.local_input_nodes.setBit(80);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="shared_input_top";
    parent->is_blackbox=false; parent->nets.reserve(64);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_101",{0,1}); owner=cell("owner_102",{3,1});
    stable=cell("stable_103",{4,2}); sibling=cell("sibling_104",{3,1},1);
    request_source=cell("request_source_201",{0,0});
    request_trunk=cell("request_trunk_202",{1,0});
    request_sink=cell("request_sink_203",{1,1});
    signal=connect(*source,*owner); connect(*source,*stable);

    edge({0,1},{1,1},300,100,200,true);
    edge({1,1},{2,1},200,246,20);
    auto &cross=*device.getTile(1,1)->cb_type;
    cross.dst_src[200].jump.clearBit(246);
    cross.dst_joint[200].joint.setBit(33);
    cross.joint_src[33].jump.setBit(246);
    name(*device.getTile(1,1),fpga::CB_NODE_JOINT,33);
    edge({2,1},{3,1},20,140,220); terminal({3,1},220);
    if(with_bypass) {
      edge({1,1},{1,2},200,180,260);
      edge({1,2},{2,1},260,181,20);
    }
    // This sibling must remain complete after the cut, just like the other
    // completed branches in the real failure. It does not use JOINT33.
    edge({0,1},{4,2},300,102,222,true); terminal({4,2},222);
    edge({0,0},{1,0},300,110,210,true); terminal({1,0},210);
    edge({1,0},{1,1},210,111,201);
    cross.dst_joint[201].joint.setBit(33);
    cross.joint_local[33].local.setBit(80);
    for (auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
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
  rtl::Inst *cell(const std::string &label,fpga::Coord c,int pos=0) {
    auto &type=cells.emplace_back(); type.name=label; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=pos; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=1; i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {
      auto &conn=i.conns.emplace_back(); conn.port_ref.set(&p); conn.inst_ref.set(&i);
    }
    device.getTile(c.x,c.y)->assign(&i); return &i;
  }
  rtl::Net *connect(rtl::Inst &a,rtl::Inst &b) {
    auto &out=a.conns[1]; auto &in=b.conns[0]; int id=out.port_ref->designator;
    if(id<0) {
      id=parent->nets.size()+1;
      auto &net=parent->nets.emplace_back(); net.name="signal_"+std::to_string(id);
      net.designators.push_back(id); out.port_ref->designator=id;
    }
    in.port_ref->designator=id; in.set(&out); return &parent->nets[id-1];
  }
  void run(std::initializer_list<rtl::Inst *> sinks) {
    bunches.clear();
    for(auto *sink:sinks) {
      auto &b=bunches.emplace_back(); b.reg=sink; b.size=1; b.size_comb=1;
    }
    tech.route.routeDesign(bunches);
  }
  std::vector<fpga::Wire> *route(rtl::Net &net,rtl::Inst &sink) {
    for(auto &b:net.routes) if(b.to==&sink && b.owner && b.route_index<b.owner->wires.size())
      return &b.owner->wires[b.route_index];
    return nullptr;
  }
  bool complete(rtl::Net &net,rtl::Inst &sink) {
    auto *w=route(net,sink); return w && fpga::isRouteComplete(*w);
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream report;
      std::vector<rtl::Net *> nets{signal};
      if(request) nets.push_back(request);
      auto a=fpga::auditTileCongestion(t,report,nets);
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
              !a.stale_registrations,report.str());
    }
  }
  void checkComplete() {
    require(complete(*signal,*owner) && complete(*signal,*stable) &&
            complete(*signal,*sibling) && complete(*request,*request_trunk) &&
            complete(*request,*request_sink),"shared-input dependency left an incomplete route");
    require(device.getTile(1,1)->cb.src.jump.testBit(180) &&
            device.getTile(1,2)->cb.src.jump.testBit(181),"owner repair did not use the bypass");
    const auto &repaired=*route(*signal,*owner);
    const auto &reused=*route(*signal,*sibling);
    // Full-flow ownership reconciliation can promote shared fragments because
    // the two resource pins have different slots. Check physical reuse, not
    // that transient ownership flag (the repair-first batch control stops
    // before reconciliation). No extra fabric path may have been consumed.
    require(reused.size()==repaired.size(),"new sibling did not reuse the repaired path");
    for(size_t i=0;i<reused.size();++i) {
      const auto &a=repaired[i]; const auto &b=reused[i];
      require(a.type==b.type && a.from.x==b.from.x && a.from.y==b.from.y &&
              a.to.x==b.to.x && a.to.y==b.to.y && a.local==b.local &&
              a.jump==b.jump && a.dst==b.dst && a.joint==b.joint && a.joint2==b.joint2,
              "new sibling differs from the repaired owner's physical route");
    }
    require(reused.back().type==fpga::Wire::WIRE_TILE_PIN && reused.back().local==80,
            "new sibling did not reuse the repaired input");
    require(source->tile.peer==device.getTile(0,1) && owner->tile.peer==device.getTile(3,1) &&
            stable->tile.peer==device.getTile(4,2) && sibling->tile.peer==device.getTile(3,1) &&
            request_source->tile.peer==device.getTile(0,0) && request_trunk->tile.peer==device.getTile(1,0) &&
            request_sink->tile.peer==device.getTile(1,1) && owner->pos==0 && sibling->pos==1,
            "routing moved a fixed endpoint or overlapped the shared-input cells");
    audit();
  }
};
} // namespace

void fanout_shared_input_repair_regression(bool repair_control, bool blocked_control) {
  debug_level=1;
  Fixture f(!blocked_control);
  f.run({f.owner,f.stable});
  require(f.complete(*f.signal,*f.owner) && f.complete(*f.signal,*f.stable),
          "initial routes did not complete");
  require(f.device.getTile(1,1)->cb.joint.jump.testBit(33),"initial owner did not use contested transit");
  f.audit();
  f.request=f.connect(*f.request_source,*f.request_trunk);
  f.connect(*f.source,*f.sibling); f.connect(*f.request_source,*f.request_sink);
  std::string failure;
  try { f.run({f.request_trunk,f.request_sink,f.sibling}); }
  catch(const std::runtime_error &e) { failure=e.what(); }
  if(!failure.empty()) {
    const std::string expected=blocked_control
        ? "routing failed: Moving destinations: fixed destination in-place routing failed"
        : "routing failed: Fanouts routing: current connection did not finish; no task/stage handoff";
    require(failure==expected,
            "unexpected fixture failure: "+failure);
    auto *owner=f.route(*f.signal,*f.owner);
    require(owner && fpga::hasRouteGaps(*owner),"preempted owner has no retained gap");
    require(f.complete(*f.request,*f.request_sink),"preempting fanout did not complete");
    require(f.complete(*f.signal,*f.stable) &&
            f.tech.route.sourceTreeHasCompleteExit(*f.source,"O"),"no surviving complete source branch");
    require(f.device.getTile(3,1)->isPinNodeLeased(80),"shared input lease was not retained");
    const auto owners=fpga::findNetOwnersByNode(*f.device.getTile(3,1),fpga::CB_NODE_LOCAL,80);
    require(owners.size()==1 && owners.front().net==f.signal &&
            f.signal->routes[owners.front().binding_index].to==f.owner,
            "shared input is not owned exclusively by the incomplete sibling");
    bool pending=false;
    for(auto &task:f.tech.route.pending_route_todo)
      if(task.net==f.signal && task.to==f.owner) pending=true;
    // Soft Fanout failure hands the repair to Moving's active queue instead
    // of terminating while it is still in the pending queue.
    if(blocked_control) {
      require(f.tech.route.moving_stage && !f.tech.route.moving_sources_stage,
              "blocked shared-input repair did not reach Moving destinations");
      for(auto &task:f.tech.route.route_todo)
        if(task.net==f.signal && task.to==f.owner) pending=true;
    }
    require(pending,"owner repair is not pending behind new sibling");
    require(!f.complete(*f.signal,*f.sibling),"new sibling was already routed");
    f.audit();
    if(blocked_control) {
      // Fixed placement still permits in-place repair. With no free bypass,
      // that attempt must fail without releasing the retained input lease.
      std::puts("BLOCKED CONTROL PASSED: owner repair failed explicitly; retained owners and completed routes intact");
      return;
    }
    std::printf("shared-input state: owner=%s slot=%d, sibling=%s slot=%d, CB=(3,1)/LOCAL80; "
                "owner_gapped=1 repair_pending=1 stable_complete=1 input_owners=%zu\n",
                f.owner->cell_ref->name.c_str(),f.owner->pos,
                f.sibling->cell_ref->name.c_str(),f.sibling->pos,owners.size());
    std::puts("REPRODUCED: pending owner repair, retained input lease, surviving complete branch, failed new sibling; audit clean");
    if(repair_control) {
      // Control only: execute the pending repair first. The normal regression
      // requires the unmodified full-flow scheduler to handle the dependency.
      pnr::RouteDesign::RouteTask repair;
      for(auto &task:f.tech.route.pending_route_todo)
        if(task.net==f.signal && task.to==f.owner) repair=task;
      repair.fanout=true;
      pnr::RouteDesign::RouteTask sibling{.from=f.source,.to=f.sibling,.net=f.signal,
          .from_port="O",.to_port="I0",
          .net_name=f.sibling->conns[0].makeNetName(nullptr,std::numeric_limits<size_t>::max()),
          .fanout=true};
      std::vector<pnr::RouteDesign::RouteTask> work{repair,sibling};
      f.tech.route.fanout_stage=true;
      auto result=f.tech.route.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Fanout,work,2,200);
      require(result.completed==2,"control: repair then shared sibling failed");
      f.checkComplete();
      std::puts("CONTROL PASSED: repairing the owner first enables shared-input reuse; all routes complete and audit clean");
      return;
    }
    throw std::runtime_error("shared-input owner repair was deferred past dependent sibling: "+failure);
  }
  require(!blocked_control,"fixture without a repair path unexpectedly completed");
  f.checkComplete();
}

// The scheduler regression above supplies a bypass back to the OLD downstream
// DST. Example 1 subsequently failed in a different case: a free approach
// reaches a NEW DST, but must replace the incoming arc of the disconnected
// owner's private terminal JOINT. Repair permits this at LOCAL, not JOINT.
//
// This reduced topology uses genuine completed routes and real preemption:
//   old: source -> contested JOINT33 -> (2,1) -> DST220 -> JOINT56 -> LOCAL80
//   new: source -> (0,2) -> (1,2) -> (2,2) -> DST221 ----^
// Both approaches exist before routing. The longer free approach must repair
// the victim after the old transit is cut, without disturbing other routes.
// The direct-input control removes only JOINT56 (both DSTs feed LOCAL80).
// All names remain abstract. This is a mechanism reproduction, not a copy of
// the full-chip search size or its unrelated retained branches.
void fanout_private_grounding_repair_regression(bool direct_input) {
  debug_level=1;
  Fixture f(false);
  auto &sink=*f.device.getTile(3,1);
  if (!direct_input) {
    sink.cb_type->dst_local[220].local.clearBit(80);
    sink.cb_type->dst_joint[220].joint.setBit(56);
    sink.cb_type->joint_local[56].local.setBit(80);
    Fixture::name(sink,fpga::CB_NODE_JOINT,56);
  }
  f.edge({0,1},{0,2},300,190,290,true);
  f.edge({0,2},{1,2},290,191,291);
  f.edge({1,2},{2,2},291,192,292);
  f.edge({2,2},{3,1},292,193,221);
  if (direct_input) f.terminal({3,1},221);
  else sink.cb_type->dst_joint[221].joint.setBit(56);
  for (auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();

  f.run({f.owner,f.stable});
  require(f.complete(*f.signal,*f.owner) && f.complete(*f.signal,*f.stable),
          "private-grounding fixture: initial routes did not complete");
  require(f.device.getTile(1,1)->cb.joint.jump.testBit(33) &&
          sink.cb.dst.jump.testBit(220) && !sink.cb.dst.jump.testBit(221),
          "private-grounding fixture: initial owner did not take the old approach");
  f.audit();

  f.request=f.connect(*f.request_source,*f.request_trunk);
  f.connect(*f.source,*f.sibling); f.connect(*f.request_source,*f.request_sink);
  std::string failure;
  try { f.run({f.request_trunk,f.request_sink,f.sibling}); }
  catch (const std::runtime_error &e) { failure=e.what(); }
  if (!failure.empty()) {
    require(failure=="routing failed: Fanout shared-input owner repair",
            "private-grounding fixture: unexpected failure: "+failure);
    auto *route=f.route(*f.signal,*f.owner);
    require(route && fpga::hasRouteGaps(*route),"failed owner has no retained gap");
    require(f.complete(*f.request,*f.request_sink) &&
            f.complete(*f.request,*f.request_trunk) && f.complete(*f.signal,*f.stable),
            "preempting or unaffected routes did not remain complete");
    require(!sink.cb.dst.jump.testBit(221),"alternative approach is not free");
    auto owners=fpga::findNetOwnersByNode(sink,fpga::CB_NODE_LOCAL,80);
    require(owners.size()==1 && owners.front().net==f.signal &&
            f.signal->routes[owners.front().binding_index].to==f.owner,
            "retained terminal is not exclusively owned by the repair victim");
    const size_t binding=owners.front().binding_index;
    if (!direct_input) {
      // Check ALL users, including replicas which do not own a lease: merely
      // finding one physical owner does not prove that changing an arc is safe.
      auto users=fpga::findNetRoutesByNode(sink,fpga::CB_NODE_JOINT,56);
      require(users.size()==1 && users.front().net==f.signal &&
              users.front().binding_index==binding,
              "terminal JOINT is not private to the disconnected owner");
    }
    std::ostringstream strict, relaxed;
    const bool strict_reached=fpga::auditNetRouteReachability(*f.signal,binding,strict,true,4096);
    const bool relaxed_reached=fpga::auditNetRouteReachability(*f.signal,binding,relaxed,false,4096);
    std::printf("Private grounding diagnostic: direct_input=%d strict_reached=%d relaxed_reached=%d\n%s%s",
                direct_input,strict_reached,relaxed_reached,strict.str().c_str(),relaxed.str().c_str());
    require(strict.str().find("limited=0")!=std::string::npos &&
            relaxed.str().find("limited=0")!=std::string::npos,
            "reachability hit its budget; the terminal-input diagnosis is inconclusive");
    require(relaxed_reached,"no free alternative approach exists even with private input replacement");
    if (!direct_input) require(!strict_reached,"failure was not caused by retained input preservation");
    f.audit();
    throw std::runtime_error("private grounding repair rejected a free alternative into its own retained terminal: "+failure);
  }
  require(f.complete(*f.signal,*f.owner) && f.complete(*f.signal,*f.sibling) &&
          f.complete(*f.signal,*f.stable) && f.complete(*f.request,*f.request_trunk) &&
          f.complete(*f.request,*f.request_sink),"private grounding repair left an incomplete route");
  require(sink.cb.dst.jump.testBit(221),"repair did not use the alternative approach");
  require(!sink.cb.dst.jump.testBit(220),"repair retained the obsolete private input approach");
  require(f.source->tile.peer==f.device.getTile(0,1) &&
          f.owner->tile.peer==&sink && f.sibling->tile.peer==&sink &&
          f.owner->pos==0 && f.sibling->pos==1 &&
          f.stable->tile.peer==f.device.getTile(4,2) &&
          f.request_source->tile.peer==f.device.getTile(0,0) &&
          f.request_trunk->tile.peer==f.device.getTile(1,0) &&
          f.request_sink->tile.peer==f.device.getTile(1,1),
          "private grounding repair moved a fixed endpoint");
  f.audit();
  std::puts("Private grounding repair: all routes complete through the new approach; audit clean");
}
