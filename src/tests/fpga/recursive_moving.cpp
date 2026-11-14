#include "Tech.h"
#include "RepairStatistics.h"
#include "route/RouteDiagnostics.h"
#include "route/RoutePassState.h"

#include <list>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Only one narrow unoccupied lane remains beside complete transit trees.
// The old input drivers cannot take off: completing either move needs recursion.
struct RecursiveMovingFixture {
  technology::Tech tech;
  fpga::Device &device = fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent, *primitive;
  rtl::Inst *a, *b, *c, *endpoint;
  std::vector<rtl::Inst> noise_sinks;
  rtl::Inst noise_source;
  rtl::Net *ab, *bc, *trigger;
  static constexpr int output = 141, data = 63, extra = 64;

  fpga::Tile &tile(int x) { return *device.getTile(x, 0); }
  rtl::Conn &port(rtl::Inst &inst, const std::string &name) {
    for (auto &conn : inst.conns) if (conn.port_ref->name == name) return conn;
    throw std::runtime_error("missing fixture port");
  }
  rtl::Inst *cell(const std::string &name, int x,
                  std::initializer_list<std::pair<const char *, bool>> ports) {
    auto &type = cells.emplace_back();
    type.name = name; type.type = "FDRE"; type.module_ref.set(primitive);
    type.ports.reserve(ports.size());
    for (auto [name, out] : ports) {
      auto &p = type.ports.emplace_back(); p.name = name; p.designator = -1;
      p.type = out ? rtl::Port::PORT_OUT : rtl::Port::PORT_IN;
    }
    auto &inst = tech.design.top.insts.emplace_back();
    inst.cell_ref.set(&type); inst.parent_ref.set(&tech.design.top);
    inst.coord = {x, 0}; inst.depth = 1; inst.conns.reserve(ports.size());
    for (auto &p : type.ports) {
      auto &conn = inst.conns.emplace_back();
      conn.inst_ref.set(&inst); conn.port_ref.set(&p);
      if (p.type == rtl::Port::PORT_IN) ++inst.cnt_inputs;
      else ++inst.cnt_outputs;
    }
    check(tile(x).tryAddAt(&inst, 0) == 0, "fixture packing failed for " + name);
    return &inst;
  }
  rtl::Net *connect(rtl::Inst &from, rtl::Inst &to, const char *input) {
    auto &net = parent->nets.emplace_back();
    int id = parent->nets.size(); net.name = "signal_" + std::to_string(id);
    net.designators.push_back(id);
    port(from, "Q").port_ref->designator = port(to, input).port_ref->designator = id;
    port(to, input).set(&rtl::Conn::fromBase(port(from, "Q")));
    return &net;
  }
  fpga::Wire edge(int x, int y, int local, int src, int dst, bool takeoff) {
    auto &cb = *tile(x).cb_type;
    if (takeoff) cb.local_src[local].jump.setBit(src);
    else cb.dst_src[local].jump.setBit(src);
    cb.dst_by_src[src].push_back({{y-x, 0}, static_cast<uint16_t>(y),
                                  {NodeMask{0, 1} << dst}, {}, true});
    cb.rememberNodeName(takeoff ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST,
                        local, "arrival_" + std::to_string(local));
    cb.rememberNodeName(fpga::CB_NODE_SRC, src, "departure_" + std::to_string(src));
    tile(y).cb_type->rememberNodeName(fpga::CB_NODE_DST, dst, "arrival_" + std::to_string(dst));
    fpga::Wire wire; wire.from={x,0}; wire.to={y,0}; wire.local=local;
    wire.jump=src; wire.dst=dst; wire.pos=takeoff?0:1; wire.owns_landing=true;
    return wire;
  }
  fpga::Wire terminal(int x, int dst, int local) {
    tile(x).cb_type->dst_local[dst].local.setBit(local);
    fpga::Wire w; w.from=w.to={x,0}; w.local=dst; w.pos=1; return w;
  }
  fpga::Wire pin(int x, int local, bool out) {
    fpga::Wire w; w.type=fpga::Wire::WIRE_TILE_PIN; w.from=w.to={x,0};
    w.local=local; w.pin_dir=out?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT; return w;
  }
  explicit RecursiveMovingFixture(bool destination) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=14; device.size_height=1; device.grid_spec.size={14,1};
    device.tile_grid.resize(14); device.tile_types.reserve(14); device.cb_types.reserve(14);
    for (int x=0;x<14;++x) {
      auto &cb=device.cb_types.emplace_back(); cb.name="matrix_"+std::to_string(x);
      cb.type_id=cb.base_type_id=x;
      auto &resource=device.tile_types.emplace_back(); resource.name="slots_"+std::to_string(x);
      resource.type=fpga::Tile::TILE_LUTS;
      fpga::Element slot; slot.name="storage"; slot.type=fpga::ELEMENT_FD; slot.bitmap_pos=0;
      resource.elements.push_back(slot);
      auto &t=tile(x); t.coord=t.cb_coord={x,0}; t.type=fpga::Tile::TILE_LUTS;
      t.tile_type=&resource; t.cb_type=t.cb.type=&cb; t.full_name="tile_"+std::to_string(x);
      resource.pin_map.nodes[{"FDRE","D",0}].setBit(data);
      resource.pin_map.nodes[{"FDRE","R",0}].setBit(extra);
      resource.pin_map.output_nodes[t.getNodeNum("FDRE","Q",0)].setBit(output);
      cb.local_input_nodes.setBit(data); cb.local_input_nodes.setBit(extra);
      cb.local_output_nodes.setBit(output);
    }
    tech.init(); tech.design.GND=tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="recursive_top"; parent->nets.reserve(32);
    primitive=&tech.design.modules.emplace_back(); primitive->name="storage_primitive";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name=tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent); tech.design.top.cell_ref.set(&tech.design.top_cell);
    a=cell("upstream_a",10,{{"Q",true}});
    b=cell("upstream_b",11,{{"D",false},{"Q",true}});
    c=cell("moving_c",12,{{"D",false},{"R",false},{"Q",true}});
    endpoint=cell("endpoint",0,destination ? std::initializer_list<std::pair<const char*,bool>>{{"Q",true}}
                                         : std::initializer_list<std::pair<const char*,bool>>{{"D",false}});
    endpoint->outline.fixed=true;
    ab=connect(*a,*b,"D"); bc=connect(*b,*c,destination?"R":"D");
    trigger=destination?connect(*endpoint,*c,"D"):connect(*c,*endpoint,"D");
    edge(1,2,output,20,30,true); terminal(2,30,data);
    edge(2,3,output,21,31,true); terminal(3,31,destination?extra:data);
    if (destination) { edge(0,3,output,22,32,true); terminal(3,32,data); }
    else { edge(3,0,output,22,32,true); terminal(0,32,data); }

    noise_sinks.resize(24);
    for(int n=0;n<24;++n) {
      auto &net=parent->nets.emplace_back(); net.name="transit_"+std::to_string(n);
      std::vector<fpga::Wire> route{pin(0,500+n,true)};
      for(int x=0;x<5;++x) route.push_back(edge(x,x+1,x?800+n:500+n,700+n,800+n,x==0));
      route.push_back(terminal(5,800+n,600+n)); route.push_back(pin(5,600+n,false));
      for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
      check(pnr::commitPreparedRoute(route),"cannot commit transit owner");
      auto &sink=noise_sinks[n]; sink.wires.push_back(std::move(route));
      size_t index=fpga::attachNetRoute(net,sink,0,&noise_source,&sink,"OUT","IN",net.name);
      fpga::registerNetRouteTiles(net,sink.wires[0],index);
    }
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    tech.route.moving_stage=true; tech.route.route_deadends_enabled=false;
    tech.route.route_preemption_enabled=false; tech.route.route_suffix_depth_limit=5;
    tech.route.route_stage_deadline_enabled=true;
    tech.route.route_stage_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  }
};
}

void recursive_moving_inputs(bool destination, bool binding_only, bool disconnected,
                            bool retry_destination) {
  RecursiveMovingFixture f(destination); auto &router=f.tech.route;
  if (retry_destination) {
    // The first reachable parent position (3) has no upstream input path.
    // Its continuation reaches position 4, where the same recursive subtree
    // has a legal placement. Actual input failure must advance the parent.
    f.tile(2).cb_type->dst_by_src[21].clear();
    f.edge(2,4,RecursiveMovingFixture::output,21,31,true);
    f.terminal(4,31,RecursiveMovingFixture::extra);
    f.edge(3,4,32,23,33,false);
    f.terminal(4,33,RecursiveMovingFixture::data);
    for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
  }
  using Task=pnr::RouteDesign::RouteTask;
  Task root{destination?f.endpoint:f.c,destination?f.c:f.endpoint,f.trigger,"Q","D",
      f.port(destination?*f.c:*f.endpoint,"D").makeNetName()};
  Task ab{f.a,f.b,f.ab,"Q","D",f.port(*f.b,"D").makeNetName()};
  Task bc{f.b,f.c,f.bc,"Q",destination?"R":"D",
      f.port(*f.c,destination?"R":"D").makeNetName()};
  std::vector<Task> queued{ab,bc,ab};
  if (binding_only) {
    // Generated endpoints may have a physical binding but no logical Conn edge.
    // The mover must retain this input obligation when reconstructing its work.
    f.c->wires.emplace_back();
    fpga::attachNetRoute(*f.bc,*f.c,f.c->wires.size()-1,f.b,f.c,"Q",bc.to_port,bc.net_name);
    f.port(*f.c,bc.to_port).clear();
  }
  if (disconnected) {
    f.tile(1).cb_type->dst_by_src[20].clear();
    f.tile(1).cb_type->rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
  }
  router.route_todo=queued;
  struct Profile {
    std::string old;
    bool present;
    Profile():present(std::getenv("SCALEPNR_PROFILE_REPAIR")!=nullptr) {
      if(present)old=std::getenv("SCALEPNR_PROFILE_REPAIR");
      setenv("SCALEPNR_PROFILE_REPAIR","1",1); fpga::resetRepairStatistics();
    }
    ~Profile(){if(present)setenv("SCALEPNR_PROFILE_REPAIR",old.c_str(),1);else unsetenv("SCALEPNR_PROFILE_REPAIR");}
  } profile;
  std::vector<Task> affected; std::string reason;
  const bool done=destination?router.moveUnfinishedCell(root,&affected,&root,&reason)
                             :router.moveUnfinishedSource(root,&affected,&reason);
  // Completing the parent requires real nested source relocations, not previews.
  if (!disconnected) {
  check(done,"recursive move failed: "+reason);
  check(f.c->coord.x==(retry_destination?4:3) && f.b->coord.x==2 && f.a->coord.x==1,
        "nested source movers did not select the available placements");
  if (retry_destination) {
    // Prove that success came from retrying the rejected committed position,
    // not from adding a pre-placement input search that skipped it entirely.
    const auto &tried=router.move_tried_placements.at(reinterpret_cast<uintptr_t>(f.c));
    const auto first=std::find(tried.begin(),tried.end(),pnr::movingPlacementKey(3,0,0));
    const auto second=std::find(tried.begin(),tried.end(),pnr::movingPlacementKey(4,0,0));
    check(first!=tried.end() && second!=tried.end() && first<second,
          "destination retry did not attempt the bad parent before the good one");
  }
  // Inputs queued before recursion must now reference complete live bindings.
  check(router.routeTaskIsComplete(root) && router.routeTaskIsComplete(ab) && router.routeTaskIsComplete(bc),
        "move returned with incomplete recursive inputs");
  auto result=router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving,queued,10,0);
  check(result.attempted==0 && result.completed==queued.size() &&
        std::all_of(queued.begin(),queued.end(),[](const Task &task){return task.remove_after_pass;}),
        "completed queued input was traced again");
  // Relocation invalidates a source tree while queued copies may still say
  // Fanout. A synchronous call must promote and attempt its Generic seed in
  // the same invocation, even with only one remaining dispatch allowance.
  const auto binding=root.net->findRouteBinding(root.from,root.to,root.from_port,
      root.to_port,root.net_name);
  check(binding.index<root.net->routes.size() &&
        fpga::unrouteNetRoute(*root.net,binding.index),"cannot invalidate completed seed for dispatch control");
  root.fanout=root.endpoints_prepared=true;
  router.complete_routes_only=true;
  router.route_recursion_budget=1;
  check(router.routeNetTask(root) && !root.fanout && router.routeTaskIsComplete(root),
        "missing-seed promotion returned failure without attempting Generic routing");
  } else {
    // A failed grandchild must not turn the parent into a successful move.
    check(!done && !reason.empty() && !router.routeTaskIsComplete(ab),
          "disconnected upstream input was silently accepted");
    // Report the failing grandchild, not only the parent's completed trunk.
    check(reason.find(ab.net_name)!=std::string::npos &&
              reason.find("upstream_a")!=std::string::npos,
          "recursive failure lost the actual unfinished input's identity");
    // Preserve committed child placements, but retain their unfinished work.
    check(f.c->coord.x==3 && f.b->coord.x==2 && f.a->coord.x==10,
          "failed grandchild restored stale parent placement snapshots");
    check(std::any_of(router.pending_route_todo.begin(),router.pending_route_todo.end(),
        [&](const Task &task){return task.from==f.a && task.to==f.b;}),
        "failed recursive input disappeared from pending work");
  }
  // All blockers remain complete, and every leased resource has a real owner.
  for(auto &sink:f.noise_sinks) check(fpga::isRouteComplete(sink.wires[0]),"nested move damaged transit route");
  std::vector<rtl::Net*> nets; for(auto &net:f.parent->nets) nets.push_back(&net);
  for(auto &tile:f.device.tile_grid) {
    std::ostringstream details; const auto audit=fpga::auditTileCongestion(tile,details,nets);
    check(!audit.orphan_leases && !audit.missing_leases && !audit.missing_registrations &&
          !audit.stale_registrations && !audit.conflicting_driver_nodes,details.str());
  }
  check(router.moving_input_cells.empty(),"recursive movement left an active parent");
  check(!router.moving_sources_stage,"nested source move leaked its stage to the parent");
  // No input/output connection may be proved by a broad graph search in Moving.
  const auto &work=fpga::repairStatistics();
  check(work[fpga::RepairCounter::source_calls]==0 && work[fpga::RepairCounter::gap_calls]==0 &&
        work[fpga::RepairCounter::two_ended_calls]==0,"recursive moving launched a broad route search");
}
