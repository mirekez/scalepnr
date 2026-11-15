#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"
#include <iostream>

using routing_tests::SourceInputFixture;
using routing_tests::require;

namespace {
void secondRegister(SourceInputFixture &f, fpga::Coord at) {
  fpga::Element slot; slot.name="register_lane_2"; slot.type=fpga::ELEMENT_FD;
  slot.bitmap_pos=1;
  f.tile(at).tile_type->elements.push_back(slot);
  f.tile(at).invalidatePlacementCaches();
  // Separate Q outputs, but one shared physical CE input for both registers.
  f.tile(at).tile_type->pin_map.nodes[{"FDRE","CE",4}].setBit(63);
  f.tile(at).tile_type->pin_map.output_nodes[
      f.tile(at).getNodeNum("FDRE","Q",4)].setBit(142);
}
void finalize(SourceInputFixture &f) {
  for(auto &type:f.device.cb_types) type.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
}
void auditPreserved(SourceInputFixture &f) {
  f.auditAll();
  for(auto &net:f.parent->nets) for(auto &binding:net.routes) {
    const bool background=binding.from && binding.from->cell_ref->name.starts_with("background_driver_");
    const bool sibling=binding.to && binding.to->cell_ref->name=="control_receiver_47";
    if (background || sibling)
      require(binding.owner && binding.route_index<binding.owner->wires.size() &&
          fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
          "recursive Moving damaged a retained CE sibling or background transit");
  }
}
rtl::Net *blockBranch(SourceInputFixture &f,fpga::Coord landing,int src) {
  // Occupy the known good CE branch with a complete unrelated transit, then
  // remove only that route between searches. Topology remains immutable.
  secondRegister(f,{3,1});
  auto *driver=f.cell("branch_blocker_91","FDRE",{{"Q",true}},{3,1},4);
  driver->outline.fixed=true;
  rtl::Inst *sink=nullptr;
  for(auto &inst:f.tech.design.top.insts)
    if(inst.cell_ref->name=="background_receiver_2") sink=&inst;
  require(sink!=nullptr,"missing blocker sink");
  auto *net=f.connect(*driver,"Q",*sink,"I2");
  std::vector<fpga::Wire> route{SourceInputFixture::pin({3,1},142,true),
      f.edge({3,1},landing,142,src,src+1,true),
      f.edge(landing,{7,2},src+1,src+2,src+3),
      f.terminal({7,2},src+3,214),SourceInputFixture::pin({7,2},214,false)};
  finalize(f);
  require(pnr::commitPreparedRoute(route),"cannot install branch blocker");
  sink->wires.push_back(std::move(route));
  const auto id=fpga::attachNetRoute(*net,*sink,sink->wires.size()-1,driver,sink,"Q","I2",
      SourceInputFixture::port(*sink,"I2").makeNetName());
  fpga::registerNetRouteTiles(*net,sink->wires.back(),id);
  auditPreserved(f);
  return net;
}
}

void moving_source_input_blacklist_regression() {
  SourceInputFixture f(false,0,32);
  f.installCEFrontier(false,{2,0});
  f.edge({3,1},{1,0},713,1780,1781); f.terminal({1,0},1781,63);
  auto *blocker=blockBranch(f,{1,0},1780);
  secondRegister(f,{2,0});
  for(int n=0;n<32;++n) f.tile({2,0}).cb_type->local_src[142].jump.setBit(100+n);
  finalize(f);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","source_blacklist_signal"};
  task.endpoints_prepared=true;
  pnr::MovingPlacementBlacklist blacklist({{f.source}});
  require(blacklist.rejects(f.tile({0,0})),"packing control unexpectedly accepts a register");
  auto *ce_driver=SourceInputFixture::port(*f.source,"CE").follow();
  require(ce_driver && ce_driver->inst_ref.peer,"missing real CE driver");
  pnr::RouteDesign::RouteTask ce_task{ce_driver->inst_ref.peer,f.source,f.enable,
      ce_driver->port_ref->makeName(),"CE",SourceInputFixture::port(*f.source,"CE").makeNetName()};
  std::vector<pnr::RouteDesign::RouteTask> closed_todo;
  bool closed_moved=false; std::string closed_reason;
  require(!router.moveUnfinishedSource(task,&closed_todo,&closed_reason,nullptr,&closed_moved,
      nullptr,nullptr,false,&blacklist),"closed CE graph completed all incident routing");
  require(!router.routeTaskIsComplete(ce_task),"closed CE graph acquired a complete route");
  if(closed_moved) require(std::any_of(closed_todo.begin(),closed_todo.end(),[&](const auto &next) {
      return next.to==f.source && next.to_port=="CE";
    }) || std::any_of(router.pending_route_todo.begin(),router.pending_route_todo.end(),[&](const auto &next) {
      return next.to==f.source && next.to_port=="CE";
    }),"committed parent placement dropped its unfinished CE task");
  auditPreserved(f);
  // Opening the physical branch must permit recursive CE completion with the
  // same packing blacklist, regardless of a prior committed parent candidate.
  fpga::unrouteNet(*blocker);
  router.move_tried_placements.clear();
  bool moved=false; std::string reason;
  require(router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved,
      nullptr,nullptr,false,&blacklist),reason);
  require(moved && f.source->tile.peer==&f.tile({1,0}),"source blacklist hid a feasible placement");
  require(router.routeTaskIsComplete(task) && router.routeTaskIsComplete(ce_task),
      "opened branch did not complete both output and CE input");
  const auto packing_checks=blacklist.checks;
  require(blacklist.rejects(f.tile({0,0})) && blacklist.checks==packing_checks,
      "source routing retry discarded its unchanged packing blacklist");
  auditPreserved(f);
}

void moving_destination_input_blacklist_regression() {
  for(int terminal_case:{0,1,2}) {
  const bool shared_ce = terminal_case != 0;
  const bool typed_terminal = terminal_case == 2;
  SourceInputFixture f(false,0,32);
  auto [control,sibling]=f.installCEFrontier(false);
  auto &router=f.tech.route;
  router.moving_stage=true;
  router.complete_routes_only=true;
  router.route_recursion_budget=16;
  router.route_preemption_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  f.source->outline.fixed=true;
  // The trigger is D; CE is a different incident input. The bad Tile offers
  // thirty-two genuinely reachable D arrivals and two register slots. Fourteen
  // CE ingresses have real owners; two are free but cannot reach the driver.
  // The next Tile can accept both D and CE via its retained branch.
  for(fpga::Coord at:std::array<fpga::Coord,1>{{{5,2}}}) {
    fpga::Element slot; slot.name="destination_register"; slot.type=fpga::ELEMENT_FD;
    f.tile(at).tile_type->elements.push_back(slot);
    f.tile(at).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(87);
    f.tile(at).cb_type->local_input_nodes.setBit(87);
  }
  secondRegister(f,{1,0});
  f.tile({1,0}).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(87);
  f.tile({1,0}).tile_type->pin_map.nodes[{"FDRE","D",4}].setBit(87);
  f.tile({2,0}).tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(87);
  auto *load=f.cell("movable_receiver_71","FDRE",{{"D",false},{"CE",false}},{2,0});
  // Reuse the real CE net with its complete retained sibling, and attach an
  // empty binding so production Moving must account for this additional input.
  auto &ce=SourceInputFixture::port(*load,"CE");
  ce.port_ref->designator=SourceInputFixture::port(*f.source,"CE").port_ref->designator;
  ce.set(&rtl::Conn::fromBase(SourceInputFixture::port(*control,"O")));
  load->wires.emplace_back();
  fpga::attachNetRoute(*f.enable,*load,0,control,load,"O","CE",ce.makeNetName());
  // Add another sink to Q's existing net, preserving the fixed original
  // sink's designator rather than changing the driver's signal identity.
  auto *data=f.signal;
  auto &d=SourceInputFixture::port(*load,"D");
  d.port_ref->designator=SourceInputFixture::port(*f.source,"Q").port_ref->designator;
  d.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.source,"Q")));
  for(int n=0;n<32;++n) {
    f.edge({7,0},{1,0},141,1000+n,1500+n,true);
    f.terminal({1,0},1500+n,87);
  }
  f.edge({7,0},{5,2},141,2000,2001,true); f.terminal({5,2},2001,87);
  f.edge({3,1},{5,2},713,2100,2101); f.terminal({5,2},2101,63);
  auto *blocker=blockBranch(f,{5,2},2100);
  finalize(f);
  pnr::RouteDesign::RouteTask task{f.source,load,data,"Q","D","destination_blacklist_signal"};
  task.endpoints_prepared=true;
  pnr::RouteDesign::RouteTask ce_task{control,load,f.enable,"O","CE",ce.makeNetName()};
  std::vector<pnr::RouteDesign::RouteTask> closed_todo;
  std::string closed_reason;
  require(!router.moveUnfinishedCell(task,&closed_todo,&task,&closed_reason),
      "destination accepted a locally free CE without a driver connection");
  require(!router.routeTaskIsComplete(ce_task),"closed destination CE acquired a complete route");
  if(load->tile.peer!=&f.tile({2,0})) require(std::any_of(router.pending_route_todo.begin(),
      router.pending_route_todo.end(),[&](const auto &next) {
        return next.to==load && next.to_port=="CE";
      }),"committed destination placement dropped its unfinished CE task");
  auditPreserved(f);
  // A new routing view must reconsider the previously rejected good Tile.
  fpga::unrouteNet(*blocker);
  router.move_tried_placements.clear();
  if(shared_ce) {
    // A correctly driven CE shared with a fixed register is usable, even
    // though its physical pin is leased and no fresh ingress can be leased.
    secondRegister(f,{5,2});
    auto *neighbor=f.cell("shared_control_receiver_95","FDRE",{{"CE",false}},{5,2},4);
    neighbor->outline.fixed=neighbor->pnr_completed=true;
    auto &input=SourceInputFixture::port(*neighbor,"CE");
    input.port_ref->designator=ce.port_ref->designator;
    input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*control,"O")));
    std::vector<fpga::Wire> route(sibling->wires.front().begin(),sibling->wires.front().begin()+5);
    for(auto &wire:route) wire.shared=true;
    auto fork=f.edge({3,1},{5,2},713,2100,2101); fork.pos=2; fork.owns_dst=false;
    std::vector<fpga::Wire> suffix{fork};
    if (typed_terminal) {
      // A shared terminal may be a numeric DST -> JOINT -> LOCAL tail rather
      // than one legacy crossbar fragment; both must retain the original owner.
      auto &cb = *f.tile({5,2}).cb_type;
      cb.dst_local[2101].local.clearBit(63);
      cb.dst_joint[2101].joint.setBit(41);
      cb.joint_local[41].local.setBit(63);
      SourceInputFixture::name(f.tile({5,2}),fpga::CB_NODE_JOINT,41);
      fpga::Wire entry;
      entry.type = fpga::Wire::WIRE_ROUTE_EDGE;
      entry.from = entry.to = {5,2};
      entry.from_node_type = fpga::CB_NODE_DST;
      entry.from_node = 2101;
      entry.to_node_type = fpga::CB_NODE_JOINT;
      entry.to_node = 41;
      entry.owns_dst = false;
      suffix.push_back(entry);
      entry.from_node_type = fpga::CB_NODE_JOINT;
      entry.from_node = 41;
      entry.to_node_type = fpga::CB_NODE_LOCAL;
      entry.to_node = 63;
      suffix.push_back(entry);
      cb.rebuildOutgoingSrcs();
    } else {
      suffix.push_back(f.terminal({5,2},2101,63));
    }
    suffix.push_back(SourceInputFixture::pin({5,2},63,false));
    require(pnr::commitPreparedRoute(suffix),"cannot install shared CE control");
    route.insert(route.end(),suffix.begin(),suffix.end());
    neighbor->wires.push_back(std::move(route));
    const auto id=fpga::attachNetRoute(*f.enable,*neighbor,0,control,neighbor,"O","CE",input.makeNetName());
    fpga::registerNetRouteTiles(*f.enable,neighbor->wires.front(),id);
    pnr::MovingPlacementBlacklist packing({{load}});
    require(!packing.rejects(f.tile({5,2})) && f.tile({5,2}).isPinNodeLeased(63),
        "shared CE control is not physically packable or routed");
    auditPreserved(f);
    // Deliberately remove the owner's endpoint while retaining its fabric
    // leases. An incomplete/orphan terminal is not proof of shared completion.
    const auto saved_pin = neighbor->wires.front().back();
    neighbor->wires.front().pop_back();
    fpga::registerNetRouteTiles(*f.enable,neighbor->wires.front(),id);
    require(!fpga::isRouteComplete(neighbor->wires.front()),
        "shared terminal negative control retained a complete owner");
    std::vector<pnr::RouteDesign::RouteTask> rejected_tasks;
    std::string rejected_reason;
    require(!router.moveUnfinishedCell(ce_task,&rejected_tasks,&ce_task,&rejected_reason),
        "destination borrowed an incomplete shared terminal owner");
    neighbor->wires.front().push_back(saved_pin);
    fpga::registerNetRouteTiles(*f.enable,neighbor->wires.front(),id);
    router.move_tried_placements.clear();
    auditPreserved(f);
  }
  std::string reason; std::vector<pnr::RouteDesign::RouteTask> todo;
  std::cout << "destination retry: shared_ce=" << shared_ce
            << " typed_terminal=" << typed_terminal
            << " tile=(" << load->tile->coord.x << ',' << load->tile->coord.y
            << ") data_complete=" << router.routeTaskIsComplete(task)
            << " ce_complete=" << router.routeTaskIsComplete(ce_task) << '\n';
  // The rejected move committed D before CE failed. Retry the unfinished CE
  // task, not completed D, which the real scheduler must retire without work.
  require(router.routeTaskIsComplete(task) && !router.routeTaskIsComplete(ce_task),
      "destination retry must start with completed D and unfinished CE");
  require(router.moveUnfinishedCell(ce_task,&todo,&ce_task,&reason),reason);
  require(load->tile.peer==&f.tile({5,2}),"destination accepted a disconnected CE or hid the feasible Tile");
  if(shared_ce) require(f.tile({5,2}).isPinNodeLeased(63),
      "destination lost its correctly driven shared CE pin");
  require(!todo.empty(),"destination silently dropped its CE repair task");
  for(auto &repair:todo)
    require(router.routeNetTask(repair),"selected destination cannot reconnect an incident input");
  require(router.routeTaskIsComplete(ce_task),"selected destination did not complete CE");
  require(router.routeTaskIsComplete(task),"selected destination lost its completed D input");
  auditPreserved(f);
  if (shared_ce) {
    // The moved sink borrows the terminal; removing that branch must preserve
    // the original receiver's pin, joint (when present), and complete route.
    const auto binding = f.enable->findRouteBinding(control,load,"O","CE",ce_task.net_name);
    require(binding.index < f.enable->routes.size(),"moved shared CE binding is missing");
    const auto &bound = f.enable->routes[binding.index];
    const auto &wires = bound.owner->wires[bound.route_index];
    require(wires.size() >= 2 && wires.back().shared && wires[wires.size()-2].shared,
        "moved CE acquired private ownership of a borrowed terminal");
    // This route can be entirely shared, so discard its binding rather than
    // asking the private-suffix trimmer to find a suffix that does not exist.
    require(fpga::discardNetBranch(*f.enable,binding.index),
        "cannot remove the moved shared CE branch");
    require(f.tile({5,2}).isPinNodeLeased(63) &&
                f.tile({5,2}).cb.local.local.testBit(63) &&
                f.tile({5,2}).cb.dst.jump.testBit(2101) &&
                (!typed_terminal || f.tile({5,2}).cb.joint.jump.testBit(41)),
        "removing the moved branch released another receiver's shared terminal");
    const auto owners = fpga::findNetOwnersByNode(f.tile({5,2}),fpga::CB_NODE_LOCAL,63);
    require(!owners.empty() && std::all_of(owners.begin(),owners.end(),[&](const auto &owner) {
      const auto &remaining = owner.net->routes.at(owner.binding_index);
      return remaining.to != load && remaining.from == control && remaining.owner &&
          fpga::isRouteComplete(remaining.owner->wires.at(remaining.route_index));
    }),"shared branch removal damaged the original complete terminal owner");
    require(router.routeTaskIsComplete(task),"shared CE removal invalidated the data input");
    auditPreserved(f);
  }
  }
}
