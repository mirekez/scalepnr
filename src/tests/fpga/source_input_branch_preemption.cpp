#include "SourceInputFixture.h"
#include "route/MovingSourceRecovery.h"

#include <cstdio>

using namespace routing_tests;

// The saved source-relocation failure had a three-hop CE suffix from a
// retained tree landing. Only the first jump was foreign transit; the final
// DST -> JOINT -> input was free. A terminal-only preview cannot discover it.
// Preserve that nine-node structure with synthetic coordinates and names.
void source_input_branch_preemption_regression(int mode) {
  const bool control_only=mode==1, far=mode==2, blocked=mode==3;
  size_t repeated_proofs=0,repeated_expanded=0;
  SourceInputFixture f(false,0,8,far?16:8);
  const fpga::Coord target_coord{far?14:1,0};
  auto& target=f.tile(target_coord);
  // The real failed register had no CE binding of its own: its available
  // branches belonged to completed, independently placed sibling registers.
  // Replace the fixture's initial own-input route with such a sibling route.
  fpga::Element sibling_slot;sibling_slot.name="retained_control_slot";
  sibling_slot.type=fpga::ELEMENT_FD;
  f.tile({3,2}).tile_type->elements.push_back(sibling_slot);
  auto* sibling=f.cell("retained_control_receiver_89","FDRE",{{"CE",false}},{3,2});
  sibling->outline.fixed=sibling->pnr_completed=true;
  auto& sibling_input=SourceInputFixture::port(*sibling,"CE");
  sibling_input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
  sibling_input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
  auto retained=f.source->wires[0];
  fpga::unrouteNetRoute(*f.enable,0);f.enable->eraseRouteBinding(0);f.source->wires.clear();
  retained[retained.size()-3]=f.edge({0,1},{3,2},716,1510,718);
  retained[retained.size()-2]=f.terminal({3,2},718,63);
  retained.back()=SourceInputFixture::pin({3,2},63,false);
  require(pnr::commitPreparedRoute(retained),"cannot install the retained sibling CE route");
  sibling->wires.push_back(std::move(retained));
  auto sibling_binding=fpga::attachNetRoute(*f.enable,*sibling,0,f.control,sibling,
      "O","CE",sibling_input.makeNetName());
  fpga::registerNetRouteTiles(*f.enable,sibling->wires[0],sibling_binding);
  if(far) {
    fpga::Element e;e.name="distant_register_slot";e.type=fpga::ELEMENT_FD;
    target.tile_type->elements.push_back(e);
    f.tile({1,0}).tile_type->elements.clear();
    f.tile({1,0}).cb_type->local_src[141].jump.clearBit(300);
    f.edge(target_coord,{0,0},141,300,800,true);
  }
  target.tile_type->pin_map.input_nodes[target.getNodeNum("FDRE","CE",0)]={};
  target.tile_type->pin_map.input_nodes[target.getNodeNum("FDRE","CE",0)].setBit(40);
  target.cb_type->local_input_nodes.setBit(40);
  f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);
  f.edge({5,1},{3,0},711,3537,977);
  f.edge({3,0},{2,0},977,545,545);
  f.edge({2,0},target_coord,545,1026,1025);
  if(blocked)f.tile({2,0}).cb_type->dst_src[545].jump.clearBit(1026);
  SourceInputFixture::name(target,fpga::CB_NODE_JOINT,63);
  SourceInputFixture::name(target,fpga::CB_NODE_LOCAL,40);
  target.cb_type->dst_joint[1025].joint.setBit(63);
  target.cb_type->joint_local[63].local.setBit(40);
  if(mode==5) {
    // The next device failure admitted CE in the independent view, but its
    // candidate Q reservation isolated CE. Thousands of previews released
    // unrelated foreign transit and repeated the same exhausted input walk.
    // All 32 Q alternatives here reserve the same destination JOINT. CE has
    // a real free route through that JOINT before Q is reserved; releasing
    // the branch victim below cannot free a privately reserved Q resource.
    auto& cb=*f.tile({0,0}).cb_type;
    SourceInputFixture::name(f.tile({0,0}),fpga::CB_NODE_JOINT,65);
    for(int n=0;n<32;++n) {
      cb.dst_local[500+n].local.clearBit(87);
      cb.dst_joint[500+n].joint.setBit(65);
    }
    cb.joint_local[65].local.setBit(87);
    // Preserve a distinct potential grounding, so this JOINT is not a
    // mandatory packing reservation for the already placed data sink.
    f.terminal({0,0},1999,87);
    f.edge({5,1},{0,0},711,1980,1981);
    cb.dst_joint[1981].joint.setBit(65);
    cb.joint_local[65].local.setBit(186);
    f.resolvedLocal({0,0},186,{2,0},fpga::CB_NODE_LOCAL,63);
  }
  if(mode==6) {
    // The native repeater has a NEGATIVE exact free-input preflight, unlike
    // mode 5's positive preflight followed by a private output conflict.
    // This free corridor reaches a retained-tree TILE but not its actual
    // landing node 711. The coarse region must remain uncertain, while the
    // exact input proof must fail. Releasing the legal transit cut elsewhere
    // cannot connect this corridor to that retained tree.
    f.edge({5,1},{4,0},1990,1991,1992);
    f.tile({4,0}).cb_type->dst_src[1992].jump.setBit(1000);
    // Enlarge the disconnected reverse region with neutral-named joints.
    // Each real preview traverses these nodes; there is no test-side loop
    // calling the proof repeatedly to manufacture the observed work.
    auto& cb=*f.tile({4,0}).cb_type;
    for(int n=0;n<128;++n) {
      SourceInputFixture::name(f.tile({4,0}),fpga::CB_NODE_JOINT,2000+n);
      cb.joint_src[2000+n].jump.setBit(1000+n%24);
      if(n) cb.joint_joint[2000+n-1].joint.setBit(2000+n);
    }
  }
  for(auto [at,type]:std::vector<std::pair<fpga::Coord,fpga::ElementType>>{
      {{4,2},fpga::ELEMENT_FD},{{5,2},fpga::ELEMENT_LUT5},{{5,2},fpga::ELEMENT_LUT1}}) {
    fpga::Element e;e.name="branch_owner_slot_"+std::to_string(type);e.type=type;
    f.tile(at).tile_type->elements.push_back(e);
  }
  f.tile({4,2}).tile_type->pin_map.output_nodes[f.tile({4,2}).getNodeNum("FDRE","Q",0)].setBit(170);
  f.tile({4,2}).cb_type->local_output_nodes.setBit(170);
  auto* driver=f.cell("branch_transit_driver_71","FDRE",{{"Q",true}},{4,2});
  auto* receiver=f.cell("branch_transit_receiver_73","LUT6",{{"I0",false}},{5,2});
  auto* victim=f.connect(*driver,"Q",*receiver,"I0");
  std::vector<fpga::Wire> route{SourceInputFixture::pin({4,2},170,true),
      f.edge({4,2},{5,1},170,1500,1503,true),
      f.edge({5,1},{3,0},1503,3537,977),
      f.edge({3,0},{5,2},977,1501,1502),f.terminal({5,2},1502,87),
      SourceInputFixture::pin({5,2},87,false)};
  f.edge({5,1},{5,2},1503,1504,1505);f.terminal({5,2},1505,87);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(route),"cannot install the CE branch transit owner");
  receiver->wires.push_back(std::move(route));
  auto id=fpga::attachNetRoute(*victim,*receiver,0,driver,receiver,"Q","I0",victim->name);
  fpga::registerNetRouteTiles(*victim,receiver->wires[0],id);
  f.auditAll();
  auto& router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=!control_only;
  router.complete_routes_only=router.preserve_routes=true;
  router.route_deadends_enabled=false;router.route_preemption_enabled=true;
  router.iteration_limit=router.route_suffix_depth_limit=router.route_recursion_budget=200;
  if(mode==6) {
    auto* old_tile=f.source->tile.peer;
    const auto old_coord=f.source->coord;
    const int old_pos=f.source->pos;
    f.source->tile.clear();
    f.source->tile.set(static_cast<Referable<fpga::Tile>*>(&f.tile({2,0})));
    f.source->coord={2,0};f.source->pos=0;
    pnr::RouteDesign::RouteTask input{f.control,f.source,f.enable,"O","CE","repeater_input"};
    pnr::MovingPlacementBlacklist preflight({});
    const bool exact=router.movingInputMayRoute(input,{f.source},{},preflight);
    const bool coarse=preflight.inputRegion(f.control,"O",f.tile({2,0})).uncertain();
    f.source->tile.clear();f.source->tile.set(old_tile);
    f.source->coord=old_coord;f.source->pos=old_pos;
    // Prove we exercise the missing exact-frontier case, not the already
    // covered disconnected coarse region or private output reservation case.
    require(!exact && coarse,"repeater must have a negative exact proof and uncertain coarse region");
    NodeMask pins;pins.setBit(63);
    const std::vector<fpga::RouteCutNode> unrelated{{{5,1},fpga::CB_NODE_SRC,3537}};
    // The observed free-input failure must filter this actual transit cut,
    // but must not reject another physical input pin on the same tile.
    require(!preflight.inputCutMayHelp(f.control,"O",f.tile({2,0}),pins,unrelated),
        "exact negative preflight did not retain its cut boundary");
    NodeMask other_pin;other_pin.setBit(87);
    require(preflight.inputCutMayHelp(f.control,"O",f.tile({2,0}),other_pin,unrelated),
        "input boundary leaked to another physical pin");
    // Any live routing revision drops this optimization; do not mistake a
    // previously negative proof for current reachability after an unroute.
    preflight.clearRoutingFailures({&f.tile({5,1})});
    require(preflight.inputCutMayHelp(f.control,"O",f.tile({2,0}),pins,unrelated),
        "input boundary survived live-routing invalidation");
    f.auditAll();
  }
  if(mode==5) {
    auto* old_tile=f.source->tile.peer;
    const auto old_coord=f.source->coord;
    const int old_pos=f.source->pos;
    f.source->tile.clear();f.source->tile.set(static_cast<Referable<fpga::Tile>*>(&f.tile({2,0})));
    f.source->coord={2,0};f.source->pos=0;
    pnr::RouteDesign::RouteTask input{f.control,f.source,f.enable,"O","CE","private_ce_reference"};
    pnr::MovingPlacementBlacklist independent({}),reserved({});
    const bool free_ce=router.movingInputMayRoute(input,{f.source},{},independent);
    pnr::BackwardTakeoffStateView output;
    output.emplace(&f.tile({0,0}),f.tile({0,0}).cb);
    output.at(&f.tile({0,0})).joint.jump.setBit(65);
    const bool reserved_ce=router.movingInputMayRoute(input,{f.source},output,reserved);
    f.source->tile.clear();f.source->tile.set(old_tile);f.source->coord=old_coord;f.source->pos=old_pos;
    require(free_ce,"private-conflict reference has no independent CE route");
    require(!reserved_ce,"private Q reservation did not isolate the reference CE route");
  }
  if(control_only) {
    auto* probe=f.cell("branch_control_probe_79","FDRE",{{"CE",false}},{1,0});
    auto& input=SourceInputFixture::port(*probe,"CE");
    input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
    input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
    pnr::RouteDesign::RouteTask task{f.control,probe,f.enable,"O","CE","branch_control_reference"};
    task.fanout=true;
    require(router.routeTaskOrFail(task,"CE branch reference",200),
        "ordinary CE branching cannot use the transit-preemptible reference");
  } else {
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","branch_register_output_83"};
    pnr::MovingPlacementBlacklist placements({{f.source}});
    std::string reason;size_t released=0,retries=0;bool moved=false,retryable=false;
    const bool done=pnr::recoverMovingSourceBoundaries(router,task,placements,nullptr,
        reason,released,moved,retryable,retries,false,true);
    if(blocked) {
      require(!done,"disconnected CE branch unexpectedly completed relocation");
      require(f.source->tile.peer==&f.tile({7,0}),"failed branch preview moved the register");
      require(router.route_stats.preempt_success==0 && router.pending_route_todo.empty() &&
          fpga::isRouteComplete(receiver->wires[0]),"failed branch preview cut or queued its transit owner");
      require(fpga::isRouteComplete(sibling->wires[0]),"failed branch preview damaged its retained sibling");
      require(placements.input_branch_previews==0,
          "branch release outside the negative input region repeated an exact proof");
      f.auditAll();return;
    }
    require(done,
        "Source Moving rejected CE with a free terminal and a preemptible branch Takeoff: "+reason);
    require(f.source->tile.peer==&target,"Source Moving missed the reference bank");
    auto complete=[&](rtl::Net& net,rtl::Inst* from,rtl::Inst* to,const char* input) {
      return std::any_of(net.routes.begin(),net.routes.end(),[&](const auto& b) {
        return b.from==from && b.to==to && b.to_port==input && b.owner &&
            b.route_index<b.owner->wires.size() && fpga::isRouteComplete(b.owner->wires[b.route_index]);
      });
    };
    require(complete(*f.signal,f.source,f.sink,"I0"),"branch relocation left Q unfinished");
    require(complete(*f.enable,f.control,f.source,"CE"),"branch relocation left CE unfinished");
    // The fixture also exposes 32 Q exits at a disconnected CE bank. Releasing
    // this victim cannot affect that bank's negative region; do not repeat a
    // full input walk for every output exit before reaching the working bank.
    std::printf("branch proofs: mode=%d exact=%zu expanded=%zu\n",mode,
        placements.input_branch_previews,placements.input_branch_expanded);
    if(mode==6) {
      std::printf("cut proof repeater: eligibility=%zu reused=%zu proofs=%zu expanded=%zu skipped=%zu\n",
          placements.input_branch_checks,placements.input_branch_reuses,
          placements.input_branch_previews,placements.input_branch_expanded,placements.input_cut_skips);
      // All 32 output alternatives must exercise the cached cut eligibility;
      // routing must not pass by skipping the early candidate altogether.
      require(placements.input_branch_checks>0 && placements.input_branch_checks<=16 &&
          placements.input_branch_reuses>=16,"repeater did not reuse legal-cut checks across output exits");
      require(placements.input_cut_skips>=32,
          "repeater did not filter irrelevant cuts using the exact negative boundary");
      // Keep live ownership valid before checking work. Complete the victim
      // repair below too, so failure cannot conceal an ownership regression.
      f.auditAll();
      repeated_proofs=placements.input_branch_previews;
      repeated_expanded=placements.input_branch_expanded;
    }
    require(placements.input_branch_previews>0 && (mode==6 || placements.input_branch_previews<=2),
        "CE branch proof repeated for unusable output alternatives: "+
        std::to_string(placements.input_branch_previews));
    require(placements.input_branch_expanded>0,"branch preemption never proved a full input path");
    if(mode==4) {
      // The failed device run spent nine of ten late samples revalidating
      // victims, without even starting an exact input proof. The 32 output
      // exits here share the same CE tree and unchanged foreign transit.
      // Count the expensive eligibility calls, not wall-clock time or only
      // the exact route proofs which come after those calls.
      std::printf("branch work: eligibility=%zu reused=%zu exact_proofs=%zu expanded=%zu\n",
          placements.input_branch_checks,placements.input_branch_reuses,
          placements.input_branch_previews,placements.input_branch_expanded);
      require(placements.input_branch_checks<=16,
          "unchanged CE branch eligibility repeated across output alternatives: "+
          std::to_string(placements.input_branch_checks));
      require(placements.input_branch_reuses>=16,
          "output alternatives did not exercise reuse of branch eligibility");
    }
  }
  require(router.route_stats.preempt_success>0,"reference never preempted the branch transit");
  pnr::RouteDesign::RouteTask repair{driver,receiver,victim,"Q","I0",victim->name};
  require(router.routeTaskOrFail(repair,"repair CE branch transit",200),"branch victim cannot use its free bypass");
  require(fpga::isRouteComplete(receiver->wires[victim->routes[0].route_index]),"branch transit victim stayed incomplete");
  require(sibling->tile.peer==&f.tile({3,2}) && fpga::isRouteComplete(sibling->wires[0]),
      "branch recovery damaged its retained control sibling");
  f.auditAll();
  if(mode==6) {
    // The independent preflight supplies the negative boundary once. Only
    // the later useful cut needs a branch proof (nine states in this graph).
    require(repeated_proofs==1 && repeated_expanded<=16,
        "cached legal cut repeated the exhausted full input-route proof: proofs="+
        std::to_string(repeated_proofs)+" expanded="+std::to_string(repeated_expanded));
  }
}
