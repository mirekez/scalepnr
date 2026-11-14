#include "SourceInputFixture.h"

using routing_tests::SourceInputFixture;
using routing_tests::require;

std::pair<rtl::Inst*,rtl::Inst*> routing_tests::SourceInputFixture::installCEFrontier(
    bool connect_branch, fpga::Coord candidate_coord) {
    auto &f=*this; auto &router=tech.route;
    std::vector<fpga::Wire> retained(f.source->wires.front().begin(),
                                   f.source->wires.front().begin()+5);
    require(fpga::unrouteNetRoute(*f.enable,0),"cannot replace original CE binding");
    f.enable->clearRouteBindings();

    // Preserve the real alias relationship, not just equal source names.
    fpga::Element mux; mux.name="control_link_slot"; mux.type=fpga::ELEMENT_MUXF7;
    f.tile({6,0}).tile_type->elements.push_back(mux);
    f.tile({6,0}).invalidatePlacementCaches();
    f.tile({6,0}).tile_type->pin_map.output_nodes[
        f.tile({6,0}).getNodeNum("MUXF7","O",1)].setBit(169);
    auto *alias=f.cell("control_link_43","MUXF7",{{"I1",false},{"O",true}},{6,0});
    alias->cell_ref->attributes["scalepnr_passthrough"]="source";
    alias->outline.fixed=alias->pnr_completed=f.control->pnr_completed=true;
    const int enable_id=SourceInputFixture::port(*f.source,"CE").port_ref->designator;
    auto *local=f.connect(*f.control,"O",*alias,"I1"); local->void_net=true;
    SourceInputFixture::port(*alias,"O").port_ref->designator=enable_id;
    SourceInputFixture::port(*f.source,"CE").set(
        &rtl::Conn::fromBase(SourceInputFixture::port(*alias,"O")));
    fpga::Element slot;slot.name="stationary_register";slot.type=fpga::ELEMENT_FD;
    f.tile({3,0}).tile_type->elements.push_back(slot);
    auto *sibling=f.cell("control_receiver_47","FDRE",{{"CE",false}},{3,0});
    sibling->outline.fixed=sibling->pnr_completed=true;
    auto &ce=SourceInputFixture::port(*sibling,"CE");ce.port_ref->designator=enable_id;
    ce.set(&rtl::Conn::fromBase(SourceInputFixture::port(*alias,"O")));
    retained.push_back(f.edge({3,1},{3,0},713,1917,1917));
    retained.push_back(f.terminal({3,0},1917,63));
    retained.push_back(SourceInputFixture::pin({3,0},63,false));
    f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);

    auto &candidate=f.tile(candidate_coord);auto &cb=*candidate.cb_type;
    for(auto &[id,paths]:cb.dst_local.values.write()) paths.local.clearBit(63);
    f.edge({0,1},candidate_coord,1851,1799,700);
    cb.dst_local[700].local.clearBit(63);
    SourceInputFixture::name(candidate,fpga::CB_NODE_JOINT,50);
    cb.joint_local[50].local.setBit(63);
    cb.dst_joint[700].joint.setBit(50);
    f.edge({3,1},candidate_coord,connect_branch?713:1850,1800,1800);
    cb.dst_joint[1800].joint.setBit(50);
    // Use separate source Tiles so each register has an unambiguous physical
    // Q mapping; the background sinks use three distinct LUT lanes.
    const std::array<fpga::Coord,14> source_tiles{{{4,0},{5,0},{0,1},{1,1},
        {2,1},{3,1},{4,1},{5,1},{6,1},{7,1},{0,2},{1,2},{2,2},{3,2}}};
    for(auto coord:source_tiles) f.tile(coord).tile_type->elements.push_back(slot);
    for(int n=0;n<3;++n) {
      fpga::Element e;e.name="background_logic_"+std::to_string(n);
      e.type=fpga::ELEMENT_LUT5;e.bitmap_pos=n;
      f.tile({7,2}).tile_type->elements.push_back(e);
      e.name="background_extension_"+std::to_string(n);e.type=fpga::ELEMENT_LUT1;
      f.tile({7,2}).tile_type->elements.push_back(e);
    }
    std::array<rtl::Inst*,3> receivers;
    for(int n=0;n<3;++n) {
      const int pos=n*4+3;
      for(int input=0;input<6;++input) {
        const int pin=200+n*6+input;
        f.tile({7,2}).tile_type->pin_map.nodes[{"LUT6","I"+std::to_string(input),pos}]={};
        f.tile({7,2}).tile_type->pin_map.nodes[{"LUT6","I"+std::to_string(input),pos}].setBit(pin);
        f.tile({7,2}).cb_type->local_input_nodes.setBit(pin);
      }
      receivers[n]=f.cell("background_receiver_"+std::to_string(n),"LUT6",
          {{"I0",false},{"I1",false},{"I2",false},{"I3",false},{"I4",false},{"I5",false}},
          {7,2},pos);
      receivers[n]->outline.fixed=receivers[n]->pnr_completed=true;
    }
    std::vector<rtl::Inst*> background_sinks;
    std::vector<rtl::Inst*> background_drivers;
    for(int n=0;n<14;++n) {
      const auto coord=source_tiles[n];
      auto *driver=f.cell("background_driver_"+std::to_string(n),"FDRE",{{"Q",true}},coord);
      driver->outline.fixed=driver->pnr_completed=true;
      auto *receiver=receivers[n/6];const auto port="I"+std::to_string(n%6);
      auto *net=f.connect(*driver,"Q",*receiver,port);
      std::vector<fpga::Wire> route{SourceInputFixture::pin(coord,141,true),
          f.edge(coord,candidate_coord,141,1500+n,1900+n,true),
          f.edge(candidate_coord,{7,2},1900+n,1600+n,2000+n),
          f.terminal({7,2},2000+n,200+n),SourceInputFixture::pin({7,2},200+n,false)};
      cb.dst_joint[1900+n].joint.setBit(50);
      require(pnr::commitPreparedRoute(route),"cannot install background transit");
      receiver->wires.push_back(std::move(route));
      const auto binding=fpga::attachNetRoute(*net,*receiver,receiver->wires.size()-1,
          driver,receiver,"Q",port,SourceInputFixture::port(*receiver,port).makeNetName());
      fpga::registerNetRouteTiles(*net,receiver->wires.back(),binding);
      background_sinks.push_back(receiver);
      background_drivers.push_back(driver);
    }
    for(int n=0;n<14;++n) {
      require(background_drivers[n]->tile->getOutputPinNodes("FDRE","Q",0).testBit(141) &&
          background_sinks[n]->tile->getPinNodes("LUT6","I"+std::to_string(n%6),
              background_sinks[n]->pos).testBit(200+n),
          "background route does not match its real placed endpoint pins");
    }
    for(auto &type:f.device.cb_types)type.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(retained),"cannot install fixed CE sibling");
    sibling->wires.push_back(std::move(retained));
    auto binding=fpga::attachNetRoute(*f.enable,*sibling,0,alias,sibling,"O","CE",ce.makeNetName());
    fpga::registerNetRouteTiles(*f.enable,sibling->wires[0],binding);
    router.indexSourceRoute(f.enable,alias,"O");
    int free=0,blocked=0;
    for(const auto &entry:cb.terminalEntries(63)) {
      require(candidate.incoming_dst_nodes.testBit(entry.dst),"CE alternative has no physical incoming jump");
      (candidate.cb.dst.jump.testBit(entry.dst)?blocked:free)++;
    }
    require(free==2 && blocked==14 && !candidate.cb.joint.jump.testBit(50) &&
        !candidate.cb.local.local.testBit(63),"fixture lost its two-free/fourteen-blocked CE approaches");
    auditAll();
    return {alias,sibling};
}

void moving_source_fan_input_frontier_regression() {
  // The last real failure had a fixed generated CE source, five retained
  // sibling landings, and sixteen CE fan-in alternatives: fourteen blocked,
  // two locally free. Local compatibility did not prove a connection to the
  // driver. Every occupied DST below has its own complete transit between
  // real placed cells; the common terminal joint and local remain free.
  // Both remaining entries physically arrive but are disconnected in the
  // negative control. Opening one known branch must make relocation succeed.
  // This is a hypothesis test, not a claim of a captured real-device bypass.
  for(bool connect_branch : {false,true}) {
    SourceInputFixture f(false,0,32);
    auto &router=f.tech.route;
    router.moving_stage=router.moving_sources_stage=true;
    router.route_preemption_enabled=true; router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    auto [alias,sibling]=f.installCEFrontier(connect_branch);
    auto audit=[&] {
      std::vector<rtl::Net*> nets;
      for(auto &net:f.parent->nets)nets.push_back(&net);
      for(auto &tile:f.device.tile_grid) {
        std::ostringstream out;
        const auto a=fpga::auditTileCongestion(tile,out,nets);
        require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
            !a.stale_registrations && !a.conflicting_driver_nodes,out.str());
      }
    };
    audit();
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
        SourceInputFixture::port(*f.sink,"I0").makeNetName()};
    task.endpoints_prepared=true;bool moved=false;std::string reason;
    std::vector<pnr::RouteDesign::RouteTask> affected;
    const bool done=router.moveUnfinishedSource(task,&affected,&reason,nullptr,&moved);
    std::cout << "CE frontier branch=" << connect_branch << " complete=" << done
              << " moved=" << moved << " reason=" << reason << '\n';
    audit();
    require(done==connect_branch,
        "source routing disagrees with the known CE connectivity: "+reason);
    if(connect_branch) require(moved && f.source->tile.peer==&f.tile({1,0}),
        "source routing missed the reachable parent placement");
    require(fpga::isRouteComplete(sibling->wires.front()),"source routing damaged the fixed sibling");
    if(!connect_branch) {
      // Failed recursive input routing may leave a committed parent placement,
      // but it must not claim completion or drop the remaining CE task.
      auto pending_input=[&](const auto& next) {return next.to==f.source && next.to_port=="CE";};
      require(!reason.empty() && (std::any_of(affected.begin(),affected.end(),pending_input) ||
          std::any_of(router.pending_route_todo.begin(),router.pending_route_todo.end(),pending_input)),
          "disconnected recursive CE was not retained as unfinished work");
      for(const auto& next:affected) if(pending_input(next))
        require(!router.routeTaskIsComplete(next),"disconnected CE acquired a complete binding");
      for(const auto& next:router.pending_route_todo) if(pending_input(next))
        require(!router.routeTaskIsComplete(next),"queued disconnected CE acquired a complete binding");
    }
    for(auto &net:f.parent->nets) if(net.name.starts_with("signal_"))
      for(auto &binding:net.routes) if(binding.owner && binding.owner!=f.sink && binding.owner!=f.source)
        require(fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
            "input probing damaged a background transit owner");
  }
}
