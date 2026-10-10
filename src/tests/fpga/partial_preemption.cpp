#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/RouteDesign.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

// 24 sinks branch before the contested transit, eight after it. All endpoint
// names and numeric topology are synthetic. The alternate path is a detour,
// not a request to release any other signal's routing.
struct Fixture {
  fpga::Device &device = fpga::Device::current();
  Referable<rtl::Net> net;
  rtl::Inst driver;
  std::vector<rtl::Inst> sinks{32};
  fpga::CBType *cb;
  Fixture() {
    device.tile_grid.clear(); device.cb_types.clear();
    device.cb_types.emplace_back(); cb = &device.cb_types.back();
    cb->name = "matrix_731"; cb->type_id = cb->base_type_id = 0;
    device.grid_spec.size = {5, 2}; device.size_width = 5; device.size_height = 2;
    device.tile_grid.resize(10);
    for (int y = 0; y < 2; ++y) for (int x = 0; x < 5; ++x) {
      auto &t = device.tile_grid[y * 5 + x];
      t.coord = t.cb_coord = {x,y}; t.cb_type = cb; t.cb.type = cb;
    }
    driver.tile.set(&device.tile_grid[0]); net.name = "signal_593";
    fpga::Wire source;
    source.type = fpga::Wire::WIRE_TILE_PIN; source.from = source.to = {0,0};
    source.local = 300; source.pin_dir = 1;
    const auto takeoff = edge({0,0},{1,0},300,100,200,0);
    auto shared = edge({1,0},{2,0},200,120,20);
    shared.joint = 33;
    cb->dst_src[200].jump.clearBit(120);
    cb->dst_joint[200].joint.setBit(33);
    cb->joint_src[33].jump.setBit(120);
    for (size_t i = 0; i < sinks.size(); ++i) {
      auto &sink = sinks[i]; sink.tile.set(&device.tile_grid[4]);
      std::vector<fpga::Wire> wires{source,takeoff};
      if (i < 8) wires.push_back(shared);
      auto branch = edge(i < 8 ? fpga::Coord{2,0} : fpga::Coord{1,0},
          {3,0}, i < 8 ? 20 : 200, 140+i, 220+i);
      wires.push_back(branch);
      wires.push_back(edge({3,0},{4,0},220+i,400+i,500+i));
      fpga::Wire terminal;
      terminal.from = terminal.to = {4,0}; terminal.local = 500+i; terminal.pos = 1;
      wires.push_back(terminal);
      auto pin = source; pin.from = pin.to = {4,0}; pin.local = 600+i; pin.pin_dir = 0;
      cb->dst_local[500+i].local.setBit(600+i);
      wires.push_back(pin);
      for (size_t f = 0; f < (i < 8 ? 3 : 2); ++f) wires[f].shared = i != 0;
      sink.wires.push_back(std::move(wires));
      lease(sink.wires[0]);
      fpga::attachNetRoute(net,sink,0,&driver,&sink,"OUT","IN","route_"+std::to_string(i));
      fpga::registerNetRouteTiles(net,sink.wires[0],i);
    }
    // Replacement bypasses the shared hop. A second extension can also bypass
    // one private branch if it is preempted before the first hole is repaired.
    edge({1,0},{1,1},200,180,260);
    edge({1,1},{2,0},260,181,20);
    edge({1,1},{4,0},260,182,500);
    cb->rebuildOutgoingSrcs();
  }
  fpga::Wire edge(fpga::Coord from, fpga::Coord to, int dst, int src, int landing, int pos=1) {
    if (pos == 0) cb->local_src[dst].jump.setBit(src);
    else cb->dst_src[dst].jump.setBit(src);
    cb->rememberNodeName(fpga::CB_NODE_SRC,src,"node_s"+std::to_string(src));
    cb->rememberNodeName(fpga::CB_NODE_DST,landing,"node_d"+std::to_string(landing));
    fpga::CBType::ResolvedJump jump{}; jump.delta = to-from;
    jump.target_cb_type_id = 0; jump.dsts.jump.setBit(landing);
    cb->dst_by_src[src].push_back(jump);
    device.getTile(to.x,to.y)->incoming_dst_nodes.setBit(landing);
    fpga::Wire w; w.from=from; w.to=to; w.local=dst; w.jump=src; w.dst=landing; w.pos=pos;
    return w;
  }
  void lease(const std::vector<fpga::Wire> &route) {
    for (const auto &w : route) {
      auto &t = *device.getTile(w.from.x,w.from.y);
      if (w.type == fpga::Wire::WIRE_TILE_PIN) {
        t.cb.local.local.setBit(w.local); t.pin_state.leased_nodes.setBit(w.local);
      } else {
        if (w.pos) t.cb.dst.jump.setBit(w.local);
        if (w.jump >= 0) t.cb.src.jump.setBit(w.jump);
        if (w.joint >= 0) t.cb.joint.jump.setBit(w.joint);
      }
    }
  }
  void audit() {
    for (auto &tile : device.tile_grid) {
      std::ostringstream report;
      auto a = fpga::auditTileCongestion(tile,report,{&net});
      if (a.orphan_leases || a.missing_leases || a.missing_registrations || a.stale_registrations)
        throw std::runtime_error(report.str());
    }
  }
  void cut(int x, int src) {
    std::vector<fpga::RouteCutNode> nodes{{{x,0},src==33 ? fpga::CB_NODE_JOINT : fpga::CB_NODE_SRC,src}};
    auto victims = fpga::findNetRouteCutVictims(nodes);
    const auto preview = fpga::previewNetRouteCuts(victims,nodes,true);
    check(!preview.empty(),"partial cut has no release preview");
    check(std::none_of(preview.begin(),preview.end(),[](auto &n) {
      return n.type == fpga::CB_NODE_LOCAL;
    }),"partial preview releases a reserved endpoint pin");
    check(fpga::unrouteNetRoutesFromNodes(victims,nodes,true),"partial transit cut failed");
  }
};

void shared_hole_preserves_and_repairs_fanouts() {
  Fixture f;
  f.audit();
  rtl::Inst requester; Referable<rtl::Net> request_net;
  pnr::RouteDesign router;
  f.cb->dst_joint[201].joint.setBit(33);
  f.cb->joint_local[33].local.setBit(80);
  f.cb->rebuildOutgoingSrcs();
  pnr::RouteDesign::RouteTask request_task{.from=&requester,.to=&requester,.net=&request_net,
      .from_port="OUT",.to_port="IN",.net_name="request_719"};
  const auto grounding = router.preemptGroundingTerminal(*f.device.getTile(1,0),80,
      NodeMask{0,1} << 201,{},request_task);
  check(grounding.dst==201 && router.pending_route_todo.size()==8,
        "production grounding did not cut exactly the eight dependent fanouts");
  for (size_t i=0; i<32; ++i) {
    const auto &w=f.sinks[i].wires[0];
    check(fpga::routeState(w) == (i<8 ? fpga::RouteState::PartiallyRouted : fpga::RouteState::Routed),
          "shared cut changed the wrong fanouts or reported disconnected sinks complete");
    check(f.device.getTile(3,0)->cb.src.jump.testBit(400+i) &&
          f.device.getTile(4,0)->isPinNodeLeased(600+i),"cut released a downstream fanout");
  }
  f.audit();
  // A requester's grounding claims the released joint. The bypass must be
  // found while every downstream lease and this grounding remain held.
  fpga::Wire request; request.from=request.to={1,0}; request.local=201;
  request.pos=1; request.joint=33;
  fpga::Wire pin; pin.type=fpga::Wire::WIRE_TILE_PIN;
  pin.from=pin.to={1,0}; pin.local=80; pin.pin_dir=0;
  requester.wires.push_back({request,pin}); f.lease(requester.wires[0]);
  fpga::attachNetRoute(request_net,requester,0,&requester,&requester,"OUT","IN","request_719");
  fpga::registerNetRouteTiles(request_net,requester.wires[0],0);
  // Production entry point must repair before ordinary fanout continuation.
  pnr::RouteDesign::RouteTask task{.from=&f.driver,.to=&f.sinks[0],.net=&f.net,
      .from_port="OUT",.to_port="IN",.net_name="route_0",.fanout=true,.endpoints_prepared=true};
  check(router.routeNetTask(task),"production task did not repair retained shared fanouts");
  for (size_t i=0; i<32; ++i) check(fpga::isRouteComplete(f.sinks[i].wires[0]),"shared repair left a fanout disconnected");
  check(f.device.getTile(1,0)->cb.src.jump.testBit(180) &&
        f.device.getTile(1,1)->cb.src.jump.testBit(181),"repair did not take the bypass");
  // Remove the synthetic requester, then audit the victim alone.
  fpga::unrouteNet(request_net);
  f.audit();
}

void repeated_cuts_and_failed_repair_are_safe() {
  Fixture f;
  f.cut(1,33);
  f.cut(2,140); // extend just one of the eight holes, before any repair/pass
  f.audit();
  check(fpga::hasRouteGaps(f.sinks[0].wires[0]),"repeated cut lost partial state");
  const size_t before=f.sinks[0].wires[0].size();
  check(!fpga::repairNetRouteGaps(f.net,0,[]{return true;}),"cancelled repair succeeded");
  check(!fpga::repairNetRouteGaps(f.net,0,{},1),"state-limited repair succeeded");
  check(f.sinks[0].wires[0].size()==before,"failed repair changed retained storage");
  f.audit();
  // Also exhaust a genuinely disconnected topology, not just a budget. Keep
  // all old islands leased and verify the unsuccessful search changes none.
  const auto old_direct = f.cb->dst_src[200].jump;
  const auto old_joints = f.cb->dst_joint[200].joint;
  f.cb->dst_src[200].jump.clearBit(180);
  f.cb->dst_joint[200].joint.clearBit(33);
  f.cb->rebuildOutgoingSrcs();
  std::vector<fpga::CBState> states;
  for (const auto &tile : f.device.tile_grid) states.push_back(tile.cb);
  std::vector<fpga::Coord> audited;
  check(!fpga::repairNetRouteGaps(f.net,0,{},32768,200,[&](fpga::Tile &tile) {
    audited.push_back(tile.coord);
  }),"disconnected repair unexpectedly succeeded");
  check(std::any_of(audited.begin(),audited.end(),[](const auto &c) {
    return c.x==1 && c.y==0;
  }),"failed repair did not audit its actual starting frontier");
  check(std::any_of(audited.begin(),audited.end(),[](const auto &c) {
    return c.x==4 && c.y==0;
  }),"failed repair did not audit its retained destination boundary");
  for (size_t i=0; i<states.size(); ++i) {
    const auto &a=states[i], &b=f.device.tile_grid[i].cb;
    check(a.src.jump==b.src.jump && a.dst.jump==b.dst.jump &&
          a.joint.jump==b.joint.jump && a.local.local==b.local.local,
          "unsuccessful repair mutated routing leases");
  }
  check(f.sinks[0].wires[0].size()==before,"exhausted repair changed route storage");
  f.cb->dst_src[200].jump=old_direct;
  f.cb->dst_joint[200].joint=old_joints;
  f.cb->rebuildOutgoingSrcs();
  check(fpga::repairNetRouteGaps(f.net,0),"extended hole could not reconnect");
  check(fpga::repairNetRouteGaps(f.net,1),"remaining shared holes could not reconnect");
  for (auto &sink:f.sinks) check(fpga::isRouteComplete(sink.wires[0]),"repair did not finish all bindings");
  f.audit();
  for (const auto &cut : {fpga::RouteCutNode{{0,0},fpga::CB_NODE_SRC,100},
                         fpga::RouteCutNode{{4,0},fpga::CB_NODE_DST,500}}) {
    auto victims=fpga::findNetRouteCutVictims({cut});
    check(!fpga::unrouteNetRoutesFromNodes(victims,{cut},true),"preemption removed a takeoff or grounding");
  }
  f.audit();
}

void mandatory_grounding_keeps_transit_fanouts() {
  Fixture f;
  f.cb->constant_one_nodes.setBit(4);
  f.cb->local_joint[4].joint.setBit(33);
  f.cb->joint_local[33].local.setBit(80);
  f.cb->rebuildOutgoingSrcs();
  fpga::TileType type{"resource_419",0};
  type.pin_map.nodes[{"sink_kind","input",0}].setBit(80);
  auto &tile=f.device.tile_grid[1]; tile.tile_type=&type;
  Referable<rtl::Cell> source_cell, sink_cell;
  source_cell.type="distributed_kind"; sink_cell.type="sink_kind";
  rtl::Inst source, sink;
  source.cell_ref.set(&source_cell); sink.cell_ref.set(&sink_cell);
  sink.tile.set(&tile); sink.pos=0;
  Referable<rtl::Net> net;
  net.name="mandatory_907"; net.route_protected=true; net.distributed_source=true;
  pnr::RouteDesign router; router.fpga=&f.device; router.moving_stage=true;
  pnr::RouteDesign::RouteTask task{&source,&sink,&net,"out","input",net.name};
  check(router.routeDistributedLocalTask(task),"mandatory grounding did not preempt genuine transit");
  check(router.pending_route_todo.size()==8,"mandatory grounding tore down unrelated fanouts");
  for (size_t i=0; i<32; ++i)
    check(fpga::routeState(f.sinks[i].wires[0]) ==
        (i<8 ? fpga::RouteState::PartiallyRouted : fpga::RouteState::Routed),
        "mandatory grounding discarded downstream routing");
  check(fpga::repairNetRouteGaps(f.net,0),"mandatory grounding victim could not repair around endpoint");
  for (auto &s:f.sinks) check(fpga::isRouteComplete(s.wires[0]),"mandatory grounding repair left incomplete fanouts");
  for (auto &t:f.device.tile_grid) {
    std::ostringstream report;
    auto a=fpga::auditTileCongestion(t,report,{&f.net,&net});
    if (a.orphan_leases || a.missing_leases || a.missing_registrations || a.stale_registrations)
      throw std::runtime_error(report.str());
  }
}

// A last-hop transit cut must not require retaining the old incoming DST at
// the sink. Keep its LOCAL pin reserved, and repair only this branch's tail
// through another arrival. Direct requests for a Grounding node stay illegal.
void terminal_tail_retargets_arrival() {
  Fixture f;
  const fpga::RouteCutNode cut{{3,0},fpga::CB_NODE_SRC,400};
  const fpga::RouteCutNode grounding{{4,0},fpga::CB_NODE_DST,500};
  auto ground_victims=fpga::findNetRouteCutVictims({grounding});
  check(fpga::previewNetRouteCuts(ground_victims,{grounding},true).empty(),
        "direct grounding preemption was allowed");
  const auto victims=fpga::findNetRouteCutVictims({cut});
  const auto preview=fpga::previewNetRouteCuts(victims,{cut},true);
  check(!preview.empty(),"last transit before grounding is incorrectly uncuttable");
  check(std::none_of(preview.begin(),preview.end(),[](const auto &n) {
    return n.type==fpga::CB_NODE_LOCAL;
  }),"terminal repair releases the sink pin");
  check(fpga::unrouteNetRoutesFromNodes(victims,{cut},true),"terminal-tail cut failed");
  auto &tail=f.sinks[0].wires[0];
  check(tail[tail.size()-2].type==fpga::Wire::WIRE_GAP &&
        tail[tail.size()-2].to_node_type==fpga::CB_NODE_LOCAL &&
        tail[tail.size()-2].to_node==600,"terminal gap is tied to old DST");
  check(!f.device.getTile(4,0)->cb.dst.jump.testBit(500) &&
        f.device.getTile(4,0)->isPinNodeLeased(600),"old arrival retained or endpoint lost");
  for(size_t i=1;i<32;++i) check(fpga::isRouteComplete(f.sinks[i].wires[0]),
                               "terminal cut disturbed another fanout");
  f.audit();
  // Remove the old physical jump from the topology: success must really use
  // a different incoming DST, not silently reacquire the preempted hop.
  f.cb->dst_by_src[400].clear();
  // Whole-source repair can also reach this sink through the earlier branch.
  // Remove that real alternative too for this unreachable negative control.
  f.cb->dst_by_src[182].clear();
  f.cb->rebuildOutgoingSrcs();
  check(!fpga::repairNetRouteGaps(f.net,0),"unreachable terminal repair succeeded");
  f.audit();
  f.edge({3,0},{3,1},220,800,900);
  f.edge({3,1},{4,0},900,801,901);
  f.cb->dst_joint[901].joint.setBit(70);
  f.cb->joint_joint[70].joint.setBit(71);
  f.cb->joint_local[71].local.setBit(600);
  f.cb->rebuildOutgoingSrcs();
  // A blocked terminal joint must not be treated as free just because the
  // reserved LOCAL pin belongs to this victim. Failed repair stays read-only.
  auto &terminal_tile=*f.device.getTile(4,0);
  terminal_tile.cb.joint.jump.setBit(71);
  const auto before=terminal_tile.cb;
  check(!fpga::repairNetRouteGaps(f.net,0),"terminal repair crossed an occupied joint");
  check(before.joint.jump==terminal_tile.cb.joint.jump &&
        before.dst.jump==terminal_tile.cb.dst.jump &&
        before.src.jump==terminal_tile.cb.src.jump,
        "failed terminal repair changed leases");
  terminal_tile.cb.joint.jump.clearBit(71);
  pnr::RouteDesign router;
  pnr::RouteDesign::RouteTask task{.from=&f.driver,.to=&f.sinks[0],.net=&f.net,
      .from_port="OUT",.to_port="IN",.net_name="route_0",.fanout=true,.endpoints_prepared=true};
  check(router.routeNetTask(task),"production terminal repair failed");
  const auto &terminal=tail[tail.size()-2];
  check(terminal.local==901 && terminal.joint2==70 && terminal.joint==71,
        "repair did not use the new two-joint arrival");
  for(auto &sink:f.sinks) check(fpga::isRouteComplete(sink.wires[0]),"repair left a fanout incomplete");
  f.audit();
}
} // namespace

void route_owner_query_regressions() {
  Fixture f;
  f.audit();
  auto &tile = *f.device.getTile(1,0);
  // Compare the predicate query with exhaustive physical-owner enumeration
  // on every occupied node, including shared replicas and endpoint resources.
  for (bool authoritative : {true, false}) {
    for (auto &t : f.device.tile_grid) {
      t.routed_bindings_authoritative = authoritative;
      for (auto kind : {fpga::CB_NODE_LOCAL, fpga::CB_NODE_DST,
                        fpga::CB_NODE_SRC, fpga::CB_NODE_JOINT}) {
        fpga::congestionNodeMask(t.cb,kind).for_each_set_bit([&](int node) {
          const auto owners = fpga::findNetOwnersByNode(t,kind,node);
          size_t visited = 0;
          const bool all = fpga::allNetOwnersMatch(t,kind,node,[&](const auto &ref) {
            check(visited < owners.size() && ref.net == owners[visited].net &&
                  ref.binding_index == owners[visited].binding_index,
                  "predicate query changed physical owner order or identity");
            ++visited;
            return true;
          });
          check(all == !owners.empty() && visited == owners.size(),
                "predicate query omitted an owner or accepted an orphan");
          visited = 0;
          check(!fpga::allNetOwnersMatch(t,kind,node,[&](const auto &) {
            ++visited; return false;
          }) && visited == size_t(!owners.empty()),
                "rejection did not stop on the first physical owner");
          return false;
        });
      }
    }
    // Shared transit copies do not own the source node. An explicit shared
    // landing does own its DST, and must not be lost by the fast query.
    check(fpga::findNetOwnersByNode(tile,fpga::CB_NODE_SRC,120).size() == 1,
          "shared transit replica counted as an owner");
    check(!fpga::allNetOwnersMatch(*f.device.getTile(3,0),fpga::CB_NODE_SRC,120,
          [](const auto &) { return true; }), "matching numeric ID on another Tile was accepted");
    // Explicit numeric edges own only their output, not their input reference.
    auto &edge = f.sinks[0].wires[0][2];
    const auto saved_edge = edge;
    edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
    edge.from = edge.to = {1,0};
    edge.from_node_type = fpga::CB_NODE_DST; edge.from_node = 904;
    edge.to_node_type = fpga::CB_NODE_JOINT; edge.to_node = 905;
    check(!fpga::allNetOwnersMatch(tile,fpga::CB_NODE_DST,904,
          [](const auto &) { return true; }) &&
          !fpga::findNetRoutesByNode(tile,fpga::CB_NODE_DST,904).empty(),
          "explicit edge input reference became a lease owner");
    check(fpga::allNetOwnersMatch(tile,fpga::CB_NODE_JOINT,905,
          [](const auto &) { return true; }), "explicit edge output owner was filtered out");
    edge.shared = true;
    check(!fpga::allNetOwnersMatch(tile,fpga::CB_NODE_JOINT,905,
          [](const auto &) { return true; }), "shared explicit edge became a lease owner");
    edge = saved_edge;
    auto &replica = f.sinks[1].wires[0][2];
    replica.owns_landing = true;
    auto &landing = *f.device.getTile(2,0);
    bool landing_found = false;
    check(fpga::allNetOwnersMatch(landing,fpga::CB_NODE_DST,20,[&](const auto &ref) {
      landing_found |= ref.binding_index == 1;
      return true;
    }) && landing_found, "explicit shared landing was not an owner");
    replica.owns_landing = false;

    // An acceptable first owner cannot hide a later foreign source. Add three
    // physical claims, then prove rejection stops at the second, not the third.
    replica.shared = false;
    f.sinks[2].wires[0][2].shared = false;
    rtl::Inst other_source;
    f.net.routes[1].from = &other_source;
    size_t visited = 0;
    check(!fpga::allNetOwnersMatch(tile,fpga::CB_NODE_SRC,120,[&](const auto &ref) {
      ++visited;
      return ref.net->routes[ref.binding_index].from == &f.driver;
    }) && visited == 2, "later foreign owner was ignored or query did not short-circuit");
    f.net.routes[1].from = &f.driver;
    visited = 0;
    check(fpga::allNetOwnersMatch(tile,fpga::CB_NODE_SRC,120,[&](const auto &) {
      ++visited; return true;
    }) && visited == 3, "acceptance did not validate every physical owner");
    replica.shared = true;
    f.sinks[2].wires[0][2].shared = true;

    // A busy bit without a live owner is never permission to reuse/preempt.
    tile.cb.src.jump.setBit(999);
    check(!fpga::allNetOwnersMatch(tile,fpga::CB_NODE_SRC,999,[](const auto &) {
      return true;
    }), "orphan lease was accepted");
    tile.cb.src.jump.clearBit(999);
  }
  for (auto &t : f.device.tile_grid) t.routed_bindings_authoritative = true;
  tile.routed_bindings.insert(tile.routed_bindings.begin(), {&f.net, UINT64_MAX});
  tile.routed_bindings.insert(tile.routed_bindings.begin(), {nullptr, 0});
  check(fpga::allNetOwnersMatch(tile,fpga::CB_NODE_SRC,120,[](const auto &) {
    return true;
  }), "stale/null registrations masked a live owner");
  tile.routed_bindings.erase(tile.routed_bindings.begin(),tile.routed_bindings.begin()+2);
  check(!fpga::allNetOwnersMatch(*f.device.getTile(0,0),fpga::CB_NODE_SRC,100,
        [](const auto &) { return true; },true), "Takeoff counted as a transit owner");
  // Report, but do not assert, elapsed time. Correctness and short-circuiting
  // above are deterministic; performance thresholds would depend on the host.
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 20000; ++i) {
    const auto owners = fpga::findNetOwnersByNode(tile,fpga::CB_NODE_SRC,120);
    check(!owners.empty() && !std::all_of(owners.begin(),owners.end(),
          [](const auto &) { return false; }), "exhaustive control accepted rejection");
  }
  const auto middle = std::chrono::steady_clock::now();
  for (int i = 0; i < 20000; ++i)
    check(!fpga::allNetOwnersMatch(tile,fpga::CB_NODE_SRC,120,
          [](const auto &) { return false; }), "short-circuit control accepted rejection");
  const auto end = std::chrono::steady_clock::now();
  std::printf("owner query 20000 rejects: exhaustive_ms=%.3f short_circuit_ms=%.3f\n",
      std::chrono::duration<double,std::milli>(middle-begin).count(),
      std::chrono::duration<double,std::milli>(end-middle).count());
  f.audit();
}

void partial_preemption_regressions() {
  shared_hole_preserves_and_repairs_fanouts();
  repeated_cuts_and_failed_repair_are_safe();
  mandatory_grounding_keeps_transit_fanouts();
  terminal_tail_retargets_arrival();
}
