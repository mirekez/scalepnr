#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>

static void require(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}

// A broad source repair revisits a working set just beyond the old 64-Tile
// owner-index limit. Every busy SRC belongs to a real two-hop signal. Revisiting
// those Tiles must not repeatedly rebuild the unchanged ownership information.
// Node names are synthetic; the criterion is work counters, not elapsed time.
void owner_working_set_regression() {
  constexpr int count=65, passes=8;
  auto &device=fpga::Device::current();
  device.tile_grid.clear(); device.cb_types.clear();
  device.grid_spec.size={count,3};device.size_width=count;device.size_height=3;
  device.tile_grid.resize(count*3);
  auto &cb=device.cb_types.emplace_back();cb.type_id=cb.base_type_id=0;
  cb.name="pattern_owner_matrix_719";
  for(int y=0;y<3;++y)for(int x=0;x<count;++x) {
    auto &t=*device.getTile(x,y);t.coord=t.cb_coord={x,y};t.cb_type=t.cb.type=&cb;
  }
  for(auto [kind,node]:{std::pair{fpga::CB_NODE_LOCAL,100},{fpga::CB_NODE_LOCAL,501},
       {fpga::CB_NODE_SRC,101},{fpga::CB_NODE_SRC,301},
       {fpga::CB_NODE_DST,201},{fpga::CB_NODE_DST,401}})
    cb.rememberNodeName(kind,node,"pattern_node_"+std::to_string(kind)+"_"+std::to_string(node));
  cb.local_src[100].jump.setBit(101);cb.dst_src[201].jump.setBit(301);
  cb.dst_local[401].local.setBit(501);
  for(auto [src,dst]:{std::pair{101,201},{301,401}}) {
    fpga::CBType::ResolvedJump j{};j.delta={0,1};j.target_tile_coord=true;
    j.target_cb_type_id=0;j.dsts.jump.setBit(dst);cb.dst_by_src[src].push_back(j);
  }
  cb.rebuildOutgoingSrcs();device.rebuildIncomingDstMasks();
  struct Signal {Referable<rtl::Net> net;Referable<rtl::Cell> source_cell,sink_cell;rtl::Inst source,sink;};
  std::array<Signal,count> signals;
  std::vector<rtl::Net*> nets;
  for(int x=0;x<count;++x) {
    auto &s=signals[x];s.net.name="pattern_signal_"+std::to_string(x);
    s.source_cell.name="pattern_driver_"+std::to_string(x);s.source_cell.type="LUT6";
    s.sink_cell.name="pattern_sink_"+std::to_string(x);s.sink_cell.type="LUT6";
    s.source.cell_ref.set(&s.source_cell);s.sink.cell_ref.set(&s.sink_cell);
    s.source.tile.set(&device.tile_grid[x]);s.sink.tile.set(&device.tile_grid[2*count+x]);
    std::vector<fpga::Wire> route;
    fpga::Wire pin;pin.type=fpga::Wire::WIRE_TILE_PIN;pin.from=pin.to=pin.resource={x,0};
    pin.local=100;pin.pin_dir=1;route.push_back(pin);
    fpga::Wire first;first.from={x,0};first.to={x,1};first.pos=0;first.local=100;
    first.jump=first.route_jump=101;first.dst=201;route.push_back(first);
    fpga::Wire second;second.from={x,1};second.to={x,2};second.pos=1;second.local=201;
    second.jump=second.route_jump=301;second.dst=401;route.push_back(second);
    fpga::Wire end;end.from=end.to={x,2};end.pos=1;end.local=401;route.push_back(end);
    pin.from=pin.to=pin.resource={x,2};pin.local=501;pin.pin_dir=0;route.push_back(pin);
    s.sink.wires.push_back(std::move(route));
    auto b=fpga::attachNetRoute(s.net,s.sink,0,&s.source,&s.sink,"OUT","IN",s.net.name);
    fpga::registerNetRouteTiles(s.net,s.sink.wires[0],b);nets.push_back(&s.net);
    auto &a=*device.getTile(x,0),&m=*device.getTile(x,1),&z=*device.getTile(x,2);
    a.cb.local.local.setBit(100);a.pin_state.leased_nodes.setBit(100);a.cb.src.jump.setBit(101);
    m.cb.dst.jump.setBit(201);m.cb.src.jump.setBit(301);
    z.cb.dst.jump.setBit(401);z.cb.local.local.setBit(501);z.pin_state.leased_nodes.setBit(501);
    auto jump=device.resolveJumpTargets(a,101);
    require(jump.size()==1 && jump[0].tile==&m && jump[0].dst_node==201,"invalid first physical hop");
    jump=device.resolveJumpTargets(m,301);
    require(jump.size()==1 && jump[0].tile==&z && jump[0].dst_node==401,"invalid second physical hop");
    require(fpga::isRouteComplete(s.sink.wires[0]),"incomplete fixture route");
  }
  auto audit=[&] {
    fpga::CongestionAuditIndex index(nets);
    for(auto &t:device.tile_grid) {
      std::ostringstream text;auto a=fpga::auditTileCongestion(t,text,index);
      require(!a.orphan_leases && !a.missing_leases && !a.conflicting_driver_nodes &&
              !a.missing_registrations && !a.stale_registrations,"fixture ownership audit failed");
    }
  };
  audit();
  fpga::RouteOwnerQueries owners;
  size_t predicates=0;
  for(int pass=0;pass<passes;++pass)for(int x=0;x<count;++x) {
    require(owners.allMatch(*device.getTile(x,1),fpga::CB_NODE_SRC,301,
        [&](const fpga::NetRouteRef &r) {
          ++predicates;return r.net==&signals[x].net && r.binding_index==0;
        },true),"ownership changed while walking the same working set");
  }
  auto work=owners.work();
  std::cout<<"owner working set: tiles="<<count<<" passes="<<passes
      <<" builds="<<work.tile_builds<<" bindings="<<work.binding_resolutions
      <<" fragments="<<work.fragment_visits<<" predicates="<<predicates<<std::endl;
  require(predicates==count*passes && work.queries==count*passes,"owner predicates were cached");
  require(work.tile_builds==count && work.binding_resolutions==count,
          "unchanged ownership working set was repeatedly discarded and rebuilt");
  require(!owners.allMatch(*device.getTile(1,1),fpga::CB_NODE_SRC,301,
          [](const auto &){return false;}),"a cached answer bypassed the live predicate");
  require(fpga::unrouteNetRoute(signals[0].net,0),"mutation control did not unroute");
  require(!owners.allMatch(*device.getTile(0,1),fpga::CB_NODE_SRC,301,
          [](const auto &){return true;}),"removed owner remained cached");
  require(owners.allMatch(*device.getTile(1,1),fpga::CB_NODE_SRC,301,
          [&](const auto &r){return r.net==&signals[1].net;}),"mutation damaged another owner");
  require(owners.work().tile_builds==count+2,"mutation did not rebuild the affected query view");
  audit();
}
