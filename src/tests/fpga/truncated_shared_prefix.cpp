#include "SourceInputFixture.h"

using routing_tests::SourceInputFixture;
using routing_tests::require;

void truncated_shared_prefix_regression() {
  // Example 1, chunk 6: destination relocation shortened an incomplete owner
  // before a hop still borrowed by two completed siblings. truncateNetRoute
  // released its SRC because both survivors were marked shared. A later
  // ownership promotion did not reacquire that missing lease.
  // Reproduce the retained GAP, shared physical prefix and real fork tails;
  // all node and instance labels are abstract fixture patterns.
  for (size_t keep : {6u,2u}) {
    SourceInputFixture f(false,0,16);
    auto& owner=f.source->wires[0];
    for (auto& w:owner) w.owns_landing=false; // Ordinary next-hop DST ownership.
    const auto original_id=f.enable->routes[0].route_id;
    std::vector<rtl::Inst*> siblings;
    for (int n=0;n<2;++n) {
      auto* sink=f.cell("pattern_survivor_"+std::to_string(n),"FDRE",{{"CE",false}},{1+n,0});
      auto& input=SourceInputFixture::port(*sink,"CE");
      input.port_ref->designator=SourceInputFixture::port(*f.control,"O").port_ref->designator;
      input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
      std::vector<fpga::Wire> prefix(owner.begin(),owner.begin()+7);
      for (auto& w:prefix) { w.shared=true;w.owns_landing=false; }
      std::vector<fpga::Wire> route;
      auto fork=f.edge({1,1},{1+n,0},715,1400+n,1500+n);
      fork.pos=2;fork.owns_dst=false;
      route.push_back(fork);
      route.push_back(f.terminal({1+n,0},1500+n,63));
      route.push_back(SourceInputFixture::pin({1+n,0},63,false));
      for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
      f.device.rebuildIncomingDstMasks();
      require(pnr::commitPreparedRoute(route),"cannot install the real sibling fork");
      route.insert(route.begin(),prefix.begin(),prefix.end());
      sink->wires.push_back(std::move(route));
      const auto binding=fpga::attachNetRoute(*f.enable,*sink,0,f.control,sink,"O","CE",
          "pattern_branch_"+std::to_string(n));
      fpga::registerNetRouteTiles(*f.enable,sink->wires[0],binding);
      siblings.push_back(sink);
    }
    // Cut only the original route beyond the siblings' branch point. Its
    // surviving prefix remains the owner of SRC 915 in (2,1).
    std::vector<fpga::RouteCutNode> cuts{{{0,1},fpga::CB_NODE_SRC,917}};
    const auto victims=fpga::findNetRouteCutVictims(cuts);
    require(fpga::unrouteNetRoutesFromNodes(victims,cuts,true),"cannot create the retained owner GAP");
    require(fpga::hasRouteGaps(owner),"owner must be incomplete before relocation truncates it");
    f.auditAll();
    const auto before0=siblings[0]->wires[0].size(), before1=siblings[1]->wires[0].size();
    const bool truncated=keep==2 ? fpga::unrouteNetRouteToTakeoff(*f.enable,0)
        : fpga::truncateNetRoute(*f.enable,0,keep);
    require(truncated,"destination prefix truncation failed");
    require(owner.size()==keep && f.enable->routes[0].route_id==original_id,
        "truncation changed the wrong binding or retained the removed tail");
    require(std::none_of(owner.begin(),owner.end(),[](const auto& w){return w.shared;}),
        "truncation transferred ownership of the prefix that was not removed");
    require(f.tile({2,1}).cb.src.jump.testBit(915),
        "truncation freed a shared SRC still used by completed siblings; keep="+std::to_string(keep));
    auto owners=fpga::findNetOwnersByNode(f.tile({2,1}),fpga::CB_NODE_SRC,915);
    require(owners.size()==1 && owners.front().net==f.enable && owners.front().binding_index!=0,
        "truncation did not transfer exactly one live owner to a surviving sibling");
    require(siblings[0]->wires[0].size()==before0 && siblings[1]->wires[0].size()==before1 &&
        fpga::isRouteComplete(siblings[0]->wires[0]) && fpga::isRouteComplete(siblings[1]->wires[0]),
        "truncation damaged a completed sibling");
    f.auditAll();
    // Removing the promoted owner transfers the hop once more; removing the
    // last user releases it. Thus the fix cannot simply pin the busy bit forever.
    require(fpga::unrouteNetRoute(*f.enable,1),"cannot remove first sibling");
    require(f.tile({2,1}).cb.src.jump.testBit(915),"second sibling lost the shared hop");
    f.auditAll();
    require(fpga::unrouteNetRoute(*f.enable,2),"cannot remove last sibling");
    require(!f.tile({2,1}).cb.src.jump.testBit(915),"last user left an orphan SRC lease");
    f.auditAll();
  }
}
