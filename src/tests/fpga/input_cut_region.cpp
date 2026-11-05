#include "SourceInputFixture.h"
#include "route/MovingInputCutCache.h"

// Native source recovery repeated the same 735929-state failed input walk
// for distinct transit cuts outside its reachable region. The independent
// pin boundary was unavailable, so exact-release-set caching could not help.
// Build a smaller converging fabric and real unrelated transit owners.
void input_cut_region_regression() {
  using namespace routing_tests;
  SourceInputFixture f(false,0,4,24);
  f.edge({4,1},{4,2},712,1500,200);
  for(int x=4;x<=23;++x)for(int a=0;a<64;++a)for(int b=0;b<64;++b)
    f.edge({x,2},x==23?fpga::Coord{2,2}:fpga::Coord{x+1,2},
        200+a,1000+b,x==23?500+b:200+b);
  for(int b=0;b<64;++b)f.terminal({2,2},500+b,87);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();f.auditAll();
  auto index=pnr::buildBackwardResolveIndex(f.device,{2,2},24);
  std::vector<pnr::BackwardRouteAnchor> anchors{{&f.tile({5,1}),711,{},1}};
  pnr::MovingInputCutCache cache;
  pnr::MovingInputCutCache::Key key{&f.tile({2,2}),NodeMask{0,1}<<87,
      {5,1},0,24,200,&f.tile({5,1}),711};
  size_t calls=0,failed_expanded=0,first_expanded=0,hits=0;
  auto prove=[&](const std::vector<fpga::RouteCutNode>& cuts) {
    return cache.prove(key,cuts,[&](const pnr::InputRouteBlocked& blocked) {
      ++calls;
      pnr::BackwardTakeoffStateView view;
      for(const auto& cut:cuts) {
        auto& tile=f.tile(cut.tile);
        auto [it,inserted]=view.try_emplace(&tile,tile.cb);
        auto& mask=cut.type==fpga::CB_NODE_SRC?it->second.src.jump:it->second.dst.jump;
        // Every proposed release has an actual registered owner, not fake
        // congestion bits. Only the private proof view is changed here.
        require(mask.testBit(cut.node),"cut does not release a real lease");
        mask.clearBit(cut.node);
      }
      const auto result=pnr::prepareInputRoute(*key.target,key.pins,key.source_hint,24,
          {},&index,{},&view,&anchors,200,{},blocked);
      if(!result.success) {
        if(!first_expanded)first_expanded=result.expanded;
        failed_expanded+=result.expanded;
      }
      return result.success;
    },[]{return false;});
  };
  for(auto cut:std::vector<fpga::RouteCutNode>{
      {{2,1},fpga::CB_NODE_SRC,915},{{1,1},fpga::CB_NODE_SRC,916},
      {{0,1},fpga::CB_NODE_SRC,917}}) {
    auto result=prove({cut});
    require(!result.possible,"unrelated cut opened the isolated input region");
    hits+=result.reused;
  }
  // This cut really opens the boundary and reaches the retained anchor. It
  // must be searched and succeed, rather than reusing the earlier failure.
  auto opened=prove({{{4,1},fpga::CB_NODE_DST,712},{{5,1},fpga::CB_NODE_SRC,912}});
  require(opened.possible && !opened.reused,"a boundary-opening cut was incorrectly rejected");
  f.auditAll();
  std::cout<<"input cut region calls="<<calls<<" failed_expanded="<<failed_expanded
      <<" first_expanded="<<first_expanded<<" reused="<<hits<<'\n';
  // Require one exhausted proof and one successful boundary-opening proof.
  require(first_expanded>=2000,"fixture did not build a substantial input region");
  require(calls==2 && hits==2 && failed_expanded==first_expanded,
      "different irrelevant cuts repeated the same exhausted input region");
}
