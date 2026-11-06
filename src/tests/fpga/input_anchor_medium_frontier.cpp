#include "SourceInputFixture.h"
#include "route/MovingInputCutCache.h"

// A native input has 8,290 forward-reachable nodes, while its failed reverse
// proofs scan over a million nodes. A 4,096-label preliminary probe misses
// that negative witness. Reproduce the imbalance without device names.
void input_anchor_medium_frontier_regression() {
  using namespace routing_tests;
  SourceInputFixture f(false,0,4,300,3);
  constexpr int reverse_count=(1<<20)-1, forward_count=8290;
  auto reverse=[](int n) {return fpga::Coord{24+n/CB_MAX_NODES,0};};
  auto forward=[](int n) {return fpga::Coord{288+n/CB_MAX_NODES,1};};
  auto id=[](int n) {return n%CB_MAX_NODES;};
  auto matrix=[&](fpga::Coord at) {
    auto& cb=f.device.cb_types.emplace_back();
    cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
    cb.name="FILL_MATRIX_"+std::to_string(cb.type_id);
    auto& tile=f.tile(at);tile.cb_type=tile.cb.type=&cb;
  };
  for(int n=0;n<reverse_count;n+=CB_MAX_NODES)matrix(reverse(n));
  for(int n=0;n<forward_count;n+=CB_MAX_NODES)matrix(forward(n));
  for(int n=1;n<reverse_count;++n)
    f.resolvedLocal(reverse(n),id(n),reverse((n-1)/2),fpga::CB_NODE_LOCAL,id((n-1)/2));
  for(int n=1;n<forward_count;++n)
    f.resolvedLocal(forward((n-1)/2),id((n-1)/2),forward(n),fpga::CB_NODE_LOCAL,id(n));
  auto& anchor=f.tile({5,1});
  anchor.cb_type->dst_joint[711].joint.setBit(200);
  anchor.cb_type->joint_local[200].local.setBit(600);
  SourceInputFixture::name(anchor,fpga::CB_NODE_JOINT,200);
  f.resolvedLocal({5,1},600,forward(0),fpga::CB_NODE_LOCAL,0);
  f.edge({4,1},reverse(0),712,1500,4000);
  f.terminal(reverse(0),4000,0);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto& target=f.tile(reverse(0));
  auto index=pnr::buildBackwardResolveIndex(f.device,target.coord,300);
  std::vector<pnr::BackwardRouteAnchor> anchors{{&anchor,711,{},1}};
  NodeMask pins=NodeMask{0,1};
  // The exact path engine proves this fixture really has the large fan-in;
  // no artificially low work budget or invented occupancy may pass the test.
  auto control=pnr::prepareInputRoute(target,pins,anchor.coord,300,{},&index,{},nullptr,&anchors,200);
  require(!control.success && control.expanded>1000000,"large reverse fan-in was not reproduced");
  pnr::MovingInputCutCache cache;
  pnr::MovingInputCutCache::Key key{&target,pins,anchor.coord,0,300,200,&anchor,711};
  size_t calls=0,negative_work=0,positive_work=0;
  auto prove=[&](const std::vector<fpga::RouteCutNode>& cuts) {
    return cache.prove(key,cuts,[&](const pnr::InputRouteBlocked& blocked) {
      ++calls;
      pnr::BackwardTakeoffStateView view;
      for(const auto& cut:cuts) {
        auto& tile=f.tile(cut.tile);
        auto& state=view.try_emplace(&tile,tile.cb).first->second;
        auto& mask=cut.type==fpga::CB_NODE_SRC?state.src.jump:state.dst.jump;
        require(mask.testBit(cut.node),"preview release has no live owner");
        mask.clearBit(cut.node);
      }
      size_t work=0;
      bool possible=pnr::inputMayReachSource(target,pins,anchor.coord,300,{},&index,{},
          &view,&anchors,200,blocked,nullptr,&work);
      (possible?positive_work:negative_work)+=work;
      return possible;
    },[]{return false;});
  };
  // Unrelated transit cuts cannot connect the source island; preserve and
  // reuse its actual occupied boundary, not an empty negative witness.
  for(auto cut:std::vector<fpga::RouteCutNode>{
      {{2,1},fpga::CB_NODE_SRC,915},{{1,1},fpga::CB_NODE_SRC,916},
      {{0,1},fpga::CB_NODE_SRC,917}})
    require(!prove({cut}).possible,"unrelated release connected the isolated input");
  auto opened=prove({{{5,1},fpga::CB_NODE_SRC,912},{{4,1},fpga::CB_NODE_DST,712}});
  // A useful release must invalidate the negative proof and find the short
  // route. Do not pay the larger source probe for this cheap positive case.
  require(opened.possible && !opened.reused,"useful source release was hidden by a negative proof");
  f.auditAll();
  std::cout<<"FILL_MEDIUM_FRONTIER control="<<control.expanded<<" negative="<<negative_work
      <<" positive="<<positive_work<<" calls="<<calls<<'\n';
  require(calls==2,"unchanged failed proof was searched again");
  require(negative_work>forward_count && negative_work<50000,
      "medium closed source component triggered a million-node reverse proof");
  require(positive_work<20000,"cheap positive input paid for the enlarged preliminary search");
}
