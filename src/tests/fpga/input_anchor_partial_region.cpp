#include "SourceInputFixture.h"
#include "route/InputRouteRegion.h"

// An early source-island negative may interrupt a single-file reverse walk
// immediately after its last queued node is popped. That is not exhaustion:
// the popped node still has predecessors, including a second valid anchor.
void input_anchor_partial_region_regression() {
  using namespace routing_tests;
  SourceInputFixture f(false,0,4,64,3);
  constexpr int path_count=32769, island_count=8290;
  auto path=[](int n){return fpga::Coord{24+n/CB_MAX_NODES,0};};
  auto island=[](int n){return fpga::Coord{40+n/CB_MAX_NODES,1};};
  auto id=[](int n){return n%CB_MAX_NODES;};
  auto matrix=[&](fpga::Coord at) {
    auto& cb=f.device.cb_types.emplace_back();
    cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
    cb.name="FILL_PARTIAL_MATRIX_"+std::to_string(cb.type_id);
    auto& t=f.tile(at);t.cb_type=t.cb.type=&cb;
  };
  for(int n=0;n<path_count;n+=CB_MAX_NODES)matrix(path(n));
  for(int n=0;n<island_count;n+=CB_MAX_NODES)matrix(island(n));
  for(int n=1;n<path_count;++n)
    f.resolvedLocal(path(n),id(n),path(n-1),fpga::CB_NODE_LOCAL,id(n-1));
  for(int n=1;n<island_count;++n)
    f.resolvedLocal(island((n-1)/2),id((n-1)/2),island(n),fpga::CB_NODE_LOCAL,id(n));
  auto& isolated=f.tile({5,1});
  isolated.cb_type->dst_joint[711].joint.setBit(200);
  isolated.cb_type->joint_local[200].local.setBit(600);
  SourceInputFixture::name(isolated,fpga::CB_NODE_JOINT,200);
  f.resolvedLocal({5,1},600,island(0),fpga::CB_NODE_LOCAL,0);
  auto& connected=f.tile(path(path_count-1));
  connected.cb_type->dst_local[900].local.setBit(id(path_count-1));
  SourceInputFixture::name(connected,fpga::CB_NODE_DST,900);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto& target=f.tile(path(0));
  auto index=pnr::buildBackwardResolveIndex(f.device,target.coord,64);
  std::vector<pnr::BackwardRouteAnchor> anchors{{&isolated,711,{},1}};
  pnr::InputRouteRegionCache region;
  size_t negative_work=0;
  // The medium forward island must stop the long reverse proof at the
  // 32,768th pop, with the continuation to the other anchor still unexpanded.
  require(!pnr::inputMayReachSource(target,NodeMask{0,1},target.coord,64,{},&index,
      {},nullptr,&anchors,200,{},&region,&negative_work),"isolated anchor unexpectedly reached target");
  require(negative_work>32768 && negative_work<50000,"early-negative boundary was not exercised");
  const auto saved_tiles=region.reached.size();
  anchors={{&connected,900,{},2}};
  // An uncached control proves that the second anchor really reaches the
  // same target through the unexpanded end of this otherwise thin path.
  size_t control_work=0;
  require(pnr::inputMayReachSource(target,NodeMask{0,1},target.coord,64,{},&index,
      {},nullptr,&anchors,200,{},nullptr,&control_work),"connected control did not reach target");
  region.reuse_allowed=true;
  size_t cached_work=0;
  const bool cached=pnr::inputMayReachSource(target,NodeMask{0,1},target.coord,64,
      {},&index,{},nullptr,&anchors,200,{},&region,&cached_work);
  std::cout<<"FILL_PARTIAL_REGION negative="<<negative_work<<" saved_tiles="<<saved_tiles
      <<" control="<<control_work<<" cached="<<cached_work<<" possible="<<cached<<'\n';
  // A proof about the first anchor must not hide a route from another anchor
  // by publishing a reverse component whose last node was never expanded.
  require(cached,"early source proof cached an unfinished reverse region");
  require(saved_tiles==0,"interrupted reverse traversal was published as exhausted");
  f.auditAll();
}
