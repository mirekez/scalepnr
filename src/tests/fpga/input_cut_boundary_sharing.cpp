#include "SourceInputFixture.h"
#include "route/MovingInputCutCache.h"

// Replay the native proof observer's 148809 distinct blocked nodes for three
// different anchors. Two copies exceed the 262144-node aggregate budget:
// formerly each insertion discarded the other anchors' identical witnesses.
// These are neutral numeric observer records, not fictitious live leases.
void input_cut_boundary_sharing_regression() {
  using namespace routing_tests;
  SourceInputFixture f(false,0,4,40);
  constexpr size_t boundary_size=148809;
  pnr::MovingInputCutCache cache;
  pnr::MovingInputCutCache::Key key{&f.tile({2,0}),NodeMask{0,1}<<63,
      {5,1},0,40,200,&f.tile({5,1}),711};
  size_t calls=0,observed=0,hits=0;
  auto failed=[&](const pnr::InputRouteBlocked& record) {
    ++calls;
    for(size_t i=0;i<boundary_size;++i) {
      record(f.tile({int(i/CB_MAX_NODES),2}),fpga::CB_NODE_DST,int(i%CB_MAX_NODES));
      ++observed;
    }
    return false;
  };
  for(int round=0;round<4;++round)for(int anchor=0;anchor<3;++anchor) {
    key.dst=711+anchor;
    // Each release set is different, but none opens the recorded boundary.
    auto result=cache.prove(key,{{{0,0},fpga::CB_NODE_SRC,100+round*3+anchor}},
        failed,[]{return false;});
    require(!result.possible,"closed observer region unexpectedly became reachable");
    hits+=result.reused;
  }
  std::cout<<"boundary sharing calls="<<calls<<" observed_nodes="<<observed
      <<" hits="<<hits<<'\n';
  // One proof per distinct anchor, not one per unrelated cut or repetition.
  require(calls==3 && hits==9 && observed==3*boundary_size,
      "identical boundaries evicted each other and repeated exhausted searches");
  require(cache.boundaryNodeCount()==boundary_size,
      "shared witnesses were duplicated or exceeded their memory budget");
  // Opening an actual boundary node still requires a new proof for this key.
  auto opened=cache.prove(key,{{{0,2},fpga::CB_NODE_DST,0}},
      [&](const pnr::InputRouteBlocked&){++calls;return true;},[]{return false;});
  require(opened.possible && !opened.reused && calls==4,
      "boundary sharing suppressed a potentially successful cut");

  // Equal aggregate hashes are only a lookup hint, never boundary identity.
  pnr::MovingInputCutCache collisions;
  auto first=key,second=key;
  ++second.dst;
  auto record=[&](auto k,int a,int b) {
    collisions.prove(k,{},[&](const pnr::InputRouteBlocked& visit) {
      visit(f.tile({0,2}),fpga::CB_NODE_DST,a);
      visit(f.tile({0,2}),fpga::CB_NODE_DST,b);
      return false;
    },[]{return false;});
  };
  record(first,10,13);record(second,11,12);
  require(collisions.boundaryNodeCount()==4,"different boundaries aliased by hash");
  auto distinct=collisions.prove(second,{{{0,2},fpga::CB_NODE_DST,11}},
      [](const pnr::InputRouteBlocked&){return true;},[]{return false;});
  require(distinct.possible && !distinct.reused,"hash collision hid an opening cut");

  // A changed live routing revision invalidates every reference to a witness.
  fpga::unrouteNetRoute(*f.enable,0);
  auto revised=cache.prove(key,{{{0,0},fpga::CB_NODE_SRC,101}},
      [](const pnr::InputRouteBlocked&){return true;},[]{return false;});
  require(revised.possible && !revised.reused && cache.boundaryNodeCount()==0,
      "shared boundary survived routing mutation");
  f.auditAll();
}
