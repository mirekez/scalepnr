#include "SourceInputFixture.h"
#include "route/MovingInputCutCache.h"

// A source-move input proof explored 840089 destination-side states even
// though its retained source had only 1020 reachable states. Reproduce that
// imbalance with neutral node names and real control-tree lease owners.
void input_anchor_frontier_regression() {
  using namespace routing_tests;
  setenv("SCALEPNR_PROFILE_TASK_NET","FILL_ANCHOR",1);
  SourceInputFixture f(false,0,4,288,3);
  constexpr int count=(1<<20)-1;
  auto location=[](int n) {return fpga::Coord{24+n/CB_MAX_NODES,0};};
  auto id=[](int n) {return n%CB_MAX_NODES;};
  for(int n=0;n<count;n+=CB_MAX_NODES) {
    auto& tile=f.tile(location(n));
    auto& cb=f.device.cb_types.emplace_back();
    cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
    cb.name="FILL_MATRIX_"+std::to_string(cb.type_id);
    tile.cb_type=tile.cb.type=&cb;
  }
  for(int n=1;n<count;++n)
    f.resolvedLocal(location(n),id(n),location((n-1)/2),fpga::CB_NODE_LOCAL,id((n-1)/2));
  auto& anchor=f.tile({5,1});
  anchor.cb_type->dst_joint[711].joint.setBit(200);
  anchor.cb_type->joint_local[200].local.setBit(0);
  SourceInputFixture::name(anchor,fpga::CB_NODE_JOINT,200);
  for(int n=0;n<1023;++n) {
    SourceInputFixture::name(anchor,fpga::CB_NODE_LOCAL,n);
    if(n)anchor.cb_type->local_local[(n-1)/2].local.setBit(n);
  }
  f.edge({4,1},location(0),712,1500,4000);
  f.terminal(location(0),4000,0);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto& target=f.tile(location(0));
  auto index=pnr::buildBackwardResolveIndex(f.device,target.coord,288);
  std::vector<pnr::BackwardRouteAnchor> anchors{{&anchor,711,{},1}};
  const NodeMask pins=NodeMask{0,1};
  // Materialization remains exhaustive: prove the synthetic reverse region
  // really is as large as the captured failure, without inventing busy bits.
  auto control=pnr::prepareInputRoute(target,pins,anchor.coord,288,{},&index,{},nullptr,&anchors,200);
  require(!control.success && control.expanded>800000,"large reverse component was not exercised");
  pnr::MovingInputCutCache cache;
  pnr::MovingInputCutCache::Key key{&target,pins,anchor.coord,0,288,200,&anchor,711};
  size_t calls=0,closed_expanded=0;
  auto prove=[&](const std::vector<fpga::RouteCutNode>& cuts) {
    return cache.prove(key,cuts,[&](const pnr::InputRouteBlocked& blocked) {
      ++calls;
      pnr::BackwardTakeoffStateView view;
      for(const auto& cut:cuts) {
        auto& tile=f.tile(cut.tile);
        auto& cb=view.try_emplace(&tile,tile.cb).first->second;
        auto& mask=cut.type==fpga::CB_NODE_SRC?cb.src.jump:cb.dst.jump;
        require(mask.testBit(cut.node),"proposed cut has no real lease");
        mask.clearBit(cut.node);
      }
      std::ostringstream log;
      struct Capture {
        std::streambuf* previous;
        void restore() {if(previous) {std::cout.rdbuf(previous);previous=nullptr;}}
        ~Capture() {restore();}
      } capture{std::cout.rdbuf(log.rdbuf())};
      bool possible;
      {
        pnr::RouteTaskTiming timing("fixture","FILL_ANCHOR");
        possible=pnr::inputMayReachSource(target,pins,anchor.coord,288,{},&index,{},&view,&anchors,200,blocked);
      }
      capture.restore();
      std::istringstream lines(log.str());
      std::string line;
      while(std::getline(lines,line))if(!possible && line.starts_with("TASK_TIME_END") &&
          line.find("phase=\"input_reachability\"")!=std::string::npos) {
        const auto at=line.find(" expanded=");
        require(at!=std::string::npos,"input proof did not report its work");
        closed_expanded+=std::stoull(line.substr(at+10));
      }
      return possible;
    },[]{return false;});
  };
  // Distinct unrelated releases must reuse the negative proof. The boundary
  // must describe the source component, not an empty or target-only witness.
  for(auto cut:std::vector<fpga::RouteCutNode>{
      {{2,1},fpga::CB_NODE_SRC,915},{{1,1},fpga::CB_NODE_SRC,916},
      {{0,1},fpga::CB_NODE_SRC,917}})
    require(!prove({cut}).possible,"unrelated cut connected isolated source");
  auto opened=prove({{{5,1},fpga::CB_NODE_SRC,912},{{4,1},fpga::CB_NODE_DST,712}});
  // Releasing the genuine boundary must invalidate the negative result and
  // reach the target without changing live routes or their owner registrations.
  require(opened.possible && !opened.reused,"source-boundary release reused a false negative");
  f.auditAll();
  std::cout<<"FILL_ANCHOR_FRONTIER control_expanded="<<control.expanded
      <<" closed_expanded="<<closed_expanded<<" calls="<<calls<<'\n';
  require(calls==2 && closed_expanded>0 && closed_expanded<=2048,
      "input proof explored the million-node destination instead of the small source component");
}
