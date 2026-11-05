#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/Docking.h"
#include "route/RouteDesign.h"
#include "route/MovingInputCutCache.h"
#include "route/RouteTaskTiming.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

// A large free fan-in ends at an owned gate, beyond which a retained tree
// supplies three anchors. Candidate input pins share this same free region.
// Before region reuse each of nine queries expanded 722496 states. Keep the
// real graph and owner checks, but require at most one such expansion now.
struct AnchorArena {
  static constexpr int side=24, lanes=320, width=26;
  fpga::Device& device=fpga::Device::current();
  std::array<Referable<rtl::Net>,2> nets;
  std::array<Referable<rtl::Cell>,4> cells;
  std::array<rtl::Inst,4> insts;

  fpga::Tile& tile(int x,int y) { return *device.getTile(x,y); }
  static void name(fpga::CBType& cb,fpga::CBNodeNameType kind,int node) {
    cb.rememberNodeName(kind,node,"FILL_"+std::to_string(int(kind))+"_"+std::to_string(node));
  }
  void jump(int type,int src,fpga::Coord delta,int target_type,int dst) {
    auto& cb=device.cb_types[type];
    name(cb,fpga::CB_NODE_SRC,src);
    name(device.cb_types[target_type],fpga::CB_NODE_DST,dst);
    fpga::CBType::ResolvedJump link{};
    link.delta=delta;link.target_cb_type_id=target_type;link.target_tile_coord=true;
    link.dsts.jump.setBit(dst);cb.dst_by_src[src].push_back(link);
  }
  void entry(int type,int dst,int src) {
    auto& cb=device.cb_types[type];
    name(cb,fpga::CB_NODE_DST,dst);name(cb,fpga::CB_NODE_SRC,src);
    cb.dst_src[dst].jump.setBit(src);
  }
  void terminal(int type,int dst,int local) {
    auto& cb=device.cb_types[type];
    name(cb,fpga::CB_NODE_DST,dst);name(cb,fpga::CB_NODE_LOCAL,local);
    cb.dst_local[dst].local.setBit(local);
  }
  void takeoff(int type,int local,int src) {
    auto& cb=device.cb_types[type];
    name(cb,fpga::CB_NODE_LOCAL,local);name(cb,fpga::CB_NODE_SRC,src);
    cb.local_src[local].jump.setBit(src);
  }
  static fpga::Wire pin(fpga::Coord at,int local,bool output) {
    fpga::Wire wire;wire.type=fpga::Wire::WIRE_TILE_PIN;
    wire.from=wire.to=at;wire.local=local;
    wire.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;return wire;
  }
  static fpga::Wire hop(fpga::Coord from,fpga::Coord to,int input,int src,int dst,bool first=false) {
    fpga::Wire wire;wire.from=from;wire.to=to;wire.local=input;
    wire.jump=src;wire.dst=dst;wire.pos=first?0:1;wire.owns_landing=true;return wire;
  }
  static fpga::Wire finish(fpga::Coord at,int dst) {
    fpga::Wire wire;wire.from=wire.to=at;wire.local=dst;
    wire.jump=wire.dst=-1;wire.pos=1;return wire;
  }
  void install(int n,std::vector<fpga::Wire> route) {
    // Commit actual routes and register their owners, rather than manufacturing busy bits.
    require(pnr::commitPreparedRoute(route),"could not commit fixture owner");
    auto& sink=insts[2*n+1];sink.wires.push_back(std::move(route));
    auto binding=fpga::attachNetRoute(nets[n],sink,0,&insts[2*n],&sink,
                                    "OUT","IN","FILL_ROUTE_"+std::to_string(n));
    fpga::registerNetRouteTiles(nets[n],sink.wires[0],binding);
  }
  AnchorArena() {
    std::vector<Referable<fpga::Tile>> fresh(width*side);
    device.tile_grid.swap(fresh);fresh.clear();
    device.cb_types.clear();device.tile_types.clear();
    device.route_wire_graph.clear();device.local_route_wire_mappings.clear();
    device.size_width=width;device.size_height=side;device.grid_spec.size={width,side};
    device.cb_types.resize(6);
    for(int i=0;i<6;++i) {
      auto& cb=device.cb_types[i];cb.name="FILL_MATRIX_"+std::to_string(i);
      cb.type_id=cb.base_type_id=i;
    }
    for(int y=0;y<side;++y)for(int x=0;x<width;++x) {
      auto& t=tile(x,y);t.coord=t.cb_coord={x,y};t.name=t.coord;
      if(x<side)t.cb_type=t.cb.type=&device.cb_types[0];
    }
    for(auto [coord,type]:std::array<std::pair<fpga::Coord,int>,5>{{
        {{25,1},1},{{24,0},2},{{24,1},3},{{24,2},4},{{25,0},5}}})
      tile(coord.x,coord.y).cb_type=tile(coord.x,coord.y).cb.type=&device.cb_types[type];
    auto& cb=device.cb_types[0];
    constexpr std::array<fpga::Coord,4> directions{{{1,0},{0,1},{-1,0},{0,-1}}};
    for(int n=0;n<lanes;++n) {
      for(auto kind:{fpga::CB_NODE_LOCAL,fpga::CB_NODE_JOINT,fpga::CB_NODE_SRC,fpga::CB_NODE_DST})
        name(cb,kind,n);
      cb.dst_joint[n].joint.setBit(n);cb.joint_local[n].local.setBit(n);
      cb.local_src[n].jump.setBit(n);cb.dst_src[n].jump.setBit(n);
      cb.joint_joint[n].joint.setBit((n+1)%lanes);
      jump(0,n,directions[n%4],0,n);
    }
    for(int pin=1000;pin<1003;++pin) {
      name(cb,fpga::CB_NODE_LOCAL,pin);cb.joint_local[0].local.setBit(pin);
    }
    // Only one physically incoming gate connects this region to the retained tree.
    name(cb,fpga::CB_NODE_DST,960);cb.dst_joint[960].joint.setBit(0);
    takeoff(1,900,900);jump(1,900,{-1,-1},2,900);
    entry(2,900,901);jump(2,901,{0,1},3,900);
    entry(3,900,901);jump(3,901,{0,1},4,900);terminal(4,900,901);
    entry(2,900,911);jump(2,911,{-1,0},0,960);
    takeoff(5,910,910);jump(5,910,{-2,0},0,960);terminal(0,960,962);
    for(auto& type:device.cb_types)type.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    for(int i=0;i<4;++i) {
      cells[i].name="FILL_CELL_"+std::to_string(i);cells[i].type="FILL_RESOURCE";
      insts[i].cell_ref.set(&cells[i]);
    }
    nets[0].name="FILL_RETAINED_TREE";nets[1].name="FILL_GATE_OWNER";
    install(0,{pin({25,1},900,true),hop({25,1},{24,0},900,900,900,true),
        hop({24,0},{24,1},900,901,900),hop({24,1},{24,2},900,901,900),
        finish({24,2},900),pin({24,2},901,false)});
    install(1,{pin({25,0},910,true),hop({25,0},{23,0},910,910,960,true),
        finish({23,0},960),pin({23,0},962,false)});
  }
  void audit() {
    for(auto& t:device.tile_grid) {
      std::ostringstream details;
      auto a=fpga::auditTileCongestion(t,details,{&nets[0],&nets[1]});
      if(a.orphan_leases || a.missing_leases || a.missing_registrations ||
          a.stale_registrations || a.conflicting_driver_nodes)
        throw std::runtime_error(details.str());
    }
  }
  ~AnchorArena() { device.tile_grid.clear(); }
};
}

void input_anchor_expansions_regression() {
  setenv("SCALEPNR_PROFILE_TASK_NET","FILL_PROFILE",1);
  AnchorArena f;f.audit();
  auto& target=f.tile(12,12);
  auto index=pnr::buildBackwardResolveIndex(f.device,target.coord,32);
  pnr::MovingInputCutCache cache;
  std::array<pnr::BackwardRouteAnchor,3> goals{{
      {&f.tile(24,0),900,{},1},{&f.tile(24,1),900,{},2},{&f.tile(24,2),900,{},3}}};
  std::vector<fpga::CBState> before;
  for(auto& tile:f.device.tile_grid)before.push_back(tile.cb);
  size_t searches=0,expansions=0,cache_hits=0;
  // The existing opt-in timer reports the real boolean engine's expansion
  // count. Do not substitute a callback that invents boundaries or counts.
  auto prove=[&](int pin,int anchor,bool open_gate) {
    std::ostringstream records;
    struct Capture {
      std::streambuf* old;
      void restore(){if(old){std::cout.rdbuf(old);old=nullptr;}}
      ~Capture(){restore();}
    } capture{std::cout.rdbuf(records.rdbuf())};
    pnr::MovingInputCutCache::Key key{&target,NodeMask{0,1}<<pin,
        {25,1},0,32,200,goals[anchor].tile,900};
    std::vector<fpga::RouteCutNode> releases;
    if(open_gate)releases.push_back({{23,0},fpga::CB_NODE_DST,960});
    auto result=cache.prove(key,releases,[&](const pnr::InputRouteBlocked& blocked,
                                          pnr::InputRouteRegionCache* region) {
      ++searches;
      pnr::BackwardTakeoffStateView view;
      if(open_gate) {
        view.emplace(&f.tile(23,0),f.tile(23,0).cb);
        view.at(&f.tile(23,0)).dst.jump.clearBit(960);
      }
      const std::vector<pnr::BackwardRouteAnchor> anchors{goals[anchor]};
      pnr::RouteTaskTiming measured("FILL_PROOF","FILL_PROFILE");
      auto observe=[&](fpga::Tile& tile,fpga::CBNodeNameType kind,int node) {
        // Every negative proof must rediscover the same genuinely owned boundary.
        if(!open_gate)require(&tile==&f.tile(23,0) && kind==fpga::CB_NODE_DST && node==960,
                             "failed search found an unexpected occupied boundary");
        blocked(tile,kind,node);
      };
      return pnr::inputMayReachSource(target,key.pins,key.source_hint,32,{},&index,{},
                                     &view,&anchors,200,observe,region);
    },[]{return false;});
    size_t expanded=0,measured_calls=0;
    std::istringstream lines(records.str());std::string line;
    while(std::getline(lines,line)) {
      if(line.find("TASK_TIME_END")==std::string::npos ||
          line.find("phase=\"input_reachability\"")==std::string::npos)continue;
      ++measured_calls;
      expanded+=std::stoull(line.substr(line.find(" expanded=")+10));
    }
    // A cache hit must perform no traversal; a miss must measure exactly one real traversal.
    require(measured_calls==(result.reused?0u:1u),"missing or duplicated production proof timing");
    expansions+=expanded;cache_hits+=result.reused;
    capture.restore();
    std::cout<<"anchor_expansions pin="<<pin<<" anchor="<<anchor<<" opened="<<open_gate
             <<" success="<<result.possible<<" reused="<<result.reused
             <<" expanded="<<expanded<<'\n';
    return std::pair{result,expanded};
  };
  const auto start=std::chrono::steady_clock::now();
  size_t first_expanded=0;
  for(int pin=1000;pin<1003;++pin)for(int anchor=0;anchor<3;++anchor) {
    auto [result,expanded]=prove(pin,anchor,false);
    require(!result.possible && !result.reused,"distinct query unexpectedly used an exact-key result");
    // The first proof must still walk the full graph, so reducing the fixture cannot pass.
    if(!first_expanded) {
      first_expanded=expanded;
      require(expanded>=700000 && expanded<=735000,"cold-proof expansion count left its expected range");
    } else require(expanded<=1,"another pin/anchor repeated the exhausted-region traversal");
  }
  const size_t baseline=expansions;
  // Nine goal/pin variants cost one complete proof plus at most eight root nodes.
  require(searches==9 && baseline<=735008 && baseline<=first_expanded+8 && cache.regionHits()>=8,
          "exhausted-region work exceeded the one-search counter budget");
  // Storage sharing is already working; it does not prevent repeated computation.
  require(cache.boundaryNodeCount()==1,"equal blocked boundaries were not shared");
  // Exact repeats must hit: this isolates distinct-goal work from a broken exact-key cache.
  for(int pin=1000;pin<1003;++pin)for(int anchor=0;anchor<3;++anchor) {
    auto [result,expanded]=prove(pin,anchor,false);
    require(!result.possible && result.reused && !expanded,"exact repeat repeated the graph traversal");
  }
  require(searches==9 && cache_hits==9 && expansions==baseline,"cache-hit work accounting changed");
  // A private release of the real blocker must succeed, without changing any live leases.
  auto [opened,expanded]=prove(1000,0,true);
  require(opened.possible && !opened.reused,"opening the owned gate did not reach the retained tree");
  for(size_t i=0;i<before.size();++i) {
    const auto& cb=f.device.tile_grid[i].cb;const auto& old=before[i];
    require(cb.src.jump==old.src.jump && cb.dst.jump==old.dst.jump &&
        cb.joint.jump==old.joint.jump && cb.local.local==old.local.local &&
        cb.src_deadend.jump==old.src_deadend.jump,"read-only proof modified live state");
  }
  f.audit();
  std::cout<<"anchor_expansions queries=9 failed_expanded="<<baseline
      <<" first_expanded="<<first_expanded<<" region_hits="<<cache.regionHits()
      <<" exact_hits="<<cache_hits<<" opened_success="<<opened.possible<<" seconds="
      <<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
}

// Negative reuse must not turn an incomplete proof or a newly reachable goal
// into a placement rejection. These controls exercise the same numeric mesh.
void input_anchor_region_safety_regression() {
  AnchorArena f;
  auto& target=f.tile(12,12);
  auto index=pnr::buildBackwardResolveIndex(f.device,target.coord,32);
  const pnr::BackwardRouteAnchor retained{&f.tile(24,0),900,{},1};
  size_t searches=0;
  auto query=[&](pnr::MovingInputCutCache& cache,pnr::BackwardRouteAnchor anchor,
                 int radius,int depth,size_t cancel_after=0,bool takeoff=false) {
    size_t checks=0;
    auto cancel=[&]{return cancel_after && ++checks>=cancel_after;};
    const pnr::MovingInputCutCache::Key key{&target,NodeMask{0,1}<<1000,
        {25,1},0,radius,depth,anchor.tile,anchor.dst};
    return cache.prove(key,{},[&](const pnr::InputRouteBlocked& blocked,
                                 pnr::InputRouteRegionCache* region) {
      ++searches;
      std::vector<pnr::BackwardRouteAnchor> anchors{anchor};
      pnr::BackwardTakeoffProbe probe;
      if(takeoff)probe=[&](fpga::Tile& tile,int src,pnr::BackwardTakeoffChoice& choice) {
        if(&tile!=&target || src!=0)return false;
        choice={0,-1,-1};return true;
      };
      return pnr::inputMayReachSource(target,key.pins,key.source_hint,radius,probe,
          &index,cancel,nullptr,&anchors,depth,blocked,region);
    },cancel);
  };
  pnr::MovingInputCutCache cache;
  require(!query(cache,retained,32,200).possible && cache.hasRegion(),"closed region was not captured");
  // A new goal in the previously explored region must be tested, not skipped.
  require(query(cache,{&target,0,{},2},32,200).possible,"region reuse hid a reachable free goal");
  // A retained goal is allowed to terminate on an occupied boundary node.
  require(query(cache,{&f.tile(23,0),960,{},3},32,200).possible,"region reuse hid a retained boundary goal");
  // A caller with a takeoff predicate cannot reuse a fixed-anchor failure.
  require(query(cache,{&f.tile(24,1),900,{},4},32,200,0,true).possible,
          "fixed-anchor region suppressed a successful takeoff predicate");
  require(cache.regionHits()==0,"goal safety controls incorrectly pruned a region");

  pnr::MovingInputCutCache limited;
  require(!query(limited,retained,32,1).possible && !limited.hasRegion(),
          "depth-limited traversal was saved as an exhausted region");
  require(!query(limited,retained,32,200).possible && limited.hasRegion(),
          "full retry did not traverse the region after a depth-limited proof");
  pnr::MovingInputCutCache interrupted;
  require(!query(interrupted,retained,32,200,50).possible && !interrupted.hasRegion(),
          "cancelled traversal was saved as an exhausted region");
  auto retry=query(interrupted,retained,32,200);
  require(!retry.possible && !retry.reused && interrupted.hasRegion(),
          "cancelled proof suppressed its retry");

  pnr::MovingInputCutCache window;
  require(!query(window,retained,1,200).possible && window.hasRegion(),"small window was not exhausted");
  require(!query(window,retained,32,200).possible && window.regionHits()==0,
          "small-window failure suppressed a larger-window search");
  f.audit();
  // A real unroute changes the live graph; both exact and region witnesses expire.
  fpga::unrouteNetRoute(f.nets[1],0);
  auto opened=query(cache,retained,32,200);
  require(opened.possible && !opened.reused,"routing mutation left a stale failed region");
  f.audit();
  std::cout<<"anchor_region_safety searches="<<searches<<" guards=goals,takeoff,depth,cancel,window,revision passed\n";
}
