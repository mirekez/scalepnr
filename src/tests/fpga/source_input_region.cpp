#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"
#include "route/RouteTaskTiming.h"

// Source relocation repeatedly proved its unchanged input driver could not
// reach each proposed placement. The native proof expanded about one million
// reverse nodes per candidate, although the driver had a small closed region.
// Exercise that production source-moving guard with real owners and neutral IDs.
void source_input_region_regression() {
  using namespace routing_tests;
  setenv("SCALEPNR_PROFILE_TASK_NET","FILL_SOURCE_REGION",1);
  SourceInputFixture f(false,0,4,90,17);
  auto init=[&](fpga::Coord at) {
    auto& tile=f.tile(at); if(tile.cb_type)return;
    auto& cb=f.device.cb_types.emplace_back();cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
    cb.name="FILL_MATRIX_"+std::to_string(cb.type_id);
    auto& tt=f.device.tile_types.emplace_back();tt.name="FILL_RESOURCE_"+std::to_string(cb.type_id);
    tt.type=tile.type=fpga::Tile::TILE_LUTS;tile.tile_type=&tt;tile.cb_type=tile.cb.type=&cb;
  };
  constexpr int forward_count=19544,reverse_count=(1<<20)-1;
  auto forward=[](int n){const int t=n/CB_MAX_NODES;return fpga::Coord{8+t%8,t/8};};
  auto reverse=[](int n){const int t=n/CB_MAX_NODES;return fpga::Coord{16+t%64,t/64};};
  auto id=[](int n){return n%CB_MAX_NODES;};
  for(int n=0;n<forward_count;n+=CB_MAX_NODES)init(forward(n));
  for(int n=0;n<reverse_count;n+=CB_MAX_NODES)init(reverse(n));
  // The receiver's retained input trunk supplies this genuinely owned DST.
  f.tile({5,1}).cb_type->dst_local[711].local.setBit(2000);
  f.resolvedLocal({5,1},2000,forward(0),fpga::CB_NODE_LOCAL,0);
  for(int n=1;n<forward_count;++n)
    f.resolvedLocal(forward((n-1)/2),id((n-1)/2),forward(n),fpga::CB_NODE_LOCAL,id(n));
  for(int n=1;n<reverse_count;++n)
    f.resolvedLocal(reverse(n),id(n),reverse((n-1)/2),fpga::CB_NODE_LOCAL,id((n-1)/2));
  std::vector<fpga::Coord> candidates;
  for(int x=82;x<90;++x) {
    fpga::Coord at{x,3};init(at);candidates.push_back(at);
    auto& t=f.tile(at);t.tile_type->pin_map.nodes[{"FDRE","CE",0}].setBit(63);
    t.cb_type->local_input_nodes.setBit(63);
    f.resolvedLocal(reverse(0),0,at,fpga::CB_NODE_LOCAL,62);
    t.cb_type->local_local[62].local.setBit(63);
  }
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto& router=f.tech.route;router.moving_stage=true;router.moving_sources_stage=true;
  router.route_preemption_enabled=false;router.route_deadends_enabled=false;
  pnr::RouteDesign::RouteTask input{f.control,f.source,f.enable,"O","CE","FILL_SOURCE_REGION_INPUT"};
  pnr::MovingPlacementBlacklist cache({{f.source}});
  const auto old_tile=f.source->tile.peer;const auto old_coord=f.source->coord;
  size_t reverse_expanded=0;
  auto prove=[&](fpga::Coord at) {
    f.source->tile.set(static_cast<Referable<fpga::Tile>*>(&f.tile(at)));f.source->coord=at;
    std::ostringstream log;
    struct Capture {std::streambuf* saved;~Capture(){std::cout.rdbuf(saved);}} capture{std::cout.rdbuf(log.rdbuf())};
    bool possible;
    {pnr::RouteTaskTiming timing("fixture","FILL_SOURCE_REGION");
      possible=router.movingInputMayRoute(input,{f.source},{},cache);}
    std::istringstream lines(log.str());
    for(std::string line;std::getline(lines,line);)if(line.starts_with("TASK_TIME_END") &&
        line.find("phase=\"input_reachability\"")!=std::string::npos) {
      const auto at=line.find(" expanded=");require(at!=std::string::npos,"missing reverse expansion count");
      reverse_expanded+=std::stoull(line.substr(at+10));
    }
    return possible;
  };
  // Different pins must not restart the same exhausted source component.
  for(auto at:candidates)require(!prove(at),"disconnected candidate accepted");
  const size_t closed_work=cache.input_expanded+reverse_expanded;
  const size_t closed_reverse=reverse_expanded;
  std::cout<<"FILL_SOURCE_REGION closed_work="<<closed_work<<" reverse_expanded="<<reverse_expanded
      <<" queries="<<candidates.size()<<" region_hits="<<cache.input_region_hits<<'\n';
  // All later distinct target pins must reuse the same complete source proof.
  require(cache.input_region_hits==candidates.size()-1,"closed input region was not reused");
  // A different input may retain a different private prefix of the same tree.
  NodeMask pin;pin.setBit(63);
  require(!cache.sourceRegionMayReach(f.control,"O",f.source,"D",f.tile(candidates.back()),pin).has_value(),
      "source proof crossed input-port identity");
  require(!cache.sourceRegionMayReach(f.control,"O",f.sink,"CE",f.tile(candidates.back()),pin).has_value(),
      "source proof crossed receiver identity");
  // Membership in a closed component is only permission for exact validation.
  NodeMask reached;reached.setBit(id(forward_count-1));
  require(cache.sourceRegionMayReach(f.control,"O",f.source,"CE",f.tile(forward(forward_count-1)),reached)==true,
      "closed region discarded its reachable locals");
  // The closed-region optimization cannot survive a graph/lease mutation.
  f.resolvedLocal(forward(forward_count-1),id(forward_count-1),candidates.back(),fpga::CB_NODE_LOCAL,63);
  f.device.rebuildIncomingDstMasks();
  router.docking_resolved_arcs={};
  router.docking_resolved_arcs_complete=false;
  cache.clearRoutingFailures({&f.tile(forward(forward_count-1))});
  require(prove(candidates.back()),"opened candidate retained a stale negative proof");
  f.source->tile.set(old_tile);f.source->coord=old_coord;
  f.auditAll();
  // Keep each candidate's coarse map for subsequent transit-cut validation,
  // but run only one exact proof (10,485,776 total expansions before reuse).
  require(closed_work<3500000 && closed_reverse>=1000000 && closed_reverse<1100000,
      "source-move candidates repeated the same disconnected input component");
  cache.clearRoutingFailures();
  require(!cache.sourceRegionMayReach(f.control,"O",f.source,"CE",f.tile(candidates.back()),pin).has_value(),
      "full routing invalidation retained a source-region proof");
}
