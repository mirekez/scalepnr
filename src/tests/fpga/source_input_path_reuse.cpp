#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"

// Reproduce the native move's successful first input followed by a negative
// second input, with preemption enabled. Every distinct Q exit used to redo
// the first path before discovering the cached failure of the second input.
void source_input_path_reuse_regression(bool conflict, bool banks) {
  using namespace routing_tests;
  SourceInputFixture f(false,0,16);
  auto slot=[&](fpga::Coord at,fpga::ElementType kind) {
    fpga::Element e;e.name="extra_slot";e.type=kind;
    f.tile(at).tile_type->elements.push_back(e);
  };
  slot({7,2},fpga::ELEMENT_FD);
  for(fpga::Coord at:{fpga::Coord{6,2},fpga::Coord{5,2}}) {
    slot(at,fpga::ELEMENT_LUT5);
    fpga::Element e;e.name="extra_extension";e.type=fpga::ELEMENT_LUT1;e.bitmap_pos=0;
    f.tile(at).tile_type->elements.push_back(e);
  }
  auto* source=f.cell("moving_register","FDRE",{{"CE",false},{"D",false},{"Q",true}},{7,2});
  auto* control=f.cell("first_driver","LUT6",{{"O",true}},{6,2});
  auto* data=f.cell("second_driver","LUT6",{{"O",true}},{5,2});
  auto* output=f.connect(*source,"Q",*f.sink,"I0");
  auto* first=f.connect(*control,"O",*source,"CE");
  auto* second=f.connect(*data,"O",*source,"D");
  for(auto& tile:f.device.tile_grid) {
    tile.tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(88);
    tile.cb_type->local_input_nodes.setBit(88);
    SourceInputFixture::name(tile,fpga::CB_NODE_LOCAL,88);
  }
  f.edge({6,2},{2,0},169,1500,1500,true);
  if(conflict) {
    // Q exit 100 leaves this CE path free. Q exit 101 leases its SRC and
    // landing; reusing the saved CE path blindly would double-allocate them.
    f.tile({2,0}).cb_type->dst_src[1500].jump.setBit(101);
    f.edge({0,0},{2,0},501,1900,1901);
    f.terminal({2,0},1901,63);
  } else f.terminal({2,0},1500,63);
  if(banks) {
    // Each output is considered at both physical input banks. A one-entry
    // witness cache evicts the first bank on every visit to the second bank,
    // repeating both successful searches before the same failed data input.
    auto& t=f.tile({2,0});
    fpga::Element e;e.name="second_bank";e.type=fpga::ELEMENT_FD;e.bitmap_pos=8;
    t.tile_type->elements.push_back(e);t.elements_initialized=false;
    t.tile_type->pin_map.nodes[{"FDRE","CE",128}].setBit(64);
    t.tile_type->pin_map.nodes[{"FDRE","D",128}].setBit(88);
    t.tile_type->pin_map.output_nodes[t.getNodeNum("FDRE","Q",128)].setBit(142);
    t.cb_type->local_input_nodes.setBit(64);t.cb_type->local_output_nodes.setBit(142);
    SourceInputFixture::name(t,fpga::CB_NODE_LOCAL,64);
    SourceInputFixture::name(t,fpga::CB_NODE_LOCAL,142);
    for(int n=0;n<32;++n)t.cb_type->local_src[142].jump.setBit(100+n);
    f.edge({6,2},{2,0},169,1504,1504,true);f.terminal({2,0},1504,64);
  }
  // The early bank's D has free incoming fabric but no connected driver.
  f.edge({3,0},{2,0},1503,1503,1503);f.terminal({2,0},1503,88);
  // A later bank can host the cell and route all three incident signals.
  f.edge({6,2},{1,0},169,1501,1501,true);f.terminal({1,0},1501,63);
  f.edge({5,2},{1,0},169,1502,1502,true);f.terminal({1,0},1502,88);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();f.auditAll();
  auto& router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=true;router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  pnr::RouteDesign::RouteTask task{source,f.sink,output,"Q","I0","output_signal"};
  task.endpoints_prepared=true;
  pnr::MovingPlacementBlacklist work({{source}});
  std::string reason;
  const bool done=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,nullptr,
      nullptr,nullptr,false,&work);
  // Efficiency must never replace successful relocation or complete bindings.
  require(done && source->tile.peer==&f.tile({1,0}),"later source bank was lost: "+reason);
  for(auto* net:{output,first,second}) {
    bool complete=false;
    for(const auto& b:net->routes)if(b.owner && b.route_index<b.owner->wires.size())
      complete|=fpga::isRouteComplete(b.owner->wires[b.route_index]);
    require(complete,"relocation published an unfinished incident route");
  }
  // Audit actual bindings and leases, including the unrelated existing tree.
  f.auditAll();
  std::cout << "input path reuse conflict=" << conflict << " banks=" << banks << " searches=" << work.input_private_searches
    << " expanded=" << work.input_private_expanded << " reused=" << work.input_path_reuses
    << " conflicts=" << work.input_path_conflicts << '\n';
  // One CE proof at the bad bank, then CE/D at the good bank. The conflict
  // variant permits one extra search, but must detect the changed reservation.
  require(work.input_private_searches<=size_t(conflict || banks?4:3),
    "successful earlier input repeatedly searched before cached later failure");
  require(work.input_private_expanded<=size_t(conflict?256:banks?10:8),
    "reusing a positive input path still repeats the expensive search work");
  require(work.input_path_reuses>=size_t(banks?60:30),"output choices did not reuse the validated input path");
  require(!conflict || work.input_path_conflicts>0,"changed Q reservation bypassed path validation");
}
