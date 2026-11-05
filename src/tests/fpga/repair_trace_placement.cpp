#include "SourceInputFixture.h"

// The large-design trace scanned every instance for every newly printed tile.
// Reproduce the registry size and repeated tile snapshots without depending on
// runtime timing. Live relocation must remain visible; caching occupants by
// tile would be incorrect even when the route masks/revision stay unchanged.
void repair_trace_placement_regression() {
  using routing_tests::require;
  auto& device=fpga::Device::current();
  device.tile_grid.clear();
  device.size_width=device.size_height=8;
  device.grid_spec.size={8,8};
  device.tile_grid.resize(64);
  for(int y=0;y<8;++y)for(int x=0;x<8;++x)
    device.getTile(x,y)->coord={x,y};
  Referable<rtl::Cell> type;
  type.name="neutral_cell";type.type="neutral_logic";
  std::vector<Referable<rtl::Inst>> insts(51749);
  std::vector<rtl::Inst*> registry;
  for(size_t i=0;i<insts.size();++i) {
    auto& inst=insts[i];inst.cell_ref.set(&type);inst.pos=i%16;inst.depth=1;
    registry.push_back(&inst);
    // Later chunks are still unplaced, but remain in the design registry.
    if(i<1024)inst.tile.set(&device.tile_grid[i/16]);
  }
  auto count=[](const std::string& text,const std::string& token) {
    size_t n=0,pos=0;
    while((pos=text.find(token,pos))!=std::string::npos){++n;pos+=token.size();}
    return n;
  };
  std::ostringstream out;
  size_t checks=0;
  Referable<fpga::Tile> standalone;standalone.coord={0,0};
  fpga::Tile bare;bare.coord={1,0};
  {
    pnr::RouteCongestionTrace trace("neutral_signal","test",{},&out,registry,true);
    for(int pass=0;pass<3;++pass) {
      trace.changed();
      for(auto& tile:device.tile_grid)trace.snapshot(tile);
    }
    checks=trace.placementChecks();
    // Every occupant is printed once per tile snapshot, including fixed cells.
    require(count(out.str(),"OCCUPANT name=")==3*1024,
        "placement trace omitted or duplicated registry occupants");
    out.str("");out.clear();
    // Move without changing any route mask/revision, then explicitly request
    // fresh snapshots. Only live Ref membership may determine the occupants.
    insts[0].tile.set(&device.tile_grid[1]);
    trace.changed();trace.snapshot(device.tile_grid[0]);
    require(count(out.str(),"OCCUPANT name=")==15,"old tile retained moved occupant");
    out.str("");out.clear();trace.snapshot(device.tile_grid[1]);
    require(count(out.str(),"OCCUPANT name=")==17,"new tile omitted moved occupant");
    // New placement of an already registered future-chunk cell is visible too.
    insts[1024].tile.set(&device.tile_grid[0]);
    trace.changed();out.str("");out.clear();trace.snapshot(device.tile_grid[0]);
    require(count(out.str(),"OCCUPANT name=")==16,"newly placed instance was missed");
    // A non-instance reference is legal and must not be cast into an Inst.
    Ref<fpga::Tile> unrelated;unrelated.set(&device.tile_grid[0]);
    trace.changed();out.str("");out.clear();trace.snapshot(device.tile_grid[0]);
    require(count(out.str(),"OCCUPANT name=")==16,"non-instance reference changed occupants");
    // Non-grid diagnostic tiles must keep the registry fallback, even when
    // their coordinates happen to equal a live grid tile's coordinates.
    insts[1025].tile.set(&standalone);
    trace.changed();out.str("");out.clear();trace.snapshot(standalone);
    require(count(out.str(),"OCCUPANT name=")==1,"standalone tile registry fallback failed");
    trace.changed();out.str("");out.clear();trace.snapshot(bare);
    require(count(out.str(),"OCCUPANT name=")==0,"bare tile was treated as a grid referable");
  }
  std::cout<<"placement trace registry="<<registry.size()<<" snapshots=192 checks="<<checks<<'\n';
  // One registry indexing pass plus live occupants, not 192 whole-design scans.
  require(checks<=registry.size()+3*1024,
      "placement diagnostics repeatedly scan the full design registry");
}
