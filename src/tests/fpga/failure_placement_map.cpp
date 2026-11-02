#include "SourceInputFixture.h"
#include "route/FailurePlacementMap.h"

using routing_tests::SourceInputFixture;
using routing_tests::require;

void failure_placement_map_regression() {
  SourceInputFixture f(false, 0, 16);
  auto& incompatible = f.tile({1,0});
  auto second_slot = incompatible.tile_type->elements.front();
  second_slot.name = "register_lane_2";
  second_slot.bitmap_pos = 1;
  incompatible.tile_type->elements.push_back(second_slot);
  incompatible.tile_type->pin_map.nodes[{"FDRE","CE",4}].setBit(63);
  incompatible.invalidatePlacementCaches();
  auto* other = f.cell("occupied_register_31", "FDRE", {{"CE",false}}, {1,0});
  fpga::Element logic;
  logic.name = "independent_control_slot";
  logic.type = fpga::ELEMENT_LUT5;
  f.tile({4,2}).tile_type->elements.push_back(logic);
  auto* other_driver = f.cell("independent_control_37", "LUT6", {{"O",true}}, {4,2});
  f.connect(*other_driver, "O", *other, "CE");
  // Populate the resource-pin direction table too, as the database loader does.
  for (int pos : {0,4})
    incompatible.tile_type->pin_map.input_nodes[
        incompatible.getNodeNum("FDRE", "CE", pos)].setBit(63);
  auto& available = f.tile({2,0});
  available.tile_type->pin_map.input_nodes[available.getNodeNum("FDRE", "CE", 0)].setBit(63);
  incompatible.invalidatePlacementCaches();
  const auto original_tile = f.source->tile.peer;
  const auto original_coord = f.source->coord;
  const int original_pos = f.source->pos;
  std::ostringstream before, after, report;
  pnr::dumpFailureTileContents(f.device, before);
  auto candidates = pnr::dumpFailurePlacementMap(f.device, *f.source, {{f.source}}, report);
  // The report must include legal tiles never visited by any moving search, with live checks enabled.
  auto good = std::find_if(candidates.begin(), candidates.end(), [&](const auto& entry) {
    return entry.tile == &f.tile({2,0});
  });
  require(good != candidates.end() && good->packing_positions == std::vector<int>{0},
      "whole-grid diagnostic missed unvisited legal register slot");
  require(!good->live_positions.empty(), "free physical candidate was omitted from live positions");
  // A free slot is not a legal placement when its shared control has another driver.
  require(std::none_of(candidates.begin(), candidates.end(), [&](const auto& entry) {
    return entry.tile == &incompatible;
  }), "diagnostic accepted incompatible shared control despite free register slot");
  // The original occupied location must not be advertised as a new relocation.
  require(std::none_of(candidates.begin(), candidates.end(), [&](const auto& entry) {
    return entry.tile == original_tile;
  }), "diagnostic advertised the original tile as an alternative");
  pnr::dumpFailureTileContents(f.device, after);
  // Preview must restore cells, membership, masks, and original coordinates, not unroute anything.
  require(before.str() == after.str() && f.source->tile.peer == original_tile &&
      f.source->coord == original_coord && f.source->pos == original_pos,
      "failure inventory altered the live design");
  require(before.str().find("TILE coord=(7,2)") != std::string::npos &&
      before.str().find("occupied_register_31") != std::string::npos &&
      before.str().find("INPUT port=\"CE\"") != std::string::npos,
      "inventory omitted empty tiles, occupants, or control connections");
  f.auditAll();
  std::cout << report.str();
}
