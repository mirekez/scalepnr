#include "SourceInputFixture.h"

namespace {

using routing_tests::require;
using routing_tests::SourceInputFixture;

void moving_foreign_terminal_reservation() {
  SourceInputFixture f(false);
  auto &target = f.tile({0, 0});
  f.edge({7, 0}, {0, 0}, 141, 2800, 2900, true);
  f.terminal({0, 0}, 2900, 90);
  target.cb_type->local_local[90].local.setBit(87);
  for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();

  auto *requested_driver = SourceInputFixture::port(*f.sink, "I0").follow();
  auto *foreign_driver = &SourceInputFixture::port(*f.control, "O");
  require(requested_driver && requested_driver != foreign_driver,
          "terminal reservation control has no distinct driver");
  target.input_local_reservations[87] = foreign_driver;
  target.input_joint_reservations_initialized = true;
  require(!fpga::inputLocalReservedByDriver(target, 87, requested_driver),
          "foreign terminal reservation was not installed");

  auto &router = f.tech.route;
  router.moving_stage = router.complete_routes_only = true;
  router.route_preemption_enabled = router.fanout_preemption_enabled = false;
  pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal, "Q", "I0",
                                    "terminal_foreign_reservation"};
  task.endpoints_prepared = true;
  require(!router.routeBeforeMoving(task),
          "Moving accepted a terminal LOCAL reserved for another driver");
  require(!router.routeTaskIsComplete(task) &&
              target.input_local_reservations.at(87) == foreign_driver,
          "failed terminal probe changed binding or reservation ownership");
  f.auditAll();
}

void moving_root_local_input_pin_lease() {
  SourceInputFixture f(false);
  auto &tile = f.tile({7, 0});
  fpga::Element slot;
  slot.name = "local_receiver_slot";
  slot.type = fpga::ELEMENT_FD;
  slot.bitmap_pos = 1;
  tile.tile_type->elements.push_back(slot);
  tile.tile_type->pin_map.nodes[{"FDRE", "D", 4}].setBit(141);
  tile.cb_type->local_input_nodes.setBit(141);
  f.edge({7, 0}, {0, 0}, 141, 3301, 3302, true);
  f.terminal({0, 0}, 3302, 87);
  tile.cb_type->dst_local[3300].local.setBit(141);
  f.edge({1, 0}, {7, 0}, 3350, 3303, 3300);
  for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  tile.invalidatePlacementCaches();
  auto *local_sink = f.cell("local_receiver", "FDRE", {{"D", false}}, {7, 0}, 4);
  auto &input = SourceInputFixture::port(*local_sink, "D");
  auto &output = SourceInputFixture::port(*f.source, "Q");
  input.port_ref->designator = output.port_ref->designator;
  input.set(&rtl::Conn::fromBase(output));

  auto &router = f.tech.route;
  router.moving_stage = router.complete_routes_only = true;
  router.route_preemption_enabled = router.fanout_preemption_enabled = false;
  pnr::RouteDesign::RouteTask task{f.source, local_sink, f.signal, "Q", "D",
                                    "root_local_input_pin"};
  task.endpoints_prepared = true;
  require(!tile.isPinNodeLeased(141), "fixture input pin is already leased");
  require(router.routeBeforeMoving(task), "same-resource LOCAL route did not complete");
  require(router.routeTaskIsComplete(task), "same-resource LOCAL binding is incomplete");
  require(tile.isPinNodeLeased(141),
          "complete same-resource LOCAL route did not lease its input pin");
  f.auditAll();
}

void moving_same_binding_fork_boundary_owner() {
  SourceInputFixture f(false);
  auto takeoff = f.edge({7, 0}, {1, 0}, 141, 2800, 220, true);
  auto old_next = f.edge({1, 0}, {0, 0}, 220, 2801, 221);
  auto new_fork = f.edge({1, 0}, {0, 0}, 220, 2802, 222);
  f.terminal({0, 0}, 222, 87);
  for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();

  takeoff.owns_landing = false;
  old_next.owns_landing = false;
  std::vector<fpga::Wire> original{
      SourceInputFixture::pin({7, 0}, 141, true), takeoff, old_next};
  require(pnr::commitPreparedRoute(original), "cannot lease original partial route");
  fpga::Wire gap;
  gap.type = fpga::Wire::WIRE_GAP;
  original.push_back(gap);
  const size_t old_storage = f.source->wires.size();
  f.source->wires.push_back(std::move(original));
  const size_t old_binding = fpga::attachNetRoute(
      *f.signal, *f.source, old_storage, f.source, f.sink, "Q", "I0",
      "same_binding_fork_boundary");
  fpga::registerNetRouteTiles(*f.signal, f.source->wires[old_storage], old_binding);
  auto &boundary = f.tile({1, 0});
  require(boundary.cb.dst.jump.testBit(220) &&
              f.source->wires[old_storage][2].owns_dst,
          "original TRANSIT does not own the boundary DST");
  f.auditAll();

  new_fork.pos = 2;
  new_fork.owns_dst = false;
  new_fork.owns_landing = false;
  std::vector<fpga::Wire> suffix{
      new_fork, f.terminal({0, 0}, 222, 87),
      SourceInputFixture::pin({0, 0}, 87, false)};
  require(pnr::commitPreparedRoute(suffix), "cannot lease replacement fork suffix");
  auto prefix_pin = SourceInputFixture::pin({7, 0}, 141, true);
  prefix_pin.shared = true;
  auto shared_takeoff = takeoff;
  shared_takeoff.shared = true;
  suffix.insert(suffix.begin(), {prefix_pin, shared_takeoff});
  const size_t new_storage = f.sink->wires.size();
  f.sink->wires.push_back(std::move(suffix));
  const size_t replacement = fpga::attachNetRoute(
      *f.signal, *f.sink, new_storage, f.source, f.sink, "Q", "I0",
      "same_binding_fork_boundary");
  fpga::registerNetRouteTiles(*f.signal, f.sink->wires[new_storage], replacement);

  require(replacement == old_binding && f.signal->routes.size() == 1 &&
              f.source->wires[old_storage].empty(),
          "replacement did not transfer the exact incomplete binding");
  require(boundary.cb.dst.jump.testBit(220),
          "same-binding replacement released the fork's incoming DST");
  auto owners = fpga::findNetOwnersByNode(boundary, fpga::CB_NODE_DST, 220);
  require(owners.size() == 1 && owners.front().net == f.signal &&
              owners.front().binding_index == replacement,
          "same-binding fork has no physical boundary DST owner");
  require(f.sink->wires[new_storage][1].owns_landing &&
              !f.sink->wires[new_storage][2].owns_dst,
          "replacement did not transfer the boundary owner to its prefix");
  f.auditAll();
}

} // namespace

void moving_terminal_ownership_regression(int mode) {
  if (mode == 0) moving_foreign_terminal_reservation();
  else if (mode == 1) moving_root_local_input_pin_lease();
  else moving_same_binding_fork_boundary_owner();
}
